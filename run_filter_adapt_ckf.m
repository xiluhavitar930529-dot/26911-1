function est = run_filter_adapt_ckf(fused_xyz, fused_R, frame_times, cfg, passive_bearing, platform, fused_ids, event_meta)
%RUN_FILTER_ADAPT_CKF  自适应CKF滤波器（CA模型 + CS自适应Q + 自适应R + 匈牙利匹配）
%
%  支持非均匀变采样间隔：每步根据实际dt动态重建F和Q矩阵。
%
%  【本版本在“抗断裂版”基础上做了系统性的航迹管理升级】
%  =====================================================================
%  抗断裂改造（上一版已含）：
%    - 三态生命周期 tentative/confirmed/deleted，确认黏性，滑行保持输出；
%    - 出生抑制门 birth_guard_m；
%    - 按位置去重(零速新生可并入运动航迹)；
%    - 确认权重地板 + 命中增益。
%
%  本版新增的改进（带 on/off 句柄，缺省可不改 config 直接运行）：
%    A) 滑窗累积速度趋势（vel_trend_*）：密集且高噪声量测下，单帧滤波速度抖动大。
%       为每条航迹维护“位置-时间”滑窗，用最小二乘拟合得到该段航迹的累积速度
%       (大小+方向)作为速度趋势，在预测步按比例替代/融合状态速度来外推位置，
%       不再单纯依赖上一帧的瞬时速度估计。开关 cfg.vel_trend_enabled。
%
%  其余管理升级（上一版已含）：交叉保护去重、NIS一致性监控、滑行阻尼、
%    两点速度初始化、权重语义统一；主动命中和成组被动等效命中分别记为S=1/4。
%
%  Output:
%    est : 结构体
%      .X{k}        状态估计 [9×N_confirmed]
%      .P{k}        协方差 [9×9×N_confirmed]
%      .N(k)        确认航迹数
%      .L{k}        航迹标签 [出生帧; ID]
%      .tracks{k}   所有航迹信息（含未确认）
%      .passive_bearing_stats  被动角度输入、关联、拒绝及节流统计
%      .track_timeout_stats    秒级最长等待期删除统计与逐航迹记录
%      .birth_stats            普通/冲突候选的出生、确认与删除统计
%      .active_assoc_stats     分层主关联、二次分配和次回波诊断统计
%      .track_delete_stats     各删除路径的互斥计数
%      .end_of_stream          数据结束时仍存活航迹及其计划超时截止时刻
%      .timing      耗时统计

K = numel(fused_xyz);
if nargin < 5 || isempty(passive_bearing)
    passive_bearing = cell(K, 1);
end
if nargin < 6
    platform = [];
end
if nargin < 7 || isempty(fused_ids)
    fused_ids = cell(K, 1);   % 未提供编号时，评价关联字段保持NaN
end
if nargin < 8 || isempty(event_meta)
    event_meta = default_event_meta(K);
end
x_dim = cfg.x_dim;
z_dim = cfg.z_dim;
pos_idx = [1, 4, 7];   % 状态中X,Y,Z位置索引
vel_idx = [2, 5, 8];   % 速度索引
acc_idx = [3, 6, 9];   % 加速度索引

fprintf('\n========== 自适应CKF滤波（管理升级版） ==========\n');
fprintf('帧数: %d, 状态维数: %d (CA)\n', K, x_dim);

if K == 0
    est.X = {}; est.P = {}; est.N = []; est.L = {}; est.tracks = {}; est.assoc = {};
    est.passive_assoc = {};
    est.gate = {}; est.gate_gamma = cfg.gating_gamma;
    est.timing = struct('predict', 0, 'update', 0, 'manage', 0, 'total', 0);
    est.passive_bearing_stats = empty_passive_bearing_stats();
    est.passive_bearing_stats.n_equivalent_confirm_hits = 0;
    est.track_timeout_stats = make_track_timeout_stats( ...
        get_cfg_field(cfg, 'track_timeout_enabled', true) ~= 0, ...
        get_cfg_field(cfg, 'tentative_max_silence_s', 2), ...
        get_cfg_field(cfg, 'confirmed_max_silence_s', 6), 0, 0, ...
        empty_track_timeout_records());
    est.birth_stats = empty_active_birth_stats();
    est.active_assoc_stats = empty_active_assoc_stats();
    est.track_delete_stats = empty_track_delete_stats();
    est.end_of_stream = make_end_of_stream_status(trk_init(x_dim), NaN, ...
        get_cfg_field(cfg, 'tentative_max_silence_s', 2), ...
        get_cfg_field(cfg, 'confirmed_max_silence_s', 6));
    est.output_freshness = struct('max_silence_s', ...
        get_cfg_field(cfg, 'confirmed_output_max_silence_s', 0.5), ...
        'n_suppressed', 0);
    fprintf('无帧数据，跳过滤波。\n');
    return;
end

%% ── 滤波器参数 ────────────────────────────────────────────────────────
R_default_diag = cfg.R_default_diag;
R_min_diag     = cfg.R_min_diag;
R_default = diag(R_default_diag);

gamma_gate     = cfg.gating_gamma;
cost_unmatched = cfg.cost_unmatched;
assoc_pos_gate_m = get_cfg_field(cfg, 'assoc_pos_gate_m', inf);
active_pre_gate_enabled = get_cfg_field(cfg, 'active_pre_gate_enabled', false) ~= 0;
miss_decay     = get_miss_decay(cfg);

% 抗断裂参数
birth_guard_m    = get_cfg_field(cfg, 'birth_guard_m',          1000);
birth_suppress_gated = get_cfg_field(cfg, 'birth_suppress_gated', true) ~= 0;
merge_pos_dist_m = get_cfg_field(cfg, 'merge_pos_dist_m',       400);
max_coast_frames = get_cfg_field(cfg, 'max_coast_frames',       12);
tent_max_miss    = get_cfg_field(cfg, 'tentative_max_miss',     4);
track_timeout_enabled = get_cfg_field(cfg, 'track_timeout_enabled', true) ~= 0;
tentative_max_silence_s = get_cfg_field(cfg, 'tentative_max_silence_s', 2);
confirmed_max_silence_s = get_cfg_field(cfg, 'confirmed_max_silence_s', 6);
confirmed_output_max_silence_s = get_cfg_field(cfg, ...
    'confirmed_output_max_silence_s', 0.5);
confirmed_time_authoritative = track_timeout_enabled && isfinite(confirmed_max_silence_s);
w_floor_conf     = get_cfg_field(cfg, 'weight_floor_confirmed', 0.6);
hit_gain         = get_cfg_field(cfg, 'hit_weight_gain',        0.2);
passive_bearing_enabled = get_cfg_field(cfg, 'passive_bearing_enabled', true) && ~isempty(platform);
passive_bearing_gate = get_cfg_field(cfg, 'passive_bearing_gate', 16);
passive_bearing_nis_gate = get_cfg_field(cfg, 'passive_bearing_nis_gate', 16);
passive_bearing_weight_gain = get_cfg_field(cfg, 'passive_bearing_weight_gain', 0.5) * hit_gain;
passive_bearing_confirm_hit = get_cfg_field(cfg, 'passive_bearing_confirm_hit', false) ~= 0;
passive_bearing_update_on_active = get_cfg_field(cfg, 'passive_bearing_update_on_active', true) ~= 0;
passive_bearing_update_on_pure = get_cfg_field(cfg, 'passive_bearing_update_on_pure', true) ~= 0;
passive_bearing_update_active_hit_tracks = get_cfg_field(cfg, 'passive_bearing_update_active_hit_tracks', false) ~= 0;
passive_bearing_min_dt_s = get_cfg_field(cfg, 'passive_bearing_min_dt_s', 0.10);
passive_bearing_fast_gate_deg = get_cfg_finite_or_posinf_field( ...
    cfg, 'passive_bearing_fast_gate_deg', 2.0);
joint_extension_enabled = get_cfg_field(cfg, 'joint_extension_enabled', false) ~= 0;
joint_passive_confirm_hits = max(1, round(get_cfg_field(cfg, ...
    'joint_passive_confirm_consecutive_hits', 3)));
joint_passive_confirm_max_gap_s = get_cfg_field(cfg, ...
    'joint_passive_confirm_max_gap_s', 0.20);

% 管理升级参数
coast_acc_decay  = get_cfg_field(cfg, 'coast_acc_decay',        0.6);  % 滑行加速度衰减
nis_gate         = get_cfg_field(cfg, 'nis_gate',               16);   % NIS门限(3自由度)
nis_max_bad      = get_cfg_field(cfg, 'nis_max_bad',            5);    % 连续超界次数→删除
assoc_accept_nis = min(get_cfg_field(cfg, 'assoc_accept_nis', nis_gate), nis_gate); % 主动最终验收门不宽于健康NIS门
if ~isscalar(cost_unmatched) || ~isfinite(cost_unmatched) || ...
        cost_unmatched < assoc_accept_nis
    error(['cfg.cost_unmatched must be finite and >= the effective ' ...
        'cfg.assoc_accept_nis; otherwise it becomes a hidden acceptance gate.']);
end
meas_fuse_enabled = get_cfg_field(cfg, 'meas_fuse_enabled', true);   % 同目标多回波融合更新(治抖)
meas_fuse_R_scale = get_cfg_field(cfg, 'meas_fuse_R_scale',  1);     % 融合后R放大系数(回波相关时可>1)
meas_fuse_weight  = get_cfg_text_field(cfg, 'meas_fuse_weight', 'likelihood');  % 'likelihood'(抗野值,略偏预测) | 'equal'(等权,无偏)
meas_fuse_cluster_gamma = get_cfg_field(cfg, 'meas_fuse_cluster_gamma', 16); % 次回波与主回波的一致性门
meas_fuse_cluster_dist_m = get_cfg_field(cfg, 'meas_fuse_cluster_dist_m', 200); % 次回波与主回波的欧氏距离门
meas_robust_enabled = get_cfg_field(cfg, 'meas_robust_enabled', true);  % 逐轴Huber抗野值更新(夹平外点尖刺)
robust_k            = get_cfg_field(cfg, 'robust_k',            3);      % 单轴标准化残差阈值(σ)，超过则降权
robust_R_max_infl   = get_cfg_field(cfg, 'robust_R_max_infl',   100);    % 单轴R膨胀封顶
reacq_P_inflate     = get_cfg_field(cfg, 'reacq_P_inflate',     100);    % 确认航迹NIS异常→位置协方差膨胀(保号重捕获)
reacq_vel_inflate   = get_cfg_field(cfg, 'reacq_vel_inflate',   10);     % 速度协方差膨胀
dedup_vel_angle  = get_cfg_field(cfg, 'dedup_vel_angle_deg',    30);   % 交叉保护速度夹角门
dedup_min_speed  = get_cfg_field(cfg, 'dedup_min_speed',        20);   % 低于此速度不做速度门
vinit_baseline_s = get_cfg_field(cfg, 'vinit_baseline_s',       0.5);  % 两点速度初始化时间基线
vinit_min_disp_m = get_cfg_field(cfg, 'vinit_min_disp_m',       300);  % 两点速度初始化最小位移

% ── 改进A：滑窗累积速度趋势（大小+方向，抗密集高噪声下的单帧速度抖动）──
vel_trend_enabled   = get_cfg_field(cfg, 'vel_trend_enabled',   true) ~= 0;
vel_trend_window    = max(2, round(get_cfg_field(cfg, 'vel_trend_window',    8)));   % 滑窗最大样本数
vel_trend_span_s    = get_cfg_field(cfg, 'vel_trend_span_s',    0.4);   % 滑窗最小时间基线(s)
vel_trend_min_n     = max(2, round(get_cfg_field(cfg, 'vel_trend_min_n',     4)));   % 拟合所需最小样本数
vel_trend_blend     = min(max(get_cfg_field(cfg, 'vel_trend_blend',  0.7), 0), 1);   % 趋势替代比例[0,1]
vel_trend_min_speed = get_cfg_field(cfg, 'vel_trend_min_speed',  20);   % 低于此速度不施加趋势(避免给静止/慢目标硬塞方向)
vel_trend_conf_only = get_cfg_field(cfg, 'vel_trend_conf_only',  false) ~= 0;        % 仅对确认航迹施加

% ── 改进B：CA+CV 等维 IMM 多模型（模型1=CV, 模型2=CA）──
%  组合估计写回 trk.m/trk.P，所有现有管理逻辑(门控/关联/去重/NIS/输出)无感；
%  模型条件状态存于 trk.imm{i}=struct('x'[9×2],'P'[9×9×2],'mu'[2×1])。
%  自适应R：与模型无关，各模型共享每航迹的自适应R；自适应Q：CA用CS自适应Q,
%  CV用白噪声加速度固定Q，IMM模型概率切换本身即等效Q自适应。
use_imm          = get_cfg_field(cfg, 'use_imm',           true) ~= 0;
imm_n_models     = 2;                                                  % 固定 CV + CA
imm_p_cv_stay    = min(max(get_cfg_field(cfg, 'imm_p_cv_stay', 0.95), 0), 1);  % CV自保持概率
imm_p_ca_stay    = min(max(get_cfg_field(cfg, 'imm_p_ca_stay', 0.95), 0), 1);  % CA自保持概率
imm_mu_init_cv   = min(max(get_cfg_field(cfg, 'imm_mu_init_cv', 0.5), 0), 1);  % 新生CV初始模型概率
% 转移概率矩阵 TPM(行=从, 列=到)；模型顺序 [CV, CA]
imm_TPM = [imm_p_cv_stay, 1 - imm_p_cv_stay;
           1 - imm_p_ca_stay, imm_p_ca_stay];

fprintf('管理参数: 关联门=%.0fm, 出生抑制=%.0fm, 门内未分配禁止新生=%d, 去重=%.0fm(交叉保护%.0f°), 滑行=%d帧(accel衰减%.2f), NIS门=%.0f×%d\n', ...
    assoc_pos_gate_m, birth_guard_m, birth_suppress_gated, merge_pos_dist_m, dedup_vel_angle, max_coast_frames, coast_acc_decay, nis_gate, nis_max_bad);
fprintf('航迹秒级超时: enabled=%d, 试探无主动命中>%.2fs, 确认无有效更新>%.2fs\n', ...
    track_timeout_enabled, tentative_max_silence_s, confirmed_max_silence_s);
if confirmed_time_authoritative
    fprintf('确认航迹删除策略: 秒级超时为权威门；max_coast_frames仅用于滑行/重捕获\n');
else
    fprintf('确认航迹删除策略: 秒级门未启用，回退到max_coast_frames=%d\n', max_coast_frames);
end
fprintf('主动验收: NIS<=%.1f, 一对一主关联, 次回波融合=%d(pair NIS<=%.1f, 距离<=%.0fm)\n', ...
    assoc_accept_nis, meas_fuse_enabled, meas_fuse_cluster_gamma, meas_fuse_cluster_dist_m);
fprintf('active pre-gate: %d\n', active_pre_gate_enabled || birth_suppress_gated);
if vel_trend_enabled
    if vel_trend_conf_only, vt_scope = '(仅确认航迹)'; else, vt_scope = ''; end
    fprintf('滑窗速度趋势: 窗=%d样本/≥%.2fs, 最小样本=%d, 替代比例=%.2f, 最小速度=%.0fm/s%s\n', ...
        vel_trend_window, vel_trend_span_s, vel_trend_min_n, vel_trend_blend, vel_trend_min_speed, vt_scope);
end
if use_imm
    fprintf('IMM多模型: CV+CA, TPM自保持=[CV %.2f, CA %.2f], 新生CV先验=%.2f, CV过程噪声σa=%.1f\n', ...
        imm_p_cv_stay, imm_p_ca_stay, imm_mu_init_cv, get_cfg_field(cfg, 'imm_cv_sigma_a', 3));
end
if passive_bearing_enabled
    fprintf('被动bearing-only更新: gate=%.1f, NIS门=%.1f, 确认命中=%d\n', ...
        passive_bearing_gate, passive_bearing_nis_gate, passive_bearing_confirm_hit);
    fprintf('passive bearing throttle: active=%d, pure=%d, active_hit=%d, min_dt=%.3fs, fast_gate=%.2fdeg\n', ...
        passive_bearing_update_on_active, passive_bearing_update_on_pure, ...
        passive_bearing_update_active_hit_tracks, passive_bearing_min_dt_s, ...
        passive_bearing_fast_gate_deg);
    fprintf(['passive timing: bounded event batches; raw timestamps retained upstream; ' ...
        'TXT shards share one physical source\n']);
end

% H矩阵（提取XYZ位置）
if z_dim ~= 3
    error('CKF Cartesian measurement update expects cfg.z_dim == 3.');
end

% 新生目标协方差
P_birth = diag(repmat([cfg.P_pos_birth, cfg.P_vel_birth, cfg.P_acc_birth], 1, 3));
P_pos_min = 100^2;  % 位置协方差下限

%% ── 输出容器 ──────────────────────────────────────────────────────────
est.X = cell(K, 1);
est.P = cell(K, 1);
est.N = zeros(K, 1);
est.L = cell(K, 1);
est.tracks = cell(K, 1);
est.assoc = cell(K, 1);   % 每帧量测归属：struct('id',[1×n],'xyz',[3×n])，记录进入匹配的3D量测(主动)+新生种子
est.gate = cell(K, 1);    % 每帧关联门：struct('id',[1×n],'pos',[3×n],'S',[3×3×n])，CKF预测位置+S_gate=Pzz+R
est.gate_gamma = gamma_gate;   % 卡方门限；门椭球马氏半径 = sqrt(gamma_gate)
est.timing = struct('predict', 0, 'update', 0, 'manage', 0);
est.event_meta = event_meta;
est.filter_times = frame_times(:);
est.passive_assoc = cell(K, 1);
if joint_extension_enabled
    joint2d_cfg = make_online_joint2d_cfg(cfg, frame_times, passive_bearing);
    joint2d_state = [];
    est.joint2d = init_online_joint2d_estimate(K, frame_times);
    fprintf('  Joint online residual-angle branch: enabled (single causal pass).\n');
else
    joint2d_cfg = struct();
    joint2d_state = [];
    est.joint2d = [];
end

%% ── 航迹集合（结构体形式，统一管理所有逐航迹属性） ────────────────────
trk = trk_init(x_dim);
next_id = 1;
prev_t  = frame_times(1);
n_passive_bearing_used_total = 0;
n_passive_bearing_skipped_total = 0;
passive_bearing_stats = empty_passive_bearing_stats();
passive_confirm_state = empty_passive_confirm_state();
passive_equivalent_hits_total = 0;
last_passive_bearing_t = -inf;
n_birth_total = 0;
n_birth_suppressed_gated = 0;
n_birth_suppressed_guard = 0;
n_assoc_nis_rejected_total = 0;
n_secondary_fused_total = 0;
n_secondary_released_total = 0;
n_timeout_tentative_total = 0;
n_timeout_confirmed_total = 0;
n_output_stale_suppressed = 0;
timeout_records = empty_track_timeout_records();
birth_stats = empty_active_birth_stats();
active_assoc_stats = empty_active_assoc_stats();
delete_stats = empty_track_delete_stats();

% Birth classes are association-priority classes, not truth labels.
% Ordinary tentative tracks use genuinely unexplained measurements.  A
% rejected primary or an incompatible secondary starts as a lower-priority
% conflict tentative so that it cannot steal the next primary return from a
% mature confirmed track.
BIRTH_ORDINARY = 0;
BIRTH_NIS_CONFLICT = 1;
BIRTH_SECONDARY_CONFLICT = 2;

%% ═══════════════════════════════════════════════════════════════════════════
%  主循环
%% ═══════════════════════════════════════════════════════════════════════════
for k = 1:K
    t_k = frame_times(k);
    z_k = fused_xyz{k};
    R_k_meas = fused_R{k};
    M = size(z_k, 2);
    meta_k = get_event_meta(event_meta, k, M);
    count_miss_cycle = meta_k.miss_cycle;
    count_confirm_cycle = meta_k.confirm_cycle;
    passive_detail_k = empty_passive_assoc_detail(get_passive_count(passive_bearing, k));

    % 本帧融合量测的目标关联编号(与 z_k 列对齐)；缺失 → 全 NaN
    if k <= numel(fused_ids) && ~isempty(fused_ids{k})
        ids_k = fused_ids{k}(:).';
        if numel(ids_k) < M
            ids_k = [ids_k, nan(1, M - numel(ids_k))];
        elseif numel(ids_k) > M
            ids_k = ids_k(1:M);
        end
    else
        ids_k = nan(1, M);
    end

    % 秒级最长等待期在关联前执行：超过期限的旧航迹不能被迟到量测复活。
    % 试探航迹只看最后主动命中；确认航迹允许有效被动更新延长存活。
    t_timeout = tic;
    if track_timeout_enabled && trk.N > 0
        active_silence_s = max(t_k - trk.last_active_hit_t, 0);
        update_silence_s = max(t_k - trk.last_update_t, 0);
        timeout_tent = (trk.conf == 0) & isfinite(active_silence_s) & ...
            (active_silence_s > tentative_max_silence_s);
        timeout_conf = (trk.conf == 1) & isfinite(update_silence_s) & ...
            (update_silence_s > confirmed_max_silence_s);
        timeout_idx = find(timeout_tent | timeout_conf);
        for qi = reshape(timeout_idx, 1, [])
            rec = track_timeout_record_template();
            rec.track_id = trk.L(2, qi);
            rec.was_confirmed = trk.conf(qi) == 1;
            rec.timeout_time = t_k;
            rec.last_active_hit_t = trk.last_active_hit_t(qi);
            rec.last_update_t = trk.last_update_t(qi);
            if rec.was_confirmed
                rec.silence_s = update_silence_s(qi);
                rec.limit_s = confirmed_max_silence_s;
                rec.deadline_t = rec.last_update_t + rec.limit_s;
            else
                rec.silence_s = active_silence_s(qi);
                rec.limit_s = tentative_max_silence_s;
                rec.deadline_t = rec.last_active_hit_t + rec.limit_s;
            end
            timeout_records(end+1, 1) = rec; %#ok<AGROW>
        end
        n_timeout_tentative_total = n_timeout_tentative_total + nnz(timeout_tent);
        n_timeout_confirmed_total = n_timeout_confirmed_total + nnz(timeout_conf);
        delete_stats.n_timeout_tentative = delete_stats.n_timeout_tentative + nnz(timeout_tent);
        delete_stats.n_timeout_confirmed = delete_stats.n_timeout_confirmed + nnz(timeout_conf);
        if ~isempty(timeout_idx)
            trk_before_delete = trk;
            trk = trk_subset(trk, ~(timeout_tent | timeout_conf));
            [n_nis_removed, n_secondary_removed] = removed_birth_kind_counts( ...
                trk_before_delete, trk, BIRTH_NIS_CONFLICT, BIRTH_SECONDARY_CONFLICT);
            birth_stats.n_nis_conflict_deleted = ...
                birth_stats.n_nis_conflict_deleted + n_nis_removed;
            birth_stats.n_secondary_conflict_deleted = ...
                birth_stats.n_secondary_conflict_deleted + n_secondary_removed;
        end
    end
    est.timing.manage = est.timing.manage + toc(t_timeout);

    if k == 1
        dt = cfg.frame_time_window_s;
    else
        dt = max(t_k - prev_t, 0.001);
    end

    %% ── 第1步：预测（含滑行加速度阻尼） ──────────────────────────────
    t_pred = tic;

    if trk.N > 0
        % 预先算好每条航迹的滑窗速度趋势(两条路径共用)
        vtr_track = cell(1, trk.N);
        if vel_trend_enabled
            for i = 1:trk.N
                vtr_track{i} = [];
                if vel_trend_conf_only && trk.conf(i) ~= 1
                    continue;
                end
                [vtr, ok] = window_velocity(trk.poshist{i}, vel_trend_min_n, ...
                                            vel_trend_span_s, vel_trend_window);
                if ok && norm(vtr) >= vel_trend_min_speed
                    vtr_track{i} = vtr;
                end
            end
        else
            for i = 1:trk.N, vtr_track{i} = []; end
        end

        coasting = trk.miss >= 1;

        if use_imm
            % ── 改进B：IMM 预测（交互混合 + 各模型预测 + 组合）──
            m_pred = zeros(x_dim, trk.N);
            P_pred = zeros(x_dim, x_dim, trk.N);
            pred_imm = cell(1, trk.N);
            for i = 1:trk.N
                if isempty(trk.imm{i}) || ~isfield(trk.imm{i}, 'mu')
                    trk.imm{i} = imm_init_bank(trk.m(:, i), trk.P(:, :, i), imm_mu_init_cv, imm_n_models);
                end
                [bp, mpi, Ppi] = imm_predict_bank(trk.imm{i}, dt, cfg, imm_TPM, ...
                    vtr_track{i}, vel_trend_blend, coasting(i), coast_acc_decay, ...
                    vel_idx, acc_idx);
                m_pred(:, i)    = mpi;
                P_pred(:, :, i) = Ppi;
                pred_imm{i}     = bp;
            end
        else
            % ── 原 CA 单模型预测 ──
            F = build_CA_F(dt);
            m_prop = trk.m;
            if vel_trend_enabled
                for i = 1:trk.N
                    if ~isempty(vtr_track{i})
                        v_state = trk.m(vel_idx, i);
                        m_prop(vel_idx, i) = (1 - vel_trend_blend) * v_state + vel_trend_blend * vtr_track{i};
                    end
                end
            end
            Q_cs = build_CS_Q(m_prop, dt, cfg);
            m_pred = zeros(x_dim, trk.N);
            P_pred = zeros(x_dim, x_dim, trk.N);
            for i = 1:trk.N
                [m_pred(:, i), P_pred(:, :, i)] = ckf_predict_linear( ...
                    m_prop(:, i), trk.P(:, :, i), F, Q_cs(:, :, i));
            end
            if any(coasting)
                m_pred(acc_idx, coasting) = coast_acc_decay * m_pred(acc_idx, coasting);
            end
            pred_imm = trk.imm;   % 透传(非IMM时为占位)
        end

        % 协方差下限保护
        for i = 1:trk.N
            for d = 1:3
                id = pos_idx(d);
                if P_pred(id, id, i) < P_pos_min
                    P_pred(id, id, i) = P_pos_min;
                end
            end
        end

        pred = trk;
        pred.m   = m_pred;
        pred.P   = P_pred;
        pred.imm = pred_imm;                               % 预测后的模型bank(供更新使用)
        if count_miss_cycle
            pred.w = cfg.P_S * trk.w;
        else
            pred.w = trk.w;
        end
        if count_confirm_cycle
            pred.age = trk.age + 1;
            pred.S   = [trk.S(2:end, :); 2 * ones(1, trk.N)];  % 主动检测周期：推进M/N窗口
        else
            pred.age = trk.age;
            pred.S   = trk.S;                                % 纯被动事件：不消耗确认/漏检窗口
        end
    else
        pred = trk_init(x_dim);
    end

    % ── 导出关联门(供回放绘制)：每条预测航迹的 CKF预测位置 + S_gate=Pzz+R ──
    gate_ids = zeros(1, 0);
    gate_pos = zeros(3, 0);
    gate_S   = zeros(3, 3, 0);
    if pred.N > 0
        for ig = 1:pred.N
            [gate_z, gate_Pzz] = ckf_cartesian_stats(pred.m(:, ig), pred.P(:, :, ig), pos_idx);
            if ig <= numel(pred.R) && isequal(size(pred.R{ig}), [3, 3])
                R_g = pred.R{ig};
            else
                R_g = R_default;
            end
            gate_ids(end+1) = pred.L(2, ig);             %#ok<AGROW>
            gate_pos(:, end+1) = gate_z;                 %#ok<AGROW>
            gate_S(:, :, end+1) = make_spd(gate_Pzz + R_g); %#ok<AGROW>
        end
    end

    est.timing.predict = est.timing.predict + toc(t_pred);

    %% ── 第2步：门控 ───────────────────────────────────────────────────
    if M > 0 && pred.N > 0
        if active_pre_gate_enabled || birth_suppress_gated
            [z_gated, gated_idx] = gate_meas_ckf(z_k, gamma_gate, ...
                pos_idx, pred.m, pred.P, R_k_meas, R_default, assoc_pos_gate_m);
            Mg = size(z_gated, 2);
        else
            z_gated = z_k;
            gated_idx = 1:M;
            Mg = M;
        end
    else
        z_gated = zeros(3, 0);
        gated_idx = [];
        Mg = 0;
    end

    %% ── 第3步：关联（量测→最近航迹，允许同目标多回波归入一条航迹） ─────
    t_upd = tic;

    track_meas = cell(max(pred.N, 1), 1);   % 每条航迹收纳的门内量测下标(在 z_gated 中)
    primary_stage = zeros(1, pred.N);       % 1=confirmed, 2=ordinary tentative, 3=conflict tentative
    reassigned_track = false(1, pred.N);
    nis_reject_track_count = zeros(1, pred.N);
    meas_birth_kind = BIRTH_ORDINARY * ones(1, M);
    meas_parent_id = nan(1, M);
    secondary_release_orig_mask = false(1, M);
    secondary_pair_fail_orig_mask = false(1, M);
    secondary_group_release_orig_mask = false(1, M);
    if Mg > 0 && pred.N > 0
        cost_mat = inf(pred.N, Mg);
        nis_mat = inf(pred.N, Mg);           % 未施加编号折扣的原始运动学NIS
        for i = 1:pred.N
            [z_pred_i, Pzz_i] = ckf_cartesian_stats(pred.m(:, i), pred.P(:, :, i), pos_idx);
            for mi = 1:Mg
                orig_mi = gated_idx(mi);
                R_meas_i = get_meas_R(R_k_meas, orig_mi, R_default);
                R_cost = meas_fuse_R_scale * ...
                    active_track_R(pred, i, R_meas_i, Pzz_i, cfg, R_min_diag);
                for di = 1:z_dim
                    R_cost(di, di) = max(R_cost(di, di), R_min_diag(di));
                end
                R_cost = make_spd(R_cost);
                S_mat = make_spd(Pzz_i + R_cost);
                delta = z_gated(:, mi) - z_pred_i;
                if isfinite(assoc_pos_gate_m) && norm(delta) > assoc_pos_gate_m
                    continue;
                end
                nis_ij = delta' * (S_mat \ delta);
                cost_mat(i, mi) = nis_ij;
                nis_mat(i, mi) = nis_ij;
            end
        end
        cost_mat(~isfinite(cost_mat)) = inf;
        nis_mat(~isfinite(nis_mat)) = inf;
        % The assignment solver's unmatched penalty is not an acceptance
        % gate.  Invalid motion edges must be removed before assignment so
        % that an NIS=20..150 pair cannot occupy a row/column and then fail
        % only after all alternative matches have been lost.
        active_assoc_stats.n_edges_over_accept_nis = active_assoc_stats.n_edges_over_accept_nis + ...
            nnz(isfinite(nis_mat) & nis_mat > assoc_accept_nis);
        cost_mat(nis_mat > assoc_accept_nis) = inf;

        % Confirmed tracks own the first association opportunity.  Ordinary
        % tentative tracks use only confirmed leftovers; conflict-born
        % tentative tracks are last and therefore cannot steal a primary
        % measurement merely because their birth covariance is broad.
        available_meas = true(1, Mg);
        tier_tracks = {find(pred.conf == 1), ...
            find(pred.conf == 0 & pred.birth_kind == BIRTH_ORDINARY), ...
            find(pred.conf == 0 & pred.birth_kind ~= BIRTH_ORDINARY)};
        for stage_code = 1:3
            [track_meas, primary_stage, available_meas, added_tracks] = ...
                assign_primary_tier(track_meas, primary_stage, available_meas, ...
                tier_tracks{stage_code}, cost_mat, cost_unmatched, stage_code);
            if meas_fuse_enabled && ~isempty(added_tracks)
                [track_meas, available_meas, failed_mi, failed_parent, pair_nis_used, pair_dist_used] = ...
                    attach_secondary_tier(track_meas, available_meas, added_tracks, ...
                    nis_mat, z_gated, gated_idx, R_k_meas, R_default, ...
                    assoc_accept_nis, meas_fuse_cluster_gamma, meas_fuse_cluster_dist_m);
                active_assoc_stats.secondary_pair_nis = ...
                    [active_assoc_stats.secondary_pair_nis, pair_nis_used]; %#ok<AGROW>
                active_assoc_stats.secondary_pair_dist_m = ...
                    [active_assoc_stats.secondary_pair_dist_m, pair_dist_used]; %#ok<AGROW>
                for qi = 1:numel(failed_mi)
                    oi = gated_idx(failed_mi(qi));
                    secondary_release_orig_mask(oi) = true;
                    secondary_pair_fail_orig_mask(oi) = true;
                    meas_birth_kind(oi) = max(meas_birth_kind(oi), BIRTH_SECONDARY_CONFLICT);
                    if failed_parent(qi) > 0
                        meas_parent_id(oi) = pred.L(2, failed_parent(qi));
                    end
                end
            end
        end

        % Validate complete groups before mutating any track state.  A bad
        % fused group first falls back to its primary.  A bad primary edge
        % is banned and gets one finite re-assignment pass.
        banned_edges = false(pred.N, Mg);
        [track_meas, primary_stage, available_meas, banned_edges, rejected_pairs, released_pairs] = ...
            validate_active_groups(track_meas, primary_stage, available_meas, banned_edges, ...
            1:pred.N, pred, z_gated, gated_idx, R_k_meas, R_default, cfg, pos_idx, ...
            R_min_diag, meas_fuse_R_scale, meas_fuse_weight, assoc_accept_nis);
        for qi = 1:size(rejected_pairs, 1)
            ti = rejected_pairs(qi, 1); mi = rejected_pairs(qi, 2);
            nis_reject_track_count(ti) = nis_reject_track_count(ti) + 1;
            oi = gated_idx(mi);
            meas_birth_kind(oi) = BIRTH_NIS_CONFLICT;
            meas_parent_id(oi) = pred.L(2, ti);
        end
        for qi = 1:size(released_pairs, 1)
            ti = released_pairs(qi, 1); mi = released_pairs(qi, 2);
            oi = gated_idx(mi);
            secondary_release_orig_mask(oi) = true;
            secondary_group_release_orig_mask(oi) = true;
            meas_birth_kind(oi) = max(meas_birth_kind(oi), BIRTH_SECONDARY_CONFLICT);
            meas_parent_id(oi) = pred.L(2, ti);
        end
        n_assoc_nis_rejected_total = n_assoc_nis_rejected_total + size(rejected_pairs, 1);
        active_assoc_stats.n_primary_group_rejected = ...
            active_assoc_stats.n_primary_group_rejected + size(rejected_pairs, 1);
        active_assoc_stats.n_reassign_attempted = active_assoc_stats.n_reassign_attempted + ...
            size(rejected_pairs, 1) + size(released_pairs, 1);

        if ~isempty(rejected_pairs) || ~isempty(released_pairs)
            cost_retry = cost_mat;
            cost_retry(banned_edges) = inf;
            retry_tracks_all = zeros(1, 0);
            for stage_code = 1:3
                retry_tier = tier_tracks{stage_code};
                retry_tier = retry_tier(cellfun(@isempty, track_meas(retry_tier)));
                [track_meas, primary_stage, available_meas, retry_tracks] = ...
                    assign_primary_tier(track_meas, primary_stage, available_meas, ...
                    retry_tier, cost_retry, cost_unmatched, stage_code);
                retry_tracks_all = [retry_tracks_all, retry_tracks]; %#ok<AGROW>
                reassigned_track(retry_tracks) = true;
                if meas_fuse_enabled && ~isempty(retry_tracks)
                    [track_meas, available_meas, failed_mi, failed_parent, pair_nis_used, pair_dist_used] = ...
                        attach_secondary_tier(track_meas, available_meas, retry_tracks, ...
                        nis_mat, z_gated, gated_idx, R_k_meas, R_default, ...
                        assoc_accept_nis, meas_fuse_cluster_gamma, meas_fuse_cluster_dist_m);
                    active_assoc_stats.secondary_pair_nis = ...
                        [active_assoc_stats.secondary_pair_nis, pair_nis_used]; %#ok<AGROW>
                    active_assoc_stats.secondary_pair_dist_m = ...
                        [active_assoc_stats.secondary_pair_dist_m, pair_dist_used]; %#ok<AGROW>
                    for qi = 1:numel(failed_mi)
                        oi = gated_idx(failed_mi(qi));
                        secondary_release_orig_mask(oi) = true;
                        secondary_pair_fail_orig_mask(oi) = true;
                        meas_birth_kind(oi) = max(meas_birth_kind(oi), BIRTH_SECONDARY_CONFLICT);
                        if failed_parent(qi) > 0
                            meas_parent_id(oi) = pred.L(2, failed_parent(qi));
                        end
                    end
                end
            end
            [track_meas, primary_stage, available_meas, banned_edges, retry_rejected, retry_released] = ...
                validate_active_groups(track_meas, primary_stage, available_meas, banned_edges, ...
                unique(retry_tracks_all), pred, z_gated, gated_idx, R_k_meas, R_default, cfg, pos_idx, ...
                R_min_diag, meas_fuse_R_scale, meas_fuse_weight, assoc_accept_nis);
            active_assoc_stats.n_reassign_success = active_assoc_stats.n_reassign_success + ...
                max(0, numel(unique(retry_tracks_all)) - size(retry_rejected, 1));
            for qi = 1:size(retry_rejected, 1)
                ti = retry_rejected(qi, 1); mi = retry_rejected(qi, 2);
                nis_reject_track_count(ti) = nis_reject_track_count(ti) + 1;
                oi = gated_idx(mi);
                meas_birth_kind(oi) = BIRTH_NIS_CONFLICT;
                meas_parent_id(oi) = pred.L(2, ti);
            end
            for qi = 1:size(retry_released, 1)
                ti = retry_released(qi, 1); mi = retry_released(qi, 2);
                oi = gated_idx(mi);
                secondary_release_orig_mask(oi) = true;
                secondary_group_release_orig_mask(oi) = true;
                meas_birth_kind(oi) = max(meas_birth_kind(oi), BIRTH_SECONDARY_CONFLICT);
                meas_parent_id(oi) = pred.L(2, ti);
            end
            n_assoc_nis_rejected_total = n_assoc_nis_rejected_total + size(retry_rejected, 1);
            active_assoc_stats.n_primary_group_rejected = ...
                active_assoc_stats.n_primary_group_rejected + size(retry_rejected, 1);
        end
        % Measurements that were close enough for the old broad assignment
        % gate but failed the strict acceptance NIS are conflict candidates,
        % not ordinary new targets.  They remain eligible at the lowest
        % association tier and can still prove an independent trajectory.
        for mi = find(available_meas)
            oi = gated_idx(mi);
            if meas_birth_kind(oi) ~= BIRTH_ORDINARY
                continue;
            end
            broad_conflict = isfinite(nis_mat(:, mi).') & ...
                nis_mat(:, mi).' > assoc_accept_nis & nis_mat(:, mi).' <= cost_unmatched;
            candidate_tracks = find(pred.conf == 1 & broad_conflict);
            if isempty(candidate_tracks)
                candidate_tracks = find(broad_conflict);
            end
            if isempty(candidate_tracks)
                continue;
            end
            [~, jj] = min(nis_mat(candidate_tracks, mi));
            ti = candidate_tracks(jj);
            meas_birth_kind(oi) = BIRTH_NIS_CONFLICT;
            meas_parent_id(oi) = pred.L(2, ti);
            if nis_reject_track_count(ti) == 0
                nis_reject_track_count(ti) = 1;
            end
            n_assoc_nis_rejected_total = n_assoc_nis_rejected_total + 1;
            active_assoc_stats.n_preassignment_conflict_measurements = ...
                active_assoc_stats.n_preassignment_conflict_measurements + 1;
        end
        n_secondary_released_total = n_secondary_released_total + nnz(secondary_release_orig_mask);
        active_assoc_stats.n_secondary_pair_conflict = ...
            active_assoc_stats.n_secondary_pair_conflict + nnz(secondary_pair_fail_orig_mask);
        active_assoc_stats.n_secondary_group_released = ...
            active_assoc_stats.n_secondary_group_released + nnz(secondary_group_release_orig_mask);
    end

    %% ── 第4步：更新（先全体置漏检，再覆盖成功关联） ──────────────────
    trk = pred;
    if trk.N > 0 && ~isempty(nis_reject_track_count)
        trk.nisbad = trk.nisbad + nis_reject_track_count;
    end
    assigned_orig_idx = [];
    assoc_meas_idx = zeros(1, 0);
    assoc_ids = zeros(1, 0);   % 本帧量测归属（A方案，供回放选择性绘制）
    % Diagnostics-only action label.  This does not participate in gating,
    % assignment, filtering or lifecycle decisions; it lets the outer joint
    % adapter distinguish an accepted update from a birth without inference.
    assoc_type = cell(1, 0);
    assoc_xyz = zeros(3, 0);
    assoc_tid = zeros(1, 0);
    assoc_innovation = zeros(z_dim, 0);
    assoc_nis = zeros(1, 0);
    assoc_group_size = zeros(1, 0);
    assoc_innovation_kind = zeros(1, 0); % 0=无, 1=角度deg, 2=位置m

    if trk.N > 0
        if count_miss_cycle
            % 全体默认漏检：权重衰减(确认航迹有地板)，漏检计数+1
            w_dec = miss_decay * trk.w;
            isc = (trk.conf == 1);
            w_dec(isc) = max(w_dec(isc), w_floor_conf);
            trk.w = w_dec;
            trk.miss = trk.miss + 1;
            % S 的第k行已经是2(漏检)，成功关联后再改为1
        end
    end

    active_hit_mask = false(1, trk.N);
    for ti = 1:pred.N
        J = track_meas{ti};
        if isempty(J), continue; end

        x_pred   = pred.m(:, ti);
        P_pred_i = pred.P(:, :, ti);
        g = active_group_stats(pred, ti, J, z_gated, gated_idx, R_k_meas, ...
            R_default, cfg, pos_idx, R_min_diag, meas_fuse_R_scale, meas_fuse_weight);
        if ~g.valid || ~isfinite(g.nis) || g.nis > assoc_accept_nis
            % This is a numerical safety net.  The same group was validated
            % before state mutation, so reaching this branch indicates an
            % internal inconsistency and must never be counted as a hit.
            orig_bad = gated_idx(J);
            meas_birth_kind(orig_bad(1)) = BIRTH_NIS_CONFLICT;
            meas_parent_id(orig_bad(1)) = pred.L(2, ti);
            if numel(orig_bad) > 1
                new_release = orig_bad(2:end);
                new_release = new_release(~secondary_release_orig_mask(new_release));
                n_secondary_released_total = n_secondary_released_total + numel(new_release);
                active_assoc_stats.n_secondary_group_released = ...
                    active_assoc_stats.n_secondary_group_released + numel(new_release);
                secondary_release_orig_mask(orig_bad(2:end)) = true;
                secondary_group_release_orig_mask(orig_bad(2:end)) = true;
                meas_birth_kind(orig_bad(2:end)) = BIRTH_SECONDARY_CONFLICT;
                meas_parent_id(orig_bad(2:end)) = pred.L(2, ti);
            end
            trk.nisbad(ti) = trk.nisbad(ti) + 1;
            n_assoc_nis_rejected_total = n_assoc_nis_rejected_total + 1;
            active_assoc_stats.n_postvalidation_inconsistency = ...
                active_assoc_stats.n_postvalidation_inconsistency + 1;
            continue;
        end

        nJ = g.nJ;
        Z = g.Z;
        orig = g.orig;
        z_pred_i = g.z_pred;
        Pzz_pred = g.Pzz;
        R_use = g.R_use;
        R_bar = g.R_bar;
        innovation = g.innovation;
        S_innov = g.S_innov;
        nis = g.nis;
        wv = g.wv;
        z_bar = g.z_bar;

        % ── 抗野值稳健更新（逐轴Huber）：某轴标准化残差超 robust_k·σ 时，
        %    只膨胀该轴的R、压低该轴增益，防止单个外点(尤其高程Up)把状态拽飞成尖刺 ──
        R_upd = R_bar;
        if meas_robust_enabled
            sd = sqrt(max(diag(S_innov), realmin));
            for di = 1:z_dim
                r_d = abs(innovation(di)) / sd(di);
                if r_d > robust_k
                    R_upd(di, di) = R_bar(di, di) * min((r_d / robust_k)^2, robust_R_max_infl);
                end
            end
        end

        % CKF更新（用稳健化后的R）
        if use_imm
            % ── 改进B：IMM 各模型更新 + 模型概率更新 + 组合 ──
            [bu, m_upd, P_upd] = imm_update_bank(pred.imm{ti}, z_bar, pos_idx, R_upd);
            trk.imm{ti} = bu;
        else
            [~, m_upd, P_upd] = ckf_update_cartesian(z_bar, R_upd, x_pred, P_pred_i, pos_idx);
        end

        % 自适应R缓冲：用"主导单回波"残差，避免融合(降方差)把R越估越小→门收缩(①)
        [~, idom] = max(wv);
        nu_dom = Z(:, idom) - z_pred_i;
        buf = pred.innov{ti};
        buf = [buf, nu_dom];
        if size(buf, 2) > cfg.adapt_R_window
            buf = buf(:, end - cfg.adapt_R_window + 1 : end);
        end

        % Write back active 3-D hit update.
        trk.m(:, ti)    = m_upd;
        trk.P(:, :, ti) = P_upd;
        trk.w(ti)       = min(1, pred.w(ti) + hit_gain);
        trk.miss(ti)    = 0;
        trk.S(end, ti)  = 1;
        trk.last_active_hit_t(ti) = t_k;
        trk.last_update_t(ti) = t_k;
        trk.last_active_nis_norm(ti) = nis / z_dim;
        if ti <= numel(active_hit_mask)
            active_hit_mask(ti) = true;
        end
        trk.innov{ti}   = buf;
        trk.R{ti}       = R_use;

        switch primary_stage(ti)
            case 1
                active_assoc_stats.n_confirmed_primary = active_assoc_stats.n_confirmed_primary + 1;
            case 2
                active_assoc_stats.n_tentative_primary = active_assoc_stats.n_tentative_primary + 1;
            case 3
                active_assoc_stats.n_conflict_primary = active_assoc_stats.n_conflict_primary + 1;
        end
        if reassigned_track(ti)
            active_assoc_stats.n_reassign_updates = active_assoc_stats.n_reassign_updates + 1;
        end
        if nJ > 1
            n_secondary_fused_total = n_secondary_fused_total + (nJ - 1);
            active_assoc_stats.n_secondary_used = active_assoc_stats.n_secondary_used + (nJ - 1);
        end

        if nis > nis_gate
            trk.nisbad(ti) = trk.nisbad(ti) + 1;
        else
            trk.nisbad(ti) = 0;
        end

        % Two-point velocity initialization from the fused centroid.
        if trk.vinit(ti) == 0
            elapsed = t_k - trk.born_t(ti);
            if isfinite(elapsed) && elapsed > 0 && elapsed >= vinit_baseline_s
                dseg = z_bar - trk.born_xyz(:, ti);
                if norm(dseg) >= vinit_min_disp_m
                    v_seed = dseg / elapsed;
                    if all(isfinite(v_seed))
                        trk.m(vel_idx, ti) = v_seed;
                        if use_imm
                            trk.imm{ti}.x(vel_idx, :) = repmat(v_seed, 1, size(trk.imm{ti}.x, 2));
                        end
                        trk.vinit(ti) = 1;
                    end
                end
            end
        end

        trk.poshist{ti} = push_poshist(trk.poshist{ti}, t_k, trk.m(pos_idx, ti), ...
                                       vel_trend_window);

        for jj = 1:nJ
            assigned_orig_idx(end+1) = orig(jj);
            assoc_meas_idx(end+1) = orig(jj);
            assoc_ids(end+1) = trk.L(2, ti);
            assoc_type{end+1} = 'active';                  %#ok<AGROW>
            assoc_xyz(:, end+1) = Z(:, jj);
            assoc_tid(end+1) = ids_k(orig(jj));
            assoc_innovation(:, end+1) = innovation;
            assoc_nis(end+1) = nis;
            assoc_group_size(end+1) = nJ;
            assoc_innovation_kind(end+1) = 2;
        end
    end

    est.timing.update = est.timing.update + toc(t_upd);

    %% ── 第5步：新生目标（出生抑制门） ────────────────────────────────
    t_birth = tic;

    n_meas_k = size(z_k, 2);
    if pred.N == 0
        unassoc_idx = 1:n_meas_k;
    else
        raw_unassoc_idx = setdiff(1:n_meas_k, assigned_orig_idx);
        if birth_suppress_gated
            conflict_orig_idx = find(meas_birth_kind ~= BIRTH_ORDINARY);
            suppressible_gated_idx = setdiff(gated_idx, conflict_orig_idx);
            unassoc_idx = setdiff(raw_unassoc_idx, suppressible_gated_idx);
            n_birth_suppressed_gated = n_birth_suppressed_gated + ...
                (numel(raw_unassoc_idx) - numel(unassoc_idx));
        else
            unassoc_idx = raw_unassoc_idx;
        end
    end

    % 出生抑制：落在任一现有航迹 birth_guard_m 半径内的孤儿量测不新生
    if trk.N > 0 && birth_guard_m > 0 && ~isempty(unassoc_idx)
        n_before_guard = numel(unassoc_idx);
        trk_pos = trk.m(pos_idx, :);   % [3×N]
        keep_birth = true(1, numel(unassoc_idx));
        for bi = 1:numel(unassoc_idx)
            if meas_birth_kind(unassoc_idx(bi)) ~= BIRTH_ORDINARY
                % Conflict candidates are controlled by association priority
                % and M/N persistence, not by the fixed birth radius.
                continue;
            end
            zc = z_k(:, unassoc_idx(bi));
            if min(sqrt(sum((trk_pos - zc).^2, 1))) < birth_guard_m
                keep_birth(bi) = false;
            end
        end
        unassoc_idx = unassoc_idx(keep_birth);
        n_birth_suppressed_guard = n_birth_suppressed_guard + ...
            (n_before_guard - numel(unassoc_idx));
    end
    N_birth = numel(unassoc_idx);
    n_birth_total = n_birth_total + N_birth;
    if N_birth > 0
        kinds_now = meas_birth_kind(unassoc_idx);
        birth_stats.n_ordinary = birth_stats.n_ordinary + nnz(kinds_now == BIRTH_ORDINARY);
        birth_stats.n_nis_conflict = birth_stats.n_nis_conflict + nnz(kinds_now == BIRTH_NIS_CONFLICT);
        birth_stats.n_secondary_conflict = birth_stats.n_secondary_conflict + ...
            nnz(kinds_now == BIRTH_SECONDARY_CONFLICT);
    end

    if N_birth > 0
        b = trk_init(x_dim);
        b.m   = zeros(x_dim, N_birth);
        b.P   = zeros(x_dim, x_dim, N_birth);
        b.w   = 0.5 * ones(1, N_birth);
        b.L   = zeros(2, N_birth);
        b.S   = zeros(cfg.N_confirm, N_birth);
        b.S(end, :) = 1;                       % 出生主动量测就是第1次合规检测
        b.innov = cell(1, N_birth);
        b.R   = cell(1, N_birth);
        b.conf = zeros(1, N_birth);
        b.miss = zeros(1, N_birth);
        b.age  = ones(1, N_birth);
        b.nisbad = zeros(1, N_birth);
        b.last_active_nis_norm = nan(1, N_birth);
        b.born_xyz = z_k(:, unassoc_idx);
        b.born_t = t_k * ones(1, N_birth);
        b.last_active_hit_t = t_k * ones(1, N_birth);
        b.last_update_t = t_k * ones(1, N_birth);
        b.last_passive_update_t = nan(1, N_birth);
        b.birth_kind = meas_birth_kind(unassoc_idx);
        b.parent_id = meas_parent_id(unassoc_idx);
        b.vinit = zeros(1, N_birth);
        b.poshist = cell(1, N_birth);
        b.imm = cell(1, N_birth);
        b.N = N_birth;

        for bb = 1:N_birth
            b.m(pos_idx, bb) = z_k(:, unassoc_idx(bb));
            b.P(:, :, bb) = P_birth;
            b.L(:, bb) = [k; next_id];
            oid = unassoc_idx(bb);
            birth_stats.records(end+1, 1) = make_active_birth_record( ...
                next_id, t_k, b.birth_kind(bb), b.parent_id(bb), oid); %#ok<AGROW>
            % 记录新生种子量测归属于此新ID
            assoc_ids(end+1) = next_id;                    %#ok<AGROW>
            assoc_type{end+1} = 'active_birth';            %#ok<AGROW>
            assoc_meas_idx(end+1) = oid;                   %#ok<AGROW>
            assoc_xyz(:, end+1) = z_k(:, oid);             %#ok<AGROW>
            assoc_tid(end+1) = ids_k(oid);                 %#ok<AGROW>
            assoc_innovation(:, end+1) = nan(z_dim, 1);    %#ok<AGROW>
            assoc_nis(end+1) = NaN;                        %#ok<AGROW>
            assoc_group_size(end+1) = 1;                   %#ok<AGROW>
            assoc_innovation_kind(end+1) = 0;              %#ok<AGROW>
            next_id = next_id + 1;
            b.innov{bb} = zeros(z_dim, 0);
            b.R{bb} = get_meas_R(R_k_meas, oid, R_default);
            % 滑窗种子：出生点(t, xyz)
            b.poshist{bb} = [t_k; z_k(:, oid)];
            % IMM bank 种子：两模型均置于出生态，模型概率 [CV; CA]
            if use_imm
                b.imm{bb} = imm_init_bank(b.m(:, bb), P_birth, imm_mu_init_cv, imm_n_models);
            else
                b.imm{bb} = [];
            end
        end

        trk = trk_append(trk, b);
    end

    est.timing.manage = est.timing.manage + toc(t_birth);

    %% ── 第6步：被动角度更新（在新生之后统一执行，允许同拍新生受益） ──
    t_pb = tic;
    if passive_bearing_enabled
        pb_k = get_passive_bearing(passive_bearing, k);
        if ~isempty(pb_k) && isfield(pb_k, 'n_meas') && pb_k.n_meas > 0
            if trk.N == 0
                passive_bearing_stats.n_input = passive_bearing_stats.n_input + pb_k.n_meas;
                passive_bearing_stats.n_no_track = passive_bearing_stats.n_no_track + pb_k.n_meas;
                n_passive_bearing_skipped_total = n_passive_bearing_skipped_total + pb_k.n_meas;
            else
                allow_by_type = (meta_k.has_active && passive_bearing_update_on_active) || ...
                    (~meta_k.has_active && passive_bearing_update_on_pure);
                allow_existing_by_dt = passive_bearing_min_dt_s <= 0 || ...
                    (t_k - last_passive_bearing_t) >= passive_bearing_min_dt_s;
                allow_newborn_same_event = meta_k.has_active && N_birth > 0 && ...
                    passive_bearing_update_on_active;
                if allow_by_type && (allow_existing_by_dt || allow_newborn_same_event)
                    pb_track_mask = true(1, trk.N);
                    if ~allow_existing_by_dt
                        % 节流期只放行本拍新生，旧航迹仍遵守全局被动更新时间间隔。
                        n_existing = min(pred.N, trk.N);
                        if n_existing > 0
                            pb_track_mask(1:n_existing) = false;
                        end
                    end
                    if ~passive_bearing_update_active_hit_tracks && ~isempty(active_hit_mask)
                        % 旧航迹继续遵守“本拍主动命中后不重复做被动更新”；
                        % 本拍刚出生的航迹没有经过主动滤波更新，保持 eligible=true。
                        n_existing = min([pred.N, numel(active_hit_mask), trk.N]);
                        if n_existing > 0
                            pb_track_mask(1:n_existing) = pb_track_mask(1:n_existing) & ...
                                ~active_hit_mask(1:n_existing);
                        end
                    end
                    if any(pb_track_mask)
                        if use_imm, m_before_pb = trk.m; end
                        [trk, n_pb_used, pb_stats_k, passive_detail_k] = update_passive_bearing(trk, pb_k, t_k, platform, cfg, ...
                            pos_idx, passive_bearing_gate, passive_bearing_nis_gate, ...
                            passive_bearing_weight_gain, passive_bearing_confirm_hit, ...
                            passive_bearing_fast_gate_deg, pb_track_mask);
                        n_passive_bearing_used_total = n_passive_bearing_used_total + n_pb_used;
                        n_passive_bearing_skipped_total = n_passive_bearing_skipped_total + ...
                            max(0, pb_stats_k.n_input - pb_stats_k.n_measurements_used);
                        passive_bearing_stats = add_passive_bearing_stats(passive_bearing_stats, pb_stats_k);
                        if n_pb_used > 0
                            last_passive_bearing_t = t_k;
                        end
                        if use_imm
                            changed = any(abs(trk.m - m_before_pb) > 1e-9, 1);
                            for ti_pb = reshape(find(changed), 1, [])
                                trk.imm{ti_pb} = imm_reseed(trk.imm{ti_pb}, trk.m(:, ti_pb), trk.P(:, :, ti_pb));
                            end
                        end
                    else
                        passive_bearing_stats.n_input = passive_bearing_stats.n_input + pb_k.n_meas;
                        passive_bearing_stats.n_masked = passive_bearing_stats.n_masked + pb_k.n_meas;
                        n_passive_bearing_skipped_total = n_passive_bearing_skipped_total + pb_k.n_meas;
                    end
                else
                    passive_bearing_stats.n_input = passive_bearing_stats.n_input + pb_k.n_meas;
                    if ~allow_by_type
                        passive_bearing_stats.n_type_disabled = passive_bearing_stats.n_type_disabled + pb_k.n_meas;
                    else
                        passive_bearing_stats.n_throttled = passive_bearing_stats.n_throttled + pb_k.n_meas;
                    end
                    n_passive_bearing_skipped_total = n_passive_bearing_skipped_total + pb_k.n_meas;
                end
            end
        end
    end
    if joint_extension_enabled && passive_bearing_enabled && trk.N > 0
        direct_ids = passive_detail_k.updated_active_angle_track_ids;
        grouped_ids = setdiff(passive_detail_k.updated_passive_track_ids, direct_ids);
        [trk, passive_confirm_state, n_direct] = apply_passive_equivalent_hits( ...
            trk, passive_confirm_state, direct_ids, ...
            t_k, count_confirm_cycle, 1, joint_passive_confirm_max_gap_s);
        [trk, passive_confirm_state, n_grouped] = apply_passive_equivalent_hits( ...
            trk, passive_confirm_state, grouped_ids, ...
            t_k, count_confirm_cycle, joint_passive_confirm_hits, ...
            joint_passive_confirm_max_gap_s);
        passive_equivalent_hits_total = passive_equivalent_hits_total + ...
            n_direct + n_grouped;
    end
    est.timing.update = est.timing.update + toc(t_pb);
    t_mgmt = tic;

    %% ── 第7步：去重合并（先合并，避免本可与健康副本合并的航迹被误删） ──
    if trk.N > 1
        trk_before_dedup = trk;
        n_before_dedup = trk.N;
        trk = dedup_tracks(trk, pos_idx, vel_idx, merge_pos_dist_m, ...
                           dedup_vel_angle, dedup_min_speed);
        delete_stats.n_dedup_merged = delete_stats.n_dedup_merged + ...
            (n_before_dedup - trk.N);
        [n_nis_removed, n_secondary_removed] = removed_birth_kind_counts( ...
            trk_before_dedup, trk, BIRTH_NIS_CONFLICT, BIRTH_SECONDARY_CONFLICT);
        birth_stats.n_nis_conflict_deleted = ...
            birth_stats.n_nis_conflict_deleted + n_nis_removed;
        birth_stats.n_secondary_conflict_deleted = ...
            birth_stats.n_secondary_conflict_deleted + n_secondary_removed;
    end

    %% ── 第8步：删除 / 重捕获 ──────────────────────────────────────────
    %  确认航迹 NIS 持续异常时不删，而是膨胀协方差、保留标签去重新捕获(防自残式断裂)；
    %  只有"长时间真丢"(miss超界)才删确认航迹。试探航迹照旧(含NIS发散即删)。
    if trk.N > 0
        isc = (trk.conf == 1);
        bad_nis = (trk.nisbad >= nis_max_bad);

        if confirmed_time_authoritative
            reacq = isc & bad_nis;
        else
            reacq = isc & bad_nis & (trk.miss <= max_coast_frames);
        end
        if any(reacq)
            for rr = find(reacq)
                trk.P(pos_idx, pos_idx, rr) = trk.P(pos_idx, pos_idx, rr) * reacq_P_inflate;
                trk.P(vel_idx, vel_idx, rr) = trk.P(vel_idx, vel_idx, rr) * reacq_vel_inflate;
                trk.P(:, :, rr) = make_spd(trk.P(:, :, rr));
            end
            trk.nisbad(reacq) = 0;        % 清零，给一次重捕获机会
        end

        if confirmed_time_authoritative
            % The configured seconds-based timeout is authoritative.  miss
            % still controls coasting/reacquisition, but must not silently
            % shorten a 6 s wait to roughly six active scans.
            del_conf = false(1, trk.N);
        else
            del_conf = isc & (trk.miss > max_coast_frames);
        end
        tent_nis = (~isc) & bad_nis;
        tent_miss = (~isc) & ~tent_nis & (trk.miss > tent_max_miss);
        tent_weight = (~isc) & ~tent_nis & ~tent_miss & (trk.w < cfg.prune_threshold);
        del_tent = tent_nis | tent_miss | tent_weight;
        delete_stats.n_confirmed_coast = delete_stats.n_confirmed_coast + nnz(del_conf);
        delete_stats.n_tentative_nis = delete_stats.n_tentative_nis + nnz(tent_nis);
        delete_stats.n_tentative_miss = delete_stats.n_tentative_miss + nnz(tent_miss);
        delete_stats.n_tentative_weight = delete_stats.n_tentative_weight + nnz(tent_weight);
        delete_mask = del_conf | del_tent;
        if any(delete_mask)
            trk_before_delete = trk;
            trk = trk_subset(trk, ~delete_mask);
            [n_nis_removed, n_secondary_removed] = removed_birth_kind_counts( ...
                trk_before_delete, trk, BIRTH_NIS_CONFLICT, BIRTH_SECONDARY_CONFLICT);
            birth_stats.n_nis_conflict_deleted = ...
                birth_stats.n_nis_conflict_deleted + n_nis_removed;
            birth_stats.n_secondary_conflict_deleted = ...
                birth_stats.n_secondary_conflict_deleted + n_secondary_removed;
        end
    end

    %% ── 第9步：航迹数上限保护（优先确认航迹） ────────────────────────
    if trk.N > cfg.max_tracks
        trk_before_capacity = trk;
        n_overflow = trk.N - cfg.max_tracks;
        score = trk.conf * 1e6 + trk.w;
        [~, si] = sort(score, 'descend');
        trk = trk_subset(trk, si(1:cfg.max_tracks));
        delete_stats.n_capacity = delete_stats.n_capacity + n_overflow;
        [n_nis_removed, n_secondary_removed] = removed_birth_kind_counts( ...
            trk_before_capacity, trk, BIRTH_NIS_CONFLICT, BIRTH_SECONDARY_CONFLICT);
        birth_stats.n_nis_conflict_deleted = ...
            birth_stats.n_nis_conflict_deleted + n_nis_removed;
        birth_stats.n_secondary_conflict_deleted = ...
            birth_stats.n_secondary_conflict_deleted + n_secondary_removed;
    end

    %% ── 第10步：航迹确认（最近N帧达到M次即立即确认） ─────────────────
    idx_confirmed = [];
    if trk.N > 0
        for tr = 1:trk.N
            if trk.conf(tr) == 1
                idx_confirmed(end+1) = tr; %#ok<AGROW>
            else
                n_hits = sum(trk.S(:, tr) == 1 | trk.S(:, tr) == 4);
                if n_hits >= cfg.M_confirm
                    trk.conf(tr) = 1;
                    trk.w(tr) = max(trk.w(tr), w_floor_conf);
                    if trk.birth_kind(tr) == BIRTH_NIS_CONFLICT
                        birth_stats.n_nis_conflict_confirmed = ...
                            birth_stats.n_nis_conflict_confirmed + 1;
                    elseif trk.birth_kind(tr) == BIRTH_SECONDARY_CONFLICT
                        birth_stats.n_secondary_conflict_confirmed = ...
                            birth_stats.n_secondary_conflict_confirmed + 1;
                    end
                    idx_confirmed(end+1) = tr; %#ok<AGROW>
                end
            end
        end
    end

    %% ── 第11步：存储结果 ──────────────────────────────────────────────
    % IMM 模型概率矩阵 [M×N]（便于诊断；非IMM时为空）
    if use_imm && trk.N > 0
        imm_mu_mat = zeros(imm_n_models, trk.N);
        for ii = 1:trk.N
            if ~isempty(trk.imm{ii}) && isfield(trk.imm{ii}, 'mu')
                imm_mu_mat(:, ii) = trk.imm{ii}.mu(:);
            end
        end
    else
        imm_mu_mat = [];
    end
    est.tracks{k} = struct('m', trk.m, 'P', trk.P, 'w', trk.w, ...
                           'L', trk.L, 'S', trk.S, 'R', {trk.R}, ...
                           'conf', trk.conf, 'miss', trk.miss, 'age', trk.age, ...
                           'nisbad', trk.nisbad, 'vinit', trk.vinit, ...
                           'born_t', trk.born_t, 'born_xyz', trk.born_xyz, ...
                           'last_active_hit_t', trk.last_active_hit_t, ...
                           'last_active_nis_norm', trk.last_active_nis_norm, ...
                           'last_update_t', trk.last_update_t, ...
                           'last_passive_update_t', trk.last_passive_update_t, ...
                           'birth_kind', trk.birth_kind, 'parent_id', trk.parent_id, ...
                           'imm_mu', imm_mu_mat);
    est.assoc{k} = struct('id', assoc_ids, 'xyz', assoc_xyz, 'tid', assoc_tid, ...
        'meas_index', assoc_meas_idx, 'type', {assoc_type}, ...
        'innovation', assoc_innovation, 'nis', assoc_nis, ...
        'group_size', assoc_group_size, 'innovation_kind', assoc_innovation_kind);
    est.passive_assoc{k} = passive_detail_k;
    est.gate{k}  = struct('id', gate_ids, 'pos', gate_pos, 'S', gate_S);

    idx_output_confirmed = idx_confirmed;
    if isfinite(confirmed_output_max_silence_s) && ~isempty(idx_output_confirmed)
        fresh = (t_k - trk.last_update_t(idx_output_confirmed)) <= ...
            confirmed_output_max_silence_s;
        n_output_stale_suppressed = n_output_stale_suppressed + sum(~fresh);
        idx_output_confirmed = idx_output_confirmed(fresh);
    end
    if ~isempty(idx_output_confirmed)
        est.X{k} = trk.m(:, idx_output_confirmed);
        est.P{k} = trk.P(:, :, idx_output_confirmed);
        est.N(k) = numel(idx_output_confirmed);
        est.L{k} = trk.L(:, idx_output_confirmed)';
    else
        est.X{k} = []; est.P{k} = []; est.N(k) = 0; est.L{k} = [];
    end

    est.timing.manage = est.timing.manage + toc(t_mgmt);

    if joint_extension_enabled
        residual_event = make_online_residual_event(k, t_k, passive_bearing, ...
            passive_detail_k, meta_k, cfg, trk, idx_confirmed, platform);
        [joint2d_chunk, joint2d_state] = run_filter_joint_2d3d( ...
            residual_event, platform, joint2d_cfg, joint2d_state);
        est.joint2d = store_online_joint2d_chunk(est.joint2d, ...
            joint2d_chunk, joint2d_state, residual_event, k);
    end

    %% ── 进度打印 ──────────────────────────────────────────────────────
    if mod(k, max(1, floor(K/10))) == 0 || k == K
        n_conf_alive = sum(trk.conf == 1);
        fprintf('  帧 %d/%d (t=%.2f s), dt=%.4f s, 航迹=%d(确认存活=%d), 输出=%d, 量测=%d\n', ...
            k, K, t_k, dt, trk.N, n_conf_alive, est.N(k), n_meas_k);
    end

    prev_t = t_k;
end

%% ── 耗时统计 ──────────────────────────────────────────────────────────
est.timing.total = est.timing.predict + est.timing.update + est.timing.manage;
est.passive_bearing_stats = passive_bearing_stats;
est.passive_bearing_stats.n_equivalent_confirm_hits = passive_equivalent_hits_total;
est.track_timeout_stats = make_track_timeout_stats(track_timeout_enabled, ...
    tentative_max_silence_s, confirmed_max_silence_s, ...
    n_timeout_tentative_total, n_timeout_confirmed_total, timeout_records);
birth_stats.n_total = n_birth_total;
active_assoc_stats.n_nis_rejected_total = n_assoc_nis_rejected_total;
active_assoc_stats.n_secondary_fused = n_secondary_fused_total;
active_assoc_stats.n_secondary_released = n_secondary_released_total;
active_assoc_stats.nis_accounting_ok = n_assoc_nis_rejected_total == ...
    active_assoc_stats.n_preassignment_conflict_measurements + ...
    active_assoc_stats.n_primary_group_rejected + ...
    active_assoc_stats.n_postvalidation_inconsistency;
birth_stats.n_nis_conflict_alive = nnz(trk.birth_kind == BIRTH_NIS_CONFLICT);
birth_stats.n_secondary_conflict_alive = nnz(trk.birth_kind == BIRTH_SECONDARY_CONFLICT);
birth_stats.source_accounting_ok = birth_stats.n_total == ...
    birth_stats.n_ordinary + birth_stats.n_nis_conflict + birth_stats.n_secondary_conflict;
birth_stats.lifecycle_accounting_ok = ...
    birth_stats.n_nis_conflict == birth_stats.n_nis_conflict_deleted + ...
        birth_stats.n_nis_conflict_alive && ...
    birth_stats.n_secondary_conflict == birth_stats.n_secondary_conflict_deleted + ...
        birth_stats.n_secondary_conflict_alive;
delete_stats.n_total = delete_stats.n_timeout_tentative + delete_stats.n_timeout_confirmed + ...
    delete_stats.n_confirmed_coast + delete_stats.n_tentative_nis + ...
    delete_stats.n_tentative_miss + delete_stats.n_tentative_weight + ...
    delete_stats.n_dedup_merged + delete_stats.n_capacity;
est.birth_stats = birth_stats;
est.active_assoc_stats = active_assoc_stats;
est.track_delete_stats = delete_stats;
est.end_of_stream = make_end_of_stream_status(trk, frame_times(end), ...
    tentative_max_silence_s, confirmed_max_silence_s);
est.output_freshness = struct('max_silence_s', ...
    confirmed_output_max_silence_s, ...
    'n_suppressed', n_output_stale_suppressed);
fprintf('\n滤波完成: 预测=%.1fs, 更新=%.1fs, 管理=%.1fs, 总计=%.1fs\n', ...
    est.timing.predict, est.timing.update, est.timing.manage, est.timing.total);
if passive_bearing_enabled
    fprintf('被动bearing-only更新成功: %d 次 (消耗角度量测=%d, 完成CKF更新=%d次)\n', ...
        n_passive_bearing_used_total, passive_bearing_stats.n_measurements_used, ...
        passive_bearing_stats.n_single_updates);
    fprintf('被动bearing-only跳过量测: %d 个 (门外=%d, 更新拒绝=%d, 节流=%d)\n', ...
        n_passive_bearing_skipped_total, passive_bearing_stats.n_gate_rejected, ...
        passive_bearing_stats.n_update_rejected, passive_bearing_stats.n_throttled);
end
if n_output_stale_suppressed > 0
    fprintf('确认航迹过期输出抑制: %d 次 (输出新鲜度<=%.3f s)\n', ...
        n_output_stale_suppressed, confirmed_output_max_silence_s);
end
fprintf('新生航迹: %d 条, 新生抑制(门内未分配=%d, 出生半径=%d)\n', ...
    n_birth_total, n_birth_suppressed_gated, n_birth_suppressed_guard);
fprintf('  新生来源: 普通=%d, NIS冲突=%d(确认=%d,已删除=%d), 次回波冲突=%d(确认=%d,已删除=%d)\n', ...
    birth_stats.n_ordinary, birth_stats.n_nis_conflict, ...
    birth_stats.n_nis_conflict_confirmed, birth_stats.n_nis_conflict_deleted, ...
    birth_stats.n_secondary_conflict, birth_stats.n_secondary_conflict_confirmed, ...
    birth_stats.n_secondary_conflict_deleted);
if ~birth_stats.source_accounting_ok || ~birth_stats.lifecycle_accounting_ok
    warning('run_filter_adapt_ckf:BirthAccounting', ...
        'Birth source/lifecycle accounting invariant failed; inspect est.birth_stats.');
end
fprintf('主动关联验收: NIS拒绝=%d, 次回波追加=%d, 次回波一致性/回退释放=%d\n', ...
    n_assoc_nis_rejected_total, n_secondary_fused_total, n_secondary_released_total);
fprintf('  NIS拒绝拆分: 分配前冲突=%d, 主组验收拒绝=%d, 验证后数值不一致=%d\n', ...
    active_assoc_stats.n_preassignment_conflict_measurements, ...
    active_assoc_stats.n_primary_group_rejected, ...
    active_assoc_stats.n_postvalidation_inconsistency);
if ~active_assoc_stats.nis_accounting_ok
    warning('run_filter_adapt_ckf:NISAccounting', ...
        'NIS rejection accounting invariant failed; inspect est.active_assoc_stats.');
end
fprintf('  次回波释放拆分: 配对门冲突=%d, 融合组回退=%d\n', ...
    active_assoc_stats.n_secondary_pair_conflict, ...
    active_assoc_stats.n_secondary_group_released);
fprintf('  分层主关联: 确认=%d, 普通试探=%d, 冲突试探=%d, 二次分配=%d/%d\n', ...
    active_assoc_stats.n_confirmed_primary, active_assoc_stats.n_tentative_primary, ...
    active_assoc_stats.n_conflict_primary, active_assoc_stats.n_reassign_updates, ...
    active_assoc_stats.n_reassign_attempted);
if ~isempty(active_assoc_stats.secondary_pair_nis)
    qn = simple_percentiles(active_assoc_stats.secondary_pair_nis, [0.50, 0.90, 0.95, 0.99]);
    qd = simple_percentiles(active_assoc_stats.secondary_pair_dist_m, [0.50, 0.90, 0.95, 0.99]);
    fprintf('  次回波通过配对门分位数 P50/P90/P95/P99: pairNIS=[%.2f %.2f %.2f %.2f], 距离=[%.0f %.0f %.0f %.0f]m\n', ...
        qn(1), qn(2), qn(3), qn(4), qd(1), qd(2), qd(3), qd(4));
end
fprintf('航迹秒级超时删除: 试探=%d, 确认=%d\n', ...
    n_timeout_tentative_total, n_timeout_confirmed_total);
fprintf('航迹删除明细: 确认帧级=%d, 试探(miss=%d,NIS=%d,weight=%d), 去重=%d, 容量=%d\n', ...
    delete_stats.n_confirmed_coast, delete_stats.n_tentative_miss, ...
    delete_stats.n_tentative_nis, delete_stats.n_tentative_weight, ...
    delete_stats.n_dedup_merged, delete_stats.n_capacity);

%% ── 统计确认航迹 ──────────────────────────────────────────────────────
total_confirmed = 0;
active_tracks = [];
for k = 1:K
    if est.N(k) > 0
        total_confirmed = total_confirmed + est.N(k);
        for i = 1:size(est.L{k}, 1)
            active_tracks = [active_tracks; est.L{k}(i, 2)]; %#ok<AGROW>
        end
    end
end
unique_tracks = unique(active_tracks);
fprintf('确认航迹总输出: %d 次, 唯一航迹ID: %d 个\n', total_confirmed, numel(unique_tracks));
if ~isempty(unique_tracks)
    fprintf('航迹ID范围: %d ~ %d\n', min(unique_tracks), max(unique_tracks));
end

end

%% ═══════════════════════════════════════════════════════════════════════════
%  航迹结构体助手（统一管理所有逐航迹属性，避免并行数组错位）
%% ═══════════════════════════════════════════════════════════════════════════
function records = empty_track_timeout_records()
records = repmat(track_timeout_record_template(), 0, 1);
end

function rec = track_timeout_record_template()
rec = struct('track_id', nan, 'was_confirmed', false, ...
    'timeout_time', nan, 'last_active_hit_t', nan, 'last_update_t', nan, ...
    'silence_s', nan, 'limit_s', nan, 'deadline_t', nan);
end

function stats = make_track_timeout_stats(enabled, tentative_limit_s, confirmed_limit_s, ...
        n_tentative, n_confirmed, records)
stats = struct('enabled', logical(enabled), ...
    'tentative_max_silence_s', tentative_limit_s, ...
    'confirmed_max_silence_s', confirmed_limit_s, ...
    'n_tentative', n_tentative, 'n_confirmed', n_confirmed, ...
    'n_total', n_tentative + n_confirmed, ...
    'enforcement', 'pre_association_on_next_event', 'records', records);
end

function stats = empty_active_birth_stats()
stats = struct('n_total', 0, 'n_ordinary', 0, 'n_nis_conflict', 0, ...
    'n_secondary_conflict', 0, 'n_nis_conflict_confirmed', 0, ...
    'n_secondary_conflict_confirmed', 0, 'n_nis_conflict_deleted', 0, ...
    'n_secondary_conflict_deleted', 0, 'n_nis_conflict_alive', 0, ...
    'n_secondary_conflict_alive', 0, 'source_accounting_ok', true, ...
    'lifecycle_accounting_ok', true, ...
    'records', repmat(active_birth_record_template(), 0, 1));
end

function rec = active_birth_record_template()
rec = struct('track_id', nan, 'time', nan, 'birth_kind', 0, ...
    'parent_track_id', nan, 'measurement_index', nan);
end

function rec = make_active_birth_record(track_id, t, birth_kind, parent_id, meas_idx)
rec = active_birth_record_template();
rec.track_id = track_id;
rec.time = t;
rec.birth_kind = birth_kind;
rec.parent_track_id = parent_id;
rec.measurement_index = meas_idx;
end

function stats = empty_active_assoc_stats()
stats = struct('n_edges_over_accept_nis', 0, 'n_confirmed_primary', 0, ...
    'n_tentative_primary', 0, 'n_conflict_primary', 0, ...
    'n_preassignment_conflict_measurements', 0, ...
    'n_primary_group_rejected', 0, ...
    'n_reassign_attempted', 0, 'n_reassign_success', 0, ...
    'n_reassign_updates', 0, 'n_postvalidation_inconsistency', 0, ...
    'n_secondary_used', 0, 'n_nis_rejected_total', 0, ...
    'n_secondary_fused', 0, 'n_secondary_released', 0, ...
    'n_secondary_pair_conflict', 0, 'n_secondary_group_released', 0, ...
    'nis_accounting_ok', true, ...
    'secondary_pair_nis', zeros(1, 0), ...
    'secondary_pair_dist_m', zeros(1, 0));
end

function stats = empty_track_delete_stats()
stats = struct('n_total', 0, 'n_timeout_tentative', 0, ...
    'n_timeout_confirmed', 0, 'n_confirmed_coast', 0, ...
    'n_tentative_nis', 0, 'n_tentative_miss', 0, ...
    'n_tentative_weight', 0, 'n_dedup_merged', 0, 'n_capacity', 0);
end

function eos = make_end_of_stream_status(trk, t_end, tentative_limit_s, confirmed_limit_s)
eos = struct('time', t_end, 'track_id', zeros(1, 0), ...
    'confirmed', false(1, 0), 'last_active_hit_t', zeros(1, 0), ...
    'last_update_t', zeros(1, 0), 'scheduled_deadline', zeros(1, 0));
if trk.N == 0
    return;
end
eos.track_id = trk.L(2, :);
eos.confirmed = logical(trk.conf);
eos.last_active_hit_t = trk.last_active_hit_t;
eos.last_update_t = trk.last_update_t;
eos.scheduled_deadline = trk.last_active_hit_t + tentative_limit_s;
idx = trk.conf == 1;
eos.scheduled_deadline(idx) = trk.last_update_t(idx) + confirmed_limit_s;
end

function qv = simple_percentiles(v, q)
v = sort(v(isfinite(v)));
qv = nan(size(q));
if isempty(v)
    return;
end
n = numel(v);
for i = 1:numel(q)
    x = 1 + min(max(q(i), 0), 1) * (n - 1);
    lo = floor(x); hi = ceil(x);
    if lo == hi
        qv(i) = v(lo);
    else
        qv(i) = v(lo) + (x - lo) * (v(hi) - v(lo));
    end
end
end

function [n_nis, n_secondary] = removed_birth_kind_counts(before, after, nis_kind, secondary_kind)
% Count removed IDs, rather than the net track-count change, so that a
% merge/replacement cannot hide which conflict candidate actually left.
if before.N == 0
    n_nis = 0;
    n_secondary = 0;
    return;
end
removed = ~ismember(before.L(2, :), after.L(2, :));
n_nis = nnz(removed & before.birth_kind == nis_kind);
n_secondary = nnz(removed & before.birth_kind == secondary_kind);
end

function meta = default_event_meta(K)
meta = repmat(struct('has_active', true, 'has_passive', false, ...
    'miss_cycle', true, 'confirm_cycle', true, ...
    'n_active', 0, 'n_passive', 0, 't_start', NaN, 't_end', NaN), K, 1);
end

function meta = get_event_meta(event_meta, k, n_active)
if isempty(event_meta) || numel(event_meta) < k || ~isstruct(event_meta(k))
    meta = default_event_meta(1);
    meta.n_active = n_active;
    return;
end
meta = event_meta(k);
if ~isfield(meta, 'has_active'), meta.has_active = n_active > 0; end
if ~isfield(meta, 'has_passive'), meta.has_passive = false; end
if ~isfield(meta, 'miss_cycle'), meta.miss_cycle = meta.has_active; end
if ~isfield(meta, 'confirm_cycle'), meta.confirm_cycle = meta.has_active; end
if ~isfield(meta, 'n_active'), meta.n_active = n_active; end
if ~isfield(meta, 'n_passive'), meta.n_passive = 0; end
if ~isfield(meta, 't_start'), meta.t_start = NaN; end
if ~isfield(meta, 't_end'), meta.t_end = NaN; end
end

function t = trk_init(x_dim)
t.m = zeros(x_dim, 0);
t.P = zeros(x_dim, x_dim, 0);
t.w = zeros(1, 0);
t.L = zeros(2, 0);
t.S = zeros(0, 0);
t.innov = {};
t.R = {};
t.conf = zeros(1, 0);
t.miss = zeros(1, 0);
t.age  = zeros(1, 0);
t.nisbad = zeros(1, 0);
t.last_active_nis_norm = zeros(1, 0);
t.born_xyz = zeros(3, 0);
t.born_t = zeros(1, 0);
t.last_active_hit_t = zeros(1, 0);
t.last_update_t = zeros(1, 0);
t.last_passive_update_t = zeros(1, 0);
t.birth_kind = zeros(1, 0); % 0=ordinary, 1=NIS conflict, 2=secondary conflict
t.parent_id = zeros(1, 0);  % originating mature track when the birth was a conflict
t.vinit = zeros(1, 0);
t.poshist = {};          % 改进A：每条航迹的位置-时间滑窗，元素 [4×n]=[t; E; N; U]
t.imm = {};              % 改进B：每条航迹的IMM模型bank, struct('x','P','mu')；非IMM时为[]
t.N = 0;
end

function t = trk_subset(t, idx)
t.m = t.m(:, idx);
t.P = t.P(:, :, idx);
t.w = t.w(idx);
t.L = t.L(:, idx);
t.S = t.S(:, idx);
t.innov = t.innov(idx);
t.R = t.R(idx);
t.conf = t.conf(idx);
t.miss = t.miss(idx);
t.age  = t.age(idx);
t.nisbad = t.nisbad(idx);
t.last_active_nis_norm = t.last_active_nis_norm(idx);
t.born_xyz = t.born_xyz(:, idx);
t.born_t = t.born_t(idx);
t.last_active_hit_t = t.last_active_hit_t(idx);
t.last_update_t = t.last_update_t(idx);
t.last_passive_update_t = t.last_passive_update_t(idx);
t.birth_kind = t.birth_kind(idx);
t.parent_id = t.parent_id(idx);
t.vinit = t.vinit(idx);
t.poshist = t.poshist(idx);
t.imm = t.imm(idx);
t.N = numel(t.w);
end

function t = trk_append(a, b)
if a.N == 0, t = b; return; end
if b.N == 0, t = a; return; end
t.m = cat(2, a.m, b.m);
t.P = cat(3, a.P, b.P);
t.w = [a.w, b.w];
t.L = [a.L, b.L];
t.S = [a.S, b.S];
t.innov = [a.innov, b.innov];
t.R = [a.R, b.R];
t.conf = [a.conf, b.conf];
t.miss = [a.miss, b.miss];
t.age  = [a.age, b.age];
t.nisbad = [a.nisbad, b.nisbad];
t.last_active_nis_norm = [a.last_active_nis_norm, b.last_active_nis_norm];
t.born_xyz = [a.born_xyz, b.born_xyz];
t.born_t = [a.born_t, b.born_t];
t.last_active_hit_t = [a.last_active_hit_t, b.last_active_hit_t];
t.last_update_t = [a.last_update_t, b.last_update_t];
t.last_passive_update_t = [a.last_passive_update_t, b.last_passive_update_t];
t.birth_kind = [a.birth_kind, b.birth_kind];
t.parent_id = [a.parent_id, b.parent_id];
t.vinit = [a.vinit, b.vinit];
t.poshist = [a.poshist, b.poshist];
t.imm = [a.imm, b.imm];
t.N = a.N + b.N;
end

%% ═══════════════════════════════════════════════════════════════════════════
function trk = dedup_tracks(trk, pos_idx, vel_idx, dist_thresh, vel_angle_gate, ...
        min_speed)
%DEDUP_TRACKS  按位置距离去重，确认航迹主导标签，状态取最新被喂养子航迹。
%  交叉保护：两条都已确认且都在运动的航迹，需速度方向一致才合并。
N = trk.N;
if N <= 1, return; end

prio = trk.conf * 1e6 + trk.age + 1e-3 * trk.w;
[~, order] = sort(prio, 'descend');

labeled = false(1, N);
clusters = {};
for oi = 1:N
    i = order(oi);
    if labeled(i), continue; end
    labeled(i) = true;
    members = i;
    pc = trk.m(pos_idx, i);
    for oj = 1:N
        j = order(oj);
        if labeled(j), continue; end
        if norm(trk.m(pos_idx, j) - pc) < dist_thresh && ...
                can_merge(trk, i, j, vel_idx, vel_angle_gate, min_speed)
            members(end+1) = j; %#ok<AGROW>
            labeled(j) = true;
        end
    end
    clusters{end+1} = members; %#ok<AGROW>
end

n_out = numel(clusters);
xd = size(trk.m, 1);
out = trk_init(xd);
out.m = zeros(xd, n_out);
out.P = zeros(xd, xd, n_out);
out.w = zeros(1, n_out);
out.L = zeros(2, n_out);
out.S = zeros(size(trk.S, 1), n_out);
out.innov = cell(1, n_out);
out.R = cell(1, n_out);
out.conf = zeros(1, n_out);
out.miss = zeros(1, n_out);
out.age  = zeros(1, n_out);
out.nisbad = zeros(1, n_out);
out.last_active_nis_norm = nan(1, n_out);
out.born_xyz = zeros(3, n_out);
out.born_t = zeros(1, n_out);
out.last_active_hit_t = zeros(1, n_out);
out.last_update_t = zeros(1, n_out);
out.last_passive_update_t = nan(1, n_out);
out.birth_kind = zeros(1, n_out);
out.parent_id = nan(1, n_out);
out.vinit = zeros(1, n_out);
out.poshist = cell(1, n_out);
out.imm = cell(1, n_out);

for c = 1:n_out
    mem = clusters{c};
    cm = trk.conf(mem); am = trk.age(mem); wm = trk.w(mem); mm = trk.miss(mem);

    % 标签来源：确认 > 年龄大 > 权重高
    [~, li] = max(cm * 1e6 + am + 1e-3 * wm);
    lead = mem(li);
    % 辅助字段来源：漏检最少(最新) > 权重高
    [~, bi] = min(mm - 1e-3 * wm);
    body = mem(bi);

    % ── 状态：协方差交叉(CI)融合；对相关未知的重复航迹保持一致(不过度自信) ──
    nm = numel(mem);
    if nm == 1
        out.m(:, c)    = trk.m(:, body);
        out.P(:, :, c) = make_spd(trk.P(:, :, body));
    else
        omega = zeros(1, nm);
        for q = 1:nm
            omega(q) = 1 / max(trace(make_spd(trk.P(:, :, mem(q)))), realmin);
        end
        omega = omega / sum(omega);
        Pinfo = zeros(xd); xinfo = zeros(xd, 1);
        for q = 1:nm
            Wi = omega(q) * inv(make_spd(trk.P(:, :, mem(q))));
            Pinfo = Pinfo + Wi;
            xinfo = xinfo + Wi * trk.m(:, mem(q));
        end
        Pf = inv(make_spd(Pinfo));
        out.m(:, c)    = Pf * xinfo;
        out.P(:, :, c) = make_spd(Pf);
    end

    % 命中历史：主动命中优先；没有主动命中时保留被动等效逻辑命中。
    sc = trk.S(:, body);
    active_rows = any(trk.S(:, mem) == 1, 2);
    passive_rows = ~active_rows & any(trk.S(:, mem) == 4, 2);
    sc(active_rows) = 1;
    sc(passive_rows) = 4;
    out.S(:, c)    = sc;

    out.innov{c}   = trk.innov{body};
    out.R{c}       = trk.R{body};
    out.born_xyz(:, c) = trk.born_xyz(:, body);
    out.born_t(c)  = trk.born_t(body);
    out.last_active_hit_t(c) = max(trk.last_active_hit_t(mem));
    out.last_update_t(c) = max(trk.last_update_t(mem));
    finite_passive_t = trk.last_passive_update_t(mem);
    finite_passive_t = finite_passive_t(isfinite(finite_passive_t));
    if ~isempty(finite_passive_t)
        out.last_passive_update_t(c) = max(finite_passive_t);
    end
    out.birth_kind(c) = trk.birth_kind(lead);
    out.parent_id(c) = trk.parent_id(lead);
    out.vinit(c)   = trk.vinit(body);
    out.nisbad(c)  = trk.nisbad(body);
    out.last_active_nis_norm(c) = trk.last_active_nis_norm(body);
    out.poshist{c} = trk.poshist{body};       % 位置历史沿用最新被喂养的子航迹
    % IMM bank：把各模型重置到CI融合后的组合态，模型概率沿用 body(下一帧重新展开)
    if ~isempty(trk.imm) && numel(trk.imm) >= body && ~isempty(trk.imm{body})
        out.imm{c} = imm_reseed(trk.imm{body}, out.m(:, c), out.P(:, :, c));
    else
        out.imm{c} = [];
    end

    out.L(:, c)    = trk.L(:, lead);
    out.conf(c)    = max(cm);
    out.miss(c)    = min(mm);
    out.age(c)     = max(am);
    out.w(c)       = max(wm);       % 取最大而非求和，避免杂波簇被"喂"成确认
end
out.N = n_out;
trk = out;
end

function tf = can_merge(trk, i, j, vel_idx, vel_angle_gate, min_speed)
% 交叉保护：两条航迹只要都在运动，就需速度方向一致才允许合并(对所有配对生效，
% 含试探态)，避免邻近/交叉的不同目标被错误并轨。
tf = true;
vi = trk.m(vel_idx, i); vj = trk.m(vel_idx, j);
si = norm(vi); sj = norm(vj);
if si > min_speed && sj > min_speed
    cosang = dot(vi, vj) / (si * sj);
    ang = acosd(max(-1, min(1, cosang)));
    if ang > vel_angle_gate
        tf = false;
    end
end
end

%% ═══════════════════════════════════════════════════════════════════════════
%  改进A辅助函数
%% ═══════════════════════════════════════════════════════════════════════════
function hist = push_poshist(hist, t_k, pos, max_n)
% 把 (t_k, pos) 压入位置-时间滑窗，按列时间升序，仅保留最近 max_n 个样本。
col = [t_k; pos(:)];
if isempty(hist)
    hist = col;
else
    hist = [hist, col];
end
if size(hist, 2) > max_n
    hist = hist(:, end - max_n + 1 : end);
end
end

function [v, ok] = window_velocity(hist, min_n, min_span_s, max_n)
% 对位置-时间滑窗做逐轴最小二乘线性拟合，斜率即速度趋势(大小+方向)。
% hist: [4×n] = [t; E; N; U]，列按时间升序。
v = [0; 0; 0];
ok = false;
if isempty(hist)
    return;
end
n = size(hist, 2);
if n > max_n
    hist = hist(:, end - max_n + 1 : end);
    n = max_n;
end
if n < min_n
    return;
end
t = hist(1, :);
span = t(end) - t(1);
if ~isfinite(span) || span < min_span_s
    return;
end
tc = t - mean(t);
denom = sum(tc .^ 2);
if denom <= 0
    return;
end
for d = 1:3
    pd = hist(d + 1, :);
    v(d) = sum(tc .* (pd - mean(pd))) / denom;   % 最小二乘斜率
end
if any(~isfinite(v))
    v = [0; 0; 0];
    return;
end
ok = true;
end

%% ═══════════════════════════════════════════════════════════════════════════
%  数值/模型工具函数
%% ═══════════════════════════════════════════════════════════════════════════
function v = get_cfg_field(cfg, name, default_value)
if isfield(cfg, name) && ~isempty(cfg.(name)) && ...
        (isnumeric(cfg.(name)) || islogical(cfg.(name))) && ...
        isscalar(cfg.(name)) && isfinite(double(cfg.(name)))
    v = cfg.(name);
else
    v = default_value;
end
end

function v = get_cfg_finite_or_posinf_field(cfg, name, default_value)
if isfield(cfg, name) && ~isempty(cfg.(name)) && ...
        (isnumeric(cfg.(name)) || islogical(cfg.(name))) && ...
        isscalar(cfg.(name)) && ...
        (isfinite(double(cfg.(name))) || double(cfg.(name)) == inf)
    v = cfg.(name);
else
    v = default_value;
end
end

function v = get_cfg_text_field(cfg, name, default_value)
if isfield(cfg, name) && ~isempty(cfg.(name))
    raw = cfg.(name);
    if isa(raw, 'string') && isscalar(raw)
        raw = char(raw);
    end
    if ischar(raw)
        v = lower(strtrim(raw));
        return;
    end
end
v = lower(default_value);
end

function stats = empty_passive_bearing_stats()
stats = struct('n_events_attempted', 0, 'n_input', 0, ...
    'n_updates', 0, 'n_measurements_used', 0, ...
    'n_single_updates', 0, 'n_gate_rejected', 0, ...
    'n_update_rejected', 0, 'n_no_track', 0, 'n_masked', 0, ...
    'n_type_disabled', 0, 'n_throttled', 0);
end

function total = add_passive_bearing_stats(total, delta)
names = fieldnames(total);
for i = 1:numel(names)
    name = names{i};
    if isfield(delta, name) && isscalar(delta.(name)) && isfinite(delta.(name))
        total.(name) = total.(name) + delta.(name);
    end
end
end

function miss_decay = get_miss_decay(cfg)
if isfield(cfg, 'missed_weight_decay') && ~isempty(cfg.missed_weight_decay)
    miss_decay = cfg.missed_weight_decay;
else
    miss_decay = max(0.5, 1 - cfg.P_D);
end
miss_decay = min(max(miss_decay, 0), 1);
end

function F = build_CA_F(dt)
A0 = [1, dt, dt^2/2;
      0, 1,  dt;
      0, 0,  1];
F = kron(eye(3), A0);
end

function Q = build_CS_Q(m, dt, cfg)
x_dim = size(m, 1);
N = size(m, 2);
Q = zeros(x_dim, x_dim, N);

Q0 = [dt^5/20, dt^4/8, dt^3/6;
      dt^4/8,  dt^3/3, dt^2/2;
      dt^3/6,  dt^2/2, dt];

cs_gain = (4 - pi) / pi;
alpha = cfg.alpha_cs;
a_max = cfg.a_max_cs;
sigma_a_base = cfg.sigma_a_cs;

for i = 1:N
    ax = m(3, i); ay = m(6, i); az = m(9, i);
    a_xy = sqrt(ax^2 + ay^2);
    a_xy_eff = min(a_xy, a_max);
    a_z_eff  = min(abs(az), a_max);
    q_xy = max(sigma_a_base^2 + 2 * alpha * cs_gain * a_xy_eff^2, 1e-4);
    q_z  = max(sigma_a_base^2 + 2 * alpha * cs_gain * a_z_eff^2,  1e-4);

    sigma_floor = 10;
    if isfield(cfg, 'sigma_pos_noise') && ~isempty(cfg.sigma_pos_noise)
        sigma_floor = cfg.sigma_pos_noise;
    end
    q_pos_floor = sigma_floor^2 * dt;

    Q_i = zeros(x_dim);
    Q_i(1:3, 1:3) = q_xy * Q0;
    Q_i(4:6, 4:6) = q_xy * Q0;
    Q_i(7:9, 7:9) = q_z  * Q0;
    Q_i(1, 1) = Q_i(1, 1) + q_pos_floor;
    Q_i(4, 4) = Q_i(4, 4) + q_pos_floor;
    Q_i(7, 7) = Q_i(7, 7) + q_pos_floor;
    Q(:, :, i) = Q_i;
end
end

%% ═══════════════════════════════════════════════════════════════════════════
%  改进B：CA+CV 等维 IMM 多模型核心函数（模型1=CV, 模型2=CA）
%% ═══════════════════════════════════════════════════════════════════════════
function F = build_CV_F(dt)
% 等维 CV：位置积分速度，速度恒定，加速度不建模(置零)
A0 = [1, dt, 0; 0, 1, 0; 0, 0, 0];
F = kron(eye(3), A0);
end

function Q = build_CV_Q(dt, cfg)
% 白噪声加速度模型(作用于 pos-vel)，加速度轴给极小地板保持 SPD
sigma_a   = get_cfg_field(cfg, 'imm_cv_sigma_a',   3);
acc_floor = get_cfg_field(cfg, 'imm_cv_acc_floor', 1e-3);
q = sigma_a^2;
B0 = [dt^3/3, dt^2/2, 0; dt^2/2, dt, 0; 0, 0, max(acc_floor, 1e-6)];
Q = kron(eye(3), q * B0);
sigma_floor = 10;
if isfield(cfg, 'sigma_pos_noise') && ~isempty(cfg.sigma_pos_noise)
    sigma_floor = cfg.sigma_pos_noise;
end
qpf = sigma_floor^2 * dt;
Q(1,1)=Q(1,1)+qpf; Q(4,4)=Q(4,4)+qpf; Q(7,7)=Q(7,7)+qpf;
end

function bank = imm_init_bank(m0, P0, mu_cv, M)
% 新生：两模型均置于出生态，模型概率 [CV; CA]
bank.x = repmat(m0(:), 1, M);
bank.P = repmat(make_spd(P0), 1, 1, M);
mu = zeros(M, 1);
mu(1) = mu_cv; mu(2) = 1 - mu_cv;       % 模型1=CV, 模型2=CA
if M > 2, mu(:) = 1/M; end
bank.mu = mu;
end

function bank = imm_reseed(bank, m, P)
% 把各模型重置到给定的组合态(均值/协方差)，保留模型概率。
M = numel(bank.mu);
bank.x = repmat(m(:), 1, M);
bank.P = repmat(make_spd(P), 1, 1, M);
end

function [m, P] = imm_combine(x, Pb, mu)
% 模型条件估计 → 组合(矩匹配)估计
M = numel(mu); xd = size(x, 1); mu = mu(:);
m = x * mu;
P = zeros(xd);
for j = 1:M
    dx = x(:, j) - m;
    P = P + mu(j) * (Pb(:, :, j) + dx * dx');
end
P = make_spd(P);
end

function [bp, m_pred, P_pred] = imm_predict_bank(bank, dt, cfg, TPM, ...
        vtrend, blend, do_coast, coast_decay, vel_idx, acc_idx)
% IMM 交互(混合) + 各模型预测 + 组合。
% bank: struct('x'[xd×M],'P'[xd×xd×M],'mu'[M×1])
M = numel(bank.mu); xd = size(bank.x, 1); mu = bank.mu(:);
cbar = TPM' * mu;                                    % 预测模型概率
Wmix = zeros(M, M);
for j = 1:M
    if cbar(j) > 1e-12
        Wmix(:, j) = (TPM(:, j) .* mu) / cbar(j);
    else
        Wmix(:, j) = 1 / M; cbar(j) = max(cbar(j), 1e-12);
    end
end
% 混合初值
x0 = zeros(xd, M); P0 = zeros(xd, xd, M);
for j = 1:M
    for i = 1:M, x0(:, j) = x0(:, j) + Wmix(i, j) * bank.x(:, i); end
end
for j = 1:M
    for i = 1:M
        dx = bank.x(:, i) - x0(:, j);
        P0(:, :, j) = P0(:, :, j) + Wmix(i, j) * (bank.P(:, :, i) + dx * dx');
    end
    P0(:, :, j) = make_spd(P0(:, :, j));
end
% 改进A：滑窗速度趋势(对两模型同等施加)
if ~isempty(vtrend)
    for j = 1:M
        x0(vel_idx, j) = (1 - blend) * x0(vel_idx, j) + blend * vtrend(:);
    end
end
% 各模型预测：1=CV, 2=CA
F_cv = build_CV_F(dt);  Q_cv = build_CV_Q(dt, cfg);
F_ca = build_CA_F(dt);  Q_ca_all = build_CS_Q(x0(:, 2), dt, cfg);  Q_ca = Q_ca_all(:, :, 1);
Fs = {F_cv, F_ca};  Qs = {Q_cv, Q_ca};
bp.x = zeros(xd, M); bp.P = zeros(xd, xd, M); bp.mu = cbar / sum(cbar);
for j = 1:M
    [bp.x(:, j), bp.P(:, :, j)] = ckf_predict_linear(x0(:, j), P0(:, :, j), Fs{j}, Qs{j});
end
% 滑行阻尼：CA 模型(2)加速度向 0 衰减
if do_coast
    bp.x(acc_idx, 2) = coast_decay * bp.x(acc_idx, 2);
end
[m_pred, P_pred] = imm_combine(bp.x, bp.P, bp.mu);
end

function [bu, m_upd, P_upd] = imm_update_bank(bp, z, pos_idx, R)
% 各模型量测更新 + 模型概率更新 + 组合
M = numel(bp.mu); xd = size(bp.x, 1); mu = bp.mu(:);
llh = -inf(M, 1);
bu.x = zeros(xd, M); bu.P = zeros(xd, xd, M);
for j = 1:M
    [l, xu, Pu] = ckf_update_cartesian(z, R, bp.x(:, j), bp.P(:, :, j), pos_idx);
    bu.x(:, j) = xu; bu.P(:, :, j) = Pu; llh(j) = l;
end
w = exp(llh - max(llh));            % 似然(数值稳定)
post = mu .* w; s = sum(post);
if ~isfinite(s) || s <= 0, post = mu; s = sum(post); end
bu.mu = post / s;
[m_upd, P_upd] = imm_combine(bu.x, bu.P, bu.mu);
end

function R_use = active_track_R(pred, ti, R_meas, Pzz_pred, cfg, R_min_diag)
% Use one covariance definition for assignment and the eventual update.
z_dim = size(R_meas, 1);
if cfg.adapt_R_enabled && size(pred.innov{ti}, 2) >= cfg.adapt_R_min_samples
    buf0 = pred.innov{ti};
    C_hat = (buf0 * buf0') / size(buf0, 2);
    R_new = zeros(z_dim);
    for di = 1:z_dim
        R_new(di, di) = max(C_hat(di, di) - Pzz_pred(di, di), R_min_diag(di));
    end
    if ~isempty(pred.R{ti})
        R_old = pred.R{ti};
    else
        R_old = R_meas;
    end
    R_use = (1 - cfg.adapt_R_alpha) * R_old + cfg.adapt_R_alpha * R_new;
else
    R_use = R_meas;
end
R_use = 0.5 * (R_use + R_use');
for di = 1:z_dim
    R_use(di, di) = max(R_use(di, di), R_min_diag(di));
end
R_use = make_spd(R_use);
end

function g = active_group_stats(pred, ti, J, z_gated, gated_idx, R_k_meas, ...
        R_default, cfg, pos_idx, R_min_diag, R_scale, weight_mode)
g = struct('valid', false, 'nJ', 0, 'Z', zeros(3, 0), 'orig', zeros(1, 0), ...
    'z_pred', zeros(3, 1), 'Pzz', zeros(3), 'R_use', zeros(3), ...
    'R_bar', zeros(3), 'innovation', zeros(3, 1), 'S_innov', zeros(3), ...
    'nis', inf, 'wv', zeros(1, 0), 'z_bar', zeros(3, 1));
if isempty(J) || ti < 1 || ti > pred.N
    return;
end
J = J(:).';
nJ = numel(J);
Z = z_gated(:, J);
orig = gated_idx(J);
[z_pred, Pzz] = ckf_cartesian_stats(pred.m(:, ti), pred.P(:, :, ti), pos_idx);
R_acc = zeros(3);
for jj = 1:nJ
    R_acc = R_acc + get_meas_R(R_k_meas, orig(jj), R_default);
end
R_meas = R_acc / nJ;
R_use = active_track_R(pred, ti, R_meas, Pzz, cfg, R_min_diag);
S_w = make_spd(Pzz + R_use);
dnis = zeros(1, nJ);
for jj = 1:nJ
    nu_j = Z(:, jj) - z_pred;
    dnis(jj) = nu_j' * (S_w \ nu_j);
end
if strcmp(weight_mode, 'equal')
    wv = ones(1, nJ) / nJ;
else
    wv = exp(-0.5 * (dnis - min(dnis)));
    sw = sum(wv);
    if sw <= 0 || ~isfinite(sw)
        wv = ones(1, nJ) / nJ;
    else
        wv = wv / sw;
    end
end
z_bar = Z * wv(:);
R_bar = (sum(wv.^2) * R_scale) * R_use;
for di = 1:3
    R_bar(di, di) = max(R_bar(di, di), R_min_diag(di));
end
R_bar = make_spd(R_bar);
innovation = z_bar - z_pred;
S_innov = make_spd(Pzz + R_bar);
nis = innovation' * (S_innov \ innovation);
g = struct('valid', all(isfinite([z_bar; innovation])) && isfinite(nis), ...
    'nJ', nJ, 'Z', Z, 'orig', orig, 'z_pred', z_pred, 'Pzz', Pzz, ...
    'R_use', R_use, 'R_bar', R_bar, 'innovation', innovation, ...
    'S_innov', S_innov, 'nis', nis, 'wv', wv, 'z_bar', z_bar);
end

function [track_meas, primary_stage, available, added_tracks] = assign_primary_tier( ...
        track_meas, primary_stage, available, tier_tracks, cost_mat, unmatched_cost, stage_code)
added_tracks = zeros(1, 0);
if isempty(tier_tracks) || ~any(available)
    return;
end
tier_tracks = tier_tracks(:).';
empty_track = cellfun(@isempty, track_meas(tier_tracks));
tier_tracks = tier_tracks(empty_track);
meas_idx = find(available);
if isempty(tier_tracks) || isempty(meas_idx)
    return;
end
pairs_local = solve_assignment(cost_mat(tier_tracks, meas_idx), unmatched_cost);
for ai = 1:size(pairs_local, 1)
    ti = tier_tracks(pairs_local(ai, 1));
    mi = meas_idx(pairs_local(ai, 2));
    if ~available(mi) || ~isempty(track_meas{ti})
        continue;
    end
    track_meas{ti} = mi;
    primary_stage(ti) = stage_code;
    available(mi) = false;
    added_tracks(end+1) = ti; %#ok<AGROW>
end
end

function [track_meas, available, failed_mi, failed_parent, pair_nis_used, pair_dist_used] = attach_secondary_tier( ...
        track_meas, available, tier_tracks, nis_mat, z_gated, ...
        gated_idx, R_k_meas, R_default, accept_nis, pair_gate, dist_gate)
failed_mi = zeros(1, 0);
failed_parent = zeros(1, 0);
pair_nis_used = zeros(1, 0);
pair_dist_used = zeros(1, 0);
if isempty(tier_tracks) || ~any(available)
    return;
end
tier_tracks = tier_tracks(:).';
secondary_idx = find(available);
for mi = secondary_idx
    best_ti = 0;
    best_pair_nis = inf;
    best_pair_dist = inf;
    nearest_parent = 0;
    nearest_pair = inf;
    orig_mi = gated_idx(mi);
    had_individual_candidate = false;
    for ti = tier_tracks
        if isempty(track_meas{ti})
            continue;
        end
        pmi = track_meas{ti}(1);
        orig_pmi = gated_idx(pmi);
        if nis_mat(ti, pmi) > accept_nis || nis_mat(ti, mi) > accept_nis
            continue;
        end
        had_individual_candidate = true;
        dz_pair = z_gated(:, mi) - z_gated(:, pmi);
        pair_dist = norm(dz_pair);
        R_pair = make_spd(get_meas_R(R_k_meas, orig_mi, R_default) + ...
            get_meas_R(R_k_meas, orig_pmi, R_default));
        pair_nis = dz_pair' * (R_pair \ dz_pair);
        if isfinite(pair_nis) && pair_nis < nearest_pair
            nearest_pair = pair_nis;
            nearest_parent = ti;
        end
        if pair_dist > dist_gate || ~isfinite(pair_nis) || pair_nis > pair_gate
            continue;
        end
        if pair_nis < best_pair_nis
            best_pair_nis = pair_nis;
            best_pair_dist = pair_dist;
            best_ti = ti;
        end
    end
    if best_ti > 0
        track_meas{best_ti}(end+1) = mi;
        available(mi) = false;
        pair_nis_used(end+1) = best_pair_nis; %#ok<AGROW>
        pair_dist_used(end+1) = best_pair_dist; %#ok<AGROW>
    elseif had_individual_candidate
        failed_mi(end+1) = mi; %#ok<AGROW>
        failed_parent(end+1) = nearest_parent; %#ok<AGROW>
    end
end
end

function [track_meas, primary_stage, available, banned_edges, rejected_pairs, released_pairs] = validate_active_groups( ...
        track_meas, primary_stage, available, banned_edges, tracks_to_check, ...
        pred, z_gated, gated_idx, R_k_meas, R_default, cfg, pos_idx, R_min_diag, ...
        R_scale, weight_mode, accept_nis)
rejected_pairs = zeros(0, 2);
released_pairs = zeros(0, 2);
if isempty(tracks_to_check)
    return;
end
for ti = reshape(unique(tracks_to_check), 1, [])
    J = track_meas{ti};
    if isempty(J)
        continue;
    end
    g = active_group_stats(pred, ti, J, z_gated, gated_idx, R_k_meas, ...
        R_default, cfg, pos_idx, R_min_diag, R_scale, weight_mode);
    if g.valid && g.nis <= accept_nis
        continue;
    end
    if numel(J) > 1
        for mi = J(2:end)
            released_pairs(end+1, :) = [ti, mi]; %#ok<AGROW>
            available(mi) = true;
        end
        J = J(1);
        track_meas{ti} = J;
        g = active_group_stats(pred, ti, J, z_gated, gated_idx, R_k_meas, ...
            R_default, cfg, pos_idx, R_min_diag, R_scale, weight_mode);
    end
    if ~g.valid || g.nis > accept_nis
        mi = J(1);
        rejected_pairs(end+1, :) = [ti, mi]; %#ok<AGROW>
        banned_edges(ti, mi) = true;
        available(mi) = true;
        track_meas{ti} = zeros(1, 0);
        primary_stage(ti) = 0;
    end
end
end


function pairs = solve_assignment(cost_mat, unmatched_cost)
if isempty(cost_mat) || ~any(isfinite(cost_mat(:)))
    pairs = zeros(0, 2);
    return;
end
if exist('matchpairs', 'file') == 2
    C = cost_mat;
    bad_mask = ~isfinite(C);
    large_cost = unmatched_cost + max(1, abs(unmatched_cost)) * 1e6;
    C(bad_mask) = large_cost;
    pairs = matchpairs(C, unmatched_cost);
    if ~isempty(pairs)
        keep = false(size(pairs, 1), 1);
        for i = 1:size(pairs, 1)
            keep(i) = isfinite(cost_mat(pairs(i, 1), pairs(i, 2))) && ...
                cost_mat(pairs(i, 1), pairs(i, 2)) <= unmatched_cost;
        end
        pairs = pairs(keep, :);
    end
    return;
end
C = cost_mat;
pairs = zeros(0, 2);
while ~isempty(C) && any(isfinite(C(:)))
    [best_val, lin_idx] = min(C(:));
    if ~isfinite(best_val) || best_val > unmatched_cost
        break;
    end
    [r, c] = ind2sub(size(C), lin_idx);
    pairs(end+1, :) = [r, c]; %#ok<AGROW>
    C(r, :) = inf;
    C(:, c) = inf;
end
end

function pb = get_passive_bearing(passive_bearing, k)
pb = [];
if isempty(passive_bearing) || k > numel(passive_bearing)
    return;
end
pb = passive_bearing{k};
if isempty(pb) || ~isstruct(pb) || ~isfield(pb, 'ang_deg')
    pb = [];
    return;
end
pb.n_meas = size(pb.ang_deg, 2);
end

function n = get_passive_count(passive_bearing, k)
n = 0;
pb = get_passive_bearing(passive_bearing, k);
if ~isempty(pb), n = pb.n_meas; end
end

function d = empty_passive_assoc_detail(n)
if nargin < 1, n = 0; end
d = struct('used_mask', false(1, n), 'explained_mask', false(1, n), ...
    'ambiguous_mask', false(1, n), 'track_id', nan(1, n), ...
    'innovation', nan(2, n), 'nis', nan(1, n), ...
    'group_size', zeros(1, n), 'innovation_kind', ones(1, n), ...
    'updated_track_ids', zeros(1, 0), ...
    'updated_passive_track_ids', zeros(1, 0), ...
    'updated_active_angle_track_ids', zeros(1, 0));
end

function s = empty_passive_confirm_state()
s = struct('id', zeros(1, 0), 'streak', zeros(1, 0), ...
    'last_t', zeros(1, 0));
end

function [trk, state, n_equiv] = apply_passive_equivalent_hits( ...
        trk, state, updated_ids, t, active_confirm_cycle, hits_per_equiv, max_gap_s)
n_equiv = 0;
updated_ids = unique(updated_ids(isfinite(updated_ids)));
for id = reshape(updated_ids, 1, [])
    si = find(state.id == id, 1);
    if isempty(si)
        state.id(end + 1) = id;
        state.streak(end + 1) = 0;
        state.last_t(end + 1) = -inf;
        si = numel(state.id);
    end
    if isfinite(state.last_t(si)) && t - state.last_t(si) <= max_gap_s
        state.streak(si) = state.streak(si) + 1;
    else
        state.streak(si) = 1;
    end
    state.last_t(si) = t;
    if state.streak(si) < hits_per_equiv
        continue;
    end
    state.streak(si) = 0;
    ti = find(trk.L(2, :) == id, 1);
    if isempty(ti)
        continue;
    end
    active_hit = isfinite(trk.last_active_hit_t(ti)) && ...
        abs(trk.last_active_hit_t(ti) - t) <= 1e-9;
    if active_hit
        continue;
    end
    if ~active_confirm_cycle
        trk.S(:, ti) = [trk.S(2:end, ti); 2];
    end
    trk.S(end, ti) = 4; % passive-equivalent logical hit; not active range evidence
    n_equiv = n_equiv + 1;
end
end

function [trk, n_used, stats, detail] = update_passive_bearing(trk, pb, t_k, platform, cfg, ...
    pos_idx, gate, nis_gate, weight_gain, confirm_hit, fast_gate_deg, track_mask)
n_used = 0;
stats = empty_passive_bearing_stats();
z_all = pb.ang_deg;
M = size(z_all, 2);
kind_all = ones(1, M);
if isfield(pb, 'kind') && ~isempty(pb.kind)
    nk = min(M, numel(pb.kind));
    kind_all(1:nk) = reshape(pb.kind(1:nk), 1, []);
end
detail = empty_passive_assoc_detail(M);
stats.n_input = M;
stats.n_events_attempted = double(M > 0);
if trk.N == 0 || M == 0
    return;
end
if nargin < 11 || isempty(fast_gate_deg)
    fast_gate_deg = inf;
end
if nargin < 12 || isempty(track_mask)
    track_mask = true(1, trk.N);
else
    track_mask = logical(track_mask(:).');
    if numel(track_mask) < trk.N
        track_mask = [track_mask, false(1, trk.N - numel(track_mask))];
    elseif numel(track_mask) > trk.N
        track_mask = track_mask(1:trk.N);
    end
end
if ~any(track_mask)
    stats.n_masked = M;
    return;
end
geom = bearing_geometry(t_k, platform, cfg);
if isfield(pb, 'src') && numel(pb.src) == M
    src = pb.src(:).';
else
    src = zeros(1, M);
end
src(~isfinite(src)) = 0;

% Physical sensor types are processed sequentially. File shard numbers were
% normalized upstream and never create independent sensor identities. A
% bounded event batch may contain several raw bearing samples; their original
% timestamps and measurement identities remain available in pb.
sources = unique(src);
for si = 1:numel(sources)
    meas_idx = find(src == sources(si));
    cost_mat = inf(trk.N, numel(meas_idx));
    for ti = 1:trk.N
        if ~track_mask(ti)
            continue;
        end
        z_fast = bearing_model_geom(trk.m(pos_idx, ti), geom);
        if any(~isfinite(z_fast(:)))
            continue;
        end
        possible = false(1, numel(meas_idx));
        for qi = 1:numel(meas_idx)
            mi = meas_idx(qi);
            nu_fast = bearing_innovation(z_all(:, mi), z_fast);
            possible(qi) = all(abs(nu_fast) <= fast_gate_deg);
        end
        if ~any(possible)
            continue;
        end
        [z_pred, Pzz, ~, ok] = ckf_bearing_stats(trk.m(:, ti), trk.P(:, :, ti), ...
            t_k, platform, cfg, pos_idx, geom);
        if ~ok || any(~isfinite(z_pred(:)))
            continue;
        end
        for qi = 1:numel(meas_idx)
            if ~possible(qi)
                continue;
            end
            mi = meas_idx(qi);
            Rb = get_bearing_R(pb, mi, cfg);
            S = make_spd(Pzz + Rb);
            nu = bearing_innovation(z_all(:, mi), z_pred);
            nis = nu' * (S \ nu);
            if isfinite(nis) && nis <= gate
                cost_mat(ti, qi) = nis;
            end
        end
    end

    groups = cell(1, trk.N);
    pairs = solve_assignment(cost_mat, gate);
    for a = 1:size(pairs, 1)
        groups{pairs(a, 1)} = pairs(a, 2);
    end
    stats.n_gate_rejected = stats.n_gate_rejected + ...
        nnz(~any(isfinite(cost_mat), 1));

    for ti = 1:trk.N
        local_idx = groups{ti};
        if isempty(local_idx)
            continue;
        end
        global_idx = meas_idx(local_idx);
        z_group = z_all(:, global_idx);
        Rb = get_bearing_R(pb, global_idx, cfg);
        [~, x_upd, P_upd, S, ~, z_pred] = ckf_update_bearing( ...
            z_group, Rb, trk.m(:, ti), trk.P(:, :, ti), t_k, platform, cfg, pos_idx, geom);
        if any(~isfinite(x_upd(:))) || any(~isfinite(P_upd(:)))
            stats.n_update_rejected = stats.n_update_rejected + numel(global_idx);
            continue;
        end
        nu = bearing_innovation(z_group, z_pred);
        nis = nu' * (S \ nu);
        if ~isfinite(nis) || nis > gate
            stats.n_update_rejected = stats.n_update_rejected + numel(global_idx);
            continue;
        end

        trk.m(:, ti) = x_upd;
        trk.P(:, :, ti) = P_upd;
        trk.w(ti) = min(1, trk.w(ti) + weight_gain);
        % miss is the consecutive ACTIVE-detection miss count.  A passive
        % bearing may extend the valid-update timeout, but it must not erase
        % the fact that the active sensor missed this track.
        trk.last_update_t(ti) = t_k;
        trk.last_passive_update_t(ti) = t_k;
        detail.used_mask(global_idx) = true;
        detail.explained_mask(global_idx) = true;
        detail.track_id(global_idx) = trk.L(2, ti);
        detail.innovation(:, global_idx) = repmat(nu, 1, numel(global_idx));
        detail.nis(global_idx) = nis;
        detail.group_size(global_idx) = numel(global_idx);
        detail.updated_track_ids(end + 1) = trk.L(2, ti); %#ok<AGROW>
        if any(kind_all(global_idx) == 2)
            detail.updated_active_angle_track_ids(end + 1) = trk.L(2, ti); %#ok<AGROW>
        else
            detail.updated_passive_track_ids(end + 1) = trk.L(2, ti); %#ok<AGROW>
        end
        if confirm_hit
            trk.S(end, ti) = 1;
        elseif trk.S(end, ti) ~= 1
            trk.S(end, ti) = 3; % bearing-only hit; does not count toward confirmation by default
        end
        if nis > nis_gate
            trk.nisbad(ti) = trk.nisbad(ti) + 1;
        else
            trk.nisbad(ti) = max(0, trk.nisbad(ti) - 1);
        end
        n_used = n_used + 1;
        stats.n_updates = stats.n_updates + 1;
        stats.n_measurements_used = stats.n_measurements_used + numel(global_idx);
        stats.n_single_updates = stats.n_single_updates + 1;
    end
end
end

function geom = bearing_geometry(t_sec, platform, cfg)
plat_lat = platform.interp_lat(t_sec);
plat_lon = platform.interp_lon(t_sec);
plat_alt = platform.interp_alt(t_sec);
[anchor_lat, anchor_lon, anchor_alt] = get_anchor_llh(platform, cfg);

geom.plat_ecef = llh_to_ecef(plat_lat, plat_lon, plat_alt);
geom.anchor_ecef = llh_to_ecef(anchor_lat, anchor_lon, anchor_alt);
geom.R_plat = ecef_to_enu_rot(plat_lat, plat_lon);
geom.R_anchor = ecef_to_enu_rot(anchor_lat, anchor_lon);
end

function z = bearing_model_geom(xyz_anchor, geom)
target_ecef = geom.anchor_ecef + geom.R_anchor' * xyz_anchor(:);
rel_plat_enu = geom.R_plat * (target_ecef - geom.plat_ecef);
east = rel_plat_enu(1);
north = rel_plat_enu(2);
up = rel_plat_enu(3);
az = atan2d(east, north);
el = atan2d(up, hypot(east, north));
z = [az; el];
end

function nu = bearing_innovation(z_obs, z_pred)
nu = [angle_signed_diff_deg(z_obs(1), z_pred(1)); z_obs(2) - z_pred(2)];
end

function Rb = get_bearing_R(pb, idx, cfg)
if isfield(pb, 'R_deg2') && ndims(pb.R_deg2) == 3 && size(pb.R_deg2, 3) >= idx
    Rb = pb.R_deg2(:, :, idx);
elseif isfield(pb, 'R_deg2') && isequal(size(pb.R_deg2), [2, 2]) && idx == 1
    Rb = pb.R_deg2;
else
    sig_az = get_cfg_field(cfg, 'sigma_passive_az_deg', 0.1);
    sig_el = get_cfg_field(cfg, 'sigma_passive_el_deg', 0.1);
    Rb = diag([sig_az^2, sig_el^2]);
end
Rb = make_spd(Rb);
end

function [z_gate, gated_idx] = gate_meas_ckf(z, gamma, pos_idx, m, P, R_meas, R_default, pos_gate_m)
if nargin < 8 || isempty(pos_gate_m)
    pos_gate_m = inf;
end
z_dim = size(z, 1);
M = size(z, 2);
N_tracks = size(m, 2);
if M == 0 || N_tracks == 0
    z_gate = zeros(z_dim, 0);
    gated_idx = [];
    return;
end
valid_idx = false(1, M);
for j = 1:N_tracks
    [z_pred, Pzz] = ckf_cartesian_stats(m(:, j), P(:, :, j), pos_idx);
    for mi = 1:M
        R_j = get_meas_R(R_meas, mi, R_default);
        S_j = make_spd(Pzz + R_j);
        [Lc, flag] = chol(S_j, 'lower');
        if flag ~= 0, continue; end
        nu = z(:, mi) - z_pred;
        if isfinite(pos_gate_m) && norm(nu) > pos_gate_m
            continue;
        end
        dist = sum((Lc \ nu).^2, 1);
        valid_idx(mi) = valid_idx(mi) || (dist < gamma);
    end
end
gated_idx = find(valid_idx);
z_gate = z(:, valid_idx);
end

function R = get_meas_R(R_meas, idx, R_default)
if ~isempty(R_meas) && ndims(R_meas) == 3 && size(R_meas, 3) >= idx
    R = R_meas(:, :, idx);
elseif ~isempty(R_meas) && isequal(size(R_meas), [3, 3]) && idx == 1
    R = R_meas;
else
    R = R_default;
end
R = 0.5 * (R + R');
end

function A = make_spd(A)
A = 0.5 * (A + A');
if all(isfinite(A(:)))
    [~, flag] = chol(A, 'lower');
    if flag == 0
        return;
    end

    jitter = max(1e-9, 1e-12 * max(1, trace(abs(A))));
    I = eye(size(A));
    for attempt = 1:4
        A_try = A + jitter * I;
        [~, flag] = chol(A_try, 'lower');
        if flag == 0
            A = A_try;
            return;
        end
        jitter = jitter * 10;
    end
end

[V, D] = eig(A);
d = max(real(diag(D)), 1e-9);
A = V * diag(d) * V';
A = 0.5 * (A + A');
end

function [anchor_lat, anchor_lon, anchor_alt] = get_anchor_llh(platform, cfg)
if ischar(cfg.local_origin) && strcmp(cfg.local_origin, 'first_platform')
    anchor_lat = platform.lat_deg(1);
    anchor_lon = platform.lon_deg(1);
    anchor_alt = platform.alt_m(1);
elseif isnumeric(cfg.local_origin) && numel(cfg.local_origin) == 3
    anchor_lat = cfg.local_origin(1);
    anchor_lon = cfg.local_origin(2);
    anchor_alt = cfg.local_origin(3);
else
    anchor_lat = platform.lat_deg(1);
    anchor_lon = platform.lon_deg(1);
    anchor_alt = platform.alt_m(1);
end
end

function ecef = llh_to_ecef(lat_deg, lon_deg, alt_m)
a = 6378137.0; f = 1/298.257223563; e2 = 2*f - f^2;
lat = deg2rad(lat_deg); lon = deg2rad(lon_deg);
sin_lat = sin(lat); cos_lat = cos(lat);
N = a / sqrt(1 - e2 * sin_lat^2);
x = (N + alt_m) * cos_lat * cos(lon);
y = (N + alt_m) * cos_lat * sin(lon);
z = (N * (1 - e2) + alt_m) * sin_lat;
ecef = [x; y; z];
end

function R = ecef_to_enu_rot(lat_deg, lon_deg)
lat = deg2rad(lat_deg); lon = deg2rad(lon_deg);
sin_lat = sin(lat); cos_lat = cos(lat);
sin_lon = sin(lon); cos_lon = cos(lon);
R = [-sin_lon,            cos_lon,            0;
     -sin_lat*cos_lon,   -sin_lat*sin_lon,    cos_lat;
      cos_lat*cos_lon,    cos_lat*sin_lon,    sin_lat];
end

function d = angle_signed_diff_deg(a, b)
d = mod(a - b + 180, 360) - 180;
end

function [x_pred, P_pred] = ckf_predict_linear(x, P, F, Q)
% Current CV/CA motion models are linear in the 9-D state. Closed-form KF
% prediction is identical to the CKF sigma-point result and avoids repeated
% point generation in every IMM prediction.
x_pred = F * x(:);
P_pred = make_spd(F * P * F' + Q);
end

function [z_pred, Pzz, Pxz] = ckf_cartesian_stats(x, P, pos_idx)
% Active Cartesian measurements observe position directly: z = x(pos_idx).
% This is exact for the current measurement model and is much cheaper than
% generating CKF points for every gate and association cost.
z_pred = x(pos_idx);
Pzz = make_spd(P(pos_idx, pos_idx));
Pxz = P(:, pos_idx);
end

function [llh, x_upd, P_upd, S, Kg, z_pred] = ckf_update_cartesian(z, R, x_pred, P_pred, pos_idx)
[z_pred, Pzz, Pxz] = ckf_cartesian_stats(x_pred, P_pred, pos_idx);
S = make_spd(Pzz + R);
nu = z(:) - z_pred;
Kg = Pxz / S;
x_upd = x_pred + Kg * nu;
P_upd = make_spd(P_pred - Kg * S * Kg');
llh = gaussian_loglik(nu, S);
end

function [z_pred, Pzz, Pxz, ok] = ckf_bearing_stats(x, P, t_sec, platform, cfg, pos_idx, geom)
if nargin < 7 || isempty(geom)
    geom = bearing_geometry(t_sec, platform, cfg);
end
[Xi, w] = ckf_points(x, P);
n_pts = size(Xi, 2);
Zi = nan(2, n_pts);
ok = true;
for ii = 1:n_pts
    Zi(:, ii) = bearing_model_geom(Xi(pos_idx, ii), geom);
    if any(~isfinite(Zi(:, ii)))
        ok = false;
        z_pred = [NaN; NaN];
        Pzz = nan(2);
        Pxz = nan(numel(x), 2);
        return;
    end
end

sin_az = sum(sind(Zi(1, :))) * w;
cos_az = sum(cosd(Zi(1, :))) * w;
z_pred = [atan2d(sin_az, cos_az); sum(Zi(2, :)) * w];

dz = zeros(2, n_pts);
for ii = 1:n_pts
    dz(:, ii) = bearing_innovation(Zi(:, ii), z_pred);
end
dx = bsxfun(@minus, Xi, x(:));
Pzz = make_spd(dz * dz' * w);
Pxz = dx * dz' * w;
end

function [llh, x_upd, P_upd, S, Kg, z_pred] = ckf_update_bearing( ...
    z, R, x_pred, P_pred, t_sec, platform, cfg, pos_idx, geom)
if nargin < 9 || isempty(geom)
    geom = bearing_geometry(t_sec, platform, cfg);
end
[z_pred, Pzz, Pxz, ok] = ckf_bearing_stats(x_pred, P_pred, t_sec, platform, cfg, pos_idx, geom);
if ~ok
    llh = -inf;
    x_upd = nan(size(x_pred));
    P_upd = nan(size(P_pred));
    S = nan(size(R));
    Kg = nan(numel(x_pred), size(R, 1));
    return;
end
S = make_spd(Pzz + R);
nu = bearing_innovation(z(:), z_pred);
Kg = Pxz / S;
x_upd = x_pred + Kg * nu;
P_upd = make_spd(P_pred - Kg * S * Kg');
llh = gaussian_loglik(nu, S);
end

function [Xi, w] = ckf_points(x, P)
x = x(:);
n = numel(x);
P = make_spd(P);
jitter = 0;
S = [];
flag = 1;
for attempt = 1:7
    [S, flag] = chol(P + jitter * eye(n), 'lower');
    if flag == 0
        break;
    end
    jitter = max(1e-9, 10 * max(jitter, 1e-9));
end
if flag ~= 0
    [V, D] = eig(P);
    S = V * diag(sqrt(max(diag(D), 1e-9)));
end
scale = sqrt(n);
Xi = [bsxfun(@plus, x, scale * S), bsxfun(@minus, x, scale * S)];
w = 1 / (2 * n);
end

function llh = gaussian_loglik(nu, S)
z_dim = numel(nu);
S = make_spd(S);
[L, flag] = chol(S, 'lower');
if flag == 0
    y = L \ nu;
    maha = y' * y;
    logdetS = 2 * sum(log(max(diag(L), realmin)));
else
    maha = nu' / S * nu;
    logdetS = log(max(det(S), realmin));
end
llh = -0.5 * (z_dim * log(2*pi) + logdetS + maha);
end

function cfg2 = make_online_joint2d_cfg(cfg, frame_times, passive_bearing)
equiv_hits = max(1, round(get_cfg_field(cfg, ...
    'joint_passive_confirm_consecutive_hits', 3)));
cfg2 = cfg;
cfg2.joint_confirm_M = max(1, round(get_cfg_field(cfg, ...
    'joint_confirm_M', get_cfg_field(cfg, 'M_confirm', 3))));
base_N = max(1, round(get_cfg_field(cfg, ...
    'joint_confirm_N', get_cfg_field(cfg, 'N_confirm', 5))));
has_angle = false(numel(frame_times), 1);
for k = 1:min(numel(frame_times), numel(passive_bearing))
    has_angle(k) = get_passive_count(passive_bearing, k) > 0;
end
angle_times = frame_times(has_angle);
dt = diff(angle_times(:)); dt = dt(isfinite(dt) & dt > 1e-9);
if isempty(dt)
    window_events = base_N * equiv_hits;
else
    window_s = get_cfg_field(cfg, 'joint_passive_confirm_window_s', 1.50);
    ratio = window_s / median(dt);
    ratio_tol = 1e-9 * max(ratio, 1);
    window_events = ceil(ratio - ratio_tol) + 1;
end
cfg2.joint_confirm_N = max([cfg2.joint_confirm_M, ...
    base_N * equiv_hits, window_events]);
cfg2.joint_passive_confirm_group_size = equiv_hits;
cfg2.joint_passive_confirmation_cycles_only = true;
cfg2.joint_birth_explain_nis = get_cfg_field(cfg, ...
    'joint_birth_explain_nis', 1.0);
cfg2.joint_streaming_quiet = true;
end

function est = init_online_joint2d_estimate(K, times)
est = struct();
est.X = cell(K, 1); est.P = cell(K, 1); est.L = cell(K, 1); est.N = zeros(K, 1);
est.X2 = cell(K, 1); est.P2 = cell(K, 1); est.L2 = cell(K, 1); est.N2 = zeros(K, 1);
est.N_total = zeros(K, 1); est.logical_tracks = cell(K, 1); est.output = cell(K, 1);
est.tracks = cell(K, 1); est.assoc = cell(K, 1); est.companions = cell(K, 1);
est.measurement_disposition = cell(K, 1);
est.filter_times = times(:); est.event_meta = repmat(online_event_template(), K, 1);
est.mode_counts = struct('n2d', zeros(K, 1), 'n3d', zeros(K, 1), ...
    'nhold', zeros(K, 1));
est.timing = struct('total', 0);
est.transition_log = struct('id', {}, 't_sec', {}, 'from', {}, 'to', {}, 'reason', {});
est.stats = struct();
est.confirmation = struct();
est.framework = 'joint_2d3d';
est.output_contract = 'logical_track_v1';
end

function event = make_online_residual_event(k, t, passive_bearing, detail, meta, cfg, ...
        trk, idx_confirmed, platform)
event = online_event_template();
event.cycle_id = k; event.t_sec = t;
event.t_start = meta.t_start; event.t_end = meta.t_end;
event.confirm_cycle = logical(meta.has_passive);
geom_now = bearing_geometry(t, platform, cfg);
if trk.N > 0
    idx_external = 1:trk.N;
    event.external_3d.id = trk.L(2, idx_external);
    event.external_3d.confirmed = ismember(idx_external, idx_confirmed);
    event.external_3d.state = trk.m(:, idx_external);
    event.external_3d.cov = trk.P(:, :, idx_external);
    event.external_3d.last_active_t = trk.last_active_hit_t(idx_external);
    event.external_3d.last_update_t = trk.last_update_t(idx_external);
    event.external_3d.nis_norm = trk.last_active_nis_norm(idx_external);
    event.external_3d.active_hit = abs(trk.last_active_hit_t(idx_external) - t) <= 1e-9;
    event.external_3d.active_opportunity = logical(meta.has_active);
    event.external_3d.dimension_ready = false(1, numel(idx_external));
    event.external_3d.fresh = false(1, numel(idx_external));
    event.external_3d.ang = zeros(2, numel(idx_external));
    event.external_3d.rate = nan(2, numel(idx_external));
    output_silence = get_cfg_field(cfg, 'confirmed_output_max_silence_s', 0.5);
    upgrade_M = max(1, round(get_cfg_field(cfg, 'joint_3d_upgrade_M', 2)));
    upgrade_N = max(upgrade_M, round(get_cfg_field(cfg, 'joint_3d_upgrade_N', 3)));
    rate_dt = 0.05;
    if isfield(platform, 't_sec') && ~isempty(platform.t_sec) && ...
            t + rate_dt > max(platform.t_sec)
        rate_dt = -rate_dt;
    end
    geom_rate = bearing_geometry(t + rate_dt, platform, cfg);
    for q = 1:numel(idx_external)
        ti = idx_external(q);
        event.external_3d.ang(:, q) = bearing_model_geom( ...
            trk.m([1, 4, 7], ti), geom_now);
        event.external_3d.rate(:, q) = bearing_rate_from_geometries( ...
            trk.m(:, ti), rate_dt, geom_now, geom_rate);
        w = trk.S(max(1, end - upgrade_N + 1):end, ti);
        event.external_3d.dimension_ready(q) = nnz(w == 1) >= upgrade_M;
        event.external_3d.fresh(q) = ~isfinite(output_silence) || ...
            t - trk.last_update_t(ti) <= output_silence;
    end
end

pb = get_passive_bearing(passive_bearing, k);
if isempty(pb), return; end

n = pb.n_meas;
pb_kind_all = online_sized_row(field_or_default(pb, 'kind', ones(1, n)), n, 1);
pb_R_all = online_covariance(field_or_default(pb, 'R_deg2', []), ...
    2, n, diag([get_cfg_field(cfg, 'sigma_passive_az_deg', 0.05)^2, ...
    get_cfg_field(cfg, 'sigma_passive_el_deg', 0.04)^2]));
if isstruct(detail) && isfield(detail, 'used_mask') && isfield(detail, 'track_id')
    used = find(online_sized_logical(detail.used_mask, n, false));
    for mi = reshape(used, 1, [])
        if mi <= numel(detail.track_id) && isfinite(detail.track_id(mi))
            event = append_external_angle_update(event, detail.track_id(mi), ...
                pb.ang_deg(:, mi), pb_R_all(:, :, mi), pb_kind_all(mi));
        end
    end
end
explained = false(1, n);
if isstruct(detail) && isfield(detail, 'explained_mask')
    explained = online_sized_logical(detail.explained_mask, n, false);
end
idx = find(~explained);
event.has_passive = ~isempty(idx);
event.passive.n_meas = numel(idx);
if isempty(idx), return; end

event.passive.ang = pb.ang_deg(:, idx);
event.passive.R_ae = pb_R_all(:, :, idx);
% The mature 3-D loop processes every packet at the current event time.
% Use the same timestamp in the embedded 2-D branch so shard jitter cannot
% make persistent streaming state move backwards in time.
event.passive.t_sec = repmat(t, 1, numel(idx));
event.passive.ids = online_sized_row(field_or_default(pb, 'tracklet_id', []), n, NaN);
event.passive.ids = event.passive.ids(idx);
event.passive.src = online_sized_row(field_or_default(pb, 'src', ones(1, n)), n, 1);
event.passive.src = event.passive.src(idx);
event.passive.kind = pb_kind_all(idx);
event.passive.original_index = idx;
end

function event = append_external_angle_update(event, track_id, ang, R, kind)
if ~isfinite(track_id) || numel(ang) < 2 || any(~isfinite(ang(1:2)))
    return;
end
q = numel(event.external_3d.update_track_id) + 1;
event.external_3d.update_track_id(q) = track_id;
event.external_3d.update_ang(:, q) = reshape(ang(1:2), 2, 1);
event.external_3d.update_R(:, :, q) = R;
event.external_3d.update_kind(q) = kind;
end

function est = store_online_joint2d_chunk(est, chunk, state, event, k)
cell_fields = {'X', 'P', 'L', 'X2', 'P2', 'L2', 'logical_tracks', ...
    'output', 'tracks', 'assoc', 'companions', 'measurement_disposition'};
for i = 1:numel(cell_fields)
    name = cell_fields{i};
    est.(name){k} = chunk.(name){1};
end
est.N(k) = chunk.N(1); est.N2(k) = chunk.N2(1);
est.N_total(k) = chunk.N_total(1);
est.mode_counts.n2d(k) = chunk.mode_counts.n2d(1);
est.mode_counts.n3d(k) = chunk.mode_counts.n3d(1);
est.mode_counts.nhold(k) = chunk.mode_counts.nhold(1);
est.event_meta(k) = event;
est.transition_log = [est.transition_log, chunk.transition_log];
est.stats = state.stats;
est.timing.total = state.timing_total;
est.confirmation = struct('M', state.params.confirm_M, ...
    'N_events', state.params.confirm_N, ...
    'passive_group_size', state.params.passive_confirm_group, ...
    'passive_max_gap_s', state.params.passive_confirm_max_gap_s);
end

function e = online_event_template()
active = struct('t_sec', zeros(1, 0), 'xyz', zeros(3, 0), ...
    'rae', zeros(3, 0), 'R_xyz', zeros(3, 3, 0), ...
    'R_ae', zeros(2, 2, 0), 'has_range', false(1, 0), ...
    'ids', zeros(1, 0), 'src', zeros(1, 0), 'n_meas', 0);
passive = struct('t_sec', zeros(1, 0), 'ang', zeros(2, 0), ...
    'R_ae', zeros(2, 2, 0), 'ids', zeros(1, 0), ...
    'src', zeros(1, 0), 'kind', zeros(1, 0), ...
    'original_index', zeros(1, 0), 'n_meas', 0);
e = struct('cycle_id', 0, 't_sec', NaN, 't_start', NaN, 't_end', NaN, ...
    'confirm_cycle', false, 'has_active', false, 'has_passive', false, ...
    'active', active, 'passive', passive, ...
    'external_3d', struct('id', zeros(1, 0), 'confirmed', false(1, 0), ...
    'state', zeros(9, 0), 'cov', zeros(9, 9, 0), ...
    'last_active_t', zeros(1, 0), 'last_update_t', zeros(1, 0), ...
    'nis_norm', zeros(1, 0), ...
    'active_hit', false(1, 0), 'active_opportunity', false, ...
    'dimension_ready', false(1, 0), 'fresh', false(1, 0), ...
    'ang', zeros(2, 0), 'rate', zeros(2, 0), ...
    'update_track_id', zeros(1, 0), 'update_ang', zeros(2, 0), ...
    'update_R', zeros(2, 2, 0), 'update_kind', zeros(1, 0)));
end

function rate = bearing_rate_from_geometries(x, dt, geom_now, geom_next)
xyz = x([1, 4, 7]); vel = x([2, 5, 8]);
z0 = bearing_model_geom(xyz, geom_now);
z1 = bearing_model_geom(xyz + dt * vel, geom_next);
rate = [angle_signed_diff_deg(z1(1), z0(1)); z1(2) - z0(2)] / dt;
if any(~isfinite(rate)), rate(:) = NaN; end
end

function value = field_or_default(s, name, fallback)
if isstruct(s) && isfield(s, name) && ~isempty(s.(name))
    value = s.(name);
else
    value = fallback;
end
end

function value = online_sized_row(value, n, fallback)
if isempty(value), value = fallback * ones(1, n); else, value = value(:).'; end
if isscalar(value) && n > 1, value = repmat(value, 1, n); end
if numel(value) < n
    value = [value, fallback * ones(1, n - numel(value))];
end
value = value(1:n);
end

function value = online_sized_logical(value, n, fallback)
value = logical(online_sized_row(value, n, fallback));
end

function C = online_covariance(C, dim, n, fallback)
if isempty(C), C = repmat(fallback, 1, 1, n); return; end
if ismatrix(C), C = repmat(C, 1, 1, n); end
if size(C, 3) < n
    C(:, :, end + 1:n) = repmat(fallback, 1, 1, n - size(C, 3));
end
C = C(1:dim, 1:dim, 1:n);
end
