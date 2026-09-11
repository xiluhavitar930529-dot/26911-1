function [frames, info] = cohere_measurements(active_list, passive_list, platform, cfg)
%COHERE_MEASUREMENTS  空时凝聚：平台插值 + 地理系RAE→本地ENU + 时间分帧
%
%  Inputs:
%    active_list  : cell数组，每个元素是 parse_active_wide_txt 的输出
%    passive_list : cell数组，每个元素是 parse_passive_wide_txt 的输出（可为空）
%    platform     : load_platform_txt 的输出
%    cfg          : config_fusion() 返回的配置
%
%  Output:
%    frames : 结构体数组，每个元素是一帧
%      frames(k).t_sec         帧代表时间
%      frames(k).active_xyz    主动量测 [3×M_a] 本地ENU直角坐标(米)
%      frames(k).active_rae   主动量测 [3×M_a] 地理系 [range; az_deg; el_deg]
%      frames(k).active_R     主动量测ENU协方差 [3×3×M_a]
%      frames(k).passive_ang  被动量测 [2×M_p] [az_deg; el_deg]
%      frames(k).passive_src  被动量测文件分片索引 [1×M_p]
%      frames(k).active_src   主动物理雷达索引 [1×M_a]
%      frames(k).target_ids   主动目标编号 [1×M_a]
%      frames(k).passive_ids  被动目标编号 [1×M_p]
%      frames(k).active_t     主动量测原始时间 [1×M_a]
%      frames(k).passive_t    被动量测原始时间 [1×M_p]
%
%  坐标转换链:
%    平台(lat,lon,alt) → ECEF
%    地理系(az,el,range) → 平台ENU → ECEF → 锚点ENU(跟踪系)

fprintf('\n========== 空时凝聚 ==========\n');

%% ── 合并所有主动量测 ──────────────────────────────────────────────────
n_active_total = sum(cellfun(@(x) x.n_meas, active_list));
all_t_active = zeros(n_active_total, 1);
all_az_active = zeros(n_active_total, 1);
all_el_active = zeros(n_active_total, 1);
all_range_active = nan(n_active_total, 1);
all_range_valid_active = false(n_active_total, 1);
all_tid_active = nan(n_active_total, 1);
all_src_active = zeros(n_active_total, 1);

cursor = 0;
share_active_sensor = logical(local_get_cfg(cfg, ...
    'active_files_share_sensor', true));
for i = 1:numel(active_list)
    ad = active_list{i};
    n = ad.n_meas;
    jj = cursor + (1:n); cursor = cursor + n;
    all_t_active(jj) = ad.t_sec;
    all_az_active(jj) = ad.az_deg;
    all_el_active(jj) = ad.el_deg;
    all_range_active(jj) = ad.range_m;
    if isfield(ad, 'range_valid')
        all_range_valid_active(jj) = logical(ad.range_valid);
    else
        all_range_valid_active(jj) = isfinite(ad.range_m) & ad.range_m > 0;
    end
    all_tid_active(jj) = ad.target_id;
    if share_active_sensor
        all_src_active(jj) = 1;
    else
        all_src_active(jj) = i;
    end
end

fprintf('主动量测总计: %d 点\n', numel(all_t_active));
if numel(active_list) > 1
    if share_active_sensor
        fprintf('主动物理源: 1部雷达, %d个TXT分片\n', numel(active_list));
    else
        fprintf('主动物理源: %d部独立雷达（每个TXT对应一部）\n', numel(active_list));
    end
end

%% ── 合并所有被动量测 ──────────────────────────────────────────────────
n_passive_total = sum(cellfun(@(x) x.n_meas, passive_list));
info = struct('n_active_input', n_active_total, ...
    'n_active_rae_input', nnz(all_range_valid_active), ...
    'n_active_ae_only_input', nnz(~all_range_valid_active), ...
    'n_passive_input', n_passive_total, ...
    'n_active_after_condensation', 0, ...
    'n_passive_after_condensation', 0, ...
    'n_active_condensed', 0);
all_t_passive = zeros(n_passive_total, 1);
all_az_passive = zeros(n_passive_total, 1);
all_el_passive = zeros(n_passive_total, 1);
all_passive_src = zeros(n_passive_total, 1);  % 文件分片索引
all_tid_passive = nan(n_passive_total, 1);

cursor = 0;
for i = 1:numel(passive_list)
    pd = passive_list{i};
    n = pd.n_meas;
    jj = cursor + (1:n); cursor = cursor + n;
    all_t_passive(jj) = pd.t_sec;
    all_az_passive(jj) = pd.az_deg;
    all_el_passive(jj) = pd.el_deg;
    all_passive_src(jj) = i;
    all_tid_passive(jj) = pd.target_id;
end

fprintf('被动量测总计: %d 点\n', numel(all_t_passive));

% Platform interpolation clamps outside its time range. Make any coverage
% mismatch explicit because endpoint clamping biases moving-platform geometry.
all_meas_t = [all_t_active; all_t_passive];
if ~isempty(all_meas_t) && isfield(platform, 't_sec') && ~isempty(platform.t_sec)
    tol_t = max(local_get_cfg(cfg, 'frame_time_window_s', 0.015), 1e-6);
    outside = all_meas_t < min(platform.t_sec) - tol_t | ...
        all_meas_t > max(platform.t_sec) + tol_t;
    if any(outside)
        warning('cohere_measurements:PlatformTimeCoverage', ...
            ['%d/%d个量测时间超出平台轨迹覆盖区间；平台位置将钳位到端点。' ...
            '量测=[%.3f, %.3f]s，平台=[%.3f, %.3f]s。'], ...
            nnz(outside), numel(all_meas_t), min(all_meas_t), max(all_meas_t), ...
            min(platform.t_sec), max(platform.t_sec));
    end
end

%% ── 设定坐标锚点 ──────────────────────────────────────────────────────
if platform.n_rows < 1
    error('平台数据为空，无法进行坐标转换');
end

% 将跟踪坐标系锚定在首条平台位置对应的ENU原点
anchor_lat = platform.lat_deg(1);
anchor_lon = platform.lon_deg(1);
anchor_alt = platform.alt_m(1);

if ischar(cfg.local_origin) && strcmp(cfg.local_origin, 'first_platform')
    % 使用默认（首条平台位置）
elseif isnumeric(cfg.local_origin) && numel(cfg.local_origin) == 3
    anchor_lat = cfg.local_origin(1);
    anchor_lon = cfg.local_origin(2);
    anchor_alt = cfg.local_origin(3);
end

% 锚点ECEF坐标和ENU旋转矩阵
anchor_ecef = llh_to_ecef(anchor_lat, anchor_lon, anchor_alt);
R_ecef2enu_anchor = ecef_to_enu_rot(anchor_lat, anchor_lon);

fprintf('跟踪系锚点: lat=%.4f deg, lon=%.4f deg, alt=%.0f m\n', ...
    anchor_lat, anchor_lon, anchor_alt);
fprintf('锚点ECEF: [%.0f, %.0f, %.0f] m\n', anchor_ecef(1), anchor_ecef(2), anchor_ecef(3));

%% ── 转换主动量测：地理系RAE → 本地ENU ─────────────────────────────────
n_act = numel(all_t_active);
active_xyz_local = nan(3, n_act);
active_rae_orig  = zeros(3, n_act);
active_cov_local = nan(3, 3, n_act);   % 每点ENU量测协方差(内部用于凝聚门/加权融合)
% 主动量测RAE噪声标准差（缺省回退）
sig_r  = local_get_cfg(cfg, 'sigma_range_m', 150);
sig_az = deg2rad(local_get_cfg(cfg, 'sigma_az_deg', 0.08));
sig_el = deg2rad(local_get_cfg(cfg, 'sigma_el_deg', 0.06));
sig2   = [sig_r^2; sig_az^2; sig_el^2];

for i = 1:n_act
    t_i = all_t_active(i);
    az_i = all_az_active(i);
    el_i = all_el_active(i);
    r_i  = all_range_active(i);

    % 保存原始RAE（用于后续角度关联）
    active_rae_orig(:, i) = [r_i; az_i; el_i];

    if ~all_range_valid_active(i) || ~isfinite(r_i) || r_i <= 0
        continue;
    end

    % 插值平台状态
    plat_lat = platform.interp_lat(t_i);
    plat_lon = platform.interp_lon(t_i);
    plat_alt = platform.interp_alt(t_i);

    % 平台ECEF
    plat_ecef = llh_to_ecef(plat_lat, plat_lon, plat_alt);
    R_ecef2enu_plat = ecef_to_enu_rot(plat_lat, plat_lon);

    % 地理系RAE → 平台ENU
    % 方位角从北顺时针，东=sin(az), 北=cos(az)
    az_rad = deg2rad(az_i);
    el_rad = deg2rad(el_i);
    de = r_i * cos(el_rad) * sin(az_rad);  % 东向分量
    dn = r_i * cos(el_rad) * cos(az_rad);  % 北向分量
    du = r_i * sin(el_rad);                 % 天向分量

    % 平台ENU → ECEF
    target_ecef = plat_ecef + R_ecef2enu_plat' * [de; dn; du];

    % ECEF → 锚点ENU（跟踪系）
    rel_ecef = target_ecef - anchor_ecef;
    xyz_local = R_ecef2enu_anchor * rel_ecef;  % [E; N; U]

    active_xyz_local(:, i) = xyz_local;

    % 该点ENU量测协方差：RAE→平台ENU雅可比，再旋到锚点ENU
    ce = cos(el_rad); se = sin(el_rad); ca = cos(az_rad); sa = sin(az_rad);
    J = [ ce*sa,  r_i*ce*ca, -r_i*se*sa; ...
          ce*ca, -r_i*ce*sa, -r_i*se*ca; ...
          se,     0,          r_i*ce    ];
    Rp = J * diag(sig2) * J';                       % 平台ENU下协方差
    Mrot = R_ecef2enu_anchor * R_ecef2enu_plat';    % 平台ENU → 锚点ENU 旋转
    active_cov_local(:, :, i) = Mrot * Rp * Mrot';
end

fprintf('主动量测坐标转换完成: %d 点\n', n_act);

%% ── 时间分帧 ──────────────────────────────────────────────────────────
% 策略：将所有量测（主动+被动）按时间排序，用滑动时间窗口分帧
% 每帧的窗口内包含该时间段的所有量测

% 收集所有时间戳
all_times = all_t_active;
if ~isempty(all_t_passive)
    all_times = [all_times; all_t_passive];
end
all_times = sort(all_times);

if isempty(all_times)
    frames = struct('t_sec', {}, 'active_xyz', {}, 'active_rae', {}, 'active_R', {}, ...
                    'active_ang_precondense', {}, ...
                    'active_ae_only', {}, 'active_ae_only_src', {}, ...
                    'active_ae_only_ids', {}, 'active_ae_only_t', {}, ...
                    'passive_ang', {}, 'passive_src', {}, 'active_src', {}, ...
                    'target_ids', {}, 'passive_ids', {}, ...
                    'active_t', {}, 'passive_t', {});
    info.n_active_after_condensation = 0;
    info.n_passive_after_condensation = 0;
    info.n_active_condensed = info.n_active_input;
    fprintf('警告: 无量测数据\n');
    return;
end

% 用时间窗口分帧
tw = cfg.frame_time_window_s;
frame_times = zeros(1, numel(all_times));
n_frame_times = 0;

i = 1;
while i <= numel(all_times)
    n_frame_times = n_frame_times + 1;
    frame_times(n_frame_times) = all_times(i);
    % 找到窗口结束位置
    t_start = all_times(i);
    j = i;
    while j <= numel(all_times) && (all_times(j) - t_start) <= tw
        j = j + 1;
    end
    i = j;
end
frame_times = frame_times(1:n_frame_times);

n_frames = numel(frame_times);

fprintf('时间分帧: %d 帧 (窗口=%.3f s)\n', n_frames, tw);
if n_frames > 1
    dt_frames = diff(frame_times);
    fprintf('  帧间间隔: min=%.4f s, median=%.4f s, max=%.4f s\n', ...
        min(dt_frames), median(dt_frames), max(dt_frames));
end

%% ── 构建每帧数据结构 ──────────────────────────────────────────────────
frames = struct();
frames(n_frames).t_sec = [];  % 预分配
frame_cov = cell(1, n_frames);   % 每帧主动量测协方差(并行存储,内部用)
active_frame_indices = partition_frame_indices(all_t_active, frame_times, tw);
passive_frame_indices = partition_frame_indices(all_t_passive, frame_times, tw);

for k = 1:n_frames
    t_k = frame_times(k);

    act_idx = active_frame_indices{k};
    pas_idx = passive_frame_indices{k};

    % 帧代表时刻只服务于主动量测的 frame 时间模式；所有被动量测仍保留
    % passive_t 中的原始时间戳，异步事件构造不会把它们插值到帧代表时刻。
    if ~isempty(act_idx)
        t_rep = median(all_t_active(act_idx));
    elseif ~isempty(pas_idx)
        t_rep = median(all_t_passive(pas_idx));
    else
        t_rep = t_k;
    end

    frames(k).t_sec        = t_rep;
    act3_idx = act_idx(all_range_valid_active(act_idx));
    act2_idx = act_idx(~all_range_valid_active(act_idx));
    frames(k).active_xyz   = active_xyz_local(:, act3_idx);
    frames(k).active_rae   = active_rae_orig(:, act3_idx);
    frames(k).active_R     = active_cov_local(:, :, act3_idx);
    frames(k).active_ae_only = [all_az_active(act2_idx)'; all_el_active(act2_idx)'];
    frames(k).active_ang_precondense = [active_rae_orig(2:3, act3_idx), ...
        frames(k).active_ae_only];
    frames(k).active_ae_only_src = all_src_active(act2_idx)';
    frames(k).active_ae_only_ids = all_tid_active(act2_idx)';
    frames(k).active_ae_only_t = all_t_active(act2_idx)';
    frames(k).passive_ang  = [all_az_passive(pas_idx)'; all_el_passive(pas_idx)'];
    frames(k).passive_src  = all_passive_src(pas_idx)';
    frames(k).active_src   = all_src_active(act3_idx)';
    frames(k).target_ids   = all_tid_active(act3_idx)';
    frames(k).passive_ids  = all_tid_passive(pas_idx)';
    frames(k).active_t     = all_t_active(act3_idx)';
    frames(k).passive_t    = all_t_passive(pas_idx)';
    frame_cov{k}           = active_cov_local(:, :, act3_idx);
end

%% ── 帧内主动空时凝聚（可选） ──────────────────────────────────────────
%  仅对主动量测做凝聚，被动角度原样保留，不受此开关影响。
%  方法(cfg.condense_method)：
%    'spatiotemporal' 默认：per-sensor分辨单元去重 + 时空速度一致性门(防交叉粘连)
%    'resolution'     仅 per-sensor 分辨单元去重(各向异性统计门, 信息加权融合)
%    'radius'         旧逻辑：固定半径欧氏质心聚类(回退用)
condense_enable = isfield(cfg, 'condense_enable') && cfg.condense_enable;
if condense_enable
    method = lower(local_get_cfg(cfg, 'condense_method', 'spatiotemporal'));
    n_before = sum(cellfun(@(x) size(x, 2), {frames.active_xyz}));
    switch method
        case 'radius'
            frames = condense_radius(frames, cfg);
        case 'resolution'
            frames = condense_resolution(frames, frame_cov, cfg);
        case 'spatiotemporal'
            frames = condense_spatiotemporal(frames, frame_cov, cfg);
        otherwise
            warning('未知 condense_method=%s，改用 spatiotemporal', method);
            frames = condense_spatiotemporal(frames, frame_cov, cfg);
    end
    n_after = sum(cellfun(@(x) size(x, 2), {frames.active_xyz}));
    fprintf('主动空时凝聚[%s]: %d → %d 点 (压缩比=%.1f%%)\n', ...
        method, n_before, n_after, 100*(1 - n_after/max(n_before, 1)));
end

% 统计
n_act_total = sum(cellfun(@(x) size(x,2), {frames.active_xyz}));
n_act_ae_only = sum(cellfun(@(x) size(x,2), {frames.active_ae_only}));
n_pas_total = sum(cellfun(@(x) size(x,2), {frames.passive_ang}));
info.n_active_after_condensation = n_act_total + n_act_ae_only;
info.n_passive_after_condensation = n_pas_total;
info.n_active_condensed = info.n_active_input - info.n_active_after_condensation;
if info.n_active_condensed < 0 || info.n_passive_after_condensation ~= info.n_passive_input
    error('cohere_measurements:InputAccountingMismatch', ...
        '空时凝聚输入守恒失败：主动差额=%d，被动差额=%d。', ...
        info.n_active_condensed, info.n_passive_input - info.n_passive_after_condensation);
end
fprintf('分帧结果: %d 帧, 主动RAE=%d, 主动AE-only=%d, 被动AE=%d\n', ...
    n_frames, n_act_total, n_act_ae_only, n_pas_total);

end

function groups = partition_frame_indices(times, frame_times, tw)
% Assign sorted timestamps once, then restore the original in-frame order.
n_frames = numel(frame_times);
groups = cell(1, n_frames);
if isempty(times) || n_frames == 0, return; end

[sorted_times, original_indices] = sort(times(:));
cursor = 1;
n_times = numel(sorted_times);
for k = 1:n_frames
    t_lo = frame_times(k) - 1e-9;
    while cursor <= n_times && sorted_times(cursor) < t_lo
        cursor = cursor + 1;
    end
    first = cursor;
    if k < n_frames
        upper = frame_times(k + 1);
        while cursor <= n_times && sorted_times(cursor) < upper
            cursor = cursor + 1;
        end
    else
        upper = frame_times(k) + tw;
        while cursor <= n_times && sorted_times(cursor) <= upper
            cursor = cursor + 1;
        end
    end
    groups{k} = sort(original_indices(first:cursor - 1));
end
end

%% ═══════════════════════════════════════════════════════════════════════════
%  坐标转换工具函数
%% ═══════════════════════════════════════════════════════════════════════════

function ecef = llh_to_ecef(lat_deg, lon_deg, alt_m)
% CGCS2000/WGS84 大地坐标 → ECEF直角坐标
% WGS84椭球参数
a = 6378137.0;           % 长半轴 (m)
f = 1 / 298.257223563;   % 扁率
e2 = 2*f - f^2;          % 第一偏心率平方

lat = deg2rad(lat_deg);
lon = deg2rad(lon_deg);

sin_lat = sin(lat);
cos_lat = cos(lat);
sin_lon = sin(lon);
cos_lon = cos(lon);

N = a / sqrt(1 - e2 * sin_lat^2);  % 卯酉圈曲率半径

x = (N + alt_m) * cos_lat * cos_lon;
y = (N + alt_m) * cos_lat * sin_lon;
z = (N * (1 - e2) + alt_m) * sin_lat;

ecef = [x; y; z];
end

%% ═══════════════════════════════════════════════════════════════════════════
function R = ecef_to_enu_rot(lat_deg, lon_deg)
% ECEF → ENU (东北天) 旋转矩阵
% 输入 lat, lon 单位为度
% R * v_ecef → v_enu

lat = deg2rad(lat_deg);
lon = deg2rad(lon_deg);

sin_lat = sin(lat); cos_lat = cos(lat);
sin_lon = sin(lon); cos_lon = cos(lon);

R = [-sin_lon,            cos_lon,            0;
     -sin_lat*cos_lon,   -sin_lat*sin_lon,    cos_lat;
      cos_lat*cos_lon,    cos_lat*sin_lon,    sin_lat];
end

%% ═══════════════════════════════════════════════════════════════════════════
%  空时凝聚：方法实现
%% ═══════════════════════════════════════════════════════════════════════════

function frames = condense_radius(frames, cfg)
% 旧逻辑(回退)：固定半径欧氏质心聚类，每帧独立。
radius_m = local_get_cfg(cfg, 'condense_radius_m', 200);
radius2  = radius_m * radius_m;
for k = 1:numel(frames)
    xyz = frames(k).active_xyz; rae = frames(k).active_rae;
    tids = frames(k).target_ids; srcs = frames(k).active_src;
    if isfield(frames, 'active_R') && ~isempty(frames(k).active_R)
        cov = frames(k).active_R;
    else
        cov = repmat(eye(3) * 1e6, 1, 1, size(xyz, 2));
    end
    M = size(xyz, 2);
    ts = frame_active_times(frames(k), M);
    if M <= 1, frames(k).active_t = ts; frames(k).active_R = cov; continue; end
    visited = false(1, M); clusters = {};
    for i = 1:M
        if visited(i), continue; end
        visited(i) = true; cluster = i; changed = true;
        while changed
            changed = false; centroid = mean(xyz(:, cluster), 2);
            for j = 1:M
                if visited(j), continue; end
                if sum((xyz(:, j) - centroid).^2) < radius2
                    cluster(end+1) = j; visited(j) = true; changed = true; %#ok<AGROW>
                end
            end
        end
        clusters{end+1} = cluster; %#ok<AGROW>
    end
    nc = numel(clusters);
    xn = zeros(3, nc); rn = zeros(3, nc); tn = zeros(1, nc); sn = zeros(1, nc); tm = zeros(1, nc); Cn = zeros(3, 3, nc);
    for c = 1:nc
        mem = clusters{c};
        [xn(:, c), rn(:, c), tn(c), sn(c), Cn(:, :, c), tm(c)] = ...
            merge_group(xyz, rae, tids, srcs, cov, ts, mem);
    end
    frames(k).active_xyz = xn; frames(k).active_rae = rn;
    frames(k).target_ids = tn; frames(k).active_src = sn; frames(k).active_t = tm;
    frames(k).active_R = Cn;
end
end

%% ═══════════════════════════════════════════════════════════════════════════
function frames = condense_resolution(frames, frame_cov, cfg)
% 方法4：per-sensor 分辨单元去重（各向异性统计门 + 信息加权融合），每帧独立。
[res, protect] = res_params(cfg);
for k = 1:numel(frames)
    M = size(frames(k).active_xyz, 2);
    tm = frame_active_times(frames(k), M);
    if M <= 1, frames(k).active_t = tm; frames(k).active_R = frame_cov{k}; continue; end
    [xo, ro, to, so, Co, tmo] = dedup_resolution_frame(frames(k).active_xyz, ...
        frames(k).active_rae, frames(k).target_ids, frames(k).active_src, ...
        frame_cov{k}, tm, res, protect);
    frames(k).active_xyz = xo; frames(k).active_rae = ro;
    frames(k).target_ids = to; frames(k).active_src = so; frames(k).active_t = tmo;
    frames(k).active_R = Co;
end
end

%% ═══════════════════════════════════════════════════════════════════════════
function frames = condense_spatiotemporal(frames, frame_cov, cfg)
% 方法4 + 方法1：先 per-sensor 分辨单元去重，再用时空速度一致性门把跨帧同源点
% 归入同一微航迹（恒速预测门控），每帧每条微航迹只输出一个凝聚点。
% 关键：交叉目标速度方向不同→即使瞬时空间重合也不会被并；快目标窗内不拖尾。
% 注意：微航迹仅用于"分组凝聚"，不做状态估计，滤波器完全不受影响。
[res, protect] = res_params(cfg);
gamma_trk   = local_get_cfg(cfg, 'condense_gate_gamma',  16);
gamma_birth = local_get_cfg(cfg, 'condense_birth_gamma',  9);
amax        = local_get_cfg(cfg, 'condense_amax',        30);
beta        = local_get_cfg(cfg, 'condense_vel_beta',   0.5);
coast       = round(local_get_cfg(cfg, 'condense_coast', 3));

TRK = struct('x', {}, 'v', {}, 'P', {}, 't', {}, 'last_k', {});
for k = 1:numel(frames)
    xyz = frames(k).active_xyz; M0 = size(xyz, 2);
    if M0 == 0, frames(k).active_t = zeros(1, 0); frames(k).active_R = zeros(3, 3, 0); continue; end
    rae = frames(k).active_rae; tid = frames(k).target_ids;
    src = frames(k).active_src; cov = frame_cov{k};
    tm = frame_active_times(frames(k), M0); t = frames(k).t_sec;

    % Stage A：per-sensor 分辨单元去重
    [xyz, rae, tid, src, cov, tm] = dedup_resolution_frame(xyz, rae, tid, src, cov, tm, res, protect);
    M = size(xyz, 2);

    % Stage B：恒速预测门控，把点关联到活动微航迹
    lab = zeros(1, M);
    act = [];
    for ti = 1:numel(TRK)
        if (k - TRK(ti).last_k) <= coast, act(end+1) = ti; end %#ok<AGROW>
    end
    na = numel(act);
    xhat = zeros(3, na); Phat = zeros(3, 3, na);
    for a = 1:na
        ti = act(a); dt = t - TRK(ti).t;
        xhat(:, a) = TRK(ti).x + TRK(ti).v * dt;
        Phat(:, :, a) = TRK(ti).P + eye(3) * (0.5 * amax * dt * dt)^2;
    end
    for i = 1:M
        bestd = gamma_trk; besta = 0;
        for a = 1:na
            d = maha2(xyz(:, i) - xhat(:, a), cov(:, :, i) + Phat(:, :, a));
            if d < bestd, bestd = d; besta = a; end
        end
        if besta > 0, lab(i) = act(besta); end
    end

    % 余点出生（马氏门 + ID保护，种子单遍；不可分辨的重复并入同一新航迹）
    left = find(lab == 0); usedl = false(1, numel(left));
    for a = 1:numel(left)
        if usedl(a), continue; end
        ia = left(a);
        TRK(end+1) = struct('x', xyz(:, ia), 'v', [0; 0; 0], ...
            'P', cov(:, :, ia), 't', t, 'last_k', k); %#ok<AGROW>
        newid = numel(TRK); lab(ia) = newid; usedl(a) = true;
        for b = a+1:numel(left)
            if usedl(b), continue; end
            ib = left(b);
            if protect && isfinite(tid(ia)) && isfinite(tid(ib)) && tid(ia) ~= tid(ib)
                continue;
            end
            if maha2(xyz(:, ia) - xyz(:, ib), cov(:, :, ia) + cov(:, :, ib)) < gamma_birth
                lab(ib) = newid; usedl(b) = true;
            end
        end
    end

    % 按标签合并为每帧凝聚点，并更新微航迹状态
    ulab = unique(lab); n = numel(ulab);
    xo = zeros(3, n); ro = zeros(3, n); to = zeros(1, n); so = zeros(1, n); tmo = zeros(1, n); Co = zeros(3, 3, n);
    for c = 1:n
        idx = find(lab == ulab(c));
        [xm, rm, tidm, sm, Cm, tmm] = merge_group(xyz, rae, tid, src, cov, tm, idx);
        xo(:, c) = xm; ro(:, c) = rm; to(c) = tidm; so(c) = sm; tmo(c) = tmm; Co(:, :, c) = Cm;
        ti = ulab(c); dt = t - TRK(ti).t;
        if dt > 1e-6 && (k - TRK(ti).last_k) >= 1
            vmeas = (xm - TRK(ti).x) / dt;
            if all(TRK(ti).v == 0), TRK(ti).v = vmeas;
            else, TRK(ti).v = beta * TRK(ti).v + (1 - beta) * vmeas; end
        end
        TRK(ti).x = xm; TRK(ti).P = Cm; TRK(ti).t = t; TRK(ti).last_k = k;
    end
    frames(k).active_xyz = xo; frames(k).active_rae = ro;
    frames(k).target_ids = to; frames(k).active_src = so; frames(k).active_t = tmo;
    frames(k).active_R = Co;
end
end

%% ═══════════════════════════════════════════════════════════════════════════
function [xo, ro, to, so, Co, tmo] = dedup_resolution_frame(xyz, rae, tid, src, cov, tm, res, protect)
% 单帧内：同一传感器、且RAE落在分辨单元内的点视为不可分辨重复，信息加权合并。
M = size(xyz, 2);
if M == 0, xo = xyz; ro = rae; to = tid; so = src; Co = cov; tmo = tm; return; end
lab = zeros(1, M); nextid = 0;
usrc = unique(src);
for s = usrc
    mi = find(src == s); used = false(1, numel(mi));
    for a = 1:numel(mi)
        if used(a), continue; end
        ia = mi(a); nextid = nextid + 1; lab(ia) = nextid; used(a) = true;
        for b = a+1:numel(mi)
            if used(b), continue; end
            ib = mi(b);
            if protect && isfinite(tid(ia)) && isfinite(tid(ib)) && tid(ia) ~= tid(ib)
                continue;
            end
            if abs(rae(1, ia) - rae(1, ib)) < res(1) && ...
               abs(angdiff_deg(rae(2, ia), rae(2, ib))) < res(2) && ...
               abs(rae(3, ia) - rae(3, ib)) < res(3)
                lab(ib) = lab(ia); used(b) = true;
            end
        end
    end
end
ulab = unique(lab); n = numel(ulab);
xo = zeros(3, n); ro = zeros(3, n); to = zeros(1, n); so = zeros(1, n); Co = zeros(3, 3, n); tmo = zeros(1, n);
for c = 1:n
    idx = find(lab == ulab(c));
    [xo(:, c), ro(:, c), to(c), so(c), Co(:, :, c), tmo(c)] = merge_group(xyz, rae, tid, src, cov, tm, idx);
end
end

%% ═══════════════════════════════════════════════════════════════════════════
function [x, rae1, tid1, src1, Cf, tm1] = merge_group(xyz, rae, tid, src, cov, tm, idx)
% 一组量测 → 一个点：位置用逆协方差(信息)加权，方位用圆形均值，编号取众数。
n = numel(idx);
Wsum = zeros(3); bsum = zeros(3, 1);
for q = 1:n
    W = safe_inv3(cov(:, :, idx(q)));
    Wsum = Wsum + W; bsum = bsum + W * xyz(:, idx(q));
end
Cf = safe_inv3(Wsum); x = Cf * bsum;
rae1 = zeros(3, 1);
rae1(1) = mean(rae(1, idx));
ar = deg2rad(rae(2, idx)); rae1(2) = rad2deg(atan2(mean(sin(ar)), mean(cos(ar))));
rae1(3) = mean(rae(3, idx));
vt = tid(idx); vt = vt(~isnan(vt));
if isempty(vt), tid1 = NaN; else, tid1 = mode(vt); end
src1 = src(idx(1));
tm1 = median(tm(idx));
end

%% ═══════════════════════════════════════════════════════════════════════════
function [res, protect] = res_params(cfg)
res = [local_get_cfg(cfg, 'condense_res_range_m', 300), ...
       local_get_cfg(cfg, 'condense_res_az_deg',  0.16), ...
       local_get_cfg(cfg, 'condense_res_el_deg',  0.12)];
% target_id is evaluation metadata and must never affect preprocessing.
protect = false;
end

%% ═══════════════════════════════════════════════════════════════════════════
function d = maha2(dx, S)
d = dx' * (safe_inv3(S) * dx);
end

%% ═══════════════════════════════════════════════════════════════════════════
function Ai = safe_inv3(A)
A = 0.5 * (A + A');
r = rcond(A);
if ~isfinite(r) || r < 1e-12
    A = A + (1e-9 * max(trace(A), 1) / 3) * eye(3);
end
Ai = A \ eye(3);
end

%% ═══════════════════════════════════════════════════════════════════════════
function d = angdiff_deg(a, b)
d = mod(a - b + 180, 360) - 180;
end

%% ═══════════════════════════════════════════════════════════════════════════
function v = local_get_cfg(cfg, f, d)
if isfield(cfg, f) && ~isempty(cfg.(f)), v = cfg.(f); else, v = d; end
end

function t = frame_active_times(frame, n)
if isfield(frame, 'active_t') && numel(frame.active_t) == n
    t = frame.active_t(:).';
else
    t = frame.t_sec * ones(1, n);
end
end
