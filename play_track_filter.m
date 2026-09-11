function play_track_filter(est, frame_times, opts)
%PLAY_TRACK_FILTER  滤波后回放：按时间轴推进，选择性绘制指定航迹ID的
%                   估计轨迹（亮色实线）、归属量测（浅灰）、估计协方差椭球与关联门椭球。
%                   3D（锚点 ENU）。
%
%  依赖：run_filter_adapt_ckf 已导出
%        est.assoc{k}=struct('id','xyz')        —— 归属量测（A方案）
%        est.gate{k} =struct('id','pos','S')    —— 关联门（CKF预测位置+S=Pzz+R）
%        est.gate_gamma                          —— 卡方门限
%
%  两种模式（opts.mode）：
%    'slider' 交互模式（不录像时默认）：定时器驱动，可暂停/继续，可拖动滑条到
%             任意时刻、可后退、可设起点；随时自由旋转/缩放。
%    'play'   自动播放（指定 save_path 录像时默认）：阻塞循环，顺序写帧。
%
%  常用：
%    play_track_filter(est, frame_times);                 % 交互查看全部确认航迹
%    opts.track_ids=[3 7]; play_track_filter(est,frame_times,opts);
%    opts.save_path='play.mp4'; play_track_filter(est,frame_times,opts);  % 录像
%
%  opts（全部可选）:
%    mode           'slider'|'play'                         默认 自动
%    track_ids      [向量]要看的ID；留空=全部确认航迹        默认 []
%    trail_mode     'cumulative'|'window'                   默认 'cumulative'
%    trail_window_s window 模式时间窗(秒)                    默认 10
%    time_step      抽帧步长                                 默认 1
%    pause_s        每帧间隔(秒)                             默认 0.05
%    save_path      ''/'*.mp4'/'*.avi'/'*.gif'               默认 ''
%    fps            录像帧率                                 默认 20
%    show_meas      画归属量测                               默认 true
%    meas_color     量测颜色                                 默认 浅灰[0.7 0.7 0.7]
%    show_cov       画估计协方差椭球(实心,各航迹色)          默认 true
%    cov_sigma      估计椭球σ倍数                            默认 2
%    cov_inflate    估计椭球视觉放大系数(仅显示,不改数据)    默认 1
%    show_gate      画关联门椭球(网格)                       默认 true
%    gate_color     门椭球颜色                               默认 橙[0.85 0.33 0.10]
%    line_width     轨迹线宽                                 默认 2
%    show_background 截至当前全部融合量测淡背景(需fused_xyz)  默认 false
%    fused_xyz      {K×1}                                    默认 {}
%    fused_ids      {K×1} 与 fused_xyz 列对应的目标编号        默认 {}
%    cfg            配置结构；提供后可按 truth_id_split_* 画拆分实例 默认 []
%    truth_use_split true=伪真值使用拆分实例编号；留空=跟随cfg       默认 []
%    title_prefix   标题前缀                                 默认 ''
%    show_truth     画按编号tid连接的关联伪真值轨迹(浅色参考)默认 true
%    truth_ids      指定要画的目标编号tid；留空=选中航迹主导tid 默认 []
%    truth_color    统一真值色；留空=按航迹色淡化            默认 []
%    truth_tint     朝白淡化比例(0~1,越小越浅)               默认 0.30
%    truth_line_width 真值线宽                               默认 1.2
%    truth_full     true=整条静态/false=随时间揭示           默认 false
%    smooth         smooth_filter_tracks 的输出(叠加平滑线)  默认 []
%    show_smooth    画二次平滑虚线                            默认 ~isempty(smooth)
%    smooth_line_width 平滑线宽                               默认 line_width+0.6
%    smooth_style   平滑线型                                 默认 '--'

if nargin < 3, opts = struct(); end
gp = @(f, d) get_opt(opts, f, d);
pos_idx = [1, 4, 7];
K = numel(frame_times);

mode            = gp('mode', '');
track_ids       = gp('track_ids', [142]);
trail_mode      = lower(gp('trail_mode', 'cumulative'));
trail_window_s  = gp('trail_window_s', 10);
time_step       = max(1, round(gp('time_step', 1)));
pause_s         = gp('pause_s', 0.05);
show            = gp('show', true);
save_path       = gp('save_path', '');
fps             = gp('fps', 20);
show_meas       = gp('show_meas', true);
meas_color      = gp('meas_color', [0.70 0.70 0.70]);
show_cov        = gp('show_cov', true);
cov_sigma       = gp('cov_sigma', 2);
cov_inflate     = gp('cov_inflate', 1);
show_gate       = gp('show_gate', true);
gate_color      = gp('gate_color', [0.85 0.33 0.10]);
line_width      = gp('line_width', 2);
show_background = gp('show_background', false);
fused_xyz       = gp('fused_xyz', {});
fused_ids       = gp('fused_ids', {});
cfg_in          = gp('cfg', []);
title_prefix    = gp('title_prefix', '');
% ── 关联伪真值轨迹（按目标编号tid连接关联量测，浅色参考）──
show_truth      = gp('show_truth', true);        % 画按编号连接的关联伪真值轨迹
truth_ids       = gp('truth_ids', []);           % 指定要画的目标编号tid；留空=自动取选中航迹的主导tid
truth_color     = gp('truth_color', []);         % 统一真值色；留空=按对应航迹色淡化
truth_tint      = gp('truth_tint', 0.30);        % 朝白淡化比例(0~1)，越小越浅
truth_line_width= gp('truth_line_width', 1.2);   % 真值线宽(细)
truth_full      = gp('truth_full', false);       % true=整条静态显示；false=随时间揭示(同轨迹trail)
truth_use_split = gp('truth_use_split', []);
if isempty(truth_use_split)
    truth_use_split = isstruct(cfg_in) && isfield(cfg_in, 'truth_id_split_enabled') && ...
        ~isempty(cfg_in.truth_id_split_enabled) && cfg_in.truth_id_split_enabled ~= 0;
end
truth_use_split = truth_use_split ~= 0 && iscell(fused_xyz) && iscell(fused_ids) && ...
    ~isempty(fused_xyz) && ~isempty(fused_ids);
truth_ids_full = fused_ids;
truth_split_info = [];
if truth_use_split
    [truth_ids_full, truth_split_info] = build_split_truth_labels(frame_times, fused_xyz, fused_ids, cfg_in);
end
% ── 二次平滑航迹叠加（传入 smooth_filter_tracks 的输出即可一块出）──
smooth_in        = gp('smooth', []);              % smooth_filter_tracks 的输出 smt；空=不叠加
show_smooth      = gp('show_smooth', ~isempty(smooth_in)) ~= 0;
smooth_line_width= gp('smooth_line_width', line_width + 0.6);
smooth_style     = gp('smooth_style', '--');      % 平滑线型(默认虚线,便于与滤波实线区分)

% 解析模式：未指定时，录像走 play，否则走 slider
if isempty(mode)
    if isempty(save_path), mode = 'slider'; else, mode = 'play'; end
else
    mode = lower(mode);
end

if K == 0 || ~isfield(est, 'tracks')
    warning('est 为空或缺少 tracks 字段，无法回放'); return;
end
if show_meas && ~isfield(est, 'assoc')
    warning('est 缺少 assoc 字段，请用导出关联的滤波器重跑；本次不画量测。');
    show_meas = false;
end
if show_truth && ~isfield(est, 'assoc')
    warning('est 缺少 assoc 字段，无法构建按编号的关联伪真值轨迹；本次不画伪真值。');
    show_truth = false;
end
gate_radius = 0;
if show_gate
    if isfield(est, 'gate') && isfield(est, 'gate_gamma') && ~isempty(est.gate_gamma)
        gate_radius = sqrt(est.gate_gamma);
    else
        warning('est 缺少 gate/gate_gamma 字段，请用导出关联门的滤波器重跑；本次不画门。');
        show_gate = false;
    end
end
% 录像必须顺序写帧 → play 模式
if ~isempty(save_path) && strcmp(mode, 'slider')
    warning('slider 为交互模式，已忽略 save_path（录像请用 mode=''play''）。');
    save_path = '';
end
if strcmp(mode, 'slider'), show = true; end

%% ── 选定航迹ID ────────────────────────────────────────────────────────
all_conf_ids = [];
for k = 1:K
    if ~isempty(est.L{k}), all_conf_ids = [all_conf_ids; est.L{k}(:, 2)]; end %#ok<AGROW>
end
all_conf_ids = unique(all_conf_ids(:))';
if isempty(track_ids), sel_ids = all_conf_ids; else, sel_ids = unique(track_ids(:))'; end
if isempty(sel_ids)
    warning('没有可显示的航迹ID。'); return;
end
n_sel = numel(sel_ids);

% 亮色、饱和的各航迹配色
base = lines(max(n_sel, 1));
hsvc = rgb2hsv(base(1:n_sel, :));
hsvc(:, 2) = max(hsvc(:, 2), 0.75);   % 提饱和
hsvc(:, 3) = max(hsvc(:, 3), 0.85);   % 提亮度
colors = hsv2rgb(hsvc);

idmap = containers.Map('KeyType', 'double', 'ValueType', 'double');
for s = 1:n_sel, idmap(sel_ids(s)) = s; end

%% ── 预扫描：估计(位置/协方差/时间)、归属量测、关联门 ─────────────────────
EST  = repmat(struct('t', zeros(1,K), 'p', zeros(3,K), 'C', zeros(3,3,K), 'n', 0), 1, n_sel);
MEAS = repmat(struct('t', zeros(1,K), 'p', zeros(3,K), 'n', 0), 1, n_sel);
GATE = repmat(struct('t', zeros(1,K), 'pos', zeros(3,K), 'S', zeros(3,3,K), 'n', 0), 1, n_sel);
for k = 1:K
    tk = frame_times(k);
    Tr = est.tracks{k};
    if ~isempty(Tr) && isfield(Tr, 'L') && ~isempty(Tr.L)
        ids = Tr.L(2, :);
        for c = 1:numel(ids)
            if isKey(idmap, ids(c))
                s = idmap(ids(c)); m = EST(s).n + 1;
                EST(s).t(m) = tk; EST(s).p(:, m) = Tr.m(pos_idx, c);
                EST(s).C(:, :, m) = Tr.P(pos_idx, pos_idx, c); EST(s).n = m;
            end
        end
    end
    if show_meas
        As = est.assoc{k};
        if ~isempty(As) && isfield(As, 'id') && ~isempty(As.id)
            for c = 1:numel(As.id)
                if isKey(idmap, As.id(c))
                    s = idmap(As.id(c)); m = MEAS(s).n + 1;
                    MEAS(s).t(m) = tk; MEAS(s).p(:, m) = As.xyz(:, c); MEAS(s).n = m;
                end
            end
        end
    end
    if show_gate
        Gk = est.gate{k};
        if ~isempty(Gk) && isfield(Gk, 'id') && ~isempty(Gk.id)
            for c = 1:numel(Gk.id)
                if isKey(idmap, Gk.id(c))
                    s = idmap(Gk.id(c)); m = GATE(s).n + 1;
                    GATE(s).t(m) = tk; GATE(s).pos(:, m) = Gk.pos(:, c);
                    GATE(s).S(:, :, m) = Gk.S(:, :, c); GATE(s).n = m;
                end
            end
        end
    end
end
for s = 1:n_sel
    EST(s).t = EST(s).t(1:EST(s).n); EST(s).p = EST(s).p(:, 1:EST(s).n);
    EST(s).C = EST(s).C(:, :, 1:EST(s).n);
    MEAS(s).t = MEAS(s).t(1:MEAS(s).n); MEAS(s).p = MEAS(s).p(:, 1:MEAS(s).n);
    GATE(s).t = GATE(s).t(1:GATE(s).n); GATE(s).pos = GATE(s).pos(:, 1:GATE(s).n);
    GATE(s).S = GATE(s).S(:, :, 1:GATE(s).n);
end

%% ── 关联伪真值轨迹：按目标编号tid把关联量测连成折线（浅色参考）───────────
% TR(it): struct('tid', t[1×m], p[3×m], color[1×3])，每个tid同一时刻取均值后按时间排序
TR = repmat(struct('tid', NaN, 't', [], 'p', zeros(3, 0), 'color', [0 0 0]), 1, 0);
if show_truth
  try
    % est.assoc 只用于确定“选中滤波航迹对应哪些 tid”；真正画线优先用 fused_xyz/fused_ids 全量量测。
    AssocT = zeros(1, 0); AssocTid = zeros(1, 0); AssocP = zeros(3, 0);
    sel_tid_votes = cell(1, n_sel);
    for k = 1:K
        As = est.assoc{k};
        if isempty(As) || ~isfield(As, 'id') || isempty(As.id), continue; end
        if ~isfield(As, 'tid') || isempty(As.tid), continue; end
        if ~isfield(As, 'xyz') || isempty(As.xyz), continue; end
        ids = As.id(:).'; tids = As.tid(:).'; P = As.xyz;
        ncol = min([numel(ids), numel(tids), size(P, 2)]);
        for c = 1:ncol
            raw_tc = tids(c);
            if truth_use_split
                tc = local_map_assoc_to_plot_id(k, P(1:3, c), raw_tc, fused_xyz, fused_ids, truth_ids_full);
            else
                tc = raw_tc;
            end
            if ~isfinite(tc), continue; end
            AssocT(end+1)   = frame_times(k);  %#ok<AGROW>
            AssocTid(end+1) = tc;              %#ok<AGROW>
            AssocP(:, end+1) = P(1:3, c);      %#ok<AGROW>
            if isKey(idmap, ids(c))
                s = idmap(ids(c));
                if s >= 1 && s <= n_sel
                    sel_tid_votes{s}(end+1) = tc; %#ok<AGROW>
                end
            end
        end
    end
    AllT = zeros(1, 0); AllTid = zeros(1, 0); AllP = zeros(3, 0);
    truth_source = '已关联量测(est.assoc)';
    has_full_truth = iscell(fused_xyz) && iscell(fused_ids) && ...
                     ~isempty(fused_xyz) && ~isempty(fused_ids);
    if has_full_truth
        K_full = min([K, numel(fused_xyz), numel(fused_ids)]);
        for k = 1:K_full
            P = fused_xyz{k};
            ids = truth_ids_full{k};
            if isempty(P) || isempty(ids) || size(P, 1) < 3, continue; end
            ids = ids(:).';
            ncol = min(numel(ids), size(P, 2));
            for c = 1:ncol
                tc = ids(c);
                if ~isfinite(tc), continue; end
                AllT(end+1) = frame_times(k);       %#ok<AGROW>
                AllTid(end+1) = tc;                 %#ok<AGROW>
                AllP(:, end+1) = P(1:3, c);         %#ok<AGROW>
            end
        end
        if truth_use_split
            truth_source = '全量融合量测拆分实例(fused_xyz/fused_ids + truth split)';
            fprintf('[伪真值] 回放使用拆分后伪真值实例：原始编号=%d，拆分实例=%d，被拆分原始编号=%d。\n', ...
                truth_split_info.raw_id_count, truth_split_info.instance_count, truth_split_info.n_split_raw_ids);
        else
            truth_source = '全量融合量测(fused_xyz/fused_ids)';
        end
        if isempty(AllTid) && ~isempty(AssocTid)
            fprintf('[伪真值] fused_ids 中没有有效 tid，回放退回使用 est.assoc 已关联量测。\n');
            AllT = AssocT; AllTid = AssocTid; AllP = AssocP;
            truth_source = '已关联量测(est.assoc)';
        end
    else
        AllT = AssocT; AllTid = AssocTid; AllP = AssocP;
    end
    fprintf('[伪真值] 回放画线来源=%s，tid点数=%d。\n', truth_source, numel(AllTid));
    % 决定要显示的tid：显式指定优先，否则自动取每条选中航迹碰过的全部tid
    if ~isempty(truth_ids)
        show_tids = unique(truth_ids(:)).';
    else
        % 安全：仅遍历“非空胞元”的下标，sel_tid_votes{·} 永不越界
        nz = find(~cellfun('isempty', sel_tid_votes));
        nz = nz(nz >= 1 & nz <= numel(sel_tid_votes));   % 绝对边界保护
        show_tids = zeros(1, 0);
        for ii = 1:numel(nz)
            show_tids = [show_tids, unique(sel_tid_votes{nz(ii)})]; %#ok<AGROW>
        end
        show_tids = unique(show_tids);
        % 兜底：选中航迹没投出tid时，画全部出现过的tid
        if isempty(show_tids) && ~isempty(AllTid)
            show_tids = unique(AllTid(isfinite(AllTid)));
        end
    end
    white = [1 1 1];
    for it = 1:numel(show_tids)
        tdv = show_tids(it);
        sel = (AllTid == tdv);
        if ~any(sel), continue; end
        tt = AllT(sel); pp = AllP(:, sel);
        % 同一时刻多回波 → 取均值，得到每时刻一个伪真值点
        [ut, ~, ic] = unique(tt);
        pm = zeros(3, numel(ut));
        for j = 1:numel(ut), pm(:, j) = mean(pp(:, ic == j), 2); end
        [ut, ord] = sort(ut); pm = pm(:, ord);
        % 配色：显式统一色 > 对应选中航迹色淡化 > 中性浅灰
        if ~isempty(truth_color)
            col = truth_color;
        else
            cs = [];
            nz = find(~cellfun('isempty', sel_tid_votes));
            nz = nz(nz >= 1 & nz <= numel(sel_tid_votes));
            for ii = 1:numel(nz)
                s = nz(ii);
                if s <= size(colors, 1) && local_mode(sel_tid_votes{s}) == tdv
                    cs = colors(s, :); break;
                end
            end
            if isempty(cs), cs = [0.45 0.50 0.60]; end
            col = truth_tint * cs + (1 - truth_tint) * white;   % 朝白淡化
        end
        TR(end+1) = struct('tid', tdv, 't', ut, 'p', pm, 'color', col); %#ok<AGROW>
    end
    if isempty(TR)
        warning('未找到可用的关联伪真值(tid)，本次不画伪真值。');
        show_truth = false;
    end
  catch ME_truth
    nv = 0; if exist('sel_tid_votes', 'var'), nv = numel(sel_tid_votes); end
    fprintf(2, ['[伪真值] 构建失败：%s  (n_sel=%d, numel(sel_tid_votes)=%d)\n' ...
                '         本次回放不画伪真值线；其余照常。若你项目里有自定义的 mode.m/unique.m 等，' ...
                '请改名以免顶替内置函数。\n'], ME_truth.message, n_sel, nv);
    show_truth = false;
    TR = repmat(struct('tid', NaN, 't', [], 'p', zeros(3, 0), 'color', [0 0 0]), 1, 0);
  end
end
n_tr = numel(TR);

%% ── 二次平滑叠加：取每条选中航迹的平滑序列 ────────────────────────────
SM = repmat(struct('t', [], 'p', zeros(3, 0), 'n', 0), 1, n_sel);
if show_smooth && ~isempty(smooth_in) && isfield(smooth_in, 'tracks') && ~isempty(smooth_in.tracks)
    sm_ids = [smooth_in.tracks.id];
    for s = 1:n_sel
        j = find(sm_ids == sel_ids(s), 1);
        if ~isempty(j)
            tk = smooth_in.tracks(j);
            SM(s).t = tk.t; SM(s).p = tk.smooth; SM(s).n = numel(tk.t);
        end
    end
end
show_smooth = show_smooth && any([SM.n] > 0);
if ~isempty(smooth_in) && ~show_smooth
    warning('opts.smooth 已提供但未匹配到选中航迹ID的平滑结果，本次不叠加平滑线。');
end

%% ── 坐标范围 ──────────────────────────────────────────────────────────
allp = zeros(3, 0);
for s = 1:n_sel
    if EST(s).n  > 0, allp = [allp, EST(s).p]; end %#ok<AGROW>
    if MEAS(s).n > 0, allp = [allp, MEAS(s).p]; end %#ok<AGROW>
end
for it = 1:n_tr
    if ~isempty(TR(it).p), allp = [allp, TR(it).p]; end %#ok<AGROW>
end
for s = 1:n_sel
    if SM(s).n > 0, allp = [allp, SM(s).p]; end %#ok<AGROW>
end
if isempty(allp), warning('选中的航迹无数据。'); return; end
mn = min(allp, [], 2); mx = max(allp, [], 2);
rngv = mx - mn; rngv(rngv < 1) = 1; pad = 0.06 * rngv;

%% ── 背景量测（可选） ──────────────────────────────────────────────────
BG = struct('t', [], 'p', zeros(3, 0));
if show_background && ~isempty(fused_xyz)
    nb = 0; for k = 1:K, nb = nb + size(fused_xyz{k}, 2); end
    BG.t = zeros(1, nb); BG.p = zeros(3, nb); p = 0;
    for k = 1:K
        z = fused_xyz{k}; mz = size(z, 2);
        if mz > 0, BG.t(p+(1:mz)) = frame_times(k); BG.p(:, p+(1:mz)) = z; p = p + mz; end
    end
end

%% ── 建图 ──────────────────────────────────────────────────────────────
vis = 'on'; if ~show, vis = 'off'; end
fig = figure('Name', '航迹滤波回放', 'Color', 'w', 'Visible', vis, ...
             'Position', [80, 80, 1040, 840]);
ax = axes('Parent', fig); hold(ax, 'on'); grid(ax, 'on'); box(ax, 'on');
xlabel(ax, 'East (m)'); ylabel(ax, 'North (m)'); zlabel(ax, 'Up (m)');
xlim(ax, [mn(1)-pad(1), mx(1)+pad(1)]);
ylim(ax, [mn(2)-pad(2), mx(2)+pad(2)]);
zlim(ax, [mn(3)-pad(3), mx(3)+pad(3)]);
view(ax, 45, 30);
try, axis(ax, 'vis3d'); catch, end

hBg = [];
if show_background && ~isempty(BG.t)
    hBg = plot3(ax, nan, nan, nan, '.', 'Color', [0.88 0.88 0.88], 'MarkerSize', 3);
end

[ux, uy, uz] = sphere(14);
H = struct('line', {}, 'meas', {}, 'head', {}, 'txt', {}, 'cov', {}, 'gate', {});
for s = 1:n_sel
    col = colors(s, :);
    % 量测：浅灰
    H(s).meas = plot3(ax, nan, nan, nan, '.', 'Color', meas_color, 'MarkerSize', 8);
    if ~show_meas, set(H(s).meas, 'Visible', 'off'); end
    % 轨迹：亮色实线、加粗
    H(s).line = plot3(ax, nan, nan, nan, '-', 'Color', col, 'LineWidth', line_width);
    % 当前估计点
    H(s).head = plot3(ax, nan, nan, nan, 'o', 'MarkerEdgeColor', 'k', ...
                      'MarkerFaceColor', col, 'MarkerSize', 7, 'LineWidth', 0.5);
    H(s).txt  = text(ax, mn(1), mn(2), mn(3), sprintf(' Track %d', sel_ids(s)), ...
                     'Color', col, 'FontWeight', 'bold', 'FontSize', 9, ...
                     'VerticalAlignment', 'bottom', 'Visible', 'off');
    % 估计协方差椭球：实心、各航迹色
    if show_cov
        H(s).cov = surf(ax, nan(size(ux)), nan(size(uy)), nan(size(uz)), ...
                        'EdgeColor', 'none', 'FaceColor', col, 'FaceAlpha', 0.18);
    else
        H(s).cov = [];
    end
    % 关联门椭球：网格、统一门色
    if show_gate
        H(s).gate = surf(ax, nan(size(ux)), nan(size(uy)), nan(size(uz)), ...
                         'FaceColor', 'none', 'EdgeColor', gate_color, 'EdgeAlpha', 0.55);
    else
        H(s).gate = [];
    end
end
% 关联伪真值轨迹：浅色、细线、置于底层（参考用，避免压住滤波轨迹）
HT = gobjects(1, n_tr);
for it = 1:n_tr
    HT(it) = plot3(ax, nan, nan, nan, '-', 'Color', TR(it).color, ...
                   'LineWidth', truth_line_width);
end
if n_tr > 0, try, uistack(HT, 'bottom'); catch, end; end
% 二次平滑线：虚线、稍粗、航迹色加深（与滤波实线区分；置于滤波线之上）
HS = gobjects(1, n_sel);
if show_smooth
    for s = 1:n_sel
        if SM(s).n == 0, continue; end
        HS(s) = plot3(ax, nan, nan, nan, smooth_style, ...
                      'Color', 0.65 * colors(s, :), 'LineWidth', smooth_line_width);
    end
end
% 图例：航迹 + 伪真值
if n_sel >= 1
    leg_h = [H.line];
    leg_s = arrayfun(@(g) sprintf('Track %d', g), sel_ids, 'UniformOutput', false);
    if n_tr > 0
        leg_h = [leg_h, HT];
        leg_s = [leg_s, arrayfun(@(e) local_truth_display_name(e.tid, truth_split_info, truth_use_split), ...
            TR, 'UniformOutput', false)];
    end
    if show_smooth
        vsm = arrayfun(@(s) SM(s).n > 0, 1:n_sel);
        if any(vsm)
            leg_h = [leg_h, HS(vsm)];
            leg_s = [leg_s, arrayfun(@(g) sprintf('Track %d 平滑', g), ...
                sel_ids(vsm), 'UniformOutput', false)];
        end
    end
    legend(ax, leg_h, leg_s, 'Location', 'bestoutside');
end
if show_cov || show_gate || (show_truth && n_tr > 0) || show_smooth
    note = '';
    if show_cov
        if cov_inflate ~= 1
            note = sprintf('实心=估计协方差(%gσ×%g放大)   ', cov_sigma, cov_inflate);
        else
            note = sprintf('实心=估计协方差(%gσ)   ', cov_sigma);
        end
    end
    if show_gate, note = [note, sprintf('网格=关联门(√γ=%.2f)   ', gate_radius)]; end
    if show_truth && n_tr > 0, note = [note, '浅细线=关联伪真值(按编号tid连接)   ']; end
    if show_smooth, note = [note, sprintf('虚线=二次平滑(%s)', get_smt_method(smooth_in))]; end
    annotation(fig, 'textbox', [0.01, 0.955, 0.8, 0.035], 'String', note, ...
               'EdgeColor', 'none', 'FontSize', 9, 'Color', [0.25 0.25 0.25]);
end

ks = 1:time_step:K;
nks = numel(ks);

% 共享状态（供嵌套函数）
cur = 1;          % 当前帧在 ks 中的下标
tmr = [];         % 定时器
sld = [];         % 滑条
btn_play = [];    % 播放/暂停按钮

if strcmp(mode, 'slider')
    run_slider();
else
    run_play();
end

% =======================================================================
% 逐帧绘制（所有模式共用）
% =======================================================================
    function draw_frame(k)
        k = min(max(round(k), 1), K);
        t = frame_times(k);
        if ~isempty(hBg)
            mb = BG.t <= t;
            set(hBg, 'XData', BG.p(1, mb), 'YData', BG.p(2, mb), 'ZData', BG.p(3, mb));
        end
        for s = 1:n_sel
            % 轨迹 + 当前点 + 估计协方差
            ne = EST(s).n;
            if ne > 0
                te = EST(s).t;
                if strcmp(trail_mode, 'window'), me = (te <= t) & (te > t - trail_window_s);
                else, me = (te <= t); end
                set(H(s).line, 'XData', EST(s).p(1, me), ...
                               'YData', EST(s).p(2, me), 'ZData', EST(s).p(3, me));
                hi = find(te <= t, 1, 'last');
                if ~isempty(hi)
                    ph = EST(s).p(:, hi);
                    set(H(s).head, 'XData', ph(1), 'YData', ph(2), 'ZData', ph(3));
                    first_visible = find(me, 1);
                    if isempty(first_visible)
                        set(H(s).txt, 'Visible', 'off');
                    else
                        p0 = EST(s).p(:, first_visible);
                        set(H(s).txt, 'Position', p0', 'Visible', 'on');
                    end
                    if show_cov && ~isempty(H(s).cov)
                        [Xe, Ye, Ze] = cov_ellipsoid(EST(s).C(:, :, hi), ph, ...
                                                     ux, uy, uz, cov_sigma * cov_inflate);
                        set(H(s).cov, 'XData', Xe, 'YData', Ye, 'ZData', Ze);
                    end
                else
                    hide_transient(H(s), show_cov, ux);
                end
            else
                set(H(s).line, 'XData', [], 'YData', [], 'ZData', []);
                hide_transient(H(s), show_cov, ux);
            end
            % 归属量测（浅灰）
            if show_meas && MEAS(s).n > 0
                tm = MEAS(s).t;
                if strcmp(trail_mode, 'window'), mm = (tm <= t) & (tm > t - trail_window_s);
                else, mm = (tm <= t); end
                set(H(s).meas, 'XData', MEAS(s).p(1, mm), ...
                               'YData', MEAS(s).p(2, mm), 'ZData', MEAS(s).p(3, mm));
            end
            % 关联门（网格椭球）：当前帧若有则画，无则隐藏
            if show_gate && ~isempty(H(s).gate)
                gi = find(GATE(s).t == t, 1);
                if ~isempty(gi)
                    [Xg, Yg, Zg] = cov_ellipsoid(GATE(s).S(:, :, gi), GATE(s).pos(:, gi), ...
                                                 ux, uy, uz, gate_radius);
                    set(H(s).gate, 'XData', Xg, 'YData', Yg, 'ZData', Zg);
                else
                    set(H(s).gate, 'XData', nan(size(ux)), ...
                                   'YData', nan(size(ux)), 'ZData', nan(size(ux)));
                end
            end
        end
        % 关联伪真值（浅色）：整条静态 或 随时间揭示(同轨迹trail)
        if show_truth
            for it = 1:n_tr
                tt = TR(it).t;
                if truth_full
                    mt = true(size(tt));
                elseif strcmp(trail_mode, 'window')
                    mt = (tt <= t) & (tt > t - trail_window_s);
                else
                    mt = (tt <= t);
                end
                set(HT(it), 'XData', TR(it).p(1, mt), ...
                            'YData', TR(it).p(2, mt), 'ZData', TR(it).p(3, mt));
            end
        end
        % 二次平滑线（虚线）：随时间揭示(同轨迹trail)
        if show_smooth
            for s = 1:n_sel
                if SM(s).n == 0 || ~isgraphics(HS(s)), continue; end
                ts = SM(s).t;
                if strcmp(trail_mode, 'window'), ms = (ts <= t) & (ts > t - trail_window_s);
                else, ms = (ts <= t); end
                set(HS(s), 'XData', SM(s).p(1, ms), ...
                           'YData', SM(s).p(2, ms), 'ZData', SM(s).p(3, ms));
            end
        end
        title(ax, sprintf('%s滤波回放   t = %.2f s    (帧 %d/%d)', title_prefix, t, k, K));
    end

% =======================================================================
% 自动播放（阻塞循环，可录像）
% =======================================================================
    function run_play()
        do_gif = false; vw = [];
        if ~isempty(save_path)
            [~, ~, ext] = fileparts(save_path);
            if strcmpi(ext, '.gif')
                do_gif = true;
            else
                try, vw = VideoWriter(save_path, 'MPEG-4');
                catch, vw = VideoWriter(save_path, 'Motion JPEG AVI'); end
                vw.FrameRate = fps; open(vw);
            end
        end
        for ki = 1:nks
            draw_frame(ks(ki)); drawnow;
            if ~isempty(save_path)
                fr = getframe(fig);
                if do_gif
                    [A, map] = rgb2ind(frame2im(fr), 256);
                    if ki == 1
                        imwrite(A, map, save_path, 'gif', 'LoopCount', inf, 'DelayTime', 1/fps);
                    else
                        imwrite(A, map, save_path, 'gif', 'WriteMode', 'append', 'DelayTime', 1/fps);
                    end
                else
                    writeVideo(vw, fr);
                end
            elseif show
                pause(pause_s);
            end
        end
        if ~isempty(vw), close(vw); end
        if ~isempty(save_path), fprintf('回放已保存: %s\n', save_path); end
    end

% =======================================================================
% 交互模式（定时器驱动；滑条/按钮可控起点、后退、暂停）
% =======================================================================
    function run_slider()
        set(ax, 'Units', 'normalized', 'Position', [0.09, 0.17, 0.74, 0.76]);
        cur = 1; draw_frame(ks(cur));
        try, enableDefaultInteractivity(ax); catch, end

        % 时间滑条（按 ks 下标）
        sld = uicontrol(fig, 'Style', 'slider', 'Units', 'normalized', ...
            'Position', [0.09, 0.055, 0.55, 0.035], ...
            'Min', 1, 'Max', max(nks, 1.0001), 'Value', 1, ...
            'SliderStep', [1/max(nks-1, 1), max(1, round(nks/20))/max(nks-1, 1)]);
        % 拖动连续刷新 + 点击/松开也刷新（双保险，兼容不同版本）
        try
            addlistener(sld, 'ContinuousValueChange', @(src, ~) scrub(get(src, 'Value')));
        catch
        end
        set(sld, 'Callback', @(src, ~) scrub(get(src, 'Value')));

        bw = 0.058; y = 0.05; h = 0.05; x0 = 0.655;
        uicontrol(fig, 'Style', 'pushbutton', 'String', '|◀ 起点', 'Units', 'normalized', ...
            'Position', [x0,            y, bw+0.012, h], 'Callback', @(~,~) goto(1));
        uicontrol(fig, 'Style', 'pushbutton', 'String', '◀', 'Units', 'normalized', ...
            'Position', [x0+bw+0.02,    y, 0.035,    h], 'Callback', @(~,~) step_rel(-1));
        btn_play = uicontrol(fig, 'Style', 'pushbutton', 'String', '播放', 'Units', 'normalized', ...
            'Position', [x0+bw+0.06,    y, bw,       h], 'Callback', @(b,~) toggle_play(b));
        uicontrol(fig, 'Style', 'pushbutton', 'String', '▶', 'Units', 'normalized', ...
            'Position', [x0+2*bw+0.07,  y, 0.035,    h], 'Callback', @(~,~) step_rel(1));
        uicontrol(fig, 'Style', 'pushbutton', 'String', '▶| 末尾', 'Units', 'normalized', ...
            'Position', [x0+2*bw+0.11,  y, bw+0.012, h], 'Callback', @(~,~) goto(nks));

        % 定时器（播放用，不阻塞主线程）
        tmr = timer('ExecutionMode', 'fixedRate', 'BusyMode', 'drop', ...
                    'Period', round(max(pause_s, 0.03) * 1000) / 1000, ...
                    'TimerFcn', @(~,~) on_tick());
        set(fig, 'CloseRequestFcn', @(~,~) cleanup());

        fprintf(['交互模式：拖动滑条到任意时刻(可后退)，或用 起点/◀/播放/▶/末尾 按钮控制；' ...
                 '任意时刻都能旋转/缩放。\n']);
    end

    function scrub(v)
        cur = min(max(round(v), 1), nks);
        if ~isempty(sld) && ishghandle(sld), set(sld, 'Value', cur); end
        draw_frame(ks(cur));
    end

    function goto(idx)
        stop_timer();
        scrub(idx);
    end

    function step_rel(d)
        goto(cur + d);
    end

    function toggle_play(btn)
        if ~isempty(tmr) && isvalid(tmr) && strcmp(tmr.Running, 'on')
            stop_timer();
        else
            if cur >= nks, scrub(1); end   % 已在末尾则从头播
            if ~isempty(btn) && ishghandle(btn), set(btn, 'String', '暂停'); end
            try, start(tmr); catch, end
        end
    end

    function on_tick()
        if ~ishghandle(fig), return; end
        if cur >= nks
            stop_timer();
            return;
        end
        cur = cur + 1;
        if ~isempty(sld) && ishghandle(sld), set(sld, 'Value', cur); end
        draw_frame(ks(cur));
        drawnow limitrate;
    end

    function stop_timer()
        if ~isempty(tmr) && isvalid(tmr) && strcmp(tmr.Running, 'on')
            stop(tmr);
        end
        if ~isempty(btn_play) && ishghandle(btn_play), set(btn_play, 'String', '播放'); end
    end

    function cleanup()
        try, if ~isempty(tmr) && isvalid(tmr), stop(tmr); delete(tmr); end, catch, end
        delete(fig);
    end

end

%% ═══════════════════════════════════════════════════════════════════════════
function [X, Y, Z] = cov_ellipsoid(C, center, ux, uy, uz, ksig)
C = 0.5 * (C + C');
[V, D] = eig(C);
d = max(diag(D), 0);
A = V * diag(ksig * sqrt(d));
P = A * [ux(:)'; uy(:)'; uz(:)'];
X = reshape(P(1, :), size(ux)) + center(1);
Y = reshape(P(2, :), size(uy)) + center(2);
Z = reshape(P(3, :), size(uz)) + center(3);
end

%% ═══════════════════════════════════════════════════════════════════════════
function hide_transient(h, show_cov, ux)
set(h.head, 'XData', nan, 'YData', nan, 'ZData', nan);
set(h.txt, 'Visible', 'off');
if show_cov && ~isempty(h.cov)
    set(h.cov, 'XData', nan(size(ux)), 'YData', nan(size(ux)), 'ZData', nan(size(ux)));
end
end

%% ═══════════════════════════════════════════════════════════════════════════
function plot_id = local_map_assoc_to_plot_id(k, pos, raw_id, fused_xyz, fused_ids, truth_ids_full)
plot_id = nan;
if k > numel(fused_xyz) || k > numel(fused_ids) || k > numel(truth_ids_full)
    return;
end
Z = fused_xyz{k};
raw = fused_ids{k};
sid = truth_ids_full{k};
if isempty(Z) || isempty(raw) || isempty(sid) || size(Z, 1) < 3
    return;
end
raw = raw(:).';
sid = sid(:).';
n = min([size(Z, 2), numel(raw), numel(sid)]);
idx = find(raw(1:n) == raw_id & isfinite(sid(1:n)));
if isempty(idx)
    return;
end
d = sqrt(sum((Z(1:3, idx) - pos(:)).^2, 1));
[~, im] = min(d);
plot_id = sid(idx(im));
end

function name = local_truth_display_name(key, split_info, use_split)
if use_split && isstruct(split_info) && isfield(split_info, 'key_raw_id') && ...
        key >= 1 && key <= numel(split_info.key_raw_id) && isfinite(split_info.key_raw_id(key))
    name = sprintf('伪真值key=%g(原tid=%g,实例=%g)', ...
        key, split_info.key_raw_id(key), split_info.key_instance(key));
else
    name = sprintf('伪真值%g', key);
end
end

%% ═══════════════════════════════════════════════════════════════════════════
function v = get_opt(opts, f, d)
if isfield(opts, f) && ~isempty(opts.(f)), v = opts.(f); else, v = d; end
end

function m = get_smt_method(smt)
m = 'smooth';
if isstruct(smt) && isfield(smt, 'method') && ~isempty(smt.method), m = smt.method; end
if isstruct(smt) && isfield(smt, 'params')
    pr = smt.params;
    online = ~isfield(pr, 'online') || pr.online ~= 0;
    if ~online
        m = '离线RTS';
    elseif isfield(pr, 'lag') && ~isempty(pr.lag)
        if pr.lag == 0, m = '在线,零滞后'; else, m = sprintf('在线,滞后%g帧', pr.lag); end
    end
end
end

%% ── 自带众数（避免被项目里同名 mode.m 顶替导致的报错）─────────────────────
function m = local_mode(v)
v = v(:);
v = v(~isnan(v));
if isempty(v), m = NaN; return; end
u = unique(v);
c = zeros(numel(u), 1);
for i = 1:numel(u), c(i) = sum(v == u(i)); end
[~, k] = max(c);
m = u(k);
end
