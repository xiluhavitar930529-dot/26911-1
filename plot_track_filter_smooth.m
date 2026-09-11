function smt = plot_track_filter_smooth(est, frame_times, opts)
%PLOT_TRACK_FILTER_SMOOTH  画指定航迹的【一次滤波轨迹 + 二次平滑轨迹 + 量测点】对比图。
%   与 play_track_filter 类似（3D，锚点ENU），但为静态对比图：
%     · 量测点：浅灰色散点
%     · 一次滤波轨迹：细实线
%     · 二次平滑轨迹：粗实线
%   以线条【粗细】区分一次滤波与二次平滑（同航迹同色）。
%
%   smt = plot_track_filter_smooth(est, frame_times, opts)
%   返回所用的平滑结果 smt（便于复用/叠加到 play_track_filter）。
%
%   opts（全部可选）:
%     track_ids     [向量] 要画的航迹ID；留空=全部确认航迹       默认 []
%     smooth        smooth_filter_tracks 的输出；留空=内部自动算  默认 []
%     smooth_method 'fixedlag'|'forward'|'movmean'|'rts'          默认 'fixedlag'
%     smooth_lag    固定滞后帧数                                  默认 8
%     smooth_meas_std / smooth_proc_std / smooth_min_life         见 smooth_filter_tracks
%     show_meas     画归属量测点                                  默认 true
%     meas_color    量测颜色                                      默认 浅灰 [0.7 0.7 0.7]
%     filter_width  一次滤波线宽(细)                              默认 1.2
%     smooth_width  二次平滑线宽(粗)                              默认 3.0
%     view_az/view_el  视角                                       默认 45/30
%     title_str     标题                                          默认 自动

if nargin < 3 || isempty(opts), opts = struct(); end
gp = @(f, d) pfs_get(opts, f, d);
pos_idx = [1, 4, 7];
K = numel(frame_times);

if K == 0 || ~isfield(est, 'tracks')
    warning('est 为空或缺少 tracks 字段，无法绘制。'); smt = []; return;
end

track_ids    = gp('track_ids', []);
show_meas    = gp('show_meas', true) ~= 0;
meas_color   = gp('meas_color', [0.70 0.70 0.70]);
filter_width = gp('filter_width', 1.2);
smooth_width = gp('smooth_width', 3.0);
view_az      = gp('view_az', 45);
view_el      = gp('view_el', 30);

%% —— 选定航迹ID ——
all_conf_ids = [];
if isfield(est, 'L')
    for k = 1:K
        if ~isempty(est.L{k}), all_conf_ids = [all_conf_ids; est.L{k}(:, 2)]; end %#ok<AGROW>
    end
end
all_conf_ids = unique(all_conf_ids(:))';
if isempty(track_ids), sel_ids = all_conf_ids; else, sel_ids = unique(track_ids(:))'; end
if isempty(sel_ids)
    warning('没有可显示的航迹ID。'); smt = []; return;
end
n_sel = numel(sel_ids);

idmap = containers.Map('KeyType', 'double', 'ValueType', 'double');
for s = 1:n_sel, idmap(sel_ids(s)) = s; end

% 各航迹配色（亮色）
base = lines(max(n_sel, 1));
hsvc = rgb2hsv(base(1:n_sel, :));
hsvc(:, 2) = max(hsvc(:, 2), 0.75);
hsvc(:, 3) = max(hsvc(:, 3), 0.80);
colors = hsv2rgb(hsvc);

%% —— 提取一次滤波轨迹与归属量测 ——
EST  = repmat(struct('p', zeros(3, K), 'n', 0), 1, n_sel);
MEAS = repmat(struct('p', zeros(3, K), 'n', 0), 1, n_sel);
for k = 1:K
    Tr = est.tracks{k};
    if ~isempty(Tr) && isfield(Tr, 'L') && ~isempty(Tr.L)
        ids = Tr.L(2, :);
        for c = 1:numel(ids)
            if isKey(idmap, ids(c))
                s = idmap(ids(c)); m = EST(s).n + 1;
                EST(s).p(:, m) = Tr.m(pos_idx, c); EST(s).n = m;
            end
        end
    end
    if show_meas && isfield(est, 'assoc')
        As = est.assoc{k};
        if ~isempty(As) && isfield(As, 'id') && ~isempty(As.id) && isfield(As, 'xyz')
            for c = 1:numel(As.id)
                if isKey(idmap, As.id(c))
                    s = idmap(As.id(c)); m = MEAS(s).n + 1;
                    MEAS(s).p(:, m) = As.xyz(:, c); MEAS(s).n = m;
                end
            end
        end
    end
end
for s = 1:n_sel
    EST(s).p  = EST(s).p(:, 1:EST(s).n);
    MEAS(s).p = MEAS(s).p(:, 1:MEAS(s).n);
end

%% —— 取/算二次平滑结果 ——
smt = gp('smooth', []);
if isempty(smt)
    so = struct('verbose', false);
    so.method   = gp('smooth_method', 'fixedlag');
    so.lag      = gp('smooth_lag', 8);
    if isfield(opts, 'smooth_meas_std'), so.meas_std = opts.smooth_meas_std; end
    if isfield(opts, 'smooth_proc_std'), so.proc_std = opts.smooth_proc_std; end
    if isfield(opts, 'smooth_min_life'), so.min_life = opts.smooth_min_life; end
    so.track_ids = sel_ids;
    if exist('smooth_filter_tracks', 'file')
        smt = smooth_filter_tracks(est, frame_times, so);
    else
        warning('未找到 smooth_filter_tracks.m，无法绘制平滑轨迹，仅画滤波与量测。');
        smt = [];
    end
end

%% —— 绘图 ——
fig = figure('Name', '一次滤波 vs 二次平滑', 'Color', 'w', 'Position', [80, 80, 1000, 800]);
ax = axes('Parent', fig); hold(ax, 'on'); grid(ax, 'on'); box(ax, 'on');

hMeas = []; hFilt = []; hSmooth = [];
for s = 1:n_sel
    col = colors(s, :);
    % 量测点（浅灰）
    if show_meas && MEAS(s).n > 0
        h = plot3(ax, MEAS(s).p(1, :), MEAS(s).p(2, :), MEAS(s).p(3, :), '.', ...
                  'Color', meas_color, 'MarkerSize', 6);
        if isempty(hMeas), hMeas = h; end
    end
    % 一次滤波轨迹（细线）
    if EST(s).n > 0
        h = plot3(ax, EST(s).p(1, :), EST(s).p(2, :), EST(s).p(3, :), '-', ...
                  'Color', col, 'LineWidth', filter_width);
        if isempty(hFilt), hFilt = h; end
        plot3(ax, EST(s).p(1, 1), EST(s).p(2, 1), EST(s).p(3, 1), 'o', ...
              'MarkerEdgeColor', 'k', 'MarkerFaceColor', col, 'MarkerSize', 5);
        annotate_track_start(ax, EST(s).p, sel_ids(s), col);
    end
    % 二次平滑轨迹（粗线，同色）
    tk = local_get_smoothed(smt, sel_ids(s));
    if ~isempty(tk) && size(tk.smooth, 2) > 0
        h = plot3(ax, tk.smooth(1, :), tk.smooth(2, :), tk.smooth(3, :), '-', ...
                  'Color', col, 'LineWidth', smooth_width);
        if isempty(hSmooth), hSmooth = h; end
        if EST(s).n == 0
            annotate_track_start(ax, tk.smooth, sel_ids(s), col);
        end
    end
end

xlabel(ax, 'East (m)'); ylabel(ax, 'North (m)'); zlabel(ax, 'Up (m)');
view(ax, view_az, view_el); axis(ax, 'equal');

% 图例（细=滤波 / 粗=平滑 / 浅灰=量测）
leg_h = []; leg_s = {};
if ~isempty(hFilt),   leg_h(end+1) = hFilt;   leg_s{end+1} = '一次滤波(细)'; end
if ~isempty(hSmooth), leg_h(end+1) = hSmooth; leg_s{end+1} = '二次平滑(粗)'; end
if ~isempty(hMeas),   leg_h(end+1) = hMeas;   leg_s{end+1} = '量测'; end
if ~isempty(leg_h), legend(ax, leg_h, leg_s, 'Location', 'bestoutside'); end

default_title = sprintf('一次滤波 vs 二次平滑 (航迹: %s)', mat2str(sel_ids));
title(ax, gp('title_str', default_title));
hold(ax, 'off');
end

%% ═══════════════════════════════════════════════════════════════════════════
function tk = local_get_smoothed(smt, id)
tk = [];
if isempty(smt) || ~isfield(smt, 'tracks') || isempty(smt.tracks), return; end
ids = [smt.tracks.id];
j = find(ids == id, 1);
if ~isempty(j), tk = smt.tracks(j); end
end

%% ═══════════════════════════════════════════════════════════════════════════
function v = pfs_get(opts, f, d)
if isfield(opts, f) && ~isempty(opts.(f)), v = opts.(f); else, v = d; end
end
