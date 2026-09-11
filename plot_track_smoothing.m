function plot_track_smoothing(smt, opts)
%PLOT_TRACK_SMOOTHING  画"二次平滑"前/后对比图。
%   smt  —— smooth_filter_tracks 的输出
%   opts（全部可选）:
%     track_ids   只画这些ID；留空=全部             默认 []
%     bg_xyz      [3×M] 量测背景点(淡灰)            默认 []
%     raw_width   滤波(平滑前)线宽                  默认 1.0
%     smooth_width平滑后线宽                        默认 2.2
%     show_axes   额外画单轴(E/N/U)随时间对比       默认 false
%     axes_id     单轴图针对的航迹ID;留空=取位移最大的一条
%     save_dir    保存目录;留空=不保存              默认 ''
%
%   例：plot_track_smoothing(smt);
%       o.bg_xyz = all_meas_xyz; o.show_axes = true; plot_track_smoothing(smt, o);

if nargin < 2 || isempty(opts), opts = struct(); end
gp = @(f, d) pts_get(opts, f, d);
track_ids   = gp('track_ids', []);
bg_xyz      = gp('bg_xyz', []);
raw_width   = gp('raw_width', 1.0);
smooth_width= gp('smooth_width', 2.2);
show_axes   = gp('show_axes', false) ~= 0;
axes_id     = gp('axes_id', []);
save_dir    = gp('save_dir', '');

if isempty(smt) || ~isfield(smt, 'tracks') || isempty(smt.tracks)
    warning('smt 为空，无可绘制的平滑航迹。'); return;
end
TR = smt.tracks;
if ~isempty(track_ids)
    keep = ismember([TR.id], track_ids(:).');
    TR = TR(keep);
end
if isempty(TR), warning('过滤后无航迹可画。'); return; end
T = numel(TR);

% 各航迹配色（亮色），平滑前用其淡化版
base = lines(max(T, 1));
hsvc = rgb2hsv(base(1:T, :));
hsvc(:, 2) = max(hsvc(:, 2), 0.75);
hsvc(:, 3) = max(hsvc(:, 3), 0.85);
colS = hsv2rgb(hsvc);                 % 平滑后(鲜亮)
colR = 0.45 * colS + 0.55 * [1 1 1];  % 平滑前(淡)

method_txt = smooth_descr(smt);

%% ── 图1：3D 平滑前/后 ─────────────────────────────────────────────────
f1 = figure('Name', '二次平滑前/后 3D', 'Color', 'w', 'Position', [90, 90, 1000, 800]);
ax1 = axes('Parent', f1); hold(ax1, 'on'); grid(ax1, 'on'); box(ax1, 'on');
if ~isempty(bg_xyz)
    plot3(ax1, bg_xyz(1, :), bg_xyz(2, :), bg_xyz(3, :), '.', ...
        'Color', [0.85 0.85 0.85], 'MarkerSize', 3);
end
hR = []; hS = [];
for i = 1:T
    s = TR(i);
    hR(end+1) = plot3(ax1, s.raw(1, :), s.raw(2, :), s.raw(3, :), '-', ...
        'Color', colR(i, :), 'LineWidth', raw_width); %#ok<AGROW>
    hS(end+1) = plot3(ax1, s.smooth(1, :), s.smooth(2, :), s.smooth(3, :), '-', ...
        'Color', colS(i, :), 'LineWidth', smooth_width); %#ok<AGROW>
    plot3(ax1, s.smooth(1, 1), s.smooth(2, 1), s.smooth(3, 1), 'o', ...
        'MarkerEdgeColor', 'k', 'MarkerFaceColor', colS(i, :), 'MarkerSize', 5);
    annotate_track_start(ax1, s.smooth, s.id, colS(i, :));
end
xlabel(ax1, 'East (m)'); ylabel(ax1, 'North (m)'); zlabel(ax1, 'Up (m)');
view(ax1, 45, 30); axis(ax1, 'equal');
legend(ax1, [hR(1), hS(1)], {'滤波(平滑前)', sprintf('平滑后(%s)', method_txt)}, ...
    'Location', 'bestoutside');
title(ax1, sprintf('二次平滑前/后 3D 对比 (共%d条, %s)', T, method_txt));
hold(ax1, 'off');

%% ── 图2：俯视图 平滑前/后 ─────────────────────────────────────────────
f2 = figure('Name', '二次平滑前/后 俯视图', 'Color', 'w', 'Position', [140, 140, 900, 740]);
ax2 = axes('Parent', f2); hold(ax2, 'on'); grid(ax2, 'on'); box(ax2, 'on');
if ~isempty(bg_xyz)
    plot(ax2, bg_xyz(1, :), bg_xyz(2, :), '.', 'Color', [0.85 0.85 0.85], 'MarkerSize', 3);
end
hR2 = []; hS2 = [];
for i = 1:T
    s = TR(i);
    hR2(end+1) = plot(ax2, s.raw(1, :), s.raw(2, :), '-', ...
        'Color', colR(i, :), 'LineWidth', raw_width); %#ok<AGROW>
    hS2(end+1) = plot(ax2, s.smooth(1, :), s.smooth(2, :), '-', ...
        'Color', colS(i, :), 'LineWidth', smooth_width); %#ok<AGROW>
    plot(ax2, s.smooth(1, 1), s.smooth(2, 1), 'o', ...
        'MarkerEdgeColor', 'k', 'MarkerFaceColor', colS(i, :), 'MarkerSize', 5);
    annotate_track_start(ax2, s.smooth(1:2, :), s.id, colS(i, :));
end
xlabel(ax2, 'East (m)'); ylabel(ax2, 'North (m)'); axis(ax2, 'equal');
legend(ax2, [hR2(1), hS2(1)], {'滤波(平滑前)', sprintf('平滑后(%s)', method_txt)}, ...
    'Location', 'bestoutside');
title(ax2, sprintf('二次平滑前/后 俯视图 (共%d条, %s)', T, method_txt));
hold(ax2, 'off');

%% ── 图3（可选）：单条航迹 E/N/U 随时间，平滑前/后 ─────────────────────
if show_axes
    if isempty(axes_id)
        [~, im] = max([TR.rms_shift]); s = TR(im);
    else
        j = find([TR.id] == axes_id, 1);
        if isempty(j), j = 1; end
        s = TR(j);
    end
    f3 = figure('Name', '二次平滑前/后 单轴时序', 'Color', 'w', 'Position', [190, 190, 950, 720]);
    names = {'East', 'North', 'Up'};
    for ax = 1:3
        subplot(3, 1, ax); hold on; grid on;
        plot(s.t, s.raw(ax, :), '-', 'Color', [0.55 0.55 0.60], 'LineWidth', 1.0);
        plot(s.t, s.smooth(ax, :), '-', 'Color', [0.20 0.45 0.80], 'LineWidth', 1.8);
        annotate_track_start(gca, [s.t; s.smooth(ax, :)], s.id, [0.20 0.45 0.80]);
        ylabel(sprintf('%s (m)', names{ax}));
        if ax == 1
            legend({'滤波(平滑前)', '平滑后'}, 'Location', 'best');
            title(sprintf('Track %g 单轴时序 平滑前/后 (%s, 位移RMS=%.2fm)', ...
                s.id, method_txt, s.rms_shift));
        end
        if ax == 3, xlabel('Time (s)'); end
        hold off;
    end
end

%% ── 保存 ──────────────────────────────────────────────────────────────
if ~isempty(save_dir)
    if exist(save_dir, 'dir') ~= 7, mkdir(save_dir); end
    saveas(f1, fullfile(save_dir, 'smoothing_3d.png'));
    saveas(f2, fullfile(save_dir, 'smoothing_topview.png'));
    if show_axes && exist('f3', 'var'), saveas(f3, fullfile(save_dir, 'smoothing_axes.png')); end
    fprintf('平滑对比图已保存到: %s\n', save_dir);
end
end

%% ═══════════════════════════════════════════════════════════════════════════
function v = pts_get(opts, f, d)
if isfield(opts, f) && ~isempty(opts.(f)), v = opts.(f); else, v = d; end
end

function d = smooth_descr(smt)
m = 'smooth';
if isfield(smt, 'method') && ~isempty(smt.method), m = smt.method; end
lag = []; online = true;
if isfield(smt, 'params')
    if isfield(smt.params, 'lag'), lag = smt.params.lag; end
    if isfield(smt.params, 'online'), online = smt.params.online ~= 0; end
end
if ~online
    d = sprintf('离线RTS');
elseif strcmpi(m, 'movmean')
    d = '在线/尾部均值';
elseif ~isempty(lag) && lag == 0
    d = '在线/零滞后';
elseif ~isempty(lag)
    d = sprintf('在线/滞后%g帧', lag);
else
    d = upper(m);
end
end
