function plot_joint_tracking_results(est, events, cfg, frames)
%PLOT_JOINT_TRACKING_RESULTS Plot unified angle and spatial track outputs.

if nargin < 4, frames = []; end

[ids, hist] = collect_output_history(est);
min_life = get_cfg(cfg, 'joint_plot_min_life', 3);
colors = lines(max(numel(ids), 1));
has2 = arrayfun(@(h) count_mode_points(h, 2) >= min_life, hist);
has3 = arrayfun(@(h) count_mode_points(h, 3) >= min_life, hist);

% Figure 1: formal 2-D output only. Formal 3-D samples are masked out.
if any(has2)
    figure('Name', '二维角度滤波航迹总体图', 'Position', [80, 100, 920, 720]);
    hold on; grid on; box on;
    handles = gobjects(1, 0); labels = cell(1, 0);
    idx2 = find(has2);
    for q = 1:numel(idx2)
        i = idx2(q);
        [az, el] = angle_mode_series(hist(i), 2);
        h = plot(az, el, '.-', 'Color', colors(i, :), 'LineWidth', 1.0, 'MarkerSize', 7);
        handles(end + 1) = h; %#ok<AGROW>
        labels{end + 1} = sprintf('Track %d', ids(i)); %#ok<AGROW>
        annotate_track_start(gca, [az; el], ids(i), colors(i, :));
    end
    xlabel('方位角 (deg)'); ylabel('俯仰角 (deg)');
    title(sprintf('二维角度滤波航迹总体图（至少%d个二维输出点，共%d条）', ...
        min_life, numel(idx2)));
    if ~isempty(handles), legend(handles, labels, 'Location', 'bestoutside'); end
    hold off;
end

% Figure 2: filter-event AE observations, split by physical measurement type.
[act_rae_ang, act_ae_only, pas_ang] = collect_raw_angles(events);
figure('Name', '二维角度量测', 'Position', [120, 130, 920, 720]);
hold on; grid on; box on;
if ~isempty(act_rae_ang)
    plot(act_rae_ang(1, :), act_rae_ang(2, :), '.', 'Color', [0.15 0.45 0.80], ...
        'MarkerSize', 7, 'DisplayName', '主动RAE中的角度');
end
if ~isempty(act_ae_only)
    plot(act_ae_only(1, :), act_ae_only(2, :), 'x', 'Color', [0.20 0.65 0.35], ...
        'MarkerSize', 6, 'DisplayName', '主动AE-only');
end
if ~isempty(pas_ang)
    plot(pas_ang(1, :), pas_ang(2, :), '.', 'Color', [0.85 0.35 0.15], ...
        'MarkerSize', 7, 'DisplayName', '物理被动AE');
end
xlabel('方位角 (deg)'); ylabel('俯仰角 (deg)');
title(sprintf(['二维角度量测（主动RAE角度%d点，主动AE-only%d点，' ...
    '物理被动AE%d点）'], size(act_rae_ang, 2), size(act_ae_only, 2), ...
    size(pas_ang, 2)));
if ~isempty(act_rae_ang) || ~isempty(act_ae_only) || ~isempty(pas_ang)
    legend('Location', 'best');
end
hold off;

% Additional diagnostic: active AE before condensation versus original passive AE.
if ~isempty(frames) && isfield(frames, 'active_ang_precondense')
    [act_pre, pas_pre] = collect_precondense_angles(frames);
    figure('Name', '凝聚前主动与被动AE量测', 'Position', [140, 145, 920, 720]);
    hold on; grid on; box on;
    if ~isempty(act_pre)
        plot(act_pre(1, :), act_pre(2, :), '.', 'Color', [0.15 0.45 0.80], ...
            'MarkerSize', 7, 'DisplayName', '主动AE（凝聚前）');
    end
    if ~isempty(pas_pre)
        plot(pas_pre(1, :), pas_pre(2, :), '.', 'Color', [0.85 0.35 0.15], ...
            'MarkerSize', 7, 'DisplayName', '被动AE（原始）');
    end
    xlabel('方位角 (deg)'); ylabel('俯仰角 (deg)');
    title(sprintf('凝聚前主动与被动AE量测（主动%d点，被动%d点）', ...
        size(act_pre, 2), size(pas_pre, 2)));
    if ~isempty(act_pre) || ~isempty(pas_pre), legend('Location', 'best'); end
    hold off;
end

% Figure 3: formal 3-D output only. 2-D intervals remain as line breaks.
if any(has3)
    figure('Name', '三维主动空间滤波航迹', 'Position', [160, 160, 980, 760]);
    hold on; grid on; box on;
    h3 = gobjects(1, 0); l3 = cell(1, 0);
    idx3 = find(has3);
    for q = 1:numel(idx3)
        i = idx3(q);
        pos = position_mode_series(hist(i), 3);
        h = plot3(pos(1, :), pos(2, :), pos(3, :), ...
            '.-', 'Color', colors(i, :), 'LineWidth', 1.0, 'MarkerSize', 7);
        h3(end + 1) = h; %#ok<AGROW>
        l3{end + 1} = sprintf('Track %d', ids(i)); %#ok<AGROW>
        annotate_track_start(gca, pos, ids(i), colors(i, :));
    end
    xlabel('East (m)'); ylabel('North (m)'); zlabel('Up (m)');
    title(sprintf('三维主动空间滤波航迹（至少%d个三维输出点，共%d条）', ...
        min_life, numel(idx3)));
    view(45, 30); axis equal;
    if ~isempty(h3), legend(h3, l3, 'Location', 'bestoutside'); end
    hold off;
end

figure('Name', '逻辑航迹模式统计', 'Position', [200, 190, 900, 420]);
plot(est.filter_times, est.N2, 'LineWidth', 1.3, 'DisplayName', '二维输出'); hold on;
plot(est.filter_times, est.N, 'LineWidth', 1.3, 'DisplayName', '三维输出');
plot(est.filter_times, est.N_total, 'k:', 'LineWidth', 1.1, 'DisplayName', '逻辑航迹总数');
grid on; box on; xlabel('Time (s)'); ylabel('航迹数'); title('逻辑航迹输出维度');
legend('Location', 'best'); hold off;

if isfield(cfg, 'plot_save_dir') && ~isempty(cfg.plot_save_dir)
    save_all_open_figures(cfg.plot_save_dir);
end
end

function n = count_mode_points(h, mode)
if mode == 2
    valid = h.dim == 2 & isfinite(h.az) & isfinite(h.el);
else
    valid = h.dim == 3 & all(isfinite(h.pos), 1);
end
n = sum(valid);
end

function [az, el] = angle_mode_series(h, mode)
valid = h.dim == mode & isfinite(h.az) & isfinite(h.el);
az = h.az; el = h.el;
az(~valid) = NaN; el(~valid) = NaN;
[az, el] = break_az_wrap(az, el);
end

function pos = position_mode_series(h, mode)
valid = h.dim == mode & all(isfinite(h.pos), 1);
pos = h.pos;
pos(:, ~valid) = NaN;
end

function [ids, hist] = collect_output_history(est)
ids = zeros(1, 0);
hist = repmat(struct('t', [], 'az', [], 'el', [], 'dim', [], 'pos', zeros(3, 0)), 0, 1);
for k = 1:numel(est.output)
    out = est.output{k};
    for q = 1:numel(out)
        id = out(q).id; i = find(ids == id, 1);
        if isempty(i)
            ids(end + 1) = id; %#ok<AGROW>
            hist(end + 1, 1) = struct('t', [], 'az', [], 'el', [], ...
                'dim', [], 'pos', zeros(3, 0)); %#ok<AGROW>
            i = numel(ids);
        end
        hist(i).t(end + 1) = out(q).t_sec;
        hist(i).az(end + 1) = out(q).az_deg;
        hist(i).el(end + 1) = out(q).el_deg;
        hist(i).dim(end + 1) = out(q).output_dim;
        hist(i).pos(:, end + 1) = out(q).position_enu;
    end
end
end

function [active_rae, active_ae_only, passive] = collect_raw_angles(events)
active_rae = zeros(2, 0); active_ae_only = zeros(2, 0);
passive = zeros(2, 0);
for k = 1:numel(events)
    if events(k).active.n_meas > 0
        active_rae = [active_rae, events(k).active.rae(2:3, :)]; %#ok<AGROW>
    end
    if events(k).passive.n_meas > 0
        n = events(k).passive.n_meas;
        kind = ones(1, n);
        if isfield(events(k).passive, 'kind') && ...
                numel(events(k).passive.kind) == n
            kind = reshape(events(k).passive.kind, 1, []);
        end
        active_ae_only = [active_ae_only, ...
            events(k).passive.ang(:, kind == 2)]; %#ok<AGROW>
        passive = [passive, events(k).passive.ang(:, kind ~= 2)]; %#ok<AGROW>
    end
end
end

function [active, passive] = collect_precondense_angles(frames)
na = 0; np = 0;
for k = 1:numel(frames)
    if isfield(frames(k), 'active_ang_precondense') && ...
            size(frames(k).active_ang_precondense, 1) == 2
        na = na + size(frames(k).active_ang_precondense, 2);
    end
    if isfield(frames(k), 'passive_ang') && size(frames(k).passive_ang, 1) == 2
        np = np + size(frames(k).passive_ang, 2);
    end
end

active = nan(2, na); passive = nan(2, np);
ia = 0; ip = 0;
for k = 1:numel(frames)
    if isfield(frames(k), 'active_ang_precondense') && ...
            size(frames(k).active_ang_precondense, 1) == 2
        n = size(frames(k).active_ang_precondense, 2);
        active(:, ia + (1:n)) = frames(k).active_ang_precondense;
        ia = ia + n;
    end
    if isfield(frames(k), 'passive_ang') && size(frames(k).passive_ang, 1) == 2
        n = size(frames(k).passive_ang, 2);
        passive(:, ip + (1:n)) = frames(k).passive_ang;
        ip = ip + n;
    end
end
active = active(:, all(isfinite(active), 1));
passive = passive(:, all(isfinite(passive), 1));
end

function [az, el] = break_az_wrap(az0, el0)
az = zeros(1, 0); el = zeros(1, 0);
for k = 1:numel(az0)
    if k > 1 && isfinite(az0(k - 1)) && isfinite(az0(k)) && abs(az0(k) - az0(k - 1)) > 180
        az(end + 1) = NaN; el(end + 1) = NaN; %#ok<AGROW>
    end
    az(end + 1) = az0(k); el(end + 1) = el0(k); %#ok<AGROW>
end
end

function v = get_cfg(cfg, name, fallback)
if isfield(cfg, name) && ~isempty(cfg.(name)), v = cfg.(name); else, v = fallback; end
end
