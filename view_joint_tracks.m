function info = view_joint_tracks(est, events, opts)
%VIEW_JOINT_TRACKS Inspect selected logical tracks in angle or ENU space.

if nargin < 3, opts = struct(); end
[ids, H] = collect_history(est);
requested = get_opt(opts, 'track_ids', []);
if isempty(requested), selected = ids; else, selected = intersect(requested(:).', ids, 'stable'); end
if isempty(selected)
    error('view_joint_tracks:NoTrack', '所选航迹不存在。可用ID: %s', mat2str(ids));
end
show_ref = get_opt(opts, 'show_reference', true);
show_angle = get_opt(opts, 'show_angle', true);
show_enu = get_opt(opts, 'show_enu', true);
ref_data = prepare_reference_data(est, events, H, ids, selected, opts, show_ref);
colors = lines(numel(selected));
selected_has2 = false(1, numel(selected));
selected_has3 = false(1, numel(selected));
for s = 1:numel(selected)
    h = H(ids == selected(s));
    selected_has2(s) = any(angle_mode_mask(h));
    selected_has3(s) = any(position_mode_mask(h));
end

fprintf('联合逻辑航迹可用ID: %s\n', mat2str(ids));
fprintf('当前查看ID: %s\n', mat2str(selected));

if show_angle && any(selected_has2)
    figure('Name', '选中二维纯角度航迹 AE', 'Position', [80, 90, 900, 700]);
    hold on; grid on; box on;
    if show_ref, plot_full_angle_truth(ref_data, 1.4); end
    for s = 1:numel(selected)
        if ~selected_has2(s), continue; end
        h = H(ids == selected(s));
        [az, el] = angle_mode_series(h);
        plot(az, el, '.-', 'Color', colors(s, :), 'LineWidth', 1.2, ...
            'DisplayName', sprintf('Track %d', selected(s)));
        annotate_track_start(gca, [az; el], selected(s), colors(s, :));
    end
    xlabel('方位角 (deg)'); ylabel('俯仰角 (deg)'); title('选中二维纯角度航迹');
    legend('Location', 'bestoutside'); hold off;

    figure('Name', '二维角度参考对比', 'Position', [120, 120, 980, 720]);
    for ax = 1:2
        subplot(2, 1, ax); hold on; grid on; box on;
        if show_ref, plot_full_angle_truth_axis(ref_data, ax); end
        for s = 1:numel(selected)
            if ~selected_has2(s), continue; end
            h = H(ids == selected(s));
            valid = angle_mode_mask(h);
            val = h.az; if ax == 2, val = h.el; end
            val(~valid) = NaN;
            plot_val = unwrap_for_plot(val);
            plot(h.t, plot_val, '-', 'Color', colors(s, :), 'LineWidth', 1.3, ...
                'DisplayName', sprintf('Track %d 滤波', selected(s)));
            annotate_track_start(gca, [h.t; plot_val], selected(s), colors(s, :));
        end
        ylabel(axis_name(ax)); if ax == 1, title('二维纯角度滤波与完整真值'); end
        if ax == 2, xlabel('Time (s)'); end
        legend('Location', 'bestoutside'); hold off;
    end

    if show_ref
        figure('Name', '二维角度误差', 'Position', [150, 145, 980, 720]);
        for ax = 1:2
            subplot(2, 1, ax); hold on; grid on; box on;
            for s = 1:numel(selected)
                if ~selected_has2(s), continue; end
                h = H(ids == selected(s));
                valid = angle_mode_mask(h);
                ref = ref_data(ids == selected(s)).angle;
                ref(:, ~valid) = NaN;
                err = nan(1, numel(h.t));
                if ax == 1
                    err(valid) = angle_diff(h.az(valid), ref(1, valid));
                else
                    err(valid) = h.el(valid) - ref(2, valid);
                end
                plot(h.t, err, '-', 'Color', colors(s, :), 'LineWidth', 1.2, ...
                    'DisplayName', sprintf('Track %d', selected(s)));
                annotate_track_start(gca, [h.t; err], selected(s), colors(s, :));
            end
            yline(0, 'k:'); ylabel([axis_name(ax), '误差']);
            if ax == 1, title('二维角度误差'); end
            if ax == 2, xlabel('Time (s)'); end
            legend('Location', 'bestoutside'); hold off;
        end
    end
end

if show_enu && any(selected_has3)
    figure('Name', '选中三维主动航迹 ENU', 'Position', [180, 170, 940, 720]);
    hold on; grid on; box on;
    if show_ref, plot_full_position_truth(ref_data, 1.4); end
    for s = 1:numel(selected)
        if ~selected_has3(s), continue; end
        h = H(ids == selected(s));
        pos = position_mode_series(h);
        plot3(pos(1, :), pos(2, :), pos(3, :), '.-', ...
            'Color', colors(s, :), 'LineWidth', 1.2, ...
            'DisplayName', sprintf('Track %d', selected(s)));
        annotate_track_start(gca, pos, selected(s), colors(s, :));
    end
    xlabel('East (m)'); ylabel('North (m)'); zlabel('Up (m)');
    title('选中三维主动空间航迹'); view(45, 30); axis equal;
    legend('Location', 'bestoutside'); hold off;

    plot_enu_comparison(ids, H, selected, colors, show_ref, ref_data);
end

selected_ref = ref_data(ismember(ids, selected));
info = struct('available_ids', ids, 'selected_ids', selected, 'history', H, ...
    'angle_ids', selected(selected_has2), 'enu_ids', selected(selected_has3), ...
    'reference', selected_ref, 'truth_summary', reference_summary(selected_ref));
end

function plot_enu_comparison(ids, H, selected, colors, show_ref, ref_data)
names = {'East (m)', 'North (m)', 'Up (m)'};
figure('Name', 'ENU参考对比', 'Position', [210, 190, 1020, 800]);
for ax = 1:3
    subplot(3, 1, ax); hold on; grid on; box on;
    if show_ref, plot_full_position_truth_axis(ref_data, ax); end
    for s = 1:numel(selected)
        h = H(ids == selected(s)); valid = position_mode_mask(h);
        if ~any(valid), continue; end
        val = h.pos(ax, :); val(~valid) = NaN;
        plot(h.t, val, '-', 'Color', colors(s, :), 'LineWidth', 1.2, ...
            'DisplayName', sprintf('Track %d 滤波', selected(s)));
        annotate_track_start(gca, [h.t; val], selected(s), colors(s, :));
    end
    ylabel(names{ax}); if ax == 1, title('三维ENU滤波与完整真值'); end
    if ax == 3, xlabel('Time (s)'); end
    legend('Location', 'bestoutside'); hold off;
end
if ~show_ref, return; end

figure('Name', 'ENU误差', 'Position', [240, 215, 1020, 800]);
for ax = 1:3
    subplot(3, 1, ax); hold on; grid on; box on;
    for s = 1:numel(selected)
        h = H(ids == selected(s)); valid = position_mode_mask(h);
        if ~any(valid), continue; end
        ref = ref_data(ids == selected(s)).position;
        ref(:, ~valid) = NaN;
        err = nan(1, numel(h.t));
        err(valid) = h.pos(ax, valid) - ref(ax, valid);
        plot(h.t, err, '-', 'Color', colors(s, :), ...
            'LineWidth', 1.2, 'DisplayName', sprintf('Track %d', selected(s)));
        annotate_track_start(gca, [h.t; err], selected(s), colors(s, :));
    end
    yline(0, 'k:'); ylabel([names{ax}, '误差']);
    if ax == 1, title('三维ENU误差'); end
    if ax == 3, xlabel('Time (s)'); end
    legend('Location', 'bestoutside'); hold off;
end
end

function valid = angle_mode_mask(h)
valid = h.dim == 2 & isfinite(h.az) & isfinite(h.el);
end

function [az, el] = angle_mode_series(h)
valid = angle_mode_mask(h);
az = h.az; el = h.el;
az(~valid) = NaN; el(~valid) = NaN;
[az, el] = break_wrap(az, el);
end

function valid = position_mode_mask(h)
valid = h.dim == 3 & all(isfinite(h.pos), 1);
end

function pos = position_mode_series(h)
valid = position_mode_mask(h);
pos = h.pos;
pos(:, ~valid) = NaN;
end

function [ids, H] = collect_history(est)
ids = zeros(1, 0);
H = repmat(struct('t', [], 'az', [], 'el', [], 'dim', [], ...
    'pos', zeros(3, 0), 'event_index', [], 'truth_id', []), 0, 1);
for k = 1:numel(est.output)
    out = est.output{k};
    for q = 1:numel(out)
        i = find(ids == out(q).id, 1);
        if isempty(i)
            ids(end + 1) = out(q).id; %#ok<AGROW>
            H(end + 1, 1) = struct('t', [], 'az', [], 'el', [], 'dim', [], ...
                'pos', zeros(3, 0), 'event_index', [], 'truth_id', []); %#ok<AGROW>
            i = numel(ids);
        end
        H(i).t(end + 1) = out(q).t_sec; H(i).az(end + 1) = out(q).az_deg;
        H(i).el(end + 1) = out(q).el_deg; H(i).dim(end + 1) = out(q).output_dim;
        H(i).pos(:, end + 1) = out(q).position_enu; H(i).event_index(end + 1) = k;
        H(i).truth_id(end + 1) = out(q).truth_id;
    end
end
end

function data = prepare_reference_data(est, events, H, ids, selected, opts, enabled)
template = struct('track_id', 0, 'truth_key', NaN, 'truth_label', '', ...
    'angle', zeros(2, 0), 'position', zeros(3, 0), ...
    'n_angle', 0, 'n_position', 0, ...
    'truth_tracks', repmat(empty_truth_track(), 1, 0));
data = repmat(template, numel(ids), 1);
for i = 1:numel(ids)
    data(i).track_id = ids(i);
    data(i).angle = nan(2, numel(H(i).t));
    data(i).position = nan(3, numel(H(i).t));
end
if ~enabled || isempty(events) || isempty(ids), return; end

truth_opts = struct('events', events, 'track_ids', selected, 'no_figure', true, ...
    'cfg', get_opt(opts, 'cfg', struct()), ...
    'tid_select_mode', get_opt(opts, 'truth_select_mode', 'dominant'), ...
    'truth_all', get_opt(opts, 'truth_all', false));
if isfield(opts, 'truth_use_split') && ~isempty(opts.truth_use_split)
    truth_opts.truth_use_split = opts.truth_use_split;
end
truth_info = plot_pseudo_truth(est, reshape([events.t_sec], [], 1), truth_opts);
truth_tracks = truth_info.TRk;

for i = 1:numel(ids)
    if ~ismember(ids(i), selected), continue; end
    summary = truth_info.track_tid_summary;
    q = find([summary.track_id] == ids(i), 1);
    if isempty(q), key = NaN; else, key = summary(q).main_tid; end
    data(i).truth_key = key;
    data(i).truth_tracks = truth_tracks;
    slot = find([truth_tracks.tid] == key, 1);
    if ~isempty(slot)
        data(i).truth_label = truth_tracks(slot).label;
        data(i).angle = interpolate_angle_series( ...
            truth_tracks(slot).angle_t, truth_tracks(slot).angle, H(i).t);
        data(i).position = interpolate_linear_series( ...
            truth_tracks(slot).t, truth_tracks(slot).p, H(i).t);
    end
    data(i).n_angle = nnz(all(isfinite(data(i).angle), 1));
    data(i).n_position = nnz(all(isfinite(data(i).position), 1));
    if data(i).n_angle == 0 && data(i).n_position == 0
        fprintf(2, '[view_track] Track %d 未找到可用真值参考，误差曲线不可计算。\n', ids(i));
    end
end
end

function ref = interpolate_angle_series(t, z, query)
ref = nan(2, numel(query));
if isempty(t), return; end
if numel(t) == 1
    ref(:, abs(query - t) <= time_tolerance(t)) = z(:, 1);
    return;
end
az = unwrap(deg2rad(z(1, :)));
ref(1, :) = rad2deg(interp1(t, az, query, 'linear', NaN));
ref(1, :) = mod(ref(1, :) + 180, 360) - 180;
ref(2, :) = interp1(t, z(2, :), query, 'linear', NaN);
end

function ref = interpolate_linear_series(t, z, query)
ref = nan(size(z, 1), numel(query));
if isempty(t), return; end
if numel(t) == 1
    ref(:, abs(query - t) <= time_tolerance(t)) = z(:, 1);
    return;
end
for row = 1:size(z, 1)
    ref(row, :) = interp1(t, z(row, :), query, 'linear', NaN);
end
end

function plot_full_angle_truth(data, line_width)
tracks = selected_truth_tracks(data);
colors = lines(max(numel(tracks), 1));
for i = 1:numel(tracks)
    if isempty(tracks(i).angle_t), continue; end
    [az, el] = break_wrap(tracks(i).angle(1, :), tracks(i).angle(2, :));
    plot(az, el, 'o-', 'Color', colors(i, :), 'LineWidth', line_width, ...
        'MarkerSize', 3.5, ...
        'DisplayName', sprintf('真值 %s（完整）', tracks(i).label));
    annotate_truth_start(gca, [az; el], tracks(i).label, colors(i, :));
end
end

function plot_full_angle_truth_axis(data, axis_index)
tracks = selected_truth_tracks(data);
colors = lines(max(numel(tracks), 1));
for i = 1:numel(tracks)
    if isempty(tracks(i).angle_t), continue; end
    value = tracks(i).angle(axis_index, :);
    if axis_index == 1, value = unwrap_for_plot(value); end
    plot(tracks(i).angle_t, value, 'o-', 'Color', colors(i, :), ...
        'LineWidth', 1.2, 'MarkerSize', 3.5, ...
        'DisplayName', sprintf('真值 %s（完整）', tracks(i).label));
end
end

function plot_full_position_truth(data, line_width)
tracks = selected_truth_tracks(data);
colors = lines(max(numel(tracks), 1));
for i = 1:numel(tracks)
    if isempty(tracks(i).t), continue; end
    plot3(tracks(i).p(1, :), tracks(i).p(2, :), tracks(i).p(3, :), 'o-', ...
        'Color', colors(i, :), 'LineWidth', line_width, 'MarkerSize', 3.5, ...
        'DisplayName', sprintf('真值 %s（完整）', tracks(i).label));
    annotate_truth_start(gca, tracks(i).p, tracks(i).label, colors(i, :));
end
end

function plot_full_position_truth_axis(data, axis_index)
tracks = selected_truth_tracks(data);
colors = lines(max(numel(tracks), 1));
for i = 1:numel(tracks)
    if isempty(tracks(i).t), continue; end
    plot(tracks(i).t, tracks(i).p(axis_index, :), 'o-', ...
        'Color', colors(i, :), 'LineWidth', 1.2, 'MarkerSize', 3.5, ...
        'DisplayName', sprintf('真值 %s（完整）', tracks(i).label));
end
end

function tracks = selected_truth_tracks(data)
tracks = repmat(empty_truth_track(), 1, 0);
for i = 1:numel(data)
    if ~isempty(data(i).truth_tracks)
        tracks = data(i).truth_tracks;
        return;
    end
end
end

function annotate_truth_start(ax, values, label, color)
if isempty(values), return; end
first = find(all(isfinite(values), 1), 1);
if isempty(first), return; end
p = values(:, first);
if size(values, 1) >= 3
    text(ax, p(1), p(2), p(3), sprintf(' 真值 %s', label), ...
        'Color', color, 'FontSize', 8, 'VerticalAlignment', 'bottom', ...
        'HandleVisibility', 'off');
else
    text(ax, p(1), p(2), sprintf(' 真值 %s', label), ...
        'Color', color, 'FontSize', 8, 'VerticalAlignment', 'bottom', ...
        'HandleVisibility', 'off');
end
end

function track = empty_truth_track()
track = struct('tid', NaN, 'label', '', 't', zeros(1, 0), ...
    'p', zeros(3, 0), 'angle_t', zeros(1, 0), 'angle', zeros(2, 0));
end

function tol = time_tolerance(t)
tol = max(1e-9, 32 * eps(max(1, max(abs(t)))));
end

function summary = reference_summary(data)
summary = struct('n_tracks', numel(data), ...
    'n_mapped_tracks', nnz(isfinite([data.truth_key])), ...
    'n_tracks_with_angle', nnz([data.n_angle] > 0), ...
    'n_tracks_with_position', nnz([data.n_position] > 0));
end

function y = unwrap_for_plot(x)
x = x(:).';
y = nan(size(x));
good = isfinite(x);
edges = diff([false, good, false]);
starts = find(edges == 1);
stops = find(edges == -1) - 1;
for q = 1:numel(starts)
    j = starts(q):stops(q);
    y(j) = rad2deg(unwrap(deg2rad(x(j))));
end
end

function [az, el] = break_wrap(az0, el0)
az = []; el = [];
for k = 1:numel(az0)
    if k > 1 && abs(az0(k) - az0(k - 1)) > 180
        az(end + 1) = NaN; el(end + 1) = NaN; %#ok<AGROW>
    end
    az(end + 1) = az0(k); el(end + 1) = el0(k); %#ok<AGROW>
end
end

function d = angle_diff(a, b)
d = mod(a - b + 180, 360) - 180;
end

function s = axis_name(ax)
if ax == 1, s = '方位角 (deg)'; else, s = '俯仰角 (deg)'; end
end

function v = get_opt(opts, name, fallback)
if isfield(opts, name) && ~isempty(opts.(name)), v = opts.(name); else, v = fallback; end
end
