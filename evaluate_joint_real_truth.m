function result = evaluate_joint_real_truth(est, events, platform, cfg, labels, output_pairs)
%EVALUATE_JOINT_REAL_TRUTH Evaluate formal outputs against an external truth CSV.
% Real truth is evaluation-only. Track-to-target identity is inherited from
% measurement-label association results and never feeds the filter.

result = result_template();
out = collect_formal_outputs(est);
result.n_formal_outputs = numel(out.t_sec);
if nargin < 3 || isempty(platform) || ~isstruct(platform)
    result.reason = 'platform_unavailable';
    return;
end
if nargin < 4 || isempty(cfg), cfg = struct(); end
if nargin < 5 || isempty(labels), labels = struct(); end
if nargin < 6, output_pairs = struct('track_id', {}, 'truth_id', {}); end

[truth_file, reason] = resolve_truth_file(cfg);
if isempty(truth_file)
    result.reason = reason;
    return;
end
result.source_file = truth_file;

try
    truth = load_truth_csv(truth_file, platform, cfg);
catch ME
    result.status = 'invalid';
    result.reason = ME.message;
    return;
end
result.time_reference = truth.time_reference;
if isempty(truth.t_sec)
    result.status = 'invalid';
    result.reason = 'truth_file_has_no_valid_rows';
    return;
end

[eval_start, eval_end] = evaluation_interval(events, out);
truth_in_interval = true(size(truth.t_sec));
if isfinite(eval_start) && isfinite(eval_end)
    truth_in_interval = truth.t_sec >= eval_start & truth.t_sec <= eval_end;
end
evaluation_truth_ids = unique(truth.id(truth_in_interval & isfinite(truth.id)));
result.n_truth_samples = nnz(truth_in_interval);
result.n_truth_targets = numel(evaluation_truth_ids);
result.truth_target_ids = evaluation_truth_ids;
if isempty(out.t_sec)
    result.status = 'ok';
    result.reason = 'no_formal_outputs';
    result.target_coverage_rate = safe_ratio(0, result.n_truth_targets);
    return;
end

[pair_track, pair_raw] = output_pair_raw_ids(output_pairs, labels, cfg);
valid_pair = isfinite(pair_track) & isfinite(pair_raw);
pair_track = pair_track(valid_pair); pair_raw = pair_raw(valid_pair);
if isempty(pair_track)
    result.status = 'unavailable';
    result.reason = 'no_track_to_truth_identity_mapping';
    return;
end
[mapped, location] = ismember(out.track_id, pair_track);
out.raw_id = nan(size(out.track_id));
out.raw_id(mapped) = pair_raw(location(mapped));
result.n_identity_mapped_outputs = nnz(mapped);
result.mapping_rate = safe_ratio(result.n_identity_mapped_outputs, result.n_formal_outputs);

n = numel(out.t_sec);
truth_pos = nan(3, n);
time_matched = false(1, n);
tolerance = get_cfg(cfg, 'truth_real_time_tolerance_s', 0.03);
for q = 1:numel(evaluation_truth_ids)
    raw_id = evaluation_truth_ids(q);
    output_indices = find(out.raw_id == raw_id & isfinite(out.t_sec));
    if isempty(output_indices), continue; end
    truth_q = find(truth.target_ids == raw_id, 1);
    truth_indices = truth.target_index{truth_q};
    tt = truth.t_sec(truth_indices);
    pp = truth.position_enu(:, truth_indices);
    [tt, unique_indices] = unique(tt, 'stable');
    pp = pp(:, unique_indices);
    query = out.t_sec(output_indices);
    if numel(tt) == 1
        nearest_t = tt(1) * ones(size(query));
        interpolated = repmat(pp(:, 1), 1, numel(query));
    else
        nearest_t = interp1(tt, tt, query, 'nearest', NaN);
        interpolated = interp1(tt, pp.', query, 'linear', NaN).';
    end
    good = isfinite(nearest_t) & abs(query - nearest_t) <= tolerance & ...
        all(isfinite(interpolated), 1);
    target = output_indices(good);
    truth_pos(:, target) = interpolated(:, good);
    time_matched(target) = true;
end

result.n_time_matched_outputs = nnz(time_matched);
result.time_match_rate = safe_ratio(result.n_time_matched_outputs, ...
    result.n_identity_mapped_outputs);
matched_targets = unique(out.raw_id(time_matched & isfinite(out.raw_id)));
result.n_output_truth_targets = numel(matched_targets);
result.target_coverage_rate = safe_ratio(result.n_output_truth_targets, ...
    result.n_truth_targets);

sensor = platform_positions_enu(out.t_sec, platform, cfg);
truth_angle = relative_to_ae(truth_pos, sensor);
los_error = los_separation_deg(out.az_deg, out.el_deg, truth_angle(1, :), truth_angle(2, :));
angle_valid = time_matched & all(isfinite(truth_angle), 1) & ...
    isfinite(out.az_deg) & isfinite(out.el_deg);
position_valid = time_matched & out.output_dim == 3 & ...
    all(isfinite(out.position_enu), 1);
two_d = angle_valid & out.output_dim == 2;
three_d_angle = angle_valid & out.output_dim == 3;

result.angle = angle_metrics( ...
    angle_difference(out.az_deg(angle_valid), truth_angle(1, angle_valid)), ...
    out.el_deg(angle_valid) - truth_angle(2, angle_valid), los_error(angle_valid));
result.position = position_metrics( ...
    out.position_enu(:, position_valid) - truth_pos(:, position_valid));
result.two_d = struct('n_outputs', nnz(out.output_dim == 2), ...
    'n_matched', nnz(two_d), ...
    'angle', angle_metrics( ...
        angle_difference(out.az_deg(two_d), truth_angle(1, two_d)), ...
        out.el_deg(two_d) - truth_angle(2, two_d), los_error(two_d)));
result.three_d = struct('n_outputs', nnz(out.output_dim == 3), ...
    'n_matched', nnz(position_valid), ...
    'angle', angle_metrics( ...
        angle_difference(out.az_deg(three_d_angle), truth_angle(1, three_d_angle)), ...
        out.el_deg(three_d_angle) - truth_angle(2, three_d_angle), los_error(three_d_angle)), ...
    'position', result.position);
result.status = 'ok';
result.reason = '';
end

function result = result_template()
angle = angle_metrics([], [], []);
position = position_metrics(zeros(3, 0));
result = struct('status', 'unavailable', 'reason', 'truth_not_configured', ...
    'source_file', '', 'coordinate_contract', 'ENU: east_m,north_m,alt_m-origin_alt', ...
    'mapping_source', 'measurement_label_association_only', ...
    'time_reference', struct('origin_s', NaN, 'source', 'absolute_time', 'n_relative_rows', 0), ...
    'n_truth_samples', 0, 'n_truth_targets', 0, ...
    'truth_target_ids', zeros(1, 0), 'n_formal_outputs', 0, ...
    'n_identity_mapped_outputs', 0, 'n_time_matched_outputs', 0, ...
    'n_output_truth_targets', 0, 'mapping_rate', NaN, ...
    'time_match_rate', NaN, 'target_coverage_rate', NaN, ...
    'angle', angle, 'position', position, ...
    'two_d', struct('n_outputs', 0, 'n_matched', 0, 'angle', angle), ...
    'three_d', struct('n_outputs', 0, 'n_matched', 0, ...
        'angle', angle, 'position', position));
end

function [path, reason] = resolve_truth_file(cfg)
path = ''; reason = 'truth_file_not_found';
configured = strtrim(char(get_cfg(cfg, 'truth_file', '')));
data_dir = char(get_cfg(cfg, 'data_dir', pwd));
if ~isempty(configured)
    candidates = {configured, fullfile(data_dir, configured), fullfile(pwd, configured)};
    for q = 1:numel(candidates)
        candidate = candidates{q};
        if exist(candidate, 'file') == 2
            path = canonical_path(candidate);
            return;
        end
    end
    reason = 'configured_truth_file_not_found';
end
if ~logical(get_cfg(cfg, 'truth_auto_discover', true)), return; end

directories = unique({data_dir, fileparts(data_dir)}, 'stable');
found = cell(1, 0);
for q = 1:numel(directories)
    if isempty(directories{q}) || exist(directories{q}, 'dir') ~= 7, continue; end
    entries = dir(fullfile(directories{q}, 'truth*.csv'));
    for j = 1:numel(entries)
        found{end + 1} = fullfile(entries(j).folder, entries(j).name); %#ok<AGROW>
    end
end
found = unique(found, 'stable');
if isempty(found), return; end
preferred = find(strcmpi(cellfun(@file_name, found, 'UniformOutput', false), ...
    'truth_100_targets_500k.csv'));
if numel(preferred) == 1
    path = canonical_path(found{preferred});
elseif numel(found) == 1
    path = canonical_path(found{1});
else
    reason = 'multiple_truth_files_found';
end
end

function name = file_name(path)
[~, base, ext] = fileparts(path); name = [base, ext];
end

function path = canonical_path(path)
f = java.io.File(path);
path = char(f.getCanonicalPath());
end

function truth = load_truth_csv(path, platform, cfg)
opts = detectImportOptions(path, 'Delimiter', ',');
required = {'target_id', 'east_m', 'north_m', 'alt_m'};
missing = setdiff(required, opts.VariableNames);
if ~isempty(missing)
    error('真实真值缺少字段: %s', strjoin(missing, ', '));
end
selected = required;
if ismember('time', opts.VariableNames)
    selected = [{'time'}, selected];
end
if ismember('time_s', opts.VariableNames)
    selected = [{'time_s'}, selected];
end
opts.SelectedVariableNames = selected;
T = readtable(path, opts);

ids = numeric_column(T.target_id);
east = numeric_column(T.east_m);
north = numeric_column(T.north_m);
altitude = numeric_column(T.alt_m);
absolute_time = nan(height(T), 1);
if ismember('time', T.Properties.VariableNames)
    absolute_time = clock_seconds(T.time);
end
time_reference = struct('origin_s', NaN, 'source', 'absolute_time', 'n_relative_rows', 0);
if ismember('time_s', T.Properties.VariableNames)
    relative_time = numeric_column(T.time_s);
    use_relative = ~isfinite(absolute_time) & isfinite(relative_time);
    if any(use_relative)
        [origin, source] = relative_time_origin(absolute_time, relative_time, platform, cfg);
        absolute_time(use_relative) = relative_time(use_relative) + origin;
        time_reference = struct('origin_s', origin, 'source', source, ...
            'n_relative_rows', nnz(use_relative));
    end
end

origin_alt = local_origin_altitude(platform, cfg);
if logical(get_cfg(cfg, 'truth_altitude_is_absolute', true))
    up = altitude - origin_alt;
else
    up = altitude;
end
valid = isfinite(absolute_time) & isfinite(ids) & ...
    isfinite(east) & isfinite(north) & isfinite(up);
absolute_time = absolute_time(valid); ids = ids(valid);
position = [east(valid).'; north(valid).'; up(valid).'];
[~, order] = sortrows([ids, absolute_time], [1, 2]);
ids = ids(order); absolute_time = absolute_time(order); position = position(:, order);

target_ids = unique(ids).';
target_index = cell(1, numel(target_ids));
for q = 1:numel(target_ids)
    target_index{q} = find(ids == target_ids(q));
end
truth = struct('t_sec', absolute_time.', 'id', ids.', ...
    'position_enu', position, 'target_ids', target_ids, ...
    'target_index', {target_index}, 'time_reference', time_reference);
end

function values = numeric_column(values)
if isnumeric(values)
    values = double(values(:));
elseif iscell(values)
    values = cellfun(@str2double, values(:));
elseif isstring(values) || iscategorical(values)
    values = str2double(string(values(:)));
else
    values = double(values(:));
end
end

function seconds_value = clock_seconds(values)
if isduration(values)
    seconds_value = seconds(values(:));
    seconds_value = unwrap_fusion_clock_times(seconds_value);
    return;
elseif isdatetime(values)
    seconds_value = hour(values(:)) * 3600 + minute(values(:)) * 60 + second(values(:));
    seconds_value = unwrap_fusion_clock_times(seconds_value);
    return;
elseif isnumeric(values)
    seconds_value = double(values(:));
    return;
end
text = cellstr(string(values(:)));
[unique_text, ~, group] = unique(text);
parsed = nan(numel(unique_text), 1);
for q = 1:numel(unique_text)
    parsed(q) = parse_fusion_time(unique_text{q}, 'hms');
end
seconds_value = parsed(group);
seconds_value = unwrap_fusion_clock_times(seconds_value);
end

function [origin, source] = relative_time_origin(absolute_time, relative_time, platform, cfg)
origin = get_cfg(cfg, 'truth_time_origin_s', []);
if ~isempty(origin)
    if ~isnumeric(origin) || ~isreal(origin) || ~isscalar(origin) || ~isfinite(origin)
        error('evaluate_joint_real_truth:InvalidTimeOrigin', ...
            'cfg.truth_time_origin_s must be a finite scalar in event-clock seconds.');
    end
    source = 'configured';
    return;
end
anchors = isfinite(absolute_time) & isfinite(relative_time);
if any(anchors)
    origin = median(absolute_time(anchors) - relative_time(anchors));
    source = 'truth_absolute_relative_pairs';
    return;
end
% The main pipeline loads the complete platform file, independent of the
% radar selection window. Its start is the default acquisition-time origin.
if isfield(platform, 't_sec') && any(isfinite(platform.t_sec))
    origin = min(platform.t_sec(isfinite(platform.t_sec)));
    source = 'platform_start';
    return;
end
error('evaluate_joint_real_truth:MissingTimeOrigin', ...
    'Relative truth time requires cfg.truth_time_origin_s or a complete platform timeline.');
end

function altitude = local_origin_altitude(platform, cfg)
origin = get_cfg(cfg, 'local_origin', 'first_platform');
if isnumeric(origin) && numel(origin) == 3
    altitude = origin(3);
else
    altitude = platform.alt_m(1);
end
end

function [track_ids, raw_ids] = output_pair_raw_ids(pairs, labels, cfg)
track_ids = zeros(1, 0); raw_ids = zeros(1, 0);
if isempty(pairs) || ~isfield(pairs, 'track_id') || ~isfield(pairs, 'truth_id')
    return;
end
track_ids = reshape([pairs.track_id], 1, []);
instance_ids = reshape([pairs.truth_id], 1, []);
raw_ids = nan(size(instance_ids));
if ~isfield(labels, 'summary')
    raw_ids = instance_ids;
    return;
end
summary = labels.summary;
if ~isfield(summary, 'instance_labels') || ~isfield(summary, 'key_raw_id')
    raw_ids = instance_ids;
    return;
end
[found, location] = ismember(instance_ids, summary.instance_labels);
if ~logical(get_cfg(cfg, 'truth_cross_sensor_id_consistent', false)) && ...
        isfield(summary, 'passive_only_instance_labels')
    found = found & ~ismember(instance_ids, summary.passive_only_instance_labels);
end
raw_ids(found) = summary.key_raw_id(location(found));
end

function [t0, t1] = evaluation_interval(events, out)
times = zeros(1, 0);
if ~isempty(events) && isstruct(events)
    if isfield(events, 't_start'), times = [times, [events.t_start]]; end %#ok<AGROW>
    if isfield(events, 't_end'), times = [times, [events.t_end]]; end %#ok<AGROW>
    if isempty(times) && isfield(events, 't_sec'), times = [events.t_sec]; end
end
if isempty(times), times = out.t_sec; end
times = times(isfinite(times));
if isempty(times)
    t0 = NaN; t1 = NaN;
else
    t0 = min(times); t1 = max(times);
end
end

function out = collect_formal_outputs(est)
out = struct('track_id', zeros(1, 0), 'raw_id', zeros(1, 0), ...
    't_sec', zeros(1, 0), 'output_dim', zeros(1, 0), ...
    'az_deg', zeros(1, 0), 'el_deg', zeros(1, 0), ...
    'position_enu', zeros(3, 0));
if ~isstruct(est) || ~isfield(est, 'output'), return; end
counts = cellfun(@numel, est.output);
n = sum(counts); if n == 0, return; end
out.track_id = nan(1, n); out.raw_id = nan(1, n); out.t_sec = nan(1, n);
out.output_dim = zeros(1, n); out.az_deg = nan(1, n); out.el_deg = nan(1, n);
out.position_enu = nan(3, n); p = 0;
for k = 1:numel(est.output)
    values = est.output{k}; m = numel(values); if m == 0, continue; end
    ii = p + (1:m);
    out.track_id(ii) = reshape([values.id], 1, []);
    out.t_sec(ii) = reshape([values.t_sec], 1, []);
    out.output_dim(ii) = reshape([values.output_dim], 1, []);
    out.az_deg(ii) = reshape([values.az_deg], 1, []);
    out.el_deg(ii) = reshape([values.el_deg], 1, []);
    out.position_enu(:, ii) = cat(2, values.position_enu);
    p = p + m;
end
end

function sensor = platform_positions_enu(t, platform, cfg)
sensor = nan(3, numel(t));
if isempty(t), return; end
origin = get_cfg(cfg, 'local_origin', 'first_platform');
if isnumeric(origin) && numel(origin) == 3
    lat0 = origin(1); lon0 = origin(2); alt0 = origin(3);
else
    lat0 = platform.lat_deg(1); lon0 = platform.lon_deg(1); alt0 = platform.alt_m(1);
end
anchor = llh_to_ecef(lat0, lon0, alt0); rotation = ecef_to_enu_rot(lat0, lon0);
[unique_t, ~, group] = unique(t);
unique_sensor = nan(3, numel(unique_t));
for q = 1:numel(unique_t)
    lat = platform.interp_lat(unique_t(q));
    lon = platform.interp_lon(unique_t(q));
    alt = platform.interp_alt(unique_t(q));
    unique_sensor(:, q) = rotation * (llh_to_ecef(lat, lon, alt) - anchor);
end
sensor = unique_sensor(:, group);
end

function ecef = llh_to_ecef(lat_deg, lon_deg, alt_m)
a = 6378137; e2 = 6.69437999014e-3;
lat = deg2rad(lat_deg); lon = deg2rad(lon_deg);
N = a / sqrt(1 - e2 * sin(lat)^2);
ecef = [(N + alt_m) * cos(lat) * cos(lon); ...
    (N + alt_m) * cos(lat) * sin(lon); ...
    (N * (1 - e2) + alt_m) * sin(lat)];
end

function R = ecef_to_enu_rot(lat_deg, lon_deg)
lat = deg2rad(lat_deg); lon = deg2rad(lon_deg);
R = [-sin(lon), cos(lon), 0; ...
    -sin(lat) * cos(lon), -sin(lat) * sin(lon), cos(lat); ...
    cos(lat) * cos(lon), cos(lat) * sin(lon), sin(lat)];
end

function ae = relative_to_ae(position, sensor)
d = position - sensor; horizontal = hypot(d(1, :), d(2, :));
ae = [atan2d(d(1, :), d(2, :)); atan2d(d(3, :), horizontal)];
end

function metrics = angle_metrics(az_error, el_error, los_error)
metrics = struct('n', numel(az_error), 'rmse_az_deg', NaN, ...
    'rmse_el_deg', NaN, 'rmse_los_deg', NaN);
if isempty(az_error), return; end
metrics.rmse_az_deg = sqrt(mean(az_error.^2));
metrics.rmse_el_deg = sqrt(mean(el_error.^2));
metrics.rmse_los_deg = sqrt(mean(los_error.^2));
end

function metrics = position_metrics(error)
metrics = struct('n', size(error, 2), 'rmse_e_m', NaN, ...
    'rmse_n_m', NaN, 'rmse_u_m', NaN, 'rmse_3d_m', NaN);
if isempty(error), return; end
metrics.rmse_e_m = sqrt(mean(error(1, :).^2));
metrics.rmse_n_m = sqrt(mean(error(2, :).^2));
metrics.rmse_u_m = sqrt(mean(error(3, :).^2));
metrics.rmse_3d_m = sqrt(mean(sum(error.^2, 1)));
end

function difference = angle_difference(a, b)
difference = mod(a - b + 180, 360) - 180;
end

function value = safe_ratio(numerator, denominator)
if denominator > 0, value = numerator / denominator; else, value = NaN; end
end

function value = get_cfg(cfg, name, fallback)
if isstruct(cfg) && isfield(cfg, name) && ~isempty(cfg.(name))
    value = cfg.(name);
else
    value = fallback;
end
end
