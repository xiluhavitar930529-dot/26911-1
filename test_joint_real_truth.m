function test_joint_real_truth()
%TEST_JOINT_REAL_TRUTH Real-truth and measurement-consistency separation.

fprintf('Running joint real-truth tests...\n');
truth_file = [tempname, '.csv'];
cleanup = onCleanup(@() delete_if_exists(truth_file)); %#ok<NASGU>
fid = fopen(truth_file, 'w');
assert(fid >= 0, 'Could not create temporary truth CSV.');
fprintf(fid, 'time_s,time,target_id,east_m,north_m,alt_m\n');
for k = 0:2
    if k == 0
        clock_text = '23:59:59.000';
    elseif k == 1
        clock_text = 'invalid';
    else
        clock_text = '00:00:01.000';
    end
    fprintf(fid, '%d.000,%s,7,1000,5000,100\n', k, clock_text);
end
fprintf(fid, '100.000,00:01:40.000,9,9000,5000,100\n');
fclose(fid);

cfg = config_fusion();
cfg.joint_history_level = 'output';
cfg.joint_confirm_M = 1; cfg.joint_confirm_N = 1;
cfg.joint_3d_birth_M = 1; cfg.joint_3d_birth_N = 1;
cfg.joint_merge_angle_deg = 0;
cfg.metrics_max_print = 0;
cfg.truth_file = truth_file;
cfg.truth_auto_discover = false;
cfg.truth_altitude_is_absolute = true;
cfg.truth_real_time_tolerance_s = 0.01;
platform = static_platform_at_altitude(100);
platform.t_sec = [86399; 86409];
events = repmat(active_event(), 3, 1);
for k = 1:3
    t = 86398 + k;
    events(k) = active_event(k, t, [1000; 5000; 0], 7);
end

est = run_filter_joint_2d3d(events, platform, cfg);
metrics = evaluate_joint_tracking_metrics(est, events, cfg, platform);
R = metrics.real_truth;
assert(strcmp(R.status, 'ok') && R.n_truth_targets == 1 && ...
    R.n_time_matched_outputs == 3, ...
    'External truth CSV was not loaded or time matched.');
assert(R.position.n == 3 && R.position.rmse_3d_m < 1e-6, ...
    'Absolute truth altitude was not converted to local ENU correctly.');
assert(R.n_truth_samples == 3 && R.n_truth_targets == 1, ...
    'Truth rows or targets outside the evaluated event interval entered the denominator.');
assert(isfield(metrics, 'measurement_consistency') && ...
    strcmp(metrics.measurement_consistency.status, 'ok'), ...
    'Measurement consistency was not reported separately from real accuracy.');

cfg.truth_file = [truth_file, '.missing'];
missing = evaluate_joint_tracking_metrics(est, events, cfg, platform);
assert(strcmp(missing.real_truth.status, 'unavailable') && ...
    strcmp(missing.measurement_consistency.status, 'ok'), ...
    'No-truth mode did not preserve measurement-consistency evaluation.');

cfg.truth_file = truth_file;
unmapped_est = est;
unmapped_est.assoc = cell(size(est.assoc));
unmapped = evaluate_joint_tracking_metrics(unmapped_est, events, cfg, platform);
assert(strcmp(unmapped.real_truth.status, 'unavailable') && ...
       unmapped.real_truth.n_formal_outputs == 3 && ...
       strcmp(unmapped.real_truth.reason, 'no_track_to_truth_identity_mapping'), ...
    'Formal outputs disappeared from real-truth accounting when identity mapping was unavailable.');
without_platform = evaluate_joint_real_truth(est, events, [], cfg);
assert(strcmp(without_platform.reason, 'platform_unavailable') && ...
       without_platform.n_formal_outputs == 3, ...
    'Formal outputs disappeared from accounting when platform data was unavailable.');
fprintf('Joint real-truth tests passed.\n');
end

function e = active_event(k, t, xyz, target_id)
if nargin == 0
    e = empty_event();
    return;
end
ae = [atan2d(xyz(1), xyz(2)); atan2d(xyz(3), hypot(xyz(1), xyz(2)))];
e = empty_event(); e.cycle_id = k; e.t_sec = t; e.t_start = t; e.t_end = t;
e.has_active = true;
e.active.t_sec = t; e.active.xyz = xyz; e.active.rae = [norm(xyz); ae];
e.active.R_xyz = diag([10^2, 10^2, 10^2]);
e.active.R_ae = diag([0.02^2, 0.02^2]); e.active.has_range = true;
e.active.ids = target_id; e.active.src = 1; e.active.n_meas = 1;
end

function e = empty_event()
e = struct('cycle_id', 0, 't_sec', NaN, 't_start', NaN, 't_end', NaN, ...
    'has_active', false, 'has_passive', false, ...
    'active', struct('t_sec', zeros(1, 0), 'xyz', zeros(3, 0), ...
        'rae', zeros(3, 0), 'R_xyz', zeros(3, 3, 0), ...
        'R_ae', zeros(2, 2, 0), 'has_range', false(1, 0), ...
        'ids', zeros(1, 0), 'src', zeros(1, 0), 'n_meas', 0), ...
    'passive', struct('t_sec', zeros(1, 0), 'ang', zeros(2, 0), ...
        'R_ae', zeros(2, 2, 0), 'ids', zeros(1, 0), ...
        'src', zeros(1, 0), 'n_meas', 0));
end

function platform = static_platform_at_altitude(altitude)
platform = struct('n_rows', 2, 't_sec', [0; 10], ...
    'lat_deg', [0; 0], 'lon_deg', [0; 0], ...
    'alt_m', [altitude; altitude]);
platform.interp_lat = @(t) zeros(size(t));
platform.interp_lon = @(t) zeros(size(t));
platform.interp_alt = @(t) altitude * ones(size(t));
end

function delete_if_exists(path)
if exist(path, 'file') == 2, delete(path); end
end
