function results = test_joint_metric_contract(report_dir)
%TEST_JOINT_METRIC_CONTRACT Scope, LOS, time origin and export regressions.
root = fileparts(mfilename('fullpath'));
addpath(root);
if nargin < 1, report_dir = tempname; end
folder = report_dir;
if ~exist(folder, 'dir'), mkdir(folder); end
cfg = config_fusion();
cfg.metrics_max_print = 0;
cfg.track_accuracy_min_assoc = 1;
cfg.track_accuracy_purity_th = 0.9;
cfg.truth_id_split_enabled = true;
cfg.truth_auto_discover = false;
cfg.truth_file = '';
platform = static_platform();

% Same physical inputs; only one passive assignment changes destination.
[events, est] = fixture(0:9, repmat([1000; 50000; 1000], 1, 10));
for k = 1:10
    ae = events(k).active.rae(2:3, 1);
    events(k).passive = struct('n_meas', 1, 't_sec', k-1, ...
        'ids', 801, 'ang', ae);
    est.assoc{k} = struct('id', [1, 2], 'type', {{'active', 'passive'}}, ...
        'meas_index', [1, 1], 'tid', [601, 801], ...
        'measurement_dim', [3, 2], 'filter_dim', [3, 2], ...
        'range_updated', [true, false]);
    extra = est.output{k}; extra.id = 2; extra.truth_id = 801;
    extra.output_dim = 2; extra.position_enu(:) = NaN;
    est.output{k}(2) = extra;
end
before = evaluate_joint_tracking_metrics(est, events, cfg);
est.assoc{1}.id(2) = 1;
est.assoc{1}.filter_dim(2) = 3;
after = evaluate_joint_tracking_metrics(est, events, cfg);
b = before.three_d.accuracy.output_pairs(1);
a = after.three_d.accuracy.output_pairs(1);
d = after.track_details([after.track_details.track_id] == 1).three_d;
results.cross_dimension = struct('unified_truth_instances', after.truth_targets.instance_count, ...
    'before_numerator', b.n_correct, 'before_score', b.coverage, ...
    'after_numerator', a.n_correct, 'after_aggregate_score', a.coverage, ...
    'after_detail_numerator', d.n_correct, 'after_detail_denominator', d.truth_total, ...
    'after_detail_score', d.coverage, 'after_detail_consistency', d.association_consistency, ...
    'aggregate_correct', after.three_d.track_accuracy.n_correct_tracks, ...
    'detail_correct', d.is_correct);
assert(after.truth_targets.instance_count == 1);
assert(b.coverage == 1 && a.coverage == 1 && d.coverage == 1);
assert(a.n_correct == 10 && d.truth_total == 10 && d.n_labeled_assoc == 10);
assert(after.three_d.track_accuracy.n_correct_tracks == 1 && d.is_correct);
distribution = after.three_d.track_accuracy.coverage_distribution;
assert(distribution.n_tracks == 1 && distribution.mean == 1 && ...
    distribution.median == 1 && distribution.n_ge_90 == 1 && ...
    distribution.n_ge_95 == 1 && distribution.n_ge_98 == 1 && ...
    distribution.n_ge_99 == 1);
assert(before.two_d.reference.n_passive_to_2d == 10 && after.two_d.reference.n_passive_to_2d == 9);
assert(after.two_d.track_accuracy.purity == 1);
assert(after.two_d.output_coverage.n_reference_measurements == 9);
assert(after.three_d.output_coverage.n_reference_measurements == 10);
assert_details_agree(before); assert_details_agree(after);
for k = 1:10
    est.assoc{k}.id(2) = 1;
    est.assoc{k}.filter_dim(2) = 3;
end
all_fused = evaluate_joint_tracking_metrics(est, events, cfg);
d = all_fused.track_details([all_fused.track_details.track_id] == 1).three_d;
results.cross_dimension.all_fused_aggregate_score = all_fused.three_d.track_accuracy.purity;
results.cross_dimension.all_fused_detail_score = d.coverage;
assert(d.coverage == 1 && d.association_consistency == 1);
assert(all_fused.two_d.reference.n_passive_to_2d == 0);
assert(all_fused.two_d.track_accuracy.n_correct_tracks == 0);
assert_details_agree(all_fused);

% True LOS separation at high elevation is not hypot(dAz,dEl).
xyz = direction(0, 80) * 50000;
[events, est] = fixture(0, xyz);
est.output{1}.az_deg = 1;
est.output{1}.position_enu = direction(1, 80) * 50000;
cfg.truth_file = fullfile(folder, 'los_truth.csv');
write_truth(cfg.truth_file, 0, xyz, false);
los = evaluate_joint_tracking_metrics(est, events, cfg, platform);
u = direction(0, 80); v = direction(1, 80);
exact = atan2d(norm(cross(u, v)), dot(u, v));
results.los = struct('proxy_report_deg', los.angle.rmse_los_deg, ...
    'real_truth_report_deg', los.real_truth.angle.rmse_los_deg, ...
    'geometric_los_deg', exact);
assert(abs(los.angle.rmse_los_deg - exact) < 1e-10);
assert(abs(los.real_truth.angle.rmse_los_deg - exact) < 1e-10);
assert(abs(los.track_details.angle_rmse.los_deg - exact) < 1e-10);
assert(los.angle.rmse_az_deg == 1 && los.angle.rmse_el_deg == 0);
assert(abs(los_separation_deg(179, 0, -179, 0) - 2) < 1e-12);
assert(los_separation_deg(90, 90, 0, 90) < 1e-12);
assert(los_separation_deg(180, 0, 0, 0) == 180);
assert(abs(exact - 0.1736460401) < 1e-8);

% First selected measurement is t=10; truth time_s is relative to run t=0.
tt = 0:20; pp = [100*tt; 50000*ones(size(tt)); zeros(size(tt))];
[events, est] = fixture(10:20, pp(:, 11:21));
cfg.truth_file = fullfile(folder, 'absolute_truth.csv');
write_truth(cfg.truth_file, tt, pp, false);
absolute = evaluate_joint_tracking_metrics(est, events, cfg, platform);
cfg.truth_file = fullfile(folder, 'relative_truth.csv');
write_truth(cfg.truth_file, tt, pp, true);
relative = evaluate_joint_tracking_metrics(est, events, cfg, platform);
results.time_origin = struct('absolute_time_rmse_m', absolute.real_truth.position.rmse_3d_m, ...
    'relative_time_rmse_m', relative.real_truth.position.rmse_3d_m, ...
    'relative_time_matched_outputs', relative.real_truth.n_time_matched_outputs);
assert(absolute.real_truth.position.rmse_3d_m == 0);
assert(relative.real_truth.position.rmse_3d_m == 0);
assert(strcmp(relative.real_truth.time_reference.source, 'platform_start'));
shifted_platform = platform; shifted_platform.t_sec = [10; 100];
cfg.truth_time_origin_s = 0;
explicit = evaluate_joint_tracking_metrics(est, events, cfg, shifted_platform);
assert(explicit.real_truth.position.rmse_3d_m == 0);
assert(strcmp(explicit.real_truth.time_reference.source, 'configured'));
cfg.truth_time_origin_s = NaN;
invalid = evaluate_joint_tracking_metrics(est, events, cfg, platform);
assert(strcmp(invalid.real_truth.status, 'invalid'));
cfg.truth_time_origin_s = [];

% Target presence coverage and full measurement coverage are different.
tt = 0:9; pp = repmat([1000; 50000; 1000], 1, 10);
[events, est] = fixture(tt, pp);
cfg.truth_file = fullfile(folder, 'fragment_truth.csv');
write_truth(cfg.truth_file, tt, pp, false);
fragment = est; fragment.assoc(4:10) = {[]}; fragment.output(4:10) = {[]};
m = evaluate_joint_tracking_metrics(fragment, events, cfg, platform);
results.fragment = struct('association_rate', m.association.rate_all_tracks, ...
    'association_consistency', m.accuracy.accuracy, ...
    'track_score', m.track_accuracy.purity, ...
    'track_correct', m.track_accuracy.n_correct_tracks, ...
    'formal_event_coverage', m.overall.output_coverage.rate, ...
    'target_presence_coverage', m.real_truth.target_coverage_rate, ...
    'real_position_rmse_m', m.real_truth.position.rmse_3d_m);
assert(m.track_accuracy.purity == 0.3 && m.track_accuracy.n_correct_tracks == 0);
distribution = m.track_accuracy.coverage_distribution;
assert(distribution.n_tracks == 1 && distribution.mean == 0.3 && ...
    distribution.median == 0.3 && distribution.n_ge_90 == 0 && ...
    distribution.n_ge_95 == 0 && distribution.n_ge_98 == 0 && ...
    distribution.n_ge_99 == 0);
assert(m.real_truth.target_coverage_rate == 1 && m.overall.output_coverage.rate == 0.3);
short_output = est; short_output.output(1:9) = {[]};
m = evaluate_joint_tracking_metrics(short_output, events, cfg, platform);
results.full_association_short_output = struct('track_score', m.track_accuracy.purity, ...
    'track_correct', m.track_accuracy.n_correct_tracks, ...
    'formal_event_coverage', m.overall.output_coverage.rate);
assert(m.track_accuracy.purity == 1 && m.overall.output_coverage.rate == 0.1);
empty_output = est; empty_output.output(:) = {[]};
m = evaluate_joint_tracking_metrics(empty_output, events, cfg, platform);
results.no_outputs = struct('truth_targets', m.real_truth.n_truth_targets, ...
    'target_coverage_is_nan', isnan(m.real_truth.target_coverage_rate), ...
    'formal_event_coverage', m.overall.output_coverage.rate);
assert(m.real_truth.n_truth_targets == 1 && m.real_truth.target_coverage_rate == 0);

% Zero measurement residual does not prove zero independent truth error.
noisy = pp; noisy(1, :) = noisy(1, :) + 50;
[events, est] = fixture(tt, noisy);
m = evaluate_joint_tracking_metrics(est, events, cfg, platform);
results.proxy_vs_truth = struct('proxy_rmse_m', m.position.rmse_3d_m, ...
    'real_truth_rmse_m', m.real_truth.position.rmse_3d_m, ...
    'per_track_report_rmse_m', m.track_details.position_rmse.three_d_m);
assert(m.position.rmse_3d_m == 0 && m.real_truth.position.rmse_3d_m == 50);

% Interpolation exists, but the nearest-time gate excludes midpoints.
[events, est] = fixture(0:0.5:9, repmat(pp(:, 1), 1, 19));
m = evaluate_joint_tracking_metrics(est, events, cfg, platform);
results.time_gate = struct('outputs', m.real_truth.n_formal_outputs, ...
    'time_matched', m.real_truth.n_time_matched_outputs, ...
    'time_match_rate', m.real_truth.time_match_rate);
assert(m.real_truth.n_time_matched_outputs == 10);

results.passive_reference = test_passive_reference(cfg);
results.export = test_export_reference(cfg, folder);
results.note = 'Contract regressions; filter behavior is not modified by evaluation.';
fid = fopen(fullfile(folder, 'results.json'), 'w');
assert(fid >= 0); cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fwrite(fid, jsonencode(results), 'char');
disp(jsonencode(results));
fprintf('Joint metric contract tests passed. Reports: %s\n', folder);
end

function [events, est] = fixture(times, positions)
e = struct('t_sec', NaN, 't_start', NaN, 't_end', NaN, ...
    'active', struct('n_meas', 0, 't_sec', [], 'ids', [], ...
        'has_range', false(1, 0), 'rae', zeros(3, 0), 'xyz', zeros(3, 0)), ...
    'passive', struct('n_meas', 0, 't_sec', [], 'ids', [], 'ang', zeros(2, 0)));
n = numel(times); events = repmat(e, n, 1);
est = struct('output', {cell(n, 1)}, 'assoc', {cell(n, 1)}, 'transition_log', []);
for k = 1:n
    t = times(k); xyz = positions(:, k);
    az = atan2d(xyz(1), xyz(2)); el = atan2d(xyz(3), hypot(xyz(1), xyz(2)));
    events(k).t_sec = t; events(k).t_start = t; events(k).t_end = t;
    events(k).active = struct('n_meas', 1, 't_sec', t, 'ids', 601, ...
        'has_range', true, 'rae', [norm(xyz); az; el], 'xyz', xyz);
    est.assoc{k} = struct('id', 1, 'type', {{'active'}}, 'meas_index', 1, ...
        'tid', 601, 'measurement_dim', 3, 'filter_dim', 3, 'range_updated', true);
    est.output{k} = struct('id', 1, 'truth_id', 601, 'output_dim', 3, ...
        't_sec', t, 'az_deg', az, 'el_deg', el, 'position_enu', xyz);
end
end

function write_truth(path, time, xyz, relative)
name = 'time'; if relative, name = 'time_s'; end
T = table(time(:), repmat(601, numel(time), 1), xyz(1, :).', xyz(2, :).', xyz(3, :).', ...
    'VariableNames', {name, 'target_id', 'east_m', 'north_m', 'alt_m'});
writetable(T, path);
end

function u = direction(az, el)
u = [cosd(el)*sind(az); cosd(el)*cosd(az); sind(el)];
end

function p = static_platform()
p = struct('t_sec', [0; 100], 'lat_deg', [0; 0], ...
    'lon_deg', [0; 0], 'alt_m', [0; 0], ...
    'interp_lat', @(t) zeros(size(t)), ...
    'interp_lon', @(t) zeros(size(t)), ...
    'interp_alt', @(t) zeros(size(t)));
end

function assert_details_agree(m)
for name = {'two_d', 'three_d', 'overall'}
    s = m.(name{1});
    for q = 1:numel(s.track_accuracy.track_ids)
        id = s.track_accuracy.track_ids(q);
        d = m.track_details([m.track_details.track_id] == id).(name{1});
        assert(d.applicable && d.is_correct == s.track_accuracy.is_correct(q));
        assert(isequaln(d.coverage, s.track_accuracy.purity(q)));
        assert(d.n_labeled_assoc >= d.n_correct && d.truth_total >= d.n_correct);
        if d.has_match
            assert(d.coverage >= 0 && d.coverage <= 1);
            assert(d.association_consistency >= 0 && d.association_consistency <= 1);
        end
    end
end
end

function result = test_passive_reference(cfg)
cfg.truth_file = '';
[events, est] = fixture(0:9, repmat([1000; 50000; 1000], 1, 10));
empty_active = events(1).active;
empty_active.n_meas = 0; empty_active.t_sec = []; empty_active.ids = [];
empty_active.has_range = false(1, 0); empty_active.rae = zeros(3, 0);
empty_active.xyz = zeros(3, 0);
empty_active.R_ae = zeros(2, 2, 0);
empty_active.R_xyz = zeros(3, 3, 0);
empty_active.src = zeros(1, 0);
for k = 1:10
    events(k).passive = struct('n_meas', 1, 't_sec', k-1, 'ids', 801, ...
        'ang', events(k).active.rae(2:3, :));
    events(k).active = empty_active;
    est.assoc{k}.type = {'passive'}; est.assoc{k}.tid = 801;
    est.assoc{k}.measurement_dim = 2; est.assoc{k}.filter_dim = 2;
    est.assoc{k}.range_updated = false;
    est.output{k}.output_dim = 2; est.output{k}.position_enu(:) = NaN;
    est.measurement_disposition{k}.passive.input_dim = uint8(2);
end
est.assoc(1:5) = {[]}; est.output(1:9) = {[]};
m = evaluate_joint_tracking_metrics(est, events, cfg);
assert(m.two_d.track_accuracy.purity == 0.5);
assert(m.measurement_accounting.passive.utilization_rate == 0.5);
assert(m.two_d.output_coverage.n_reference_measurements == 10);
assert(m.two_d.output_coverage.rate == 0.1);
assert_details_agree(m);
result = struct('received_2d', 10, 'associated', 5, 'score', m.two_d.track_accuracy.purity);
for k = 1:5, est.measurement_disposition{k}.passive.input_dim = uint8(3); end
m = evaluate_joint_tracking_metrics(est, events, cfg);
assert(m.two_d.reference.n_passive_to_2d == 5 && m.two_d.reference.n_passive_to_3d == 5);
assert(m.two_d.track_accuracy.purity == 1 && m.two_d.output_coverage.rate == 0.2);

% Real filter: capacity-deleted tentative births remain in the input reference.
e = events(1); e.cycle_id = 1; e.has_active = false; e.has_passive = true;
e.passive.n_meas = 2; e.passive.t_sec = [0, 0]; e.passive.ids = [801, 802];
e.passive.ang = [10, 50; 2, 5]; e.passive.src = [1, 1];
e.passive.R_ae = repmat(0.01*eye(2), 1, 1, 2);
cfg.max_tracks = 1;
cfg.joint_confirm_M = 3; cfg.joint_confirm_N = 5;
dead = run_filter_joint_2d3d(e, static_platform(), cfg);
d = dead.measurement_disposition{1}.passive;
assert(all(d.input_dim == 2) && nnz(d.filter_dim == 0) == 1);
m = evaluate_joint_tracking_metrics(dead, e, cfg);
assert(m.two_d.reference.n_passive_to_2d == 2);
assert(m.two_d.output.n_outputs == 0 && m.two_d.output_coverage.rate == 0);
result.capacity_deleted_reference = m.two_d.reference.n_passive_to_2d;
end

function result = test_export_reference(cfg, folder)
cfg.truth_file = ''; cfg.truth_cross_sensor_id_consistent = true;
pa = [1000; 50000; 1000]; pb = [20000; 50000; 5000];
[events, est] = fixture(0:9, repmat(pa, 1, 10));
ae_b = [atan2d(pb(1), pb(2)); atan2d(pb(3), hypot(pb(1), pb(2)))];
for k = 1:10
    e = events(k);
    e.active.n_meas = 2; e.active.ids = [601, 602];
    e.active.has_range = [true, true]; e.active.t_sec = [k-1, k-1];
    e.active.xyz = [pa, pb]; e.active.rae = [e.active.rae, [norm(pb); ae_b]];
    e.passive = struct('n_meas', 1, 't_sec', k-1, 'ids', 602, 'ang', ae_b);
    events(k) = e;
    if k > 4
        est.assoc{k}.type = {'passive'}; est.assoc{k}.tid = 602;
        est.assoc{k}.measurement_dim = 2; est.assoc{k}.filter_dim = 2;
        est.assoc{k}.range_updated = false;
        est.output{k}.output_dim = 2; est.output{k}.position_enu(:) = NaN;
        est.output{k}.az_deg = ae_b(1); est.output{k}.el_deg = ae_b(2);
    end
end
m = evaluate_joint_tracking_metrics(est, events, cfg);
d = m.track_details;
assert(d.overall.matched_truth_id ~= d.three_d.matched_truth_id);
expected = norm(pa-pb);
assert(abs(d.position_rmse.three_d_m - expected) < 1e-9);
opts = struct('output_dir', folder, 'print_console', false, 'metrics', m);
report = export_track_diagnostics(est, events, cfg, 1, opts);
text = fileread(report.files{1});
assert(contains(text, sprintf('RMSE_reference\tmeasurement_event_mean')));
T = read_output_table(report.files{1}, text);
assert(all(T.position_truth == d.overall.matched_truth_id));
position_errors = T.err_3D_m(isfinite(T.err_3D_m));
assert(numel(position_errors) == d.position_rmse.n);
assert(abs(sqrt(mean(position_errors.^2)) - d.position_rmse.three_d_m) < 1e-8);
angle_errors = T.los_error_deg(isfinite(T.los_error_deg));
assert(abs(sqrt(mean(angle_errors.^2)) - d.angle_rmse.los_deg) < 1e-8);
stale = rmfield(m, 'evaluation_version'); stale.track_details.position_rmse.three_d_m = -123;
opts.metrics = stale;
report = export_track_diagnostics(est, events, cfg, 1, opts);
assert(~contains(fileread(report.files{1}), sprintf('3D_RMSE_m\t-123')));
result = struct('position_rmse_m', expected, 'sample_count', numel(position_errors), ...
    'summary_and_rows_agree', true, 'stale_metrics_recomputed', true);
end

function T = read_output_table(path, text)
lines = regexp(text, '\r?\n', 'split');
first = find(startsWith(lines, sprintf('event\ttime_s\toutput_dim')), 1);
last = first + find(cellfun(@isempty, lines(first+1:end)), 1) - 1;
opts = detectImportOptions(path, 'FileType', 'text', 'Delimiter', '\t', ...
    'NumHeaderLines', first-1);
opts.VariableNamesLine = first; opts.DataLines = [first+1, last];
T = readtable(path, opts);
end
