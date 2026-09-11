function results = test_joint_legacy_backbone_regression()
%TEST_JOINT_LEGACY_BACKBONE_REGRESSION Branch isolation and switch contracts.
root = fileparts(mfilename('fullpath'));
addpath(root);
platform = static_platform();
cfg = test_config();

results = struct();
results.pure_active = test_pure_active(cfg, platform);
results.pure_passive = test_pure_passive(cfg, platform);
results.switch_transaction = test_switch_transaction(cfg, platform);
results.committed_mapping = test_committed_mapping();
results.measurement_disposition = test_measurement_conservation(cfg, platform);
results.ever_confirmed = test_ever_confirmed(cfg);
fprintf('Joint legacy backbone regression tests passed.\n');
end

function result = test_pure_active(cfg, platform)
[xyz, covariance, times, bearing, ids] = active_fixture(6, false);
baseline_cfg = cfg;
baseline_cfg.joint_extension_enabled = false;
baseline_cfg.passive_bearing_enabled = false;
baseline_cfg.passive_bearing_confirm_hit = false;
baseline_cfg.passive_bearing_update_on_active = true;
baseline_cfg.passive_bearing_update_on_pure = true;
baseline_cfg.passive_bearing_update_active_hit_tracks = true;
baseline_cfg.passive_bearing_min_dt_s = 0;
baseline_cfg.passive_bearing_fast_gate_deg = cfg.joint_passive_fast_gate_deg;
baseline_cfg.use_target_id_prior = false;
baseline = run_filter_adapt_ckf(xyz, covariance, times, baseline_cfg, ...
    bearing, platform, ids, []);
[joint, ~] = run_filter_joint_legacy_backbone(xyz, covariance, times, ...
    bearing, platform, ids, [], cfg, []);
assert(isequal(baseline.N, joint.mature3d.N));
assert_numeric_cells_equal(baseline.X, joint.mature3d.X, 1e-10);
assert_numeric_cells_equal(baseline.P, joint.mature3d.P, 1e-10);
assert_numeric_cells_equal(baseline.L, joint.mature3d.L, 0);
for k = 1:numel(times)
    assert(isequal(baseline.assoc{k}.id, joint.mature3d.assoc{k}.id));
    assert(isequal(baseline.assoc{k}.meas_index, ...
        joint.mature3d.assoc{k}.meas_index));
end
result = struct('events', numel(times), 'outputs', sum(joint.N), ...
    'exact_branch_match', true);
end

function result = test_pure_passive(cfg, platform)
K = 12;
times = (0:K-1)' * 0.05;
xyz = repmat({zeros(3, 0)}, K, 1);
covariance = repmat({zeros(3, 3, 0)}, K, 1);
ids = repmat({zeros(1, 0)}, K, 1);
bearing = cell(K, 1);
for k = 1:K
    bearing{k} = bearing_packet(5 + 0.01*k, 1, times(k), 801);
end
[joint, ~] = run_filter_joint_legacy_backbone(xyz, covariance, times, ...
    bearing, platform, ids, [], cfg, []);

cfg2 = cfg;
cfg2.joint_confirm_M = joint.passive2d.confirmation.M;
cfg2.joint_confirm_N = joint.passive2d.confirmation.N_events;
cfg2.joint_passive_confirm_group_size = ...
    joint.passive2d.confirmation.passive_group_size;
cfg2.joint_passive_confirmation_cycles_only = true;
cfg2.joint_streaming_quiet = true;
baseline = run_filter_joint_2d3d(joint.passive2d.event_meta, platform, cfg2);
assert(sum(joint.N2) > 0, ...
    'Pure-passive regression fixture did not reach a formal 2-D output.');
assert(isequal(baseline.N2, joint.passive2d.N2));
assert_numeric_cells_equal(baseline.X2, joint.passive2d.X2, 1e-10);
assert_numeric_cells_equal(baseline.P2, joint.passive2d.P2, 1e-10);
assert_numeric_cells_equal(baseline.L2, joint.passive2d.L2, 0);
for k = 1:K
    assert(isequal(baseline.assoc{k}.id, joint.passive2d.assoc{k}.id));
    assert(isequal(baseline.assoc{k}.type, joint.passive2d.assoc{k}.type));
end
offset = cfg.joint_2d_id_offset;
for k = 1:K
    local = joint.passive2d.output{k};
    formal = joint.output{k};
    if isempty(local), continue; end
    assert(isequal(sort([formal.id]), sort([local.id] + offset)));
end
result = struct('events', K, 'outputs', sum(joint.N2), ...
    'exact_branch_match', true);
end

function result = test_switch_transaction(cfg, platform)
cfg.joint_down_consecutive = 1;
cfg.joint_up_consecutive = 1;
cfg.joint_3d_upgrade_M = 2;
cfg.joint_3d_upgrade_N = 3;
cfg.joint_quality_eval_interval_s = 0;
cfg.joint_streaming_quiet = true;

% Degraded 3-D with no observed 2-D branch must remain the formal owner.
blocked_event = external_event(0, large_space_covariance(), false);
blocked = run_filter_joint_2d3d(blocked_event, platform, cfg);
q = blocked.companions{1};
assert(isscalar(q));
assert(q.active_output_dim == 3 && q.output_dim == 3);
assert(strcmp(q.switch_state, 'pending_3d_to_2d'));
assert(blocked.stats.switch_3d_to_2d_blocked_not_ready == 1);

% Once 2-D is observed, commit 3D->2D; retain 2-D while 3-D gathers 2/3;
% commit back only after the mature 3-D candidate is ready.
events = repmat(external_event(0, eye(9), true), 3, 1);
events(1) = external_event(0, large_space_covariance(), true);
events(2) = external_event(1, small_space_covariance(), true);
events(3) = external_event(2, small_space_covariance(), true);
switched = run_filter_joint_2d3d(events, platform, cfg);
q1 = switched.companions{1};
q2 = switched.companions{2};
q3 = switched.companions{3};
assert(q1.active_output_dim == 2 && q1.output_dim == 2);
assert(q2.active_output_dim == 2 && q2.output_dim == 2);
assert(strcmp(q2.switch_state, 'pending_2d_to_3d'));
assert(q3.active_output_dim == 3 && q3.output_dim == 3);
assert(strcmp(q3.switch_state, 'stable_3d'));
assert(q1.logical_id == q2.logical_id && q2.logical_id == q3.logical_id);
assert(switched.stats.switch_3d_to_2d_committed == 1);
assert(switched.stats.switch_2d_to_3d_committed == 1);
result = struct('blocked_owner_dim', q.active_output_dim, ...
    'pending_upgrade_owner_dim', q2.active_output_dim, ...
    'committed_upgrade_dim', q3.active_output_dim, ...
    'logical_id_preserved', true);
end

function result = test_committed_mapping()
offset = 1000000;
c = struct('external_3d_id', 10, 'pending_external_3d_id', 20, ...
    'local_2d_id', 7, 'logical_id', 7 + offset, 'from_2d', true);
committed_id = joint_legacy_committed_logical_id(10, c, offset);
pending_id = joint_legacy_committed_logical_id(20, c, offset);
historical_pending_id = pending_id;
assert(committed_id == 7 + offset);
assert(pending_id == 20);
c.external_3d_id = 20;
c.pending_external_3d_id = NaN;
after_commit_id = joint_legacy_committed_logical_id(20, c, offset);
assert(after_commit_id == 7 + offset);
assert(historical_pending_id == 20);
result = struct('pending_id', pending_id, ...
    'committed_logical_id', after_commit_id, ...
    'historical_id_unchanged', true);
end

function result = test_measurement_conservation(cfg, platform)
[xyz, covariance, times, bearing, ids] = active_fixture(6, true);
[joint, ~] = run_filter_joint_legacy_backbone(xyz, covariance, times, ...
    bearing, platform, ids, [], cfg, []);
s = joint.stats;
assert(s.active_measurements == ...
    s.active_assigned + s.active_births + s.active_birth_suppressed + ...
    s.active_unaccounted);
assert(s.passive_measurements == ...
    s.passive_assigned + s.passive_births + s.passive_birth_suppressed + ...
    s.passive_unaccounted);
assert(s.passive_measurements == ...
    s.passive_to_2d + s.passive_to_3d + s.passive_not_routed);
assert(s.active_unaccounted == 0 && s.passive_unaccounted == 0);
for k = 1:numel(joint.output)
    out = joint.output{k};
    assert(numel(unique([out.id])) == numel(out));
end
result = struct('active_input', s.active_measurements, ...
    'passive_input', s.passive_measurements, 'unaccounted', 0, ...
    'no_duplicate_formal_output', true);
end

function result = test_ever_confirmed(cfg)
cfg.metrics_max_print = 0;
cfg.truth_auto_discover = false;
cfg.truth_file = '';
cfg.track_accuracy_min_assoc = 1;
events = repmat(metric_event(), 2, 1);
for k = 1:2
    events(k) = metric_event(k-1);
end
a = struct('id', 1, 'type', {{'active'}}, 'meas_index', 1, ...
    'tid', 11, 'measurement_dim', 3, 'filter_dim', 3, ...
    'input_dim', 3, 'range_updated', true);
est = struct('assoc', {{a; a}}, 'output', {{metric_output(0); []}}, ...
    'logical_tracks', {{struct('id', 1, 'confirmed', true); []}}, ...
    'transition_log', struct([]));
m = evaluate_joint_tracking_metrics(est, events, cfg);
assert(m.three_d.association.n_assigned_confirmed == 2);
assert(m.three_d.association.rate_confirmed_tracks == 1);
result = struct('assigned_to_ever_confirmed', ...
    m.three_d.association.n_assigned_confirmed, 'rate', 1);
end

function cfg = test_config()
cfg = config_fusion();
cfg.metrics_max_print = 0;
cfg.metrics_progress_enabled = false;
cfg.joint_streaming_quiet = true;
cfg.do_plot = false;
cfg.result_save_file = '';
cfg.joint_progress_interval_events = 1000000;
end

function [xyz, covariance, times, bearing, ids] = active_fixture(K, with_passive)
times = (0:K-1)';
xyz = cell(K, 1); covariance = cell(K, 1);
bearing = cell(K, 1); ids = cell(K, 1);
for k = 1:K
    z = [1000 + 20*k; 50000 + 100*k; 500];
    xyz{k} = z;
    covariance{k} = diag([100, 100, 100].^2);
    ids{k} = 601;
    if with_passive
        ae = xyz_to_ae(z);
        bearing{k} = bearing_packet(ae(1), ae(2), times(k), 801);
    else
        bearing{k} = [];
    end
end
end

function packet = bearing_packet(az, el, t, target_id)
packet = struct('ang_deg', [az; el], ...
    'R_deg2', diag([0.05, 0.05].^2), 'src', 1, 'shard', 1, ...
    'kind', 1, 'tracklet_id', target_id, 't_sec', t, 'n_meas', 1);
end

function e = external_event(t, covariance, observed_angle)
e = empty_event();
e.cycle_id = round(t) + 1;
e.t_sec = t; e.t_start = t; e.t_end = t;
x = [1000; 0; 0; 50000; 0; 0; 500; 0; 0];
ae = xyz_to_ae(x([1, 4, 7]));
e.external_3d.id = 1;
e.external_3d.confirmed = true;
e.external_3d.state = x;
e.external_3d.cov = covariance;
e.external_3d.last_active_t = t;
e.external_3d.last_update_t = t;
e.external_3d.nis_norm = 1;
e.external_3d.active_hit = true;
e.external_3d.active_opportunity = true;
e.external_3d.dimension_ready = true;
e.external_3d.fresh = true;
e.external_3d.ang = ae;
e.external_3d.rate = [0; 0];
if observed_angle
    e.external_3d.update_track_id = 1;
    e.external_3d.update_ang = ae;
    e.external_3d.update_R = diag([0.02, 0.02].^2);
    e.external_3d.update_kind = 1;
end
end

function e = empty_event()
active = struct('t_sec', zeros(1, 0), 'xyz', zeros(3, 0), ...
    'rae', zeros(3, 0), 'R_xyz', zeros(3, 3, 0), ...
    'R_ae', zeros(2, 2, 0), 'has_range', false(1, 0), ...
    'ids', zeros(1, 0), 'src', zeros(1, 0), 'n_meas', 0);
passive = struct('t_sec', zeros(1, 0), 'ang', zeros(2, 0), ...
    'R_ae', zeros(2, 2, 0), 'ids', zeros(1, 0), ...
    'src', zeros(1, 0), 'kind', zeros(1, 0), 'n_meas', 0);
external = struct('id', zeros(1, 0), 'confirmed', false(1, 0), ...
    'state', zeros(9, 0), 'cov', zeros(9, 9, 0), ...
    'last_active_t', zeros(1, 0), 'last_update_t', zeros(1, 0), ...
    'nis_norm', zeros(1, 0), 'active_hit', false(1, 0), ...
    'active_opportunity', false, 'dimension_ready', false(1, 0), ...
    'fresh', false(1, 0), 'ang', zeros(2, 0), 'rate', zeros(2, 0), ...
    'update_track_id', zeros(1, 0), 'update_ang', zeros(2, 0), ...
    'update_R', zeros(2, 2, 0), 'update_kind', zeros(1, 0));
e = struct('cycle_id', 0, 't_sec', NaN, 't_start', NaN, 't_end', NaN, ...
    'confirm_cycle', false, 'has_active', false, 'has_passive', false, ...
    'active', active, 'passive', passive, 'external_3d', external);
end

function P = large_space_covariance()
P = eye(9);
P([1, 4, 7], [1, 4, 7]) = diag([30000, 30000, 30000].^2);
end

function P = small_space_covariance()
P = eye(9);
P([1, 4, 7], [1, 4, 7]) = diag([100, 100, 100].^2);
end

function e = metric_event(t)
if nargin == 0, t = 0; end
xyz = [1000; 50000; 500]; ae = xyz_to_ae(xyz);
e = struct('t_sec', t, 't_start', t, 't_end', t, ...
    'active', struct('n_meas', 1, 't_sec', t, 'ids', 11, ...
        'has_range', true, 'rae', [norm(xyz); ae], 'xyz', xyz), ...
    'passive', struct('n_meas', 0, 't_sec', [], 'ids', [], ...
        'ang', zeros(2, 0)));
end

function o = metric_output(t)
xyz = [1000; 50000; 500]; ae = xyz_to_ae(xyz);
o = struct('id', 1, 'truth_id', 11, 'output_dim', 3, ...
    't_sec', t, 'az_deg', ae(1), 'el_deg', ae(2), ...
    'position_enu', xyz);
end

function ae = xyz_to_ae(xyz)
ae = [atan2d(xyz(1), xyz(2)); ...
    atan2d(xyz(3), hypot(xyz(1), xyz(2)))];
end

function platform = static_platform()
platform = struct('n_rows', 2, 't_sec', [0; 10], ...
    'lat_deg', [0; 0], 'lon_deg', [0; 0], 'alt_m', [0; 0], ...
    'interp_lat', @(t) zeros(size(t)), ...
    'interp_lon', @(t) zeros(size(t)), ...
    'interp_alt', @(t) zeros(size(t)));
end

function assert_numeric_cells_equal(a, b, tolerance)
assert(numel(a) == numel(b));
for k = 1:numel(a)
    assert(isequal(size(a{k}), size(b{k})));
    if isempty(a{k}), continue; end
    delta = abs(a{k} - b{k});
    assert(all(delta(:) <= tolerance | ...
        (isnan(a{k}(:)) & isnan(b{k}(:)))));
end
end
