function result = test_joint_confirmed_association_fix()
%TEST_JOINT_CONFIRMED_ASSOCIATION_FIX Regression for partial transition logs.
%
% The old evaluator used transition_log exclusively as soon as it contained
% one logical_confirm_* entry. This fixture deliberately records confirmation
% for track 2 only, while the formal confirmed output and every association
% belong to track 1. Correct behavior must still classify track 1 as an
% ever-confirmed logical track.

cfg = struct();
cfg.metrics_max_print = 0;
cfg.truth_id_split_enabled = false;
cfg.truth_auto_discover = false;
cfg.truth_file = '';
cfg.track_accuracy_min_assoc = 1;

K = 5;
events = repmat(event_template(), K, 1);
est = struct();
est.output = cell(K, 1);
est.assoc = cell(K, 1);
est.logical_tracks = cell(K, 1);
est.L = cell(K, 1);
est.L2 = cell(K, 1);
est.transition_log = struct('id', 2, 'reason', 'logical_confirm_2d');

for k = 1:K
    t = k - 1;
    xyz = [1000 + 20*t; 50000; 1000];
    az = atan2d(xyz(1), xyz(2));
    el = atan2d(xyz(3), hypot(xyz(1), xyz(2)));
    events(k).t_sec = t;
    events(k).t_start = t;
    events(k).t_end = t;
    events(k).has_active = true;
    events(k).active.n_meas = 1;
    events(k).active.t_sec = t;
    events(k).active.ids = 101;
    events(k).active.has_range = true;
    events(k).active.rae = [norm(xyz); az; el];
    events(k).active.xyz = xyz;

    est.assoc{k} = struct('id', 1, 'type', {{'active'}}, ...
        'meas_index', 1, 'tid', 101, 'measurement_dim', 3, ...
        'filter_dim', 3, 'input_dim', 3, 'range_updated', true);
    est.output{k} = struct('id', 1, 'truth_id', 101, 'output_dim', 3, ...
        't_sec', t, 'az_deg', az, 'el_deg', el, 'position_enu', xyz, ...
        'confirmed', true, 'formal', true);
    est.logical_tracks{k} = struct('id', 1, 'confirmed', true);
    est.L{k} = [1, 1];
    est.L2{k} = zeros(0, 2);
end

metrics = evaluate_joint_tracking_metrics(est, events, cfg, []);
assert(metrics.three_d.association.n_assigned == K);
assert(metrics.three_d.association.n_assigned_confirmed == K);
assert(abs(metrics.three_d.association.rate_confirmed_tracks - 1) < 1e-12);
assert(abs(metrics.overall.association.rate_confirmed_tracks - 1) < 1e-12);
assert(ismember(1, metrics.association_confirmation.confirmed_track_ids));
assert(ismember(2, metrics.association_confirmation.confirmed_track_ids));

result = struct('passed', true, ...
    'confirmed_ids', metrics.association_confirmation.confirmed_track_ids, ...
    'three_d_rate', metrics.three_d.association.rate_confirmed_tracks, ...
    'overall_rate', metrics.overall.association.rate_confirmed_tracks);
disp(result);
end

function e = event_template()
e = struct('t_sec', NaN, 't_start', NaN, 't_end', NaN, ...
    'has_active', false, 'has_passive', false, ...
    'active', struct('n_meas', 0, 't_sec', [], 'ids', [], ...
        'has_range', false(1, 0), 'rae', zeros(3, 0), 'xyz', zeros(3, 0)), ...
    'passive', struct('n_meas', 0, 't_sec', [], 'ids', [], 'ang', zeros(2, 0)));
end
