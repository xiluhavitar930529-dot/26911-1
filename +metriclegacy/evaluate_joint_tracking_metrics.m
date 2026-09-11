function metrics = evaluate_joint_tracking_metrics(est, events, cfg, platform)
%EVALUATE_JOINT_TRACKING_METRICS 分别评价二维、三维及全部逻辑航迹。
%
% 关联率按物理量测类型划分。三维身份评价只使用RAE；二维被动身份
% 评价只使用已进入二维处理支路的被动AE，不要求确认或正式输出。
% 总体评价不混合角度与位置的物理单位。

if nargin < 1 || isempty(est), est = struct(); end
if nargin < 2 || isempty(events), events = repmat(empty_event(), 0, 1); end
if nargin < 3 || isempty(cfg), cfg = struct(); end
if nargin < 4, platform = []; end

truth_labels = build_joint_truth_labels(events, cfg);
data = collect_metric_data(est, events, truth_labels);
confirmed_ids = collect_confirmed_ids(est, data.output_track);

metrics = struct();
metrics.mode = 'joint_2d3d';
metrics.evaluation_version = 2;
metrics.status = 'ok';
metrics.truth_targets = truth_labels.summary;
metrics.id_split = metrics.truth_targets;
metrics.two_d = build_scope_metrics(data, 2, confirmed_ids, cfg);
metrics.three_d = build_scope_metrics(data, 3, confirmed_ids, cfg);
metrics.overall = build_scope_metrics(data, 0, confirmed_ids, cfg);
metrics.measurement_flow = build_measurement_flow(data);
metrics.measurement_accounting = build_measurement_accounting(est, data);
metrics.track_details = build_track_details(data, metrics);
metrics.measurement_consistency = struct( ...
    'status', 'ok', ...
    'reference', 'measurement_labels_and_active_measurement_positions', ...
    'two_d', metrics.two_d, 'three_d', metrics.three_d, ...
    'overall', metrics.overall);
metrics.real_truth = evaluate_joint_real_truth(est, events, platform, cfg, ...
    truth_labels, metrics.overall.accuracy.output_pairs);

% 保留原联合评价器字段，避免已有脚本失效。
metrics.association = metrics.overall.association;
metrics.angle = metrics.overall.angle;
metrics.position = metrics.three_d.position;
metrics.accuracy = metrics.overall.accuracy;
metrics.track_accuracy = metrics.overall.track_accuracy;
metrics.start_time = metrics.overall.start_time;
metrics.logical_tracks = struct( ...
    'n_unique', metrics.overall.output.n_unique_tracks, ...
    'n_transitions', get_transition_count(est), ...
    'n_2d_outputs', metrics.two_d.output.n_outputs, ...
    'n_3d_outputs', metrics.three_d.output.n_outputs);

if get_cfg(cfg, 'metrics_max_print', 12) > 0
    print_joint_report(metrics);
end
end

function data = collect_metric_data(est, events, labels)
K = numel(events);
n2_event = zeros(K, 1); n3_event = zeros(K, 1);
assoc_event = zeros(K, 1); output_event = zeros(K, 1);
for k = 1:K
    e = events(k); na = e.active.n_meas; np = e.passive.n_meas;
    has_range = measurement_has_range(e.active, na);
    n3_event(k) = nnz(has_range);
    n2_event(k) = np + na - n3_event(k);
    a = event_assoc(est, k); out = event_output(est, k);
    assoc_event(k) = numel(a.id);
    output_event(k) = numel(out);
end
data = preallocate_metric_data(sum(n2_event), sum(n3_event), ...
    sum(assoc_event), sum(output_event), n2_event, n3_event);
data.n_meas_2d = sum(n2_event);
data.n_meas_3d = sum(n3_event);
data.n_active_measurements = sum(arrayfun(@(e) e.active.n_meas, events));
data.n_passive_measurements = sum(arrayfun(@(e) e.passive.n_meas, events));
data.n_active_range_measurements = data.n_meas_3d;
p2 = 0; p3 = 0; pa = 0; po = 0;
for k = 1:K
    e = events(k);
    na = e.active.n_meas;
    np = e.passive.n_meas;
    has_range = measurement_has_range(e.active, na);

    active_ids = sized_row(labels.active{k}, na, NaN);
    passive_ids = sized_row(labels.passive{k}, np, NaN);
    active_t = measurement_times(e.active.t_sec, na, e.t_sec);
    passive_t = measurement_times(e.passive.t_sec, np, e.t_sec);
    n2 = n2_event(k); i2 = p2 + (1:n2);
    data.truth_2d_id(i2) = [active_ids(~has_range), passive_ids];
    data.truth_2d_t(i2) = [active_t(~has_range), passive_t];
    data.truth_2d_event(i2) = k;
    data.truth_2d_az(i2) = [e.active.rae(2, ~has_range), e.passive.ang(1, :)];
    data.truth_2d_el(i2) = [e.active.rae(3, ~has_range), e.passive.ang(2, :)];
    p2 = p2 + n2;

    n3 = n3_event(k); i3 = p3 + (1:n3);
    data.truth_3d_id(i3) = active_ids(has_range);
    data.truth_3d_t(i3) = active_t(has_range);
    data.truth_3d_event(i3) = k;
    data.truth_3d_az(i3) = e.active.rae(2, has_range);
    data.truth_3d_el(i3) = e.active.rae(3, has_range);
    data.truth_3d_xyz(:, i3) = e.active.xyz(:, has_range);
    p3 = p3 + n3;

    a = event_assoc(est, k);
    na_assoc = assoc_event(k); ia = pa + (1:na_assoc);
    filter_dim = association_filter_dimensions(a, est, k);
    input_dim = association_input_dimensions(a, filter_dim);
    active_input = measurement_input_dimensions(est, a, input_dim, k, 'active', na);
    passive_input = measurement_input_dimensions(est, a, input_dim, k, 'passive', np);
    data.truth_2d_input_dim(i2) = [active_input(~has_range), passive_input];
    data.truth_2d_is_passive(i2) = [false(1, nnz(~has_range)), true(1, np)];
    if na_assoc > 0
        data.assoc_track(ia) = reshape(a.id, 1, []);
        data.assoc_filter_dim(ia) = filter_dim;
        data.assoc_input_dim(ia) = input_dim;
        data.assoc_event(ia) = k;
    end
    for q = 1:na_assoc
        j = pa + q;
        data.assoc_truth(j) = association_truth_label(a, q, e, labels, k);
        meas_dim = association_measurement_dim(a, q, e);
        data.assoc_meas_dim(j) = meas_dim;
        type = indexed_text(a, 'type', q, '');
        data.assoc_is_active(j) = strncmp(type, 'active', 6);
        data.assoc_is_passive(j) = strncmp(type, 'passive', 7);
        mi = round(indexed_field_value(a, 'meas_index', q, 0));
        if data.assoc_is_active(j) && mi >= 1 && mi <= numel(active_t)
            data.assoc_t(j) = active_t(mi);
            data.assoc_input_dim(j) = active_input(mi);
        elseif data.assoc_is_passive(j) && mi >= 1 && mi <= numel(passive_t)
            data.assoc_t(j) = passive_t(mi);
            data.assoc_input_dim(j) = passive_input(mi);
        else
            data.assoc_t(j) = e.t_sec;
        end
        % Legacy association schemas only recorded accepted associations;
        % for them, a 3-D active record implies a completed range update.
        range_updated = meas_dim == 3;
        if isfield(a, 'range_updated')
            range_updated = logical(indexed_value(a.range_updated, q, false));
        end
        data.assoc_range_updated(j) = range_updated;
    end
    pa = pa + na_assoc;

    out = event_output(est, k);
    nout = output_event(k); io = po + (1:nout);
    if nout > 0
        data.output_track(io) = reshape([out.id], 1, []);
        data.output_truth(io) = reshape([out.truth_id], 1, []);
        data.output_dim(io) = reshape([out.output_dim], 1, []);
        data.output_t(io) = reshape([out.t_sec], 1, []);
        data.output_event(io) = k;
        data.output_az(io) = reshape([out.az_deg], 1, []);
        data.output_el(io) = reshape([out.el_deg], 1, []);
        data.output_pos(:, io) = cat(2, out.position_enu);
    end
    po = po + nout;
end
end

function scope = build_scope_metrics(data, dim, confirmed_ids, cfg)
if dim == 2
    meas_mask = data.assoc_meas_dim == 2;
    filter_mask = data.assoc_filter_dim == 2;
    output_mask = data.output_dim == 2;
    name = 'two_d';
    n_meas = data.n_meas_2d;
elseif dim == 3
    meas_mask = data.assoc_meas_dim == 3;
    filter_mask = data.assoc_filter_dim == 3;
    output_mask = data.output_dim == 3;
    name = 'three_d';
    n_meas = data.n_meas_3d;
else
    meas_mask = data.assoc_meas_dim > 0;
    filter_mask = true(size(data.assoc_track));
    output_mask = data.output_dim > 0;
    name = 'overall';
    n_meas = data.n_meas_2d + data.n_meas_3d;
end

output_ids = unique(data.output_track(output_mask & isfinite(data.output_track)));
if dim == 2
    scoped_assoc_mask = data.assoc_is_passive & data.assoc_input_dim == 2;
elseif dim == 3
    scoped_assoc_mask = meas_mask & data.assoc_input_dim == 3;
else
    scoped_assoc_mask = filter_mask;
end
[truth_id, ~, truth_t, reference_basis] = evaluation_truth_scope(data, dim);
scope = struct();
scope.name = name;
scope.reference = struct('basis', reference_basis, ...
    'n_labeled_measurements', nnz(isfinite(truth_id)));
scope.association = association_metrics(n_meas, data.assoc_track(meas_mask), confirmed_ids);
scope.association.n_mode_assigned = nnz(scoped_assoc_mask);
scope.accuracy = id_accuracy_metrics(data.assoc_track(scoped_assoc_mask), ...
    data.assoc_truth(scoped_assoc_mask), output_ids, truth_id, cfg);
scope.track_accuracy = scope.accuracy.track_level;
scope.start_time = start_delay_metrics(scope.accuracy, truth_id, truth_t, ...
    data.output_track(output_mask), data.output_t(output_mask), ...
    data.assoc_track(scoped_assoc_mask), data.assoc_truth(scoped_assoc_mask), ...
    data.assoc_t(scoped_assoc_mask));
[scope.angle, scope.position] = output_error_metrics( ...
    data, dim, output_mask, scope.accuracy.output_pairs);
scope.output_coverage = formal_output_coverage( ...
    data, dim, output_mask, scope.accuracy.output_pairs, output_ids);
scope.output = struct('n_outputs', nnz(output_mask), ...
    'n_unique_tracks', numel(output_ids), 'track_ids', output_ids);
if dim == 2
    passive = data.truth_2d_is_passive;
    scope.reference.n_passive_input = nnz(passive);
    scope.reference.n_passive_to_2d = nnz(passive & data.truth_2d_input_dim == 2);
    scope.reference.n_passive_to_3d = nnz(passive & data.truth_2d_input_dim == 3);
    scope.reference.n_passive_not_routed = nnz(passive & data.truth_2d_input_dim == 0);
end
end

function m = association_metrics(n_meas, assigned_ids, confirmed_ids)
n_assigned = numel(assigned_ids);
n_confirmed = nnz(ismember(assigned_ids, confirmed_ids));
m = struct('basis', 'measurement_dimension', ...
    'n_measurements', n_meas, ...
    'n_assigned', n_assigned, ...
    'n_assigned_confirmed', n_confirmed, ...
    'rate_all_tracks', safe_ratio(n_assigned, n_meas), ...
    'rate_confirmed_tracks', safe_ratio(n_confirmed, n_meas), ...
    'n_mode_assigned', 0);
end

function acc = id_accuracy_metrics(track_id, truth_id, output_ids, truth_reference, cfg)
valid = isfinite(track_id) & isfinite(truth_id);
track_id = track_id(valid);
truth_id = truth_id(valid);
truth_reference = truth_reference(isfinite(truth_reference));
[rows, ~, row_index] = unique(truth_id);
[cols, ~, col_index] = unique(track_id);
C = accumarray([row_index(:), col_index(:)], 1, ...
    [numel(rows), numel(cols)], @sum, 0);

pairs = count_pairs(C, rows, cols, truth_reference);
correct = sum([pairs.n_correct]);
if isempty(pairs), correct = 0; end
acc = struct();
acc.n_assoc_total = numel(valid);
acc.n_labeled_assoc = nnz(valid);
acc.n_correct = correct;
acc.n_error = numel(track_id) - correct;
acc.accuracy = safe_ratio(correct, numel(track_id));
acc.n_truth = numel(rows);
acc.n_tracks = numel(cols);
acc.truth_ids = rows;
acc.track_ids = cols;
acc.confusion = C;
acc.pairs = pairs;
acc.fragmented_truth_count = nnz(sum(C > 0, 2) > 1);
acc.mixed_track_count = nnz(sum(C > 0, 1) > 1);

output_ids = unique(output_ids(isfinite(output_ids)));
[~, output_cols] = ismember(output_ids, cols);
valid_output = output_cols > 0;
Cout = zeros(numel(rows), numel(output_ids));
Cout(:, valid_output) = C(:, output_cols(valid_output));
output_pairs = count_pairs(Cout, rows, output_ids, truth_reference);
acc.output_pairs = output_pairs;
acc.output_labeled_counts = sum(Cout, 1);
acc.track_level = track_accuracy_metrics(Cout, output_ids, ...
    output_pairs, truth_reference, cfg);
end

function m = track_accuracy_metrics(C, output_ids, pairs, truth_reference, cfg)
purity_th = get_cfg(cfg, 'track_accuracy_purity_th', 0.9);
min_assoc = max(1, round(get_cfg(cfg, 'track_accuracy_min_assoc', 3)));
n_output = numel(output_ids);
evaluable = false(1, n_output);
correct = false(1, n_output);
purity = nan(1, n_output);
consistency = nan(1, n_output);
for c = 1:n_output
    n_track_assoc = sum(C(:, c));
    evaluable(c) = n_track_assoc >= min_assoc;
    p = find([pairs.track_id] == output_ids(c), 1);
    if isempty(p), continue; end
    % The numerator and complete reference count use the same measurement
    % scope (RAE / passive AE entering 2-D / overall), never formal retention.
    purity(c) = pairs(p).coverage;
    consistency(c) = safe_ratio(pairs(p).n_correct, n_track_assoc);
    correct(c) = evaluable(c) && pairs(p).n_correct >= min_assoc && ...
        purity(c) >= purity_th;
end
n_truth_reference = numel(unique(truth_reference(isfinite(truth_reference))));
coverage_distribution = summarize_track_coverage(purity);
m = struct('purity_threshold', purity_th, ...
    'score_basis', 'matched_truth_measurement_coverage', ...
    'min_labeled_assoc', min_assoc, ...
    'n_output_tracks', n_output, 'n_truth_reference', n_truth_reference, ...
    'n_evaluable_tracks', nnz(evaluable), ...
    'n_correct_tracks', nnz(correct), ...
    'n_error_or_extra_tracks', n_output - nnz(correct), ...
    'track_ids', output_ids, 'purity', purity, 'coverage', purity, ...
    'coverage_distribution', coverage_distribution, ...
    'association_consistency', consistency, 'is_correct', correct, ...
    'accuracy_vs_output', safe_ratio(nnz(correct), n_output), ...
    'accuracy_vs_truth', safe_ratio(nnz(correct), n_truth_reference));
end

function d = summarize_track_coverage(coverage)
% Distribution over matched output tracks only. Unmatched/extra tracks remain
% visible in n_error_or_extra_tracks and accuracy_vs_output.
values = coverage(isfinite(coverage));
d = struct('basis', 'matched_output_tracks', 'n_tracks', numel(values), ...
    'mean', NaN, 'median', NaN, ...
    'n_ge_90', 0, 'n_ge_95', 0, 'n_ge_98', 0, 'n_ge_99', 0, ...
    'rate_ge_90', NaN, 'rate_ge_95', NaN, ...
    'rate_ge_98', NaN, 'rate_ge_99', NaN);
if isempty(values), return; end
d.mean = mean(values);
d.median = median(values);
d.n_ge_90 = nnz(values >= 0.90);
d.n_ge_95 = nnz(values >= 0.95);
d.n_ge_98 = nnz(values >= 0.98);
d.n_ge_99 = nnz(values >= 0.99);
d.rate_ge_90 = d.n_ge_90 / d.n_tracks;
d.rate_ge_95 = d.n_ge_95 / d.n_tracks;
d.rate_ge_98 = d.n_ge_98 / d.n_tracks;
d.rate_ge_99 = d.n_ge_99 / d.n_tracks;
end

function pairs = count_pairs(C, truth_ids, track_ids, truth_reference)
pairs = repmat(struct('truth_id', NaN, 'track_id', NaN, 'n_correct', 0, ...
    'truth_assoc', 0, 'track_assoc', 0, 'truth_total', 0, 'coverage', NaN, ...
    'association_consistency', NaN), 0, 1);
if isempty(C) || ~any(C(:) > 0), return; end
max_count = max(C(:));
ij = solve_global_assignment(max_count - C, max_count);
for q = 1:size(ij, 1)
    r = ij(q, 1); c = ij(q, 2);
    if C(r, c) <= 0, continue; end
    p = struct();
    p.truth_id = truth_ids(r);
    p.track_id = track_ids(c);
    p.n_correct = C(r, c);
    p.truth_assoc = sum(C(r, :));
    p.track_assoc = sum(C(:, c));
    p.truth_total = nnz(truth_reference == truth_ids(r));
    p.coverage = safe_ratio(C(r, c), p.truth_total);
    p.association_consistency = safe_ratio(C(r, c), p.track_assoc);
    pairs(end + 1, 1) = p; %#ok<AGROW>
end
end

function s = start_delay_metrics(acc, truth_id, truth_t, output_id, output_t, ...
        assoc_track, assoc_truth, assoc_t)
valid_truth = isfinite(truth_id) & isfinite(truth_t);
truth_id = truth_id(valid_truth);
truth_t = truth_t(valid_truth);
truth_keys = unique(truth_id);
delays = zeros(1, 0);
main_delays = zeros(1, 0);
n_started = 0;
n_main = 0;
n_fragment_candidates = 0;
for q = 1:numel(truth_keys)
    tid = truth_keys(q);
    t0 = min(truth_t(truth_id == tid));
    r = find(acc.truth_ids == tid, 1);
    candidate_ids = zeros(1, 0);
    if ~isempty(r) && ~isempty(acc.confusion)
        candidate_ids = acc.track_ids(acc.confusion(r, :) > 0);
        candidate_ids = intersect(candidate_ids, unique(output_id), 'stable');
    end
    n_fragment_candidates = n_fragment_candidates + numel(candidate_ids);
    first_t = inf;
    for c = 1:numel(candidate_ids)
        first_correct_assoc = min_or_inf(assoc_t( ...
            assoc_track == candidate_ids(c) & assoc_truth == tid));
        if ~isfinite(first_correct_assoc), continue; end
        tt = output_t(output_id == candidate_ids(c));
        tt = tt(isfinite(tt) & tt >= max(t0, first_correct_assoc));
        if ~isempty(tt), first_t = min(first_t, min(tt)); end
    end
    if isfinite(first_t)
        n_started = n_started + 1;
        delays(end + 1) = first_t - t0; %#ok<AGROW>
    end

    p = find([acc.output_pairs.truth_id] == tid, 1);
    if ~isempty(p)
        main_track = acc.output_pairs(p).track_id;
        first_correct_assoc = min_or_inf(assoc_t( ...
            assoc_track == main_track & assoc_truth == tid));
        tt = output_t(output_id == main_track);
        tt = tt(isfinite(tt) & tt >= max(t0, first_correct_assoc));
        if ~isempty(tt)
            n_main = n_main + 1;
            main_delays(end + 1) = min(tt) - t0; %#ok<AGROW>
        end
    end
end
s = struct('n_truth', numel(truth_keys), 'n_started', n_started, ...
    'n_main_confirmed', n_main, 'n_fragment_candidates', n_fragment_candidates, ...
    'mean_track_start_delay_s', mean_or_nan(delays), ...
    'mean_main_track_start_delay_s', mean_or_nan(main_delays));
end

function value = min_or_inf(values)
values = values(isfinite(values));
if isempty(values), value = inf; else, value = min(values); end
end

function [angle, position] = output_error_metrics(data, dim, output_mask, pairs)
[az_err, el_err, pos_err, ~, ~, los_err] = output_error_samples(data, dim, output_mask, pairs);
angle = angle_metrics(az_err, el_err, los_err);
position = position_metrics(pos_err);
end

function [az_err, el_err, pos_err, angle_track, position_track, los_err] = ...
        output_error_samples(data, dim, output_mask, pairs)
az_err = zeros(1, 0); el_err = zeros(1, 0); pos_err = zeros(3, 0);
los_err = zeros(1, 0);
angle_track = zeros(1, 0); position_track = zeros(1, 0);
out_index = find(output_mask);
if isempty(out_index) || isempty(pairs)
    return;
end
pair_track = reshape([pairs.track_id], 1, []);
pair_truth = reshape([pairs.truth_id], 1, []);
[matched, pair_index] = ismember(data.output_track(out_index), pair_track);
out_index = out_index(matched);
paired_truth = pair_truth(pair_index(matched));

[angle_keys, ref_az, ref_el] = angle_reference_table(data, dim);
if ~isempty(out_index) && ~isempty(angle_keys)
    query = [data.output_event(out_index).', paired_truth(:)];
    [has_reference, reference_index] = ismember(query, angle_keys, 'rows');
    valid = has_reference & isfinite(data.output_az(out_index)).' & ...
        isfinite(data.output_el(out_index)).';
    candidate = find(valid);
    if ~isempty(candidate)
        r = reference_index(candidate);
        finite_reference = isfinite(ref_az(r)) & isfinite(ref_el(r));
        candidate = candidate(finite_reference); r = r(finite_reference);
        az_err = angle_diff(data.output_az(out_index(candidate)), ref_az(r).');
        el_err = data.output_el(out_index(candidate)) - ref_el(r).';
        los_err = los_separation_deg(data.output_az(out_index(candidate)), ...
            data.output_el(out_index(candidate)), ref_az(r).', ref_el(r).');
        angle_track = data.output_track(out_index(candidate));
    end
end

position_candidate = data.output_dim(out_index) == 3 & ...
    all(isfinite(data.output_pos(:, out_index)), 1);
if any(position_candidate)
    position_index = out_index(position_candidate);
    position_truth = paired_truth(position_candidate);
    [position_keys, ref_position] = position_reference_table(data);
    query = [data.output_event(position_index).', position_truth(:)];
    [has_reference, reference_index] = ismember(query, position_keys, 'rows');
    candidate = find(has_reference);
    if ~isempty(candidate)
        r = reference_index(candidate);
        finite_reference = all(isfinite(ref_position(:, r)), 1);
        candidate = candidate(finite_reference); r = r(finite_reference);
        pos_err = data.output_pos(:, position_index(candidate)) - ref_position(:, r);
        position_track = data.output_track(position_index(candidate));
    end
end
end

function coverage = formal_output_coverage(data, dim, output_mask, pairs, output_ids)
[truth_id, truth_event] = evaluation_truth_scope(data, dim);
per_track = repmat(struct('track_id', NaN, 'truth_id', NaN, ...
    'n_reference_measurements', 0, 'n_covered_measurements', 0, ...
    'rate', NaN), numel(output_ids), 1);
valid_reference = isfinite(truth_id) & isfinite(truth_event);
covered_reference = false(size(truth_id));
pair_track = reshape([pairs.track_id], 1, []);
pair_truth = reshape([pairs.truth_id], 1, []);
[matched_tracks, pair_index] = ismember(output_ids, pair_track);
track_truth = nan(size(output_ids));
track_truth(matched_tracks) = pair_truth(pair_index(matched_tracks));

out_index = find(output_mask);
if ~isempty(out_index) && ~isempty(pair_track)
    [matched_output, output_pair_index] = ...
        ismember(data.output_track(out_index), pair_track);
    formal_keys = [data.output_event(out_index(matched_output)).', ...
        pair_truth(output_pair_index(matched_output)).'];
    formal_keys = unique(formal_keys(all(isfinite(formal_keys), 2), :), 'rows');
    reference_query = [truth_event(valid_reference).', truth_id(valid_reference).'];
    covered_reference(valid_reference) = ismember(reference_query, formal_keys, 'rows').';
end

reference_truth = truth_id(valid_reference);
covered_truth = truth_id(valid_reference & covered_reference);
truth_keys = unique(reference_truth);
reference_counts = zeros(size(truth_keys)); covered_counts = zeros(size(truth_keys));
if ~isempty(truth_keys)
    [~, location] = ismember(reference_truth, truth_keys);
    reference_counts = accumarray(location(:), 1, [numel(truth_keys), 1]).';
    if ~isempty(covered_truth)
        [~, location] = ismember(covered_truth, truth_keys);
        covered_counts = accumarray(location(:), 1, [numel(truth_keys), 1]).';
    end
end
for q = 1:numel(output_ids)
    id = output_ids(q);
    per_track(q).track_id = id;
    if ~matched_tracks(q), continue; end
    tid = track_truth(q);
    [found, location] = ismember(tid, truth_keys);
    if found
        n_ref = reference_counts(location); n_covered_track = covered_counts(location);
    else
        n_ref = 0; n_covered_track = 0;
    end
    per_track(q).truth_id = tid;
    per_track(q).n_reference_measurements = n_ref;
    per_track(q).n_covered_measurements = n_covered_track;
    per_track(q).rate = safe_ratio(n_covered_track, n_ref);
end
n_reference = nnz(valid_reference);
n_covered = nnz(covered_reference & valid_reference);
coverage = struct('basis', 'same_event_formal_output', ...
    'n_reference_measurements', n_reference, ...
    'n_covered_measurements', n_covered, ...
    'n_uncovered_measurements', n_reference - n_covered, ...
    'rate', safe_ratio(n_covered, n_reference), ...
    'n_matched_tracks', nnz(matched_tracks), 'per_track', per_track);
end

function [truth_id, truth_event, truth_t, basis] = evaluation_truth_scope(data, dim)
if dim == 2
    keep = data.truth_2d_is_passive & data.truth_2d_input_dim == 2;
    truth_id = data.truth_2d_id(keep);
    truth_event = data.truth_2d_event(keep);
    truth_t = data.truth_2d_t(keep);
    basis = 'passive_ae_entering_2d_branch';
elseif dim == 3
    truth_id = data.truth_3d_id;
    truth_event = data.truth_3d_event;
    truth_t = data.truth_3d_t;
    basis = 'all_input_rae';
else
    truth_id = [data.truth_2d_id, data.truth_3d_id];
    truth_event = [data.truth_2d_event, data.truth_3d_event];
    truth_t = [data.truth_2d_t, data.truth_3d_t];
    basis = 'all_input_measurements';
end
end

function [keys, ref_az, ref_el] = angle_reference_table(data, dim)
if dim == 2 || dim == 3
    dim = 0;
end
[truth_id, truth_event, truth_az, truth_el] = truth_scope_arrays(data, dim);
valid = isfinite(truth_id) & isfinite(truth_event) & ...
    isfinite(truth_az) & isfinite(truth_el);
if ~any(valid)
    keys = zeros(0, 2); ref_az = zeros(0, 1); ref_el = zeros(0, 1);
    return;
end
[keys, ~, group] = unique([truth_event(valid).', truth_id(valid).'], 'rows');
n = size(keys, 1); count = accumarray(group, 1, [n, 1]);
sin_sum = accumarray(group, reshape(sind(truth_az(valid)), [], 1), [n, 1], @sum);
cos_sum = accumarray(group, reshape(cosd(truth_az(valid)), [], 1), [n, 1], @sum);
el_sum = accumarray(group, reshape(truth_el(valid), [], 1), [n, 1], @sum);
ref_az = atan2d(sin_sum ./ count, cos_sum ./ count);
ref_el = el_sum ./ count;
end

function [keys, ref_position] = position_reference_table(data)
valid = isfinite(data.truth_3d_id) & isfinite(data.truth_3d_event) & ...
    all(isfinite(data.truth_3d_xyz), 1);
if ~any(valid)
    keys = zeros(0, 2); ref_position = zeros(3, 0);
    return;
end
[keys, ~, group] = unique( ...
    [data.truth_3d_event(valid).', data.truth_3d_id(valid).'], 'rows');
n = size(keys, 1); count = accumarray(group, 1, [n, 1]);
ref_position = zeros(3, n);
for axis = 1:3
    total = accumarray(group, reshape(data.truth_3d_xyz(axis, valid), [], 1), ...
        [n, 1], @sum);
    ref_position(axis, :) = (total ./ count).';
end
end

function [id, event, az, el] = truth_scope_arrays(data, dim)
if dim == 2
    id = data.truth_2d_id; event = data.truth_2d_event;
    az = data.truth_2d_az; el = data.truth_2d_el;
elseif dim == 3
    id = data.truth_3d_id; event = data.truth_3d_event;
    az = data.truth_3d_az; el = data.truth_3d_el;
else
    id = [data.truth_2d_id, data.truth_3d_id];
    event = [data.truth_2d_event, data.truth_3d_event];
    az = [data.truth_2d_az, data.truth_3d_az];
    el = [data.truth_2d_el, data.truth_3d_el];
end
end

function m = angle_metrics(az_err, el_err, los_err)
m = struct('n', numel(az_err), 'rmse_az_deg', NaN, ...
    'rmse_el_deg', NaN, 'rmse_los_deg', NaN);
if isempty(az_err), return; end
m.rmse_az_deg = sqrt(mean(az_err.^2));
m.rmse_el_deg = sqrt(mean(el_err.^2));
m.rmse_los_deg = sqrt(mean(los_err.^2));
end

function m = position_metrics(pos_err)
m = struct('n', size(pos_err, 2), 'rmse_e_m', NaN, ...
    'rmse_n_m', NaN, 'rmse_u_m', NaN, 'rmse_3d_m', NaN);
if isempty(pos_err), return; end
m.rmse_e_m = sqrt(mean(pos_err(1, :).^2));
m.rmse_n_m = sqrt(mean(pos_err(2, :).^2));
m.rmse_u_m = sqrt(mean(pos_err(3, :).^2));
m.rmse_3d_m = sqrt(mean(sum(pos_err.^2, 1)));
end

function print_joint_report(metrics)
fprintf('\n========== 二维/三维分维度定量评价 ==========\n');
R = metrics.real_truth;
fprintf('\n[真实真值精度]\n');
if strcmp(R.status, 'ok')
    fprintf('  真值文件: %s\n', R.source_file);
    fprintf(['  真值目标=%d, 正式输出=%d, 身份映射=%d(%.2f%%), ', ...
        '时间匹配=%d(%.2f%%), 目标覆盖=%d(%.2f%%)\n'], ...
        R.n_truth_targets, R.n_formal_outputs, R.n_identity_mapped_outputs, ...
        100 * R.mapping_rate, R.n_time_matched_outputs, ...
        100 * R.time_match_rate, R.n_output_truth_targets, ...
        100 * R.target_coverage_rate);
    if R.two_d.angle.n > 0
        fprintf('  二维真实角度RMSE: az=%.4fdeg, el=%.4fdeg, LOS=%.4fdeg (%d点)\n', ...
            R.two_d.angle.rmse_az_deg, R.two_d.angle.rmse_el_deg, ...
            R.two_d.angle.rmse_los_deg, R.two_d.angle.n);
    end
    if R.three_d.position.n > 0
        fprintf('  三维真实位置RMSE: E/N/U=[%.2f %.2f %.2f]m, 3D=%.2fm (%d点)\n', ...
            R.three_d.position.rmse_e_m, R.three_d.position.rmse_n_m, ...
            R.three_d.position.rmse_u_m, R.three_d.position.rmse_3d_m, ...
            R.three_d.position.n);
    end
else
    fprintf('  不可用: %s (%s)\n', R.status, R.reason);
end

fprintf('\n[量测伪真值一致性]\n');
T = metrics.truth_targets;
fprintf('量测标签参考数: 原始编号=%d, 拆分后实例=%d, 被拆分原始编号=%d\n', ...
    T.raw_id_count, T.instance_count, T.n_split_raw_ids);
if T.enabled && T.n_angle_only_raw_ids > 0
    fprintf('  其中纯角度编号=%d（无三维位置，各按1个目标实例计数）\n', ...
        T.n_angle_only_raw_ids);
end
print_scope_report('二维被动AE航迹/输出', metrics.two_d, 2);
print_scope_report('三维主动空间', metrics.three_d, 3);
print_scope_report('全部逻辑航迹', metrics.overall, 0);
F = metrics.measurement_flow;
fprintf('\n[物理量测维度 -> 关联航迹维度]\n');
fprintf('  二维量测: 关联后未保留=%d, 二维=%d, 三维=%d\n', F.counts(1, :));
fprintf('  三维量测: 关联后未保留=%d, 二维=%d, 三维=%d\n', F.counts(2, :));
fprintf('  主动距离关联: %d, 完成空间更新=%d, 未更新=%d\n', ...
    F.n_range_associated, F.n_range_updated, ...
    F.n_range_associated_without_update);
A = metrics.measurement_accounting;
fprintf('\n[物理量测去向]\n');
print_accounting_row('主动全部', A.active);
print_accounting_row('主动距离', A.active_range);
print_accounting_row('被动角度', A.passive);
end

function print_accounting_row(label, row)
fprintf(['  %s: 输入=%d, 量测利用(关联或新生)=%d(%.2f%%), ' ...
    '去2D=%d, 去3D=%d, 关联/新生后未保留=%d, ' ...
    '明确抑制=%d, 未解释=%d, 去向字段差=%d\n'], ...
    label, row.n_input, row.n_associated_or_born, 100 * row.utilization_rate, ...
    row.n_to_2d, row.n_to_3d, row.n_associated_not_retained, ...
    row.n_explicitly_suppressed, row.unaccounted, row.destination_gap);
end

function print_scope_report(label, s, dim)
fprintf('\n[%s]\n', label);
fprintf('  输出: %d点, %d条逻辑航迹\n', s.output.n_outputs, s.output.n_unique_tracks);
if dim == 2
    fprintf('  被动输入分流: 进入2D=%d, 进入3D=%d, 未送入支路=%d; 二维带标签参考=%d\n', ...
        s.reference.n_passive_to_2d, s.reference.n_passive_to_3d, ...
        s.reference.n_passive_not_routed, s.reference.n_labeled_measurements);
elseif dim == 3
    fprintf('  三维带标签RAE参考=%d（被动AE不参与三维得分计数）\n', ...
        s.reference.n_labeled_measurements);
end
fprintf('  量测关联率(全部/曾确认): %.2f%% / %.2f%%  (%d/%d, %d/%d)\n', ...
    100 * s.association.rate_all_tracks, 100 * s.association.rate_confirmed_tracks, ...
    s.association.n_assigned, s.association.n_measurements, ...
    s.association.n_assigned_confirmed, s.association.n_measurements);
fprintf('  关联点一致率: %.2f%%  一致=%d, 不一致=%d, 带编号=%d\n', ...
    100 * s.accuracy.accuracy, s.accuracy.n_correct, ...
    s.accuracy.n_error, s.accuracy.n_labeled_assoc);
fprintf('  航迹级正确率(比输出/比参考): %.2f%% / %.2f%%  正确航迹数=%d, 输出=%d, 参考=%d\n', ...
    100 * s.track_accuracy.accuracy_vs_output, ...
    100 * s.track_accuracy.accuracy_vs_truth, ...
    s.track_accuracy.n_correct_tracks, s.track_accuracy.n_output_tracks, ...
    s.track_accuracy.n_truth_reference);
d = s.track_accuracy.coverage_distribution;
fprintf(['  航迹覆盖率分布(已匹配输出=%d): 均值=%.2f%%, 中位数=%.2f%%; ' ...
    '>=90/95/98/99%%: %.2f/%.2f/%.2f/%.2f%%\n'], ...
    d.n_tracks, 100 * d.mean, 100 * d.median, ...
    100 * d.rate_ge_90, 100 * d.rate_ge_95, ...
    100 * d.rate_ge_98, 100 * d.rate_ge_99);
fprintf('  正式输出同步覆盖率: %.2f%%  覆盖=%d/%d个带标签物理量测点\n', ...
    100 * s.output_coverage.rate, s.output_coverage.n_covered_measurements, ...
    s.output_coverage.n_reference_measurements);
fprintf('  航迹起始: 最早确认=%d/%d, 平均延迟=%.3fs; 主航迹=%d/%d, 平均延迟=%.3fs\n', ...
    s.start_time.n_started, s.start_time.n_truth, ...
    s.start_time.mean_track_start_delay_s, s.start_time.n_main_confirmed, ...
    s.start_time.n_truth, s.start_time.mean_main_track_start_delay_s);
if s.angle.n > 0
    if dim == 3
        angle_label = '三维航迹角度投影RMSE';
    elseif dim == 2
        angle_label = '二维角度RMSE';
    else
        angle_label = '全部输出角度RMSE';
    end
    fprintf('  %s: az=%.4fdeg, el=%.4fdeg, LOS=%.4fdeg (%d点)\n', ...
        angle_label, s.angle.rmse_az_deg, s.angle.rmse_el_deg, ...
        s.angle.rmse_los_deg, s.angle.n);
end
if dim ~= 2 && s.position.n > 0
    fprintf('  三维位置RMSE: E/N/U=[%.2f %.2f %.2f]m, 3D=%.2fm (%d点)\n', ...
        s.position.rmse_e_m, s.position.rmse_n_m, s.position.rmse_u_m, ...
        s.position.rmse_3d_m, s.position.n);
end
end

function a = event_assoc(est, k)
a = struct('id', zeros(1, 0), 'type', {cell(1, 0)}, ...
    'meas_index', zeros(1, 0), 'tid', zeros(1, 0));
if isfield(est, 'assoc') && k <= numel(est.assoc) && ~isempty(est.assoc{k})
    a = est.assoc{k};
end
end

function out = event_output(est, k)
out = repmat(struct('id', 0, 'truth_id', NaN, 'output_dim', 0, ...
    't_sec', NaN, 'az_deg', NaN, 'el_deg', NaN, ...
    'position_enu', nan(3, 1)), 0, 1);
if isfield(est, 'output') && k <= numel(est.output) && ~isempty(est.output{k})
    out = est.output{k};
end
end

function dim = association_measurement_dim(a, q, e)
dim = 0;
if isfield(a, 'measurement_dim') && q <= numel(a.measurement_dim) && ...
        isfinite(a.measurement_dim(q)) && any(a.measurement_dim(q) == [2, 3])
    dim = a.measurement_dim(q);
    return;
end
type = indexed_text(a, 'type', q, '');
if isempty(type), return; end
if strncmp(type, 'passive', 7)
    dim = 2;
elseif strncmp(type, 'active', 6)
    mi = indexed_field_value(a, 'meas_index', q, 0);
    if mi >= 1 && mi <= numel(e.active.has_range) && e.active.has_range(mi)
        dim = 3;
    else
        dim = 2;
    end
end
end

function truth = association_truth_label(a, q, ~, labels, k)
truth = NaN;
if isfield(a, 'tid'), truth = indexed_value(a.tid, q, NaN); end
type = indexed_text(a, 'type', q, '');
mi = round(indexed_field_value(a, 'meas_index', q, 0));
if mi < 1, return; end
if strncmp(type, 'active', 6) && k <= numel(labels.active) && ...
        mi <= numel(labels.active{k})
    truth = labels.active{k}(mi);
elseif strncmp(type, 'passive', 7) && k <= numel(labels.passive) && ...
        mi <= numel(labels.passive{k})
    truth = labels.passive{k}(mi);
end
end

function dims = association_filter_dimensions(a, est, k)
ids = reshape(a.id, 1, []);
dims = zeros(size(ids));
if isempty(ids), return; end
if isfield(a, 'filter_dim') && ~isempty(a.filter_dim)
    n = min(numel(ids), numel(a.filter_dim));
    supplied = reshape(a.filter_dim(1:n), 1, []);
    valid = isfinite(supplied) & ismember(supplied, [2, 3]);
    dims(find(valid)) = supplied(valid); %#ok<FNDSB>
end
unresolved = dims == 0;
out = event_output(est, k);
if any(unresolved) && ~isempty(out)
    out_ids = reshape([out.id], 1, []);
    out_dims = reshape([out.output_dim], 1, []);
    [found, location] = ismember(ids(unresolved), out_ids);
    target = find(unresolved); target = target(found);
    dims(target) = out_dims(location(found));
end
unresolved = dims == 0;
if ~any(unresolved) || ~isfield(est, 'logical_tracks') || ...
        k > numel(est.logical_tracks) || isempty(est.logical_tracks{k})
    return;
end
tracks = est.logical_tracks{k}; track_ids = reshape([tracks.id], 1, []);
[found, location] = ismember(ids(unresolved), track_ids);
target = find(unresolved); target = target(found);
matched_location = location(found);
for q = 1:numel(target)
    dims(target(q)) = snapshot_dimension(tracks(matched_location(q)));
end
end

function dims = association_input_dimensions(a, retained_dims)
dims = retained_dims;
if isfield(a, 'input_dim') && numel(a.input_dim) == numel(dims)
    dims = reshape(a.input_dim, 1, []);
else
    % A legacy passive birth with no retained track still entered the 2-D branch.
    for q = find(dims == 0)
        if strcmp(indexed_text(a, 'type', q, ''), 'passive_birth'), dims(q) = 2; end
    end
end
end

function dims = measurement_input_dimensions(est, a, assoc_dims, k, type, n)
dims = zeros(1, n);
for q = 1:numel(a.id)
    if ~strncmp(indexed_text(a, 'type', q, ''), type, numel(type)), continue; end
    mi = indexed_field_value(a, 'meas_index', q, 0);
    if isfinite(mi) && mi >= 1 && mi <= n && mi == round(mi)
        dims(mi) = assoc_dims(q);
    end
end
if isfield(est, 'measurement_disposition') && k <= numel(est.measurement_disposition)
    record = est.measurement_disposition{k};
    if isstruct(record) && isfield(record, type) && isfield(record.(type), 'input_dim')
        supplied = record.(type).input_dim;
        assert(numel(supplied) == n && all(ismember(supplied, [0, 2, 3])), ...
            'evaluate_joint_tracking_metrics:InvalidInputDimension', ...
            'Input branch ledger must match the event measurement indices.');
        dims = double(reshape(supplied, 1, []));
    end
end
end

function flow = build_measurement_flow(data)
filter_dims = [0, 2, 3];
M = zeros(2, numel(filter_dims));
for r = 1:2
    meas_dim = r + 1;
    for c = 1:numel(filter_dims)
        M(r, c) = nnz(data.assoc_meas_dim == meas_dim & ...
            data.assoc_filter_dim == filter_dims(c));
    end
end
range_mask = data.assoc_meas_dim == 3;
flow = struct('row_measurement_dims', [2, 3], ...
    'column_filter_dims', filter_dims, 'counts', M, ...
    'n_range_associated', nnz(range_mask), ...
    'n_range_updated', nnz(range_mask & data.assoc_range_updated), ...
    'n_range_associated_without_update', ...
        nnz(range_mask & ~data.assoc_range_updated));
end

function accounting = build_measurement_accounting(est, data)
stats = struct();
if isstruct(est) && isfield(est, 'stats') && isstruct(est.stats)
    stats = est.stats;
end
active_assoc = nnz(data.assoc_is_active);
passive_assoc = nnz(data.assoc_is_passive);
range_assoc = nnz(data.assoc_is_active & data.assoc_meas_dim == 3);
active_suppressed = stat_value(stats, 'active_birth_suppressed', 0);
passive_suppressed = stat_value(stats, 'passive_birth_suppressed', 0);
range_suppressed = stat_value(stats, 'active_range_birth_suppressed', 0);
active_destination = stat_sum_or(stats, {'active_assigned', 'active_births'}, active_assoc);
passive_destination = stat_sum_or(stats, {'passive_assigned', 'passive_births'}, passive_assoc);
range_destination = stat_value(stats, 'active_range_updates', range_assoc);
accounting = struct();
accounting.active = accounting_row(data.n_active_measurements, ...
    active_destination, active_suppressed);
accounting.passive = accounting_row(data.n_passive_measurements, ...
    passive_destination, passive_suppressed);
accounting.active_range = accounting_row(data.n_active_range_measurements, ...
    range_destination, range_suppressed);
accounting.physical_2d = accounting_row(data.n_meas_2d, ...
    nnz(data.assoc_meas_dim == 2), ...
    passive_suppressed + max(active_suppressed - range_suppressed, 0));
accounting.active = attach_destinations(accounting.active, ...
    data.assoc_is_active, data.assoc_filter_dim);
accounting.passive = attach_destinations(accounting.passive, ...
    data.assoc_is_passive, data.assoc_filter_dim);
accounting.active_range = attach_destinations(accounting.active_range, ...
    data.assoc_is_active & data.assoc_meas_dim == 3, data.assoc_filter_dim);
accounting.physical_2d = attach_destinations(accounting.physical_2d, ...
    data.assoc_meas_dim == 2, data.assoc_filter_dim);
accounting.conserved = accounting.active.unaccounted == 0 && ...
    accounting.passive.unaccounted == 0 && ...
    accounting.active_range.unaccounted == 0;
accounting.destinations_complete = accounting.active.destination_gap == 0 && ...
    accounting.passive.destination_gap == 0 && ...
    accounting.active_range.destination_gap == 0;
end

function row = accounting_row(total, associated, suppressed)
row = struct('n_input', total, 'n_associated_or_born', associated, ...
    'utilization_rate', safe_ratio(associated, total), ...
    'n_explicitly_suppressed', suppressed, ...
    'unaccounted', total - associated - suppressed);
end

function row = attach_destinations(row, mask, filter_dim)
row.n_to_2d = nnz(mask & filter_dim == 2);
row.n_to_3d = nnz(mask & filter_dim == 3);
row.n_associated_not_retained = nnz(mask & filter_dim == 0);
row.destination_gap = row.n_associated_or_born - row.n_to_2d - ...
    row.n_to_3d - row.n_associated_not_retained;
end

function details = build_track_details(data, metrics)
ids = unique(data.output_track(isfinite(data.output_track)));
details = repmat(track_detail_template(), numel(ids), 1);
[~, output_group] = ismember(data.output_track, ids);
[~, assoc_group] = ismember(data.assoc_track, ids);
n_id = numel(ids);
n_output = grouped_count(output_group, output_group > 0, n_id);
n_output_2d = grouped_count(output_group, output_group > 0 & data.output_dim == 2, n_id);
n_output_3d = grouped_count(output_group, output_group > 0 & data.output_dim == 3, n_id);
n_assoc = grouped_count(assoc_group, assoc_group > 0, n_id);
n_active_range = grouped_count(assoc_group, assoc_group > 0 & ...
    data.assoc_is_active & data.assoc_meas_dim == 3, n_id);
n_active_angle = grouped_count(assoc_group, assoc_group > 0 & ...
    data.assoc_is_active & data.assoc_meas_dim == 2, n_id);
n_passive = grouped_count(assoc_group, assoc_group > 0 & data.assoc_is_passive, n_id);
n_output_event = zeros(n_id, 1);
valid_event = output_group > 0 & isfinite(data.output_event);
if any(valid_event)
    event_pairs = unique([output_group(valid_event).', data.output_event(valid_event).'], 'rows');
    n_output_event = accumarray(event_pairs(:, 1), 1, [n_id, 1]);
end
start_time = nan(n_id, 1); end_time = nan(n_id, 1);
valid_time = output_group > 0 & isfinite(data.output_t);
if any(valid_time)
    group = output_group(valid_time).'; time = data.output_t(valid_time).';
    start_time = accumarray(group, time, [n_id, 1], @min, NaN);
    end_time = accumarray(group, time, [n_id, 1], @max, NaN);
end
[two_d_detail, three_d_detail, overall_detail] = deal( ...
    build_track_scope_details(metrics.two_d, ids), ...
    build_track_scope_details(metrics.three_d, ids), ...
    build_track_scope_details(metrics.overall, ids));
[az_error, el_error, position_error, angle_track, position_track, los_error] = ...
    output_error_samples(data, 0, data.output_dim > 0, ...
    metrics.overall.accuracy.output_pairs);
[angle_by_track, position_by_track] = grouped_error_metrics( ...
    ids, angle_track, az_error, el_error, position_track, position_error, los_error);
for q = 1:numel(ids)
    d = track_detail_template(); d.track_id = ids(q);
    d.n_output_points = n_output(q); d.n_output_events = n_output_event(q);
    d.n_output_2d = n_output_2d(q); d.n_output_3d = n_output_3d(q);
    if isfinite(start_time(q))
        d.start_time_s = start_time(q); d.end_time_s = end_time(q);
        d.duration_s = d.end_time_s - d.start_time_s;
    end
    d.n_associations = n_assoc(q); d.n_active_range_assoc = n_active_range(q);
    d.n_active_angle_assoc = n_active_angle(q); d.n_passive_assoc = n_passive(q);
    d.two_d = two_d_detail(q); d.three_d = three_d_detail(q);
    d.overall = overall_detail(q);
    a = angle_by_track(q); p = position_by_track(q);
    d.angle_rmse = struct('n', a.n, 'az_deg', a.rmse_az_deg, ...
        'el_deg', a.rmse_el_deg, 'los_deg', a.rmse_los_deg);
    d.position_rmse = struct('n', p.n, 'e_m', p.rmse_e_m, ...
        'n_m', p.rmse_n_m, 'u_m', p.rmse_u_m, ...
        'three_d_m', p.rmse_3d_m);
    details(q) = d;
end
end

function detail = build_track_scope_details(scope, ids)
n_id = numel(ids); detail = repmat(track_scope_detail_template(), n_id, 1);
[applicable, scope_index] = ismember(ids, scope.track_accuracy.track_ids);
pairs = scope.accuracy.output_pairs;
pair_track = reshape([pairs.track_id], 1, []);
[has_pair, pair_index] = ismember(ids, pair_track);
coverage = scope.output_coverage.per_track;
coverage_track = reshape([coverage.track_id], 1, []);
[has_coverage, coverage_index] = ismember(ids, coverage_track);
for q = 1:n_id
    d = track_scope_detail_template(); d.applicable = applicable(q);
    d.min_labeled_assoc = scope.track_accuracy.min_labeled_assoc;
    d.purity_threshold = scope.track_accuracy.purity_threshold;
    if applicable(q)
        c = scope_index(q);
        d.n_labeled_assoc = scope.accuracy.output_labeled_counts(c);
        d.is_correct = scope.track_accuracy.is_correct(c);
    end
    if has_pair(q)
        pair = pairs(pair_index(q)); d.has_match = true;
        d.matched_truth_id = pair.truth_id; d.n_correct = pair.n_correct;
        d.n_inconsistent = max(d.n_labeled_assoc - d.n_correct, 0);
        d.association_consistency = pair.association_consistency;
        d.truth_total = pair.truth_total;
        d.coverage = pair.coverage;
        d.n_truth_not_correct = max(d.truth_total - d.n_correct, 0);
    end
    if has_coverage(q)
        oc = coverage(coverage_index(q)); d.formal_output_coverage = oc.rate;
        d.n_formal_output_covered = oc.n_covered_measurements;
        d.n_formal_output_reference = oc.n_reference_measurements;
    end
    d.track_level_score = double(d.is_correct); detail(q) = d;
end
end

function count = grouped_count(group, mask, n_group)
group = group(mask);
if isempty(group), count = zeros(n_group, 1); return; end
count = accumarray(group(:), 1, [n_group, 1]);
end

function [angle, position] = grouped_error_metrics( ...
        ids, angle_track, az_error, el_error, position_track, position_error, los_error)
n = numel(ids);
angle = repmat(angle_metrics([], [], []), n, 1);
position = repmat(position_metrics(zeros(3, 0)), n, 1);
[found, group] = ismember(angle_track, ids);
if any(found)
    group = group(found); az = az_error(found); el = el_error(found);
    count = accumarray(group(:), 1, [n, 1]);
    az_square = accumarray(group(:), az(:).^2, [n, 1]);
    el_square = accumarray(group(:), el(:).^2, [n, 1]);
    los = los_error(found);
    los_square = accumarray(group(:), los(:).^2, [n, 1]);
    for q = find(count > 0).'
        angle(q).n = count(q); angle(q).rmse_az_deg = sqrt(az_square(q) / count(q));
        angle(q).rmse_el_deg = sqrt(el_square(q) / count(q));
        angle(q).rmse_los_deg = sqrt(los_square(q) / count(q));
    end
end
[found, group] = ismember(position_track, ids);
if any(found)
    group = group(found); error = position_error(:, found);
    count = accumarray(group(:), 1, [n, 1]); sums = zeros(3, n);
    for axis = 1:3
        values = reshape(error(axis, :).^2, [], 1);
        sums(axis, :) = accumarray(group(:), values, [n, 1]).';
    end
    for q = find(count > 0).'
        position(q).n = count(q);
        position(q).rmse_e_m = sqrt(sums(1, q) / count(q));
        position(q).rmse_n_m = sqrt(sums(2, q) / count(q));
        position(q).rmse_u_m = sqrt(sums(3, q) / count(q));
        position(q).rmse_3d_m = sqrt(sum(sums(:, q)) / count(q));
    end
end
end

function d = track_detail_template()
d = struct('track_id', NaN, 'n_output_points', 0, 'n_output_events', 0, ...
    'n_output_2d', 0, 'n_output_3d', 0, 'start_time_s', NaN, ...
    'end_time_s', NaN, 'duration_s', NaN, 'n_associations', 0, ...
    'n_active_range_assoc', 0, 'n_active_angle_assoc', 0, ...
    'n_passive_assoc', 0, 'two_d', track_scope_detail_template(), ...
    'three_d', track_scope_detail_template(), ...
    'overall', track_scope_detail_template(), ...
    'angle_rmse', struct('n', 0, 'az_deg', NaN, 'el_deg', NaN, 'los_deg', NaN), ...
    'position_rmse', struct('n', 0, 'e_m', NaN, 'n_m', NaN, ...
    'u_m', NaN, 'three_d_m', NaN));
end

function d = track_scope_detail_template()
d = struct('applicable', false, 'has_match', false, ...
    'matched_truth_id', NaN, 'n_labeled_assoc', 0, 'n_correct', 0, ...
    'n_inconsistent', 0, 'association_consistency', NaN, ...
    'truth_total', 0, 'coverage', NaN, 'n_truth_not_correct', 0, ...
    'formal_output_coverage', NaN, 'n_formal_output_covered', 0, ...
    'n_formal_output_reference', 0, 'purity_threshold', NaN, ...
    'min_labeled_assoc', 0, 'is_correct', false, 'track_level_score', 0);
end

function dim = snapshot_dimension(tr)
dim = 0;
if isfield(tr, 'mode') && strncmp(tr.mode, '2d', 2)
    dim = 2;
elseif isfield(tr, 'mode') && strncmp(tr.mode, '3d', 2)
    dim = 3;
elseif isfield(tr, 'state3d') && all(isfinite(tr.state3d))
    dim = 3;
elseif isfield(tr, 'angle_state') && all(isfinite(tr.angle_state))
    dim = 2;
end
end

function d = angle_diff(a, b)
d = mod(a - b + 180, 360) - 180;
end

function x = sized_row(x0, n, fill)
x = fill * ones(1, n);
if n == 0 || isempty(x0), return; end
m = min(n, numel(x0));
x(1:m) = reshape(x0(1:m), 1, []);
end

function has_range = measurement_has_range(meas, n)
has_range = false(1, n);
if n == 0 || ~isstruct(meas) || ~isfield(meas, 'has_range') || ...
        isempty(meas.has_range)
    return;
end
m = min(n, numel(meas.has_range));
has_range(1:m) = logical(reshape(meas.has_range(1:m), 1, []));
end

function t = measurement_times(t0, n, fallback)
t = fallback * ones(1, n);
if n == 0 || isempty(t0), return; end
if isscalar(t0)
    t(:) = t0;
else
    m = min(n, numel(t0));
    t(1:m) = reshape(t0(1:m), 1, []);
end
end

function value = indexed_value(x, i, fallback)
if i >= 1 && i <= numel(x), value = x(i); else, value = fallback; end
end

function value = indexed_field_value(s, name, i, fallback)
value = fallback;
if isstruct(s) && isfield(s, name)
    value = indexed_value(s.(name), i, fallback);
end
end

function value = indexed_text(s, name, i, fallback)
value = fallback;
if ~isstruct(s) || ~isfield(s, name), return; end
x = s.(name);
if iscell(x) && i >= 1 && i <= numel(x) && ischar(x{i})
    value = x{i};
elseif ischar(x) && i == 1
    value = x;
end
end

function value = stat_value(stats, name, fallback)
if isstruct(stats) && isfield(stats, name) && isscalar(stats.(name)) && ...
        isfinite(stats.(name))
    value = stats.(name);
else
    value = fallback;
end
end

function value = stat_sum_or(stats, names, fallback)
value = 0;
for q = 1:numel(names)
    name = names{q};
    if ~isstruct(stats) || ~isfield(stats, name) || ...
            ~isscalar(stats.(name)) || ~isfinite(stats.(name))
        value = fallback;
        return;
    end
    value = value + stats.(name);
end
end

function ids = collect_confirmed_ids(est, fallback_output_ids)
ids = zeros(1, 0);

% Current joint-filter results retain transition_log independently of
% history_level. The logical_confirm_* transitions are therefore the
% authoritative source for logical IDs that have ever reached confirmed
% state, including a track confirmed directly into hold without output.
if isstruct(est) && isfield(est, 'transition_log') && ~isempty(est.transition_log)
    log = est.transition_log;
    if isfield(log, 'id') && isfield(log, 'reason')
        reasons = {log.reason};
        is_confirm = false(size(reasons));
        for q = 1:numel(reasons)
            reason = reasons{q};
            is_confirm(q) = ischar(reason) && strncmp(reason, 'logical_confirm_', 16);
        end
        if any(is_confirm)
            ids = reshape([log(is_confirm).id], 1, []);
        end
    end
end

% Compatibility fallback for older or synthetic estimates that do not
% contain confirmation transitions but do retain diagnostic/full snapshots.
if isempty(ids) && isstruct(est) && isfield(est, 'logical_tracks')
    counts = zeros(numel(est.logical_tracks), 1);
    for k = 1:numel(est.logical_tracks)
        tracks = est.logical_tracks{k};
        if isempty(tracks) || ~isfield(tracks, 'id') || ~isfield(tracks, 'confirmed')
            continue;
        end
        counts(k) = nnz(logical([tracks.confirmed]));
    end
    ids = nan(1, sum(counts)); p = 0;
    for k = 1:numel(est.logical_tracks)
        if counts(k) == 0, continue; end
        tracks = est.logical_tracks{k}; confirmed = logical([tracks.confirmed]);
        values = reshape([tracks(confirmed).id], 1, []);
        ii = p + (1:numel(values)); ids(ii) = values; p = p + numel(values);
    end
end
if isempty(ids)
    ids = fallback_output_ids;
end
ids = unique(ids(isfinite(ids)));
end

function n = get_transition_count(est)
if isfield(est, 'transition_log'), n = numel(est.transition_log); else, n = 0; end
end

function v = safe_ratio(a, b)
if b > 0, v = a / b; else, v = NaN; end
end

function v = mean_or_nan(x)
if isempty(x), v = NaN; else, v = mean(x); end
end

function v = get_cfg(cfg, name, fallback)
if isfield(cfg, name) && ~isempty(cfg.(name)), v = cfg.(name); else, v = fallback; end
end

function data = empty_metric_data()
data = struct('n_meas_2d', 0, 'n_meas_3d', 0, ...
    'n_active_measurements', 0, 'n_passive_measurements', 0, ...
    'n_active_range_measurements', 0, ...
    'truth_2d_id', zeros(1, 0), 'truth_2d_t', zeros(1, 0), ...
    'truth_2d_event', zeros(1, 0), 'truth_2d_az', zeros(1, 0), ...
    'truth_2d_el', zeros(1, 0), 'truth_2d_input_dim', zeros(1, 0), ...
    'truth_2d_is_passive', false(1, 0), ...
    'truth_3d_id', zeros(1, 0), 'truth_3d_t', zeros(1, 0), ...
    'truth_3d_event', zeros(1, 0), 'truth_3d_az', zeros(1, 0), ...
    'truth_3d_el', zeros(1, 0), 'truth_3d_xyz', zeros(3, 0), ...
    'assoc_track', zeros(1, 0), 'assoc_truth', zeros(1, 0), ...
    'assoc_meas_dim', zeros(1, 0), 'assoc_filter_dim', zeros(1, 0), ...
    'assoc_input_dim', zeros(1, 0), ...
    'assoc_event', zeros(1, 0), 'assoc_t', zeros(1, 0), ...
    'assoc_is_active', false(1, 0), ...
    'assoc_is_passive', false(1, 0), 'assoc_range_updated', false(1, 0), ...
    'output_track', zeros(1, 0), 'output_truth', zeros(1, 0), ...
    'output_dim', zeros(1, 0), 'output_t', zeros(1, 0), ...
    'output_event', zeros(1, 0), 'output_az', zeros(1, 0), ...
    'output_el', zeros(1, 0), 'output_pos', zeros(3, 0));
end

function data = preallocate_metric_data(n2, n3, na, no, n2_event, n3_event)
data = empty_metric_data();
data.truth_2d_id = nan(1, n2); data.truth_2d_t = nan(1, n2);
data.truth_2d_event = zeros(1, n2); data.truth_2d_az = nan(1, n2);
data.truth_2d_el = nan(1, n2);
data.truth_2d_input_dim = zeros(1, n2); data.truth_2d_is_passive = false(1, n2);
data.truth_3d_id = nan(1, n3); data.truth_3d_t = nan(1, n3);
data.truth_3d_event = zeros(1, n3); data.truth_3d_az = nan(1, n3);
data.truth_3d_el = nan(1, n3); data.truth_3d_xyz = nan(3, n3);
data.truth_2d_offsets = [1; 1 + cumsum(n2_event(:))];
data.truth_3d_offsets = [1; 1 + cumsum(n3_event(:))];
data.assoc_track = nan(1, na); data.assoc_truth = nan(1, na);
data.assoc_meas_dim = zeros(1, na); data.assoc_filter_dim = zeros(1, na);
data.assoc_input_dim = zeros(1, na);
data.assoc_event = zeros(1, na); data.assoc_t = nan(1, na);
data.assoc_is_active = false(1, na);
data.assoc_is_passive = false(1, na); data.assoc_range_updated = false(1, na);
data.output_track = nan(1, no); data.output_truth = nan(1, no);
data.output_dim = zeros(1, no); data.output_t = nan(1, no);
data.output_event = zeros(1, no); data.output_az = nan(1, no);
data.output_el = nan(1, no); data.output_pos = nan(3, no);
end

function e = empty_event()
e = struct('t_sec', NaN, ...
    'active', struct('n_meas', 0, 't_sec', zeros(1, 0), ...
        'ids', zeros(1, 0), 'has_range', false(1, 0), ...
        'rae', zeros(3, 0), 'xyz', zeros(3, 0)), ...
    'passive', struct('n_meas', 0, 't_sec', zeros(1, 0), ...
        'ids', zeros(1, 0), 'ang', zeros(2, 0)));
end
