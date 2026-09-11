function labels = build_joint_truth_labels(events, cfg)
%BUILD_JOINT_TRUTH_LABELS Build evaluation-only split-instance labels.
% target_id is never used by filtering here. Spatial observations define
% split instances; bearing-only observations are mapped to those instances.

K = numel(events);
labels.active = cell(K, 1);
labels.passive = cell(K, 1);
frame_times = nan(K, 1);
spatial_xyz = cell(K, 1);
spatial_ids = cell(K, 1);
all_ids = zeros(1, 0);
all_active_ids = zeros(1, 0);
all_passive_ids = zeros(1, 0);

for k = 1:K
    e = events(k);
    frame_times(k) = e.t_sec;
    na = e.active.n_meas;
    np = e.passive.n_meas;
    active_ids = sized_row(e.active.ids, na, NaN);
    passive_ids = sized_row(e.passive.ids, np, NaN);
    passive_kind = sized_row(field_or(e.passive, 'kind', ones(1, np)), np, 1);
    active_angle = passive_kind == 2;
    physical_passive = passive_kind == 1;
    all_ids = [all_ids, active_ids(isfinite(active_ids)), ... %#ok<AGROW>
        passive_ids(isfinite(passive_ids))]; %#ok<AGROW>
    all_active_ids = [all_active_ids, ...
        active_ids(isfinite(active_ids)), ...
        passive_ids(active_angle & isfinite(passive_ids))]; %#ok<AGROW>
    all_passive_ids = [all_passive_ids, ...
        passive_ids(physical_passive & isfinite(passive_ids))]; %#ok<AGROW>

    Z = nan(3, na);
    n_xyz = min(na, size(e.active.xyz, 2));
    if n_xyz > 0 && size(e.active.xyz, 1) >= 3
        Z(:, 1:n_xyz) = e.active.xyz(1:3, 1:n_xyz);
    end
    has_range = sized_logical(e.active.has_range, na);
    Z(:, ~has_range) = NaN;
    spatial_xyz{k} = Z;
    spatial_ids{k} = active_ids;
end

all_ids = unique(all_ids(isfinite(all_ids)));
all_active_ids = unique(all_active_ids(isfinite(all_active_ids)));
all_passive_ids = unique(all_passive_ids(isfinite(all_passive_ids)));
split_enabled = get_cfg(cfg, 'truth_id_split_enabled', true) ~= 0;
[spatial_labels, spatial_info] = build_split_truth_labels( ...
    frame_times, spatial_xyz, spatial_ids, cfg);

if ~split_enabled
    for k = 1:K
        labels.active{k} = sized_row(events(k).active.ids, events(k).active.n_meas, NaN);
        labels.passive{k} = sized_row(events(k).passive.ids, events(k).passive.n_meas, NaN);
    end
    spatial_raw_ids = collect_spatial_raw_ids(spatial_ids, spatial_xyz);
    labels.summary = make_summary(spatial_info, all_ids, spatial_raw_ids, ...
        setdiff(all_ids, spatial_raw_ids), all_active_ids, ...
        all_passive_ids, cfg, false);
    labels.summary.instance_labels = all_ids;
    labels.summary.key_raw_id = all_ids;
    labels.summary.key_instance = ones(size(all_ids));
    labels.summary.instance_label_text = make_label_text(all_ids, ...
        all_ids, ones(size(all_ids)), false);
    return;
end

spatial_raw_ids = unique(spatial_info.key_raw_id( ...
    isfinite(spatial_info.key_raw_id)));
angle_only_ids = setdiff(all_ids, spatial_raw_ids);
summary = make_summary(spatial_info, all_ids, spatial_raw_ids, ...
    angle_only_ids, all_active_ids, all_passive_ids, cfg, true);

next_key = spatial_info.instance_count + 1;
key_raw_id = spatial_info.key_raw_id;
key_instance = spatial_info.key_instance;
for i = 1:numel(angle_only_ids)
    key_raw_id(next_key) = angle_only_ids(i);
    key_instance(next_key) = 1;
    next_key = next_key + 1;
end
summary.key_raw_id = key_raw_id;
summary.key_instance = key_instance;
summary.instance_count = numel(key_raw_id);
summary.instance_labels = 1:summary.instance_count;
summary.instance_label_text = make_label_text(summary.instance_labels, ...
    key_raw_id, key_instance, true);

[ref_time, ref_angle] = collect_spatial_references(events, spatial_labels, summary.instance_count);
time_scale = max(get_cfg(cfg, 'truth_id_split_max_gap_s', 30), 1e-3);
if ~get_cfg(cfg, 'truth_cross_sensor_id_consistent', false)
    [labels.active, active_angle_labels, reference_info] = ...
        build_geometric_active_references( ...
        events, spatial_labels, spatial_info, spatial_raw_ids, ...
        ref_time, ref_angle, time_scale, cfg);
    [reference_time, reference_angle] = collect_active_references( ...
        events, labels.active, active_angle_labels, reference_info.instance_count);
    [labels.passive, geometric] = assign_passive_geometric_labels( ...
        events, active_angle_labels, reference_time, reference_angle, ...
        reference_info.instance_count, cfg);
    labels.summary = make_geometric_summary( ...
        reference_info, spatial_raw_ids, all_active_ids, all_passive_ids, ...
        geometric, cfg);
    return;
end

for k = 1:K
    e = events(k);
    na = e.active.n_meas;
    np = e.passive.n_meas;
    active_label = sized_row(spatial_labels{k}, na, NaN);
    active_ids = sized_row(e.active.ids, na, NaN);
    active_t = measurement_times(e.active.t_sec, na, e.t_sec);
    active_ang = sized_matrix(e.active.rae, 2:3, na);
    missing = ~isfinite(active_label) & isfinite(active_ids);
    active_label(missing) = assign_bearing_labels(active_ids(missing), ...
        active_t(missing), active_ang(:, missing), key_raw_id, ...
        ref_time, ref_angle, time_scale);
    labels.active{k} = active_label;

    passive_ids = sized_row(e.passive.ids, np, NaN);
    passive_t = measurement_times(e.passive.t_sec, np, e.t_sec);
    passive_ang = sized_matrix(e.passive.ang, 1:2, np);
    labels.passive{k} = assign_bearing_labels(passive_ids, passive_t, ...
        passive_ang, key_raw_id, ref_time, ref_angle, time_scale);
end

labels.summary = summary;
end

function [active_labels, active_angle_labels, info] = build_geometric_active_references( ...
        events, spatial_labels, spatial_info, spatial_raw_ids, ...
        ref_time, ref_angle, time_scale, cfg)
K = numel(events);
active_labels = cell(K, 1);
active_angle_labels = cell(K, 1);
n_total = sum(arrayfun(@(e) e.active.n_meas + e.passive.n_meas, events));
rec_event = zeros(1, n_total); rec_index = zeros(1, n_total);
rec_container = zeros(1, n_total);
raw = nan(1, n_total); times = nan(1, n_total);
p = 0;
for k = 1:K
    e = events(k); na = e.active.n_meas; np = e.passive.n_meas;
    active_label = sized_row(spatial_labels{k}, na, NaN);
    active_ids = sized_row(e.active.ids, na, NaN);
    active_t = measurement_times(e.active.t_sec, na, e.t_sec);
    active_ang = sized_matrix(e.active.rae, 2:3, na);
    missing = ~isfinite(active_label) & isfinite(active_ids);
    active_label(missing) = assign_bearing_labels(active_ids(missing), ...
        active_t(missing), active_ang(:, missing), spatial_info.key_raw_id, ...
        ref_time, ref_angle, time_scale);
    active_labels{k} = active_label;

    missing = find(~isfinite(active_label) & isfinite(active_ids) & ...
        isfinite(active_t) & all(isfinite(active_ang), 1));
    if ~isempty(missing)
        ii = p + (1:numel(missing));
        rec_event(ii) = k; rec_index(ii) = missing; rec_container(ii) = 1;
        raw(ii) = active_ids(missing); times(ii) = active_t(missing);
        p = p + numel(missing);
    end

    angle_label = nan(1, np);
    kind = sized_row(field_or(e.passive, 'kind', ones(1, np)), np, 1);
    active_idx = find(kind == 2);
    if ~isempty(active_idx)
        ids = sized_row(e.passive.ids, np, NaN);
        tt = measurement_times(e.passive.t_sec, np, e.t_sec);
        ang = sized_matrix(e.passive.ang, 1:2, np);
        angle_label(active_idx) = assign_bearing_labels(ids(active_idx), ...
            tt(active_idx), ang(:, active_idx), spatial_info.key_raw_id, ...
            ref_time, ref_angle, time_scale);
        missing = active_idx(~isfinite(angle_label(active_idx)) & ...
            isfinite(ids(active_idx)) & isfinite(tt(active_idx)) & ...
            all(isfinite(ang(:, active_idx)), 1));
        if ~isempty(missing)
            ii = p + (1:numel(missing));
            rec_event(ii) = k; rec_index(ii) = missing; rec_container(ii) = 2;
            raw(ii) = ids(missing); times(ii) = tt(missing);
            p = p + numel(missing);
        end
    end
    active_angle_labels{k} = angle_label;
end
rec_event = rec_event(1:p); rec_index = rec_index(1:p);
rec_container = rec_container(1:p);
raw = raw(1:p); times = times(1:p);

info = spatial_info;
info.n_spatial_instances = spatial_info.instance_count;
raw_ids = unique(raw(isfinite(raw)));
info.active_angle_only_raw_ids = raw_ids;
max_gap = get_cfg(cfg, 'truth_id_split_max_gap_s', 30);
n_split = 0;
for rid = reshape(raw_ids, 1, [])
    jj = find(raw == rid);
    [~, order] = sort(times(jj)); jj = jj(order);
    cuts = [1, find(diff(times(jj)) > max_gap) + 1, numel(jj) + 1];
    n_part = numel(cuts) - 1;
    n_split = n_split + (n_part > 1);
    for part = 1:n_part
        records = jj(cuts(part):cuts(part + 1) - 1);
        info.instance_count = info.instance_count + 1;
        info.key_raw_id(end + 1) = rid;
        info.key_instance(end + 1) = part;
        key = info.instance_count;
        for q = reshape(records, 1, [])
            if rec_container(q) == 1
                active_labels{rec_event(q)}(rec_index(q)) = key;
            else
                active_angle_labels{rec_event(q)}(rec_index(q)) = key;
            end
        end
    end
end
info.n_active_angle_only_split_raw_ids = n_split;
info.n_split_raw_ids = info.n_split_raw_ids + n_split;
info.n_spatial_raw_ids = numel(spatial_raw_ids);
end

function [passive_labels, info] = assign_passive_geometric_labels( ...
        events, active_angle_labels, ref_time, ref_angle, n_active, cfg)
K = numel(events);
passive_labels = active_angle_labels;
n_total = sum(arrayfun(@(e) e.passive.n_meas, events));
rec_event = zeros(1, n_total); rec_index = zeros(1, n_total);
raw = nan(1, n_total); times = nan(1, n_total); angles = nan(2, n_total);
p = 0;
for k = 1:K
    n = events(k).passive.n_meas;
    if n == 0, continue; end
    kind = sized_row(field_or(events(k).passive, 'kind', ones(1, n)), n, 1);
    physical = find(kind == 1);
    if isempty(physical), continue; end
    ii = p + (1:numel(physical));
    rec_event(ii) = k; rec_index(ii) = physical;
    ids = sized_row(events(k).passive.ids, n, NaN);
    tt = measurement_times(events(k).passive.t_sec, n, events(k).t_sec);
    ang = sized_matrix(events(k).passive.ang, 1:2, n);
    raw(ii) = ids(physical); times(ii) = tt(physical);
    angles(:, ii) = ang(:, physical);
    p = p + numel(physical);
end
rec_event = rec_event(1:p); rec_index = rec_index(1:p);
raw = raw(1:p); times = times(1:p); angles = angles(:, 1:p);

segments = struct('raw_id', {}, 'instance', {}, 'records', {});
max_gap = get_cfg(cfg, 'truth_id_split_max_gap_s', 30);
raw_ids = unique(raw(isfinite(raw)));
for rid = reshape(raw_ids, 1, [])
    jj = find(raw == rid & isfinite(times) & all(isfinite(angles), 1));
    [~, order] = sort(times(jj)); jj = jj(order);
    if isempty(jj), continue; end
    cuts = [1, find(diff(times(jj)) > max_gap) + 1, numel(jj) + 1];
    for part = 1:numel(cuts) - 1
        records = jj(cuts(part):cuts(part + 1) - 1);
        segments(end + 1) = struct('raw_id', rid, ...
            'instance', part, 'records', records); %#ok<AGROW>
    end
end

gate = get_cfg(cfg, 'truth_cross_sensor_match_angle_deg', 1.0);
max_dt = get_cfg(cfg, 'truth_cross_sensor_match_max_dt_s', 0.5);
min_points = max(1, round(get_cfg(cfg, ...
    'truth_cross_sensor_match_min_points', 3)));
min_ratio = get_cfg(cfg, 'truth_cross_sensor_match_min_ratio', 0.20);
ambiguity = get_cfg(cfg, 'truth_cross_sensor_match_ambiguity_deg', 0.10);
max_samples = max(1, round(get_cfg(cfg, ...
    'truth_cross_sensor_match_max_samples', 200)));
next_key = n_active + 1;
matched_segments = 0;
unmatched_raw = zeros(1, 0); unmatched_instance = zeros(1, 0);
segment_keys = cell(1, numel(segments));

for s = 1:numel(segments)
    jj = segments(s).records;
    if numel(jj) > max_samples
        sample_pos = unique(round(linspace(1, numel(jj), max_samples)));
        sample = jj(sample_pos);
    else
        sample = jj;
    end
    candidate = zeros(1, numel(sample));
    for q = 1:numel(sample)
        costs = active_reference_costs(times(sample(q)), angles(:, sample(q)), ...
            ref_time, ref_angle, max_dt, gate);
        [sorted, order] = sort(costs);
        if isempty(sorted) || ~isfinite(sorted(1)), continue; end
        if numel(sorted) > 1 && isfinite(sorted(2)) && ...
                sorted(2) - sorted(1) < ambiguity
            continue;
        end
        candidate(q) = order(1);
    end
    accepted = zeros(1, 0);
    required = max(min_points, ceil(min_ratio * numel(sample)));
    for key = unique(candidate(candidate > 0))
        if nnz(candidate == key) >= required
            accepted(end + 1) = key; %#ok<AGROW>
        end
    end
    if isempty(accepted)
        key = next_key; next_key = next_key + 1;
        assigned = repmat(key, 1, numel(jj));
        unmatched_raw(end + 1) = segments(s).raw_id; %#ok<AGROW>
        unmatched_instance(end + 1) = segments(s).instance; %#ok<AGROW>
        segment_keys{s} = key;
    else
        matched_segments = matched_segments + 1;
        assigned = zeros(1, numel(jj));
        for q = 1:numel(jj)
            costs = active_reference_costs(times(jj(q)), angles(:, jj(q)), ...
                ref_time(accepted), ref_angle(accepted), inf, inf);
            [~, local] = min(costs);
            assigned(q) = accepted(local);
        end
        segment_keys{s} = unique(assigned);
    end
    for q = 1:numel(jj)
        passive_labels{rec_event(jj(q))}(rec_index(jj(q))) = assigned(q);
    end
end

split_raw = 0;
for rid = reshape(raw_ids, 1, [])
    split_raw = split_raw + (nnz([segments.raw_id] == rid) > 1);
end
info = struct('raw_ids', raw_ids, 'n_segments', numel(segments), ...
    'n_split_raw_ids', split_raw, 'n_matched_segments', matched_segments, ...
    'n_unmatched_segments', numel(unmatched_raw), ...
    'unmatched_raw_ids', unmatched_raw, ...
    'unmatched_instances', unmatched_instance, ...
    'segment_keys', {segment_keys});
end

function costs = active_reference_costs(t, angle, ref_time, ref_angle, max_dt, gate)
costs = inf(1, numel(ref_time));
if ~isfinite(t) || any(~isfinite(angle)), return; end
for key = 1:numel(ref_time)
    if isempty(ref_time{key}), continue; end
    j = nearest_time_index(ref_time{key}, t);
    dt = abs(ref_time{key}(j) - t);
    da = angle_diff(angle(1), ref_angle{key}(1, j));
    de = angle(2) - ref_angle{key}(2, j);
    sep = hypot(da, de);
    if dt <= max_dt && sep <= gate, costs(key) = sep; end
end
end

function summary = make_geometric_summary(reference_info, spatial_raw_ids, ...
        active_raw_ids, passive_raw_ids, info, cfg)
summary = reference_info;
if isfield(reference_info, 'active_angle_only_raw_ids')
    active_angle_only_ids = reference_info.active_angle_only_raw_ids;
else
    active_angle_only_ids = zeros(1, 0);
end
summary.enabled = true;
summary.raw_id_count = numel(active_raw_ids) + numel(passive_raw_ids);
summary.instance_count = reference_info.instance_count + info.n_unmatched_segments;
% Only active truth has split-instance semantics. Passive observations are
% segmented solely for geometric mapping and must not inflate split counts.
summary.n_split_raw_ids = reference_info.n_split_raw_ids;
summary.n_active_raw_targets = numel(active_raw_ids);
summary.n_active_split_instances = reference_info.instance_count;
summary.n_active_split_raw_ids = reference_info.n_split_raw_ids;
summary.n_passive_raw_targets = numel(passive_raw_ids);
summary.n_passive_mapping_segments = info.n_segments;
summary.n_passive_segmented_raw_ids = info.n_split_raw_ids;
summary.n_unified_truth_instances = summary.instance_count;
summary.n_spatial_raw_ids = numel(spatial_raw_ids);
summary.n_active_angle_only_raw_ids = numel(active_angle_only_ids);
summary.n_angle_only_raw_ids = numel(active_angle_only_ids) + ...
    numel(unique(info.unmatched_raw_ids));
summary.raw_ids = [active_raw_ids, passive_raw_ids];
summary.spatial_raw_ids = spatial_raw_ids;
summary.angle_only_raw_ids = [active_angle_only_ids, ...
    unique(info.unmatched_raw_ids)];
summary.dist_gate_m = get_cfg(cfg, 'truth_id_split_dist_m', 10000);
summary.max_gap_s = get_cfg(cfg, 'truth_id_split_max_gap_s', 30);
summary.assoc_map_max_dist_m = get_cfg(cfg, 'truth_assoc_map_max_dist_m', inf);
summary.label_basis = 'geometric_cross_sensor_split_truth';
summary.cross_sensor_id_consistent = false;
summary.cross_sensor_matched_segments = info.n_matched_segments;
summary.cross_sensor_unmatched_segments = info.n_unmatched_segments;
summary.key_raw_id = [reference_info.key_raw_id, info.unmatched_raw_ids];
summary.key_instance = [reference_info.key_instance, info.unmatched_instances];
summary.instance_labels = 1:summary.instance_count;
active_angle_keys = reference_info.n_spatial_instances + 1:reference_info.instance_count;
passive_only_keys = reference_info.instance_count + (1:info.n_unmatched_segments);
summary.active_angle_only_instance_labels = active_angle_keys;
summary.passive_only_instance_labels = passive_only_keys;
summary.angle_only_instance_labels = unique([active_angle_keys, passive_only_keys]);
text = make_label_text(1:reference_info.instance_count, ...
    reference_info.key_raw_id, reference_info.key_instance, true);
for q = 1:info.n_unmatched_segments
    key = reference_info.instance_count + q;
    text{end + 1} = sprintf('%g(passive_raw=%g,part=%g)', ...
        key, info.unmatched_raw_ids(q), info.unmatched_instances(q)); %#ok<AGROW>
end
summary.instance_label_text = text;
end

function summary = make_summary(spatial_info, all_ids, spatial_ids, angle_ids, ...
        active_ids, passive_ids, cfg, enabled)
summary = spatial_info;
summary.enabled = enabled;
summary.raw_id_count = numel(all_ids);
if enabled
    summary.instance_count = spatial_info.instance_count + numel(angle_ids);
else
    summary.instance_count = numel(all_ids);
    summary.n_split_raw_ids = 0;
end
summary.n_active_raw_targets = numel(active_ids);
if enabled
    spatial_active = ismember(spatial_info.key_raw_id, active_ids);
    active_angle_ids = setdiff(active_ids, spatial_ids);
    summary.n_active_split_instances = nnz(spatial_active) + ...
        numel(active_angle_ids);
    summary.n_active_split_raw_ids = spatial_info.n_split_raw_ids;
else
    summary.n_active_split_instances = numel(active_ids);
    summary.n_active_split_raw_ids = 0;
end
summary.n_passive_raw_targets = numel(passive_ids);
summary.n_passive_mapping_segments = 0;
summary.n_passive_segmented_raw_ids = 0;
summary.n_unified_truth_instances = summary.instance_count;
summary.n_spatial_raw_ids = numel(spatial_ids);
summary.n_angle_only_raw_ids = numel(angle_ids);
summary.raw_ids = all_ids;
summary.spatial_raw_ids = spatial_ids;
summary.angle_only_raw_ids = angle_ids;
summary.dist_gate_m = get_cfg(cfg, 'truth_id_split_dist_m', 10000);
summary.max_gap_s = get_cfg(cfg, 'truth_id_split_max_gap_s', 30);
summary.assoc_map_max_dist_m = get_cfg(cfg, 'truth_assoc_map_max_dist_m', inf);
summary.label_basis = 'split_truth_instance';
if enabled
    angle_keys = spatial_info.instance_count + (1:numel(angle_ids));
    summary.active_angle_only_instance_labels = angle_keys( ...
        ismember(angle_ids, setdiff(active_ids, spatial_ids)));
    summary.passive_only_instance_labels = angle_keys( ...
        ismember(angle_ids, setdiff(passive_ids, active_ids)));
    summary.angle_only_instance_labels = angle_keys;
else
    summary.active_angle_only_instance_labels = setdiff(active_ids, spatial_ids);
    summary.passive_only_instance_labels = setdiff(passive_ids, active_ids);
    summary.angle_only_instance_labels = angle_ids;
end
end

function raw_ids = collect_spatial_raw_ids(ids, xyz)
raw_ids = zeros(1, 0);
for k = 1:numel(ids)
    id = sized_row(ids{k}, size(xyz{k}, 2), NaN);
    good = isfinite(id) & all(isfinite(xyz{k}), 1);
    raw_ids = [raw_ids, id(good)]; %#ok<AGROW>
end
raw_ids = unique(raw_ids);
end

function [ref_time, ref_angle] = collect_spatial_references(events, split_labels, n_key)
ref_time = cell(1, n_key);
ref_angle = cell(1, n_key);
for k = 1:numel(events)
    e = events(k);
    na = e.active.n_meas;
    keys = sized_row(split_labels{k}, na, NaN);
    times = measurement_times(e.active.t_sec, na, e.t_sec);
    angles = sized_matrix(e.active.rae, 2:3, na);
    for j = find(isfinite(keys) & keys >= 1 & keys <= n_key)
        key = round(keys(j));
        ref_time{key}(end + 1) = times(j);
        ref_angle{key}(:, end + 1) = angles(:, j);
    end
end
for key = 1:n_key
    good = isfinite(ref_time{key}) & all(isfinite(ref_angle{key}), 1);
    ref_time{key} = ref_time{key}(good);
    ref_angle{key} = ref_angle{key}(:, good);
    [ref_time{key}, order] = sort(ref_time{key});
    ref_angle{key} = ref_angle{key}(:, order);
end
end

function [ref_time, ref_angle] = collect_active_references( ...
        events, active_labels, active_angle_labels, n_key)
[ref_time, ref_angle] = collect_spatial_references(events, active_labels, n_key);
for k = 1:numel(events)
    e = events(k); n = e.passive.n_meas;
    keys = sized_row(active_angle_labels{k}, n, NaN);
    times = measurement_times(e.passive.t_sec, n, e.t_sec);
    angles = sized_matrix(e.passive.ang, 1:2, n);
    kind = sized_row(field_or(e.passive, 'kind', ones(1, n)), n, 1);
    for j = find(kind == 2 & isfinite(keys) & keys >= 1 & keys <= n_key)
        key = round(keys(j));
        ref_time{key}(end + 1) = times(j);
        ref_angle{key}(:, end + 1) = angles(:, j);
    end
end
for key = 1:n_key
    good = isfinite(ref_time{key}) & all(isfinite(ref_angle{key}), 1);
    ref_time{key} = ref_time{key}(good);
    ref_angle{key} = ref_angle{key}(:, good);
    [ref_time{key}, order] = sort(ref_time{key});
    ref_angle{key} = ref_angle{key}(:, order);
end
end

function result = assign_bearing_labels(raw_ids, times, angles, key_raw_id, ...
        ref_time, ref_angle, time_scale)
n = numel(raw_ids);
result = nan(1, n);
for rid = unique(raw_ids(isfinite(raw_ids)))
    meas_idx = find(raw_ids == rid);
    keys = find(key_raw_id == rid);
    if isempty(keys), continue; end
    if numel(keys) == 1
        result(meas_idx) = keys;
        continue;
    end

    C = inf(numel(meas_idx), numel(keys));
    for i = 1:numel(meas_idx)
        for j = 1:numel(keys)
            C(i, j) = reference_cost(times(meas_idx(i)), angles(:, meas_idx(i)), ...
                ref_time{keys(j)}, ref_angle{keys(j)}, time_scale);
        end
    end
    finite_cost = C(isfinite(C));
    if isempty(finite_cost)
        continue;
    end
    pairs = solve_global_assignment(C, max(finite_cost) + 1);
    assigned = false(1, numel(meas_idx));
    for q = 1:size(pairs, 1)
        result(meas_idx(pairs(q, 1))) = keys(pairs(q, 2));
        assigned(pairs(q, 1)) = true;
    end
    for i = find(~assigned)
        [best, j] = min(C(i, :));
        if isfinite(best), result(meas_idx(i)) = keys(j); end
    end
end
end

function cost = reference_cost(t, angle, times, angles, time_scale)
cost = inf;
if ~isfinite(t) || any(~isfinite(angle)) || isempty(times), return; end
j = nearest_time_index(times, t);
nearest_dt = abs(times(j) - t);
interval_gap = max([times(1) - t, t - times(end), 0]);
angle_gap = hypot(angle_diff(angle(1), angles(1, j)), angle(2) - angles(2, j));
cost = (interval_gap / time_scale)^2 + angle_gap^2 + ...
    1e-6 * (nearest_dt / time_scale)^2;
end

function j = nearest_time_index(times, t)
lo = 1;
hi = numel(times);
while lo < hi
    mid = floor((lo + hi) / 2);
    if times(mid) < t
        lo = mid + 1;
    else
        hi = mid;
    end
end
j = lo;
if j > 1 && abs(times(j - 1) - t) <= abs(times(j) - t)
    j = j - 1;
end
end

function text = make_label_text(labels, raw_ids, instances, split_enabled)
text = cell(1, numel(labels));
for i = 1:numel(labels)
    if split_enabled
        text{i} = sprintf('%g(raw=%g,part=%g)', ...
            labels(i), raw_ids(i), instances(i));
    else
        text{i} = sprintf('%g', labels(i));
    end
end
end

function A = sized_matrix(A0, rows, n)
A = nan(numel(rows), n);
if isempty(A0) || n == 0 || size(A0, 1) < max(rows), return; end
m = min(n, size(A0, 2));
A(:, 1:m) = A0(rows, 1:m);
end

function x = sized_row(x0, n, fill)
x = fill * ones(1, n);
if n == 0 || isempty(x0), return; end
m = min(n, numel(x0));
x(1:m) = reshape(x0(1:m), 1, []);
end

function x = sized_logical(x0, n)
x = false(1, n);
if n == 0 || isempty(x0), return; end
m = min(n, numel(x0));
x(1:m) = logical(x0(1:m));
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

function d = angle_diff(a, b)
d = mod(a - b + 180, 360) - 180;
end

function v = get_cfg(cfg, name, fallback)
if isstruct(cfg) && isfield(cfg, name) && ~isempty(cfg.(name))
    v = cfg.(name);
else
    v = fallback;
end
end

function v = field_or(s, name, fallback)
if isstruct(s) && isfield(s, name) && ~isempty(s.(name))
    v = s.(name);
else
    v = fallback;
end
end
