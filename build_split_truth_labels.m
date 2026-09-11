function [split_ids, info] = build_split_truth_labels(frame_times, fused_xyz, fused_ids, cfg)
%BUILD_SPLIT_TRUTH_LABELS  Build split pseudo-truth instance labels per fused point.
%   split_ids{k}(j) is the split-instance key for fused_xyz{k}(:,j).
%   Splitting uses only distance and time gates, matching the metric logic.

K = min(numel(frame_times), numel(fused_xyz));
split_ids = cell(size(fused_xyz));
for k = 1:numel(fused_xyz)
    M = size(fused_xyz{k}, 2);
    split_ids{k} = nan(1, M);
end

info = struct('enabled', false, 'raw_id_count', 0, 'instance_count', 0, ...
    'n_split_raw_ids', 0, 'key_raw_id', zeros(1, 0), ...
    'key_instance', zeros(1, 0), 'raw_summary', struct([]));

if K == 0 || isempty(fused_ids)
    return;
end

enabled = get_cfg(cfg, 'truth_id_split_enabled', true) ~= 0;
dist_gate = get_cfg(cfg, 'truth_id_split_dist_m', 10000);
gap_gate = get_cfg(cfg, 'truth_id_split_max_gap_s', 30);
info.enabled = enabled;

n = 0;
for k = 1:K
    Z = fused_xyz{k};
    if isempty(Z) || size(Z, 1) < 3, continue; end
    ids = get_ids(fused_ids, k, size(Z, 2));
    good = isfinite(ids) & all(isfinite(Z(1:3, :)), 1);
    n = n + nnz(good);
end
if n == 0
    return;
end

frame = zeros(n, 1);
col = zeros(n, 1);
time = zeros(n, 1);
raw_id = nan(n, 1);
xyz = nan(3, n);
p = 0;
for k = 1:K
    Z = fused_xyz{k};
    if isempty(Z) || size(Z, 1) < 3, continue; end
    ids = get_ids(fused_ids, k, size(Z, 2));
    idx = find(isfinite(ids) & all(isfinite(Z(1:3, :)), 1));
    m = numel(idx);
    if m == 0, continue; end
    ii = p + (1:m);
    frame(ii) = k;
    col(ii) = idx(:);
    time(ii) = frame_times(k);
    raw_id(ii) = ids(idx).';
    xyz(:, ii) = Z(1:3, idx);
    p = p + m;
end
frame = frame(1:p);
col = col(1:p);
time = time(1:p);
raw_id = raw_id(1:p);
xyz = xyz(:, 1:p);

raw_ids = unique(raw_id(isfinite(raw_id))).';
info.raw_id_count = numel(raw_ids);

if ~enabled
    for i = 1:numel(raw_ids)
        idx = find(raw_id == raw_ids(i));
        for q = idx(:).'
            split_ids{frame(q)}(col(q)) = raw_id(q);
        end
    end
    info.instance_count = numel(raw_ids);
    info.key_raw_id = raw_ids;
    info.key_instance = ones(size(raw_ids));
    return;
end

key = nan(numel(raw_id), 1);
inst = nan(numel(raw_id), 1);
next_key = 1;
raw_summary = repmat(struct('raw_id', nan, 'keys', [], 'instances', [], ...
    'n_instances', 0, 'n_points', 0), 0, 1);

for rr = 1:numel(raw_ids)
    rid = raw_ids(rr);
    idx_raw = find(raw_id == rid & all(isfinite(xyz), 1).');
    if isempty(idx_raw), continue; end
    [~, ord] = sortrows([time(idx_raw), frame(idx_raw)]);
    idx_raw = idx_raw(ord);
    frames_raw = unique(frame(idx_raw), 'stable').';

    comp_key = zeros(1, 0);
    comp_inst = zeros(1, 0);
    comp_last_t = zeros(1, 0);
    comp_last_pos = zeros(3, 0);
    next_inst = 1;

    for ff = 1:numel(frames_raw)
        f = frames_raw(ff);
        idx_frame = idx_raw(frame(idx_raw) == f);
        t_now = mean(time(idx_frame));
        [cluster_id, n_cluster] = cluster_points_by_distance(xyz(:, idx_frame).', dist_gate);
        cent = nan(3, n_cluster);
        for cc = 1:n_cluster
            members = idx_frame(cluster_id == cc);
            cent(:, cc) = mean(xyz(:, members), 2);
        end

        assign_comp = zeros(1, n_cluster);
        if ~isempty(comp_key)
            cand = zeros(numel(comp_key) * n_cluster, 3);
            pc = 0;
            for ci = 1:numel(comp_key)
                dt = max(t_now - comp_last_t(ci), 0);
                if dt > gap_gate, continue; end
                for cc = 1:n_cluster
                    d = norm(cent(:, cc) - comp_last_pos(:, ci));
                    if ~isfinite(d) || d > dist_gate, continue; end
                    pc = pc + 1;
                    cand(pc, :) = [d, ci, cc];
                end
            end
            cand = cand(1:pc, :);
            if ~isempty(cand)
                [~, co] = sort(cand(:, 1), 'ascend');
                used_comp = false(1, numel(comp_key));
                used_cluster = false(1, n_cluster);
                for ii = co.'
                    ci = cand(ii, 2);
                    cc = cand(ii, 3);
                    if ~used_comp(ci) && ~used_cluster(cc)
                        assign_comp(cc) = ci;
                        used_comp(ci) = true;
                        used_cluster(cc) = true;
                    end
                end
            end
        end

        for cc = 1:n_cluster
            ci = assign_comp(cc);
            if ci == 0
                ci = numel(comp_key) + 1;
                comp_key(ci) = next_key;
                comp_inst(ci) = next_inst;
                next_key = next_key + 1;
                next_inst = next_inst + 1;
            end
            comp_last_t(ci) = t_now;
            comp_last_pos(:, ci) = cent(:, cc);

            members = idx_frame(cluster_id == cc);
            key(members) = comp_key(ci);
            inst(members) = comp_inst(ci);
        end
    end

    s = struct('raw_id', rid, 'keys', comp_key, 'instances', comp_inst, ...
        'n_instances', numel(comp_key), 'n_points', numel(idx_raw));
    raw_summary(end+1, 1) = s; %#ok<AGROW>
end

for q = 1:numel(key)
    if isfinite(key(q))
        split_ids{frame(q)}(col(q)) = key(q);
    end
end

max_key = max(key(isfinite(key)));
if isempty(max_key) || ~isfinite(max_key)
    return;
end
key_raw_id = nan(1, max_key);
key_instance = nan(1, max_key);
for i = 1:numel(raw_summary)
    for j = 1:numel(raw_summary(i).keys)
        kk = raw_summary(i).keys(j);
        key_raw_id(kk) = raw_summary(i).raw_id;
        key_instance(kk) = raw_summary(i).instances(j);
    end
end
info.instance_count = numel(unique(key(isfinite(key))));
info.n_split_raw_ids = sum([raw_summary.n_instances] > 1);
info.key_raw_id = key_raw_id;
info.key_instance = key_instance;
info.raw_summary = raw_summary;
end

function ids = get_ids(fused_ids, k, M)
ids = nan(1, M);
if k <= numel(fused_ids) && ~isempty(fused_ids{k})
    v = fused_ids{k}(:).';
    q = min(M, numel(v));
    ids(1:q) = v(1:q);
end
end

function [cluster_id, n_cluster] = cluster_points_by_distance(P, dist_gate)
n = size(P, 1);
cluster_id = zeros(n, 1);
n_cluster = 0;
for i = 1:n
    if cluster_id(i) ~= 0, continue; end
    n_cluster = n_cluster + 1;
    cluster_id(i) = n_cluster;
    changed = true;
    while changed
        changed = false;
        members = find(cluster_id == n_cluster);
        for j = 1:n
            if cluster_id(j) ~= 0, continue; end
            d = sqrt(sum((P(members, :) - P(j, :)).^2, 2));
            if any(d <= dist_gate)
                cluster_id(j) = n_cluster;
                changed = true;
            end
        end
    end
end
end

function v = get_cfg(cfg, name, default_value)
if isstruct(cfg) && isfield(cfg, name) && ~isempty(cfg.(name))
    v = cfg.(name);
else
    v = default_value;
end
end
