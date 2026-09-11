function metrics = evaluate_fusion_quant_metrics(frames, fused_xyz, fused_ids, est, frame_times, cfg)
%EVALUATE_FUSION_QUANT_METRICS  融合跟踪统一定量评价。
%
% 指标：
%   1) 滤波关联率：滤波器成功归属的点数 / 滤波输入点数；确认航迹口径
%      按最终曾确认的航迹ID回溯包含其出生/试探阶段的合规关联。
%   2) 关联点一致率；航迹级正确率分别以确认输出航迹数、拆分实例数为分母。
%   3) 航迹起始延迟：最早有效确认碎片、主要一对一航迹分别统计。
%   4) 跟踪RMS：估计航迹相对离线RTS平滑后的拆分伪真值实例统计。

if nargin < 1, frames = []; end
if nargin < 2, fused_xyz = {}; end
if nargin < 3, fused_ids = {}; end
if nargin < 4, est = struct(); end
if nargin < 5, frame_times = []; end
if nargin < 6 || isempty(cfg), cfg = struct(); end

params = metric_params(cfg);

metrics = struct();
metrics.status = 'ok';
metrics.params = params;
metrics.association = compute_filter_association(fused_xyz, est);
metrics.track_timeout = collect_track_timeout_stats(est);

truth = collect_truth_points(frames, fused_xyz, fused_ids, frame_times);
assoc = collect_assoc_events(est, frame_times);

metrics.truth_source = truth.source;
metrics.n_truth_points = numel(truth.raw_id);
metrics.n_assoc_events = numel(assoc.track_id);

if isempty(truth.raw_id)
    metrics.status = 'no_labeled_truth_points';
    metrics.accuracy_raw = empty_accuracy('raw_target_id');
    metrics.accuracy_split = empty_accuracy('split_target_id');
    metrics.id_split = empty_split_result();
    metrics.truth_tracks = empty_truth_tracks();
    metrics.est_tracks = collect_est_tracks(est, frame_times);
    output_ids = [metrics.est_tracks.id];
    metrics.track_accuracy_raw = compute_track_level_accuracy( ...
        metrics.accuracy_raw, params, output_ids, 0);
    metrics.track_accuracy_split = compute_track_level_accuracy( ...
        metrics.accuracy_split, params, output_ids, 0);
    metrics.start_time = empty_start_metrics();
    metrics.rms = empty_rms_metrics();
    print_metrics_report(metrics);
    return;
end

split_result = split_truth_ids(truth, params);
truth_split = split_result.truth;
assoc = attach_split_keys_to_assoc(assoc, truth_split, params);

metrics.id_split = split_result.summary;
metrics.accuracy_raw = compute_accuracy(assoc.track_id, assoc.raw_id, ...
    'raw_target_id', truth.raw_id);
metrics.accuracy_split = compute_accuracy(assoc.track_id, assoc.split_key, ...
    'split_target_id', truth_split.key);

truth_tracks = build_truth_tracks(truth_split, params);
est_tracks = collect_est_tracks(est, frame_times);
metrics.truth_tracks = truth_tracks;
metrics.est_tracks = est_tracks;
output_ids = [est_tracks.id];
metrics.track_accuracy_raw = compute_track_level_accuracy(metrics.accuracy_raw, params, ...
    output_ids, get_field(metrics.id_split, 'raw_id_count', 0));
metrics.track_accuracy_split = compute_track_level_accuracy(metrics.accuracy_split, params, ...
    output_ids, get_field(metrics.id_split, 'instance_count', 0));
metrics.start_time = compute_start_time_metrics(truth_tracks, est_tracks, ...
    metrics.track_accuracy_split, metrics.accuracy_split);
metrics.rms = compute_rms_metrics(truth_tracks, est_tracks, ...
    metrics.track_accuracy_split, params);

print_metrics_report(metrics);
end

function params = metric_params(cfg)
frame_dt = get_cfg(cfg, 'frame_time_window_s', 0.1);
params.enabled = get_cfg(cfg, 'metrics_enabled', true) ~= 0;
params.truth_id_split_enabled = get_cfg(cfg, 'truth_id_split_enabled', true) ~= 0;
params.truth_id_split_dist_m = get_cfg(cfg, 'truth_id_split_dist_m', 10000);
params.truth_id_split_max_gap_s = get_cfg(cfg, 'truth_id_split_max_gap_s', 30);
params.truth_assoc_map_max_dist_m = get_cfg(cfg, 'truth_assoc_map_max_dist_m', inf);
params.truth_rts_meas_std_m = get_cfg(cfg, 'truth_rts_meas_std_m', get_cfg(cfg, 'smooth_meas_std', 30));
params.truth_rts_proc_std_mps2 = get_cfg(cfg, 'truth_rts_proc_std_mps2', get_cfg(cfg, 'smooth_proc_std', 3));
params.rms_time_tolerance_s = get_cfg(cfg, 'truth_rms_time_tolerance_s', max(frame_dt, 1e-3));
params.max_print = max(1, round(get_cfg(cfg, 'metrics_max_print', 12)));
params.track_accuracy_purity_th = get_cfg(cfg, 'track_accuracy_purity_th', 0.80);
params.track_accuracy_min_assoc = max(1, round(get_cfg(cfg, 'track_accuracy_min_assoc', 3)));
end

function v = get_cfg(cfg, name, default_value)
if isstruct(cfg) && isfield(cfg, name) && ~isempty(cfg.(name))
    v = cfg.(name);
else
    v = default_value;
end
end

function stats = collect_track_timeout_stats(est)
stats = struct('enabled', false, 'tentative_max_silence_s', nan, ...
    'confirmed_max_silence_s', nan, 'n_tentative', 0, ...
    'n_confirmed', 0, 'n_total', 0, 'enforcement', '', 'records', []);
if isstruct(est) && isfield(est, 'track_timeout_stats') && ...
        isstruct(est.track_timeout_stats)
    src = est.track_timeout_stats;
    names = fieldnames(stats);
    for i = 1:numel(names)
        name = names{i};
        if isfield(src, name)
            stats.(name) = src.(name);
        end
    end
end
end

function assoc = compute_filter_association(fused_xyz, est)
n_filter_points = 0;
for k = 1:numel(fused_xyz)
    n_filter_points = n_filter_points + size(fused_xyz{k}, 2);
end

n_assoc_all = 0;
n_assoc_confirmed = 0;
if isstruct(est) && isfield(est, 'assoc') && ~isempty(est.assoc)
    % “确认航迹关联点”按最终曾进入确认输出的航迹ID回溯统计，包含其出生和
    % 试探阶段用于起批的合规关联；确认时刻本身仍由 est.L/est.X 首次输出决定。
    confirmed_track_ids = zeros(0, 1);
    if isfield(est, 'L') && ~isempty(est.L)
        for k = 1:numel(est.L)
            if ~isempty(est.L{k}) && size(est.L{k}, 2) >= 2
                confirmed_track_ids = [confirmed_track_ids; est.L{k}(:, 2)]; %#ok<AGROW>
            end
        end
        confirmed_track_ids = unique(confirmed_track_ids(isfinite(confirmed_track_ids)));
    end
    K = numel(est.assoc);
    for k = 1:K
        As = est.assoc{k};
        if ~isstruct(As) || ~isfield(As, 'id') || isempty(As.id)
            continue;
        end
        ids = As.id(:);
        n_assoc_all = n_assoc_all + numel(ids);
        if ~isempty(confirmed_track_ids)
            n_assoc_confirmed = n_assoc_confirmed + sum(ismember(ids, confirmed_track_ids));
        end
    end
end

assoc = struct();
assoc.n_filter_points = n_filter_points;
assoc.n_assoc_all_tracks = n_assoc_all;
assoc.n_assoc_confirmed_tracks = n_assoc_confirmed;
assoc.rate_all_tracks = safe_ratio(n_assoc_all, n_filter_points);
assoc.rate_confirmed_tracks = safe_ratio(n_assoc_confirmed, n_filter_points);
end

function truth = collect_truth_points(frames, fused_xyz, fused_ids, frame_times)
truth = empty_truth_points();
truth.source = 'fused_ids';

n = 0;
for k = 1:numel(fused_xyz)
    Z = fused_xyz{k};
    M = size(Z, 2);
    if size(Z, 1) < 3
        continue;
    end
    ids = get_cell_ids(fused_ids, k, M);
    good = isfinite(ids) & all(isfinite(Z(1:3, :)), 1);
    n = n + nnz(good);
end

if n == 0 && ~isempty(frames)
    truth.source = 'frames.active';
    for k = 1:numel(frames)
        if ~isfield(frames, 'active_xyz') || ~isfield(frames, 'target_ids')
            continue;
        end
        Z = frames(k).active_xyz;
        M = size(Z, 2);
        if size(Z, 1) < 3
            continue;
        end
        ids = normalize_row(frames(k).target_ids, M);
        good = isfinite(ids) & all(isfinite(Z(1:3, :)), 1);
        n = n + nnz(good);
    end
end

if n == 0
    return;
end

truth.frame = zeros(n, 1);
truth.time = zeros(n, 1);
truth.raw_id = nan(n, 1);
truth.xyz = nan(3, n);
truth.key = nan(n, 1);
truth.instance = nan(n, 1);

p = 0;
if strcmp(truth.source, 'fused_ids')
    K = numel(fused_xyz);
    for k = 1:K
        Z = fused_xyz{k};
        M = size(Z, 2);
        if size(Z, 1) < 3
            continue;
        end
        ids = get_cell_ids(fused_ids, k, M);
        good = isfinite(ids) & all(isfinite(Z(1:3, :)), 1);
        idx = find(good);
        m = numel(idx);
        if m == 0, continue; end
        ii = p + (1:m);
        truth.frame(ii) = k;
        truth.time(ii) = get_frame_time(frame_times, k);
        truth.raw_id(ii) = ids(idx).';
        truth.xyz(:, ii) = Z(1:3, idx);
        p = p + m;
    end
else
    for k = 1:numel(frames)
        if ~isfield(frames, 'active_xyz') || ~isfield(frames, 'target_ids')
            continue;
        end
        Z = frames(k).active_xyz;
        M = size(Z, 2);
        if size(Z, 1) < 3
            continue;
        end
        ids = normalize_row(frames(k).target_ids, M);
        good = isfinite(ids) & all(isfinite(Z(1:3, :)), 1);
        idx = find(good);
        m = numel(idx);
        if m == 0, continue; end
        ii = p + (1:m);
        truth.frame(ii) = k;
        if isfield(frames, 't_sec')
            truth.time(ii) = frames(k).t_sec;
        else
            truth.time(ii) = get_frame_time(frame_times, k);
        end
        truth.raw_id(ii) = ids(idx).';
        truth.xyz(:, ii) = Z(1:3, idx);
        p = p + m;
    end
end

truth.frame = truth.frame(1:p);
truth.time = truth.time(1:p);
truth.raw_id = truth.raw_id(1:p);
truth.xyz = truth.xyz(:, 1:p);
truth.key = truth.key(1:p);
truth.instance = truth.instance(1:p);
end

function ids = get_cell_ids(fused_ids, k, M)
if k <= numel(fused_ids) && ~isempty(fused_ids{k})
    ids = normalize_row(fused_ids{k}, M);
else
    ids = nan(1, M);
end
end

function assoc = collect_assoc_events(est, frame_times)
assoc = empty_assoc_events();
if ~isstruct(est) || ~isfield(est, 'assoc') || isempty(est.assoc)
    return;
end

n = 0;
for k = 1:numel(est.assoc)
    As = est.assoc{k};
    if isstruct(As) && isfield(As, 'id') && ~isempty(As.id)
        n = n + numel(As.id);
    end
end
if n == 0
    return;
end

assoc.frame = zeros(n, 1);
assoc.time = zeros(n, 1);
assoc.track_id = nan(n, 1);
assoc.raw_id = nan(n, 1);
assoc.split_key = nan(n, 1);
assoc.xyz = nan(3, n);

p = 0;
for k = 1:numel(est.assoc)
    As = est.assoc{k};
    if ~isstruct(As) || ~isfield(As, 'id') || isempty(As.id)
        continue;
    end
    ids = As.id(:);
    m = numel(ids);
    tid = nan(m, 1);
    if isfield(As, 'tid') && ~isempty(As.tid)
        tid0 = As.tid(:);
        q = min(m, numel(tid0));
        tid(1:q) = tid0(1:q);
    end
    pos = nan(3, m);
    if isfield(As, 'xyz') && ~isempty(As.xyz) && size(As.xyz, 1) >= 3
        q = min(m, size(As.xyz, 2));
        pos(:, 1:q) = As.xyz(1:3, 1:q);
    end
    ii = p + (1:m);
    assoc.frame(ii) = k;
    assoc.time(ii) = get_frame_time(frame_times, k);
    assoc.track_id(ii) = ids;
    assoc.raw_id(ii) = tid;
    assoc.xyz(:, ii) = pos;
    p = p + m;
end

assoc.frame = assoc.frame(1:p);
assoc.time = assoc.time(1:p);
assoc.track_id = assoc.track_id(1:p);
assoc.raw_id = assoc.raw_id(1:p);
assoc.split_key = assoc.split_key(1:p);
assoc.xyz = assoc.xyz(:, 1:p);
end

function split_result = split_truth_ids(truth, params)
split_result = struct();
split_result.truth = truth;
split_result.summary = empty_split_result();

n = numel(truth.raw_id);
if n == 0
    return;
end

if ~params.truth_id_split_enabled
    raw_ids = unique(truth.raw_id(isfinite(truth.raw_id))).';
    key = nan(n, 1);
    inst = nan(n, 1);
    for i = 1:numel(raw_ids)
        idx = truth.raw_id == raw_ids(i);
        key(idx) = i;
        inst(idx) = 1;
    end
    split_result.truth.key = key;
    split_result.truth.instance = inst;
    split_result.summary.enabled = false;
    split_result.summary.raw_id_count = numel(raw_ids);
    split_result.summary.instance_count = numel(raw_ids);
    return;
end

key = nan(n, 1);
inst = nan(n, 1);
raw_ids = unique(truth.raw_id(isfinite(truth.raw_id))).';
next_key = 1;
summaries = empty_id_split_summary();
events = empty_split_event();

% 对每个原始目标编号单独处理：先按帧内空间距离聚类，再把当前帧簇与上一时刻
% 已存在实例做一对一连接。连接只看两个硬门限：距离不超过 truth_id_split_dist_m，
% 时间间隔不超过 truth_id_split_max_gap_s；速度不参与伪真值拆分。
for rr = 1:numel(raw_ids)
    raw_id = raw_ids(rr);
    idx_raw = find(truth.raw_id == raw_id & all(isfinite(truth.xyz), 1).');
    if isempty(idx_raw), continue; end
    [~, ord] = sortrows([truth.time(idx_raw), truth.frame(idx_raw)]);
    idx_raw = idx_raw(ord);
    frames_raw = unique(truth.frame(idx_raw), 'stable').';

    comp_key = zeros(1, 0);
    comp_inst = zeros(1, 0);
    comp_last_frame = zeros(1, 0);
    comp_last_t = zeros(1, 0);
    comp_last_pos = zeros(3, 0);
    next_inst = 1;

    for fi = 1:numel(frames_raw)
        frame_now = frames_raw(fi);
        idx_frame = idx_raw(truth.frame(idx_raw) == frame_now);
        t_now = mean(truth.time(idx_frame));
        [cluster_id, n_cluster] = cluster_points_by_distance(truth.xyz(:, idx_frame).', params.truth_id_split_dist_m);
        cent = nan(3, n_cluster);
        for cc = 1:n_cluster
            members = idx_frame(cluster_id == cc);
            cent(:, cc) = mean(truth.xyz(:, members), 2);
        end

        assign_comp = zeros(1, n_cluster);
        if ~isempty(comp_key)
            cand = zeros(numel(comp_key) * n_cluster, 3);
            pp = 0;
            for ci = 1:numel(comp_key)
                dt = max(t_now - comp_last_t(ci), 0);
                if dt > params.truth_id_split_max_gap_s
                    continue;
                end
                for cc = 1:n_cluster
                    d = norm(cent(:, cc) - comp_last_pos(:, ci));
                    if ~isfinite(d) || d > params.truth_id_split_dist_m
                        continue;
                    end
                    pp = pp + 1;
                    cand(pp, :) = [d, ci, cc];
                end
            end
            cand = cand(1:pp, :);
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
                [nearest_ci, nearest_dt, nearest_dist, nearest_speed, reason] = ...
                    nearest_split_reason(comp_last_t, comp_last_frame, comp_last_pos, ...
                    t_now, frame_now, cent(:, cc), params);
                ci = numel(comp_key) + 1;
                comp_key(ci) = next_key;
                comp_inst(ci) = next_inst;
                comp_last_frame(ci) = frame_now;
                comp_last_t(ci) = t_now;
                comp_last_pos(:, ci) = cent(:, cc);
                if ~isnan(nearest_ci)
                    ev = split_event_template();
                    ev.raw_id = raw_id;
                    ev.new_key = next_key;
                    ev.new_instance = next_inst;
                    ev.nearest_key = comp_key(nearest_ci);
                    ev.nearest_instance = comp_inst(nearest_ci);
                    ev.frame = frame_now;
                    ev.time = t_now;
                    ev.dt_s = nearest_dt;
                    ev.dist_m = nearest_dist;
                    ev.speed_mps = nearest_speed;
                    ev.reason = reason;
                    events(end+1, 1) = ev; %#ok<AGROW>
                end
                next_key = next_key + 1;
                next_inst = next_inst + 1;
            else
                comp_last_frame(ci) = frame_now;
                comp_last_t(ci) = t_now;
                comp_last_pos(:, ci) = cent(:, cc);
            end

            members = idx_frame(cluster_id == cc);
            key(members) = comp_key(ci);
            inst(members) = comp_inst(ci);
        end
    end

    s = id_split_summary_template();
    s.raw_id = raw_id;
    s.n_instances = numel(comp_key);
    s.keys = comp_key;
    s.instances = comp_inst;
    s.n_points = numel(idx_raw);
    summaries(end+1, 1) = s; %#ok<AGROW>
end

split_result.truth.key = key;
split_result.truth.instance = inst;
split_result.summary.enabled = true;
split_result.summary.raw_id_count = numel(raw_ids);
split_result.summary.instance_count = numel(unique(key(isfinite(key))));
split_result.summary.n_split_raw_ids = sum([summaries.n_instances] > 1);
split_result.summary.id_summary = summaries;
split_result.summary.events = events;
end

function [nearest_ci, nearest_dt, nearest_dist, nearest_speed, reason] = nearest_split_reason(last_t, last_frame, last_pos, t_now, frame_now, pos_now, params)
nearest_ci = nan;
nearest_dt = nan;
nearest_dist = nan;
nearest_speed = nan;
reason = '';
if isempty(last_t)
    return;
end
best = inf;
for ci = 1:numel(last_t)
    dt = max(t_now - last_t(ci), 0);
    frame_gap = frame_now - last_frame(ci); %#ok<NASGU>
    d = norm(pos_now - last_pos(:, ci));
    speed = d / max(dt, eps);
    if d < best
        best = d;
        nearest_ci = ci;
        nearest_dt = dt;
        nearest_dist = d;
        nearest_speed = speed;
        if d > params.truth_id_split_dist_m
            reason = 'distance_gate';
        elseif dt > params.truth_id_split_max_gap_s
            reason = 'time_gate';
        else
            reason = 'distance_gate';
        end
    end
end
end

function assoc = attach_split_keys_to_assoc(assoc, truth, params)
if isempty(assoc.track_id) || isempty(truth.raw_id)
    return;
end
split_key = nan(numel(assoc.track_id), 1);

max_frame = max([assoc.frame(:); truth.frame(:)]);
if isempty(max_frame) || ~isfinite(max_frame) || max_frame < 1
    assoc.split_key = split_key;
    return;
end
max_frame = round(max_frame);

truth_by_frame = cell(max_frame, 1);
assoc_by_frame = cell(max_frame, 1);
for i = 1:numel(truth.frame)
    f = round(truth.frame(i));
    if f >= 1 && f <= max_frame
        truth_by_frame{f}(end+1) = i; %#ok<AGROW>
    end
end
for i = 1:numel(assoc.frame)
    f = round(assoc.frame(i));
    if f >= 1 && f <= max_frame
        assoc_by_frame{f}(end+1) = i; %#ok<AGROW>
    end
end

for f = 1:max_frame
    ai = assoc_by_frame{f};
    ti = truth_by_frame{f};
    if isempty(ai) || isempty(ti)
        continue;
    end
    raw_ids = unique(assoc.raw_id(ai(isfinite(assoc.raw_id(ai)))));
    for rr = 1:numel(raw_ids)
        raw_id = raw_ids(rr);
        aidx = ai(assoc.raw_id(ai) == raw_id);
        tidx = ti(truth.raw_id(ti) == raw_id);
        if isempty(aidx) || isempty(tidx)
            continue;
        end
        for aa = aidx(:).'
            d = sqrt(sum((truth.xyz(:, tidx) - assoc.xyz(:, aa)).^2, 1));
            [dmin, im] = min(d);
            if ~isempty(im) && isfinite(dmin) && dmin <= params.truth_assoc_map_max_dist_m
                split_key(aa) = truth.key(tidx(im));
            end
        end
    end
end

assoc.split_key = split_key;
end

function acc = compute_accuracy(track_id, truth_key, label, truth_reference_key)
acc = empty_accuracy(label);
if nargin < 4 || isempty(truth_reference_key)
    truth_reference_key = truth_key;
end
if isempty(track_id) || isempty(truth_key)
    return;
end
valid = isfinite(track_id) & isfinite(truth_key);
acc.n_assoc_total = numel(track_id);
acc.n_labeled_assoc = nnz(valid);
if ~any(valid)
    return;
end

tk = truth_key(valid);
tr = track_id(valid);
row_keys = unique(tk).';
col_ids = unique(tr).';
C = zeros(numel(row_keys), numel(col_ids));
truth_totals = zeros(numel(row_keys), 1);
truth_reference_key = truth_reference_key(:);
for r = 1:numel(row_keys)
    truth_totals(r) = nnz(isfinite(truth_reference_key) & ...
        truth_reference_key == row_keys(r));
end
for i = 1:numel(tk)
    r = find(row_keys == tk(i), 1);
    c = find(col_ids == tr(i), 1);
    C(r, c) = C(r, c) + 1;
end

pairs = assign_max_counts(C);
correct = 0;
pair_info = empty_match_pair();
row_sum = sum(C, 2);
col_sum = sum(C, 1);
for i = 1:size(pairs, 1)
    r = pairs(i, 1);
    c = pairs(i, 2);
    n = C(r, c);
    if n <= 0, continue; end
    correct = correct + n;
    p = match_pair_template();
    p.truth_key = row_keys(r);
    p.track_id = col_ids(c);
    p.n_correct = n;
    p.truth_assoc = row_sum(r);
    p.track_assoc = col_sum(c);
    p.recall = safe_ratio(n, row_sum(r));
    % 本项目“航迹纯度”的定义：正确匹配量测数 / 对应伪真值的全部量测数。
    p.purity = safe_ratio(n, truth_totals(r));
    pair_info(end+1, 1) = p; %#ok<AGROW>
end

acc.row_keys = row_keys;
acc.track_ids = col_ids;
acc.confusion = C;
acc.truth_totals = truth_totals;
acc.match_pairs = pair_info;
acc.n_truth = numel(row_keys);
acc.n_tracks = numel(col_ids);
acc.n_correct = correct;
acc.n_error = sum(C(:)) - correct;
acc.accuracy = safe_ratio(correct, sum(C(:)));
acc.mean_track_purity = mean_best_col_truth_coverage(C, truth_totals);
acc.mean_truth_recall = mean_best_row_recall(C);
acc.fragmented_truth_count = sum(sum(C > 0, 2) > 1);
acc.mixed_track_count = sum(sum(C > 0, 1) > 1);
end

function ta = compute_track_level_accuracy(acc, params, output_track_ids, n_truth_reference)
% Only IDs that reached confirmed output participate in track-level scores.
% Unlabeled/unevaluable output tracks remain in the output denominator.
ta = empty_track_accuracy(acc.label);
ta.purity_threshold = params.track_accuracy_purity_th;
ta.min_labeled_assoc = params.track_accuracy_min_assoc;
output_track_ids = unique(output_track_ids(isfinite(output_track_ids))).';
ta.n_truth_reference = max(0, n_truth_reference);
ta.n_output_tracks = numel(output_track_ids);
ta.n_tracks_total = ta.n_output_tracks;
records = empty_track_accuracy_record();
for c = 1:numel(output_track_ids)
    rec = track_accuracy_record_template();
    rec.track_id = output_track_ids(c);
    if isstruct(acc) && isfield(acc, 'confusion') && ~isempty(acc.confusion) && ...
            isfield(acc, 'track_ids') && ~isempty(acc.track_ids)
        ac = find(acc.track_ids == rec.track_id, 1);
    else
        ac = [];
    end
    if ~isempty(ac)
        counts = acc.confusion(:, ac);
        rec.n_labeled_assoc = sum(counts);
        if rec.n_labeled_assoc > 0 && ~isempty(acc.row_keys)
            [rec.n_dominant, ridx] = max(counts);
            rec.truth_key = acc.row_keys(ridx);
            rec.purity = safe_ratio(rec.n_dominant, get_truth_total(acc, ridx));
        end
        rec.evaluable = rec.n_labeled_assoc >= ta.min_labeled_assoc;
        rec.is_purity_ok = rec.evaluable && rec.purity >= ta.purity_threshold;
    end
    records(end+1, 1) = rec; %#ok<AGROW>
end

if ~isempty(records) && isstruct(acc) && isfield(acc, 'confusion') && ...
        ~isempty(acc.confusion) && ~isempty(acc.row_keys)
    C_output = zeros(size(acc.confusion, 1), numel(records));
    for c = 1:numel(records)
        ac = find(acc.track_ids == records(c).track_id, 1);
        if ~isempty(ac)
            C_output(:, c) = acc.confusion(:, ac);
        end
    end
    pairs = assign_max_counts(C_output);
    for i = 1:size(pairs, 1)
        r = pairs(i, 1);
        c = pairs(i, 2);
        n_pair = C_output(r, c);
        pair_purity = safe_ratio(n_pair, get_truth_total(acc, r));
        records(c).is_primary_match = true;
        records(c).matched_truth_key = acc.row_keys(r);
        records(c).matched_n_correct = n_pair;
        records(c).matched_purity = pair_purity;
        records(c).is_correct = records(c).evaluable && ...
            n_pair >= ta.min_labeled_assoc && ...
            pair_purity >= ta.purity_threshold;
    end
end

eval_mask = [records.evaluable];
purity_mask = [records.is_purity_ok];
correct_mask = [records.is_correct];
ta.records = records;
ta.n_evaluable_tracks = nnz(eval_mask);
ta.n_unevaluable_output_tracks = ta.n_output_tracks - ta.n_evaluable_tracks;
ta.n_purity_ok_tracks = nnz(purity_mask);
ta.n_correct_tracks = nnz(correct_mask);
ta.n_error_tracks = ta.n_output_tracks - ta.n_correct_tracks;
ta.n_error_or_extra_tracks = ta.n_error_tracks;
ta.accuracy_vs_output = safe_ratio(ta.n_correct_tracks, ta.n_output_tracks);
ta.accuracy_vs_truth = safe_ratio(ta.n_correct_tracks, ta.n_truth_reference);
ta.accuracy = ta.accuracy_vs_output; % Backward-compatible alias.
if any(eval_mask)
    dom_keys = [records(eval_mask).truth_key];
    dom_keys = dom_keys(isfinite(dom_keys));
    u = unique(dom_keys);
    n_frag_truth = 0;
    n_extra = 0;
    for ii = 1:numel(u)
        c = sum(dom_keys == u(ii));
        if c > 1
            n_frag_truth = n_frag_truth + 1;
            n_extra = n_extra + c - 1;
        end
    end
    ta.n_fragmented_truth = n_frag_truth;
    ta.n_extra_tracks = n_extra;
end
if ta.n_evaluable_tracks > 0
    ta.mean_purity = mean([records(eval_mask).purity]);
    ta.median_purity = median([records(eval_mask).purity]);
else
    ta.mean_purity = nan;
    ta.median_purity = nan;
end
end

function pairs = assign_max_counts(C)
pairs = zeros(0, 2);
if isempty(C) || ~any(C(:) > 0)
    return;
end
% 精确求解“行-列一对一、关联量测总数最大”的匹配。按正权边的连通分量
% 分解后再求解，可避免大而稀疏的伪真值/航迹矩阵整体立方复杂度；同时不再
% 因 Statistics Toolbox 中是否存在 matchpairs 而改变评价语义。
edge = sparse(C > 0);
active_rows = find(any(edge, 2)).';
seen_rows = false(size(C, 1), 1);
seen_cols = false(size(C, 2), 1);
for start_row = active_rows
    if seen_rows(start_row), continue; end
    component_rows = start_row;
    component_cols = zeros(1, 0);
    pending_rows = start_row;
    seen_rows(start_row) = true;
    while ~isempty(pending_rows)
        neighbor_cols = find(any(edge(pending_rows, :), 1));
        new_cols = neighbor_cols(~seen_cols(neighbor_cols));
        if isempty(new_cols)
            break;
        end
        seen_cols(new_cols) = true;
        component_cols = [component_cols, new_cols]; %#ok<AGROW>
        neighbor_rows = find(any(edge(:, new_cols), 2)).';
        new_rows = neighbor_rows(~seen_rows(neighbor_rows));
        seen_rows(new_rows) = true;
        component_rows = [component_rows, new_rows]; %#ok<AGROW>
        pending_rows = new_rows;
    end

    local_pairs = max_weight_component_pairs(C(component_rows, component_cols));
    if ~isempty(local_pairs)
        global_rows = component_rows(local_pairs(:, 1));
        global_cols = component_cols(local_pairs(:, 2));
        pairs = [pairs; global_rows(:), global_cols(:)]; %#ok<AGROW>
    end
end
end

function pairs = max_weight_component_pairs(W)
% 矩形最大权一对一匹配。较小一侧作为必须分配的一侧，并加入足够的零权
% 虚拟列允许不匹配；最终仅保留正关联计数的真实边。
pairs = zeros(0, 2);
if isempty(W) || ~any(W(:) > 0)
    return;
end
was_transposed = size(W, 1) > size(W, 2);
if was_transposed
    W = W.';
end
n_row = size(W, 1);
n_real_col = size(W, 2);
weights = [double(W), zeros(n_row, n_row)];
cost = max(weights(:)) - weights;
assignment = hungarian_min_rect(cost);
for r = 1:n_row
    c = assignment(r);
    if c >= 1 && c <= n_real_col && W(r, c) > 0
        if was_transposed
            pairs(end+1, :) = [c, r]; %#ok<AGROW>
        else
            pairs(end+1, :) = [r, c]; %#ok<AGROW>
        end
    end
end
end

function assignment = hungarian_min_rect(cost)
% Hungarian shortest-augmenting-path form for n_row <= n_col.
[n_row, n_col] = size(cost);
if n_row > n_col
    error('evaluate_fusion_quant_metrics:AssignmentShape', ...
        'Hungarian assignment requires rows <= columns.');
end
u = zeros(n_row + 1, 1);
v = zeros(n_col + 1, 1);
p = zeros(n_col + 1, 1);
way = zeros(n_col + 1, 1);
for i = 1:n_row
    p(1) = i;
    j0 = 1;
    minv = inf(n_col + 1, 1);
    used = false(n_col + 1, 1);
    while true
        used(j0) = true;
        i0 = p(j0);
        delta = inf;
        j1 = 0;
        for j = 2:n_col + 1
            if used(j), continue; end
            cur = cost(i0, j - 1) - u(i0 + 1) - v(j);
            if cur < minv(j)
                minv(j) = cur;
                way(j) = j0;
            end
            if minv(j) < delta
                delta = minv(j);
                j1 = j;
            end
        end
        if ~isfinite(delta) || j1 == 0
            error('evaluate_fusion_quant_metrics:AssignmentFailure', ...
                'Unable to complete one-to-one assignment.');
        end
        for j = 1:n_col + 1
            if used(j)
                u(p(j) + 1) = u(p(j) + 1) + delta;
                v(j) = v(j) - delta;
            else
                minv(j) = minv(j) - delta;
            end
        end
        j0 = j1;
        if p(j0) == 0
            break;
        end
    end
    while true
        j1 = way(j0);
        p(j0) = p(j1);
        j0 = j1;
        if j0 == 1
            break;
        end
    end
end

assignment = zeros(n_row, 1);
for j = 2:n_col + 1
    if p(j) > 0
        assignment(p(j)) = j - 1;
    end
end
end

function y = mean_best_col_truth_coverage(C, truth_totals)
if isempty(C) || ~any(C(:) > 0)
    y = nan;
    return;
end
[best, ridx] = max(C, [], 1);
den = reshape(truth_totals(ridx), 1, []);
valid = best > 0 & den > 0;
y = mean(best(valid) ./ den(valid));
end

function y = mean_best_row_recall(C)
if isempty(C) || ~any(C(:) > 0)
    y = nan;
    return;
end
rs = sum(C, 2);
best = max(C, [], 2);
valid = rs > 0;
y = mean(best(valid) ./ rs(valid));
end

function total = get_truth_total(acc, ridx)
total = 0;
if isstruct(acc) && isfield(acc, 'truth_totals') && ...
        ridx >= 1 && ridx <= numel(acc.truth_totals)
    total = acc.truth_totals(ridx);
elseif isstruct(acc) && isfield(acc, 'confusion') && ...
        ridx >= 1 && ridx <= size(acc.confusion, 1)
    total = sum(acc.confusion(ridx, :));
end
end

function tracks = build_truth_tracks(truth, params)
keys = unique(truth.key(isfinite(truth.key))).';
tracks = empty_truth_tracks();
for ii = 1:numel(keys)
    key = keys(ii);
    idx = find(truth.key == key);
    if isempty(idx), continue; end
    frames = unique(truth.frame(idx), 'stable').';
    t = zeros(1, numel(frames));
    p = nan(3, numel(frames));
    for f = 1:numel(frames)
        jj = idx(truth.frame(idx) == frames(f));
        t(f) = mean(truth.time(jj));
        p(:, f) = mean(truth.xyz(:, jj), 2);
    end
    [t, ord] = sort(t);
    frames = frames(ord);
    p = p(:, ord);
    [t, uidx] = unique(t, 'stable');
    frames = frames(uidx);
    p = p(:, uidx);
    s = truth_track_template();
    s.key = key;
    s.raw_id = dominant_value(truth.raw_id(idx));
    s.instance = dominant_value(truth.instance(idx));
    s.frames = frames;
    s.t = t;
    s.pos = p;
    s.smooth = smooth_truth_position(t, p, params);
    s.n = numel(t);
    s.start_time = t(1);
    s.end_time = t(end);
    tracks(end+1, 1) = s; %#ok<AGROW>
end
end

function est_tracks = collect_est_tracks(est, frame_times)
est_tracks = empty_est_tracks();
if ~isstruct(est) || ~isfield(est, 'L') || ~isfield(est, 'X')
    return;
end

id_list = [];
cell_t = {};
cell_p = {};
idmap = containers.Map('KeyType', 'double', 'ValueType', 'double');
K = min(numel(est.L), numel(frame_times));
for k = 1:K
    if isempty(est.L{k}) || isempty(est.X{k})
        continue;
    end
    Lk = est.L{k};
    Xk = est.X{k};
    nrow = size(Lk, 1);
    for i = 1:nrow
        id = Lk(i, 2);
        if i > size(Xk, 2), continue; end
        pos = Xk([1, 4, 7], i);
        if ~all(isfinite(pos)), continue; end
        if isKey(idmap, id)
            sidx = idmap(id);
        else
            sidx = numel(id_list) + 1;
            idmap(id) = sidx;
            id_list(sidx) = id; %#ok<AGROW>
            cell_t{sidx} = zeros(1, 0); %#ok<AGROW>
            cell_p{sidx} = zeros(3, 0); %#ok<AGROW>
        end
        cell_t{sidx}(end+1) = frame_times(k); %#ok<AGROW>
        cell_p{sidx}(:, end+1) = pos(:); %#ok<AGROW>
    end
end

for sidx = 1:numel(id_list)
    [t, ord] = sort(cell_t{sidx});
    p = cell_p{sidx}(:, ord);
    tr = est_track_template();
    tr.id = id_list(sidx);
    tr.t = t;
    tr.pos = p;
    tr.n = numel(t);
    tr.first_confirmed_time = t(1);
    tr.last_time = t(end);
    est_tracks(end+1, 1) = tr; %#ok<AGROW>
end
end

function start_metrics = compute_start_time_metrics(truth_tracks, est_tracks, track_accuracy, accuracy_split)
start_metrics = empty_start_metrics();
records = empty_start_record();
if nargin < 4 || ~isstruct(accuracy_split)
    accuracy_split = empty_accuracy('split_target_id');
end
if isstruct(track_accuracy) && isfield(track_accuracy, 'records')
    ta_records = track_accuracy.records;
else
    ta_records = empty_track_accuracy_record();
end
ta_track_cols = zeros(1, numel(ta_records));
if ~isempty(ta_records) && isfield(accuracy_split, 'track_ids') && ...
        ~isempty(accuracy_split.track_ids)
    [~, ta_track_cols] = ismember([ta_records.track_id], accuracy_split.track_ids);
end
for i = 1:numel(truth_tracks)
    tt = truth_tracks(i);
    rec = start_record_template();
    rec.truth_key = tt.key;
    rec.raw_id = tt.raw_id;
    rec.instance = tt.instance;
    rec.truth_start_time = tt.start_time;
    rec.truth_end_time = tt.end_time;
    rec.truth_n = tt.n;

    % 最早有效起始：组成该伪真值实例的全部有效确认输出碎片。
    %
    % 这里故意不使用 is_purity_ok。项目航迹纯度的分母是“该伪真值
    % 的全部量测数”，它度量整轨覆盖率；短碎片即使完全属于本目标，
    % 覆盖率也必然较低。若把该门槛用于起始评价，最早碎片会被排除，
    % “最早确认起始”就会错误退化成“一对一主航迹确认”。
    %
    % 碎片归属直接取拆分实例×确认输出航迹的关联计数矩阵。只要某个
    % 已确认输出航迹对当前伪真值存在至少1个实际归属点，它就是组成该
    % 伪真值的一个滤波航迹碎片。这里不使用纯度、整轨覆盖率、航迹长度、
    % 最大关联量测数或最少关联点门槛；这些条件只属于主航迹/正确率评价。
    candidate_idx = zeros(1, 0);
    truth_row = find(accuracy_split.row_keys == tt.key, 1);
    if ~isempty(truth_row) && ~isempty(accuracy_split.confusion)
        fragment_counts = zeros(1, numel(ta_records));
        mapped = ta_track_cols > 0;
        if any(mapped)
            fragment_counts(mapped) = accuracy_split.confusion( ...
                truth_row, ta_track_cols(mapped));
        end
        candidate_idx = find(fragment_counts > 0);
    end
    rec.n_fragment_candidates = numel(candidate_idx);
    earliest_time = inf;
    for ci = candidate_idx
        et_idx = find([est_tracks.id] == ta_records(ci).track_id, 1);
        if isempty(et_idx)
            continue;
        end
        confirmed_time = est_tracks(et_idx).first_confirmed_time;
        if confirmed_time < tt.start_time
            rec.n_preconfirmed_candidates = rec.n_preconfirmed_candidates + 1;
            continue;
        end
        if confirmed_time < earliest_time
            earliest_time = confirmed_time;
            rec.first_confirmed_track_id = est_tracks(et_idx).id;
            rec.first_confirmed_time = confirmed_time;
            rec.track_start_delay_s = confirmed_time - tt.start_time;
        end
    end

    % 主要航迹：只使用拆分实例与确认输出航迹之间的全局一对一最大
    % 关联量测数分配。主航迹起始不使用纯度、正确率或最少点数门槛；
    % is_correct 仅供独立的航迹级正确率指标使用。
    main_idx = find([ta_records.is_primary_match] & ...
        [ta_records.matched_truth_key] == tt.key, 1);
    if ~isempty(main_idx)
        track_id = ta_records(main_idx).track_id;
        et_idx = find([est_tracks.id] == track_id, 1);
        if ~isempty(et_idx)
            et = est_tracks(et_idx);
            if et.first_confirmed_time < tt.start_time
                rec.main_track_preconfirmed = true;
            else
                rec.main_track_id = track_id;
                rec.main_track_confirmed_time = et.first_confirmed_time;
                rec.main_track_n = et.n;
                rec.main_track_start_delay_s = et.first_confirmed_time - tt.start_time;
            end
        end
    end
    records(end+1, 1) = rec; %#ok<AGROW>
end

started = isfinite([records.first_confirmed_track_id]);
main_confirmed = isfinite([records.main_track_id]);
paired = started & main_confirmed;
start_metrics.records = records;
start_metrics.n_truth = numel(records);
start_metrics.n_started = nnz(started);
start_metrics.n_main_confirmed = nnz(main_confirmed);
start_metrics.n_fragment_candidates = sum([records.n_fragment_candidates]);
start_metrics.n_multi_fragment_truth = nnz([records.n_fragment_candidates] > 1);
start_metrics.n_preconfirmed_candidates = sum([records.n_preconfirmed_candidates]);
start_metrics.n_main_preconfirmed = nnz([records.main_track_preconfirmed]);
if any(started)
    start_metrics.mean_track_start_delay_s = mean([records(started).track_start_delay_s]);
else
    start_metrics.mean_track_start_delay_s = nan;
end
if any(main_confirmed)
    start_metrics.mean_main_track_start_delay_s = mean( ...
        [records(main_confirmed).main_track_start_delay_s]);
else
    start_metrics.mean_main_track_start_delay_s = nan;
end
start_metrics.n_paired = nnz(paired);
if any(paired)
    first_paired = [records(paired).track_start_delay_s];
    main_paired = [records(paired).main_track_start_delay_s];
    gap = main_paired - first_paired;
    tol = 1e-9;
    start_metrics.mean_track_start_delay_paired_s = mean(first_paired);
    start_metrics.mean_main_track_start_delay_paired_s = mean(main_paired);
    start_metrics.mean_main_minus_first_paired_s = mean(gap);
    start_metrics.n_first_earlier_than_main = nnz(gap > tol);
    start_metrics.n_first_equal_main = nnz(abs(gap) <= tol);
    start_metrics.n_start_order_violations = nnz(gap < -tol);
    if start_metrics.n_start_order_violations > 0
        warning('evaluate_fusion_quant_metrics:StartOrder', ...
            '%d个伪真值的最早确认碎片晚于其一对一主航迹，请检查关联计数。', ...
            start_metrics.n_start_order_violations);
    end
end
end

function rms_metrics = compute_rms_metrics(truth_tracks, est_tracks, track_accuracy, params)
rms_metrics = empty_rms_metrics();
records = empty_rms_record();
sum_sq = zeros(3, 1);
n_total = 0;

if isstruct(track_accuracy) && isfield(track_accuracy, 'records')
    track_records = track_accuracy.records;
else
    track_records = empty_track_accuracy_record();
end
% RMSE使用同一套全局一对一主航迹，不以纯度或最少点数再次筛选。
% is_correct只属于独立的航迹级正确率统计。
main_idx = find([track_records.is_primary_match]);
for i = 1:numel(main_idx)
    tr = track_records(main_idx(i));
    ti = find([truth_tracks.key] == tr.matched_truth_key, 1);
    ei = find([est_tracks.id] == tr.track_id, 1);
    if isempty(ti) || isempty(ei), continue; end
    truth = truth_tracks(ti);
    est = est_tracks(ei);
    [err, n_used] = compare_track_to_truth(est, truth, params);
    if n_used == 0, continue; end
    ss = sum(err.^2, 2);
    rec = rms_record_template();
    rec.truth_key = truth.key;
    rec.raw_id = truth.raw_id;
    rec.instance = truth.instance;
    rec.track_id = est.id;
    rec.n = n_used;
    rec.rmse_e = sqrt(ss(1) / n_used);
    rec.rmse_n = sqrt(ss(2) / n_used);
    rec.rmse_u = sqrt(ss(3) / n_used);
    rec.rmse_3d = sqrt(sum(ss) / n_used);
    records(end+1, 1) = rec; %#ok<AGROW>
    sum_sq = sum_sq + ss;
    n_total = n_total + n_used;
end

rms_metrics.records = records;
rms_metrics.n_tracks = numel(records);
rms_metrics.n_samples = n_total;
if n_total > 0
    rms_metrics.rmse_e = sqrt(sum_sq(1) / n_total);
    rms_metrics.rmse_n = sqrt(sum_sq(2) / n_total);
    rms_metrics.rmse_u = sqrt(sum_sq(3) / n_total);
    rms_metrics.rmse_3d = sqrt(sum(sum_sq) / n_total);
    vals = [records.rmse_3d];
    rms_metrics.median_track_rmse_3d = median(vals);
    rms_metrics.p90_track_rmse_3d = percentile_local(vals, 90);
else
    rms_metrics.rmse_e = nan;
    rms_metrics.rmse_n = nan;
    rms_metrics.rmse_u = nan;
    rms_metrics.rmse_3d = nan;
    rms_metrics.median_track_rmse_3d = nan;
    rms_metrics.p90_track_rmse_3d = nan;
end
end

function [err, n_used] = compare_track_to_truth(est, truth, params)
err = zeros(3, 0);
n_used = 0;
if isempty(est.t) || isempty(truth.t)
    return;
end

tt = truth.t(:).';
tp = truth.smooth;
[tt, uidx] = unique(tt, 'stable');
tp = tp(:, uidx);

if numel(tt) >= 2
    mask = est.t >= (tt(1) - params.rms_time_tolerance_s) & ...
           est.t <= (tt(end) + params.rms_time_tolerance_s);
    if ~any(mask), return; end
    tq = min(max(est.t(mask), tt(1)), tt(end));
    q = nan(3, numel(tq));
    for ax = 1:3
        q(ax, :) = interp1(tt, tp(ax, :), tq, 'linear');
    end
    good = all(isfinite(q), 1);
    ep = est.pos(:, mask);
    err = ep(:, good) - q(:, good);
else
    dt = abs(est.t - tt(1));
    mask = dt <= params.rms_time_tolerance_s;
    if ~any(mask), return; end
    q = repmat(tp(:, 1), 1, nnz(mask));
    err = est.pos(:, mask) - q;
end
n_used = size(err, 2);
end

function S = smooth_truth_position(t, p, params)
S = p;
if numel(t) < 2
    return;
end
for ax = 1:3
    S(ax, :) = cv_rts_axis(t, p(ax, :), params.truth_rts_meas_std_m, params.truth_rts_proc_std_mps2);
end
end

function xs = cv_rts_axis(t, z, rstd, qstd)
n = numel(z);
if n <= 1
    xs = z;
    return;
end
[xf, Pf, xp, Pp] = cv_forward_axis(t, z, rstd, qstd);
xsm = xf;
for k = n-1:-1:1
    dt = max(t(k+1) - t(k), 1e-3);
    F = [1, dt; 0, 1];
    C = (Pf(:, :, k) * F') / make_spd2(Pp(:, :, k+1));
    xsm(:, k) = xf(:, k) + C * (xsm(:, k+1) - xp(:, k+1));
end
xs = xsm(1, :);
end

function [xf, Pf, xp, Pp] = cv_forward_axis(t, z, rstd, qstd)
n = numel(z);
R = max(rstd, 1e-6)^2;
q = max(qstd, 1e-9)^2;
H = [1, 0];
I2 = eye(2);
xf = zeros(2, n);
Pf = zeros(2, 2, n);
xp = zeros(2, n);
Pp = zeros(2, 2, n);
x = [z(1); 0];
P = diag([R, (10 * max(rstd, 1))^2]);
for k = 1:n
    if k > 1
        dt = max(t(k) - t(k-1), 1e-3);
        F = [1, dt; 0, 1];
        Q = q * [dt^3/3, dt^2/2; dt^2/2, dt];
        x = F * x;
        P = F * P * F' + Q;
    end
    xp(:, k) = x;
    Pp(:, :, k) = P;
    S = H * P * H' + R;
    K = (P * H') / S;
    x = x + K * (z(k) - H * x);
    P = (I2 - K * H) * P;
    P = make_spd2(P);
    xf(:, k) = x;
    Pf(:, :, k) = P;
end
end

function A = make_spd2(A)
A = (A + A') / 2;
[V, D] = eig(A);
d = max(real(diag(D)), 1e-12);
A = V * diag(d) * V';
A = (A + A') / 2;
end

function print_metrics_report(metrics)
fprintf('\n========== 定量评价指标 ==========\n');
A = metrics.association;
fprintf('滤波关联率(全部航迹):       %.2f%%  (%d/%d)\n', ...
    100 * A.rate_all_tracks, A.n_assoc_all_tracks, A.n_filter_points);
fprintf('滤波关联率(最终确认航迹回溯): %.2f%%  (%d/%d)\n', ...
    100 * A.rate_confirmed_tracks, A.n_assoc_confirmed_tracks, A.n_filter_points);
if isfield(metrics, 'track_timeout') && metrics.track_timeout.enabled
    T = metrics.track_timeout;
    fprintf('航迹秒级超时删除: 试探=%d, 确认=%d, 合计=%d\n', ...
        T.n_tentative, T.n_confirmed, T.n_total);
end

if isfield(metrics, 'id_split') && isfield(metrics.id_split, 'raw_id_count')
    S = metrics.id_split;
    fprintf('伪真值编号: 原始编号=%d, 拆分实例=%d, 被拆分原始编号=%d\n', ...
        get_field(S, 'raw_id_count', 0), get_field(S, 'instance_count', 0), ...
        get_field(S, 'n_split_raw_ids', 0));
end

R = metrics.accuracy_raw;
S = metrics.accuracy_split;
fprintf('关联点一致率(原始编号): %.2f%%  一致点=%d 不一致点=%d 带编号点=%d\n', ...
    100 * R.accuracy, R.n_correct, R.n_error, R.n_labeled_assoc);
fprintf('关联点一致率(拆分实例): %.2f%%  一致点=%d 不一致点=%d 带编号点=%d\n', ...
    100 * S.accuracy, S.n_correct, S.n_error, S.n_labeled_assoc);
if isfield(metrics, 'track_accuracy_raw') && isfield(metrics, 'track_accuracy_split')
    TS = metrics.track_accuracy_split;
    fprintf(['航迹级正确率_比滤波输出航迹数: %.2f%%  正确航迹数=%d, 滤波输出总航迹数=%d, ', ...
        '可评价航迹数=%d, 多余/错误航迹数=%d, 拆分实例数=%d\n'], ...
        100 * TS.accuracy_vs_output, TS.n_correct_tracks, TS.n_output_tracks, ...
        TS.n_evaluable_tracks, TS.n_error_or_extra_tracks, TS.n_truth_reference);
    fprintf(['航迹级正确率_比参考伪真值: %.2f%%  正确航迹数=%d, 滤波输出总航迹数=%d, ', ...
        '可评价航迹数=%d, 多余/错误航迹数=%d, 拆分实例数=%d\n'], ...
        100 * TS.accuracy_vs_truth, TS.n_correct_tracks, TS.n_output_tracks, ...
        TS.n_evaluable_tracks, TS.n_error_or_extra_tracks, TS.n_truth_reference);
end
fprintf('拆分实例问题: 伪真值碎片=%d, 混合航迹=%d\n', ...
    S.fragmented_truth_count, S.mixed_track_count);

if isfield(metrics, 'start_time')
    T = metrics.start_time;
    fprintf(['航迹起始: 最早确认碎片=%d/%d, 平均航迹起始延迟=%.3fs; ', ...
        '一对一主航迹确认=%d/%d, 平均主航迹确认起始延迟=%.3fs\n'], ...
        T.n_started, T.n_truth, T.mean_track_start_delay_s, ...
        T.n_main_confirmed, T.n_truth, T.mean_main_track_start_delay_s);
    fprintf('起始碎片候选: 归属候选=%d, 多碎片伪真值=%d, 起点前排除=%d\n', ...
        T.n_fragment_candidates, T.n_multi_fragment_truth, ...
        T.n_preconfirmed_candidates);
    fprintf('主航迹口径: 确认输出航迹全局一对一最大关联量测数；不使用纯度或最少点数门槛\n');
    if T.n_paired > 0
        fprintf(['起始延迟同集对比(%d条): 最早碎片=%.3fs, 主航迹=%.3fs, ', ...
            '主航迹相对晚=%.3fs, 最早更早/相同/违例=%d/%d/%d\n'], ...
            T.n_paired, T.mean_track_start_delay_paired_s, ...
            T.mean_main_track_start_delay_paired_s, ...
            T.mean_main_minus_first_paired_s, T.n_first_earlier_than_main, ...
            T.n_first_equal_main, T.n_start_order_violations);
    end
end
if isfield(metrics, 'rms')
    Q = metrics.rms;
    fprintf('伪真值RTS平滑RMSE: 三维=%.2fm, E/N/U=[%.2f %.2f %.2f]m, 样本=%d\n', ...
        Q.rmse_3d, Q.rmse_e, Q.rmse_n, Q.rmse_u, Q.n_samples);
end

if isfield(metrics, 'id_split') && isfield(metrics.id_split, 'events') && ...
        ~isempty(metrics.id_split.events)
    ev = metrics.id_split.events;
    n = min(metrics.params.max_print, numel(ev));
    print_split_gate_log(ev, 'distance_gate', ...
        sprintf('量测连接距离超过%.1f米', metrics.params.truth_id_split_dist_m), n);
    print_split_gate_log(ev, 'time_gate', ...
        sprintf('量测间隔超过%.1f秒', metrics.params.truth_id_split_max_gap_s), n);
    fprintf('主要伪真值拆分事件:\n');
    fprintf('  原始编号  最近实例  新实例  帧号  时间差(s)  距离(m)  原因\n');
    for i = 1:n
        fprintf('  %.0f     %.0f       %.0f    %.0f    %.3f      %.1f    %s\n', ...
            ev(i).raw_id, ev(i).nearest_key, ev(i).new_key, ev(i).frame, ...
            ev(i).dt_s, ev(i).dist_m, split_reason_text(ev(i).reason));
    end
elseif isfield(metrics, 'id_split')
    if isfield(metrics.id_split, 'enabled') && ~metrics.id_split.enabled
        fprintf('伪真值拆分日志: 伪真值编号分割已关闭。\n');
    else
        fprintf('伪真值拆分日志: 无空间门或时间门拆分事件。\n');
    end
end
end

function print_split_gate_log(events, reason_code, reason_text, max_print)
if isempty(events), return; end
reasons = {events.reason};
idx = find(strcmp(reasons, reason_code));
if isempty(idx)
    fprintf('%s: 无伪真值拆分。\n', reason_text);
    return;
end
idx = idx(1:min(max_print, numel(idx)));
fprintf('%s:\n', reason_text);
for ii = 1:numel(idx)
    ev = events(idx(ii));
    fprintf('  伪真值id%.0f-(%.0f,%.0f)，帧=%.0f，间隔=%.3fs，距离=%.1fm\n', ...
        ev.raw_id, ev.nearest_key, ev.new_key, ev.frame, ev.dt_s, ev.dist_m);
end
end

function txt = split_reason_text(reason)
switch char(reason)
    case 'distance_gate'
        txt = '超过空间距离门';
    case 'time_gate'
        txt = '超过时间间隔门';
    otherwise
        txt = '未知原因';
end
end

function y = get_field(s, name, default_value)
if isstruct(s) && isfield(s, name) && ~isempty(s.(name))
    y = s.(name);
else
    y = default_value;
end
end

function ids = normalize_row(ids, n)
ids = ids(:).';
if numel(ids) < n
    ids = [ids, nan(1, n - numel(ids))];
elseif numel(ids) > n
    ids = ids(1:n);
end
end

function t = get_frame_time(frame_times, k)
if ~isempty(frame_times) && k <= numel(frame_times)
    t = frame_times(k);
else
    t = k;
end
end

function r = safe_ratio(a, b)
if b <= 0 || ~isfinite(b)
    r = nan;
else
    r = a / b;
end
end

function p = percentile_local(x, pct)
x = sort(x(isfinite(x)));
if isempty(x)
    p = nan;
    return;
end
q = 1 + (numel(x) - 1) * pct / 100;
lo = floor(q);
hi = ceil(q);
if lo == hi
    p = x(lo);
else
    p = x(lo) + (x(hi) - x(lo)) * (q - lo);
end
end

function [cluster_id, n_cluster] = cluster_points_by_distance(P, dist_m)
n = size(P, 1);
cluster_id = zeros(n, 1);
if n == 0
    n_cluster = 0;
    return;
end
parent = 1:n;
for a = 1:n-1
    for b = a+1:n
        if norm(P(a, :) - P(b, :)) <= dist_m
            parent = uf_union(parent, a, b);
        end
    end
end
root = zeros(n, 1);
for i = 1:n
    root(i) = uf_find(parent, i);
end
u = unique(root).';
n_cluster = numel(u);
for i = 1:n_cluster
    cluster_id(root == u(i)) = i;
end
end

function parent = uf_union(parent, a, b)
ra = uf_find(parent, a);
rb = uf_find(parent, b);
if ra ~= rb
    parent(rb) = ra;
end
end

function r = uf_find(parent, a)
r = a;
while parent(r) ~= r
    r = parent(r);
end
end

function v = dominant_value(x)
x = x(:);
x = x(isfinite(x));
if isempty(x)
    v = nan;
    return;
end
u = unique(x);
cnt = zeros(size(u));
for i = 1:numel(u)
    cnt(i) = sum(x == u(i));
end
[~, im] = max(cnt);
v = u(im);
end

function truth = empty_truth_points()
truth = struct('source', '', 'frame', zeros(0, 1), 'time', zeros(0, 1), ...
    'raw_id', zeros(0, 1), 'xyz', zeros(3, 0), ...
    'key', zeros(0, 1), 'instance', zeros(0, 1));
end

function assoc = empty_assoc_events()
assoc = struct('frame', zeros(0, 1), 'time', zeros(0, 1), ...
    'track_id', zeros(0, 1), 'raw_id', zeros(0, 1), ...
    'split_key', zeros(0, 1), 'xyz', zeros(3, 0));
end

function s = empty_split_result()
s = struct('enabled', true, 'raw_id_count', 0, 'instance_count', 0, ...
    'n_split_raw_ids', 0, 'id_summary', empty_id_split_summary(), ...
    'events', empty_split_event());
end

function s = empty_id_split_summary()
s = repmat(id_split_summary_template(), 0, 1);
end

function s = id_split_summary_template()
s = struct('raw_id', nan, 'n_instances', 0, 'keys', [], ...
    'instances', [], 'n_points', 0);
end

function s = empty_split_event()
s = repmat(split_event_template(), 0, 1);
end

function s = split_event_template()
s = struct('raw_id', nan, 'new_key', nan, 'new_instance', nan, ...
    'nearest_key', nan, 'nearest_instance', nan, ...
    'frame', nan, 'time', nan, 'dt_s', nan, 'dist_m', nan, ...
    'speed_mps', nan, 'reason', '');
end

function acc = empty_accuracy(label)
acc = struct('label', label, 'n_assoc_total', 0, 'n_labeled_assoc', 0, ...
    'n_truth', 0, 'n_tracks', 0, 'n_correct', 0, 'n_error', 0, ...
    'accuracy', nan, 'row_keys', [], 'track_ids', [], 'confusion', [], ...
    'truth_totals', zeros(0, 1), ...
    'match_pairs', empty_match_pair(), 'mean_track_purity', nan, ...
    'mean_truth_recall', nan, 'fragmented_truth_count', 0, ...
    'mixed_track_count', 0);
end

function ta = empty_track_accuracy(label)
ta = struct('label', label, 'purity_threshold', nan, 'min_labeled_assoc', 0, ...
    'n_tracks_total', 0, 'n_output_tracks', 0, 'n_evaluable_tracks', 0, ...
    'n_unevaluable_output_tracks', 0, ...
    'n_purity_ok_tracks', 0, 'n_correct_tracks', 0, 'n_error_tracks', 0, ...
    'n_error_or_extra_tracks', 0, 'n_truth_reference', 0, ...
    'n_fragmented_truth', 0, 'n_extra_tracks', 0, ...
    'accuracy', nan, 'accuracy_vs_output', nan, 'accuracy_vs_truth', nan, ...
    'mean_purity', nan, 'median_purity', nan, ...
    'records', empty_track_accuracy_record());
end

function r = empty_track_accuracy_record()
r = repmat(track_accuracy_record_template(), 0, 1);
end

function r = track_accuracy_record_template()
r = struct('track_id', nan, 'truth_key', nan, 'n_labeled_assoc', 0, ...
    'n_dominant', 0, 'purity', nan, 'evaluable', false, ...
    'is_purity_ok', false, 'is_primary_match', false, ...
    'matched_truth_key', nan, 'matched_n_correct', 0, ...
    'matched_purity', nan, 'is_correct', false);
end

function s = empty_match_pair()
s = repmat(match_pair_template(), 0, 1);
end

function s = match_pair_template()
s = struct('truth_key', nan, 'track_id', nan, 'n_correct', 0, ...
    'truth_assoc', 0, 'track_assoc', 0, 'recall', nan, 'purity', nan);
end

function tracks = empty_truth_tracks()
tracks = repmat(truth_track_template(), 0, 1);
end

function s = truth_track_template()
s = struct('key', nan, 'raw_id', nan, 'instance', nan, ...
    'frames', [], 't', [], 'pos', zeros(3, 0), 'smooth', zeros(3, 0), ...
    'n', 0, 'start_time', nan, 'end_time', nan);
end

function tracks = empty_est_tracks()
tracks = repmat(est_track_template(), 0, 1);
end

function s = est_track_template()
s = struct('id', nan, 't', [], 'pos', zeros(3, 0), 'n', 0, ...
    'first_confirmed_time', nan, 'last_time', nan);
end

function s = empty_start_metrics()
s = struct('records', empty_start_record(), 'n_truth', 0, ...
    'n_started', 0, 'n_main_confirmed', 0, ...
    'n_fragment_candidates', 0, 'n_multi_fragment_truth', 0, ...
    'n_preconfirmed_candidates', 0, 'n_main_preconfirmed', 0, ...
    'mean_track_start_delay_s', nan, 'mean_main_track_start_delay_s', nan, ...
    'n_paired', 0, 'mean_track_start_delay_paired_s', nan, ...
    'mean_main_track_start_delay_paired_s', nan, ...
    'mean_main_minus_first_paired_s', nan, ...
    'n_first_earlier_than_main', 0, 'n_first_equal_main', 0, ...
    'n_start_order_violations', 0);
end

function s = empty_start_record()
s = repmat(start_record_template(), 0, 1);
end

function s = start_record_template()
s = struct('truth_key', nan, 'raw_id', nan, 'instance', nan, ...
    'truth_start_time', nan, 'truth_end_time', nan, 'truth_n', 0, ...
    'first_confirmed_track_id', nan, 'first_confirmed_time', nan, ...
    'track_start_delay_s', nan, 'n_fragment_candidates', 0, ...
    'n_preconfirmed_candidates', 0, ...
    'main_track_id', nan, 'main_track_confirmed_time', nan, ...
    'main_track_n', 0, 'main_track_start_delay_s', nan, ...
    'main_track_preconfirmed', false);
end

function s = empty_rms_metrics()
s = struct('records', empty_rms_record(), 'n_tracks', 0, 'n_samples', 0, ...
    'rmse_e', nan, 'rmse_n', nan, 'rmse_u', nan, 'rmse_3d', nan, ...
    'median_track_rmse_3d', nan, 'p90_track_rmse_3d', nan);
end

function s = empty_rms_record()
s = repmat(rms_record_template(), 0, 1);
end

function s = rms_record_template()
s = struct('truth_key', nan, 'raw_id', nan, 'instance', nan, ...
    'track_id', nan, 'n', 0, ...
    'rmse_e', nan, 'rmse_n', nan, 'rmse_u', nan, 'rmse_3d', nan);
end
