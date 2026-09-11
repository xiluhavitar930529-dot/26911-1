function [est, stream_state] = run_filter_joint_2d3d(events, platform, cfg, stream_state)
%RUN_FILTER_JOINT_2D3D Unified logical-track filter with 2-D/3-D branches.
%
% Each physical target owns one logical ID. The angle IMM-KF and spatial
% IMM filter are internal branches of that logical track. Association and
% lifecycle evidence are counted once at logical-track level.

if nargin < 4, stream_state = []; end
if isempty(stream_state)
    p = joint_params(cfg);
    stream_state = init_stream_state(p);
else
    validate_stream_state(stream_state);
    if ~isfield(stream_state, 'schema_version') || ...
            stream_state.schema_version ~= stream_schema_version()
        stream_state = upgrade_stream_state(stream_state, cfg);
    end
    p = stream_state.params;
end
event_offset = stream_state.event_count;
tracks = stream_state.tracks(:);
companions = stream_state.companions(:);
next_id = stream_state.next_id;
transitions = stream_state.transitions;
stats = stream_state.stats;
last_quality_t = stream_state.last_quality_t;
transition_offset = numel(transitions);

K = numel(events);
est = init_estimate(K, events);
est.framework = 'joint_2d3d';
est.output_contract = 'logical_track_v1';
est.transition_log = transitions([]); est.stats = stats;
if K == 0
    return;
end
quiet = logical(get_cfg(cfg, 'joint_streaming_quiet', false));
if ~quiet

fprintf('\n========== 统一二维/三维逻辑航迹滤波 ==========\n');
fprintf('  事件=%d, 逻辑确认=%d/%d, 二维门=%.2f, 三维门=%.2f\n', ...
    K, p.confirm_M, p.confirm_N, p.gate_2d, p.gate_3d);
fprintf('  空间角度预测缓存=%d, 进度间隔=%d事件\n', ...
    p.projection_cache_enabled, p.progress_interval_events);
fprintf('  三维伴随被动验收: NIS<=%.2f, 方位/俯仰残差<=%.3fdeg\n', ...
    p.accept_nis_3d_companion, p.fast_gate_3d_companion_deg);

end
t_start = tic;
for k = 1:K
    event_index = event_offset + k;
    e = events(k);
    t = primary_event_time(e);
    if ~isfinite(t)
        error('run_filter_joint_2d3d:InvalidEventTime', ...
            '事件%d没有有限处理时间。', k);
    end
    if k > 1
        previous_t = est.filter_times(k - 1);
    else
        previous_t = stream_state.last_t;
    end
    if isfinite(previous_t) && t < previous_t - 1e-9
        error('run_filter_joint_2d3d:NonmonotonicEvents', ...
            '事件%d时间%.9f早于上一事件滤波时间%.9f。', ...
            event_index, t, previous_t);
    end

    % One prediction per event. The residual passive-only controller can
    % restrict M/N opportunities to physical passive scan cycles.
    confirm_cycle = confirmation_cycle(e, p);
    quality_cycle = quality_assessment_cycle(e, t, last_quality_t, p);
    if quality_cycle
        last_quality_t = t;
        stats.quality_cycles = stats.quality_cycles + 1;
    end
    for i = 1:numel(tracks)
        tracks(i) = predict_track(tracks(i), t, p);
        if confirm_cycle
            tracks(i).hit_history = shift_window(tracks(i).hit_history, 0);
            tracks(i).age = tracks(i).age + 1;
        end
        if e.has_active
            tracks(i).active_history = shift_window(tracks(i).active_history, 0);
        end
    end

    assoc = assoc_template();
    event_hit_ids = zeros(1, 0);
    sensor = platform_enu(t, platform, cfg);
    [companions, companion_changes, rebind_info] = update_external_companions( ...
        companions, e, t, sensor, p, event_index, quality_cycle);
    transitions = [transitions, companion_changes]; %#ok<AGROW>
    stats.external_rebind_candidates = stats.external_rebind_candidates + ...
        rebind_info.n_candidates;
    stats.external_rebind_committed = stats.external_rebind_committed + ...
        rebind_info.n_committed;
    stats.external_rebind_ambiguous = stats.external_rebind_ambiguous + ...
        rebind_info.n_ambiguous;
    stats.external_rebind_active_unknown = stats.external_rebind_active_unknown + rebind_info.n_active_unknown;
    stats.external_rebind_eligible = stats.external_rebind_eligible + rebind_info.n_eligible;
    stats.external_rebind_projection_invalid = stats.external_rebind_projection_invalid + rebind_info.n_projection_invalid;
    stats.external_rebind_reject_angle = stats.external_rebind_reject_angle + rebind_info.n_reject_angle;
    stats.external_rebind_reject_rate = stats.external_rebind_reject_rate + rebind_info.n_reject_rate;
    stats.external_rebind_reject_switch = stats.external_rebind_reject_switch + rebind_info.n_reject_switch;
    stats.external_rebind_assigned = stats.external_rebind_assigned + rebind_info.n_assigned;
    stats.external_rebind_wait_history = stats.external_rebind_wait_history + rebind_info.n_wait_history;
    stats.external_rebind_wait_ready = stats.external_rebind_wait_ready + rebind_info.n_wait_ready;
    stats.external_rebind_wait_bound = stats.external_rebind_wait_bound + rebind_info.n_wait_bound;
    stats.switch_3d_to_2d_committed = ...
        stats.switch_3d_to_2d_committed + ...
        rebind_info.n_switch_3d_to_2d_committed;
    stats.switch_2d_to_3d_committed = ...
        stats.switch_2d_to_3d_committed + ...
        rebind_info.n_switch_2d_to_3d_committed;
    stats.switch_3d_to_2d_blocked_not_ready = ...
        stats.switch_3d_to_2d_blocked_not_ready + ...
        rebind_info.n_switch_3d_to_2d_blocked_not_ready;

    % Active RAE has priority inside a synchronized event.
    [pairs_a, cost_a, active_accept_info] = associate_active( ...
        tracks, e.active, t, platform, cfg, sensor, p);
    stats.active_2d_candidate_edges = stats.active_2d_candidate_edges + ...
        active_accept_info.n_candidate_edges;
    stats.active_2d_accept_edges = stats.active_2d_accept_edges + ...
        active_accept_info.n_accept_edges;
    stats.active_2d_reject_nis = stats.active_2d_reject_nis + ...
        active_accept_info.n_reject_nis;
    stats.active_2d_reject_gap = stats.active_2d_reject_gap + ...
        active_accept_info.n_reject_gap;
    stats.active_2d_reject_az = stats.active_2d_reject_az + ...
        active_accept_info.n_reject_az;
    stats.active_2d_reject_el = stats.active_2d_reject_el + ...
        active_accept_info.n_reject_el;
    stats.active_2d_reject_los = stats.active_2d_reject_los + ...
        active_accept_info.n_reject_los;
    stats.active_2d_released_measurements = ...
        stats.active_2d_released_measurements + ...
        nnz(active_accept_info.released_measurements);
    used_a = false(1, e.active.n_meas);
    for q = 1:size(pairs_a, 1)
        ti = pairs_a(q, 1); mi = pairs_a(q, 2);
        [innovation, innovation_nis, innovation_kind] = ...
            active_association_diagnostic(tracks(ti), e.active, mi, sensor);
        [tracks(ti), range_updated] = update_track_active( ...
            tracks(ti), e.active, mi, t, sensor, p);
        tracks(ti).hit_history(end) = 1;
        tracks(ti).passive_confirm_streak = 0;
        if range_updated
            tracks(ti).active_history(end) = 1;
            tracks(ti).last_active_t = t;
        end
        tracks(ti).last_update_t = t;
        tracks(ti).miss = 0;
        tracks(ti) = vote_truth_id(tracks(ti), e.active.ids(mi));
        used_a(mi) = true;
        event_hit_ids(end + 1) = tracks(ti).id; %#ok<AGROW>
        assoc = append_assoc(assoc, tracks(ti).id, 'active', mi, ...
            e.active.ids(mi), e.active.rae(2:3, mi), e.active.xyz(:, mi), ...
            cost_a(ti, mi), innovation, innovation_nis, 1, innovation_kind, ...
            2 + double(tracks(ti).space.valid), ...
            2 + double(e.active.has_range(mi)), range_updated);
        stats.active_assigned = stats.active_assigned + 1;
    end

    for mi = find(~used_a)
        if explained_measurement(cost_a, mi, p.birth_explain_nis)
            stats.active_birth_suppressed = stats.active_birth_suppressed + 1;
            continue;
        end
        tr = new_track(next_id, event_index, t, e.active.rae(2:3, mi), ...
            e.active.R_ae(:, :, mi), e.active.xyz(:, mi), ...
            e.active.R_xyz(:, :, mi), e.active.has_range(mi), p);
        tr = vote_truth_id(tr, e.active.ids(mi));
        tracks(end + 1) = tr; %#ok<AGROW>
        tracks = tracks(:);
        event_hit_ids(end + 1) = next_id; %#ok<AGROW>
        assoc = append_assoc(assoc, next_id, 'active_birth', mi, ...
            e.active.ids(mi), e.active.rae(2:3, mi), e.active.xyz(:, mi), NaN, ...
            [], [], [], [], 2 + double(tr.space.valid), ...
            2 + double(e.active.has_range(mi)), e.active.has_range(mi));
        next_id = next_id + 1;
        stats.active_births = stats.active_births + 1;
    end

    % Preserve the passive scan timestamp inside a synchronized logical
    % opportunity. No lifecycle window is advanced during this substep.
    t_passive = passive_event_time(e, t);
    if e.has_passive && t_passive > t
        for i = 1:numel(tracks)
            tracks(i) = predict_track(tracks(i), t_passive, p);
        end
        t = t_passive;
        sensor = platform_enu(t, platform, cfg);
    end

    % Re-associate passive AE after active updates/births. A passive return
    % can update both branches of the same logical track, but hit evidence
    % remains one for the event.
    [companions, detached_used_p, detached_assoc, detached_accept] = ...
        associate_detached_companions(companions, e, t, platform, cfg, sensor, p);
    assoc = concat_assoc(assoc, detached_assoc);
    stats.detached_passive_assigned = stats.detached_passive_assigned + ...
        nnz(detached_used_p);
    [pairs_p, cost_p, ~, accept_info] = associate_passive( ...
        tracks, e.passive, t, platform, cfg, sensor, p, detached_used_p);
    accept_info = add_accept_info(accept_info, detached_accept);
    stats.passive_candidate_edges = stats.passive_candidate_edges + ...
        accept_info.n_candidate_edges;
    stats.passive_accept_edges = stats.passive_accept_edges + ...
        accept_info.n_accept_edges;
    stats.passive_reject_nis = stats.passive_reject_nis + accept_info.n_reject_nis;
    stats.passive_reject_gap = stats.passive_reject_gap + accept_info.n_reject_gap;
    stats.passive_reject_az = stats.passive_reject_az + accept_info.n_reject_az;
    stats.passive_reject_el = stats.passive_reject_el + accept_info.n_reject_el;
    stats.passive_reject_los = stats.passive_reject_los + accept_info.n_reject_los;
    stats.passive_released_measurements = stats.passive_released_measurements + ...
        nnz(accept_info.released_measurements);
    used_p = detached_used_p;
    for q = 1:size(pairs_p, 1)
        ti = pairs_p(q, 1); mi = pairs_p(q, 2);
        kind = passive_measurement_kind(e.passive, mi);
        [innovation, innovation_nis] = passive_association_diagnostic( ...
            tracks(ti), e.passive, mi, sensor);
        [tracks(ti), equivalent_hit] = register_passive_confirm_hit( ...
            tracks(ti), t, p, kind);
        tracks(ti) = update_track_passive(tracks(ti), e.passive, mi, t, sensor, p);
        if equivalent_hit
            tracks(ti).hit_history(end) = 1;
        end
        tracks(ti).last_update_t = t;
        tracks(ti).last_passive_t = t;
        tracks(ti).miss = 0;
        tracks(ti) = vote_truth_id(tracks(ti), e.passive.ids(mi));
        used_p(mi) = true;
        event_hit_ids(end + 1) = tracks(ti).id; %#ok<AGROW>
        assoc = append_assoc(assoc, tracks(ti).id, 'passive', mi, ...
            e.passive.ids(mi), e.passive.ang(:, mi), nan(3, 1), ...
            cost_p(ti, mi), innovation, innovation_nis, 1, 1, ...
            2 + double(tracks(ti).space.valid), 2, false);
        stats.passive_assigned = stats.passive_assigned + 1;
    end

    for mi = find(~used_p)
        if ~accept_info.released_measurements(mi) && ...
                explained_measurement(cost_p, mi, p.birth_explain_nis)
            stats.passive_birth_suppressed = stats.passive_birth_suppressed + 1;
            continue;
        end
        tr = new_track(next_id, event_index, t, e.passive.ang(:, mi), ...
            e.passive.R_ae(:, :, mi), [], [], false, p);
        tr.last_passive_t = t;
        if p.passive_confirm_group > 1 && passive_measurement_kind(e.passive, mi) ~= 2
            tr.hit_history(end) = 0;
            tr.passive_confirm_streak = 1;
        end
        tr = vote_truth_id(tr, e.passive.ids(mi));
        tracks(end + 1) = tr; %#ok<AGROW>
        tracks = tracks(:);
        event_hit_ids(end + 1) = next_id; %#ok<AGROW>
        assoc = append_assoc(assoc, next_id, 'passive_birth', mi, ...
            e.passive.ids(mi), e.passive.ang(:, mi), nan(3, 1), NaN, ...
            [], [], [], [], 2, 2, false);
        next_id = next_id + 1;
        stats.passive_births = stats.passive_births + 1;
    end

    event_hit_ids = unique(event_hit_ids);
    for i = 1:numel(tracks)
        if ~ismember(tracks(i).id, event_hit_ids)
            tracks(i).miss = tracks(i).miss + 1;
            if confirm_cycle && p.passive_confirm_group > 1 && ...
                    isfinite(tracks(i).last_passive_t) && ...
                    t - tracks(i).last_passive_t > p.passive_confirm_max_gap_s
                tracks(i).passive_confirm_streak = 0;
            end
        end
    end

    [tracks, n_cross_suppressed, suppressed_ids, target_ids, merge_records] = ...
        suppress_tracks_explained_by_3d( ...
        tracks, e, t, p, event_index);
    stats.cross_dimension_suppressed = stats.cross_dimension_suppressed + ...
        n_cross_suppressed;
    stats.cross_suppressed_ids = [stats.cross_suppressed_ids, suppressed_ids];
    stats.cross_target_ids = [stats.cross_target_ids, target_ids];
    [companions, merge_changes, adoption] = adopt_cross_dimension_merges( ...
        companions, merge_records, t, sensor, p);
    stats.cross_dimension_adopted = stats.cross_dimension_adopted + ...
        adoption.n_adopted;
    stats.cross_dimension_tentative_suppressed = ...
        stats.cross_dimension_tentative_suppressed + adoption.n_tentative;
    stats.cross_dimension_late_suppressed = ...
        stats.cross_dimension_late_suppressed + adoption.n_late;
    stats.cross_dimension_bound_suppressed = ...
        stats.cross_dimension_bound_suppressed + adoption.n_already_bound;
    stats.cross_dimension_missing_companion = ...
        stats.cross_dimension_missing_companion + adoption.n_missing_companion;
    transitions = [transitions, merge_changes]; %#ok<AGROW>

    [tracks, n_merged] = merge_tentative_duplicates(tracks, t, sensor, p);
    stats.duplicates_merged = stats.duplicates_merged + n_merged;

    % Confirmation, quality assessment, and hysteretic mode transitions.
    for i = 1:numel(tracks)
        old_mode = tracks(i).mode;
        [tracks(i), reason] = manage_mode(tracks(i), t, sensor, p, quality_cycle);
        if ~strcmp(old_mode, tracks(i).mode)
            transitions(end + 1) = struct('id', tracks(i).id, 't_sec', t, ...
                'from', old_mode, 'to', tracks(i).mode, 'reason', reason); %#ok<AGROW>
        end
    end

    % Remove stale candidates and confirmed tracks only after mode handling.
    keep = true(1, numel(tracks));
    for i = 1:numel(tracks)
        silence = max(t - tracks(i).last_update_t, 0);
        if ~tracks(i).confirmed
            exhausted = tracks(i).age >= p.confirm_N && sum(tracks(i).hit_history) < p.confirm_M;
            if exhausted || silence > p.tentative_timeout_s
                keep(i) = false;
                stats.deleted_tentative = stats.deleted_tentative + 1;
            end
        elseif silence > p.confirmed_timeout_s
            keep(i) = false;
            stats.deleted_confirmed = stats.deleted_confirmed + 1;
        end
    end
    tracks = tracks(keep);
    tracks = tracks(:);

    if numel(tracks) > p.max_tracks
        score = arrayfun(@(x) double(x.confirmed) * 1e6 + sum(x.hit_history), tracks);
        [~, ord] = sort(score, 'descend');
        tracks = tracks(ord(1:p.max_tracks));
        tracks = tracks(:);
    end

    % Diagnostics only: lifecycle/capacity pruning may remove a branch that
    % nevertheless consumed a physical input.  Keep input_dim as routed and
    % set filter_dim=0 when that routed branch did not survive this event.
    assoc = finalize_assoc_dimensions(assoc, tracks, companions);

    confirmed_mask = [tracks.confirmed];
    fresh_mask = arrayfun(@(tr) output_is_fresh(tr, t, p), tracks);
    stats.output_stale_suppressed = stats.output_stale_suppressed + ...
        nnz(confirmed_mask & ~fresh_mask);
    [est, outputs] = store_event(est, tracks, assoc, k, t, sensor, p);
    est.measurement_disposition{k} = event_measurement_disposition( ...
        assoc, e, event_index);
    est.companions{k} = companion_snapshots(companions, t, p);
    if ~quiet && (event_index == 1 || ...
            mod(event_index, p.progress_interval_events) == 0 || k == K)
        dims = [outputs.output_dim];
        elapsed_now = toc(t_start);
        rate = k / max(elapsed_now, eps);
        eta = (K - k) / max(rate, eps);
        fprintf(['  事件 %d/%d t=%.3f: 存活=%d, 输出2D=%d, 输出3D=%d, ', ...
            '耗时=%.1fs, 预计剩余=%.1fs\n'], ...
            event_index, event_offset + K, t, numel(tracks), ...
            nnz(dims == 2), nnz(dims == 3), ...
            elapsed_now, eta);
    end
end

est.timing.total = toc(t_start);
est.transition_log = transitions(transition_offset + 1:end);
est.stats = stats;
stream_state.event_count = event_offset + K;
stream_state.tracks = tracks;
stream_state.companions = companions;
stream_state.next_id = next_id;
stream_state.transitions = transitions;
stream_state.stats = stats;
stream_state.last_t = est.filter_times(end);
stream_state.last_quality_t = last_quality_t;
stream_state.timing_total = stream_state.timing_total + est.timing.total;
if ~quiet
fprintf('统一逻辑航迹滤波完成: %.2fs, 最终存活=%d, 模式切换=%d\n', ...
    est.timing.total, numel(tracks), numel(transitions));
if stats.output_stale_suppressed > 0
    fprintf('  静默航迹正式输出抑制: %d次（内部航迹仍保留至删除超时）\n', ...
        stats.output_stale_suppressed);
end
end
end

function state = init_stream_state(p)
state = struct();
state.schema_version = stream_schema_version();
state.event_count = 0;
state.params = p;
state.tracks = repmat(track_template(p), 0, 1);
state.companions = repmat(track_template(p), 0, 1);
state.next_id = 1;
state.transitions = struct('id', {}, 't_sec', {}, 'from', {}, 'to', {}, 'reason', {});
state.stats = struct('active_assigned', 0, 'passive_assigned', 0, ...
    'active_births', 0, 'passive_births', 0, ...
    'active_birth_suppressed', 0, 'passive_birth_suppressed', 0, ...
    'deleted_tentative', 0, 'deleted_confirmed', 0, 'duplicates_merged', 0, ...
    'cross_dimension_suppressed', 0, 'cross_suppressed_ids', zeros(1, 0), ...
    'cross_target_ids', zeros(1, 0), ...
    'cross_dimension_adopted', 0, ...
    'cross_dimension_tentative_suppressed', 0, ...
    'cross_dimension_late_suppressed', 0, ...
    'cross_dimension_bound_suppressed', 0, ...
    'cross_dimension_missing_companion', 0, ...
    'active_2d_candidate_edges', 0, 'active_2d_accept_edges', 0, ...
    'active_2d_reject_nis', 0, 'active_2d_reject_gap', 0, ...
    'active_2d_reject_az', 0, 'active_2d_reject_el', 0, ...
    'active_2d_reject_los', 0, 'active_2d_released_measurements', 0, ...
    'passive_candidate_edges', 0, 'passive_accept_edges', 0, ...
    'passive_reject_nis', 0, 'passive_reject_gap', 0, ...
    'passive_reject_az', 0, 'passive_reject_el', 0, ...
    'passive_reject_los', 0, 'passive_released_measurements', 0, ...
    'output_stale_suppressed', 0, 'quality_cycles', 0, ...
    'detached_passive_assigned', 0, ...
    'external_rebind_candidates', 0, ...
    'external_rebind_committed', 0, ...
    'external_rebind_ambiguous', 0, ...
    'external_rebind_active_unknown', 0, ...
    'external_rebind_eligible', 0, ...
    'external_rebind_projection_invalid', 0, ...
    'external_rebind_reject_angle', 0, ...
    'external_rebind_reject_rate', 0, ...
    'external_rebind_reject_switch', 0, ...
    'external_rebind_assigned', 0, ...
    'external_rebind_wait_history', 0, ...
    'external_rebind_wait_ready', 0, ...
    'external_rebind_wait_bound', 0, ...
    'switch_3d_to_2d_committed', 0, ...
    'switch_2d_to_3d_committed', 0, ...
    'switch_3d_to_2d_blocked_not_ready', 0);
state.last_t = NaN;
state.last_quality_t = NaN;
state.timing_total = 0;
end

function validate_stream_state(state)
required = {'event_count', 'params', 'tracks', 'next_id', 'transitions', 'stats', ...
    'last_t', 'timing_total'};
for i = 1:numel(required)
    if ~isfield(state, required{i})
        error('run_filter_joint_2d3d:InvalidStreamState', ...
            'Streaming state is missing field %s.', required{i});
    end
end
end

function state = upgrade_stream_state(state, cfg)
current_params = joint_params(cfg);
param_names = fieldnames(current_params);
for i = 1:numel(param_names)
    name = param_names{i};
    if ~isfield(state.params, name)
        state.params.(name) = current_params.(name);
    end
end
defaults = init_stream_state(state.params);
if ~isfield(state, 'companions')
    state.companions = defaults.companions;
end
state.tracks = upgrade_track_array(state.tracks, track_template(state.params));
state.companions = upgrade_track_array( ...
    state.companions, track_template(state.params));
if ~isfield(state, 'last_quality_t')
    state.last_quality_t = defaults.last_quality_t;
end
stat_names = fieldnames(defaults.stats);
for i = 1:numel(stat_names)
    name = stat_names{i};
    if ~isfield(state.stats, name)
        state.stats.(name) = defaults.stats.(name);
    end
end
state.schema_version = stream_schema_version();
end

function version = stream_schema_version()
version = 7;
end

function tracks = upgrade_track_array(tracks, template)
if isempty(tracks)
    tracks = repmat(template, 0, 1);
    return;
end
names = fieldnames(template);
for i = 1:numel(names)
    name = names{i};
    if ~isfield(tracks, name)
        value = template.(name);
        [tracks.(name)] = deal(value);
    end
end
end

function p = joint_params(cfg)
p.confirm_M = get_cfg(cfg, 'joint_confirm_M', 3);
p.confirm_N = get_cfg(cfg, 'joint_confirm_N', 5);
p.gate_2d = get_cfg(cfg, 'joint_gate_2d', 9.2103);
p.gate_3d = get_cfg(cfg, 'joint_gate_3d', 11.3449);
p.angle_assoc_cov_penalty = get_cfg(cfg, ...
    'joint_angle_assoc_cov_penalty', 1.0);
p.unmatched_cost = get_cfg(cfg, 'joint_unmatched_cost', 50);
p.birth_explain_nis = get_cfg(cfg, 'joint_birth_explain_nis', 1.0);
p.accept_nis_2d = get_cfg(cfg, 'joint_2d_accept_nis', 16);
p.max_direct_gap_2d_s = get_cfg(cfg, 'joint_2d_max_direct_gap_s', 2.0);
p.max_az_residual_2d_deg = get_cfg(cfg, 'joint_2d_max_az_residual_deg', 15);
p.max_el_residual_2d_deg = get_cfg(cfg, 'joint_2d_max_el_residual_deg', 5);
p.max_los_residual_2d_deg = get_cfg(cfg, 'joint_2d_max_los_residual_deg', 15);
p.accept_nis_3d_companion = get_cfg(cfg, ...
    'joint_3d_companion_accept_nis', get_cfg(cfg, 'passive_bearing_gate', 16));
p.fast_gate_3d_companion_deg = get_cfg(cfg, ...
    'joint_3d_companion_fast_gate_deg', ...
    get_cfg(cfg, 'joint_passive_fast_gate_deg', inf));
p.max_tracks = get_cfg(cfg, 'max_tracks', 400);
p.tentative_timeout_s = get_cfg(cfg, 'joint_tentative_timeout_s', 3);
p.confirmed_timeout_s = get_cfg(cfg, 'joint_confirmed_timeout_s', 12);
p.output_max_silence_s = get_cfg(cfg, 'joint_2d_output_max_silence_s', 0.5);
p.angle_q_cv = get_cfg(cfg, 'joint_angle_q_cv', 0.02);
p.angle_q_ca = get_cfg(cfg, 'joint_angle_q_ca', 0.12);
p.angle_rate_std = get_cfg(cfg, 'joint_angle_rate_birth_std_dps', 0.75);
p.angle_acc_std = get_cfg(cfg, 'joint_angle_acc_birth_std_dps2', 1.0);
p.space_sigma_a_cv = get_cfg(cfg, 'joint_space_sigma_a_cv', 12);
p.space_sigma_j_ca = get_cfg(cfg, 'joint_space_sigma_j_ca', 15);
p.space_vel_std = get_cfg(cfg, 'joint_space_vel_birth_std_mps', 500);
p.space_acc_std = get_cfg(cfg, 'joint_space_acc_birth_std_mps2', 80);
p.imm_tpm = [get_cfg(cfg, 'joint_imm_cv_stay', 0.96), 1 - get_cfg(cfg, 'joint_imm_cv_stay', 0.96); ...
    1 - get_cfg(cfg, 'joint_imm_ca_stay', 0.94), get_cfg(cfg, 'joint_imm_ca_stay', 0.94)];
p.imm_mu0 = [get_cfg(cfg, 'joint_imm_cv_probability', 0.65); ...
    1 - get_cfg(cfg, 'joint_imm_cv_probability', 0.65)];
p.angle95_max_deg = get_cfg(cfg, 'joint_angle95_max_deg', 1.0);
p.birth3_M = get_cfg(cfg, 'joint_3d_birth_M', 3);
p.birth3_N = get_cfg(cfg, 'joint_3d_birth_N', 5);
p.upgrade3_M = get_cfg(cfg, 'joint_3d_upgrade_M', 2);
p.upgrade3_N = get_cfg(cfg, 'joint_3d_upgrade_N', 3);
p.up_consecutive = get_cfg(cfg, 'joint_up_consecutive', 1);
p.down_consecutive = get_cfg(cfg, 'joint_down_consecutive', 3);
p.quality_eval_interval_s = get_cfg(cfg, 'joint_quality_eval_interval_s', 0.10);
p.switch_gate = get_cfg(cfg, 'joint_switch_gate_2d', 9.2103);
p.switch_cov_inflate = get_cfg(cfg, 'joint_switch_cov_inflate', 2.0);
p.pos95_warn_m = get_cfg(cfg, 'joint_pos95_warn_m', 15000);
p.pos95_down_m = get_cfg(cfg, 'joint_pos95_down_m', 30000);
p.pos95_recover_m = get_cfg(cfg, 'joint_pos95_recover_m', 10000);
p.radial95_warn_m = get_cfg(cfg, 'joint_radial95_warn_m', 10000);
p.radial95_down_m = get_cfg(cfg, 'joint_radial95_down_m', 20000);
p.radial95_recover_m = get_cfg(cfg, 'joint_radial95_recover_m', 7000);
p.relative_range_down = get_cfg(cfg, 'joint_relative_range_down', 0.60);
p.space_nis_window = get_cfg(cfg, 'joint_space_nis_window', 5);
p.space_nis_recover = get_cfg(cfg, 'joint_space_nis_recover', 1.5);
p.space_nis_warn = get_cfg(cfg, 'joint_space_nis_warn', 2.5);
p.space_nis_down = get_cfg(cfg, 'joint_space_nis_down', 4.0);
p.shadow_max_s = get_cfg(cfg, 'joint_shadow_max_s', 20);
p.mode_3d_prior_cost = get_cfg(cfg, 'joint_mode_3d_prior_cost', 0.5);
p.merge_angle_deg = get_cfg(cfg, 'joint_merge_angle_deg', 0.08);
p.merge_rate_dps = get_cfg(cfg, 'joint_merge_rate_dps', 1.0);
p.merge_nis = get_cfg(cfg, 'joint_merge_nis', 13.2767);
p.max_dt = get_cfg(cfg, 'joint_max_predict_dt_s', 5);
p.projection_cache_enabled = logical(get_cfg(cfg, 'joint_projection_cache_enabled', true));
p.progress_interval_events = max(1, floor(get_cfg(cfg, 'joint_progress_interval_events', 50)));
p.passive_confirm_group = max(1, round(get_cfg(cfg, ...
    'joint_passive_confirm_group_size', 1)));
p.passive_confirm_max_gap_s = get_cfg(cfg, ...
    'joint_passive_confirm_max_gap_s', inf);
p.passive_confirmation_cycles_only = logical(get_cfg(cfg, ...
    'joint_passive_confirmation_cycles_only', false));
p.cross_suppress_angle_deg = get_cfg(cfg, 'joint_2d3d_fusion_angle_deg', 0.30);
p.cross_suppress_rate_dps = get_cfg(cfg, 'joint_2d3d_fusion_rate_dps', 1.5);
p.cross_suppress_min_cycles = max(1, round(get_cfg(cfg, ...
    'joint_2d3d_fusion_min_cycles', 3)));
p.dimension_switch_enabled = logical(get_cfg(cfg, ...
    'joint_dimension_switch_enabled', true));
p.cross_upgrade_cycles = max(1, round(get_cfg(cfg, ...
    'joint_2d3d_upgrade_match_cycles', 2)));
p.rebind_enabled = logical(get_cfg(cfg, 'joint_external_rebind_enabled', true));
p.rebind_ambiguity_nis = get_cfg(cfg, 'joint_external_rebind_ambiguity_nis', 1.0);
p.rebind_max_gap_s = get_cfg(cfg, 'joint_external_rebind_max_gap_s', 2.0);
end

function tf = confirmation_cycle(e, p)
if ~p.passive_confirmation_cycles_only
    tf = true;
elseif isfield(e, 'confirm_cycle') && ~isempty(e.confirm_cycle)
    tf = logical(e.confirm_cycle);
else
    tf = logical(e.has_passive);
end
end

function tf = quality_assessment_cycle(e, t, last_t, p)
active_opportunity = logical(e.has_active);
if isfield(e, 'external_3d') && isstruct(e.external_3d) && ...
        isfield(e.external_3d, 'active_opportunity')
    active_opportunity = active_opportunity || ...
        logical(e.external_3d.active_opportunity);
end
tf = active_opportunity || ~isfinite(last_t) || ...
    p.quality_eval_interval_s == 0 || ...
    t - last_t >= p.quality_eval_interval_s - 1e-12;
end

function [tr, equivalent_hit] = register_passive_confirm_hit(tr, t, p, kind)
if kind == 2 || p.passive_confirm_group <= 1
    equivalent_hit = true;
    tr.passive_confirm_streak = 0;
    return;
end
continuous = isfinite(tr.last_passive_t) && ...
    t >= tr.last_passive_t && ...
    t - tr.last_passive_t <= p.passive_confirm_max_gap_s;
if continuous
    tr.passive_confirm_streak = tr.passive_confirm_streak + 1;
else
    tr.passive_confirm_streak = 1;
end
equivalent_hit = tr.passive_confirm_streak >= p.passive_confirm_group;
if equivalent_hit
    tr.passive_confirm_streak = 0;
end
end

function kind = passive_measurement_kind(meas, idx)
kind = 1;
if isfield(meas, 'kind') && idx >= 1 && idx <= numel(meas.kind) && ...
        isfinite(meas.kind(idx))
    kind = meas.kind(idx);
end
end

function tr = track_template(p)
tr = struct('id', 0, 'birth_event', 0, 'birth_t', NaN, 'confirmed', false, ...
    'external_id', NaN, 'external_fresh', false, 'from_2d', false, ...
    'previous_external_id', NaN, 'rebind_candidate_id', NaN, ...
    'rebind_last_hit_t', NaN, 'rebind_candidate_since', NaN, ...
    'rebind_history', zeros(1, p.upgrade3_N), ...
    'active_output_dim', 0, 'switch_candidate_since', NaN, ...
    'switch_commit_t', NaN, ...
    'mode', 'tentative', 'age', 0, 'miss', 0, 'last_t', NaN, ...
    'last_update_t', NaN, 'last_active_t', NaN, 'last_passive_t', NaN, ...
    'last_angle_observation_t', NaN, ...
    'passive_confirm_streak', 0, ...
    'cross_candidate_id', NaN, 'cross_candidate_count', 0, ...
    'hit_history', zeros(1, p.confirm_N), ...
    'active_history', zeros(1, max(p.birth3_N, p.upgrade3_N)), ...
    'angle', invalid_branch(6), 'space', invalid_branch(9), ...
    'quality', quality_template(), 'up_count', 0, 'down_count', 0, ...
    'truth_id', NaN, 'truth_conf', 0);
end

function b = invalid_branch(n)
b = struct('valid', false, 'x', nan(n, 1), 'P', nan(n), ...
    'xm', nan(n, 2), 'Pm', nan(n, n, 2), 'mu', [0.5; 0.5], ...
    'last_t', NaN, 'last_nis', NaN, 'last_nis_norm', NaN, ...
    'nis_history', zeros(1, 0), 'nis_norm_history', zeros(1, 0));
end

function tr = new_track(id, birth_event, t, ae, R_ae, xyz, R_xyz, has_active, p)
tr = track_template(p);
tr.id = id; tr.birth_event = birth_event; tr.birth_t = t;
tr.last_t = t; tr.last_update_t = t; tr.age = 1;
tr.hit_history(end) = 1;
tr.angle = init_angle_branch(ae, R_ae, t, p);
if has_active
    tr.space = init_space_branch(xyz, R_xyz, t, p);
    tr.active_history(end) = 1;
    tr.last_active_t = t;
else
    tr.last_angle_observation_t = t;
end
end

function b = init_angle_branch(ae, R, t, p)
b = invalid_branch(6);
b.valid = true;
b.x = [wrap_az(ae(1)); 0; 0; ae(2); 0; 0];
P0 = diag([max(R(1, 1), 1e-6), p.angle_rate_std^2, p.angle_acc_std^2, ...
    max(R(2, 2), 1e-6), p.angle_rate_std^2, p.angle_acc_std^2]);
b.P = P0; b.xm = repmat(b.x, 1, 2); b.Pm = repmat(P0, 1, 1, 2);
b.mu = p.imm_mu0; b.last_t = t;
end

function b = init_space_branch(xyz, R, t, p)
b = invalid_branch(9);
b.valid = true;
b.x = zeros(9, 1); b.x([1, 4, 7]) = xyz;
P0 = zeros(9);
for d = 1:3
    ii = (d - 1) * 3 + (1:3);
    P0(ii, ii) = diag([max(R(d, d), 1), p.space_vel_std^2, p.space_acc_std^2]);
end
b.P = make_spd(P0); b.xm = repmat(b.x, 1, 2); b.Pm = repmat(b.P, 1, 1, 2);
b.mu = p.imm_mu0; b.last_t = t;
end

function tr = predict_track(tr, t, p)
remaining = max(t - tr.last_t, 0);
while remaining > 0
    dt = min(remaining, p.max_dt);
    if tr.angle.valid
        tr.angle = predict_angle_branch(tr.angle, dt, p);
    end
    if tr.space.valid
        tr.space = predict_space_branch(tr.space, dt, p);
    end
    remaining = max(remaining - dt, 0);
end
if tr.angle.valid, tr.angle.last_t = t; end
if tr.space.valid, tr.space.last_t = t; end
tr.last_t = t;
end

function b = predict_angle_branch(b, dt, p)
[x0, P0, cbar] = imm_mix(b.xm, b.Pm, b.mu, p.imm_tpm, true);
for j = 1:2
    if j == 1
        [F, Q] = angle_model(dt, p.angle_q_cv, true);
    else
        [F, Q] = angle_model(dt, p.angle_q_ca, false);
    end
    b.xm(:, j) = F * x0(:, j);
    b.xm(1, j) = wrap_az(b.xm(1, j));
    b.Pm(:, :, j) = make_spd(F * P0(:, :, j) * F' + Q);
end
b.mu = cbar;
[b.x, b.P] = imm_combine(b.xm, b.Pm, b.mu, true);
end

function b = predict_space_branch(b, dt, p)
[x0, P0, cbar] = imm_mix(b.xm, b.Pm, b.mu, p.imm_tpm, false);
for j = 1:2
    if j == 1
        [F, Q] = space_model(dt, p.space_sigma_a_cv, true);
    else
        [F, Q] = space_model(dt, p.space_sigma_j_ca, false);
    end
    b.xm(:, j) = F * x0(:, j);
    b.Pm(:, :, j) = make_spd(F * P0(:, :, j) * F' + Q);
end
b.mu = cbar;
[b.x, b.P] = imm_combine(b.xm, b.Pm, b.mu, false);
end

function [F, Q] = angle_model(dt, q, is_cv)
if is_cv
    f = [1 dt 0; 0 1 0; 0 0 0];
    q1 = q * [dt^3/3 dt^2/2 0; dt^2/2 max(dt, 1e-6) 0; 0 0 1];
else
    f = [1 dt 0.5*dt^2; 0 1 dt; 0 0 1];
    q1 = q * [dt^5/20 dt^4/8 dt^3/6; dt^4/8 dt^3/3 dt^2/2; dt^3/6 dt^2/2 max(dt, 1e-6)];
end
F = blkdiag(f, f); Q = blkdiag(q1, q1);
end

function [F, Q] = space_model(dt, q, is_cv)
if is_cv
    f = [1 dt 0; 0 1 0; 0 0 0];
    q1 = q^2 * [dt^3/3 dt^2/2 0; dt^2/2 max(dt, 1e-6) 0; 0 0 1];
else
    f = [1 dt 0.5*dt^2; 0 1 dt; 0 0 1];
    q1 = q^2 * [dt^5/20 dt^4/8 dt^3/6; dt^4/8 dt^3/3 dt^2/2; dt^3/6 dt^2/2 max(dt, 1e-6)];
end
F = blkdiag(f, f, f); Q = blkdiag(q1, q1, q1);
end

function [x0, P0, cbar] = imm_mix(xm, Pm, mu, TPM, circular)
n = size(xm, 1); M = size(xm, 2);
cbar = TPM' * mu; cbar = max(cbar, realmin); cbar = cbar / sum(cbar);
x0 = zeros(n, M); P0 = zeros(n, n, M);
for j = 1:M
    w = TPM(:, j) .* mu / max(cbar(j), realmin);
    w = w / sum(w);
    x0(:, j) = xm * w;
    if circular
        ref = xm(1, 1);
        x0(1, j) = wrap_az(ref + sum(w(:)' .* angle_diff(xm(1, :), ref)));
    end
    for i = 1:M
        dx = xm(:, i) - x0(:, j);
        if circular, dx(1) = angle_diff(xm(1, i), x0(1, j)); end
        P0(:, :, j) = P0(:, :, j) + w(i) * (Pm(:, :, i) + dx * dx');
    end
    P0(:, :, j) = make_spd(P0(:, :, j));
end
end

function [x, P] = imm_combine(xm, Pm, mu, circular)
x = xm * mu;
if circular
    ref = xm(1, 1);
    x(1) = wrap_az(ref + sum(mu(:)' .* angle_diff(xm(1, :), ref)));
end
P = zeros(size(Pm, 1));
for j = 1:numel(mu)
    dx = xm(:, j) - x;
    if circular, dx(1) = angle_diff(xm(1, j), x(1)); end
    P = P + mu(j) * (Pm(:, :, j) + dx * dx');
end
P = make_spd(P);
end

function [pairs, C, accept_info] = associate_active( ...
        tracks, meas, t, platform, cfg, sensor, p)
C = inf(numel(tracks), meas.n_meas);
projection_cache = build_space_bearing_cache(tracks, sensor, ...
    p.projection_cache_enabled, any(~meas.has_range));
rae_lambda = covariance_max_eigenvalues(meas.R_ae);
xyz_lambda = covariance_max_eigenvalues(meas.R_xyz);
for i = 1:numel(tracks)
    if p.projection_cache_enabled
        C(i, :) = active_cost_row_cached(tracks(i), meas, projection_cache(i), ...
            rae_lambda, xyz_lambda, p);
    else
        for j = 1:meas.n_meas
            C(i, j) = active_cost(tracks(i), meas, j, t, platform, cfg, p);
        end
    end
end
[accept_mask, accept_info] = active_2d_acceptance_mask( ...
    C, tracks, meas, t, p);
C(~accept_mask) = inf;
pairs = cascade_assignment(C, tracks, p.unmatched_cost);
end

function [accept, info] = active_2d_acceptance_mask(C, tracks, meas, t, p)
candidate = isfinite(C);
accept = candidate;
strict_candidate = false(size(C));
reject_nis = false(size(C)); reject_gap = false(size(C));
reject_az = false(size(C)); reject_el = false(size(C));
reject_los = false(size(C));
angles = meas.rae(2:3, :);
for i = 1:numel(tracks)
    requires_space = starts_with(tracks(i).mode, '3d') || ...
        (~tracks(i).confirmed && tracks(i).space.valid);
    if requires_space || ~tracks(i).angle.valid
        continue;
    end
    for j = find(candidate(i, :))
        strict_candidate(i, j) = true;
        [accept(i, j), reject_nis(i, j), reject_gap(i, j), ...
            reject_az(i, j), reject_el(i, j), reject_los(i, j)] = ...
            strict_2d_edge_acceptance( ...
                tracks(i), angles(:, j), C(i, j), t, p);
    end
end
info = struct('n_candidate_edges', nnz(strict_candidate), ...
    'n_accept_edges', nnz(strict_candidate & accept), ...
    'n_reject_nis', nnz(reject_nis), ...
    'n_reject_gap', nnz(reject_gap), ...
    'n_reject_az', nnz(reject_az), ...
    'n_reject_el', nnz(reject_el), ...
    'n_reject_los', nnz(reject_los), ...
    'released_measurements', any(strict_candidate, 1) & ~any(accept, 1));
end

function row = active_cost_row_cached(tr, m, cache, rae_lambda, xyz_lambda, p)
row = inf(1, m.n_meas);
requires_space = starts_with(tr.mode, '3d') || (~tr.confirmed && tr.space.valid);
if requires_space
    c3 = active_space_cost_row(tr, m, cache, rae_lambda, xyz_lambda, ...
        1:m.n_meas, p);
    good = isfinite(c3);
    row(good) = max(0, c3(good) - p.mode_3d_prior_cost);
    return;
end
if tr.angle.valid
    nu = [angle_diff(m.rae(2, :), tr.angle.x(1)); ...
        m.rae(3, :) - tr.angle.x(4)];
    candidates = nis_candidate_mask(nu, p.gate_2d, cache.angle_lambda, rae_lambda);
    for j = find(candidates)
        c2 = angle_nis(tr.angle, m.rae(2:3, j), m.R_ae(:, :, j));
        if isfinite(c2) && c2 <= p.gate_2d, row(j) = c2; end
    end
end
end

function row = active_space_cost_row(tr, m, cache, rae_lambda, xyz_lambda, indices, p)
row = inf(1, numel(indices));
if ~tr.space.valid || isempty(indices), return; end
range_local = find(m.has_range(indices));
if ~isempty(range_local)
    jj = indices(range_local);
    nu = m.xyz(:, jj) - tr.space.x([1, 4, 7]);
    candidates = nis_candidate_mask(nu, p.gate_3d, cache.position_lambda, ...
        xyz_lambda(jj));
    for q = find(candidates)
        j = jj(q);
        c3 = space_position_nis(tr.space, m.xyz(:, j), m.R_xyz(:, :, j));
        if isfinite(c3) && c3 <= p.gate_3d, row(range_local(q)) = c3; end
    end
end
bearing_local = find(~m.has_range(indices));
if ~isempty(bearing_local) && cache.valid
    jj = indices(bearing_local);
    nu = [angle_diff(m.rae(2, jj), cache.zp(1)); m.rae(3, jj) - cache.zp(2)];
    candidates = nis_candidate_mask(nu, p.gate_2d, ...
        cache.bearing_lambda, rae_lambda(jj));
    for q = find(candidates)
        j = jj(q);
        row(bearing_local(q)) = cached_bearing_nis(cache, m.rae(2:3, j), ...
            m.R_ae(:, :, j), false, p.gate_2d, rae_lambda(j));
    end
end
end

function c = active_cost(tr, m, j, t, platform, cfg, p)
c2 = inf; c3 = inf;
if tr.angle.valid
    c2 = angle_nis(tr.angle, m.rae(2:3, j), m.R_ae(:, :, j));
    if c2 > p.gate_2d, c2 = inf; end
end
if tr.space.valid
    if m.has_range(j)
        c3 = space_position_nis(tr.space, m.xyz(:, j), m.R_xyz(:, :, j));
        if c3 > p.gate_3d, c3 = inf; end
    else
        c3 = space_bearing_nis(tr.space, m.rae(2:3, j), m.R_ae(:, :, j), ...
            t, platform, cfg);
        if c3 > p.gate_2d, c3 = inf; end
    end
end
requires_space = starts_with(tr.mode, '3d') || (~tr.confirmed && tr.space.valid);
if requires_space
    if isfinite(c3), c = max(0, c3 - p.mode_3d_prior_cost); else, c = inf; end
elseif isfinite(c2)
    c = c2;
else
    c = inf;
end
end

function [pairs, C, assignment_cost, accept_info] = associate_passive( ...
        tracks, meas, t, platform, cfg, sensor, p, blocked_measurements)
if nargin < 9 || isempty(blocked_measurements)
    blocked_measurements = false(1, meas.n_meas);
else
    blocked_measurements = logical(sized_row( ...
        blocked_measurements, meas.n_meas, false));
end
C = inf(numel(tracks), meas.n_meas);
projection_cache = build_space_bearing_cache(tracks, sensor, ...
    p.projection_cache_enabled, true);
rae_lambda = covariance_max_eigenvalues(meas.R_ae);
for i = 1:numel(tracks)
    if p.projection_cache_enabled
        C(i, :) = passive_cost_row_cached(tracks(i), meas, projection_cache(i), ...
            rae_lambda, p);
    else
        for j = 1:meas.n_meas
            C(i, j) = passive_cost(tracks(i), meas, j, t, platform, cfg, p);
        end
    end
end
C(:, blocked_measurements) = inf;
assignment_cost = passive_assignment_cost(C, tracks, meas, p);
acceptance_cache = projection_cache;
if ~p.projection_cache_enabled && any(arrayfun(@(x) x.space.valid, tracks))
    acceptance_cache = build_space_bearing_cache(tracks, sensor, true, true);
end
[accept_mask, accept_info] = passive_acceptance_mask( ...
    C, tracks, meas, t, p, acceptance_cache);
assignment_cost(~accept_mask) = inf;
pairs = cascade_assignment(assignment_cost, tracks, p.unmatched_cost);
end

function total = add_accept_info(total, extra)
names = {'n_candidate_edges', 'n_accept_edges', 'n_reject_nis', ...
    'n_reject_gap', 'n_reject_az', 'n_reject_el', 'n_reject_los'};
for i = 1:numel(names)
    total.(names{i}) = total.(names{i}) + extra.(names{i});
end
total.released_measurements = total.released_measurements | ...
    extra.released_measurements;
end

function info = empty_accept_info(n_meas)
info = struct('n_candidate_edges', 0, 'n_accept_edges', 0, ...
    'n_reject_nis', 0, 'n_reject_gap', 0, 'n_reject_az', 0, ...
    'n_reject_el', 0, 'n_reject_los', 0, ...
    'released_measurements', false(1, n_meas));
end

function [accept, info] = passive_acceptance_mask( ...
        C, tracks, meas, t, p, projection_cache)
candidate = isfinite(C);
accept = false(size(C));
reject_nis = false(size(C)); reject_gap = false(size(C));
reject_az = false(size(C)); reject_el = false(size(C));
reject_los = false(size(C));
for i = 1:numel(tracks)
    requires_space = starts_with(tracks(i).mode, '3d') || ...
        (~tracks(i).confirmed && tracks(i).space.valid);
    for j = find(candidate(i, :))
        if requires_space
            predicted = projection_cache(i).center;
            if numel(predicted) < 2 || any(~isfinite(predicted(1:2)))
                continue;
            end
            residual_az = abs(angle_diff(meas.ang(1, j), predicted(1)));
            residual_el = abs(meas.ang(2, j) - predicted(2));
            reject_nis(i, j) = C(i, j) > p.accept_nis_3d_companion;
            reject_az(i, j) = residual_az > p.fast_gate_3d_companion_deg;
            reject_el(i, j) = residual_el > p.fast_gate_3d_companion_deg;
            accept(i, j) = ~(reject_nis(i, j) || ...
                reject_az(i, j) || reject_el(i, j));
            continue;
        end
        if ~tracks(i).angle.valid
            continue;
        end
        [accept(i, j), reject_nis(i, j), reject_gap(i, j), ...
            reject_az(i, j), reject_el(i, j), reject_los(i, j)] = ...
            strict_2d_edge_acceptance( ...
                tracks(i), meas.ang(:, j), C(i, j), t, p);
    end
end
info = struct('n_candidate_edges', nnz(candidate), ...
    'n_accept_edges', nnz(accept), ...
    'n_reject_nis', nnz(reject_nis), ...
    'n_reject_gap', nnz(reject_gap), ...
    'n_reject_az', nnz(reject_az), ...
    'n_reject_el', nnz(reject_el), ...
    'n_reject_los', nnz(reject_los), ...
    'released_measurements', any(candidate, 1) & ~any(accept, 1));
end

function [accept, reject_nis, reject_gap, reject_az, reject_el, reject_los] = ...
        strict_2d_edge_acceptance(track, angle, cost, t, p)
predicted = track.angle.x([1, 4]);
residual_az = abs(angle_diff(angle(1), predicted(1)));
residual_el = abs(angle(2) - predicted(2));
residual_los = los_separation_deg(predicted, angle);
gap_s = max(t - track.last_update_t, 0);
reject_nis = cost > p.accept_nis_2d;
reject_gap = gap_s > p.max_direct_gap_2d_s;
reject_az = residual_az > p.max_az_residual_2d_deg;
reject_el = residual_el > p.max_el_residual_2d_deg;
reject_los = residual_los > p.max_los_residual_2d_deg;
accept = ~(reject_nis || reject_gap || reject_az || reject_el || reject_los);
end

function sep = los_separation_deg(a, b)
ua = [cosd(a(2)) * sind(a(1)); cosd(a(2)) * cosd(a(1)); sind(a(2))];
ub = [cosd(b(2)) * sind(b(1)); cosd(b(2)) * cosd(b(1)); sind(b(2))];
sep = acosd(min(max(dot(ua, ub), -1), 1));
end

function A = passive_assignment_cost(C, tracks, meas, p)
% Keep raw NIS in C for gating and birth explanation. For one-to-one
% assignment, penalize broad predicted angle covariance so an uncertain
% track cannot win a measurement merely because its normalized NIS is low.
A = C;
w = p.angle_assoc_cov_penalty;
if w <= 0 || isempty(C), return; end
ii = [1, 4];
R_logdet = inf(1, meas.n_meas);
for j = 1:meas.n_meas
    R_logdet(j) = spd_logdet(meas.R_ae(:, :, j));
end
for i = 1:numel(tracks)
    if starts_with(tracks(i).mode, '3d') || ~tracks(i).angle.valid
        continue;
    end
    Pz = tracks(i).angle.P(ii, ii);
    for j = find(isfinite(C(i, :)))
        R = make_spd(meas.R_ae(:, :, j));
        S = make_spd(Pz + R);
        log_ratio = spd_logdet(S) - R_logdet(j);
        if isfinite(log_ratio)
            A(i, j) = C(i, j) + w * max(log_ratio, 0);
        end
    end
end
end

function value = spd_logdet(S)
[L, flag] = chol(make_spd(S), 'lower');
if flag == 0
    value = 2 * sum(log(max(diag(L), realmin)));
else
    value = inf;
end
end

function row = passive_cost_row_cached(tr, m, cache, rae_lambda, p)
row = inf(1, m.n_meas);
if starts_with(tr.mode, '3d')
    c3 = passive_space_cost_row(tr, m, cache, rae_lambda, 1:m.n_meas, p);
    good = isfinite(c3);
    row(good) = max(0, c3(good) - p.mode_3d_prior_cost);
    return;
end
if tr.angle.valid
    nu = [angle_diff(m.ang(1, :), tr.angle.x(1)); ...
        m.ang(2, :) - tr.angle.x(4)];
    candidates = nis_candidate_mask(nu, p.gate_2d, cache.angle_lambda, rae_lambda);
    for j = find(candidates)
        c2 = angle_nis(tr.angle, m.ang(:, j), m.R_ae(:, :, j));
        if isfinite(c2) && c2 <= p.gate_2d, row(j) = c2; end
    end
end
fallback = find(~isfinite(row));
if isempty(fallback), return; end
c3 = passive_space_cost_row(tr, m, cache, rae_lambda, fallback, p);
good = isfinite(c3);
row(fallback(good)) = c3(good);
end

function row = passive_space_cost_row(tr, m, cache, rae_lambda, indices, p)
row = inf(1, numel(indices));
if ~tr.space.valid || ~cache.valid || isempty(indices), return; end
nu = [angle_diff(m.ang(1, indices), cache.center(1)); ...
    m.ang(2, indices) - cache.center(2)];
candidates = nis_candidate_mask(nu, p.gate_2d, cache.bearing_lambda, ...
    rae_lambda(indices));
for q = find(candidates)
    j = indices(q);
    row(q) = cached_bearing_nis(cache, m.ang(:, j), m.R_ae(:, :, j), ...
        true, p.gate_2d, rae_lambda(j));
end
end

function cache = build_space_bearing_cache(tracks, sensor, enabled, build_bearing)
template = struct('valid', false, 'zp', nan(2, 1), ...
    'center', nan(2, 1), 'S0', nan(2), ...
    'angle_lambda', inf, 'position_lambda', inf, 'bearing_lambda', inf);
cache = repmat(template, numel(tracks), 1);
if ~enabled || isempty(tracks), return; end
for i = 1:numel(tracks)
    if tracks(i).angle.valid
        cache(i).angle_lambda = max(real(eig( ...
            tracks(i).angle.P([1, 4], [1, 4]))));
    end
    if tracks(i).space.valid
        cache(i).position_lambda = max(real(eig( ...
            tracks(i).space.P([1, 4, 7], [1, 4, 7]))));
    end
end
if ~build_bearing || ~any(arrayfun(@(x) x.space.valid, tracks))
    return;
end
for i = 1:numel(tracks)
    if ~tracks(i).space.valid, continue; end
    [zp, S0, ~, ok] = space_bearing_moments( ...
        tracks(i).space.x, tracks(i).space.P, sensor);
    if ~ok, continue; end
    cache(i).valid = true;
    cache(i).zp = zp;
    cache(i).center = space_to_ae_sensor(tracks(i).space.x([1, 4, 7]), sensor);
    cache(i).S0 = S0;
    cache(i).bearing_lambda = max(real(eig(S0)));
end
end

function nis = cached_bearing_nis(cache, z, R, use_state_center, gate, r_lambda)
nis = inf;
if ~cache.valid, return; end
if use_state_center, zp = cache.center; else, zp = cache.zp; end
nu = [angle_diff(z(1), zp(1)); z(2) - zp(2)];
if outside_nis_lower_bound(nu, gate, cache.bearing_lambda, r_lambda), return; end
S = make_spd(cache.S0 + R);
if any(~isfinite(S(:))) || rcond(S) <= 1e-12, return; end
nis = nu' * (S \ nu);
if nis > gate, nis = inf; end
end

function values = covariance_max_eigenvalues(R)
n = size(R, 3);
values = inf(1, n);
for i = 1:n
    Ri = R(:, :, i);
    if all(isfinite(Ri(:)))
        values(i) = max(real(eig(0.5 * (Ri + Ri'))));
    end
end
end

function tf = outside_nis_lower_bound(nu, gate, state_lambda, meas_lambda)
lambda_upper = max(state_lambda + meas_lambda, 1e-10);
tf = all(isfinite(nu)) && isfinite(lambda_upper) && ...
    sum(nu.^2) > gate * lambda_upper * (1 + 1e-12);
end

function mask = nis_candidate_mask(nu, gate, state_lambda, meas_lambda)
lambda_upper = max(state_lambda + meas_lambda, 1e-10);
outside = all(isfinite(nu), 1) & isfinite(lambda_upper) & ...
    sum(nu.^2, 1) > gate .* lambda_upper .* (1 + 1e-12);
mask = ~outside;
end

function c = passive_cost(tr, m, j, t, platform, cfg, p)
c2 = inf; c3 = inf;
if tr.angle.valid
    c2 = angle_nis(tr.angle, m.ang(:, j), m.R_ae(:, :, j));
    if c2 > p.gate_2d, c2 = inf; end
end
if tr.space.valid
    [~, S, ~, ok] = space_bearing_stats(tr.space.x, tr.space.P, t, platform, cfg, m.R_ae(:, :, j));
    if ok
        z3 = space_to_ae(tr.space.x([1, 4, 7]), t, platform, cfg);
        nu = [angle_diff(m.ang(1, j), z3(1)); m.ang(2, j) - z3(2)];
        c3 = nu' * (S \ nu);
        if c3 > p.gate_2d, c3 = inf; end
    end
end
if starts_with(tr.mode, '3d')
    if isfinite(c3), c = max(0, c3 - p.mode_3d_prior_cost); else, c = inf; end
elseif isfinite(c2)
    c = c2;
elseif isfinite(c3)
    c = c3;
else
    c = inf;
end
end

function pairs = cascade_assignment(C, tracks, unmatched)
pairs = zeros(0, 2);
if isempty(C), return; end
available = true(1, size(C, 2));
tiers = {find([tracks.confirmed]), find(~[tracks.confirmed])};
for q = 1:2
    rows = tiers{q}; cols = find(available);
    if isempty(rows) || isempty(cols), continue; end
    local = solve_assignment(C(rows, cols), unmatched);
    if isempty(local), continue; end
    add = [rows(local(:, 1)).', cols(local(:, 2)).'];
    pairs = [pairs; add]; %#ok<AGROW>
    available(add(:, 2)) = false;
end
end

function pairs = solve_assignment(C, unmatched)
pairs = solve_global_assignment(C, unmatched);
end

function nis = angle_nis(b, z, R)
H = zeros(2, 6); H(1, 1) = 1; H(2, 4) = 1;
nu = [angle_diff(z(1), b.x(1)); z(2) - b.x(4)];
S = make_spd(H * b.P * H' + R);
nis = nu' * (S \ nu);
end

function [innovation, nis, kind] = active_association_diagnostic(tr, m, j, sensor)
innovation = nan(3, 1); nis = NaN; kind = 0;
if m.has_range(j) && tr.space.valid
    H = zeros(3, 9); H(:, [1, 4, 7]) = eye(3);
    nu = m.xyz(:, j) - H * tr.space.x;
    S = make_spd(H * tr.space.P * H' + m.R_xyz(:, :, j));
    innovation = nu; nis = nu' * (S \ nu); kind = 2;
elseif tr.angle.valid
    H = zeros(2, 6); H(1, 1) = 1; H(2, 4) = 1;
    nu = [angle_diff(m.rae(2, j), tr.angle.x(1)); ...
        m.rae(3, j) - tr.angle.x(4)];
    S = make_spd(H * tr.angle.P * H' + m.R_ae(:, :, j));
    innovation(1:2) = nu; nis = nu' * (S \ nu); kind = 1;
elseif tr.space.valid
    [zp, S, ~, ok] = space_bearing_stats_sensor( ...
        tr.space.x, tr.space.P, sensor, m.R_ae(:, :, j));
    if ok
        nu = [angle_diff(m.rae(2, j), zp(1)); m.rae(3, j) - zp(2)];
        innovation(1:2) = nu; nis = nu' * (S \ nu); kind = 1;
    end
end
end

function [innovation, nis] = passive_association_diagnostic(tr, m, j, sensor)
innovation = nan(3, 1); nis = NaN;
z = m.ang(:, j); R = m.R_ae(:, :, j);
if starts_with(tr.mode, '3d') && tr.space.valid
    [zp, S, ~, ok] = space_bearing_stats_sensor(tr.space.x, tr.space.P, sensor, R);
    if ~ok, return; end
    nu = [angle_diff(z(1), zp(1)); z(2) - zp(2)];
elseif tr.angle.valid
    H = zeros(2, 6); H(1, 1) = 1; H(2, 4) = 1;
    nu = [angle_diff(z(1), tr.angle.x(1)); z(2) - tr.angle.x(4)];
    S = make_spd(H * tr.angle.P * H' + R);
elseif tr.space.valid
    [zp, S, ~, ok] = space_bearing_stats_sensor(tr.space.x, tr.space.P, sensor, R);
    if ~ok, return; end
    nu = [angle_diff(z(1), zp(1)); z(2) - zp(2)];
else
    return;
end
innovation(1:2) = nu;
nis = nu' * (S \ nu);
end

function [companions, used, assoc, accept_info] = ...
        associate_detached_companions(companions, event, t, platform, cfg, sensor, p)
n_meas = event.passive.n_meas;
used = false(1, n_meas);
assoc = assoc_template();
accept_info = empty_accept_info(n_meas);
if n_meas == 0 || isempty(companions), return; end

external_ids = current_external_ids(event);
bound = ismember([companions.external_id], external_ids);
angle_eligible = reshape(arrayfun(@(x) x.angle.valid && ...
    (starts_with(x.mode, '2d') || isfinite(x.rebind_candidate_id)), ...
    companions), 1, []);
eligible = find([companions.confirmed] & ~bound & ...
    angle_eligible);
if isempty(eligible), return; end

candidates = companions(eligible);
for i = 1:numel(candidates)
    % Detached logical tracks are associated only in angle space. Their old
    % radial shadow must not win a bearing through a broad 3-D covariance.
    candidates(i).space = invalid_branch(9);
end
[pairs, C, ~, accept_info] = associate_passive( ...
    candidates, event.passive, t, platform, cfg, sensor, p);
for q = 1:size(pairs, 1)
    local_i = pairs(q, 1); mi = pairs(q, 2); i = eligible(local_i);
    [innovation, nis] = passive_association_diagnostic( ...
        candidates(local_i), event.passive, mi, sensor);
    companions(i).angle = update_angle_branch(companions(i).angle, ...
        event.passive.ang(:, mi), event.passive.R_ae(:, :, mi));
    companions(i).last_t = t;
    companions(i).last_update_t = t;
    companions(i).last_passive_t = t;
    companions(i).last_angle_observation_t = t;
    used(mi) = true;
    if companions(i).from_2d
        assoc_type = 'companion_passive_2d';
    else
        assoc_type = 'companion_passive_3d';
    end
    assoc = append_assoc(assoc, companions(i).id, assoc_type, ...
        mi, event.passive.ids(mi), event.passive.ang(:, mi), nan(3, 1), ...
        C(local_i, mi), innovation, nis, 1, 1);
end
end

function ids = current_external_ids(event)
ids = zeros(1, 0);
if isfield(event, 'external_3d') && isstruct(event.external_3d) && ...
        isfield(event.external_3d, 'id')
    ids = reshape(event.external_3d.id, 1, []);
    ids = ids(isfinite(ids));
end
end

function nis = space_position_nis(b, z, R)
H = zeros(3, 9); H(:, [1, 4, 7]) = eye(3);
nu = z - H * b.x; S = make_spd(H * b.P * H' + R);
nis = nu' * (S \ nu);
end

function [tr, range_updated] = update_track_active(tr, m, j, t, sensor, p)
z2 = m.rae(2:3, j);
range_updated = false;
if ~tr.angle.valid
    tr.angle = init_angle_branch(z2, m.R_ae(:, :, j), t, p);
else
    tr.angle = update_angle_branch(tr.angle, z2, m.R_ae(:, :, j));
end
if m.has_range(j)
    if ~tr.space.valid
        tr.space = init_space_branch(m.xyz(:, j), m.R_xyz(:, :, j), t, p);
        range_updated = true;
    elseif space_position_nis(tr.space, m.xyz(:, j), m.R_xyz(:, :, j)) <= p.gate_3d
        tr.space = update_space_active(tr.space, m.xyz(:, j), m.R_xyz(:, :, j));
        range_updated = true;
    end
elseif tr.space.valid
    nis = space_bearing_nis_sensor(tr.space, z2, m.R_ae(:, :, j), sensor);
    if isfinite(nis) && nis <= p.gate_2d
        tr.space = update_space_bearing_sensor(tr.space, z2, m.R_ae(:, :, j), sensor);
    end
end
if ~m.has_range(j)
    tr.last_angle_observation_t = t;
end
end

function tr = update_track_passive(tr, m, j, t, sensor, p)
z = m.ang(:, j); R = m.R_ae(:, :, j);
if ~tr.angle.valid
    tr.angle = init_angle_branch(z, R, t, p);
else
    tr.angle = update_angle_branch(tr.angle, z, R);
end
if tr.space.valid
    nis = space_bearing_nis_sensor(tr.space, z, R, sensor);
    if isfinite(nis) && nis <= p.gate_2d
        tr.space = update_space_bearing_sensor(tr.space, z, R, sensor);
    end
end
tr.last_angle_observation_t = t;
end

function nis = space_bearing_nis(b, z, R, t, platform, cfg)
sensor = platform_enu(t, platform, cfg);
nis = space_bearing_nis_sensor(b, z, R, sensor);
end

function nis = space_bearing_nis_sensor(b, z, R, sensor)
[zp, S, ~, ok] = space_bearing_stats_sensor(b.x, b.P, sensor, R);
if ~ok
    nis = inf;
else
    nu = [angle_diff(z(1), zp(1)); z(2) - zp(2)];
    nis = nu' * (S \ nu);
end
end

function b = update_angle_branch(b, z, R)
H = zeros(2, 6); H(1, 1) = 1; H(2, 4) = 1;
like = zeros(2, 1); nis_model = nan(2, 1);
for j = 1:2
    x = b.xm(:, j); P = b.Pm(:, :, j);
    nu = [angle_diff(z(1), x(1)); z(2) - x(4)];
    S = make_spd(H * P * H' + R); K = (P * H') / S;
    x = x + K * nu; x(1) = wrap_az(x(1));
    I = eye(6); P = (I - K * H) * P * (I - K * H)' + K * R * K';
    b.xm(:, j) = x; b.Pm(:, :, j) = make_spd(P);
    nis_model(j) = nu' * (S \ nu);
    like(j) = gaussian_likelihood(nu, S);
end
b.mu = normalize_probability(b.mu .* like);
[b.x, b.P] = imm_combine(b.xm, b.Pm, b.mu, true);
b.last_nis = weighted_finite(nis_model, b.mu, NaN);
b.last_nis_norm = b.last_nis / 2;
b.nis_history = append_history(b.nis_history, b.last_nis, 20);
b.nis_norm_history = append_history(b.nis_norm_history, b.last_nis_norm, 20);
end

function b = update_space_active(b, z, R)
H = zeros(3, 9); H(:, [1, 4, 7]) = eye(3);
like = zeros(2, 1); nis_model = nan(2, 1);
for j = 1:2
    x = b.xm(:, j); P = b.Pm(:, :, j);
    nu = z - H * x; S = make_spd(H * P * H' + R); K = (P * H') / S;
    x = x + K * nu; I = eye(9);
    P = (I - K * H) * P * (I - K * H)' + K * R * K';
    b.xm(:, j) = x; b.Pm(:, :, j) = make_spd(P);
    nis_model(j) = nu' * (S \ nu);
    like(j) = gaussian_likelihood(nu, S);
end
b.mu = normalize_probability(b.mu .* like);
[b.x, b.P] = imm_combine(b.xm, b.Pm, b.mu, false);
b.last_nis = weighted_finite(nis_model, b.mu, NaN);
b.last_nis_norm = b.last_nis / 3;
b.nis_history = append_history(b.nis_history, b.last_nis, 20);
b.nis_norm_history = append_history(b.nis_norm_history, b.last_nis_norm, 20);
end

function b = update_space_bearing_sensor(b, z, R, sensor)
like = zeros(2, 1); nis_model = nan(2, 1);
for j = 1:2
    x = b.xm(:, j); P = b.Pm(:, :, j);
    [zp, S, Pxz, ok] = space_bearing_stats_sensor(x, P, sensor, R);
    if ~ok, like(j) = realmin; continue; end
    nu = [angle_diff(z(1), zp(1)); z(2) - zp(2)];
    K = Pxz / S; x = x + K * nu; P = make_spd(P - K * S * K');
    b.xm(:, j) = x; b.Pm(:, :, j) = P;
    nis_model(j) = nu' * (S \ nu);
    like(j) = gaussian_likelihood(nu, S);
end
b.mu = normalize_probability(b.mu .* like);
[b.x, b.P] = imm_combine(b.xm, b.Pm, b.mu, false);
b.last_nis = weighted_finite(nis_model, b.mu, NaN);
b.last_nis_norm = b.last_nis / 2;
b.nis_history = append_history(b.nis_history, b.last_nis, 20);
b.nis_norm_history = append_history(b.nis_norm_history, b.last_nis_norm, 20);
end

function [zp, S, Pxz, ok] = space_bearing_stats(x, P, t, platform, cfg, R)
sensor = platform_enu(t, platform, cfg);
[zp, S, Pxz, ok] = space_bearing_stats_sensor(x, P, sensor, R);
end

function [zp, S, Pxz, ok] = space_bearing_stats_sensor(x, P, sensor, R)
[zp, S0, Pxz, ok] = space_bearing_moments(x, P, sensor);
if ~ok, S = nan(2); return; end
S = make_spd(S0 + R); ok = all(isfinite(S(:))) && rcond(S) > 1e-12;
end

function [zp, S0, Pxz, ok] = space_bearing_moments(x, P, sensor)
n = numel(x); [Xi, ok] = cubature_points(x, P);
if ~ok, zp = nan(2, 1); S0 = nan(2); Pxz = nan(n, 2); return; end
Zi = zeros(2, 2*n);
for q = 1:2*n
    Zi(:, q) = space_to_ae_sensor(Xi([1, 4, 7], q), sensor);
end
w = 1 / (2*n);
zp = [atan2d(sum(w * sind(Zi(1, :))), sum(w * cosd(Zi(1, :)))); ...
    sum(w * Zi(2, :))];
S0 = zeros(2); Pxz = zeros(n, 2);
for q = 1:2*n
    dz = [angle_diff(Zi(1, q), zp(1)); Zi(2, q) - zp(2)];
    dx = Xi(:, q) - x;
    S0 = S0 + w * (dz * dz'); Pxz = Pxz + w * (dx * dz');
end
ok = all(isfinite(zp)) && all(isfinite(S0(:)));
end

function [Xi, ok] = cubature_points(x, P)
n = numel(x); P = make_spd(P); ok = true;
[S, flag] = chol(P, 'lower');
if flag ~= 0, Xi = zeros(n, 0); ok = false; return; end
D = sqrt(n) * S; Xi = [x + D, x - D];
end

function ae = space_to_ae(xyz, t, platform, cfg)
sensor = platform_enu(t, platform, cfg);
ae = space_to_ae_sensor(xyz, sensor);
end

function ae = space_to_ae_sensor(xyz, sensor)
d = xyz - sensor; rxy = hypot(d(1), d(2));
ae = [wrap_az(atan2d(d(1), d(2))); atan2d(d(3), max(rxy, realmin))];
end

function [tracks, n_suppressed, suppressed_ids, target_ids, records] = ...
        suppress_tracks_explained_by_3d(tracks, event, t, p, event_index)
n_suppressed = 0;
suppressed_ids = zeros(1, 0); target_ids = zeros(1, 0);
records = struct('local_2d_id', {}, 'external_3d_id', {}, 'track', {});
if isempty(tracks) || ~isfield(event, 'confirm_cycle') || ...
        ~logical(event.confirm_cycle)
    return;
end
has_external = isfield(event, 'external_3d') && ...
    ~isempty(event.external_3d) && isfield(event.external_3d, 'ang') && ...
    ~isempty(event.external_3d.ang);
if has_external
    external_ang = event.external_3d.ang;
    n_external = size(external_ang, 2);
    external_ids = sized_row(field_or(event.external_3d, 'id', []), ...
        n_external, NaN);
    external_confirmed = logical(sized_row(field_or( ...
        event.external_3d, 'confirmed', true), n_external, true));
    if isfield(event.external_3d, 'dimension_ready') && ...
            ~isempty(event.external_3d.dimension_ready)
        external_ready = logical(sized_row( ...
            event.external_3d.dimension_ready, n_external, false));
    else
        external_ready = false(1, n_external);
    end
    external_rate = field_or(event.external_3d, 'rate', nan(2, n_external));
    if ~isequal(size(external_rate), [2, n_external])
        external_rate = nan(2, n_external);
    end
end
keep = true(1, numel(tracks));
for i = 1:numel(tracks)
    if ~isstruct(tracks(i).angle) || ~isscalar(tracks(i).angle) || ...
            ~isfield(tracks(i).angle, 'valid')
        error('run_filter_joint_2d3d:InvalidAngleBranch', ...
            ['Event %d, 2-D track %g has invalid angle branch type %s ' ...
            'with size %s.'], event_index, tracks(i).id, ...
            class(tracks(i).angle), mat2str(size(tracks(i).angle)));
    end
    updated_by_residual = abs(tracks(i).last_update_t - t) <= 1e-9;
    if ~tracks(i).angle.valid || updated_by_residual || ~has_external
        tracks(i).cross_candidate_id = NaN;
        tracks(i).cross_candidate_count = 0;
        continue;
    end
    daz = angle_diff(external_ang(1, :), tracks(i).angle.x(1));
    del = external_ang(2, :) - tracks(i).angle.x(4);
    sep = hypot(daz, del);
    rate_sep = hypot(external_rate(1, :) - tracks(i).angle.x(2), ...
        external_rate(2, :) - tracks(i).angle.x(5));
    rate_ok = ~all(isfinite(external_rate), 1) | ...
        rate_sep <= p.cross_suppress_rate_dps;
    candidates = find(isfinite(sep) & sep <= p.cross_suppress_angle_deg & rate_ok);
    if isempty(candidates)
        tracks(i).cross_candidate_id = NaN;
        tracks(i).cross_candidate_count = 0;
        continue;
    end
    [~, local_best] = min(sep(candidates));
    best = candidates(local_best);
    target_id = external_ids(best);
    if isfinite(target_id) && tracks(i).cross_candidate_id == target_id
        tracks(i).cross_candidate_count = tracks(i).cross_candidate_count + 1;
    else
        tracks(i).cross_candidate_id = target_id;
        tracks(i).cross_candidate_count = 1;
    end
    ready = external_confirmed(best) || external_ready(best);
    required_cycles = p.cross_suppress_min_cycles;
    if external_ready(best)
        required_cycles = p.cross_upgrade_cycles;
    end
    if ready && tracks(i).cross_candidate_count >= required_cycles
        keep(i) = false;
        n_suppressed = n_suppressed + 1;
        suppressed_ids(end + 1) = tracks(i).id; %#ok<AGROW>
        target_ids(end + 1) = target_id; %#ok<AGROW>
        records(end + 1) = struct('local_2d_id', tracks(i).id, ...
            'external_3d_id', target_id, 'track', tracks(i)); %#ok<AGROW>
    end
end
tracks = tracks(keep);
tracks = tracks(:);
end

function [companions, changes, rebind_info] = update_external_companions( ...
        companions, event, t, sensor, p, event_index, quality_cycle)
changes = empty_transition_array();
rebind_info = empty_rebind_info();
if ~p.dimension_switch_enabled
    companions = companions([]);
    return;
end

has_external = isfield(event, 'external_3d') && ...
    isstruct(event.external_3d) && isfield(event.external_3d, 'id');
if has_external
    ext = event.external_3d;
    ids = reshape(field_or(ext, 'id', zeros(1, 0)), 1, []);
else
    ext = struct();
    ids = zeros(1, 0);
end
for i = 1:numel(companions)
    predict_space = ~ismember(companions(i).external_id, ids);
    companions(i) = predict_companion(companions(i), t, p, predict_space);
    companions(i).external_fresh = false;
end
n = numel(ids);
confirmed = logical(sized_row(field_or(ext, 'confirmed', false(1, n)), n, false));
active_hit = logical(sized_row(field_or(ext, 'active_hit', false(1, n)), n, false));
active_opportunity = logical(field_or(ext, 'active_opportunity', false));
states = field_or(ext, 'state', zeros(9, 0));
covariances = field_or(ext, 'cov', zeros(9, 9, 0));
angles = field_or(ext, 'ang', zeros(2, n));
rates = field_or(ext, 'rate', nan(2, n));
last_active = sized_row(field_or(ext, 'last_active_t', nan(1, n)), n, NaN);
last_update = sized_row(field_or(ext, 'last_update_t', nan(1, n)), n, NaN);
nis_norm = sized_row(field_or(ext, 'nis_norm', nan(1, n)), n, NaN);
external_fresh = logical(sized_row(field_or(ext, 'fresh', false(1, n)), n, false));

if active_opportunity
    for i = 1:numel(companions)
        companions(i).active_history = shift_window(companions(i).active_history, 0);
    end
end

% Once the mature tracker drops an external ID, its companion becomes the
% sole 2-D owner of the logical track. The old spatial branch remains only
% as a bounded shadow for later cross-dimensional recovery.
alive_before = ismember([companions.external_id], ids);
for i = find([companions.confirmed] & ~alive_before)
    if starts_with(companions(i).mode, '3d')
        old_mode = companions(i).mode;
        if companions(i).angle.valid && ...
                isfinite(companions(i).last_angle_observation_t)
            companions(i).mode = '2d_shadow';
            companions(i).active_output_dim = 2;
            companions(i).switch_commit_t = t;
            reason = 'external_3d_lost';
        else
            companions(i).mode = 'hold';
            reason = 'external_3d_lost_no_angle';
        end
        companions(i).down_count = 0;
        companions(i).up_count = 0;
        changes(end + 1) = make_transition(companions(i), t, ...
            old_mode, companions(i).mode, reason); %#ok<AGROW>
    end
end

deferred = pending_external_mask(companions, ids, t, p);
if p.rebind_enabled && active_opportunity && n > 0 && ~isempty(companions)
    [companions, rebound_changes, current_rebind, active_deferred] = ...
        rebind_external_candidates(companions, ext, ids, t, sensor, p);
    deferred = deferred | active_deferred;
    if ~isempty(rebound_changes)
        changes(end + 1:end + numel(rebound_changes)) = rebound_changes;
    end
    rebind_info.n_candidates = rebind_info.n_candidates + current_rebind.n_candidates;
    rebind_info.n_committed = rebind_info.n_committed + current_rebind.n_committed;
    rebind_info.n_ambiguous = rebind_info.n_ambiguous + current_rebind.n_ambiguous;
    rebind_info.n_active_unknown = rebind_info.n_active_unknown + current_rebind.n_active_unknown;
    rebind_info.n_eligible = rebind_info.n_eligible + current_rebind.n_eligible;
    rebind_info.n_projection_invalid = rebind_info.n_projection_invalid + current_rebind.n_projection_invalid;
    rebind_info.n_reject_angle = rebind_info.n_reject_angle + current_rebind.n_reject_angle;
    rebind_info.n_reject_rate = rebind_info.n_reject_rate + current_rebind.n_reject_rate;
    rebind_info.n_reject_switch = rebind_info.n_reject_switch + current_rebind.n_reject_switch;
    rebind_info.n_assigned = rebind_info.n_assigned + current_rebind.n_assigned;
    rebind_info.n_wait_history = rebind_info.n_wait_history + current_rebind.n_wait_history;
    rebind_info.n_wait_ready = rebind_info.n_wait_ready + current_rebind.n_wait_ready;
    rebind_info.n_wait_bound = rebind_info.n_wait_bound + current_rebind.n_wait_bound;
end

[known, companion_index] = ismember(ids, [companions.external_id]);
for q = 1:n
    if ~isfinite(ids(q)) || size(states, 2) < q || size(covariances, 3) < q
        continue;
    end
    i = companion_index(q);
    if ~known(q)
        if deferred(q), continue; end
        c = track_template(p);
        c.id = ids(q);
        c.external_id = ids(q);
        c.birth_event = event_index;
        c.birth_t = t;
        c.last_t = t;
        c.last_update_t = last_update(q);
        c.last_active_t = last_active(q);
        c.confirmed = confirmed(q);
        c.active_output_dim = 3;
        if size(angles, 2) >= q && all(isfinite(angles(:, q)))
            R0 = diag([0.25^2, 0.15^2]);
            c.angle = init_angle_branch(angles(:, q), R0, t, p);
            if size(rates, 2) >= q && all(isfinite(rates(:, q)))
                c.angle.x([2, 5]) = rates(:, q);
                c.angle.xm([2, 5], :) = repmat(rates(:, q), 1, 2);
            end
        end
        c.space = external_space_branch(states(:, q), covariances(:, :, q), t, p);
        if c.confirmed, c.mode = '3d'; else, c.mode = 'tentative'; end
        companions(end + 1, 1) = c; %#ok<AGROW>
        i = numel(companions);
        known(q) = true;
        companion_index(q) = i;
    end

    nis_history = companions(i).space.nis_norm_history;
    companions(i).space = external_space_branch( ...
        states(:, q), covariances(:, :, q), t, p);
    companions(i).space.nis_norm_history = nis_history;
    if active_hit(q) && isfinite(nis_norm(q))
        companions(i).space.last_nis_norm = nis_norm(q);
        companions(i).space.nis_norm_history = append_history( ...
            companions(i).space.nis_norm_history, nis_norm(q), ...
            p.space_nis_window);
    end
    companions(i).external_fresh = external_fresh(q);
    companions(i).confirmed = companions(i).confirmed || confirmed(q);
    companions(i).last_t = t;
    companions(i).last_active_t = max_finite( ...
        companions(i).last_active_t, last_active(q));
    companions(i).last_update_t = max_finite( ...
        companions(i).last_update_t, last_update(q));
    if active_opportunity && active_hit(q)
        companions(i).active_history(end) = 1;
    end
end

update_ids = reshape(field_or(ext, 'update_track_id', zeros(1, 0)), 1, []);
update_ang = field_or(ext, 'update_ang', zeros(2, 0));
update_R = field_or(ext, 'update_R', zeros(2, 2, 0));
update_kind = sized_row(field_or(ext, 'update_kind', zeros(1, 0)), ...
    numel(update_ids), 0);
[update_found, update_index] = ismember(update_ids, [companions.external_id]);
pending_has_update = false(1, numel(companions));
for i = 1:numel(companions)
    if isfinite(companions(i).rebind_candidate_id)
        pending_has_update(i) = any(update_ids == ...
            companions(i).rebind_candidate_id);
    end
end
for q = 1:numel(update_ids)
    i = update_index(q);
    if update_found(q) && pending_has_update(i) && ...
            update_ids(q) == companions(i).external_id
        % Prefer the replacement candidate's accepted angle update during
        % the overlap; applying both stale and replacement tracks would
        % count one physical target twice.
        continue;
    end
    if ~update_found(q)
        i = find([companions.rebind_candidate_id] == update_ids(q), 1);
    end
    if isempty(i) || size(update_ang, 2) < q || size(update_R, 3) < q
        continue;
    end
    if ~companions(i).angle.valid
        companions(i).angle = init_angle_branch( ...
            update_ang(:, q), update_R(:, :, q), t, p);
    else
        companions(i).angle = update_angle_branch( ...
            companions(i).angle, update_ang(:, q), update_R(:, :, q));
    end
    companions(i).last_update_t = t;
    companions(i).last_angle_observation_t = t;
    if update_kind(q) == 2
        companions(i).last_active_t = t;
    else
        companions(i).last_passive_t = t;
    end
end

keep = true(1, numel(companions));
external_alive = ismember([companions.external_id], ids);
for i = 1:numel(companions)
    old_mode = companions(i).mode;
    old_active_output_dim = companions(i).active_output_dim;
    needs_confirmation_eval = companions(i).confirmed && strcmp(old_mode, 'tentative');
    if quality_cycle || needs_confirmation_eval
        companions(i).quality = assess_quality(companions(i), sensor, p);
        [companions(i), reason, blocked_not_ready] = manage_companion_mode( ...
            companions(i), t, p, quality_cycle);
    else
        reason = '';
        blocked_not_ready = false;
    end
    if old_active_output_dim == 3 && companions(i).active_output_dim == 2
        rebind_info.n_switch_3d_to_2d_committed = ...
            rebind_info.n_switch_3d_to_2d_committed + 1;
    elseif old_active_output_dim == 2 && companions(i).active_output_dim == 3
        rebind_info.n_switch_2d_to_3d_committed = ...
            rebind_info.n_switch_2d_to_3d_committed + 1;
    end
    if blocked_not_ready
        rebind_info.n_switch_3d_to_2d_blocked_not_ready = ...
            rebind_info.n_switch_3d_to_2d_blocked_not_ready + 1;
    end
    if ~strcmp(old_mode, companions(i).mode)
        changes(end + 1) = make_transition(companions(i), t, ...
            old_mode, companions(i).mode, reason); %#ok<AGROW>
    end
    if companions(i).confirmed
        external_timeout = p.confirmed_timeout_s;
    else
        external_timeout = p.tentative_timeout_s;
    end
    if ~external_alive(i) && ...
            max(t - companions(i).last_update_t, 0) > external_timeout
        keep(i) = false;
    elseif starts_with(companions(i).mode, '2d') && ...
            isfinite(companions(i).last_active_t) && ...
            t - companions(i).last_active_t > p.shadow_max_s
        companions(i).space = invalid_branch(9);
    end
end
companions = companions(keep);
companions = companions(:);
end

function [companions, changes, info, deferred] = rebind_external_candidates( ...
        companions, ext, ids, t, sensor, p)
changes = empty_transition_array();
info = empty_rebind_info();
deferred = false(1, numel(ids));
known_external = ismember(ids, [companions.external_id]);
unknown = find(~known_external & isfinite(ids));
if isempty(unknown), return; end

external_active_hit = logical(sized_row(field_or(ext, 'active_hit', ...
    false(1, numel(ids))), numel(ids), false));

% Rebind evidence belongs to the candidate track, not to unrelated active
% detections in the same global event stream. Advance the window only when
% that candidate itself has a current hit and expire evidence by time.
for i = 1:numel(companions)
    candidate_id = companions(i).rebind_candidate_id;
    if ~isfinite(candidate_id), continue; end
    if ~isfinite(companions(i).rebind_last_hit_t) || ...
            t - companions(i).rebind_last_hit_t > p.rebind_max_gap_s
        companions(i).rebind_candidate_id = NaN;
        companions(i).rebind_last_hit_t = NaN;
        companions(i).rebind_candidate_since = NaN;
        companions(i).rebind_history(:) = 0;
        continue;
    end
    q = find(ids == candidate_id, 1);
    if ~isempty(q) && external_active_hit(q)
        companions(i).rebind_history = shift_window( ...
            companions(i).rebind_history, 0);
        if ~any(companions(i).rebind_history)
            companions(i).rebind_candidate_id = NaN;
            companions(i).rebind_last_hit_t = NaN;
            companions(i).rebind_candidate_since = NaN;
        end
    end
end

% Restore pending ownership before any geometric early return. A candidate
% remains hidden while at least one evidence hit is still inside its 2/3
% window, even if the current event fails the strict continuation gate.
for q = unknown
    pending = arrayfun(@(x) x.rebind_candidate_id == ids(q) && ...
        any(x.rebind_history), companions);
    deferred(q) = any(pending);
end

% A persisted/coasting external state is not new identity evidence.
unknown = unknown(external_active_hit(unknown));
if isempty(unknown), return; end
info.n_active_unknown = info.n_active_unknown + numel(unknown);

bound = ismember([companions.external_id], ids);
bound_active_hit = false(1, numel(companions));
for i = find(bound)
    q = find(ids == companions(i).external_id, 1);
    bound_active_hit(i) = ~isempty(q) && external_active_hit(q);
end
within_shadow = reshape(arrayfun(@(x) ~isfinite(x.last_active_t) || ...
    max(t - x.last_active_t, 0) <= p.shadow_max_s, companions), 1, []);
angle_eligible = reshape(arrayfun(@(x) x.angle.valid && ...
    (starts_with(x.mode, '2d') || isfinite(x.rebind_candidate_id)), ...
    companions), 1, []);
pending_rebind = reshape(arrayfun(@(x) ...
    isfinite(x.rebind_candidate_id) && any(x.rebind_history), ...
    companions), 1, []);
% A replacement external track can be born before the mature tracker times
% out its stale predecessor. Let the existing 2-D owner collect rebind
% evidence during that overlap, but commit only after the old external ID
% has left the mature tracker.
eligible = find([companions.confirmed] & ...
    (~bound_active_hit | pending_rebind) & ...
    within_shadow & angle_eligible);
if isempty(eligible), return; end
info.n_eligible = info.n_eligible + numel(eligible);

states = field_or(ext, 'state', zeros(9, 0));
covariances = field_or(ext, 'cov', zeros(9, 9, 0));
angles = field_or(ext, 'ang', zeros(2, numel(ids)));
rates = field_or(ext, 'rate', nan(2, numel(ids)));
ready = logical(sized_row(field_or(ext, 'dimension_ready', ...
    false(1, numel(ids))), numel(ids), false));
C = inf(numel(eligible), numel(unknown));

% The candidate projection depends only on the external state. Cache it
% once per event instead of repeating a 9-D cubature projection for every
% companion/candidate edge.
candidate_z = nan(2, numel(unknown));
candidate_S = nan(2, 2, numel(unknown));
candidate_valid = false(1, numel(unknown));
for b = 1:numel(unknown)
    q = unknown(b);
    if size(states, 2) < q || size(covariances, 3) < q
        continue;
    end
    space = external_space_branch(states(:, q), covariances(:, :, q), t, p);
    [candidate_z(:, b), candidate_S(:, :, b), ~, candidate_valid(b)] = ...
        space_bearing_stats_sensor(space.x, space.P, sensor, zeros(2));
end

for a = 1:numel(eligible)
    tr = companions(eligible(a));
    for b = 1:numel(unknown)
        q = unknown(b);
        if size(states, 2) < q || size(covariances, 3) < q || ...
                size(angles, 2) < q || ~candidate_valid(b)
            info.n_projection_invalid = info.n_projection_invalid + 1;
            continue;
        end
        daz = angle_diff(angles(1, q), tr.angle.x(1));
        del = angles(2, q) - tr.angle.x(4);
        continuing_candidate = tr.rebind_candidate_id == ids(q) && ...
            any(tr.rebind_history);
        if ~continuing_candidate && ...
                hypot(daz, del) > p.cross_suppress_angle_deg
            info.n_reject_angle = info.n_reject_angle + 1;
            continue;
        end
        if size(rates, 2) >= q && all(isfinite(rates(:, q)))
            rate_sep = hypot(rates(1, q) - tr.angle.x(2), ...
                rates(2, q) - tr.angle.x(5));
            if rate_sep > p.cross_suppress_rate_dps
                info.n_reject_rate = info.n_reject_rate + 1;
                continue;
            end
        end
        [nis, ok] = angle_projection_switch_nis(tr.angle, ...
            candidate_z(:, b), candidate_S(:, :, b), p);
        if ok && nis <= p.switch_gate
            C(a, b) = nis;
        else
            info.n_reject_switch = info.n_reject_switch + 1;
        end
    end
end

pairs = solve_global_assignment(C, p.unmatched_cost);
for r = 1:size(pairs, 1)
    a = pairs(r, 1); b = pairs(r, 2);
    if ~isfinite(C(a, b)), continue; end
    if assignment_is_ambiguous(C, a, b, p.rebind_ambiguity_nis)
        info.n_ambiguous = info.n_ambiguous + 1;
        continue;
    end
    i = eligible(a); q = unknown(b); candidate_id = ids(q);
    info.n_candidates = info.n_candidates + 1;
    info.n_assigned = info.n_assigned + 1;
    if companions(i).rebind_candidate_id ~= candidate_id
        companions(i).rebind_candidate_id = candidate_id;
        companions(i).rebind_candidate_since = t;
        companions(i).rebind_history(:) = 0;
    end
    companions(i).rebind_history(end) = 1;
    companions(i).rebind_last_hit_t = t;
    deferred(q) = true;
    if sum(companions(i).rebind_history) < p.upgrade3_M
        info.n_wait_history = info.n_wait_history + 1;
        continue;
    end
    if ~ready(q)
        info.n_wait_ready = info.n_wait_ready + 1;
        continue;
    end
    if bound(i)
        info.n_wait_bound = info.n_wait_bound + 1;
        continue;
    end

    old_external = companions(i).external_id;
    old_mode = companions(i).mode;
    companions(i).previous_external_id = old_external;
    companions(i).external_id = candidate_id;
    companions(i).external_fresh = false;
    companions(i).switch_commit_t = t;
    ncopy = min(numel(companions(i).active_history), ...
        numel(companions(i).rebind_history));
    companions(i).active_history(end - ncopy + 1:end) = max( ...
        companions(i).active_history(end - ncopy + 1:end), ...
        companions(i).rebind_history(end - ncopy + 1:end));
    companions(i).rebind_candidate_id = NaN;
    companions(i).rebind_last_hit_t = NaN;
    companions(i).rebind_candidate_since = NaN;
    companions(i).rebind_history(:) = 0;
    changes(end + 1) = make_transition(companions(i), t, ...
        old_mode, old_mode, sprintf('external_3d_rebound_%g_to_%g', ...
        old_external, candidate_id)); %#ok<AGROW>
    info.n_committed = info.n_committed + 1;
end

% A candidate accepted for the first time in this event also remains
% internal for the rest of its 2/3 evidence window.
for q = unknown
    pending = arrayfun(@(x) x.rebind_candidate_id == ids(q) && ...
        any(x.rebind_history), companions);
    deferred(q) = deferred(q) || any(pending);
end
end

function deferred = pending_external_mask(companions, ids, t, p)
deferred = false(1, numel(ids));
for i = 1:numel(companions)
    candidate_id = companions(i).rebind_candidate_id;
    valid = isfinite(candidate_id) && any(companions(i).rebind_history) && ...
        isfinite(companions(i).rebind_last_hit_t) && ...
        t - companions(i).rebind_last_hit_t <= p.rebind_max_gap_s;
    if ~valid, continue; end
    q = find(ids == candidate_id, 1);
    if ~isempty(q), deferred(q) = true; end
end
end

function info = empty_rebind_info()
info = struct('n_candidates', 0, 'n_committed', 0, 'n_ambiguous', 0, ...
    'n_active_unknown', 0, 'n_eligible', 0, 'n_projection_invalid', 0, ...
    'n_reject_angle', 0, 'n_reject_rate', 0, 'n_reject_switch', 0, ...
    'n_assigned', 0, 'n_wait_history', 0, 'n_wait_ready', 0, ...
    'n_wait_bound', 0, 'n_switch_3d_to_2d_committed', 0, ...
    'n_switch_2d_to_3d_committed', 0, ...
    'n_switch_3d_to_2d_blocked_not_ready', 0);
end

function [nis, ok] = angle_projection_switch_nis(angle, z3, S3, p)
nis = inf; ok = false;
if ~angle.valid || any(~isfinite(z3)) || any(~isfinite(S3(:))), return; end
z2 = angle.x([1, 4]); P2 = angle.P([1, 4], [1, 4]);
nu = [angle_diff(z2(1), z3(1)); z2(2) - z3(2)];
S = make_spd(p.switch_cov_inflate * (P2 + S3));
if any(~isfinite(S(:))) || rcond(S) <= 1e-12, return; end
nis = nu' * (S \ nu);
ok = isfinite(nis);
end

function tf = assignment_is_ambiguous(C, row, col, margin)
tf = false;
if margin <= 0, return; end
best = C(row, col);
row_other = C(row, :); row_other(col) = inf;
col_other = C(:, col); col_other(row) = inf;
second = min([row_other(isfinite(row_other)), ...
    reshape(col_other(isfinite(col_other)), 1, [])]);
if ~isempty(second), tf = min(second) - best < margin; end
end

function tr = predict_companion(tr, t, p, predict_space)
remaining = max(t - tr.last_t, 0);
while remaining > 0
    dt = min(remaining, p.max_dt);
    if tr.angle.valid
        tr.angle = predict_angle_branch(tr.angle, dt, p);
    end
    if predict_space && tr.space.valid
        tr.space = predict_space_branch(tr.space, dt, p);
    end
    remaining = max(remaining - dt, 0);
end
if tr.angle.valid, tr.angle.last_t = t; end
if tr.space.valid, tr.space.last_t = t; end
tr.last_t = t;
end

function [companions, changes, info] = adopt_cross_dimension_merges( ...
        companions, records, t, sensor, p)
changes = empty_transition_array();
info = struct('n_adopted', 0, 'n_tentative', 0, 'n_late', 0, ...
    'n_already_bound', 0, 'n_missing_companion', 0);
if isempty(records), return; end

birth_event = arrayfun(@(r) r.track.birth_event, records);
local_id = [records.local_2d_id];
[~, order] = sortrows([birth_event(:), local_id(:)], [1, 2]);
for q = reshape(order, 1, [])
    i = find([companions.external_id] == records(q).external_3d_id, 1);
    if isempty(i)
        info.n_missing_companion = info.n_missing_companion + 1;
        continue;
    end
    if ~records(q).track.confirmed
        info.n_tentative = info.n_tentative + 1;
        continue;
    end
    if companions(i).from_2d
        info.n_already_bound = info.n_already_bound + 1;
        continue;
    end
    if records(q).track.birth_event >= companions(i).birth_event
        info.n_late = info.n_late + 1;
        continue;
    end
    old_mode = companions(i).mode;
    companions(i).id = records(q).local_2d_id;
    companions(i).from_2d = true;
    companions(i).confirmed = true;
    companions(i).active_output_dim = 2;
    companions(i).birth_event = records(q).track.birth_event;
    companions(i).birth_t = records(q).track.birth_t;
    companions(i).angle = records(q).track.angle;
    companions(i).last_angle_observation_t = max_finite( ...
        companions(i).last_angle_observation_t, ...
        records(q).track.last_angle_observation_t);
    companions(i).last_update_t = max_finite( ...
        companions(i).last_update_t, records(q).track.last_update_t);
    companions(i).quality = assess_quality(companions(i), sensor, p);
    if companions(i).quality.recover3d && ...
            companions(i).quality.switch_consistent && space_upgrade_ready(companions(i), p)
        companions(i).mode = '3d';
        companions(i).active_output_dim = 3;
        companions(i).switch_commit_t = t;
        reason = '2d_to_3d_2of3_confirmed';
    else
        companions(i).mode = '2d_to_3d';
        companions(i).switch_candidate_since = t;
        reason = '2d_attached_3d_candidate';
    end
    changes(end + 1) = make_transition(companions(i), t, ...
        old_mode, companions(i).mode, reason); %#ok<AGROW>
    info.n_adopted = info.n_adopted + 1;
end
end

function [tr, reason, blocked_not_ready] = manage_companion_mode( ...
        tr, t, p, quality_cycle)
reason = '';
blocked_not_ready = false;
if ~tr.confirmed
    tr.mode = 'tentative';
    return;
end
if strcmp(tr.mode, 'tentative')
    if tr.from_2d
        tr.mode = '2d_to_3d';
        tr.active_output_dim = 2;
        tr.switch_candidate_since = t;
        reason = 'confirmed_2d_with_3d_candidate';
    else
        tr.mode = '3d';
        tr.active_output_dim = 3;
        reason = 'external_3d_confirmed';
    end
end

if ~quality_cycle
    return;
end

if starts_with(tr.mode, '3d')
    if tr.quality.down3d
        if tr.down_count == 0 && ~isfinite(tr.switch_candidate_since)
            tr.switch_candidate_since = t;
        end
        tr.down_count = tr.down_count + 1;
    else
        tr.down_count = 0;
        tr.switch_candidate_since = NaN;
    end
    if tr.down_count >= p.down_consecutive
        if tr.quality.valid2d && tr.quality.switch_consistent
            tr.mode = '2d_shadow';
            tr.active_output_dim = 2;
            tr.switch_commit_t = t;
            tr.switch_candidate_since = NaN;
            reason = '3d_quality_degraded';
            tr.active_history(:) = 0;
            tr.down_count = 0;
            tr.up_count = 0;
        else
            % Transaction is not committed until the mature 2-D branch can
            % take over.  Keep the 3-D owner formal and saturate evidence so
            % readiness can commit on the next quality opportunity.
            tr.mode = '3d_warn';
            tr.active_output_dim = 3;
            tr.down_count = p.down_consecutive;
            blocked_not_ready = true;
            reason = '3d_degradation_blocked_2d_not_ready';
        end
    elseif tr.quality.warn3d
        tr.mode = '3d_warn';
    else
        tr.mode = '3d';
    end
    return;
end

if starts_with(tr.mode, '2d') || strcmp(tr.mode, 'hold')
    if ~tr.quality.valid2d
        tr.mode = 'hold';
        tr.up_count = 0;
        if isempty(reason), reason = 'angle_quality_degraded'; end
        return;
    end
    % While a replacement external ID is collecting evidence, the stale
    % predecessor must remain the 2-D logical owner. Otherwise passive
    % updates can make the old spatial branch look fresh and prematurely
    % cancel the pending handoff.
    no_pending_rebind = ~isfinite(tr.rebind_candidate_id);
    ready = no_pending_rebind && tr.external_fresh && tr.space.valid && ...
        space_upgrade_ready(tr, p) && ...
        tr.quality.recover3d && tr.quality.switch_consistent;
    if ready
        if tr.up_count == 0 && ~isfinite(tr.switch_candidate_since)
            tr.switch_candidate_since = t;
        end
        tr.up_count = tr.up_count + 1;
    else
        tr.up_count = 0;
    end
    if tr.up_count >= p.up_consecutive
        tr.mode = '3d';
        tr.active_output_dim = 3;
        tr.switch_commit_t = t;
        tr.switch_candidate_since = NaN;
        tr.up_count = 0;
        tr.down_count = 0;
        reason = '2d_to_3d_2of3_confirmed';
    elseif strcmp(tr.mode, 'hold')
        tr.mode = '2d_shadow';
        reason = 'angle_quality_recovered';
    elseif tr.space.valid && isfinite(tr.last_active_t) && ...
            t - tr.last_active_t <= p.shadow_max_s
        tr.mode = '2d_to_3d';
        if ~isfinite(tr.switch_candidate_since)
            tr.switch_candidate_since = t;
        end
    else
        tr.mode = '2d_shadow';
        if ~isfinite(tr.rebind_candidate_id)
            tr.switch_candidate_since = NaN;
        end
    end
end
end

function b = external_space_branch(x, P, t, p)
b = invalid_branch(9);
if numel(x) ~= 9 || ~isequal(size(P), [9, 9]) || ...
        any(~isfinite(x(:))) || any(~isfinite(P(:)))
    return;
end
b.valid = true;
b.x = x(:);
b.P = make_spd(P);
b.xm = repmat(b.x, 1, 2);
b.Pm = repmat(b.P, 1, 1, 2);
b.mu = p.imm_mu0;
b.last_t = t;
end

function snapshots = companion_snapshots(companions, t, p)
template = struct('external_3d_id', NaN, 'local_2d_id', NaN, ...
    'logical_id', NaN, 'previous_external_3d_id', NaN, ...
    'pending_external_3d_id', NaN, ...
    'branch_3d_id', NaN, 'branch_2d_id', NaN, ...
    'active_output_dim', 0, 'switch_state', 'hold_recovery', ...
    'pending_target_branch_id', NaN, 'switch_candidate_since', NaN, ...
    'switch_commit_t', NaN, ...
    'from_2d', false, 'confirmed', false, 'mode', '', 'output_dim', 0, ...
    'external_fresh', false, ...
    'birth_event', 0, 'birth_t', NaN, 'last_update_t', NaN, ...
    'angle_state', nan(6, 1), 'angle_cov', nan(6), ...
    'quality', quality_template());
snapshots = repmat(template, numel(companions), 1);
for i = 1:numel(companions)
    tr = companions(i);
    s = template;
    s.external_3d_id = tr.external_id;
    s.local_2d_id = tr.id;
    s.logical_id = tr.id;
    s.previous_external_3d_id = tr.previous_external_id;
    s.pending_external_3d_id = tr.rebind_candidate_id;
    s.branch_3d_id = tr.external_id;
    s.branch_2d_id = tr.id;
    s.active_output_dim = tr.active_output_dim;
    [s.switch_state, s.pending_target_branch_id, ...
        s.switch_candidate_since] = companion_switch_state(tr);
    s.switch_commit_t = tr.switch_commit_t;
    s.from_2d = tr.from_2d;
    s.external_fresh = tr.external_fresh;
    s.confirmed = tr.confirmed;
    s.mode = tr.mode;
    s.birth_event = tr.birth_event;
    s.birth_t = tr.birth_t;
    s.last_update_t = tr.last_update_t;
    s.quality = tr.quality;
    fresh = ~isfinite(p.output_max_silence_s) || ...
        max(t - tr.last_update_t, 0) <= p.output_max_silence_s;
    if tr.confirmed && tr.active_output_dim == 3 && ...
            tr.external_fresh && tr.space.valid
        s.output_dim = 3;
    elseif tr.confirmed && tr.active_output_dim == 2 && fresh && tr.angle.valid
        s.output_dim = 2;
        s.angle_state = tr.angle.x;
        s.angle_cov = tr.angle.P;
    end
    snapshots(i) = s;
end
end

function [state, pending_id, since] = companion_switch_state(tr)
pending_id = NaN;
since = tr.switch_candidate_since;
if isfinite(tr.rebind_candidate_id)
    state = 'pending_2d_to_3d';
    pending_id = tr.rebind_candidate_id;
    since = tr.rebind_candidate_since;
elseif tr.active_output_dim == 3 && tr.down_count > 0
    state = 'pending_3d_to_2d';
    pending_id = tr.id;
elseif tr.active_output_dim == 2 && ...
        (starts_with(tr.mode, '2d_to_3d') || tr.up_count > 0)
    state = 'pending_2d_to_3d';
    pending_id = tr.external_id;
elseif tr.active_output_dim == 3
    state = 'stable_3d';
elseif tr.active_output_dim == 2
    state = 'stable_2d';
else
    state = 'hold_recovery';
end
end

function out = empty_transition_array()
out = struct('id', {}, 't_sec', {}, 'from', {}, 'to', {}, 'reason', {});
end

function item = make_transition(tr, t, from, to, reason)
item = struct('id', tr.id, 't_sec', t, 'from', from, 'to', to, 'reason', reason);
end

function [tracks, n_merged] = merge_tentative_duplicates(tracks, t, sensor, p)
n_merged = 0;
if numel(tracks) < 2, return; end
tracks = normalize_confirmed_flags(tracks);
keep = true(1, numel(tracks));
for i = 1:numel(tracks)
    if ~keep(i), continue; end
    for j = i + 1:numel(tracks)
        if ~keep(j) || (tracks(i).confirmed && tracks(j).confirmed), continue; end
        if ~tracks(i).angle.valid || ~tracks(j).angle.valid, continue; end
        updated_i = abs(tracks(i).last_update_t - t) <= 1e-9;
        updated_j = abs(tracks(j).last_update_t - t) <= 1e-9;
        if updated_i && updated_j
            % Two unique measurements updated two tracks in this event.
            continue;
        end
        da = hypot(angle_diff(tracks(i).angle.x(1), tracks(j).angle.x(1)), ...
            tracks(i).angle.x(4) - tracks(j).angle.x(4));
        if da > p.merge_angle_deg, continue; end
        dr = hypot(tracks(i).angle.x(2) - tracks(j).angle.x(2), ...
            tracks(i).angle.x(5) - tracks(j).angle.x(5));
        if dr > p.merge_rate_dps, continue; end
        ii = [1, 2, 4, 5];
        dx = tracks(i).angle.x(ii) - tracks(j).angle.x(ii);
        dx(1) = angle_diff(tracks(i).angle.x(1), tracks(j).angle.x(1));
        S = make_spd(tracks(i).angle.P(ii, ii) + tracks(j).angle.P(ii, ii));
        if dx' * (S \ dx) > p.merge_nis, continue; end
        if ~merge_space_consistent(tracks(i), tracks(j), sensor, p)
            continue;
        end
        if tracks(j).confirmed && ~tracks(i).confirmed
            tracks(j) = absorb_duplicate(tracks(j), tracks(i));
            keep(i) = false; break;
        elseif ~tracks(i).confirmed && ~tracks(j).confirmed && ...
                sum(tracks(j).hit_history) > sum(tracks(i).hit_history)
            tracks(j) = absorb_duplicate(tracks(j), tracks(i));
            keep(i) = false; break;
        else
            tracks(i) = absorb_duplicate(tracks(i), tracks(j));
            keep(j) = false;
        end
        n_merged = n_merged + 1;
    end
end
tracks = tracks(keep);
tracks = tracks(:);
end

function tracks = normalize_confirmed_flags(tracks)
% A logical track has one confirmation state. Normalize malformed legacy or
% high-density intermediate values before scalar short-circuit expressions.
for i = 1:numel(tracks)
    value = tracks(i).confirmed;
    if isempty(value)
        tracks(i).confirmed = false;
    elseif ~isscalar(value)
        tracks(i).confirmed = any(logical(value(:)));
    else
        tracks(i).confirmed = logical(value);
    end
end
end

function tf = merge_space_consistent(a, b, sensor, p)
tf = true;
if a.space.valid && b.space.valid
    ii = [1, 4, 7];
    dx = a.space.x(ii) - b.space.x(ii);
    S = make_spd(a.space.P(ii, ii) + b.space.P(ii, ii));
    tf = all(isfinite(dx)) && dx' * (S \ dx) <= p.gate_3d;
    return;
end
if a.space.valid && starts_with(a.mode, '3d')
    tf = angle_matches_space(b.angle, a.space, sensor, p);
elseif b.space.valid && starts_with(b.mode, '3d')
    tf = angle_matches_space(a.angle, b.space, sensor, p);
end
end

function tf = angle_matches_space(angle, space, sensor, p)
tf = false;
if ~angle.valid, return; end
[z3, S3, ~, ok] = space_bearing_stats_sensor(space.x, space.P, sensor, zeros(2));
if ~ok, return; end
z2 = angle.x([1, 4]); P2 = angle.P([1, 4], [1, 4]);
nu = [angle_diff(z2(1), z3(1)); z2(2) - z3(2)];
S = make_spd(p.switch_cov_inflate * (P2 + S3));
tf = nu' * (S \ nu) <= p.switch_gate;
end

function keep = absorb_duplicate(keep, drop)
keep.hit_history = max(keep.hit_history, drop.hit_history);
keep.active_history = max(keep.active_history, drop.active_history);
keep.passive_confirm_streak = max(keep.passive_confirm_streak, ...
    drop.passive_confirm_streak);
if drop.cross_candidate_count > keep.cross_candidate_count
    keep.cross_candidate_id = drop.cross_candidate_id;
    keep.cross_candidate_count = drop.cross_candidate_count;
end
keep.age = max(keep.age, drop.age); keep.miss = min(keep.miss, drop.miss);
keep.last_update_t = max_finite(keep.last_update_t, drop.last_update_t);
keep.last_active_t = max_finite(keep.last_active_t, drop.last_active_t);
keep.last_passive_t = max_finite(keep.last_passive_t, drop.last_passive_t);
keep.last_angle_observation_t = max_finite( ...
    keep.last_angle_observation_t, drop.last_angle_observation_t);
if ~keep.angle.valid && drop.angle.valid
    keep.angle = drop.angle;
elseif keep.angle.valid && drop.angle.valid && trace(drop.angle.P) < trace(keep.angle.P)
    keep.angle = drop.angle;
end
if ~keep.space.valid && drop.space.valid
    keep.space = drop.space;
elseif keep.space.valid && drop.space.valid && trace(drop.space.P) < trace(keep.space.P)
    keep.space = drop.space;
end
if (~isfinite(keep.truth_id) || drop.truth_conf > keep.truth_conf) && isfinite(drop.truth_id)
    keep.truth_id = drop.truth_id; keep.truth_conf = drop.truth_conf;
end
end

function [tr, reason] = manage_mode(tr, t, sensor, p, quality_cycle)
reason = '';
if ~starts_with(tr.mode, '3d') && tr.space.valid && isfinite(tr.last_active_t) && ...
        t - tr.last_active_t > p.shadow_max_s
    tr.space = invalid_branch(9);
    tr.active_history(:) = 0;
end
ready_to_confirm = ~tr.confirmed && sum(tr.hit_history) >= p.confirm_M;
if quality_cycle || ready_to_confirm
    tr.quality = assess_quality(tr, sensor, p);
end
if ready_to_confirm
    tr.confirmed = true;
    if space_birth_ready(tr, p) && tr.quality.recover3d
        tr.mode = '3d'; reason = 'logical_confirm_3d';
    elseif tr.quality.valid2d
        tr.mode = mode_2d_name(tr, t, p); reason = 'logical_confirm_2d';
    else
        tr.mode = 'hold'; reason = 'logical_confirm_hold';
    end
    return;
end
if ~tr.confirmed || ~quality_cycle, return; end

if strcmp(tr.mode, 'hold')
    can_recover3d = tr.quality.valid3d && tr.quality.recover3d && ...
        space_upgrade_ready(tr, p) && tr.quality.switch_consistent;
    if can_recover3d
        tr.mode = '3d'; reason = 'space_quality_recovered';
    elseif tr.quality.valid2d
        tr.mode = mode_2d_name(tr, t, p); reason = 'angle_quality_recovered';
    end
    return;
end

if starts_with(tr.mode, '2d')
    if ~tr.quality.valid2d
        tr.mode = 'hold'; tr.up_count = 0; tr.down_count = 0;
        reason = 'angle_quality_degraded';
        return;
    end
    ready = space_upgrade_ready(tr, p) && tr.quality.recover3d && ...
        tr.quality.switch_consistent;
    if ready, tr.up_count = tr.up_count + 1; else, tr.up_count = 0; end
    if tr.up_count >= p.up_consecutive
        tr.mode = '3d'; tr.up_count = 0; tr.down_count = 0;
        reason = '2d_to_3d_quality_confirmed';
    elseif tr.space.valid
        if isfinite(tr.last_active_t) && t - tr.last_active_t <= p.shadow_max_s
            tr.mode = '2d_to_3d';
        else
            tr.mode = '2d_shadow';
        end
    else
        tr.mode = '2d';
    end
    return;
end

if starts_with(tr.mode, '3d')
    if tr.quality.down3d
        tr.down_count = tr.down_count + 1;
    else
        tr.down_count = 0;
    end
    if tr.down_count >= p.down_consecutive
        if tr.quality.valid2d
            tr.mode = '2d_shadow'; reason = '3d_quality_degraded';
        else
            tr.mode = 'hold'; reason = 'all_output_quality_degraded';
        end
        tr.active_history(:) = 0;
        tr.down_count = 0; tr.up_count = 0;
    elseif tr.quality.warn3d
        tr.mode = '3d_warn';
    else
        tr.mode = '3d';
    end
end
end

function q = assess_quality(tr, sensor, p)
q = quality_template();
q.observed2d = isfinite(tr.last_angle_observation_t);
if tr.angle.valid && q.observed2d
    s = sqrt(max([tr.angle.P(1, 1), tr.angle.P(4, 4)], 0));
    q.angle95_deg = 1.96 * max(s);
    q.valid2d = isfinite(q.angle95_deg) && q.angle95_deg <= p.angle95_max_deg;
end
if tr.space.valid
    Pp = make_spd(tr.space.P([1, 4, 7], [1, 4, 7]));
    rel = tr.space.x([1, 4, 7]) - sensor; q.range_m = norm(rel);
    if q.range_m > 0, u = rel / q.range_m; else, u = [1; 0; 0]; end
    q.radial_sigma_m = sqrt(max(u' * Pp * u, 0));
    q.radial95_m = 1.96 * q.radial_sigma_m;
    q.position95_m = sqrt(7.8147279 * max(real(eig(Pp))));
    q.relative_range_sigma = q.radial_sigma_m / max(q.range_m, 1);
    nh = tr.space.nis_norm_history;
    nh = nh(max(1, end - p.space_nis_window + 1):end);
    nh = nh(isfinite(nh));
    if ~isempty(nh), q.nis_norm = median(nh); end
    nis_warn = isfinite(q.nis_norm) && q.nis_norm > p.space_nis_warn;
    nis_down = isfinite(q.nis_norm) && q.nis_norm > p.space_nis_down;
    nis_recover = ~isfinite(q.nis_norm) || q.nis_norm <= p.space_nis_recover;
    q.warn3d = q.position95_m > p.pos95_warn_m || ...
        q.radial95_m > p.radial95_warn_m || nis_warn;
    q.down3d = q.position95_m > p.pos95_down_m || q.radial95_m > p.radial95_down_m || ...
        q.relative_range_sigma > p.relative_range_down || nis_down;
    q.valid3d = ~q.down3d;
    q.recover3d = q.position95_m <= p.pos95_recover_m && ...
        q.radial95_m <= p.radial95_recover_m && nis_recover;
end
if tr.angle.valid && tr.space.valid
    z2 = tr.angle.x([1, 4]); P2 = tr.angle.P([1, 4], [1, 4]);
    [z3, S3, ~, ok] = space_bearing_stats_sensor( ...
        tr.space.x, tr.space.P, sensor, zeros(2));
    if ok
        nu = [angle_diff(z2(1), z3(1)); z2(2) - z3(2)];
        S = make_spd(p.switch_cov_inflate * (P2 + S3));
        q.switch_nis = nu' * (S \ nu);
        q.switch_consistent = q.switch_nis <= p.switch_gate;
    end
end
end

function tf = space_birth_ready(tr, p)
w = tr.active_history(max(1, end - p.birth3_N + 1):end);
tf = sum(w) >= p.birth3_M;
end

function tf = space_upgrade_ready(tr, p)
w = tr.active_history(max(1, end - p.upgrade3_N + 1):end);
tf = sum(w) >= p.upgrade3_M;
end

function name = mode_2d_name(tr, t, p)
if tr.space.valid && isfinite(tr.last_active_t)
    if t - tr.last_active_t <= p.shadow_max_s, name = '2d_to_3d'; else, name = '2d_shadow'; end
else
    name = '2d';
end
end

function q = quality_template()
q = struct('valid2d', false, 'valid3d', false, 'warn3d', false, ...
    'down3d', true, 'recover3d', false, 'switch_consistent', false, ...
    'observed2d', false, ...
    'angle95_deg', inf, 'position95_m', inf, 'radial_sigma_m', inf, ...
    'radial95_m', inf, 'relative_range_sigma', inf, 'range_m', NaN, ...
    'switch_nis', inf, 'nis_norm', NaN);
end

function [est, outputs] = store_event(est, tracks, assoc, k, t, sensor, p)
snap = repmat(snapshot_template(), numel(tracks), 1);
outputs = repmat(output_template(), 0, 1);
for i = 1:numel(tracks)
    snap(i) = make_snapshot(tracks(i));
    if tracks(i).confirmed && output_is_fresh(tracks(i), t, p)
        candidate = make_output(tracks(i), t, sensor);
        if candidate.output_dim > 0
            outputs(end + 1, 1) = candidate; %#ok<AGROW>
        end
    end
end

est.logical_tracks{k} = snap;
est.output{k} = outputs;
est.assoc{k} = assoc;
est.filter_times(k) = t;
est.N_total(k) = numel(outputs);

idx2 = find([outputs.output_dim] == 2);
idx3 = find([outputs.output_dim] == 3);
est.N2(k) = numel(idx2); est.N(k) = numel(idx3);
if ~isempty(idx2)
    est.X2{k} = cat(2, outputs(idx2).angle_state);
    est.P2{k} = cat(3, outputs(idx2).angle_cov);
    est.L2{k} = [[outputs(idx2).birth_event].', [outputs(idx2).id].'];
else
    est.X2{k} = zeros(6, 0); est.P2{k} = zeros(6, 6, 0); est.L2{k} = zeros(0, 2);
end
if ~isempty(idx3)
    est.X{k} = cat(2, outputs(idx3).state3d);
    est.P{k} = cat(3, outputs(idx3).cov3d);
    est.L{k} = [[outputs(idx3).birth_event].', [outputs(idx3).id].'];
else
    est.X{k} = zeros(9, 0); est.P{k} = zeros(9, 9, 0); est.L{k} = zeros(0, 2);
end

est.mode_counts.n2d(k) = numel(idx2);
est.mode_counts.n3d(k) = numel(idx3);
est.mode_counts.nhold(k) = nnz([tracks.confirmed] & strcmp({tracks.mode}, 'hold'));

% Legacy all-track 3-D snapshot for existing diagnostics.
valid3 = find(arrayfun(@(x) x.space.valid, tracks));
if isempty(valid3)
    est.tracks{k} = struct('m', zeros(9, 0), 'P', zeros(9, 9, 0), ...
        'L', zeros(2, 0), 'conf', zeros(1, 0), 'mode', {cell(1, 0)});
else
    m3 = zeros(9, numel(valid3)); P3 = zeros(9, 9, numel(valid3));
    for q = 1:numel(valid3)
        m3(:, q) = tracks(valid3(q)).space.x;
        P3(:, :, q) = tracks(valid3(q)).space.P;
    end
    est.tracks{k} = struct('m', m3, 'P', P3, ...
        'L', [[tracks(valid3).birth_event]; [tracks(valid3).id]], ...
        'conf', [tracks(valid3).confirmed], 'mode', {{tracks(valid3).mode}});
end
end

function tf = output_is_fresh(tr, t, p)
tf = isfinite(tr.last_update_t) && ...
    max(t - tr.last_update_t, 0) <= p.output_max_silence_s;
end

function s = make_snapshot(tr)
s = snapshot_template();
s.id = tr.id; s.birth_event = tr.birth_event; s.confirmed = tr.confirmed;
s.mode = tr.mode; s.last_update_t = tr.last_update_t; s.truth_id = tr.truth_id;
s.quality = tr.quality; s.hit_history = tr.hit_history;
if tr.angle.valid, s.angle_state = tr.angle.x; s.angle_cov = tr.angle.P; end
if tr.space.valid, s.state3d = tr.space.x; s.cov3d = tr.space.P; end
end

function s = snapshot_template()
s = struct('id', 0, 'birth_event', 0, 'confirmed', false, 'mode', '', ...
    'last_update_t', NaN, 'truth_id', NaN, 'quality', quality_template(), ...
    'hit_history', zeros(1, 0), ...
    'angle_state', nan(6, 1), ...
    'angle_cov', nan(6), 'state3d', nan(9, 1), 'cov3d', nan(9));
end

function o = make_output(tr, t, sensor)
o = output_template();
o.id = tr.id; o.birth_event = tr.birth_event; o.t_sec = t; o.mode = tr.mode;
o.confirmed = tr.confirmed; o.truth_id = tr.truth_id; o.quality = tr.quality;
o.last_update_t = tr.last_update_t;
if tr.angle.valid
    o.angle_state = tr.angle.x; o.angle_cov = tr.angle.P;
end
if starts_with(tr.mode, '3d') && tr.space.valid
    o.output_dim = 3; o.state3d = tr.space.x; o.cov3d = tr.space.P;
    o.position_enu = tr.space.x([1, 4, 7]);
    o.velocity_enu = tr.space.x([2, 5, 8]);
    o.acceleration_enu = tr.space.x([3, 6, 9]);
    ae = space_to_ae_sensor(o.position_enu, sensor);
    o.az_deg = ae(1); o.el_deg = ae(2); o.range_m = tr.quality.range_m;
elseif starts_with(tr.mode, '2d') && tr.angle.valid
    o.output_dim = 2; o.az_deg = tr.angle.x(1); o.el_deg = tr.angle.x(4);
end
end

function o = output_template()
o = struct('id', 0, 'birth_event', 0, 't_sec', NaN, 'mode', '', ...
    'output_dim', 0, 'confirmed', false, 'truth_id', NaN, ...
    'last_update_t', NaN, ...
    'az_deg', NaN, 'el_deg', NaN, 'range_m', NaN, ...
    'angle_state', nan(6, 1), 'angle_cov', nan(6), ...
    'state3d', nan(9, 1), 'cov3d', nan(9), ...
    'position_enu', nan(3, 1), 'velocity_enu', nan(3, 1), ...
    'acceleration_enu', nan(3, 1), 'quality', quality_template());
end

function est = init_estimate(K, events)
est = struct();
est.X = cell(K, 1); est.P = cell(K, 1); est.L = cell(K, 1); est.N = zeros(K, 1);
est.X2 = cell(K, 1); est.P2 = cell(K, 1); est.L2 = cell(K, 1); est.N2 = zeros(K, 1);
est.N_total = zeros(K, 1); est.logical_tracks = cell(K, 1); est.output = cell(K, 1);
est.tracks = cell(K, 1); est.assoc = cell(K, 1); est.companions = cell(K, 1);
est.measurement_disposition = cell(K, 1);
if K > 0 && isstruct(events) && isfield(events, 't_sec')
    est.filter_times = reshape([events.t_sec], [], 1);
else
    est.filter_times = zeros(K, 1);
end
est.event_meta = events;
est.mode_counts = struct('n2d', zeros(K, 1), 'n3d', zeros(K, 1), 'nhold', zeros(K, 1));
est.timing = struct('total', 0);
end

function t = primary_event_time(e)
if e.has_active && ~isempty(e.active.t_sec)
    t = median(e.active.t_sec);
elseif e.has_passive && ~isempty(e.passive.t_sec)
    t = median(e.passive.t_sec);
else
    t = e.t_sec;
end
end

function t = passive_event_time(e, fallback)
if e.has_passive && ~isempty(e.passive.t_sec)
    % Active keeps processing priority inside the synchronization window;
    % never propagate the filter backward if the passive stamp is earlier.
    t = max(median(e.passive.t_sec), fallback);
else
    t = fallback;
end
end

function a = assoc_template()
a = struct('id', zeros(1, 0), 'type', {cell(1, 0)}, 'meas_index', zeros(1, 0), ...
    'tid', zeros(1, 0), 'ang', zeros(2, 0), 'xyz', zeros(3, 0), ...
    'cost', zeros(1, 0), 'measurement_dim', zeros(1, 0), ...
    'filter_dim', zeros(1, 0), 'input_dim', zeros(1, 0), ...
    'range_updated', false(1, 0), 'innovation', zeros(3, 0), ...
    'nis', zeros(1, 0), 'group_size', zeros(1, 0), ...
    'innovation_kind', zeros(1, 0));
end

function a = append_assoc(a, id, type, mi, tid, ang, xyz, cost, ...
        innovation, nis, group_size, innovation_kind, filter_dim, ...
        input_dim, range_updated)
if nargin < 9 || isempty(innovation), innovation = nan(3, 1); end
if nargin < 10 || isempty(nis), nis = NaN; end
if nargin < 11 || isempty(group_size), group_size = 1; end
if nargin < 12 || isempty(innovation_kind), innovation_kind = 0; end
if nargin < 13 || isempty(filter_dim), filter_dim = 2; end
if nargin < 14 || isempty(input_dim), input_dim = filter_dim; end
if nargin < 15 || isempty(range_updated), range_updated = false; end
a.id(end + 1) = id; a.type{end + 1} = type; a.meas_index(end + 1) = mi;
a.tid(end + 1) = tid; a.ang(:, end + 1) = ang; a.xyz(:, end + 1) = xyz;
a.cost(end + 1) = cost;
if strncmp(type, 'active', 6)
    a.measurement_dim(end + 1) = input_dim;
else
    a.measurement_dim(end + 1) = 2;
end
a.filter_dim(end + 1) = filter_dim;
a.input_dim(end + 1) = input_dim;
a.range_updated(end + 1) = logical(range_updated);
a.innovation(:, end + 1) = sized_innovation(innovation);
a.nis(end + 1) = nis;
a.group_size(end + 1) = group_size;
a.innovation_kind(end + 1) = innovation_kind;
end

function a = concat_assoc(a, b)
if isempty(b.id), return; end
a.id = [a.id, b.id];
a.type = [a.type, b.type];
a.meas_index = [a.meas_index, b.meas_index];
a.tid = [a.tid, b.tid];
a.ang = [a.ang, b.ang];
a.xyz = [a.xyz, b.xyz];
a.cost = [a.cost, b.cost];
a.measurement_dim = [a.measurement_dim, b.measurement_dim];
a.filter_dim = [a.filter_dim, b.filter_dim];
a.input_dim = [a.input_dim, b.input_dim];
a.range_updated = [a.range_updated, b.range_updated];
a.innovation = [a.innovation, b.innovation];
a.nis = [a.nis, b.nis];
a.group_size = [a.group_size, b.group_size];
a.innovation_kind = [a.innovation_kind, b.innovation_kind];
end

function a = finalize_assoc_dimensions(a, tracks, companions)
if isempty(a.id), return; end
live_ids = [reshape([tracks.id], 1, []), reshape([companions.id], 1, [])];
removed = ~ismember(a.id, live_ids);
a.filter_dim(removed) = 0;
a.range_updated(removed) = false;
end

function d = event_measurement_disposition(a, e, event_index)
active_suppressed = true(1, e.active.n_meas);
passive_suppressed = true(1, e.passive.n_meas);
for q = 1:numel(a.id)
    mi = a.meas_index(q);
    if ~isfinite(mi) || mi ~= round(mi), continue; end
    if strncmp(a.type{q}, 'active', 6) && mi >= 1 && mi <= e.active.n_meas
        active_suppressed(mi) = false;
    elseif strncmp(a.type{q}, 'passive', 7) && mi >= 1 && mi <= e.passive.n_meas
        passive_suppressed(mi) = false;
    end
end
d = struct( ...
    'active', joint_measurement_disposition(e.active.n_meas, a, ...
        'active', active_suppressed, event_index), ...
    'passive', joint_measurement_disposition(e.passive.n_meas, a, ...
        'passive', passive_suppressed, event_index));
end

function v = sized_innovation(x)
v = nan(3, 1);
if isempty(x), return; end
n = min(3, numel(x)); v(1:n) = x(1:n);
end

function tr = vote_truth_id(tr, tid)
if ~isfinite(tid), return; end
if ~isfinite(tr.truth_id) || tr.truth_conf <= 0
    tr.truth_id = tid; tr.truth_conf = 1;
elseif tr.truth_id == tid
    tr.truth_conf = min(tr.truth_conf + 1, 100);
else
    tr.truth_conf = tr.truth_conf - 1;
    if tr.truth_conf <= 0, tr.truth_id = tid; tr.truth_conf = 1; end
end
end

function tf = explained_measurement(C, j, gate)
tf = ~isempty(C) && j <= size(C, 2) && any(isfinite(C(:, j)) & C(:, j) <= gate);
end

function w = shift_window(w, value)
if isempty(w), w = value; else, w = [w(2:end), value]; end
end

function v = append_history(v, x, n)
v = [v, x]; if numel(v) > n, v = v(end - n + 1:end); end
end

function p = normalize_probability(p)
p(~isfinite(p) | p < 0) = 0;
if sum(p) <= realmin, p = ones(size(p)) / numel(p); else, p = p / sum(p); end
end

function y = weighted_finite(x, w, fallback)
good = isfinite(x) & isfinite(w) & w >= 0;
if ~any(good) || sum(w(good)) <= 0
    y = fallback;
else
    wg = w(good) / sum(w(good));
    y = sum(wg .* x(good));
end
end

function y = max_finite(a, b)
if isfinite(a) && isfinite(b)
    y = max(a, b);
elseif isfinite(a)
    y = a;
else
    y = b;
end
end

function L = gaussian_likelihood(nu, S)
d = numel(nu); S = make_spd(S);
logL = -0.5 * (nu' * (S \ nu) + log(max(det(S), realmin)) + d * log(2*pi));
L = max(exp(max(logL, log(realmin))), realmin);
end

function sensor = platform_enu(t, platform, cfg)
if isempty(platform) || ~isfield(platform, 'interp_lat')
    sensor = zeros(3, 1); return;
end
lat = platform.interp_lat(t); lon = platform.interp_lon(t); alt = platform.interp_alt(t);
if ischar(cfg.local_origin) && strcmp(cfg.local_origin, 'first_platform')
    lat0 = platform.lat_deg(1); lon0 = platform.lon_deg(1); alt0 = platform.alt_m(1);
else
    lat0 = cfg.local_origin(1); lon0 = cfg.local_origin(2); alt0 = cfg.local_origin(3);
end
ecef = llh_to_ecef(lat, lon, alt); ecef0 = llh_to_ecef(lat0, lon0, alt0);
sensor = ecef_to_enu_rot(lat0, lon0) * (ecef - ecef0);
end

function ecef = llh_to_ecef(lat_deg, lon_deg, alt_m)
a = 6378137.0; f = 1 / 298.257223563; e2 = f * (2 - f);
lat = deg2rad(lat_deg); lon = deg2rad(lon_deg);
N = a / sqrt(1 - e2 * sin(lat)^2);
ecef = [(N + alt_m) * cos(lat) * cos(lon); ...
    (N + alt_m) * cos(lat) * sin(lon); ...
    (N * (1 - e2) + alt_m) * sin(lat)];
end

function R = ecef_to_enu_rot(lat_deg, lon_deg)
lat = deg2rad(lat_deg); lon = deg2rad(lon_deg);
R = [-sin(lon), cos(lon), 0; ...
    -sin(lat)*cos(lon), -sin(lat)*sin(lon), cos(lat); ...
    cos(lat)*cos(lon), cos(lat)*sin(lon), sin(lat)];
end

function A = make_spd(A)
A = 0.5 * (A + A');
if any(~isfinite(A(:))), return; end
[V, D] = eig(A); d = max(real(diag(D)), 1e-10);
A = real(V * diag(d) * V'); A = 0.5 * (A + A');
end

function d = angle_diff(a, b)
d = mod(a - b + 180, 360) - 180;
end

function a = wrap_az(a)
a = mod(a + 180, 360) - 180;
end

function tf = starts_with(s, prefix)
tf = ischar(s) && numel(s) >= numel(prefix) && strcmp(s(1:numel(prefix)), prefix);
end

function v = get_cfg(cfg, name, fallback)
if isfield(cfg, name) && ~isempty(cfg.(name)), v = cfg.(name); else, v = fallback; end
end

function v = field_or(s, name, fallback)
if isstruct(s) && isfield(s, name) && ~isempty(s.(name))
    v = s.(name);
else
    v = fallback;
end
end

function row = sized_row(value, n, fallback)
if isempty(value), row = fallback * ones(1, n); else, row = value(:).'; end
if isscalar(row) && n > 1, row = repmat(row, 1, n); end
if numel(row) < n, row = [row, fallback * ones(1, n - numel(row))]; end
row = row(1:n);
end
