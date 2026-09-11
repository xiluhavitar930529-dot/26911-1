function results = run_passive_gate_sweep(read_end_percent, candidates)
%RUN_PASSIVE_GATE_SWEEP Compare 3-D passive-association gates on one input slice.

if nargin < 1 || isempty(read_end_percent)
    read_end_percent = 20;
end
if nargin < 2 || isempty(candidates)
    candidates = [ ...
        16.000, inf; ...
        16.000, 1.00; ...
        16.000, 0.50; ...
        16.000, 0.30; ...
         9.2103, 0.50; ...
         5.991, 0.50; ...
         5.991, 0.30; ...
         5.991, 0.10; ...
         5.991, 0.03];
end
if size(candidates, 2) ~= 2
    error('candidates must be [passive_bearing_gate, fast_gate_deg].');
end

cfg = config_fusion();
cfg.read_start_percent = 0;
cfg.read_end_percent = read_end_percent;
cfg.read_percent = read_end_percent;
cfg.do_plot = false;
cfg.metrics_enabled = true;
cfg.metrics_progress_enabled = false;
cfg.joint_streaming_quiet = true;
cfg = validate_config_fusion(cfg);
cfg.joint_shard_dedup_enabled = false;

fprintf('Preparing shared %.1f%% input slice...\n', read_end_percent);
[active_list, passive_list] = load_radar_measurement_files(cfg);
platform_cfg = cfg;
platform_cfg.read_start_percent = 0;
platform_cfg.read_end_percent = 100;
platform_cfg.read_percent = 100;
platform_cfg.max_rows = inf;
platform = load_platform_txt(resolve_data_path(cfg.data_dir, cfg.platform_file), platform_cfg);
frames = cohere_measurements(active_list, passive_list, platform, cfg);
[fused_xyz, fused_R, ~, passive_bearing, fused_ids, frame_times, event_meta] = ...
    build_async_measurement_events(frames, cfg);

n = size(candidates, 1);
template = struct('bearing_gate', NaN, 'fast_gate_deg', NaN, ...
    'passive_input', 0, ...
    'runtime_s', NaN, 'passive_to_3d', 0, 'passive_to_2d', 0, ...
    'passive_unassigned', 0, 'n2_tracks', 0, 'n3_tracks', 0, ...
    'acc2_output', NaN, 'acc2_truth', NaN, ...
    'acc3_output', NaN, 'acc3_truth', NaN, ...
    'acc_all_output', NaN, 'acc_all_truth', NaN, ...
    'coverage2', NaN, 'coverage3', NaN, 'coverage_all', NaN);
results = repmat(template, n, 1);

for q = 1:n
    run_cfg = cfg;
    run_cfg.passive_bearing_gate = candidates(q, 1);
    run_cfg.passive_bearing_nis_gate = candidates(q, 1);
    run_cfg.joint_passive_fast_gate_deg = candidates(q, 2);
    run_cfg = validate_config_fusion(run_cfg);
    fprintf('\n[%d/%d] bearing_gate=%.4g, fast_gate=%.4g deg\n', ...
        q, n, candidates(q, 1), candidates(q, 2));
    t_run = tic;
    [est, events] = run_filter_joint_legacy_backbone( ...
        fused_xyz, fused_R, frame_times, passive_bearing, platform, ...
        fused_ids, event_meta, run_cfg, []);
    metrics = evaluate_joint_tracking_metrics(est, events, run_cfg);

    r = template;
    r.bearing_gate = candidates(q, 1);
    r.fast_gate_deg = candidates(q, 2);
    r.runtime_s = toc(t_run);
    pm = metrics.passive_measurements;
    r.passive_input = pm.n_measurements;
    r.passive_to_3d = pm.n_assigned_3d;
    r.passive_to_2d = pm.n_assigned_2d;
    r.passive_unassigned = pm.n_unassigned;
    r.n2_tracks = metrics.two_d.output.n_unique_tracks;
    r.n3_tracks = metrics.three_d.output.n_unique_tracks;
    r.acc2_output = metrics.two_d.track_accuracy.accuracy_vs_output;
    r.acc2_truth = metrics.two_d.track_accuracy.accuracy_vs_truth;
    r.acc3_output = metrics.three_d.track_accuracy.accuracy_vs_output;
    r.acc3_truth = metrics.three_d.track_accuracy.accuracy_vs_truth;
    r.acc_all_output = metrics.overall.track_accuracy.accuracy_vs_output;
    r.acc_all_truth = metrics.overall.track_accuracy.accuracy_vs_truth;
    r.coverage2 = metrics.two_d.track_accuracy.mean_coverage_purity;
    r.coverage3 = metrics.three_d.track_accuracy.mean_coverage_purity;
    r.coverage_all = metrics.overall.track_accuracy.mean_coverage_purity;
    results(q) = r;
    fprintf(['  time=%.1fs, passive 3D/2D/unassigned=%d/%d/%d, tracks 2D/3D=%d/%d, ' ...
        'accuracy truth 2D/3D/all=%.1f/%.1f/%.1f%%\n'], ...
        r.runtime_s, r.passive_to_3d, r.passive_to_2d, r.passive_unassigned, ...
        r.n2_tracks, r.n3_tracks, 100*r.acc2_truth, 100*r.acc3_truth, 100*r.acc_all_truth);
    clear est events metrics
end

results = struct2table(results);
disp(results);
end
