function metrics = evaluate_joint_tracking_metrics(est, events, cfg, platform)
%EVALUATE_JOINT_TRACKING_METRICS Corrected joint 2-D/3-D tracking evaluation.
%
% This compatibility wrapper preserves the original evaluator under the
% +metriclegacy package, then repairs the "assigned to ever-confirmed track"
% association statistic using the UNION of all confirmation evidence.
%
% Why this is needed:
%   The previous evaluator treated logical_confirm_* transition records as
%   exclusive whenever at least one such record existed. In the legacy
%   mature-3D adapter, transition_log mainly describes the 2-D/dimension
%   manager and therefore does not enumerate every confirmed mature 3-D
%   logical ID. As a result, a large number of valid 3-D/passive-to-3-D
%   associations were incorrectly classified as "never confirmed".
%
% Filter/association behavior is not modified by this file.

if nargin < 1 || isempty(est), est = struct(); end
if nargin < 2 || isempty(events), events = repmat(empty_event_local(), 0, 1); end
if nargin < 3 || isempty(cfg), cfg = struct(); end
if nargin < 4, platform = []; end

% Suppress the legacy report so the user sees only the corrected values.
cfg_legacy = cfg;
cfg_legacy.metrics_max_print = 0;
metrics = metriclegacy.evaluate_joint_tracking_metrics( ...
    est, events, cfg_legacy, platform);

[confirmed_ids, evidence] = collect_confirmed_ids_union(est);
assoc = collect_association_id_scopes(est, events);
metrics = repair_association_scope(metrics, 'two_d', assoc.two_d_ids, confirmed_ids);
metrics = repair_association_scope(metrics, 'three_d', assoc.three_d_ids, confirmed_ids);
metrics = repair_association_scope(metrics, 'overall', assoc.overall_ids, confirmed_ids);
metrics.association = metrics.overall.association;
metrics.evaluation_version = max(field_or_local(metrics, 'evaluation_version', 2), 3);
metrics.association_confirmation = struct( ...
    'basis', 'union_of_confirmation_evidence', ...
    'confirmed_track_ids', confirmed_ids, ...
    'n_confirmed_track_ids', numel(confirmed_ids), ...
    'evidence', evidence);

if get_cfg_local(cfg, 'metrics_max_print', 12) > 0
    print_joint_report_corrected(metrics);
end
end

function metrics = repair_association_scope(metrics, name, assigned_ids, confirmed_ids)
if ~isfield(metrics, name) || ~isstruct(metrics.(name)) || ...
        ~isfield(metrics.(name), 'association')
    return;
end
m = metrics.(name).association;
assigned_ids = assigned_ids(isfinite(assigned_ids));
n_assigned = numel(assigned_ids);
n_confirmed = nnz(ismember(assigned_ids, confirmed_ids));

% Preserve the original physical-measurement denominator. Recompute the
% assigned count from the ledger as an internal consistency check.
if ~isfield(m, 'n_measurements'), m.n_measurements = 0; end
m.n_assigned = n_assigned;
m.n_assigned_confirmed = n_confirmed;
m.rate_all_tracks = safe_ratio_local(n_assigned, m.n_measurements);
m.rate_confirmed_tracks = safe_ratio_local(n_confirmed, m.n_measurements);
m.rate_confirmed_given_assigned = safe_ratio_local(n_confirmed, n_assigned);
m.confirmed_track_id_basis = 'union_of_confirmation_evidence';
m.n_confirmed_track_ids = numel(confirmed_ids);
metrics.(name).association = m;
end

function [ids, evidence] = collect_confirmed_ids_union(est)
ids_transition = zeros(1, 0);
ids_output = zeros(1, 0);
ids_snapshot = zeros(1, 0);
ids_labels = zeros(1, 0);

% 1) Explicit logical confirmation transitions.
if isstruct(est) && isfield(est, 'transition_log') && ~isempty(est.transition_log)
    log = est.transition_log;
    if isstruct(log) && isfield(log, 'id') && isfield(log, 'reason')
        for q = 1:numel(log)
            reason = log(q).reason;
            if ischar(reason) && strncmp(reason, 'logical_confirm_', 16) && ...
                    isfinite(log(q).id)
                ids_transition(end + 1) = log(q).id; %#ok<AGROW>
            end
        end
    end
end

% 2) Formal outputs. In the joint adapter, est.output is the formal logical
% output stream; shadow candidates live in est.shadow_output and are not read.
if isstruct(est) && isfield(est, 'output') && ~isempty(est.output)
    for k = 1:numel(est.output)
        out = est.output{k};
        if isempty(out) || ~isstruct(out) || ~isfield(out, 'id'), continue; end
        for q = 1:numel(out)
            keep = true;
            if isfield(out, 'formal') && ~isempty(out(q).formal)
                keep = keep && logical(out(q).formal);
            end
            if isfield(out, 'confirmed') && ~isempty(out(q).confirmed)
                keep = keep && logical(out(q).confirmed);
            end
            if keep && isfinite(out(q).id)
                ids_output(end + 1) = out(q).id; %#ok<AGROW>
            end
        end
    end
end

% 3) Diagnostic logical-track snapshots, when retained.
if isstruct(est) && isfield(est, 'logical_tracks') && ~isempty(est.logical_tracks)
    for k = 1:numel(est.logical_tracks)
        tr = est.logical_tracks{k};
        if isempty(tr) || ~isstruct(tr) || ~isfield(tr, 'id'), continue; end
        for q = 1:numel(tr)
            keep = true;
            if isfield(tr, 'confirmed') && ~isempty(tr(q).confirmed)
                keep = logical(tr(q).confirmed);
            end
            if keep && isfinite(tr(q).id)
                ids_snapshot(end + 1) = tr(q).id; %#ok<AGROW>
            end
        end
    end
end

% 4) Legacy confirmed-output label matrices. L is 3-D, L2 is 2-D.
for field_name = {'L', 'L2'}
    name = field_name{1};
    if ~isstruct(est) || ~isfield(est, name) || isempty(est.(name)), continue; end
    cells = est.(name);
    for k = 1:numel(cells)
        L = cells{k};
        if isempty(L) || size(L, 2) < 2, continue; end
        values = reshape(L(:, 2), 1, []);
        ids_labels = [ids_labels, values(isfinite(values))]; %#ok<AGROW>
    end
end

ids = unique([ids_transition, ids_output, ids_snapshot, ids_labels]);
ids = ids(isfinite(ids));
evidence = struct( ...
    'n_transition_ids', numel(unique(ids_transition)), ...
    'n_formal_output_ids', numel(unique(ids_output)), ...
    'n_snapshot_ids', numel(unique(ids_snapshot)), ...
    'n_label_matrix_ids', numel(unique(ids_labels)));
end

function assoc = collect_association_id_scopes(est, events)
ids2 = zeros(1, 0);
ids3 = zeros(1, 0);
ids_all = zeros(1, 0);
K = 0;
if isstruct(est) && isfield(est, 'assoc'), K = numel(est.assoc); end
for k = 1:K
    a = est.assoc{k};
    if isempty(a) || ~isstruct(a) || ~isfield(a, 'id') || isempty(a.id), continue; end
    ids = reshape(a.id, 1, []);
    n = numel(ids);
    dims = zeros(1, n);
    if isfield(a, 'measurement_dim') && ~isempty(a.measurement_dim)
        m = min(n, numel(a.measurement_dim));
        supplied = reshape(a.measurement_dim(1:m), 1, []);
        valid = isfinite(supplied) & ismember(supplied, [2, 3]);
        idx = find(valid);
        dims(idx) = supplied(idx);
    end
    for q = find(dims == 0)
        dims(q) = infer_measurement_dim(a, q, events, k);
    end
    valid_id = isfinite(ids);
    ids_all = [ids_all, ids(valid_id)]; %#ok<AGROW>
    ids2 = [ids2, ids(valid_id & dims == 2)]; %#ok<AGROW>
    ids3 = [ids3, ids(valid_id & dims == 3)]; %#ok<AGROW>
end
assoc = struct('two_d_ids', ids2, 'three_d_ids', ids3, 'overall_ids', ids_all);
end

function dim = infer_measurement_dim(a, q, events, k)
dim = 0;
type = indexed_text_local(a, 'type', q, '');
if strncmp(type, 'passive', 7)
    dim = 2;
    return;
end
if ~strncmp(type, 'active', 6), return; end
mi = round(indexed_field_local(a, 'meas_index', q, 0));
if k >= 1 && k <= numel(events) && isfield(events(k), 'active') && ...
        isfield(events(k).active, 'has_range') && mi >= 1 && ...
        mi <= numel(events(k).active.has_range) && events(k).active.has_range(mi)
    dim = 3;
else
    dim = 2;
end
end

function print_joint_report_corrected(metrics)
fprintf('\n========== 二维/三维分维度定量评价（确认关联统计修正版 v3） ==========\n');
R = metrics.real_truth;
fprintf('\n[真实真值精度]\n');
if isstruct(R) && isfield(R, 'status') && strcmp(R.status, 'ok')
    fprintf('  真值文件: %s\n', field_or_local(R, 'source_file', ''));
    fprintf(['  真值目标=%d, 正式输出=%d, 身份映射=%d(%.2f%%), ' ...
        '时间匹配=%d(%.2f%%), 目标覆盖=%d(%.2f%%)\n'], ...
        field_or_local(R, 'n_truth_targets', 0), ...
        field_or_local(R, 'n_formal_outputs', 0), ...
        field_or_local(R, 'n_identity_mapped_outputs', 0), ...
        100 * field_or_local(R, 'mapping_rate', NaN), ...
        field_or_local(R, 'n_time_matched_outputs', 0), ...
        100 * field_or_local(R, 'time_match_rate', NaN), ...
        field_or_local(R, 'n_output_truth_targets', 0), ...
        100 * field_or_local(R, 'target_coverage_rate', NaN));
    if isfield(R, 'two_d') && R.two_d.angle.n > 0
        fprintf('  二维真实角度RMSE: az=%.4fdeg, el=%.4fdeg, LOS=%.4fdeg (%d点)\n', ...
            R.two_d.angle.rmse_az_deg, R.two_d.angle.rmse_el_deg, ...
            R.two_d.angle.rmse_los_deg, R.two_d.angle.n);
    end
    if isfield(R, 'three_d') && R.three_d.position.n > 0
        fprintf('  三维真实位置RMSE: E/N/U=[%.2f %.2f %.2f]m, 3D=%.2fm (%d点)\n', ...
            R.three_d.position.rmse_e_m, R.three_d.position.rmse_n_m, ...
            R.three_d.position.rmse_u_m, R.three_d.position.rmse_3d_m, ...
            R.three_d.position.n);
    end
else
    fprintf('  不可用: %s (%s)\n', field_or_local(R, 'status', 'unknown'), ...
        field_or_local(R, 'reason', ''));
end

fprintf('\n[量测伪真值一致性]\n');
T = metrics.truth_targets;
fprintf('量测标签参考数: 原始编号=%d, 拆分后实例=%d, 被拆分原始编号=%d\n', ...
    field_or_local(T, 'raw_id_count', 0), field_or_local(T, 'instance_count', 0), ...
    field_or_local(T, 'n_split_raw_ids', 0));
if field_or_local(T, 'enabled', false) && field_or_local(T, 'n_angle_only_raw_ids', 0) > 0
    fprintf('  其中纯角度编号=%d（无三维位置，各按1个目标实例计数）\n', ...
        T.n_angle_only_raw_ids);
end
print_scope_corrected('二维被动AE航迹/输出', metrics.two_d, 2);
print_scope_corrected('三维主动空间', metrics.three_d, 3);
print_scope_corrected('全部逻辑航迹', metrics.overall, 0);

if isfield(metrics, 'measurement_flow')
    F = metrics.measurement_flow;
    fprintf('\n[物理量测维度 -> 关联航迹维度]\n');
    fprintf('  二维量测: 关联后未保留=%d, 二维=%d, 三维=%d\n', F.counts(1, :));
    fprintf('  三维量测: 关联后未保留=%d, 二维=%d, 三维=%d\n', F.counts(2, :));
    fprintf('  主动距离关联: %d, 完成空间更新=%d, 未更新=%d\n', ...
        F.n_range_associated, F.n_range_updated, F.n_range_associated_without_update);
end
if isfield(metrics, 'measurement_accounting')
    A = metrics.measurement_accounting;
    fprintf('\n[物理量测去向]\n');
    print_accounting_corrected('主动全部', A.active);
    print_accounting_corrected('主动距离', A.active_range);
    print_accounting_corrected('被动角度', A.passive);
end
E = metrics.association_confirmation.evidence;
fprintf('\n[确认ID统计依据]\n');
fprintf(['  曾确认逻辑ID并集=%d条（transition=%d, 正式输出=%d, ' ...
    '确认快照=%d, L/L2=%d）\n'], ...
    metrics.association_confirmation.n_confirmed_track_ids, ...
    E.n_transition_ids, E.n_formal_output_ids, E.n_snapshot_ids, E.n_label_matrix_ids);
end

function print_scope_corrected(label, s, dim)
fprintf('\n[%s]\n', label);
fprintf('  输出: %d点, %d条逻辑航迹\n', s.output.n_outputs, s.output.n_unique_tracks);
if dim == 2
    fprintf('  被动输入分流: 进入2D=%d, 进入3D=%d, 未送入支路=%d; 二维带标签参考=%d\n', ...
        s.reference.n_passive_to_2d, s.reference.n_passive_to_3d, ...
        s.reference.n_passive_not_routed, s.reference.n_labeled_measurements);
elseif dim == 3
    fprintf('  三维带标签RAE参考=%d（被动AE不参与三维身份得分计数）\n', ...
        s.reference.n_labeled_measurements);
end
fprintf(['  量测关联率(全部/归属曾确认逻辑航迹): %.2f%% / %.2f%%  ' ...
    '(%d/%d, %d/%d)\n'], ...
    100 * s.association.rate_all_tracks, ...
    100 * s.association.rate_confirmed_tracks, ...
    s.association.n_assigned, s.association.n_measurements, ...
    s.association.n_assigned_confirmed, s.association.n_measurements);
fprintf('  已关联量测中归属曾确认逻辑航迹: %.2f%% (%d/%d)\n', ...
    100 * s.association.rate_confirmed_given_assigned, ...
    s.association.n_assigned_confirmed, s.association.n_assigned);
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

function print_accounting_corrected(label, row)
fprintf(['  %s: 输入=%d, 量测利用(关联或新生)=%d(%.2f%%), ' ...
    '去2D=%d, 去3D=%d, 关联/新生后未保留=%d, ' ...
    '明确抑制=%d, 未解释=%d, 去向字段差=%d\n'], ...
    label, row.n_input, row.n_associated_or_born, 100 * row.utilization_rate, ...
    row.n_to_2d, row.n_to_3d, row.n_associated_not_retained, ...
    row.n_explicitly_suppressed, row.unaccounted, row.destination_gap);
end

function value = indexed_field_local(s, name, i, fallback)
value = fallback;
if isstruct(s) && isfield(s, name) && i >= 1 && i <= numel(s.(name))
    value = s.(name)(i);
end
end

function value = indexed_text_local(s, name, i, fallback)
value = fallback;
if ~isstruct(s) || ~isfield(s, name), return; end
x = s.(name);
if iscell(x) && i >= 1 && i <= numel(x) && ischar(x{i})
    value = x{i};
elseif ischar(x) && i == 1
    value = x;
end
end

function v = safe_ratio_local(a, b)
if b > 0, v = a / b; else, v = NaN; end
end

function v = get_cfg_local(cfg, name, fallback)
if isstruct(cfg) && isfield(cfg, name) && ~isempty(cfg.(name))
    v = cfg.(name);
else
    v = fallback;
end
end

function v = field_or_local(s, name, fallback)
if isstruct(s) && isfield(s, name) && ~isempty(s.(name))
    v = s.(name);
else
    v = fallback;
end
end

function e = empty_event_local()
e = struct('t_sec', NaN, 'active', struct('n_meas', 0, ...
    'has_range', false(1, 0)), 'passive', struct('n_meas', 0));
end
