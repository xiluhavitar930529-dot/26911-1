function info = export_track_diagnostics(est, events, cfg, track_ids, opts)
%EXPORT_TRACK_DIAGNOSTICS Export selected logical-track metrics and history.
% The report only prints diagnostics that are actually stored by the filter.
% In joint_history_level='output', association cost/NIS are compacted away;
% rerun with 'diagnostic' or 'full' when detailed association diagnostics are
% required. Birth measurements do not have update residuals/NIS.

if nargin < 1 || isempty(est), est = struct(); end
if nargin < 2 || isempty(events), events = repmat(empty_event(), 0, 1); end
if nargin < 3 || isempty(cfg), cfg = struct(); end
if nargin < 4, track_ids = []; end
if nargin < 5 || isempty(opts), opts = struct(); end

output_dir = get_opt(opts, 'output_dir', fullfile(pwd, 'track_reports'));
print_console = logical(get_opt(opts, 'print_console', true));
metrics = get_opt(opts, 'metrics', struct());

available_ids = output_track_ids(est);
if isempty(track_ids)
    selected_ids = available_ids;
else
    selected_ids = intersect(reshape(track_ids, 1, []), available_ids, 'stable');
end
missing_ids = setdiff(reshape(track_ids, 1, []), available_ids);
if print_console && ~isempty(missing_ids)
    fprintf(2, '[单轨报告] 以下航迹没有正式输出，已跳过: %s\n', mat2str(missing_ids));
end

history_level = field_text(est, 'history_level', 'unknown');
detailed_assoc_available = ~strcmp(history_level, 'output');
info = struct('selected_ids', selected_ids, 'missing_ids', missing_ids, ...
    'files', {cell(1, numel(selected_ids))}, 'output_dir', output_dir, ...
    'n_reports', 0, 'history_level', history_level, ...
    'detailed_association_diagnostics_available', detailed_assoc_available);
if isempty(selected_ids)
    if print_console, fprintf('[单轨报告] 没有可导出的正式逻辑航迹。\n'); end
    return;
end

metric_ids = zeros(1, 0);
if isstruct(metrics) && isfield(metrics, 'evaluation_version') && ...
        metrics.evaluation_version == 2 && isfield(metrics, 'track_details') && ...
        ~isempty(metrics.track_details) && isfield(metrics.track_details, 'track_id')
    metric_ids = [metrics.track_details.track_id];
end
if ~all(ismember(selected_ids, metric_ids))
    cfg_eval = cfg;
    cfg_eval.metrics_max_print = 0;
    cfg_eval.metrics_progress_enabled = false;
    metrics = evaluate_joint_tracking_metrics(est, events, cfg_eval);
end

if ~exist(output_dir, 'dir')
    [ok, msg] = mkdir(output_dir);
    if ~ok, error('export_track_diagnostics:CreateDir', '无法创建报告目录: %s', msg); end
end
if print_console && ~detailed_assoc_available
    fprintf(2, ['[单轨报告] 当前 history_level=output；滤波器已压缩 association cost/NIS。' ...
        '如需详细关联诊断，请以 cfg.joint_history_level=''diagnostic'' 或 ''full'' 重新运行。\n']);
end

truth_labels = build_joint_truth_labels(events, cfg);
for i = 1:numel(selected_ids)
    id = selected_ids(i);
    detail = find_track_detail(metrics, id);
    file_path = fullfile(output_dir, sprintf('track_%s_report.txt', safe_id_text(id)));
    write_track_report(file_path, id, detail, est, events, truth_labels, history_level);
    info.files{i} = file_path;
    info.n_reports = info.n_reports + 1;
    if print_console
        print_track_console(detail, file_path, history_level);
    end
end
end

function write_track_report(path, id, detail, est, events, truth_labels, history_level)
[fid, msg] = fopen(path, 'w', 'n', 'UTF-8');
if fid < 0, error('export_track_diagnostics:OpenFile', '无法写入 %s: %s', path, msg); end
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>

fprintf(fid, '逻辑航迹诊断报告\n');
fprintf(fid, '生成时间\t%s\n', datestr(now, 'yyyy-mm-dd HH:MM:SS'));
fprintf(fid, '航迹ID\t%.15g\n', id);
fprintf(fid, '历史级别\t%s\n', history_level);
fprintf(fid, '说明\t航迹正确判定使用一对一匹配、最少关联点和真值量测覆盖门限。\n');
fprintf(fid, '判定公式\t正确关联到匹配真值的量测数/该真值在对应评价范围的全部量测数。\n');
fprintf(fid, '评价范围\t总体为全部输入，三维为RAE，二维被动为进入二维处理支路的被动AE。\n');
fprintf(fid, '误差参考\t同事件同标签参考量测均值；仿真真实真值精度由独立真实真值评价模块给出。\n');
fprintf(fid, 'RMSE_reference\tmeasurement_event_mean\n');
if strcmp(history_level, 'output')
    fprintf(fid, ['诊断完整性\t当前output模式仅保留路由/来源字段；association cost和update NIS已被压缩，' ...
        '本报告相应字段为NaN。\n\n']);
else
    fprintf(fid, '诊断完整性\tassociation cost及已记录的空间更新NIS可用；出生量测没有更新NIS。\n\n');
end

fprintf(fid, '[航迹基本信息]\n');
fprintf(fid, '输出点数\t%d\n', detail.n_output_points);
fprintf(fid, '输出事件/帧数\t%d\n', detail.n_output_events);
fprintf(fid, '二维输出点数\t%d\n', detail.n_output_2d);
fprintf(fid, '三维输出点数\t%d\n', detail.n_output_3d);
fprintf(fid, '起始时间_s\t%.9f\n', detail.start_time_s);
fprintf(fid, '结束时间_s\t%.9f\n', detail.end_time_s);
fprintf(fid, '持续时间_s\t%.9f\n', detail.duration_s);
fprintf(fid, '量测利用记录总数\t%d\n', detail.n_associations);
fprintf(fid, '主动三维利用数\t%d\n', detail.n_active_range_assoc);
fprintf(fid, '主动纯角度利用数\t%d\n', detail.n_active_angle_assoc);
fprintf(fid, '被动角度利用数\t%d\n\n', detail.n_passive_assoc);

write_scope_summary(fid, '总体主被动评价', detail.overall);
write_scope_summary(fid, '三维主动评价', detail.three_d);
write_scope_summary(fid, '二维被动AE评价', detail.two_d);

fprintf(fid, '[单轨RMSE：标签量测参考一致性]\n');
fprintf(fid, '匹配参考实例\t%.15g\n', detail.overall.matched_truth_id);
fprintf(fid, '角度样本数\t%d\n', detail.angle_rmse.n);
fprintf(fid, '方位RMSE_deg\t%.9f\n', detail.angle_rmse.az_deg);
fprintf(fid, '俯仰RMSE_deg\t%.9f\n', detail.angle_rmse.el_deg);
fprintf(fid, 'LOS_RMSE_deg\t%.9f\n', detail.angle_rmse.los_deg);
fprintf(fid, '位置样本数\t%d\n', detail.position_rmse.n);
fprintf(fid, 'East_RMSE_m\t%.9f\n', detail.position_rmse.e_m);
fprintf(fid, 'North_RMSE_m\t%.9f\n', detail.position_rmse.n_m);
fprintf(fid, 'Up_RMSE_m\t%.9f\n', detail.position_rmse.u_m);
fprintf(fid, '3D_RMSE_m\t%.9f\n\n', detail.position_rmse.three_d_m);

write_output_history(fid, id, detail, est, events, truth_labels);
write_association_history(fid, id, detail, est, events, truth_labels);
end

function write_scope_summary(fid, label, d)
fprintf(fid, '[%s]\n', label);
fprintf(fid, '适用\t%d\n', d.applicable);
fprintf(fid, '存在一对一匹配\t%d\n', d.has_match);
fprintf(fid, '匹配参考实例\t%.15g\n', d.matched_truth_id);
fprintf(fid, '带标签利用数\t%d\n', d.n_labeled_assoc);
fprintf(fid, '正确标签利用数\t%d\n', d.n_correct);
fprintf(fid, '不一致标签利用数\t%d\n', d.n_inconsistent);
fprintf(fid, '关联一致率\t%.9f\n', d.association_consistency);
fprintf(fid, '对应参考实例全部量测数\t%d\n', d.truth_total);
fprintf(fid, '参考量测覆盖率\t%.9f\n', d.coverage);
fprintf(fid, '未正确覆盖参考量测数\t%d\n', d.n_truth_not_correct);
fprintf(fid, '正式输出同步覆盖率\t%.9f\n', d.formal_output_coverage);
fprintf(fid, '正式输出覆盖量测数\t%d\n', d.n_formal_output_covered);
fprintf(fid, '正式输出参考量测数\t%d\n', d.n_formal_output_reference);
fprintf(fid, '覆盖率判定门限\t%.9f\n', d.purity_threshold);
fprintf(fid, '最少带标签利用点\t%d\n', d.min_labeled_assoc);
fprintf(fid, '航迹级正确判定\t%d\n', d.is_correct);
fprintf(fid, '单轨航迹级正确率\t%.2f%%\n\n', 100 * d.track_level_score);
end

function write_output_history(fid, id, detail, est, events, truth_labels)
fprintf(fid, '[逐输出点]\n');
fprintf(fid, ['event\ttime_s\toutput_dim\tmode\taz_deg\tel_deg\trange_m' ...
    '\tE_m\tN_m\tU_m\tVE_mps\tVN_mps\tVU_mps\tmatched_reference' ...
    '\tposition_truth' ...
    '\tref_az_deg\tref_el_deg\taz_error_deg\tel_error_deg\tlos_error_deg' ...
    '\tref_E_m\tref_N_m\tref_U_m\terr_E_m\terr_N_m\terr_U_m\terr_3D_m\n']);
for k = 1:min(numel(events), numel_field(est, 'output'))
    out = est.output{k};
    for q = 1:numel(out)
        if field_num(out(q), 'id', NaN) ~= id, continue; end
        reference_id = detail.overall.matched_truth_id;
        [ref_angle, ref_position] = event_reference(events(k), truth_labels, k, reference_id);
        az = field_num(out(q), 'az_deg', NaN); el = field_num(out(q), 'el_deg', NaN);
        pos = field_vec(out(q), 'position_enu', 3); vel = field_vec(out(q), 'velocity_enu', 3);
        az_err = angle_delta(az, ref_angle(1)); el_err = el - ref_angle(2);
        los_err = los_separation_deg(az, el, ref_angle(1), ref_angle(2));
        pos_err = pos - ref_position;
        if field_num(out(q), 'output_dim', 0) ~= 3, pos_err(:) = NaN; end
        fprintf(fid, ['%d\t%.9f\t%d\t%s\t%.9f\t%.9f\t%.9f' ...
            '\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.15g' ...
            '\t%.15g' ...
            '\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f' ...
            '\t%.9f\t%.9f\t%.9f\t%.9f\n'], ...
            k, field_num(out(q), 't_sec', events(k).t_sec), ...
            field_num(out(q), 'output_dim', 0), field_text(out(q), 'mode', ''), ...
            az, el, field_num(out(q), 'range_m', NaN), pos, vel, ...
            reference_id, reference_id, ...
            ref_angle, az_err, el_err, los_err, ref_position, pos_err, norm_if_finite(pos_err));
    end
end
fprintf(fid, '\n');
end

function write_association_history(fid, id, detail, est, events, truth_labels)
fprintf(fid, '[逐量测利用与关联诊断]\n');
fprintf(fid, ['event\ttime_s\ttype\tfilter_dim\tmeas_index\traw_id\tmapped_reference' ...
    '\tmatched_reference\tis_consistent\tmeasurement_dim\trange_updated' ...
    '\tassociation_cost\tspace_update_nis\tnormalized_space_nis' ...
    '\tmeas_az_deg\tmeas_el_deg\tmeas_E_m\tmeas_N_m\tmeas_U_m\n']);
total = 0; cost_available = 0; nis_available = 0;
for k = 1:min(numel(events), numel_field(est, 'assoc'))
    a = est.assoc{k};
    if isempty(a) || ~isfield(a, 'id'), continue; end
    for q = find(reshape(a.id, 1, []) == id)
        total = total + 1;
        type = indexed_text(a, 'type', q, ''); mi = indexed_num(a, 'meas_index', q, 0);
        [raw_id, mapped_reference, meas_t, meas_angle, meas_pos] = ...
            association_measurement(events(k), truth_labels, k, type, mi);
        matched_reference = detail.overall.matched_truth_id;
        is_consistent = isfinite(mapped_reference) && isfinite(matched_reference) && ...
            mapped_reference == matched_reference;
        measurement_dim = indexed_num(a, 'measurement_dim', q, 0);
        range_updated = indexed_bool(a, 'range_updated', q, false);
        cost = indexed_num(a, 'cost', q, NaN);
        space_nis = indexed_num(a, 'space_nis', q, NaN);
        if isfinite(cost), cost_available = cost_available + 1; end
        if isfinite(space_nis), nis_available = nis_available + 1; end
        if measurement_dim == 3
            normalized_space_nis = space_nis / 3;
        else
            normalized_space_nis = NaN;
        end
        fprintf(fid, ['%d\t%.9f\t%s\t%d\t%d\t%.15g\t%.15g\t%.15g\t%d\t%d\t%d' ...
            '\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\n'], ...
            k, meas_t, type, association_filter_dim(est, k, a, q, id), mi, ...
            raw_id, mapped_reference, matched_reference, is_consistent, ...
            measurement_dim, range_updated, cost, space_nis, normalized_space_nis, ...
            meas_angle, meas_pos);
    end
end
fprintf(fid, '\n[诊断字段完整性]\n');
fprintf(fid, '量测利用记录数\t%d\n', total);
fprintf(fid, '含association cost记录数\t%d\n', cost_available);
fprintf(fid, '含主动空间update NIS记录数\t%d\n', nis_available);
fprintf(fid, '说明\tassociation cost是关联/分配代价，不等同于所有更新分支的NIS。\n');
fprintf(fid, '说明\tspace_update_nis仅表示已由滤波器显式保存的主动三维空间更新前NIS；出生及其他分支可为NaN。\n');
end

function print_track_console(d, path, history_level)
if d.n_output_3d > 0 && d.n_output_2d > 0, mode = '2D/3D';
elseif d.n_output_3d > 0, mode = '3D'; else, mode = '2D'; end
fprintf(['[单轨报告] Track %.15g: %s, 输出=%d点/%d帧, 参考实例=%.15g, ' ...
    '参考覆盖=%.2f%%, 输出覆盖=%.2f%%, 一致率=%.2f%%, 航迹级=%s\n'], ...
    d.track_id, mode, d.n_output_points, d.n_output_events, ...
    d.overall.matched_truth_id, 100*d.overall.coverage, ...
    100*d.overall.formal_output_coverage, 100*d.overall.association_consistency, ...
    pass_text(d.overall.is_correct));
if strcmp(history_level, 'output')
    fprintf('             详细关联cost/NIS未保存；重新运行 diagnostic/full 可获得。\n');
end
fprintf('             TXT: %s\n', path);
end

function d = find_track_detail(metrics, id)
details = metrics.track_details; i = find([details.track_id] == id, 1);
if isempty(i), error('export_track_diagnostics:MissingDetail', '评价中不存在航迹 %.15g。', id); end
d = details(i);
end

function ids = output_track_ids(est)
ids = zeros(1, 0);
if isfield(est, 'output')
    for k = 1:numel(est.output)
        out = est.output{k};
        if ~isempty(out) && isfield(out, 'id'), ids = [ids, [out.id]]; end %#ok<AGROW>
    end
end
ids = unique(ids(isfinite(ids)));
end

function [raw_id, mapped_reference, t, angle, position] = association_measurement(e, labels, k, type, mi)
raw_id = NaN; mapped_reference = NaN; t = e.t_sec;
angle = nan(2,1); position = nan(3,1);
if mi < 1 || ~ischar(type), return; end
if strncmp(type, 'active', 6)
    if mi <= e.active.n_meas
        if isfield(e.active, 'ids') && mi <= numel(e.active.ids), raw_id = e.active.ids(mi); end
        if k <= numel(labels.active) && mi <= numel(labels.active{k}), mapped_reference = labels.active{k}(mi); end
        t = indexed_vector(e.active.t_sec, mi, e.t_sec);
        if size(e.active.rae,1) >= 3 && size(e.active.rae,2) >= mi, angle = e.active.rae(2:3,mi); end
        if size(e.active.xyz,1) >= 3 && size(e.active.xyz,2) >= mi, position = e.active.xyz(1:3,mi); end
    end
elseif strncmp(type, 'passive', 7)
    if mi <= e.passive.n_meas
        if isfield(e.passive, 'ids') && mi <= numel(e.passive.ids), raw_id = e.passive.ids(mi); end
        if k <= numel(labels.passive) && mi <= numel(labels.passive{k}), mapped_reference = labels.passive{k}(mi); end
        t = indexed_vector(e.passive.t_sec, mi, e.t_sec);
        if size(e.passive.ang,1) >= 2 && size(e.passive.ang,2) >= mi, angle = e.passive.ang(1:2,mi); end
    end
end
end

function [angle, position] = event_reference(e, labels, k, truth_id)
angle = nan(2,1); position = nan(3,1);
if ~isfinite(truth_id) || k > numel(labels.active) || k > numel(labels.passive), return; end
active_label = labels.active{k}; passive_label = labels.passive{k}; angles = zeros(2,0);
ia = find(active_label == truth_id); ia = ia(ia <= size(e.active.rae,2));
if ~isempty(ia) && size(e.active.rae,1) >= 3, angles = [angles, e.active.rae(2:3,ia)]; end %#ok<AGROW>
ip = find(passive_label == truth_id); ip = ip(ip <= size(e.passive.ang,2));
if ~isempty(ip) && size(e.passive.ang,1) >= 2, angles = [angles, e.passive.ang(1:2,ip)]; end %#ok<AGROW>
angles = angles(:, all(isfinite(angles),1));
if ~isempty(angles)
    angle = [atan2d(mean(sind(angles(1,:))), mean(cosd(angles(1,:)))); mean(angles(2,:))];
end
has_range = false(1,e.active.n_meas); n = min(numel(e.active.has_range),e.active.n_meas);
if n > 0, has_range(1:n) = logical(e.active.has_range(1:n)); end
iz = find(active_label == truth_id & has_range); iz = iz(iz <= size(e.active.xyz,2));
if ~isempty(iz) && size(e.active.xyz,1) >= 3
    Z = e.active.xyz(1:3,iz); Z = Z(:,all(isfinite(Z),1));
    if ~isempty(Z), position = mean(Z,2); end
end
end

function out = empty_event()
out = struct('t_sec', NaN, 'active', struct('n_meas',0,'t_sec',[],'ids',[], ...
    'rae',zeros(3,0),'xyz',zeros(3,0),'has_range',false(1,0)), ...
    'passive', struct('n_meas',0,'t_sec',[],'ids',[],'ang',zeros(2,0)));
end

function n = numel_field(s,name)
if isfield(s,name), n = numel(s.(name)); else, n = 0; end
end
function v = field_num(s,name,fallback)
if isstruct(s) && isfield(s,name) && ~isempty(s.(name)), v = s.(name); else, v = fallback; end
if ~isscalar(v), v = fallback; end
end
function v = field_vec(s,name,n)
v = nan(n,1); if ~isfield(s,name) || isempty(s.(name)), return; end
x = s.(name)(:); q = min(n,numel(x)); v(1:q)=x(1:q);
end
function s = field_text(x,name,fallback)
s = fallback;
if isstruct(x) && isfield(x,name) && ischar(x.(name)), s = x.(name); end
s = strrep(s,sprintf('\t'),' '); s = strrep(s,sprintf('\n'),' ');
end
function v = indexed_num(s,name,i,fallback)
v = fallback;
if isfield(s,name) && numel(s.(name)) >= i
    x=s.(name)(i); if isnumeric(x) || islogical(x), v=double(x); end
end
end
function v = indexed_bool(s,name,i,fallback)
v = fallback;
if isfield(s,name) && numel(s.(name)) >= i, v = logical(s.(name)(i)); end
end
function s = indexed_text(x,name,i,fallback)
s = fallback;
if isfield(x,name) && iscell(x.(name)) && numel(x.(name)) >= i && ischar(x.(name){i}), s=x.(name){i}; end
end
function dim = association_filter_dim(est,k,a,q,id)
dim = indexed_num(a,'filter_dim',q,NaN);
if isfinite(dim) && any(dim == [2,3]), return; end
dim = indexed_num(a,'input_dim',q,0);
if isfinite(dim) && any(dim == [2,3]), return; end
dim = 0;
if ~isfield(est,'logical_tracks') || k > numel(est.logical_tracks) || isempty(est.logical_tracks{k}), return; end
tracks=est.logical_tracks{k}; j=find([tracks.id]==id,1); if isempty(j), return; end
mode=field_text(tracks(j),'mode','');
if strncmp(mode,'2d',2), dim=2; elseif strncmp(mode,'3d',2), dim=3;
elseif isfield(tracks,'state3d') && all(isfinite(tracks(j).state3d)), dim=3;
elseif isfield(tracks,'angle_state') && all(isfinite(tracks(j).angle_state)), dim=2; end
end
function v = indexed_vector(x,i,fallback)
if isempty(x), v=fallback; elseif isscalar(x), v=x; elseif numel(x)>=i, v=x(i); else, v=fallback; end
end
function d = angle_delta(a,b)
if ~isfinite(a) || ~isfinite(b), d=NaN; else, d=mod(a-b+180,360)-180; end
end
function n = norm_if_finite(x)
if all(isfinite(x)), n=norm(x); else, n=NaN; end
end
function s = safe_id_text(id)
s=sprintf('%.15g',id); s=regexprep(s,'[^0-9A-Za-z_-]','_');
end
function s = pass_text(tf)
if tf, s='正确'; else, s='不正确'; end
end
function v = get_opt(opts,name,fallback)
if isstruct(opts) && isfield(opts,name) && ~isempty(opts.(name)), v=opts.(name); else, v=fallback; end
end
