function results = test_single_track_diagnostics_contract(report_dir)
%TEST_SINGLE_TRACK_DIAGNOSTICS_CONTRACT Regression for report/history contract.
root = fileparts(mfilename('fullpath'));
addpath(root);
if nargin < 1, report_dir = tempname; end
if ~exist(report_dir, 'dir'), mkdir(report_dir); end

cfg = struct();
cfg.metrics_max_print = 0;
cfg.truth_id_split_enabled = false;
cfg.track_accuracy_min_assoc = 1;
cfg.track_accuracy_purity_th = 0;

event = struct();
event.t_sec = 0;
event.active = struct('n_meas',1,'t_sec',0,'ids',11,'has_range',true, ...
    'rae',[1000;0;0],'xyz',[0;1000;0]);
event.passive = struct('n_meas',0,'t_sec',[],'ids',[],'ang',zeros(2,0));

o = struct('id',1,'truth_id',11,'output_dim',3,'t_sec',0,'mode','3d', ...
    'az_deg',0,'el_deg',0,'range_m',1000,'position_enu',[0;1000;0], ...
    'velocity_enu',[0;0;0]);

base_assoc = struct('id',1,'type',{{'active'}},'meas_index',1, ...
    'measurement_dim',3,'filter_dim',3,'input_dim',3,'range_updated',true);

% output mode: compact history is allowed, but report must explicitly state
% that detailed association diagnostics are unavailable instead of inventing
% innovation/NIS values.
est = struct('output',{{o}},'assoc',{{base_assoc}},'logical_tracks',{{}}, ...
    'history_level','output');
metrics = evaluate_joint_tracking_metrics(est, event, cfg);
opts = struct('output_dir',report_dir,'print_console',false,'metrics',metrics);
info_output = export_track_diagnostics(est,event,cfg,1,opts);
text_output = fileread(info_output.files{1});
assert(contains(text_output,'诊断完整性'));
assert(contains(text_output,'association cost和update NIS已被压缩'));
assert(~contains(text_output,'nu1'));
assert(~contains(text_output,'innovation_kind'));

% diagnostic mode: stored cost and spatial NIS must be exported verbatim.
diag_assoc = base_assoc;
diag_assoc.cost = 2.5;
diag_assoc.space_nis = 3.0;
est.assoc = {diag_assoc};
est.history_level = 'diagnostic';
metrics = evaluate_joint_tracking_metrics(est,event,cfg);
opts.metrics = metrics;
info_diag = export_track_diagnostics(est,event,cfg,1,opts);
text_diag = fileread(info_diag.files{1});
assert(contains(text_diag,'association_cost'));
assert(contains(text_diag,'space_update_nis'));
assert(contains(text_diag,sprintf('2.500000000')));
assert(contains(text_diag,sprintf('3.000000000')));

results = struct('output_contract_ok',true,'diagnostic_contract_ok',true, ...
    'report_dir',report_dir);
disp(results);
end
