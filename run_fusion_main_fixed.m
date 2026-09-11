% run_fusion_main_fixed.m
% One-click launcher for the confirmed-association metric hotfix.
%
% First patch evaluate_joint_tracking_metrics.m in-place, then execute the
% original run_fusion_main.m. No +package directory or extra MATLAB path is
% required after the patch.

root = fileparts(mfilename('fullpath'));
apply_confirmed_association_fix(fullfile(root, 'evaluate_joint_tracking_metrics.m'));
clear evaluate_joint_tracking_metrics;
rehash;
run(fullfile(root, 'run_fusion_main.m'));
