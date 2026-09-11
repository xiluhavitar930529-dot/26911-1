% run_fusion_main_fixed.m
% One-click launcher for the latest joint-tracking fixes.
%
% The 3D->2D degradation logic is now implemented directly inside
% run_filter_joint_2d3d.m. This launcher only applies the independent
% confirmed-association metric correction before running the original main.

root = fileparts(mfilename('fullpath'));
apply_confirmed_association_fix(fullfile(root, 'evaluate_joint_tracking_metrics.m'));
clear evaluate_joint_tracking_metrics run_filter_joint_2d3d config_fusion validate_config_fusion;
rehash;
run(fullfile(root, 'run_fusion_main.m'));
