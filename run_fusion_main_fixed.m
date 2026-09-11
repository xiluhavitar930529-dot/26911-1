% run_fusion_main_fixed.m
% One-click launcher for the latest joint-tracking fixes.
%
% Applied before the original main script:
%   1) confirmed-association metric correction;
%   2) 3D->2D degradation control based only on
%      - radial/range 95% uncertainty, and
%      - time since last range-capable active 3-D measurement.
%
% No +package dependency is required.

root = fileparts(mfilename('fullpath'));
apply_confirmed_association_fix(fullfile(root, 'evaluate_joint_tracking_metrics.m'));
apply_3d2d_control_fix(root);
clear evaluate_joint_tracking_metrics run_filter_joint_2d3d config_fusion validate_config_fusion;
rehash;
run(fullfile(root, 'run_fusion_main.m'));
