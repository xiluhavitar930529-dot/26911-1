% run_fusion_main_fixed.m
% Compatibility launcher for the complete direct-source version.
% All metric and 3D->2D fixes are already implemented in the source files;
% this launcher does not modify source code at runtime.

root = fileparts(mfilename('fullpath'));
clear evaluate_joint_tracking_metrics run_filter_joint_2d3d config_fusion validate_config_fusion;
rehash;
run(fullfile(root, 'run_fusion_main.m'));
