function cfg = config_fusion()
%CONFIG_FUSION 主被动雷达融合跟踪系统统一配置。
%
% 当前推荐框架 joint_2d3d 的在线处理顺序：
%   数据读取 -> 原始时间事件组织 -> 成熟三维主干与残余二维分支因果更新
%   -> 同ID二维/三维质量切换 -> 分维度评价与绘图。
%
% 参数标签说明：
%   [联合] joint_2d3d 在线联合框架使用。
%   [三维] 成熟三维主干 run_filter_adapt_ckf 使用。
%   [二维] 在线残余纯角度分支使用。
%   [兼容] legacy_active3d 或通用独立滤波路径保留。
%   [评价] 只参与结果评价/绘图，不参与滤波关联。

cfg = struct();

%% 1. 运行入口与数据位置
cfg.processing_framework = 'joint_2d3d'; % joint_2d3d（推荐）或 legacy_active3d
cfg.data_dir = 'C:\Users\topsy\Desktop\数据仿真\sim_output_air_air_100t_500k\fusion_wide_input'; % 输入目录
cfg.active_files = {
                    'meas_sensor1.txt'
                    'meas_sensor2.txt'
                    'meas_sensor3.txt'
                   }; % 主动文件/分片
cfg.active_files_share_sensor = true;     % true=上述TXT均为同一部主动雷达的存储分片

cfg.passive_files = {
                     'passive1.txt'
                     'passive2.txt'
                     'passive3.txt'                
                     'passive4.txt'
                     'passive5.txt'
                     'passive6.txt'
                     };
cfg.platform_file = 'platform.csv';       % 观测平台轨迹文件
cfg.delimiter = 'auto';                   % 文本分隔符；auto=自动识别

%% 2. 数据读取范围、并行与缓存
% 百分比和max_rows用于雷达文件；主入口始终完整加载平台支持数据。
cfg.read_start_percent = 0;               % 读取起点[%]
cfg.read_end_percent = [];                % 读取终点[%]；空值使用 read_percent
cfg.read_percent = 100;                   % read_end_percent 为空时的读取比例[%]
cfg.time_range_s = [];                    % 限定时间段[s]；空值=不限制
cfg.max_rows = inf;                       % 每个文件最大读取行数
cfg.max_targets_per_row = 192;            % 单行最大目标数

cfg.parallel_file_loading = true;         % 并行解析多个雷达文件
cfg.parallel_file_workers = 0;            % 并行进程数；0=自动
cfg.parse_cache_enabled = true;           % 缓存文件解析结果
cfg.parse_cache_dir = fullfile(tempdir, 'fusion_radar_parse_cache'); % 缓存目录

%% 3. 输入文件列格式
% 3.1 主动雷达宽表
cfg.active.time_col = 1;                  % 时间列
cfg.active.count_col = 5;                 % 目标数列
cfg.active.target_id_col = 6;             % 首目标真值编号列；仅评价使用
cfg.active.valid_cols = [12, 13, 14];     % 首目标有效标志列
cfg.active.az_col = 19;                   % 首目标方位角列
cfg.active.el_col = 20;                   % 首目标俯仰角列
cfg.active.range_col = 23;                % 首目标距离列
cfg.active.stride = 56;                   % 相邻目标字段跨度
cfg.active.angle_unit = 'mrad';           % 输入角度单位
cfg.active.time_format = 'hms';           % 输入时间格式

% 3.2 被动雷达宽表
cfg.passive.time_col = 1;                 % 时间列
cfg.passive.count_col = 5;                % 目标数列
cfg.passive.target_id_col = 6;            % 首目标真值编号列；仅评价使用
cfg.passive.valid_cols = [33, 34];        % 首目标有效标志列
cfg.passive.az_col = 35;                  % 首目标方位角列
cfg.passive.el_col = 36;                  % 首目标俯仰角列
cfg.passive.stride = 76;                  % 相邻目标字段跨度
cfg.passive.angle_unit = 'mrad';          % 输入角度单位
cfg.passive.time_format = 'hms';          % 输入时间格式

% 3.3 动平台轨迹
cfg.platform.time_col = 1;                % 时间列
cfg.platform.lat_col = 2;                 % 纬度列
cfg.platform.lon_col = 3;                 % 经度列
cfg.platform.alt_col = 4;                 % 高度列
cfg.platform.angle_unit = 'mrad';         % 经纬度单位
cfg.platform.time_format = 'hms';         % 输入时间格式
cfg.platform_max_extrapolation_s = 0.1;   % 平台端点线性外推上限[s]，另限一个采样周期

%% 4. 坐标转换、时间分帧与主动量测凝聚
cfg.local_origin = 'first_platform';      % ENU原点；first_platform=首个平台点
cfg.frame_time_window_s = 0.1;          % 原始量测同帧时间窗[s]

cfg.condense_enable = true;              % 启用主动量测空时凝聚
cfg.condense_method = 'spatiotemporal';   % 凝聚方法
cfg.condense_radius_m = 200;              % 初始空间聚类半径[m]
cfg.condense_res_range_m = 100;           % 距离分辨率[m]
cfg.condense_res_az_deg = 0.2;            % 方位分辨率[deg]
cfg.condense_res_el_deg = 0.2;            % 俯仰分辨率[deg]
cfg.condense_gate_gamma = 9;              % 已有凝聚中心统计门
cfg.condense_birth_gamma = 6;             % 新生凝聚中心统计门
cfg.condense_amax = 10;                   % 凝聚运动最大加速度[m/s^2]
cfg.condense_vel_beta = 0.15;             % 凝聚速度更新系数
cfg.condense_coast = 3;                   % 凝聚中心最大滑行帧数
cfg.condense_protect_diff_id = false;      % 必须false；真值编号不得影响主动凝聚

%% 5. [联合] 扫描事件、异步量测与重叠文件去重
cfg.async_microbatch_dt_s = 0.005;        % 实时事件微批窗口[s]；不凝聚原始角度点
cfg.async_same_time_tolerance_s = 1e-9;   % 浮点时间相等容差[s]
cfg.async_active_time_mode = 'frame';     % 主动事件时间；frame=使用帧时间
cfg.async_passive_count_miss = false;     % 纯被动事件不计为主动漏检

cfg.joint_shard_dedup_enabled = false;    % 仅文件确有重叠重复行时启用
cfg.joint_shard_dedup_use_truth_id = false; % 必须false；分片去重禁止依赖真值编号
cfg.joint_shard_duplicate_time_s = 1e-9;  % 精确重复行时间容差[s]
cfg.joint_shard_duplicate_angle_deg = 1e-9; % 精确重复行角度容差[deg]

%% 6. 传感器量测噪声
cfg.sigma_range_m = 150;                  % 主动距离标准差[m]
cfg.sigma_az_deg = 0.08;                  % 主动方位标准差[deg]
cfg.sigma_el_deg = 0.06;                  % 主动俯仰标准差[deg]
cfg.sigma_passive_az_deg = 0.05;          % 被动方位标准差[deg]
cfg.sigma_passive_el_deg = 0.04;          % 被动俯仰标准差[deg]

%% 7. [三维] 成熟主动三维CKF主干
% joint_2d3d 与 legacy_active3d 均使用本节。联合模式没有另起一套三维滤波。

% 7.1 状态模型与自适应量测噪声
cfg.x_dim = 9;                            % CA状态维数：[E,vE,aE,N,vN,aN,U,vU,aU]
cfg.z_dim = 3;                            % 笛卡尔位置量测维数
cfg.sigma_a_cs = 25;                     % 当前统计模型加速度噪声
cfg.alpha_cs = 0.4;                      % 当前统计模型机动频率
cfg.a_max_cs = 120;                      % 当前统计模型最大加速度[m/s^2]

cfg.adapt_R_enabled = true;              % 在线估计主动量测协方差
cfg.adapt_R_alpha = 0.15;                % 自适应协方差更新系数
cfg.adapt_R_window = 200;                % 协方差统计窗口
cfg.adapt_R_min_samples = 100;            % 开始自适应的最少样本数

cfg.R_default_diag = [2500^2, 2500^2, 2500^2]; % 默认位置量测方差[m^2]
cfg.R_min_diag = [200^2, 200^2, 200^2];   % 位置量测方差下限[m^2]

cfg.P_S = 0.99;                          % 航迹生存概率
cfg.P_D = 0.70;                          % 主动检测概率
cfg.missed_weight_decay = 1e-28;         % 漏检权重衰减系数

% 7.2 主动量测关联、新生与生命周期
cfg.gating_gamma = 35;                   % 主动统计关联门
cfg.assoc_pos_gate_m = 3000;             % 主动位置预关联门[m]
cfg.active_pre_gate_enabled = false;     % 启用主动位置预门控

cfg.cost_unmatched = 150;                % 未匹配虚拟分配代价
cfg.assoc_accept_nis = 16;               % 主动关联最终接受NIS门
cfg.nis_gate = 16;                       % CKF更新健康NIS门
cfg.nis_max_bad = 4;                     % 连续异常NIS删除次数

cfg.M_confirm = 3;                       % 三维航迹M/N确认的M
cfg.N_confirm = 5;                       % 三维航迹M/N确认的N

cfg.P_pos_birth = 1e6;                   % 新生位置方差[m^2]
cfg.P_vel_birth = 500^2;                 % 新生速度方差[(m/s)^2]
cfg.P_acc_birth = 80^2;                  % 新生加速度方差[(m/s^2)^2]

cfg.vinit_baseline_s = 0.2;              % 两点速度初始化最小时间差[s]
cfg.vinit_min_disp_m = 80;               % 两点速度初始化最小位移[m]

cfg.birth_guard_m = 1000;                % 主动新生空间抑制半径[m]
cfg.birth_suppress_gated = true;         % 门内未分配量测禁止新生

cfg.merge_pos_dist_m = 600;              % 三维重复航迹位置合并门[m]
cfg.dedup_vel_angle_deg = 30;             % 三维合并速度夹角门[deg]
cfg.dedup_min_speed = 20;                % 使用速度夹角门的最低速度[m/s]
cfg.max_tracks = 400;                    % 三维主干及二维分支共用容量上限

cfg.max_coast_frames = 2;               % 帧级滑行/重捕获控制长度
cfg.tentative_max_miss = 1;              % 暂态航迹最大连续漏检数
cfg.track_timeout_enabled = true;        % 启用秒级静默超时

cfg.tentative_max_silence_s = 1.5;       % 三维暂态静默超时[s]
cfg.confirmed_max_silence_s = 5;        % 三维确认航迹静默超时[s]
cfg.confirmed_output_max_silence_s = 0.5; % 三维正式输出最大静默[s]，不删除内部航迹

cfg.prune_threshold = 1e-12;             % 暂态权重删除阈值
cfg.weight_floor_confirmed = 0.55;        % 确认航迹权重下限
cfg.hit_weight_gain = 0.08;              % 命中权重增量
cfg.coast_acc_decay = 0.30;              % 滑行时加速度衰减系数

% 7.3 多主动回波融合、鲁棒更新与重捕获
cfg.meas_fuse_enabled = true;            % 融合同一目标的多主动回波
cfg.meas_fuse_weight = 'likelihood';     % 主动回波融合权重
cfg.meas_fuse_R_scale = 2.0;             % 主动融合协方差缩放
cfg.meas_fuse_cluster_gamma = 30;        % 次回波统计一致性门
cfg.meas_fuse_cluster_dist_m = 2000;     % 次回波空间一致性门[m]

cfg.meas_robust_enabled = true;          % 使用鲁棒量测更新
cfg.robust_k = 2.0;                      % 鲁棒降权转折系数
cfg.robust_R_max_infl = 4;               % 鲁棒协方差最大膨胀
cfg.reacq_P_inflate = 15;                % 重捕获状态协方差膨胀
cfg.reacq_vel_inflate = 5;               % 重捕获速度协方差额外膨胀

% 7.4 三维速度趋势辅助
cfg.vel_trend_enabled = true;            % 使用历史位置趋势辅助速度
cfg.vel_trend_window = 50;               % 最大历史样本数
cfg.vel_trend_span_s = 1.0;              % 最小拟合时间跨度[s]
cfg.vel_trend_min_n = 10;                % 最少拟合样本数
cfg.vel_trend_blend = 0.55;              % 趋势速度融合比例
cfg.vel_trend_min_speed = 20;            % 启用趋势的最低速度[m/s]
cfg.vel_trend_conf_only = true;          % 仅修正确认航迹

% 7.5 真值隔离与旧三维IMM
cfg.use_target_id_prior = false;         % 必须保持false；真值编号不得参与滤波

cfg.use_imm = false;                     % 启用成熟三维主干的旧IMM实现
cfg.imm_p_cv_stay = 0.95;                % CV模型保持概率
cfg.imm_p_ca_stay = 0.95;                % CA模型保持概率
cfg.imm_mu_init_cv = 0.5;                % 新生CV初始概率
cfg.imm_cv_sigma_a = 5;                  % CV模型加速度噪声[m/s^2]
cfg.imm_cv_acc_floor = 1e-3;             % CV加速度状态方差下限

%% 8. [三维] 被动角度辅助更新
% 联合模式会强制允许同步/纯被动事件更新已有三维航迹，并禁止被动命中直接确认三维航迹。
cfg.passive_bearing_enabled = true;      % 开启已有三维航迹的被动角度更新
cfg.passive_bearing_gate = 16;           % 被动角度关联门
cfg.passive_bearing_nis_gate = 16;       % 被动更新后的航迹健康NIS门
cfg.passive_bearing_weight_gain = 0.5;   % 被动命中权重增益比例
cfg.passive_bearing_confirm_hit = false; % 单次被动命中不直接计入逻辑确认
cfg.passive_bearing_update_on_active = true;  % 主动事件中融合被动角度
cfg.passive_bearing_update_on_pure = true;    % 纯被动事件中维持三维航迹

cfg.passive_bearing_update_active_hit_tracks = false; % legacy路径是否重复更新主动命中航迹
cfg.passive_bearing_min_dt_s = 0.15;     % legacy路径被动更新最小间隔[s]
cfg.passive_bearing_fast_gate_deg = 2.5; % legacy路径角度快速门[deg]
cfg.joint_passive_fast_gate_deg = 0.15;  % [联合] 三维被动更新方位/俯仰硬门[deg]；inf=仅使用NIS门
cfg.joint_3d_companion_accept_nis = 16;  % 三维伴随分支被动投影NIS最终验收门
cfg.joint_3d_companion_fast_gate_deg = 0.15; % 三维伴随分支方位/俯仰硬门[deg]

%% 9. [二维] 在线残余纯角度分支
% 三维主干未解释的角度量测在同一因果循环内进入本分支。

% 9.1 起始确认、关联与生命周期
cfg.joint_confirm_M = 3;                 % 二维确认所需等效命中数M
cfg.joint_confirm_N = 5;                 % 二维基础确认事件窗N

cfg.joint_passive_confirm_consecutive_hits = 1 ; % 连续被动命中折算一次逻辑确认命中
cfg.joint_passive_confirm_max_gap_s = 0.20; % 连续被动命中最大间隔[s]
cfg.joint_passive_confirm_window_s = 1.50; % 二维起始确认总时间窗[s]

cfg.joint_gate_2d = 9.2103;              % 二维角度关联卡方门
cfg.joint_angle_assoc_cov_penalty = 1.0; % 关联代价中的预测协方差惩罚
cfg.joint_unmatched_cost = 50;           % 未匹配虚拟分配代价
cfg.joint_birth_explain_nis = 1.0;       % 已有航迹对新生量测的解释门

cfg.joint_2d_accept_nis = 16;            % 宽门候选的最终二维NIS验收门
cfg.joint_2d_max_direct_gap_s = 2.0;      % 旧二维航迹允许直接续接的最大静默[s]
cfg.joint_2d_max_az_residual_deg = 15;    % 直接续接最大方位残差[deg]
cfg.joint_2d_max_el_residual_deg = 5;     % 直接续接最大俯仰残差[deg]
cfg.joint_2d_max_los_residual_deg = 15;   % 直接续接最大视线夹角[deg]

cfg.joint_tentative_timeout_s = 3;       % 二维暂态静默超时[s]
cfg.joint_confirmed_timeout_s = 12;      % 二维确认航迹静默超时[s]
cfg.joint_2d_output_max_silence_s = 0.5; % 二维正式输出最大静默[s]，不删除内部航迹
cfg.joint_2d_id_offset = 1000000;        % 二维输出ID命名空间偏移
cfg.joint_max_predict_dt_s = 5;          % 二维单次预测最大间隔[s]

cfg.joint_projection_cache_enabled = true; % 缓存事件内角度投影
cfg.joint_progress_interval_events = 100; % 进度日志事件间隔

% 9.2 二维角度IMM
cfg.joint_angle_q_cv = 0.02;             % 角度CV过程噪声强度
cfg.joint_angle_q_ca = 0.12;             % 角度CA过程噪声强度
cfg.joint_angle_rate_birth_std_dps = 0.75; % 新生角速度标准差[deg/s]
cfg.joint_angle_acc_birth_std_dps2 = 1.0;  % 新生角加速度标准差[deg/s^2]
cfg.joint_imm_cv_stay = 0.96;            % 统一滤波器CV保持概率
cfg.joint_imm_ca_stay = 0.94;            % 统一滤波器CA保持概率
cfg.joint_imm_cv_probability = 0.65;      % 新生CV初始概率

% 9.3 二维重复抑制及与三维航迹的一致性判定
cfg.joint_merge_angle_deg = 0.08;        % 二维候选航迹合并角度门[deg]
cfg.joint_merge_rate_dps = 1.0;          % 二维候选航迹合并角速度门[deg/s]
cfg.joint_merge_nis = 13.2767;           % 二维候选航迹合并NIS门

cfg.joint_2d3d_fusion_angle_deg = 0.30;  % 二维航迹被三维解释的角度门[deg]
cfg.joint_2d3d_fusion_rate_dps = 1.5;    % 二维/三维角速度一致门[deg/s]
cfg.joint_2d3d_fusion_min_cycles = 3;    % 二维被三维吸收的连续一致周期
cfg.joint_dimension_switch_enabled = true; % 启用同ID二维/三维输出切换
cfg.joint_2d3d_upgrade_match_cycles = 2; % 二维升三维所需连续异维匹配周期
cfg.joint_external_rebind_enabled = true; % 三维重建后继承原二维影子逻辑ID
cfg.joint_external_rebind_ambiguity_nis = 1.0; % 重绑定最优/次优NIS差门
cfg.joint_external_rebind_max_gap_s = 2.0; % 重绑定连续主动命中最大间隔[s]

%% 10. [联合] 跨维状态与质量切换
% 成熟三维状态仍由第7节滤波器产生。本节控制伴随角度分支、2/3主动证据
% 升维、三维质量降维和影子状态恢复；通用独立滤波路径也复用这些参数。
cfg.joint_gate_3d = 11.3449;             % 统一滤波器三维关联卡方门
cfg.joint_space_sigma_a_cv = 12;         % 空间CV加速度噪声[m/s^2]
cfg.joint_space_sigma_j_ca = 15;         % 空间CA加加速度噪声[m/s^3]
cfg.joint_space_vel_birth_std_mps = 500; % 空间新生速度标准差[m/s]
cfg.joint_space_acc_birth_std_mps2 = 80; % 空间新生加速度标准差[m/s^2]

cfg.joint_angle95_max_deg = 1.0;         % 二维模式角度95%误差上限[deg]
cfg.joint_3d_birth_M = 3;                % 通用空间分支三维出生M
cfg.joint_3d_birth_N = 5;                % 通用空间分支三维出生N
cfg.joint_3d_upgrade_M = 2;              % 二维升三维主动命中数M
cfg.joint_3d_upgrade_N = 3;              % 二维升三维主动机会窗N
cfg.joint_up_consecutive = 1;            % 连续升级判定次数
cfg.joint_down_consecutive = 3;          % 连续降级判定次数
cfg.joint_quality_eval_interval_s = 0.10; % 无主动扫描时质量评估最小间隔[s]
cfg.joint_switch_gate_2d = 9.2103;       % 模式切换角度一致门
cfg.joint_switch_cov_inflate = 2.0;      % 模式切换协方差膨胀

cfg.joint_pos95_warn_m = 15000;          % 位置95%不确定度预警门[m]
cfg.joint_pos95_down_m = 30000;          % 位置95%不确定度降级门[m]
cfg.joint_pos95_recover_m = 10000;       % 位置95%不确定度恢复门[m]
cfg.joint_radial95_warn_m = 10000;       % 径向95%不确定度预警门[m]
cfg.joint_radial95_down_m = 20000;       % 径向95%不确定度降级门[m]
cfg.joint_radial95_recover_m = 7000;     % 径向95%不确定度恢复门[m]
cfg.joint_relative_range_down = 0.60;    % 相对距离不确定度降级门

cfg.joint_space_nis_window = 5;          % 空间NIS滑窗长度
cfg.joint_space_nis_recover = 1.5;       % 空间NIS恢复门
cfg.joint_space_nis_warn = 2.5;          % 空间NIS预警门
cfg.joint_space_nis_down = 4.0;          % 空间NIS降级门
cfg.joint_shadow_max_s = 20;             % 降维后三维影子状态最长保留时间[s]
cfg.joint_mode_3d_prior_cost = 0.5;       % 通用空间分支三维关联优先代价

%% 11. [评价] 二维、三维与总体指标
% joint_2d3d 使用同一评价骨架分别统计二维、三维和总体结果：
%   共用：量测关联率、编号一致率、航迹级正确率、起始延迟、角度RMSE。
%   二维：仅方位/俯仰/LOS角度误差，不计算ENU或三维位置RMSE。
%   三维：除上述共用项外，计算E/N/U及三维位置RMSE。
%   总体：汇总逻辑航迹数量与角度指标；位置指标只来自三维输出。
% 正式二维评价只统计保持为纯角度分支的被动参考目标；跨维目标由被动量测
% 覆盖指标统计进入二维、进入三维及总关联量，避免维度切换重复扣分。
cfg.metrics_enabled = true;              % 运行结束后计算指标
cfg.metrics_max_print = 102;              % 指标报告开关/上限；0=不打印
cfg.metrics_progress_enabled = true;      % 打印评价阶段计时和匹配矩阵规模
cfg.track_accuracy_purity_th = 0.99;      % 整轨覆盖纯度门：正确关联点/该真值全部量测
cfg.track_accuracy_min_assoc = 3;        % 航迹级评价最少带标签关联点

% 真实仿真真值与量测标签伪真值分别评价。空路径只触发自动发现，
% 不会把量测标签伪真值伪装成真实物理真值。
cfg.truth_file = '';
cfg.truth_auto_discover = true;
cfg.truth_altitude_is_absolute = true;
cfg.truth_real_time_tolerance_s = 0.03;
cfg.truth_time_origin_s = [];

% 默认流程中真值编号只用于评价/绘图；不参与分片去重和滤波关联。
cfg.truth_id_split_enabled = true;       % 仅对主动真值按空间/时间断裂拆分
cfg.truth_id_split_dist_m = 8000;       % 主动同编号真值实例距离断裂门[m]
cfg.truth_id_split_max_gap_s = 10;       % 主动同编号真值实例时间断裂门[s]
cfg.truth_assoc_map_max_dist_m = inf;    % 关联量测映射真值实例的最大距离[m]

cfg.truth_cross_sensor_id_consistent = false; % 主被动真值编号是否同义
cfg.truth_cross_sensor_match_angle_deg = 1.0; % 主被动真值AE几何匹配门[deg]
cfg.truth_cross_sensor_match_max_dt_s = 0.5;  % 几何匹配最近时刻门[s]
cfg.truth_cross_sensor_match_min_points = 3;  % 几何匹配最少重叠点
cfg.truth_cross_sensor_match_min_ratio = 0.20; % 几何匹配最少时间重叠比例
cfg.truth_cross_sensor_match_ambiguity_deg = 0.10; % 最优/次优代价差门[deg]
cfg.truth_cross_sensor_match_max_samples = 200; % 每个被动实例最多抽样点数

% 下列RTS参数只用于 legacy_active3d 的旧三维伪真值评价。
cfg.truth_rts_meas_std_m = 1000;         % 旧三维真值RTS量测标准差[m]
cfg.truth_rts_proc_std_mps2 = 10;        % 旧三维真值RTS过程噪声[m/s^2]
cfg.truth_rms_time_tolerance_s = cfg.frame_time_window_s; % 旧三维真值时间匹配窗[s]

%% 12. 绘图、结果保存与离线平滑
cfg.do_plot = true;                      % 自动绘制二维/三维总体结果
cfg.joint_plot_min_life = 3;             % 总体图最少输出点数
cfg.plot_save_dir = 'track_reports';                  % 图片目录；空值=不自动保存
cfg.result_save_file = fullfile('track_reports', ...
    'fusion_result.mat');                              % MAT结果文件；空值=不自动保存
cfg.result_save_mode = 'summary';         % summary=指标/统计；full=完整历史（可能数GB）

cfg.do_smoothing = false;                % 启用离线航迹平滑
cfg.smooth_method = 'fixedlag';          % 平滑方法
cfg.smooth_lag = 8;                      % 固定滞后步数
cfg.smooth_min_life = 10;                % 执行平滑的最短航迹长度
cfg.smooth_meas_std = 30;                % 平滑量测标准差[m]
cfg.smooth_proc_std = 3;                 % 平滑过程噪声[m/s^2]
cfg.smooth_show_axes = false;            % 绘制平滑分轴对比
end
