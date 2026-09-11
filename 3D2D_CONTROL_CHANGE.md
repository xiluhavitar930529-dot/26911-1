# 3D -> 2D 降级控制修改

本分支基于 `fix/confirmed-association-metric-v2`。

## 新的直接降级指标

三维正式输出的质量降级只由两类指标直接控制：

1. **距离95%不确定度**：`radial95_m = 1.96 * radial_sigma_m`
2. **三维主动量测缺失时间**：`active3d_missing_s = current_time - last_active_t`

其中 `last_active_t` 仅代表最近一次**带距离主动三维更新**，主动 AE-only 角度更新不再刷新这个时间戳。

## 默认阈值

```matlab
cfg.joint_radial95_warn_m = 10000;
cfg.joint_radial95_down_m = 20000;
cfg.joint_radial95_recover_m = 7000;

cfg.joint_active3d_missing_warn_s = 0.20;
cfg.joint_active3d_missing_down_s = 0.30;
cfg.joint_active3d_missing_recover_s = 0.15;
```

## 判据

```matlab
warn3d = radial95_m > radial95_warn_m || ...
         active3d_missing_s > active3d_missing_warn_s;

down3d = radial95_m > radial95_down_m || ...
         active3d_missing_s > active3d_missing_down_s;

recover3d = radial95_m <= radial95_recover_m && ...
            active3d_missing_s <= active3d_missing_recover_s;
```

原来的 `position95_m`、`relative_range_sigma`、`nis_norm` 继续计算并保留用于诊断，但不再直接触发 `warn3d/down3d/recover3d`。

## 保留的切换保护

- `joint_down_consecutive = 3`：连续三次 down 判据才进入正式降维决策；
- 二维分支必须 `valid2d`；
- 二维/三维角度投影必须通过 `switch_consistent`；
- 2D 尚未准备好时保持 `3d_warn`，不强制切换；
- 成熟三维主干 ID 真正消失时的 `external_3d_lost` 兜底逻辑保持不变。

## 运行

下载完整分支后直接运行：

```matlab
run_fusion_main_fixed
```

该启动脚本会依次应用评价统计修复和本次 3D->2D 控制修复，然后运行原 `run_fusion_main`。
