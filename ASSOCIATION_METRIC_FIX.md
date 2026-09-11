# Association metric fix (v3)

This branch fixes the abnormally low `量测关联率(全部/曾确认)` / ever-confirmed
association statistic without changing the tracker, CKF, association gate, or
track-management behavior.

## Root cause

The previous `collect_confirmed_ids` logic in `evaluate_joint_tracking_metrics.m`
used `transition_log` as the *exclusive* source whenever any
`logical_confirm_*` transition existed. In the current architecture,
`run_filter_joint_legacy_backbone` combines a mature 3-D tracker with the online
2-D/dimension manager. The manager transition log does not enumerate every
confirmed mature 3-D logical ID, so many legitimate associations were omitted
from the "ever confirmed" numerator.

This explains the contradiction seen in the supplied run log: nearly all active
RAE measurements were associated and the 3-D tracks had near-complete formal
coverage, while the old ever-confirmed association rate was only about 2%.

## Fix

The corrected evaluator builds the confirmed logical-ID set as a union of:

1. `logical_confirm_*` transition records;
2. formal confirmed `est.output` IDs;
3. confirmed `est.logical_tracks` snapshots;
4. IDs present in confirmed-output label matrices `est.L` and `est.L2`.

The original evaluator is preserved unchanged at:

`+metriclegacy/evaluate_joint_tracking_metrics.m`

The root-level `evaluate_joint_tracking_metrics.m` is a compatibility wrapper,
so existing calls in `run_fusion_main.m` do not need to change.

The report also adds:

`已关联量测中归属曾确认逻辑航迹`

which uses `n_assigned_confirmed / n_assigned` and is often easier to interpret
than dividing by all input measurements.

## Regression test

Run in MATLAB:

```matlab
result = test_joint_confirmed_association_fix();
```

The fixture intentionally provides a partial confirmation transition log while
associations belong to another formally confirmed track. The corrected
three-dimensional and overall ever-confirmed rates must both be 100%.

For the full project regression suite, also run the existing tests such as:

```matlab
test_joint_metric_contract();
test_joint_legacy_backbone_regression();
```

## Scope

This change is evaluation-only. It does not alter filter estimates, association
decisions, births, deletions, 2-D/3-D switching, or measurement disposition.
