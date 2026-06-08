# Phase107 preflight vs raw-capture discrepancy diagnosis report

Status: COMPLETE_DIAGNOSIS_ONLY_STOP_BEFORE_PHASE108

## Goal

Diagnose the Phase106 discrepancy where the Phase105 preflight rejected before ingress with `ingress_preflight_timeout` / `ingress_map_base_tf_missing`, while the later Phase106 raw capture showed `/scan`, `/map`, `/local_costmap/costmap`, `/odom`, and TF evidence available.

This phase is diagnosis-only. It does not change Phase105 preflight logic, Phase101 carry-over logic, Phase88/92 logic, `maze_explorer` strategy, branch scoring, exploration order, or Nav2/controller parameters.

## Required reading completed

- `doc/doc_report/phase106_preflighted_carry_over_bounded_goal1_staging_validation_report.md`
- `doc/doc_report/phase105_inner_ingress_tf_controller_preflight_hardening_minimal_implementation_report.md`
- `doc/doc_report/phase104_inner_ingress_tf_controller_preflight_hardening_design_review_report.md`
- `doc/doc_report/phase103_inner_ingress_goal_failure_diagnosis_report.md`
- `log/phase106_preflighted_carry_over_bounded_goal1_staging_validation/*`

## Added Phase107 files

- `tools/analyze_phase107_preflight_raw_capture_discrepancy_diagnosis.py`
- `src/tugbot_maze/test/test_phase107_preflight_raw_capture_discrepancy_diagnosis.py`
- `log/phase107_preflight_raw_capture_discrepancy_diagnosis/phase107_preflight_raw_capture_discrepancy_diagnosis_analysis.json`
- `log/phase107_preflight_raw_capture_discrepancy_diagnosis/phase107_preflight_raw_capture_discrepancy_diagnosis_minimal_summary.md`
- `doc/doc_report/phase107_preflight_raw_capture_discrepancy_diagnosis_report.md`

## Classification set

The Phase107 analyzer emits exactly one of:

- `PREFLIGHT_FRAME_QUERY_MISMATCH`
- `PREFLIGHT_LIFECYCLE_READY_ORDERING_ISSUE`
- `PREFLIGHT_RAW_CAPTURE_DISCREPANCY_INSUFFICIENT_EVIDENCE`
- `PREFLIGHT_SAMPLING_TOO_EARLY`
- `PREFLIGHT_STABILITY_WINDOW_TOO_SHORT`
- `PREFLIGHT_TOOL_FALSE_NEGATIVE`
- `PREFLIGHT_TOPIC_QUERY_MISMATCH`
- `RAW_CAPTURE_LATER_THAN_PREFLIGHT`

## Analyzer command

```bash
PYTHONDONTWRITEBYTECODE=1 python3 tools/analyze_phase107_preflight_raw_capture_discrepancy_diagnosis.py \
  --phase106-artifact-dir log/phase106_preflighted_carry_over_bounded_goal1_staging_validation \
  --output-json log/phase107_preflight_raw_capture_discrepancy_diagnosis/phase107_preflight_raw_capture_discrepancy_diagnosis_analysis.json \
  --minimal-summary-output log/phase107_preflight_raw_capture_discrepancy_diagnosis/phase107_preflight_raw_capture_discrepancy_diagnosis_minimal_summary.md
```

Observed stdout:

```json
{"classification": "PREFLIGHT_TOOL_FALSE_NEGATIVE", "contributing_classifications": ["PREFLIGHT_TOOL_FALSE_NEGATIVE", "RAW_CAPTURE_LATER_THAN_PREFLIGHT"], "evidence_gaps": ["missing_runtime_timeline_samples"]}
```

## Final classification

Primary classification: `PREFLIGHT_TOOL_FALSE_NEGATIVE`

Contributing classifications: `PREFLIGHT_TOOL_FALSE_NEGATIVE`, `RAW_CAPTURE_LATER_THAN_PREFLIGHT`

Evidence gaps: `missing_runtime_timeline_samples`

Interpretation: The best-supported discrepancy diagnosis is a Phase105 preflight false negative, with an important timing caveat that raw capture was taken later than the preflight timeout window. Phase106 does not prove that all raw-capture evidence existed inside the preflight window; it does show that the preflight reported missing TF/controller/scan evidence even though lifecycle logs indicate managed Nav2 nodes were already active before the preflight began.

## Phase106 preflight evidence reviewed

- preflight passed: `False`
- reject reason: `ingress_preflight_timeout`
- last specific reject reason: `ingress_map_base_tf_missing`
- ingress goal sent: `False`
- maze_explorer started: `False`
- failed gates: `['ingress_map_base_tf_missing', 'ingress_map_odom_tf_stale', 'ingress_odom_base_tf_stale', 'ingress_scan_transform_unstable', 'ingress_controller_robot_pose_unavailable', 'ingress_goal_pose_transform_unavailable', 'ingress_preflight_timeout']`
- preflight sample count: `3`
- preflight sample wall times: `[1780476322.6121438, 1780476332.4260972, 1780476342.253443]`

The analyzer preserved the Phase106 fail-closed result: no ingress goal was sent and `maze_explorer` was not started.

## Timestamp comparison

- preflight start wall time: `1780476322.6121438`
- preflight end wall time: `1780476342.253443`
- preflight timeout deadline wall time: `1780476342.6121438`
- raw capture start wall time: `1780476343.7920732`
- raw capture end wall time: `1780476348.8133264`
- raw capture started after preflight end: `True`
- raw capture started after preflight timeout deadline: `True`
- seconds between preflight end and raw capture start: `1.5386302471160889`
- seconds between preflight timeout deadline and raw capture start: `1.179929494857788`

Direct answer: raw capture availability occurred after the preflight timeout window in the Phase106 artifact timeline. Therefore raw capture availability is later context, not proof that the same evidence was available during preflight sampling.

## Topic comparison

- preflight scan topic: `/scan`
- raw scan topic: `/scan`
- scan topic same: `True`
- raw map topic: `/map`
- raw local costmap topic: `/local_costmap/costmap`
- raw odom topic: `/odom`

Direct answer: no scan-topic mismatch was found. Both preflight source/runtime contract and raw capture use `/scan`.

## Frame comparison

- preflight scan target frame: `map`
- preflight scan frame id: `None`
- raw scan frame id: `tugbot/scan_omni/scan_omni`
- scan frame same: `None`
- preflight goal frame id: `map`
- preflight global costmap frame: `map`
- preflight controller frame: `odom`
- raw map frame id: `map`
- raw local costmap frame id: `odom`
- raw odom frame id: `odom`
- raw odom child frame id: `base_link`
- raw core TF available: `{'map->base_link': True, 'map->odom': True, 'odom->base_link': True}`
- raw scan static TF errors: `{'base_link->base_scan': '"base_scan" passed to lookupTransform argument source_frame does not exist. ', 'base_link->scan_omni': '"scan_omni" passed to lookupTransform argument source_frame does not exist. '}`

Direct answer: no definite frame mismatch was proven for the core preflight checks (`map`, `odom`, `base_link`). The scan frame comparison is inconclusive because preflight never received a scan sample (`preflight_scan_frame_id=None`), while raw capture later saw `tugbot/scan_omni/scan_omni`. The analyzer therefore did not classify this as `PREFLIGHT_FRAME_QUERY_MISMATCH`.

## Lifecycle / startup ordering comparison

- preflight controller_server_active: `False`
- preflight bt_navigator_active: `False`
- preflight NavigateToPose action ready: `True`
- preflight robot pose available: `False`
- ros_graph_ready marker before preflight: `True`
- ros_graph_ready first epoch: `1780476298.8360229`
- managed nodes active time: `1780476297.266283`
- controller server bond time: `1780476295.1854546`
- bt_navigator bond time: `1780476297.0299888`
- managed nodes active before preflight start: `True`
- managed nodes active during preflight: `False`
- managed nodes active after preflight end: `False`
- final ROS graph had controller and bt nodes: `True`

Direct answer: `bt_navigator_active=false` and `controller_server_active=false` in the preflight artifact are not best explained as startup ordering in this run. The launch log reports managed Nav2 nodes active at `1780476297.266283`, before preflight start `1780476322.6121438`. This supports `PREFLIGHT_TOOL_FALSE_NEGATIVE` rather than `PREFLIGHT_LIFECYCLE_READY_ORDERING_ISSUE`.

## Query method comparison

- preflight TF method: `tf2 Buffer.lookup_transform(parent, child, Time()) plus can_transform for scan/goal frames`
- preflight scan method: `subscribe /scan, then require scan freshness and can_transform(map, scan_frame)`
- preflight lifecycle method: `ros2 lifecycle get /controller_server and /bt_navigator subprocess checks plus ros2 action list`
- raw capture method: `read-only subscriptions for /scan,/map,/local_costmap/costmap,/odom plus tf2 Buffer.lookup_transform snapshot after Phase106 analyzer`
- query method same: `False`

The methods are intentionally different: preflight makes pass/fail gated live queries inside a bounded window, while raw capture records a later read-only snapshot. This difference explains why raw capture cannot by itself overturn preflight timing, but it does not explain the lifecycle false-negative evidence because lifecycle activation was logged before preflight start.

## Raw capture summary

- raw capture present: `True`
- scan available: `True`
- map available: `True`
- local_costmap available: `True`
- odom available: `True`
- TF available: `True`

## Direct answers

- raw_capture available after preflight timeout? `True`
- preflight used different frame/topic than raw_capture? `False`
- bt_navigator/controller_server inactive was startup ordering? `False`
- bt_navigator/controller_server inactive is tool false-negative candidate? `True`

## Guardrails

- No Phase105 preflight changed.
- No Phase101 carry-over changed.
- No Phase88/92 logic changed.
- No maze_explorer strategy changed.
- No branch scoring changed.
- No exploration order changed.
- No centerline gate changed.
- No directional readiness/fallback/terminal acceptance changed.
- No Nav2/MPPI/controller/inflation/robot_radius/clearance_radius_m/map threshold tuning.
- No autonomous exploration success claimed.
- No exit success claimed.
- Phase108 not entered.

Analyzer guardrail fields:

```json
{
  "diagnosis_only": true,
  "maze_explorer_strategy_changed": false,
  "nav2_config_changed": false,
  "nav2_controller_tuning_changed": false,
  "no_exit_success_claimed": true,
  "no_success_claimed": true,
  "phase101_carry_over_changed": false,
  "phase105_preflight_changed": false,
  "phase108_entered": false,
  "phase88_92_logic_changed": false
}
```

## Verification

Focused tests, syntax check, and package build:

```bash
PYTHONDONTWRITEBYTECODE=1 pytest -q src/tugbot_maze/test/test_phase107_preflight_raw_capture_discrepancy_diagnosis.py && \
PYTHONDONTWRITEBYTECODE=1 python3 -m py_compile \
  tools/analyze_phase107_preflight_raw_capture_discrepancy_diagnosis.py \
  src/tugbot_maze/test/test_phase107_preflight_raw_capture_discrepancy_diagnosis.py && \
source /opt/ros/jazzy/setup.bash && \
colcon build --packages-select tugbot_maze --symlink-install
```

Observed result:

```text
......                                                                   [100%]
6 passed in 0.04s
Starting >>> tugbot_maze
Finished <<< tugbot_maze [0.85s]

Summary: 1 package finished [0.96s]
```

Nav2 config/process/pycache guard:

```bash
git diff -- src/tugbot_navigation/config && \
ps -eo pid,cmd | grep -E 'phase107_preflight|phase106_preflighted|record_explorer_state_series|record_phase106|maze_explorer|ros2 launch tugbot_bringup tugbot_maze_slam_nav|gz sim|rviz2|slam_toolbox|controller_server|planner_server|bt_navigator|ros_gz_bridge|parameter_bridge' | grep -v grep || true && \
find . -name __pycache__ -type d
```

Observed result after removing test-created `__pycache__` directories: empty output. This means no Nav2 config diff, no matching project runtime residues, and no remaining `__pycache__` directories were found.

## Conclusion

Phase107 diagnoses the Phase106 discrepancy as `PREFLIGHT_TOOL_FALSE_NEGATIVE`, with `RAW_CAPTURE_LATER_THAN_PREFLIGHT` as a contributing classification.

The raw capture did occur after preflight timeout, so it is later context. However, lifecycle evidence from the visible launch log shows managed Nav2 nodes were active before the preflight began, while the preflight artifact still recorded `controller_server_active=false` and `bt_navigator_active=false`. No definite topic mismatch was found; no core TF frame mismatch was found; scan-frame comparison remains inconclusive because preflight did not receive a scan sample. This phase makes no runtime logic changes and stops before Phase108.
