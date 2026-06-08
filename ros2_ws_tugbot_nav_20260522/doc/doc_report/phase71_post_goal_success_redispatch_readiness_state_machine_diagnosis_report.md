# Phase71 Post-Goal Success / Re-dispatch Readiness State Machine Diagnosis

run_id: `phase71_post_goal_success_redispatch_readiness_state_machine_diagnosis`
source_artifact_dir: `/home/hyh/Desktop/agent_playground/playground_hermes/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260522/log/phase70_centerline_gate_relaxation_balance_first_runtime_validation`
analyzer_output: `/home/hyh/Desktop/agent_playground/playground_hermes/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260522/log/phase70_centerline_gate_relaxation_balance_first_runtime_validation/phase71_post_goal_success_redispatch_readiness_state_machine_diagnosis.json`

## Upstream acceptance context

Phase70 已人工验收，结论保持：`BALANCE_FIRST_GATE_NO_APPLY`。

Phase71 只复用 Phase70 artifacts 做状态机诊断，不继续改 centerline gate，不调 Nav2/MPPI/controller，不调 inflation/robot_radius/clearance_radius_m/map threshold，不改 branch scoring，不改 fallback/terminal acceptance。

不宣称 autonomous exploration success，不宣称 exit success。

## Objective

重建 Goal1 success 后状态机 timeline，回答：Goal1 成功后为什么没有进入下一轮 candidate formation / dispatch，是否被 readiness、local topology、costmap/scan/TF 时序、goal_count、entry state 或 exhausted 判据卡住。

## Added artifacts

- `tools/analyze_phase71_post_goal_success_redispatch_readiness_state_machine.py`
- `src/tugbot_maze/test/test_phase71_post_goal_success_redispatch_readiness_state_machine.py`
- `doc/doc_report/phase71_post_goal_success_redispatch_readiness_state_machine_diagnosis_report.md`
- Analyzer JSON output under Phase70 artifact dir:
  - `log/phase70_centerline_gate_relaxation_balance_first_runtime_validation/phase71_post_goal_success_redispatch_readiness_state_machine_diagnosis.json`

## Analyzer scope

The analyzer reads Phase70:

- `/maze/goal_events` JSONL per replay
- `/maze/explorer_state` JSONL per replay
- inner ingress action result
- preflight `MAX_GOALS`
- Nav2 config diff and cleanup proof

It reconstructs:

- goal outcome and event timing
- `goal_count`, `goal_success_count`, `last_completed_goal_sequence_id`
- mode transitions
- post-success dispatch-entry readiness gate status and blocking reasons
- candidate/open-direction counts
- local topology kind
- map/scan/local-costmap/global-costmap/TF sufficiency and age/stamp fields where available
- blacklist/visited/frontier/exhausted reasons
- whether a re-dispatch gate was actually reached after success

Allowed classifications:

- `POST_SUCCESS_REDISTPATCH_BLOCKED_BY_READINESS`
- `POST_SUCCESS_NO_CANDIDATE`
- `POST_SUCCESS_EXHAUSTED_PREMATURELY`
- `SUCCESS_EVENT_SEMANTICS_MISMATCH`
- `INSUFFICIENT_EVIDENCE`

## Phase70 replay evidence summarized by Phase71

### replay_01

Goal1:

- dispatch observed: `true`
- dispatch elapsed: `7.694s`
- outcome_event: `success`
- outcome elapsed: `10.047s`
- result_reason: `succeeded`
- result_status: `4`
- state counter confirmed success: `goal_success_count=1`, `last_completed_goal_sequence_id=1`

Post-success state:

- post-success state samples: `5`
- mode sequence after success: `SETTLING -> FAILED_EXHAUSTED`
- final_mode: `FAILED_EXHAUSTED`
- final_goal_count: `1`
- max_goals from Phase70 preflight: `1`
- last_terminal_reason: `goal budget reached`
- dispatch_after_success_count: `0`
- redispatch_gate_triggered: `false`
- blocking_stage: `goal_budget_exhausted_before_readiness_or_topology_resample`

Readiness/topology evidence carried in the final post-success state:

- dispatch readiness passed: `true`
- readiness blocking reasons: `[]`
- checks: map, scan, TF, local costmap, Nav2 lifecycle, navigate_to_pose action, goal_pose subscriber all `true`
- local topology kind: `junction`
- raw_open_direction_count: `4`
- filtered_open_direction_count: `4`
- candidate_after_filter_count: `4`
- candidate_count: `4`
- candidate rejection reason: `null`
- blacklist/blocked counts: `0/0`

Resource timing/sufficiency evidence available in readiness payload:

- map sufficient: `true`, known_ratio about `0.967`, free_ratio about `0.948`, map_stamp `35.001`
- scan sufficient: `true`, finite_count about `634`
- local_costmap sufficient: `true`, sample_age_sec about `0.579`, known_ratio `1.0`, free_ratio about `0.577`, map_stamp `35.421`
- TF sufficient: `true`

Interpretation: replay_01 的 success 事件不是孤立语义误报；explorer state 也确认了 Goal1 success。但因为 Phase70 wrapper/preflight 明确 `MAX_GOALS=1`，状态机在 success settle 后先命中 `goal_count >= max_goals` 的 exhausted 判据，直接 `FAILED_EXHAUSTED`，没有机会进入下一轮 dispatch-entry readiness / topology resample / candidate formation / dispatch。

### replay_02

Goal1:

- dispatch observed: `true`
- outcome_event: `timeout`
- result_reason: `goal_timeout`
- final_mode: `FAILED_EXHAUSTED`
- final_goal_count: `1`
- last_failure_reason: `goal_canceled_after_timeout`
- last_terminal_reason: `goal budget reached`

Local target risk note from Phase71 extraction:

- candidate local costmap cell state: `clear`
- candidate local max_radius_cost: `46`
- candidate map cell state: `free`, nearest_obstacle_distance_m about `0.702`

Interpretation: replay_02 支持用户指出的“目标点局部风险不高却 timeout”现象，但 Phase71 主问题是 post-success re-dispatch；replay_02 没有 Goal1 success 后重派发窗口，因此只作为对照 evidence，不在本阶段转向 Nav2/MPPI/controller 或 centerline gate 调整。

## Final classification

`POST_SUCCESS_EXHAUSTED_PREMATURELY`

Reason: Phase70 replay_01 中 Goal1 `outcome_event=success` 且 state counter 确认 success；之后没有 Goal2 dispatch，不是因为 readiness 未通过，也不是因为 local topology/candidate_count 为空，而是因为 `max_goals=1` 且 `goal_count=1`，状态机在 post-success settle 后优先触发 `goal budget reached` exhausted 判据，导致 `FAILED_EXHAUSTED`。

## Guardrail checks

- Nav2 config diff from Phase70 artifact: empty.
- Cleanup proof from Phase70 artifact: empty.
- No Nav2/MPPI/controller parameter change.
- No inflation/robot_radius/clearance_radius_m/map threshold change.
- No branch scoring change.
- No centerline gate runtime behavior change.
- No fallback/terminal acceptance change.
- No autonomous exploration success claim.
- No exit success claim.

## Verification

Commands run:

- `pytest -q src/tugbot_maze/test/test_phase71_post_goal_success_redispatch_readiness_state_machine.py` before analyzer/report existed -> expected RED: `6 failed`.
- `pytest -q src/tugbot_maze/test/test_phase71_post_goal_success_redispatch_readiness_state_machine.py -k 'not report_contract'` after analyzer implementation before report creation -> `5 passed, 1 deselected`.
- `python3 -m py_compile tools/analyze_phase71_post_goal_success_redispatch_readiness_state_machine.py src/tugbot_maze/test/test_phase71_post_goal_success_redispatch_readiness_state_machine.py` -> PASS.
- `pytest -q src/tugbot_maze/test/test_phase71_post_goal_success_redispatch_readiness_state_machine.py` -> `6 passed in 0.03s`.
- `colcon build --packages-select tugbot_maze tugbot_bringup --symlink-install` -> `2 packages finished [0.83s]`.
- `git diff -- src/tugbot_navigation/config | wc -c` -> `0`.
- cleanup process scan for ROS/Gazebo/Nav2/explorer/RViz processes -> empty.

Additional broad check:

- `colcon test --packages-select tugbot_maze --event-handlers console_direct+ && colcon test-result --verbose` was run after focused validation. It did not pass globally: `382 passed, 30 failed`. The failures are outside Phase71 focused scope and mostly reference missing/deprecated Phase29-33 assets/worlds such as `maze_wall_segments_20260522.yaml`, `tugbot_maze_world.sdf`, `maze_20260528_instance.yaml`, plus one pre-existing Phase26E branch-choice string-count contract. Phase71’s own test module passed in that run (`6 passed`). No Phase71 failure was introduced.

## Stop condition

Phase71 完成后停止，等待人工验收；不进入 Phase72。
