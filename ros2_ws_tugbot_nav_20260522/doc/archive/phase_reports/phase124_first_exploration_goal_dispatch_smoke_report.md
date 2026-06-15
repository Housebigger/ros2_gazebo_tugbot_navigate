# Phase124 First exploration goal dispatch smoke report

Status: PHASE124_FIRST_EXPLORATION_GOAL_DISPATCH_SMOKE_COMPLETE_STOP_BEFORE_PHASE125

## Summary

Phase124 ran a bounded visible-stack first exploration goal dispatch smoke.

The run reused the Phase120 ingress chain and the Phase122 handoff gate. After handoff was ready, `maze_explorer` was started with `max_goals=1`. Exactly one current-algorithm first exploration goal was dispatched with `goal_kind=explore`, Nav2 acceptance was observed, and the runner stopped at the first-goal smoke boundary.

Final classification:

```text
FIRST_EXPLORATION_GOAL_DISPATCHED_ACCEPTED_STOP
```

This is first-goal smoke success only. It is not autonomous exploration success and not exit success. It does not validate full-maze exploration, Goal1 carry-over, staging, fallback, terminal acceptance, or exit reaching.

Phase125 not entered.

## Scope and guardrails

Allowed in Phase124:

- visible Gazebo/RViz/SLAM/Nav2 runtime;
- explicit inner-ingress goal only: `frame_id=map`, `x=2.0`, `y=0.0`, `yaw=0.0`;
- Phase122-style handoff readiness gate;
- `maze_explorer` startup with `max_goals=1` after handoff readiness;
- exactly one current-algorithm first `goal_kind=explore` dispatch;
- immediate stop after first accepted/rejected/timeout/no-valid-goal terminal boundary.

Forbidden and preserved:

- no manually specified Goal1;
- no carry-over/staging/branch/centerline/fallback/terminal/exit goal was sent by the runner;
- no fallback/terminal/exit goal was counted as first-goal success;
- no Nav2/MPPI/controller/config tuning;
- no exploration strategy change;
- no branch scoring change;
- no centerline gate change;
- no fallback change;
- no terminal acceptance change;
- no autonomous exploration success claim;
- no exit success claim.

## Implementation and tests

Added Phase124 focused test/runner/analyzer scaffold:

- `src/tugbot_maze/test/test_phase124_first_exploration_goal_dispatch_smoke.py`
- `tools/run_phase124_first_exploration_goal_dispatch_smoke.py`
- `tools/analyze_phase124_first_exploration_goal_dispatch_smoke.py`

A Phase123 static test was also narrowed so Phase123 remains doc-only for Phase123 files while allowing Phase124 runtime files after Phase123 completion:

- `src/tugbot_maze/test/test_phase123_first_exploration_goal_dispatch_design.py`

TDD evidence:

- RED: Phase124 focused tests initially failed because runner/analyzer/report did not exist.
- GREEN: focused/static bundle passed after implementation.

Final static verification:

```text
python3 -m py_compile tools/run_phase124_first_exploration_goal_dispatch_smoke.py tools/analyze_phase124_first_exploration_goal_dispatch_smoke.py
PYTHONDONTWRITEBYTECODE=1 pytest -q \
  src/tugbot_maze/test/test_phase121_post_ingress_handoff_design.py \
  src/tugbot_maze/test/test_phase122_post_ingress_handoff_dry_start.py \
  src/tugbot_maze/test/test_phase123_first_exploration_goal_dispatch_design.py \
  src/tugbot_maze/test/test_phase124_first_exploration_goal_dispatch_smoke.py
```

Result:

```text
30 passed in 0.07s
```

## Runtime artifacts

Final rerun artifact paths:

- artifact: `log/phase124_first_exploration_goal_dispatch_smoke/phase124_first_exploration_goal_dispatch_smoke_rerun_artifact.json`
- analysis: `log/phase124_first_exploration_goal_dispatch_smoke/phase124_first_exploration_goal_dispatch_smoke_rerun_analysis.json`
- summary: `log/phase124_first_exploration_goal_dispatch_smoke/phase124_first_exploration_goal_dispatch_smoke_rerun_summary.md`
- runner stdout: `log/phase124_first_exploration_goal_dispatch_smoke/phase124_first_exploration_goal_dispatch_smoke_rerun_runner_stdout.json`
- analyzer stdout: `log/phase124_first_exploration_goal_dispatch_smoke/phase124_first_exploration_goal_dispatch_smoke_rerun_analyzer_stdout.json`
- visible stack stdout: `log/phase124_first_exploration_goal_dispatch_smoke/phase124_first_exploration_goal_dispatch_smoke_rerun_visible_stack_stdout.log`
- visible stack stderr: `log/phase124_first_exploration_goal_dispatch_smoke/phase124_first_exploration_goal_dispatch_smoke_rerun_visible_stack_stderr.log`
- maze_explorer stdout: `log/phase124_first_exploration_goal_dispatch_smoke/phase124_first_exploration_goal_dispatch_smoke_rerun_artifact_maze_explorer_stdout.log`
- maze_explorer stderr: `log/phase124_first_exploration_goal_dispatch_smoke/phase124_first_exploration_goal_dispatch_smoke_rerun_artifact_maze_explorer_stderr.log`

Analyzer result:

```text
classification: FIRST_EXPLORATION_GOAL_DISPATCHED_ACCEPTED_STOP
valid: true
classification_matches_evidence: true
```

## Phase120 ingress and Phase122 handoff evidence

Ingress result:

```text
accepted: true
result_received: true
result_status_label: SUCCEEDED
locked_explicit_inner_ingress_goal: true
```

Handoff readiness:

```text
handoff_allowed: true
same_run_readiness_wait_passed: true
preflight_passed: true
inner_ingress_result_succeeded: true
robot_pose_near_ingress_goal: true
nav2_action_idle: true
costmap_fresh: true
scan_fresh: true
tf_fresh: true
```

Pose/handoff details:

```text
robot_pose_after_ingress: x=1.8538556568923468, y=0.02183342995476324, yaw=-0.006870047456431466, frame_id=map
distance_to_ingress_goal: 0.1477662603098455 m <= 0.35 m
orientation_error: 0.006870047456431466 rad <= 0.35 rad
```

Freshness/idle evidence:

```text
global_costmap_available: true
global_costmap_age_sec: 0.5994284152984619
local_costmap_available: true
local_costmap_age_sec: 0.018615007400512695
scan_seen: true
scan_age_sec: 0.059598684310913086
tf_fresh: true
map_base_age_sec: 0.003
odom_base_age_sec: 0.003
map_odom_age_sec: 0.0
nav2_action_idle: true
active_goal_count: 0
pending_goal_count: 0
executing_goal_count: 0
canceling_goal_count: 0
```

## First-goal evidence

Top-level first-goal outcome:

```text
maze_explorer_started: true
maze_explorer_max_goals: 1
goal_event_count: 1
dispatch_event_count: 1
goal_kind: explore
accepted: true
rejected: false
timeout: false
result_status_label: ACCEPTED
abort_text: null
stop_reason: first_explore_goal_accepted_stop
```

Candidate and pose:

```text
candidate_source: maze_explorer_current_algorithm
candidate_family: junction
candidate_id: 1
candidate_rank: 1
candidate_count: 4
current_node_id: 2
start_node_id: 2
near_exit: false
raw_target: [2.0857203439562952, 1.0202640827301475]
refined_target: [2.0857203439562952, 1.0202640827301475]
original_target: [2.0857203439562952, 1.0202640827301475]
pose.frame_id: map
pose.x: 2.0857203439562952
pose.y: 1.0202640827301475
pose.yaw: 1.5639262793384652
dispatch_wall_time_sec: 1780544309.2327554
selection_reason: topology_exit_bias_score
```

Frontier/topology evidence summary:

```text
last_open_direction_count: 4
last_candidate_count: 4
candidate_branch_count: 4
chosen_branch_rank: 1
chosen_branch_score: -2.3462284192413114
chosen_exit_progress_delta_m: 0.9170132404750295
chosen_target_exit_dist: 2.7538737995178915
chosen_target_local_cost: 0
chosen_dispatch_path_local_cost_max: 0
local_topology: junction
blocked_branch_count: 0
blacklisted_goal_count: 0
topology_consistency_guard_status: idle
single_open_exception_applied: false
```

Sensor/runtime evidence captured around first dispatch:

```text
local_costmap_sample_count: 3
nav2_feedback_count: 0
scan_seen: true
map_seen: true
odom_seen: true
tf_fresh: true
```

`nav2_feedback_count=0` is expected for this bounded stop because the runner stops as soon as action acceptance is observed; it does not wait for path execution or result completion.

## Shutdown note

The runner intentionally terminated the `maze_explorer` subprocess after first-goal acceptance to enforce the Phase124 stop boundary. The subprocess stderr includes an rclpy context shutdown traceback caused by bounded external termination after the dispatch/acceptance evidence was already captured. This is not counted as an exploration failure, timeout, or autonomous success. The accepted first-goal evidence is from `/maze/goal_events` and `/navigate_to_pose/_action/status`; no second dispatch occurred.

## Cleanup and guards

Pre-runtime cleanup:

- artifact: `log/phase124_first_exploration_goal_dispatch_smoke/phase124_first_exploration_goal_dispatch_smoke_cleanup_before_runtime.txt`
- matched_before: empty
- matched_after: empty

Final cleanup:

- artifact: `log/phase124_first_exploration_goal_dispatch_smoke/phase124_first_exploration_goal_dispatch_smoke_final_cleanup.txt`
- stopped remaining visible-stack process group, including one residual `gz sim` process
- final matched_after: empty

Final process guard:

- artifact: `log/phase124_first_exploration_goal_dispatch_smoke/phase124_first_exploration_goal_dispatch_smoke_final_process_guard_after_cleanup.txt`
- matching runtime processes: none

Nav2 config diff guard:

- artifact: `log/phase124_first_exploration_goal_dispatch_smoke/phase124_first_exploration_goal_dispatch_smoke_final_nav2_config_diff.txt`
- `src/tugbot_navigation/config` diff: empty

Scoped pycache cleanup:

- artifact: `log/phase124_first_exploration_goal_dispatch_smoke/phase124_first_exploration_goal_dispatch_smoke_final_pycache_after_cleanup.txt`
- remaining pycache under `tools` and `src/tugbot_maze/test`: empty

## Final conclusion

Phase124 completed the first exploration goal dispatch smoke and stopped before Phase125.

Accepted claim:

```text
Phase120 ingress succeeded + Phase122 handoff ready + maze_explorer max_goals=1 started + exactly one current-algorithm goal_kind=explore dispatch was accepted by Nav2, then stopped.
```

Not claimed:

```text
autonomous exploration success
exit success
full-maze exploration success
Goal1/carry-over/staging success
fallback/terminal/exit success
```
