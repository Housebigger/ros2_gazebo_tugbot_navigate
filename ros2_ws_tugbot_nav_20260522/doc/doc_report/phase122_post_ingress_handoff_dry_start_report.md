# Phase122 Post-ingress handoff smoke with maze_explorer dry-start report

Status: PHASE122_POST_INGRESS_HANDOFF_DRY_START_COMPLETE_STOP_BEFORE_PHASE123

## Goal

Phase122 reused the Phase120 visible-stack ingress path to complete the managed-active readiness wait, strict preflight, and the single explicit inner-ingress goal. It then evaluated the Phase121 fail-closed handoff gate and dry-started `maze_explorer` with `max_goals=0` only after the gate passed.

This phase does not validate autonomous exploration, exit success, Goal1, carry-over, staging, branch scoring, centerline behavior, fallback, or terminal acceptance.

## Scope guardrails

Allowed:

- launch visible Gazebo/RViz/SLAM/Nav2;
- send the one explicit inner-ingress goal only: `frame_id=map`, `x=2.0`, `y=0.0`, `yaw=0.0`;
- after same-run handoff gate passes, dry-start `maze_explorer` with `maze_explorer_max_goals=0`;
- observe that `exploration_goal_dispatched=false`.

Forbidden / unchanged:

- no Goal1/carry-over/staging/branch/centerline/fallback/terminal/exit goal;
- no exploration goal dispatched by `maze_explorer`;
- no Nav2/MPPI/controller/config tuning;
- no exploration strategy change;
- no branch scoring change;
- no centerline/fallback/terminal acceptance change;
- no autonomous exploration success or exit success claim;
- Phase123 not entered.

## Delivered files

```text
tools/run_phase122_post_ingress_handoff_dry_start.py
tools/analyze_phase122_post_ingress_handoff_dry_start.py
src/tugbot_maze/test/test_phase122_post_ingress_handoff_dry_start.py
log/phase122_post_ingress_handoff_dry_start/
doc/doc_report/phase122_post_ingress_handoff_dry_start_report.md
```

## Focused/static verification

Command:

```bash
python3 -m py_compile tools/run_phase122_post_ingress_handoff_dry_start.py tools/analyze_phase122_post_ingress_handoff_dry_start.py
PYTHONDONTWRITEBYTECODE=1 pytest -q \
  src/tugbot_maze/test/test_phase120_controlled_ingress_dispatch_with_readiness_wait.py \
  src/tugbot_maze/test/test_phase121_post_ingress_handoff_design.py \
  src/tugbot_maze/test/test_phase122_post_ingress_handoff_dry_start.py
```

Result:

```text
19 passed in 0.11s
```

## Runtime artifacts

Final accepted runtime artifact set:

```text
log/phase122_post_ingress_handoff_dry_start/phase122_post_ingress_handoff_dry_start_rerun_phase120_artifact.json
log/phase122_post_ingress_handoff_dry_start/phase122_post_ingress_handoff_dry_start_corrected_final_artifact.json
log/phase122_post_ingress_handoff_dry_start/phase122_post_ingress_handoff_dry_start_corrected_final_analysis.json
log/phase122_post_ingress_handoff_dry_start/phase122_post_ingress_handoff_dry_start_corrected_final_summary.md
log/phase122_post_ingress_handoff_dry_start/phase122_post_ingress_handoff_dry_start_corrected_final_artifact_maze_explorer_stderr.log
log/phase122_post_ingress_handoff_dry_start/phase122_post_ingress_handoff_dry_start_corrected_final_artifact_maze_explorer_stdout.log
```

Earlier diagnostic artifacts are retained in the same log directory, including the first `COSTMAP_NOT_READY` run that revealed the need for a TRANSIENT_LOCAL QoS subscriber for `/global_costmap/costmap`, and the first dry-start termination attempt that showed SIGINT produced a noisy shutdown traceback. The accepted corrected final artifact uses the repaired QoS and controlled SIGTERM dry-start stop.

## Runtime classification

Analyzer output:

```text
classification: INGRESS_SUCCESS_HANDOFF_READY_DRY_START_STOP
valid: True
classification_matches_evidence: True
handoff_allowed: True
maze_explorer_start_allowed: True
maze_explorer_started: True
maze_explorer_max_goals: 0
exploration_goal_dispatched: False
```

## Handoff artifact fields

The Phase122 artifact records:

```text
ingress_goal_result
robot_pose_after_ingress
distance_to_ingress_goal
orientation_error
costmap_freshness
scan_freshness
tf_freshness
nav2_action_idle_state
handoff_allowed
maze_explorer_start_allowed
maze_explorer_started
maze_explorer_max_goals=0
exploration_goal_dispatched=false
```

Observed corrected final values:

```text
ingress_goal_result.result_status_label: SUCCEEDED
ingress_goal_result.locked_explicit_inner_ingress_goal: True
robot_pose_after_ingress: x=1.8486002482783355, y=0.024219712807181885, yaw=-0.0056824557496431335
distance_to_ingress_goal: 0.15332475113250318 m <= 0.35 m
orientation_error: 0.0056824557496431335 rad <= 0.35 rad
costmap_freshness: fresh=True, global_age=0.5027792453765869 s, local_age=0.17691326141357422 s
scan_freshness: fresh=True, scan_age=0.046717166900634766 s
tf_freshness: fresh=True, map_base_age=0.021 s, odom_base_age=0.021 s, map_odom_age=0.0 s
nav2_action_idle_state: idle=True, active_goal_count=0, pending_goal_count=0, executing_goal_count=0, canceling_goal_count=0
```

## Dry-start evidence

Corrected final dry-start evidence:

```text
maze_explorer_started: True
maze_explorer_max_goals: 0
dry_start_observed: True
dry_start_duration_sec: 5.000479459762573
goal_event_count: 0
dispatch_event_count: 0
exploration_goal_dispatched: False
latest explorer_state.goal_count: 0
latest explorer_state.goal_active: False
latest explorer_state.last_terminal_reason: goal budget reached
latest explorer_state.mode: FAILED_EXHAUSTED
```

`maze_explorer` stderr confirms it started with `max_goals=0`:

```text
maze_explorer started: ... max_goals=0
```

The dry-start process was intentionally terminated by the Phase122 runner after the bounded observation window. Return code `-15` is the controlled SIGTERM stop for the dry-start process, not an exploration failure. It is not counted as autonomous success, exit success, Goal1 success, or exploration success.

## Classifications

The analyzer supports:

```text
INGRESS_SUCCESS_HANDOFF_READY_DRY_START_STOP
INGRESS_SUCCESS_HANDOFF_NOT_READY
POSE_NOT_AT_INGRESS_GOAL
NAV2_ACTION_NOT_IDLE
TF_OR_SCAN_STALE
COSTMAP_NOT_READY
MAZE_EXPLORER_DRY_START_VIOLATION
```

Final classification:

```text
INGRESS_SUCCESS_HANDOFF_READY_DRY_START_STOP
```

## Guardrail conclusions

- Phase120 ingress chain succeeded in the same visible stack run.
- The only ingress goal was the locked explicit inner-ingress goal: `frame_id=map`, `x=2.0`, `y=0.0`, `yaw=0.0`.
- Handoff gate passed only after pose, Nav2 idle, costmap freshness, scan freshness, and TF freshness evidence were collected.
- `maze_explorer` was started only after `handoff_allowed=True`.
- `maze_explorer` was started with `maze_explorer_max_goals=0`.
- `exploration_goal_dispatched=false`.
- `goal_event_count=0` and `dispatch_event_count=0`.
- No Goal1/carry-over/staging/branch/centerline/fallback/terminal/exit goal was dispatched.
- Branch scoring was not changed.
- Nav2/MPPI/controller/inflation/robot radius/clearance/map threshold config was not tuned.
- No autonomous exploration success or exit success is claimed.
- Phase123 not entered.

## Cleanup

Runtime cleanup was performed after the corrected final run. Final process and config guards are recorded under:

```text
log/phase122_post_ingress_handoff_dry_start/phase122_final_cleanup.txt
log/phase122_post_ingress_handoff_dry_start/phase122_final_process_guard.txt
log/phase122_post_ingress_handoff_dry_start/phase122_final_nav2_config_diff.txt
```

Final cleanup result is expected to show no residual Gazebo/RViz/SLAM/Nav2/maze_explorer/ros2 launch processes and an empty `src/tugbot_navigation/config` diff guard.

## Stop condition

Phase122 is complete and stopped for human acceptance. Phase123 was not entered.
