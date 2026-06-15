# Phase134 bounded corridor-alignment staging-contract smoke report

Status: PHASE134_BOUNDED_CORRIDOR_ALIGNMENT_STAGING_SMOKE_COMPLETE_STOP_BEFORE_PHASE135

## Scope

Phase134 executed a real visible-stack bounded corridor-alignment staging-contract smoke after Phase133.

Allowed in this phase:

- visible-stack Gazebo/RViz/SLAM/Nav2 startup;
- explicit inner-ingress goal: map,x=2.0,y=0.0,yaw=0.0;
- start `maze_explorer` only with a first-dispatch/staging budget guard;
- `max_goals=1`;
- observe and classify only the first literal dispatch;
- if first literal dispatch is `goal_kind=corridor_alignment_staging`, wait only for a bounded staging accepted/result/timeout outcome and stop.

Guardrails preserved:

- No second-step goal_kind=explore.
- No second exploration goal.
- No manual Goal1/carry-over/staging/branch/centerline/fallback/terminal/exit goal.
- No Nav2/MPPI/controller/goal checker/config tuning.
- No exploration strategy, branch scoring, centerline, fallback, or terminal acceptance change.
- No direct staging disablement.
- No autonomous exploration success or exit success is claimed.
- Phase127 `FIRST_GOAL_TIMEOUT_LOCAL_COST_BLOCKED` is not mixed into this staging-contract smoke.
- Phase135 not entered.

## Implemented deliverables

- Runner: `tools/run_phase134_bounded_corridor_alignment_staging_smoke.py`
- Analyzer: `tools/analyze_phase134_bounded_corridor_alignment_staging_smoke.py`
- Focused tests: `src/tugbot_maze/test/test_phase134_bounded_corridor_alignment_staging_smoke.py`
- Runtime/log directory: `log/phase134_bounded_corridor_alignment_staging_smoke/`
- Report: `doc/doc_report/phase134_bounded_corridor_alignment_staging_smoke_report.md`

## TDD/static verification

Focused Phase134 tests were written first and verified RED before implementation:

```text
python3 -m pytest src/tugbot_maze/test/test_phase134_bounded_corridor_alignment_staging_smoke.py -q
# initial RED: 6 failed, 1 passed in 0.05s
```

After implementing runner/analyzer:

```text
python3 -m pytest src/tugbot_maze/test/test_phase134_bounded_corridor_alignment_staging_smoke.py -q
# 7 passed in 0.03s
```

Pre-runtime static/focused bundle:

```text
python3 -m pytest src/tugbot_maze/test/test_phase134_bounded_corridor_alignment_staging_smoke.py -q
# 7 passed in 0.02s
python3 -m pytest \
  src/tugbot_maze/test/test_phase132_corridor_alignment_staging_contract_design.py \
  src/tugbot_maze/test/test_phase133_corridor_alignment_staging_contract_replay.py \
  src/tugbot_maze/test/test_phase134_bounded_corridor_alignment_staging_smoke.py -q
# 22 passed in 0.07s
python3 -m py_compile \
  tools/run_phase134_bounded_corridor_alignment_staging_smoke.py \
  tools/analyze_phase134_bounded_corridor_alignment_staging_smoke.py
# py_compile ok
```

Recorded under:

- `log/phase134_bounded_corridor_alignment_staging_smoke/phase134_static_test_bundle.txt`
- `log/phase134_bounded_corridor_alignment_staging_smoke/phase134_pre_runtime_guard.txt`

## Runtime commands and artifacts

Visible-stack launch:

```text
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch tugbot_bringup tugbot_maze_slam_nav.launch.py headless:=false use_rviz:=true
```

Phase134 bounded smoke command:

```text
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 tools/run_phase134_bounded_corridor_alignment_staging_smoke.py \
  --output log/phase134_bounded_corridor_alignment_staging_smoke/phase134_bounded_corridor_alignment_staging_smoke.json \
  --phase120-output log/phase134_bounded_corridor_alignment_staging_smoke/phase134_phase120_ingress_artifact.json \
  --preflight-output log/phase134_bounded_corridor_alignment_staging_smoke/phase134_ingress_preflight.json \
  --run-id phase134_bounded_corridor_alignment_staging_smoke_visible_stack \
  --launch-log-path log/phase134_bounded_corridor_alignment_staging_smoke/phase134_visible_stack_launch_stdout.log \
  --readiness-wait-timeout-sec 60 \
  --bounded-goal-result-wait-sec 45 \
  --handoff-collection-sec 4 \
  --staging-result-observation-sec 75
```

Runner output:

```json
{"accepted": true, "artifact": "log/phase134_bounded_corridor_alignment_staging_smoke/phase134_bounded_corridor_alignment_staging_smoke.json", "classification": "FIRST_DISPATCH_STAGING_ACCEPTED_STOP", "first_literal_goal_kind": "corridor_alignment_staging", "handoff_allowed": true, "maze_explorer_max_goals": 1, "maze_explorer_started": true, "rejected": false, "result_status_label": "SUCCEEDED", "second_goal_dispatched": false, "second_step_attempted": false, "staging_applied": true, "stop_reason": "first_literal_staging_terminal_result_bounded_stop", "timeout": false, "two_step_stage_dispatch_requested": true}
```

Analyzer command:

```text
python3 tools/analyze_phase134_bounded_corridor_alignment_staging_smoke.py \
  --artifact log/phase134_bounded_corridor_alignment_staging_smoke/phase134_bounded_corridor_alignment_staging_smoke.json \
  --output-json log/phase134_bounded_corridor_alignment_staging_smoke/phase134_bounded_corridor_alignment_staging_smoke_analysis.json \
  --output-md log/phase134_bounded_corridor_alignment_staging_smoke/phase134_bounded_corridor_alignment_staging_smoke_summary.md
```

Analyzer output:

```json
{"analysis": "log/phase134_bounded_corridor_alignment_staging_smoke/phase134_bounded_corridor_alignment_staging_smoke_analysis.json", "artifact": "log/phase134_bounded_corridor_alignment_staging_smoke/phase134_bounded_corridor_alignment_staging_smoke.json", "classification": "FIRST_DISPATCH_STAGING_ACCEPTED_STOP", "valid": true}
```

Primary artifacts:

- `log/phase134_bounded_corridor_alignment_staging_smoke/phase134_bounded_corridor_alignment_staging_smoke.json`
- `log/phase134_bounded_corridor_alignment_staging_smoke/phase134_bounded_corridor_alignment_staging_smoke_analysis.json`
- `log/phase134_bounded_corridor_alignment_staging_smoke/phase134_bounded_corridor_alignment_staging_smoke_summary.md`
- `log/phase134_bounded_corridor_alignment_staging_smoke/phase134_phase120_ingress_artifact.json`
- `log/phase134_bounded_corridor_alignment_staging_smoke/phase134_ingress_preflight.json`
- `log/phase134_bounded_corridor_alignment_staging_smoke/phase134_visible_stack_launch_stdout.log`
- `log/phase134_bounded_corridor_alignment_staging_smoke/phase134_bounded_corridor_alignment_staging_smoke_maze_explorer_stdout.log`
- `log/phase134_bounded_corridor_alignment_staging_smoke/phase134_bounded_corridor_alignment_staging_smoke_maze_explorer_stderr.log`

## Runtime result classification

Final classification:

```text
FIRST_DISPATCH_STAGING_ACCEPTED_STOP
```

Reason:

- The visible stack was active and the same-run inner-ingress/handoff chain allowed `maze_explorer` startup.
- `maze_explorer` started with `max_goals=1`.
- Exactly one literal dispatch was observed.
- `first_literal_dispatch.goal_kind` was `corridor_alignment_staging`.
- The staging dispatch produced a bounded terminal result with `result_status_label=SUCCEEDED`.
- The smoke stopped at the staging result.
- `second_step_attempted=false`.
- `second_goal_dispatched=false`.

This is only a bounded staging-contract smoke result. It is not autonomous exploration success, not exit success, not a Phase127 first-goal timeout diagnosis, and not an authorization to tune or repair behavior.

## Required recorded fields

Observed first-literal dispatch fields:

```text
first_literal_dispatch.goal_kind = corridor_alignment_staging
original_target = [1.6847770135179339, 0.974985491460126]
refined_target = [1.6797071058217734, 0.024999019958582575]
staging target = {x=1.6797071058217734, y=0.024999019958582575, yaw=1.5654595565187426, staging_distance_m=0.1750000026077029}
staging_applied = true
two_step_stage_dispatch_requested = true
staging_reason = reduce_lateral_residual_and_front_wedge_risk_before_second_step_forward_goal
lateral residual before = 0.175000002607703
lateral residual after = 7.719519468096792e-17
front_wedge_risk = {sample_count=139, max=99, mean=47.82014388489208, high_cost_count=40, lethal_count=9}
staging_executability_check.hard_safety_pass = true
staging_executability_check.lateral_residual_reduced = true
staging_executability_check.local_same_corridor = true
staging_executability_check.local_two_side_wall_evidence = true
pending_corridor_alignment_second_step = {available=false, exists=false, runtime_serialized=false, source=not_serialized_or_not_applicable}
second_step_forward_goal = null
accepted = true
rejected = false
result_status_label = SUCCEEDED
abort_text = succeeded
timeout = false
second_step_attempted=false
second_goal_dispatched=false
```

Evidence counts:

```text
goal_event_count = 2
dispatch_event_count = 1
explorer_states = 26
nav2_feedback_samples = 160
action_status_samples = 3
cmd_vel_samples = 160
odom_velocity_samples = 160
```

## Cleanup and final boundary

The visible-stack launch process was stopped after the bounded smoke.

Post-cleanup process guard recorded no matching Gazebo/RViz/Nav2/SLAM/maze_explorer process in:

- `log/phase134_bounded_corridor_alignment_staging_smoke/phase134_post_runtime_process_check_before_cleanup.txt`

Phase135 not entered. Stop here for human acceptance.
