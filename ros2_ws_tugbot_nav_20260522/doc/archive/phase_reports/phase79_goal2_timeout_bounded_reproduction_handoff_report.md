# Phase79: Goal2 timeout bounded reproduction handoff run

Status: `SCENE_HELD_WAITING_FOR_USER_SCREENSHOT`

Workspace: `/home/hyh/Desktop/agent_playground/playground_hermes/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260522`

Workflow:

```text
automatic bounded reproduction -> user screenshot -> ChatGPT discussion -> Hermes executes confirmed plan
```

Phase79 applies the Phase78-flow-fix workflow to the Phase75/74 Goal2 timeout chain.  The runbook starts visible Gazebo/RViz, sends the known inner-ingress goal, starts `maze_explorer` with `max_goals=2`, waits only for the known Goal2 timeout/local-cost/recovery-loop trigger, stops further exploration, and holds the scene for screenshots.

## Required source documents read

- `doc/doc_proposal/obstacle_reproduction_handoff_workflow.md`
- `doc/doc_report/phase78_flow_fix_obstacle_reproduction_handoff_report.md`
- `doc/doc_report/phase75_goal2_timeout_after_directional_redispatch_diagnosis_report.md`
- `doc/doc_report/phase77_goal2_timeout_visual_root_cause_replay_first_application_report.md`

## Reused artifact chain

Phase79 prefers the real Phase75/74 Goal2 timeout chain:

- Phase75 diagnosis JSON:
  `log/phase75_goal2_timeout_after_directional_redispatch_diagnosis/phase75_goal2_timeout_after_directional_redispatch_diagnosis.json`
- Phase74 replay source:
  `log/phase74_directional_local_costmap_readiness_gate_validation/replay_02/`
- Phase77 lightweight target/pose event:
  `log/phase77_goal2_visual_root_cause_replay/phase77_goal2_timeout_visual_root_cause_event.json`

Key known evidence from prior real artifacts:

- Classification: `GOAL2_TIMEOUT_LOCAL_COST_RECOVERY_LOOP`
- Trigger chain: `goal_timeout`, `local_cost_risk`, `recovery loop`, `near-goal outside tolerance`
- Goal2 target: `[2.058503376259287, 1.0236490342193865]`
- Goal2 final/terminal pose: `[2.426872326194781, 1.0327763735179467, 1.6075951571994673]`
- Recovery count evidence: `3`
- Final XY error evidence: `0.36848200987191976`

## New runbook

Runbook:

```bash
tools/run_phase79_goal2_timeout_bounded_reproduction_handoff.sh
```

Default behavior:

- visible mode
- `headless=false`
- `use_rviz=true`
- `max_goals=2`
- `goal_timeout_sec=45.0`
- send inner ingress goal first using the existing Phase74 helper
- start runtime recorders
- start `maze_explorer`
- wait for Goal2 timeout/local-cost/recovery trigger
- stop only `maze_explorer` to prevent continued exploration
- keep Gazebo/RViz/SLAM/Nav2 alive for user screenshots
- write `SCENE_HELD_WAITING_FOR_USER_SCREENSHOT` marker when the scene is ready

The runbook uses these key outputs:

- artifact dir:
  `log/phase79_goal2_timeout_bounded_reproduction_handoff/`
- minimal field summary:
  `log/phase79_goal2_timeout_bounded_reproduction_handoff/phase79_goal2_timeout_bounded_reproduction_handoff_minimal_field_summary.md`
- held-scene marker:
  `log/phase79_goal2_timeout_bounded_reproduction_handoff/phase79_goal2_timeout_bounded_reproduction_handoff_SCENE_HELD_WAITING_FOR_USER_SCREENSHOT.txt`
- trigger evidence file:
  `log/phase79_goal2_timeout_bounded_reproduction_handoff/phase79_goal2_timeout_bounded_reproduction_handoff_trigger_detected.json`

## Live execution result

Phase79 visible bounded reproduction was executed with:

```bash
PHASE79_HOLD_AFTER_TRIGGER_SEC=600 \
PHASE79_CLEANUP_ON_EXIT=0 \
PYTHONDONTWRITEBYTECODE=1 \
tools/run_phase79_goal2_timeout_bounded_reproduction_handoff.sh
```

Observed result:

- Status marker: `SCENE_HELD_WAITING_FOR_USER_SCREENSHOT`
- Visible Gazebo/RViz launch remains alive for observation.
- `maze_explorer` was stopped after the Goal2 trigger to prevent continued exploration.
- Current held pose from `/odom` / TF: approximately `(x=2.42, y=1.03, yaw=1.60 rad)`.
- Trigger file:
  `log/phase79_goal2_timeout_bounded_reproduction_handoff/phase79_goal2_timeout_bounded_reproduction_handoff_trigger_detected.json`
- Trigger evidence:
  ```json
  {
    "elapsed_sec": 55.39170718193054,
    "event": "timeout",
    "goal_sequence": 2,
    "result_reason": "goal_timeout",
    "target": [2.057855221699651, 1.0261005743935105],
    "timeout_front_wedge_cost_max": 99
  }
  ```
- Nav2 feedback evidence: `number_of_recoveries=3`, `distance_remaining≈0.357 m` near timeout.

## Minimal screenshot handoff

No lengthy human observation report is required.

User only needs 1-4 screenshots plus a brief judgment for ChatGPT discussion:

1. Gazebo wide: robot stopped at the Goal2 problem area and nearby corridor/walls.
2. RViz local costmap / footprint / front wedge: local-cost risk around the robot nose and footprint.
3. Goal tolerance: target marker/tolerance circle vs final robot pose.
4. Recovery loop: Nav2 recovery/local oscillation context if visible.

## Guardrails

No navigation strategy changed.

No Nav2/MPPI/controller tuning.

No inflation/robot_radius/clearance_radius_m/map threshold tuning.

No branch scoring, centerline gate, directional readiness override, fallback, or terminal acceptance changes.

No autonomous exploration success claimed.

No exit success claimed.

This is a handoff reproduction run only; algorithm repair phase not entered.

Phase80 not entered.

## Verification plan and results

RED static test was executed before implementation:

```text
3 failed in 0.03s
```

Expected failure cause: missing Phase79 runbook, minimal field summary, and report.

Final verification executed after implementation and live handoff:

```text
PYTHONDONTWRITEBYTECODE=1 pytest -q src/tugbot_maze/test/test_phase79_goal2_timeout_bounded_reproduction_handoff.py
3 passed in 0.03s

PYTHONDONTWRITEBYTECODE=1 python3 -m py_compile src/tugbot_maze/test/test_phase79_goal2_timeout_bounded_reproduction_handoff.py
# exit code 0

bash -n tools/run_phase79_goal2_timeout_bounded_reproduction_handoff.sh
# exit code 0

git diff -- src/tugbot_maze/tugbot_maze/maze_explorer.py src/tugbot_navigation/config src/tugbot_bringup/launch | cat
# empty output

scene/process check:
- ros2 launch tugbot_bringup tugbot_maze_slam_nav.launch.py is running
- gz sim server/gui are running
- rviz2 is running
- slam_toolbox, controller_server, bt_navigator are running
- maze_explorer is not running after trigger stop

cleanup:
removed_count=1
remaining_count=0
```

Interpretation:

- Phase79 static contract passed.
- Runbook shell syntax passed.
- Strategy/config guard diff stayed empty.
- Visible scene remains available for screenshots.
- Temporary `__pycache__` was removed.
- No algorithm repair phase was entered.
