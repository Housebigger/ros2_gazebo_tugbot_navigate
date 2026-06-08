# Phase67: Goal 1 Timeout Visual Replay / Terminal Pose Diagnosis

Status: COMPLETED - visual replay running for human observation

Classification: `VISUAL_EVIDENCE_INCONCLUSIVE`

Candidate visual hypotheses to check manually:

- `VISUALLY_TARGET_TOO_CLOSE_TO_WALL`
- `VISUALLY_TURNING_FOOTPRINT_COLLIDES`
- `VISUALLY_GOAL_TOLERANCE_ORIENTATION_BLOCKED`
- `VISUALLY_COSTMAP_RECOVERY_LOOP`

Run id: `phase67_goal1_timeout_visual_replay`

Artifact directory: `log/phase67_goal1_timeout_visual_replay/`

Marker topic: `/phase67/goal1_timeout_visual_markers`

## Scope

Phase66 已验收方向明确：Phase65 inner ingress 生效，Goal 1 dispatch 产生，但第一个 autonomous goal timeout。Phase67 的目标是让用户在 Gazebo/RViz 中直接观察 Goal 1 为什么 timeout：dispatch target、dispatch pose、trajectory path、timeout terminal pose、robot footprint、front wedge、target tolerance circle、local costmap high-cost/lethal 区域，以及目标点与墙体/高代价区域的关系。

This phase is visual/manual diagnosis only. It is not autonomous exploration success and not exit success.

## Guardrails

Held / required:

- 不调 Nav2/MPPI/controller；
- 不调 inflation/robot_radius/clearance_radius_m/map threshold；
- 不改 branch scoring；
- 不接入 target projection；
- 不改 fallback/terminal acceptance；
- 不声明 autonomous exploration success；
- 不声明 exit success；
- bounded Goal 1 replay only, `max_goals=1` or equivalent.

## Implemented artifacts

- Visual overlay node: `src/tugbot_maze/tugbot_maze/phase67_goal1_timeout_visual_overlay.py`
- Visible launch: `src/tugbot_bringup/launch/phase67_goal1_timeout_visual_replay.launch.py`
- Run script: `tools/run_phase67_goal1_timeout_visual_replay.sh`
- Payload analyzer: `tools/analyze_phase67_goal1_timeout_visual_replay.py`
- Focused tests: `src/tugbot_maze/test/test_phase67_goal1_timeout_visual_replay.py`
- Report: `doc/doc_report/phase67_goal1_timeout_visual_replay_report.md`

## Phase66 evidence preserved as visual replay input

From accepted Phase66 artifact:

- Phase66 classification: `INNER_INGRESS_TIMEOUT_REMAINS`
- Secondary observed risk: `LOCAL_COST_RISK_REMAINS`
- Inner ingress waypoint: map `(2.0, 0.0, 0.0)`
- Goal 1 dispatch target: `(2.0836008737, 1.0229248117)`
- Goal 1 dispatch pose: `(1.8531101281, 0.0241760857, -0.0054941657)`
- Goal 1 terminal pose: `(2.4175180047, 1.0220449274, 1.5843474170)`
- Goal 1 outcome: timeout / canceled after timeout
- final distance_remaining: about `0.330906868 m`
- max recoveries: `4`
- cmd_vel nonzero samples: `1056`
- robot total motion: about `1.739336487 m`
- robot_stuck: `false`
- timeout front wedge max cost: `99`
- timeout footprint lethal cells: `48`
- timeout robot local cost max: `100`
- timeout right side clearance: `0.0 m`
- timeout right side cost max: `100`

## Marker overlay contract

RViz MarkerArray topic:

`/phase67/goal1_timeout_visual_markers`

Markers:

- `goal1_dispatch_target`: red sphere at Phase66 Goal 1 target.
- `goal1_dispatch_pose`: cyan arrow at dispatch pose.
- `goal1_timeout_terminal_pose`: yellow arrow at timeout terminal pose.
- `goal1_trajectory_path`: green line strip from dispatch pose to terminal pose.
- `terminal_robot_footprint_circle`: robot radius circle at terminal pose.
- `terminal_front_wedge`: front wedge from terminal pose / heading.
- `target_tolerance_circle`: reference tolerance circle around Goal 1 target.
- `nearest_wall_or_costmap_risk_marker`: magenta marker at observed timeout front-wedge / wall-side risk point.
- `timeout_diagnostics_text`: text summary of recoveries, distance_remaining, front-wedge, footprint lethal evidence.

## How to run visual replay

Recommended command from workspace root:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select tugbot_gazebo tugbot_description tugbot_maze tugbot_bringup --symlink-install
source install/setup.bash
chmod +x tools/run_phase67_goal1_timeout_visual_replay.sh
PHASE67_MAX_GOALS=1 tools/run_phase67_goal1_timeout_visual_replay.sh
```

Default behavior leaves Gazebo/RViz/SLAM/Nav2/overlay alive for human observation. For CI cleanup-only runs:

```bash
PHASE67_CLEANUP_ON_EXIT=1 PHASE67_MAX_GOALS=1 tools/run_phase67_goal1_timeout_visual_replay.sh
```

## RViz observation steps

1. Set Fixed Frame to `map`.
2. Confirm active clean scaled2x Gazebo world is visible.
3. Add or verify displays for Map, LaserScan, TF, RobotModel, local costmap, and footprint.
4. Add MarkerArray display for `/phase67/goal1_timeout_visual_markers`.
5. Inspect the red Goal 1 dispatch target relative to walls and local costmap cells.
6. Inspect the green trajectory path from dispatch pose to terminal pose.
7. Inspect the yellow terminal pose arrow and orange robot footprint circle.
8. Inspect the red front wedge against local costmap high-cost/lethal cells.
9. Inspect whether the terminal orientation / target tolerance circle suggests a goal tolerance or orientation blockage.
10. Record screenshots / human observations before changing any code or parameters.

## Suggested screenshot moments

- After inner ingress succeeds, before `maze_explorer` starts.
- Immediately after Goal 1 dispatch target appears.
- While robot turns near the Goal 1 target.
- At timeout terminal pose, with local costmap high-cost/lethal cells visible.
- With local costmap + footprint + front wedge + target tolerance circle visible together.

## Classification guidance after human observation

Use one of:

- `VISUALLY_TARGET_TOO_CLOSE_TO_WALL`
- `VISUALLY_TURNING_FOOTPRINT_COLLIDES`
- `VISUALLY_GOAL_TOLERANCE_ORIENTATION_BLOCKED`
- `VISUALLY_COSTMAP_RECOVERY_LOOP`
- `VISUAL_EVIDENCE_INCONCLUSIVE`

Current classification remains `VISUAL_EVIDENCE_INCONCLUSIVE` until user visual feedback confirms one of the specific visual modes.

## Validation log

Completed:

- `python3 tools/analyze_phase67_goal1_timeout_visual_replay.py --phase66-artifact log/phase66_bounded_autonomous_rerun_from_inner_ingress/phase66_bounded_autonomous_rerun_from_inner_ingress.json --output log/phase67_goal1_timeout_visual_replay/phase67_goal1_timeout_visual_replay.json`: PASS, generated `VISUAL_EVIDENCE_INCONCLUSIVE` payload.
- `python3 -m py_compile tools/analyze_phase67_goal1_timeout_visual_replay.py src/tugbot_maze/tugbot_maze/phase67_goal1_timeout_visual_overlay.py src/tugbot_bringup/launch/phase67_goal1_timeout_visual_replay.launch.py src/tugbot_maze/test/test_phase67_goal1_timeout_visual_replay.py`: PASS.
- `bash -n tools/run_phase67_goal1_timeout_visual_replay.sh`: PASS.
- `pytest -q src/tugbot_maze/test/test_phase67_goal1_timeout_visual_replay.py`: PASS, `4 passed in 0.59s`.
- `colcon build --packages-select tugbot_gazebo tugbot_description tugbot_maze tugbot_bringup --symlink-install`: PASS, `4 packages finished [2.05s]`.
- `git diff -- src/tugbot_navigation/config | wc -c`: `0`.
- Phase67 visible replay launch: running for human observation.
- Marker topic observed in topic list: `/phase67/goal1_timeout_visual_markers`.
- Ready marker: `log/phase67_goal1_timeout_visual_replay/phase67_goal1_timeout_visual_replay_ready_for_human_observation.txt`.

Runtime/visual replay process state at completion:

- Gazebo sim: running.
- RViz2: running.
- SLAM toolbox: running.
- Nav2 controller/planner/bt_navigator: running.
- `phase67_goal1_timeout_visual_overlay`: running.
- `maze_explorer`: running with `max_goals:=1`, `near_exit_fallback_enabled:=false`, `startup_warmup_no_dispatch:=false`.

Cleanup intentionally NOT performed because the user requested live visual observation. To clean up after human observation, terminate the Phase67 launch process or run the usual ROS/Gazebo cleanup command/pkill pattern.

## Stop condition

Stop after Phase67 visual replay handoff / evidence report and wait for human observation. 不进入 Phase68.
