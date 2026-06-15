# Phase68: Corridor Centerline Target Selection Design / Static Replay

Status: COMPLETED - waiting for human acceptance

Classification: `CENTERLINE_REPLAY_IMPROVES_LOCAL_COST`

Run id: `phase68_corridor_centerline_target_selection_static_replay`

Artifact directory: `log/phase68_corridor_centerline_target_selection_static_replay/`

## Scope

Phase67 human visual observation found that Goal 1 timeout happened while the robot was not following the corridor centerline and drifted toward one wall, increasing local-cost / footprint / front-wedge risk.

Phase68 is static/log replay only. It validates a design question: if the Goal 1 target were shifted toward the local corridor centerline / maximum-clearance region, would local-cost proxy, footprint lethal risk, and front-wedge high-cost risk decrease?

This phase does not connect to runtime, does not modify `maze_explorer.py`, does not change dispatch behavior, and does not claim autonomous exploration success or exit success. It is not autonomous exploration success and not exit success.

## Guardrails

Held:

- static/log replay only;
- no runtime dispatch integration;
- no Nav2/MPPI/controller parameter edits;
- no inflation/robot_radius/clearance_radius_m/map threshold tuning;
- no branch scoring change;
- no corridor-following cmd_vel control;
- no fallback/terminal acceptance change;
- no autonomous exploration success claim;
- no exit success claim.

## Inputs

- Phase67 artifact: `log/phase67_goal1_timeout_visual_replay/phase67_goal1_timeout_visual_replay.json`
- Active world: `src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean_scaled2x.sdf`
- Active metadata: `src/tugbot_maze/config/maze_20260528_scaled_instance.yaml`
- Active Nav2 geometry config: `src/tugbot_navigation/config/nav2_slam_params.yaml`

## Replay method

The analyzer samples candidate targets around the accepted Phase66/67 Goal 1 target using the same open direction:

- forward offsets around the original target;
- lateral offsets across the corridor;
- SDF wall ray casts left/right of each candidate;
- `balance_error=abs(left-right)`;
- `min_clearance` and occupancy;
- Nav2-geometry static local-cost proxy from robot radius, inflation radius, and cost scaling;
- footprint cost summary from disk samples around the target;
- front wedge cost summary from samples in front of the candidate heading;
- forward progress from dispatch pose.

It compares the original Goal 1 target with the best centerline / max-clearance candidate. Missing finite side-wall ray hits are recorded with hit flags and a bounded no-hit value of 6.0 m, so the evidence remains interpretable rather than silently treating no-hit as safe.

## Implemented artifacts

- Analyzer: `tools/analyze_phase68_corridor_centerline_target_selection_static_replay.py`
- Focused tests: `src/tugbot_maze/test/test_phase68_corridor_centerline_target_selection_static_replay.py`
- Report: `doc/doc_report/phase68_corridor_centerline_target_selection_static_replay_report.md`
- Artifact: `log/phase68_corridor_centerline_target_selection_static_replay/phase68_corridor_centerline_target_selection_static_replay.json`

## Static replay result

Classification: `CENTERLINE_REPLAY_IMPROVES_LOCAL_COST`

Candidate sampling:

- candidate_count: `55`
- safe_candidate_count: `20`

Original Phase66/67 Goal 1 target:

- target: `(2.0836008737, 1.0229248117)`
- left_wall_clearance_m: `6.0` bounded no-hit value
- left_wall_hit: `false`
- right_wall_clearance_m: `0.819692`
- right_wall_hit: `true`
- balance_error_m: `5.180308`
- min_clearance_m: `0.81968`
- static_local_cost: `24`
- footprint_high_cost_count: `0`
- footprint_lethal_count: `0`
- front_wedge_high_cost_count: `1`
- front_wedge_lethal_count: `0`

Best centerline candidate:

- target: `(1.9847012106, 1.2234712069)`
- forward_offset_m: `0.2`
- lateral_offset_m: `0.1`
- left_wall_clearance_m: `0.847433`
- left_wall_hit: `true`
- right_wall_clearance_m: `0.918594`
- right_wall_hit: `true`
- balance_error_m: `0.071161`
- min_clearance_m: `0.84742`
- static_local_cost: `22`
- footprint_high_cost_count: `0`
- footprint_lethal_count: `0`
- front_wedge_high_cost_count: `0`
- front_wedge_lethal_count: `0`
- safe_by_static_replay: `true`
- improves_over_original: `true`

Interpretation:

The static/log replay supports the Phase67 visual observation: a target shifted toward a more balanced corridor centerline / maximum-clearance region reduces the centerline balance error, slightly improves the static local-cost proxy (`24 -> 22`), and removes the static front-wedge high-cost count in this replay (`1 -> 0`) without increasing footprint lethal risk.

This is design evidence only. It does not modify runtime dispatch and does not prove a future autonomous run will succeed. A later phase would need explicit implementation and bounded runtime validation before any behavior claim.

## Validation log

Completed:

- Initial focused tests were RED before analyzer/report implementation.
- `python3 tools/analyze_phase68_corridor_centerline_target_selection_static_replay.py --phase67-artifact log/phase67_goal1_timeout_visual_replay/phase67_goal1_timeout_visual_replay.json --output log/phase68_corridor_centerline_target_selection_static_replay/phase68_corridor_centerline_target_selection_static_replay.json`: PASS, classification `CENTERLINE_REPLAY_IMPROVES_LOCAL_COST`.
- `python3 -m py_compile tools/analyze_phase68_corridor_centerline_target_selection_static_replay.py src/tugbot_maze/test/test_phase68_corridor_centerline_target_selection_static_replay.py`: PASS.
- `pytest -q src/tugbot_maze/test/test_phase68_corridor_centerline_target_selection_static_replay.py`: PASS, `5 passed in 0.42s`.
- `source /opt/ros/jazzy/setup.bash && colcon build --packages-select tugbot_maze tugbot_bringup --symlink-install`: PASS, `2 packages finished [1.06s]`.
- `git diff -- src/tugbot_navigation/config | wc -c`: `0`.
- cleanup check: empty after filtering Hermes shell wrapper; no ROS/Gazebo/Nav2/RViz/maze_explorer residuals.

## Stop condition

Stop after Phase68 static/log replay report and wait for human acceptance. 不进入 Phase69.
