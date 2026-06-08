# Phase64 Corridor Width / Robot Footprint Feasibility Decision Report

Run id: `phase64_corridor_width_robot_footprint_feasibility_decision`
Artifact dir: `log/phase64_corridor_width_robot_footprint_feasibility_decision/`
Status: complete; stop for human acceptance. Do not enter Phase65. 不进入 Phase65.

## Preserved conclusions

- Phase61: `SINGLE_OPEN_EXCEPTION_APPLIED_AND_DISPATCHED`.
- Phase62: `CORRIDOR_TOO_NARROW`; secondary `LOCAL_COSTMAP_INFLATION_DOMINANT`.
- Phase63: `NO_SAFE_PROJECTION_IN_CORRIDOR`.
- First dispatch is not autonomous exploration success and not exit success.

## Guardrails

- static geometry/config/log replay only.
- no runtime dispatch integration.
- no Nav2/MPPI/controller parameter edits.
- no clearance_radius_m tuning.
- no map sufficiency threshold tuning.
- no branch selection scoring change.
- no entrance/fallback/terminal acceptance change.
- no autonomous exploration success claim.
- first dispatch is not exit success.
- no long run.

## Classification

`GEOMETRY_FEASIBILITY_BLOCKED`

Reasons:

- `active_world_global_min_gap_below_local_inflation_full_envelope`
- `first_dispatch_effective_width_below_local_inflation_full_envelope`
- `first_dispatch_mesh_clearance_margin_under_5cm`
- `first_dispatch_local_cost_or_footprint_hits_lethal_radius`
- `phase63_no_safe_projection_in_corridor`

Interpretation: Phase64 elevates the issue from exploration-candidate repair to a world/robot scale/footprint/inflation geometry mismatch decision. Runtime dispatch was not changed.

## Active geometry/config evidence

- active_world: `src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean_scaled2x.sdf`
- active_metadata: `src/tugbot_maze/config/maze_20260528_scaled_instance.yaml`
- Nav2 config: `/home/hyh/Desktop/agent_playground/playground_hermes/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260522/src/tugbot_navigation/config/nav2_slam_params.yaml`
- local robot_radius_m: `0.35`
- local inflation_radius_m: `0.7`
- local cost_scaling_factor: `3.0`
- Tugbot mesh turning_outer_diameter_m: `0.738244`

Required/envelope widths:

- robot_body_width_required_m: `0.738244`
- comfort_contract_width_m: `1.476488`
- inflation_full_envelope_width_m: `2.1`
- mesh_inflation_full_envelope_width_m: `2.138244`

World SDF wall-gap evidence:

- wall_count: `53`
- global_min_positive_wall_gap_m: `1.698`
- global_min_positive_wall_gap_pair: `['maze_wall_seg_011_h_010', 'maze_wall_seg_015_h_014']`

## First-dispatch corridor replay evidence

- target_map: `[0.558242117171631, 0.5108474753216039]`
- target_world_from_metadata_offset: `[-10.45303888282837, -8.514222524678395]`
- target_clearance_m: `0.403113`
- path_corridor_min_clearance_m: `0.403113`
- first_dispatch_effective_width_from_replay_clearance_m: `0.806226`
- nearest_sdf_wall_distance_m: `0.625145` (`maze_wall_outer_026_outer_003`)
- target_crosses_narrow_passage: `True`
- local_radius_max_cost: `99`
- local_cell_state: `None`
- footprint_max_cost: `99`
- footprint_lethal_count: `8`
- phase63_safe_projection_found: `False`

Margins:

- active_world_min_gap_minus_robot_body_width_m: `0.959756`
- active_world_min_gap_minus_comfort_contract_m: `0.221512`
- active_world_min_gap_minus_inflation_full_envelope_m: `-0.402`
- first_dispatch_clearance_minus_robot_radius_m: `0.053113`
- first_dispatch_clearance_minus_mesh_radius_m: `0.033991`
- first_dispatch_effective_width_minus_robot_body_width_m: `0.067982`
- first_dispatch_effective_width_minus_inflation_full_envelope_m: `-1.293774`

## Validation

- `python3 -m py_compile tools/analyze_phase64_corridor_width_robot_footprint_feasibility_decision.py`: PASS
- `pytest -q src/tugbot_maze/test/test_phase64_corridor_width_robot_footprint_feasibility_decision.py`: `5 passed in 0.08s`
- `colcon build --packages-select tugbot_maze tugbot_bringup --symlink-install`: PASS (`2 packages finished [0.94s]`)
- `git diff -- src/tugbot_navigation/config | wc -c`: `0`
- cleanup check: empty

## Stop condition

Phase64 complete. Stop for human acceptance. Do not enter Phase65. 不进入 Phase65.
