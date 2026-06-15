# Phase53: Entrance Ingress Waypoint Design and Geometry Validation

Status: COMPLETE

Classification: INGRESS_WAYPOINT_GEOMETRY_VALIDATED

## Scope

Phase53 designs and statically validates a short `ingress_waypoint_map` candidate inside the active scaled2x maze entrance.

This phase is intentionally offline/static. It reads active metadata, active SDF wall geometry, and the accepted Phase34 SLAM map snapshot. It does not start ROS/Gazebo/Nav2 and does not run `maze_explorer`.

## Guardrails

Held guardrails:

- No Nav2 / MPPI / controller parameter tuning.
- No `clearance_radius_m` tuning.
- No map sufficiency threshold tuning.
- maze_explorer not started; 不启动 maze_explorer runtime.
- No goal dispatch.
- No full exploration.
- No old scaffold world/map.
- No autonomous exploration success claim.
- No Phase54 work started.

## Inputs read

- `README.md`
- `doc/ACTIVE_MAZE_WORLD.md`
- `doc/doc_report/phase48_baseline_separation_and_next_step_design_lock_report.md`
- `doc/doc_report/phase49_dispatch_entry_readiness_gate_implementation_report.md`
- `doc/doc_report/phase50_dispatch_entry_readiness_gate_bounded_runtime_validation_report.md`
- `doc/doc_report/phase51_map_sufficiency_gate_discrepancy_diagnostics_report.md`
- `doc/doc_report/phase52_startup_map_boundary_warmup_report.md`
- `src/tugbot_maze/config/maze_20260528_scaled_instance.yaml`
- `src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean_scaled2x.sdf`
- `log/phase34_scaled_clean_world_slam_smoke/phase34_scaled_clean_world_slam_smoke_map_snapshot.yaml`
- `log/phase34_scaled_clean_world_slam_smoke/phase34_scaled_clean_world_slam_smoke_map_snapshot.pgm`

## Active world and truth convention

Active world:

```text
src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean_scaled2x.sdf
```

Active config:

```text
src/tugbot_maze/config/maze_20260528_scaled_instance.yaml
```

Map-frame truth:

```text
entrance_map = (0.0, 0.0, 0.0)
exit_map     = (21.072562, 18.083566), radius=1.2m
```

World-frame truth:

```text
entrance_world = (-11.011281, -9.02507, 0.0)
exit_world     = (10.061281, 9.058496), radius=1.2m
```

Entrance geometry:

```text
opening_side: left
inward_direction: +X
opening_center_world: (-10.061281, -9.02507)
opening_width_m: 2.072423
```

Phase52 showed that starting directly at `entrance_map=(0,0,0)` leaves the 1.0m inclusive /map sufficiency sample partially outside the current SLAM map boundary:

```text
robot_cell: [3, 21]
sample_window: min_x=-17, max_x=23, min_y=1, max_y=41
out_of_bounds_count: 481
known_ratio/free_ratio: 0.4429369513168396
```

So Phase53 asks whether a short map-internal ingress waypoint can be designed as a future autonomous-exploration starting point candidate.

## Method

The analyzer is:

```text
tools/analyze_phase53_entrance_ingress_waypoint.py
```

It performs static validation only:

1. Reads active config and active SDF.
2. Reads accepted Phase34 SLAM occupancy snapshot.
3. Generates candidates along entrance yaw/inward +X direction at distances:

```text
0.8m, 1.0m, 1.2m, 1.5m
```

4. Converts map candidates to world-frame points using Phase41/active truth convention:

```text
point_world = entrance_world + point_map
```

5. Validates each candidate against:

- inside Phase34 map boundary;
- candidate cell is free, not occupied/unknown;
- local 0.35m radius is free;
- minimum clearance to active SDF wall boxes is at least 0.65m;
- entrance-to-candidate straight line is free and wall-clear.

The wall/clearance check uses active SDF collision boxes. The occupancy check uses the accepted Phase34 SLAM PGM snapshot. Runtime local costmap is not sampled in Phase53 because the guardrail forbids launching exploration/runtime; this phase therefore validates static geometry and map occupancy, not a live Nav2 costmap.

## Candidate results

| candidate | ingress_waypoint_map | world point | map cell | local free ratio 0.35m | min wall clearance | straight line | status |
|---|---:|---:|---:|---:|---:|---:|---|
| ingress_0.8m | (0.8, 0.0, 0.0) | (-10.211281, -9.02507, 0.0) | [19, 21] | 1.0 | 0.8762951431230236m | free | valid |
| ingress_1.0m | (1.0, 0.0, 0.0) | (-10.011281, -9.02507, 0.0) | [23, 21] | 1.0 | 0.8491058696422964m | free | selected |
| ingress_1.2m | (1.2, 0.0, 0.0) | (-9.811281, -9.02507, 0.0) | [27, 21] | 0.9798657718120806 | 0.8489300000000003m | free | rejected: local radius contains occupied/unknown |
| ingress_1.5m | (1.5, 0.0, 0.0) | (-9.511281, -9.02507, 0.0) | [33, 21] | 0.87248322147651 | 0.8489300000000003m | free | rejected: local radius contains occupied/unknown |

## Selected ingress_waypoint_map

Selected candidate:

```yaml
ingress_waypoint_map:
  x_m: 1.0
  y_m: 0.0
  yaw_rad: 0.0
```

Corresponding active-world coordinate:

```yaml
ingress_waypoint_world:
  x_m: -10.011281
  y_m: -9.02507
  yaw_rad: 0.0
```

Selection rationale:

- It is inside the Phase34 SLAM map boundary.
- It is exactly along the active entrance yaw/inward +X direction.
- It is within the requested 0.8m to 1.5m ingress distance range.
- It is far enough from the entrance boundary to avoid the Phase52 out-of-bounds gate geometry at `(0,0)`.
- Its occupancy cell is free (`PGM value=254`).
- Its local 0.35m radius is fully free in the accepted SLAM snapshot.
- Its minimum active-SDF wall clearance is `0.8491058696422964m`, above the Phase53 static criterion `0.65m`.
- The short entrance-to-candidate line is free in the map snapshot and wall-clear.
- It is still close to the entrance, so it is an entrance-guide candidate rather than an exploration shortcut.

## Artifacts

Structured JSON:

```text
log/phase53_entrance_ingress_waypoint_design/phase53_entrance_ingress_waypoint_design.json
```

Overlay:

```text
log/phase53_entrance_ingress_waypoint_design/phase53_entrance_ingress_waypoint_overlay.png
```

Test:

```text
src/tugbot_maze/test/test_phase53_entrance_ingress_waypoint_design.py
```

## Classification rationale

Classification: `INGRESS_WAYPOINT_GEOMETRY_VALIDATED`

Rationale:

- At least one candidate within 0.8m to 1.5m passed all static geometry checks.
- The selected 1.0m candidate is map-internal and locally free.
- The short entrance-to-candidate path is straight and static-map free.
- Active SDF wall clearance is above the configured static minimum.
- No runtime exploration, goal dispatch, Nav2 tuning, clearance tuning, or threshold tuning was performed.

This is not an autonomous exploration success claim. It only validates a geometry candidate for a future entrance-guidance phase, if authorized.

## Verification

Final verification commands are recorded in the assistant run, including:

```bash
python3 -m py_compile tools/analyze_phase53_entrance_ingress_waypoint.py
python3 -m pytest src/tugbot_maze/test/test_phase53_entrance_ingress_waypoint_design.py -q
git diff -- src/tugbot_navigation/config
```

Cleanup process check was also performed.

## Stop condition

Phase53 stops here for human acceptance. No Phase54 work was started.
