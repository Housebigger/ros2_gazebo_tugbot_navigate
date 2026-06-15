# Phase65: Ingress Goal Depth Adjustment / Inner Entrance Staging Point

Status: COMPLETED - waiting for human acceptance

Classification: `INNER_INGRESS_FIX_IMPROVES_FIRST_DISPATCH`

Run id: `phase65_ingress_goal_depth_adjustment_inner_staging`

Artifact directory: `log/phase65_ingress_goal_depth_adjustment_inner_staging/`

## Scope

Phase65 tested the human-observation hypothesis `ENTRY_STAGING_TOO_SHALLOW`.
The old Phase53/61/62 ingress target was map `(1.0, 0.0, 0.0)`, and the manual
visual check after Phase64.5 showed the robot still looked outside / on the map
edge. Phase65 preserved that old coordinate as evidence and introduced only a
minimal inner staging candidate at map `(2.0, 0.0, 0.0)`, one meter further
inward along the active entrance +X direction.

This phase does not claim autonomous exploration success and does not claim exit
success. The observed dispatch is only first-dispatch evidence after inner ingress
staging.

## Prior context preserved

- Phase61: `SINGLE_OPEN_EXCEPTION_APPLIED_AND_DISPATCHED`; first dispatch was
  recovered, but it was not autonomous exploration success and not exit success.
- Phase62: `CORRIDOR_TOO_NARROW`, auxiliary `LOCAL_COSTMAP_INFLATION_DOMINANT`.
- Phase64: `GEOMETRY_FEASIBILITY_BLOCKED` from static/config/log replay.
- Phase64.5/manual check: old ingress Nav2 goal to `(1.0, 0.0)` succeeded, final
  pose was about `(0.875, 0.021)`, but human visual inspection judged that staging
  point too shallow.
- Phase65 hypothesis: `ENTRY_STAGING_TOO_SHALLOW`.

## Guardrails

Held:

- bounded runtime only;
- `max_goals=1`;
- no Nav2/MPPI/controller parameter edits;
- no `clearance_radius_m` tuning;
- no map sufficiency threshold tuning;
- no branch selection scoring change;
- no target projection integration;
- no fallback/terminal acceptance change;
- no old scaffold world/map;
- no autonomous exploration success claim;
- no exit success claim.

## Implemented artifacts

- Wrapper: `tools/run_phase65_ingress_goal_depth_adjustment_inner_staging.sh`
- Analyzer: `tools/analyze_phase65_ingress_goal_depth_adjustment_inner_staging.py`
- Focused tests: `src/tugbot_maze/test/test_phase65_ingress_goal_depth_adjustment_inner_staging.py`
- Report: `doc/doc_report/phase65_ingress_goal_depth_adjustment_inner_staging_report.md`
- Runtime summary: `log/phase65_ingress_goal_depth_adjustment_inner_staging/phase65_ingress_goal_depth_adjustment_inner_staging.json`

## Ingress coordinates

Old / preserved Phase53 ingress:

```yaml
ingress_waypoint_map:
  x_m: 1.0
  y_m: 0.0
  yaw_rad: 0.0
```

Phase65 inner ingress staging:

```yaml
phase65_inner_ingress_waypoint_map:
  x_m: 2.0
  y_m: 0.0
  yaw_rad: 0.0
phase65_inner_ingress_depth_adjustment_m: 1.0
```

## Runtime protocol

The wrapper used the active scaled2x world only:

```text
src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean_scaled2x.sdf
```

Protocol:

1. Start SLAM + Nav2 + Gazebo with the active world.
2. Send a single Nav2 goal to the Phase65 inner ingress staging point `(2.0, 0.0)`.
3. If that goal succeeds, start `maze_explorer` with `max_goals=1`.
4. Record `/maze/explorer_state`, `/maze/goal_events`, and first-dispatch execution evidence.
5. Stop and clean up; do not enter any long run.

Runtime executed one bounded replay.

## Evidence summary

From `phase65_ingress_goal_depth_adjustment_inner_staging.json` / replay_01:

- Inner ingress Nav2 goal: succeeded.
- Old ingress waypoint map: `(1.0, 0.0, 0.0)`.
- Old/manual runtime pose evidence: `(0.874650, 0.020960, 0.031257)`.
- Inner ingress waypoint map: `(2.0, 0.0, 0.0)`.
- Inner ingress / dispatch pose: `(1.857528, 0.024700, -0.007427)`.
- Robot pose considered inside maze after inner ingress: `true`.
- Old-to-inner dispatch pose shift: `0.983459 m`.
- First dispatch observed: `true`.
- First dispatch target: `(2.039950, 1.023373)`.
- First dispatch target shift from Phase62 target: `1.567846 m`.
- Dispatch target local cost: `0` vs Phase62 baseline `49`.
- Target footprint lethal count: `0` vs Phase62 baseline `8`.
- Front wedge high-cost count: `40` vs Phase62 baseline `131`.
- Explorer state samples: `47`.
- Goal event count: `1`.
- Complete autonomous success claimed: `false`.
- Dispatch observed not exit success: `true`.

## Classification rationale

Classification: `INNER_INGRESS_FIX_IMPROVES_FIRST_DISPATCH`.

Reason: the inner ingress replay produced a successful inner ingress, a first
dispatch from an interior pose, and improved first-dispatch local-cost / footprint
/ front-wedge evidence relative to the Phase62 baseline. This supports the Phase65
hypothesis that the old entry staging was too shallow for the first topology /
first dispatch observation point.

This does not override the prior no-autonomous-success boundary. It is not an exit
success and not a long-run exploration result.

## Validation log

Completed validation:

- `python3 -m py_compile tools/analyze_phase65_ingress_goal_depth_adjustment_inner_staging.py src/tugbot_maze/test/test_phase65_ingress_goal_depth_adjustment_inner_staging.py`: PASS
- `bash -n tools/run_phase65_ingress_goal_depth_adjustment_inner_staging.sh`: PASS
- `pytest -q src/tugbot_maze/test/test_phase65_ingress_goal_depth_adjustment_inner_staging.py`: `5 passed in 0.01s`
- `source /opt/ros/jazzy/setup.bash && colcon build --packages-select tugbot_maze tugbot_bringup --symlink-install`: PASS, `2 packages finished [0.96s]`
- `git diff -- src/tugbot_navigation/config | wc -c`: `0`
- Cleanup check: empty after filtering the check command itself.

## Stop condition

Stop after Phase65 evidence and wait for human acceptance. 不进入 Phase66.
