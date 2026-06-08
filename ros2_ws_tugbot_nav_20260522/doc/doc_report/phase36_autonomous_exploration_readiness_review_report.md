# Phase36 Scaled Clean World Autonomous Exploration Readiness Review Report

Date: 2026-05-28
Workspace: `ros2_ws_tugbot_nav_20260522`
Status: `NOT_READY_WITH_BLOCKERS`

## Scope

Phase36 is a readiness review only. It did not start `maze_explorer`, did not start
autonomous exploration, did not tune Nav2/MPPI/controller parameters, did not
modify navigation strategy, and did not run a long smoke.

Phase35 human acceptance baseline:

```text
PASS_AS_SCALED_CLEAN_WORLD_NAV2_MANUAL_GOAL_SMOKE_ACCEPTED
```

The active world reviewed in this phase is:

```text
src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean_scaled2x.sdf
```

Read inputs:

```text
README.md
doc/ACTIVE_MAZE_WORLD.md
doc/doc_report/phase35_scaled_clean_world_nav2_manual_goal_smoke_report.md
doc/doc_report/phase35_pre_map_world_artifact_cleanup_report.md
```

Machine-readable check artifact:

```text
log/phase36_autonomous_exploration_readiness_review/phase36_readiness_check.json
```

## Guardrails held

- no Nav2/MPPI/controller parameter changes;
- no fallback / terminal-acceptance continuation;
- no navigation strategy changes;
- no `maze_explorer` process started;
- no autonomous exploration process started;
- no old scaffold world/map used for runtime;
- no long run started;
- no autonomous exploration success claimed.

Analyzer guardrail fields:

```text
autonomous_exploration_started: False
maze_explorer_started: False
nav2_params_modified: False
strategy_modified: False
long_run_started: False
```

## Active metadata check

Active metadata file:

```text
src/tugbot_maze/config/maze_20260528_scaled_instance.yaml
```

Metadata values found:

```text
source_image: package://tugbot_maze/assets/maze_20260528.png
output_world: src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean_scaled2x.sdf
scale_factor: 2.0
entrance: x=-11.011281, y=-9.025070, yaw=0.000000
exit: x=10.061281, y=9.058496, radius=1.200000
image_allowed_for_runtime_path_planning: false
```

Required Phase36 approximate truth:

```text
entrance approx (-11.011, -9.025, yaw=0)
exit approx (10.061, 9.058)
exit radius 1.2
```

Result: PASS. The active metadata matches the required scaled2x entrance/exit/radius,
and runtime path planning from the image is disabled.

## Launch default world audit

Current maze workflow launch defaults checked:

```text
src/tugbot_bringup/launch/tugbot_maze_slam.launch.py
src/tugbot_bringup/launch/tugbot_maze_slam_nav.launch.py
src/tugbot_bringup/launch/tugbot_maze_explore.launch.py
```

Results:

```text
src/tugbot_bringup/launch/tugbot_maze_explore.launch.py: active_world_default_present=True, old_scaffold_default_present=False, legacy_static_map_default_present=False
src/tugbot_bringup/launch/tugbot_maze_slam.launch.py: active_world_default_present=True, old_scaffold_default_present=False, legacy_static_map_default_present=False
src/tugbot_bringup/launch/tugbot_maze_slam_nav.launch.py: active_world_default_present=True, old_scaffold_default_present=False, legacy_static_map_default_present=False
```

Result: PASS for world default alignment. All current maze workflow launch files point
to `tugbot_maze_world_20260528_clean_scaled2x.sdf`; none silently default to the old
`tugbot_maze_world.sdf` scaffold or a legacy static map.

## maze_explorer active truth audit

`maze_explorer.py` itself declares fallback parameter defaults for the old first-pass
scaffold values:

```text
entrance_x=-4.0
entrance_y=-3.0
entrance_yaw=0.0
exit_x=4.0
exit_y=3.0
exit_radius=0.6
```

That can be acceptable only if the active launch injects current metadata-derived
values. The active exploration launch currently does not do that.

`tugbot_maze_explore.launch.py` defaults found:

```text
entrance_x=-4.0
entrance_y=-3.0
entrance_yaw=0.0
exit_x=4.0
exit_y=3.0
exit_radius=0.6
maze_config default file=maze_instance.yaml
maze_config default file exists=False
uses_active_metadata_defaults=False
```

Expected from active metadata:

```text
entrance_x=-11.011281
entrance_y=-9.025070
entrance_yaw=0.000000
exit_x=10.061281
exit_y=9.058496
exit_radius=1.200000
```

Result: FAIL / blocker. The active exploration launch still exposes old entrance/exit
values as current defaults and points `maze_config` at missing `maze_instance.yaml`
instead of `maze_20260528_scaled_instance.yaml`. Therefore the old entrance/exit
would still be active truth for a default autonomous exploration smoke.

## Old world/map/image reference audit

Current workflow audit result:

```text
active_source_legacy_files_absent: True
current_launch_legacy_default_refs: []
decorative_route_archived: True
old_scaffold_archived: True
legacy_static_maps_archived: True
nav2_config_diff_empty: True
```

Interpretation:

- `tugbot_maze_world.sdf` is not a current maze launch default.
- `maze_20260522.jpg` decorative route is archived/deprecated and is not part of the current workflow.
- legacy static/explored maps are archived/deprecated and are not current workflow defaults.
- historical docs/archive references still exist for provenance; those are not current defaults.

Result: PASS for old world/map/image default exclusion.

## goal_events / explorer_state diagnostic readiness

Checked source schema support in `maze_explorer.py` and launch topic wiring in
`tugbot_maze_explore.launch.py`.

```text
goal_events_topic_configurable: True
state_topic_configurable: True
required_goal_event_fields_present: True
missing_goal_event_fields: []
required_state_fields_present: True
missing_state_fields: []
```

The reviewed fields include dispatch/outcome goal sequence, target, dispatch pose,
exit distance, timeout, Nav2 result reason/status, candidate branches, local-cost
sample age/bounds/coverage, timeout footprint/front/path-ahead costs, mode,
blocked/blacklisted counters, known junctions, edges, goal counters, and exit distance.

Result: PASS for bounded-smoke diagnostics schema sufficiency.

## New read-only check artifacts and tests

Added read-only analyzer/check script:

```text
tools/check_phase36_autonomous_readiness.py
```

Added tests:

```text
src/tugbot_maze/test/test_phase36_autonomous_readiness.py
```

These files only inspect existing source/config/docs. They do not alter Nav2 params,
maze strategy, launch behavior, or runtime state.

RED/GREEN evidence:

```text
Initial RED: 5 failing tests because tools/check_phase36_autonomous_readiness.py did not exist.
GREEN: python3 -m pytest -p no:cacheprovider src/tugbot_maze/test/test_phase36_autonomous_readiness.py -q -> 6 passed.
```

## Verification run

Commands run:

```bash
python3 -m py_compile \
  tools/check_phase36_autonomous_readiness.py \
  src/tugbot_maze/test/test_phase36_autonomous_readiness.py \
  src/tugbot_bringup/launch/tugbot_maze_slam.launch.py \
  src/tugbot_bringup/launch/tugbot_maze_slam_nav.launch.py \
  src/tugbot_bringup/launch/tugbot_maze_explore.launch.py

python3 -m pytest -p no:cacheprovider src/tugbot_maze/test/test_phase36_autonomous_readiness.py -q

python3 tools/check_phase36_autonomous_readiness.py --workspace-root . \
  --output log/phase36_autonomous_exploration_readiness_review/phase36_readiness_check.json

git diff -- src/tugbot_navigation/config

git diff -- \
  src/tugbot_bringup/launch/tugbot_maze_explore.launch.py \
  src/tugbot_bringup/launch/tugbot_maze_slam.launch.py \
  src/tugbot_bringup/launch/tugbot_maze_slam_nav.launch.py

ps -eo pid,args | grep -E 'ros2 launch|gz sim|rviz2|slam_toolbox|maze_explorer|frontier_explorer|controller_server|planner_server|bt_navigator|ros_gz_bridge|parameter_bridge|static_transform_publisher' | grep -v -E 'grep -E|hermes-snap|pgrep -af' || true
```

Observed results:

```text
py_compile analyzer/test: PASS
pytest phase36: 6 passed
git diff -- src/tugbot_navigation/config: empty
git diff -- current maze launch files: empty
relevant ROS/Gazebo/Nav2/explorer process check: empty
```

## Blockers

- MAZE_EXPLORE_ENTRANCE_EXIT_DEFAULTS_NOT_ACTIVE_SCALED2X: tugbot_maze_explore.launch.py still defaults entrance/exit to the old first-pass scaffold values instead of active scaled2x metadata.
- MAZE_EXPLORE_METADATA_DEFAULT_POINTS_TO_MISSING_LEGACY_FILE: maze_config default is maze_instance.yaml and the file is not present in active config; active metadata is maze_20260528_scaled_instance.yaml.

## Decision

```text
NOT_READY_WITH_BLOCKERS
```

Phase36 is complete as a readiness review. The active scaled2x world, current maze
world launch defaults, deprecated artifact exclusion, and diagnostic event/state
schemas are ready. However, default autonomous exploration is not ready because
`tugbot_maze_explore.launch.py` still passes old first-pass scaffold entrance/exit
values and a missing legacy metadata filename as defaults. Do not start bounded
autonomous exploration until a later phase aligns those defaults to
`maze_20260528_scaled_instance.yaml` / `doc/ACTIVE_MAZE_WORLD.md` and re-runs this
readiness check.

No autonomous exploration success is claimed. Stop here for human review.
