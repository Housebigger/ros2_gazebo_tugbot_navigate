# Phase36B Active Maze Metadata Wiring Fix Report

Date: 2026-05-28 12:52:18 UTC

## Scope

Phase36B fixed the two Phase36 readiness blockers by wiring the autonomous exploration launch defaults to the active scaled2x maze metadata.

This phase did not start `maze_explorer`, did not start autonomous exploration, did not run Gazebo/Nav2, and did not change navigation strategy or Nav2/MPPI/controller parameters.

## Inputs

- Phase36 human acceptance: PASS with conclusion preserved as `NOT_READY_WITH_BLOCKERS`.
- Active world: `src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean_scaled2x.sdf`
- Active metadata: `src/tugbot_maze/config/maze_20260528_scaled_instance.yaml`
- Required active values:
  - entrance_x = -11.011281
  - entrance_y = -9.025070
  - entrance_yaw = 0.0
  - exit_x = 10.061281
  - exit_y = 9.058496
  - exit_radius = 1.2
  - maze_config = maze_20260528_scaled_instance.yaml

## Files changed

1. `src/tugbot_bringup/launch/tugbot_maze_explore.launch.py`
   - `maze_config` default changed from the missing legacy `maze_instance.yaml` to `maze_20260528_scaled_instance.yaml`.
   - `entrance_x`, `entrance_y`, `entrance_yaw`, `exit_x`, `exit_y`, and `exit_radius` defaults changed from first-pass scaffold values to the active scaled2x values from `maze_20260528_scaled_instance.yaml`.
   - Launch argument descriptions now state that these defaults mirror the active scaled2x metadata.

2. `src/tugbot_maze/test/test_phase36_autonomous_readiness.py`
   - Focused readiness tests were updated for Phase36B so they assert the launch defaults are wired to the active scaled2x metadata.
   - The readiness test now expects `READY_FOR_BOUNDED_AUTONOMOUS_EXPLORATION_SMOKE` and zero blockers after the wiring fix.

No Nav2 config files were modified.

## TDD evidence

The tests were changed first and run before the launch wiring fix. The expected RED state was observed:

```text
Phase36B TDD RED check (expected failure before launch wiring fix)

Command:
python3 -m pytest -p no:cacheprovider src/tugbot_maze/test/test_phase36_autonomous_readiness.py -q

Observed result:
2 failed, 4 passed in 0.38s

Expected failing assertions after updating tests but before modifying tugbot_maze_explore.launch.py:
1. test_phase36b_explorer_launch_defaults_are_wired_to_active_scaled2x_metadata
   AssertionError: metadata_file_declared was 'maze_instance.yaml' instead of 'maze_20260528_scaled_instance.yaml'
2. test_phase36b_readiness_is_ready_after_active_metadata_wiring_fix
   AssertionError: decision was 'NOT_READY_WITH_BLOCKERS' instead of 'READY_FOR_BOUNDED_AUTONOMOUS_EXPLORATION_SMOKE'

This was the intended RED state for the Phase36B wiring fix.
```

After the minimal launch default fix, the focused tests passed.

## Readiness analyzer result

Machine-readable analyzer output:

- `log/phase36b_active_maze_metadata_wiring_fix/phase36b_readiness_check.json`
- `log/phase36b_active_maze_metadata_wiring_fix/phase36b_readiness_check.stdout.json`

Analyzer decision:

```text
READY_FOR_BOUNDED_AUTONOMOUS_EXPLORATION_SMOKE
```

Blockers:

```text
0
```

Explorer active truth:

```json
{
  "expected_from_metadata": {
    "entrance_x": -11.011281,
    "entrance_y": -9.02507,
    "entrance_yaw": 0.0,
    "exit_radius": 1.2,
    "exit_x": 10.061281,
    "exit_y": 9.058496
  },
  "launch_defaults": {
    "entrance_x": "-11.011281",
    "entrance_y": "-9.025070",
    "entrance_yaw": "0.0",
    "exit_radius": "1.2",
    "exit_x": "10.061281",
    "exit_y": "9.058496"
  },
  "launch_file": "src/tugbot_bringup/launch/tugbot_maze_explore.launch.py",
  "metadata_file_declared": "maze_20260528_scaled_instance.yaml",
  "metadata_file_exists": true,
  "source_status": "launch_arguments_not_metadata_loaded",
  "uses_active_metadata_defaults": true
}
```

## Verification

Commands run:

```bash
python3 -m py_compile   tools/check_phase36_autonomous_readiness.py   src/tugbot_maze/test/test_phase36_autonomous_readiness.py   src/tugbot_bringup/launch/tugbot_maze_explore.launch.py

python3 -m pytest -p no:cacheprovider src/tugbot_maze/test/test_phase36_autonomous_readiness.py -q

python3 tools/check_phase36_autonomous_readiness.py --workspace-root . --output log/phase36b_active_maze_metadata_wiring_fix/phase36b_readiness_check.json

python3 tools/check_phase36_autonomous_readiness.py --workspace-root . --json > log/phase36b_active_maze_metadata_wiring_fix/phase36b_readiness_check.stdout.json

git diff -- src/tugbot_navigation/config

ps -eo pid,args | grep -E 'ros2 launch|gz sim|rviz2|slam_toolbox|maze_explorer|frontier_explorer|controller_server|planner_server|bt_navigator|ros_gz_bridge|parameter_bridge|static_transform_publisher' | grep -v -E 'grep -E|hermes-snap|pgrep -af' || true
```

Observed summary:

```text
Phase36B verification summary
timestamp_utc=2026-05-28T12:50:28Z
focused_tests=python3 -m pytest -p no:cacheprovider src/tugbot_maze/test/test_phase36_autonomous_readiness.py -q -> 6 passed
py_compile=tools/check_phase36_autonomous_readiness.py src/tugbot_maze/test/test_phase36_autonomous_readiness.py src/tugbot_bringup/launch/tugbot_maze_explore.launch.py -> PASS
readiness_decision=READY_FOR_BOUNDED_AUTONOMOUS_EXPLORATION_SMOKE
readiness_blockers=0
metadata_file_declared=maze_20260528_scaled_instance.yaml
metadata_file_exists=True
uses_active_metadata_defaults=True
launch_defaults={"entrance_x": "-11.011281", "entrance_y": "-9.025070", "entrance_yaw": "0.0", "exit_radius": "1.2", "exit_x": "10.061281", "exit_y": "9.058496"}
nav2_config_diff_empty=true
cleanup_check_pycache_pytest_cache_empty=true
process_check_relevant_ros_gazebo_nav2_explorer_empty=true
```

Additional static cleanup/reference checks:

- Search in `tugbot_maze_explore.launch.py` for legacy active defaults (`maze_instance.yaml`, `-4.0`, `-3.0`, `4.0`, `3.0`, `0.6` in the relevant launch-argument default patterns): no matches.
- Authorized cleanup removed cache directories listed in `log/phase36b_active_maze_metadata_wiring_fix/cache_dirs_before_cleanup_authorized.txt`.
- Post-cleanup check `log/phase36b_active_maze_metadata_wiring_fix/cache_dirs_after_cleanup_authorized.txt`: empty.
- Relevant ROS/Gazebo/Nav2/explorer process check: empty.

## Guardrail confirmation

- Nav2/MPPI/controller parameters: unchanged; `git diff -- src/tugbot_navigation/config` was empty.
- Navigation strategy: unchanged.
- Fallback / terminal acceptance: unchanged.
- `maze_explorer`: not started.
- Autonomous exploration: not started.
- Long run: not started.
- Old scaffold world/map: not used as current launch default.
- Autonomous exploration success: not claimed.

## Conclusion

Phase36B fixed the Phase36 active-metadata wiring blockers. The readiness analyzer now returns:

```text
READY_FOR_BOUNDED_AUTONOMOUS_EXPLORATION_SMOKE
```

This is only a readiness result for a future bounded smoke. It is not evidence that autonomous exploration has run or succeeded.

Stop here for human review. Do not enter Phase37 until explicitly instructed.
