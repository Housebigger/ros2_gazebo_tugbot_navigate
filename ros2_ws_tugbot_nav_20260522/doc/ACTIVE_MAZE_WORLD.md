# Tugbot Maze Active World Status

> **Note (2026-06-15):** this is the Phase35-pre *world-selection* record. The world it
> names below is still the active one, but its "does not claim navigation success" framing
> is historical — navigation has since been completed reliably via Guided Corridor
> Navigation (4/4). See `../README.md` for the current navigation status and run commands.

Updated: 2026-05-28
Phase: Phase35-pre Map/World Artifact Cleanup and Active Clean World Selection
Status: `PASS_AS_ACTIVE_CLEAN_WORLD_SELECTED_AND_OLD_ARTIFACTS_ARCHIVED`

## Active candidate world

The active Gazebo world for the current Tugbot maze workflow is:

```text
src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean_scaled2x.sdf
```

Human acceptance trail:

```text
Phase33F: PASS_AS_SCALED_CLEAN_WORLD_VISUALLY_ACCEPTED
Phase34:  PASS_AS_SCALED_CLEAN_WORLD_SLAM_DATAFLOW_AND_MAP_VISUAL_ACCEPTED
```

Use this world explicitly for smoke/recheck wrappers and as the default for current
maze SLAM launch entries. This selection does not claim navigation success.

## Active core files to keep

```text
src/tugbot_maze/assets/maze_20260528.png
src/tugbot_maze/config/maze_20260528_scaled_instance.yaml
src/tugbot_maze/config/maze_wall_segments_20260528.yaml
src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean_scaled2x.sdf
log/phase34_scaled_clean_world_slam_smoke/phase34_scaled_clean_world_slam_smoke_map_snapshot.yaml
log/phase34_scaled_clean_world_slam_smoke/phase34_scaled_clean_world_slam_smoke_map_snapshot.pgm
log/phase34_scaled_clean_world_slam_smoke/phase34_scaled_clean_world_slam_smoke_map_snapshot_after_cmd_pulse.yaml
log/phase34_scaled_clean_world_slam_smoke/phase34_scaled_clean_world_slam_smoke_map_snapshot_after_cmd_pulse.pgm
```

## Deprecated routes

The following artifacts/routes are deprecated for current navigation conclusions:

- `maze_20260522.jpg` decorative route is deprecated. It was a shaded/decorative
  reference image, not a clean occupancy-map source for current navigation conclusions.
- `manual_simplified_first_pass` scaffold world is deprecated for navigation
  conclusions. It may remain in archive for provenance/regression reference only.
- The unscaled clean candidate world is superseded by the scaled2x world because
  the scaled2x world passed visual and SLAM/map human acceptance.
- Legacy static/explored maps from the pre-clean/scaffold workflows are archived
  and should not be silently used as current maze workflow defaults.

Archived deprecated artifacts live under:

```text
archive/deprecated_maze_worlds/
doc/archive/phase_old_maze_artifacts/
```

The manifest is:

```text
archive/deprecated_maze_worlds/phase35_pre_deprecated_artifact_manifest.json
```

## Launch defaults updated in Phase35-pre

Current maze workflow launch defaults now point at the active scaled2x world:

```text
src/tugbot_bringup/launch/tugbot_maze_slam.launch.py
src/tugbot_bringup/launch/tugbot_maze_slam_nav.launch.py
src/tugbot_bringup/launch/tugbot_maze_explore.launch.py
```

Guardrail: Phase35-pre did not start navigation and did not modify Nav2/MPPI/controller parameters.
