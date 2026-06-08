# Phase35-pre Map/World Artifact Cleanup and Active Clean World Selection Report

Date: 2026-05-28
Workspace: `ros2_ws_tugbot_nav_20260522`
Status: `PASS_AS_ACTIVE_CLEAN_WORLD_SELECTED_AND_OLD_ARTIFACTS_ARCHIVED`

## Goal

Clean up old and misleading map/world/image artifacts without irreversible deletion,
and make the Phase34 human-accepted scaled clean maze world the active candidate
for the current workflow.

Active world selected:

```text
src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean_scaled2x.sdf
```

Human acceptance trail:

```text
Phase33F: PASS_AS_SCALED_CLEAN_WORLD_VISUALLY_ACCEPTED
Phase34:  PASS_AS_SCALED_CLEAN_WORLD_SLAM_DATAFLOW_AND_MAP_VISUAL_ACCEPTED
```

## Guardrails held

- no Nav2/MPPI/controller parameter changes;
- no navigation launch/run;
- no autonomous exploration;
- no Nav2/manual goal;
- no fallback / terminal-acceptance continuation;
- no irreversible deletion; deprecated artifacts were moved to archive locations;
- Phase28~Phase34 reports were not deleted.

## Active core files retained

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

## Inventory

Generated inventory artifacts:

```text
log/phase35_pre_map_world_artifact_cleanup/phase35_pre_artifact_inventory.json
log/phase35_pre_map_world_artifact_cleanup/phase35_pre_artifact_inventory.md
```

The scan listed old world/map/image artifacts and old default references,
including:

- old scaffold world;
- Phase29 decorative-image candidate world;
- unscaled clean candidate world;
- `maze_20260522.jpg` decorative route artifacts;
- old static maps and old explored maps;
- old launch/default references.

## Archive actions

Deprecated artifacts were moved, not deleted, under:

```text
archive/deprecated_maze_worlds/
doc/archive/phase_old_maze_artifacts/
```

Archive manifest:

```text
archive/deprecated_maze_worlds/phase35_pre_deprecated_artifact_manifest.json
archive/deprecated_maze_worlds/README.md
```

Key archived sources:

```text
src/tugbot_gazebo/worlds/tugbot_maze_world.sdf
  -> archive/deprecated_maze_worlds/worlds/manual_simplified_first_pass_scaffold/tugbot_maze_world.sdf

src/tugbot_gazebo/worlds/tugbot_maze_world_image_faithful.sdf
  -> archive/deprecated_maze_worlds/worlds/phase29_decorative_image_candidate/tugbot_maze_world_image_faithful.sdf

src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean.sdf
  -> archive/deprecated_maze_worlds/worlds/unscaled_clean_candidate/tugbot_maze_world_20260528_clean.sdf

src/tugbot_maze/assets/maze_20260522.jpg
  -> archive/deprecated_maze_worlds/assets/decorative_route_20260522/maze_20260522.jpg

src/tugbot_navigation/maps/map_1725111373.{yaml,pgm}
  -> archive/deprecated_maze_worlds/maps/static_legacy/

src/tugbot_navigation/maps/explored/*.yaml / *.pgm
  -> archive/deprecated_maze_worlds/maps/explored_legacy/
```

Deprecated old debug artifact directories moved:

```text
log/phase28_maze_world_fidelity
  -> doc/archive/phase_old_maze_artifacts/phase28_maze_world_fidelity

log/phase29_maze_world_reconstruction
  -> doc/archive/phase_old_maze_artifacts/phase29_maze_world_reconstruction

log/phase30_candidate_world_visual_acceptance
  -> doc/archive/phase_old_maze_artifacts/phase30_candidate_world_visual_acceptance
```

## README / status documentation

Updated:

```text
README.md
doc/ACTIVE_MAZE_WORLD.md
```

The documentation now explicitly states:

- `maze_20260522.jpg` decorative route deprecated;
- `manual_simplified_first_pass` scaffold deprecated for navigation conclusions;
- unscaled clean candidate superseded by scaled2x world;
- legacy static/explored maps should not be silently used as current maze workflow defaults;
- active world is `tugbot_maze_world_20260528_clean_scaled2x.sdf`;
- Phase35-pre does not claim navigation success.

## Launch/default updates

Updated current maze workflow launch defaults to use the active scaled2x world:

```text
src/tugbot_bringup/launch/tugbot_maze_slam.launch.py
src/tugbot_bringup/launch/tugbot_maze_slam_nav.launch.py
src/tugbot_bringup/launch/tugbot_maze_explore.launch.py
```

Default world now resolves to:

```text
tugbot_maze_world_20260528_clean_scaled2x.sdf
```

Notes:

- `tugbot_maze_slam.launch.py` remains the current recommended smoke entry.
- `tugbot_maze_slam_nav.launch.py` and `tugbot_maze_explore.launch.py` were default-updated only to avoid silently using old scaffold worlds in later explicitly allowed phases. They were not launched in Phase35-pre.
- Non-maze inherited launch files such as `tugbot_slam.launch.py`, `tugbot_slam_nav.launch.py`, `tugbot_explore.launch.py`, and legacy static map replay launch files were not repurposed; README marks them as legacy/non-current workflow.

## Verification

Check artifact:

```text
log/phase35_pre_map_world_artifact_cleanup/phase35_pre_checks.txt
log/phase35_pre_map_world_artifact_cleanup/phase35_pre_reclosure_checks.txt
```

Results:

```text
phase35-pre checks passed
```

Active world exists:

```text
OK src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean_scaled2x.sdf
```

Active core source/config exists:

```text
OK src/tugbot_maze/assets/maze_20260528.png
OK src/tugbot_maze/config/maze_20260528_scaled_instance.yaml
OK src/tugbot_maze/config/maze_wall_segments_20260528.yaml
```

Phase34 accepted map snapshot artifacts retained:

```text
OK log/phase34_scaled_clean_world_slam_smoke/phase34_scaled_clean_world_slam_smoke_map_snapshot.yaml
OK log/phase34_scaled_clean_world_slam_smoke/phase34_scaled_clean_world_slam_smoke_map_snapshot.pgm
```

Deprecated source artifacts archived from active source locations:

```text
ARCHIVED_FROM_SOURCE src/tugbot_gazebo/worlds/tugbot_maze_world.sdf
ARCHIVED_FROM_SOURCE src/tugbot_gazebo/worlds/tugbot_maze_world_image_faithful.sdf
ARCHIVED_FROM_SOURCE src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean.sdf
ARCHIVED_FROM_SOURCE src/tugbot_maze/assets/maze_20260522.jpg
ARCHIVED_FROM_SOURCE src/tugbot_navigation/maps/map_1725111373.yaml
ARCHIVED_FROM_SOURCE src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase14_perimeter_no_spin.yaml
```

Archive presence verified:

```text
OK_ARCHIVE archive/deprecated_maze_worlds/worlds/manual_simplified_first_pass_scaffold/tugbot_maze_world.sdf
OK_ARCHIVE archive/deprecated_maze_worlds/worlds/phase29_decorative_image_candidate/tugbot_maze_world_image_faithful.sdf
OK_ARCHIVE archive/deprecated_maze_worlds/worlds/unscaled_clean_candidate/tugbot_maze_world_20260528_clean.sdf
OK_ARCHIVE archive/deprecated_maze_worlds/assets/decorative_route_20260522/maze_20260522.jpg
OK_ARCHIVE archive/deprecated_maze_worlds/maps/static_legacy/map_1725111373.yaml
OK_ARCHIVE archive/deprecated_maze_worlds/maps/explored_legacy/tugbot_nav_world_slam_phase14_perimeter_no_spin.yaml
```

Current maze launch defaults verified:

```text
src/tugbot_bringup/launch/tugbot_maze_slam.launch.py:
  tugbot_maze_world_20260528_clean_scaled2x.sdf

src/tugbot_bringup/launch/tugbot_maze_slam_nav.launch.py:
  tugbot_maze_world_20260528_clean_scaled2x.sdf

src/tugbot_bringup/launch/tugbot_maze_explore.launch.py:
  tugbot_maze_world_20260528_clean_scaled2x.sdf
```

No current maze launch file still silently defaults to the old scaffold world:

```text
old scaffold references in current maze launch files: empty
```

Nav config diff is empty:

```text
git diff -- src/tugbot_navigation/config
# empty
```

Cleanup check is empty:

```text
ros2 launch / gz sim / rviz2 / slam_toolbox / maze_explorer / controller_server / ros_gz_bridge / static_transform_publisher: no live process matches
```

cleanup check is empty: verified.

Syntax checks:

```text
python3 -m py_compile \
  src/tugbot_bringup/launch/tugbot_maze_slam.launch.py \
  src/tugbot_bringup/launch/tugbot_maze_slam_nav.launch.py \
  src/tugbot_bringup/launch/tugbot_maze_explore.launch.py

phase35-pre launch py_compile/default checks passed
```

## Limitations / caution

- This phase intentionally changes current maze launch defaults but does not run them.
- This phase does not validate Nav2 behavior on the active scaled2x world.
- This phase does not promote the world to a generic default outside the maze workflow.
- Archived old maps/worlds remain available for provenance/regression reference.
- Some historical reports still mention old artifact paths as history; those reports were preserved rather than rewritten.

## Phase35-pre human acceptance reclosure

After the next-session handoff, Phase35-pre was rechecked and accepted as the
active clean-world selection.

Reclosure artifact:

```text
log/phase35_pre_map_world_artifact_cleanup/phase35_pre_reclosure_checks.txt
```

Reclosure result:

```text
PASS_AS_ACTIVE_CLEAN_WORLD_SELECTED_AND_OLD_ARTIFACTS_ARCHIVED
```

Verified again:

- active world/source/config/map snapshot artifacts exist;
- current maze SLAM / SLAM+Nav2 / explore launch defaults point to
  `tugbot_maze_world_20260528_clean_scaled2x.sdf`;
- current maze launch defaults do not silently reference the old scaffold world;
- `git diff -- src/tugbot_navigation/config` is empty;
- launch syntax checks pass;
- cleanup process check is empty.

## Result

```text
PASS_AS_ACTIVE_CLEAN_WORLD_SELECTED_AND_OLD_ARTIFACTS_ARCHIVED
```

Phase35-pre cleanup and active-world selection are complete and accepted as the
active clean-world baseline for the next phase. No navigation was started in
Phase35-pre, no Nav2/MPPI/controller parameters were modified, and no
fallback/terminal acceptance work continued.
