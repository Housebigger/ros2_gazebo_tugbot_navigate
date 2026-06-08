# Phase33E Visual Marker Orientation Correction Report

Date: 2026-05-28
Workspace: `ros2_ws_tugbot_nav_20260522`
Status: `PASS_AS_VISUAL_MARKER_ORIENTATION_CORRECTED_NOT_PROMOTED`

## Scope

Phase33E performs yaw-only orientation correction for refined entrance/exit visual markers in:

```text
src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean_scaled2x.sdf
```

Human Gazebo review confirmed before this phase:

- scaled clean maze walls and scale are correct;
- entrance/exit marker positions are correct;
- marker styles are correct;
- only marker orientation needs correction.

This phase changes only:

- `maze_entrance_arrow_visual` yaw: `current_yaw + pi`;
- `maze_exit_finish_band_visual` yaw: `current_yaw + pi/2`.

No marker x/y/z, Tugbot pose, wall geometry, scale factor, or collision policy is changed.

## Guardrails held

- no autonomous navigation launched;
- no SLAM/Nav2 exploration launched;
- no `maze_explorer` launched;
- no Nav2/MPPI/controller parameter changes;
- no fallback / terminal-acceptance continuation;
- no wall segment geometry changes;
- no `scale_factor` changes;
- no entrance/exit coordinate changes;
- no Tugbot initial pose changes;
- no overwrite of scaffold world;
- no overwrite of unscaled clean world;
- no promotion of scaled candidate world;
- marker models remain visual-only and collision-free.

## Files changed

```text
tools/correct_phase33e_visual_marker_orientation.py
src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean_scaled2x.sdf
src/tugbot_maze/config/maze_20260528_scaled_instance.yaml
src/tugbot_maze/test/test_phase33e_visual_marker_orientation.py
doc/doc_report/phase33e_visual_marker_orientation_report.md
```

## Orientation correction

Yaw normalization used in summary/report:

```text
[-pi, pi]
```

Entrance arrow:

```text
model: maze_entrance_arrow_visual
style: triangle_arrow
x/y/z unchanged: (-10.661, -9.025, 0.005)
roll/pitch unchanged: 0.0 / 0.0
yaw_before: 0.0
yaw_delta:  3.141592653589793
yaw_after:  3.141592653589793
```

Exit finish band:

```text
model: maze_exit_finish_band_visual
style: rotated_checker_finish_band
x/y/z unchanged: (10.061, 9.058, 0.005)
roll/pitch unchanged: 0.0 / 0.0
yaw_before: 1.571
yaw_delta:  1.5707963267948966
yaw_after:  -3.1413889803846904
```

Note on exit yaw:

```text
1.571 + pi/2 = 3.14179632679, normalized to [-pi, pi] as -3.14138898038.
```

Exit semantic marker:

```text
model: maze_exit_marker
pose unchanged: (10.061, 9.058, 0.010, 0.0, 0.0, 0.0)
style remains: hidden_semantic_marker
collision: none
```

Tugbot initial pose unchanged:

```text
-11.011 -9.025 0.000 0.000 0.000 0.000
```

## Metadata update

Metadata file:

```text
src/tugbot_maze/config/maze_20260528_scaled_instance.yaml
```

New metadata section:

```yaml
markers:
  phase33e_orientation_correction:
    status: orientation_corrected_not_promoted
    coordinates_modified: false
    tugbot_pose_modified: false
    wall_geometry_modified: false
    scale_factor_modified: false
    collision_policy_modified: false
    entrance_arrow:
      yaw_before: 0.0
      yaw_delta: 3.141593
      yaw_after: 3.141593
      normalization: '[-pi, pi]'
    exit_finish_band:
      yaw_before: 1.571
      yaw_delta: 1.570796
      yaw_after: -3.141389
      normalization: '[-pi, pi]'
```

Existing marker style fields remain intact.

## Wall / position preservation

Summary:

```text
wall_count: 53
wall_geometry_unchanged: true
marker_xyz_unchanged: true
tugbot_pose_unchanged: true
markers_visual_only: true
marker_collision_count: 0
```

Preserved:

- all `maze_wall*` model names/count;
- all `maze_wall*` pose/size values;
- all marker x/y/z fields;
- marker roll/pitch fields;
- Tugbot initial pose;
- exit marker pose;
- marker no-collision policy;
- scale factor.

## Artifacts

Artifact directory:

```text
log/phase33e_visual_marker_orientation/
```

Files:

```text
marker_orientation_summary.json
entrance_arrow_orientation_preview.png
exit_finish_band_orientation_preview.png
```

Preview assessment:

- entrance preview communicates a 180-degree rotation about marker center with
  x/y/z unchanged;
- exit preview communicates a 90-degree rotation about marker center with x/y/z
  unchanged;
- exit preview uses normalized `-3.141` after-yaw; this is equivalent to the
  wrapped result of `before + pi/2`.

## TDD

RED was observed first:

```text
5 failed
```

Expected RED reason:

```text
Phase33E marker orientation correction tool did not exist yet.
```

GREEN after implementation:

```text
5 passed in 0.30s
```

Focused test file:

```text
src/tugbot_maze/test/test_phase33e_visual_marker_orientation.py
```

Test coverage:

- wall count unchanged;
- wall pose/size unchanged;
- marker x/y/z unchanged;
- Tugbot pose unchanged;
- entrance arrow yaw rotated by `pi`;
- exit finish band yaw rotated by `pi/2`;
- markers remain visual-only and collision-free;
- metadata records yaw before/after;
- scaffold/unscaled clean world hashes preserved;
- artifacts generated.

## Verification

Commands run:

```bash
python3 -m pytest src/tugbot_maze/test/test_phase33e_visual_marker_orientation.py -q
python3 -m py_compile tools/correct_phase33e_visual_marker_orientation.py src/tugbot_maze/test/test_phase33e_visual_marker_orientation.py
```

Results:

```text
focused Phase33E orientation tests: 5 passed
py_compile: passed
artifact/metadata/SDF checks: passed
cleanup check: empty
```

Preservation diffs:

```text
git diff -- src/tugbot_gazebo/worlds/tugbot_maze_world.sdf
# empty

git diff -- src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean.sdf
# empty

git diff -- src/tugbot_navigation/config
# empty
```

Hash preservation:

```text
scaffold before/after:
5867385cd3eb72202c25e8bdd16c63591df6b947facada139d8fa8bab403d3a5
5867385cd3eb72202c25e8bdd16c63591df6b947facada139d8fa8bab403d3a5

unscaled clean before/after:
da33e9e58c6c07089b6bb7156315759b4bceb989db2f1ace18f0b1af0b0c0301
da33e9e58c6c07089b6bb7156315759b4bceb989db2f1ace18f0b1af0b0c0301
```

## Summary JSON

Primary machine-readable summary:

```text
log/phase33e_visual_marker_orientation/marker_orientation_summary.json
```

Important fields:

```json
{
  "status": "orientation_corrected_not_promoted",
  "wall_count": 53,
  "wall_geometry_unchanged": true,
  "marker_xyz_unchanged": true,
  "tugbot_pose_unchanged": true,
  "markers_visual_only": true,
  "candidate_not_promoted": true,
  "navigation_started": false
}
```

## Current status

```text
PASS_AS_VISUAL_MARKER_ORIENTATION_CORRECTED_NOT_PROMOTED
```

Phase33E orientation correction is complete and stopped for human acceptance. No
Gazebo/RViz run is left active and no navigation was launched.

Recommended next step:

```text
Visual-only Gazebo/RViz re-check of corrected marker orientation
```
