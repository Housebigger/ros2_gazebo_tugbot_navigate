# Phase33D Entrance/Exit Visual Marker Style Refinement Report

Date: 2026-05-28
Workspace: `ros2_ws_tugbot_nav_20260522`
Status: `PASS_AS_VISUAL_MARKER_STYLE_REFINED_NOT_PROMOTED`
Phase33C accepted inputs: corrected marker positions and Tugbot initial pose are correct.

## Scope

Phase33D performs style-only refinement on entrance/exit visual markers in:

```text
src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean_scaled2x.sdf
```

The phase does not alter:

- wall segment geometry;
- wall count;
- wall pose/size;
- `scale_factor`;
- entrance/exit coordinates;
- Tugbot initial pose;
- Nav2/MPPI/controller parameters;
- navigation strategy.

## Guardrails held

- no autonomous navigation launched;
- no SLAM/Nav2 exploration launched;
- no `maze_explorer` launched;
- no Nav2/MPPI/controller parameter changes;
- no fallback / terminal-acceptance continuation;
- no wall segment geometry changes;
- no `scale_factor` changes;
- no entrance/exit coordinate changes;
- no overwrite of `src/tugbot_gazebo/worlds/tugbot_maze_world.sdf`;
- no overwrite of `src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean.sdf`;
- no promotion of scaled candidate world;
- all marker style elements remain visual-only and collision-free;
- no navigation conclusion is made.

## Files changed

```text
tools/refine_phase33d_visual_marker_style.py
src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean_scaled2x.sdf
src/tugbot_maze/config/maze_20260528_scaled_instance.yaml
src/tugbot_maze/test/test_phase33d_visual_marker_style.py
doc/doc_report/phase33d_visual_marker_style_report.md
```

## Style changes

### Entrance marker

Model:

```text
maze_entrance_arrow_visual
```

Before Phase33D:

```text
visual_count: 2
visuals: shaft, head
geometry: box + box
problem: looked like a square/block in Gazebo
```

After Phase33D:

```text
style: green_isosceles_triangle_arrow
phase33d_style: triangle_arrow
pose unchanged: (-10.661, -9.025, 0.005, 0, 0, 0)
direction: +X into maze
collision: none
visual_count: 3
visuals:
  - triangle_arrow_tip
  - triangle_arrow_left_edge
  - triangle_arrow_right_edge
```

Implementation note:

The triangle arrow is built from three thin visual-only boxes: a small forward tip
and two angled edges. This avoids collision geometry and keeps the marker simple
for Gazebo while making the arrow read as a triangle rather than a square block.

### Exit finish band

Model:

```text
maze_exit_finish_band_visual
```

Before Phase33D:

```text
pose: (10.061, 9.058, 0.005, 0, 0, 0)
checker band at correct position but not rotated as requested
```

After Phase33D:

```text
style: rotated_black_white_checker_band
phase33d_style: rotated_checker_finish_band
position unchanged: (10.061, 9.058)
z: 0.005
yaw: 1.57079632679 rad
collision: none
checker tiles: 4 columns x 2 rows
opening width target: 2.005571 m
```

The checker band is now rotated 90 degrees and its long dimension is sized to
approximately span the detected right-side exit opening width.

### Exit marker

Model:

```text
maze_exit_marker
```

Before Phase33D:

```text
visible green circular/cylinder ground marker above finish band
```

After Phase33D:

```text
style: hidden_semantic_marker
visible_ground_disk: false
pose unchanged: (10.061, 9.058, 0.010, 0, 0, 0)
collision: none
visual: one tiny transparent 0.001m marker
```

The exit marker semantic model remains in SDF, but it no longer creates a visible
green disk that covers the finish band.

## Coordinates preserved

Tugbot:

```text
(-11.011, -9.025, 0.0, 0.0, 0.0, 0.0)
```

Entrance arrow:

```text
(-10.661, -9.025, 0.005, 0.0, 0.0, 0.0)
```

Exit finish band:

```text
(10.061, 9.058, 0.005, 0.0, 0.0, 1.57079632679)
```

Exit marker semantic pose:

```text
(10.061, 9.058, 0.010, 0.0, 0.0, 0.0)
```

Only the finish band yaw changed as requested. Entrance/exit x/y coordinates did
not change.

## Wall preservation

Summary:

```text
wall_count: 53
wall_geometry_unchanged: true
wall_segment_geometry_modified: false
```

Tests verify every scaled wall remains equivalent to the unscaled clean candidate
contract:

- center x/y remain 2x the unscaled clean wall centers;
- long dimensions remain 2x;
- wall thickness remains unchanged;
- wall height remains unchanged.

## Metadata updates

Metadata file:

```text
src/tugbot_maze/config/maze_20260528_scaled_instance.yaml
```

Updated style fields:

```yaml
markers:
  entrance_arrow:
    style: green_isosceles_triangle_arrow
    geometry: three_thin_visual_boxes_triangle_outline
  exit_finish_band:
    style: rotated_black_white_checker_band
    yaw_rad: 1.570796
    spans_exit_opening_width_m: 2.005571
  exit_marker:
    semantic_only: true
    visible_ground_disk: false
    style: hidden_semantic_marker
```

Existing entrance/exit coordinates remain unchanged.

## Debug artifacts

Artifact directory:

```text
log/phase33d_visual_marker_style/
```

Files:

```text
entrance_triangle_arrow_preview.png
exit_finish_band_rotated_preview.png
marker_style_summary.json
```

Visual preview assessment:

- entrance preview clearly reads as a green triangle arrow pointing +X, not a
  square block;
- exit preview clearly reads as a horizontal black/white checker band;
- preview explicitly notes the green circular ground marker is hidden.

## TDD

RED was observed first:

```text
5 failed
```

Expected RED reason:

```text
Phase33D marker style tool did not exist yet.
```

GREEN after implementation:

```text
5 passed in 0.29s
```

Focused test file:

```text
src/tugbot_maze/test/test_phase33d_visual_marker_style.py
```

Test coverage:

- wall count unchanged;
- wall pose/size unchanged;
- entrance/exit coordinates unchanged;
- entrance marker visual-only and collision-free;
- entrance marker no longer uses old square-like `shaft/head` visuals;
- entrance marker has triangle/arrow geometry features;
- finish band visual-only and collision-free;
- finish band yaw is 90 degrees;
- finish band spans approximately the detected exit width;
- green circular exit marker no longer creates a visible ground disk covering the
  finish band;
- scaffold/unscaled clean world preserved.

## Verification

Commands run:

```bash
python3 -m pytest src/tugbot_maze/test/test_phase33d_visual_marker_style.py -q
python3 -m py_compile tools/refine_phase33d_visual_marker_style.py src/tugbot_maze/test/test_phase33d_visual_marker_style.py
```

Results:

```text
focused Phase33D tests: 5 passed
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
log/phase33d_visual_marker_style/marker_style_summary.json
```

Important fields:

```json
{
  "status": "visual_marker_style_refined_not_promoted",
  "wall_count": 53,
  "wall_geometry_unchanged": true,
  "coordinates_unchanged": true,
  "entrance_marker_style": "green_isosceles_triangle_arrow",
  "finish_band_style": "rotated_black_white_checker_band",
  "exit_marker_visible_ground_disk_removed": true,
  "markers_visual_only": true,
  "marker_collision_count": 0,
  "candidate_not_promoted": true,
  "navigation_started": false
}
```

## Current status

```text
PASS_AS_VISUAL_MARKER_STYLE_REFINED_NOT_PROMOTED
```

Phase33D is complete and stopped for human acceptance. No Gazebo/RViz run is left active, and no navigation was launched.

Recommended next step:

```text
Phase33E or Phase34: visual-only Gazebo/RViz re-check of refined marker style
```
