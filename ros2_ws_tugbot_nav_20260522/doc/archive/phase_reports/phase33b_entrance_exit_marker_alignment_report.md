# Phase33B Scaled Clean World Entrance/Exit Marker Alignment Report

Date: 2026-05-28
Workspace: `ros2_ws_tugbot_nav_20260522`
Status: `PASS_AS_MARKER_ALIGNMENT_CORRECTED_NOT_PROMOTED`

## Scope

Phase33B corrects only the scaled clean candidate world's entrance/exit marker
alignment after Phase33 human visual acceptance found the wall pattern and 2x
scale acceptable but marker placement offset.

Updated candidate world:

```text
src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean_scaled2x.sdf
```

Updated metadata:

```text
src/tugbot_maze/config/maze_20260528_scaled_instance.yaml
```

This phase intentionally does not change maze wall geometry, wall count,
wall thickness, wall height, scale factor, Nav2 parameters, or navigation
strategy.

## Guardrails held

- no autonomous navigation launched;
- no SLAM/Nav2 exploration launched;
- no `maze_explorer` launched;
- no Nav2/MPPI/controller parameter changes;
- no fallback / terminal-acceptance continuation;
- no wall segment geometry changes;
- no scale factor changes;
- no overwrite of `src/tugbot_gazebo/worlds/tugbot_maze_world.sdf`;
- no overwrite of `src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean.sdf`;
- no promotion of scaled candidate world as default;
- markers remain visual-only and collision-free.

## Root-cause audit

Phase33B no longer uses the previous scaled candidate assumption:

```text
old entrance: (-12.0, -2.0, yaw=0)
old exit:     (0.0, 12.0)
```

Instead, the opening detector reads:

```text
src/tugbot_maze/config/maze_wall_segments_20260528.yaml
```

It detects gaps on the outer boundary by:

1. identifying the outer wall bounding box from `outer: true` wall segments;
2. collecting covered intervals on top/bottom/left/right boundary sides;
3. subtracting covered intervals from each side to find gaps;
4. converting pixel-gap centers to Phase32B 2x scaled world coordinates.

Detected boundary openings:

```text
1. left opening
   center_px:    (29, 314.5)
   center_world: (-10.061281337, -9.025069638)
   width_m:      2.072423398
   inward:       +X

2. right opening
   center_px:    (330, 44.0)
   center_world: (10.061281337, 9.058495822)
   width_m:      2.005571031
   inward:       -X
```

Selection policy:

- entrance: detected left/lower-side opening, because it is the obvious left-side
  entry gap and points inward along +X;
- exit: detected right/upper-side opening, because it is the opposing right-side
  boundary gap and is the most plausible exit after rejecting the old `(0, 12)`
  assumption.

Exit ambiguity note:

```text
The detector found one right-side upper opening and one left-side lower opening.
The right-side upper opening is selected as the exit candidate. This is the most
reasonable candidate from boundary topology, but still requires human visual
confirmation in Gazebo/RViz.
```

## Corrected metadata values

Metadata file:

```text
src/tugbot_maze/config/maze_20260528_scaled_instance.yaml
```

Entrance is now aligned to the detected left opening:

```yaml
entrance:
  status: candidate_aligned_to_detected_boundary_opening
  opening_side: left
  opening_center_x_m: -10.061281
  opening_center_y_m: -9.02507
  opening_width_m: 2.072423
  inward_direction: +X
  x_m: -11.011281
  y_m: -9.02507
  yaw_rad: 0.0
  marker_x_m: -10.661281
  marker_y_m: -9.02507
```

Exit is now aligned to the detected right opening:

```yaml
exit:
  status: candidate_aligned_to_detected_boundary_opening
  opening_side: right
  opening_center_x_m: 10.061281
  opening_center_y_m: 9.058496
  opening_width_m: 2.005571
  inward_direction: -X
  x_m: 10.061281
  y_m: 9.058496
  radius_m: 1.2
  ambiguity_note: selected right-side upper opening from detected boundary openings; requires human confirmation
```

## Corrected SDF marker poses

Updated SDF:

```text
src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean_scaled2x.sdf
```

Tugbot initial pose was moved to the corrected entrance outside the left opening:

```text
tugbot_pose: (-11.011281337, -9.025069638, yaw=0)
```

Entrance arrow:

```text
model: maze_entrance_arrow_visual
pose:  (-10.661281337, -9.025069638, 0.005, 0, 0, 0)
direction: +X into maze
collision: none
```

Exit finish band:

```text
model: maze_exit_finish_band_visual
pose:  (10.061281337, 9.058495822, 0.005, 0, 0, 0)
collision: none
```

Exit marker:

```text
model: maze_exit_marker
pose:  (10.061281337, 9.058495822, 0.010, 0, 0, 0)
collision: none
```

## Wall preservation

The alignment tool captured wall geometry before and after marker updates and
failed if any wall pose or size changed.

Result:

```text
wall_count_before: 53
wall_count_after:  53
wall_count_unchanged: true
wall_geometry_unchanged: true
```

The tests also re-compare the scaled world against the unscaled clean world's
wall geometry contract:

- every `maze_wall*` model still exists;
- wall centers remain exactly 2x the unscaled clean candidate centers;
- wall long dimensions remain exactly 2x;
- wall thickness remains unchanged;
- wall height remains unchanged.

## Debug artifacts

Artifact directory:

```text
log/phase33b_entrance_exit_marker_alignment/
```

Files:

```text
boundary_openings_overlay.png
corrected_marker_overlay.png
corrected_scaled_sdf_plan_view.png
phase33b_marker_alignment_summary.json
```

Overlay visual check:

- left entrance/right exit opening markers are visible;
- green arrow is placed at the lower-left/left opening and points +X inward;
- finish band is placed at the upper-right/right opening;
- actual wall gaps are subtle in the simplified overlay and still need human
  Gazebo confirmation.

## TDD

RED was observed before implementation:

```text
5 failed
```

Expected RED reason:

```text
Phase33B alignment tool did not exist yet.
```

GREEN after implementation:

```text
5 passed in 0.38s
```

Focused test file:

```text
src/tugbot_maze/test/test_phase33b_entrance_exit_marker_alignment.py
```

Test coverage:

- boundary openings detected from `maze_wall_segments_20260528.yaml`;
- metadata entrance/exit candidates aligned to detected openings;
- wall count unchanged;
- wall poses/sizes unchanged;
- marker visual-only / no collision;
- entrance arrow located near detected left entrance opening;
- entrance arrow direction points +X inward;
- exit finish band located near detected right exit opening;
- finish band within world bounds;
- Tugbot pose updated to corrected entrance;
- scaffold world and unscaled clean world remain preserved.

## Verification

Commands run:

```bash
python3 -m pytest src/tugbot_maze/test/test_phase33b_entrance_exit_marker_alignment.py -q
python3 -m py_compile tools/align_phase33b_entrance_exit_markers.py src/tugbot_maze/test/test_phase33b_entrance_exit_marker_alignment.py
```

Results:

```text
focused Phase33B tests: 5 passed
py_compile: passed
artifact/metadata/SDF checks: passed
cleanup check: empty
```

Preservation checks:

```text
git diff -- src/tugbot_gazebo/worlds/tugbot_maze_world.sdf
# empty

git diff -- src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean.sdf
# empty

git diff -- src/tugbot_navigation/config
# empty
```

Hash preservation from summary:

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
log/phase33b_entrance_exit_marker_alignment/phase33b_marker_alignment_summary.json
```

Important fields:

```json
{
  "status": "marker_alignment_corrected_not_promoted",
  "wall_count_before": 53,
  "wall_count_after": 53,
  "wall_count_unchanged": true,
  "wall_geometry_unchanged": true,
  "markers": {
    "visual_only": true,
    "collision_free": true
  },
  "selected_entrance_opening": {
    "side": "left",
    "center_world": [-10.061281337047355, -9.025069637883007],
    "width_m": 2.0724233983286915,
    "inward": "+X"
  },
  "selected_exit_opening": {
    "side": "right",
    "center_world": [10.061281337047355, 9.05849582172702],
    "width_m": 2.00557103064067,
    "inward": "-X"
  }
}
```

## Current status

```text
PASS_AS_MARKER_ALIGNMENT_CORRECTED_NOT_PROMOTED
```

Phase33B is complete and stopped for human acceptance. No Gazebo/RViz run is
left active and no navigation was launched.

Recommended next step, if accepted:

```text
Phase33C or Phase34: visual-only Gazebo/RViz re-check of corrected markers in the scaled clean world
```
