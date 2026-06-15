# Phase32B Clean Maze Scale-up and Visual Markers Report

Date: 2026-05-28
Workspace: `ros2_ws_tugbot_nav_20260522`
Status: `PASS_AS_SCALED_CLEAN_WORLD_CANDIDATE_GENERATED_NOT_PROMOTED`

## Scope

Phase32 visual acceptance found that the clean 2D maze wall pattern was basically
correct, but the world scale was too small for Tugbot relative to corridor width.
Phase32B therefore generates a new scaled candidate world from the existing clean
candidate world:

```text
base clean candidate:
src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean.sdf

new scaled candidate:
src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean_scaled2x.sdf
```

This phase changes only world geometry scale and visual floor markers. It does
not start navigation, does not promote the new world, and does not overwrite the
scaffold or unscaled clean candidate world.

## Guardrails held

- no Nav2/MPPI/controller parameter changes;
- no SLAM/Nav2/maze_explorer launched;
- no navigation strategy changes;
- no fallback / terminal-acceptance continuation;
- no candidate promotion;
- no overwrite of `src/tugbot_gazebo/worlds/tugbot_maze_world.sdf`;
- no overwrite of `src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean.sdf`;
- entrance arrow and finish band are visual-only and collision-free;
- no navigation conclusions made.

## Phase32 cleanup

The Phase32 visual-only Gazebo/RViz run was stopped before Phase32B generation.
Cleanup check after stopping residual visual processes was empty.

## Metadata

Added:

```text
src/tugbot_maze/config/maze_20260528_scaled_instance.yaml
```

Key values:

```yaml
source_image: package://tugbot_maze/assets/maze_20260528.png
base_world: src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean.sdf
output_world: src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean_scaled2x.sdf
scale_factor: 2.0
original_bounds_m:
  xmin: -6.0
  xmax: 6.0
  ymin: -6.0
  ymax: 6.0
scaled_bounds_m:
  xmin: -12.0
  xmax: 12.0
  ymin: -12.0
  ymax: 12.0
wall_thickness_policy:
  mode: keep_original_thickness
  wall_thickness_m: 0.24
wall_height_policy:
  mode: keep_original_height
  wall_height_m: 1.2
```

Policy rationale:

- XY wall centers and wall lengths are scaled by `2.0`.
- Wall thickness remains `0.24m` to increase usable corridor clearance.
- Wall height remains `1.2m` because visual wall height is already adequate.
- Entrance/exit candidate coordinates are scaled by `2.0`.

Scaled entrance/exit:

```text
entrance: (-6.0, -1.0, yaw=0) -> (-12.0, -2.0, yaw=0)
exit:     (0.0, 6.0)          -> (0.0, 12.0)
```

Exit radius is also scaled from `0.6m` to `1.2m` in the candidate SDF marker.

## Generator

Added:

```text
tools/generate_phase32b_scaled_clean_maze_world.py
```

Command:

```bash
python3 tools/generate_phase32b_scaled_clean_maze_world.py \
  --metadata src/tugbot_maze/config/maze_20260528_scaled_instance.yaml \
  --segments-yaml src/tugbot_maze/config/maze_wall_segments_20260528.yaml \
  --base-world src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean.sdf \
  --output-world src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean_scaled2x.sdf \
  --artifact-dir log/phase32b_scaled_clean_maze_world
```

Generator behavior:

1. parses the unscaled clean candidate SDF;
2. refuses to overwrite scaffold or unscaled clean candidate world;
3. scales all `maze_wall*` model center `x/y` coordinates by `2.0`;
4. scales each wall's long dimension by `2.0`;
5. keeps each wall's short dimension/thickness at `0.24m`;
6. keeps wall height at `1.2m`;
7. updates Tugbot include pose to `(-12.0, -2.0, yaw=0)`;
8. updates exit marker to `(0.0, 12.0)` with radius `1.2m`;
9. adds visual-only entrance arrow and finish band models;
10. writes overlay artifacts and JSON summary.

## Visual-only markers

Added to scaled SDF:

```text
maze_entrance_arrow_visual
maze_exit_finish_band_visual
```

Marker policy:

```yaml
visual_only: true
collision: none
z_m: 0.005
```

Entrance arrow:

```text
position: (-11.2, -2.0)
color: green
points toward +X / maze interior
design: shaft + head as flat box visuals
collision: none
```

Finish band:

```text
position: (0.0, 12.0)
style: black/white checker band
collision: none
```

The tests parse the SDF and confirm marker models contain visual elements and no
collision elements.

## Generated outputs

Scaled candidate world:

```text
src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean_scaled2x.sdf
```

Debug artifacts:

```text
log/phase32b_scaled_clean_maze_world/scaled_sdf_plan_view.png
log/phase32b_scaled_clean_maze_world/entrance_arrow_exit_finish_overlay.png
log/phase32b_scaled_clean_maze_world/scale_comparison_summary.json
```

Vision review of `entrance_arrow_exit_finish_overlay.png`:

- wall pattern is clearly visible;
- green entrance arrow is visible on the left side and points inward;
- black/white finish band is visible near the top-center exit area;
- no route/path/navigation overlays are present;
- it appears as a scaled world plan with visual markers only.

## Summary metrics

From:

```text
log/phase32b_scaled_clean_maze_world/scale_comparison_summary.json
```

Key values:

```json
{
  "status": "scaled_candidate_generated_not_promoted",
  "scale_factor": 2.0,
  "base_wall_count": 53,
  "scaled_wall_count": 53,
  "wall_thickness_m": 0.24,
  "wall_thickness_preserved": true,
  "wall_height_m": 1.2,
  "wall_height_preserved": true,
  "base_clean_world_preserved": true,
  "scaffold_world_preserved": true,
  "markers": {
    "visual_only": true,
    "collision_free": true,
    "z_m": 0.005
  }
}
```

Hash preservation:

```text
unscaled clean candidate before/after:
da33e9e58c6c07089b6bb7156315759b4bceb989db2f1ace18f0b1af0b0c0301

scaffold before/after:
5867385cd3eb72202c25e8bdd16c63591df6b947facada139d8fa8bab403d3a5
```

## Tests

Added:

```text
src/tugbot_maze/test/test_phase32b_scaled_clean_maze_world.py
```

Coverage:

- scaled metadata schema;
- scaled world generation without overwriting base/scaffold worlds;
- SDF parseability;
- wall count preserved from unscaled clean candidate;
- wall centers scaled by `2.0`;
- wall long dimensions scaled by `2.0`;
- wall thickness not scaled;
- wall height preserved;
- entrance pose scaled to `(-12.0, -2.0, yaw=0)`;
- exit finish band at `(0.0, 12.0)`;
- arrow and finish marker visual-only / no collision;
- debug artifacts and summary JSON generated.

RED evidence:

```text
5 failed
```

Expected initial failures:

- scaled metadata missing;
- generator missing;
- scaled SDF missing;
- marker/artifact/summary outputs missing.

GREEN evidence:

```text
5 passed in 0.32s
```

## Verification

Commands run:

```bash
python3 -m pytest src/tugbot_maze/test/test_phase32b_scaled_clean_maze_world.py -q
python3 -m py_compile \
  tools/generate_phase32b_scaled_clean_maze_world.py \
  src/tugbot_maze/test/test_phase32b_scaled_clean_maze_world.py
python3 tools/generate_phase32b_scaled_clean_maze_world.py \
  --metadata src/tugbot_maze/config/maze_20260528_scaled_instance.yaml \
  --segments-yaml src/tugbot_maze/config/maze_wall_segments_20260528.yaml \
  --base-world src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean.sdf \
  --output-world src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean_scaled2x.sdf \
  --artifact-dir log/phase32b_scaled_clean_maze_world
pgrep -af 'ros2 launch|gz sim|rviz2|maze_explorer|controller_server|planner_server|bt_navigator|slam_toolbox|ros_gz_bridge|parameter_bridge|static_transform_publisher'
git diff -- src/tugbot_navigation/config | cat
git diff -- src/tugbot_gazebo/worlds/tugbot_maze_world.sdf | cat
git diff -- src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean.sdf | cat
```

Results:

```text
focused Phase32B tests: 5 passed
py_compile: passed
generator run: passed
artifact/summary checks: passed
cleanup check: empty
src/tugbot_navigation/config diff: empty
scaffold world diff: empty
unscaled clean world diff: empty
```

## Limitations

- No autonomous navigation was run.
- No SLAM/Nav2/maze_explorer was run.
- No Gazebo visual smoke was launched in this phase beyond static artifact
  overlays; the next phase should visually inspect the scaled candidate in
  Gazebo/RViz.
- The arrow/finish visuals are flat box visuals, not texture decals. They are
  intentionally collision-free.
- The scaled candidate is not promoted and should not be treated as default.

## Conclusion

Phase32B result:

```text
PASS_AS_SCALED_CLEAN_WORLD_CANDIDATE_GENERATED_NOT_PROMOTED
```

A new 2x scaled clean maze candidate world has been generated with visual-only
entrance and finish markers. The original scaffold world and the unscaled clean
candidate world remain unchanged. No navigation stack was launched and no runtime
navigation conclusions were made.

Next required step:

```text
Phase33: Gazebo/RViz visual acceptance of src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean_scaled2x.sdf
```
