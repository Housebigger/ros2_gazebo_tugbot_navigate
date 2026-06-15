# Phase31 Clean 2D Maze Image Import and 3D Gazebo World Generation Report

Date: 2026-05-28
Workspace: `ros2_ws_tugbot_nav_20260522`
Status: `PASS_AS_CLEAN_2D_IMAGE_WORLD_CANDIDATE_GENERATED_NOT_PROMOTED`

## Scope

Phase31 pauses the Phase29 decorative-image candidate route and treats the new
clean 2D maze image as the source candidate for world generation:

```text
/home/hyh/Desktop/agent_playground/playground_hermes/ros2_gazebo_tugbot_navigate/tmp_resources/maze_20260528.png
```

This phase only imports the clean image, extracts walls, writes an editable wall
segment YAML, generates a separate candidate SDF world, creates overlay artifacts,
and runs static tests. It does not run autonomous navigation and does not promote
any world.

## Guardrails held

- no Nav2/MPPI/controller parameter changes;
- no fallback / terminal-acceptance continuation;
- no navigation strategy changes;
- no autonomous navigation launched;
- no SLAM/Nav2/explorer launched;
- no overwrite of `src/tugbot_gazebo/worlds/tugbot_maze_world.sdf`;
- no promotion of the new world as default;
- Phase29 candidate world preserved;
- original scaffold world preserved;
- Phase31 is limited to image import, wall extraction, SDF generation, overlay
  verification, and host-level tests.

## Source image import

Archived clean image:

```text
src/tugbot_maze/assets/maze_20260528.png
```

SHA256 matches the user-provided tmp resource:

```text
173704e201da6f60042f1f39cc84220d9ee7592a3b2934f933bdeeaf9d223c63
```

Image inspection:

```text
format: PNG
size: 360 x 360 px
mode: grayscale-compatible clean image
black bbox: [29, 29, 330, 330]
unique grayscale levels: 8
wall mask ratio at threshold 128: 0.0798225 before close / 0.0805401 after close
```

Vision inspection summary:

- clean high-contrast 2D orthogonal maze;
- white background;
- black wall lines;
- no decorative shadows/watermarks;
- suitable for binary wall extraction;
- entrance/exit are not explicitly labeled, so Phase31 records them as candidate
  positions only.

## Metadata config

Added:

```text
src/tugbot_maze/config/maze_20260528_instance.yaml
```

Important metadata:

```yaml
source_image: package://tugbot_maze/assets/maze_20260528.png
source_image_kind: clean_2d_binary_maze
conversion:
  mode: clean_binary_image_to_wall_segments
  candidate_not_promoted: true
  do_not_overwrite_scaffold_world: true
world_bounds_m:
  xmin: -6.0
  xmax: 6.0
  ymin: -6.0
  ymax: 6.0
```

Scale policy:

```text
Use a 12m candidate extent, x=-6..6 and y=-6..6, for continuity with current
Tugbot maze workspace assumptions. This is still a candidate scale pending
Gazebo visual acceptance.
```

Transform:

```text
x_m = xmin + x_px / (image_width_px - 1) * (xmax - xmin)
y_m = ymax - y_px / (image_height_px - 1) * (ymax - ymin)
```

Initial candidate entrance/exit:

```yaml
entrance:
  status: candidate
  x_m: -6.0
  y_m: -1.0
  yaw_rad: 0.0
exit:
  status: candidate
  x_m: 0.0
  y_m: 6.0
  radius_m: 0.6
```

Important caveat:

- These positions are candidates only.
- They are not force-carried from `maze_20260522.jpg`.
- The entrance appears plausible as a left-side opening.
- The exit marker is ambiguous in overlay because no explicit source-image label
  exists and the top opening/exit intent needs human visual acceptance.

## Generator

Added:

```text
tools/generate_phase31_clean_maze_world.py
```

Generator command:

```bash
python3 tools/generate_phase31_clean_maze_world.py \
  --metadata src/tugbot_maze/config/maze_20260528_instance.yaml \
  --output-segments src/tugbot_maze/config/maze_wall_segments_20260528.yaml \
  --output-world src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean.sdf \
  --artifact-dir log/phase31_clean_maze_world_generation
```

Extraction logic:

1. load clean PNG;
2. convert to grayscale;
3. threshold dark pixels as walls with threshold 128;
4. close small antialias gaps using a 3x3 morphological close;
5. scan horizontal and vertical wall-pixel runs;
6. group adjacent run rows/columns into centerline segments;
7. merge nearby collinear intervals;
8. filter very short segments using `min_segment_length_px: 12`;
9. classify likely outer frame vs inner walls;
10. write editable segment YAML;
11. convert segments to uniform-height Gazebo box walls;
12. generate candidate SDF;
13. write overlay artifacts and summary JSON.

## Generated outputs

Editable wall segment YAML:

```text
src/tugbot_maze/config/maze_wall_segments_20260528.yaml
```

Candidate SDF world:

```text
src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean.sdf
```

This candidate world is not promoted and does not replace:

```text
src/tugbot_gazebo/worlds/tugbot_maze_world.sdf
```

It also preserves Phase29 candidate:

```text
src/tugbot_gazebo/worlds/tugbot_maze_world_image_faithful.sdf
```

## Generation metrics

From:

```text
log/phase31_clean_maze_world_generation/phase31_generation_summary.json
```

Summary:

```json
{
  "status": "candidate_generated_not_promoted",
  "source_image_kind": "clean_2d_binary_maze",
  "image_width_px": 360,
  "image_height_px": 360,
  "binary_threshold": 128,
  "wall_mask_black_pixel_ratio": 0.08054012345679012,
  "wall_model_count_total": 53,
  "inner_wall_count": 49,
  "outer_wall_count": 4,
  "baseline_scaffold_preserved": true,
  "phase29_candidate_preserved": true
}
```

Hash preservation:

```text
scaffold before/after:
5867385cd3eb72202c25e8bdd16c63591df6b947facada139d8fa8bab403d3a5

Phase29 candidate before/after:
b4ced2a422fd976f2a9a12455e06d6626526b1cebfc748b0c4820f5149c423dc
```

## Debug / overlay artifacts

Artifact directory:

```text
log/phase31_clean_maze_world_generation/
```

Files:

```text
00_source_image.png
01_binary_wall_mask.png
02_detected_wall_segments_overlay.png
03_generated_sdf_plan_view.png
04_sdf_overlay_on_source_image.png
05_entrance_exit_overlay.png
phase31_generation_summary.json
```

Vision review of `04_sdf_overlay_on_source_image.png`:

- extracted wall segment overlay aligns well with the clean black 2D maze lines;
- outer boundary is detected as a blue rectangular frame;
- internal red segments match major horizontal and vertical wall runs;
- no obvious large extra floating segments;
- no obvious missing major internal wall section;
- expected limitations remain: endpoint simplification at T-junctions/corners and
  loss of exact raster thickness.

Vision review of `05_entrance_exit_overlay.png`:

- entrance candidate is within world bounds and plausibly aligned with a left-side
  opening, but marker is outside the drawn wall frame as an entrance annotation;
- exit candidate is within world bounds but ambiguous/questionable because no
  explicit source label exists and the top boundary opening is not clearly marked;
- both require Phase32 Gazebo visual acceptance before use as runtime truth.

## Tests

Added:

```text
src/tugbot_maze/test/test_phase31_clean_maze_world_generation.py
```

Covered:

- clean image asset exists;
- metadata schema correctness;
- wall segment YAML schema correctness;
- generated SDF existence and contract;
- wall count above lower bound;
- horizontal/vertical wall-only contract;
- scaffold world not overwritten;
- debug artifacts all generated;
- entrance/exit candidates inside world bounds;
- generator preserves scaffold hash;
- generator records guardrails.

RED evidence:

```text
5 failed
```

Initial failures were expected:

- image asset had not yet been archived;
- metadata did not exist;
- generator did not exist;
- wall YAML/SDF/artifacts did not exist.

GREEN evidence:

```text
5 passed in 0.35s
```

## Verification

Commands run:

```bash
python3 -m pytest src/tugbot_maze/test/test_phase31_clean_maze_world_generation.py -q
python3 -m py_compile \
  tools/generate_phase31_clean_maze_world.py \
  src/tugbot_maze/test/test_phase31_clean_maze_world_generation.py
python3 tools/generate_phase31_clean_maze_world.py \
  --metadata src/tugbot_maze/config/maze_20260528_instance.yaml \
  --output-segments src/tugbot_maze/config/maze_wall_segments_20260528.yaml \
  --output-world src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean.sdf \
  --artifact-dir log/phase31_clean_maze_world_generation
pgrep -af 'ros2 launch|gz sim|rviz2|maze_explorer|controller_server|planner_server|bt_navigator|slam_toolbox|ros_gz_bridge|parameter_bridge|static_transform_publisher'
git diff -- src/tugbot_navigation/config | cat
git diff -- src/tugbot_gazebo/worlds/tugbot_maze_world.sdf | cat
git diff -- src/tugbot_gazebo/worlds/tugbot_maze_world_image_faithful.sdf | cat
```

Results:

```text
focused Phase31 tests: 5 passed
py_compile: passed
generator run: passed
artifact/summary checks: passed
cleanup check: empty
src/tugbot_navigation/config diff: empty
scaffold world diff: empty
Phase29 candidate world diff: empty
```

## Limitations

- No Gazebo visual acceptance was performed in Phase31.
- No navigation was launched.
- Entrance/exit are candidate annotations, not accepted runtime truth.
- Wall thickness is uniform `0.24m` and wall height is uniform `1.2m`.
- Extraction simplifies raster line thickness into rectilinear box walls.
- Very small endpoint deviations can occur at corners/T-junctions.
- Candidate scale uses `x/y=-6..6` for continuity and still requires visual review.

## Conclusion

Phase31 result:

```text
PASS_AS_CLEAN_2D_IMAGE_WORLD_CANDIDATE_GENERATED_NOT_PROMOTED
```

The clean image `maze_20260528.png` is now archived as a clean source candidate,
converted into editable wall-segment YAML, and used to generate a separate
candidate Gazebo SDF world. The original scaffold world and Phase29 candidate
world are preserved. No navigation was run and no default world was promoted.

Next required step:

```text
Phase32: Gazebo visual acceptance of src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean.sdf
```
