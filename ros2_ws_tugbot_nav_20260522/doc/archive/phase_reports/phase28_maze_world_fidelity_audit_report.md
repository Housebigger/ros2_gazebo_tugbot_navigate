# Phase28 Maze Image to Gazebo World Fidelity Audit Report

Date: 2026-05-28
Workspace: `ros2_ws_tugbot_nav_20260522`
Status: `PASS_AS_WORLD_FIDELITY_ROOT_CAUSE_CONFIRMED`

## Scope

Phase27-alt follow-up is paused. Phase28 audits the fidelity of the maze image to
Gazebo world chain after manual observation showed that the Gazebo world has far
fewer walls than the original `maze_20260522.jpg` image.

This phase is diagnostics-only:

- no Nav2/MPPI tuning;
- no fallback / terminal-acceptance work;
- no navigation strategy changes;
- no long navigation run;
- no rewrite of the Gazebo world.

The Phase27-alt-R6 manual observation processes were stopped before this static
audit. Cleanup check found no remaining ROS/Gazebo/RViz/Nav2/SLAM/explorer
processes.

## Input / output files found

Source maze image:

```text
src/tugbot_maze/assets/maze_20260522.jpg
```

Original copied source noted in Phase0:

```text
tmp_resources/maze_20260522.jpg
```

Within the current workspace root this `tmp_resources` path is not present; the
active archived source image is the package asset above.

Current Gazebo world:

```text
src/tugbot_gazebo/worlds/tugbot_maze_world.sdf
```

Metadata:

```text
src/tugbot_maze/config/maze_instance.yaml
```

World utility:

```text
src/tugbot_maze/tugbot_maze/maze_image_to_world.py
```

Phase0 provenance:

```text
doc/doc_report/phase0_maze_workspace_baseline_report.md
```

Debug artifact directory:

```text
log/phase28_maze_world_fidelity/
```

Analyzer:

```text
tools/audit_phase28_maze_world_fidelity.py
```

## Actual generation chain found

The expected chain in the user request was:

```text
maze_20260522.jpg
  -> processed mask
  -> contour / segments
  -> SDF / world walls
  -> Gazebo world
```

The actual current chain is:

```text
maze_20260522.jpg
  -> archived as a reference asset
  -> metadata says source_image_kind: reference_3d_illustration
  -> conversion.mode: manual_simplified_first_pass
  -> no existing threshold / mask / contour / skeleton stage
  -> hand-written simplified SDF scaffold
  -> Gazebo world
```

Evidence:

- `maze_image_to_world.py` is explicitly a placeholder.
- Its header says the JPEG is a 3D/shadowed reference image, not a clean occupancy
  map.
- It says a later phase should replace the placeholder with cleaned/manual wall
  extraction that writes `tugbot_maze_world.sdf`.
- `maze_instance.yaml` records `conversion.mode: manual_simplified_first_pass`.
- `phase0_maze_workspace_baseline_report.md` records that the world is a first
  hand-written simplified maze scaffold.

Therefore, the root cause is not a bad threshold or contour filter in an existing
image-processing pipeline. The automated image-to-world extraction pipeline does
not exist yet for this workspace.

Explicit audit statement: The automated image-to-world extraction pipeline does not exist yet.

## Current SDF wall inventory

Parsed `src/tugbot_gazebo/worlds/tugbot_maze_world.sdf` wall models:

```text
maze_wall_bottom
maze_wall_top
maze_wall_left
maze_wall_right
maze_wall_i01
maze_wall_i02
maze_wall_i03
maze_wall_i04
maze_wall_i05
```

Counts:

```json
{
  "maze_wall_model_count_total": 9,
  "outer_wall_count": 4,
  "inner_wall_count": 5
}
```

Area ratios over the 12m x 12m scaffold world:

```json
{
  "sdf_wall_area_m2": 11.214,
  "sdf_wall_area_ratio_vs_12x12": 0.077875,
  "sdf_inner_wall_area_m2": 2.574,
  "sdf_inner_wall_area_ratio_vs_12x12": 0.017875,
  "sdf_outer_wall_area_m2": 8.64
}
```

Interpretation:

- The world has only 5 inner walls.
- Most SDF wall area is outer-frame area, not dense interior maze structure.
- This matches the manual observation: Gazebo shows only a small scaffold, not the
  original complex maze.

## Source image static diagnostics

The source image is a complex decorative maze illustration:

- 1000 x 1000 JPEG, RGB.
- White raised maze walls, shadows, bevels, watermarks, and gradients.
- Not a clean black/white occupancy map.

Generated diagnostic image metrics:

```json
{
  "orthogonal_hough_segment_count": 72,
  "otsu_dark_connected_components": {
    "component_count": 68,
    "component_area_sum_px": 174822
  },
  "adaptive_dark_connected_components": {
    "component_count": 344,
    "component_area_sum_px": 153332
  },
  "wall_mask_light_proxy_area_ratio": 0.828574,
  "interior_wall_mask_area_ratio": 0.7578202947845805
}
```

Important caveat:

The high wall-mask area ratios are diagnostic only. Because the source image has
white walls, gray shadows, bevels, and watermarks, simple threshold masks can
merge floor, wall tops, shadows, and background. The useful evidence is not that
one threshold is already production-ready; it is that the source contains many
orthogonal visual structures while the SDF contains only 5 inner walls.

Vision check of the segment overlay confirms many detected source-image segments,
but with expected caveats:

- shadows may be detected as walls;
- detections may be offset to visual edges rather than wall centerlines;
- watermarks can create false positives or breaks;
- segment detections are fragmented and need consolidation.

## Debug image artifacts

All requested static artifacts were generated under:

```text
log/phase28_maze_world_fidelity/
```

Files:

```text
00_source_image.png
01_cropped_maze_region.png
02_grayscale.png
03_threshold_binary_otsu_dark.png
04_threshold_binary_otsu_light.png
05_threshold_binary_adaptive_dark.png
06_wall_mask_light_proxy.png
07_contour_overlay.png
08_generated_wall_segments_overlay_static_detected.png
09_final_world_footprint_overlay_on_source.png
10_current_sdf_wall_plan_view.png
phase28_metrics.json
```

Requested artifact mapping:

- cropped maze region:
  - `01_cropped_maze_region.png`
- grayscale:
  - `02_grayscale.png`
- threshold / binary mask:
  - `03_threshold_binary_otsu_dark.png`
  - `04_threshold_binary_otsu_light.png`
  - `05_threshold_binary_adaptive_dark.png`
- wall mask:
  - `06_wall_mask_light_proxy.png`
- contour overlay:
  - `07_contour_overlay.png`
- generated wall segments overlay:
  - `08_generated_wall_segments_overlay_static_detected.png`
- final world footprint overlay on source image:
  - `09_final_world_footprint_overlay_on_source.png`
- SDF-only plan view:
  - `10_current_sdf_wall_plan_view.png`

## Final world overlay assessment

The current SDF wall overlay on the source image does not cover the source maze
interior wall pattern well.

Visual assessment:

- The SDF overlay contains the outer frame plus a sparse connected set of red
  scaffold-like interior wall boxes.
- It does not trace the many branches, corners, dead-end-like segments, and
  nested passages visible across the original image.
- Most source-image interior maze structure remains unrepresented.

This directly explains the user's manual observation that Gazebo contains only a
few walls while the original image is a complex maze.

## Coordinate / scale audit

Metadata says:

```text
resolution_m_per_pixel: 0.02
source_image_width_px: 1000
source_image_height_px: 1000
```

This implies a full image width of:

```text
1000 px * 0.02 m/px = 20 m
```

Current SDF scaffold wall extent is:

```text
x: -6.0 .. 6.0
 y: -6.0 .. 6.0
width: 12 m
```

Entrance / exit metadata and current world positions:

```json
{
  "entrance_m": [-4.0, -3.0],
  "exit_m": [4.0, 3.0]
}
```

Interpretation:

- Entrance and exit are internally consistent with the manual scaffold world.
- They are not proven to be correctly mapped to the original 1000px image via a
  real image-to-world transform, because the current scaffold did not come from
  a pixel-space extraction chain.
- There is a scale mismatch between metadata-implied image width (20m) and current
  scaffold width (12m). That is another sign that the current world is a manual
  engineering scaffold rather than an image-faithful conversion.

## Direct answers to audit questions

### Why did many walls disappear?

They were never generated from the image. The current world is a hand-written
simplified scaffold with 5 inner walls. `maze_image_to_world.py` is a placeholder,
not an implemented extractor.

### Is this a threshold / crop / contour / skeleton filtering bug?

Not in the current active chain. There is no active image-processing chain with
threshold/mask/contour/skeleton filtering. The issue is missing extraction / using
manual scaffold as if it were image-faithful.

### Does the current Gazebo world mostly contain outer frame / few inner walls?

Yes. SDF has 4 outer walls and 5 inner walls. Inner-wall area is only about
`1.7875%` of the 12m x 12m world area.

### Is the original image complex?

Yes. Static diagnostics detected many orthogonal visual segments (`72` Hough
segments) and vision inspection describes a complex decorative maze, not a simple
scaffold.

### Are entrance / exit still correct?

They are correct for the manual scaffold coordinates used by the running system:

```text
entrance = (-4.0, -3.0)
exit = (4.0, 3.0)
radius = 0.6
```

They are not validated against an image-derived coordinate transform because no
such transform is currently implemented.

## Root-cause hypothesis

Confirmed root cause:

```text
The project intentionally started with a manual simplified first-pass world.
The placeholder image-to-world utility never converted maze_20260522.jpg into
mask/contour/segments/SDF geometry. Phase27 behavior was therefore observed in a
simplified scaffold world, not in the complex source maze.
```

## Recommended next direction, not executed in Phase28

Do not adjust Nav2/MPPI/fallback based on the current scaffold mismatch.

A later phase should choose one of these world-fidelity routes before returning to
navigation behavior work:

1. Clean/manual vectorization route:
   - remove watermark/background;
   - define crop and pixel-to-meter transform;
   - manually or semi-automatically trace wall centerlines/polygons;
   - generate SDF boxes or mesh walls;
   - validate wall count and overlay fidelity.

2. Automated extraction route:
   - crop maze region;
   - normalize lighting / suppress shadows;
   - segment wall tops rather than shadows;
   - consolidate orthogonal segments;
   - reject watermark false positives;
   - generate candidate SDF;
   - compare overlays before runtime.

3. Hybrid route:
   - use static detector output only as a guide;
   - manually approve / correct wall segments;
   - then generate SDF.

Phase28 does not implement these routes.

## Verification performed

Commands / checks:

```bash
python3 tools/audit_phase28_maze_world_fidelity.py \
  --source-image src/tugbot_maze/assets/maze_20260522.jpg \
  --world src/tugbot_gazebo/worlds/tugbot_maze_world.sdf \
  --output-dir log/phase28_maze_world_fidelity

python3 -m py_compile tools/audit_phase28_maze_world_fidelity.py

pgrep -af 'ros2 launch|gz sim|rviz2|maze_explorer|controller_server|planner_server|bt_navigator|slam_toolbox|record_explorer_state_series.py'

git diff -- src/tugbot_navigation/config | cat
```

Results:

- analyzer completed successfully;
- debug image artifacts written;
- `phase28_metrics.json` written;
- py_compile passed;
- cleanup check empty;
- `src/tugbot_navigation/config` diff empty.

## Conclusion

Phase28 audit result:

```text
DIAGNOSTIC_AUDIT_COMPLETE_PENDING_HUMAN_ACCEPTANCE
```

The Gazebo world mismatch is real and explained by project provenance: the current
world is a deliberately hand-written simplified first-pass scaffold, not a product
of image mask/contour/segment extraction. The large wall loss is therefore not a
navigation failure and not an MPPI/fallback issue. Further navigation conclusions
should be paused until the world-fidelity route is chosen and validated.
