# Phase29 Maze World Reconstruction Report

Date: 2026-05-28
Workspace: `ros2_ws_tugbot_nav_20260522`
Status: `PASS_AS_HYBRID_IMAGE_FAITHFUL_CANDIDATE_GENERATED_NOT_PROMOTED`
Phase28 accepted status: `PASS_AS_WORLD_FIDELITY_ROOT_CAUSE_CONFIRMED`

## Scope

Phase29 pauses Phase27-alt, Nav2/MPPI, fallback, terminal-acceptance, and
navigation-strategy work. The goal is to build a candidate Gazebo maze world that
is substantially closer to `maze_20260522.jpg` than the current
`manual_simplified_first_pass` scaffold.

Guardrails held:

- no Nav2/MPPI/controller parameter tuning;
- no fallback / terminal-acceptance continuation;
- no navigation strategy changes;
- no long navigation run;
- no automatic detector dump used directly as final world;
- no overwrite of `src/tugbot_gazebo/worlds/tugbot_maze_world.sdf`;
- current scaffold world preserved as baseline/reference;
- candidate world generated separately and not promoted.

## Phase28 context confirmed

Phase28 report was updated after human acceptance:

```text
doc/doc_report/phase28_maze_world_fidelity_audit_report.md
Status: PASS_AS_WORLD_FIDELITY_ROOT_CAUSE_CONFIRMED
```

Phase28 root cause remains:

```text
maze_20260522.jpg was archived as a reference asset, but the active Gazebo world
was a hand-written simplified scaffold. No implemented threshold/mask/contour/
skeleton/segment extraction chain existed. The scaffold contained only 5 inner
walls and was not image-faithful.
```

Phase29 therefore reconstructs world geometry before returning to navigation
behavior work.

## Hybrid reconstruction design

Phase29 uses a hybrid/manual-reviewed flow:

```text
maze_20260522.jpg
  -> Phase28 crop / grayscale / threshold / segment overlays reviewed
  -> manual visual review of major orthogonal wall centerlines
  -> human-editable segment YAML
  -> deterministic SDF generator
  -> candidate world SDF
  -> overlay artifacts for human review
```

Important policy:

- The YAML is not a raw automatic detector dump.
- The Phase28 detector overlay was used only as a guide.
- The segments were manually filtered / organized into major horizontal and
  vertical wall runs.
- Candidate world is not promoted automatically.

## New files

Human-editable wall segment YAML:

```text
src/tugbot_maze/config/maze_wall_segments_20260522.yaml
```

Generator:

```text
tools/generate_phase29_image_faithful_world.py
```

Candidate world:

```text
src/tugbot_gazebo/worlds/tugbot_maze_world_image_faithful.sdf
```

Host-level tests:

```text
src/tugbot_maze/test/test_phase29_maze_world_reconstruction.py
```

Overlay / summary artifacts:

```text
log/phase29_maze_world_reconstruction/
```

## Transform / scale policy

Phase29 chooses an explicit candidate transform, not hidden implicit scaling.

YAML transform:

```yaml
image_width_px: 1000
image_height_px: 1000
coordinate_origin: top_left
world_bounds_m:
  xmin: -6.0
  xmax: 6.0
  ymin: -6.0
  ymax: 6.0
mapping:
  x_m: xmin + x_pct / 100.0 * (xmax - xmin)
  y_m: ymax - y_pct / 100.0 * (ymax - ymin)
```

Rationale:

- Phase28 found the metadata `0.02 m/px` implies 20m image width.
- The current scaffold and launch assumptions use a 12m `-6..6` maze span.
- Phase29 keeps the 12m candidate span for continuity with the existing entrance,
  exit, and launch assumptions, while making the transform explicit and auditable.

This is a candidate transform, not final ground truth.

## Entrance / exit mapping

Candidate entrance:

```json
{
  "pixel_pct": [16.666666666666668, 75.0],
  "x_m": -4.0,
  "y_m": -3.0,
  "yaw_rad": 0.0
}
```

Candidate exit:

```json
{
  "pixel_pct": [83.33333333333333, 25.0],
  "x_m": 4.0,
  "y_m": 3.0,
  "radius_m": 0.6
}
```

Tested consistency:

- `(50%, 50%) -> (0.0, 0.0)`
- `(0%, 100%) -> (-6.0, -6.0)`
- `(100%, 0%) -> (6.0, 6.0)`
- entrance pixel percentage maps exactly to `(-4.0, -3.0)`
- exit pixel percentage maps exactly to `(4.0, 3.0)`

Vision review of the entrance/exit overlay confirms the markers are plausibly
lower-left and upper-right respectively, but still candidate transform evidence,
not validated ground truth.

## Wall segment YAML summary

The YAML contains:

```text
review_status: hybrid_manual_reviewed_candidate
source_image: src/tugbot_maze/assets/maze_20260522.jpg
wall_defaults.thickness_m: 0.24
wall_defaults.height_m: 1.2
```

Segment inventory:

```text
outer wall segments: 4
inner wall segments: 59
total wall segments: 63
```

This is intentionally much denser than the scaffold's 5 inner walls, while still
remaining editable and reviewable.

## Candidate generation result

Generator command:

```bash
python3 tools/generate_phase29_image_faithful_world.py \
  --segments-yaml src/tugbot_maze/config/maze_wall_segments_20260522.yaml \
  --output-world src/tugbot_gazebo/worlds/tugbot_maze_world_image_faithful.sdf \
  --artifact-dir log/phase29_maze_world_reconstruction
```

Generated summary:

```json
{
  "wall_model_count_total": 63,
  "outer_wall_count": 4,
  "inner_wall_count": 59,
  "scaffold_inner_wall_baseline_count": 5,
  "improvement_inner_wall_count_delta": 54,
  "inner_wall_centerline_length_m": 103.8,
  "total_wall_centerline_length_m": 136.92000000000002,
  "baseline_scaffold_preserved": true
}
```

Candidate world:

```text
src/tugbot_gazebo/worlds/tugbot_maze_world_image_faithful.sdf
```

The original scaffold remains unchanged:

```text
src/tugbot_gazebo/worlds/tugbot_maze_world.sdf
```

SHA256 before/after scaffold guard:

```text
5867385cd3eb72202c25e8bdd16c63591df6b947facada139d8fa8bab403d3a5
```

The before and after hash match.

## Overlay artifacts

Artifacts written to:

```text
log/phase29_maze_world_reconstruction/
```

Files:

```text
phase29_source_with_wall_segment_overlay.png
phase29_generated_sdf_plan_view.png
phase29_sdf_overlay_on_source_image.png
phase29_entrance_exit_overlay.png
phase29_generation_summary.json
```

Artifact meaning:

- `phase29_source_with_wall_segment_overlay.png`
  - source image with manually reviewed wall segments overlaid;
- `phase29_generated_sdf_plan_view.png`
  - candidate SDF wall centerline plan view;
- `phase29_sdf_overlay_on_source_image.png`
  - candidate SDF/segment overlay blended onto source image;
- `phase29_entrance_exit_overlay.png`
  - same overlay with candidate entrance and exit markers;
- `phase29_generation_summary.json`
  - deterministic metrics and guardrail status.

Vision review of `phase29_sdf_overlay_on_source_image.png`:

- Phase29 overlay substantially covers the major maze structure compared with the
  sparse Phase28 scaffold overlay.
- It includes many internal horizontal/vertical wall runs in upper, central,
  lower, left, and right regions.
- It is still approximate: some small wall stubs, tiny dead ends, exact wall
  centers, bevels, and image-shadow effects remain simplified or omitted.

## Tests added

```text
src/tugbot_maze/test/test_phase29_maze_world_reconstruction.py
```

Covered contracts:

1. YAML schema and manual-review policy:
   - schema version;
   - source image;
   - `review_status: hybrid_manual_reviewed_candidate`;
   - `do_not_overwrite_scaffold_world: true`;
   - transform bounds;
   - wall defaults;
   - segment count and percent-coordinate validation.

2. Image/world transform consistency:
   - center and corners;
   - entrance and exit exact mapping.

3. Candidate world generation:
   - output SDF exists;
   - candidate world name is image-faithful;
   - wall count is much larger than scaffold;
   - overlay artifacts are written.

4. No overwrite of scaffold world:
   - scaffold hash unchanged before/after generation.

5. Segment pose/size contract:
   - each generated wall has pose and box size;
   - height and thickness match YAML defaults.

RED evidence:

```text
5 failed
```

Initial failure reason:

- YAML did not exist;
- generator did not exist.

GREEN evidence:

```text
5 passed in 1.35s
```

## Verification performed

Commands:

```bash
python3 -m pytest src/tugbot_maze/test/test_phase29_maze_world_reconstruction.py -q
python3 -m py_compile \
  tools/generate_phase29_image_faithful_world.py \
  tools/audit_phase28_maze_world_fidelity.py \
  src/tugbot_maze/test/test_phase29_maze_world_reconstruction.py
python3 tools/generate_phase29_image_faithful_world.py \
  --segments-yaml src/tugbot_maze/config/maze_wall_segments_20260522.yaml \
  --output-world src/tugbot_gazebo/worlds/tugbot_maze_world_image_faithful.sdf \
  --artifact-dir log/phase29_maze_world_reconstruction
pgrep -af 'ros2 launch|gz sim|rviz2|maze_explorer|controller_server|planner_server|bt_navigator|slam_toolbox|record_explorer_state_series.py'
git diff -- src/tugbot_navigation/config | cat
git diff -- src/tugbot_gazebo/worlds/tugbot_maze_world.sdf | cat
```

Results:

```text
focused Phase29 tests: 5 passed
py_compile: passed
phase29 summary static checks: passed
phase29 artifact checks: passed
cleanup check: empty
src/tugbot_navigation/config diff: empty
scaffold world diff: empty
```

No runtime navigation was launched.

## Limitations / not promoted

This is not a final world promotion.

Known limitations:

- the source image is decorative, shadowed, beveled, and watermarked;
- segments are approximate centerlines, not full wall polygons;
- some short caps, small stubs, and exact dead-end geometry are omitted;
- wall thickness is uniform `0.24m`, not image-derived per-wall width;
- 12m scale is chosen for continuity, not proven as physical ground truth;
- entrance/exit markers are plausible candidate mappings, not final validated
  world truth;
- no Gazebo visual check or navigation run was performed in Phase29.

## Conclusion

Phase29 result:

```text
PASS_AS_HYBRID_IMAGE_FAITHFUL_CANDIDATE_GENERATED_NOT_PROMOTED
```

The candidate world is substantially more image-faithful than the current scaffold
by wall-count and overlay evidence:

```text
scaffold inner walls: 5
candidate inner walls: 59
candidate delta: +54 inner walls
```

The candidate is generated beside the scaffold, with overlay artifacts for human
review. It is not promoted, and no navigation behavior conclusions should be made
from it until a later phase performs human visual acceptance and, only after that,
controlled runtime validation.
