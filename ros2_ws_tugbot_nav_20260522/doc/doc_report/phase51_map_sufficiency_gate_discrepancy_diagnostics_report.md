# Phase51: Map Sufficiency Gate Discrepancy Diagnostics

Status: COMPLETE

Classification: RECORDER_IN_BOUNDS_DENOMINATOR_MISMATCH

## Scope and guardrails

This phase was diagnostics-only.

- No clearance strategy change.
- No Nav2 / MPPI / controller parameter tuning.
- No map sufficiency threshold change.
- No maze_explorer dispatch or exploration strategy change.
- No autonomous exploration success claim.
- Active scaled2x world and map-frame truth convention are preserved.

## Question being answered

Phase50 showed an apparent discrepancy for the same startup/entrance-near map sufficiency evidence:

- maze_explorer dispatch readiness gate reported /map near robot as not sufficient:
  - known_ratio = 0.4429369513168396
  - free_ratio = 0.4429369513168396
  - sample_count = 1253
  - blocking_reasons included map_sufficient
- Phase50 runtime recorder legacy near_robot summary reported:
  - known_ratio = 0.7189119170984456
  - free_ratio = 0.7189119170984456
  - total = 772

Phase51 checked whether this difference came from radius, pose, timestamp, frame/origin/resolution, index rounding, sampling window, map boundary handling, or a later snapshot.

## Implementation changes

Diagnostics only:

1. `src/tugbot_maze/tugbot_maze/maze_explorer.py`
   - Expanded map/local-costmap gate payload with Phase51 sampling metadata:
     - `map_stamp`
     - `grid_frame_id`
     - `grid_width`, `grid_height`
     - `grid_resolution`
     - `grid_origin`
     - `robot_cell`
     - `radius_cells`
     - `sample_window`
     - `bbox_world`
     - `sample_count`
     - `in_bounds_count`
     - `out_of_bounds_count`
     - `known_count`, `free_count`, `occupied_count`, `unknown_count`
     - `out_of_bounds_as_unknown`
     - `phase51_map_sufficiency_discrepancy_diagnostics`
   - Kept gate threshold defaults unchanged:
     - `dispatch_readiness_near_robot_radius_m = 1.0`
     - `dispatch_readiness_min_map_known_ratio = 0.70`
     - `dispatch_readiness_min_map_free_ratio = 0.50`
   - Kept `clearance_radius_m = 0.35` unchanged.
   - Did not move topology sampling earlier or later beyond existing gate behavior.

2. `tools/analyze_phase50_dispatch_entry_readiness_gate_runtime.py`
   - Added recorder-side `inclusive_near_robot` map sampling that treats out-of-bounds samples as unknown, matching maze_explorer gate semantics.
   - Preserved legacy `near_robot` as the old in-bounds-only denominator for compatibility.
   - Added `in_bounds_near_robot` explicitly so future reports can compare both denominators.

3. `tools/analyze_phase51_map_sufficiency_gate_discrepancy.py`
   - New offline analyzer comparing gate map ratios to runtime recorder inclusive/in-bounds summaries.
   - Classifies the discrepancy and writes structured JSON.

4. `src/tugbot_maze/test/test_phase51_map_sufficiency_gate_discrepancy_diagnostics.py`
   - Added contract tests for Phase51 payload fields, unchanged thresholds/strategy, recorder inclusive-vs-in-bounds diagnostics, analyzer classification, and this report.

## Evidence

Phase50 final gate evidence:

- gate map known_ratio: `0.4429369513168396`
- gate map free_ratio: `0.4429369513168396`
- gate sample_count: `1253`
- gate robot_cell: `[3, 21]`
- gate reason: `map_ratio_or_bounds_insufficient`
- min_known_ratio: `0.70`
- min_free_ratio: `0.50`

Phase50 legacy runtime recorder evidence:

- recorder legacy near_robot known_ratio: `0.7189119170984456`
- recorder legacy near_robot free_ratio: `0.7189119170984456`
- recorder legacy total: `772`
- recorder center_cell: `[3, 21]`
- map frame: `map`
- map resolution: `0.05000000074505806`
- map origin: `(-0.1854999999893486, -1.0763693963416974)`
- map size: `62 x 243`

Phase51 reconstruction/alignment result:

- inclusive sample_count: `1253`
- in_bounds_count: `772`
- out_of_bounds_count: `481`
- inclusive known_ratio: `0.4429369513168396`
- inclusive free_ratio: `0.4429369513168396`
- in-bounds-only known_ratio: `0.7189119170984456`
- in-bounds-only free_ratio: `0.7189119170984456`
- robot_cell matched: `true`
- sample_count matched gate when using inclusive denominator: `true`
- gate matched runtime inclusive ratios: `true`
- legacy recorder excluded out-of-bounds cells: `true`

Structured artifact:

- `log/phase51_map_sufficiency_gate_discrepancy/phase51_map_sufficiency_gate_discrepancy.json`

## Root cause

The ratio discrepancy is explained by denominator semantics, not by a confirmed gate bug.

- maze_explorer gate included cells outside the current /map bounds inside the 1.0 m sampling circle and treated them as unknown.
- Phase50 recorder legacy `near_robot` only counted cells that were inside the OccupancyGrid bounds.
- Because the robot was near the left/start edge of the current SLAM map, the 1.0 m sampling circle extended out of the known OccupancyGrid bounds.
- The out-of-bounds portion contributed `481` unknown samples to the gate denominator.
- Therefore:
  - Gate/inclusive denominator: `1253`
  - Legacy recorder/in-bounds denominator: `772`
  - Same known/free count: `555`
  - `555 / 1253 = 0.4429369513168396`
  - `555 / 772 = 0.7189119170984456`

This matches the observed Phase50 discrepancy exactly.

## Interpretation

This phase supports:

- The gate and recorder are aligned when the recorder uses inclusive out-of-bounds-as-unknown semantics.
- The earlier recorder value was optimistic because it excluded the map-boundary/out-of-bounds portion of the sample window.
- The Phase50 gate did not appear to use a different radius, robot cell, map frame, origin/resolution, or index rounding for the final evidence being compared.
- The available evidence does not require threshold tuning or clearance strategy modification.

## Verification

Commands run:

```bash
python3 -m py_compile \
  src/tugbot_maze/tugbot_maze/maze_explorer.py \
  tools/analyze_phase50_dispatch_entry_readiness_gate_runtime.py \
  tools/analyze_phase51_map_sufficiency_gate_discrepancy.py

python3 -m pytest \
  src/tugbot_maze/test/test_phase49_dispatch_entry_readiness_gate.py \
  src/tugbot_maze/test/test_phase50_dispatch_entry_readiness_gate_bounded_runtime.py \
  src/tugbot_maze/test/test_phase51_map_sufficiency_gate_discrepancy_diagnostics.py \
  -q -k 'not report_exists'
```

Result before report write:

- `15 passed, 1 deselected in 0.08s`

Build:

```bash
colcon build --packages-select tugbot_maze tugbot_bringup --symlink-install
```

Result:

- `Summary: 2 packages finished [0.84s]`

The final full Phase49/50/51 test including report existence is run after this report is written.

## Conclusion

Phase51 classification is:

`RECORDER_IN_BOUNDS_DENOMINATOR_MISMATCH`

This is not an autonomous exploration success. It is a startup map sufficiency evidence-hardening result.

## Next-step boundary

Do not proceed to clearance strategy changes from this phase.

If the user accepts Phase51, the next decision should be framed as one of these bounded startup-map options, not as Nav2/controller tuning:

1. Keep gate behavior and require more startup map evidence before topology sampling.
2. Consider a bounded initial scan/map warmup behavior in a later phase.
3. Only after map-boundary evidence is accepted, discuss whether any threshold change is justified.

Stop here for human acceptance.
