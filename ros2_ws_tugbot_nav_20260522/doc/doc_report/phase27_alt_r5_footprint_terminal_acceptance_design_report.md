# Phase27-alt-R5 Footprint Terminal Acceptance Design Report

Date: 2026-05-28
Workspace: `ros2_ws_tugbot_nav_20260522`
Status: `PASS_AS_FOOTPRINT_TERMINAL_ACCEPTANCE_DESIGN_ONLY`
R4 accepted status: `PASS_AS_FALLBACK_ENABLED_NON_REGRESSION_REPEAT`
R3 accepted status: `PASS_AS_TARGETED_TERMINAL_ACCEPTANCE_COVERAGE_DESIGN`
R2 coverage result preserved: `NOT_COVERED_TERMINAL_ACCEPTANCE_BRANCH`

## Scope

Phase27-alt-R5 is design/static/offline only. It does not start runtime smoke and
does not directly replace the existing terminal predicate. The goal is to evaluate
the proposed footprint-based terminal line/cylinder idea and compare it offline
against existing R1/R4 logs.

## Guardrails

Held:

- Did not enter Phase26Y/Phase26Z.
- Did not fetch `nav2_mppi_controller` source.
- Did not modify Nav2/MPPI/controller parameter semantics.
- Did not promote or reject `CostCritic=2.75`.
- Did not change general DFS/topology branch-selection strategy.
- Did not relax the local-cost safety gate.
- Did not claim MPPI selected-control near-zero root cause.
- Did not start runtime smoke.
- Did not directly replace the existing terminal predicate.
- Did not force terminal_acceptance runtime coverage.
- Did not use a fallback behavior success label.

## Files added

- `doc/doc_proposal/phase27_alt_r5_footprint_terminal_acceptance_design.md`
- `doc/doc_report/phase27_alt_r5_footprint_terminal_acceptance_design_report.md`
- `tools/analyze_phase27_alt_r5_footprint_acceptance_offline.py`
- `src/tugbot_maze/test/test_phase27_alt_r5_footprint_acceptance.py`
- `log/phase27_alt_r5_footprint_acceptance_offline.json`

## Footprint / SDF / config audit

Inspected:

- `src/tugbot_description/models/tugbot/model.sdf`
- `src/tugbot_description/models/tugbot/meshes/base/tugbot_simp.stl`
- `src/tugbot_navigation/config/nav2_slam_params.yaml`
- `src/tugbot_maze/tugbot_maze/maze_explorer.py`

Measured / inferred values:

```json
{
  "base_mesh_xy_circumscribed_radius_m": 0.36912210068126217,
  "nav2_robot_radius_m": 0.35,
  "sdf_collision_approx_max_radius_m": 0.4942061445948485
}
```

Design value selected:

```text
robot_circumscribed_radius_m = 0.4942061445948485
```

Reason:

- Nav2 uses `robot_radius: 0.35`, but SDF includes rear gripper collision boxes.
- A body-footprint terminal-line predicate should be conservative with respect to
  the actual collision geometry, so R5 uses the larger SDF collision estimate.

## Current 0.6m radius comparison

Current Phase27-alt terminal acceptance radius:

```text
near_exit_terminal_acceptance_radius_m = 0.6
```

R5 design with a conservative margin:

```text
robot_circumscribed_radius_m + terminal_acceptance_margin_m
= 0.4942061445948485 + 0.05
= 0.5442061445948485 m
```

Comparison:

```text
0.5442061445948485 m is 0.05579385540515147 m smaller than 0.6 m
```

Therefore the R5 footprint predicate, with the selected conservative margin, is
stricter than the existing 0.6m center-distance predicate. It should not make R4
runs pass earlier unless a larger margin is deliberately chosen in a future phase.
R5 does not recommend such a margin change.

## Proposed acceptance predicate

A future optional helper should accept only if:

```text
distance(base_link_xy, exit_xy) <= robot_circumscribed_radius_m + terminal_acceptance_margin_m
```

and all map gates pass:

```text
robot cell is not occupied
exit cell is not occupied
line_of_sight(robot_xy -> exit_xy) has occupied_count == 0
unknown cells are conservatively rejected by default
```

Default unknown policy:

```text
conservative_reject
```

This avoids隔墙误判 and unmapped-space false positives.

## Offline comparison on R1/R4 logs

Analyzer:

```bash
python3 tools/analyze_phase27_alt_r5_footprint_acceptance_offline.py \
  --output-json log/phase27_alt_r5_footprint_acceptance_offline.json \
  --robot-circumscribed-radius-m 0.4942061445948485 \
  --terminal-acceptance-margin-m 0.05 \
  --run phase27_alt_r1_smoke1 log/phase27_alt_r1_smoke1_explorer_state.jsonl \
  --run phase27_alt_r4_enabled_run1 log/phase27_alt_r4_enabled_run1_explorer_state.jsonl \
  --run phase27_alt_r4_enabled_run2 log/phase27_alt_r4_enabled_run2_explorer_state.jsonl
```

Output:

- `log/phase27_alt_r5_footprint_acceptance_offline.json`

Aggregate:

```json
{
  "current_radius_final_accept_count": 1,
  "footprint_radius_final_accept_count": 1,
  "current_radius_min_distance_accept_count": 1,
  "footprint_radius_min_distance_accept_count": 1,
  "footprint_accepts_more_runs_than_current": false,
  "run_count": 3
}
```

Per-run threshold comparison:

```json
{
  "phase27_alt_r1_smoke1": {
    "final_mode": "EXIT_REACHED",
    "final_exit_distance_m": 0.5408336208400051,
    "current_radius_final_accepts": true,
    "footprint_radius_final_accepts": true,
    "current_radius_accept_count": 13,
    "footprint_radius_accept_count": 10,
    "first_current_radius_accept_exit_distance_m": 0.5876039015539865,
    "first_footprint_radius_accept_exit_distance_m": 0.5408336208400051
  },
  "phase27_alt_r4_enabled_run1": {
    "final_mode": "NAVIGATING",
    "final_exit_distance_m": 0.7816547799762239,
    "min_exit_distance_m": 0.6087463086688935,
    "current_radius_final_accepts": false,
    "footprint_radius_final_accepts": false,
    "min_distance_current_radius_accepts": false,
    "min_distance_footprint_radius_accepts": false
  },
  "phase27_alt_r4_enabled_run2": {
    "final_mode": "FAILED_EXHAUSTED",
    "final_exit_distance_m": 0.7034997055891423,
    "min_exit_distance_m": 0.7034997055891423,
    "current_radius_final_accepts": false,
    "footprint_radius_final_accepts": false,
    "min_distance_current_radius_accepts": false,
    "min_distance_footprint_radius_accepts": false
  }
}
```

Important interpretation:

- R1: both current 0.6m and footprint+margin radius accept at final distance.
  The footprint radius would accept later/fewer samples than 0.6m because it is
  smaller.
- R4 run1: final distance about `0.782m`, min sampled distance about `0.609m`.
  It would not pass either the current 0.6m threshold or the footprint+margin
  threshold.
- R4 run2: final/min distance about `0.703m`. It would not pass either threshold.
- Therefore R5 footprint design does not reclassify R4 non-EXIT runs as success.

## Test coverage

R5 host-level tests cover:

- footprint touches terminal line -> accepted
- base_link origin is not at exit but footprint covers terminal line -> accepted
- distance greater than footprint radius + margin -> rejected
- occupied wall / line-of-sight between robot and exit -> rejected
- occupied robot or exit cell -> rejected
- unknown cells -> conservative reject by default
- explicit record-only unknown policy records unknown cells without default accept semantics
- margin boundary stability
- CLI offline comparison for R4-like final distances

## Verification

Commands:

```bash
python3 -m pytest src/tugbot_maze/test/test_phase27_alt_r5_footprint_acceptance.py -q
python3 -m pytest \
  src/tugbot_maze/test/test_phase27_alt_r5_footprint_acceptance.py \
  src/tugbot_maze/test/test_phase27_alt_r4_non_regression.py \
  src/tugbot_maze/test/test_phase27_alt_r3_decision_path_coverage.py \
  src/tugbot_maze/test/test_phase27_alt_terminal_acceptance_coverage.py \
  src/tugbot_maze/test/test_phase27_alt_runtime_analysis.py \
  src/tugbot_maze/test/test_phase27_alt_near_exit_fallback.py -q
python3 -m py_compile \
  tools/analyze_phase27_alt_r5_footprint_acceptance_offline.py \
  src/tugbot_maze/test/test_phase27_alt_r5_footprint_acceptance.py \
  src/tugbot_maze/tugbot_maze/maze_explorer.py
colcon build --symlink-install
colcon test --event-handlers console_direct+
colcon test-result --verbose
git diff -- src/tugbot_navigation/config | cat
```

Final verification results are recorded in the assistant completion message after
running the full verification sequence.

## Conclusion

R5 result:

```text
PASS_AS_FOOTPRINT_TERMINAL_ACCEPTANCE_DESIGN_ONLY
```

The footprint terminal line/cylinder idea is feasible as a future optional,
default-off supplemental terminal predicate with strict occupancy, line-of-sight,
and known-space gates. With the audited conservative Tugbot collision radius and
`0.05m` margin, it is stricter than the current `0.6m` radius and does not convert
R4 run1 or run2 into success. R5 does not start runtime smoke, does not replace
the current predicate, and does not recommend tuning or controller changes.
