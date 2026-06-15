# Phase38 Initial Local Topology Diagnostics on Scaled Clean World

Date: 2026-05-28 14:35:37 UTC

## Scope

Phase38 diagnoses why Phase37 entered `FAILED_EXHAUSTED` before dispatching any autonomous goal on the active scaled2x clean world.

This phase is diagnostics-only. It does not tune Nav2/MPPI/controller parameters, does not change `maze_explorer` strategy, does not continue fallback/terminal acceptance, does not use the old scaffold world/map, and does not claim autonomous exploration success.

## Inputs

- Phase37 artifacts: `log/phase37_scaled_clean_world_maze_explorer_bounded_smoke/`
- Active world: `src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean_scaled2x.sdf`
- Active metadata: `src/tugbot_maze/config/maze_20260528_scaled_instance.yaml`
- Phase37 report: `doc/doc_report/phase37_scaled_clean_world_maze_explorer_bounded_smoke_report.md`

Active truth:

- entrance_x = -11.011281
- entrance_y = -9.02507
- entrance_yaw = 0.0
- exit_x = 10.061281
- exit_y = 9.058496
- exit_radius = 1.2

## Added files

- `tools/analyze_phase38_initial_local_topology.py`
- `src/tugbot_maze/test/test_phase38_initial_local_topology.py`

## Generated artifacts

Directory:

- `log/phase38_initial_local_topology/`

Key artifacts:

- `phase38_initial_topology_analysis.json`
- `phase38_analyzer_stdout.json`
- `phase38_readiness_check.json`
- `initial_pose_map_overlay.png`
- `local_topology_sampling_overlay.png`
- `entrance_alignment_overlay.png`
- `cache_dirs_before_cleanup.txt`
- `cache_dirs_removed_final.txt`
- `cache_dirs_after_cleanup.txt`

## TDD / verification

Initial RED:

```text
python3 -m pytest -p no:cacheprovider src/tugbot_maze/test/test_phase38_initial_local_topology.py -q
3 failed
```

Final GREEN:

```text
python3 -m pytest -p no:cacheprovider   src/tugbot_maze/test/test_phase36_autonomous_readiness.py   src/tugbot_maze/test/test_phase37_bounded_smoke_wrapper.py   src/tugbot_maze/test/test_phase38_initial_local_topology.py -q

12 passed in 0.50s
```

Static checks:

```text
python3 -m py_compile tools/analyze_phase38_initial_local_topology.py src/tugbot_maze/test/test_phase38_initial_local_topology.py: PASS
Phase36/37 readiness analyzer: READY_FOR_BOUNDED_AUTONOMOUS_EXPLORATION_SMOKE, blockers=0
git diff -- src/tugbot_navigation/config: empty
runtime process check: empty
```

## Phase37 failure signature preserved

```json
{
  "explorer_state_samples": 10,
  "goal_count": 0,
  "goal_events_samples": 0,
  "last_candidate_count": 0,
  "last_local_topology_kind": "unknown",
  "last_open_direction_count": 0,
  "last_terminal_reason": "no untried branches remain",
  "mode": "FAILED_EXHAUSTED"
}
```

Phase37 conclusion remains:

```text
BOUNDED_SMOKE_PARTIAL_FAIL_NO_DISPATCH
```

Autonomous success claimed:

```text
False
```

## Robot pose and entrance alignment

Robot pose reconstructed from Phase37 `tf_once`:

```json
{
  "available": true,
  "source": "tf_once odom->base_link",
  "x": 1.068594207002688e-11,
  "y": -1.0400781548742887e-24,
  "yaw": 2.740399069523727e-13
}
```

Entrance alignment:

```json
{
  "bearing_from_robot_to_entrance_rad": -2.4550020544844346,
  "distance_m": 14.23728196553318,
  "dx_m": -11.011281000010687,
  "dy_m": -9.02507,
  "entrance_pose": {
    "x": -11.011281,
    "y": -9.02507,
    "yaw": 0.0
  },
  "entrance_yaw_minus_robot_yaw_rad": -2.740399069523727e-13,
  "exit_pose": {
    "radius": 1.2,
    "x": 10.061281,
    "y": 9.058496
  },
  "robot_pose": {
    "x": 1.068594207002688e-11,
    "y": -1.0400781548742887e-24,
    "yaw": 2.740399069523727e-13
  }
}
```

Key observation:

- Artifact pose is near map/odom origin: x≈0.000000, y≈-0.000000, yaw≈0.000000
- Active entrance truth is x=-11.011281, y=-9.02507
- Distance from artifact pose to active entrance: 14.237 m

This indicates the saved Phase37 pose sample was not aligned to the active entrance coordinates, or the SLAM map frame was still a local startup frame around odom origin at the sampled time. This is evidence to consider in the next phase, but Phase38 conclusion is driven mainly by the map/topology evidence below.

## Map / costmap evidence near robot

`/map` sample:

```json
{
  "bounds": {
    "xmax": 2.914500046202711,
    "xmin": -0.18549999999088873,
    "ymax": 11.073614719858146,
    "ymin": -1.0763854611909636
  },
  "cell_stats_near_robot": {
    "center_cell": {
      "x": 3,
      "y": 21
    },
    "free_cells": 0,
    "free_ratio": 0.0,
    "known_ratio": 0.0007079646017699115,
    "occupied_cells": 2,
    "out_of_bounds_cells": 1341,
    "out_of_bounds_ratio": 0.47469026548672566,
    "radius_m": 1.5,
    "total_cells": 2825,
    "truncated_missing_cells": 1429,
    "truncated_missing_ratio": 0.5058407079646018,
    "unknown_cells": 2823,
    "unknown_ratio": 0.9992920353982301
  },
  "data_expected_len": 15066,
  "data_observed_len": 128,
  "data_truncated": true,
  "frame_id": "map",
  "free_ratio_near_robot": 0.0,
  "height": 243,
  "known_ratio_near_robot": 0.0007079646017699115,
  "resolution": 0.05000000074505806,
  "robot_cell": {
    "x": 3,
    "y": 21
  },
  "sample_exists": true,
  "unknown_ratio_near_robot": 0.9992920353982301,
  "width": 62
}
```

Local costmap near robot:

```json
{
  "center_cell": {
    "x": 28,
    "y": 28
  },
  "free_cells": 0,
  "free_ratio": 0.0,
  "known_ratio": 0.0,
  "occupied_cells": 0,
  "out_of_bounds_cells": 0,
  "out_of_bounds_ratio": 0.0,
  "radius_m": 1.0,
  "total_cells": 1264,
  "truncated_missing_cells": 1264,
  "truncated_missing_ratio": 1.0,
  "unknown_cells": 1264,
  "unknown_ratio": 1.0
}
```

Global costmap near robot:

```json
{
  "center_cell": {
    "x": 3,
    "y": 21
  },
  "free_cells": 0,
  "free_ratio": 0.0,
  "known_ratio": 0.006017699115044248,
  "occupied_cells": 17,
  "out_of_bounds_cells": 1341,
  "out_of_bounds_ratio": 0.47469026548672566,
  "radius_m": 1.5,
  "total_cells": 2825,
  "truncated_missing_cells": 1429,
  "truncated_missing_ratio": 0.5058407079646018,
  "unknown_cells": 2808,
  "unknown_ratio": 0.9939823008849558
}
```

Important details:

- `/map` artifact exists, but the saved CLI sample is truncated: observed data length 128 vs expected 15066.
- Near the robot, reconstructed `/map` evidence is dominated by unknown/out-of-bounds/truncated-missing cells:
  - known_ratio_near_robot = 0.000708
  - unknown_ratio_near_robot = 0.999292
  - free_ratio_near_robot = 0.000000
  - out_of_bounds_ratio = 0.474690
  - truncated_missing_ratio = 0.505841
- Local costmap CLI sample is also truncated, so it cannot prove free local topology.

## Scan evidence

```json
{
  "finite_count": 0,
  "finite_ratio": 0.0,
  "nearest_obstacle_m": null,
  "range_max": 100.0,
  "range_min": 0.20000000298023224,
  "sample_count": 129,
  "sample_exists": true
}
```

The saved scan sample contains no finite ranges (`finite_count=0`). That means the Phase37 artifacts do not provide obstacle geometry supporting any open direction at startup.

## Topology sampling replay

The analyzer replayed the same core sampling shape used by `maze_explorer` startup topology classification:

- angle_step_deg = 90.0
- lookahead_m = 1.5
- clearance_radius_m = 0.38
- min_open_distance_m = 0.45

Replay result:

```json
{
  "attempted_from_artifact_map": true,
  "block_reason_counts": {
    "unknown_cell_or_unknown_clearance": 4
  },
  "kind_recomputed": "unknown",
  "open_direction_count_recomputed": 0,
  "sampled_direction_count": 4,
  "unknown_safe_open_direction_count": 0
}
```

Why `open_direction_count=0`:

```json
[
  "robot pose from tf/odom is far from active entrance pose",
  "saved /map around robot is mostly unknown",
  "local neighborhood around robot is mostly unknown/unknown-dominated",
  "all 4 sampled rays failed min_open_distance_m; strict unknown blocks=4",
  "robot pose used by Phase37 artifacts is near map origin while active entrance is far away"
]
```

Each of the four sampled rays was blocked immediately at 0.05 m by `unknown_cell_or_unknown_clearance` in the saved map artifact. No ray reached `min_open_distance_m=0.45`.

## Runtime diagnostic decision

```text
runtime_diagnostic_needed = False
runtime_diagnostic_reason = Phase37 artifacts were sufficient to classify initial topology failure.
```

No additional runtime diagnostic run was started in Phase38. The Phase37 artifacts were sufficient for a conservative classification: the startup map/scan evidence was insufficient for local topology sampling to produce any open direction.

## Conclusion classification

Allowed labels:

```json
[
  "INSUFFICIENT_EVIDENCE",
  "INSUFFICIENT_MAP_AT_START",
  "START_POSE_OUTSIDE_MAZE_OR_TOO_CLOSE_TO_BOUNDARY",
  "TF_OR_POSE_ALIGNMENT_ISSUE",
  "TOPOLOGY_SAMPLING_SCALE_MISMATCH",
  "UNKNOWN_CELL_POLICY_TOO_STRICT"
]
```

Selected conclusion:

```text
INSUFFICIENT_MAP_AT_START
```

Rationale:

- Phase37 `maze_explorer` did start with active scaled2x truth, and `/maze/explorer_state` confirmed `FAILED_EXHAUSTED` before dispatch.
- `/maze/goal_events` had zero samples and dispatch_events=0, so this is not autonomous success.
- The saved startup `/map` and costmap samples were not sufficient/free around the robot for clearance-based topology sampling.
- Topology replay found zero open directions; all four sampled directions failed immediately on unknown/unknown-clearance.
- Scan sample had zero finite ranges, so it did not compensate for missing known map evidence.

Therefore Phase38 conclusion is:

```text
INSUFFICIENT_MAP_AT_START
```

Secondary notes for the next phase:

- The artifact robot pose is ~14.237 m from the active entrance truth, suggesting a map/odom/entrance-frame alignment issue may also need targeted evidence. However, given the startup map/scan sparsity and truncated CLI samples, Phase38 does not promote this to the primary `TF_OR_POSE_ALIGNMENT_ISSUE` conclusion.
- A later bounded startup diagnostic should capture full serialized map/costmap arrays or bag/JSON summaries rather than relying on truncated `ros2 topic echo --once` output.
- Do not tune Nav2 or exploration strategy until startup map sufficiency and map-frame/entrance alignment are proven.

## Cleanup

Process check after Phase38 analysis: empty.

Cache cleanup after final verification:

```text
(empty)
```

## Stop point

Phase38 is complete and stops here for human review.
