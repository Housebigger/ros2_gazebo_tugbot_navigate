# Phase52: Startup Map Boundary Warmup Bounded Runtime Validation

Status: COMPLETE

Classification: MAP_BOUNDARY_STILL_BLOCKING_AFTER_WARMUP

## Scope and guardrails

Phase52 validated bounded startup map-boundary warmup in the active scaled2x world only. It did not validate autonomous exploration success.

Guardrails held:

- No autonomous exploration success claim.
- No clearance strategy modification.
- No Nav2 / MPPI / controller parameter tuning.
- No map sufficiency threshold change.
- No fallback / terminal acceptance.
- No old scaffold world/map.
- No dispatch goal.
- No topology sampling.

Allowed classifications for this phase:

- MAP_SUFFICIENCY_NATURALLY_READY_AFTER_WARMUP
- MAP_BOUNDARY_STILL_BLOCKING_AFTER_WARMUP
- WARMUP_INCONCLUSIVE_RUNTIME_DATA_GAP
- GUARDRAIL_VIOLATION_DISPATCH_OCCURRED

## Inputs read

- README.md
- doc/ACTIVE_MAZE_WORLD.md
- doc/doc_report/phase48_baseline_separation_and_next_step_design_lock_report.md
- doc/doc_report/phase49_dispatch_entry_readiness_gate_implementation_report.md
- doc/doc_report/phase50_dispatch_entry_readiness_gate_bounded_runtime_validation_report.md
- doc/doc_report/phase51_map_sufficiency_gate_discrepancy_diagnostics_report.md
- tools/run_phase50_dispatch_entry_readiness_gate_bounded_runtime.sh
- tools/analyze_phase50_dispatch_entry_readiness_gate_runtime.py
- src/tugbot_maze/tugbot_maze/maze_explorer.py
- src/tugbot_bringup/launch/tugbot_maze_explore.launch.py

## Added artifacts and code

- tools/run_phase52_startup_map_boundary_warmup.sh
- tools/analyze_phase52_startup_map_boundary_warmup.py
- src/tugbot_maze/test/test_phase52_startup_map_boundary_warmup.py
- doc/doc_report/phase52_startup_map_boundary_warmup_report.md
- log/phase52_startup_map_boundary_warmup/phase52_startup_map_boundary_warmup.json

A diagnostics-only maze_explorer parameter was added:

- startup_warmup_no_dispatch

When true, maze_explorer still evaluates dispatch-entry readiness gate payloads, but if the gate passes it enters STARTUP_WARMUP_NO_DISPATCH and returns before topology sampling or dispatch. This is a safety/no-dispatch guard for Phase52 bounded startup validation, not an exploration strategy change. The wrapper also sets max_goals:=0.

## Runtime command

```bash
PHASE52_RUN_TIMEOUT_SEC=360 \
PHASE52_RUNTIME_RECORDER_TIMEOUT_SEC=330 \
PHASE52_STATE_RECORDER_TIMEOUT_SEC=390 \
PHASE52_SNAPSHOT_DELAY_SEC=90 \
tools/run_phase52_startup_map_boundary_warmup.sh
```

Runtime used:

- active world: tugbot_maze_world_20260528_clean_scaled2x.sdf
- active metadata: maze_20260528_scaled_instance.yaml
- truth frame: map
- entrance: (0.0, 0.0, 0.0)
- exit: (21.072562, 18.083566), radius 1.2m
- max_goals: 0
- startup_warmup_no_dispatch: true
- map threshold unchanged: known >= 0.70, free >= 0.50
- clearance_radius_m unchanged: 0.38

## Phase47-style readiness evidence

The bounded runtime captured the required readiness evidence:

- /controller_server active [3]: true
- /planner_server active [3]: true
- /bt_navigator active [3]: true
- /navigate_to_pose Action servers: 1: true
- /goal_pose Subscription count: 1: true

Summary fields:

- nav2_lifecycle_active: true
- nav2_action_server_available: true
- goal_pose_subscriber_available: true

## No-dispatch evidence

- explorer_state_samples: 420
- goal_events_samples: 0
- dispatch_events: 0
- goal_count_violation: false
- topology_sampling_occurred: false
- first_gate_ready_elapsed_sec: null
- complete_autonomous_success_claimed: false
- cleanup_empty: true

This confirms Phase52 did not dispatch goals and did not enter topology sampling.

## Map boundary warmup evidence

The inclusive near-robot /map ratio remained unchanged throughout the bounded warmup:

First runtime map sample:

- elapsed_sec: about 10.0
- known_ratio: 0.4429369513168396
- free_ratio: 0.4429369513168396
- unknown_ratio: 0.5570630486831604
- sample_count: 1253
- in_bounds_count: 772
- out_of_bounds_count: 481
- robot_cell: [3, 21]
- sample_window: min_x=-17, max_x=23, min_y=1, max_y=41
- map_width: 62
- map_height: 243
- map_origin: x=-0.1854999999893486, y=-1.0763693963416974

Final runtime map sample:

- elapsed_sec: about 330.0
- known_ratio: 0.4429369513168396
- free_ratio: 0.4429369513168396
- unknown_ratio: 0.5570630486831604
- sample_count: 1253
- in_bounds_count: 772
- out_of_bounds_count: 481
- robot_cell: [3, 21]
- sample_window: min_x=-17, max_x=23, min_y=1, max_y=41
- map_width: 62
- map_height: 243
- map_origin: x=-0.1854999999893486, y=-1.0763693963416974

Observed map boundary:

- min_x: -0.1854999999893486
- max_x: 2.914500046204251
- min_y: -1.0763693963416974
- max_y: 11.073630784707412
- width_m: 3.1000000461935997
- height_m: 12.150000181049109

The robot was in-bounds, but the 1.0m inclusive sample window extended outside the map on the negative-x side, leaving 481 cells out-of-bounds-as-unknown. Since the robot was stationary during warmup and SLAM map boundary did not expand around the entrance, the dispatch-entry map gate remained below threshold.

## Classification rationale

Classification: MAP_BOUNDARY_STILL_BLOCKING_AFTER_WARMUP

Reason:

- Phase47-style Nav2 readiness was available.
- Runtime snapshots and explorer states were present.
- No dispatch or topology sampling occurred.
- The /map inclusive near-robot ratio never crossed the unchanged threshold.
- Out-of-bounds-as-unknown count remained 481 across the warmup window.
- first_gate_ready_elapsed_sec stayed null.

This means bounded passive startup warmup alone did not make /map sufficient naturally become true at the entrance.

## Next-step boundary

Do not proceed to clearance strategy modification based on this phase.

Recommended next investigation options, requiring explicit next-phase authorization:

1. Initial scan/map warmup behavior: bounded non-goal behavior that can increase entrance-side map coverage before dispatch-entry readiness is evaluated.
2. Entrance initial pose / map-boundary handling: evaluate whether the active entrance is too close to the SLAM map boundary for an inclusive 1.0m gate radius.
3. Threshold policy review only with explicit authorization: evaluate whether out-of-bounds-as-unknown should remain a hard denominator requirement near a known active map boundary.

Phase52 stops here for human acceptance. No Phase53 work was started.
