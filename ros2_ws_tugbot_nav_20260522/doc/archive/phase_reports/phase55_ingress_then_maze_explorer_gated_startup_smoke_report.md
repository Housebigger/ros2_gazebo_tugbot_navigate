# Phase55: Ingress-then-maze_explorer Gated Startup Smoke

## Classification

INGRESS_THEN_GATE_READY_NO_DISPATCH_TOPOLOGY_REJECTED

## Scope

Phase55 validates the bounded startup chain:

1. launch active scaled2x world + SLAM + Nav2;
2. wait for Phase47 readiness gates;
3. send one Nav2 ingress goal to `map=(1.0, 0.0, 0.0)`;
4. after ingress is reached, start `maze_explorer` with Phase49 dispatch-entry readiness gate enabled;
5. observe whether the gate passes, topology sampling occurs, and a first dispatch is formed.

This report does not claim autonomous exploration success. It does not require reaching the exit. It does not treat a first dispatch, if present, as exit success.

## Guardrails

Preserved:

- active world: `tugbot_maze_world_20260528_clean_scaled2x.sdf`
- active metadata: `maze_20260528_scaled_instance.yaml`
- no old scaffold world/map
- no Nav2/MPPI/controller parameter tuning
- no `clearance_radius_m` tuning
- no map sufficiency threshold tuning
- no fallback/terminal acceptance
- bounded runtime only
- `max_goals=1`
- `near_exit_fallback_enabled=false`
- `startup_warmup_no_dispatch=false`
- no autonomous exploration success claim

Nav2 config diff: empty.

## Commands

Static checks during implementation:

```bash
python3 -m py_compile tools/analyze_phase55_ingress_then_maze_explorer_gated_startup_smoke.py
bash -n tools/run_phase55_ingress_then_maze_explorer_gated_startup_smoke.sh
python3 -m pytest src/tugbot_maze/test/test_phase55_ingress_then_maze_explorer_gated_startup_smoke.py -q -k 'not report_exists'
```

Focused static result before runtime:

```text
3 passed, 1 deselected in 0.00s
```

Runtime command:

```bash
PHASE55_RUN_TIMEOUT_SEC=300 \
PHASE55_RECORD_TIMEOUT_SEC=220 \
PHASE55_GOAL_TIMEOUT_SEC=100 \
PHASE55_READINESS_TIMEOUT_SEC=170 \
PHASE55_EXPLORER_OBSERVE_SEC=90 \
tools/run_phase55_ingress_then_maze_explorer_gated_startup_smoke.sh
```

The wrapper was interrupted after collecting the required runtime artifacts, then targeted cleanup was executed. The analyzer was run directly against the collected artifacts.

## Artifacts

Artifact directory:

`log/phase55_ingress_then_maze_explorer_gated_startup_smoke/`

Key files:

- `phase55_ingress_then_maze_explorer_gated_startup_smoke_launch.log`
- `phase55_ingress_then_maze_explorer_gated_startup_smoke_lifecycle_readiness.txt`
- `phase55_ingress_then_maze_explorer_gated_startup_smoke_navigate_to_pose_action_info.txt`
- `phase55_ingress_then_maze_explorer_gated_startup_smoke_goal_pose_topic_info.txt`
- `phase55_ingress_then_maze_explorer_gated_startup_smoke_ingress_navigate_to_pose_action_result.json`
- `phase55_ingress_then_maze_explorer_gated_startup_smoke_runtime_evidence.json`
- `phase55_ingress_then_maze_explorer_gated_startup_smoke_explorer_state.jsonl`
- `phase55_ingress_then_maze_explorer_gated_startup_smoke_goal_events.jsonl`
- `phase55_ingress_then_maze_explorer_gated_startup_smoke.json`
- `phase55_ingress_then_maze_explorer_gated_startup_smoke_nav2_config_diff.txt`

## Readiness evidence

Phase47-style readiness gates reached:

- `/controller_server`: active [3]
- `/planner_server`: active [3]
- `/bt_navigator`: active [3]
- `/navigate_to_pose`: Action servers: 1
- `/goal_pose`: Subscription count: 1
- `readiness_ready=1`

## Ingress Nav2 goal evidence

Ingress goal action result:

- action server available: true
- goal_sent: true
- goal_accepted: true
- result_received: true
- status_text: `STATUS_SUCCEEDED`
- error_code: 0
- success: true
- target: `map=(1.0, 0.0, 0.0)`

Robot motion evidence:

- first pose near startup: `[~0.0, ~0.0, ~0.0]`
- final pose near ingress: `[0.8615300237457176, 0.0200202534999986, 0.02869795429746887]`
- final distance to ingress: `0.13990977404765448 m`
- acceptance radius: `0.35 m`
- robot_moved_distance_m: `0.8617626078840324`
- pose samples: `105`

## Post-ingress map / scan / TF / costmap evidence

After ingress:

- `/map` inclusive near robot:
  - known_ratio: `0.8375594294770206`
  - free_ratio: `0.8304278922345484`
  - out_of_bounds_count: `0`
  - sample_count: `1262`
- `/local_costmap/costmap` inclusive near robot:
  - known_ratio: `1.0`
  - free_ratio: `0.6923688394276629`
  - out_of_bounds_count: `0`
  - sample_count: `1258`
- `/global_costmap/costmap` inclusive near robot:
  - known_ratio: `0.9183835182250396`
  - free_ratio: `0.7258320126782885`
  - out_of_bounds_count: `0`
- `/scan`:
  - finite_count: `405`
  - nearest_obstacle_m: `0.9287360906600952`
- TF:
  - `map->base_link`: available
  - `odom->base_link`: available
  - `map->odom`: available

## maze_explorer gate and topology evidence

`maze_explorer` was started only after ingress action success.

Explorer state samples: `13`.
Goal event samples: `0`.
Dispatch events: `0`.

Initial post-ingress gate samples showed all sensor/map evidence sufficient, with temporary blocker:

- blocker: `nav2_lifecycle_active`
- map_sufficient: true
- local_costmap_sufficient: true
- scan_sufficient: true
- tf_sufficient: true
- navigate_to_pose_action_ready: true
- goal_pose_subscriber_ready: true

The gate subsequently passed:

- `dispatch_readiness_gate_passed=true`
- blocking_reasons: `[]`
- nav2_lifecycle_active: true
- map_sufficient: true
- local_costmap_sufficient: true
- scan_sufficient: true
- tf_sufficient: true
- navigate_to_pose_action_ready: true
- goal_pose_subscriber_ready: true

First topology sampling occurred after gate-ready.

Topology diagnostics:

- sampled_direction_count: `4`
- raw_open_direction_count: `1`
- reject_reason_counts:
  - `accepted_open_direction`: `1`
  - `clearance_radius_blocked`: `3`
- last_open_direction_count: `1`
- last_candidate_count: `0`
- goal_count: `0`
- mode: `FAILED_EXHAUSTED`
- terminal reason: `dead end reached and no untried junction remains`

No `/maze/goal_events` dispatch event was observed.

## Interpretation

Phase55 restored the startup chain up to gate-ready and first topology sampling after ingress. The Phase49 dispatch-entry readiness gate is no longer blocked by startup map boundary evidence after the Phase54 ingress move.

However, topology did not form a dispatch candidate. The explorer reached `FAILED_EXHAUSTED` with no dispatch events and no goal count increment. The immediate rejection/terminal evidence is topology-side, not map-readiness-side:

- gate passed;
- topology sampled four directions;
- one raw open direction existed;
- no candidate dispatch formed;
- no first dispatch was sent.

Therefore Phase55 classification is:

`INGRESS_THEN_GATE_READY_NO_DISPATCH_TOPOLOGY_REJECTED`

This is not autonomous exploration success.

## Next-step recommendation

Next phase should diagnose why post-ingress topology has `raw_open_direction_count=1` but `last_candidate_count=0` and no dispatch event. The investigation should remain diagnostic-first and should not tune `clearance_radius_m`, map sufficiency thresholds, or Nav2/MPPI/controller parameters until candidate formation diagnostics justify a specific change.
