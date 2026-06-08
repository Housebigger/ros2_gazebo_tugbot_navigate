# Phase50: Dispatch Entry Readiness Gate Bounded Runtime Validation

Date: 2026-06-01

## Classification

`GATE_WAITING_FOR_READINESS_DATA`

## Scope

Phase50 ran a bounded runtime validation of the Phase49 `maze_explorer` dispatch-entry readiness gate.

The goal was to validate real runtime behavior only:

- Start active scaled2x world + SLAM + Nav2 + `maze_explorer`.
- Capture lifecycle/action/topic/scan/map/TF/local-costmap evidence.
- Confirm the gate holds in `WAIT_FOR_DISPATCH_ENTRY_READINESS` while not ready.
- Confirm whether first topology sampling happens only after gate ready.
- If topology sampling occurs and reports `clearance_radius_blocked`, classify that as the next layer: clearance sampling geometry / entrance local-map geometry.

## Guardrails

Preserved:

- No Nav2 / MPPI / controller parameter tuning.
- No `clearance_radius_m` tuning.
- No `maze_explorer` strategy change.
- No fallback / terminal acceptance continuation.
- No old scaffold world/map.
- No long run.
- No autonomous exploration success claim.

Note: a runtime-blocking Phase49 launch compatibility bug was fixed before the final Phase50 bounded run. The fix changes the default declaration type of `dispatch_readiness_required_lifecycle_nodes` from a ROS string-array default to the same comma-separated string form used by launch overrides. This is not a strategy or clearance change; it allows the Phase49 gate to launch under the existing `tugbot_maze_explore.launch.py` parameter path.

## Files Added / Changed

- `tools/run_phase50_dispatch_entry_readiness_gate_bounded_runtime.sh`
  - Bounded runtime wrapper.
  - Uses active scaled2x world and active scaled2x metadata.
  - Records `/maze/explorer_state`, `/maze/goal_events`, action/topic/lifecycle evidence, and full map/scan/TF/costmap runtime snapshots.

- `tools/analyze_phase50_dispatch_entry_readiness_gate_runtime.py`
  - Runtime recorder/analyzer.
  - Computes gate timing classification.
  - Detects waiting-before-ready, topology-after-ready, and clearance-blocked-after-ready conditions.

- `src/tugbot_maze/test/test_phase50_dispatch_entry_readiness_gate_bounded_runtime.py`
  - Phase50 static/TDD contract tests.

- `src/tugbot_maze/tugbot_maze/maze_explorer.py`
  - Compatibility repair only:
    - `dispatch_readiness_required_lifecycle_nodes` default is now declared as comma-separated string `'/controller_server,/planner_server,/bt_navigator'`.
    - Existing `_string_list_parameter(...)` parsing remains responsible for producing the list.
  - No clearance, Nav2, MPPI, controller, or exploration strategy change.

## Runtime Command

Wrapper:

```bash
PHASE50_RUN_TIMEOUT_SEC=180 \
PHASE50_RUNTIME_RECORDER_TIMEOUT_SEC=150 \
PHASE50_STATE_RECORDER_TIMEOUT_SEC=210 \
PHASE50_SNAPSHOT_DELAY_SEC=75 \
tools/run_phase50_dispatch_entry_readiness_gate_bounded_runtime.sh
```

Result:

- Wrapper exit: `0`
- Runtime bound: `180s`
- `max_goals`: `1`
- Active world: `src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean_scaled2x.sdf`
- Active metadata: `src/tugbot_maze/config/maze_20260528_scaled_instance.yaml`
- Truth frame: `map`
- Entrance: `(0.0, 0.0, 0.0)`
- Exit: `(21.072562, 18.083566)`, radius `1.2`

Artifacts:

- `log/phase50_dispatch_entry_readiness_gate_bounded_runtime/phase50_dispatch_entry_readiness_gate_bounded_runtime.json`
- `log/phase50_dispatch_entry_readiness_gate_bounded_runtime/phase50_runtime_evidence.json`
- `log/phase50_dispatch_entry_readiness_gate_bounded_runtime/phase50_runtime_brief.json`
- `log/phase50_dispatch_entry_readiness_gate_bounded_runtime/phase50_dispatch_entry_readiness_gate_bounded_runtime_explorer_state.jsonl`
- `log/phase50_dispatch_entry_readiness_gate_bounded_runtime/phase50_dispatch_entry_readiness_gate_bounded_runtime_goal_events.jsonl`
- `log/phase50_dispatch_entry_readiness_gate_bounded_runtime/phase50_dispatch_entry_readiness_gate_bounded_runtime_launch.log`

## Runtime Evidence Summary

From final analyzed summary:

```text
classification=GATE_WAITING_FOR_READINESS_DATA
explorer_state_samples=247
goal_events_samples=0
dispatch_events=0
outcome_events=0
waited_before_ready=False
first_topology_after_gate_ready=False
clearance_radius_blocked_after_gate_ready=False
gate_never_bypassed_before_ready=True
nav2_action_server_available=True
goal_pose_subscriber_available=True
nav2_lifecycle_active=True
cleanup_empty=True
complete_autonomous_success_claimed=False
```

Interpretation:

- The Phase49 gate did not bypass readiness.
- `maze_explorer` stayed in `WAIT_FOR_DISPATCH_ENTRY_READINESS`.
- No goal dispatch occurred.
- No topology sampling occurred.
- Therefore no `clearance_radius_blocked` evidence was produced in Phase50.
- The run did not reach the condition needed to classify clearance geometry as the next layer.

## Gate Timeline

Observed gate state transitions:

```text
seq=1  elapsed=15.737s  mode=WAIT_FOR_DISPATCH_ENTRY_READINESS  passed=false
  blocking_reasons=[nav2_lifecycle_active, map_sufficient, local_costmap_sufficient]
  checks:
    goal_pose_subscriber_ready=true
    local_costmap_sufficient=false
    map_sufficient=false
    nav2_lifecycle_active=false
    navigate_to_pose_action_ready=true
    scan_sufficient=true
    tf_sufficient=true

seq=2  elapsed=16.739s  mode=WAIT_FOR_DISPATCH_ENTRY_READINESS  passed=false
  blocking_reasons=[nav2_lifecycle_active, map_sufficient]
  checks:
    goal_pose_subscriber_ready=true
    local_costmap_sufficient=true
    map_sufficient=false
    nav2_lifecycle_active=false
    navigate_to_pose_action_ready=true
    scan_sufficient=true
    tf_sufficient=true

seq=5  elapsed=18.739s  mode=WAIT_FOR_DISPATCH_ENTRY_READINESS  passed=false
  blocking_reasons=[map_sufficient]
  checks:
    goal_pose_subscriber_ready=true
    local_costmap_sufficient=true
    map_sufficient=false
    nav2_lifecycle_active=true
    navigate_to_pose_action_ready=true
    scan_sufficient=true
    tf_sufficient=true
```

Final state at `179.738s`:

```text
mode=WAIT_FOR_DISPATCH_ENTRY_READINESS
passed=false
blocking_reasons=[map_sufficient]
```

Final gate details:

```text
/map near robot:
  sufficient=false
  reason=map_ratio_or_bounds_insufficient
  known_ratio=0.4429369513168396
  free_ratio=0.4429369513168396
  occupied_ratio=0.0
  unknown_ratio=0.5570630486831604
  min_known_ratio=0.7
  min_free_ratio=0.5
  radius_m=1.0
  robot_in_bounds=true
  sample_count=1253

/local_costmap/costmap near robot:
  sufficient=true
  reason=sufficient
  known_ratio=1.0
  free_ratio=0.9588607594936709
  occupied_ratio=0.04113924050632911
  unknown_ratio=0.0
  min_known_ratio=0.95
  min_free_ratio=0.5
  radius_m=1.0
  robot_in_bounds=true
  sample_age_sec=0.393
  sample_count=1264

/scan:
  sufficient=true
  finite_count=322
  min_required_finite_count=120
  nearest_obstacle_m=1.4781546592712402

TF:
  sufficient=true
  source=map->base_link

Nav2 lifecycle:
  /controller_server active [3]
  /planner_server active [3]
  /bt_navigator active [3]

Action/topic:
  /navigate_to_pose Action servers: 1
  /goal_pose Subscription count: 1
```

## Full Runtime Snapshot Evidence

Runtime recorder captured `31` snapshots.

Representative final snapshot:

```text
robot_pose_in_map=[~0.0, ~0.0, ~0.0]
robot_to_active_entrance_m≈1.07e-11
scan finite_count=322
scan nearest_obstacle_m=1.4781546592712402
TF map->base_link available=true
TF odom->base_link available=true
TF map->odom available=true

/map near robot:
  known_ratio=0.7189119170984456
  free_ratio=0.7189119170984456
  unknown_ratio=0.2810880829015544
  center_in_bounds=true

/local_costmap/costmap near robot:
  known_ratio=1.0
  free_ratio=0.9588607594936709
  unknown_ratio=0.0
  center_in_bounds=true

/global_costmap/costmap near robot:
  known_ratio=0.7448186528497409
  free_ratio=0.7448186528497409
  unknown_ratio=0.2551813471502591
  center_in_bounds=true
```

Note on ratio discrepancy:

- The runtime recorder and the `maze_explorer` gate both use full subscribed occupancy-grid data, not truncated `ros2 topic echo` output.
- The final gate decision is authoritative for gate behavior because it is emitted by `maze_explorer` itself in `/maze/explorer_state`.
- In that gate payload, `/map` remained below the configured gate thresholds: known/free `0.4429 < 0.70/0.50`.
- Therefore `maze_explorer` correctly remained waiting.

## What Phase50 Validated

Validated:

1. Phase49 gate is active in real bounded runtime.
2. Gate publishes structured readiness diagnostics.
3. Gate does not immediately fall through to topology sampling while readiness is false.
4. Gate does not dispatch goals while readiness is false.
5. Gate does not enter `FAILED_EXHAUSTED` just because startup readiness is incomplete.
6. Nav2 lifecycle/action/topic became ready in the bounded runtime.
7. Scan, TF, and local costmap became sufficient.
8. `/map` sufficiency remained the final blocker according to the `maze_explorer` gate payload.

Not validated in this run:

1. Gate-ready transition to first topology sampling.
2. Clearance sampling geometry / entrance local-map geometry as the next layer.
3. Any autonomous exploration success.

Reason: gate never reached ready state during the bounded run because `map_sufficient=false` remained in the final `maze_explorer` gate payload.

## Conclusion

Phase50 supports the following conclusion:

`GATE_WAITING_FOR_READINESS_DATA`

The Phase49 gate behaved conservatively and correctly for this bounded runtime: it held `maze_explorer` in `WAIT_FOR_DISPATCH_ENTRY_READINESS`, did not dispatch, did not topology-sample, and did not prematurely enter `FAILED_EXHAUSTED`.

The current bounded-runtime blocker is still map sufficiency at the dispatch-entry gate, not clearance geometry. Since no topology sampling occurred, there is no Phase50 evidence for `clearance_radius_blocked` and no basis to move the conclusion to clearance sampling geometry yet.

## Verification

Static/TDD verification:

```text
python3 -m py_compile \
  src/tugbot_maze/tugbot_maze/maze_explorer.py \
  tools/analyze_phase50_dispatch_entry_readiness_gate_runtime.py \
  src/tugbot_maze/test/test_phase50_dispatch_entry_readiness_gate_bounded_runtime.py

python3 -m pytest \
  src/tugbot_maze/test/test_phase49_dispatch_entry_readiness_gate.py \
  src/tugbot_maze/test/test_phase50_dispatch_entry_readiness_gate_bounded_runtime.py -q

11 passed in 0.05s
```

Build verification:

```text
colcon build --packages-select tugbot_maze tugbot_bringup --symlink-install

Summary: 2 packages finished [0.86s]
```

Runtime wrapper:

```text
wrapper_exit=0
```

Cleanup:

```text
cleanup_empty=true
```

Nav2 config guardrail:

`src/tugbot_navigation/config` diff remained empty for Phase50.

## Stop Point

Phase50 stops here for human review.

Do not enter Phase51 without explicit approval.
