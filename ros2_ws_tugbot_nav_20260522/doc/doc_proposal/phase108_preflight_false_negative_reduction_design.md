# Phase108: Ingress preflight false-negative reduction design

Status: DESIGN_ONLY_PENDING_HUMAN_ACCEPTANCE

Decision target: keep the Phase105 ingress preflight fail-closed, but redesign a future implementation so it is less likely to reject from tool/query/timing false negatives.

Phase108 is DESIGN_ONLY. It does not implement code, does not rerun Phase106, does not remove preflight, does not send ingress goal, does not start maze_explorer, does not tune Nav2/MPPI/controller, and does not change exploration strategy. This design does not prove the bug is fixed and does not prove it is safe to send a goal. Phase109 not entered.

## Background from Phase107

Phase107 diagnosed the Phase106 discrepancy as:

```text
PREFLIGHT_TOOL_FALSE_NEGATIVE
contributing: RAW_CAPTURE_LATER_THAN_PREFLIGHT
```

Observed Phase106 preflight facts:

```text
passed=false
reject_reason=ingress_preflight_timeout
last_specific_reject_reason=ingress_map_base_tf_missing
controller_server_active=false
bt_navigator_active=false
navigate_to_pose_action_ready=true
map->base_link unavailable
scan_transform_unstable
```

Observed later raw-capture facts:

```text
scan_available=true
map_available=true
local_costmap_available=true
odom_available=true
tf_available=true
raw_capture_start_wall_time > preflight_timeout_deadline_wall_time
```

Observed launch-log fact:

```text
managed Nav2 nodes active before preflight start
```

Interpretation for design: the fail-closed policy is still correct because Phase103 proved that sending an ingress goal into bad TF/controller state can fail immediately. The Phase105 implementation, however, needs stronger evidence handling before it declares lifecycle inactive, TF missing, or scan transform unstable.

## Non-goals and hard guardrails

- DESIGN_ONLY: no runtime code change in Phase108.
- Do not remove the ingress preflight; do not remove preflight as part of this design review.
- Do not weaken the fail-closed contract.
- Do not send ingress goal in Phase108; do not send ingress goal as part of this design review.
- Do not start maze_explorer in Phase108; do not start maze_explorer as part of this design review.
- Do not rerun Phase106 in Phase108.
- Do not tune Nav2 in Phase108; do not tune Nav2/MPPI/controller parameters as part of this design review.
- Do not change the explicit inner ingress goal: `frame_id=map, x=2.0, y=0.0, yaw=0.0`.
- Do not modify Phase88/92/101/105 runtime logic in this phase.
- Do not change branch scoring, exploration order, centerline gate, directional readiness, fallback, or terminal acceptance.
- Do not tune Nav2, MPPI, controller, inflation, robot_radius, clearance_radius_m, or map threshold.
- Do not claim autonomous exploration success.
- Do not claim exit success.
- A future preflight pass would only authorize sending the unchanged ingress goal in that future phase; it would not prove Goal1 carry-over/staging correctness.
- This document does not prove the bug is fixed.
- This document does not prove it is safe to send a goal.
- Phase109 not entered.

## Design principles

1. Keep fail-closed semantics.
   - Missing, stale, unstable, contradictory, or ambiguous evidence must not become pass.
   - The wrapper still must not send the ingress goal unless every hard gate passes in a bounded window.

2. Reduce false negatives by improving evidence quality, not by lowering safety.
   - More sources, more samples, and better artifacts are allowed.
   - Looser Nav2/controller/TF safety thresholds are not allowed.

3. Separate `inactive`, `unavailable`, and `ambiguous`.
   - A single subprocess failure should not become `controller_server_active=false` when node graph and action readiness disagree.
   - Ambiguous evidence remains fail-closed, but the artifact must preserve why it is ambiguous.

4. Treat raw-style availability as a cross-check, not an override.
   - A raw-style snapshot before reject may explain a false negative and produce `ambiguous` evidence.
   - It must not force pass unless the normal stable-window gates also pass.

5. Make every reject explainable sample-by-sample.
   - The Phase106 artifact had only a compact tail and top-level failures. Future artifacts must preserve per-sample failed gates and exact failure reasons.

## Future high-level flow

```text
launch visible stack
  -> wrapper readiness check remains outside the ingress preflight
  -> Phase108-designed preflight starts
      phase A: startup_grace window
        collect lifecycle/node/action/topic/first-scan/TF observations
        do not pass during grace unless all hard gates also satisfy stable-window rules
      phase B: stable_window accumulation
        sample TF, scan, lifecycle, action, costmap frames, and goal transform repeatedly
        require consecutive passing samples across stable_window_sec
      before reject:
        take raw_style_snapshot_cross_check
        merge contradictions into ambiguity fields
      if all gates pass continuously:
        passed=true
        wrapper may send unchanged ingress goal in a later implementation phase
      else:
        passed=false
        ingress_goal_sent=false
        maze_explorer_started=false
        reject fail-closed with specific token/history
```

The future design keeps `startup_grace_plus_stable_window` bounded. It waits long enough to avoid a single early miss, but it never waits indefinitely.

## Required design topics

### 1. `lifecycle_multi_source_confirmation`

Current Phase105 checks lifecycle through `ros2 lifecycle get` subprocess plus node graph and action list. Phase107 showed a contradiction: launch logs indicated managed Nav2 nodes were active before preflight, while the preflight artifact recorded `controller_server_active=false` and `bt_navigator_active=false`.

Future design:

For each managed node, collect and record at least these sources per sample:

```text
ros2_lifecycle_get_state
ros2_lifecycle_get_returncode
node_graph_present
action_server_available
lifecycle_manager_log_active_marker_seen
bond_or_managed_nodes_active_marker_seen
related_topic_or_service_present
query_exception_or_timeout
```

Suggested future semantic states:

```text
active_confirmed
inactive_confirmed
ambiguous
unavailable
query_error
```

Pass criteria:

```text
controller_server.lifecycle_state == active_confirmed
bt_navigator.lifecycle_state == active_confirmed
navigate_to_pose.action_ready == true
```

Fail-closed criteria:

- `inactive_confirmed`, `unavailable`, `query_error`, or `ambiguous` does not pass.
- However, if sources conflict, the top-level artifact records `lifecycle_ambiguous=true` and the reject token should prefer `ingress_lifecycle_ambiguous` over falsely reporting a hard inactive state.

Future conflict example:

```text
ros2 lifecycle get /controller_server -> subprocess timeout
node graph contains /controller_server -> true
/navigate_to_pose action available -> true
launch log says Managed nodes are active before preflight start -> true
=> lifecycle_state=ambiguous
=> lifecycle_ambiguous=true
=> fail closed, but do not record controller_server_active=false as a definitive fact
```

### 2. `tf_consecutive_sampling_stable_window`

Current design already intends a continuous stability window, but Phase106 still produced a top-level timeout with only three samples and a final raw-capture contradiction. The future implementation should distinguish single misses from sustained instability.

Future design:

```text
startup_grace_sec: bounded initial grace for first TF/scan/lifecycle observations
stable_window_sec: required continuous pass duration after grace or whenever all gates start passing
sample_period_sec: deterministic bounded sampling interval
required_consecutive_pass_samples: at least 2, preferably ceil(stable_window_sec/sample_period_sec)+1
```

For `map->base_link`, `map->odom`, and `odom->base_link`, each sample records:

```text
available
lookup_exception
stamp_age_sec
finite_transform
translation_xy
rotation_yaw
stamp_regressed
pose_jump_detected
stable_this_sample
```

Pass criteria:

- every TF hard gate passes in every sample in the stable window;
- no timestamp regression or large pose jump appears;
- final sample also passes.

Reject criteria:

- timeout before any complete stable window;
- sustained missing transform;
- sustained stale transform;
- timestamp regression or jump inside stable window.

New future diagnostic token:

```text
ingress_tf_stable_window_not_met
```

This token should not replace the older specific tokens. It explains that the high-level reason for rejection was failure to accumulate a stable window, while `failed_gates` and `failure_reasons_by_gate` retain the exact per-sample subgate failures.

### 3. `scan_wait_for_first_sample`

Phase107 showed preflight did not have a scan frame sample, while raw capture later observed `/scan` with `frame_id=tugbot/scan_omni/scan_omni`. A future implementation should not mark scan transform unstable before it has allowed a bounded first-scan wait.

Future design:

- Subscribe to `/scan` immediately at preflight start.
- During `startup_grace_sec`, classify missing scan as `waiting_for_first_scan`, not immediately as `ingress_scan_transform_unstable`.
- Record the exact wait duration.
- Once the first scan arrives, evaluate freshness and transformability in the stable-window logic.
- If no scan arrives before the first-scan deadline, reject fail-closed.

Future fields:

```text
first_scan_seen
first_scan_wait_elapsed_sec
first_scan_deadline_sec
scan_topic
scan_frame_id
scan_stamp_age_sec
scan_transform_target_frame
scan_transform_available
scan_transform_exception
scan_transform_stable_this_sample
```

Future token for no first scan:

```text
ingress_first_scan_timeout
```

If a scan arrives but cannot be transformed across the stable window, preserve the existing token:

```text
ingress_scan_transform_unstable
```

### 4. `per_sample_failure_reason_history`

Future artifacts must make false-negative review possible without rerunning the visible stack.

Required artifact structure:

```json
{
  "ingress_preflight": {
    "evaluated": true,
    "passed": false,
    "reject_reason": "ingress_preflight_timeout",
    "last_specific_reject_reason": "ingress_lifecycle_ambiguous",
    "failed_gates": ["ingress_lifecycle_ambiguous"],
    "sample_history": [
      {
        "sample_index": 0,
        "sample_wall_time_sec": 0.0,
        "elapsed_sec": 0.0,
        "phase": "startup_grace",
        "failed_gates": ["waiting_for_first_scan"],
        "failure_reasons_by_gate": {
          "scan": ["waiting_for_first_scan"]
        },
        "lifecycle_sources": {},
        "tf_checks": {},
        "scan_check": {},
        "goal_pose_transform_check": {}
      }
    ]
  }
}
```

Required per-sample fields:

```text
sample_history
sample_index
sample_wall_time_sec
elapsed_sec
phase
failed_gates
failure_reasons_by_gate
lifecycle_sources
lifecycle_ambiguous
tf_checks
scan_check
goal_pose_transform_check
raw_style_snapshot_before_reject
startup_grace_sec
stable_window_sec
first_scan_wait_elapsed_sec
```

Important: `sample_tail` may remain for compact summaries, but full `sample_history` must be preserved in the main preflight artifact or a linked sidecar JSONL.

### 5. `raw_style_snapshot_cross_check`

Phase106 raw capture was useful, but it happened after the preflight timeout. A future implementation should take a raw-style snapshot immediately before fail-closed reject, inside the preflight process/window.

Future raw-style snapshot fields:

```text
raw_style_snapshot_before_reject.present
raw_style_snapshot_before_reject.capture_wall_time_sec
raw_style_snapshot_before_reject.scan_available
raw_style_snapshot_before_reject.scan_topic
raw_style_snapshot_before_reject.scan_frame_id
raw_style_snapshot_before_reject.map_available
raw_style_snapshot_before_reject.map_frame_id
raw_style_snapshot_before_reject.local_costmap_available
raw_style_snapshot_before_reject.local_costmap_frame_id
raw_style_snapshot_before_reject.odom_available
raw_style_snapshot_before_reject.odom_frame_id
raw_style_snapshot_before_reject.odom_child_frame_id
raw_style_snapshot_before_reject.tf_pairs_available
raw_style_snapshot_before_reject.lifecycle_node_graph_present
raw_style_snapshot_before_reject.navigate_to_pose_action_ready
raw_style_snapshot_before_reject.cross_check_contradictions
```

If raw-style snapshot contradicts a preflight missing/inactive claim:

- do not pass automatically;
- mark the related source as ambiguous;
- emit `ingress_raw_snapshot_cross_check_failed` as a diagnostic or contributing reject token;
- keep `ingress_goal_sent=false` and `maze_explorer_started=false`.

Future token:

```text
ingress_raw_snapshot_cross_check_failed
```

This token means the preflight has contradictory evidence requiring human/design review, not that it is safe to proceed.

### 6. `lifecycle_ambiguous_not_inactive`

When lifecycle subprocess output conflicts with logs/action/node graph, the future preflight must not flatten the state to inactive.

Future state derivation:

```text
if lifecycle_get says active and node graph present and action ready:
    active_confirmed
elif lifecycle_get says inactive/error and node graph absent and action absent:
    inactive_confirmed_or_unavailable
elif lifecycle_get times out or disagrees with graph/action/log evidence:
    ambiguous
else:
    query_error
```

Artifact rule:

```text
controller_server_active: true only when active_confirmed
controller_server_inactive_confirmed: true only when inactive is confirmed by multiple sources
controller_server_ambiguous: true when sources disagree
bt_navigator_active: true only when active_confirmed
bt_navigator_inactive_confirmed: true only when inactive is confirmed by multiple sources
bt_navigator_ambiguous: true when sources disagree
lifecycle_ambiguous: true if any required lifecycle node is ambiguous
```

Future reject token:

```text
ingress_lifecycle_ambiguous
```

`ingress_lifecycle_ambiguous` is fail-closed. It is intentionally not a pass condition. Its value is diagnostic precision: it avoids a misleading artifact that claims `inactive=false` when the truth is source disagreement.

### 7. `startup_grace_plus_stable_window`

Phase105 currently uses one bounded timeout and a stable window. Future design should make the timing semantics explicit:

```text
total_timeout_sec = startup_grace_sec + stable_window_budget_sec
startup_grace_sec: allow first scan, TF buffer fill, lifecycle subprocess warmup
stable_window_sec: require continuous passing samples
stable_window_budget_sec: bounded maximum time to achieve the stable window after grace begins
```

Rules:

- The preflight may pass during startup grace only if it has already observed the full stable window.
- Missing first scan during grace is a waiting state, not an immediate hard scan-transform reject.
- Fail-closed on total timeout.
- If startup grace completes without first scan, reject as `ingress_first_scan_timeout` or keep waiting only if total timeout permits and artifact says why.
- If all individual gates pass intermittently but never continuously, reject as `ingress_tf_stable_window_not_met` plus per-sample gate history.

## Future reject / diagnostic token inventory

Keep existing tokens for compatibility:

```text
ingress_tf_unstable
ingress_map_base_tf_missing
ingress_map_odom_tf_stale
ingress_odom_base_tf_stale
ingress_scan_transform_unstable
ingress_controller_robot_pose_unavailable
ingress_goal_pose_transform_unavailable
ingress_preflight_timeout
```

Add future diagnostic tokens for false-negative reduction clarity:

```text
ingress_lifecycle_ambiguous
ingress_first_scan_timeout
ingress_tf_stable_window_not_met
ingress_raw_snapshot_cross_check_failed
```

Recommended top-level reject priority:

1. `ingress_raw_snapshot_cross_check_failed` when pre-reject snapshot contradicts a hard missing/inactive claim.
2. `ingress_lifecycle_ambiguous` when lifecycle sources conflict.
3. `ingress_map_base_tf_missing` when sustained map-base absence is confirmed.
4. `ingress_tf_unstable` for jump/regression/cache instability.
5. `ingress_tf_stable_window_not_met` when individual gates are intermittent but no stable window is achieved.
6. `ingress_map_odom_tf_stale`.
7. `ingress_odom_base_tf_stale`.
8. `ingress_first_scan_timeout`.
9. `ingress_scan_transform_unstable`.
10. `ingress_controller_robot_pose_unavailable`.
11. `ingress_goal_pose_transform_unavailable`.
12. `ingress_preflight_timeout` as the bounded deadline wrapper token.

Compatibility note: `ingress_preflight_timeout` may remain the top-level reason for deadline expiry, but the artifact must always expose `last_specific_reject_reason` and full `failed_gates`/`failure_reasons_by_gate` so analyzers can classify the true blocker.

## Future pass contract

A future preflight pass requires all of the following in the same bounded continuous stable window:

```text
lifecycle_multi_source_confirmation active_confirmed for controller_server
lifecycle_multi_source_confirmation active_confirmed for bt_navigator
NavigateToPose action available
map->base_link available, finite, fresh, stable
map->odom available and fresh
odom->base_link available and fresh
first /scan sample seen and fresh
scan frame transform available and stable to required target frame
inner ingress goal transformable to global_costmap frame and controller/local frame
no fresh TF jump/cache-drop/robot-pose-unavailable/goal-transform failure markers
raw_style_snapshot_cross_check has no contradiction at final sample
```

Fail-closed output on any reject:

```text
passed=false
ingress_goal_sent=false
maze_explorer_started=false
```

Pass output does not mean autonomous exploration success. It only means a future wrapper may send the unchanged explicit ingress goal.

## Future implementation tasks for Phase109 or later

If human acceptance authorizes implementation later, the recommended bite-sized tasks are:

1. Add failing focused tests for lifecycle ambiguity.
2. Implement lifecycle multi-source state derivation as a pure helper.
3. Add failing tests for first-scan grace semantics.
4. Implement first-scan wait state without changing pass criteria.
5. Add failing tests for consecutive stable-window accumulation.
6. Implement full `sample_history` with `failure_reasons_by_gate`.
7. Add failing tests for raw-style snapshot contradiction handling.
8. Implement pre-reject raw-style snapshot cross-check.
9. Integrate with Phase102/106 wrappers while preserving fail-closed no-goal/no-explorer contract.
10. Run static tests only first; runtime validation must be a separate phase.

## Acceptance criteria for this design review

- The document covers lifecycle multi-source confirmation.
- The document covers TF consecutive sampling and stable window semantics.
- The document covers waiting for the first `/scan` sample.
- The document requires per-sample failure reasons in artifacts.
- The document requires a pre-reject raw-style snapshot cross-check.
- The document marks lifecycle contradictions as ambiguous instead of definitive inactive.
- The document splits bounded wait into startup grace plus stable window.
- The document keeps fail-closed behavior.
- The document explicitly says it does not fix the bug and does not prove it is safe to send a goal.
- The document stops before Phase109.
