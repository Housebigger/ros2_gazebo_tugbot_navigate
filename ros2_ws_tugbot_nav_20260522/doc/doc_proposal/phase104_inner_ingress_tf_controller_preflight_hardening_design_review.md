# Phase104: Inner-ingress TF/controller preflight hardening design review

Status: `DESIGN_REVIEW_ONLY_NOT_RUNTIME_ENABLED`

Decision: `DESIGN_REVIEW_COMPLETE_PENDING_EXPLICIT_PHASE105_IMPLEMENTATION`

Phase104 is design review only. No runtime code is changed. No Phase88/92/101 logic is changed. Phase105 not entered.

## Purpose

Phase103 diagnosed the Phase102 explicit inner-ingress Nav2 goal failure as:

`INNER_INGRESS_CONTROLLER_EXECUTION_FAILED`

The same explicit inner-ingress goal pose succeeded in Phase96-fix and Phase97:

```text
frame_id=map
x=2.0
y=0.0
yaw=0.0
```

Phase102 accepted the goal, then failed almost immediately:

```text
goal_sent=True
goal_accepted=True
result_received=True
status=6
success=False
error_code=102
recoveries_max=0
navigation_time_last_sec≈0.375
last_distance_remaining≈1.9926457405090332
```

Log-derived Phase103 evidence included:

```text
controller_goal_received=True
controller_unable_to_transform_goal_pose=True
follow_path_abort=True
bt_robot_pose_unavailable=True
tf_jump_back_count=793
scan_transform_cache_drop=True
scan_frame_transform_error=True
```

Phase104 designs a future fail-closed preflight before sending that explicit inner-ingress Nav2 goal. The goal is to avoid sending an ingress action into a known bad TF/controller/pose state. This design does not validate Phase101 carry-over. It only addresses the Phase102/103 ingress precondition reliability gap that occurred before maze_explorer and before Goal1 dispatch.

## Required reading completed

Phase104 design is based on the required preread set:

- `doc/doc_report/phase103_inner_ingress_goal_failure_diagnosis_report.md`
- `doc/doc_report/phase102_carry_over_bounded_goal1_staging_validation_report.md`
- `doc/doc_report/phase101_staging_corridor_evidence_carry_over_minimal_implementation_report.md`
- `doc/doc_report/phase96_fix_ingress_guided_startup_correction_report.md`
- `doc/doc_report/phase97_ingress_guided_refinement_chain_bounded_multi_goal_smoke_report.md`
- `doc/doc_proposal/obstacle_reproduction_handoff_workflow.md`
- top-level designs under `doc/doc_proposal`: Phase100, Phase91, Phase87, Phase83, Phase76, Phase27-alt-R5, and Phase27-alt near-exit fallback.

## Non-goals and hard guardrails

Design review only.

- Phase105 not entered.
- No runtime code is changed.
- No Phase88/92/101 logic is changed.
- No maze_explorer strategy change.
- No branch scoring change.
- No exploration order change.
- No centerline gate change.
- No directional readiness change.
- No fallback/terminal acceptance change.
- Do not change the inner-ingress target.
- No Nav2/MPPI/controller tuning.
- No inflation/robot_radius/clearance_radius_m/map threshold tuning.
- No autonomous exploration success claim.
- No exit success claim.
- This design does not validate Phase101 carry-over.
- A preflight rejection is not maze_explorer failure, not carry-over failure, not staging failure, not branch failure, and not autonomous success.

## Design principle

The future ingress preflight is a fail-closed gate in the visible/bounded startup wrapper before the explicit inner-ingress `NavigateToPose` goal is sent.

Do not send the ingress goal until every hard gate passes in the same bounded preflight window.

If any gate is missing, stale, unstable, contradictory, or times out, the wrapper must not send the ingress goal. It must emit an explicit reject reason and stop or hold according to the phase runbook. Missing evidence is rejected, not fabricated.

This is intentionally different from Nav2 lifecycle readiness alone. Phase102 had lifecycle/action readiness, but controller execution still aborted because robot pose / goal pose / scan transforms were unstable or unavailable at execution time.

## Future preflight core flow

```text
launch visible stack
  -> wait for Nav2 lifecycle/action readiness
  -> collect bounded TF/controller/scan stability samples
  -> require all hard preflight gates pass in one continuous window
  -> if pass: send explicit inner-ingress NavigateToPose goal unchanged
  -> if reject/timeout: ingress_goal_sent=false; record ingress_preflight_reject_reason
  -> do not start maze_explorer after rejected preflight
```

The future Phase105 implementation should be in the ingress wrapper/preflight tooling around the existing explicit inner-ingress handoff. It should not change maze_explorer dispatch strategy or the Phase88/92/101 helpers.

## Fail-closed preflight gates

### 1. map->base_link TF continuous stability window

Gate name: `map->base_link TF continuous stability window`.

A future implementation should sample `map -> base_link` repeatedly for a bounded continuous window:

```text
tf_stability_window_sec = N seconds, configured by wrapper/runbook only
sample_rate_hz = bounded and deterministic
required_samples >= 2
```

Pass semantics:

- transform exists for every sample in the continuous window;
- transform timestamps are monotonic or non-regressing within tolerance;
- pose values are finite;
- no large discontinuity / time jump appears in the sampled window;
- the robot pose can be expressed in the `map` frame at the end of the window.

Reject examples:

- missing transform for any sample;
- repeated lookup exception;
- transform timestamp regresses;
- Phase103-like `Detected jump back in time` or equivalent TF jump marker appears;
- robot pose cannot be computed from `map -> base_link`.

Primary reject token:

```text
ingress_map_base_tf_missing
ingress_tf_unstable
```

### 2. map->odom timestamp age gate

Gate name: `map->odom timestamp age gate`.

A future implementation should check `map -> odom` availability and timestamp freshness. The age threshold should be a wrapper preflight freshness bound, not a Nav2 parameter tuning change.

Pass semantics:

```text
map_odom_tf_available=true
map_odom_tf_age_sec <= ingress_preflight_tf_max_age_sec
```

Reject token:

```text
ingress_map_odom_tf_stale
```

If the transform is missing entirely, the future diagnostic may record `available=false` under the same token rather than inventing a separate success path.

### 3. odom->base_link timestamp age gate

Gate name: `odom->base_link timestamp age gate`.

A future implementation should check `odom -> base_link` availability and timestamp freshness.

Pass semantics:

```text
odom_base_tf_available=true
odom_base_tf_age_sec <= ingress_preflight_tf_max_age_sec
```

Reject token:

```text
ingress_odom_base_tf_stale
```

This gate distinguishes localization/map-frame freshness from odometry/body-frame freshness. Both must pass.

### 4. scan frame transform stability gate

Gate name: `scan frame transform stability gate`.

A future implementation should verify that the current `/scan` frame can be transformed into the controller/global costmap frame and/or `map` frame over the same bounded preflight window.

The check should record:

```text
scan_frame_id
scan_stamp_age_sec
scan_transform_target_frame
scan_transform_available
scan_transform_stable
scan_transform_exception_count
scan_transform_cache_drop_detected
```

Pass semantics:

- at least one recent scan is available;
- scan timestamp age is within the preflight freshness bound;
- `scan_frame -> map` or `scan_frame -> global_costmap.global_frame` transform is available;
- repeated checks do not produce cache-drop / extrapolation / old-data exceptions.

Reject token:

```text
ingress_scan_transform_unstable
```

This directly targets Phase103 `scan_transform_cache_drop=True` and `scan_frame_transform_error=True` evidence.

### 5. controller robot pose availability gate

Gate name: `controller robot pose availability gate`.

Lifecycle readiness is necessary but not sufficient. A future implementation must check:

```text
controller_server active
bt_navigator active
NavigateToPose action ready
robot pose available to controller/global costmap context
```

The controller-pose check should approximate the same precondition that `controller_server` and Nav2 BT need before executing FollowPath:

- `map -> base_link` available and stable;
- global costmap frame can express the robot pose;
- local/controller costmap frame can express the robot pose if available;
- no fresh `Robot pose is not available` marker is observed in the launch log during the preflight window.

Reject token:

```text
ingress_controller_robot_pose_unavailable
```

This directly targets Phase103 `bt_robot_pose_unavailable=True` and the immediate FollowPath abort.

### 6. inner-ingress goal pose transform gate

Gate name: `inner-ingress goal pose transform gate`.

The explicit inner-ingress target remains unchanged. The preflight only checks whether the goal pose can be transformed into the frames required by planner/controller/global costmap execution.

Pass semantics:

```text
goal_pose_frame_id=map
goal_pose_transform_to_global_costmap_frame_available=true
goal_pose_transform_to_controller_frame_available=true
transformed_goal_pose_finite=true
```

A future wrapper should not change x/y/yaw or frame_id to make this pass. It should only test transformability.

Reject token:

```text
ingress_goal_pose_transform_unavailable
```

This directly targets Phase103 `controller_unable_to_transform_goal_pose=True` evidence.

### 7. TF jump / cache-drop / robot-pose-unavailable detector gate

Gate name: `TF jump / cache-drop / robot-pose-unavailable detector gate`.

A future implementation should monitor bounded launch/preflight output and sampled exceptions for high-risk markers observed in Phase103:

```text
Detected jump back in time
Message Filter dropping message
frame ... at time ... for reason 'the timestamp on the message is earlier than all the data in the transform cache'
Unable to transform goal pose into costmap frame
Robot pose is not available
TF_ERROR
extrapolation into the past
extrapolation into the future
```

Pass semantics:

- no fresh marker appears during the preflight window;
- previous historical markers from earlier launches are not counted unless inside the current preflight window;
- detector counts are recorded even when zero.

Reject token:

```text
ingress_tf_unstable
ingress_controller_robot_pose_unavailable
ingress_scan_transform_unstable
ingress_goal_pose_transform_unavailable
```

The exact token should be chosen by the strongest specific failing subgate. If multiple fail, the artifact records all failed gates and sets `ingress_preflight_reject_reason` to the first-priority reason.

### 8. bounded wait then fail-closed

Gate name: `bounded wait then fail-closed`.

If preflight is briefly unstable, the wrapper may wait within a bounded startup window. The wait is not exploration and not a long unbounded run.

Recommended future behavior:

```text
preflight_deadline = now + ingress_preflight_timeout_sec
while now < preflight_deadline:
    sample all gates
    if all pass continuously for tf_stability_window_sec:
        pass preflight
        send ingress goal unchanged
        break
    sleep bounded interval
else:
    reject fail-closed
```

Reject token on deadline:

```text
ingress_preflight_timeout
```

If timeout occurs because a specific subgate never passed, the future artifact should record both:

```text
ingress_preflight_reject_reason=ingress_preflight_timeout
failed_gates=[specific gate tokens]
last_specific_reject_reason=<most recent specific token>
```

## Reject reason priority

When multiple failures are present, use a deterministic priority so analyzers can compare runs:

1. `ingress_map_base_tf_missing`
2. `ingress_tf_unstable`
3. `ingress_map_odom_tf_stale`
4. `ingress_odom_base_tf_stale`
5. `ingress_scan_transform_unstable`
6. `ingress_controller_robot_pose_unavailable`
7. `ingress_goal_pose_transform_unavailable`
8. `ingress_preflight_timeout`

The timeout token may be top-level when the only conclusive fact is that no stable window was achieved before the deadline. The artifact still records all subgate histories.

## Future reject tokens

The complete Phase104 future reject-token inventory is:

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

Reject contract:

```text
ingress_preflight.passed=false
ingress_goal_sent=false
ingress_goal_sent=false when rejected
ingress_preflight_reject_reason=<one token from inventory>
preflight rejection is not maze_explorer failure
maze_explorer_started=false
```

## Future artifacts and goal diagnostics contract

A future Phase105 implementation should emit a standalone ingress preflight artifact, for example:

```text
log/<phase>/ingress_preflight.json
```

It should also propagate a compact copy into wrapper analysis summaries. If maze_explorer is never started, these are wrapper-level diagnostics, not `/maze/goal_events` from maze_explorer. If a later wrapper emits a synthetic startup event or combines diagnostics with goal-event analysis, the field names should remain identical.

Required fields:

```text
ingress_preflight: {
  evaluated: bool,
  passed: bool,
  start_time_sec: float|null,
  end_time_sec: float|null,
  bounded_wait_elapsed_sec: float|null,
  ingress_preflight_timeout_sec: float|null,
  ingress_preflight_reject_reason: string|null,
  failed_gates: [string],
  passed_gates: [string],
  ingress_goal_sent: bool,
  ingress_goal_sent=false when rejected,
  inner_ingress_goal_pose: {
    frame_id: string,
    x_m: float,
    y_m: float,
    yaw_rad: float
  },
  tf_stability_window_sec: float|null,
  tf_jump_count: int,
  map_base_tf_check: {
    available: bool,
    stable: bool,
    sample_count: int,
    latest_age_sec: float|null
  },
  map_base_tf_age_sec: float|null,
  map_odom_tf_age_sec: float|null,
  odom_base_tf_age_sec: float|null,
  scan_transform_check: {
    scan_available: bool,
    scan_frame_id: string|null,
    target_frame: string|null,
    transform_available: bool,
    stable: bool,
    cache_drop_detected: bool,
    exception_count: int
  },
  controller_pose_check: {
    controller_server_active: bool,
    bt_navigator_active: bool,
    navigate_to_pose_action_ready: bool,
    robot_pose_available: bool,
    robot_pose_unavailable_log_count: int
  },
  goal_pose_transform_check: {
    goal_frame_id: string,
    global_costmap_frame: string|null,
    controller_frame: string|null,
    transform_to_global_costmap_available: bool,
    transform_to_controller_frame_available: bool,
    exception_count: int
  },
  tf_detector: {
    tf_jump_count: int,
    cache_drop_count: int,
    robot_pose_unavailable_count: int,
    goal_pose_transform_failure_count: int
  }
}
```

Compatibility notes:

- `ingress_goal_sent=false when rejected` is mandatory.
- `ingress_preflight_reject_reason` must be null only when `passed=true`.
- Missing optional sensor details are recorded as null/false with a failed gate, not fabricated.
- The explicit inner-ingress target is recorded for audit but not changed by preflight.

## Phase105 implementation boundary

If Phase105 is explicitly opened later, it should remain minimal:

1. Add focused tests first.
2. Add a pure-ish preflight result schema helper or wrapper-local function.
3. Add bounded TF/scan/controller/goal-transform checks before the existing ingress sender.
4. Emit `ingress_preflight.json` and minimal summary fields.
5. Do not start maze_explorer when preflight rejects.
6. Do not modify Phase88/92/101 helpers or maze_explorer strategy.
7. Do not tune Nav2/MPPI/controller/inflation/robot_radius/clearance_radius_m/map threshold.
8. Run only bounded validation after static approval.

Phase104 does not authorize implementation now.

## Relationship to Phase101 carry-over

This design does not validate Phase101 carry-over. Phase102 never reached Goal1 dispatch, so carry-over fields were absent. The Phase104 preflight design exists only to recover the prerequisite ability to enter the known-good ingress-guided startup path before any future carry-over validation.

If a later Phase105 preflight passes and ingress succeeds, a separate later validation phase would still be required to test Phase101 carry-over. Such a later validation must still not treat timeout as success and must still preserve the Phase101 rule that safety evidence is recomputed.

## Relationship to obstacle reproduction handoff

If a future preflight rejects in visible mode, the wrapper may hold Gazebo/RViz for screenshots when useful, but the output burden remains minimal:

- reject token;
- failed gates;
- latest TF/scan/controller/goal-transform evidence;
- screenshot suggestions if a visible stack is held;
- guardrails and stop condition.

A preflight reject is a bounded startup reliability blocker. It should not lead to blind algorithm changes in the same phase.

## Future focused static tests

If Phase105 is explicitly opened, minimum tests should cover:

- all future reject tokens are enumerated;
- missing `map -> base_link` causes `ingress_map_base_tf_missing` and `ingress_goal_sent=false`;
- unstable TF jump/cachedrop causes `ingress_tf_unstable` or a more specific transform token;
- stale `map -> odom` causes `ingress_map_odom_tf_stale`;
- stale `odom -> base_link` causes `ingress_odom_base_tf_stale`;
- unstable scan transform causes `ingress_scan_transform_unstable`;
- active lifecycle but robot pose unavailable causes `ingress_controller_robot_pose_unavailable`;
- goal pose cannot transform to costmap/controller frame causes `ingress_goal_pose_transform_unavailable`;
- bounded wait expires with no continuous stable window causes `ingress_preflight_timeout`;
- pass case sends the same explicit inner-ingress goal unchanged;
- reject case does not start maze_explorer and does not mark branch/carry-over/staging failure;
- guardrails prove no Nav2 config or maze runtime strategy changes.

## Acceptance criteria for Phase104

- Phase104 design document exists under `doc/doc_proposal`.
- Phase104 report exists under `doc/doc_report`.
- Focused static tests verify design-only status, fail-closed gates, reject tokens, diagnostics contract, and guardrails.
- Nav2 config diff guard remains zero.
- Maze runtime logic diff guard remains zero.
- No runtime simulation or long autonomous exploration is run.
- Phase105 not entered.

## Phase104 decision

`DESIGN_REVIEW_COMPLETE_PENDING_EXPLICIT_PHASE105_IMPLEMENTATION`

Phase104 recommends a fail-closed TF/controller preflight before the explicit inner-ingress Nav2 goal. It should require a continuous stable TF window, fresh map/odom/base transforms, stable scan transform, controller robot pose availability, and goal-pose transformability before sending the unchanged ingress goal. If those gates do not pass within a bounded wait, the future wrapper should reject with a specific token and `ingress_goal_sent=false`.

Phase105 not entered.
