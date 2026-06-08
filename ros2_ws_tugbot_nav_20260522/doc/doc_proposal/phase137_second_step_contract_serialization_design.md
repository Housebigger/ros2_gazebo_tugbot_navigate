# Phase137 second-step contract serialization gap design

Status: DESIGN_ONLY

Phase137 is a doc-only/design-only review. It defines how a future Phase138 may close the second-step artifact/serialization contract gap exposed by Phase136. Phase137 does not modify maze_explorer.py, does not implement runtime behavior, and does not authorize any navigation policy or Nav2 tuning.

## Runtime and scope guardrails

- doc-only/design-only.
- No Gazebo/RViz/Nav2 runtime may be launched.
- No NavigateToPose goal may be sent.
- No maze_explorer may be started.
- No staging/explore/third goal may be sent.
- No Nav2/MPPI/controller/goal checker/config tuning may be performed.
- No exploration strategy, branch scoring, centerline, fallback, or terminal acceptance change may be made.
- No direct staging disablement is authorized.
- No autonomous exploration success or exit success may be claimed.
- Phase138 not entered.

## Phase136 anchor

Phase136 final classification: SECOND_STEP_CONTRACT_AMBIGUOUS.

Phase136 proved important bounded facts:

- staging succeeded.
- fresh scan/local_costmap/TF recorded.
- unique second-step goal_kind=explore dispatch.
- accepted=true.
- third_goal_dispatched=false.

Phase136 did not prove the Phase135 second-step contract fields:

- pending_corridor_alignment_second_step present.
- second_step_forward_goal valid.
- generated_after_fresh_evidence=true.
- selected_candidate_target present.
- skip_two_step_staging=True or equivalent recursion guard.

Therefore accepted must not be interpreted as success. The accepted second-step dispatch is diagnostic only. It is not autonomous exploration success, not exit success, and not Phase127 timeout repair.

## Source read-only context

Phase137 reviewed existing source semantics only. Current source already has these internal hooks:

- `self.pending_corridor_alignment_second_step = { ... }` is populated when corridor-alignment staging is applied.
- `_dispatch_second_step_after_corridor_alignment_staging()` reads that pending object after staging.
- `_scan_callback()` and `_local_costmap_callback()` mutate pending freshness booleans.
- `_dispatch_second_step_after_corridor_alignment_staging()` sets `pending['fresh_tf_received'] = True` after robot pose/TF evidence.
- `generate_second_step_forward_goal_after_staging(...)` produces `generated_after_fresh_evidence` and `selected_candidate_target` evidence.
- `_send_goal(..., 'explore', skip_two_step_staging=True)` is the intended recursion-prevention call for the second-step dispatch.

The Phase136 gap is not necessarily that these values do not exist internally. The gap is that the artifact stream did not serialize enough of them into `/maze/goal_events` or equivalent bounded artifacts at the right time.

## Pending second-step serialization contract

Future Phase138 should serialize a dedicated `pending_corridor_alignment_second_step` payload whenever staging is selected, after staging terminal success, and before any second-step explore dispatch. The minimum payload is:

```yaml
pending_corridor_alignment_second_step:
  exists: true
  runtime_serialized: true
  original_goal_kind: explore
  original_target: [x, y]
  direction_rad: yaw
  start_node_id: <int-or-null>
  active_branch: <serializable-branch-summary-or-null>
  staging_goal_pose:
    frame_id: map
    x: <float>
    y: <float>
    yaw: <float>
  staging_plan: <existing two_step_staging_plan subset>
  staging_result_status_label: SUCCEEDED
  staging_succeeded_wall_time_sec: <float>
  fresh_scan_received: <bool>
  fresh_local_costmap_received: <bool>
  fresh_tf_received: <bool>
  fresh_scan_sample_time_sec: <float-or-null>
  fresh_local_costmap_sample_time_sec: <float-or-null>
  fresh_tf_sample_time_sec: <float-or-null>
  freshness_sample_time_sec: <max sample time or null>
  invalid_reason: <string-or-null>
  cleared_reason: <string-or-null>
```

Required field semantics:

- `exists=true` means the pending second-step state exists and is expected to produce or fail-closed before producing a second-step dispatch.
- `runtime_serialized=true` means this exact runtime pending state was serialized from runtime data, not reconstructed offline by a runner.
- `original_goal_kind=explore` preserves that the original non-staging request was an exploration goal.
- `original_target` is the original exploration target before staging replaced the literal outgoing goal.
- `direction_rad` is the intended original direction/yaw used for second-step candidate generation.
- `start_node_id` and `active_branch` are diagnostic context only; they must not drive new branch scoring in Phase138.
- `staging_goal_pose` records the literal staging target used by Nav2.
- `staging_plan` records the pre-existing staging plan subset sufficient to explain why staging was inserted.
- `staging_result_status_label` must be `SUCCEEDED` before a second-step dispatch may be considered contract-valid.
- `staging_succeeded_wall_time_sec` anchors the freshness evidence chain.
- `fresh_scan_received`, `fresh_local_costmap_received`, and `fresh_tf_received` record post-staging runtime evidence.
- `freshness_sample_time_sec` records the max sample timestamp used for the generated second-step evidence.
- `invalid_reason` is null for a valid pending payload; otherwise it gives the fail-closed reason.
- `cleared_reason` records whether pending was cleared because the second-step was dispatched, invalidated, or failed closed.

If this payload is absent, has `exists=false`, has `runtime_serialized=false`, or has non-null `invalid_reason`, classification remains `SECOND_STEP_CONTRACT_AMBIGUOUS` unless the runner explicitly stops earlier with a not-ready fail-closed classification.

## second_step_forward_goal valid contract

A future artifact may claim second_step_forward_goal valid only when the outgoing second-step dispatch serializes the full payload used to choose the target. The minimal required fields are:

```yaml
second_step_forward_goal:
  valid: true
  generated_after_fresh_evidence: true
  selected_candidate_target: [x, y]
  selected_candidate_yaw: <float>
  front_wedge_risk_after_staging:
    max: <number>
    mean: <number>
    high_cost_count: <int>
    lethal_count: <int>
    sample_count: <int>
  lateral_residual_after: <float>
  candidate_count: <int>
  hard_safety_pass_candidate_count: <int>
  selected_candidate_index: <int>
  selection_priority_trace: [<strings>]
  rejected_candidate_summaries: [<objects>]
  map_frame_id: map
  goal_kind: explore
```

Validity rules:

- `generated_after_fresh_evidence=true` is mandatory.
- `selected_candidate_target` is mandatory and must contain finite numeric x/y/yaw evidence together with `selected_candidate_yaw`.
- `selected_candidate_yaw` must be finite numeric yaw.
- `front_wedge_risk_after_staging` must be computed after staging, not copied from pre-staging evidence.
- `lateral_residual_after` must correspond to the staging result / staged robot pose.
- `candidate_count`, `hard_safety_pass_candidate_count`, and `selected_candidate_index` are diagnostic evidence for candidate selection, not authorization to change selection.
- `selection_priority_trace` and `rejected_candidate_summaries` explain the decision without changing branch scoring.
- `map_frame_id=map` is required for target coordinates.
- `goal_kind=explore` is required on the outgoing second-step dispatch.
- valid=false if any required field is missing.

The outgoing dispatch must copy `selected_candidate_target` from second_step_forward_goal. More explicitly: selected_candidate_target must be copied from second_step_forward_goal. The top-level dispatch may also mirror `selected_candidate_target`, but the canonical source is `second_step_forward_goal.selected_candidate_target`.

## Freshness evidence chain

Future Phase138 should make the freshness evidence chain explicit and timestamped:

```yaml
freshness_evidence:
  staging_result_wall_time_sec: <float>
  fresh_scan_received: true
  fresh_scan_sample_time_sec: <float>
  fresh_local_costmap_received: true
  fresh_local_costmap_sample_time_sec: <float>
  fresh_tf_received: true
  fresh_tf_sample_time_sec: <float>
  generated_after_fresh_evidence: true
```

Required inequalities:

- fresh_scan_sample_time_sec > staging_result_wall_time_sec.
- fresh_local_costmap_sample_time_sec > staging_result_wall_time_sec.
- fresh_tf_sample_time_sec >= staging_result_wall_time_sec.

`generated_after_fresh_evidence=true only after all freshness booleans are true`. It must not be inferred merely because a second `goal_kind=explore` dispatch occurred. If any post-staging freshness boolean is false or any timestamp ordering is invalid, the artifact must remain `SECOND_STEP_CONTRACT_AMBIGUOUS` or fail closed before dispatch.

## Recursion guard serialization contract

The outgoing second-step dispatch must prove it cannot recursively trigger another staging step. The preferred artifact is explicit:

```yaml
skip_two_step_staging: true
recursion_guard={
  enabled: true,
  source: phase92_corridor_alignment_staging_second_step,
  skip_two_step_staging: true,
  original_staging_goal_sequence: <int>,
  outgoing_goal_kind: explore,
  two_step_stage_dispatch_requested: false,
  staging_applied: false,
  reason: prevent second-step from triggering staging again
}
```

Required semantics:

- `skip_two_step_staging=True` should be serialized directly from the `_send_goal(..., skip_two_step_staging=True)` call path.
- `source=phase92_corridor_alignment_staging_second_step` ties the outgoing dispatch to the staging-success chain.
- `two_step_stage_dispatch_requested=false` must be present on the outgoing second-step dispatch.
- `staging_applied=false on outgoing second-step dispatch` must be present to prove this literal goal is the explore second-step, not another staging goal.
- The artifact must explicitly state `prevent second-step from triggering staging again`.

If `skip_two_step_staging` cannot be serialized directly, an equivalent recursion guard must include all of the above fields plus a source field proving it was emitted by `_dispatch_second_step_after_corridor_alignment_staging()` after second_step_forward_goal validation.

## Event placement

To avoid offline reconstruction ambiguity, Phase138 should write fields at three event points:

1. Staging dispatch event:
   - staging plan;
   - original target;
   - pending payload skeleton with `exists=true` and freshness booleans false.

2. Staging terminal success event:
   - `staging_result_status_label=SUCCEEDED`;
   - `staging_succeeded_wall_time_sec` / `staging_result_wall_time_sec`;
   - pending payload with freshness state before second-step generation.

3. Second-step dispatch event:
   - serialized pending payload before clear;
   - serialized second_step_forward_goal;
   - copied selected_candidate_target;
   - freshness evidence chain;
   - recursion guard;
   - outgoing `goal_kind=explore`;
   - `third_goal_dispatched=false` remains a runner/analyzer responsibility.

## Classification rules after serialization

Classification remains diagnostic. The artifact must not convert accepted into success.

- If any required serialization field is missing, malformed, stale, or contradictory, classification remains `SECOND_STEP_CONTRACT_AMBIGUOUS`.
- If staging succeeded but post-staging freshness or second-step target generation is not ready, the future bounded runner should stop with `STAGING_SUCCEEDED_SECOND_STEP_NOT_READY_FAIL_CLOSED` and must not send the second-step dispatch.
- If all contract fields are present and the outgoing second-step explore is accepted, the future bounded classifier may use `STAGING_SUCCEEDED_SECOND_STEP_EXPLORE_ACCEPTED_STOP`.
- If all contract fields are present and the second-step result is rejected/aborted/canceled, use `STAGING_SUCCEEDED_SECOND_STEP_EXPLORE_REJECTED_DIAGNOSTIC_FAIL`.
- If all contract fields are present and the second-step result times out, use `STAGING_SUCCEEDED_SECOND_STEP_EXPLORE_TIMEOUT_DIAGNOSTIC_FAIL`.
- If all contract fields are present and the second-step result succeeds, use `STAGING_SUCCEEDED_SECOND_STEP_EXPLORE_RESULT_SUCCEEDED_STOP`, but this still must not be described as autonomous exploration success or exit success.

## Phase138 minimal implementation scope

Phase138 minimal implementation scope is artifact/serialization/diagnostic fields only.

Allowed future work, if explicitly authorized:

- Add focused static tests first.
- Serialize pending_corridor_alignment_second_step fields into `/maze/goal_events` or equivalent diagnostic artifacts.
- Serialize second_step_forward_goal fields into the outgoing second-step dispatch event.
- Serialize freshness timestamps and recursion guard fields.
- Update bounded analyzer checks for required field presence and timestamp ordering.

Forbidden future work in Phase138 minimal scope:

- No strategy change.
- No branch scoring change.
- No centerline change.
- No fallback change.
- No terminal acceptance change.
- No Nav2/MPPI/controller/goal checker/config change.
- No direct staging disablement.
- do not change target selection.
- do not change goal timing.
- Do not change staging eligibility thresholds or candidate generation.
- Do not use the serialization change to claim success.

Future bounded runtime must still stop after one second-step goal. The phrase is intentional: future bounded runtime must still stop after one second-step goal. Phase138, if authorized, should remain focused on proving the contract fields, not improving navigation behavior.

## Acceptance criteria for Phase138 design handoff

Before any future bounded runtime uses a second-step accepted/succeeded outcome as a valid second-step contract result, static tests should prove:

- pending payload required fields are documented and emitted by intended event points.
- second_step_forward_goal required fields are documented and emitted by the outgoing dispatch.
- freshness timestamps are post-staging and internally ordered.
- recursion guard is explicit and proves no recursive staging.
- missing fields force `SECOND_STEP_CONTRACT_AMBIGUOUS`.
- accepted remains not autonomous success, not exit success, and not Phase127 timeout repair.
