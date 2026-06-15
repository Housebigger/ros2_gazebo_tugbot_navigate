# Phase137 second-step contract serialization gap design report

Status: `PHASE137_SECOND_STEP_CONTRACT_SERIALIZATION_DESIGN_COMPLETE_STOP_BEFORE_PHASE138`

## Summary

Phase137 completed the Second-step contract serialization gap design review.

This phase is DESIGN_ONLY. It documents how a future Phase138 can close the Phase136 second-step artifact/serialization gap without changing navigation behavior.

## Scope compliance

Completed within requested scope:

- focused static tests were added.
- Phase136 report and analysis artifacts were read.
- `maze_explorer.py` was inspected read-only for existing pending/second-step/recursion-guard semantics.
- Proposal document was written.
- This report document was written.

Runtime and behavior guardrails preserved:

- No Gazebo/RViz/Nav2 runtime was launched.
- No NavigateToPose goal was sent.
- No maze_explorer was started.
- No staging/explore/third goal was sent.
- No Nav2/MPPI/controller/goal checker/config tuning was performed.
- No exploration strategy, branch scoring, centerline, fallback, or terminal acceptance change was made.
- No direct staging disablement was performed.
- No autonomous exploration success or exit success is claimed.
- Phase138 not entered.

## Delivered files

- `doc/doc_proposal/phase137_second_step_contract_serialization_design.md`
- `doc/doc_report/phase137_second_step_contract_serialization_design_report.md`
- `src/tugbot_maze/test/test_phase137_second_step_contract_serialization_design.py`

## Phase136 anchor retained

Phase137 preserves the Phase136 result:

- Phase136 final classification: `SECOND_STEP_CONTRACT_AMBIGUOUS`
- staging succeeded
- fresh scan/local_costmap/TF recorded
- unique second-step `goal_kind=explore` dispatch occurred
- `accepted=true`
- `third_goal_dispatched=false`

The accepted second-step remains diagnostic only because Phase136 did not prove:

- `pending_corridor_alignment_second_step present`
- `second_step_forward_goal valid`
- `generated_after_fresh_evidence=true`
- `selected_candidate_target present`
- `skip_two_step_staging=True` or equivalent recursion guard

Therefore the result must not be interpreted as autonomous exploration success, exit success, or Phase127 timeout repair.

## Design decisions

### Pending second-step serialization

The proposal defines a required `pending_corridor_alignment_second_step` serialization payload with:

- `exists=true`
- `runtime_serialized=true`
- `original_goal_kind=explore`
- `original_target`
- `direction_rad`
- `start_node_id`
- `active_branch`
- `staging_goal_pose`
- `staging_plan`
- `staging_result_status_label`
- `staging_succeeded_wall_time_sec`
- `fresh_scan_received`
- `fresh_local_costmap_received`
- `fresh_tf_received`
- freshness timestamps / `freshness_sample_time_sec`
- `invalid_reason`
- `cleared_reason`

Missing or non-runtime-serialized pending evidence keeps the classification ambiguous.

### second_step_forward_goal validity

The proposal defines `second_step_forward_goal valid` as requiring:

- `generated_after_fresh_evidence=true`
- `selected_candidate_target`
- `selected_candidate_yaw`
- `front_wedge_risk_after_staging`
- `lateral_residual_after`
- `candidate_count`
- `hard_safety_pass_candidate_count`
- `selected_candidate_index`
- `selection_priority_trace`
- `rejected_candidate_summaries`
- `map_frame_id=map`
- finite numeric x/y/yaw
- outgoing `goal_kind=explore`

Any missing required field means `valid=false` and classification remains `SECOND_STEP_CONTRACT_AMBIGUOUS` or fails closed before dispatch.

### Freshness evidence chain

The proposal defines an explicit freshness evidence chain:

- `staging_result_wall_time_sec`
- `fresh_scan_sample_time_sec > staging_result_wall_time_sec`
- `fresh_local_costmap_sample_time_sec > staging_result_wall_time_sec`
- `fresh_tf_sample_time_sec >= staging_result_wall_time_sec`
- `generated_after_fresh_evidence=true only after all freshness booleans are true`

This prevents inferring freshness from the existence of a second `goal_kind=explore` dispatch alone.

### Recursion guard

The proposal requires either direct `skip_two_step_staging=True` serialization or an equivalent serialized recursion guard:

- `source=phase92_corridor_alignment_staging_second_step`
- `skip_two_step_staging=true`
- original staging sequence linkage
- outgoing `goal_kind=explore`
- `two_step_stage_dispatch_requested=false`
- `staging_applied=false on outgoing second-step dispatch`
- explicit reason: `prevent second-step from triggering staging again`

### Event placement

The proposal defines three serialization points:

1. staging dispatch event;
2. staging terminal success event;
3. second-step dispatch event.

This avoids requiring Phase138 runners/analyzers to reconstruct contract-critical state offline.

## Phase138 minimal implementation boundary

If Phase138 is later authorized, its minimal implementation scope is artifact/serialization/diagnostic fields only.

Allowed future work:

- focused static tests first;
- serialize pending second-step fields;
- serialize second_step_forward_goal fields;
- serialize freshness timestamps and recursion guard fields;
- update bounded analyzer checks for field presence and timestamp ordering.

Forbidden future work remains:

- No strategy change.
- No branch scoring change.
- No centerline change.
- No fallback change.
- No terminal acceptance change.
- No Nav2/MPPI/controller/goal checker/config change.
- No direct staging disablement.
- do not change target selection.
- do not change goal timing.
- no full exploration.
- no third goal.

## Verification

Focused static tests were created first and verified RED before the proposal/report existed:

- command: `pytest -q src/tugbot_maze/test/test_phase137_second_step_contract_serialization_design.py`
- RED result: `8 failed in 0.04s`

Final verification:

- Focused Phase137 static tests:
  - command: `pytest -q src/tugbot_maze/test/test_phase137_second_step_contract_serialization_design.py`
  - result: `8 passed in 0.01s`
- Phase135-137 design/static bundle:
  - command: `pytest -q src/tugbot_maze/test/test_phase135_second_step_explore_after_staging_design.py src/tugbot_maze/test/test_phase136_bounded_second_step_explore_after_staging_smoke.py src/tugbot_maze/test/test_phase137_second_step_contract_serialization_design.py`
  - result: `24 passed in 0.04s`
- No-runtime process guard:
  - no matching `gazebo|gz sim|rviz2|nav2|slam_toolbox|maze_explorer|ros2 launch|component_container|controller_server|bt_navigator` processes.
- Protected config/launch diff guard:
  - `git diff -- src/tugbot_navigation/config src/tugbot_navigation/launch src/tugbot_maze/config src/tugbot_maze/launch` returned empty diff.

Verification log:

- `log/phase137_second_step_contract_serialization_design/phase137_final_verification2.log`

## Boundary conclusion

Phase137 is a design-only phase. It defines what Phase138 should serialize; it does not implement Phase138.

No autonomous exploration success is claimed. No exit success is claimed. Phase127 timeout is not claimed fixed. Phase138 not entered.
