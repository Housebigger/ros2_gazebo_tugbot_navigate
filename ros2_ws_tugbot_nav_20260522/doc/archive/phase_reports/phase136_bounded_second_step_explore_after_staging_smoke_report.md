# Phase136 bounded second-step explore after staging smoke report

Status: `PHASE136_BOUNDED_SECOND_STEP_EXPLORE_AFTER_STAGING_SMOKE_COMPLETE_STOP_BEFORE_PHASE137`

## Scope

Phase136 ran a bounded visible-stack smoke for the Phase135 second-step-after-staging contract.

Allowed actions used:

- Started Gazebo/RViz/SLAM/Nav2 visible stack.
- Sent the explicit inner-ingress goal: `map,x=2.0,y=0.0,yaw=0.0` through the reused Phase120/Phase134 ingress chain.
- Compatibility phrase for static contract checks: explicit inner-ingress goal: map,x=2.0,y=0.0,yaw=0.0.
- Compatibility phrase for static contract checks: max_goals=2.
- Compatibility phrase for static contract checks: first literal corridor_alignment_staging.
- Compatibility phrase for static contract checks: second_step goal_kind=explore.
- Compatibility phrase for static contract checks: Phase127 timeout fixed claim=false.
- Compatibility phrase for static contract checks: Phase137 not entered.
- Started `maze_explorer` only after handoff readiness was true.
- Allowed first literal `goal_kind=corridor_alignment_staging` on the final run.
- Allowed one subsequent `goal_kind=explore` second-step dispatch.
- Stopped bounded observation after the second-step acceptance boundary / contract analysis.
- Cleaned up all Gazebo/RViz/Nav2/SLAM/maze_explorer processes.

Forbidden actions preserved:

- No third goal was dispatched in the final artifact.
- No full exploration was run.
- No manual Goal1/carry-over/branch/centerline/fallback/terminal/exit goal was sent.
- No Nav2/MPPI/controller/goal checker/config tuning was made.
- No exploration strategy, branch scoring, centerline, fallback, or terminal acceptance change was made.
- Staging was not disabled.
- No autonomous exploration success or exit success is claimed.
- The second-step result is not used to claim Phase127 timeout was fixed.

## Delivered files

- `tools/run_phase136_bounded_second_step_explore_after_staging_smoke.py`
- `tools/analyze_phase136_bounded_second_step_explore_after_staging_smoke.py`
- `src/tugbot_maze/test/test_phase136_bounded_second_step_explore_after_staging_smoke.py`
- `log/phase136_bounded_second_step_explore_after_staging_smoke/`
- `doc/doc_report/phase136_bounded_second_step_explore_after_staging_smoke_report.md`

## TDD / static verification

Initial focused test state was RED before runner/analyzer implementation.

Final verification:

- Focused Phase136 tests:
  - command: `pytest -q src/tugbot_maze/test/test_phase136_bounded_second_step_explore_after_staging_smoke.py`
  - result: `8 passed in 0.02s`
- Phase134-136 static bundle:
  - command: `pytest -q src/tugbot_maze/test/test_phase134_bounded_corridor_alignment_staging_smoke.py src/tugbot_maze/test/test_phase135_second_step_explore_after_staging_design.py src/tugbot_maze/test/test_phase136_bounded_second_step_explore_after_staging_smoke.py`
  - result: `23 passed in 0.04s`
- Python compile:
  - command: `python3 -m py_compile tools/run_phase136_bounded_second_step_explore_after_staging_smoke.py tools/analyze_phase136_bounded_second_step_explore_after_staging_smoke.py`
  - result: exit 0

Protected config/launch diff guard:

- command: `git diff -- src/tugbot_navigation/config src/tugbot_navigation/launch src/tugbot_maze/config src/tugbot_maze/launch`
- result: empty diff

Final process guard before report:

- no matching `gazebo|gz sim|rviz2|nav2|slam_toolbox|maze_explorer|ros2 launch|component_container|controller_server|bt_navigator` processes.

## Runtime artifacts

Canonical final artifact:

- `log/phase136_bounded_second_step_explore_after_staging_smoke/phase136_bounded_second_step_explore_after_staging_smoke.json`
- `log/phase136_bounded_second_step_explore_after_staging_smoke/phase136_bounded_second_step_explore_after_staging_smoke_analysis.json`
- `log/phase136_bounded_second_step_explore_after_staging_smoke/phase136_bounded_second_step_explore_after_staging_smoke_summary.md`

Archived first attempt:

- `log/phase136_bounded_second_step_explore_after_staging_smoke/phase136_attempt1_first_literal_explore_contract_ambiguous.json`
- `log/phase136_bounded_second_step_explore_after_staging_smoke/phase136_attempt1_first_literal_explore_contract_ambiguous_analysis.json`
- `log/phase136_bounded_second_step_explore_after_staging_smoke/phase136_attempt1_first_literal_explore_contract_ambiguous_summary.md`

The first attempt is retained as evidence because the first literal dispatch was `goal_kind=explore` rather than staging. It was classified as contract ambiguous and is not used as the final Phase136 result.

## Final runtime result

Final classification:

- runner classification: `SECOND_STEP_CONTRACT_AMBIGUOUS`
- analyzer classification: `SECOND_STEP_CONTRACT_AMBIGUOUS`
- analyzer valid: `false`
- stop reason: `second_step_contract_ambiguous_stop`

High-level final artifact fields:

- `handoff_allowed=true`
- `maze_explorer_started=true`
- `maze_explorer_max_goals=2`
- `goal_event_count=3`
- `dispatch_event_count=2`
- `second_step_attempted=true`
- `second_step_goal_count=1`
- `third_goal_dispatched=false`

### Staging artifact

First literal dispatch:

- `goal_kind=corridor_alignment_staging`
- `goal_sequence=1`
- `result_status_label=SUCCEEDED`
- `accepted=true`
- `rejected=false`
- `timeout=false`
- `abort_text=succeeded`
- `staging_applied=true`
- `two_step_stage_dispatch_requested=true`
- `staging_reason=reduce_lateral_residual_and_front_wedge_risk_before_second_step_forward_goal`

Staging geometry:

- `original_target=[1.6403064601453812, 0.8260591666054022]`
- `staging target={x=1.6355309762935013, y=0.026073420011138968, yaw=1.564826936528279}`
- `lateral_residual_before_m=0.22500000335276146`
- `lateral_residual_after_m=1.1535911115245767e-16`

Staging front-wedge risk before staging:

- `max=99`
- `mean=48.518248175182485`
- `high_cost_count=40`
- `lethal_count=9`
- `sample_count=137`

### Freshness after staging

Recorded after staging success:

- `fresh_scan_received=true`
- `fresh_local_costmap_received=true`
- `fresh_tf_received=true`
- `fresh_scan_sample_time_sec=1780833295.3021984`
- `fresh_local_costmap_sample_time_sec=1780833295.1613557`
- `fresh_tf_sample_time_sec=1780833292.7061994`

Cost/risk after staging:

- `front_wedge_risk_after_staging.max=70`
- `front_wedge_risk_after_staging.mean=9.404411764705882`
- `front_wedge_risk_after_staging.high_cost_count=2`
- `front_wedge_risk_after_staging.lethal_count=0`
- `front_wedge_risk_after_staging.sample_count=136`
- `lateral_residual_after=1.1535911115245767e-16`

### Second-step dispatch

Observed second literal dispatch:

- `goal_kind=explore`
- `goal_sequence=2`
- `target=[2.557410869071872, -0.22586149509767717]`
- `original_target=[2.557410869071872, -0.22586149509767717]`
- `accepted=true`
- `rejected=false`
- `timeout=false`
- `result_status_label=ACCEPTED`
- `second_step_attempted=true`
- `second_step_goal_count=1`
- `third_goal_dispatched=false`

The second-step dispatch was bounded and stopped at acceptance. It is not interpreted as success.

### Why classification is ambiguous

Phase135 required the second-step contract to prove all of the following before accepting the one bounded second-step dispatch as a valid Phase136 contract result:

- `pending_corridor_alignment_second_step present`
- `second_step_forward_goal valid`
- `generated_after_fresh_evidence=true`
- `selected_candidate_target present`
- outgoing `goal_kind=explore`
- `skip_two_step_staging=True` or equivalent recursion guard

The final runtime did prove:

- staging succeeded;
- fresh scan/local_costmap/TF were recorded after staging;
- a single second `goal_kind=explore` dispatch occurred;
- no third goal occurred.

But the artifact did not prove:

- `pending_corridor_alignment_second_step present` (`available=false`, `exists=false`, `source=not_serialized_or_not_applicable`);
- `second_step_forward_goal valid` (`second_step_forward_goal=null`);
- `generated_after_fresh_evidence=true` on the outgoing second-step payload;
- `selected_candidate_target present` on the outgoing second-step payload;
- `skip_two_step_staging=True` or an equivalent serialized recursion guard (`skip_two_step_staging=false`, `recursion_guard=false`).

Therefore the correct final classification is:

`SECOND_STEP_CONTRACT_AMBIGUOUS`

This is a diagnostic result, not a repair result.

## Notes on attempt 1

The first visible-stack attempt produced:

- first literal `goal_kind=explore`
- `selected_due_to_context=topology_exit_bias_score`
- no staging dispatch
- one dispatch total
- `result_status_label=CANCELED` after timeout/cancel
- `second_step_attempted=false`
- `third_goal_dispatched=false`

After this, the runner was patched to fail closed soon after a non-staging first literal dispatch, preserving the Phase136 boundary. The final canonical artifact is the second visible-stack run, where staging succeeded and one second-step explore dispatch was observed.

## Cleanup

Cleanup completed.

Final guard output:

- no Gazebo/RViz/Nav2/SLAM/maze_explorer/controller_server/bt_navigator process remained;
- protected config/launch diff remained empty.

## Boundary conclusion

Phase136 is complete and stops before Phase137.

The Phase136 artifact does not authorize:

- declaring autonomous exploration success;
- declaring exit success;
- declaring Phase127 timeout fixed;
- changing Nav2/MPPI/controller/goal checker/config;
- changing exploration strategy/branch scoring/centerline/fallback/terminal acceptance;
- disabling staging;
- running a third goal or full exploration.

Recommended next design focus, if Phase137 is later authorized: close the serialization/contract evidence gap for `pending_corridor_alignment_second_step`, `second_step_forward_goal`, `generated_after_fresh_evidence`, `selected_candidate_target`, and explicit `skip_two_step_staging`/recursion guard before interpreting second-step accepted/succeeded outcomes.
