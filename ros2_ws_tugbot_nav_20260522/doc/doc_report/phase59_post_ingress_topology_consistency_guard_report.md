# Phase59 Post-Ingress Multi-frame Topology Consistency Guard Report

run_id: phase59_post_ingress_topology_consistency_guard

## Context

Phase58 manual acceptance passed with the conclusion kept as:

CANDIDATE_FORMATION_UNSTABLE_DEAD_END_POLICY_SENSITIVE

Phase58 showed that two bounded post-ingress replays had close ingress pose/map/scan/costmap context, but topology sampling diverged: one replay produced raw_open=1/candidate=0/dead_end_policy_no_branch_options while another produced raw_open=2/candidate=2/dispatch. Phase59 therefore adds a conservative multi-frame consistency guard before no-candidate/dead-end terminalization.

This report intentionally does not claim autonomous exploration success. It also keeps the rule that first dispatch is not exit success; a dispatch observed by this phase is only evidence that the consistency guard allowed candidate recovery / first dispatch observed.

## Implementation

Files added or changed:

- src/tugbot_maze/tugbot_maze/maze_explorer.py
- tools/run_phase59_post_ingress_topology_consistency_guard.sh
- tools/analyze_phase59_post_ingress_topology_consistency_guard.py
- src/tugbot_maze/test/test_phase59_post_ingress_topology_consistency_guard.py
- doc/doc_report/phase59_post_ingress_topology_consistency_guard_report.md

maze_explorer now declares conservative parameters:

- topology_consistency_enabled=true
- topology_consistency_required_no_candidate_frames=2
- topology_consistency_window_sec=4.0

The guard is scoped only to no-candidate/dead-end terminalization paths:

- dead_end_policy_no_branch_options
- node_policy_no_untried_branch when no backtrack target remains

Valid candidate dispatch is not delayed. If a candidate appears during the window, the guard records candidate_recovered_during_consistency_window and normal dispatch proceeds.

Structured diagnostics include:

- topology_consistency_frame_index
- topology_consistency_frames
- topology_consistency_frame_count
- raw_open_direction_count
- filtered_open_direction_count
- candidate_before_filter_count
- candidate_after_filter_count
- dead_end_policy_state
- terminalization_delayed
- candidate_recovered_during_consistency_window
- topology_consistency_guard_status
- topology_consistency_terminalization_reason

## Guardrails

The Phase59 wrapper/analyzer preserve these guardrails:

- active scaled2x world only: tugbot_maze_world_20260528_clean_scaled2x.sdf
- active maze metadata only: maze_20260528_scaled_instance.yaml
- bounded replay only
- max_goals=1
- no Nav2/MPPI/controller parameter edits
- no clearance_radius_m tuning
- no map sufficiency threshold tuning
- no branch selection strategy change
- no fallback/terminal acceptance
- no old scaffold world/map
- no autonomous exploration success claim
- first dispatch is not exit success
- does not attribute first-dispatch timeout

## Analyzer classifications

Allowed Phase59 classifications:

- TOPOLOGY_CONSISTENCY_GUARD_IMPLEMENTED_STATIC_ONLY
- TOPOLOGY_CONSISTENCY_RECOVERED_CANDIDATE_AND_DISPATCH
- TOPOLOGY_CONSISTENCY_CONFIRMED_NO_CANDIDATE
- TOPOLOGY_CONSISTENCY_INCONCLUSIVE_RUNTIME_VARIANCE
- GUARDRAIL_VIOLATION_STRATEGY_CHANGED

Runtime interpretation:

- If dispatch appears, report only consistency guard allowed candidate recovery / first dispatch observed.
- If no dispatch appears, report whether continuous N frames confirmed no-candidate or runtime data was insufficient.
- Do not enter a long autonomous run from Phase59.

## Verification log

Initial RED test was established before implementation: 6 focused tests failed before guard/artifacts were implemented.

Final verification results:

- python3 -m py_compile src/tugbot_maze/tugbot_maze/maze_explorer.py: PASS
- python3 -m py_compile tools/analyze_phase59_post_ingress_topology_consistency_guard.py: PASS
- bash -n tools/run_phase59_post_ingress_topology_consistency_guard.sh: PASS
- python3 -m pytest src/tugbot_maze/test/test_phase59_post_ingress_topology_consistency_guard.py -q: PASS, 6 passed
- colcon build --packages-select tugbot_maze tugbot_bringup --symlink-install: PASS, 2 packages finished
- git diff -- src/tugbot_navigation/config | wc -c: 0
- cleanup process check: 0 matching ROS/Gazebo/Nav2/maze_explorer processes after cleanup

## Runtime evidence

Runtime artifact directory:

log/phase59_post_ingress_topology_consistency_guard/

Bounded replay settings:

- replay_count=2
- max_goals=1
- active world: tugbot_maze_world_20260528_clean_scaled2x.sdf
- topology_consistency_enabled=true
- topology_consistency_required_no_candidate_frames=2
- topology_consistency_window_sec=4.0

Analyzer summary:

- classification: TOPOLOGY_CONSISTENCY_CONFIRMED_NO_CANDIDATE
- replay_count: 2
- ingress_goal_success: true in replay_01 and replay_02
- terminalization_delayed: true
- confirmed_no_candidate_frames: 2
- consistency_frame_count: 4 after analyzer de-duplication, 2 unique frames per replay
- replay_01 frame counts: raw_open_direction_count=[1, 1], candidate_after_filter_count=[0, 0], dead_end_policy_state=[pending, confirmed]
- replay_02 frame counts: raw_open_direction_count=[1, 1], candidate_after_filter_count=[0, 0], dead_end_policy_state=[pending, confirmed]
- dispatch_observed: false
- candidate_recovered_during_consistency_window: false
- complete_autonomous_success_claimed: false
- first_dispatch_observed_not_exit_success: false
- nav2_config_diff_empty: true
- cleanup_empty: true

Interpretation: Phase59 confirmed no-candidate by consecutive N=2 no-candidate topology frames in both bounded replays. No first dispatch occurred in the Phase59 bounded runtime. This is not an autonomous exploration success claim.

## Current conclusion

Phase59 implementation is a bounded guard implementation for post-ingress no-candidate topology consistency. It preserves the Phase58 conclusion CANDIDATE_FORMATION_UNSTABLE_DEAD_END_POLICY_SENSITIVE and does not claim autonomous exploration success. First dispatch, if observed in a later Phase59 rerun, remains not exit success.

Final Phase59 classification for this run:

TOPOLOGY_CONSISTENCY_CONFIRMED_NO_CANDIDATE
