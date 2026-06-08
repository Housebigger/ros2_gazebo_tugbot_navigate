# Phase57: First Dispatch Navigation Timeout Diagnostics

## Scope

Phase57 attempted bounded first-dispatch execution diagnostics on the active scaled2x world, reusing the Phase56 chain:

active scaled2x world + SLAM + Nav2 -> ingress waypoint map=(1.0, 0.0, 0.0) -> post-ingress maze_explorer with max_goals=1.

Guardrails preserved:
- Active scaled2x world only: tugbot_maze_world_20260528_clean_scaled2x.sdf.
- No old scaffold world/map.
- No Nav2/MPPI/controller parameter edits.
- No clearance_radius_m tuning.
- No map sufficiency threshold tuning.
- No maze_explorer strategy changes.
- near_exit_fallback_enabled=false.
- startup_warmup_no_dispatch=false.
- max_goals=1.
- No fallback/terminal acceptance.
- No autonomous exploration success claim; first dispatch is not exit success.

## Added diagnostics

Implementation files:
- tools/run_phase57_first_dispatch_navigation_timeout_diagnostics.sh
- tools/analyze_phase57_first_dispatch_navigation_timeout_diagnostics.py
- src/tugbot_maze/test/test_phase57_first_dispatch_navigation_timeout_diagnostics.py

Primary artifacts:
- log/phase57_first_dispatch_navigation_timeout_diagnostics/phase57_first_dispatch_navigation_timeout_diagnostics.json
- log/phase57_first_dispatch_navigation_timeout_diagnostics/first_dispatch_execution_summary.json
- log/phase57_first_dispatch_navigation_timeout_diagnostics/first_dispatch_execution.jsonl
- log/phase57_first_dispatch_navigation_timeout_diagnostics/controller_dynamics.jsonl
- log/phase57_first_dispatch_navigation_timeout_diagnostics/nav2_feedback.jsonl
- log/phase57_first_dispatch_navigation_timeout_diagnostics/local_costmap_samples.jsonl
- log/phase57_first_dispatch_navigation_timeout_diagnostics/global_plan_samples.jsonl
- log/phase57_first_dispatch_navigation_timeout_diagnostics/collision_monitor_state.jsonl
- log/phase57_first_dispatch_navigation_timeout_diagnostics/phase57_first_dispatch_navigation_timeout_diagnostics_explorer_state.jsonl
- log/phase57_first_dispatch_navigation_timeout_diagnostics/phase57_first_dispatch_navigation_timeout_diagnostics_goal_events.jsonl

## Runtime result

Bounded runtime completed and cleaned up, but the Phase57 run did not reproduce the Phase56 first-dispatch condition. Ingress and readiness were successful, but maze_explorer reached FAILED_EXHAUSTED before any /maze/goal_events dispatch event appeared.

Key evidence from phase57_first_dispatch_navigation_timeout_diagnostics.json:
- classification: TIMEOUT_INCONCLUSIVE_DATA_GAP
- ingress_reached: true
- readiness_ready: true
- controller_server_active: true
- planner_server_active: true
- bt_navigator_active: true
- navigate_to_pose_action_servers_1: true
- goal_pose_subscription_count_1: true
- max_goals_limited_to_one: true
- near_exit_fallback_disabled: true
- startup_warmup_no_dispatch_false: true
- guardrail_violation: false
- complete_autonomous_success_claimed: false
- nav2_config_diff_empty: true
- cleanup_empty: true

Sampling/evidence counts:
- explorer_state_sample_count: 16
- goal_events_sample_count: 0
- first_dispatch_execution_sample_count: 0
- controller_dynamics.jsonl: 2848 odom/controller rows
- first_dispatch_execution.jsonl: 0 rows
- nav2_feedback.jsonl: 0 rows
- local_costmap_samples.jsonl: 0 rows
- global_plan_samples.jsonl: 0 rows
- collision_monitor_state.jsonl: 0 rows

Because there was no dispatch event, there was no active first-dispatch goal window. Therefore no valid Nav2 feedback, local-costmap target/path timeline, global-plan timeline, or collision-monitor execution window could be attributed to first-dispatch execution in this Phase57 run.

## Topology state at bounded run failure

The first topology state containing diagnostics showed:
- mode: FAILED_EXHAUSTED
- goal_count: 0
- dispatch_readiness_gate_passed: true
- dispatch_readiness_first_pass_time_sec: 28.002
- robot_pose_map: [0.8622262065196014, 0.020015498181262265, 0.02760224591715785]
- raw_open_direction_count: 1
- reject_reason_counts: {accepted_open_direction: 1, clearance_radius_blocked: 3}

Phase56 open-direction-to-candidate diagnostic payload embedded in explorer_state for this Phase57 run:
- phase56_raw_open_direction_count: 1
- phase56_filtered_open_direction_count: 0
- candidate_before_filter_count: 0
- candidate_after_filter_count: 0
- branch_candidate_rejection_reason: dead_end_policy_no_branch_options
- candidate_goal_point: [0.5735315842699404, 0.5122353853637351]
- chosen_candidate_goal_point: null
- candidate_map_cell_state: free, clearance_result=clear, nearest_obstacle_distance_m=0.4000000059604645
- candidate_local_costmap_cell_state: lethal_or_obstacle, value=49, max_radius_cost=99

This is not a first-dispatch execution timeout sample. It is a bounded data-gap run: post-ingress topology produced no selected branch, so the Phase57 recorder could not observe a first-dispatch execution window.

## Classification

Final Phase57 classification:

TIMEOUT_INCONCLUSIVE_DATA_GAP

Rationale:
- Phase57 intended to classify the first-dispatch Nav2 execution timeout cause.
- In this bounded runtime, no first dispatch occurred: /maze/goal_events was empty, first_dispatch_event was null, and first_dispatch_execution.jsonl was empty.
- Without a dispatch/outcome window, the required target cost/path cost/footprint/front-wedge/cmd_vel/Nav2 feedback/collision-monitor execution evidence cannot be tied to first-dispatch execution.
- Guardrails were preserved and cleanup succeeded.

This does not alter accepted Phase55/Phase56 conclusions and does not claim autonomous exploration success.

## Verification status

Verification commands:
- python3 -m py_compile tools/analyze_phase57_first_dispatch_navigation_timeout_diagnostics.py
- bash -n tools/run_phase57_first_dispatch_navigation_timeout_diagnostics.sh
- python3 -m pytest src/tugbot_maze/test/test_phase57_first_dispatch_navigation_timeout_diagnostics.py -q
- source /opt/ros/jazzy/setup.bash && colcon build --packages-select tugbot_maze tugbot_bringup --symlink-install
- git diff -- src/tugbot_navigation/config

Recorded status:
- Nav2 config diff: empty
- cleanup process check: empty
