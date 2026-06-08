# Phase58: Post-Ingress Candidate Formation Stability Replay

## Scope

Phase58 diagnoses candidate-formation stability across bounded post-ingress replays. It preserves accepted conclusions:
- Phase54: INGRESS_NAV2_GOAL_REACHED.
- Phase55: INGRESS_THEN_GATE_READY_NO_DISPATCH_TOPOLOGY_REJECTED.
- Phase56: candidate formation restored / first dispatch observed.
- Phase57: TIMEOUT_INCONCLUSIVE_DATA_GAP.

This phase does not tune Nav2/MPPI/controller parameters, does not tune clearance_radius_m, does not tune map sufficiency thresholds, and does not change maze_explorer strategy. It does not claim autonomous exploration success, does not treat first dispatch as exit success, and does not attribute the Phase57 first-dispatch timeout.

Run id: phase58_post_ingress_candidate_formation_stability_replay
Artifact dir: log/phase58_post_ingress_candidate_formation_stability_replay/

## Runtime method

Bounded replay count: 2
Each replay used:
- active scaled2x world: tugbot_maze_world_20260528_clean_scaled2x.sdf
- active maze metadata: maze_20260528_scaled_instance.yaml
- SLAM + Nav2 bringup
- ingress Nav2 goal: ingress_waypoint_map=(1.0, 0.0, 0.0)
- post-ingress maze_explorer start
- max_goals=1
- near_exit_fallback_enabled=false
- startup_warmup_no_dispatch=false

## Classification

CANDIDATE_FORMATION_UNSTABLE_DEAD_END_POLICY_SENSITIVE

Reason: bounded replays produced mixed dispatch/no-dispatch outcomes after successful ingress. The pose, costmap, and scan evidence were very similar, but the dead-end/junction policy state differed:
- replay_01: no dispatch, dead_end_policy_before_candidate_generation, branch_candidate_rejection_reason=dead_end_policy_no_branch_options.
- replay_02: dispatch observed, candidate_selected_for_dispatch.

## Replay comparison

Aggregate comparison:
- replay_count: 2
- dispatch_count: 1
- no_dispatch_count: 1
- mixed_dispatch_and_no_dispatch: true
- difference_hypothesis: dead_end_policy_sensitive
- pose_spread_at_topology_m: 0.0018258069888218438
- timing_spread_explorer_start_to_topology_sec: 1.1719486713409424
- raw_open_direction_counts: [1, 2]
- filtered_open_direction_counts: [0, 2]
- candidate_after_filter_counts: [0, 2]
- candidate_local_cost_value_spread: 0.0
- candidate_local_cost_max_radius_spread: 0.0
- rejection_reasons: [None, dead_end_policy_no_branch_options]
- dead_end_junction_policy_states: [candidate_selected_for_dispatch, dead_end_policy_before_candidate_generation]

### replay_01: no dispatch

- ingress_success: true
- dispatch_observed: false
- explorer_state_sample_count: 15
- goal_events_sample_count: 0
- stability_evidence_sample_count: 77
- robot_pose_at_first_topology_sampling: [0.8637526863480652, 0.020257575021374215, 0.029820230347893707]
- time_from_ingress_success_to_explorer_start_sec: 3.3093132972717285
- time_from_explorer_start_to_gate_ready_sec: 2.790322780609131
- time_from_explorer_start_to_first_topology_sec: 2.790322780609131
- raw_open_direction_count: 1
- filtered_open_direction_count: 0
- accepted_open_direction_angle_deg: 91.70857334304223
- accepted_open_direction_vector: [-0.02981581095694924, 0.9995554098783016]
- candidate_before_filter_count: 0
- candidate_after_filter_count: 0
- junction_or_dead_end_policy_filter: dead_end_policy_before_candidate_generation
- duplicate_or_exhausted_filter: no_candidate_created
- branch_candidate_rejection_reason: dead_end_policy_no_branch_options
- candidate_local_cost_value: 49
- max_radius_cost: 99
- map known ratio near robot at topology: 0.8378164556962026
- scan finite_count at topology: 406
- nearest_obstacle_m at topology: 0.9291761517524719

### replay_02: dispatch reproduced

- ingress_success: true
- dispatch_observed: true
- explorer_state_sample_count: 45
- goal_events_sample_count: 2
- stability_evidence_sample_count: 77
- robot_pose_at_first_topology_sampling: [0.8655784904235088, 0.020260836696480176, 0.03183338103694765]
- time_from_ingress_success_to_explorer_start_sec: 3.419926166534424
- time_from_explorer_start_to_gate_ready_sec: 3.9622714519500732
- time_from_explorer_start_to_first_topology_sec: 3.9622714519500732
- raw_open_direction_count: 2
- filtered_open_direction_count: 2
- accepted_open_direction_angle_deg: 91.82391838104888
- accepted_open_direction_vector: [-0.03182800484151353, 0.9994933607122203]
- candidate_before_filter_count: 2
- candidate_after_filter_count: 2
- junction_or_dead_end_policy_filter: candidate_selected_for_dispatch
- duplicate_or_exhausted_filter: selectable
- branch_candidate_rejection_reason: null
- candidate_local_cost_value: 49
- max_radius_cost: 99
- map known ratio near robot at topology: 0.8370429252782194
- scan finite_count at topology: 404
- nearest_obstacle_m at topology: 0.9319090843200684

## Interpretation

The replay pair reproduces the Phase56/Phase57 instability pattern in a bounded way: one run forms candidates and dispatches, one run reaches gate/topology but terminalizes before candidate generation as dead_end_policy_no_branch_options.

The strongest discriminator is not local candidate cost: both runs report candidate local cost 49 and max radius cost 99. It is also not a large ingress pose divergence: topology pose spread is about 0.00183 m. The observed split is tied to topology/dead-end policy state: one raw open direction in replay_01 is treated as dead-end policy before candidate generation, while two raw/filtered directions in replay_02 allow candidate selection.

This is not a first-dispatch execution timeout diagnosis. Phase57 remains TIMEOUT_INCONCLUSIVE_DATA_GAP. This report does not attribute first-dispatch timeout.

## Recommended next step candidates

Do not tune parameters yet. Evidence-supported next design options are:
- post-ingress stable wait before topology sampling;
- explicit pose alignment check at explorer start and first topology sampling;
- multi-frame topology consistency check before dead_end_policy terminalization.

## Guardrail verification

- complete_autonomous_success_claimed: false
- first_dispatch_timeout_attributed: false
- nav2_config_diff_empty: true
- cleanup_empty: true
- guardrail_violation: false

Final command verification is recorded in the final response.
