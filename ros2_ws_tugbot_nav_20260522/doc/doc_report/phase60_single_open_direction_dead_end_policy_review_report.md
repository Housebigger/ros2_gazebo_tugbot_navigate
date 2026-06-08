# Phase60 Single Open Direction Dead-End Policy Review Report

Run id: `phase60_single_open_direction_dead_end_policy_review`  
Artifact dir: `log/phase60_single_open_direction_dead_end_policy_review/`

## Scope

Phase60 is a read-only static/offline policy review. It reviews whether the current `maze_explorer` dead-end policy treats the post-ingress entrance corridor's single open direction as a terminal dead-end before candidate generation.

Preserved accepted conclusions:

- Phase58 conclusion remains `CANDIDATE_FORMATION_UNSTABLE_DEAD_END_POLICY_SENSITIVE`.
- Phase59 runtime classification remains `TOPOLOGY_CONSISTENCY_CONFIRMED_NO_CANDIDATE`.

This report does not claim autonomous exploration success. Any first dispatch remains first-dispatch/candidate-formation evidence only; first dispatch is not exit success.

## Guardrails

- No Gazebo/SLAM/Nav2 runtime was launched in Phase60.
- No Nav2/MPPI/controller parameters were tuned.
- No `clearance_radius_m` tuning was performed.
- No map sufficiency threshold tuning was performed.
- No branch selection runtime strategy was modified.
- No fallback/terminal acceptance was used.
- No old scaffold world/map was used.
- No long run was performed.
- Cleanup check is required and must be empty.

## Static review summary

Source evidence:

- `maze_perception.classify_local_topology()` maps `count == 1` to `DEAD_END`.
- `maze_explorer._analyze_and_dispatch()` short-circuits on `local.kind == DEAD_END` before normal branch candidate creation.
- In that short-circuit, Phase56 diagnostics record `filtered_open_directions=[]`, `candidate_before_filter_count=0`, `candidate_after_filter_count=0`, and `branch_candidate_rejection_reason='dead_end_policy_no_branch_options'`.
- `MazeTopology.choose_next_branch()` can only choose from node branch options; with `branch_options=[]`, no candidate can be selected.
- Current dead-end path has no explicit post-ingress / entrance-corridor / first-node context tag.

Interpretation for the core question:

- A. True dead-end: not proven from current offline evidence.
- B. Legal post-ingress first corridor direction: plausible, because the source has a raw single open direction and map-clear candidate geometry before policy short-circuit.
- C. Special topology needing entrance/ingress + visited-edge/backtracking context: strongest review conclusion for Phase60, because the open direction is single and stable but current policy lacks an incoming-edge/post-ingress distinction.

## Offline Phase59 first topology replay review

### replay_01

- `robot_pose_map`: `[0.8552441409604281, 0.02024027480488233, 0.02967662470065554]`; `robot_yaw_rad`: `0.02967662470065554`.
- accepted open direction: `91.700 deg`, vector `[-0.029672268848417147, 0.9995596812904106]`.
- angle to entrance yaw / ingress vector: `91.700 deg` / `91.700 deg`.
- angle to return-to-entrance / opposite-return direction: `89.655 deg` / `90.345 deg`.
- `roughly_points_into_maze`: `False`; `opposite_return_to_entrance`: `False`; `not_aligned_with_return_to_entrance`: `True`.
- raw/filtered/candidate-after: `1` / `[0, 0]` / `[0, 0]`.
- `candidate_goal_point`: `[0.5655290900853416, 0.5118602413951814]`; `candidate_clearance_result`: `clear`.
- map cell: `{'available': True, 'cell': [15, 31], 'clearance_result': 'clear', 'in_bounds': True, 'nearest_obstacle_distance_m': 0.4000000059604645, 'state': 'free', 'value': 0}`.
- local costmap cell: `{'available': True, 'cell': [24, 39], 'in_bounds': True, 'max_radius_cost': 99, 'state': 'lethal_or_obstacle', 'value': 49}`.
- dead-end policy: state `['pending', 'confirmed']`, filter `dead_end_policy_before_candidate_generation`, rejection `dead_end_policy_no_branch_options`.
### replay_02

- `robot_pose_map`: `[0.8554564582511112, 0.020045128143238992, 0.029217197064095833]`; `robot_yaw_rad`: `0.029217197064095833`.
- accepted open direction: `91.674 deg`, vector `[-0.02921304039107807, 0.9995732080598746]`.
- angle to entrance yaw / ingress vector: `91.674 deg` / `91.674 deg`.
- angle to return-to-entrance / opposite-return direction: `89.668 deg` / `90.332 deg`.
- `roughly_points_into_maze`: `False`; `opposite_return_to_entrance`: `False`; `not_aligned_with_return_to_entrance`: `True`.
- raw/filtered/candidate-after: `1` / `[0, 0]` / `[0, 0]`.
- `candidate_goal_point`: `[0.5659673017430362, 0.51179814594592]`; `candidate_clearance_result`: `clear`.
- map cell: `{'available': True, 'cell': [15, 31], 'clearance_result': 'clear', 'in_bounds': True, 'nearest_obstacle_distance_m': 0.4000000059604645, 'state': 'free', 'value': 0}`.
- local costmap cell: `{'available': True, 'cell': [24, 39], 'in_bounds': True, 'max_radius_cost': 99, 'state': 'lethal_or_obstacle', 'value': 49}`.
- dead-end policy: state `['pending', 'confirmed']`, filter `dead_end_policy_before_candidate_generation`, rejection `dead_end_policy_no_branch_options`.


Key observation: both Phase59 replays have stable `raw_open_direction_count=1` and `candidate_after_filter_count=[0, 0]`, with `dead_end_policy_state=[pending, confirmed]`. The accepted open direction is near 91.7 deg in map frame, not aligned with entrance yaw / ingress vector (+X) and not aligned with the return-to-entrance vector either. The hypothetical candidate point is map-clear (`clear`) but has a local-cost caveat (`state=lethal_or_obstacle`, `max_radius_cost=99`). Phase60 does not convert this into dispatch behavior; it only records that the policy terminalizes before candidate generation.

## Candidate strategy review options

- `keep_current_policy`: `not_preferred_for_post_ingress_without_more_context` — The current policy is simple and conservative, but Phase59 shows it terminalizes before candidate generation when a single open ray exists near ingress.
- `allow_single_open_direction_only_at_post_ingress`: `recommended_minimal_phase61_candidate` — Scope the exception to the first post-ingress topology node so ordinary dead-end handling elsewhere remains unchanged.
- `allow_single_open_direction_if_not_backtracking`: `plausible_but_needs_visited_edge_context` — Phase59 accepted direction is not aligned with return-to-entrance, but the current topology has no explicit incoming-edge/backtracking context at first node.
- `require_multi_frame_single_open_direction_then_candidate`: `compatible_with_phase59_guard` — Phase59 already provides consecutive-frame stability; Phase61 could use confirmed single-open evidence as a candidate-formation precondition rather than terminalization.
- `require entrance/ingress context tag before first node`: `recommended_guardrail_for_exception` — A post-ingress tag would distinguish entrance corridor bootstrap from true dead ends at later nodes and prevent a broad strategy change.
- `local_cost_caveat_not_a_strategy_option`: `needs_runtime_validation_before_dispatch_acceptance` — The candidate goal is map-clear, but local cost max radius is high; Phase60 must not convert this into a dispatch behavior change.

## Classification

`SINGLE_OPEN_DIRECTION_MISCLASSIFIED_AS_DEAD_END`

Rationale: static source review shows single-open topology is classified as dead-end and short-circuited before branch candidate creation. Offline Phase59 shows both replays reached consecutive confirmed no-candidate frames from a stable raw single open direction. Because the map-level candidate geometry is clear and the policy lacks post-ingress/incoming-edge context, Phase60 classifies this as a policy-level likely misclassification. The high local-cost sample remains a caveat and must be handled by later runtime evidence, not by changing Phase60 behavior.

## Phase61 recommendation only — no Phase60 behavior change

If Phase61 is approved, implement the smallest post-ingress single-open-direction candidate exception:

1. Apply only after dispatch-entry readiness and only at the first post-ingress topology node.
2. Require Phase59-style multi-frame confirmation of the single open direction before forming a candidate.
3. Require entrance/ingress context tag and reject directions aligned with return/backtracking to the entrance.
4. Preserve all Nav2/MPPI/controller parameters, `clearance_radius_m`, map sufficiency threshold, and branch selection scoring.
5. Keep structured diagnostics for raw/filtered/candidate counts, ingress context, angle-to-return, map/local-cost caveats, and whether the exception was applied.
6. Do not treat dispatch as exit success; use bounded diagnostic runtime only if Phase61 is explicitly started.

## Artifacts

- Analyzer: `tools/analyze_phase60_single_open_direction_dead_end_policy_review.py`
- Focused test: `src/tugbot_maze/test/test_phase60_single_open_direction_dead_end_policy_review.py`
- Analyzer JSON: `log/phase60_single_open_direction_dead_end_policy_review/phase60_single_open_direction_dead_end_policy_review.json`
- Analyzer markdown: `log/phase60_single_open_direction_dead_end_policy_review/phase60_single_open_direction_dead_end_policy_review.md`

## Verification log

Executed verification:

- `python3 -m py_compile tools/analyze_phase60_single_open_direction_dead_end_policy_review.py src/tugbot_maze/test/test_phase60_single_open_direction_dead_end_policy_review.py`: PASS.
- `python3 -m pytest src/tugbot_maze/test/test_phase60_single_open_direction_dead_end_policy_review.py -q`: PASS, `6 passed in 0.00s`.
- `colcon build --packages-select tugbot_maze tugbot_bringup --symlink-install`: PASS, `2 packages finished`.
- `git diff -- src/tugbot_navigation/config | wc -c`: `0`.
- cleanup process check artifact: `cleanup_matches=0`.

Final status: Phase60 complete; stop for human acceptance; do not enter Phase61.
