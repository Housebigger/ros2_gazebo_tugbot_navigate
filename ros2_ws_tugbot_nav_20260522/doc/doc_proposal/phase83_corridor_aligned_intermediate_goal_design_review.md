# Phase83: Corridor-aligned intermediate goal design review

Status: `DESIGN_REVIEW_ONLY_NOT_RUNTIME_ENABLED`

Phase83 is design review only. No runtime integration is added in Phase83. Phase84 not entered.

## Purpose

Phase80/81/82 show that Goal2 should not be repaired by making an unsafe or poorly aligned target easier to accept. The safer design direction is to make the exploration layer produce a Nav2-executable intermediate goal:

`candidate point -> corridor centerline projection -> corridor heading -> forward executability check -> Nav2 goal`

This proposal reviews corridor-aligned intermediate goal refinement as a future implementation concept. It does not modify `maze_explorer.py`, branch scoring, existing centerline gates, directional readiness, fallback, terminal acceptance, Nav2 parameters, MPPI, controller parameters, inflation, robot radius, `clearance_radius_m`, or map thresholds.

## Evidence basis

Required upstream context:

- Phase80: `NEAR_GOAL_LATERAL_RESIDUAL_WITH_FORWARD_OPEN_CORRIDOR` with the target residual almost entirely lateral at Goal2 timeout.
- Phase81: `FORWARD_OPEN_CORRIDOR_BLOCKED`; raw scan showed physical forward clearance, but `/local_costmap/costmap` showed a high/lethal robot-forward execution corridor.
- Phase82: `FOOTPRINT_OR_WEDGE_PROJECTION_SUSPECTED` primary, with `INFLATION_SPILLOVER_SUSPECTED` as a secondary supported signal.

Design implication:

- Do not lower safety boundaries to accommodate a poor target.
- Do not treat timeout as success.
- Generate a target pose that is better aligned with corridor geometry and Nav2 execution: centered enough for the footprint/wedge envelope, oriented along the corridor, and forward-executable before dispatch.

## Core design: corridor-aligned intermediate goal refinement

The future helper should be scoped to the already selected branch/candidate. It must not select a different branch and must not modify branch scoring.

### 1. Input candidate

Input is the target candidate already selected by the existing exploration logic:

- `dispatch_pose`: robot pose at dispatch decision time.
- `candidate_target_xy`: the current target produced by the existing candidate formation path.
- `branch_direction`: the already-selected open/corridor direction.
- `map` / `local_costmap` / `scan` / `TF` snapshots if available.
- existing safety thresholds only; no new tuning of inflation, robot radius, clearance radius, map threshold, MPPI, or controller parameters.

This phase does not authorize changing where candidate branches come from.

### 2. same-corridor evidence gate

Before any refinement, the candidate must have same-corridor evidence:

- candidate lies in the same traversable corridor as `dispatch_pose`.
- line of sight from dispatch pose toward candidate is known/free or conservative-pass by existing semantics.
- projected candidate does not cross an occupied wall cell.
- corridor axis agrees with the selected branch direction within a bounded angular tolerance inherited from existing topology logic.

If same-corridor evidence is missing, output `insufficient evidence -> no refinement` and keep the original target unchanged.

### 3. two-side-wall evidence gate

The design needs two-side-wall evidence before using a centerline:

- sample lateral rays left and right of the corridor axis.
- require both side walls, cost boundaries, or stable high-cost wall bands to be visible in the same local frame.
- record left/right distance and hit flags separately.
- do not silently treat a missing side-wall hit as safe or centered.

If two-side-wall evidence is missing, keep the original target unchanged. This prevents inventing a centerline in open or ambiguous geometry.

### 4. centerline projection

`centerline projection` means projecting the already-selected `candidate_target_xy` onto the corridor centerline segment estimated from the same-corridor and two-side-wall evidence.

The projected point should:

- preserve forward progress along the selected branch direction.
- reduce lateral offset from the corridor centerline.
- remain in known/free map cells when map evidence is available.
- remain inside the same corridor segment.
- stay within a small bounded neighborhood of the original candidate so this is refinement, not route replanning.

`corridor centerline projection` must be conservative:

- if projection makes occupancy/local-cost/front-wedge/footprint evidence worse, do not apply it.
- if projection would move behind the dispatch pose or create sideways-only near-goal motion, do not apply it.
- if projection evidence is ambiguous, keep the original target unchanged.

### 5. corridor heading

The refined target is a pose, not just an XY point.

`corridor heading` is the yaw angle aligned with the corridor forward direction, not the vector from terminal pose to a lateral residual point.

Contract:

- goal orientation follows corridor heading.
- heading direction follows the selected branch/open corridor direction.
- if the corridor axis has two possible directions, choose the one with positive forward progress from dispatch pose.
- if heading evidence is insufficient, do not refine.

This directly targets `near-goal lateral residual handling`: a goal pose on the centerline with yaw along the corridor should avoid lateral near-goal residual by making the final Nav2 pose match forward corridor motion rather than requiring a sideways correction at the end.

### 6. forward executability check

Before producing the Nav2 goal, the future helper must run a `forward executability check` in the dispatch frame or current local-cost frame.

Minimum checks:

- projected target cell is known/free or passes the existing conservative map policy.
- short forward corridor from dispatch pose to projected target has no lethal local-cost cells in the footprint envelope when local costmap evidence exists.
- footprint projection at target pose does not overlap lethal cost.
- front wedge aligned with corridor heading does not enter high/lethal cost beyond accepted safety floors.
- raw scan does not contradict required forward clearance when available.
- TF/costmap/scan timestamps are fresh enough by existing diagnostics policy.

If local-cost/scan/TF evidence is missing, the future implementation should keep the original target unchanged and record why. Missing evidence must not be fabricated.

### 7. Nav2 goal output

Only after all gates pass, the future helper may produce:

- `original_target`: unchanged input candidate.
- `refined_target`: centerline-projected XY.
- `refined_yaw`: corridor heading.
- `nav2_goal_pose`: refined XY plus corridor heading.
- `refinement_applied=true` with full diagnostics.

If any gate fails:

- output `refinement_applied=false`.
- dispatch the original target unchanged.
- record a reason such as `missing_same_corridor_evidence`, `missing_two_side_wall_evidence`, `centerline_projection_not_safer`, `forward_executability_blocked`, or `insufficient evidence -> no refinement`.

## Near-goal lateral residual handling

The Phase80 failure mode was not a request to accept a timed-out lateral residual. The design response is to avoid creating lateral residual in the first place:

1. keep target XY on or close to the corridor centerline.
2. set target yaw to corridor heading.
3. ensure the final approach is forward-executable.
4. reject goals that are centered in XY but force the controller to finish with a sideways correction.

This is `near-goal lateral residual handling`, not fallback/terminal acceptance.

## Diagnostics contract for a future implementation phase

If a later phase implements this design, every dispatch event should record:

- `corridor_aligned_intermediate_goal_refinement`.
- `original_target`.
- `centerline_projected_target`.
- `corridor_heading_yaw`.
- `nav2_goal_pose`.
- `refinement_applied`.
- `refinement_reject_reason`.
- `same_corridor_evidence`.
- `two_side_wall_evidence`.
- `centerline_projection_metrics`.
- `forward_executability_check`.
- `near_goal_lateral_residual_handling`.
- `branch_scoring_changed=false`.
- `fallback_terminal_acceptance_used=false`.

## Acceptance criteria before any future implementation

A future implementation phase must satisfy all of these before runtime use:

1. Static tests for projection and yaw semantics.
2. Synthetic tests for missing evidence and blocked forward executability.
3. No branch-scoring changes.
4. No Nav2 / MPPI / controller / inflation / robot-radius / clearance-radius / map-threshold tuning.
5. Bounded runtime validation only after design is explicitly accepted.
6. Dispatch diagnostics prove whether refinement applied or did not apply.
7. Timeout remains timeout unless the robot reaches the configured exit; do not treat timeout as success.

## Focused test cases for a future implementation phase

Minimum future tests:

- candidate already on centerline -> no lateral shift.
- off-center candidate -> projected to centerline.
- corridor heading yaw is used as Nav2 goal orientation.
- blocked forward executability -> keep original target unchanged.
- missing corridor evidence -> keep original target unchanged.
- missing two-side-wall evidence -> keep original target unchanged.
- projected target behind dispatch pose -> keep original target unchanged.
- centerline projection improves lateral residual but worsens lethal footprint cost -> keep original target unchanged.
- same branch selected before/after refinement; `branch_scoring_changed=false`.
- fallback/terminal acceptance remains unused.

## Guardrails

- Do not tune inflation.
- Do not tune robot_radius.
- Do not tune clearance_radius_m.
- Do not tune MPPI.
- Do not tune controller.
- Do not change maze_explorer runtime strategy.
- Do not change branch scoring.
- Do not change centerline gate.
- Do not change directional readiness.
- Do not use fallback/terminal acceptance.
- Do not treat timeout as success.
- No autonomous exploration success claimed.
- No exit success claimed.

## Phase83 decision

`DESIGN_REVIEW_COMPLETE_PENDING_EXPLICIT_IMPLEMENTATION_PHASE`

Phase83 recommends corridor-aligned intermediate goal refinement as the right design direction to review next: produce better Nav2 goals rather than weakening safety boundaries or accepting timed-out goals. It remains design-only until a later explicit implementation phase.

Phase84 not entered.
