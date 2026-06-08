# Phase61 Post-Ingress Single-Open-Direction Candidate Exception Report

run_id: `phase61_post_ingress_single_open_candidate_exception`
artifact_dir: `log/phase61_post_ingress_single_open_candidate_exception/`
classification: `SINGLE_OPEN_EXCEPTION_APPLIED_AND_DISPATCHED`

## Preserved conclusions

- Phase60 conclusion preserved: `SINGLE_OPEN_DIRECTION_MISCLASSIFIED_AS_DEAD_END`.
- Phase58/59 baseline preserved: `CANDIDATE_FORMATION_UNSTABLE_DEAD_END_POLICY_SENSITIVE`.
- Phase59 runtime classification preserved: `TOPOLOGY_CONSISTENCY_CONFIRMED_NO_CANDIDATE`.
- Phase57 timeout conclusion preserved: `TIMEOUT_INCONCLUSIVE_DATA_GAP`.

## Scope implemented

Phase61 adds a narrow post-ingress first-topology-node exception: after the Phase59 topology consistency guard confirms stable single-open/no-candidate frames, `maze_explorer` may form exactly one candidate from the single open direction before dead-end terminalization.

This does not change ordinary branch selection scoring. The generated exception candidate is inserted as an ordinary `BranchOption`, then the existing topology/branch selection path dispatches it.

## Runtime evidence

- bounded replay count: 1
- ingress goal success: True
- post-ingress context active: True
- first post-ingress topology node: True
- multi-frame single-open confirmed: True
- single-open exception eligible: True
- single-open exception applied: True
- exception reason: `['post_ingress_single_open_exception_applied']`
- dispatch observed: True
- first dispatch observed, not exit success: True
- complete autonomous success claimed: False

First dispatch subset:

- goal_sequence: `1`
- goal_kind: `explore`
- target: `[0.5743848734507782, 0.5120109438476859]`
- local_topology: `dead_end`
- raw_open_direction_count: `1`
- topology_consistency_frame_count: `2`
- dead_end_policy_state: `confirmed`
- single_open_exception_reason: `post_ingress_single_open_exception_applied`

Candidate evidence:

- angle_to_return_entrance_deg: `[89.68682244039272]`
- candidate map cell state: `[{'available': True, 'cell': [15, 31], 'clearance_result': 'clear', 'in_bounds': True, 'nearest_obstacle_distance_m': 0.4000000059604645, 'state': 'free', 'value': 0}]`
- candidate local costmap caveat: `[{'available': True, 'cell': [24, 39], 'in_bounds': True, 'max_radius_cost': 99, 'state': 'lethal_or_obstacle', 'value': 49}]`

Interpretation: Phase61 observed that the consistency guard allowed candidate recovery / first dispatch via the post-ingress single-open exception. This is only first dispatch evidence. It is not autonomous exploration success and not exit success.

## Guardrails

- bounded runtime only, `max_goals=1`.
- active scaled2x world/map only; no old scaffold world/map.
- no Nav2/MPPI/controller parameter edits.
- no `clearance_radius_m` tuning.
- no map sufficiency threshold tuning.
- no ordinary branch selection scoring change.
- no fallback/terminal acceptance.
- does not claim autonomous exploration success / 不声明 autonomous exploration success.
- if first dispatch appears, first dispatch is not exit success / first dispatch 不是出口成功.

## Verification summary

- analyzer output: `log/phase61_post_ingress_single_open_candidate_exception/phase61_post_ingress_single_open_candidate_exception.json`
- nav2_config_diff_empty: True
- cleanup_empty after replay: True

## Expected classifications

- `SINGLE_OPEN_EXCEPTION_IMPLEMENTED_STATIC_ONLY`
- `SINGLE_OPEN_EXCEPTION_APPLIED_AND_DISPATCHED`
- `SINGLE_OPEN_EXCEPTION_NOT_ELIGIBLE_CONFIRMED_DEAD_END`
- `SINGLE_OPEN_EXCEPTION_INCONCLUSIVE_RUNTIME_VARIANCE`
- `GUARDRAIL_VIOLATION_STRATEGY_CHANGED`

## Stop condition

Phase61 stops here for manual acceptance. 不进入 Phase62.
