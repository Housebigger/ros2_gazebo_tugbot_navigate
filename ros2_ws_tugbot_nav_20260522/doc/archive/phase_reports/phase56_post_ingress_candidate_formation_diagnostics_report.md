# Phase56: Post-Ingress Open Direction to Candidate Formation Diagnostics

## Classification

`CANDIDATE_FORMATION_REJECTION_CAUSE_IDENTIFIED`

Interpretation: candidate formation was restored and a bounded first dispatch was observed. This is not an autonomous exploration success claim, not an exit success claim, and not a fallback/terminal acceptance result.

## Scope and guardrails

- Active scaled2x world: `tugbot_maze_world_20260528_clean_scaled2x.sdf`
- Active metadata: `maze_20260528_scaled_instance.yaml`
- Bounded post-ingress candidate-formation diagnostics only
- `max_goals=1`
- `near_exit_fallback_enabled=false`
- `startup_warmup_no_dispatch=false`
- No Nav2/MPPI/controller parameter edits
- No `clearance_radius_m` tuning
- No map sufficiency threshold tuning
- No fallback/terminal acceptance
- No old scaffold world/map
- does not claim autonomous exploration success
- first dispatch is not exit success
- Nav2 config diff: empty

Allowed classifications preserved in analyzer:

- `CANDIDATE_REJECTED_BY_MIN_DISTANCE`
- `CANDIDATE_REJECTED_BY_CLEARANCE`
- `CANDIDATE_REJECTED_AS_DUPLICATE_OR_EXHAUSTED`
- `CANDIDATE_REJECTED_BY_JUNCTION_OR_DEAD_END_POLICY`
- `CANDIDATE_FORMATION_REJECTION_CAUSE_IDENTIFIED`
- `OPEN_DIRECTION_TO_CANDIDATE_INCONCLUSIVE`
- `GUARDRAIL_VIOLATION_STRATEGY_CHANGED`

## Runtime evidence

Artifacts: `/home/hyh/Desktop/agent_playground/playground_hermes/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260522/log/phase56_post_ingress_candidate_formation_diagnostics`

- Ingress action succeeded: `True` / `STATUS_SUCCEEDED` / error_code `0`
- Ingress treated as reached by Phase56 analyzer: `True`
- Post-dispatch final distance to ingress: `0.5467486051112918` m
  - This distance is evidence that the robot moved after ingress; it is not used to invalidate the ingress action after a bounded candidate dispatch.
- Readiness ready: `True`
- Controller active: `True`
- Planner active: `True`
- BT navigator active: `True`
- `/navigate_to_pose` action servers = 1: `True`
- `/goal_pose` subscription count = 1: `True`
- Explorer state samples: `87`
- Goal event samples: `3`
- Dispatch event count: `1`
- Candidate formation restored: `True`
- Complete autonomous success claimed: `False`
- Cleanup empty: `True`

## Open direction to candidate formation diagnostics

- Raw open direction count: `2`
- Filtered open direction count: `2`
- Candidate before filter count: `2`
- Candidate after filter count: `2`
- Candidate branch states: `['untried', 'untried']`
- Accepted angle: `91.97321818028216` deg / `1.6052354809010312` rad
- Accepted vector: `[-0.03443234671943076, 0.9994070309435455]`
- Projection/lookahead: projection `0.49999999999999994` m, lookahead `1.5` m
- Sample endpoint: `[0.8484282798168195, 0.520621429286881]`
- Raw branch goal point: `[0.8484282798168195, 0.520621429286881]`
- Candidate goal point: `[0.5486061660660582, 0.5102917251171272]`
- Chosen candidate goal point: `[0.5486061660660582, 0.5102917251171272]`
- Candidate map cell state: `{'available': True, 'cell': [14, 31], 'clearance_result': 'clear', 'in_bounds': True, 'nearest_obstacle_distance_m': 0.4031128934217776, 'state': 'free', 'value': 0}`
- Candidate local costmap cell state: `{'available': True, 'cell': [23, 39], 'in_bounds': True, 'max_radius_cost': 99, 'state': 'lethal_or_obstacle', 'value': 45}`
- Candidate clearance result: `clear`
- Min travel distance check: `{'min_goal_step_m': 0.45, 'passed': True, 'travel_distance_m': 0.5830951917845053}`
- Too-close check: `{'threshold_m': 0.45, 'too_close': False}`
- Duplicate/exhausted filter: `selectable`
- Junction/dead-end policy filter: `candidate_selected_for_dispatch`
- Branch/candidate rejection reason: `None`

## Topology sampling summary

- Sampled direction count: `4`
- Raw open direction count: `2`
- Filtered open direction count: `2`
- Candidate branch count: `2`
- Reject reason counts: `{'accepted_open_direction': 2, 'clearance_radius_blocked': 2}`

## First dispatch evidence

- Dispatch observed: `True`
- Goal sequence: `1`
- Target: `[0.5486061660660582, 0.5102917251171272]`
- Branch angle: `1.6052354809010312`
- Candidate branch count at dispatch: `2`
- Selected context: `topology_exit_bias_score`
- Dispatch target local cost: `45`
- Dispatch target local cost max radius: `99`
- Path corridor min clearance: `0.4031128934217776`

## First outcome evidence

A bounded outcome event was recorded after the first dispatch. This is post-candidate runtime evidence only; it is not Phase56 exit success/failure acceptance.

- Outcome event: `timeout`
- Result reason: `goal_timeout`
- Effective timeout: `45.0` sec
- Timeout footprint cost max: `99`
- Timeout front wedge clearance: `0.06530358957310028`

## Conclusion

Phase56 did not reproduce the Phase55 `raw_open_direction_count=1` / `last_candidate_count=0` dead-end in this bounded run after rebuilding the Phase56 instrumentation. Instead, the post-ingress topology chain produced candidates:

- open direction evidence advanced from raw/filtered directions to branch candidates;
- candidate before/after filter counts were nonzero;
- a selectable candidate was chosen;
- `/maze/goal_events` recorded one bounded dispatch;
- later timeout evidence belongs to post-dispatch navigation and is outside Phase56 strategy-tuning scope.

Therefore the Phase56 conclusion is `CANDIDATE_FORMATION_REJECTION_CAUSE_IDENTIFIED` with candidate formation restored/first dispatch observed. No Nav2/MPPI/controller, clearance radius, map threshold, fallback, or exploration strategy change was made. Stop here for human acceptance; do not enter Phase57.
