# Phase128 First goal timeout instrumentation design

Status: DESIGN_ONLY

Phase128 is a doc-only/design-only review following Phase127. It defines how a future Phase129 runtime diagnosis may close the Phase127 evidence gaps for the first current-algorithm exploration goal timeout. Phase128 does not implement instrumentation, does not run runtime, does not tune parameters, and does not reinterpret the timeout as success.

## Scope and guardrails

This phase is constrained to design review only:

- doc-only/design-only
- do not launch Gazebo/RViz/Nav2 runtime
- do not send NavigateToPose goal
- do not start maze_explorer
- do not send exploration goal
- do not tune Nav2/MPPI/controller/goal checker/config
- do not change exploration strategy
- do not change branch scoring
- do not change centerline gate
- do not change fallback
- do not change terminal acceptance
- do not claim autonomous exploration success
- do not claim exit success
- Phase129 not entered

Allowed in Phase128:

- write this design proposal;
- write the Phase128 report;
- add focused static tests that assert the design-only contract;
- run static tests and read-only guards.

Forbidden in Phase128:

- any ROS/Gazebo/Nav2 process launch;
- any NavigateToPose action;
- any maze_explorer start;
- any exploration goal dispatch;
- any Nav2, MPPI, controller, goal checker, costmap, inflation, robot radius, clearance, branch scoring, centerline, fallback, terminal acceptance, or exploration-strategy change;
- any repair attempt;
- any claim that the Phase125/Phase127 timeout is autonomous exploration success or exit success.

## Phase127 result preserved

Phase127 classified the existing Phase125 first-goal timeout artifact as:

`FIRST_GOAL_TIMEOUT_LOCAL_COST_BLOCKED`

The classification remains diagnostic. It does not authorize repair, tuning, or exploration-strategy changes.

Preserved Phase127 facts:

- Phase125 ended as a first current-algorithm `goal_kind=explore` timeout, not success.
- exactly one exploration goal was dispatched and accepted.
- no second exploration goal was dispatched.
- terminal reason was `goal_timeout`.
- retained Nav2 feedback showed a `distance_remaining plateau` near timeout.
- retained feedback showed `recoveries=4` near timeout.
- timeout local-cost evidence included `footprint_max=99`.
- timeout local-cost evidence included `front_max=100`.
- timeout local-cost evidence included `path_0_5m_max=73`.
- timeout local-cost evidence included `path_1_0m_max=84`.

Preserved Phase127 evidence gaps:

- local/global costmap windows were unavailable in Phase125 artifact.
- cmd_vel timeline was missing.
- odom velocity timeline was missing.
- robot pose trace was missing.
- goal checker state was missing.
- full planned path snapshots, BT recovery events, controller_server logs, and bt_navigator logs were not retained in a time-aligned artifact.

Interpretation boundary:

- timeout is not success.
- accepted dispatch is not execution success.
- `FIRST_GOAL_TIMEOUT_LOCAL_COST_BLOCKED` is a diagnosis-only diagnostic classification over bounded evidence.
- classification remains diagnostic even if Phase129 reproduces the same timeout with better instrumentation.
- Phase129 must not convert timeout into success.
- Phase129 evidence must preserve frame and timestamp alignment across motion, pose, path, and costmap samples.

## Phase129 instrumentation objective

A future Phase129, if accepted by the human, should repeat the Phase125 bounded first-goal result smoke with instrumentation only. Its purpose should be to collect a time-aligned dispatch-to-timeout artifact that can distinguish controller stall vs local-cost blocked vs tolerance edge vs candidate-risk hypotheses for the first goal.

The Phase129 artifact should answer these questions:

1. Did the controller continue commanding motion, command near-zero motion, or oscillate while distance stayed flat?
2. Did odom show actual robot motion that matched cmd_vel, or did measured velocity stall despite commands?
3. Did the robot approach a near-tolerance band and fail goal checker state because of xy or yaw error?
4. Did local/global costmap windows show a footprint, front wedge, or path corridor cost barrier at the same times as the stall?
5. Did the selected first goal candidate carry risk at dispatch: clearance, target_local_cost_max_radius, path_corridor_min_clearance, or branch geometry near a wall/high-cost band?

## Required timing and alignment contract

Every Phase129 diagnostic stream should be aligned to a common dispatch-to-timeout interval:

- run_id and phase label;
- artifact source paths and hashes when available;
- wall time and ROS time for every sample;
- dispatch time;
- first action feedback time;
- timeout decision time;
- cancel/requested-stop time, if any;
- action result time;
- topic name, frame_id, stamp, sample_age_sec, and in-bounds flags for costmap-derived samples;
- TF freshness and scan freshness for each major sample;
- evidence coverage ratio for each stream over the dispatch-to-timeout interval.

Any missing, stale, contradictory, or non-frame-aligned stream should be represented explicitly as a gap, not silently ignored.

## Motion instrumentation: controller stall vs local-cost blocked

Phase129 should collect a cmd_vel timeline and odom velocity timeline from dispatch through the first goal terminal result.

Required cmd_vel timeline fields:

- sample_time_ros;
- sample_time_wall;
- linear.x;
- angular.z;
- command magnitude;
- near-zero command flag;
- sign-change/oscillation flag;
- sample age and coverage ratio.

Required odom velocity timeline fields:

- sample_time_ros;
- sample_time_wall;
- measured linear velocity;
- measured angular velocity;
- measured speed magnitude;
- near-zero measured velocity flag;
- odom pose if available from the same message;
- covariance or unavailable-covariance marker;
- sample age and coverage ratio.

Derived motion evidence:

- commanded vs measured velocity comparison;
- command/actual mismatch windows;
- repeated oscillation windows;
- distance-progress slope over matching windows;
- recovery-count changes during velocity stalls;
- controller stall vs local-cost blocked evidence matrix.

Classification use:

- Controller stall requires time-aligned velocity evidence, not just a timeout.
- If cmd_vel commands motion but odom does not move, record possible physical/stuck/controller execution stall.
- If cmd_vel collapses near zero while action remains active, record controller command collapse.
- If oscillatory cmd_vel and odom coincide with high costmap windows, preserve local-cost blocked precedence instead of labeling pure controller stall.

## Pose, distance, yaw, and goal checker instrumentation

Phase129 should collect robot pose trace, distance error curve, yaw error curve, and goal checker state for the first goal.

Required robot pose trace fields:

- sample_time_ros;
- sample_time_wall;
- map-frame robot pose x/y/yaw;
- source frame and TF transform age;
- distance-to-goal at each sample;
- yaw error to goal orientation at each sample;
- local slope of distance-to-goal curve;
- near-tolerance band flag;
- pose stability window near timeout.

Required goal checker state fields:

- configured xy tolerance used by the active goal checker, recorded as evidence only;
- configured yaw tolerance used by the active goal checker, recorded as evidence only;
- current distance error;
- current yaw error;
- whether xy tolerance condition is satisfied;
- whether yaw tolerance condition is satisfied;
- whether goal checker reports complete, incomplete, or unavailable;
- controller_server log lines that mention goal checker/tolerance if explicit state cannot be read.

Derived tolerance-edge evidence:

- whether distance converged to a near-tolerance band and plateaued;
- whether yaw error stayed outside tolerance;
- whether robot pose stayed stable near the target while action did not complete;
- whether local cost evidence is too weak to explain noncompletion;
- whether cmd_vel/odom suggest fine adjustment rather than local blockage.

Classification use:

- Tolerance edge requires pose/yaw/goal checker evidence.
- A final distance near tolerance is not sufficient by itself.
- Phase129 must report tolerance edge as diagnostic only; it must not tune goal checker thresholds.

## Costmap, footprint, front wedge, and path corridor instrumentation

Phase129 should collect local costmap windows and global costmap windows around the first-goal execution. The windows should be sampled at dispatch, during progress, before/after recoveries, and near timeout.

Required local costmap windows:

- local costmap frame_id and timestamp;
- map origin, resolution, width, height;
- robot pose projected into local costmap coordinates;
- first goal target projected into local costmap coordinates when in bounds;
- in-bounds flags for robot, target, footprint, front wedge, and path samples;
- local window around robot footprint;
- local window around target when in bounds;
- local front wedge window;
- local planned-path corridor window;
- unknown/lethal/inflated/free cell counts.

Required global costmap windows:

- global costmap frame_id and timestamp;
- map origin, resolution, width, height;
- robot pose projection;
- target pose projection;
- planned path projection;
- target and path corridor cost summaries;
- unknown/lethal/inflated/free cell counts around robot, target, and path corridor.

Required cost snapshots:

- footprint cost snapshot: max, mean, lethal count, inflated count, unknown count;
- front wedge cost snapshot: max, mean, lethal count, inflated count, unknown count;
- path corridor cost snapshot: max cost, mean cost, min clearance estimate, lethal/unknown intersections;
- path_0_5m and path_1_0m cost summaries;
- target_local_cost and target_local_cost_max_radius at dispatch and near timeout where observable.

Derived local-cost blocked evidence:

- high footprint/front/path costs sustained over multiple samples;
- high costs time-aligned with distance-progress plateau;
- high costs time-aligned with recovery events;
- local/global costmap disagreement marked as evidence gap or contradiction;
- path corridor blocked near the robot or target.

Classification use:

- Local-cost blocked requires cost windows and cost snapshots aligned with robot pose trace, planned path, and timeout timing.
- Local-cost blocked should retain precedence over controller stall when high-cost windows plausibly explain cmd_vel oscillation, recovery loops, or command collapse.

## Planned path, BT recovery, and Nav2 logs

Phase129 should preserve planned path, BT recovery events, controller_server logs, and bt_navigator logs for the first goal.

Required planned path fields:

- planner/path source topic or action feedback source;
- path timestamp and frame_id;
- path point count;
- path length;
- first N path points near robot;
- path segment from robot to 0.5 m and 1.0 m ahead;
- whether path intersects local/global high-cost or unknown cells;
- whether path endpoint or local segment deviates from candidate target.

Required BT recovery events:

- recovery start/end time;
- recovery behavior name if available;
- recovery count timeline;
- Nav2 feedback `number_of_recoveries` timeline;
- relationship between recoveries, cmd_vel/odom, and costmap snapshots.

Required log capture:

- controller_server logs from dispatch through timeout;
- bt_navigator logs from dispatch through timeout;
- planner_server logs if path failure or replanning appears;
- behavior_server logs if recovery behaviors are invoked;
- log severity, timestamp, node name, and message;
- extracted progress checker, goal checker, controller failure, recovery loop, and costmap warnings.

Classification use:

- BT recovery loop evidence helps explain timeout symptoms, but it is not a standalone success/failure fix.
- Controller logs can support controller stall or goal tolerance edge only when aligned with motion and pose evidence.
- bt_navigator logs can support recovery-loop diagnosis only when aligned with feedback and recovery event counts.

## First goal candidate risk record

Phase129 should preserve first goal candidate risk as a dispatch-time record and compare it with runtime evidence. This is a diagnosis record only, not permission to change candidate selection.

Required first goal candidate risk fields:

- candidate_family;
- candidate_rank;
- candidate_id or selected_candidate_id if available;
- branch geometry including branch angle, branch endpoint, corridor direction, and junction context;
- clearance at candidate target;
- target_clearance_m;
- target_local_cost;
- target_local_cost_max_radius;
- path_corridor_min_clearance or path_corridor_min_clearance_m;
- dispatch_path_local_cost_max;
- target refinement original/refined/dispatch target comparison;
- selected branch vs lower-ranked branches comparison;
- whether lower-ranked branches had materially safer clearance or lower cost;
- whether the selected target was near wall geometry;
- whether the selected target was near a high-cost band;
- whether branch geometry placed the selected target near a corner, corridor lip, wall, or local-cost ridge.

Derived candidate-risk evidence:

- dispatch-time target risk was already visible before Nav2 execution;
- selected branch was riskier than lower-ranked alternatives;
- target refinement moved the target into higher cost or lower clearance;
- local/global cost windows confirm that the dispatch target lies near wall/high-cost geometry;
- runtime blockage occurred at a location predicted by candidate geometry.

Classification use:

- Candidate-risk diagnosis should be source-risk evidence, not a branch-scoring change.
- Even if candidate risk is found, Phase129 should not change branch scoring, centerline, fallback, target refinement, or terminal acceptance.

## Phase129 allowed scope

If the human accepts Phase128, Phase129 allowed scope should be limited to:

- repeat Phase125 bounded first-goal result smoke;
- instrumentation only;
- `max_goals=1`;
- no second goal;
- no parameter tuning;
- no repair;
- no Nav2/MPPI/controller/goal checker/config changes;
- no exploration strategy changes;
- no branch scoring/centerline/fallback/terminal acceptance changes;
- do not convert timeout into success;
- stop after first goal terminal result;
- produce a bounded artifact for diagnosis;
- preserve first-goal timeout as timeout unless the unmodified run itself produces a different terminal status.

Phase129 should not attempt full autonomous exploration. It should not send an exit goal. It should not send a second exploration goal. It should not use success rate as a repair metric because no repair is authorized.

## Phase129 artifact shape

Recommended artifact top-level fields:

- phase: `Phase129`;
- mode: `first_goal_timeout_instrumented_smoke`;
- source_boundary: `repeat Phase125 bounded first-goal result smoke`;
- max_goals: `1`;
- runtime_guardrails;
- first_goal dispatch record;
- action feedback timeline;
- cmd_vel_timeline;
- odom_velocity_timeline;
- robot_pose_trace;
- distance_error_curve;
- yaw_error_curve;
- goal_checker_state_timeline;
- local_costmap_windows;
- global_costmap_windows;
- footprint_front_path_cost_snapshots;
- planned_path_snapshots;
- bt_recovery_events;
- controller_server_log_excerpt;
- bt_navigator_log_excerpt;
- first_goal_candidate_risk;
- evidence_coverage;
- evidence_gaps;
- diagnostic_classification_candidates;
- no_success_overclaim statement.

The artifact should explicitly mark stream quality:

- present;
- missing;
- stale;
- frame-misaligned;
- out-of-bounds;
- low coverage;
- contradictory.

## Classification contract for the future analyzer

Phase129 instrumentation may feed a later analyzer, but Phase128 does not implement that analyzer.

The future analyzer should retain the Phase126/127 classification set:

- `FIRST_GOAL_TIMEOUT_LOCAL_COST_BLOCKED`
- `FIRST_GOAL_TIMEOUT_CONTROLLER_STALL`
- `FIRST_GOAL_TIMEOUT_GOAL_TOLERANCE_EDGE`
- `FIRST_GOAL_TIMEOUT_CANDIDATE_TOO_RISKY`
- `INSUFFICIENT_TIMEOUT_EVIDENCE`

Precedence remains conservative:

1. Missing timestamps/frames/coverage produce insufficient evidence for the affected class.
2. Strong local-cost windows aligned with footprint/front/path cost snapshots and distance plateau support local-cost blocked.
3. Controller stall requires cmd_vel timeline, odom velocity timeline, and distance-progress evidence that are not better explained by high local cost.
4. Tolerance edge requires robot pose trace, distance error curve, yaw error curve, and goal checker state.
5. Candidate-risk requires dispatch-time clearance/cost/branch geometry evidence and comparison with lower-ranked branches.
6. Contradictory or incomplete evidence remains diagnostic and should not be converted into a repair conclusion.

## Stop condition

Phase128 stops after this proposal, Phase128 report, focused static tests, process guard, Nav2 config diff guard, and pycache cleanup guard. Phase129 not entered.
