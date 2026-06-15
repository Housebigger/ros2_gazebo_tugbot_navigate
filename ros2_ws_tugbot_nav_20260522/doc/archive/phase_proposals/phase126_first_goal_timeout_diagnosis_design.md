# Phase126 First exploration goal timeout diagnosis design

Status: DESIGN_ONLY

Phase126 is doc-only/design-only. It reviews the Phase125 first exploration goal timeout and defines how a later diagnosis phase should separate root-cause classes without implementing code, rerunning runtime, tuning parameters, or converting a timeout into success.

## Scope

This phase is constrained to design review only:

- doc-only/design-only
- do not launch Gazebo/RViz/Nav2 runtime
- do not send NavigateToPose goal
- do not start maze_explorer
- do not tune Nav2/MPPI/controller/config
- do not change exploration strategy
- do not change branch scoring
- do not change centerline gate
- do not change fallback
- do not change terminal acceptance
- do not claim autonomous exploration success
- do not claim exit success
- Phase127 not entered

No implementation is authorized in Phase126. No replay is authorized in Phase126. No parameters, Nav2 config, MPPI settings, controller settings, goal checker settings, costmap settings, robot radius, inflation, clearance, branch selection, centerline, fallback, or terminal acceptance behavior may be changed.

## Phase125 facts preserved as evidence, not success

Phase125 ended as `FIRST_EXPLORATION_GOAL_RESULT_TIMEOUT_DIAGNOSTIC_FAIL`.

Known facts:

- Phase120 ingress succeeded.
- Phase122 handoff ready.
- maze_explorer max_goals=1.
- exactly one current-algorithm goal_kind=explore was dispatched.
- dispatch accepted.
- no second exploration goal was dispatched.
- terminal reason was `goal_timeout`.
- robot_pose_at_result x=2.4438,y=1.0153.
- distance_to_first_goal≈0.356m.
- timeout stderr included `phase23 timeout local cost seq=1 footprint_max=99 front_max=100 path_0_5m_max=73 path_1_0m_max=84`.
- retained Nav2 feedback showed recoveries≈4 near timeout.

Interpretation boundary:

- timeout is not success.
- accepted dispatch is not execution success.
- first-goal timeout is not autonomous exploration success.
- first-goal timeout is not exit success.
- Phase126 must design evidence gathering/classification only; it must not prescribe a fix.

## Root-cause taxonomy

Phase126 defines five mutually auditable timeout outcomes. Each outcome must be evidence-gated and may only be assigned when its required evidence is present. Otherwise the result remains `INSUFFICIENT_TIMEOUT_EVIDENCE`.

### 1. FIRST_GOAL_TIMEOUT_LOCAL_COST_BLOCKED

Use when the primary evidence indicates the first goal or local path corridor was blocked by local cost or footprint/front-wedge cost, not merely by controller behavior.

Signals:

- local cost high at the target or near the robot footprint;
- footprint/front/path cost sequence shows sustained high-cost or lethal cells;
- path locally blocked near the final approach;
- local costmap windows around robot, target, front wedge, and path show high-cost bands;
- global costmap windows do not contradict the local blockage;
- planned path intersects high-cost or unknown local cells near the final approach;
- robot cannot reduce distance-to-goal despite costmap blockage evidence.

Phase125 hints that motivate this class but do not yet prove it alone:

- `footprint_max=99`
- `front_max=100`
- `path_0_5m_max=73`
- `path_1_0m_max=84`
- distance remaining near timeout stayed nonzero.

### 2. FIRST_GOAL_TIMEOUT_CONTROLLER_STALL

Use when motion/control evidence dominates over geometry/cost evidence.

Signals:

- controller oscillation/stall is visible in cmd_vel and odom velocity;
- cmd_vel alternates signs or repeatedly commands small angular/linear corrections without distance progress;
- odom velocity shows little real motion despite nonzero commands, or commands collapse to near-zero while action stays active;
- distance-to-goal curve plateaus before tolerance is reached;
- recovery count rises or BT recovery loop repeats while local costs do not clearly explain the stall;
- controller_server logs indicate progress checker, oscillation, or controller failure patterns.

Required distinction:

- A controller stall classification needs velocity-stall evidence, not just a timeout.
- If high local cost and velocity stall are both present, classification precedence must check whether local-cost blockage directly explains the stall before labeling controller stall.

### 3. FIRST_GOAL_TIMEOUT_GOAL_TOLERANCE_EDGE

Use when the robot reaches a pose near the first goal but remains outside the effective goal checker tolerance, or orientation/yaw tolerance prevents completion.

Signals:

- distance-to-goal curve converges near the configured tolerance boundary and remains there;
- robot pose trace shows stable position near the target;
- yaw/orientation error remains outside tolerance or oscillates at the edge;
- goal checker state or controller_server logs show tolerance/progress issues;
- local costs are not severe enough to classify as blocked;
- cmd_vel/odom velocity suggests the robot is making fine adjustments rather than being locally blocked.

Phase125 hint:

- distance_to_first_goal≈0.356m at timeout is close enough to require goal checker tolerance analysis, but Phase126 cannot assume tolerance edge without goal checker state and pose/yaw trace.

### 4. FIRST_GOAL_TIMEOUT_CANDIDATE_TOO_RISKY

Use when target selection made the goal inherently risky even if execution evidence later manifests as cost or controller trouble.

Signals:

- candidate selection near wall is supported by candidate pose clearance and cost samples;
- goal too close to obstacle is visible in target map/local-cost windows;
- candidate pose clearance is marginal relative to footprint/inflation envelope;
- target refinement landed in high-cost area;
- target refinement landed in high-cost area or increased cost risk versus original target;
- candidate_family=junction and candidate_rank=1 selection outcompeted lower-ranked branches with safer clearance/cost margins;
- junction branch geometry places the selected branch target near a wall, corner, or blocked corridor lip;
- selected branch vs lower-ranked branches comparison shows better alternatives had materially lower local-cost/path-risk evidence.

Required distinction:

- This class diagnoses candidate-risk/source-risk, not Nav2 tuning.
- It does not authorize changing branch scoring, target refinement, centerline gate, fallback, terminal acceptance, or exploration order in Phase126.

### 5. INSUFFICIENT_TIMEOUT_EVIDENCE

Use when the available data cannot distinguish local cost, controller stall, tolerance edge, or candidate risk.

Examples:

- no cmd_vel timeline;
- no odom velocity timeline;
- no robot pose trace;
- missing local costmap windows or global costmap windows;
- missing planned path;
- no footprint/front/path cost sequence over time;
- missing controller_server logs or bt_navigator logs;
- no goal checker state;
- timestamps cannot align Nav2 feedback timeline with costmap/odom/TF samples.

## Required evidence contract for a later diagnosis phase

A later diagnosis-only phase must define or collect a read-only artifact with these fields. Phase126 does not implement this artifact; it only defines the contract.

### Timing and synchronization

- artifact source paths and SHA/file timestamps;
- monotonic wall-time and ROS-time alignment;
- dispatch time, feedback times, timeout time, and cancel-result time;
- tf freshness and scan freshness for each major sample;
- local/global costmap timestamps and frame IDs;
- evidence coverage ratios for the dispatch-to-timeout interval.

### Nav2 execution evidence

- Nav2 feedback timeline, including distance remaining, navigation time, estimated time remaining, and recovery count;
- action status timeline and terminal status;
- recovery count timeline and recovery behavior names if available;
- controller_server logs around dispatch through timeout;
- bt_navigator logs around dispatch through timeout;
- progress checker and goal checker state if available;
- result_status_label and abort_text.

### Motion evidence

- cmd_vel timeline with linear/angular commands;
- odom velocity timeline with measured linear/angular velocity;
- robot pose trace from dispatch to timeout;
- distance-to-goal curve over time;
- yaw/orientation error curve over time;
- velocity-stall evidence derived from cmd_vel vs odom velocity vs distance-to-goal progress.

### Cost/path evidence

- local costmap windows around robot pose, target pose, footprint, front wedge, and path samples;
- global costmap windows around robot pose, target pose, and planned path;
- footprint/front/path cost sequence from dispatch through timeout;
- footprint cost max/mean over time;
- front wedge cost max/mean over time;
- path cost at 0.5m and 1.0m ahead over time;
- planned path from planner, including path frame, length, and point samples;
- local path clipping/projection in the robot frame;
- map occupancy samples around the target;
- costmap unknown/lethal/inflated/free cell classification.

### Target selection diagnostics

A later diagnosis must preserve the target-selection context from Phase125 goal_events and compare it against execution evidence.

Required fields:

- first_goal.candidate;
- candidate pose clearance;
- frontier/topology evidence;
- junction branch geometry;
- candidate_family=junction;
- candidate_rank=1;
- selected_candidate_id/candidate_id;
- branch_angle;
- score_components;
- target_local_cost;
- target_local_cost_max_radius;
- path_corridor_min_clearance_m;
- chosen_branch_rank;
- candidate_branch_count;
- selected branch vs lower-ranked branches comparison;
- target refinement original/refined/dispatch target comparison;
- whether target refinement landed in high-cost area;
- whether selected target was near wall or near a high-cost band.

## Evidence thresholds and classification precedence

The later analyzer should use explicit evidence thresholds rather than narrative judgment. Exact numeric thresholds may be tuned only as analyzer thresholds over evidence, not as Nav2 runtime parameters.

Recommended evidence thresholds for classification design:

1. `FIRST_GOAL_TIMEOUT_LOCAL_COST_BLOCKED`
   - high local cost exists in footprint/front/path cost sequence near timeout;
   - cost remains high over multiple samples, not a single stale sample;
   - distance-to-goal curve stalls while cost evidence is high;
   - local/global costmap windows align in frame/time with robot pose trace.
2. `FIRST_GOAL_TIMEOUT_CONTROLLER_STALL`
   - cmd_vel/odom velocity show oscillation, stall, or command/actual mismatch;
   - distance-to-goal curve stalls without sufficient local-cost blocked evidence;
   - recovery count or BT recovery loop evidence supports control recovery behavior.
3. `FIRST_GOAL_TIMEOUT_GOAL_TOLERANCE_EDGE`
   - robot pose trace reaches near-target band;
   - distance-to-goal curve remains close to tolerance boundary;
   - goal checker state or yaw/pose tolerance evidence explains failure to complete;
   - local cost and controller stall evidence are not dominant.
4. `FIRST_GOAL_TIMEOUT_CANDIDATE_TOO_RISKY`
   - selected candidate has marginal candidate pose clearance, high target_local_cost, high target_local_cost_max_radius, or poor path_corridor_min_clearance_m;
   - selected branch vs lower-ranked branches comparison shows the selected candidate was riskier than alternatives;
   - target refinement landed in high-cost area or near-wall geometry.
5. `INSUFFICIENT_TIMEOUT_EVIDENCE`
   - any required signal is missing, stale, not frame-aligned, or contradictory enough to prevent a responsible root-cause classification.

classification precedence:

1. If required timestamps/frames are missing, return `INSUFFICIENT_TIMEOUT_EVIDENCE`.
2. If local-cost blocked evidence is strong and time-aligned, prefer `FIRST_GOAL_TIMEOUT_LOCAL_COST_BLOCKED` over controller stall because local cost can induce controller recovery/stall.
3. If local-cost blocked evidence is weak but cmd_vel/odom velocity shows oscillation/stall with recovery loops, use `FIRST_GOAL_TIMEOUT_CONTROLLER_STALL`.
4. If robot pose is near target and tolerance/yaw evidence explains noncompletion, use `FIRST_GOAL_TIMEOUT_GOAL_TOLERANCE_EDGE`.
5. If execution evidence is secondary but target-selection evidence shows near-wall/high-cost selection risk, use `FIRST_GOAL_TIMEOUT_CANDIDATE_TOO_RISKY`.
6. If none is evidence-complete, use `INSUFFICIENT_TIMEOUT_EVIDENCE`.

The analyzer must explicitly state which evidence thresholds passed/failed for each classification. It must do not convert timeout/failure into success.

## Phase127 allowed scope

Phase127 allowed scope, if approved by the human, is diagnosis-only replay/analyzer.

Allowed in Phase127:

- diagnosis-only replay/analyzer design or implementation;
- at most reuse Phase125 artifacts;
- offline analyzer over Phase125 artifact, logs, goal_events, feedback, and existing cost samples;
- optional read-only extraction from existing logs/artifacts;
- no parameter tuning;
- no runtime exploration rerun;
- no second exploration goal;
- no MPPI/controller/goal checker change;
- no Nav2 config change;
- no algorithm change;
- stop before repair.

Forbidden in Phase127 unless a later explicit phase authorizes it:

- changing Nav2/MPPI/controller/goal checker parameters;
- changing costmap inflation, robot radius, clearance radius, or map thresholds;
- changing exploration strategy, branch scoring, centerline, fallback, terminal acceptance, or target refinement behavior;
- using timeout reduction or success claim as proof of repair;
- rerunning full exploration or sending new manual/fallback/terminal/exit goals.

## Deliverable shape for a future analyzer

A later analyzer should output:

- top-level classification from the Phase126 classification set;
- per-class evidence matrix with present/missing/stale/contradictory fields;
- dispatch-to-timeout timeline;
- distance-to-goal curve summary;
- cmd_vel vs odom velocity summary;
- recovery count and BT recovery loop summary;
- local/global costmap window summary;
- footprint/front/path cost sequence summary;
- planned path summary;
- target selection diagnostics summary;
- explicit no-success-overclaim section.

The future analyzer must treat incomplete evidence conservatively. It should classify as `INSUFFICIENT_TIMEOUT_EVIDENCE` rather than guess.

## Stop condition

Phase126 stops after proposal, report, focused static tests, no-runtime process guard, and Nav2 config diff guard. Phase127 not entered.
