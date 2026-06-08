# Phase126 First exploration goal timeout diagnosis design report

Status: PHASE126_FIRST_GOAL_TIMEOUT_DIAGNOSIS_DESIGN_COMPLETE_STOP_BEFORE_PHASE127

## Summary

Phase126 completed a doc-only/design-only review for the Phase125 first exploration goal timeout. The design defines how a future diagnosis-only phase should distinguish local-cost blockage, controller stall, goal-tolerance edge, and candidate-risk causes without changing runtime behavior or claiming success.

Primary proposal:

- `doc/doc_proposal/phase126_first_goal_timeout_diagnosis_design.md`

Focused static tests:

- `src/tugbot_maze/test/test_phase126_first_goal_timeout_diagnosis_design.py`

## Phase125 evidence carried into the design

The design preserves these Phase125 facts as diagnostic inputs, not as success:

- `FIRST_EXPLORATION_GOAL_RESULT_TIMEOUT_DIAGNOSTIC_FAIL`
- Phase120 ingress succeeded.
- Phase122 handoff ready.
- `maze_explorer max_goals=1`.
- exactly one current-algorithm `goal_kind=explore` was dispatched and accepted.
- terminal reason was `goal_timeout`.
- `robot_pose_at_result x=2.4438,y=1.0153`.
- `distance_to_first_goal≈0.356m`.
- timeout local-cost line recorded `footprint_max=99 front_max=100 path_0_5m_max=73 path_1_0m_max=84`.
- retained feedback indicated `recoveries≈4`.

The report does not treat timeout as success.

## Root-cause classification design

The proposal defines these future diagnosis classifications:

- `FIRST_GOAL_TIMEOUT_LOCAL_COST_BLOCKED`
- `FIRST_GOAL_TIMEOUT_CONTROLLER_STALL`
- `FIRST_GOAL_TIMEOUT_GOAL_TOLERANCE_EDGE`
- `FIRST_GOAL_TIMEOUT_CANDIDATE_TOO_RISKY`
- `INSUFFICIENT_TIMEOUT_EVIDENCE`

The proposal requires evidence thresholds and classification precedence. Incomplete, stale, contradictory, or non-frame-aligned evidence must classify conservatively as `INSUFFICIENT_TIMEOUT_EVIDENCE`.

## Required evidence contract

The design requires a future analyzer/replay to reason over:

- Nav2 feedback timeline;
- cmd_vel;
- odom velocity;
- robot pose trace;
- distance-to-goal curve;
- local costmap windows;
- global costmap windows;
- footprint/front/path cost sequence;
- planned path;
- recovery count;
- controller_server logs;
- bt_navigator logs;
- goal checker state;
- tf freshness;
- scan freshness;
- target-selection evidence.

Target-selection diagnostics include:

- candidate pose clearance;
- frontier/topology evidence;
- junction branch geometry;
- `candidate_family=junction`;
- `candidate_rank=1`;
- `target_local_cost`;
- `target_local_cost_max_radius`;
- `path_corridor_min_clearance_m`;
- `branch_angle`;
- `score_components`;
- target refinement original/refined/dispatch comparison;
- selected branch vs lower-ranked branches comparison.

## Phase127 boundary

The proposal limits Phase127 allowed scope to diagnosis-only replay/analyzer work:

- at most reuse Phase125 artifacts;
- no parameter tuning;
- no runtime exploration rerun;
- no second exploration goal;
- no MPPI/controller/goal checker change;
- no Nav2 config change;
- no algorithm change;
- stop before repair.

## Guardrails

No implementation was performed.
No Gazebo/RViz/Nav2 runtime was launched.
No NavigateToPose goal was sent.
No maze_explorer was started.
No Nav2/MPPI/controller/config tuning was performed.
No exploration strategy was changed.
No branch scoring, centerline gate, fallback, or terminal acceptance logic was changed.
No autonomous exploration success or exit success is claimed.
Phase127 not entered.

## Verification

Verification performed:

- focused static tests for Phase126 proposal/report contract;
- no-runtime process guard;
- Nav2 config diff guard;
- pycache cleanup guard.

Final command bundle included Phase121/122/123/124/125/126 static/focused tests and passed.

## Stop condition

Phase126 stops here. Phase127 not entered. Wait for human acceptance before any Phase127 work.
