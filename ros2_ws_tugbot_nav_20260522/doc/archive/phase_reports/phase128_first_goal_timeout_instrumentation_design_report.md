# Phase128 First goal timeout instrumentation design report

Status: PHASE128_FIRST_GOAL_TIMEOUT_INSTRUMENTATION_DESIGN_COMPLETE_STOP_BEFORE_PHASE129

## Summary

Phase128 completed a doc-only/design-only review for closing the Phase127 first-goal timeout evidence gaps in a future Phase129. The design defines what Phase129 should instrument if the human later authorizes a bounded first-goal runtime diagnosis. No implementation was performed and no runtime was launched.

Primary proposal:

- `doc/doc_proposal/phase128_first_goal_timeout_instrumentation_design.md`

Focused static tests:

- `src/tugbot_maze/test/test_phase128_first_goal_timeout_instrumentation_design.py`

## Phase127 classification preserved

Phase127 remains classified as:

`FIRST_GOAL_TIMEOUT_LOCAL_COST_BLOCKED`

The classification remains diagnostic only. Phase128 does not authorize repair, tuning, or strategy changes.

Preserved Phase127 facts:

- retained feedback showed a `distance_remaining plateau`;
- retained feedback showed `recoveries=4`;
- timeout local-cost line included `footprint_max=99`;
- timeout local-cost line included `front_max=100`;
- timeout local-cost line included `path_0_5m_max=73`;
- timeout local-cost line included `path_1_0m_max=84`.

Preserved Phase127 evidence gaps:

- local/global costmap windows;
- cmd_vel timeline;
- odom velocity timeline;
- robot pose trace;
- goal checker state;
- planned path snapshots;
- BT recovery events;
- controller_server logs;
- bt_navigator logs.

The timeout is not success, accepted dispatch is not execution success, and no autonomous exploration success or exit success is claimed.

## Instrumentation design reviewed

The Phase128 proposal defines future Phase129 instrumentation requirements for:

- cmd_vel timeline;
- odom velocity timeline;
- commanded vs measured velocity comparison;
- robot pose trace;
- distance error curve;
- yaw error curve;
- goal checker state;
- local costmap windows;
- global costmap windows;
- footprint cost snapshot;
- front wedge cost snapshot;
- path corridor cost snapshot;
- planned path;
- BT recovery events;
- controller_server logs;
- bt_navigator logs;
- first goal candidate risk.

## Diagnostic distinctions preserved

The design keeps the Phase126/127 diagnostic categories evidence-gated:

- `FIRST_GOAL_TIMEOUT_LOCAL_COST_BLOCKED` requires time-aligned costmap windows and footprint/front/path corridor cost snapshots that explain the distance plateau.
- `FIRST_GOAL_TIMEOUT_CONTROLLER_STALL` requires cmd_vel timeline, odom velocity timeline, and distance-progress evidence not better explained by local-cost blockage.
- `FIRST_GOAL_TIMEOUT_GOAL_TOLERANCE_EDGE` requires robot pose trace, distance error curve, yaw error curve, and goal checker state.
- `FIRST_GOAL_TIMEOUT_CANDIDATE_TOO_RISKY` requires dispatch-time first goal candidate risk evidence including clearance, target_local_cost_max_radius, path_corridor_min_clearance, branch geometry, and selected branch vs lower-ranked branches comparison.
- `INSUFFICIENT_TIMEOUT_EVIDENCE` remains the conservative result when required evidence is missing, stale, frame-misaligned, out-of-bounds, low-coverage, or contradictory.

## Phase129 boundary defined

If later accepted, Phase129 should be limited to:

- repeat Phase125 bounded first-goal result smoke;
- instrumentation only;
- `max_goals=1`;
- no second goal;
- no parameter tuning;
- no repair;
- do not convert timeout into success;
- stop after first goal terminal result;
- produce a bounded artifact.

Phase129 should not perform full autonomous exploration, send an exit goal, send a second exploration goal, change configs, or tune behavior.

## Guardrails

No implementation was performed.
No Gazebo/RViz/Nav2 runtime was launched.
No NavigateToPose goal was sent.
No maze_explorer was started.
No exploration goal was sent.
No Nav2/MPPI/controller/goal checker/config tuning was performed.
No exploration strategy was changed.
No branch scoring, centerline gate, fallback, or terminal acceptance logic was changed.
No autonomous exploration success or exit success is claimed.
Phase129 not entered.

## Verification

Verification performed:

- focused static tests for Phase128 proposal/report contract;
- process guard;
- Nav2 config diff guard;
- pycache cleanup guard.

Recorded verification artifacts:

- static test bundle: `log/phase128_first_goal_timeout_instrumentation_design/phase128_static_test_bundle.txt`
- final guard bundle: `log/phase128_first_goal_timeout_instrumentation_design/phase128_first_goal_timeout_instrumentation_design_final_guard.txt`

Actual command results:

```bash
python3 -m pytest src/tugbot_maze/test/test_phase128_first_goal_timeout_instrumentation_design.py -q
# 8 passed in 0.01s

python3 -m pytest \
  src/tugbot_maze/test/test_phase121_post_ingress_handoff_design.py \
  src/tugbot_maze/test/test_phase122_post_ingress_handoff_dry_start.py \
  src/tugbot_maze/test/test_phase123_first_exploration_goal_dispatch_design.py \
  src/tugbot_maze/test/test_phase124_first_exploration_goal_dispatch_smoke.py \
  src/tugbot_maze/test/test_phase125_first_exploration_goal_execution_result_smoke.py \
  src/tugbot_maze/test/test_phase126_first_goal_timeout_diagnosis_design.py \
  src/tugbot_maze/test/test_phase127_first_goal_timeout_artifact_replay.py \
  src/tugbot_maze/test/test_phase128_first_goal_timeout_instrumentation_design.py \
  -q
# 61 passed in 0.11s
```

Final guard results:

```text
runtime process guard: empty
Nav2 config diff guard: empty
exploration/config source diff guard outside Phase128 focused test: empty
scoped pycache guard after cleanup: empty
```

## Stop condition

Phase128 stops here. Phase129 not entered. Wait for human acceptance before any Phase129 work.
