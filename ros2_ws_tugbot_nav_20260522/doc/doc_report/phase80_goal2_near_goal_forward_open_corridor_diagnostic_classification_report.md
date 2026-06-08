# Phase80: Goal2 near-goal forward-open corridor diagnostic classification

Status: completed / stopped at Phase80.

Classification: `NEAR_GOAL_LATERAL_RESIDUAL_WITH_FORWARD_OPEN_CORRIDOR`

Forward-open machine evidence status: `FORWARD_OPEN_EVIDENCE_INSUFFICIENT`

## Scope and guardrails

Phase80 is diagnostic classification only. It does not repair or tune the robot.

Guardrails preserved:

- No maze_explorer strategy changed.
- No branch scoring changed.
- No centerline gate changed.
- No directional readiness override changed.
- No fallback/terminal acceptance changed.
- No Nav2/MPPI/controller tuning.
- No inflation / robot_radius / clearance_radius_m / map threshold tuning.
- No autonomous exploration success claimed.
- No exit success claimed.
- This classification is not exit success, not fallback/terminal acceptance, and not a Nav2 parameter final diagnosis.

Phase81 not entered.

## Required source review

Read or checked before implementation:

- `doc/doc_proposal/obstacle_reproduction_handoff_workflow.md`
- `log/phase79_goal2_timeout_bounded_reproduction_handoff/phase79_goal2_timeout_bounded_reproduction_handoff_minimal_field_summary.md`
- `doc/doc_report/phase75_goal2_timeout_after_directional_redispatch_diagnosis_report.md`
- `doc/doc_report/phase77_goal2_timeout_visual_root_cause_replay_first_application_report.md`
- Phase79 machine artifacts under `log/phase79_goal2_timeout_bounded_reproduction_handoff/`
- Phase77 event artifact `log/phase77_goal2_visual_root_cause_replay/phase77_goal2_timeout_visual_root_cause_event.json`

Note: the requested file `doc/doc_report/phase79_goal2_timeout_bounded_reproduction_handoff_report.md` was not present in this workspace during Phase80. Phase80 did not fabricate it; it used the present Phase79 minimal field summary and machine artifacts instead.

## Analyzer and output files

Analyzer:

- `tools/analyze_phase80_goal2_near_goal_lateral_residual.py`

Analyzer output:

- `log/phase80_goal2_near_goal_forward_open_corridor_diagnostic_classification/phase80_goal2_near_goal_forward_open_corridor_diagnostic_classification.json`

Focused tests:

- `src/tugbot_maze/test/test_phase80_goal2_near_goal_lateral_residual.py`

## Phase79 Goal2 artifact basis

Source artifact directory:

- `log/phase79_goal2_timeout_bounded_reproduction_handoff`

Relevant observed trigger chain:

- `goal_timeout`
- local-cost / front-wedge risk
- recovery loop
- near-goal outside XY tolerance

The Phase79 minimal field summary had:

- Robot pose: `[2.426872326194781, 1.0327763735179467, 1.6075951571994673]`
- Target: `[2.058503376259287, 1.0236490342193865]`
- recoveries: `3`
- final_xy_error_m: `0.36848200987191976`

The machine analyzer used the full Phase79 runtime artifacts and computed the final diagnostic from the latest local-cost/controller evidence:

- terminal pose: `[2.419572731214336, 1.0337551342713849, 1.6026758930612244]`
- target: `[2.057855221699651, 1.0261005743935105]`
- yaw: `1.6026758930612244 rad`
- final_xy_error_m: `0.3617984922252026`
- xy_goal_tolerance_m: `0.25`
- near_goal_threshold_m: `0.5`
- near_goal: `true`
- outside_xy_tolerance: `true`
- near_goal_outside_xy_tolerance: `true`
- recoveries_max: `3`
- front_wedge_cost_max: `100.0` observed across Goal2 samples; latest sample front_wedge_cost.max was `99`
- front_wedge_clearance_m at timeout: `0.04354444986482647`
- local_cost_sample_count: `185`
- feedback_sample_count: `4613`
- controller_odom_sample_count: `3056`

## Body-frame target residual classification

At the terminal pose, the target relative to the robot body frame was:

- map dx: `-0.36171750951468473 m`
- map dy: `-0.007654559877874334 m`
- robot yaw: `1.6026758930612244 rad`
- forward component: `0.0038787736571661074 m`
- lateral component: `0.361777699825938 m`
- absolute forward component: `0.0038787736571661074 m`
- absolute lateral component: `0.361777699825938 m`
- dominant residual axis: `lateral`
- target_is_nearly_lateral: `true`
- target_is_behind_robot: `false`

Interpretation:

The robot is near Goal2 but outside the 0.25 m XY tolerance. The remaining target error is almost entirely lateral in the robot body frame, not forward. This matches the user observation that the robot's current forward direction appears open and the problem may be Nav2 trying to remove lateral near-goal residual rather than a simple physical obstacle directly ahead.

Phase80 therefore adds the diagnostic classification:

`NEAR_GOAL_LATERAL_RESIDUAL_WITH_FORWARD_OPEN_CORRIDOR`

This is a diagnostic label only. It does not authorize acceptance of the timed-out goal and does not change any terminal/fallback behavior.

## Forward-open evidence check

Phase80 attempted to classify machine evidence for the forward-open corridor condition from Phase79 scan/local-cost artifacts.

Result:

`FORWARD_OPEN_EVIDENCE_INSUFFICIENT`

Evidence gaps:

- `no_scan_artifact`
- `no_raw_local_costmap_geometry_for_forward_open_test`

Available local-cost notes:

- latest front_wedge_cost.max: `99.0`
- latest front_wedge lethal_count: `59.0`
- latest front_wedge high_cost_count: `150.0`
- latest front_wedge_clearance_m: `0.05`
- aggregate front-wedge local cost is high/lethal; this does not prove physical blockage or forward openness by itself.

Interpretation:

The user visual observation is important and supports the hypothesis that the visible physical corridor ahead may be open. However, the Phase79 machine artifact set does not contain raw scan/front-corridor geometry sufficient to prove forward-open automatically. The correct machine label is therefore `FORWARD_OPEN_EVIDENCE_INSUFFICIENT`, not a fabricated positive forward-open proof.

## Relation to Phase75 / Phase77 conclusions

Phase75 classified the same Goal2 timeout family as:

`GOAL2_TIMEOUT_LOCAL_COST_RECOVERY_LOOP`

Phase80 does not erase that classification. It refines the near-goal geometry interpretation using Phase79's held-scene observation and artifacts:

- Phase75 remains valid: Nav2 recoveries and high/lethal local-cost/front-wedge evidence were present.
- Phase80 adds: the final target residual is lateral relative to the robot body frame, while forward physical blockage is not proven by machine data.
- Phase80 does not conclude target acceptance, fallback acceptance, terminal acceptance, exit success, or autonomous success.
- Phase80 does not conclude that Nav2 parameters are definitively wrong.

## Verification

RED proof:

```text
PYTHONDONTWRITEBYTECODE=1 pytest -q src/tugbot_maze/test/test_phase80_goal2_near_goal_lateral_residual.py
```

Before the analyzer/report existed, the test failed as expected with missing analyzer/report failures. After the analyzer was added, remaining failures correctly exposed that the real artifact front-wedge maximum includes a `100.0` sample and that the report was not yet written.

Analyzer execution:

```text
PYTHONDONTWRITEBYTECODE=1 python3 tools/analyze_phase80_goal2_near_goal_lateral_residual.py --artifact-dir log/phase79_goal2_timeout_bounded_reproduction_handoff --output-json log/phase80_goal2_near_goal_forward_open_corridor_diagnostic_classification/phase80_goal2_near_goal_forward_open_corridor_diagnostic_classification.json
```

Observed analyzer classification:

```text
NEAR_GOAL_LATERAL_RESIDUAL_WITH_FORWARD_OPEN_CORRIDOR
FORWARD_OPEN_EVIDENCE_INSUFFICIENT
```

Final verification command was run after this report was written:

```bash
PYTHONDONTWRITEBYTECODE=1 pytest -q src/tugbot_maze/test/test_phase80_goal2_near_goal_lateral_residual.py && \
  printf '\n--- syntax ---\n' && \
  PYTHONDONTWRITEBYTECODE=1 python3 -m py_compile tools/analyze_phase80_goal2_near_goal_lateral_residual.py src/tugbot_maze/test/test_phase80_goal2_near_goal_lateral_residual.py && \
  printf '\n--- strategy/config diff ---\n' && \
  git diff -- src/tugbot_maze/tugbot_maze/maze_explorer.py src/tugbot_navigation/config src/tugbot_bringup/launch src/tugbot_maze/tugbot_maze | cat
```

Observed output:

```text
....                                                                     [100%]
4 passed in 0.18s

--- syntax ---

--- strategy/config diff ---
```

Interpretation: focused Phase80 tests passed, syntax passed, and strategy/config diff guard was empty.

After the final syntax check created `__pycache__` directories, they were removed from `tools/` and `src/tugbot_maze/test/`. Final cleanup check:

```text
removed_pycache_count 2
tools/__pycache__
src/tugbot_maze/test/__pycache__
remaining_pycache_count 0
```

## Final Phase80 conclusion

Goal2 final state is best described diagnostically as near-goal lateral residual: the target is almost exactly sideward relative to the robot's terminal yaw, with only about `0.0039 m` forward component but about `0.3618 m` lateral component. This is consistent with the user's visual observation that the corridor ahead appears physically open and that Nav2 may be trying to remove lateral residual around the goal.

The machine artifacts do not provide enough raw scan or forward-corridor geometry to prove forward-open automatically, so the forward-open evidence status remains:

`FORWARD_OPEN_EVIDENCE_INSUFFICIENT`

Stop here. Phase81 not entered.
