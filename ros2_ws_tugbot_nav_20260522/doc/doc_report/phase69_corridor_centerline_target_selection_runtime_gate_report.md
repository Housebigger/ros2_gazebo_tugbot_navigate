# Phase69 Post-Phase68 Report: Corridor Centerline Target Selection Runtime Gate / Bounded Smoke

Run id: `phase69_corridor_centerline_target_selection_runtime_gate`
Artifact dir: `log/phase69_corridor_centerline_target_selection_runtime_gate/`

## Preserved upstream conclusion

Phase68 human acceptance conclusion is preserved as `CENTERLINE_REPLAY_IMPROVES_LOCAL_COST`.
Goal1 original target was offset from corridor centerline; static replay showed a centerline / max-clearance candidate can reduce balance error and front-wedge risk.

This report does not claim autonomous exploration success and does not claim exit success.
Any dispatch below is bounded first-dispatch evidence only.

## Phase69 goal

Minimally integrate a runtime corridor centerline target refinement gate in `maze_explorer` for already-selected dispatch targets. The gate records both `original_target` and `refined_target` and replaces the dispatch target only when all narrow conditions hold:

- same-corridor evidence;
- two-side-wall evidence;
- improved balance error;
- improved/preserved min clearance;
- improved/preserved local-cost radius evidence;
- improved/preserved front-wedge evidence;
- forward progress not lowered.

## Guardrails

Observed guardrail state:

- bounded runtime only: PASS, 2 bounded replays;
- max_goals=1..2: PASS, `max_goals=1`;
- no Nav2/MPPI/controller parameter edits: PASS, Nav2 config diff empty;
- no inflation/robot_radius/clearance_radius_m/map threshold tuning: PASS;
- no branch scoring change: PASS, runtime diagnostics `branch_scoring_changed=false`;
- no corridor-following cmd_vel control: PASS;
- no fallback/terminal acceptance change: PASS, `near_exit_fallback_enabled=false`;
- no old scaffold world/map: PASS, active clean scaled2x world;
- no autonomous exploration success claim: PASS;
- no exit success claim: PASS;
- cleanup: PASS after final cleanup check empty.

## Implementation summary

Files:

- `src/tugbot_maze/tugbot_maze/maze_perception.py`
  - added pure helper `refine_corridor_centerline_target(...)`.
  - helper only evaluates the chosen target and candidate offsets; it does not rank branches or alter topology policy.
- `src/tugbot_maze/tugbot_maze/maze_explorer.py`
  - added conservative runtime parameters:
    - `centerline_target_refinement_enabled=true`
    - `centerline_target_refinement_side_probe_m=1.5`
    - `centerline_target_refinement_forward_offsets_m=0.0,0.1,0.2`
    - `centerline_target_refinement_lateral_offsets_m=-0.45,-0.30,-0.15,0.0,0.15,0.30,0.45`
  - added `_maybe_refine_corridor_centerline_dispatch_target` immediately before sending the Nav2 goal.
  - preserved `original_target` and `refined_target` in `/maze/goal_events`.
  - added structured diagnostics under `centerline_target_refinement` and flattened fields:
    - `centerline_refinement_applied`
    - `centerline_refinement_reason`
    - `centerline_refinement_candidate_count`
    - `centerline_refinement_eligible_candidate_count`
    - `centerline_refinement_gate_conditions`
    - `branch_scoring_changed=false`
- `tools/run_phase69_corridor_centerline_target_selection_runtime_gate.sh`
  - bounded runtime wrapper using Phase65 inner ingress `(2.0, 0.0, 0.0)`.
  - default `PHASE69_MAX_GOALS=1`, allowed 1..2.
  - default `PHASE69_REPLAY_COUNT=2`, allowed 1..3.
  - records goal events, explorer state, cmd_vel/controller, Nav2 feedback, local costmap samples, global plan samples, collision monitor state.
- `tools/analyze_phase69_corridor_centerline_target_selection_runtime_gate.py`
  - analyzer / runtime recorder.
  - allowed classifications:
    - `CENTERLINE_RUNTIME_GOAL1_IMPROVES`
    - `CENTERLINE_RUNTIME_TIMEOUT_REMAINS`
    - `CENTERLINE_GATE_NO_APPLY`
    - `CENTERLINE_RUNTIME_REGRESSION`
    - `INSUFFICIENT_EVIDENCE`
    - `GUARDRAIL_VIOLATION_STRATEGY_CHANGED`
- `src/tugbot_maze/test/test_phase69_corridor_centerline_target_selection_runtime_gate.py`
  - focused TDD coverage for pure gate behavior, dual-target diagnostics, wrapper/analyzer guardrails, and no-success-claim contract.

## Runtime result

Command executed:

```bash
PHASE69_REPLAY_COUNT=2 PHASE69_MAX_GOALS=1 PHASE69_EXPLORER_OBSERVE_SEC=110 PHASE69_RUNTIME_RECORD_TIMEOUT_SEC=130 PHASE69_RUN_TIMEOUT_SEC=230 PHASE69_GOAL_TIMEOUT_SEC=90 ./tools/run_phase69_corridor_centerline_target_selection_runtime_gate.sh
```

Summary JSON:

- `log/phase69_corridor_centerline_target_selection_runtime_gate/phase69_corridor_centerline_target_selection_runtime_gate.json`

Top-level runtime metrics:

- classification: `CENTERLINE_GATE_NO_APPLY`
- replay_count: 2
- inner_ingress_goal_success: true
- dispatch_count_total: 2
- centerline_refinement_applied_count: 0
- centerline_gate_no_apply_count: 2
- outcome_count_total: 4 (`timeout` plus cancel-result terminal events per replay)
- timeout_count_total: 4
- nav2_config_diff_empty: true
- cleanup_empty after final cleanup check: true
- guardrail_violation: false
- complete_autonomous_success_claimed: false
- exit_success_claimed: false

Replay 01:

- inner ingress: success
- dispatch observed: yes, max_goals=1
- target: `[2.0401681953130626, 1.0236338478324454]`
- original_target: `[2.0401681953130626, 1.0236338478324454]`
- refined_target: `[2.0401681953130626, 1.0236338478324454]`
- centerline_refinement_applied: false
- centerline_refinement_reason: `no_improving_candidate`
- candidate_count / eligible_candidate_count: 21 / 0
- original target metrics: two-side-wall evidence true, same-corridor true, min_clearance about 0.50 m, local_cost_max_radius 46, front_wedge_high_cost_count 0
- outcome: timeout / `FAILED_EXHAUSTED`
- Nav2 recoveries max: 4
- robot distance-to-target improvement: 0.646 m; final distance about 0.369 m
- local-cost/front-wedge risk still observed during motion.

Replay 02:

- inner ingress: success
- dispatch observed: yes, max_goals=1
- target: `[2.083721006796267, 1.0227875572355678]`
- original_target: `[2.083721006796267, 1.0227875572355678]`
- refined_target: `[2.083721006796267, 1.0227875572355678]`
- centerline_refinement_applied: false
- centerline_refinement_reason: `no_improving_candidate`
- candidate_count / eligible_candidate_count: 21 / 0
- original target metrics: two-side-wall evidence true, same-corridor true, min_clearance about 0.55 m, local_cost_max_radius 54, front_wedge_high_cost_count 0
- outcome: timeout / `FAILED_EXHAUSTED`
- Nav2 recoveries max: 4
- robot distance-to-target improvement: 0.690 m; final distance about 0.335 m
- local-cost/front-wedge risk still observed during motion.

Interpretation:

- Phase69 runtime gate was implemented and exercised.
- In both bounded replays, the gate did not apply because no candidate satisfied all narrow gate conditions; dispatch proceeded with the original target.
- This is not a runtime improvement claim and not an exit/autonomous-success claim.
- Because the gate did not apply, the correct classification is `CENTERLINE_GATE_NO_APPLY`, not `CENTERLINE_RUNTIME_TIMEOUT_REMAINS`.

## Validation log

All requested checks completed:

```text
python3 -m py_compile src/tugbot_maze/tugbot_maze/maze_perception.py src/tugbot_maze/tugbot_maze/maze_explorer.py tools/analyze_phase69_corridor_centerline_target_selection_runtime_gate.py
PASS

bash -n tools/run_phase69_corridor_centerline_target_selection_runtime_gate.sh
PASS

pytest -q src/tugbot_maze/test/test_phase69_corridor_centerline_target_selection_runtime_gate.py
6 passed in 0.43s

source /opt/ros/jazzy/setup.bash && colcon build --packages-select tugbot_maze tugbot_bringup --symlink-install
Summary: 2 packages finished [1.05s]

git diff -- src/tugbot_navigation/config | wc -c
0

wc -l < log/phase69_corridor_centerline_target_selection_runtime_gate/phase69_corridor_centerline_target_selection_runtime_gate_cleanup_processes_after.txt
0
```

## Final classification

`CENTERLINE_GATE_NO_APPLY`

Reason: bounded runtime produced two first dispatches, both with structured original/refined diagnostics, but the conservative centerline gate found zero eligible candidates and therefore preserved the original target in both replays. Timeouts remained after dispatch, but Phase69 did not apply the centerline refinement, so this phase cannot be classified as a centerline runtime improvement or applied-timeout result.

## Stop condition

Phase69 is complete and stopped for human acceptance. Do not enter Phase70 / 不进入 Phase70.
