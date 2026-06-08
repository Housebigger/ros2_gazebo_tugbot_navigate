# Phase83: Corridor-aligned intermediate goal design review report

Status: completed / stopped at Phase83.

Result:

`DESIGN_REVIEW_COMPLETE_PENDING_EXPLICIT_IMPLEMENTATION_PHASE`

Phase83 is design review only. It does not implement runtime behavior and does not enter Phase84.

## Scope

Phase83 reviews a future design direction after Phase80/81/82 clarified the Goal2 timeout geometry and local-cost split.

Phase83 creates:

- design proposal: `doc/doc_proposal/phase83_corridor_aligned_intermediate_goal_design_review.md`
- report: `doc/doc_report/phase83_corridor_aligned_intermediate_goal_design_review_report.md`
- focused static test: `src/tugbot_maze/test/test_phase83_corridor_aligned_intermediate_goal_design_review.py`

No runtime code is changed in this phase.

## Required source review

Read before writing the design:

- `doc/doc_proposal/obstacle_reproduction_handoff_workflow.md`
- `doc/doc_report/phase80_goal2_near_goal_forward_open_corridor_diagnostic_classification_report.md`
- `doc/doc_report/phase81_goal2_forward_open_machine_evidence_capture_report.md`
- `doc/doc_report/phase82_goal2_local_cost_scan_mismatch_root_cause_report.md`
- top-level design proposals under `doc/doc_proposal/`:
  - `phase76_obstacle_triggered_visual_root_cause_workflow.md`
  - `phase27_alt_r5_footprint_terminal_acceptance_design.md`
  - `phase27_alt_near_exit_fallback_design.md`
- corridor-centerline prior references:
  - `references/tugbot_maze_phase68_corridor_centerline_target_selection_static_replay.md`
  - `references/tugbot_maze_phase69_corridor_centerline_runtime_gate.md`
  - `references/tugbot_maze_phase70_balance_first_centerline_runtime_gate.md`

## Evidence chain from Phase80 / Phase81 / Phase82

Phase80:

- classification: `NEAR_GOAL_LATERAL_RESIDUAL_WITH_FORWARD_OPEN_CORRIDOR`
- key point: Goal2 timeout residual was almost entirely lateral in the robot body frame.
- implication: do not accept the timeout; instead avoid generating target poses that force near-goal lateral correction.

Phase81:

- classification: `FORWARD_OPEN_CORRIDOR_BLOCKED`
- key point: raw `/scan` showed physical forward clearance, but `/local_costmap/costmap` had high/lethal costs in the robot-forward execution corridor.
- implication: visible physical openness alone is not enough; a future goal must be locally executable for the robot footprint and front wedge.

Phase82:

- primary classification: `FOOTPRINT_OR_WEDGE_PROJECTION_SUSPECTED`
- secondary supported signal: `INFLATION_SPILLOVER_SUSPECTED`
- key point: the local-cost/scan mismatch is best treated as a footprint/wedge/costmap projection issue, not a reason to lower safety boundaries.
- implication: do not tune inflation or robot geometry in Phase83. Design better-aligned intermediate goals instead.

## Design reviewed

Phase83 reviews this future refinement chain:

`candidate point -> corridor centerline projection -> corridor heading -> forward executability check -> Nav2 goal`

The design is intentionally scoped to the already-selected candidate/branch:

1. take the candidate already chosen by existing exploration.
2. require same-corridor evidence.
3. require two-side-wall or equivalent corridor-boundary evidence.
4. run centerline projection in a bounded neighborhood of the candidate.
5. set goal orientation to corridor heading.
6. run a forward executability check over map/local-cost/scan/TF evidence.
7. produce a Nav2 goal pose only if all gates pass.
8. otherwise keep the original target unchanged and record a reject reason.

## Why this is the preferred design direction

The core principle is:

Do not make an unsafe or poorly aligned target easier to reach by weakening safety boundaries. Make the exploration layer produce a target that is more suitable for Nav2 execution.

For Goal2 specifically:

- Phase80 says the near-goal problem is lateral residual.
- Phase81 says the physical scan ahead can be open while local costmap execution is blocked.
- Phase82 says footprint/wedge projection and inflation spillover signals explain that split.

Therefore the future design should target:

- centerline-aligned XY target.
- corridor-heading orientation.
- no sideways near-goal correction.
- footprint/front-wedge local executability.
- conservative no-apply when evidence is missing.

## Required future diagnostics

If a later phase implements the design, each dispatch event should record:

- `corridor_aligned_intermediate_goal_refinement`
- `original_target`
- `centerline_projected_target`
- `corridor_heading_yaw`
- `nav2_goal_pose`
- `refinement_applied`
- `refinement_reject_reason`
- `same_corridor_evidence`
- `two_side_wall_evidence`
- `centerline_projection_metrics`
- `forward_executability_check`
- `near_goal_lateral_residual_handling`
- `branch_scoring_changed=false`
- `fallback_terminal_acceptance_used=false`

## Guardrails preserved

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

Additional explicit boundaries:

- No `maze_explorer.py` runtime integration.
- No branch-selection or scoring changes.
- No fallback or terminal-acceptance path is used.
- No Nav2 / MPPI / controller / inflation / robot-radius / clearance-radius / map-threshold change is proposed as Phase83 work.
- No Gazebo/Nav2 long run is performed.

## Static test contract

Focused static test:

```bash
PYTHONDONTWRITEBYTECODE=1 pytest -q src/tugbot_maze/test/test_phase83_corridor_aligned_intermediate_goal_design_review.py
```

It verifies that the proposal/report contain:

- design-only status.
- `centerline projection`.
- `corridor heading`.
- `forward executability check`.
- `near-goal lateral residual handling`.
- guardrails against tuning and fallback/terminal acceptance.
- Phase80/81/82 evidence tokens.
- `Phase84 not entered`.

RED proof:

Before the Phase83 proposal/report existed, the test failed as expected:

```text
5 failed
Phase83 design proposal is missing
Phase83 report is missing
```

## Verification

Final verification command:

```bash
PYTHONDONTWRITEBYTECODE=1 pytest -q src/tugbot_maze/test/test_phase83_corridor_aligned_intermediate_goal_design_review.py && \
  printf '\n--- syntax ---\n' && \
  PYTHONDONTWRITEBYTECODE=1 python3 -m py_compile src/tugbot_maze/test/test_phase83_corridor_aligned_intermediate_goal_design_review.py && \
  printf '\n--- runtime/config diff guard ---\n' && \
  git diff -- src/tugbot_maze/tugbot_maze/maze_explorer.py src/tugbot_navigation/config src/tugbot_bringup/launch src/tugbot_maze/tugbot_maze | cat && \
  printf '\n--- phase83 doc token checks ---\n' && \
  python3 - <<'PY'
from pathlib import Path
proposal=Path('doc/doc_proposal/phase83_corridor_aligned_intermediate_goal_design_review.md').read_text()
report=Path('doc/doc_report/phase83_corridor_aligned_intermediate_goal_design_review_report.md').read_text()
for token in ['DESIGN_REVIEW_ONLY_NOT_RUNTIME_ENABLED','centerline projection','corridor heading','forward executability check','near-goal lateral residual handling','DESIGN_REVIEW_COMPLETE_PENDING_EXPLICIT_IMPLEMENTATION_PHASE','Phase84 not entered']:
    print(token, token in proposal or token in report)
PY
```

Observed output:

```text
.....                                                                    [100%]
5 passed in 0.01s

--- syntax ---

--- runtime/config diff guard ---

--- phase83 doc token checks ---
DESIGN_REVIEW_ONLY_NOT_RUNTIME_ENABLED True
centerline projection True
corridor heading True
forward executability check True
near-goal lateral residual handling True
DESIGN_REVIEW_COMPLETE_PENDING_EXPLICIT_IMPLEMENTATION_PHASE True
Phase84 not entered True
```

Interpretation: focused static tests passed, Python syntax passed, and runtime/config guard diff was empty.

Final pycache cleanup after `py_compile`:

```text
removed_pycache_count 1
src/tugbot_maze/test/__pycache__
remaining_pycache_count 0
```

## Stop condition

Phase83 stops after the design proposal, report, and focused static tests. It does not implement repair logic and does not enter Phase84.

Phase84 not entered.
