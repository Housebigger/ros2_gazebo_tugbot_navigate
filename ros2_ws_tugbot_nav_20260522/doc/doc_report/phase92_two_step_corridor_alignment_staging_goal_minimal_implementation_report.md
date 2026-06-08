# Phase92 Two-step corridor alignment staging goal minimal implementation

Status: COMPLETE_MINIMAL_IMPLEMENTATION_VALIDATED_STOP_BEFORE_PHASE93

## Scope

Phase92 implements the minimal two-step corridor alignment staging workflow designed in Phase91. It is intentionally narrow:

- It is only evaluated for the Phase90-style trigger bundle.
- It does not change branch scoring, exploration order, centerline gate semantics, directional readiness override, fallback, or terminal acceptance.
- It does not tune Nav2, MPPI, controller, inflation, robot_radius, clearance_radius_m, or map threshold values.
- It does not claim autonomous exploration success or exit success.
- It does not treat timeout as success.

## Trigger bundle

The staging planner is only eligible when all trigger evidence is present:

1. Phase90-style trigger bundle.
2. near-goal lateral residual.
3. single-step forward search no hard-safety-pass.
4. safety_floor dominant blocker.
5. execution-time footprint/front-wedge risk.

If any trigger member is missing, the original target is preserved and the goal event contract records a staging_reject_reason.

## Implemented behavior

### Pure perception/planning helpers

File: `src/tugbot_maze/tugbot_maze/maze_perception.py`

New helpers:

- `plan_two_step_corridor_alignment_staging_goal(...)`
  - evaluates the Phase90-style trigger bundle against Phase88/centerline refinement evidence.
  - generates short-distance staging candidates toward corridor alignment.
  - uses corridor heading for staging yaw.
  - requires same-corridor and two-side-wall evidence.
  - requires hard-safety pass before staging can be applied.
  - rejects by preserving original target if the staging candidate family is unsafe.
  - emits `branch_scoring_changed=false` and `fallback_terminal_acceptance_used=false`.

- `generate_second_step_forward_goal_after_staging(...)`
  - requires fresh scan/local costmap/TF before generating a second-step forward goal.
  - calls the existing Phase88 `refine_corridor_centerline_target(...)` only after fresh evidence is available.
  - emits the selected second-step target only if a fresh-evidence Phase88 refinement applies.
  - emits `branch_scoring_changed=false` and `fallback_terminal_acceptance_used=false`.

### Runtime integration

File: `src/tugbot_maze/tugbot_maze/maze_explorer.py`

Minimal runtime wiring:

- `_send_goal(..., skip_two_step_staging=False)` evaluates staging after the existing centerline refinement evidence is available.
- When staging applies, the dispatched Nav2 goal kind becomes `corridor_alignment_staging`.
- Staging success is not treated as exploration success and does not mark the branch explored.
- After staging success, the node waits for fresh scan/local costmap/TF before dispatching the second-step forward goal.
- The second-step dispatch uses `skip_two_step_staging=True` to avoid recursive staging.
- Goal event payloads now expose:
  - `two_step_staging_plan`
  - `staging_goal_pose`
  - `staging_reason`
  - `staging_executability_check`
  - `second_step_forward_goal`
  - `staging_applied`
  - `staging_reject_reason`
  - `branch_scoring_changed=false`
  - `fallback_terminal_acceptance_used=false`

## Guardrails

Phase92 keeps these guardrails explicit:

- staging is not fallback.
- staging is not terminal acceptance.
- timeout is not success.
- No Nav2/MPPI/controller tuning.
- No inflation/robot_radius/clearance_radius_m/map threshold tuning.
- No branch scoring change.
- No exploration order change.
- No centerline gate replacement.
- No directional readiness override change.
- No autonomous exploration success claimed.
- No exit success claimed.
- No long autonomous exploration run was performed.

## Focused tests

New test file:

`src/tugbot_maze/test/test_phase92_two_step_corridor_alignment_staging_goal_minimal_implementation.py`

Coverage:

1. Positive trigger bundle generates a short, hard-safe corridor-alignment staging pose.
2. Negative trigger case preserves the original target when the single-step forward search has a hard-safe candidate.
3. Staging rejects when the candidate family cannot pass the safety floor.
4. Second-step forward goal requires fresh scan/local costmap/TF and then uses the Phase88 forward refinement path.
5. Maze Explorer exposes the goal_events contract and runtime guardrail tokens.
6. This report records validation, guardrails, and the Phase92 stop condition.

Observed RED before implementation:

```text
PYTHONDONTWRITEBYTECODE=1 pytest -q src/tugbot_maze/test/test_phase92_two_step_corridor_alignment_staging_goal_minimal_implementation.py
ImportError: cannot import name 'generate_second_step_forward_goal_after_staging'
1 error in 0.06s
```

Observed intermediate run after partial implementation:

```text
4 passed, 2 failed in 0.13s
```

The remaining failures were expected because the report did not exist yet and the publish payload had not yet exposed all Phase92 fields.

## Final validation

Focused Phase92 tests:

```text
PYTHONDONTWRITEBYTECODE=1 pytest -q src/tugbot_maze/test/test_phase92_two_step_corridor_alignment_staging_goal_minimal_implementation.py
......                                                                   [100%]
6 passed in 0.11s
```

Compatibility tests for the Phase88 and Phase84 refinement contracts:

```text
PYTHONDONTWRITEBYTECODE=1 pytest -q src/tugbot_maze/test/test_phase88_safety_first_multi_candidate_forward_search_minimal_implementation.py src/tugbot_maze/test/test_phase84_corridor_aligned_intermediate_goal_refinement.py
..........                                                               [100%]
10 passed in 0.37s
```

Syntax validation:

```text
PYTHONDONTWRITEBYTECODE=1 python3 -m py_compile src/tugbot_maze/tugbot_maze/maze_perception.py src/tugbot_maze/tugbot_maze/maze_explorer.py src/tugbot_maze/test/test_phase92_two_step_corridor_alignment_staging_goal_minimal_implementation.py
exit 0
```

Nav2/config guard diff:

```text
git diff -- src/tugbot_navigation/config
empty output
```

Cleanup check:

```text
remaining_pycache_count=0
```

## Stop condition

Stop: do not enter Phase93.

Phase92 stops after implementation, focused validation, compatibility validation, py_compile, guard diff, cleanup, and report finalization. Phase92 does not start or define Phase93 work.
