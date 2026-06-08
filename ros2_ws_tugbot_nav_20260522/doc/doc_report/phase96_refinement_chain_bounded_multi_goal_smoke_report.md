# Phase96 Phase88/92 refinement chain bounded multi-goal smoke report

Status: DRAFT_PENDING_RUNTIME_SMOKE

Scope: Phase96 runs a short visible bounded smoke with max_goals:=2~3 to validate whether the Phase88/92 refinement chain remains stable across consecutive explore goals. This phase is validation-only.

## Required cleanup

Initial cleanup was performed before required reading and tooling.

Cleanup artifact:

```text
log/phase96_refinement_chain_bounded_multi_goal_smoke/phase96_initial_cleanup_summary.json
```

Observed corrected cleanup summary:

```text
matching_processes_before_count=0
matching_processes_after_count=0
requested_components_confirmed_absent_project_scoped=true
```

Cleanup scope was limited to Phase95/Phase96 wrapper descendants and commands containing the Tugbot workspace path / tugbot launch identity / maze_explorer identity. Unrelated generic ROS/Gazebo processes without project identity were not targeted.

## Required reading completed

Read before runtime validation:

1. `doc/doc_report/phase95_applied_refinement_terminal_outcome_bounded_validation_report.md`
2. `doc/doc_report/phase92_two_step_corridor_alignment_staging_goal_minimal_implementation_report.md`
3. `doc/doc_report/phase88_safety_first_multi_candidate_forward_search_minimal_implementation_report.md`
4. `doc/doc_proposal/obstacle_reproduction_handoff_workflow.md`

## Added Phase96 tooling

```text
tools/run_phase96_refinement_chain_bounded_multi_goal_smoke.sh
tools/analyze_phase96_refinement_chain_bounded_multi_goal_smoke.py
tools/record_phase96_smoke_evidence.py
src/tugbot_maze/test/test_phase96_refinement_chain_bounded_multi_goal_smoke.py
doc/doc_report/phase96_refinement_chain_bounded_multi_goal_smoke_runbook.md
doc/doc_report/phase96_refinement_chain_bounded_multi_goal_smoke_report.md
```

## Analyzer contract

Allowed classifications:

```text
REFINEMENT_CHAIN_BOUNDED_SMOKE_PASS
REFINEMENT_CHAIN_GOAL_TIMEOUT
REFINEMENT_CHAIN_RECOVERY_DOMINANT
REFINEMENT_CHAIN_STAGING_TRIGGERED_NEEDS_REVIEW
REFINEMENT_CHAIN_INSUFFICIENT_EVIDENCE
```

Per-goal fields:

```text
refinement_applied
multi_candidate_forward_search
hard_safety_pass_candidate_count
two_step_staging_plan
staging_applied
second_step_forward_goal
terminal_outcome
recoveries
local_cost_risk
```

## Runtime result

Pending bounded visible smoke.

Required runtime artifacts:

```text
/maze/goal_events
Nav2 feedback
local costmap samples
scan
odom/TF
```

## Guardrails

No maze_explorer strategy changed.
No Phase88/92 logic changed.
No branch scoring changed.
No exploration order changed.
No centerline gate changed.
No directional readiness changed.
No fallback/terminal acceptance changed.
No Nav2/MPPI/controller tuning.
No inflation/robot_radius/clearance_radius_m/map threshold tuning.
Timeout is not success.
No autonomous exploration success claimed.
No exit success claimed.
Phase97 not entered.

## TDD evidence

Observed RED before Phase96 analyzer/wrapper/recorder/runbook/report existed:

```text
PYTHONDONTWRITEBYTECODE=1 pytest -q src/tugbot_maze/test/test_phase96_refinement_chain_bounded_multi_goal_smoke.py
7 failed in 0.05s
```

## Stop condition

Phase96 stops after bounded smoke, artifact analysis, report finalization, focused/static verification, and cleanup/held-scene decision. Phase97 is not entered.
