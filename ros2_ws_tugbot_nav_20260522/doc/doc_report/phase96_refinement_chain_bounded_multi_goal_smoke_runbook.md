# Phase96 refinement chain bounded multi-goal smoke runbook

Status: READY_FOR_BOUNDED_RUNTIME_VALIDATION

Purpose: run a short visible bounded smoke to validate the Phase88/92 refinement chain across consecutive explore goals. This is validation-only. It does not modify maze_explorer strategy, Phase88/92 logic, branch scoring, exploration order, centerline gate, directional readiness, fallback/terminal acceptance, Nav2, MPPI, controller, inflation, robot radius, clearance radius, or map thresholds.

## Scope

Run only a short smoke:

```text
max_goals:=2~3
visible Gazebo/RViz
bounded smoke window
stop at max_goals terminal set or first timeout/failure/cancel
```

No long autonomous exploration is allowed.

## Command

Recommended command:

```bash
cd /home/hyh/Desktop/agent_playground/playground_hermes/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260522
PHASE96_SKIP_BUILD=0 \
PHASE96_MAX_GOALS=3 \
PHASE96_GOAL_TIMEOUT_SEC=180.0 \
PHASE96_SMOKE_WINDOW_SEC=360 \
PHASE96_RUNTIME_RECORD_TIMEOUT_SEC=420 \
PHASE96_HOLD_AFTER_TRIGGER_SEC=0 \
PHASE96_CLEANUP_ON_EXIT=0 \
PYTHONDONTWRITEBYTECODE=1 \
tools/run_phase96_refinement_chain_bounded_multi_goal_smoke.sh
```

`PHASE96_MAX_GOALS` is intentionally restricted to `2` or `3`.

## Artifacts

Artifacts are written under:

```text
log/phase96_refinement_chain_bounded_multi_goal_smoke
```

Expected files:

```text
phase96_refinement_chain_bounded_multi_goal_smoke_goal_events.jsonl
phase96_refinement_chain_bounded_multi_goal_smoke_nav2_feedback.jsonl
phase96_refinement_chain_bounded_multi_goal_smoke_local_costmap_samples.jsonl
phase96_refinement_chain_bounded_multi_goal_smoke_runtime_timeline.jsonl
phase96_refinement_chain_bounded_multi_goal_smoke_controller_dynamics.jsonl
phase96_refinement_chain_bounded_multi_goal_smoke_global_plan_samples.jsonl
phase96_refinement_chain_bounded_multi_goal_smoke_explorer_state.jsonl
phase96_refinement_chain_bounded_multi_goal_smoke_raw_capture.json
phase96_refinement_chain_bounded_multi_goal_smoke_analysis.json
phase96_refinement_chain_bounded_multi_goal_smoke_minimal_field_summary.md
```

## Analyzer output

Allowed classifications:

```text
REFINEMENT_CHAIN_BOUNDED_SMOKE_PASS
REFINEMENT_CHAIN_GOAL_TIMEOUT
REFINEMENT_CHAIN_RECOVERY_DOMINANT
REFINEMENT_CHAIN_STAGING_TRIGGERED_NEEDS_REVIEW
REFINEMENT_CHAIN_INSUFFICIENT_EVIDENCE
```

Per-goal required fields:

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

## Failure / hold behavior

If classification is not `REFINEMENT_CHAIN_BOUNDED_SMOKE_PASS`, keep Gazebo/RViz observable for screenshots unless explicitly cleaned up later.

Minimal screenshot suggestions:

- Gazebo wide view of robot and corridor around the stopped goal.
- RViz local costmap around footprint/front wedge.
- RViz goal marker / terminal pose view for timeout or near-goal ambiguity.
- Recovery-loop view if recoveries dominate.

## Guardrails

- No maze_explorer strategy changed.
- No Phase88/92 logic changed.
- No branch scoring changed.
- No exploration order changed.
- No centerline gate changed.
- No directional readiness changed.
- No fallback/terminal acceptance changed.
- No Nav2/MPPI/controller tuning.
- No inflation/robot_radius/clearance_radius_m/map threshold tuning.
- Timeout is not success.
- No autonomous exploration success claimed.
- No exit success claimed.
- Phase97 not entered.
