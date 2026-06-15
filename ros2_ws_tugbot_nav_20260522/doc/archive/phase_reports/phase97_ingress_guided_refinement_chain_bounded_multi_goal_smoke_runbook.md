# Phase97 ingress-guided refinement chain bounded multi-goal smoke runbook

Status: READY_FOR_BOUNDED_RUNTIME_VALIDATION

Goal
- Run a short visible ingress-guided smoke under the corrected startup flow.
- First send an explicit inner-ingress Nav2 goal before maze_explorer.
- After ingress succeeds, start maze_explorer with max_goals:=2~3 and wait for terminal outcomes for the bounded goal set or a bounded failure trigger.
- Validate Phase88/92 refinement-chain stability across 2 to 3 explore goals only.

Required startup flow
1. Clean old project-scoped Gazebo/RViz/SLAM/Nav2/maze_explorer/recorder processes and write a cleanup summary.
2. Start visible Gazebo/RViz/SLAM/Nav2.
3. Wait for readiness:
   - /navigate_to_pose ready
   - /map ready
   - /scan ready
   - /local_costmap/costmap ready
   - TF ready, especially map->base_link
4. Send explicit inner-ingress Nav2 goal before maze_explorer.
5. If ingress fails, do not start maze_explorer; classify INGRESS_GUIDED_MULTI_GOAL_INGRESS_FAILED.
6. If ingress succeeds, start maze_explorer with max_goals:=2~3.
7. Run wait_for_terminal_set_or_failure.
8. Stop at the first bounded classification condition; do not continue long exploration.

Command shape
```bash
PHASE97_MAX_GOALS=3 \
PHASE97_INGRESS_TIMEOUT_SEC=90 \
PHASE97_GOAL_TIMEOUT_SEC=180.0 \
PHASE97_SMOKE_WINDOW_SEC=420 \
PHASE97_RUNTIME_RECORD_TIMEOUT_SEC=480 \
PHASE97_CLEANUP_ON_EXIT=0 \
PYTHONDONTWRITEBYTECODE=1 \
tools/run_phase97_ingress_guided_refinement_chain_bounded_multi_goal_smoke.sh
```

Primary artifacts
- log/phase97_ingress_guided_refinement_chain_bounded_multi_goal_smoke/phase97_ingress_guided_refinement_chain_bounded_multi_goal_smoke_goal_events.jsonl
- log/phase97_ingress_guided_refinement_chain_bounded_multi_goal_smoke/phase97_ingress_guided_refinement_chain_bounded_multi_goal_smoke_nav2_feedback.jsonl
- log/phase97_ingress_guided_refinement_chain_bounded_multi_goal_smoke/phase97_ingress_guided_refinement_chain_bounded_multi_goal_smoke_local_costmap_samples.jsonl
- log/phase97_ingress_guided_refinement_chain_bounded_multi_goal_smoke/phase97_ingress_guided_refinement_chain_bounded_multi_goal_smoke_raw_capture.json
- log/phase97_ingress_guided_refinement_chain_bounded_multi_goal_smoke/phase97_ingress_guided_refinement_chain_bounded_multi_goal_smoke_ingress_result.json
- log/phase97_ingress_guided_refinement_chain_bounded_multi_goal_smoke/phase97_ingress_guided_refinement_chain_bounded_multi_goal_smoke_analysis.json
- log/phase97_ingress_guided_refinement_chain_bounded_multi_goal_smoke/phase97_ingress_guided_refinement_chain_bounded_multi_goal_smoke_minimal_field_summary.md

Per-goal analysis fields
For each explore goal, analyze:
- dispatch
- terminal outcome
- refinement_applied
- multi_candidate_forward_search
- hard_safety_pass_candidate_count
- two_step_staging_plan
- staging_applied
- second_step_forward_goal
- recoveries
- distance_remaining curve
- local_cost_risk, including front_wedge and footprint lethal metrics where available

Classification set
- INGRESS_GUIDED_REFINEMENT_CHAIN_BOUNDED_PASS
- INGRESS_GUIDED_REFINEMENT_CHAIN_GOAL_TIMEOUT
- INGRESS_GUIDED_REFINEMENT_CHAIN_RECOVERY_DOMINANT
- INGRESS_GUIDED_REFINEMENT_CHAIN_STAGING_TRIGGERED_NEEDS_REVIEW
- INGRESS_GUIDED_REFINEMENT_CHAIN_INSUFFICIENT_EVIDENCE
- INGRESS_GUIDED_MULTI_GOAL_INGRESS_FAILED

Failure/hold-scene rule
- If ingress fails, goal timeout occurs, recovery-dominant behavior occurs, staging triggers and needs review, or evidence is insufficient due a stuck/capped runtime, keep Gazebo/RViz visible for screenshots when practical.
- Output only a minimal summary and screenshot suggestions in that case.

Guardrails
- No maze_explorer strategy changed.
- No Phase88/92 refinement logic changed.
- No branch scoring/exploration order/centerline gate/directional readiness/fallback/terminal acceptance changed.
- No Nav2/MPPI/controller/inflation/robot_radius/clearance_radius_m/map threshold tuning.
- No autonomous exploration success claimed.
- No exit success claimed.
- Phase98 not entered.
