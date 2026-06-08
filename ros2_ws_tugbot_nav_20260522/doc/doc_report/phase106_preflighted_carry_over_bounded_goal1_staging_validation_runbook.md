# Phase106 preflighted carry-over bounded Goal1 staging validation runbook

Purpose: rerun the Phase102 carry-over bounded Goal1 staging validation after Phase105 fail-closed ingress preflight hardening.

This is validation only. It does not modify maze_explorer strategy, Phase88/92/101/105 logic, branch scoring, exploration order, centerline gate, directional readiness, fallback, terminal acceptance, Nav2/MPPI/controller/inflation/robot_radius/clearance_radius_m, or map thresholds.

Required classification set:
- PREFLIGHTED_CARRY_OVER_APPLIED_STAGING_APPLIED
- PREFLIGHTED_CARRY_OVER_APPLIED_STAGING_SAFETY_REJECTED
- PREFLIGHTED_CARRY_OVER_REJECTED
- PREFLIGHTED_CARRY_OVER_NOT_TRIGGERED
- PHASE105_PREFLIGHT_REJECTED_BEFORE_INGRESS
- PREFLIGHTED_INGRESS_FAILED_AFTER_PREFLIGHT_PASS
- PREFLIGHTED_GOAL1_CARRY_OVER_VALIDATION_INSUFFICIENT_EVIDENCE

Procedure:
1. Project-scoped initial_cleanup closes old Gazebo/RViz/SLAM/Nav2/maze_explorer/recorder/ros2 launch processes and writes the cleanup summary.
2. Start a fresh visible stack: headless:=false, use_rviz:=true.
3. Wait for /navigate_to_pose, /map, /scan, /local_costmap/costmap, and map->base_link readiness.
4. Start read-only recorders for ingress_preflight, ingress_result, /maze/goal_events, Nav2 feedback, local_costmap samples, scan, map, odom, and TF.
5. Run Phase105 ingress_preflight.
6. If preflight rejects, do not send an ingress goal and do not start maze_explorer. Classify PHASE105_PREFLIGHT_REJECTED_BEFORE_INGRESS and record the reject token.
7. If preflight passes, send the unchanged inner-ingress goal only after the pass: frame_id=map, x=2.0, y=0.0, yaw=0.0.
8. If ingress fails after preflight pass, do not start maze_explorer. Classify PREFLIGHTED_INGRESS_FAILED_AFTER_PREFLIGHT_PASS.
9. If ingress succeeds, start maze_explorer with max_goals=1 and run only until Goal1 carry-over/staging context, Goal1 terminal outcome, or the bounded window expires.
10. Analyze Goal1 fields: corridor_evidence_carry_over, carry_over_source, carry_over_applied, carry_over_reject_reason, source_forward_window, staging_window, safety_evidence_recomputed, two_step_staging_plan, staging_executability_check, staging_applied, staging_reject_reason, branch_scoring_changed=false, fallback_terminal_acceptance_used=false.

Runtime command:
```bash
PHASE106_MAX_GOALS=1 \
PHASE106_INGRESS_TIMEOUT_SEC=90 \
PHASE106_GOAL_TIMEOUT_SEC=190.0 \
PHASE106_VALIDATION_WINDOW_SEC=260 \
PHASE106_RUNTIME_RECORD_TIMEOUT_SEC=320 \
PHASE106_CLEANUP_ON_EXIT=0 \
PYTHONDONTWRITEBYTECODE=1 \
tools/run_phase106_preflighted_carry_over_bounded_goal1_staging_validation.sh
```

If the run fails or gets stuck, keep Gazebo/RViz visible and output only a minimal summary plus screenshot suggestions:
- Gazebo wide view of robot, entrance/Goal1 corridor, and nearest walls.
- RViz map + robot pose + TF.
- RViz local costmap around robot footprint/front wedge.
- RViz Goal1 target/staging/plan markers if visible.

Direct validation questions:
- Phase105 preflight passed?
- Ingress goal sent only after preflight pass?
- Ingress succeeded?
- Goal1 dispatched?
- Was original missing_two_side_wall_evidence fixed by carry-over?
- Does carry_over_applied=true only mean corridor-level evidence reuse?
- Is safety_evidence_recomputed true?
- If staging still rejects, is the new reject reason staging_safety_recompute_failed or another token?

Guardrails:
- No autonomous exploration success claimed.
- No exit success claimed.
- No timeout/failure treated as success.
- No algorithm/config tuning.
- Phase107 not entered.
