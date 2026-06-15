# Phase93 Two-step staging bounded Goal2 validation runbook

Status: RUNBOOK_AND_TOOLING_FOR_VALIDATION_ONLY

This runbook only validates Phase92 two-step staging in a bounded Goal2-equivalent reproduction. It does not modify maze_explorer strategy, Phase92 staging logic, branch scoring, exploration order, centerline gate, directional readiness, fallback/terminal acceptance, Nav2, MPPI, controller, inflation, robot_radius, clearance_radius_m, or map threshold.

Primary command:

```bash
cd /home/hyh/Desktop/agent_playground/playground_hermes/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260522
PHASE93_MAX_GOALS=3 \
PHASE93_GOAL_TIMEOUT_SEC=45.0 \
PHASE93_TRIGGER_TIMEOUT_SEC=115 \
PHASE93_HOLD_AFTER_TRIGGER_SEC=0 \
PHASE93_CLEANUP_ON_EXIT=0 \
tools/run_phase93_two_step_staging_bounded_goal2_validation.sh
```

Visible launch mode is required:

```text
headless:=false
use_rviz:=true
```

Artifacts:

```text
log/phase93_two_step_staging_bounded_goal2_validation/
  phase93_two_step_staging_bounded_goal2_validation_goal_events.jsonl
  phase93_two_step_staging_bounded_goal2_validation_nav2_feedback.jsonl
  phase93_two_step_staging_bounded_goal2_validation_local_costmap_samples.jsonl
  phase93_two_step_staging_bounded_goal2_validation_raw_capture.json
  phase93_two_step_staging_bounded_goal2_validation_analysis.json
  phase93_two_step_staging_bounded_goal2_validation_minimal_field_summary.md
  phase93_two_step_staging_bounded_goal2_validation_SCENE_HELD_WAITING_FOR_USER_SCREENSHOT.txt
```

Stop condition:

The wrapper stops maze_explorer once it sees a Goal2-equivalent dispatch or Phase92 two-step staging context that has either:

- staging rejection;
- staging applied plus second-step dispatch;
- staging applied plus staging success/terminal evidence;
- terminal Goal2-equivalent outcome;
- bounded timeout window elapsed.

It intentionally leaves Gazebo/RViz visible by default for screenshots and manual observation. Set PHASE93_CLEANUP_ON_EXIT=1 only when you explicitly want the wrapper to clean up on exit. For this phase, if the scene is held, produce a minimal summary and screenshot suggestions rather than continuing blind algorithm repair.

Classification contract:

- TWO_STEP_STAGING_APPLIED_AND_SECOND_STEP_DISPATCHED
- TWO_STEP_STAGING_APPLIED_BUT_SECOND_STEP_FAILED
- TWO_STEP_STAGING_REJECTED
- TWO_STEP_STAGING_NOT_TRIGGERED
- TWO_STEP_STAGING_VALIDATION_INSUFFICIENT_EVIDENCE

Required Phase92 fields in /maze/goal_events dispatch rows:

- two_step_staging_plan
- staging_goal_pose
- staging_reason
- staging_executability_check
- staging_applied
- staging_reject_reason
- second_step_forward_goal
- branch_scoring_changed=false
- fallback_terminal_acceptance_used=false

Required trigger bundle checks:

- near_goal_lateral_residual
- single_step_forward_search_no_hard_safety_pass
- safety_floor_dominant_blocker
- execution_time_footprint_front_wedge_risk

Fresh evidence checks after corridor_alignment_staging success:

- fresh scan evidence timestamp
- fresh local costmap evidence timestamp
- fresh TF evidence timestamp
- second-step forward goal generated only after fresh evidence

Screenshot checklist if scene is held:

1. Dispatch moment: original/refined target, staging target if visible, robot pose and nearest wall.
2. Staging/near-goal moment: footprint and front wedge in RViz local costmap.
3. Second-step moment: selected second-step forward goal, corridor centerline direction, goal tolerance circle.
4. Timeout/recovery/final pose moment: local-cost high/lethal cells, front wedge, footprint, and recovery loop evidence.

Guardrails:

- No maze_explorer strategy changed.
- No Phase92 staging logic changed.
- No branch scoring changed.
- No exploration order changed.
- No centerline gate changed.
- No directional readiness changed.
- No fallback/terminal acceptance changed.
- No Nav2/MPPI/controller tuning.
- No inflation/robot_radius/clearance_radius_m/map threshold tuning.
- No autonomous exploration success claimed.
- No exit success claimed.
- Phase94 not entered.
