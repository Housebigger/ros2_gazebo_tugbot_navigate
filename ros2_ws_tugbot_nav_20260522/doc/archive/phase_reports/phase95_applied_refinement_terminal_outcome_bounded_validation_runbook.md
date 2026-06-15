# Phase95 Applied refinement terminal outcome bounded validation runbook

Status: RUNBOOK_ONLY_FOR_PHASE95_VALIDATION

Goal: reproduce the Phase94 applied Phase88 single-step refinement scene and extend the bounded visible window only until the Goal2-equivalent terminal outcome is captured: succeeded / timeout / cancel / failure. This runbook is validation-only.

Command shape:

```bash
cd /home/hyh/Desktop/agent_playground/playground_hermes/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260522
PHASE95_MAX_GOALS=3 \
PHASE95_GOAL_TIMEOUT_SEC=180.0 \
PHASE95_TERMINAL_WINDOW_SEC=260 \
PHASE95_RUNTIME_RECORD_TIMEOUT_SEC=320 \
PHASE95_HOLD_AFTER_TRIGGER_SEC=0 \
PHASE95_CLEANUP_ON_EXIT=0 \
PYTHONDONTWRITEBYTECODE=1 \
tools/run_phase95_applied_refinement_terminal_outcome_bounded_validation.sh
```

Key environment knobs:

- `PHASE95_TERMINAL_WINDOW_SEC`: finite cap for the Goal2-equivalent terminal outcome window. This is intentionally longer than Phase93/94 but still bounded.
- `PHASE95_GOAL_TIMEOUT_SEC`: Nav2/maze_explorer goal timeout for the applied refinement Goal2-equivalent segment.
- `PHASE95_MAX_GOALS`: must be 2 or 3; the wrapper rejects broader exploration.
- `PHASE95_CLEANUP_ON_EXIT=0`: hold Gazebo/RViz/SLAM/Nav2 after terminal timeout or failure so screenshots can be taken.

Expected artifacts:

- `log/phase95_applied_refinement_terminal_outcome_bounded_validation/phase95_applied_refinement_terminal_outcome_bounded_validation_goal_events.jsonl`
- `log/phase95_applied_refinement_terminal_outcome_bounded_validation/phase95_applied_refinement_terminal_outcome_bounded_validation_nav2_feedback.jsonl`
- `log/phase95_applied_refinement_terminal_outcome_bounded_validation/phase95_applied_refinement_terminal_outcome_bounded_validation_local_costmap_samples.jsonl`
- `log/phase95_applied_refinement_terminal_outcome_bounded_validation/phase95_applied_refinement_terminal_outcome_bounded_validation_raw_capture.json`
- `log/phase95_applied_refinement_terminal_outcome_bounded_validation/phase95_applied_refinement_terminal_outcome_bounded_validation_terminal_detected.json`
- `log/phase95_applied_refinement_terminal_outcome_bounded_validation/phase95_applied_refinement_terminal_outcome_bounded_validation_analysis.json`
- `log/phase95_applied_refinement_terminal_outcome_bounded_validation/phase95_applied_refinement_terminal_outcome_bounded_validation_minimal_field_summary.md`

Goal2-equivalent terminal outcome detection:

- Match the old Goal2-equivalent target neighborhood around `[2.057855221699651, 1.0261005743935105]`.
- Require `refinement_applied=true`, `selected_candidate_target`, `selected_candidate_yaw`, and `hard_safety_pass_candidate_count` from the Phase88 refinement payload.
- Stop autonomous progress after the terminal event: `success`, `timeout`, `failure`, `cancel`, or equivalent result reason.

Screenshot suggestions if terminal timeout/failure is observed:

1. Gazebo wide view: robot, Goal2-equivalent corridor, nearest wall, and target direction.
2. RViz local costmap: terminal robot footprint and front wedge.
3. RViz goal/tolerance: selected candidate target and tolerance relation.
4. Recovery/timeout pose: local-cost obstruction or recovery loop if visible.

Guardrails:

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
- Phase96 not entered.
