# Phase77: Goal2 timeout Phase76 visual root-cause replay first application

Status: `REPLAY_ARTIFACTS_GENERATED_PENDING_HUMAN_RVIZ_GAZEBO_OBSERVATION`

Workspace: `/home/hyh/Desktop/agent_playground/playground_hermes/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260522`

## Purpose

Phase77 applies the Phase76 obstacle-triggered visual root-cause workflow to the real Phase75 Goal2 timeout case for the first time.

This phase is evidence packaging and human-observation staging only. It does not change navigation behavior and does not claim autonomous exploration success or exit success.

## Required documents read

Read before generating artifacts:

- `doc/doc_proposal/phase27_alt_near_exit_fallback_design.md`
- `doc/doc_proposal/phase27_alt_r5_footprint_terminal_acceptance_design.md`
- `doc/doc_proposal/phase76_obstacle_triggered_visual_root_cause_workflow.md`
- `doc/doc_report/phase75_goal2_timeout_after_directional_redispatch_diagnosis_report.md`
- `doc/doc_report/phase76_obstacle_triggered_visual_root_cause_workflow_report.md`

The top-level proposal lessons preserved here are: do not silently change terminal/fallback behavior, do not use runtime maze-image answers, keep visual/root-cause phases bounded, and preserve explicit stop conditions.

## Source artifact discovery

Phase77 found the real Phase75/Phase74 Goal2 timeout artifact chain:

- Phase75 diagnosis summary:
  - `log/phase75_goal2_timeout_after_directional_redispatch_diagnosis/phase75_goal2_timeout_after_directional_redispatch_diagnosis.json`
- Phase75 source replay directory:
  - `log/phase74_directional_local_costmap_readiness_gate_validation/replay_02`
- Required replay/event inputs present:
  - `phase74_directional_local_costmap_readiness_gate_validation_replay_02_goal_events.jsonl`
  - `phase74_controller_dynamics.jsonl`
  - `phase74_local_costmap_samples.jsonl`
  - `phase74_nav2_feedback.jsonl`
  - `phase74_global_plan_samples.jsonl`
  - `phase74_runtime_timeline.jsonl`

Input assessment:

- `INSUFFICIENT_VISUAL_REPLAY_INPUT`: not emitted / not required.
- Required machine-readable artifacts were present for Phase76 template generation.
- Optional direct visual fields still missing and must be supplied by human RViz/Gazebo observation:
  - `direct_nearest_wall_point`
  - `preexisting RViz/Gazebo screenshots`

Important interpretation note: because the source artifact does not contain a direct nearest-wall point, the generated `nearest_wall_marker` should be treated as an observation prompt/placeholder anchor, not as proof of nearest-wall geometry. The human observer must verify the actual wall relation in RViz/Gazebo.

## Generated Phase77 event input

Saved event JSON passed into the Phase76 template:

- `log/phase77_goal2_visual_root_cause_replay/phase77_goal2_timeout_visual_root_cause_event.json`

Key real evidence carried into the event:

- `classification`: `GOAL2_TIMEOUT_LOCAL_COST_RECOVERY_LOOP`
- `result_reason`: `goal_timeout`
- `goal_sequence`: `2`
- `dispatch_pose`: `[2.307321326591473, 0.02930773008823818, 0.023884391949891]`
- `target` / `dispatch_target`: `[2.058503376259287, 1.0236490342193865]`
- `original_target`: `[2.058503376259287, 1.0236490342193865]`
- `refined_target`: `[2.058503376259287, 1.0236490342193865]`
- `final_pose` / `terminal_pose`: `[2.426872326194781, 1.0327763735179467, 1.6075951571994673]`
- `xy_goal_tolerance_m`: `0.25`
- `final_xy_error_m`: `0.36848200987191976`
- `near_goal_outside_tolerance`: `true`
- `number_of_recoveries`: `3`
- `timeout_front_wedge_cost_max`: `99`
- `timeout_front_wedge_clearance_m`: `0.048788617741889216`
- `timeout_footprint_cost_max`: `99`
- `timeout_footprint_lethal_cell_count`: `48`
- `timeout_local_cost_sample_age_sec`: `0.009`
- `timeout_robot_in_local_costmap_bounds`: `true`
- windowed odom samples available for Goal2 dispatch-to-timeout: `1272`
- exported trajectory points in replay input: `80`
- windowed Goal2 local-cost samples: `76`

## Phase76 template command used

```bash
PYTHONDONTWRITEBYTECODE=1 python3 tools/phase76_visual_root_cause_replay_template.py \
  --event-json log/phase77_goal2_visual_root_cause_replay/phase77_goal2_timeout_visual_root_cause_event.json \
  --artifact-dir log/phase77_goal2_visual_root_cause_replay \
  --run-id phase77_goal2_timeout_visual_root_cause_replay
```

Template output trigger:

```text
selected: goal_timeout
matched: goal_timeout, local_cost_risk, recovery_count>0, near-goal outside tolerance
requires_visual_replay: true
```

## Generated Phase77 replay artifacts

Saved under:

- `log/phase77_goal2_visual_root_cause_replay/`

Files:

- `phase77_goal2_timeout_visual_root_cause_event.json`
- `phase77_goal2_timeout_visual_root_cause_replay_visual_root_cause_plan.json`
- `phase77_goal2_timeout_visual_root_cause_replay_marker_specs.json`
- `phase77_goal2_timeout_visual_root_cause_replay_screenshot_checklist.md`
- `phase77_goal2_timeout_visual_root_cause_replay_human_observation_report_template.md`

Generated visual replay command for a later bounded human-observation run:

```bash
ros2 launch tugbot_bringup phase76_visual_root_cause_overlay.launch.py phase76_plan:=log/phase77_goal2_visual_root_cause_replay/phase77_goal2_timeout_visual_root_cause_replay_visual_root_cause_plan.json headless:=false use_rviz:=true marker_topic:=/phase76/visual_root_cause_markers
```

This command is staged only. Phase77 did not start a long autonomous exploration run.

## Marker overlay coverage

Generated marker specs cover the Phase76 contract:

- `dispatch_target_marker`
- `original_target_marker`
- `refined_target_marker`
- `robot_trajectory_marker`
- `terminal_pose_marker`
- `footprint_marker`
- `front_wedge_marker`
- `local_cost_high_lethal_marker`
- `nearest_wall_marker`
- `corridor_centerline_marker`
- `goal_tolerance_circle_marker`

Use these as visual aids only. They are not branch scoring, target projection, centerline-gate behavior, fallback acceptance, or terminal acceptance.

## Human RViz/Gazebo screenshot checklist

Use the generated checklist:

- `log/phase77_goal2_visual_root_cause_replay/phase77_goal2_timeout_visual_root_cause_replay_screenshot_checklist.md`

Required screenshots:

1. `dispatch`
   - filename hint: `phase77_goal2_timeout_visual_root_cause_replay_dispatch.png`
   - overlays: `dispatch_target_marker`, `original_target_marker`, `refined_target_marker`
   - observe: target relationship, robot pose, local costmap, corridor context.
2. `near_goal`
   - filename hint: `phase77_goal2_timeout_visual_root_cause_replay_near_goal.png`
   - overlays: `goal_tolerance_circle_marker`, `footprint_marker`, `front_wedge_marker`
   - observe: whether final approach is outside the goal tolerance circle and which side is constrained.
3. `recovery`
   - filename hint: `phase77_goal2_timeout_visual_root_cause_replay_recovery.png`
   - overlays: `robot_trajectory_marker`, `local_cost_high_lethal_marker`, `nearest_wall_marker`
   - observe: first progress-failure/recovery loop, high/lethal local cost, nearest wall relation.
4. `timeout`
   - filename hint: `phase77_goal2_timeout_visual_root_cause_replay_timeout.png`
   - overlays: `terminal_pose_marker`, `footprint_marker`, `front_wedge_marker`
   - observe: terminal footprint and front wedge at timeout/cancel.
5. `final_pose`
   - filename hint: `phase77_goal2_timeout_visual_root_cause_replay_final_pose.png`
   - overlays: `terminal_pose_marker`, `goal_tolerance_circle_marker`, `corridor_centerline_marker`
   - observe: final pose relative to target tolerance circle, corridor centerline, and wall/cost relation.

Do not mark any screenshot set as autonomous exploration success or exit success.

## Human observation classification menu

The report template asks the operator to classify conservatively using these labels:

- `TARGET_POSE_GEOMETRY_BAD`
  - Use only if RViz/Gazebo shows the Goal2 target/original/refined target itself is geometrically bad, unsafe, or placed in a wall-side squeeze despite the offline target-cost evidence.
- `LOCAL_COSTMAP_EXECUTION_BLOCKED`
  - Use if local costmap high/lethal regions at execution time visibly block the robot at/near terminal pose.
- `FOOTPRINT_FRONT_WEDGE_COLLISION_RISK`
  - Use if footprint or front wedge visibly overlaps or points into high/lethal local-cost/wall-side space during approach/recovery.
- `CORRIDOR_CENTERLINE_ALIGNMENT_BAD`
  - Use if final trajectory/pose is visibly misaligned with corridor centerline enough to explain the wedge/footprint squeeze.
- `NAV2_CONTROLLER_RECOVERY_LOOP_DOMINANT`
  - Use if the visual replay shows progress failure/recovery oscillation dominates the final interval; this aligns with recoveries reaching `3`, failed-progress logs, and last-10s near-zero command evidence.
- `INSUFFICIENT_VISUAL_EVIDENCE`
  - Use if the generated overlay/screenshot set still does not show enough wall/local-cost/footprint relation. This is allowed and should not be replaced by guesses.

Initial machine evidence before human observation remains conservative:

- Strongly points to `LOCAL_COSTMAP_EXECUTION_BLOCKED`, `FOOTPRINT_FRONT_WEDGE_COLLISION_RISK`, and `NAV2_CONTROLLER_RECOVERY_LOOP_DOMINANT`.
- Does not by itself prove the visual wall geometry; human RViz/Gazebo screenshots are still required.
- Does not prove `TARGET_POSE_GEOMETRY_BAD`; Phase75 target evidence was low risk (`dispatch_target_local_cost=0`, target clearance about `0.851m`).

## Verification

RED proof:

```text
PYTHONDONTWRITEBYTECODE=1 pytest -q src/tugbot_maze/test/test_phase77_goal2_visual_root_cause_replay_application.py
```

Before generated replay plan/report existed, the new Phase77 contract test failed as expected:

```text
2 failed, 1 passed
missing generated Phase77 replay artifact
Phase77 final report must be written under doc/doc_report/
```

GREEN/guard verification executed after this report was saved:

```bash
PYTHONDONTWRITEBYTECODE=1 pytest -q src/tugbot_maze/test/test_phase77_goal2_visual_root_cause_replay_application.py && \
  printf '\n--- syntax ---\n' && \
  PYTHONDONTWRITEBYTECODE=1 python3 -m py_compile tools/phase76_visual_root_cause_replay_template.py src/tugbot_maze/test/test_phase77_goal2_visual_root_cause_replay_application.py && \
  printf '\n--- strategy/config diff ---\n' && \
  git diff -- src/tugbot_maze/tugbot_maze/maze_explorer.py src/tugbot_navigation/config src/tugbot_bringup/launch src/tugbot_maze/tugbot_maze | cat
```

Observed output:

```text
...                                                                      [100%]
3 passed in 0.02s

--- syntax ---

--- strategy/config diff ---
```

Interpretation: Phase77 contract test passed, syntax check passed, and strategy/config diff guard was empty.

## Guardrails confirmed by scope

- No navigation strategy changed.
- No branch scoring changed.
- No centerline gate changed.
- No directional readiness override changed.
- No fallback/terminal acceptance changed.
- No Nav2/MPPI/controller/inflation/robot_radius/clearance_radius_m/map threshold tuning.
- No autonomous exploration success claimed.
- No exit success claimed.
- No long automatic exploration run performed.

## Phase78-flow-fix supersession note

Phase78-flow-fix supersedes the report-heavy handoff used by this Phase77 report.

The Phase77 artifacts remain historical evidence, but the future workflow no longer asks the user to complete a long human observation report. The report-heavy handoff is superseded by the lightweight loop:

```text
automatic bounded reproduction -> user screenshot -> ChatGPT discussion -> Hermes executes confirmed plan
```

For future obstacle-triggered investigation, use doc/doc_proposal/obstacle_reproduction_handoff_workflow.md.

## Stop condition

Phase77 stops here with replay artifacts and checklist ready for human RViz/Gazebo observation.

Phase78 not entered at the time of Phase77 completion; later Phase78-flow-fix replaced the handoff workflow without changing Phase77 navigation evidence.
