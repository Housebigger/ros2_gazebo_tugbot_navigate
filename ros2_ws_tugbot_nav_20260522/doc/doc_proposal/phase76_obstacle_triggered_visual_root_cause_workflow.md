# Phase76: Obstacle-Triggered Visual Root-Cause Workflow

Status: proposal/workflow template only.

Purpose: after Phase75 established that Explorer Goal2 dispatches but times out because of execution-time local-cost / footprint / front-wedge recovery-loop evidence (`GOAL2_TIMEOUT_LOCAL_COST_RECOVERY_LOOP`), Phase76 freezes the process rule for future exploration work:

When the algorithm meets a temporary obstacle that is not yet understood, start a bounded RViz/Gazebo visual root-cause replay and ask the user to observe the physical cause before making more algorithm changes.

This is a process-governance phase. It adds workflow documentation and reusable script templates. It does not change navigation strategy.

## Scope guardrails

- 不调 Nav2/MPPI/controller.
- 不调 inflation/robot_radius/clearance_radius_m/map threshold.
- 不改 branch scoring.
- 不改 centerline gate.
- 不改 directional readiness override.
- 不改 fallback/terminal acceptance.
- 不声明 autonomous exploration success.
- 不声明 exit success.
- 不跑长时间实验.

## Trigger policy

Any one of the following obstacle/deadlock symptoms should trigger the visual-root-cause workflow before further algorithm edits:

1. `goal_timeout`
   - A dispatched goal times out or is canceled after timeout.
   - Example Phase75 context: Goal2 reached near the target, then stalled outside XY tolerance with recovery/local-cost evidence.
2. `FAILED_EXHAUSTED`
   - The explorer reaches terminal failure/exhausted mode before a clear visual explanation exists.
3. `no_candidate`
   - Topology sampling or filtering yields no candidate when the human-visible corridor appears passable.
4. `local_cost_risk`
   - Runtime diagnostics show high/lethal local-cost evidence near the robot footprint, target, path, or front wedge.
5. `recovery_count>0`
   - Nav2 feedback/logs show recovery behavior, progress failure, costmap clearing, or controller abort patterns.
6. `near-goal outside tolerance`
   - The terminal pose is visually near the dispatch target but outside the goal tolerance circle.
7. `re-dispatch readiness blocked`
   - After a success or timeout, the robot is blocked by map/scan/TF/local-cost/Nav2 readiness before the next dispatch.

Trigger response:

- Generate a visual replay package using `tools/phase76_visual_root_cause_replay_template.py`.
- Stage the generated replay command and marker specs.
- Run a visible RViz/Gazebo observation only when needed and keep it bounded.
- Ask the user to observe root cause from screenshots or live view.
- Do not tune or modify strategy until visual evidence is recorded.

## Replay package outputs

The template creates:

- `*_visual_root_cause_plan.json`: consolidated trigger, marker, command, and guardrail plan.
- `*_marker_specs.json`: ROS-independent marker overlay specification.
- `*_screenshot_checklist.md`: screenshot checklist for required moments.
- `*_human_observation_report_template.md`: 人工观察报告模板.
- `replay command`: a staged RViz/Gazebo command template, not an autonomous success run.

Example command shape:

```bash
python3 tools/phase76_visual_root_cause_replay_template.py \
  --event-json log/<run>/<trigger_event>.json \
  --artifact-dir log/phase76_visual_root_cause/<case_id> \
  --run-id <case_id>
```

The generated plan contains a `replay command` such as:

```bash
ros2 launch tugbot_bringup phase76_visual_root_cause_overlay.launch.py phase76_plan:=log/phase76_visual_root_cause/<case_id>/<case_id>_visual_root_cause_plan.json headless:=false use_rviz:=true marker_topic:=/phase76/visual_root_cause_markers
```

The shell runbook template `tools/run_phase76_visual_root_cause_replay_template.sh` wraps the same generation step and prints `PHASE76_READY_FOR_HUMAN_OBSERVATION` when the package is ready.

## Marker overlay contract

Every replay package must include marker specs for these visual anchors:

- `dispatch_target_marker`: the actual target sent to Nav2.
- `original_target_marker`: the original target before any recorded refinement.
- `refined_target_marker`: the refined target if already present in diagnostics; this is display-only and does not change the centerline gate.
- `robot_trajectory_marker`: odom trajectory from dispatch to terminal pose.
- `terminal_pose_marker`: timeout/exhausted/final pose arrow.
- `footprint_marker`: robot footprint envelope at terminal pose.
- `front_wedge_marker`: front wedge at terminal pose or recovery pose.
- `local_cost_high_lethal_marker`: representative high/lethal local-cost point.
- `nearest_wall_marker`: nearest wall or wall-side clearance point.
- `corridor_centerline_marker`: corridor centerline reference for visual diagnosis only.
- `goal_tolerance_circle_marker`: goal tolerance circle around the dispatch target.

These markers are observational aids only. They are not target projection, branch scoring, clearance tuning, or fallback acceptance.

## Required screenshot moments

The checklist must cover at least these moments:

1. `dispatch`
   - Include dispatch target, original/refined target, robot pose, local costmap, and corridor context.
2. `near_goal`
   - Include goal tolerance circle, footprint, front wedge, and whether the robot is near-goal outside tolerance.
3. `recovery`
   - Include recovery/progress-failure moment, trajectory, high/lethal local cost, nearest wall, and front wedge.
4. `timeout`
   - Include terminal pose, footprint, front wedge, and timeout/exhausted state.
5. `final_pose`
   - Include final pose, corridor centerline, goal tolerance circle, and wall/cost relation.

## 人工观察报告模板 requirements

Each human observation report should answer:

- Which trigger fired: `goal_timeout`, `FAILED_EXHAUSTED`, `no_candidate`, `local_cost_risk`, `recovery_count>0`, `near-goal outside tolerance`, or `re-dispatch readiness blocked`?
- What did the user see at dispatch?
- Did the terminal footprint overlap or visually squeeze against walls/costmap high/lethal areas?
- Did the front wedge point into a blocked/high-cost region during recovery?
- Was the target itself clear while the robot pose/front wedge was blocked?
- Was the final pose near-goal outside tolerance, or was it a true far-away failure?
- Did readiness blockage happen before any new candidate/goal dispatch?
- Is the conclusion visual evidence, or still inconclusive?

Allowed conservative conclusion labels:

- `VISUAL_EVIDENCE_PENDING_HUMAN_OBSERVATION`
- `VISUAL_LOCAL_COST_FOOTPRINT_FRONT_WEDGE_LOOP_OBSERVED`
- `VISUAL_NEAR_GOAL_OUTSIDE_TOLERANCE_OBSERVED`
- `VISUAL_NO_CANDIDATE_TOPOLOGY_BLOCK_OBSERVED`
- `VISUAL_REDISPATCH_READINESS_BLOCK_OBSERVED`
- `VISUAL_EVIDENCE_INCONCLUSIVE`

None of these labels is autonomous exploration success or exit success.

## Operator workflow

1. Detect trigger from `/maze/goal_events`, explorer state, Nav2 feedback/logs, local-cost samples, or a prior analyzer.
2. Save one JSON object representing the trigger event.
3. Generate replay package:

```bash
python3 tools/phase76_visual_root_cause_replay_template.py --event-json <event.json> --artifact-dir <artifact_dir> --run-id <case_id>
```

4. Review generated `*_visual_root_cause_plan.json` and `*_screenshot_checklist.md`.
5. If visual run is needed, use the generated replay command or the shell runbook template.
6. Capture screenshots for `dispatch`, `near_goal`, `recovery`, `timeout`, and `final_pose`.
7. Fill `*_human_observation_report_template.md` with user-visible observations.
8. Stop and classify conservatively. Do not proceed to algorithm changes in the same phase unless the user explicitly opens the next phase.

## Acceptance criteria for Phase76

- The workflow document exists under `doc/doc_proposal`.
- A final Phase76 report exists under `doc/doc_report`.
- A reusable Python template exists under `tools` and can generate replay plan, marker specs, screenshot checklist, and human report template.
- A shell runbook template exists under `tools` and remains bounded/visual-only.
- Static tests verify all trigger, marker, screenshot, and guardrail tokens.
- No navigation strategy file is changed.

## Stop condition

Phase76 stops after the workflow/template/report are verified. Phase77 not entered.
