# Phase76: Obstacle-Triggered Visual Root-Cause Workflow

Status: completed / stopped at Phase76.

Classification: workflow/template only.

No navigation strategy changed.

## Purpose

Phase75 established that Explorer Goal2 now dispatches, but the timeout is dominated by execution-time local-cost / footprint / front-wedge recovery-loop evidence rather than target-cell wall risk.

Phase76 implements the new process rule: when exploration progress hits an obstacle that is temporarily unresolved, generate and stage a bounded RViz/Gazebo visual root-cause replay for user observation before continuing algorithm edits.

## Scope and guardrails

This phase adds governance, templates, and static tests only.

- 不调 Nav2/MPPI/controller.
- 不调 inflation/robot_radius/clearance_radius_m/map threshold.
- 不改 branch scoring.
- 不改 centerline gate.
- 不改 directional readiness override.
- 不改 fallback/terminal acceptance.
- 不声明 autonomous exploration success.
- 不声明 exit success.
- 不跑长时间实验.

## Files added

- Workflow proposal: `doc/doc_proposal/phase76_obstacle_triggered_visual_root_cause_workflow.md`
- Replay/marker/checklist template: `tools/phase76_visual_root_cause_replay_template.py`
- Shell runbook template: `tools/run_phase76_visual_root_cause_replay_template.sh`
- Static contract tests: `src/tugbot_maze/test/test_phase76_obstacle_triggered_visual_root_cause_workflow.py`
- Final report: `doc/doc_report/phase76_obstacle_triggered_visual_root_cause_workflow_report.md`

## Trigger coverage

The workflow explicitly covers these obstacle-trigger conditions:

- `goal_timeout`
- `FAILED_EXHAUSTED`
- `no_candidate`
- `local_cost_risk`
- `recovery_count>0`
- `near-goal outside tolerance`
- `re-dispatch readiness blocked`

Policy: once any of these appears and the obstacle is not yet visually understood, create the visual replay package before blind algorithm tuning.

## Marker overlay coverage

The template generates ROS-independent marker specs for:

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

The markers are diagnostic overlays only. They do not implement target projection, centerline behavior changes, branch scoring changes, or fallback/terminal acceptance.

## Screenshot checklist coverage

The generated checklist requires these screenshot moments:

- `dispatch`
- `near_goal`
- `recovery`
- `timeout`
- `final_pose`

Each checklist item records required overlays, capture filename hint, and the visual question to answer.

## Template behavior

`tools/phase76_visual_root_cause_replay_template.py` accepts a single event JSON object and writes:

- `<run_id>_visual_root_cause_plan.json`
- `<run_id>_marker_specs.json`
- `<run_id>_screenshot_checklist.md`
- `<run_id>_human_observation_report_template.md`

The plan includes a staged RViz/Gazebo replay command using `/phase76/visual_root_cause_markers` and preserves guardrail fields:

- `navigation_strategy_changed: false`
- `autonomous_success_claimed: false`
- `exit_success_claimed: false`
- no long-running experiment requirement

The shell runbook template prints `PHASE76_READY_FOR_HUMAN_OBSERVATION` after generating the package.

## Verification

RED proof before implementation:

```bash
pytest -q src/tugbot_maze/test/test_phase76_obstacle_triggered_visual_root_cause_workflow.py
```

Result before adding workflow/template files: `5 failed` because the Phase76 workflow document, Python template, shell runbook, and final report did not exist yet.

Focused static contract test after implementation:

```bash
PYTHONDONTWRITEBYTECODE=1 pytest -q src/tugbot_maze/test/test_phase76_obstacle_triggered_visual_root_cause_workflow.py
```

Result: `5 passed in 0.03s`.

Python syntax check:

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -m py_compile tools/phase76_visual_root_cause_replay_template.py src/tugbot_maze/test/test_phase76_obstacle_triggered_visual_root_cause_workflow.py
```

Result: passed with exit code 0.

Template CLI smoke:

```bash
python3 tools/phase76_visual_root_cause_replay_template.py --event-json <goal_timeout_event_json> --artifact-dir /tmp/phase76_cli_smoke --run-id phase76_cli_smoke
```

Result: generated plan/checklist/marker/report paths and selected trigger `goal_timeout` with matched evidence `goal_timeout`, `local_cost_risk`, `recovery_count>0`, and `near-goal outside tolerance`.

Shell runbook smoke:

```bash
PHASE76_EVENT_JSON=/tmp/phase76_runbook_event.json PHASE76_RUN_ID=phase76_runbook_smoke PHASE76_ARTIFACT_DIR=/tmp/phase76_runbook_smoke tools/run_phase76_visual_root_cause_replay_template.sh
```

Result: generated the Phase76 package and printed `PHASE76_READY_FOR_HUMAN_OBSERVATION`; selected trigger `FAILED_EXHAUSTED` with matched evidence `FAILED_EXHAUSTED`, `no_candidate`, and `re-dispatch readiness blocked`.

Navigation strategy guard check:

```bash
git diff -- src/tugbot_maze/tugbot_maze/maze_explorer.py src/tugbot_navigation/config src/tugbot_bringup/launch src/tugbot_maze/tugbot_maze | cat
```

Result: empty diff. Phase76 changed only docs/tools/tests/report.

## Final Phase76 conclusion

Phase76 establishes a repeatable obstacle-triggered visual root-cause workflow. Future phases should use it when `goal_timeout`, `FAILED_EXHAUSTED`, `no_candidate`, `local_cost_risk`, `recovery_count>0`, `near-goal outside tolerance`, or `re-dispatch readiness blocked` appears and the cause is not yet visually understood.

This phase does not claim autonomous exploration success and does not claim exit success.

Phase77 not entered.
