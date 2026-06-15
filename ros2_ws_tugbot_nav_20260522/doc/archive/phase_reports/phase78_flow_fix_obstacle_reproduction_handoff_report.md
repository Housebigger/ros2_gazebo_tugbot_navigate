# Phase78-flow-fix: Obstacle Reproduction Handoff Workflow

Status: `FLOW_FIX_DOCUMENTED_STATIC_VERIFIED_PENDING_USE`

Workspace: `/home/hyh/Desktop/agent_playground/playground_hermes/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260522`

## Purpose

Phase78-flow-fix corrects the development process created by Phase76/77.

Phase76/77 correctly required visual evidence before additional algorithm edits, but the overlay/report-heavy workflow created too much operator burden. Phase78-flow-fix replaces that with a lightweight handoff loop:

```text
automatic bounded reproduction -> user screenshot -> ChatGPT discussion -> Hermes executes confirmed plan
```

This is a workflow correction only. The algorithm repair phase not entered.

## Supersession decision

Phase77 overlay/report-heavy workflow is superseded.

Specifically superseded:

- requiring a long human observation report template.
- expecting the user to fill structured evidence fields.
- treating screenshot observation as a document-production task.
- stopping with a large report package instead of making Hermes reproduce and hold the problem scene.

Replacement:

- Hermes runs automatic bounded reproduction.
- Hermes runs the robot to the problem point and keeps Gazebo/RViz visible.
- The user captures screenshots and gives a brief initial judgment.
- The user and ChatGPT discuss the screenshot and decide a repair direction.
- Hermes executes the confirmed plan only in a later explicit phase.

No lengthy human observation report is required.

## New workflow document

Added:

- `doc/doc_proposal/obstacle_reproduction_handoff_workflow.md`

The document defines:

- trigger policy for `goal_timeout`, `FAILED_EXHAUSTED`, `no_candidate`, `local_cost_risk`, `recovery loop`, and `near-goal outside tolerance`.
- automatic bounded reproduction behavior.
- bounded reproduction runbook.
- minimal field summary.
- screenshot suggestions.
- ChatGPT discussion handoff.
- Hermes execution after confirmed plan.
- guardrails and stop condition.

## Bounded reproduction runbook design

The new runbook pattern is:

1. Start simulation in visible mode.
   - active Gazebo world
   - SLAM/Nav2/RViz as needed
   - `headless:=false`
   - `use_rviz:=true`
2. Trigger problem segment only.
   - bounded `max_goals`, for example `1` or `2`.
   - bounded `timeout_sec`, just long enough to reach the known problem segment.
   - optional known ingress/staging goal if prior artifacts require it.
3. Pause or hold at problem point.
   - stop when trigger appears.
   - cancel active navigation only if needed to keep the scene stable.
   - keep Gazebo/RViz observable for screenshots.
   - do not continue long exploration.
4. Print only minimal field summary.
   - trigger
   - run/artifact path
   - robot pose if available
   - target if available
   - key evidence one-liner
   - screenshot suggestions
5. Wait for user screenshot and initial judgment.
6. Discuss screenshot with ChatGPT.
7. Hermes executes the confirmed plan in the next explicit repair phase.

## Minimal user burden

The user is not asked to fill a form.

The user is only asked for:

- 1-4 screenshots.
- a short initial judgment, such as:
  - target too close to wall
  - front wedge into obstacle
  - local costmap blocks the robot
  - recovery loop dominates
  - unclear / need another angle

Hermes owns logs, launch commands, run IDs, summaries, and follow-up implementation.

## Screenshot suggestions

Use a small checklist only:

- Gazebo wide screenshot: robot, wall/corridor, target direction.
- RViz local costmap screenshot: robot footprint and front wedge.
- RViz goal tolerance screenshot: if near-goal outside tolerance is suspected.
- RViz/Gazebo recovery-loop screenshot: if robot oscillates, stops, or repeatedly clears costmaps.

## Phase77 update

Updated:

- `doc/doc_report/phase77_goal2_timeout_visual_root_cause_replay_first_application_report.md`

The report now contains a `Phase78-flow-fix supersession note` stating that the report-heavy handoff is superseded and future work should use:

- `doc/doc_proposal/obstacle_reproduction_handoff_workflow.md`

## Static test added

Added:

- `src/tugbot_maze/test/test_phase78_flow_fix_obstacle_reproduction_handoff_workflow.py`

The test verifies:

- handoff loop tokens exist.
- screenshot and ChatGPT discussion are required.
- no lengthy human report is required.
- bounded reproduction is documented.
- guardrails remain documented.
- Phase77 report contains the supersession note.
- Phase79 is not entered.

## Verification

RED proof before implementation:

```text
PYTHONDONTWRITEBYTECODE=1 pytest -q src/tugbot_maze/test/test_phase78_flow_fix_obstacle_reproduction_handoff_workflow.py
```

Observed expected failure before workflow/report existed:

```text
FFF [100%]
missing required Phase78-flow-fix file: .../doc/doc_proposal/obstacle_reproduction_handoff_workflow.md
missing required Phase78-flow-fix file: .../doc/doc_report/phase78_flow_fix_obstacle_reproduction_handoff_report.md
3 failed in 0.02s
```

Final verification executed after this report was saved:

```bash
PYTHONDONTWRITEBYTECODE=1 pytest -q src/tugbot_maze/test/test_phase78_flow_fix_obstacle_reproduction_handoff_workflow.py
```

Observed:

```text
...                                                                      [100%]
3 passed in 0.01s
```

Additional syntax and guard checks were executed after this verification section was patched:

```bash
PYTHONDONTWRITEBYTECODE=1 pytest -q src/tugbot_maze/test/test_phase78_flow_fix_obstacle_reproduction_handoff_workflow.py && \
  printf '\n--- syntax ---\n' && \
  PYTHONDONTWRITEBYTECODE=1 python3 -m py_compile src/tugbot_maze/test/test_phase78_flow_fix_obstacle_reproduction_handoff_workflow.py && \
  printf '\n--- strategy/config diff ---\n' && \
  git diff -- src/tugbot_maze/tugbot_maze/maze_explorer.py src/tugbot_navigation/config src/tugbot_bringup/launch src/tugbot_maze/tugbot_maze | cat
```

Observed:

```text
...                                                                      [100%]
3 passed in 0.01s

--- syntax ---

--- strategy/config diff ---
```

Interpretation: focused static test passed, syntax check passed, and strategy/config diff guard was empty.

Cleanup verification after final verification run:

```text
removed_count=1
remaining_count=0
```

Interpretation: the verification run created one transient `__pycache__` directory, it was removed, and no `__pycache__` directories remained under `src/` or `tools/`.

## Guardrails

- No navigation strategy changed.
- No branch scoring changed.
- No centerline gate changed.
- No directional readiness override changed.
- No fallback/terminal acceptance changed.
- No Nav2/MPPI/controller tuning.
- No inflation/robot_radius/clearance_radius_m/map threshold tuning.
- No autonomous exploration success claimed.
- No exit success claimed.
- No long autonomous exploration run performed.

## Stop condition

Phase78-flow-fix stops after workflow documentation, Phase77 supersession note, and static tests are verified.

The algorithm repair phase not entered.

Phase79 not entered.
