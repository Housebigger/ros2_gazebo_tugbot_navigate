# Obstacle Reproduction Handoff Workflow

Status: Phase78-flow-fix replacement workflow.

Purpose: replace the Phase76/77 overlay/report-heavy process with a lightweight closed loop:

```text
automatic bounded reproduction -> user screenshot -> ChatGPT discussion -> Hermes executes the confirmed plan
```

This is a process fix only. It is not an algorithm repair phase.

## Why this replaces the old flow

Phase76/77 correctly established that unresolved navigation obstacles should be visually reproduced before algorithm edits. However, the generated replay package and human-report template created too much documentation burden for the operator.

New rule:

- Hermes owns reproduction setup, logs, launch/runbook details, and minimal field summary.
- The user only captures one or a few screenshots and gives an initial judgment.
- The screenshot and user judgment are discussed with ChatGPT to choose a repair direction.
- Hermes executes the confirmed plan in a later explicit algorithm phase.
- no lengthy human report is requested.

## Trigger policy

Use this handoff workflow when exploration hits any unresolved obstacle signal:

- `goal_timeout`
- `FAILED_EXHAUSTED`
- `no_candidate`
- `local_cost_risk`
- `recovery loop`
- `recovery_count>0`
- `near-goal outside tolerance`
- `re-dispatch readiness blocked`

Do not continue blind algorithm changes after these triggers. First reproduce the scene in bounded visible simulation.

## Lightweight handoff contract

Hermes should produce only:

1. automatic bounded reproduction command/runbook.
2. minimal field summary.
3. screenshot suggestions.
4. current guardrails and stop condition.

The user should provide only:

1. user screenshot from Gazebo/RViz.
2. initial judgment in a few words, for example:
   - target looks too close to wall
   - front wedge hits inflated obstacle
   - robot is angled badly near the corridor
   - local costmap blocks motion
   - recovery loop dominates
   - unclear from screenshot

The follow-up loop is:

1. Hermes stages and starts bounded reproduction.
2. The robot should run to the problem point.
3. Hermes should keep Gazebo/RViz observable at the problem scene.
4. The user sends a screenshot and initial judgment to ChatGPT discussion.
5. ChatGPT and the user discuss options and choose a repair direction.
6. Hermes executes the confirmed plan only after that plan is explicitly accepted in a later phase.

## Bounded reproduction runbook

This bounded reproduction runbook is a template. It is intentionally conservative and should be adapted to the specific obstacle artifact/run ID.

### Inputs

- source artifact directory or wrapper run ID.
- trigger type: `goal_timeout`, `FAILED_EXHAUSTED`, `no_candidate`, `local_cost_risk`, `recovery loop`, or `near-goal outside tolerance`.
- maximum goals: `max_goals` small enough to reach only the problem segment, usually `1`, `2`, or the known failing goal sequence.
- runtime cap: `timeout_sec` bounded to the shortest known interval that reaches the problem point.
- visible mode: `headless:=false`, `use_rviz:=true`.
- optional replay/staging goal if the previous phase already identified an ingress or pre-problem waypoint.

### Steps

1. start simulation in visible mode.
   - Launch active Gazebo world + SLAM + Nav2 + RViz.
   - Keep the same world/map/truth convention as the source artifact.
   - Do not start unrelated long-running exploration.
2. trigger problem segment.
   - Start `maze_explorer` only with bounded settings such as `max_goals:=1` or `max_goals:=2`.
   - If a prior ingress waypoint is required, send only that bounded setup goal first.
   - Stop as soon as the target trigger appears: timeout, failed exhausted, no candidate, local-cost risk, recovery loop, or near-goal outside tolerance.
3. pause or hold at problem point.
   - Cancel active navigation if needed after the trigger is visible.
   - Do not reset Gazebo or clear the scene.
   - Keep Gazebo/RViz observable for the user.
   - Gazebo/RViz remains open for observation until the user captures screenshots or asks for cleanup.
4. output a minimal field summary.
   - trigger
   - run ID / artifact directory
   - robot pose if available
   - goal/target if available
   - recovery count if available
   - local-cost/front-wedge/footprint headline if available
   - screenshot suggestions
5. stop operator burden there.
   - Do not ask the user to complete a form.
   - Do not require a long classification writeup.
   - Do not ask for a structured evidence table.

### Example bounded command shape

```bash
# Example only; adapt launch arguments to the current phase wrapper.
ros2 launch tugbot_bringup tugbot_maze_explore.launch.py \
  headless:=false \
  use_rviz:=true \
  max_goals:=2 \
  timeout_sec:=75 \
  near_exit_fallback_enabled:=false
```

If the trigger occurs, the reproduction wrapper should hold the scene rather than continue no long exploration.

## Minimal field summary template

```text
Trigger: <goal_timeout | FAILED_EXHAUSTED | no_candidate | local_cost_risk | recovery loop | near-goal outside tolerance>
Run/artifacts: <path>
Robot pose: <x,y,yaw or unknown>
Target: <x,y or unknown>
Observed stop reason: <short text>
Key evidence: <one-line local cost / recovery / no-candidate / tolerance note>
Screenshot suggestions: <2-4 bullets>
Guardrails: no strategy/config tuning, no success claim
```

## Screenshot suggestions

Prefer 1-4 screenshots, not a report:

- Gazebo wide view showing robot, nearest wall/corridor, and target direction.
- RViz local costmap view around robot footprint and front wedge.
- RViz goal/target tolerance view if the issue is near-goal outside tolerance.
- RViz/Gazebo recovery-loop view if the robot oscillates, stops, or repeatedly clears costmaps.

The user screenshot plus initial judgment is enough to start ChatGPT discussion. If the evidence is unclear, the correct conclusion is simply `needs another bounded reproduction angle`.

## Guardrails

- do not tune Nav2/MPPI/controller.
- do not change exploration strategy.
- do not modify branch scoring.
- do not modify centerline gate.
- do not modify directional readiness override.
- do not modify fallback/terminal acceptance.
- do not tune inflation, robot radius, clearance radius, or map threshold.
- do not claim autonomous success.
- do not claim exit success.
- do not run no long exploration.

## Output classification

Allowed lightweight status values:

- `REPRODUCTION_STAGED_WAITING_FOR_SCREENSHOT`
- `SCENE_HELD_WAITING_FOR_USER_SCREENSHOT`
- `SCREENSHOT_RECEIVED_READY_FOR_CHATGPT_DISCUSSION`
- `DISCUSSION_COMPLETE_WAITING_FOR_CONFIRMED_PLAN`
- `CONFIRMED_PLAN_READY_FOR_HERMES_EXECUTION_NEXT_PHASE`
- `INSUFFICIENT_VISUAL_EVIDENCE_NEEDS_ANOTHER_BOUNDED_REPRODUCTION`

None of these means autonomous success or exit success.

## Stop condition

Phase78-flow-fix stops after this workflow is documented and statically verified. It does not implement algorithm fixes and does not enter a repair phase.
