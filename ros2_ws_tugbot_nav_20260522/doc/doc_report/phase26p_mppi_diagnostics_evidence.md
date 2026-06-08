# Phase26P MPPI diagnostics-only evidence collection

Date: 2026-05-26
Workspace: `/home/hyh/Desktop/agent_playground/playground_hermes/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260522`

## Goal

Phase26P remains analysis-first. The goal is not tuning and not branch-selection intervention.

Phase26P adds the smallest practical evidence collection path for MPPI/controller internals:

- keep branch selection unchanged;
- keep MPPI cost / velocity semantics unchanged;
- only enable and record MPPI debug evidence;
- join MPPI evidence with Phase26N-style first-cmd-near-zero windows and local-cost timing.

## Files changed / added

- `src/tugbot_navigation/config/nav2_slam_phase26p_mppi_diagnostics_params.yaml`
  - generated from canonical baseline `nav2_slam_params.yaml`;
  - only adds:
    - `FollowPath.publish_critics_stats: true`
    - `FollowPath.publish_optimal_trajectory: true`
  - canonical baseline `CostCritic.cost_weight` remains `3.81`.

- `src/tugbot_navigation/config/nav2_slam_phase26p_candidate_mppi_diagnostics_params.yaml`
  - generated from `nav2_slam_candidate_costcritic_275_params.yaml`;
  - only adds the same debug-publish keys;
  - candidate semantics remain `CostCritic.cost_weight: 2.75`.

- `src/tugbot_bringup/launch/tugbot_maze_explore.launch.py`
  - adds launch args:
    - `phase26p_mppi_diagnostics_profile`
    - `phase26p_candidate_mppi_diagnostics_profile`
  - routes those args to the two Phase26P diagnostics YAMLs.

- `tools/record_phase26p_mppi_evidence.py`
  - generic diagnostics-only topic recorder;
  - discovers topics matching:
    - `critics_stats`
    - `optimal_trajectory`
    - `transformed_global_plan`
    - `trajectories`
    - `trajectory`
  - records discovered topics and messages as JSONL;
  - hardened against `ExternalShutdownException` on wrapper shutdown.

- `tools/analyze_phase26p_mppi_evidence.py`
  - joins Phase26N timeline cases with Phase26P MPPI evidence around first cmd-near-zero ±1s;
  - reports:
    - critic stats sample count and top critic by max cost when available;
    - optimal trajectory / trajectory sample count;
    - path displacement evidence for `nav_msgs/Path` optimal trajectories;
    - trajectory-validator evidence if any matching topic exists;
    - local-cost high-window vs first-cmd-near-zero ordering;
    - condition hypothesis, with guardrails.

- `tools/run_phase21_controller_diagnostics_smoke.sh`
  - accepts run IDs:
    - `phase26p_baseline_diag_runN`
    - `phase26p_candidate_diag_runN`
  - records Phase26P MPPI evidence JSONL;
  - supports bounded smoke overrides:
    - `PHASE_RUN_MAX_GOALS`
    - `PHASE_RUN_STATE_MAX_SAMPLES`
    - `PHASE_RUN_GOAL_EVENTS_MAX_SAMPLES`
    - `PHASE_RUN_TIMEOUT_SEC`
    - `PHASE_RUN_SNAPSHOT_TIMEOUT_SEC`

- `src/tugbot_maze/test/test_phase26p_mppi_diagnostics.py`
  - TDD contracts for diagnostics-only params equivalence, wrapper integration, recorder shutdown hardening, and analyzer join behavior.

## TDD / verification

RED:

- `python3 -m pytest src/tugbot_maze/test/test_phase26p_mppi_diagnostics.py -q`
- initial failures were expected: missing Phase26P YAMLs, wrapper run IDs, recorder, analyzer, and later missing path-displacement analyzer support / shutdown hardening.

GREEN / regression:

- `python3 -m pytest src/tugbot_maze/test/test_phase26p_mppi_diagnostics.py -q`
  - `3 passed in 0.10s`

- `python3 -m py_compile tools/record_phase26p_mppi_evidence.py tools/analyze_phase26p_mppi_evidence.py`
  - passed under sourced ROS Jazzy env.

- `bash -n tools/run_phase21_controller_diagnostics_smoke.sh`
  - passed.

- `python3 -m pytest src/tugbot_maze/test/test_phase24*.py src/tugbot_maze/test/test_phase25*.py src/tugbot_maze/test/test_phase26*.py -q`
  - `76 passed in 1.52s`

- `. /opt/ros/jazzy/setup.sh && . install/setup.sh && colcon build --symlink-install --packages-select tugbot_bringup tugbot_navigation tugbot_maze`
  - `3 packages finished [0.85s]`

## Runtime smoke / topic discovery

Command:

```bash
PHASE_RUN_MAX_GOALS=2 \
PHASE_RUN_STATE_MAX_SAMPLES=80 \
PHASE_RUN_GOAL_EVENTS_MAX_SAMPLES=60 \
PHASE_RUN_TIMEOUT_SEC=180 \
PHASE_RUN_SNAPSHOT_TIMEOUT_SEC=200 \
bash tools/run_phase21_controller_diagnostics_smoke.sh phase26p_baseline_diag_run1
```

Runtime proof:

- `log/phase26p_baseline_diag_run1_runtime_params/controller_server.yaml`
- runtime summary printed:
  - `node: /controller_server`
  - `expected_cost_weight: 3.81`
  - extracted `FollowPath.CostCritic.cost_weight: 3.81`

Artifacts:

- `log/phase26p_baseline_diag_run1_mppi_evidence.jsonl`
  - file size: about `271.27 MB`
  - line count: `151`
  - events:
    - `topic_discovered`: `4`
    - `message`: `146`
    - `summary`: `1`
  - discovered / recorded topics:
    - `/optimal_trajectory`: `50`, type `nav_msgs/msg/Path`
    - `/trajectories`: `50`, type `visualization_msgs/msg/MarkerArray`
    - `/transformed_global_plan`: `49`, type `nav_msgs/msg/Path`
    - `/docking_trajectory`: `1`, type `nav_msgs/msg/Path`

- `log/phase26p_baseline_diag_run1_phase26p_single_goal_timeline.json`
- `log/phase26p_baseline_diag_run1_phase26p_mppi_evidence_analysis.json`
- plus normal wrapper artifacts:
  - `*_goal_events.jsonl`
  - `*_explorer_state.jsonl`
  - `*_controller_dynamics.jsonl`
  - `*_post_recovery_snapshots.jsonl`
  - `*_goal_nav2_analysis.json`
  - `*_goal_controller_dynamics.json`
  - `*_timeout_subtypes.json`
  - `*_post_recovery_enriched.json`
  - `*_params_fingerprint.json`

## Runtime topic-discovery findings

1. `publish_optimal_trajectory: true` works in this stack.
   - Runtime topic discovered: `/optimal_trajectory`.
   - Message type: `nav_msgs/msg/Path`.
   - This is useful path-shape evidence, but it does not carry per-sample velocity.

2. `visualize: true` / MPPI visualization works.
   - Runtime topic discovered: `/trajectories`.
   - Message type: `visualization_msgs/msg/MarkerArray`.
   - It is high-volume: only 50 message rows already produced a ~271 MB JSONL because MarkerArray payloads are large.
   - Future longer runs should either cap samples much more aggressively or summarize marker arrays online instead of storing raw full JSON.

3. Transformed plan evidence exists.
   - Runtime topic discovered: `/transformed_global_plan`.
   - Message type: `nav_msgs/msg/Path`.

4. `critics_stats` was not discovered in this run.
   - No topic containing `critics_stats` appeared in the recorded discovery set.
   - Therefore, Phase26P currently still cannot report per-cycle critic changed-cost flags / cost sums for this runtime stack.
   - The analyzer supports synthetic and future real `critics_stats` rows, but real availability remains unproven.

5. Trajectory validator accept/reject evidence was not discovered.
   - No validator topic/log evidence was found by the Phase26P generic topic recorder in this smoke.

## Phase26P analysis result

Analyzer output:

- `log/phase26p_baseline_diag_run1_phase26p_mppi_evidence_analysis.json`

Summary:

- analyzed cases: `1`
- real MPPI evidence case count: `1`
- condition hypothesis counts:
  - `trajectory_evidence_present_without_critic_stats`: `1`

Case: `phase26p_baseline_diag_run1`, goal sequence `2`

- first cmd near-zero after recovery: `6.408356s`
- first high local-cost window after recovery: `0.67098s`
- high-cost window before first cmd near-zero: `true`
- evidence rows in first-cmd-near-zero ±1s window: `9`
- optimal trajectory/path evidence:
  - `sample_count: 3`
  - `path_sample_count: 3`
  - `path_displacement_min: 0.15203711480609092 m`
  - `zero_displacement_path_sample_count: 0`
  - `velocity_sample_count: 0`
- critic stats evidence:
  - `sample_count: 0`
- trajectory validator evidence:
  - `sample_count: 0`

## Conclusion

Phase26P successfully added a diagnostics-only evidence collection path and verified that the runtime stack can publish useful MPPI path/trajectory evidence. It did not locate a specific actionable MPPI condition.

What Phase26P can say from the short bounded smoke:

- controller runtime params stayed baseline-equivalent for semantics (`CostCritic.cost_weight: 3.81`);
- branch selection was not changed;
- debug topics `/optimal_trajectory`, `/trajectories`, and `/transformed_global_plan` are available;
- in the analyzed recovery case, local high-cost evidence occurred before first cmd-near-zero and optimal path evidence existed around the window;
- the optimal path was not degenerate by the analyzer's path-displacement check.

What Phase26P cannot yet say:

- cannot attribute near-zero cmd to `CostCritic`, `PathAlignCritic`, `PathFollowCritic`, `GoalCritic`, trajectory validator reject, or no-valid-control pathway;
- cannot report critic changed-cost flags / cost sums because no real `critics_stats` topic was discovered;
- cannot use `/optimal_trajectory` to infer selected velocity because the runtime message is `nav_msgs/msg/Path`, not a velocity trajectory;
- cannot justify narrow reversible intervention from this single short bounded smoke.

Decision:

- Phase27 remains blocked.
- Do not tune Nav2/controller params from Phase26P.
- Do not change branch selection from Phase26P.
- Do not promote or reject `2.75` from Phase26P.

## Recommended next step

Continue analysis-first with a Phase26Q-style narrow evidence hardening step, not an intervention:

1. Reduce raw `/trajectories` payload cost.
   - The current generic recorder produces huge JSONL artifacts.
   - Add an online summarizer for `visualization_msgs/MarkerArray` that records counts, min/max/mean marker point displacement, and selected representative path features instead of full raw arrays.

2. Confirm whether Nav2 Jazzy MPPI exposes critic scores in this build.
   - Inspect installed MPPI controller parameters / source docs for exact critic stats parameter and topic name.
   - If `critics_stats` is not available in this version, document that limitation and use trajectory/path-shape + local-cost + controller logs instead.

3. If critic stats can be exposed, run one bounded matched baseline/candidate diagnostic repeat using:
   - `/optimal_trajectory`
   - `/transformed_global_plan`
   - summarized `/trajectories`
   - `/maze/goal_events`
   - `cmd_vel_nav`
   - local-cost snapshots
   - runtime params dump

4. Only after real per-cycle critic/trajectory-validator/no-valid-control evidence exists should we discuss narrow reversible intervention.
