# Phase127 First goal timeout artifact replay report

Status: PHASE127_FIRST_GOAL_TIMEOUT_ARTIFACT_REPLAY_COMPLETE_STOP_BEFORE_PHASE128

## Summary

Phase127 completed a diagnosis-only artifact replay over the existing Phase125 artifacts. No simulator or ROS runtime was launched. The analyzer read only the Phase125 first exploration goal execution-result artifact and maze_explorer stderr log, then classified the first current-algorithm `goal_kind=explore` timeout using the Phase126 evidence taxonomy.

Final classification:

`FIRST_GOAL_TIMEOUT_LOCAL_COST_BLOCKED`

Reason:

The Phase125 timeout artifact contains a time-aligned timeout with retained Nav2 feedback showing a distance plateau and recoveries, plus the maze_explorer timeout local-cost diagnostic line showing high footprint/front/path costs:

- `footprint_max=99`
- `front_max=100`
- `path_0_5m_max=73`
- `path_1_0m_max=84`

This is stronger direct evidence for local-cost blockage than for controller stall, goal-tolerance edge, or candidate-risk. The classification remains diagnostic only and does not authorize repair or parameter tuning.

## Inputs

Read-only Phase125 sources:

- `log/phase125_first_exploration_goal_execution_result_smoke/phase125_first_exploration_goal_execution_result_smoke_artifact.json`
- `log/phase125_first_exploration_goal_execution_result_smoke/phase125_first_exploration_goal_execution_result_smoke_artifact_maze_explorer_stderr.log`

Phase127 outputs:

- `tools/analyze_phase127_first_goal_timeout_artifact_replay.py`
- `src/tugbot_maze/test/test_phase127_first_goal_timeout_artifact_replay.py`
- `log/phase127_first_goal_timeout_artifact_replay/phase127_first_goal_timeout_artifact_replay_analysis.json`
- `log/phase127_first_goal_timeout_artifact_replay/phase127_first_goal_timeout_artifact_replay_summary.md`
- `log/phase127_first_goal_timeout_artifact_replay/phase127_first_goal_timeout_artifact_replay_source_phase125_artifact_snapshot.json`
- `log/phase127_first_goal_timeout_artifact_replay/phase127_first_goal_timeout_artifact_replay_source_maze_explorer_stderr_snapshot.log`

## Phase125 boundary preserved

The analyzer verified the input still matches the Phase125 first-goal timeout boundary:

- source Phase125 classification: `FIRST_EXPLORATION_GOAL_RESULT_TIMEOUT_DIAGNOSTIC_FAIL`
- `maze_explorer_max_goals=1`
- exactly one current-algorithm `goal_kind=explore`
- dispatch accepted
- `result_status_label=TIMEOUT`
- `abort_text=goal_timeout`
- `second_goal_dispatched=false`
- no autonomous exploration success claim
- no exit success claim

## Nav2 feedback timeline

Retained Phase125 feedback evidence:

- sample count: `80`
- first retained `distance_remaining`: `0.3770221173763275`
- last retained `distance_remaining`: `0.3770221173763275`
- distance delta: `0.0`
- distance plateau: `true`
- first retained `navigation_time_sec`: `44.853`
- last retained `navigation_time_sec`: `45.642`
- retained navigation time span: `0.7890000000000015 s`
- max retained recoveries: `4`

Interpretation:

The retained feedback window near timeout shows no distance progress and recovery activity. It supports the existence of a timeout/stall symptom, but by itself does not distinguish controller stall from local-cost blockage.

## Robot pose / distance-to-goal evidence

Phase125 result pose evidence:

- robot_pose_at_result: `x=2.4437687200958207`, `y=1.0153386444337151`
- first goal pose: `x=2.0874871732589573`, `y=1.0229234653829486`, `yaw=1.5652247850276695`
- distance_to_first_goal: `0.35636227371215945 m`

Interpretation:

The final distance is near a plausible goal-tolerance boundary, so `FIRST_GOAL_TIMEOUT_GOAL_TOLERANCE_EDGE` remains a hypothesis. However, Phase125 artifacts do not include goal checker state, yaw-error trace, or a robot pose trace across the full dispatch-to-timeout interval. Therefore Phase127 does not classify tolerance edge.

## Local cost evidence

Parsed timeout local-cost line:

```text
phase23 timeout local cost seq=1 footprint_max=99 front_max=100 path_0_5m_max=73 path_1_0m_max=84
```

Analyzer evidence:

- parsed seq: `1`
- footprint_max: `99`
- front_max: `100`
- path_0_5m_max: `73`
- path_1_0m_max: `84`
- local_costmap_sample_count: `20`
- costmap_fresh: `true`
- severity: `high_front_or_footprint_and_path_cost`
- supports_local_cost_blocked: `true`

Interpretation:

This is the strongest available discriminating evidence in the Phase125 artifact replay. The combination of high footprint/front cost and elevated path costs near timeout supports `FIRST_GOAL_TIMEOUT_LOCAL_COST_BLOCKED`.

Evidence limitation:

The Phase125 artifact retained local costmap sample metadata and timeout cost summaries, but did not retain full local/global costmap windows. Therefore the classification is bounded to artifact replay evidence and should be treated as a diagnostic classification, not a repair authorization.

## Candidate risk evidence

Selected candidate evidence from Phase125:

- candidate_family: `junction`
- selected rank: `1`
- candidate branch count: `4`
- selected target_local_cost: `0.0`
- selected target_local_cost_max_radius: `54.0`
- selected dispatch_path_local_cost_max: `0.0`
- selected path_corridor_min_clearance_m: `0.6519202502346335`
- selected target_clearance_m: `0.6519202502346335`
- safer lower-ranked branch exists: `false`
- supports_candidate_too_risky: `false`

Interpretation:

The selected candidate did not look risky at dispatch according to the available candidate scoring/cost/clearance fields. Lower-ranked alternatives were not clearly safer by the analyzer's evidence thresholds. Therefore Phase127 does not classify `FIRST_GOAL_TIMEOUT_CANDIDATE_TOO_RISKY`.

## Controller stall evidence

Analyzer result:

- supports_controller_stall: `false`
- cmd_vel_count: `0`
- odom_velocity_count: `0`

Evidence gaps:

- cmd_vel timeline missing
- odom velocity timeline missing

Interpretation:

Controller stall cannot be responsibly classified from Phase125 artifacts because cmd_vel and odom velocity timelines were not retained. The timeout and recoveries are compatible with controller stall, but the stronger available evidence is local-cost blockage.

## Goal tolerance edge evidence

Analyzer result:

- supports_goal_tolerance_edge: `false`
- distance_to_first_goal_m: `0.35636227371215945`
- pose_trace_count: `0`
- goal_checker_state_available: `false`
- xy_tolerance_m: `null`
- yaw_error_rad: `null`
- yaw_tolerance_rad: `null`

Evidence gaps:

- robot pose trace missing
- goal checker state missing

Interpretation:

The final distance is close enough that goal-tolerance edge remains an open hypothesis, but Phase125 artifacts do not include enough pose/yaw/goal-checker evidence to classify it.

## Evidence gaps

Phase127 identified these remaining evidence gaps:

- local/global costmap windows unavailable in Phase125 artifact
- cmd_vel timeline missing
- odom velocity timeline missing
- robot pose trace missing
- goal checker state missing

These gaps should be treated as requirements for any later diagnosis or repair-planning phase. They do not invalidate the bounded artifact replay classification, but they prevent stronger claims about controller stall or goal tolerance.

## Guardrails

No Gazebo/RViz/Nav2 runtime was launched.
No NavigateToPose goal was sent.
No maze_explorer was started.
No second exploration goal was sent.
No Nav2/MPPI/controller/goal checker/config tuning was performed.
No exploration strategy was changed.
No branch scoring, centerline gate, fallback, or terminal acceptance logic was changed.
No autonomous exploration success or exit success is claimed.
Phase128 not entered.

Analyzer guardrails:

- no_runtime_replay: `true`
- no_success_claim: `true`
- phase125_timeout_input: `true`
- max_goals_one: `true`
- second_goal_dispatched_false: `true`
- dispatch_event_count_one: `true`
- goal_kind_explore: `true`
- accepted_timeout_not_success: `true`
- nav2_config_unchanged: `true`
- branch_scoring_unchanged: `true`
- centerline_gate_unchanged: `true`
- fallback_unchanged: `true`
- terminal_acceptance_unchanged: `true`

## Verification

Focused analyzer tests:

- `src/tugbot_maze/test/test_phase127_first_goal_timeout_artifact_replay.py`

Final verification bundle includes Phase121/122/123/124/125/126/127 focused/static tests.

No-runtime/process/config guards are recorded under:

- `log/phase127_first_goal_timeout_artifact_replay/phase127_first_goal_timeout_artifact_replay_final_guard.txt`

## Stop condition

Phase127 stops here. Phase128 not entered. Wait for human acceptance before any Phase128 work.
