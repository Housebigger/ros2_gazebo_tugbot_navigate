# Phase125 First exploration goal execution result smoke report

Status: PHASE125_FIRST_EXPLORATION_GOAL_EXECUTION_RESULT_SMOKE_COMPLETE_STOP_BEFORE_PHASE126

## Summary

Phase125 ran the real visible-stack first exploration goal execution-result smoke. It reused the Phase120-style explicit inner-ingress chain and Phase122 handoff gate, then started `maze_explorer` with `max_goals=1` and waited for the bounded Nav2 result of the first current-algorithm `goal_kind=explore` goal.

Final classification:

`FIRST_EXPLORATION_GOAL_RESULT_TIMEOUT_DIAGNOSTIC_FAIL`

This is a diagnostic fail for first-goal execution result only. It is not autonomous exploration success, not exit success, and not full-maze success.

## Runtime artifacts

- Runtime artifact: `log/phase125_first_exploration_goal_execution_result_smoke/phase125_first_exploration_goal_execution_result_smoke_artifact.json`
- Analyzer result: `log/phase125_first_exploration_goal_execution_result_smoke/phase125_first_exploration_goal_execution_result_smoke_analysis.json`
- Summary: `log/phase125_first_exploration_goal_execution_result_smoke/phase125_first_exploration_goal_execution_result_smoke_summary.md`
- Phase120 ingress artifact: `log/phase125_first_exploration_goal_execution_result_smoke/phase125_first_exploration_goal_execution_result_smoke_phase120_artifact.json`
- Ingress preflight artifact: `log/phase125_first_exploration_goal_execution_result_smoke/phase125_first_exploration_goal_execution_result_smoke_ingress_preflight.json`
- maze_explorer stderr: `log/phase125_first_exploration_goal_execution_result_smoke/phase125_first_exploration_goal_execution_result_smoke_artifact_maze_explorer_stderr.log`
- visible stack stdout/stderr: `log/phase125_first_exploration_goal_execution_result_smoke/phase125_first_exploration_goal_execution_result_smoke_visible_stack_stdout.log`, `log/phase125_first_exploration_goal_execution_result_smoke/phase125_first_exploration_goal_execution_result_smoke_visible_stack_stderr.log`

## Scope and guardrails checked

Allowed runtime actions performed:

- Started visible Gazebo/RViz/SLAM/Nav2 stack.
- Sent only the explicit inner-ingress goal: `frame_id=map`, `x=2.0`, `y=0.0`, `yaw=0.0`.
- Started `maze_explorer` only after handoff readiness.
- Used `maze_explorer max_goals=1`.
- Waited for the first current-algorithm exploration goal Nav2 result boundary.

Forbidden actions were not performed:

- No second exploration goal dispatch.
- No manual Goal1.
- No carry-over/staging/branch/centerline/fallback/terminal/exit goal.
- No Nav2/MPPI/controller/config tuning.
- No exploration strategy, branch scoring, centerline, fallback, or terminal acceptance change.
- No autonomous exploration success claim.
- No exit success claim.

Analyzer guardrails all passed:

- `classification_known: true`
- `classification_matches_evidence: true`
- `handoff_not_ready_prevents_start: true`
- `max_goals_one: true`
- `dispatch_event_count_one_or_less: true`
- `second_goal_dispatched_false: true`
- `non_explore_not_success: true`
- `no_manual_goal1: true`
- `no_fallback_terminal_exit_success: true`
- `branch_scoring_changed_false: true`
- `centerline_gate_changed_false: true`
- `fallback_changed_false: true`
- `terminal_acceptance_changed_false: true`
- `nav2_config_changed_false: true`
- `no_autonomous_success_claim: true`
- `no_exit_success_claim: true`

## Phase120 ingress evidence

The same-run Phase120-style ingress chain passed:

- `preflight_passed: true`
- `same_run_readiness_wait_passed: true`
- `locked_explicit_inner_ingress_goal: true`
- ingress result accepted: `true`
- ingress result status: `SUCCEEDED`
- ingress goal pose: `map`, `x=2.0`, `y=0.0`, `yaw=0.0`

## Phase122 handoff evidence

The Phase122-style handoff gate passed:

- `handoff_allowed: true`
- `inner_ingress_result_succeeded: true`
- `robot_pose_near_ingress_goal: true`
- `nav2_action_idle: true`
- `costmap_fresh: true`
- `scan_fresh: true`
- `tf_fresh: true`

Post-ingress pose evidence:

- robot pose after ingress: `x=1.8569191491890518`, `y=0.024192576812237812`, `yaw=-0.005571541767227017`
- distance to ingress goal: `0.1451117177956379 m`, tolerance `0.35 m`
- orientation error: `0.005571541767227017 rad`, tolerance `0.35 rad`

## First exploration goal evidence

`maze_explorer` started with `max_goals=1` and produced exactly one current-algorithm exploration dispatch.

First goal fields:

- `goal_kind: explore`
- `candidate_source: maze_explorer_current_algorithm`
- `candidate_family: junction`
- `candidate_rank: 1`
- `candidate_id: 1`
- `candidate_count: 4`
- `near_exit: false`
- `selection_reason: topology_exit_bias_score`
- pose frame: `map`
- pose x: `2.0874871732589573`
- pose y: `1.0229234653829486`
- pose yaw: `1.5652247850276695`
- accepted: `true`
- rejected: `false`
- timeout: `true`
- result_status_label: `TIMEOUT`
- abort_text: `goal_timeout`
- stop_reason: `first_explore_goal_result_timeout_stop`

Count evidence:

- `goal_event_count: 2`
- `dispatch_event_count: 1`
- `second_goal_dispatched: false`
- `second_goal_dispatched=false`
- Nav2 feedback samples retained: `80`
- local_costmap samples retained: `20`

Result pose evidence:

- `robot_pose_at_result` available: `true`
- `robot_pose_at_result`: `x=2.4437687200958207`, `y=1.0153386444337151`
- `distance_to_first_goal`: `0.35636227371215945 m`

Freshness at result:

- costmap fresh: `true`, local costmap age `0.34584474563598633 s`
- scan fresh: `true`, scan age `0.2823033332824707 s`
- TF fresh: `true`
- map seen: `true`
- odom seen: `true`

## Diagnostic observations

The first exploration goal was accepted but did not reach a successful Nav2 result inside the active goal timeout. `maze_explorer` emitted:

- `maze explorer goal timed out; classifying active goal failure`
- `phase23 timeout local cost seq=1 footprint_max=99 front_max=100 path_0_5m_max=73 path_1_0m_max=84`
- `maze explorer ignoring timeout-canceled goal result seq=1 status=5`

The retained Nav2 feedback timeline showed the first goal still had nonzero distance remaining near timeout and recovery activity:

- retained feedback count: `80`
- representative final retained distance remaining: about `0.3770221173763275 m`
- representative final retained number of recoveries: `4`

The runner terminated `maze_explorer` after the bounded timeout event was captured to enforce the Phase125 stop boundary. The subprocess return code was `-15`, and stderr contained an rclpy shutdown traceback after termination. This is recorded as shutdown noise after the timeout evidence, not as a successful result.

## Classification decision

Classification is:

`FIRST_EXPLORATION_GOAL_RESULT_TIMEOUT_DIAGNOSTIC_FAIL`

Reason:

- handoff was ready;
- `maze_explorer` started with `max_goals=1`;
- exactly one current-algorithm `goal_kind=explore` dispatch occurred;
- dispatch was accepted;
- no second goal dispatch occurred;
- the first goal ended via timeout evidence (`result_status_label=TIMEOUT`, `abort_text=goal_timeout`), not success.

## Verification

Runtime analyzer:

- `valid: true`
- `classification_matches_evidence: true`

Focused/static tests before final report:

- Phase121/122/123/124/125 bundle: `36 passed in 0.21s`

Final cleanup and guards are recorded under:

- process guard: `log/phase125_first_exploration_goal_execution_result_smoke/phase125_first_exploration_goal_execution_result_smoke_cleanup_after_runtime.txt`
- final process guard / Nav2 config diff guard / scoped pycache guard: `log/phase125_first_exploration_goal_execution_result_smoke/phase125_first_exploration_goal_execution_result_smoke_final_guard.txt`

Final process guard was empty, and the Nav2 config diff guard was empty for `src/tugbot_navigation/config`.

## Stop condition

Phase125 stops here. Phase126 not entered. Do not enter Phase126 without explicit human acceptance.
