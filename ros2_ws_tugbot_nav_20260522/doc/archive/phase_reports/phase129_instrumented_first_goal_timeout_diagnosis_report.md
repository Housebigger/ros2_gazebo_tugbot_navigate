# Phase129 instrumented first-goal timeout runtime diagnosis report

Status: PHASE129_INSTRUMENTED_FIRST_GOAL_TIMEOUT_DIAGNOSIS_COMPLETE_STOP_BEFORE_PHASE130

## Scope

Phase129 repeated the Phase125 visible-stack ingress/handoff/maze_explorer chain with added first-goal instrumentation.

Allowed in this phase:

- visible-stack Gazebo/RViz/SLAM/Nav2 startup;
- explicit inner-ingress goal: `map`, `x=2.0`, `y=0.0`, `yaw=0.0`;
- `maze_explorer` startup only with `max_goals=1`;
- at most one current-algorithm exploration dispatch;
- stop after the first terminal/guarded result.

Guardrails preserved:

- No second exploration goal.
- No manual Goal1/carry-over/staging/branch/centerline/fallback/terminal/exit goal was sent by Phase129 tooling.
- No Nav2/MPPI/controller/goal checker/config tuning.
- No exploration strategy, branch scoring, centerline, fallback, or terminal acceptance changes.
- No autonomous exploration success or exit success is claimed.
- Phase130 not entered.

## Implemented deliverables

- Runner: `tools/run_phase129_instrumented_first_goal_timeout_diagnosis.py`
- Analyzer: `tools/analyze_phase129_instrumented_first_goal_timeout_diagnosis.py`
- Focused tests: `src/tugbot_maze/test/test_phase129_instrumented_first_goal_timeout_diagnosis.py`
- Runtime/log directory: `log/phase129_instrumented_first_goal_timeout_diagnosis/`
- Report: `doc/doc_report/phase129_instrumented_first_goal_timeout_diagnosis_report.md`

## TDD/static verification

Focused Phase129 tests were written first and verified RED before implementation:

```text
python3 -m pytest src/tugbot_maze/test/test_phase129_instrumented_first_goal_timeout_diagnosis.py -q
# initial RED: 8 failed, 1 passed
```

After implementing the runner/analyzer:

```text
python3 -m pytest src/tugbot_maze/test/test_phase129_instrumented_first_goal_timeout_diagnosis.py -q
# 9 passed in 0.03s
```

Pre-runtime static/focused regression bundle:

```text
python3 -m pytest \
  src/tugbot_maze/test/test_phase120_controlled_ingress_dispatch_with_readiness_wait.py \
  src/tugbot_maze/test/test_phase121_post_ingress_handoff_design.py \
  src/tugbot_maze/test/test_phase122_post_ingress_handoff_dry_start.py \
  src/tugbot_maze/test/test_phase123_first_exploration_goal_dispatch_design.py \
  src/tugbot_maze/test/test_phase124_first_exploration_goal_dispatch_smoke.py \
  src/tugbot_maze/test/test_phase125_first_exploration_goal_execution_result_smoke.py \
  src/tugbot_maze/test/test_phase126_first_goal_timeout_diagnosis_design.py \
  src/tugbot_maze/test/test_phase127_first_goal_timeout_artifact_replay.py \
  src/tugbot_maze/test/test_phase128_first_goal_timeout_instrumentation_design.py \
  src/tugbot_maze/test/test_phase129_instrumented_first_goal_timeout_diagnosis.py \
  -q
# 75 passed in 0.15s
```

Recorded under:

- `log/phase129_instrumented_first_goal_timeout_diagnosis/phase129_static_test_bundle_pre_runtime.txt`
- `log/phase129_instrumented_first_goal_timeout_diagnosis/phase129_pre_runtime_guard.txt`

## Runtime commands and artifacts

Visible-stack launch:

```text
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch tugbot_bringup tugbot_maze_slam_nav.launch.py headless:=false use_rviz:=true
```

Primary Phase129 runner attempt:

```text
python3 tools/run_phase129_instrumented_first_goal_timeout_diagnosis.py \
  --output log/phase129_instrumented_first_goal_timeout_diagnosis/phase129_instrumented_first_goal_timeout_diagnosis.json \
  --phase120-output log/phase129_instrumented_first_goal_timeout_diagnosis/phase129_phase120_ingress_artifact.json \
  --preflight-output log/phase129_instrumented_first_goal_timeout_diagnosis/phase129_ingress_preflight.json \
  --run-id phase129_instrumented_first_goal_timeout_diagnosis_visible_stack \
  --launch-log-path log/phase129_instrumented_first_goal_timeout_diagnosis/phase129_visible_stack_launch_stdout.log \
  --readiness-wait-timeout-sec 60 \
  --first-goal-result-observation-sec 85 \
  --bounded-goal-result-wait-sec 45
```

Primary attempt result:

```text
classification=INSUFFICIENT_TIMEOUT_EVIDENCE
handoff_allowed=false
maze_explorer_started=false
dispatch_event_count=0
second_goal_dispatched=false
```

The primary attempt failed closed before maze_explorer because the inherited readiness wait did not observe active lifecycle state within its bounded query window, although a later direct ROS check showed Nav2 actions and lifecycle nodes active. This artifact was retained as a readiness/launch-log evidence item, not as the final first-dispatch observation.

Primary artifacts:

- `log/phase129_instrumented_first_goal_timeout_diagnosis/phase129_instrumented_first_goal_timeout_diagnosis.json`
- `log/phase129_instrumented_first_goal_timeout_diagnosis/phase129_analysis_initial_readiness_timeout.json`
- `log/phase129_instrumented_first_goal_timeout_diagnosis/phase129_analysis_summary_initial_readiness_timeout.md`

Bounded rerun after confirming the same visible stack was active and preserving the same Phase129 scope:

```text
python3 tools/run_phase129_instrumented_first_goal_timeout_diagnosis.py \
  --output log/phase129_instrumented_first_goal_timeout_diagnosis/phase129_instrumented_first_goal_timeout_diagnosis_rerun.json \
  --phase120-output log/phase129_instrumented_first_goal_timeout_diagnosis/phase129_phase120_ingress_artifact_rerun.json \
  --preflight-output log/phase129_instrumented_first_goal_timeout_diagnosis/phase129_ingress_preflight_rerun.json \
  --run-id phase129_instrumented_first_goal_timeout_diagnosis_visible_stack_rerun \
  --launch-log-path log/phase129_instrumented_first_goal_timeout_diagnosis/phase129_visible_stack_readiness_marker.log \
  --readiness-wait-timeout-sec 20 \
  --first-goal-result-observation-sec 85 \
  --bounded-goal-result-wait-sec 45
```

Analyzer command:

```text
python3 tools/analyze_phase129_instrumented_first_goal_timeout_diagnosis.py \
  --artifact log/phase129_instrumented_first_goal_timeout_diagnosis/phase129_instrumented_first_goal_timeout_diagnosis_rerun.json \
  --output log/phase129_instrumented_first_goal_timeout_diagnosis/phase129_analysis_rerun.json \
  --summary log/phase129_instrumented_first_goal_timeout_diagnosis/phase129_analysis_summary_rerun.md
```

Rerun artifacts:

- `log/phase129_instrumented_first_goal_timeout_diagnosis/phase129_instrumented_first_goal_timeout_diagnosis_rerun.json`
- `log/phase129_instrumented_first_goal_timeout_diagnosis/phase129_analysis_rerun.json`
- `log/phase129_instrumented_first_goal_timeout_diagnosis/phase129_analysis_summary_rerun.md`
- `log/phase129_instrumented_first_goal_timeout_diagnosis/phase129_instrumented_first_goal_timeout_diagnosis_rerun_maze_explorer_stdout.log`
- `log/phase129_instrumented_first_goal_timeout_diagnosis/phase129_instrumented_first_goal_timeout_diagnosis_rerun_maze_explorer_stderr.log`

## Runtime result classification

Final recorded classification:

```text
INSUFFICIENT_TIMEOUT_EVIDENCE
```

Reason:

- The visible stack and same-run inner-ingress/handoff gate were sufficient to start `maze_explorer` with `max_goals=1`.
- `maze_explorer` produced exactly one dispatch event.
- The first dispatch was not an allowed `goal_kind=explore` dispatch. It was `goal_kind=corridor_alignment_staging`.
- The Phase129 runner stopped fail-closed immediately after observing that the first dispatch was non-explore.
- No second exploration goal was sent.
- Because no allowed first `goal_kind=explore` Nav2 result reached timeout/success/abort, Phase129 cannot classify the Phase127 timeout as local-cost blocked/controller stall/goal tolerance edge/candidate-too-risky from this runtime.

Observed first-dispatch details from the rerun artifact:

```text
handoff_allowed=true
maze_explorer_started=true
maze_explorer_max_goals=1
dispatch_event_count=1
second_goal_dispatched=false
stop_reason=first_dispatch_not_explore_diagnostic_stop
goal_kind=corridor_alignment_staging
result_status_label=REJECTED_NON_EXPLORE_GOAL_KIND
candidate_family=junction
candidate_rank=1
pose=(map, x=1.7005222087522947, y=0.07425350703890773, yaw=1.5658846410517562)
```

This is a diagnostic stop, not a repaired behavior and not an autonomous exploration/exit success.

## Required instrumentation coverage

Phase129 runner/analyzer schema supports all Phase128 evidence-gap fields:

- cmd_vel timeline
- odom velocity timeline
- commanded vs measured velocity
- robot pose trace
- distance/yaw error curve
- goal checker state if available
- local/global costmap windows
- footprint/front/path corridor cost snapshots
- planned path snapshots
- BT recovery events
- controller_server logs
- bt_navigator logs
- first goal candidate risk evidence

Observed coverage in the bounded rerun:

```text
cmd_vel timeline: 0 samples
odom velocity timeline: 6 samples
commanded vs measured velocity: present
robot pose trace: 0 samples
distance error curve: 0 samples
yaw error curve: 0 samples
goal checker state: unavailable
local/global costmap windows: local=7, global=3
footprint/front/path corridor cost snapshots: 0 samples
planned path snapshots: 0 samples
BT recovery events: 0 samples
controller_server logs: 0 samples
bt_navigator logs: 0 samples
first goal candidate risk: present
```

Coverage was limited because the runner stopped before Nav2 accepted/executed an allowed explore goal; the non-explore dispatch guard fired before cmd_vel/path/BT recovery/controller diagnostics could accumulate.

## First goal candidate risk evidence

The first current-algorithm dispatch reported:

```text
candidate_family=junction
candidate_rank=1
target_clearance_m=0.4301162697613631
path_corridor_min_clearance_m=0.4301162697613631
target_local_cost=46
target_local_cost_max_radius=99
branch_geometry.branch_angle=1.5658846410517562
candidate_branch_count=3
selected_branch_riskier_than_alternatives=true
near_high_cost_band=true
near_wall=false
```

This risk evidence is retained only as context for the non-explore dispatch. Since this dispatch was `corridor_alignment_staging`, it does not prove `FIRST_GOAL_TIMEOUT_CANDIDATE_TOO_RISKY` for an allowed `goal_kind=explore` timeout.

## Controller and BT logs

The visible stack emitted controller_server and bt_navigator startup/configuration logs during launch. Runtime artifact coverage for first-goal diagnosis remained zero for controller_server logs and bt_navigator logs because no allowed explore goal was executed before the non-explore fail-closed stop. The launcher/process output and ROS launch log are retained under the Phase129 log directory and process transcript; the maze_explorer stderr artifact records the first non-explore dispatch line.

Important maze_explorer stderr line:

```text
maze explorer sending corridor_alignment_staging goal #1 seq=1: x=1.701 y=0.074 yaw=1.566
```

## Guard and cleanup results

Final cleanup was performed after the runtime attempts. Final guard artifact:

- `log/phase129_instrumented_first_goal_timeout_diagnosis/phase129_final_guard.txt`

Final guard summary:

```text
phase129_final_process_guard: empty
phase129_final_protected_config_diff_guard: empty
phase129_final_pycache_guard: empty after cleanup
```

No Gazebo/RViz/Nav2/maze_explorer Phase129 runtime process remained after cleanup.

## Interpretation

Phase129 did not reproduce the Phase125 first `goal_kind=explore` timeout path. Instead, the current algorithm emitted a `corridor_alignment_staging` first dispatch under the visible stack, and the Phase129 runner stopped fail-closed to preserve the phase boundary.

Therefore the correct Phase129 conclusion is:

```text
INSUFFICIENT_TIMEOUT_EVIDENCE
```

The Phase127 diagnosis `FIRST_GOAL_TIMEOUT_LOCAL_COST_BLOCKED` remains a prior artifact-replay diagnosis. Phase129 did not strengthen, repair, or override it with a new timeout classification.

## Stop condition

Phase129 is complete and stopped before Phase130.

Do not proceed to Phase130 without explicit human acceptance and a new phase request.
