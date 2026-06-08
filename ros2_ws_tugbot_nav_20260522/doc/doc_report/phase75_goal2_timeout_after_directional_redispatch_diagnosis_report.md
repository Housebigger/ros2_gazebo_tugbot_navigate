# Phase75: Goal2 Timeout Diagnosis After Directional Re-dispatch

Status: completed / stopped at Phase75.

Classification: `GOAL2_TIMEOUT_LOCAL_COST_RECOVERY_LOOP`

Reason: Nav2 feedback/logs show recoveries or Failed to make progress while runtime footprint/front wedge local cost is high/lethal.

## Scope and guardrails

This phase is offline diagnosis only, reusing Phase74 `replay_02` artifacts. It does not tune Nav2/MPPI/controller, does not tune inflation/robot_radius/clearance_radius_m/map threshold, does not change branch scoring, does not change centerline gate, does not change the directional readiness override, does not change fallback/terminal acceptance, and does not claim autonomous exploration success and does not claim exit success.

Phase76 not entered.

## Source artifacts

- Phase74 replay: `log/phase74_directional_local_costmap_readiness_gate_validation/replay_02`
- Phase74 classification: `DIRECTIONAL_GATE_ENABLES_GOAL2_DISPATCH`
- Analyzer output: `log/phase75_goal2_timeout_after_directional_redispatch_diagnosis/phase75_goal2_timeout_after_directional_redispatch_diagnosis.json`
- Analyzer: `tools/analyze_phase75_goal2_timeout_after_directional_redispatch_diagnosis.py`
- Tests: `src/tugbot_maze/test/test_phase75_goal2_timeout_after_directional_redispatch_diagnosis.py`

## Goal2 dispatch-to-timeout timeline

- Goal2 dispatch observed: `True`
- Dispatch elapsed: 9.633 s
- Outcome: `timeout`
- Outcome elapsed: 55.419 s
- Duration: 45.786 s
- Effective timeout: 45.000 s

Mode transitions:
- t=2.620s mode=WAIT_FOR_MAP goal_count=0 success=0 failure=0 active_goal=None last_failure=None nav2_status=None
- t=3.419s mode=WAIT_FOR_DISPATCH_ENTRY_READINESS goal_count=0 success=0 failure=0 active_goal=None last_failure=None nav2_status=None
- t=5.577s mode=NAVIGATING goal_count=1 success=0 failure=0 active_goal=1 last_failure=None nav2_status=None
- t=8.419s mode=SETTLING goal_count=1 success=1 failure=0 active_goal=None last_failure=None nav2_status=4
- t=9.637s mode=NAVIGATING goal_count=2 success=1 failure=0 active_goal=2 last_failure=None nav2_status=4
- t=55.423s mode=SETTLING goal_count=2 success=1 failure=1 active_goal=None last_failure=goal_timeout nav2_status=4
- t=57.472s mode=WAIT_FOR_DISPATCH_ENTRY_READINESS goal_count=2 success=1 failure=1 active_goal=None last_failure=goal_canceled_after_timeout nav2_status=5

## Goal2 target / pose evidence

- Original target: `[2.058503376259287, 1.0236490342193865]`
- Refined target: `[2.058503376259287, 1.0236490342193865]`
- Actual dispatched target: `[2.058503376259287, 1.0236490342193865]`
- Dispatch pose: `[2.307321326591473, 0.02930773008823818, 0.023884391949891]`
- Final pose: `[2.426872326194781, 1.0327763735179467, 1.6075951571994673]`
- Target changed by centerline refinement: `false` (original/refined/actual are identical)

## Target wall-risk evidence

The timeout is not classified as target-too-close-to-wall:

- target_clearance_m: 0.851
- dispatch_target_local_cost: 0.000
- dispatch_target_local_cost_max_radius: 46.000
- runtime target cell final value: 0.000
- runtime target radius max final: 54.000
- target footprint max lethal count: 0
- target footprint max high-cost count: 0
- target_too_close_to_wall: `False`

Interpretation: the selected Goal2 target itself stayed locally clear in Phase74 replay_02. The stronger evidence is around the robot/front wedge during execution, not target-cell occupancy.

## Trajectory, tolerance, and progress evidence

- Odom pose samples: 1299
- Path length: 1.899 m
- Straight-line displacement: 1.009 m
- Distance-to-target first/final/min: 1.023 / 0.368 / 0.368 m
- Distance-to-target improvement: 0.655 m
- Last 10 s motion: 0.025 m
- Last 10 s stalled: `True`

Goal tolerance band:

- xy_goal_tolerance_m: 0.250
- yaw_goal_tolerance_rad: 0.250
- final_xy_error_m: 0.368
- final_yaw_error_to_branch_rad: 0.013
- inside_xy_tolerance: `False`
- inside_yaw_tolerance_to_branch: `True`
- near_goal_but_outside_xy_tolerance: `True`

Interpretation: near-goal tolerance is a contributing symptom: final XY error is about 0.368 m, outside the 0.25 m XY tolerance, while yaw is within the branch-angle tolerance proxy. However, this phase classifies the stronger causal subtype as local-cost/progress-recovery loop because Nav2 produced explicit progress failures/recoveries and the robot/front wedge carried high/lethal cost evidence.

## cmd_vel evidence

- cmd_vel samples: 1870
- nonzero command ratio: 0.516
- near-zero command ratio: 0.484
- last_10s_nonzero_command_ratio: 0.000
- last_10s_all_near_zero: `True`
- linear_x_abs max/mean/final: 0.186 / 0.041 / 0.000
- angular_z_abs max/mean/final: 0.105 / 0.036 / 0.000

Interpretation: the controller did publish commands for about half of the samples, so this is not a pure no-cmd_vel failure. In the final 10 s, commands were effectively zero while odom motion was almost stopped.

## Nav2 feedback / recovery / result evidence

- Feedback samples: 4579
- Nonzero distance_remaining first/final/min/max: 1.052 / 0.356 / 0.356 / 1.094 m
- Nonzero distance_remaining improvement: 0.696 m
- number_of_recoveries values: `[0, 1, 3]`
- number_of_recoveries max: 3
- Failed to make progress count: 2
- FollowPath abort count: 2
- Costmap clear count: 3
- BT goal canceled count: 5
- Outcome result reason: `goal_timeout`
- Cancel result reason: `goal_canceled_after_timeout`

Interpretation: replay_02 Goal2 entered a Nav2 progress-failure/recovery pattern before maze_explorer timeout/cancel. This is the main discriminator for `GOAL2_TIMEOUT_LOCAL_COST_RECOVERY_LOOP`.

## Local cost / footprint / front wedge evidence

Target local cost stayed low:

- target cell value final: 0.000
- target radius max final: 54.000

Robot footprint became high-risk:

- footprint samples: 78
- footprint max cost observed: 99.000
- footprint max lethal count observed: 53
- footprint max high-cost count observed: 79
- outcome timeout footprint max: 99.000
- outcome timeout footprint lethal count: 48

Front wedge became blocked/high-risk:

- front wedge samples: 78
- front wedge max cost observed: 100.000
- front wedge max lethal count observed: 195
- front wedge max high-cost count observed: 231
- front wedge clearance min/final: 0.050 / 0.050 m
- outcome timeout front wedge clearance: 0.049 m
- outcome timeout front wedge cost max: 99.000
- front_wedge_blocked: `True`

Interpretation: Goal2 timeout is better explained by execution-time local-cost/footprint/front-wedge obstruction around the robot than by target-cell risk.

## Global plan / collision monitor / age evidence

- Global plan samples: 45
- Global plan point count first/final/min/max: 42 / 13 / 13 / 43
- Collision monitor samples: 0
- Dispatch map age: 0.900 s
- Dispatch scan age: 0.096 s
- Dispatch local costmap age: 0.207 s
- Dispatch TF age: null s (not present in artifact)
- Timeout local cost sample age: 0.009 s
- Timeout robot in local costmap bounds: `True`

Interpretation: available ages are fresh enough to use the cost/scan/map evidence. TF age is not explicitly recorded in this Phase74 artifact, so TF timing cannot be a positive root-cause claim.

## Decision table

- `GOAL2_TIMEOUT_TARGET_TOO_CLOSE_TO_WALL`: not selected. Target clearance/cell/radius/target-footprint evidence does not show high/lethal target risk.
- `GOAL2_TIMEOUT_NEAR_GOAL_TOLERANCE_ORIENTATION`: not selected as primary. Final pose is near but outside XY tolerance; yaw proxy is within tolerance. This is a symptom, not the strongest causal evidence.
- `GOAL2_TIMEOUT_LOCAL_COST_RECOVERY_LOOP`: selected. Recoveries, Failed-to-make-progress logs, FollowPath aborts, costmap clears, high/lethal robot footprint, and blocked front wedge align.
- `GOAL2_TIMEOUT_PROGRESS_INSUFFICIENT`: not selected as primary. Progress did occur, but final stall is coupled with the stronger Nav2/local-cost recovery evidence.
- `INSUFFICIENT_EVIDENCE`: not selected. Phase74 replay_02 contains dispatch, timeout, odom, cmd_vel, feedback, local cost, global plan, and log evidence.

## Verification

- Phase75 focused pytest: `5 passed in 0.16s`
- Analyzer py_compile: passed
- Analyzer execution: classification `GOAL2_TIMEOUT_LOCAL_COST_RECOVERY_LOOP`
- `colcon build --symlink-install --packages-select tugbot_maze tugbot_bringup`: passed, 2 packages finished
- Nav2 config diff: empty
- Post-check ROS/Gazebo/Nav2 process query: empty

## Final Phase75 conclusion

Goal2 did dispatch after the Phase74 directional readiness override, but it timed out during execution. The target itself was not too close to the wall by the recorded target-clearance/target-cost evidence. The robot approached the goal corridor, stalled outside the XY tolerance band, and Nav2 entered a progress-failure/recovery loop with high/lethal robot footprint and front-wedge local-cost evidence. Therefore Phase75 classifies replay_02 as:

`GOAL2_TIMEOUT_LOCAL_COST_RECOVERY_LOOP`

Stop here. Phase76 not entered.
