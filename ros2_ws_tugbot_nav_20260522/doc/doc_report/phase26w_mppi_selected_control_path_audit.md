# Phase26W MPPI selected-control path audit

Date: 2026-05-26
Workspace: `/home/hyh/Desktop/agent_playground/playground_hermes/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260522`

## Goal

Phase26W stays diagnostics-only. It does not repeat Phase26V spatial joins. It audits installed/source-level Nav2 MPPI artifacts to locate the selected-control path and determine whether installed artifacts can explain why `cmd_vel_nav` becomes near-zero when sampled `/optimal_trajectory` and `/transformed_global_plan` do not intersect captured local high-cost points.

Forbidden in this phase:

- Enter Phase27
- Modify branch selection
- Modify Nav2/controller parameters
- Promote or reject candidate `CostCritic.cost_weight=2.75`
- Start a Gazebo/Nav2 run

## Added tool

Added:

- `tools/audit_phase26w_mppi_selected_control_path.py`

Command used:

```bash
python3 tools/audit_phase26w_mppi_selected_control_path.py \
  --ros-prefix /opt/ros/jazzy \
  --workspace-root . \
  --output-json log/phase26w_mppi_selected_control_path_audit.json
```

Artifact:

- `log/phase26w_mppi_selected_control_path_audit.json`

The audit reads:

- `/opt/ros/jazzy/include/nav2_mppi_controller/controller.hpp`
- `/opt/ros/jazzy/include/nav2_mppi_controller/optimizer.hpp`
- `/opt/ros/jazzy/include/nav2_mppi_controller/tools/trajectory_visualizer.hpp`
- `/opt/ros/jazzy/include/nav2_mppi_controller/models/control_sequence.hpp`
- `/opt/ros/jazzy/include/nav2_mppi_controller/models/optimizer_settings.hpp`
- `/opt/ros/jazzy/include/nav2_mppi_controller/models/trajectories.hpp`
- `/opt/ros/jazzy/include/nav2_mppi_controller/models/constraints.hpp`
- `/opt/ros/jazzy/include/nav2_mppi_controller/motion_models.hpp`
- `/opt/ros/jazzy/share/nav2_mppi_controller/{mppic.xml,critics.xml,package.xml}`
- `/opt/ros/jazzy/lib/libmppi_controller.so` strings
- workspace Nav2 MPPI YAML config values

No runtime was started.

## TDD

Added:

- `src/tugbot_maze/test/test_phase26w_mppi_selected_control_audit.py`

RED result:

- `2 failed`

Expected failure:

- `tools/audit_phase26w_mppi_selected_control_path.py` did not exist.

GREEN result:

- `2 passed in 0.21s`

## Audit findings

### Selected-control path located

The selected-control-to-command path is located at declaration/symbol level:

1. Nav2 calls:
   - `nav2_mppi_controller::MPPIController::computeVelocityCommands(...)`

2. Controller owns:
   - `Optimizer optimizer_`

3. Optimizer entrypoint:
   - `mppi::Optimizer::evalControl(...)`

4. Optimizer declares the core path:
   - `prepare(...)`
   - `generateNoisedTrajectories()`
   - `applyControlSequenceConstraints()`
   - `critic_manager_.evalTrajectoriesScores(...)`
   - `updateControlSequence()`
   - `getControlFromSequenceAsTwist(...)`

5. Final selected command conversion point:
   - `mppi::Optimizer::getControlFromSequenceAsTwist(const builtin_interfaces::msg::Time & stamp)`

Conclusion:

- The near-zero `cmd_vel_nav` production point is located as:
  - `Optimizer::getControlFromSequenceAsTwist` returning `TwistStamped` from optimized `control_sequence_`.

### Near-zero reason not observable from installed artifacts

The installed artifacts locate the production point, but they do not reveal why the selected control is near-zero.

Reason:

- Installed headers expose declarations, not function bodies, for:
  - `updateControlSequence()`
  - `getControlFromSequenceAsTwist()`
  - `fallback(bool fail)`
- Library strings expose symbols and coarse strings, but not per-cycle values:
  - `Optimizer fail to compute path`
  - `fallback`
  - `retry_attempt_limit`
  - `getControlFromSequenceAsTwist`
  - `evalControl`
- No per-cycle values are available for:
  - optimized `control_sequence_.vx/vy/wz`
  - selected first control
  - softmax/weight distribution
  - critic totals/min/max/finite count
  - fallback/retry path taken
  - exception/no-valid-control reason

So the audit identifies where near-zero is produced, but not why.

### Relationship of topics to final cmd_vel

Phase26W clarifies that the published visualization topics are not equivalent to final command publication.

`/optimal_trajectory`:

- Source relation: `TrajectoryVisualizer::add(const xt::xtensor<float, 2> & trajectory, ...)`
- It visualizes `Optimizer::getOptimizedTrajectory()`.
- It is not the final `cmd_vel` publish path.
- It can be near-stationary while the reason for the selected control remains hidden in optimizer/control-sequence internals.

`/trajectories`:

- Source relation: `TrajectoryVisualizer::add(const models::Trajectories & trajectories, ...)`
- It visualizes generated candidate trajectories.
- It is not the selected control and not a raw critic/cost breakdown.

`/transformed_global_plan`:

- Source relation: `TrajectoryVisualizer::visualize(const nav_msgs::msg::Path & plan)`
- It visualizes the transformed reference plan passed to visualization.
- It is not selected control.

Final command origin:

- `Optimizer::getControlFromSequenceAsTwist`

### Control constraints / fallback / failure path evidence

Declared or string evidence exists for:

- `models::ControlConstraints`
- `applyControlSequenceConstraints()`
- motion-model `applyConstraints(...)`
- `fallback(bool fail)`
- `retry_attempt_limit`
- `Optimizer fail to compute path`

Runtime/config names observed in workspace configs include:

- `batch_size`
- `time_steps`
- `trajectory_step`
- `time_step`
- `visualize`
- `vx_max`
- `vx_min`
- `vy_max`
- `wz_max`
- `temperature`
- `iteration_count`
- `model_dt`
- `cost_weight`

But installed artifacts still do not expose per-cycle values or branch taken.

### MarkerArray marker_count=7656 explanation

Phase26W explains the recurring `/trajectories` MarkerArray marker count.

From workspace config:

- `batch_size`: 2000
- `time_steps`: 56
- `trajectory_step`: 5
- `time_step`: 3

Candidate trajectory marker grid:

- `ceil(2000 / 5) * ceil(56 / 3)`
- `400 * 19 = 7600`

Observed marker count:

- 7656

Difference:

- 56

Interpretation:

- 7656 is consistent with 7600 candidate trajectory markers plus one optimal-trajectory marker per time step.
- It is not the raw trajectory count.
- It is not the selected command count.
- It should not be interpreted as 7656 independent selected controls.

## Phase26W conclusion

Phase26W answers one question and leaves one critical question open.

Answered:

- The selected-control near-zero production point is located:
  - `Optimizer::getControlFromSequenceAsTwist` converts optimized `control_sequence_` to the returned `TwistStamped`.

Not answered:

- Why `control_sequence_` produces near-zero values in the Phase26V windows.

The installed Jazzy artifacts are insufficient to explain the near-zero reason because they lack per-cycle optimizer/control-sequence/cost/fallback values.

Therefore:

- Phase27 remains blocked.
- Phase26X instrumentation is needed if we want to explain the selected-control near-zero reason.

## Phase26X diagnostics-only instrumentation proposal

Do not implement control-behavior changes. If proceeding, Phase26X should be a minimal diagnostics overlay/instrumentation run.

Recommended instrumentation scope:

- diagnostics-only near-zero cycles
- bounded JSONL output
- no branch-selection changes
- no Nav2/controller parameter tuning
- no candidate promotion/rejection

Minimal instrumentation points:

1. `MPPIController::computeVelocityCommands`
   - log robot speed
   - transformed plan size
   - returned twist
   - timestamp used for visualization/command correlation
   - only for near-zero cycles or bounded first-cmd-near-zero windows

2. `Optimizer::evalControl`
   - log retry/fallback result
   - exception/failure state if any
   - cost finite count/min/max if available
   - whether fallback was used

3. `Optimizer::updateControlSequence`
   - log first few optimized `control_sequence_.vx/vy/wz` entries
   - log weighted delta summary if available

4. `Optimizer::getControlFromSequenceAsTwist`
   - log exact `vx/vy/wz` converted to returned `TwistStamped`
   - log near-zero threshold classification

5. `TrajectoryVisualizer` / optimized trajectory correlation
   - correlate optimized trajectory first displacement with returned twist timestamp

## Phase27 gate

Phase27 remains blocked.

Reason:

- Phase26V proved sampled paths do not intersect captured high-cost choke in bounded baseline/candidate cases.
- Phase26W located the selected-control production point.
- But Phase26W did not identify the selected-control near-zero reason.
- Installed artifacts cannot expose per-cycle optimizer internals needed to explain near-zero.

Still prohibited:

- Branch-selection changes
- Nav2/controller parameter changes
- Candidate promotion
- Candidate rejection
- Phase27 entry

## Verification

Focused Phase26P/Q/R/S/T/U/V/W tests:

- `23 passed in 0.72s`

Phase24/25/26 regression:

- `96 passed in 1.95s`

Static:

- `py_compile` passed for Phase26W and related Phase26 tools.

Build:

- `colcon build --symlink-install --packages-select tugbot_bringup tugbot_navigation tugbot_maze`
- `3 packages finished [0.93s]`

Colcon test:

- `colcon test --packages-select tugbot_maze`
- `167 passed in 2.36s`
- `colcon test-result --verbose`: `0 errors, 0 failures`

Cleanup:

- No residual ROS/Gazebo/Nav2/recorder processes found.
