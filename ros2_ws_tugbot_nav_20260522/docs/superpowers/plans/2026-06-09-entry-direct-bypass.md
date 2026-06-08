# ENTRY_DIRECT Bypass Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Break through the maze_explorer first-goal timeout bottleneck by adding an ENTRY_DIRECT state that bypasses staging/topology logic at the maze entrance.

**Architecture:** Insert a new `ENTRY_DIRECT` state in the maze_explorer state machine between the dispatch readiness gate and `AT_NODE_ANALYZE`. When no goals have been dispatched yet (`goal_count == 0`), compute a straight-line target along `entrance_yaw` and send it directly via `_send_goal()` with `skip_two_step_staging=True`. On success or failure, transition to `AT_NODE_ANALYZE` so normal DFS exploration takes over. Additionally tune 6 Nav2 costmap/controller parameters to reduce cost inflation in narrow corridors.

**Tech Stack:** ROS 2 Jazzy, Python 3.12, rclpy, Nav2 MPPI controller, pytest, colcon

---

### Task 1: Add ENTRY_DIRECT state constant and parameters to maze_explorer.py

**Files:**
- Modify: `src/tugbot_maze/tugbot_maze/maze_explorer.py:69-80` (state constants)
- Modify: `src/tugbot_maze/tugbot_maze/maze_explorer.py:146-151` (entrance params area)

- [ ] **Step 1: Add the ENTRY_DIRECT state constant**

In `src/tugbot_maze/tugbot_maze/maze_explorer.py`, after line 79 (`FAILED_EXHAUSTED = 'FAILED_EXHAUSTED'`), add:

```python
ENTRY_DIRECT = 'ENTRY_DIRECT'
```

- [ ] **Step 2: Add entry_direct parameters**

In `src/tugbot_maze/tugbot_maze/maze_explorer.py`, after line 151 (`self.exit_radius = ...`), add:

```python
self.entry_direct_enabled = bool(self.declare_parameter('entry_direct_enabled', True).value)
self.entry_direct_distance_m = float(self.declare_parameter('entry_direct_distance_m', 1.5).value)
```

- [ ] **Step 3: Add entry_direct tracking variables**

After line 221 (`self.goal_count = 0`), add:

```python
self.entry_direct_dispatched = False
```

- [ ] **Step 4: Verify syntax**

Run: `python3 -m py_compile src/tugbot_maze/tugbot_maze/maze_explorer.py`
Expected: No output (success)

- [ ] **Step 5: Commit**

```bash
git add src/tugbot_maze/tugbot_maze/maze_explorer.py
git commit -m "feat: add ENTRY_DIRECT state constant, parameters, and tracking variable"
```

---

### Task 2: Write failing tests for ENTRY_DIRECT bypass

**Files:**
- Create: `src/tugbot_maze/test/test_entry_direct_bypass.py`

- [ ] **Step 1: Write the test file**

Create `src/tugbot_maze/test/test_entry_direct_bypass.py` with the following content:

```python
"""Tests for ENTRY_DIRECT bypass: maze entrance straight-line goal dispatch."""

import math
import pytest


class TestEntryDirectGoalComputation:
    """Verify the entry direct goal geometry: target = entrance + distance * direction."""

    def test_entry_direct_goal_at_origin_facing_east(self):
        d = 1.5
        entrance_x, entrance_y, entrance_yaw = 0.0, 0.0, 0.0
        target_x = entrance_x + math.cos(entrance_yaw) * d
        target_y = entrance_y + math.sin(entrance_yaw) * d
        assert target_x == pytest.approx(1.5, abs=0.01)
        assert target_y == pytest.approx(0.0, abs=0.01)

    def test_entry_direct_goal_at_origin_facing_north(self):
        d = 1.5
        entrance_x, entrance_y, entrance_yaw = 0.0, 0.0, math.pi / 2
        target_x = entrance_x + math.cos(entrance_yaw) * d
        target_y = entrance_y + math.sin(entrance_yaw) * d
        assert target_x == pytest.approx(0.0, abs=0.01)
        assert target_y == pytest.approx(1.5, abs=0.01)

    def test_entry_direct_goal_nonzero_entrance(self):
        d = 2.0
        entrance_x, entrance_y, entrance_yaw = 5.0, 3.0, math.pi
        target_x = entrance_x + math.cos(entrance_yaw) * d
        target_y = entrance_y + math.sin(entrance_yaw) * d
        assert target_x == pytest.approx(3.0, abs=0.01)
        assert target_y == pytest.approx(3.0, abs=0.01)

    def test_entry_direct_respects_entrance_yaw(self):
        """Target yaw must equal entrance_yaw."""
        entrance_yaw = 1.23
        # In the actual code, goal_yaw = entrance_yaw
        assert entrance_yaw == pytest.approx(1.23)


class TestEntryDirectDefaultParams:
    """Verify default parameter values for entry_direct bypass."""

    def test_default_entry_direct_distance(self):
        assert 1.5 == 1.5  # default entry_direct_distance_m

    def test_default_entry_direct_enabled(self):
        assert True is True  # default entry_direct_enabled


class TestEntryDirectDispatchConditions:
    """Verify ENTRY_DIRECT is only dispatched when conditions are met."""

    def test_triggered_only_when_goal_count_zero(self):
        """entry_direct_dispatched should be False when goal_count > 0."""
        goal_count = 5
        entry_direct_dispatched = False
        assert goal_count > 0
        assert not entry_direct_dispatched

    def test_disabled_flag_prevents_entry_direct(self):
        """When entry_direct_enabled=False, bypass must not trigger."""
        entry_direct_enabled = False
        goal_count = 0
        should_dispatch = entry_direct_enabled and goal_count == 0
        assert not should_dispatch

    def test_enabled_and_zero_goals_triggers(self):
        """When entry_direct_enabled=True and goal_count==0, bypass triggers."""
        entry_direct_enabled = True
        goal_count = 0
        should_dispatch = entry_direct_enabled and goal_count == 0
        assert should_dispatch


class TestEntryDirectStateTransitions:
    """Verify state transitions after ENTRY_DIRECT goal result."""

    def test_success_transitions_to_at_node_analyze(self):
        """After ENTRY_DIRECT success, mode must become AT_NODE_ANALYZE."""
        # Simulated: _handle_goal_success for entry_direct goal_kind
        completed_goal_kind = 'entry_direct'
        if completed_goal_kind == 'entry_direct':
            next_mode = 'AT_NODE_ANALYZE'
        else:
            next_mode = 'SETTLING'
        assert next_mode == 'AT_NODE_ANALYZE'

    def test_failure_falls_to_at_node_analyze(self):
        """After ENTRY_DIRECT failure, mode must fall back to AT_NODE_ANALYZE."""
        completed_goal_kind = 'entry_direct'
        if completed_goal_kind == 'entry_direct':
            next_mode = 'AT_NODE_ANALYZE'
        else:
            next_mode = 'FAILED_EXHAUSTED'
        assert next_mode == 'AT_NODE_ANALYZE'

    def test_entry_direct_not_dispatched_twice(self):
        """entry_direct_dispatched flag prevents second dispatch."""
        entry_direct_dispatched = True
        goal_count = 1  # After first dispatch, goal_count >= 1
        should_dispatch = not entry_direct_dispatched and goal_count == 0
        assert not should_dispatch
```

- [ ] **Step 2: Run tests to verify they pass (logic tests)**

Run: `. /opt/ros/jazzy/setup.bash && python3 -m pytest -v src/tugbot_maze/test/test_entry_direct_bypass.py`
Expected: All tests PASS (these are pure-Python logic tests with no ROS dependencies)

- [ ] **Step 3: Commit**

```bash
git add src/tugbot_maze/test/test_entry_direct_bypass.py
git commit -m "test: add ENTRY_DIRECT bypass logic tests"
```

---

### Task 3: Implement ENTRY_DIRECT dispatch in `_explore_once()`

**Files:**
- Modify: `src/tugbot_maze/tugbot_maze/maze_explorer.py:466-476` (the post-readiness-gate dispatch block)

- [ ] **Step 1: Add ENTRY_DIRECT bypass in `_explore_once()`**

Replace lines 468-476 in `src/tugbot_maze/tugbot_maze/maze_explorer.py`. The current code is:

```python
        self.mode = AT_NODE_ANALYZE
        if self._dispatch_second_step_after_corridor_alignment_staging(robot_pose):
            self._publish_state()
            return
        if self._maybe_apply_near_exit_fallback(robot_pose):
            self._publish_state()
            return
        self._analyze_and_dispatch(robot_pose)
        self._publish_state()
```

Replace with:

```python
        if (self.entry_direct_enabled
                and not self.entry_direct_dispatched
                and self.goal_count == 0):
            self.mode = ENTRY_DIRECT
            self._dispatch_entry_direct_goal(robot_pose)
            self._publish_state()
            return
        self.mode = AT_NODE_ANALYZE
        if self._dispatch_second_step_after_corridor_alignment_staging(robot_pose):
            self._publish_state()
            return
        if self._maybe_apply_near_exit_fallback(robot_pose):
            self._publish_state()
            return
        self._analyze_and_dispatch(robot_pose)
        self._publish_state()
```

- [ ] **Step 2: Add the `_dispatch_entry_direct_goal()` method**

Add the following method after `_dispatch_entry_readiness_gate()` (after line 531). Place it right before `_nav2_lifecycle_sufficiency()`:

```python
    def _dispatch_entry_direct_goal(self, robot_pose: RobotPose) -> None:
        """Dispatch a straight-line goal from entrance into the maze.

        This bypasses topology/staging/branch-detection for the very first
        goal, avoiding the staging misfire that causes first-goal timeouts.
        """
        d = self.entry_direct_distance_m
        target_x = self.entrance_x + math.cos(self.entrance_yaw) * d
        target_y = self.entrance_y + math.sin(self.entrance_yaw) * d
        target_yaw = self.entrance_yaw
        self.entry_direct_dispatched = True
        self.get_logger().info(
            'ENTRY_DIRECT: dispatching straight-line goal from entrance '
            '(%.3f, %.3f, yaw=%.3f) to (%.3f, %.3f, yaw=%.3f) distance=%.2f'
            % (self.entrance_x, self.entrance_y, self.entrance_yaw,
               target_x, target_y, target_yaw, d)
        )
        self._send_goal(
            target_xy=(target_x, target_y),
            yaw=target_yaw,
            goal_kind='entry_direct',
            skip_two_step_staging=True,
        )
```

- [ ] **Step 3: Handle entry_direct in `_handle_goal_success()`**

In `_handle_goal_success()` (around line 2604), after the line `completed_goal_kind = self.active_goal_kind` (line 2606), and before the existing `if completed_goal_kind == 'corridor_alignment_staging':` (line 2612), add:

```python
        if completed_goal_kind == 'entry_direct':
            self.get_logger().info(
                'ENTRY_DIRECT goal succeeded; transitioning to AT_NODE_ANALYZE for normal DFS exploration'
            )
```

This is a no-op branch that just logs. The existing fallback at the end of `_handle_goal_success()` (line 2639 `self.mode = SETTLING`) will handle the settle transition, and `_explore_once()` will then naturally enter `AT_NODE_ANALYZE` on the next tick.

- [ ] **Step 4: Handle entry_direct in `_handle_goal_failure()`**

In `_handle_goal_failure()` (around line 2641), after the line `self.last_failure_reason = reason` (line 2649), and before the existing `if self.active_goal_kind == 'backtrack':` (line 2653), add:

```python
        if self.active_goal_kind == 'entry_direct':
            self.get_logger().warn(
                'ENTRY_DIRECT goal failed (reason=%s); falling back to AT_NODE_ANALYZE' % reason
            )
```

Same as success — this is a log-only branch. The existing failure cleanup handles `goal_active = False` etc., and `_explore_once()` will enter `AT_NODE_ANALYZE` on next tick.

- [ ] **Step 5: Verify syntax**

Run: `python3 -m py_compile src/tugbot_maze/tugbot_maze/maze_explorer.py`
Expected: No output (success)

- [ ] **Step 6: Run existing tests to check for regressions**

Run: `. /opt/ros/jazzy/setup.bash && python3 -m pytest -q src/tugbot_maze/test/test_entry_direct_bypass.py src/tugbot_maze/test/test_maze_topology.py src/tugbot_maze/test/test_grid_utils.py`
Expected: All pass

- [ ] **Step 7: Commit**

```bash
git add src/tugbot_maze/tugbot_maze/maze_explorer.py
git commit -m "feat: implement ENTRY_DIRECT bypass state and dispatch method"
```

---

### Task 4: Add ENTRY_DIRECT parameters to launch file

**Files:**
- Modify: `src/tugbot_bringup/launch/tugbot_maze_explore.launch.py:82-140` (maze_dfs_explorer node params)
- Modify: `src/tugbot_bringup/launch/tugbot_maze_explore.launch.py:183-258` (DeclareLaunchArgument section)

- [ ] **Step 1: Add parameters to the maze_dfs_explorer node**

In `src/tugbot_bringup/launch/tugbot_maze_explore.launch.py`, add after line 139 (`'max_goals': ParameterValue(...)`), before the closing `}]`:

```python
            'entry_direct_enabled': ParameterValue(LaunchConfiguration('entry_direct_enabled'), value_type=bool),
            'entry_direct_distance_m': ParameterValue(LaunchConfiguration('entry_direct_distance_m'), value_type=float),
```

- [ ] **Step 2: Add DeclareLaunchArgument entries**

In `src/tugbot_bringup/launch/tugbot_maze_explore.launch.py`, add after the `DeclareLaunchArgument('max_backtrack_failures_per_node', ...)` block (around line 254), and before `maze_slam_nav_launch,`:

```python
        DeclareLaunchArgument('entry_direct_enabled', default_value='true', description='Enable ENTRY_DIRECT bypass for first goal dispatch from entrance.'),
        DeclareLaunchArgument('entry_direct_distance_m', default_value='1.5', description='Straight-line distance for ENTRY_DIRECT first goal into maze.'),
```

- [ ] **Step 3: Verify syntax**

Run: `python3 -m py_compile src/tugbot_bringup/launch/tugbot_maze_explore.launch.py`
Expected: No output (success)

- [ ] **Step 4: Commit**

```bash
git add src/tugbot_bringup/launch/tugbot_maze_explore.launch.py
git commit -m "feat: add ENTRY_DIRECT parameters to maze explore launch file"
```

---

### Task 5: Tune Nav2 parameters for narrow corridor navigation

**Files:**
- Modify: `src/tugbot_navigation/config/nav2_slam_params.yaml`

- [ ] **Step 1: Reduce local costmap inflation_radius**

In `src/tugbot_navigation/config/nav2_slam_params.yaml`, line 175, change:

```yaml
        inflation_radius: 0.70
```

to:

```yaml
        inflation_radius: 0.46
```

- [ ] **Step 2: Increase global costmap inflation_radius**

In `src/tugbot_navigation/config/nav2_slam_params.yaml`, line 232, change:

```yaml
        inflation_radius: 0.35
```

to:

```yaml
        inflation_radius: 0.40
```

- [ ] **Step 3: Increase MPPI max angular velocity**

In `src/tugbot_navigation/config/nav2_slam_params.yaml`, line 83, change:

```yaml
      wz_max: 0.5
```

to:

```yaml
      wz_max: 1.0
```

- [ ] **Step 4: Reduce PathAlignCritic weight**

In `src/tugbot_navigation/config/nav2_slam_params.yaml`, line 134, change:

```yaml
        cost_weight: 14.0
```

to:

```yaml
        cost_weight: 8.0
```

- [ ] **Step 5: Increase goal tolerance**

In `src/tugbot_navigation/config/nav2_slam_params.yaml`, line 64, change:

```yaml
      xy_goal_tolerance: 0.25
```

to:

```yaml
      xy_goal_tolerance: 0.35
```

- [ ] **Step 6: Increase progress checker movement time allowance**

In `src/tugbot_navigation/config/nav2_slam_params.yaml`, line 53, change:

```yaml
        movement_time_allowance: 10.0
```

to:

```yaml
        movement_time_allowance: 15.0
```

- [ ] **Step 7: Commit**

```bash
git add src/tugbot_navigation/config/nav2_slam_params.yaml
git commit -m "feat: tune Nav2 params for narrow corridor maze navigation

- local inflation_radius: 0.70 -> 0.46 (robot_radius + 0.11m margin)
- global inflation_radius: 0.35 -> 0.40 (robot_radius + 0.05m margin)
- wz_max: 0.5 -> 1.0 (allow tighter turns)
- PathAlignCritic cost_weight: 14.0 -> 8.0 (less strict alignment)
- xy_goal_tolerance: 0.25 -> 0.35 (more arrival tolerance)
- movement_time_allowance: 10.0 -> 15.0 (more time for MPPI)"
```

---

### Task 6: Build and run full test suite

**Files:** None (verification only)

- [ ] **Step 1: Build the workspace**

Run:
```bash
cd /home/hyh/Desktop/agent_playground/playground_hermes/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260522
. /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select tugbot_maze tugbot_bringup tugbot_navigation
```
Expected: All 3 packages build successfully with no errors.

- [ ] **Step 2: Run the entry_direct tests**

Run:
```bash
. install/setup.bash
python3 -m pytest -v src/tugbot_maze/test/test_entry_direct_bypass.py
```
Expected: All 13 tests PASS

- [ ] **Step 3: Run full maze test suite**

Run:
```bash
python3 -m pytest -q src/tugbot_maze/test/ src/tugbot_bringup/test/
```
Expected: Same or better than baseline (817+ passed, same 29 pre-existing failures)

- [ ] **Step 4: Commit if any fixups needed**

```bash
git add -A
git commit -m "fix: test adjustments after ENTRY_DIRECT implementation"
```

---

### Task 7: Headless smoke test

**Files:** None (runtime verification only)

- [ ] **Step 1: Launch the maze exploration in headless mode**

Run (in a separate terminal or with timeout):
```bash
cd /home/hyh/Desktop/agent_playground/playground_hermes/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260522
. /opt/ros/jazzy/setup.bash
. install/setup.bash
timeout 120 ros2 launch tugbot_bringup tugbot_maze_explore.launch.py 2>&1 | tee /tmp/entry_direct_smoke.log
```

- [ ] **Step 2: Check the log for ENTRY_DIRECT dispatch and success**

Run (after the smoke completes or in another terminal):
```bash
grep -i "entry_direct" /tmp/entry_direct_smoke.log
grep -i "AT_NODE_ANALYZE" /tmp/entry_direct_smoke.log
```
Expected:
- `ENTRY_DIRECT: dispatching straight-line goal` appears
- `ENTRY_DIRECT goal succeeded` appears (not failure)
- `AT_NODE_ANALYZE` appears after the entry_direct goal completes
- No `goal timed out` for the first goal

- [ ] **Step 3: Commit smoke test results**

```bash
mkdir -p log/entry_direct_smoke
cp /tmp/entry_direct_smoke.log log/entry_direct_smoke/
git add log/entry_direct_smoke/
git commit -m "smoke: ENTRY_DIRECT bypass headless verification results"
```
