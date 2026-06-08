# ENTRY_DIRECT Bypass: Maze Entrance Reliability Fix

Date: 2026-06-09
Status: APPROVED
Target: Break through maze_explorer first-goal timeout bottleneck

## Problem

The tugbot robot cannot reliably enter the maze. The root cause is a three-way interaction:

1. **Staging misfire at entrance**: maze_explorer detects 2 open directions at the entrance, classifies it as a "corridor", dispatches a `corridor_alignment_staging` goal instead of an exploration goal. Staging is rejected due to missing two-side wall evidence -> no goal dispatched -> timeout.

2. **Local costmap cost explosion**: inflation_radius (0.70m) is 2x the robot radius (0.35m). Narrow entrance corridors are marked as near-impassable (cost 99-100/100).

3. **MPPI controller over-alignment**: PathAlignCritic weight 14.0 forces strict path alignment in tight spaces, combined with high costs -> controller stalls.

## Success Criteria (MVP)

- Robot departs from entrance and reaches first direct goal without timeout
- After reaching first goal, transitions to normal DFS exploration mode (AT_NODE_ANALYZE)
- Robot continues moving along corridor with map updates
- No crash / no exception

## Design

### Section 1: ENTRY_DIRECT State Bypass

Insert a new state between `WAIT_FOR_DISPATCH_ENTRY_READINESS` and `AT_NODE_ANALYZE`:

```
WAIT_FOR_DISPATCH_ENTRY_READINESS
  -> if goal_count == 0 and entry_direct_enabled:
       ENTRY_DIRECT -> compute straight-line goal -> NAVIGATING
       -> success -> AT_NODE_ANALYZE (normal DFS takes over)
       -> failure -> AT_NODE_ANALYZE (existing logic as fallback)
  -> else:
       AT_NODE_ANALYZE (existing flow)
```

**New code in `maze_explorer.py`:**

1. State constant: `ENTRY_DIRECT = 'entry_direct'`
2. In `_explore_once()`: after readiness gate passes, if `self._goal_count == 0`, set `mode = ENTRY_DIRECT`
3. New method `_dispatch_entry_direct_goal()`:
   - Target = `(entrance_x + cos(entrance_yaw) * d, entrance_y + sin(entrance_yaw) * d, entrance_yaw)`
   - `d` = `entry_direct_distance_m` parameter (default 1.5m)
   - Calls `_send_goal()` directly, bypasses topology/staging/branch detection
   - Sets mode to `NAVIGATING`
4. Goal result callback: entry_direct success/failure both transition to `AT_NODE_ANALYZE`

**New parameters:**
- `entry_direct_distance_m`: float, default 1.5
- `entry_direct_enabled`: bool, default True

### Section 2: Nav2 Parameter Tuning

File: `src/tugbot_navigation/config/nav2_slam_params.yaml`

| Parameter | Old | New | Rationale |
|---|---|---|---|
| local_costmap inflation_radius | 0.70 | 0.46 | Robot radius + 0.11m margin (was 2x radius) |
| global_costmap inflation_radius | 0.35 | 0.40 | Robot radius + 0.05m minimum margin |
| wz_max (SLAM mode) | 0.5 | 1.0 | Tight turns need angular velocity (still below standard 1.9) |
| PathAlignCritic cost_weight | 14.0 | 8.0 | Less strict alignment, more flexible obstacle avoidance |
| goal_checker xy_goal_tolerance | 0.25 | 0.35 | More arrival tolerance in tight spaces |
| progress_checker movement_time_allowance | 10.0 | 15.0 | More time for MPPI to find path in uncertain map |

**Unchanged:**
- robot_radius: 0.35m (physical dimension)
- vx_max: 0.5m/s (reasonable)
- CostCritic critical_cost: 300 (safety floor)
- collision_cost: 1000000 (collision penalty)

### Section 3: Testing

**New test file**: `src/tugbot_maze/test/test_entry_direct_bypass.py`

| Test Case | Verifies |
|---|---|
| test_entry_direct_goal_computation | Straight-line target = entrance + distance * direction |
| test_entry_direct_respects_entrance_yaw | Target yaw matches entrance yaw |
| test_entry_direct_default_params | entry_direct_distance_m=1.5, enabled=True |
| test_entry_direct_disabled_falls_through | enabled=False skips ENTRY_DIRECT |
| test_entry_direct_goal_count_zero_only | goal_count > 0 does not trigger ENTRY_DIRECT |
| test_entry_direct_success_transitions_to_analyze | Success -> mode = AT_NODE_ANALYZE |
| test_entry_direct_failure_falls_to_analyze | Failure -> mode = AT_NODE_ANALYZE fallback |

**Headless smoke verification:**
```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch tugbot_bringup tugbot_maze_explore.launch.py
```

## Files Changed

| File | Type | Lines |
|---|---|---|
| src/tugbot_maze/tugbot_maze/maze_explorer.py | New state + method | ~80 |
| src/tugbot_navigation/config/nav2_slam_params.yaml | Param tuning | 6 values |
| src/tugbot_maze/test/test_entry_direct_bypass.py | New tests | ~120 |
| src/tugbot_bringup/launch/tugbot_maze_explore.launch.py | New params | ~5 |

## Out of Scope

- Fixing 29 failing Phase32-33 tests (world generation, unrelated)
- Full maze traversal to exit
- High-reliability multi-run validation
- Refactoring existing staging/topology logic
