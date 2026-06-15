# Reactive Wall-Following Maze Solver Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Give the Tugbot a provably-correct, wedge-proof autonomous maze solver — a ROS-free reactive wall-follower (right/left-hand rule), validated offline against the real maze graph, that reliably reaches the exit (≥ 4/5 `EXIT_REACHED`).

**Architecture:** A thin ROS node (`wall_follow_solver`) reduces `/scan` to three sector distances and delegates all control to a pure, deterministic policy (`wall_follower.WallFollower`). A test-only 2-D maze simulator (`maze_sim`) raycasts the *real* `20260528` wall segments so a unit test can prove the policy reaches the exit and pick the faster hand. Node-support logic (exit check, entry, stall watchdog) lives in a separate ROS-free module (`wall_follow_control`) so it is unit-testable. SLAM stays running only for `map→base_link` pose (exit check); Nav2/costmap/frontier are dropped from the control loop.

**Tech Stack:** Python 3.12, ROS 2 Jazzy (`rclpy`, `tf2_ros`, `sensor_msgs`, `geometry_msgs`, `std_msgs`), `numpy` (sim only), `PyYAML` (sim only), `pytest`.

---

> **Post-implementation note (2026-06-15):** the inline `WallFollower` defaults shown in Tasks 2 and 7 below (`corner_v=0.18 / turn_w=0.7 / w_max=0.8`) were the initial proposal; the **shipped** robust values are `corner_v=0.45, corner_w=0.6, turn_w=1.0, w_max=1.0` (orbit radius ≈ 0.75 m, all angular outputs clamped to `w_max`), `follow_side='left'` — validated by a 54/54 start-perturbation sweep and a 5/5 Gazebo reliability batch. See the design spec's "Implementation outcome" section for why.

## Conventions for this plan

- **`$WS`** = the ROS workspace root:
  `/home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260614`
- **Repo root** (git toplevel) = `$WS/..` (the `ros2_gazebo_tugbot_navigate` directory). The branch is `wall-following-solver`.
- Unless a path starts with `docs/` or `$WS/..`, **all paths are relative to `$WS`.**
- **Pure tests** (Tasks 1–6) need NO ROS sourced — the modules under test import only `math`/`dataclasses`/`enum`/`numpy`/`yaml`. Run them with plain `python3 -m pytest` from `$WS/src/tugbot_maze`.
- **Angle convention:** robot frame, forward = 0 rad, left = `+π/2`, right = `−π/2`. `+w` (angular velocity) = turn left / counter-clockwise (CCW).
- **`+v`** = forward.

## File Structure

| File | Responsibility |
|---|---|
| `src/tugbot_maze/tugbot_maze/wall_follower.py` (create) | ROS-free reactive core: `Sectors`, `Command`, `State`, `sectorize()`, `WallFollower` policy. Pure, deterministic, no ROS. |
| `src/tugbot_maze/tugbot_maze/wall_follow_control.py` (create) | ROS-free node-support helpers: `exit_reached()`, `entering_done()`, `StallWatchdog`. Pure. |
| `src/tugbot_maze/tugbot_maze/maze_sim.py` (create) | Test-only 2-D raycaster + unicycle integrator over the real maze segments. Uses `numpy`/`yaml`. Not imported by any node. |
| `src/tugbot_maze/tugbot_maze/wall_follow_solver.py` (create) | Thin ROS node: subscriptions, TF pose, 10 Hz control timer, wiring `sectorize`→`WallFollower`→`/cmd_vel_nav`, entry + stall watchdog + exit check. |
| `src/tugbot_maze/test/test_wall_follower.py` (create) | Unit tests for `sectorize` + `WallFollower` branches. |
| `src/tugbot_maze/test/test_maze_sim.py` (create) | Unit tests for the raycaster + integrator/collision. |
| `src/tugbot_maze/test/test_wall_follow_maze_sim.py` (create) | Integration guarantee proof: real policy + real maze → reaches exit (both hands). |
| `src/tugbot_maze/test/test_wall_follow_control.py` (create) | Unit tests for `exit_reached`/`entering_done`/`StallWatchdog`. |
| `src/tugbot_maze/test/test_wall_follow_solver_smoke.py` (create) | Import smoke test for the node (skips if `rclpy` unavailable). |
| `src/tugbot_maze/setup.py` (modify) | Add `wall_follow_solver` console_scripts entry point. |
| `src/tugbot_bringup/launch/tugbot_maze_explore.launch.py` (modify) | Add `wall_follow_solver` node gated by `explorer_type:=wall_follower`, a `follow_side` arg, and include it in the 13 s `TimerAction`. |
| `tools/run_wall_follower_maze.sh` (create) | Single headless run with `explorer_type:=wall_follower`. |
| `tools/run_wall_follower_reliability.sh` (create) | N-run reliability tally (target ≥ 4/5). |

---

## Task 1: `wall_follower` sector abstraction + `sectorize`

**Files:**
- Create: `src/tugbot_maze/tugbot_maze/wall_follower.py`
- Test: `src/tugbot_maze/test/test_wall_follower.py`

- [ ] **Step 1: Write the failing tests**

Create `src/tugbot_maze/test/test_wall_follower.py`:

```python
import math
import pytest
from tugbot_maze.wall_follower import Sectors, Command, State, sectorize


# ---- sectorize -----------------------------------------------------------

def _ranges_at(angle_to_range, n=360, max_range=12.0):
    """Build a ranges list of length n over [-pi, pi); angle_to_range(ang)->meters."""
    amin = -math.pi
    ainc = 2 * math.pi / n
    return [angle_to_range(amin + i * ainc) for i in range(n)], amin, ainc


def test_sectorize_picks_min_in_each_window_right_hand():
    # Wall close on the RIGHT (-pi/2) at 0.5 m, open elsewhere at 5 m.
    def a2r(ang):
        return 0.5 if abs(math.atan2(math.sin(ang + math.pi / 2),
                                     math.cos(ang + math.pi / 2))) <= math.radians(10) else 5.0
    ranges, amin, ainc = _ranges_at(a2r)
    s = sectorize(ranges, amin, ainc, 'right')
    assert s.side == pytest.approx(0.5, abs=1e-6)
    assert s.front == pytest.approx(5.0, abs=1e-6)


def test_sectorize_front_window_sees_obstacle_ahead():
    def a2r(ang):
        return 0.8 if abs(ang) <= math.radians(10) else 5.0
    ranges, amin, ainc = _ranges_at(a2r)
    s = sectorize(ranges, amin, ainc, 'right')
    assert s.front == pytest.approx(0.8, abs=1e-6)


def test_sectorize_left_hand_reads_left_side():
    # Wall close on the LEFT (+pi/2); right side open.
    def a2r(ang):
        return 0.6 if abs(math.atan2(math.sin(ang - math.pi / 2),
                                     math.cos(ang - math.pi / 2))) <= math.radians(10) else 5.0
    ranges, amin, ainc = _ranges_at(a2r)
    s = sectorize(ranges, amin, ainc, 'left')
    assert s.side == pytest.approx(0.6, abs=1e-6)


def test_sectorize_sanitizes_inf_nan_and_nonpositive():
    ranges = [float('inf'), float('nan'), 0.0, -1.0]  # all invalid
    amin, ainc = -math.pi, math.pi / 2
    s = sectorize(ranges, amin, ainc, 'right', max_range=9.0)
    assert s.front == 9.0 and s.side == 9.0 and s.front_side == 9.0


def test_sectorize_clamps_to_max_range():
    ranges, amin, ainc = _ranges_at(lambda ang: 99.0)
    s = sectorize(ranges, amin, ainc, 'right', max_range=12.0)
    assert s.front == 12.0
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cd $WS/src/tugbot_maze && python3 -m pytest test/test_wall_follower.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'tugbot_maze.wall_follower'`.

- [ ] **Step 3: Write the minimal implementation**

Create `src/tugbot_maze/tugbot_maze/wall_follower.py`:

```python
"""ROS-free reactive wall-following core for the perfect-maze solver.

A perfect maze (a tree: fully connected, no loops) with entrance and exit on the
outer boundary is provably solved by the wall-following rule: keep one hand on a
wall and you traverse the connected boundary until you reach an opening — the
exit. Wall-following holds a fixed lateral offset from the followed wall via
continuous LIDAR control, so it cannot wedge (no point goal is ever aimed into a
corner) and cannot loop forever (a tree has no cycles).

This module is pure policy: deterministic, no ROS / time / I/O. The node feeds it
a `Sectors` snapshot each tick and applies the returned `Command` to /cmd_vel.

Angle convention (robot frame): forward = 0, left = +pi/2, right = -pi/2.
`+w` = turn left (CCW). `+v` = forward.
"""
from __future__ import annotations
from dataclasses import dataclass
from enum import Enum
import math


@dataclass
class Sectors:
    front: float        # min range straight ahead (m)
    side: float         # min range on the followed side (m)
    front_side: float   # min range on the forward-diagonal toward the followed side (m)


@dataclass
class Command:
    v: float            # linear velocity (m/s), + forward
    w: float            # angular velocity (rad/s), + CCW (turn left)


class State(Enum):
    FIND_WALL = 'find_wall'
    FOLLOW = 'follow'
    TURN_AWAY = 'turn_away'
    CORNER = 'corner'


def _normalize_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def sectorize(ranges, angle_min, angle_increment, follow_side, *,
              front_half_rad=math.radians(25),
              side_half_rad=math.radians(30),
              diag_half_rad=math.radians(25),
              max_range=12.0):
    """Reduce a LaserScan ranges array to (front, side, front_side) minima.

    front window: |ang| <= front_half_rad.
    side window (right-hand): |ang - (-pi/2)| <= side_half_rad; (left-hand): +pi/2.
    front_side window (right-hand): |ang - (-pi/4)| <= diag_half_rad; (left-hand): +pi/4.
    Non-finite or non-positive ranges are treated as max_range (no nearer hit).
    """
    if follow_side not in ('right', 'left'):
        raise ValueError("follow_side must be 'right' or 'left'")
    side_center = -math.pi / 2 if follow_side == 'right' else math.pi / 2
    diag_center = -math.pi / 4 if follow_side == 'right' else math.pi / 4
    front = side = front_side = max_range
    for i in range(len(ranges)):
        r = ranges[i]
        if r is None or not math.isfinite(r) or r <= 0.0:
            continue
        r = min(float(r), max_range)
        ang = _normalize_angle(angle_min + i * angle_increment)
        if abs(ang) <= front_half_rad:
            front = min(front, r)
        if abs(_normalize_angle(ang - side_center)) <= side_half_rad:
            side = min(side, r)
        if abs(_normalize_angle(ang - diag_center)) <= diag_half_rad:
            front_side = min(front_side, r)
    return Sectors(front=front, side=side, front_side=front_side)
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `cd $WS/src/tugbot_maze && python3 -m pytest test/test_wall_follower.py -v`
Expected: 5 passed.

- [ ] **Step 5: Commit**

```bash
cd $WS/.. && git add ros2_ws_tugbot_nav_20260614/src/tugbot_maze/tugbot_maze/wall_follower.py ros2_ws_tugbot_nav_20260614/src/tugbot_maze/test/test_wall_follower.py
git commit -m "feat(wall-follower): Sectors/Command/State + sectorize core"
```

---

## Task 2: `WallFollower` reactive state machine

**Files:**
- Modify: `src/tugbot_maze/tugbot_maze/wall_follower.py` (append the `WallFollower` class)
- Test: `src/tugbot_maze/test/test_wall_follower.py` (append)

- [ ] **Step 1: Write the failing tests**

First, add `WallFollower` to the import at the top of `src/tugbot_maze/test/test_wall_follower.py` so the line reads:

```python
from tugbot_maze.wall_follower import Sectors, Command, State, sectorize, WallFollower
```

Then append to the same file:

```python
# ---- WallFollower policy --------------------------------------------------

def _open():  # everything far: open corridor, no wall engaged
    return Sectors(front=5.0, side=5.0, front_side=5.0)


def test_find_wall_creeps_forward_when_all_open():
    wf = WallFollower(follow_side='right')
    cmd = wf.update(_open())
    assert cmd.v > 0 and cmd.w == 0.0
    assert wf.state == State.FIND_WALL


def test_front_blocked_turns_away_in_place_right_hand():
    wf = WallFollower(follow_side='right')
    cmd = wf.update(Sectors(front=0.5, side=0.6, front_side=0.6))
    assert cmd.v == pytest.approx(0.0)
    assert cmd.w > 0          # right-hand turns LEFT (+w) away from the right wall
    assert wf.state == State.TURN_AWAY


def test_front_blocked_turns_away_mirrored_left_hand():
    wf = WallFollower(follow_side='left')
    cmd = wf.update(Sectors(front=0.5, side=0.6, front_side=0.6))
    assert cmd.w < 0          # left-hand turns RIGHT (-w)


def test_wall_lost_corners_toward_wall_right_hand():
    wf = WallFollower(follow_side='right')
    wf.state = State.FOLLOW                       # already engaged, then the wall ends
    cmd = wf.update(Sectors(front=5.0, side=1.5, front_side=5.0))
    assert cmd.v > 0          # keep moving while rounding
    assert cmd.w < 0          # right-hand arcs RIGHT (-w) toward the lost wall
    assert wf.state == State.CORNER


def test_follow_steers_toward_wall_when_too_far_right_hand():
    wf = WallFollower(follow_side='right')
    wf.state = State.FOLLOW
    # side (0.9) > target (0.6): too far from the right wall -> steer right (-w)
    cmd = wf.update(Sectors(front=5.0, side=0.9, front_side=5.0))
    assert cmd.w < 0
    assert wf.state == State.FOLLOW


def test_follow_steers_away_from_wall_when_too_close_right_hand():
    wf = WallFollower(follow_side='right')
    wf.state = State.FOLLOW
    # side (0.3) < target (0.6): too close -> steer left (+w) away from wall
    cmd = wf.update(Sectors(front=5.0, side=0.3, front_side=5.0))
    assert cmd.w > 0


def test_follow_w_is_clamped_to_w_max():
    wf = WallFollower(follow_side='right', w_max=0.8)
    wf.state = State.FOLLOW
    cmd = wf.update(Sectors(front=5.0, side=11.0, front_side=5.0))  # huge error
    assert abs(cmd.w) <= 0.8 + 1e-9


def test_corner_hysteresis_holds_until_wall_regained():
    wf = WallFollower(follow_side='right', min_state_ticks=3)
    wf.state = State.FOLLOW                                         # already engaged
    wf.update(Sectors(front=5.0, side=1.5, front_side=5.0))         # enter CORNER
    assert wf.state == State.CORNER
    # side between target (0.6) and wall_lost (1.2): stay cornering (hysteresis)
    wf.update(Sectors(front=5.0, side=0.9, front_side=5.0))
    assert wf.state == State.CORNER
    # wall regained (side <= target) but dwell not yet elapsed: still CORNER
    wf.update(Sectors(front=5.0, side=0.5, front_side=5.0))
    assert wf.state == State.CORNER
    # dwell elapsed AND wall regained: back to FOLLOW
    wf.update(Sectors(front=5.0, side=0.5, front_side=5.0))
    assert wf.state == State.FOLLOW


def test_invalid_follow_side_raises():
    with pytest.raises(ValueError):
        WallFollower(follow_side='up')
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cd $WS/src/tugbot_maze && python3 -m pytest test/test_wall_follower.py -v`
Expected: a collection error — `ImportError: cannot import name 'WallFollower' from 'tugbot_maze.wall_follower'` (the class does not exist yet, so the whole file fails to import, including the Task 1 tests). This is the expected "red" state.

- [ ] **Step 3: Write the minimal implementation**

Append to `src/tugbot_maze/tugbot_maze/wall_follower.py`:

```python
class WallFollower:
    """Stateful-but-pure right/left-hand wall-following policy.

    `update(Sectors) -> Command` is a deterministic function of (self.state,
    Sectors). No ROS, time, or progress input — stall recovery lives in the node.

    Decision order each tick (right-hand; mirror for left via `open_turn`):
      1. front < front_block_m -> TURN_AWAY: rotate toward the open side in place.
      2. FIND_WALL (initial) and nothing within engage_m -> creep forward. (CORNER
         is gated out of FIND_WALL so an open start creeps instead of spinning.)
      3. Once engaged, wall lost (side > wall_lost_m) with hysteresis -> CORNER: arc
         toward the wall while creeping, until the wall is regained (side <= target)
         and the minimum dwell has elapsed.
      4. FOLLOW: PID on (side - target_wall_m); slow down while turning hard.
    """

    def __init__(self, *, target_wall_m=0.6, front_block_m=0.7, wall_lost_m=1.2,
                 engage_m=1.0, cruise_v=0.3, corner_v=0.18, corner_w=0.6,
                 turn_w=0.7, w_max=0.8, kp=1.5, kd=0.4, follow_side='right',
                 min_state_ticks=2):
        if follow_side not in ('right', 'left'):
            raise ValueError("follow_side must be 'right' or 'left'")
        self.target_wall_m = target_wall_m
        self.front_block_m = front_block_m
        self.wall_lost_m = wall_lost_m
        self.engage_m = engage_m
        self.cruise_v = cruise_v
        self.corner_v = corner_v
        self.corner_w = corner_w
        self.turn_w = turn_w
        self.w_max = w_max
        self.kp = kp
        self.kd = kd
        self.follow_side = follow_side
        self.min_state_ticks = min_state_ticks
        # +1 means "turn toward the open side" is +w (CCW). Right-hand: open side
        # is the LEFT, so open_turn = +1. Left-hand: open side is the right, -1.
        self.open_turn = 1.0 if follow_side == 'right' else -1.0
        self.state = State.FIND_WALL
        self._prev_err = 0.0
        self._ticks_in_state = 0

    def _enter(self, state: State) -> None:
        if state != self.state:
            self.state = state
            self._ticks_in_state = 0

    def update(self, s: Sectors) -> Command:
        self._ticks_in_state += 1

        # 1. Safety: blocked ahead always preempts -> rotate toward the open side
        #    in place (inside corner / T-stem).
        if s.front < self.front_block_m:
            self._enter(State.TURN_AWAY)
            return Command(v=0.0, w=self.open_turn * self.turn_w)

        # 2. FIND_WALL: nothing engaged yet -> creep straight until any wall comes
        #    within engage range. CORNER is gated out here, or an open start (every
        #    side beyond wall_lost_m) would arc in a circle instead of advancing.
        if self.state == State.FIND_WALL:
            if min(s.front, s.side, s.front_side) > self.engage_m:
                return Command(v=self.cruise_v, w=0.0)
            # a wall is near -> engage by falling through to FOLLOW
        else:
            # 3. Wall lost (only once engaged): arc back toward the wall to round an
            #    outside corner / junction. Hysteresis: once cornering, keep arcing
            #    until the wall is regained (side <= target) AND min dwell elapsed.
            cornering = self.state == State.CORNER
            if (not cornering and s.side > self.wall_lost_m) or \
               (cornering and (s.side > self.target_wall_m
                               or self._ticks_in_state < self.min_state_ticks)):
                self._enter(State.CORNER)
                return Command(v=self.corner_v, w=-self.open_turn * self.corner_w)

        # 4. FOLLOW: PID on lateral offset. err > 0 (too far) -> steer toward wall.
        self._enter(State.FOLLOW)
        err = s.side - self.target_wall_m
        derr = err - self._prev_err
        self._prev_err = err
        w = -self.open_turn * (self.kp * err + self.kd * derr)
        w = max(-self.w_max, min(self.w_max, w))
        v = self.cruise_v * (1.0 - 0.5 * min(1.0, abs(w) / self.w_max))
        return Command(v=v, w=w)
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `cd $WS/src/tugbot_maze && python3 -m pytest test/test_wall_follower.py -v`
Expected: all tests passed (5 from Task 1 + 9 new = 14).

- [ ] **Step 5: Commit**

```bash
cd $WS/.. && git add ros2_ws_tugbot_nav_20260614/src/tugbot_maze/tugbot_maze/wall_follower.py ros2_ws_tugbot_nav_20260614/src/tugbot_maze/test/test_wall_follower.py
git commit -m "feat(wall-follower): reactive state machine (find/follow/turn/corner)"
```

---

## Task 3: `maze_sim` segment loader + raycaster

**Files:**
- Create: `src/tugbot_maze/tugbot_maze/maze_sim.py`
- Test: `src/tugbot_maze/test/test_maze_sim.py`

- [ ] **Step 1: Write the failing tests**

Create `src/tugbot_maze/test/test_maze_sim.py`:

```python
import math
import pytest
from tugbot_maze.maze_sim import px_to_map, load_segments, MazeSim


def test_px_to_map_entrance_and_exit_corners():
    # Left outer wall x=30 px maps to ~map x 1.0; near the bottom (entrance y).
    x_m, _ = px_to_map(30, 314)
    assert x_m == pytest.approx(1.02, abs=0.05)
    # Right outer wall x=329 px, exit gap near y=44 px -> map ~ (21.0, 18.08)
    ex, ey = px_to_map(329, 44)
    assert ex == pytest.approx(21.0, abs=0.1)
    assert ey == pytest.approx(18.08, abs=0.1)


def test_load_segments_returns_real_maze():
    segs = load_segments()
    assert len(segs) == 53           # 53 wall segments in the 20260528 yaml
    for x0, y0, x1, y1 in segs:
        assert all(math.isfinite(v) for v in (x0, y0, x1, y1))


def test_scan_hits_a_wall_in_front():
    # One vertical wall 2 m ahead (east) of a robot at origin facing +x.
    seg = [(2.0, -1.0, 2.0, 1.0)]
    sim = MazeSim(seg, start_xy=(0.0, 0.0), start_yaw=0.0,
                  wall_half_thickness_m=0.0)
    ranges, amin, ainc = sim.scan(n_beams=72)
    # beam nearest to forward (angle 0) should read ~2.0 m
    fwd_i = min(range(len(ranges)), key=lambda i: abs(amin + i * ainc))
    assert ranges[fwd_i] == pytest.approx(2.0, abs=0.1)


def test_scan_returns_max_range_when_no_wall():
    sim = MazeSim([], start_xy=(0.0, 0.0), start_yaw=0.0, max_range_m=12.0)
    ranges, _, _ = sim.scan(n_beams=36)
    assert all(r == 12.0 for r in ranges)


def test_scan_subtracts_wall_half_thickness():
    seg = [(2.0, -1.0, 2.0, 1.0)]
    sim = MazeSim(seg, start_xy=(0.0, 0.0), start_yaw=0.0,
                  wall_half_thickness_m=0.12)
    ranges, amin, ainc = sim.scan(n_beams=72)
    fwd_i = min(range(len(ranges)), key=lambda i: abs(amin + i * ainc))
    assert ranges[fwd_i] == pytest.approx(1.88, abs=0.1)   # 2.0 - 0.12
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cd $WS/src/tugbot_maze && python3 -m pytest test/test_maze_sim.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'tugbot_maze.maze_sim'`.

- [ ] **Step 3: Write the minimal implementation**

Create `src/tugbot_maze/tugbot_maze/maze_sim.py`:

```python
"""Test-only 2-D maze simulator: a vectorized raycaster + unicycle integrator
over the REAL 20260528 wall segments, used to prove the wall-follower reaches the
exit offline and to select the faster hand. Not imported by any ROS node.

px -> map-frame transform for the scaled2x 20260528 world (verified three ways:
derivation, entrance gap, exit gap):
    Xm = -0.988719 + x_px * (24/359)
    Ym = 21.025070 - y_px * (24/359)
Walls are modeled as zero-thickness centerline segments; their physical
half-thickness is added back as a margin (collision) and subtracted from beam
ranges (so a beam reads the wall FACE, like the real LIDAR).
"""
from __future__ import annotations
import math
import os
from typing import List, Optional, Tuple

import numpy as np
import yaml

Segment = Tuple[float, float, float, float]   # (x0, y0, x1, y1), map frame

_PX_SCALE = 24.0 / 359.0
_X_OFFSET = -12.0 + 11.011281     # = -0.988719
_Y_OFFSET = 12.0 + 9.025070       # = 21.025070


def px_to_map(x_px: float, y_px: float) -> Tuple[float, float]:
    return (_X_OFFSET + x_px * _PX_SCALE, _Y_OFFSET - y_px * _PX_SCALE)


def default_segments_path() -> str:
    return os.path.join(os.path.dirname(__file__), '..', 'config',
                        'maze_wall_segments_20260528.yaml')


def load_segments(path: Optional[str] = None) -> List[Segment]:
    """Load wall centerline segments from the YAML, transformed to map frame."""
    if path is None:
        path = default_segments_path()
    with open(path) as f:
        doc = yaml.safe_load(f)
    segs: List[Segment] = []
    for s in doc['segments']:
        x0, y0 = px_to_map(float(s['p0_px'][0]), float(s['p0_px'][1]))
        x1, y1 = px_to_map(float(s['p1_px'][0]), float(s['p1_px'][1]))
        segs.append((x0, y0, x1, y1))
    return segs


class MazeSim:
    def __init__(self, segments, start_xy, start_yaw, *, robot_radius_m=0.35,
                 wall_half_thickness_m=0.12, max_range_m=12.0):
        self.segs = np.asarray(segments, dtype=float).reshape(-1, 4)
        self.x = float(start_xy[0])
        self.y = float(start_xy[1])
        self.yaw = float(start_yaw)
        self.robot_radius_m = robot_radius_m
        self.wall_half_thickness_m = wall_half_thickness_m
        self.max_range_m = max_range_m
        # Precompute endpoint A (S,2) and edge e = B - A (S,2).
        if self.segs.shape[0] > 0:
            self._a = self.segs[:, 0:2]
            self._e = self.segs[:, 2:4] - self.segs[:, 0:2]
        else:
            self._a = np.empty((0, 2))
            self._e = np.empty((0, 2))

    @property
    def pose(self) -> Tuple[float, float, float]:
        return (self.x, self.y, self.yaw)

    def scan(self, n_beams=120, fov_rad=2 * math.pi, max_range=None):
        """Return (ranges_list, angle_min, angle_increment) from the current pose.

        Angles are robot-relative (forward = 0). Full-circle scans use a wrapping
        layout over [-pi, pi) with increment 2*pi/n_beams (matching a velodyne).
        """
        if max_range is None:
            max_range = self.max_range_m
        if fov_rad >= 2 * math.pi - 1e-9:
            angle_min = -math.pi
            angle_inc = 2 * math.pi / n_beams
        else:
            angle_min = -fov_rad / 2.0
            angle_inc = fov_rad / (n_beams - 1)
        idx = np.arange(n_beams)
        beam_ang = self.yaw + angle_min + idx * angle_inc          # (B,) world frame
        if self.segs.shape[0] == 0:
            return [float(max_range)] * n_beams, angle_min, angle_inc
        dx = np.cos(beam_ang)[:, None]                             # (B,1)
        dy = np.sin(beam_ang)[:, None]                             # (B,1)
        ex = self._e[:, 0][None, :]                               # (1,S)
        ey = self._e[:, 1][None, :]                               # (1,S)
        wx = (self._a[:, 0] - self.x)[None, :]                    # (1,S)
        wy = (self._a[:, 1] - self.y)[None, :]                    # (1,S)
        denom = dx * ey - dy * ex                                 # (B,S)
        wxe = wx * ey - wy * ex                                   # (1,S) -> broadcast
        wxd = wx * dy - wy * dx                                   # (B,S)
        with np.errstate(divide='ignore', invalid='ignore'):
            t = wxe / denom                                       # (B,S) ray param
            u = wxd / denom                                       # (B,S) segment param
        valid = (np.abs(denom) > 1e-12) & (t >= 0.0) & (u >= 0.0) & (u <= 1.0)
        t = np.where(valid, t, np.inf)
        rng = t.min(axis=1)                                       # (B,)
        rng = np.clip(rng - self.wall_half_thickness_m, 0.0, max_range)
        return rng.tolist(), angle_min, angle_inc
```

> `collides()` and `step()` are added in Task 4 (test-first), so this Task-3 `MazeSim` has only `__init__`, `pose`, and `scan`.

- [ ] **Step 4: Run the tests to verify they pass**

Run: `cd $WS/src/tugbot_maze && python3 -m pytest test/test_maze_sim.py -v`
Expected: 5 passed (loader + raycaster). The integrator/collision are added in Task 4.

- [ ] **Step 5: Commit**

```bash
cd $WS/.. && git add ros2_ws_tugbot_nav_20260614/src/tugbot_maze/tugbot_maze/maze_sim.py ros2_ws_tugbot_nav_20260614/src/tugbot_maze/test/test_maze_sim.py
git commit -m "feat(maze-sim): px->map loader + vectorized raycaster over real maze"
```

---

## Task 4: `maze_sim` collision + integrator

**Files:**
- Modify: `src/tugbot_maze/tugbot_maze/maze_sim.py` (add `_point_seg_dists`, `collides`, `step` to `MazeSim`)
- Test: `src/tugbot_maze/test/test_maze_sim.py` (append collision/integrator tests)

- [ ] **Step 1: Write the failing tests**

Append to `src/tugbot_maze/test/test_maze_sim.py`:

```python
def test_step_moves_forward_in_open_space():
    sim = MazeSim([], start_xy=(0.0, 0.0), start_yaw=0.0)
    sim.step(1.0, 0.0, 0.5)
    x, y, _ = sim.pose
    assert x == pytest.approx(0.5, abs=1e-9)
    assert y == pytest.approx(0.0, abs=1e-9)


def test_step_rotation_normalizes_yaw():
    sim = MazeSim([], start_xy=(0.0, 0.0), start_yaw=3.0)
    sim.step(0.0, 1.0, 1.0)          # yaw -> 4.0 rad, normalized into (-pi, pi]
    _, _, yaw = sim.pose
    assert yaw == pytest.approx(math.atan2(math.sin(4.0), math.cos(4.0)), abs=1e-9)


def test_collides_true_near_wall_segment():
    seg = [(1.0, -1.0, 1.0, 1.0)]    # vertical wall at x=1
    sim = MazeSim(seg, start_xy=(0.0, 0.0), start_yaw=0.0,
                  robot_radius_m=0.35, wall_half_thickness_m=0.12)
    assert sim.collides(0.6, 0.0) is True       # 0.4 m to wall < 0.47 margin
    assert sim.collides(0.4, 0.0) is False       # 0.6 m to wall > 0.47 margin


def test_step_blocked_by_wall_keeps_position_but_allows_rotation():
    seg = [(0.5, -1.0, 0.5, 1.0)]    # wall just ahead at x=0.5
    sim = MazeSim(seg, start_xy=(0.0, 0.0), start_yaw=0.0,
                  robot_radius_m=0.35, wall_half_thickness_m=0.12)
    sim.step(1.0, 1.0, 0.5)          # try to drive into the wall while turning
    x, y, yaw = sim.pose
    assert x == pytest.approx(0.0, abs=1e-9)     # translation rejected
    assert yaw != 0.0                            # rotation still applied
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cd $WS/src/tugbot_maze && python3 -m pytest test/test_maze_sim.py -v`
Expected: the 4 new tests FAIL with `AttributeError: 'MazeSim' object has no attribute 'step'` / `... 'collides'` (Task 3's `MazeSim` has neither yet). The 5 Task-3 tests still pass.

- [ ] **Step 3: Write the minimal implementation**

Append these three methods to the `MazeSim` class in `src/tugbot_maze/tugbot_maze/maze_sim.py` (same indentation as `scan`):

```python
    def _point_seg_dists(self, x, y):
        if self.segs.shape[0] == 0:
            return np.empty((0,))
        p = np.array([x, y])
        ap = p[None, :] - self._a                                 # (S,2)
        ee = np.sum(self._e * self._e, axis=1)                    # (S,)
        t = np.where(ee > 1e-12, np.sum(ap * self._e, axis=1) / np.maximum(ee, 1e-12), 0.0)
        t = np.clip(t, 0.0, 1.0)
        proj = self._a + t[:, None] * self._e                     # (S,2)
        return np.linalg.norm(p[None, :] - proj, axis=1)          # (S,)

    def collides(self, x, y) -> bool:
        d = self._point_seg_dists(x, y)
        if d.size == 0:
            return False
        return bool(np.any(d < self.robot_radius_m + self.wall_half_thickness_m))

    def step(self, v, w, dt):
        nx = self.x + v * math.cos(self.yaw) * dt
        ny = self.y + v * math.sin(self.yaw) * dt
        if not self.collides(nx, ny):
            self.x, self.y = nx, ny
        # Rotation always applies (turning in place is safe; lets TURN_AWAY recover).
        self.yaw = math.atan2(math.sin(self.yaw + w * dt), math.cos(self.yaw + w * dt))
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `cd $WS/src/tugbot_maze && python3 -m pytest test/test_maze_sim.py -v`
Expected: 9 passed (5 from Task 3 + 4 new).

- [ ] **Step 5: Commit**

```bash
cd $WS/.. && git add ros2_ws_tugbot_nav_20260614/src/tugbot_maze/tugbot_maze/maze_sim.py ros2_ws_tugbot_nav_20260614/src/tugbot_maze/test/test_maze_sim.py
git commit -m "feat(maze-sim): collision check + unicycle integrator"
```

---

## Task 5: Integration guarantee proof — policy reaches the exit on the real maze

**Files:**
- Create: `src/tugbot_maze/test/test_wall_follow_maze_sim.py`

This is the decisive test: the REAL `WallFollower` driven through the REAL maze segments must reach the exit. It also reports which hand is faster (used to set the launch default in Task 8).

- [ ] **Step 1: Write the failing test**

Create `src/tugbot_maze/test/test_wall_follow_maze_sim.py`:

```python
import math
import pytest
from tugbot_maze.wall_follower import WallFollower, sectorize
from tugbot_maze.maze_sim import MazeSim, load_segments

EXIT_XY = (21.072562, 18.083566)
EXIT_RADIUS = 1.2
START_XY = (2.0, 0.0)        # post-ENTRY_DIRECT: ~2 m inside the entrance mouth
START_YAW = 0.0              # facing +x (east), into the maze


def run_to_exit(follow_side, max_steps=20000, dt=0.1, n_beams=72):
    segs = load_segments()
    sim = MazeSim(segs, start_xy=START_XY, start_yaw=START_YAW)
    assert not sim.collides(START_XY[0], START_XY[1]), "start must be collision-free"
    wf = WallFollower(follow_side=follow_side)
    closest = float('inf')
    for step in range(max_steps):
        ranges, amin, ainc = sim.scan(n_beams=n_beams)
        sectors = sectorize(ranges, amin, ainc, follow_side)
        cmd = wf.update(sectors)
        sim.step(cmd.v, cmd.w, dt)
        x, y, _ = sim.pose
        d = math.hypot(x - EXIT_XY[0], y - EXIT_XY[1])
        closest = min(closest, d)
        if d <= EXIT_RADIUS:
            return step + 1, closest
    return None, closest


def test_right_hand_reaches_exit():
    steps, closest = run_to_exit('right')
    assert steps is not None, f"right-hand failed to reach exit; closest={closest:.2f} m"


def test_left_hand_reaches_exit():
    steps, closest = run_to_exit('left')
    assert steps is not None, f"left-hand failed to reach exit; closest={closest:.2f} m"


def test_report_faster_hand(capsys):
    steps_r, _ = run_to_exit('right')
    steps_l, _ = run_to_exit('left')
    assert steps_r is not None and steps_l is not None
    faster = 'right' if steps_r <= steps_l else 'left'
    with capsys.disabled():
        print(f"\n[GUARANTEE] right={steps_r} steps, left={steps_l} steps -> "
              f"FASTER_HAND={faster}")
    # By construction the faster hand has the smaller (or equal) step count.
    assert min(steps_r, steps_l) == (steps_r if faster == 'right' else steps_l)
```

- [ ] **Step 2: Run the test**

Run: `cd $WS/src/tugbot_maze && python3 -m pytest test/test_wall_follow_maze_sim.py -v -s`
Expected: 3 passed, and the `[GUARANTEE]` line prints both step counts and `FASTER_HAND=...`.

- [ ] **Step 3: If a hand fails to reach the exit, tune and re-run**

This test is the design's correctness gate. If `right` or `left` fails (`steps is None`), the closest-approach distance tells you where it stalls. Tune the `WallFollower` defaults (in `wall_follower.py`) — most likely candidates, in order:
- `corner_v` / `corner_w` (rounding outside corners): if it loops at a junction, reduce `corner_w` toward 0.5 or raise `corner_v` toward 0.22 so the arc radius (`corner_v/corner_w`) better matches the ~2 m cell spacing.
- `target_wall_m` (0.6): if it collides and stalls along straights, raise toward 0.7 for more margin.
- `front_block_m` (0.7): if it clips inside corners, raise toward 0.8.
Re-run until both hands reach the exit. Do NOT weaken the assertion. Keep `max_steps=20000` (= 2000 sim-seconds) — the real run budget is 1500 s wall-clock, and the faster hand should finish well under budget.

- [ ] **Step 4: Record the faster hand**

Note the `FASTER_HAND` value from the printed output — it is the `follow_side` default you will wire into the launch file in Task 8. (Report it in your task summary.)

- [ ] **Step 5: Commit**

```bash
cd $WS/.. && git add ros2_ws_tugbot_nav_20260614/src/tugbot_maze/test/test_wall_follow_maze_sim.py ros2_ws_tugbot_nav_20260614/src/tugbot_maze/tugbot_maze/wall_follower.py
git commit -m "test(wall-follower): offline guarantee proof on real maze graph"
```

---

## Task 6: Node-support helpers (`wall_follow_control`)

**Files:**
- Create: `src/tugbot_maze/tugbot_maze/wall_follow_control.py`
- Test: `src/tugbot_maze/test/test_wall_follow_control.py`

- [ ] **Step 1: Write the failing tests**

Create `src/tugbot_maze/test/test_wall_follow_control.py`:

```python
from tugbot_maze.wall_follow_control import exit_reached, entering_done, StallWatchdog


def test_exit_reached_within_radius():
    assert exit_reached((21.0, 18.0, 0.0), (21.07, 18.08), 1.2) is True


def test_exit_not_reached_outside_radius():
    assert exit_reached((10.0, 5.0, 0.0), (21.07, 18.08), 1.2) is False


def test_entering_done_after_distance():
    assert entering_done((0.0, 0.0), (2.1, 0.0), 2.0) is True
    assert entering_done((0.0, 0.0), (1.5, 0.0), 2.0) is False


def test_stall_watchdog_first_sample_not_stalled():
    wd = StallWatchdog(stall_s=4.0, progress_eps_m=0.2)
    assert wd.update(0.0, 0.0, 0.0) is False


def test_stall_watchdog_triggers_without_progress():
    wd = StallWatchdog(stall_s=4.0, progress_eps_m=0.2)
    wd.update(0.0, 0.0, 0.0)            # anchor
    assert wd.update(2.0, 0.05, 0.0) is False     # < eps, < stall_s
    assert wd.update(4.5, 0.05, 0.0) is True      # < eps moved, >= stall_s elapsed


def test_stall_watchdog_resets_on_progress():
    wd = StallWatchdog(stall_s=4.0, progress_eps_m=0.2)
    wd.update(0.0, 0.0, 0.0)
    assert wd.update(5.0, 1.0, 0.0) is False       # moved >= eps -> re-anchor, not stalled
    assert wd.update(6.0, 1.0, 0.0) is False       # only 1 s since new anchor


def test_stall_watchdog_explicit_reset():
    wd = StallWatchdog(stall_s=4.0, progress_eps_m=0.2)
    wd.update(0.0, 0.0, 0.0)
    wd.update(3.0, 0.0, 0.0)
    wd.reset(10.0, 0.0, 0.0)
    assert wd.update(13.9, 0.0, 0.0) is False      # < stall_s since reset
    assert wd.update(14.1, 0.0, 0.0) is True
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cd $WS/src/tugbot_maze && python3 -m pytest test/test_wall_follow_control.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'tugbot_maze.wall_follow_control'`.

- [ ] **Step 3: Write the minimal implementation**

Create `src/tugbot_maze/tugbot_maze/wall_follow_control.py`:

```python
"""ROS-free node-support helpers for the wall-following solver: exit detection,
entry-distance check, and a stall watchdog. Time is injected (seconds, float) so
these stay pure and unit-testable.
"""
from __future__ import annotations
import math
from typing import Optional, Tuple


def exit_reached(pose, exit_xy, radius_m) -> bool:
    """True if the robot pose (x, y, [yaw]) is within radius_m of the exit."""
    return math.hypot(pose[0] - exit_xy[0], pose[1] - exit_xy[1]) <= radius_m


def entering_done(start_xy, cur_xy, distance_m) -> bool:
    """True once the robot has travelled >= distance_m (straight-line) from start."""
    return math.hypot(cur_xy[0] - start_xy[0], cur_xy[1] - start_xy[1]) >= distance_m


class StallWatchdog:
    """Flags a stall: no net progress (>= progress_eps_m) for >= stall_s seconds.

    Anchors on the first sample; re-anchors whenever the robot moves at least
    progress_eps_m from the current anchor. `update(t, x, y)` returns True when the
    robot has sat within progress_eps_m of the anchor for at least stall_s.
    """

    def __init__(self, *, stall_s=4.0, progress_eps_m=0.2):
        self.stall_s = stall_s
        self.progress_eps_m = progress_eps_m
        self._anchor_t: Optional[float] = None
        self._anchor_xy: Optional[Tuple[float, float]] = None

    def update(self, t, x, y) -> bool:
        if self._anchor_xy is None:
            self._anchor_t, self._anchor_xy = t, (x, y)
            return False
        moved = math.hypot(x - self._anchor_xy[0], y - self._anchor_xy[1])
        if moved >= self.progress_eps_m:
            self._anchor_t, self._anchor_xy = t, (x, y)
            return False
        return (t - self._anchor_t) >= self.stall_s

    def reset(self, t, x, y) -> None:
        self._anchor_t, self._anchor_xy = t, (x, y)
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `cd $WS/src/tugbot_maze && python3 -m pytest test/test_wall_follow_control.py -v`
Expected: 7 passed.

- [ ] **Step 5: Commit**

```bash
cd $WS/.. && git add ros2_ws_tugbot_nav_20260614/src/tugbot_maze/tugbot_maze/wall_follow_control.py ros2_ws_tugbot_nav_20260614/src/tugbot_maze/test/test_wall_follow_control.py
git commit -m "feat(wall-follower): exit/entry/stall node-support helpers"
```

---

## Task 7: `wall_follow_solver` ROS node

**Files:**
- Create: `src/tugbot_maze/tugbot_maze/wall_follow_solver.py`
- Test: `src/tugbot_maze/test/test_wall_follow_solver_smoke.py`

- [ ] **Step 1: Write the failing smoke test**

Create `src/tugbot_maze/test/test_wall_follow_solver_smoke.py`:

```python
import pytest


def test_node_module_imports():
    # Skip cleanly where ROS is not sourced (pure CI); colcon test provides rclpy.
    pytest.importorskip("rclpy")
    from tugbot_maze import wall_follow_solver
    assert hasattr(wall_follow_solver, "WallFollowSolver")
    assert hasattr(wall_follow_solver, "main")
```

- [ ] **Step 2: Run the test to verify it fails (or skips)**

Run: `cd $WS/src/tugbot_maze && python3 -m pytest test/test_wall_follow_solver_smoke.py -v`
Expected: SKIPPED if `rclpy` is not on the path, or FAIL (`ModuleNotFoundError: tugbot_maze.wall_follow_solver`) if ROS is sourced. After implementing, with ROS sourced it must PASS.

- [ ] **Step 3: Write the node implementation**

Create `src/tugbot_maze/tugbot_maze/wall_follow_solver.py`:

```python
"""Thin ROS 2 node: reactive wall-following autonomous maze solver.

All control is delegated to the ROS-free `WallFollower` policy; this node owns
only ROS plumbing (scan subscription, TF pose, a 10 Hz control timer, a 5 Hz DIAG
timer) plus entry, stall recovery, and the exit self-check. The control loop needs
only /scan; SLAM (map->base_link TF) is used solely for the exit distance check.

ROS plumbing mirrors maze_solver.py: /cmd_vel_nav publisher, goal_events String
publisher ('EXIT_REACHED'), and the map->base_link yaw extraction.
"""
import math
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import tf2_ros

from tugbot_maze.wall_follower import WallFollower, State, sectorize
from tugbot_maze.wall_follow_control import exit_reached, entering_done, StallWatchdog


class WallFollowSolver(Node):
    def __init__(self):
        super().__init__('wall_follow_solver')
        # --- ROS / frames ---
        self.scan_topic = self.declare_parameter('scan_topic', '/scan').value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.goal_events_topic = self.declare_parameter('goal_events_topic', '/maze/goal_events').value
        # --- mission geometry ---
        self.exit_x = float(self.declare_parameter('exit_x', 21.072562).value)
        self.exit_y = float(self.declare_parameter('exit_y', 18.083566).value)
        self.exit_radius = float(self.declare_parameter('exit_radius', 1.2).value)
        self.entry_direct_distance_m = float(self.declare_parameter('entry_direct_distance_m', 2.0).value)
        self.startup_delay_sec = float(self.declare_parameter('startup_delay_sec', 3.0).value)
        # --- wall-follower tuning (defaults == WallFollower defaults) ---
        self.follow_side = self.declare_parameter('follow_side', 'right').value
        target_wall_m = float(self.declare_parameter('target_wall_m', 0.6).value)
        front_block_m = float(self.declare_parameter('front_block_m', 0.7).value)
        wall_lost_m = float(self.declare_parameter('wall_lost_m', 1.2).value)
        engage_m = float(self.declare_parameter('engage_m', 1.0).value)
        cruise_v = float(self.declare_parameter('cruise_v', 0.3).value)
        corner_v = float(self.declare_parameter('corner_v', 0.18).value)
        corner_w = float(self.declare_parameter('corner_w', 0.6).value)
        turn_w = float(self.declare_parameter('turn_w', 0.7).value)
        w_max = float(self.declare_parameter('w_max', 0.8).value)
        kp = float(self.declare_parameter('kp', 1.5).value)
        kd = float(self.declare_parameter('kd', 0.4).value)
        self.max_range_m = float(self.declare_parameter('max_range_m', 12.0).value)
        # --- stall watchdog ---
        stall_s = float(self.declare_parameter('stall_s', 4.0).value)
        progress_eps_m = float(self.declare_parameter('progress_eps_m', 0.2).value)
        self.backup_s = float(self.declare_parameter('backup_s', 1.0).value)
        self.backup_v = float(self.declare_parameter('backup_v', -0.15).value)
        self.cruise_v = cruise_v

        self.follower = WallFollower(
            target_wall_m=target_wall_m, front_block_m=front_block_m,
            wall_lost_m=wall_lost_m, engage_m=engage_m, cruise_v=cruise_v,
            corner_v=corner_v, corner_w=corner_w, turn_w=turn_w, w_max=w_max,
            kp=kp, kd=kd, follow_side=self.follow_side)
        self.watchdog = StallWatchdog(stall_s=stall_s, progress_eps_m=progress_eps_m)

        self.create_subscription(LaserScan, self.scan_topic, self._scan_cb, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        self.goal_events_pub = self.create_publisher(String, self.goal_events_topic, 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.scan_msg: Optional[LaserScan] = None
        self.phase = 'startup'      # startup | entering | follow | backup | done
        self.start_xy = None
        self.backup_until = 0.0

        self.create_timer(0.1, self._control_tick)   # 10 Hz control
        self.create_timer(5.0, self._diag_tick)
        self.start_time = self.get_clock().now()
        self.get_logger().info('wall_follow_solver started (follow_side=%s).' % self.follow_side)

    # --- ROS callbacks / helpers (mirror maze_solver.py) ---

    def _scan_cb(self, msg):
        self.scan_msg = msg

    def _lookup_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, rclpy.time.Time())
        except Exception:
            return None
        q = t.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return (t.transform.translation.x, t.transform.translation.y, yaw)

    def _publish_event(self, text):
        self.goal_events_pub.publish(String(data=text))

    def _publish_cmd(self, v, w):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.cmd_vel_pub.publish(msg)

    def _sectors(self):
        s = self.scan_msg
        return sectorize(s.ranges, s.angle_min, s.angle_increment, self.follow_side,
                         max_range=self.max_range_m)

    # --- control loop ---

    def _control_tick(self):
        now = self.get_clock().now()
        t = now.nanoseconds / 1e9
        pose = self._lookup_pose()

        # Exit self-check (needs pose; control loop otherwise needs only /scan).
        if pose is not None and exit_reached(pose, (self.exit_x, self.exit_y), self.exit_radius):
            if self.phase != 'done':
                self.phase = 'done'
                self.get_logger().info('EXIT_REACHED (wall_follow_solver)')
                self._publish_event('EXIT_REACHED')
                self._publish_cmd(0.0, 0.0)
            return
        if self.phase == 'done':
            return

        elapsed = (now - self.start_time).nanoseconds / 1e9
        if self.phase == 'startup':
            if elapsed >= self.startup_delay_sec and pose is not None:
                self.start_xy = (pose[0], pose[1])
                self.phase = 'entering'
                self.get_logger().info('ENTRY_DIRECT: driving %.2f m inward' % self.entry_direct_distance_m)
            return

        if self.phase == 'entering':
            # Drive straight in until ~entry_direct_distance_m, but hand off early if
            # a wall looms ahead (don't ram it — let the follower turn away). This
            # tolerates a spawn heading that is not perfectly inward.
            done = pose is not None and entering_done(
                self.start_xy, (pose[0], pose[1]), self.entry_direct_distance_m)
            blocked = self.scan_msg is not None and \
                self._sectors().front < self.follower.front_block_m
            if done or blocked:
                self.phase = 'follow'
                self.follower.state = State.FIND_WALL
                if pose is not None:
                    self.watchdog.reset(t, pose[0], pose[1])
                self.get_logger().info('engaging wall-follower (entered=%s blocked=%s)'
                                       % (done, blocked))
            else:
                self._publish_cmd(self.cruise_v, 0.0)   # straight in
            return

        # follow / backup need a scan
        if self.scan_msg is None:
            return

        if self.phase == 'backup':
            if t >= self.backup_until:
                self.phase = 'follow'
                self.follower.state = State.FIND_WALL
                if pose is not None:
                    self.watchdog.reset(t, pose[0], pose[1])
            else:
                self._publish_cmd(self.backup_v, 0.0)
            return

        # phase == 'follow'
        if pose is not None and self.watchdog.update(t, pose[0], pose[1]):
            self.phase = 'backup'
            self.backup_until = t + self.backup_s
            self._publish_event('STALL_BACKUP')
            self.get_logger().warn('stall detected — backing up %.1f s' % self.backup_s)
            self._publish_cmd(self.backup_v, 0.0)
            return

        cmd = self.follower.update(self._sectors())
        self._publish_cmd(cmd.v, cmd.w)

    def _diag_tick(self):
        pose = self._lookup_pose()
        if pose is None:
            return
        dist = math.hypot(pose[0] - self.exit_x, pose[1] - self.exit_y)
        self.get_logger().info(
            'DIAG pose=(%.2f, %.2f) yaw=%.2f dist_to_exit=%.2f phase=%s state=%s'
            % (pose[0], pose[1], pose[2], dist, self.phase, self.follower.state.value))


def main(args=None):
    rclpy.init(args=args)
    node = WallFollowSolver()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

- [ ] **Step 4: Run the smoke test (ROS sourced)**

Run: `cd $WS && source /opt/ros/jazzy/setup.bash && source install/setup.bash 2>/dev/null; cd src/tugbot_maze && python3 -m pytest test/test_wall_follow_solver_smoke.py -v`
Expected: 1 passed (with `rclpy` available). If `rclpy` is unavailable it SKIPS — that is acceptable here; the colcon build in Task 9 is the real gate.

- [ ] **Step 5: Commit**

```bash
cd $WS/.. && git add ros2_ws_tugbot_nav_20260614/src/tugbot_maze/tugbot_maze/wall_follow_solver.py ros2_ws_tugbot_nav_20260614/src/tugbot_maze/test/test_wall_follow_solver_smoke.py
git commit -m "feat(wall-follower): thin ROS node (scan->policy->cmd_vel, entry/stall/exit)"
```

---

## Task 8: Wire entry point, launch, and run scripts

**Files:**
- Modify: `src/tugbot_maze/setup.py`
- Modify: `src/tugbot_bringup/launch/tugbot_maze_explore.launch.py`
- Create: `tools/run_wall_follower_maze.sh`
- Create: `tools/run_wall_follower_reliability.sh`

- [ ] **Step 1: Add the console_scripts entry point**

In `src/tugbot_maze/setup.py`, inside `entry_points={'console_scripts': [...]}`, add this line after `'maze_solver = tugbot_maze.maze_solver:main',`:

```python
            'wall_follow_solver = tugbot_maze.wall_follow_solver:main',
```

- [ ] **Step 2: Add the launch node, arg, and timer wiring**

In `src/tugbot_bringup/launch/tugbot_maze_explore.launch.py`:

(a) Add a new node definition immediately after the `maze_solver_node = Node(...)` block (after its closing `)`):

```python
    wall_follow_solver_node = Node(
        package='tugbot_maze',
        executable='wall_follow_solver',
        name='wall_follow_solver',
        output='screen',
        condition=IfCondition(PythonExpression(["'", explorer_type, "' == 'wall_follower'"])),
        parameters=[{
            'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
            'scan_topic': '/scan',
            'base_frame': 'base_link',
            'map_frame': 'map',
            'goal_events_topic': LaunchConfiguration('goal_events_topic'),
            'exit_x': ParameterValue(LaunchConfiguration('exit_x'), value_type=float),
            'exit_y': ParameterValue(LaunchConfiguration('exit_y'), value_type=float),
            'exit_radius': ParameterValue(LaunchConfiguration('exit_radius'), value_type=float),
            'entry_direct_distance_m': ParameterValue(LaunchConfiguration('entry_direct_distance_m'), value_type=float),
            'follow_side': LaunchConfiguration('follow_side'),
        }],
    )
```

(b) Add a `follow_side` launch argument. Insert this `DeclareLaunchArgument` into the `LaunchDescription([...])` list, immediately after the `DeclareLaunchArgument('explorer_type', ...)` line:

```python
        DeclareLaunchArgument('follow_side', default_value='right', description='Wall-follower hand for explorer_type:=wall_follower: right or left (the maze_sim guarantee test selects the faster).'),
```

> **Set the default to the `FASTER_HAND` reported by the Task 5 guarantee test.** If Task 5 reported `left`, use `default_value='left'`.

(c) Update the `explorer_type` argument description to mention the new mode — change:

```python
        DeclareLaunchArgument('explorer_type', default_value='maze_dfs', description='Explorer implementation: maze_dfs, frontier, or tremaux.'),
```
to:
```python
        DeclareLaunchArgument('explorer_type', default_value='maze_dfs', description='Explorer implementation: maze_dfs, frontier, tremaux, or wall_follower.'),
```

(d) Add the node to the 13 s `TimerAction` — change:

```python
        TimerAction(period=13.0, actions=[maze_dfs_explorer, frontier_explorer, maze_solver_node]),
```
to:
```python
        TimerAction(period=13.0, actions=[maze_dfs_explorer, frontier_explorer, maze_solver_node, wall_follow_solver_node]),
```

- [ ] **Step 3: Create `tools/run_wall_follower_maze.sh`**

```bash
#!/usr/bin/env bash
# Reactive wall-following autonomous maze run (explorer_type:=wall_follower):
# the robot drives a ROS-free right/left-hand wall-follower (no Nav2 in the loop)
# and must reach the exit on its own. Same process/SHM hygiene as run_solver_maze.sh.
#
# Usage: tools/run_wall_follower_maze.sh [MAX_SECONDS] [HEADLESS] [USE_RVIZ] [FOLLOW_SIDE]
set +u
WS="$(cd "$(dirname "$0")/.." && pwd)"
cd "$WS"
source /opt/ros/jazzy/setup.bash
source install/setup.bash

MAX_SECONDS="${1:-1500}"
HEADLESS="${2:-true}"
USE_RVIZ="${3:-false}"
FOLLOW_SIDE="${4:-right}"
STAMP="$(date +%Y%m%d_%H%M%S)"
ART="log/wall_follower_run_${STAMP}"
mkdir -p "$ART"

kill_all_sim() {
    for pat in tugbot_maze_explore "ros2 launch" "gz sim" "ruby.*gz" parameter_bridge \
               bridge_node slam_toolbox controller_server planner_server bt_navigator \
               behavior_server smoother_server route_server waypoint_follower \
               velocity_smoother collision_monitor lifecycle_manager map_server amcl \
               maze_explorer maze_solver wall_follow_solver frontier_explorer maze_goal_monitor \
               robot_state_publisher static_transform_publisher component_container rviz; do
        pkill -9 -f "$pat" 2>/dev/null
    done
}

kill_all_sim
sleep 2
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
export ROS_DOMAIN_ID=$(( ($(date +%s) % 90) + 1 ))
echo "[WALLFOLLOW] artifact dir: $ART max=${MAX_SECONDS}s headless=${HEADLESS} rviz=${USE_RVIZ} side=${FOLLOW_SIDE} DOMAIN=$ROS_DOMAIN_ID"

ros2 launch tugbot_bringup tugbot_maze_explore.launch.py \
    headless:="${HEADLESS}" \
    use_rviz:="${USE_RVIZ}" \
    explorer_type:=wall_follower \
    follow_side:="${FOLLOW_SIDE}" \
    entry_direct_distance_m:=2.0 \
    > "$ART/launch.log" 2>&1 &
LAUNCH_PID=$!
echo "[WALLFOLLOW] launch PID=$LAUNCH_PID DOMAIN=$ROS_DOMAIN_ID" | tee "$ART/run_meta.txt"

END=$(( $(date +%s) + MAX_SECONDS ))
RESULT="TIMEOUT"
while [ "$(date +%s)" -lt "$END" ]; do
    sleep 10
    if grep -qa "EXIT_REACHED" "$ART/launch.log" 2>/dev/null; then RESULT="EXIT_REACHED"; break; fi
    if grep -qa "open_and_lock_file failed" "$ART/launch.log" 2>/dev/null; then RESULT="DDS_SHM_FAIL"; break; fi
    if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then RESULT="LAUNCH_DIED"; break; fi
done

echo "[WALLFOLLOW] result=$RESULT" | tee -a "$ART/run_meta.txt"
echo "$RESULT" > "$ART/result.txt"
grep -aE "EXIT_REACHED|ENTRY_DIRECT|engaging wall-follower|STALL_BACKUP|DIAG" "$ART/launch.log" | tail -60 > "$ART/wall_follower_tail.txt" 2>/dev/null

kill -INT "$LAUNCH_PID" 2>/dev/null
sleep 5
kill_all_sim
sleep 2
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
echo "[WALLFOLLOW] done result=$RESULT artifact=$ART"
echo "ARTIFACT_DIR=$ART"
```

- [ ] **Step 4: Create `tools/run_wall_follower_reliability.sh`**

```bash
#!/usr/bin/env bash
# Reliability harness for the reactive wall-following solver: run N full headless
# runs and tally how many reach the exit. Target: >= 4/5 EXIT_REACHED.
#
# Usage: tools/run_wall_follower_reliability.sh [N_RUNS] [MAX_SECONDS_PER_RUN] [FOLLOW_SIDE]
set +u
WS="$(cd "$(dirname "$0")/.." && pwd)"
cd "$WS"

N="${1:-5}"
SECS="${2:-1500}"
FOLLOW_SIDE="${3:-right}"
PASS=0
SUMMARY="log/wall_follower_reliability_$(date +%Y%m%d_%H%M%S).txt"
echo "[RELIABILITY] N=$N secs=$SECS side=$FOLLOW_SIDE" | tee "$SUMMARY"

for i in $(seq 1 "$N"); do
    echo "=== wall-follower run $i/$N ===" | tee -a "$SUMMARY"
    OUT=$(./tools/run_wall_follower_maze.sh "$SECS" true false "$FOLLOW_SIDE")
    ART=$(echo "$OUT" | grep -aoE "ARTIFACT_DIR=.*" | tail -1 | cut -d= -f2)
    R="UNKNOWN"
    [ -n "$ART" ] && R=$(cat "$ART/result.txt" 2>/dev/null)
    [ "$R" = "EXIT_REACHED" ] && PASS=$((PASS + 1))
    echo "run $i: result=$R artifact=$ART (cumulative pass=$PASS/$i)" | tee -a "$SUMMARY"
done

echo "RELIABILITY: $PASS/$N reached EXIT" | tee -a "$SUMMARY"
```

- [ ] **Step 5: Make the scripts executable**

Run: `chmod +x $WS/tools/run_wall_follower_maze.sh $WS/tools/run_wall_follower_reliability.sh`

- [ ] **Step 6: Commit**

```bash
cd $WS/.. && git add ros2_ws_tugbot_nav_20260614/src/tugbot_maze/setup.py ros2_ws_tugbot_nav_20260614/src/tugbot_bringup/launch/tugbot_maze_explore.launch.py ros2_ws_tugbot_nav_20260614/tools/run_wall_follower_maze.sh ros2_ws_tugbot_nav_20260614/tools/run_wall_follower_reliability.sh
git commit -m "feat(wall-follower): entry point, launch wiring, run/reliability scripts"
```

---

## Task 9: Build, headless smoke, and reliability batch

**Files:** none (build + run verification)

- [ ] **Step 1: Build the packages**

Run:
```bash
cd $WS && source /opt/ros/jazzy/setup.bash && colcon build --packages-select tugbot_maze tugbot_bringup --symlink-install
```
Expected: `Finished <<< tugbot_maze` and `Finished <<< tugbot_bringup`, no errors.

- [ ] **Step 2: Run the full pure test suite via colcon**

Run:
```bash
cd $WS && source install/setup.bash && colcon test --packages-select tugbot_maze --pytest-args -k "wall_follow or maze_sim" && colcon test-result --verbose
```
Expected: all wall-follower/maze-sim tests pass (the integration guarantee test included).

- [ ] **Step 3: Single headless run (use the faster hand from Task 5)**

Run (replace `right` with the faster hand if Task 5 reported `left`):
```bash
cd $WS && ./tools/run_wall_follower_maze.sh 1500 true false right
```
Expected: `[WALLFOLLOW] result=EXIT_REACHED`. Inspect `log/wall_follower_run_*/wall_follower_tail.txt` for the `ENTRY_DIRECT` → `engaging wall-follower` → decreasing `dist_to_exit` → `EXIT_REACHED` progression. If it does NOT reach the exit, capture the DIAG tail (where `dist_to_exit` plateaus) and feed it back as a tuning iteration on `wall_follower.py` defaults (re-validate with the Task 5 guarantee test, which must stay green), then rebuild and re-run.

- [ ] **Step 4: Reliability batch (the standing bar)**

Run:
```bash
cd $WS && ./tools/run_wall_follower_reliability.sh 5 1500 right
```
Expected: `RELIABILITY: 4/5 reached EXIT` or better. This is the success criterion ("reliable autonomous 通关").

- [ ] **Step 5: Commit any tuning changes from Steps 3–4**

```bash
cd $WS/.. && git add ros2_ws_tugbot_nav_20260614/src/tugbot_maze/tugbot_maze/wall_follower.py
git commit -m "tune(wall-follower): defaults validated by headless reliability batch" || echo "no tuning changes to commit"
```

---

## Success criteria (from the spec)

1. The `wall_follower` + `maze_sim` integration test proves exit-reaching on the real maze graph (offline guarantee) and fixes the faster `follow_side` — **Task 5**.
2. A headless Gazebo run logs `EXIT_REACHED` via `explorer_type:=wall_follower` — **Task 9 Step 3**.
3. Reliability batch ≥ 4/5 `EXIT_REACHED` within 1500 s — **Task 9 Step 4**.

## Notes for the executor

- The GCN (`guided_corridor_mode`) and Trémaux/frontier paths are untouched and remain the reliable fallback — this plan only adds a new `explorer_type`.
- Keep the `WallFollower` core ROS-free: no `import rclpy` in `wall_follower.py`, `wall_follow_control.py`, or `maze_sim.py`. Only `wall_follow_solver.py` imports ROS.
- The maze is a verified perfect maze (tree), so the guarantee test passing is not luck — it is the structural reason wall-following works here. If the guarantee test ever fails, suspect a sim/transform regression before doubting the algorithm.
