# Junction Heading Stability (Approach C) — Implementation Plan

> Execute inline with TDD (superpowers:executing-plans). Pairs with spec `docs/superpowers/specs/2026-06-24-junction-heading-stability-design.md`.

**Conventions:** `PKG = .../ros2_ws_tugbot_nav_20260614/src/tugbot_maze`, `REPO = .../ros2_gazebo_tugbot_navigate`. Branch `no-progress-watchdog` (already on it). Never push. Stage only each task's files. Commit trailer `Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>`; no backticks in `-m`. Tests: `cd "$PKG" && PYTHONPATH="$PKG" python3 -m pytest <path> -v`.

---

## Task 1: C1 — cross-track heading-authority cap

**Files:** `tugbot_maze/hop_controller.py`, `tugbot_maze/maze_motion.py`; Test `test/test_hop_controller.py`.

- [ ] **Step 1: Failing tests** — append to `test/test_hop_controller.py`:
```python
def test_cross_track_steer_is_capped():
    from tugbot_maze.hop_controller import corridor_drive_command
    # huge cross_track, robot aligned to cardinal (yaw=0): steering capped to kp_ang*max_cross_steer
    v, w = corridor_drive_command(0.0, 0.0, 2.0, None, kp_ang=1.5, lookahead_m=0.7, max_cross_steer=0.25)
    assert abs(w - (-1.5 * 0.25)) < 1e-6          # capped (uncapped would be atan2(-2,0.7)*1.5 -> clamp -0.5)


def test_small_cross_track_unaffected_by_cap():
    import math
    from tugbot_maze.hop_controller import corridor_drive_command
    w = corridor_drive_command(0.0, 0.0, 0.10, None, kp_ang=1.5, lookahead_m=0.7, max_cross_steer=0.25)[1]
    assert abs(w - 1.5 * math.atan2(-0.10, 0.7)) < 1e-6   # below cap -> unchanged
```
- [ ] **Step 2: Run → FAIL** (`max_cross_steer` is an unexpected kwarg): `cd "$PKG" && PYTHONPATH="$PKG" python3 -m pytest test/test_hop_controller.py -k cross_track_steer -v`
- [ ] **Step 3: Implement**
  (a) `corridor_drive_command` signature: add `max_cross_steer: float = 0.25` to the keyword-only group. Replace `setpoint = cardinal_yaw + math.atan2(-cross_track, lookahead_m)` with:
  ```python
      cross_steer = max(-max_cross_steer, min(max_cross_steer, math.atan2(-cross_track, lookahead_m)))
      setpoint = cardinal_yaw + cross_steer
  ```
  (b) `corridor_follow_command` signature: add `max_cross_steer: float = 0.25`; in its `return corridor_drive_command(...)` add `max_cross_steer=max_cross_steer`.
  (c) `maze_motion.py` `__init__`: add `self.max_cross_steer = 0.25` (near the other corridor params, e.g. after `self.grid_fallback_max_m = ...`).
  (d) `maze_motion.py` `_drive`'s `corridor_follow_command(...)` call: add `max_cross_steer=self.max_cross_steer`.
- [ ] **Step 4: Run → PASS + regression**: `cd "$PKG" && PYTHONPATH="$PKG" python3 -m pytest test/test_hop_controller.py test/test_maze_motion.py test/test_maze_motion_sim.py -q`
- [ ] **Step 5: Commit** `maze_motion.py`, `hop_controller.py`, `test/test_hop_controller.py` — `feat: cap cross-track heading authority (junction heading stability)`

---

## Task 2: C2 — front_block heading-gate

**Files:** `tugbot_maze/maze_motion.py`; Test `test/test_maze_motion.py`.

- [ ] **Step 1: Failing test** — append to `test/test_maze_motion.py`:
```python
def test_front_block_gated_by_heading():
    m = MazeMotion(); m.target_cardinal = 0.0                 # hop cardinal = E
    m.hop_dir = (1, 0)
    perp = {'E': 0.5, 'N': 2.0, 'S': 2.0, 'W': 2.0}           # short front (E), open sides
    assert m._front_blocked(perp, 'E', 0.5, 0.0) is True      # aligned + short front + moved>0.3 -> block
    assert m._front_blocked(perp, 'E', 0.5, 0.5) is False     # |yaw_err|=0.5 >= front_block_max_yaw -> angled, no block
    assert m._front_blocked(perp, 'E', 0.2, 0.0) is False     # moved<=0.3 -> no block
```
- [ ] **Step 2: Run → FAIL** (`_front_blocked` undefined): `... -k front_block_gated -v`
- [ ] **Step 3: Implement** in `maze_motion.py`:
  (a) `__init__`: add `self.front_block_max_yaw = 0.30` (near `self.front_block_m`).
  (b) Add the method (near `_drive`):
  ```python
      def _front_blocked(self, perp, dirn, moved, yaw):
          """A hop is front-blocked only when ALIGNED: a short front reading while mis-aligned is an
          angled beam hitting a side/corner wall (false), not a real obstacle (walls come from SENSING)."""
          return (perp[dirn] < self.front_block_m and moved > 0.3
                  and abs(_norm(self.target_cardinal - yaw)) < self.front_block_max_yaw)
  ```
  (c) In `_drive`, replace `front_blocked = perp[dirn] < self.front_block_m and moved > 0.3` with:
  ```python
          front_blocked = self._front_blocked(perp, dirn, moved, yaw)
  ```
  (leave the `if front_blocked or t >= self.hop_deadline:` and the STALL-event `reason` unchanged.)
- [ ] **Step 4: Run → PASS + regression**: `... test/test_maze_motion.py test/test_maze_motion_sim.py -q`
- [ ] **Step 5: Commit** `maze_motion.py`, `test/test_maze_motion.py` — `feat: gate front_block on heading (ignore angled-beam false blocks)`

---

## Task 3: C3 — remove the NH14 dead-maze latch

**Files:** `tugbot_maze/maze_motion.py`.

- [ ] **Step 1: Remove the latch.** In `maze_motion.py`:
  (a) Delete the `self._latched = False ...` init line.
  (b) In the `step()` C2 watchdog guard, remove the ` and not self._latched` term.
  (c) In `_escape`, delete the block:
  ```python
          if self._maze_exhausted():
              self.events.append("LATCH cell=%s -> permanent stuck (maze_exhausted)" % (self.cell,))  # DIAG
              self.phase = 'stuck'; self._latched = True
              return (0.0, 0.0, False)
  ```
  (d) Delete the `_maze_exhausted` method in full.
- [ ] **Step 2: Run the full suite → PASS** (nothing referenced the latch except the removed pieces): `cd "$PKG" && PYTHONPATH="$PKG" python3 -m pytest test/ -q`
  Expected: all green (118+). If a NameError/AttributeError surfaces, grep for `_latched`/`_maze_exhausted` and remove the stray reference.
- [ ] **Step 3: Commit** `maze_motion.py` — `fix: remove NH14 dead-maze latch (fired falsely -> permanent freeze)`

---

## Validation (after all tasks)
1. Full ROS-free suite green: `cd "$PKG" && PYTHONPATH="$PKG" python3 -m pytest test/ -q`.
2. Rebuild + Gazebo (user-initiated; diagnostics kept IN): `colcon build --packages-select tugbot_maze`, then `tools/run_flood_fill_maze.sh 1800 false true odom_locked true`. Watch: junction cluster no longer produces `front_block`/`wedge` false-stalls (compare STALL counts vs the 8 wedge / 9 front_block baseline); robot threads (1,3)/(2,3)/(1,4) past 18.30 m; no permanent latch.
3. No merge/push. Revert the diagnostic commit `2fe9e39` before any merge.
