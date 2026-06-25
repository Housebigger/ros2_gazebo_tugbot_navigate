# Safety-Radius Reactive Keep-Out — Implementation Plan

> Execute inline with TDD. Pairs with spec `docs/superpowers/specs/2026-06-24-safety-radius-keepout-design.md`. Branch `no-progress-watchdog`. Never push; stage only each task's files; trailer `Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>`; no backticks in `-m`.

## Task 1: keep-out steer-override + earlier slow in `corridor_drive_command`

**Files:** `tugbot_maze/hop_controller.py`; Test `test/test_hop_controller.py`.

- [ ] **Step 1: Failing test** — append to `test/test_hop_controller.py`:
```python
def test_keepout_overrides_cross_steer_cap_when_close():
    # near a wall (clearance < safety_radius): emergency cap allows stronger centering than normal.
    # kp_ang=1.0, w_max=2.0 so w doesn't saturate and the two caps are distinguishable.
    w_close = corridor_drive_command(0.0, 0.0, 2.0, 0.5, kp_ang=1.0, w_max=2.0, lookahead_m=0.7,
                                     max_cross_steer=0.35, safety_radius=0.60, keepout_max_cross_steer=0.8)[1]
    w_far = corridor_drive_command(0.0, 0.0, 2.0, 1.0, kp_ang=1.0, w_max=2.0, lookahead_m=0.7,
                                   max_cross_steer=0.35, safety_radius=0.60, keepout_max_cross_steer=0.8)[1]
    assert abs(w_close - (-0.8)) < 1e-6     # clearance 0.5 < 0.60 -> emergency cap 0.8 -> w=-0.8
    assert abs(w_far - (-0.35)) < 1e-6      # clearance 1.0 >= 0.60 -> normal cap 0.35 -> w=-0.35
```
- [ ] **Step 2: Run → FAIL** (unexpected kwargs `safety_radius`/`keepout_max_cross_steer`): `cd "$PKG" && PYTHONPATH="$PKG" python3 -m pytest test/test_hop_controller.py -k keepout_overrides -v`
- [ ] **Step 3: Implement** in `hop_controller.py`:
  - `corridor_drive_command` signature: add `safety_radius: float = 0.60, keepout_max_cross_steer: float = 0.8`; change `wedge_slow_m: float = 0.50` → `0.60`.
  - body: replace the `cross_steer = ...` line with:
  ```python
      cap = keepout_max_cross_steer if (near_wall_m is not None and near_wall_m < safety_radius) else max_cross_steer
      cross_steer = max(-cap, min(cap, math.atan2(-cross_track, lookahead_m)))
  ```
  - `corridor_follow_command` signature: add the same two params; change `wedge_slow_m: float = 0.50` → `0.60`; in its `return corridor_drive_command(...)` add `safety_radius=safety_radius, keepout_max_cross_steer=keepout_max_cross_steer`.
- [ ] **Step 4: Run → PASS + regression**: `cd "$PKG" && PYTHONPATH="$PKG" python3 -m pytest test/test_hop_controller.py test/test_maze_motion.py test/test_maze_motion_sim.py -q` (if a wedge-slow test asserts the old 0.50, update it — tuning).
- [ ] **Step 5: Commit** `hop_controller.py`, `test/test_hop_controller.py`.

## Task 2: full-scan keep-out clearance in `_drive`

**Files:** `tugbot_maze/maze_motion.py`; Test `test/test_maze_motion.py`.

- [ ] **Step 1: Failing test** — append to `test/test_maze_motion.py`:
```python
def test_keepout_clearance_uses_full_scan_min():
    m = MazeMotion()
    # spurious 0.02 filtered; 0.45 is the min valid full-scan return (< side perp 0.88)
    assert m._keepout_clearance(0.88, [2.0, 1.5, 0.45, float('inf'), 0.02]) == 0.45
    assert m._keepout_clearance(0.30, [2.0, 1.5]) == 0.30        # cardinal side smaller than any beam
    assert m._keepout_clearance(0.70, [float('inf'), float('inf')]) == 0.70   # no finite beams -> near
```
- [ ] **Step 2: Run → FAIL** (`_keepout_clearance` undefined).
- [ ] **Step 3: Implement** in `maze_motion.py`:
  - `__init__`: add `self.safety_radius = 0.60` and `self.keepout_max_cross_steer = 0.8` (near `self.max_cross_steer`); change the `wedge_slow_m: float = 0.50` constructor default → `0.60`.
  - Add the helper (near `_drive`):
  ```python
      def _keepout_clearance(self, near, ranges):
          """Nearest obstacle in ANY direction (safety bubble): min of the cardinal side clearance and
          the full LIDAR scan, filtering spurious near-zero / inf returns (corners the cardinal perp misses)."""
          finite = [r for r in ranges if r is not None and math.isfinite(r) and r > 0.05]
          return min(near, min(finite)) if finite else near
  ```
  - In `_drive`, after `near` is computed and before the `corridor_follow_command(...)` call, compute `keepout_clear = self._keepout_clearance(near, ranges)`; change the call's 5th positional arg from `near` to `keepout_clear`, and add `safety_radius=self.safety_radius, keepout_max_cross_steer=self.keepout_max_cross_steer` to it.
- [ ] **Step 4: Run → PASS + regression**: `cd "$PKG" && PYTHONPATH="$PKG" python3 -m pytest test/test_maze_motion.py test/test_maze_motion_sim.py test/test_hop_controller.py -q`
- [ ] **Step 5: Commit** `maze_motion.py`, `test/test_maze_motion.py`.

## Validation
1. Full ROS-free suite green: `cd "$PKG" && PYTHONPATH="$PKG" python3 -m pytest test/test_maze_motion.py test/test_maze_motion_sim.py test/test_hop_controller.py test/test_flood_fill_brain.py test/test_junction_log.py test/test_cell_walls.py test/test_maze_sim.py -q`.
2. Rebuild + Gazebo (user-initiated; diagnostics IN): watch the objective collision rate drop sharply, no 2-cell desync, clean RViz map, ideally deeper than ~6.9 m.
3. No merge/push; revert `2fe9e39` before any merge.
