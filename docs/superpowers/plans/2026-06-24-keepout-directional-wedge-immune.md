# Keep-Out Revision: Directional Clearance + Wedge-Immune — Plan

> Revision of the safety-radius keep-out after run `20260624_220733` regressed (104 STALL / 88 wedge, stuck SW 18.30, collision 36%). Root: the full-scan 360° `_keepout_clearance` slowed to a creep for walls in ANY direction (incl. behind), and that deliberate slow-creep (aligned, v≈0.09) tripped the wedge detector → 88 creep-wedges. Execute inline, TDD. Branch `no-progress-watchdog`. Trailer `Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>`; no backticks.

## Fixes
1. **Directional `_keepout_clearance(near, ranges, amin, ainc)`** — min over the FORWARD hemisphere + sides only (`cos(bearing) >= 0`, i.e. `|bearing| <= 90°`), excluding the rear. A wall behind is not a forward-collision risk. (Catches forward/side/corner returns; drops the rear walls that caused the spurious creep.)
2. **Wedge detector immune to the keep-out slow** — add a branch resetting the wedge baseline when `keepout_clear < self.safety_radius` (the robot is deliberately creeping near a wall, not pinned), like the existing heading-rotation skip. A genuine jam is still caught by the no-progress watchdog. Requires computing `near` + `keepout_clear` EARLY (right after `perp`, before the wedge detector); remove the later duplicate computations.
3. **`wedge_realign_yaw` 0.5 → 0.4** — catch the 6/88 heading-collapse wedges that slipped just under the gate.
(Hold `safety_radius` at 0.60 — re-evaluate after directionality is fixed.)

## Task 1: directional clearance
**Files:** `maze_motion.py`; Test `test/test_maze_motion.py`.
- [ ] **Test** (replace `test_keepout_clearance_uses_full_scan_min`):
```python
def test_keepout_clearance_forward_hemisphere_ignores_rear():
    import math
    m = MazeMotion()
    amin, ainc = -math.pi, math.pi / 2                   # 4 beams: -pi(rear), -pi/2(side), 0(front), pi/2(side)
    ranges = [0.30, 2.0, 0.45, 2.0]                       # rear 0.30 behind -> ignored; front 0.45 counts
    assert abs(m._keepout_clearance(0.88, ranges, amin, ainc) - 0.45) < 1e-9
    assert m._keepout_clearance(0.30, [2.0, 2.0, 2.0, 2.0], amin, ainc) == 0.30   # cardinal `near` smaller
    assert m._keepout_clearance(0.70, [0.02, float('inf'), 0.02, float('inf')], amin, ainc) == 0.70  # spurious/inf filtered -> near
```
- [ ] **Run → FAIL** (old `_keepout_clearance` takes 2 args, not 4): `cd "$PKG" && PYTHONPATH="$PKG" python3 -m pytest test/test_maze_motion.py -k keepout_clearance -v`
- [ ] **Implement** — replace `_keepout_clearance`:
```python
    def _keepout_clearance(self, near, ranges, amin, ainc):
        """Nearest obstacle in the FORWARD hemisphere + sides (cos(bearing) >= 0), EXCLUDING the rear
        (a wall behind is not a forward-collision risk). Filters spurious near-zero / inf returns.
        Catches forward/side/corner returns the cardinal perp misses, without the rear walls that
        caused spurious creep."""
        best = near
        for i, r in enumerate(ranges):
            if r is None or not math.isfinite(r) or r <= 0.05:
                continue
            if math.cos(amin + i * ainc) >= 0.0 and r < best:   # forward half-plane (|bearing| <= 90 deg)
                best = r
        return best
```
- [ ] **Run → PASS** (+ the `_drive` call updated in Task 2; run full motion+sim suite after Task 2).
- [ ] **Commit** with Task 2 (coupled — the signature change forces the `_drive` call update).

## Task 2: wedge-immune + early clearance + gate tune
**Files:** `maze_motion.py`; Test `test/test_maze_motion.py`.
- [ ] **Test** (append):
```python
def test_wedge_immune_to_keepout_slow():
    import math
    m = MazeMotion()
    m.cell = (5, 5); m.hop_dir = (0, 1); m.hop_target = (5, 6); m.target_cardinal = math.pi / 2
    m.phase = 'drive'; m.hop_start = (10.0, 10.0); m.hop_deadline = 1e12
    m.progress_pose = (10.0, 10.0); m.progress_t = 0.0
    n = 360; ranges = [0.50] * n                          # every beam 0.50 < safety_radius 0.60 -> keep-out slow
    m._drive((10.0, 10.0, math.pi / 2),                   # aligned (yaw == cardinal), no positional progress
             (ranges, -math.pi, 2 * math.pi / n), m.wedge_detect_s + 5.0)
    assert m.phase == 'drive'                             # keep-out slow near a wall is NOT a wedge
```
- [ ] **Run → FAIL** (old code wedges → phase 'recover'/'center').
- [ ] **Implement** in `maze_motion.py`:
  - `__init__`: `self.wedge_realign_yaw = 0.5` → `0.4`.
  - In `_drive`, after `perp = cell_wall_perp_dist(...)` (line ~505), add the early clearance:
  ```python
          near = (min(perp['E'], perp['W']) if self.hop_dir[1] != 0 else min(perp['N'], perp['S']))
          keepout_clear = self._keepout_clearance(near, ranges, amin, ainc)   # safety bubble (forward+sides)
  ```
  - Add the wedge-immune branch between the heading-rotation `elif` and the timeout `elif`:
  ```python
          elif keepout_clear < self.safety_radius:    # deliberately slowing near a wall (keep-out), not a pin
              self.progress_pose = (x, y); self.progress_t = t
  ```
  - Remove the now-duplicate later `near = ...` block (the `if self.hop_dir[1] != 0: near = ... else: near = ...`) and the later `keepout_clear = self._keepout_clearance(near, ranges)` line; keep `d_left, d_right = side_distances(...)`, the `self.dbg[...]` lines, and the `corridor_follow_command(... keepout_clear ...)` call (now using the early `keepout_clear`).
- [ ] **Run → PASS + full regression**: `cd "$PKG" && PYTHONPATH="$PKG" python3 -m pytest test/test_maze_motion.py test/test_maze_motion_sim.py test/test_hop_controller.py test/test_flood_fill_brain.py test/test_junction_log.py test/test_cell_walls.py test/test_maze_sim.py -q`
- [ ] **Commit** `maze_motion.py`, `test/test_maze_motion.py` (both tasks together).

## Validation
Rebuild + Gazebo (user-initiated; diagnostics IN): expect 88 creep-wedges → ~0, collision rate down (now that it's not churning), progress past the SW. Revert `2fe9e39` before any merge.
