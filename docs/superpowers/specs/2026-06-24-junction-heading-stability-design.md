# Junction Heading Stability (Approach C) — Design

**Status:** approved. Builds on the validated wedge-gate (`79a0796`) on branch `no-progress-watchdog`.

## 1. Motivation (from validation run `20260624_063043`)

The wedge-gate worked (wedge stalls 29→8, all remaining low-`yaw_err`), but the robot still TIMEOUT/latched at 18.30 m because the **same heading drift at open junctions** now fired `front_block` instead (9 stalls; 6/9 `perp_front` in [0.55,0.70) = false sub-threshold, 0/9 < 0.40 = none real, 8/9 `|yaw_err| ≥ 0.30`). Smoking gun: at (1,3) the logged `cross_track` (raw `grid_cross_track`) was **−0.87 / −1.00 m** — when side walls vanish at a junction, the `grid_cross_track` fallback injects a large lateral term into the pure-pursuit setpoint `cardinal + atan2(−cross_track, lookahead)`, swinging the commanded heading ~55° off cardinal → mis-aligned drive → angled LIDAR beam → false `front_block` (and the now-gated wedge). The NH14 dead-maze latch also fired falsely at (1,2) → 1085 s permanent freeze.

**Root (singular, confirmed): the cross-track term over-steers the heading at open junctions.** Fix the cause (cap the cross-track heading authority), make `front_block` robust to residual mis-alignment (symmetric with the wedge-gate), and remove the harmful latch.

## 2. Components

### C1 — Cross-track heading-authority cap (the core)
In `hop_controller.corridor_drive_command`, clamp the cross-track-induced setpoint deviation before it becomes the heading target:
```python
cross_steer = max(-max_cross_steer, min(max_cross_steer, math.atan2(-cross_track, lookahead_m)))
setpoint = cardinal_yaw + cross_steer       # was: cardinal_yaw + math.atan2(-cross_track, lookahead_m)
```
- New kwarg `max_cross_steer: float = 0.25` (rad, ~14°) on `corridor_drive_command` AND `corridor_follow_command` (passed through); `MazeMotion.__init__` adds `self.max_cross_steer = 0.25` and `_drive` passes it.
- In a normal corridor `cross_track` is small (~0.08 → ~6.5° steer < cap) so the cap never binds; it **only** binds when `cross_track` is large (the junction fallback / drift), holding the heading near the cardinal so the beam stays un-angled. The robot crosses the junction straight and re-centers at the next walled cell (where the wall-derived `cross_track` is reliable).

### C2 — `front_block` heading-gate (compose, symmetric with the wedge-gate)
Extract the predicate and gate it on heading. In `MazeMotion`:
```python
def _front_blocked(self, perp, dirn, moved, yaw):
    return (perp[dirn] < self.front_block_m and moved > 0.3
            and abs(_norm(self.target_cardinal - yaw)) < self.front_block_max_yaw)
```
`_drive` uses `front_blocked = self._front_blocked(perp, dirn, moved, yaw)` (keep the variable for the STALL-event `reason`). New `self.front_block_max_yaw = 0.30` (rad). An angled-beam reading (heading off ≥ 0.30 rad) is no longer trusted as a block → the robot re-aligns/continues instead of false-stalling; once aligned, a genuine block still fires. Safe — real walls come from SENSING (projection-median), not `front_block`; a true wall just fails the hop until the deadline or an aligned `front_block` catches it.

### C3 — Remove the NH14 latch
Strip `_maze_exhausted`, the `if self._maze_exhausted(): ... self._latched = True ...` block in `_escape`, the `self._latched` init, and `not self._latched` from the C2 watchdog guard in `step()`. It only ever fired falsely (map corruption made a live maze look dead → permanent freeze); the no-progress watchdog + `self.reopened` bound already make behavior recoverable without it.

## 3. Tests
- **C1** (`test/test_hop_controller.py`): `corridor_drive_command` with a huge `cross_track` and `yaw==cardinal` commands `w == kp_ang * (±max_cross_steer)` (capped, not the uncapped `atan2`); a small `cross_track` (below the cap) is unchanged.
- **C2** (`test/test_maze_motion.py`): `_front_blocked` returns True when aligned + short front + `moved>0.3`; False when `|yaw_err| ≥ front_block_max_yaw` (angled beam); False when `moved ≤ 0.3`.
- **C3**: full suite stays green; no test asserts permanent latching (none existed).
- **Offline regression** (`test/test_maze_motion_sim.py`): all 5 cases still reach the exit, no collision, `max_desync ≤ 1`, `escape_count == 0` (the cap only tightens steering the clean solve never needed; the gate only suppresses angled-beam blocks the clean solve never hit).

## 4. Validation & out-of-scope
- Diagnostic `STALL`/`ESCAPE` instrumentation (`2fe9e39`) stays IN for the validation run (revert before any merge).
- Gazebo watch: the (1,3)/(2,3)/(1,4) junction cluster no longer produces `front_block`/`wedge` false-stalls; the robot threads the junctions past 18.30 m. If a NEW failure mode appears, the instrumentation localizes it.
- Out of scope: the secondary (1,2) post-escape drift cause (the cap + front_block-gate should prevent reaching that state; revisit only if it recurs).
