# Visited-Edge Giveup Guard (+ cap loosen) — Design

**Status:** approved. Builds on approach C (`8c48e15`/`61a2f47`/`d8d6dc3`) on branch `no-progress-watchdog`.

## 1. Motivation (validation run `20260624_193304`)

Approach C fixed the heading drift (wedge & heading-`front_block` false-stalls gone, collision 53→36%, latch removed), but the robot livelocked at **(3,4)** (18 escapes / 18 unsticks, ~1536 s). Mechanism: (3,4) is a corner cell (walls N+E, open W+S); the robot drove ~0.82 m off-center into the N/E corner and, aligned there, its W-facing beam read the corner at perp 0.69 (<0.70) → a `front_block` on the **W edge it just came through** (from the visited cell (2,4)). After 3 such failures the `_drive` giveup `brain.mark`ed (3,4)·W a wall → severed the reverse (`can_reverse=False` for all 18 escapes, since `prev=(2,4)` is reached via W) → `_unstick` reopens, retries, re-marks → fruitless livelock.

**Two root issues:** (1) the cross-track cap (`max_cross_steer=0.25`) is too tight to recover a genuine ~0.8 m offset → corner jam; (2) a locomotion giveup walled an edge to an already-visited cell → corrupted a known-open edge and severed the reverse.

## 2. Fixes

### Fix 1 — a locomotion giveup must never wall an edge to a VISITED cell (core)
A cell the robot has occupied is traversable by construction; a hop failure toward it is a positioning problem, not a wall. Guard every locomotion wall-mark:
```python
nb = (self.cell[0] + DIRS[dirn][0], self.cell[1] + DIRS[dirn][1])
if nb not in self.visited:
    self.brain.mark(self.cell, dirn, is_wall=True)
    self._stamp_loco_wall(self.cell, dirn)
    self.committed.discard(self.cell)
    ... (pop the relevant counter)
```
Applied in: `_drive` wedge-giveup branch, `_drive` front_block/deadline-giveup branch, and `_escape` Tier-2 give-up. When the neighbor IS visited, skip the wall-marking entirely; the existing flow (recover/center, or the escape's reverse) re-approaches instead. The hop_attempts/failed_hops pop and phase transition stay as-is so the robot still re-routes/re-centers (it just doesn't corrupt the map or wall itself in).

### Fix 2 — loosen `max_cross_steer` 0.25 → 0.35
~20° of cross-track steering authority recovers the genuine ~0.8 m offset that the 14° cap could not, while staying below the `front_block` heading-gate (0.30) and wedge-gate (0.5) so the heading-swing false-stalls don't return. Change the default in `corridor_drive_command`, `corridor_follow_command`, and `MazeMotion.__init__`.

## 3. Why safe
- Fix 1: walling a visited-cell edge is **always wrong** (the robot traversed it). Real walls still come from SENSING; a real wall to an UNvisited cell is still walled (unchanged). The bounded retry/escape/watchdog still terminates the loop — now by reversing+re-approaching rather than self-walling.
- Fix 2: gates (heading 0.30 / wedge 0.5) still catch any heading swing the looser cap permits; normal corridors (small cross_track) are unaffected.

## 4. Tests
- **Fix 1** (`test/test_maze_motion.py`): a `_drive` giveup whose forward neighbor is in `self.visited` does NOT wall the edge (`not brain.is_wall`), and the cell is not falsely walled-in; the same giveup with an UNvisited neighbor still walls (regression).
- **Fix 2** (`test/test_hop_controller.py` or motion): `max_cross_steer` default is 0.35 across the three sites.
- **Offline regression** green (reaches exit, `escape_count==0`).

## 5. Validation
Diagnostic instrumentation stays IN. Gazebo watch: the robot reverses out of (3,4)-class pockets and re-approaches (no self-walling, no 18-escape livelock); progress past 18 m. Revert `2fe9e39` before any merge.
