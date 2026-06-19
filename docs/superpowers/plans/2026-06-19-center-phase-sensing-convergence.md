# `_center` Sensing-Convergence Fix — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: superpowers:subagent-driven-development or executing-plans. TDD; checkbox steps.

**Goal:** Stop the (3,4) junction churn by making `MazeMotion._center` sense from a position-gated, cardinal-aligned, settled pose, commit that read (sense-once), and protect the map with backstops — without letting a wrong commit brick the solve.

**Architecture:** All changes in `maze_motion.py`: 2 new pure predicate helpers, new `__init__` params/state, a rewritten `_center` (+ `_reanchor`/`_route`/`_stuck_backstop` helpers), and a small touch to the two `_drive` hop-failure wall-mark sites. Spec: `docs/superpowers/specs/2026-06-19-center-phase-sensing-convergence-design.md`.

**Branch:** `tighter-corridor-controller` (local only, never push). Commit trailer:
`Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>`.

Paths: `PKG = ros2_ws_tugbot_nav_20260614/src/tugbot_maze`; run pytest from there (absolute:
`/home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260614/src/tugbot_maze`). Commit from repo root.

---

## File Structure
- Modify `PKG/tugbot_maze/maze_motion.py` — helpers, `__init__`, `_center` rewrite, `_drive` touch.
- Modify `PKG/test/test_maze_motion.py` — unit tests for helpers + behaviors. (If absent, create it.)
- Regression: `PKG/test/test_maze_motion_sim.py` (existing end-to-end must stay green).

---

## Task 1: Pure predicate helpers

**Files:** Modify `PKG/tugbot_maze/maze_motion.py`; Test `PKG/test/test_maze_motion.py`.

- [ ] **Step 1 — failing tests** (add to `test/test_maze_motion.py`; create with `from tugbot_maze.maze_motion import within_commit_offset, within_cell_core` + `from tugbot_maze.flood_fill_brain import CELL_SIZE_M` if new):

```python
def test_within_commit_offset():
    assert within_commit_offset(0.3, 0.34, 0.40) is True       # both referenced, within tol
    assert within_commit_offset(0.5, 0.0, 0.40) is False       # x axis too far
    assert within_commit_offset(None, 0.9, 0.40) is False      # open x ignored, y too far
    assert within_commit_offset(None, None, 0.40) is True      # both open -> no constraint


def test_within_cell_core():
    # cell index 4 -> center at 8.0 (CELL_SIZE_M=2). Core = within (1.0 - margin) of center.
    assert within_cell_core(8.0, 4, 0.40) is True              # dead center
    assert within_cell_core(8.55, 4, 0.40) is True             # 0.55 <= 0.60 core
    assert within_cell_core(8.7, 4, 0.40) is False             # 0.70 > 0.60 -> near boundary
    assert within_cell_core(7.0, 4, 0.40) is False             # on the (3,4)/(4,4)... y=7 boundary
```

- [ ] **Step 2 — run, expect ImportError fail.** `python3 -m pytest test/test_maze_motion.py -k "within_" -q`

- [ ] **Step 3 — implement** (add near the top of `maze_motion.py`, after `_dir_name`):

```python
def within_commit_offset(ox, oy, tol: float) -> bool:
    """True if every REFERENCED axis offset (non-None) is within tol. Open axes (None) impose
    no constraint -- they cannot be wall-referenced."""
    return ((ox is None or abs(ox) <= tol) and (oy is None or abs(oy) <= tol))


def within_cell_core(coord: float, cell_idx: int, margin: float) -> bool:
    """True if `coord` (map metres on one axis) is in the central core of the cell -- at least
    `margin` from either cell boundary. Used to reject boundary-straddling poses (sensing) and
    to gate hysteretic re-anchoring."""
    return abs(coord - CELL_SIZE_M * cell_idx) <= (CELL_SIZE_M / 2.0 - margin)
```

- [ ] **Step 4 — run, expect pass.** Same command. **Step 5 — commit** (`feat: _center position-gate predicates (within_commit_offset, within_cell_core)`).

---

## Task 2: New `__init__` params + state

**Files:** Modify `PKG/tugbot_maze/maze_motion.py`.

- [ ] **Step 1** — extend the `__init__` signature (after the Task-6 line ending `half_corridor_m: float = 0.88`):

Change:
```python
                 ang_decel: float = 1.2, wall_seen_m: float = 1.3, half_corridor_m: float = 0.88):
```
to:
```python
                 ang_decel: float = 1.2, wall_seen_m: float = 1.3, half_corridor_m: float = 0.88,
                 align_timeout_s: float = 6.0, commit_offset_tol_m: float = 0.40,
                 boundary_margin_m: float = 0.40, k_corroborate: int = 2):
```

- [ ] **Step 2** — add stores (after `self.half_corridor_m = half_corridor_m`):
```python
        self.align_timeout_s = align_timeout_s
        self.commit_offset_tol = commit_offset_tol_m
        self.boundary_margin_m = boundary_margin_m
        self.k_corroborate = k_corroborate
```

- [ ] **Step 3** — add state (after `self.sensed = set()`):
```python
        self.committed = set()          # cells sensed from a good (gated) pose -- frozen, never re-sensed
        self.align_start = None         # independent timeout baseline for the cardinal-align sub-phase
        self.latched_cardinal = None    # cardinal latched once at align entry (no 45-deg basin jitter)
        self.corrob_cell = None; self.corrob_walls = None; self.corrob_count = 0
        self.locomotion_walls = set()   # (cell,dir) WALLs stamped by hop-failure (re-openable by backstop)
        self.evicted = set()            # cells given one backstop re-sense before declaring stuck
```

- [ ] **Step 4** — sanity import. `python3 -c` (from PKG): `from tugbot_maze.maze_motion import MazeMotion; MazeMotion()` runs without error. **Step 5 — commit** (`feat: _center commit/align/backstop params + state`).

---

## Task 3: Rewrite `_center` (+ helpers)

**Files:** Modify `PKG/tugbot_maze/maze_motion.py`.

- [ ] **Step 1** — replace the entire `_center` method (current body, the `def _center(...)` through its final `return (0.0, 0.0, False)` at the end of the route block) with:

```python
    def _center(self, pose, scan, t):
        x, y, yaw = pose
        ranges, amin, ainc = scan
        # Fast-path: a committed cell is trusted -- skip centering/align/sensing; just re-anchor
        # (hysteresis) and route. (Synthesis: revisits must not re-run the 4 s centering.)
        if self.cell in self.committed:
            self._reanchor(x, y)
            if self.cell == EXIT_CELL:
                self.phase = 'done'
                return (0.0, 0.0, True)
            if self.cell in self.committed:
                return self._route(x, y, t)
            # else re-anchored onto a non-committed neighbour -> fall through and sense it
        # Lateral centering (unchanged primitive).
        if self.center_start is None:
            self.center_start = t
        ox, oy = cell_center_offset(ranges, amin, ainc, yaw)
        v, w, centered = centering_command((x, y, yaw), ox, oy,
                                           tol=self.center_tol_m, yaw_tol=self.yaw_tol_rad,
                                           w_max=self.turn_w_max, kp_ang=self.kp_turn,
                                           kd_ang=self.kd_turn, yaw_rate=self.yaw_rate)
        if not centered and (t - self.center_start) < self.center_timeout_s:
            self.settle_until = t + self.settle_s
            return (v, w, False)
        # Cardinal align as a POSE-FREEZE before sensing: latch the cardinal ONCE (no 45-deg basin
        # jitter), rotate in place via the decel-limited profile to within yaw_tol. Independent
        # align_start/align_timeout (NOT shared with center_start -- that sharing was the bug).
        if self.align_start is None:
            self.align_start = t
            self.latched_cardinal = round(yaw / (math.pi / 2.0)) * (math.pi / 2.0)
        yaw_err = _norm(self.latched_cardinal - yaw)
        aligned = abs(yaw_err) <= self.yaw_tol_rad
        if not aligned and (t - self.align_start) < self.align_timeout_s:
            self.settle_until = t + self.settle_s
            w = profiled_turn_command(yaw, self.latched_cardinal, self.yaw_rate,
                                      ang_decel=self.ang_decel, turn_w_max=self.turn_w_max,
                                      kd=self.kd_turn)
            return (0.0, w, False)
        if t < self.settle_until:
            return (0.0, 0.0, False)
        # Re-anchor (bounded + hysteresis), then exit/committed checks.
        self._reanchor(x, y)
        if self.cell == EXIT_CELL:
            self.phase = 'done'
            return (0.0, 0.0, True)
        if self.cell in self.committed:
            return self._route(x, y, t)
        # Quality gate (BINDING = position), atomic with the sense below.
        ox, oy = cell_center_offset(ranges, amin, ainc, yaw)
        pos_ok = within_commit_offset(ox, oy, self.commit_offset_tol)
        taxis = 0 if (self.hop_dir and self.hop_dir[0] != 0) else 1
        coord = x if taxis == 0 else y
        not_straddling = within_cell_core(coord, self.cell[taxis], self.boundary_margin_m)
        good = aligned and pos_ok and not_straddling
        # Sense iff never sensed (first read, any quality) OR this is a GOOD read (improve/commit).
        # A POOR re-entry of an already-sensed cell does NOT re-sense -> bad poses cannot churn.
        if (self.cell not in self.committed) and (self.cell not in self.sensed or good):
            walls = sense_cell_walls(ranges, amin, ainc, yaw)
            for d, is_wall in walls.items():
                self.brain.mark(self.cell, d, is_wall)
            self.sensed.add(self.cell)
            self.dbg = {'cell': self.cell, 'pose': (round(x, 2), round(y, 2), round(yaw, 2)),
                        'off': (None if ox is None else round(ox, 2),
                                None if oy is None else round(oy, 2)),
                        'good': good, 'yaw_err': round(yaw_err, 3),
                        'pos_ok': pos_ok, 'straddle': not not_straddling, 'walls': walls}
            if good:
                if self.corrob_cell == self.cell and self.corrob_walls == walls:
                    self.corrob_count += 1
                else:
                    self.corrob_cell = self.cell; self.corrob_walls = dict(walls); self.corrob_count = 1
                if self.corrob_count >= self.k_corroborate:
                    self.committed.add(self.cell)
        return self._route(x, y, t)

    def _reanchor(self, x, y):
        """Bounded (<=1 cell) re-anchor of self.cell to the odom cell, with hysteresis: only flip
        across a boundary when the pose is clearly inside the new cell (within_cell_core), so a
        boundary-straddling pose can't make self.cell (and next_cell) flip."""
        anchored = pose_to_cell(x, y)
        if not in_grid(anchored):
            return
        if abs(anchored[0] - self.cell[0]) + abs(anchored[1] - self.cell[1]) != 1:
            if anchored == self.cell:
                return
            # >1 cell away: distrust (keep dead-reckoned cell), as the original bounded rule did.
            return
        ax = 0 if anchored[0] != self.cell[0] else 1
        coord = x if ax == 0 else y
        if within_cell_core(coord, anchored[ax], self.boundary_margin_m):
            self.cell = anchored

    def _route(self, x, y, t):
        """Pick next_cell and set up the hop, or run the deadlock backstop if boxed in."""
        nxt = self.brain.next_cell(self.cell)
        if nxt is None:
            return self._stuck_backstop(t)
        if nxt != self.hop_target:
            self.brain.mark_traversal(self.cell, nxt)
        self.hop_target = nxt
        self.hop_dir = (nxt[0] - self.cell[0], nxt[1] - self.cell[1])
        self.target_cardinal = math.atan2(self.hop_dir[1], self.hop_dir[0])
        self.hop_start = (x, y)
        self.turn_in_tol = 0
        self.turn_start = t
        self.center_start = None
        self.align_start = None; self.latched_cardinal = None
        self.phase = 'turn'
        return (0.0, 0.0, False)

    def _stuck_backstop(self, t):
        """next_cell==None. Before terminal 'stuck', give the cell ONE corrective re-sense: a
        committed-or-locomotion false WALL is otherwise permanent (walls are monotonic) and can
        disconnect the exit. Evict the cell from committed+sensed, re-open its locomotion walls,
        and re-sense next tick. Declare stuck only if still disconnected after that."""
        cell = self.cell
        has_loco = any(c == cell for (c, _d) in self.locomotion_walls)
        if cell not in self.evicted and (cell in self.committed or has_loco):
            self.committed.discard(cell)
            self.sensed.discard(cell)
            for (c, d) in list(self.locomotion_walls):
                if c == cell:
                    self.brain.mark(c, d, is_wall=False)     # re-open; forced re-sense overwrites
                    self.locomotion_walls.discard((c, d))
            self.corrob_cell = None; self.corrob_walls = None; self.corrob_count = 0
            self.center_start = None
            self.align_start = None; self.latched_cardinal = None
            self.evicted.add(cell)
            self.settle_until = t + self.settle_s
            return (0.0, 0.0, False)                          # stay in center -> re-sense next tick
        self.phase = 'stuck'
        return (0.0, 0.0, False)
```

- [ ] **Step 2** — quick import sanity (`MazeMotion()` constructs; no NameError). **Step 3 — commit** (`feat: rewrite _center -- position-gated sense-once/commit + reanchor hysteresis + deadlock backstop`).

(Note: the offline regression is exercised in Task 5; if `_center`'s new gates change the offline solve, that surfaces there.)

---

## Task 4: `_drive` hop-failure touch + align resets

**Files:** Modify `PKG/tugbot_maze/maze_motion.py`.

- [ ] **Step 1 — record + un-commit at the WEDGE giveup site.** In `_drive`, the wedge branch, change:
```python
            if self.hop_attempts[key] >= self.max_hop_attempts:      # give up this edge (last resort)
                self.brain.mark(self.cell, dirn, is_wall=True)
                self.hop_attempts.pop(key, None)
                self.sensed.discard(self.cell)
                self.phase = 'center'
```
to:
```python
            if self.hop_attempts[key] >= self.max_hop_attempts:      # give up this edge (last resort)
                self.brain.mark(self.cell, dirn, is_wall=True)
                self.locomotion_walls.add((self.cell, dirn))         # re-openable by the backstop
                self.committed.discard(self.cell)                    # un-commit: a fresh sense may re-open
                self.hop_attempts.pop(key, None)
                self.sensed.discard(self.cell)
                self.phase = 'center'
```
And in that same wedge branch, immediately after `self.center_start = None` add:
```python
            self.align_start = None; self.latched_cardinal = None
```

- [ ] **Step 2 — record + un-commit at the FRONT-BLOCK giveup site.** Change:
```python
            if self.hop_attempts[key] >= self.max_hop_attempts:
                self.brain.mark(self.cell, dirn, is_wall=True)
                self.hop_attempts.pop(key, None)
            else:
                self.sensed.discard(self.cell)                       # re-sense from a clean position
            self.settle_until = t + self.settle_s
            self.center_start = None
            self.phase = 'center'
```
to:
```python
            if self.hop_attempts[key] >= self.max_hop_attempts:
                self.brain.mark(self.cell, dirn, is_wall=True)
                self.locomotion_walls.add((self.cell, dirn))         # re-openable by the backstop
                self.committed.discard(self.cell)                    # un-commit: a fresh sense may re-open
                self.hop_attempts.pop(key, None)
            else:
                self.sensed.discard(self.cell)                       # re-sense from a clean position
            self.settle_until = t + self.settle_s
            self.center_start = None
            self.align_start = None; self.latched_cardinal = None
            self.phase = 'center'
```

- [ ] **Step 3 — align resets at the remaining center handoffs.** In `_drive` ARRIVAL branch, after `self.center_start = None` (the `moved >= CELL...` block) add `self.align_start = None; self.latched_cardinal = None`. In `_recover`, after `self.center_start = None` add `self.align_start = None; self.latched_cardinal = None`.

- [ ] **Step 4 — commit** (`feat: record+un-commit locomotion wall-marks; reset align state at all center handoffs`).

---

## Task 5: Behavior unit tests + offline regression

**Files:** Modify `PKG/test/test_maze_motion.py`, run `PKG/test/test_maze_motion_sim.py`.

Use a synthetic-scan helper so tests are deterministic. Build scans with `MazeSim` over crafted
geometry (as `test_maze_motion_sim.py::test_symmetric_following_converges_to_centerline` does), or
construct a single-cell 4-wall box and place the robot. Helper to add to the test file:

```python
import math
from tugbot_maze.maze_motion import MazeMotion
from tugbot_maze.maze_sim import MazeSim
from tugbot_maze.flood_fill_brain import CELL_SIZE_M, pose_to_cell

def _box_scan(sim, x, y, yaw):
    sim.x, sim.y, sim.yaw = x, y, yaw
    return sim.scan(n_beams=360, fov_rad=2 * math.pi)
```

- [ ] **Step 1 — tests** (add; each asserts one spec behavior):
  1. `test_good_read_commits_after_k_and_discard_is_inert`: a cell sensed from a centered+cardinal pose K_corroborate times enters `committed`; then `m.sensed.discard(cell)` followed by another `_center` tick does NOT change the brain's marks for that cell and leaves `next_cell` unchanged. (Drive `_center` at a crafted in-position pose; assert `cell in m.committed`; snapshot brain edges; discard+step; assert edges identical.)
  2. `test_poor_read_not_committed_and_poor_reentry_skips`: at a boundary-straddling pose (`within_cell_core` False), a first read marks but does NOT commit (`cell not in committed`); a second straddling read of the same (now-sensed) cell does NOT re-call `sense_cell_walls` (assert brain marks unchanged across the 2nd poor tick).
  3. `test_no_false_wall_across_yaw_tol_and_small_offset`: from a 4-open box (or a known cell), sensing at yaw within ±`yaw_tol` and ≤0.3 m offset never marks a WALL on an actually-open edge.
  4. `test_injected_bad_read_not_permanently_locked`: force `committed.add(cell)` with a brain state where `next_cell(cell) is None`; one `_center`/`_route` tick must run the backstop (evict: `cell not in committed`, `cell in m.evicted`) rather than going straight to `phase=='stuck'`; only a second dead-end declares `stuck`.
  5. `test_commit_freezes_sensing_not_map`: with `cell in committed`, a `_drive` hop-failure that marks a WALL still removes the cell from `committed` (assert `cell not in committed` and `(cell,dir) in m.locomotion_walls`).

- [ ] **Step 2 — run new tests, iterate to green.** `python3 -m pytest test/test_maze_motion.py -q`. Fix the implementation (not the tests) on any real failure; if a test encodes a wrong expectation, correct the test with a noted reason.

- [ ] **Step 3 — OFFLINE REGRESSION (critical gate).** `python3 -m pytest test/test_maze_motion_sim.py -q` → the end-to-end solve + convergence test MUST stay green. If a `test_reaches_exit_*` case fails, the new gates changed the offline solve; tune in order, re-running each time (do NOT relax test asserts): (a) `commit_offset_tol_m` up toward 0.45; (b) `align_timeout_s`; (c) `boundary_margin_m` down toward 0.30. Record final values.

- [ ] **Step 4 — FULL suite.** `python3 -m pytest test/test_flood_fill_*.py test/test_cell_walls.py test/test_hop_controller.py test/test_maze_sim.py test/test_maze_motion.py test/test_maze_motion_sim.py test/test_wall_localize.py test/test_wall_relocalize_sim.py -q` → all green.

- [ ] **Step 5 — commit** (`test: _center commit/gate/backstop behavior + regression green`).

---

## Task 6: Adversarial implementation review

**Not code.** Dispatch an adversarial review (workflow or reviewers) over the `_center` diff vs the spec: verify the gate is position-binding, `should_sense` logic, corroboration, backstop eviction (no infinite loop via `evicted`), align-reset completeness at all handoff sites, and that `_drive`/`_recover`/primitives are otherwise unchanged. Fix any confirmed issue, re-run the full suite.

---

## Task 7: Gazebo confirmation (user-gated)

- [ ] Ensure no stray sims (user clears via `!` pkill). Confirm the egg-link/symlink install reflects current src (it does; `build/tugbot_maze/tugbot_maze` -> src).
- [ ] Run **multiple seeded** runs: `tools/run_flood_fill_maze.sh 1800 false true odom_locked true` (launch the script ALONE, background). Monitor `launch.log`.
- [ ] Assert per the spec success criteria: committed (3,4) wall-set == ground truth, churn gone at (3,4), progress past it; log odom-yaw error. Stage success = wedge/churn class gone + past ~14 cells; full = `EXIT_REACHED`.
- [ ] Record outcome in the `flood-fill-maze-solver` memory.

---

## Final Review
After Tasks 1–6 green + Gazebo observed: final holistic review of the branch diff, then
`superpowers:finishing-a-development-branch` (local only; merge to main only if the user asks).
