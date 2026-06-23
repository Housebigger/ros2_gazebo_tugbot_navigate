# No-Progress Watchdog + Escalating Escape — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the falsified per-cell stall watchdog with a no-progress watchdog (monotonic `visited`/`explore_t` clock + confinement predicate, checked at the top of `step()` so it covers `stuck`), an escalating escape (re-approach → give-up-edge + one-cell reverse + flood-reroute), and a failed-hop deprioritizer — keeping the correct arrival-time `mark_traversal`.

**Architecture:** All in `MazeMotion` (`maze_motion.py`). Built on a fresh branch off `main` carrying only the kept `mark_traversal` fix (`024bd87`). Pairs with spec `docs/superpowers/specs/2026-06-23-no-progress-watchdog-design.md` — **read it, especially §4 (components), §10 (resolved risks), and §11 (the 8 must-fixes this plan encodes).**

**Tech Stack:** Python 3.12, pytest. ROS-free motion layer.

**Conventions for every task:**
- `PKG = /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260614/src/tugbot_maze`
- `REPO = /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate`
- Run tests: `cd "$PKG" && PYTHONPATH="$PKG" python3 -m pytest <path> -v`
- Branch `no-progress-watchdog` (created in Task 1; **never push**).
- Commit trailer (own line, blank line before): `Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>`. **No backticks inside `git commit -m`.** Stage only each task's files (`git -C "$REPO" add <path>`; NEVER `git add -A` — the repo has unrelated tracked build/ churn).
- Imports already in `maze_motion.py`: `math`; from `flood_fill_brain`: `DIRS, OPP, ENTRANCE_CELL, EXIT_CELL, CELL_SIZE_M, in_grid, _dir_name` (verify the exact import line; `_dir_name` is module-level in `maze_motion.py`). `brain.mark(cell,d,is_wall)` is **symmetric** (marks both directed reps).

---

## File Structure

| File | Responsibility | Task |
|------|----------------|------|
| (git) branch `no-progress-watchdog` off `main` + cherry-pick `024bd87` + docs | clean base | 1 |
| `tugbot_maze/maze_motion.py` | `__init__` state, `_track_cell`, `_confined` (C1); `_escape` + `_backout` guard (C3); `step()` C2 wiring + NH14 latch (C2/C4); `_drive`/`_unstick` failed-hop deprioritizer (C5) | 2–5 |
| `test/test_maze_motion.py` | unit tests for C1–C5 | 2–5 |
| `test/test_maze_motion_sim.py` | `escape_count` plumbing, calibration, `esc==0` gate, churn fixture | 6 |

---

## Task 1: Branch setup + clean base

**No TDD — infrastructure.** Produces the `024bd87` base (main + the kept `mark_traversal` fix) on a fresh branch, with the spec+plan docs, building green.

- [ ] **Step 1: Verify no uncommitted SOURCE changes** (only regenerable build/ artifacts may be dirty)

Run: `git -C "$REPO" status --porcelain | grep -vE "ros2_ws_tugbot_nav_20260614/(build|install|log)/" | grep -v "^$"`
Expected: empty (only build/install/log churn). If any source/docs file is listed, STOP and report — do not discard it.

- [ ] **Step 2: Create the branch off main + cherry-pick the kept fix + bring docs**

```bash
cd "$REPO"
git stash push -u -- ros2_ws_tugbot_nav_20260614/build ros2_ws_tugbot_nav_20260614/install ros2_ws_tugbot_nav_20260614/log 2>/dev/null || true
git checkout main
git checkout -b no-progress-watchdog
git cherry-pick 024bd87        # the kept arrival-time mark_traversal fix + its 2 tests
# bring the spec + this plan onto the new branch (they live on decision-cell-stall-watchdog)
git checkout decision-cell-stall-watchdog -- docs/superpowers/specs/2026-06-23-no-progress-watchdog-design.md docs/superpowers/plans/2026-06-23-no-progress-watchdog.md
git add docs/superpowers/specs/2026-06-23-no-progress-watchdog-design.md docs/superpowers/plans/2026-06-23-no-progress-watchdog.md
git commit -m "docs: carry no-progress-watchdog spec + plan onto the implementation branch

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```
Note: if `git checkout main` fails on dirty tracked build files, the implementer may `git checkout -f main` (build artifacts are regenerable) — but ONLY after Step 1 confirmed no source changes.

- [ ] **Step 3: Confirm the base is clean (no watchdog/stall code) and tests are green**

Run:
```bash
cd "$PKG" && PYTHONPATH="$PKG" python3 -m pytest test/test_maze_motion.py test/test_maze_motion_sim.py -q
grep -nE "_track_cell|cell_since_t|stall|_escape_stall" tugbot_maze/maze_motion.py || echo "CLEAN BASE"
```
Expected: all base tests pass (the 22 `test_maze_motion` tests incl. the two `mark_traversal` tests, and the sim tests); grep prints `CLEAN BASE`.

- [ ] **Step 4: Rebuild** (so later Gazebo uses this branch)

Run: `cd "$REPO/ros2_ws_tugbot_nav_20260614" && source /opt/ros/jazzy/setup.bash && colcon build --packages-select tugbot_maze --symlink-install 2>&1 | tail -3`
Expected: `Finished <<< tugbot_maze`.

---

## Task 2: C1 — exploration-progress clock + confinement (`__init__`, `_track_cell`, `_confined`)

**Files:** Modify `tugbot_maze/maze_motion.py`; Test `test/test_maze_motion.py`.

This task adds ALL new state (used by later tasks too), the `_track_cell` clock, and `_confined`, and wires `_track_cell` into `step()` and the top of `_route`. No solving behavior changes yet (the C2 check arrives in Task 4), so the offline regression must stay green.

- [ ] **Step 1: Write the failing tests** — append to `test/test_maze_motion.py`:

```python
def _two_exit_brain(cell, open_a, open_b):
    """cell open on exactly open_a/open_b (in-grid), walled elsewhere."""
    from tugbot_maze.flood_fill_brain import FloodFillBrain, DIRS, in_grid
    b = FloodFillBrain()
    for d, (dx, dy) in DIRS.items():
        nb = (cell[0] + dx, cell[1] + dy)
        if in_grid(nb) and d not in (open_a, open_b):
            b.mark(cell, d, True)
    return b


def test_track_cell_seeds_unconditionally_at_startup():
    m = MazeMotion()                       # cell == last_seen_cell == ENTRANCE_CELL, NO cell change
    m._track_cell(1.78e9)                  # large absolute clock, tick 1
    assert m.explore_t == 1.78e9           # seeded even with no cell change (MF2)
    assert ENTRANCE_CELL in m.visited      # entrance counts as visited ground


def test_track_cell_new_ground_resets_explore_t_and_tier():
    m = MazeMotion(); m._track_cell(1000.0)   # seed at entrance
    m.escape_tier = 2
    m.cell = (1, 1)                            # advance to NEW ground
    m._track_cell(1005.0)
    assert (1, 1) in m.visited and m.explore_t == 1005.0
    assert m.prev_cell == ENTRANCE_CELL and m.escape_tier == 0   # progress clears escalation


def test_track_cell_revisit_does_not_reset_explore_t():
    m = MazeMotion(); m._track_cell(1000.0)    # visit entrance
    m.cell = (1, 1); m._track_cell(1005.0)     # new ground -> explore_t = 1005
    m.cell = ENTRANCE_CELL; m._track_cell(1010.0)   # REVISIT (already in visited)
    assert m.explore_t == 1005.0               # revisit must NOT advance the clock (F1 fix)


def test_confined_true_for_small_footprint_false_for_large():
    m = MazeMotion(); m.no_progress_s = 90.0; m.confine_k = 6
    m.cell = (5, 5)
    m.recent = [(100.0, c) for c in [(5, 5), (5, 6), (5, 5), (5, 6)]]  # 2 distinct in-window
    assert m._confined(120.0) is True
    m.recent = [(100.0, c) for c in [(1, 1), (2, 2), (3, 3), (4, 4), (5, 5), (6, 6), (7, 7)]]  # 7 distinct
    assert m._confined(120.0) is False
    m.recent = []                              # frozen single cell -> footprint {self.cell} size 1
    assert m._confined(120.0) is True


def test_confined_prunes_out_of_window_entries():
    m = MazeMotion(); m.no_progress_s = 90.0; m.confine_k = 2
    m.cell = (5, 5)
    # old, out-of-window distinct cells must be pruned so they don't inflate the footprint
    m.recent = [(10.0, (1, 1)), (10.0, (2, 2)), (10.0, (3, 3)), (200.0, (5, 6))]
    assert m._confined(250.0) is True          # only (5,6)+(5,5) within [160,250] -> 2 <= K
```

- [ ] **Step 2: Run to verify they fail**

Run: `cd "$PKG" && PYTHONPATH="$PKG" python3 -m pytest test/test_maze_motion.py -k "track_cell or confined" -v`
Expected: FAIL — `AttributeError` (`_track_cell`, `_confined`, `visited`, `explore_t`, `recent`, `escape_tier`, `confine_k`, `no_progress_s` undefined).

- [ ] **Step 3a: Add `__init__` state.** In `maze_motion.py`, after the line `self.dbg = {}                   # last-tick diagnostics ...` (end of `__init__`, ~line 117), add:

```python
        # --- no-progress watchdog + escalating escape + failed-hop deprioritizer ---
        self.last_seen_cell = ENTRANCE_CELL   # for cell-change detection (prev_cell)
        self.prev_cell = None                 # cell occupied just before the current one (reverse target)
        self.visited = {ENTRANCE_CELL}        # cells ever occupied (monotonic; seeds the entrance)
        self.explore_t = None                 # sim-time of last visited-growth (None = unseeded)
        self.recent = []                      # [(t, cell)] appended on cell-change; rolling confinement window
        self.escape_tier = 0                  # current escalation level (0 = none)
        self.escape_count = 0                 # MONOTONIC observable (calibration/regression gate)
        self._escape_backout = False          # tags an escape reverse (vs a dead-end back-out)
        self.no_progress_s = 90.0             # window (calibrated in Task 6)
        self.confine_k = 6                    # max distinct cells in-window to count as "confined" (Task 6)
        self.max_escape_tier = 2
        self.failed_hops = {}                 # (cell,dir) -> consecutive cross-occupation failed-hop count
        self.failed_hop_limit = 3
        self._latched = False                 # NH14 dead-maze permanent stop (set in _escape, Task 4)
```
(Note: `self.visited` seeds `{ENTRANCE_CELL}` so the unconditional block in `_track_cell` is consistent; `explore_t` is seeded by `_track_cell`, not here.)

- [ ] **Step 3b: Add `_track_cell` and `_confined`.** Insert these two methods just before `def _route` (~line 236):

```python
    def _track_cell(self, t):
        """Maintain the exploration-progress clock + confinement record. The seed and visited-update
        blocks run UNCONDITIONALLY (NOT gated on a cell change): at startup cell == last_seen_cell ==
        ENTRANCE_CELL, so a change-gated seed would never fire -> explore_t stays None -> the C2
        watchdog would be permanently disabled (F2 returns). Idempotent across the two calls
        (step() and the top of _route, which closes the in-tick _reanchor lag)."""
        if self.explore_t is None:                       # seed (absolute Gazebo clock ~1.78e9)
            self.explore_t = t
        if self.cell not in self.visited:                # NEW GROUND
            self.visited.add(self.cell)
            self.explore_t = t
            self.escape_tier = 0                         # real progress clears in-flight escalation
        if self.cell != self.last_seen_cell:             # cell-change bookkeeping
            self.prev_cell = self.last_seen_cell
            self.last_seen_cell = self.cell
            self.recent.append((t, self.cell))

    def _confined(self, t):
        """True iff the robot stayed within <= confine_k distinct cells over the last no_progress_s
        window. A tight ping-pong keeps a small rolling footprint (confined); a long legitimate
        backtrack sweeps many distinct cells (not confined -> the watchdog stays silent)."""
        self.recent = [(t2, c) for (t2, c) in self.recent if t2 >= t - self.no_progress_s]
        footprint = {c for (_, c) in self.recent}
        footprint.add(self.cell)
        return len(footprint) <= self.confine_k
```

- [ ] **Step 3c: Call `_track_cell` in `step()`.** In `step()`, immediately after `self.prev_yaw = yaw; self.prev_t = t` (~line 123) and before `if self.phase == 'done'`, add:

```python
        self._track_cell(t)
```

- [ ] **Step 3d: Call `_track_cell` at the top of `_route`.** Make `_route` begin (right after the docstring, before `nxt = self.brain.next_cell(self.cell)`):

```python
        self._track_cell(t)                              # catch an in-tick _reanchor cell change
```

- [ ] **Step 4: Run to verify they pass + no regression**

Run: `cd "$PKG" && PYTHONPATH="$PKG" python3 -m pytest test/test_maze_motion.py test/test_maze_motion_sim.py -q`
Expected: PASS (5 new + all base unit + the offline regression — `_track_cell` is pure bookkeeping, the solve is unchanged).

- [ ] **Step 5: Commit**

```bash
git -C "$REPO" add ros2_ws_tugbot_nav_20260614/src/tugbot_maze/tugbot_maze/maze_motion.py ros2_ws_tugbot_nav_20260614/src/tugbot_maze/test/test_maze_motion.py
git -C "$REPO" commit -m "feat: exploration-progress clock + confinement (_track_cell, _confined)

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 3: C3 — escalating escape (`_escape`) + `_backout` guard

**Files:** Modify `tugbot_maze/maze_motion.py`; Test `test/test_maze_motion.py`.

`_escape` is unit-testable in isolation (call it directly). The `_backout` guard is REQUIRED here: `_escape` sets `_escape_backout=True`; `_backout` must clear it on leaving (else the C2 suppression `not _escape_backout` in Task 4 would permanently disable the watchdog), and an escape-reverse timeout must not charge the dead-end `backout_attempts`/`backout_count`.

- [ ] **Step 1: Write the failing tests** — append to `test/test_maze_motion.py`:

```python
def test_escape_tier1_reverses_to_prev_keeps_edge_open():
    b = _two_exit_brain((5, 5), 'N', 'S')      # came from S=(5,4); forward N=(5,6)
    m = MazeMotion(b)
    m.cell = (5, 5); m.last_seen_cell = (5, 5); m.prev_cell = (5, 4)
    m.hop_dir = (0, 1); m.hop_target = (5, 6); m.visited = {(5, 4), (5, 5)}
    m._escape((10.0, 10.0, 0.0), 100.0)
    assert m.phase == 'backout' and m.backout_target == (5, 4)
    assert m.escape_tier == 1 and m.escape_count == 1 and m._escape_backout is True
    assert not b.is_wall((5, 5), 'N')          # Tier-1 leaves the forward edge OPEN (re-approach)
    assert abs(m.backout_cardinal - math.pi / 2) < 1e-9   # face N -> reverse S into prev


def test_escape_tier2_gives_up_edge_then_reverses():
    b = _two_exit_brain((5, 5), 'N', 'S')
    m = MazeMotion(b)
    m.cell = (5, 5); m.last_seen_cell = (5, 5); m.prev_cell = (5, 4)
    m.hop_dir = (0, 1); m.hop_target = (5, 6); m.visited = {(5, 4), (5, 5)}
    m.escape_tier = 1                          # already escalated once
    m._escape((10.0, 10.0, 0.0), 100.0)
    assert m.escape_tier == 2
    assert b.is_wall((5, 5), 'N')              # Tier-2 GIVES UP the forward edge (real brain wall)
    assert (5, 5) not in m.committed
    assert m.phase == 'backout' and m.backout_target == (5, 4)


def test_escape_tier_caps_at_max():
    b = _two_exit_brain((5, 5), 'N', 'S')
    m = MazeMotion(b)
    m.cell = (5, 5); m.last_seen_cell = (5, 5); m.prev_cell = (5, 4)
    m.hop_dir = (0, 1); m.hop_target = (5, 6); m.visited = {(5, 4), (5, 5)}
    m.escape_tier = 2
    m._escape((10.0, 10.0, 0.0), 100.0)
    assert m.escape_tier == 2                   # capped at max_escape_tier


def test_escape_resets_explore_t_every_entry():
    b = _two_exit_brain((5, 5), 'N', 'S')
    m = MazeMotion(b)
    m.cell = (5, 5); m.last_seen_cell = (5, 5); m.prev_cell = (5, 4)
    m.hop_dir = (0, 1); m.hop_target = (5, 6); m.visited = {(5, 4), (5, 5)}
    m.explore_t = 10.0
    m._escape((10.0, 10.0, 0.0), 500.0)
    assert m.explore_t == 500.0                 # reset on entry (no busy-re-fire)


def test_escape_no_reverse_falls_to_unstick():
    b = _two_exit_brain((5, 5), 'N', 'S')       # (5,5) has E/W walled, N/S open
    m = MazeMotion(b)
    m.cell = (5, 5); m.last_seen_cell = (5, 5); m.prev_cell = (1, 1)   # NOT adjacent
    m.hop_dir = (0, 1); m.hop_target = (5, 6); m.visited = {(5, 5)}
    m._escape((10.0, 10.0, 0.0), 100.0)
    assert m.phase in ('center', 'stuck')       # handed to _unstick (reopened->center, or stuck)


def test_escape_backout_timeout_does_not_charge_dead_end_budget():
    m = MazeMotion()
    m.cell = (5, 5); m._escape_backout = True
    m.backout_start = (10.0, 10.0); m.backout_deadline = 0.0   # immediate timeout, no arrival
    bc0 = m.backout_count
    m._backout((10.0, 10.0, 0.0), 1.0)
    assert m.backout_attempts.get((5, 5), 0) == 0     # escape-reverse timeout must NOT charge it
    assert m.backout_count == bc0                     # nor inflate the dead-end metric
    assert m._escape_backout is False                 # cleared on leaving backout (re-enables C2)


def test_churn_escape_escalates_and_reroutes():
    """MF8 churn-escape: under repeated no-progress fires in a confined region, the escape escalates
    Tier-1 -> Tier-2, gives up the blocked edge, and next_cell then routes to a DIFFERENT cell
    (flood reroute) -- so the robot provably leaves the churn instead of looping. Deterministic
    FSM-level fixture (full physical liveness is validated in Gazebo)."""
    from tugbot_maze.flood_fill_brain import FloodFillBrain
    b = FloodFillBrain()                              # open map; (3,3) routes N toward the exit
    m = MazeMotion(b)
    m.cell = (3, 3); m.last_seen_cell = (3, 3); m.prev_cell = (3, 2)   # came from S=(3,2)
    m.hop_dir = (0, 1); m.hop_target = (3, 4)         # forward N toward exit
    m.visited = {(3, 2), (3, 3)}
    assert b.next_cell((3, 3)) == (3, 4)              # baseline: routes forward N
    m._escape((6.0, 6.0, 0.0), 100.0)                 # fire 1 -> Tier 1 (edge stays open)
    assert m.escape_tier == 1 and not b.is_wall((3, 3), 'N')
    m._escape((6.0, 6.0, 0.0), 200.0)                 # fire 2 (no new ground) -> Tier 2 gives up edge
    assert m.escape_tier == 2 and b.is_wall((3, 3), 'N')
    assert b.next_cell((3, 3)) != (3, 4)              # liveness: flood reroutes off the walled edge
    assert m.escape_count == 2
```

- [ ] **Step 2: Run to verify they fail**

Run: `cd "$PKG" && PYTHONPATH="$PKG" python3 -m pytest test/test_maze_motion.py -k "escape or churn" -v`
Expected: FAIL — `_escape` undefined; `_backout` does not clear `_escape_backout`.

- [ ] **Step 3a: Add `_escape`.** Insert just after `_unstick` (after its `return (0.0, 0.0, False)` at ~line 331), before `_turn`:

```python
    def _escape(self, pose, t):
        """No-progress escape (top-of-step watchdog fired: no new ground for no_progress_s while
        confined). Escalates: Tier 1 = decisive one-cell reverse to the adjacent known-open prev_cell
        (fresh re-approach clears a position-dependent false front_block); Tier 2 = ALSO give up the
        blocked forward edge (real brain wall, reopenable by _unstick) so flood/next_cell route to the
        nearest optimistic frontier (flood already IS 'retreat to nearest unexplored junction'). No
        valid reverse -> hand to _unstick AT MOST ONCE this tick (terminal; C2 revives next tick)."""
        x, y, yaw = pose
        self.escape_count += 1
        self.escape_tier = min(self.escape_tier + 1, self.max_escape_tier)
        self.explore_t = t                                   # reset on EVERY entry (no busy-re-fire)
        pc = self.prev_cell
        man = None if pc is None else abs(pc[0] - self.cell[0]) + abs(pc[1] - self.cell[1])
        d_prev = (_dir_name((pc[0] - self.cell[0], pc[1] - self.cell[1]))
                  if (pc is not None and man == 1) else None)
        can_reverse = d_prev is not None and not self.brain.is_wall(self.cell, d_prev)
        if self.escape_tier >= 2 and self.hop_dir is not None:   # Tier 2+: GIVE UP the blocked edge
            dirn = _dir_name(self.hop_dir)
            self.brain.mark(self.cell, dirn, is_wall=True)       # the real routing change (symmetric)
            self._stamp_loco_wall(self.cell, dirn)               # provenance: _unstick reopens loco first
            self.committed.discard(self.cell)
            self.failed_hops.pop((self.cell, dirn), None)
        if can_reverse:                                          # Tier 1 & 2: one-cell reverse to prev
            dx, dy = DIRS[d_prev]
            self.backout_target = pc
            self.backout_cardinal = math.atan2(-dy, -dx)         # face away from prev -> reverse into it
            self.backout_start = (x, y)
            self.backout_deadline = t + self.backout_timeout_s
            self.center_start = None
            self._escape_backout = True
            self.phase = 'backout'
            return (0.0, 0.0, False)
        return self._unstick(t)                                  # no reverse -> _unstick once (MF5)
```

- [ ] **Step 3b: Guard the `_backout` timeout counter + clear the flag.** In `_backout`, replace the `if arrived or t >= self.backout_deadline:` block's increment/clear so it reads:

```python
        if arrived or t >= self.backout_deadline:
            if arrived:
                self.cell = self.backout_target
            elif not self._escape_backout:                # an escape-reverse timeout must NOT charge
                self.backout_attempts[self.cell] = self.backout_attempts.get(self.cell, 0) + 1  # the dead-end budget
            self._escape_backout = False                  # clear on leaving backout (arrival or timeout)
            self.settle_until = t + self.settle_s
            self.center_start = None
            self.align_start = None; self.latched_cardinal = None
            self.phase = 'center'
            return (0.0, 0.0, False)
```
(Only the `else:` → `elif not self._escape_backout:` change plus the `self._escape_backout = False` line are new; the rest is the existing body unchanged.)

- [ ] **Step 4: Run to verify they pass + no regression**

Run: `cd "$PKG" && PYTHONPATH="$PKG" python3 -m pytest test/test_maze_motion.py test/test_maze_motion_sim.py -q`
Expected: PASS (7 new + all prior; the offline solve is unchanged — `_escape` is not yet wired into `step()`).

- [ ] **Step 5: Commit**

```bash
git -C "$REPO" add ros2_ws_tugbot_nav_20260614/src/tugbot_maze/tugbot_maze/maze_motion.py ros2_ws_tugbot_nav_20260614/src/tugbot_maze/test/test_maze_motion.py
git -C "$REPO" commit -m "feat: escalating no-progress escape (_escape) + backout guard

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 4: C2/C4 — wire the watchdog into `step()` + non-terminal `stuck` + latch

**Files:** Modify `tugbot_maze/maze_motion.py`; Test `test/test_maze_motion.py`.

- [ ] **Step 1: Write the failing tests** — append to `test/test_maze_motion.py`:

```python
def test_watchdog_fires_from_center_when_confined_and_no_progress():
    b = _two_exit_brain((5, 5), 'N', 'S')
    m = MazeMotion(b)
    m.cell = (5, 5); m.last_seen_cell = (5, 5); m.prev_cell = (5, 4)
    m.hop_dir = (0, 1); m.hop_target = (5, 6); m.visited = {(5, 4), (5, 5)}; m.explore_t = 100.0
    sim = MazeSim(load_segments(), cell_center((5, 5)), 0.0)
    m.step(sim.pose, _scan_at(sim), 100.0 + m.no_progress_s + 1.0)
    assert m.escape_count == 1                  # fired (no new ground + confined + past window)


def test_watchdog_fires_from_stuck():
    m = MazeMotion()
    m.cell = (3, 1); m.last_seen_cell = (3, 1); m.prev_cell = (2, 1)
    m.phase = 'stuck'; m.visited = {(2, 1), (3, 1)}; m.explore_t = 50.0
    sim = MazeSim(load_segments(), cell_center((3, 1)), 0.0)
    m.step(sim.pose, _scan_at(sim), 50.0 + m.no_progress_s + 1.0)
    assert m.escape_count == 1                  # the F2 fix: stuck is no longer a permanent freeze


def test_watchdog_silent_at_exit_cell():
    m = MazeMotion()
    m.cell = EXIT_CELL; m.last_seen_cell = EXIT_CELL; m.visited = {EXIT_CELL}; m.explore_t = 0.0
    sim = MazeSim(load_segments(), cell_center(EXIT_CELL), 0.0)
    v, w, done = m.step(sim.pose, _scan_at(sim), 1.0 + m.no_progress_s + 5.0)
    assert done is True and m.escape_count == 0  # never escapes at the exit (guard non-vacuous)


def test_watchdog_silent_when_not_confined():
    m = MazeMotion(); m.confine_k = 6
    m.cell = (5, 5); m.last_seen_cell = (5, 5); m.visited = {(5, 5)}; m.explore_t = 10.0
    t = 10.0 + m.no_progress_s + 1.0
    m.recent = [(t - 1, c) for c in [(1, 1), (2, 2), (3, 3), (4, 4), (5, 5), (6, 6), (7, 7)]]  # 7 > K
    sim = MazeSim(load_segments(), cell_center((5, 5)), 0.0)
    m.step(sim.pose, _scan_at(sim), t)
    assert m.escape_count == 0                  # large footprint -> not confined -> silent


def test_watchdog_suppressed_during_escape_reverse():
    b = _two_exit_brain((5, 5), 'N', 'S')
    m = MazeMotion(b)
    m.cell = (5, 5); m.last_seen_cell = (5, 5); m.prev_cell = (5, 4)
    m.hop_dir = (0, 1); m.hop_target = (5, 6); m.visited = {(5, 4), (5, 5)}; m.explore_t = 10.0
    m.phase = 'backout'; m._escape_backout = True            # an escape reverse is executing
    m.backout_start = (10.0, 10.0); m.backout_deadline = 1e12
    sim = MazeSim(load_segments(), cell_center((5, 5)), 0.0)
    m.step(sim.pose, _scan_at(sim), 10.0 + m.no_progress_s + 5.0)
    assert m.escape_count == 0                  # not re-fired mid-escape (still in backout)
```

- [ ] **Step 2: Run to verify they fail**

Run: `cd "$PKG" && PYTHONPATH="$PKG" python3 -m pytest test/test_maze_motion.py -k "watchdog" -v`
Expected: FAIL — `step()` does not yet call `_escape`; `escape_count` stays 0 in the fire cases, and `test_watchdog_fires_from_stuck` returns `(0,0,False)` (the old terminal `stuck`).

- [ ] **Step 3: Wire the C2 check into `step()`.** In `step()`, the `_track_cell(t)` call added in Task 2 sits right after `self.prev_yaw = yaw; self.prev_t = t`. Insert the watchdog check IMMEDIATELY AFTER that `_track_cell(t)` and BEFORE the `if self.phase == 'done' or self.cell == EXIT_CELL:` line (so the `cell != EXIT_CELL` guard is non-vacuous, NH13):

```python
        self._track_cell(t)
        if (self.phase != 'done' and self.cell != EXIT_CELL and not self._latched
                and not self._escape_backout              # don't re-fire mid-escape reverse
                and self.explore_t is not None
                and (t - self.explore_t) > self.no_progress_s
                and self._confined(t)):
            return self._escape(pose, t)
        if self.phase == 'done' or self.cell == EXIT_CELL:
            self.phase = 'done'
            return (0.0, 0.0, True)
```
This also makes `stuck` non-terminal (C4): the existing `if self.phase == 'stuck': return (0.0, 0.0, False)` stays, but the watchdog above runs first every tick, so a stuck robot escapes within `no_progress_s`. `_unstick` is unchanged (sets `stuck` on `self.reopened` exhaustion; the watchdog revives it). The `not self._latched` guard is for the NH14 dead-maze latch below.

- [ ] **Step 3b (NH14 latch — insurance for a genuinely dead maze).** So a fully-explored, disconnected, exit-unreachable maze stops permanently instead of oscillating fire→unstick→stuck forever. In `_escape`, at the very top (before `self.escape_count += 1`), add:

```python
        if self._maze_exhausted():
            self.phase = 'stuck'; self._latched = True
            return (0.0, 0.0, False)
```
(`self._latched` is already initialized in the Task 2 `__init__` block, and the C2 guard above already includes `not self._latched`.) Add the helper near `_escape`:

```python
    def _maze_exhausted(self):
        """Latched terminal: exit truly unreachable AND every reachable cell already visited AND no
        un-reopened cut edge remains (nothing left to reopen). Avoids a benign forever-oscillation in
        an unsolvable/fully-explored maze. Does not arise in the real solvable maze."""
        if self.brain.flood().get(self.cell, math.inf) != math.inf:
            return False                                  # exit still reachable -> not exhausted
        R = self._reachable_component(self.cell)
        if any(c not in self.visited for c in R):
            return False                                  # reachable unexplored ground remains
        cut = [(c, d) for c in R for d, (dx, dy) in DIRS.items()
               if self.brain.is_wall(c, d) and (c, d) not in self.reopened
               and in_grid((c[0] + dx, c[1] + dy)) and (c[0] + dx, c[1] + dy) not in R]
        return not cut                                    # nothing left to reopen -> latch
```

- [ ] **Step 4: Run to verify they pass + no regression**

Run: `cd "$PKG" && PYTHONPATH="$PKG" python3 -m pytest test/test_maze_motion.py test/test_maze_motion_sim.py -q`
Expected: PASS. The offline regression should STILL reach the exit with the watchdog silent (default `no_progress_s=90` is above the clean solve's confined dwell; Task 6 confirms/calibrates). If the regression now fires the watchdog (`reach`/`collision` regression), do NOT proceed — it means `no_progress_s`/`confine_k` need Task 6 calibration first; bump `no_progress_s` provisionally and note it.

- [ ] **Step 5: Commit**

```bash
git -C "$REPO" add ros2_ws_tugbot_nav_20260614/src/tugbot_maze/tugbot_maze/maze_motion.py ros2_ws_tugbot_nav_20260614/src/tugbot_maze/test/test_maze_motion.py
git -C "$REPO" commit -m "feat: top-of-step no-progress watchdog (covers stuck) + dead-maze latch

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 5: C5 — failed-hop deprioritizer (`_drive` + `_unstick`)

**Files:** Modify `tugbot_maze/maze_motion.py`; Test `test/test_maze_motion.py`.

`failed_hops` is the CROSS-occupation accumulator (persists across cell changes), distinct from the per-dwell `hop_attempts` (wholesale-cleared on arrival). It rides on the existing two giveup paths in `_drive` (the wedge path ~line 380 and the front-block/deadline path ~line 402), each of which already does the MF1-correct giveup (`brain.mark` + `_stamp_loco_wall` + `committed.discard`).

- [ ] **Step 1: Write the failing tests** — append to `test/test_maze_motion.py`:

```python
def test_failed_hops_persists_across_arrivals_and_walls_edge():
    b = _two_exit_brain((5, 5), 'N', 'S')
    m = MazeMotion(b); m.max_hop_attempts = 99      # isolate failed_hops from the per-dwell cap
    m.cell = (5, 5); m.hop_dir = (0, 1); m.hop_target = (5, 6)
    sim = MazeSim(load_segments(), cell_center((5, 5)), 0.0)
    for _ in range(m.failed_hop_limit):
        m.hop_attempts.clear()                      # simulate the wholesale clear that any arrival does
        m.phase = 'drive'; m.hop_start = (10.0, 10.0); m.hop_deadline = 0.0
        m._drive((10.4, 10.0, 0.0), _scan_at(sim), 1.0)   # moved 0.4 (<arrive), t>=deadline -> fail
    assert b.is_wall((5, 5), 'N')                   # failed_hops survived the clears -> edge given up (F3)


def test_single_failed_hop_does_not_wall():
    b = _two_exit_brain((5, 5), 'N', 'S')
    m = MazeMotion(b); m.max_hop_attempts = 99
    m.cell = (5, 5); m.hop_dir = (0, 1); m.hop_target = (5, 6)
    sim = MazeSim(load_segments(), cell_center((5, 5)), 0.0)
    m.phase = 'drive'; m.hop_start = (10.0, 10.0); m.hop_deadline = 0.0
    m._drive((10.4, 10.0, 0.0), _scan_at(sim), 1.0)
    assert not b.is_wall((5, 5), 'N') and m.failed_hops[((5, 5), 'N')] == 1


def test_successful_hop_clears_failed_hops_for_edge():
    b = _two_exit_brain((5, 5), 'N', 'S')
    m = MazeMotion(b); m.max_hop_attempts = 99
    m.cell = (5, 5); m.hop_dir = (0, 1); m.hop_target = (5, 6)
    m.failed_hops[((5, 5), 'N')] = 2
    m.phase = 'drive'; m.hop_start = (10.0, 10.0)
    sim = MazeSim(load_segments(), cell_center((5, 6)), 0.0)
    m._drive((10.0, 12.0, 0.0), _scan_at(sim), 0.1)         # moved ~2.0 -> arrival
    assert m.cell == (5, 6)
    assert ((5, 5), 'N') not in m.failed_hops              # cleared on success, pre-advance key (MF6)


def test_unstick_reopen_clears_failed_hops_and_resets_tier():
    from tugbot_maze.flood_fill_brain import FloodFillBrain
    b = FloodFillBrain()
    # box (5,5) in with WALLs on all four sides so _unstick must reopen a cut edge
    for d in ('N', 'S', 'E', 'W'):
        b.mark((5, 5), d, True)
    m = MazeMotion(b)
    m.cell = (5, 5); m.escape_tier = 2
    m.failed_hops[((5, 5), 'N')] = 3
    m._unstick(1.0)
    assert m.escape_tier == 0                              # map mutation (reopen) resets escalation
    # at least one incident edge was reopened and its failed_hops cleared
    assert all(k[0] != (5, 5) or k not in m.failed_hops for k in [((5, 5), 'N')]) or \
           ((5, 5), 'N') not in m.failed_hops
```

- [ ] **Step 2: Run to verify they fail**

Run: `cd "$PKG" && PYTHONPATH="$PKG" python3 -m pytest test/test_maze_motion.py -k "failed_hop or unstick_reopen_clears" -v`
Expected: FAIL — `failed_hops` never accumulates across `hop_attempts.clear()`, so the edge is not walled; `_unstick` does not clear `failed_hops` or reset `escape_tier`.

- [ ] **Step 3a: Increment `failed_hops` + extend the giveup in BOTH `_drive` failure branches.**
  In the **wedge** branch, find:
  ```python
            key = (self.cell, dirn)
            self.hop_attempts[key] = self.hop_attempts.get(key, 0) + 1
            self.center_start = None
            self.align_start = None; self.latched_cardinal = None
            self.settle_until = t + self.settle_s
            if self.hop_attempts[key] >= self.max_hop_attempts:      # give up this edge (last resort)
  ```
  Change to (add the `failed_hops` increment after the `hop_attempts` increment, and extend the condition):
  ```python
            key = (self.cell, dirn)
            self.hop_attempts[key] = self.hop_attempts.get(key, 0) + 1
            self.failed_hops[key] = self.failed_hops.get(key, 0) + 1   # cross-occupation accumulator
            self.center_start = None
            self.align_start = None; self.latched_cardinal = None
            self.settle_until = t + self.settle_s
            if (self.hop_attempts[key] >= self.max_hop_attempts
                    or self.failed_hops[key] >= self.failed_hop_limit):  # per-dwell OR cross-visit
  ```
  And in that branch's giveup body, after `self.hop_attempts.pop(key, None)`, add:
  ```python
                self.failed_hops.pop(key, None)
  ```
  In the **front-block/deadline** branch, find:
  ```python
            key = (self.cell, dirn)
            self.hop_attempts[key] = self.hop_attempts.get(key, 0) + 1
            if self.hop_attempts[key] >= self.max_hop_attempts:
                self.brain.mark(self.cell, dirn, is_wall=True)
                self._stamp_loco_wall(self.cell, dirn)               # re-openable by _unstick (both reps)
                self.committed.discard(self.cell)                    # un-commit; cell stays in `sensed`
                self.hop_attempts.pop(key, None)                     # so a poor re-entry won't re-sense
  ```
  Change to:
  ```python
            key = (self.cell, dirn)
            self.hop_attempts[key] = self.hop_attempts.get(key, 0) + 1
            self.failed_hops[key] = self.failed_hops.get(key, 0) + 1   # cross-occupation accumulator
            if (self.hop_attempts[key] >= self.max_hop_attempts
                    or self.failed_hops[key] >= self.failed_hop_limit):
                self.brain.mark(self.cell, dirn, is_wall=True)
                self._stamp_loco_wall(self.cell, dirn)               # re-openable by _unstick (both reps)
                self.committed.discard(self.cell)                    # un-commit; cell stays in `sensed`
                self.hop_attempts.pop(key, None)                     # so a poor re-entry won't re-sense
                self.failed_hops.pop(key, None)
  ```

- [ ] **Step 3b: Clear `failed_hops` for the traversed edge on arrival (pre-advance key, MF6).** In `_drive`'s arrival branch, the order is currently `mark_traversal` then `self.cell = self.hop_target` then `self.hop_attempts.clear()`. Insert the per-edge clear BEFORE advancing `self.cell`:
  ```python
          if moved >= CELL_SIZE_M - self.hop_arrive_slack_m:           # arrived
              self.brain.mark_traversal(self.cell, self.hop_target)    # count the COMPLETED traversal (was: at pick)
              _dirn = _dir_name(self.hop_dir)                          # clear failed_hops for THIS edge,
              self.failed_hops.pop((self.cell, _dirn), None)           #   keyed on the PRE-advance cell (MF6)
              self.failed_hops.pop((self.hop_target, OPP[_dirn]), None)
              self.cell = self.hop_target
              self.hop_attempts.clear()                                # fresh cell -> reset attempts
  ```

- [ ] **Step 3c: Clear `failed_hops` + reset `escape_tier` on `_unstick` reopen.** In `_unstick`, inside the reopen `for (c, d) in (incident or cand):` loop, after `self.locomotion_walls.discard((c, d)); self.locomotion_walls.discard((nb, OPP[d]))`, add:
  ```python
                    self.failed_hops.pop((c, d), None); self.failed_hops.pop((nb, OPP[d]), None)
  ```
  And just before `self.phase = 'center'` in the same `if cand:` block (after the loop), add:
  ```python
                self.escape_tier = 0                          # map mutation -> retry the cheap Tier-1 first
  ```

- [ ] **Step 4: Run to verify they pass + no regression**

Run: `cd "$PKG" && PYTHONPATH="$PKG" python3 -m pytest test/test_maze_motion.py test/test_maze_motion_sim.py -q`
Expected: PASS (4 new + all prior + the offline regression — the deprioritizer only adds a CROSS-visit giveup that the clean solve does not hit, since legit hops succeed).

- [ ] **Step 5: Commit**

```bash
git -C "$REPO" add ros2_ws_tugbot_nav_20260614/src/tugbot_maze/tugbot_maze/maze_motion.py ros2_ws_tugbot_nav_20260614/src/tugbot_maze/test/test_maze_motion.py
git -C "$REPO" commit -m "feat: cross-visit failed-hop deprioritizer (F3 fix)

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 6: Offline regression — `escape_count` plumbing, calibration, watchdog-silent gate, churn fixture

**Files:** Modify `test/test_maze_motion_sim.py`.

- [ ] **Step 1: Plumb `escape_count` through `_run`.** Add `m.escape_count` as the 6th return value in BOTH `return` statements of `_run`:
  - Line ~38: `return True, collided, max_desync, m.backout_count, backout_advances, m.escape_count`
  - Line ~46: `return (m.cell == EXIT_CELL), collided, max_desync, m.backout_count, backout_advances, m.escape_count`
  Update the two existing callers:
  - `test_reaches_exit_without_collision_or_desync` (~line 57): `reached, collided, max_desync, _, _, esc = _run(drift, latency)`
  - `test_backout_is_exercised_end_to_end` (~line 67): `reached, collided, _, backout_count, backout_advances, _ = _run(0.0, 0)`

- [ ] **Step 2: Calibrate across ALL 5 cases (measure, don't guess; MF12/SF12).** Temporarily instrument `_run` to record, per step, the **confined** no-new-cell dwell: track `last_new_t` (sim-time `len(m.visited)` last grew) and, when `(t - last_new_t)` is at a local max AND `m._confined(t)` is True, record it. Run all 5 (drift, latency) cases; note the worst-case confined dwell `D` and the footprint at that point. CONFIRM `no_progress_s (90.0) > D` with margin; if any case approaches 90, raise `no_progress_s` in `MazeMotion.__init__` above it and re-confirm. Confirm `confine_k (6)` exceeds the ping-pong footprint but is below the legit confined-stall footprint; adjust only if a footprint false-positive is seen. Remove the temporary instrumentation; record `D` and the chosen `no_progress_s`/`confine_k` in the commit message.

- [ ] **Step 3: Add the watchdog-silent assertion.** Update the parametrized regression (do NOT weaken existing asserts):

```python
@pytest.mark.parametrize("drift,latency", [(0.0, 0), (0.03, 0), (0.05, 0), (0.05, 2), (0.05, 3)])
def test_reaches_exit_without_collision_or_desync(drift, latency):
    reached, collided, max_desync, _, _, esc = _run(drift, latency)
    assert reached, f"did not reach the exit cell (drift={drift}, latency={latency})"
    assert not collided, f"robot body collided with a wall (drift={drift}, latency={latency})"
    assert max_desync <= 1, f"dcell desynced by {max_desync} (drift={drift}, latency={latency})"
    assert esc == 0, f"no-progress watchdog fired ({esc}x) on the CLEAN solve (drift={drift}, latency={latency})"
```
If `esc > 0` on any case, the threshold is below that case's legit confined dwell → raise `no_progress_s` (Step 2) and re-run; do NOT relax the `esc == 0` assertion.

(The MF8 churn-escape liveness test is the deterministic `test_churn_escape_escalates_and_reroutes` added in Task 3 — it proves the watchdog escalates and reroutes off a churn without depending on flaky sim dynamics. No churn fixture is added to the sim file; full physical churn rescue is validated in the Gazebo run.)

- [ ] **Step 4: Run to verify**

Run: `cd "$PKG" && PYTHONPATH="$PKG" python3 -m pytest test/test_maze_motion_sim.py -v`
Expected: PASS — all 5 reach-exit cases (now also asserting `esc == 0`) and `test_backout_is_exercised_end_to_end`.

- [ ] **Step 5: Commit**

```bash
git -C "$REPO" add ros2_ws_tugbot_nav_20260614/src/tugbot_maze/test/test_maze_motion_sim.py
git -C "$REPO" commit -m "test: watchdog-silent gate (calibrated across all 5 cases)

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Final validation (after all tasks)

1. **Full ROS-free suite green:**
   `cd "$PKG" && PYTHONPATH="$PKG" python3 -m pytest test/test_maze_motion.py test/test_maze_motion_sim.py test/test_hop_controller.py test/test_flood_fill_brain.py test/test_junction_log.py -q`
2. **Rebuild + Gazebo confirmation** (user-initiated; agent `pkill` hook-blocked — user clears stray sims via `! pkill -9 -f "gz sim|ruby.*gz|parameter_bridge|flood_fill_solver|ros2 launch"`):
   `colcon build --packages-select tugbot_maze`, then `tools/run_flood_fill_maze.sh 1800 false true odom_locked true`. Watch for: **no multi-minute dwell** at one `dcell` (escapes fire + relocate within ~`no_progress_s`); **no terminal `stuck`/725 s freeze**; deeper/faster progress than the regressed 18.29 m and ideally past the 1.65 m bypass-only best; objective collision rate (`MazeSim.collides` on the DIAG trajectory) not worse than the 24% combined-run baseline.
3. Do **not** merge to `main` or push — banking is a separate, user-approved step after the Gazebo run.
