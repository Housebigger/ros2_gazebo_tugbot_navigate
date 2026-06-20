# Decisive Dead-End Back-Out + Passive Junction Log — Design

**Date:** 2026-06-20
**Workspace:** `ros2_ws_tugbot_nav_20260614`
**Branch:** `deadend-backout-junction-log` (off `main`; local only, **never pushed**)
**Status:** Approved (brainstorm + scope); hardened by an adversarial multi-lens review (11 confirmed
findings folded in — see *Revisions* at the end). Ready for the user review gate, then plan.

## Problem

In the last Gazebo run the flood-fill solver correctly recognized corridor centerlines,
junctions, and dead-ends, but two behaviors were poor:

1. **Hesitant, slow dead-end exit + a collision source.** There is no dead-end concept at all.
   Leaving a dead-end means routing back to the parent, which the FSM executes as *rotate 180° in
   place inside the cramped dead-end cell* → the robot physically wedges → only after
   `wedge_detect_s = 3.0 s` of zero progress (`maze_motion.py:339`) does `_recover` nudge it back
   at `−recover_v = −0.15 m/s` for `recover_s = 1.8 s` (just 0.27 m, `maze_motion.py:402-403`) →
   then re-center. The 180°-in-a-tight-cell is a prime body-on-wall clip, and the 3 s stall +
   half-speed nudge is the hesitation.
2. **No persisted exploration memory.** "Junction" is not a first-class concept — it is implicit
   in `next_cell` routing (`flood_fill_brain.py`). Nothing records where the junctions are, so the
   run leaves no analyzable record of the maze topology the robot discovered.

## Goal

Two independent, low-risk components:

1. **Decisive dead-end back-out** — detect a dead-end right after the cell is sensed in `_center →
   _route`, **gated so the single open exit is the edge the robot just drove in through**, and
   back **straight out one cell** to that parent, firmly and immediately (no 180° turn-in-place, no
   3 s wedge wait), then resume the proven FSM. Removes both the hesitation and the dead-end clip.
2. **Passive junction record** — detect each junction (≥ 3 open exits) and log its coordinate,
   open exits, discovery order, and visit count to a per-run `junctions.json` artifact (plus a
   stdout line). **Routing is untouched** — this is a memory asset / analyzable dataset, not a
   change to the proven flood-fill + Trémaux exploration.

## Scope (decided)

**IN:** decisive dead-end back-out (Component 1) and passive junction logging (Component 2).

**OUT (deferred, separate effort):** the broader doorway/turn-transition precision controller for
collisions that occur outside dead-ends. Component 1 eliminates the *dead-end* collision class; the
general transition-precision rebuild is explicitly not in scope.

**Files touched:**
- `tugbot_maze/flood_fill_brain.py` — add **one pure read-only helper** `open_exits(brain, cell)`
  (no behavior change to routing). Reused by Component 1 and Component 2 (DRY + one test site).
- `tugbot_maze/hop_controller.py` — add one pure function `backout_command`.
- `tugbot_maze/maze_motion.py` — add a `backout` FSM phase: came-from-gated dead-end detection in
  `_route`, a new `_backout` method with a **bounded, escalating** timeout, new `__init__`
  tunables/state, and a `backout_count` observable. `_recover` stays unchanged (coexists).
- `tugbot_maze/junction_log.py` — **new** ROS-free module: a testable `JunctionLog` class plus a
  pure `update_junctions(...)` helper that encapsulates the node's per-tick bookkeeping.
- `tugbot_maze/flood_fill_solver.py` (node) — own a `JunctionLog`, call `update_junctions` each
  tick, flush JSON (guarded) on `EXIT_REACHED` and on the existing 5 s diag timer.
- `src/tugbot_bringup/launch/tugbot_maze_explore.launch.py` — add a `junction_log_dir` launch arg.
- `tools/run_flood_fill_maze.sh` — pass `junction_log_dir:=$ART`; add `JUNCTION` to the tail grep.

**Do NOT touch:** `cell_walls.py`, `wall_localize.py`; within `flood_fill_brain.py` the routing
logic (only the additive `open_exits` helper); and within `maze_motion.py`: `_center`
sensing/commit/corroboration, the brain routing in `_route` (only a dead-end branch is *added*
after the unstick guard), `_unstick`, dcell re-anchoring, the wedge detector, `_recover`, the
corridor follower, and the arrival test.

---

## Component 0 — pure helper `open_exits` (`flood_fill_brain.py`)

Both components need "the in-grid, not-walled directions of a cell". Define it once, pure and
testable, alongside `DIRS`/`in_grid`/`is_wall`:

```python
def open_exits(brain, cell):
    """In-grid, not-walled directions out of `cell` (its open exits). Pure, read-only. The in_grid
    filter is load-bearing: a border cell's out-of-grid side must NOT count as an exit."""
    return [d for d, (dx, dy) in DIRS.items()
            if in_grid((cell[0] + dx, cell[1] + dy)) and not brain.is_wall(cell, d)]
```

---

## Component 1 — Decisive dead-end back-out

### Geometry + the came-from gate (why a straight reverse is correct, and never misfires)

A back-out is only valid when the robot **drove into the cell through its single open exit** — then
that exit is directly behind it and a straight reverse exits cleanly. Detection must therefore
check the *came-from* edge, **not just** that one exit is open. (A purely topological
`len(open_exits)==1` test misfires at the entrance `(1,0)` and the bottom-row stubs, whose only
*in-grid* open exit is the way **forward** because the other sides are out-of-grid — reproduced in
review as an indefinite startup livelock.)

The robot's last completed hop direction is `self.hop_dir` (still set when `_route` runs, before
the next hop is chosen). The edge it came in through is `came_from = OPP[_dir_name(self.hop_dir)]`.
A genuine dead-end: `open_exits(cell) == [came_from]`. The robot enters facing `−came_from`
(cardinal of the entry hop); reversing while holding that cardinal moves it back along `came_from`
to the parent — **no rotation**, no body-sweep into the three surrounding walls.

At the very first route `self.hop_dir is None` (no came-from) → never a back-out, so the entrance
routes forward normally. (`self.cell != ENTRANCE_CELL` is also asserted, belt-and-suspenders.)

### New pure function (`hop_controller.py`)

Add after `profiled_turn_command`:

```python
def backout_command(yaw: float, hold_cardinal: float, *, backout_v: float = 0.30,
                    w_max: float = 0.5, kp_ang: float = 1.5) -> Tuple[float, float]:
    """Firm straight reverse out of a dead-end while holding `hold_cardinal`. Returns (v, w) with
    v = -backout_v (decisive, ~cruise speed -- NOT the 0.15 un-wedge crawl). Proportional steering
    on the heading error keeps the robot backing straight along the cardinal; the steering sign is
    correct for reverse travel because w rotates the body toward hold_cardinal regardless of the
    sign of v. No cross-track term -- the reverse is a single ~2 m cell retracing a path just
    driven centered, so heading-hold suffices."""
    err = _norm(hold_cardinal - yaw)
    w = max(-w_max, min(w_max, kp_ang * err))
    return (-backout_v, w)
```

### FSM integration (`maze_motion.py`)

**New `__init__` tunables (defaults):** `backout_v = 0.30` (m/s, firm reverse),
`backout_timeout_s = 12.0` (s), `max_backout_attempts = 2` (timeouts before escalating).

**New `__init__` state:** `self.backout_target = None`, `self.backout_cardinal = 0.0`,
`self.backout_start = None`, `self.backout_deadline = 0.0`, `self.backout_attempts = {}`
(cell → timeout count, bounds the loop), `self.backout_count = 0` (observable for tests/regression).

**Import:** add `open_exits` to the `flood_fill_brain` import (`OPP` and `_dir_name` already
available).

**Dispatch (`step`, `maze_motion.py:108-126`):** add a branch
```python
if self.phase == 'backout':
    return self._backout(pose, t)
```

**Dead-end detection (`_route`, `maze_motion.py:223-243`):** after the existing unstick guard
(`if nxt is None or not reachable: return self._unstick(t)`) and **before** the normal hop setup,
insert:
```python
# Dead-end -> decisive straight back-out, but ONLY when the lone open exit is the edge we drove
# in through (came_from). This excludes the entrance / out-of-grid-entered cells, whose single
# in-grid open exit is the FORWARD route. Bounded by backout_attempts so a pin can't livelock.
came_from = OPP[_dir_name(self.hop_dir)] if self.hop_dir is not None else None
open_dirs = open_exits(self.brain, self.cell)
if (self.cell != ENTRANCE_CELL and came_from is not None and len(open_dirs) == 1
        and open_dirs[0] == came_from
        and self.backout_attempts.get(self.cell, 0) < self.max_backout_attempts):
    dx, dy = DIRS[came_from]
    self.backout_target = (self.cell[0] + dx, self.cell[1] + dy)
    self.backout_cardinal = math.atan2(-dy, -dx)   # face INTO the dead-end; reverse toward parent
    self.backout_start = (x, y)
    self.backout_deadline = t + self.backout_timeout_s
    self.backout_count += 1
    self.center_start = None
    self.phase = 'backout'
    return (0.0, 0.0, False)
```
When this branch is skipped (entrance, not a real dead-end, or attempts exhausted) `_route`
continues to the **unchanged** normal hop setup — i.e. the proven turn+drive toward `nxt` (which
for a real dead-end is the parent), so the existing wedge detector / hop-attempt escalation still
applies as the fallback.

**New `_backout` method (bounded + escalating):**
```python
def _backout(self, pose, t):
    """Decisive straight reverse out of a dead-end to the parent cell (one cell). Holds the
    cardinal and backs at backout_v -- no turn-in-place, no wedge_detect_s wait. On arrival
    (~one cell) advance self.cell to the parent and resume the FSM. On timeout WITHOUT arrival
    (a physical pin while reversing) count an attempt and resume center; after
    max_backout_attempts the _route gate stops re-arming back-out and falls through to the normal
    turn+drive toward the parent, where the wedge detector + hop-attempt cap escalate (mark wall
    -> _unstick) exactly as before. So the timeout path always makes progress toward recovery --
    it can never deterministically re-enter back-out on the same pinned cell."""
    x, y, yaw = pose
    moved = math.hypot(x - self.backout_start[0], y - self.backout_start[1])
    arrived = moved >= CELL_SIZE_M - self.hop_arrive_slack_m
    if arrived or t >= self.backout_deadline:
        if arrived:
            self.cell = self.backout_target
        else:
            self.backout_attempts[self.cell] = self.backout_attempts.get(self.cell, 0) + 1
        self.settle_until = t + self.settle_s
        self.center_start = None
        self.align_start = None; self.latched_cardinal = None
        self.phase = 'center'
        return (0.0, 0.0, False)
    v, w = backout_command(yaw, self.backout_cardinal, backout_v=self.backout_v,
                           w_max=self.w_max, kp_ang=self.kp_turn)
    return (v, w, False)
```

### Why this fixes the problem and is safe

- **Decisive:** fires the instant a true dead-end is detected (after sensing in `_center →
  _route`), reverses at 0.30 m/s — no 3 s wedge wait, no 0.15 m/s crawl.
- **No collision, no misfire:** the came-from gate guarantees the lone open exit is directly behind
  the robot, so it backs straight out along that axis; it never attempts a 180° rotation in the
  3-walled cell, and it never fires at the entrance/forward-stub cells. The onward 180° (in the
  parent) happens later where there is open front+back.
- **"Back to the junction":** the 1-cell reverse returns the robot to the parent; the proven,
  drift-immune **forward** corridor follower then carries it onward. (Flood distance + Trémaux never
  route back into the now-known dead-end; skipping `mark_traversal` on the back-out hop is benign —
  the `{P,D}` edge was already counted on the forward hop and `D` always has higher flood distance.)
- **Bounded + escalating:** on a timeout the cell stays the dead-end and an attempt is counted;
  after `max_backout_attempts` the gate stops re-arming and `_route` falls through to the normal
  turn+drive, whose wedge detector + hop-attempt cap escalate (mark wall → `_unstick` → eventually
  `stuck`). This restores the escalation the old path had — no infinite `backout↔center` loop.
- **Coexists with `_recover`:** the wedge `_recover` (reactive physical-pin net, triggered only
  from `drive`) is unchanged; `backout` is anticipatory and topology-triggered. They do not overlap.
- **Trust:** detection reads sensed (not necessarily committed) walls — a real dead-end is entered
  once and so is never committed (corroboration needs a 2nd visit, which routing prevents), so a
  committed gate would *break* the feature. The came-from edge is physically verified (the robot
  drove through it), so a false dead-end would require all three other walls to be simultaneously
  mis-sensed (rare with projection-median) and is self-correcting — backing to the parent matches
  what current routing already does for that state; it only changes the *mechanism* (straight
  reverse vs 180° turn), introducing no new false-wall failure mode.

---

## Component 2 — Passive junction record

Routing is **untouched**. This is a structured artifact + visible memory asset.

### Detection (gated on `sensed`, not `committed`)

A **sensed** cell with `len(open_exits(brain, cell)) >= 3` is a junction. A single
`sense_cell_walls` read determines all four cardinal edges, so the open-exit set is complete at
first sense — gating on `sensed` (rather than `committed`, which needs a 2nd corroborating visit)
captures **single-pass straight-through junctions**, which are the common case and would otherwise
be silently dropped. `observe()` refreshes the exits if a later corroborating read changes them, so
fidelity still upgrades on revisit.

### New module `tugbot_maze/junction_log.py` (ROS-free)

```python
import json
import os
from typing import Dict, List, Optional, Tuple

from tugbot_maze.flood_fill_brain import open_exits

Cell = Tuple[int, int]


class JunctionLog:
    """Per-run record of discovered junctions (cells with >= 3 open exits). Pure data + JSON
    persistence; no ROS. observe() records a junction on first sight (stamping discovery order +
    first-seen time) and refreshes its exits on later sights; visit() counts cell entries."""

    def __init__(self):
        self._j: Dict[Cell, dict] = {}
        self._order = 0
        self._visits: Dict[Cell, int] = {}

    @property
    def count(self) -> int:
        return len(self._j)

    def visit(self, cell: Cell) -> None:
        self._visits[cell] = self._visits.get(cell, 0) + 1
        if cell in self._j:
            self._j[cell]['visits'] = self._visits[cell]

    def observe(self, cell: Cell, exits: List[str], t: float,
                cell_size_m: float = 2.0) -> Tuple[dict, bool]:
        """Record on first sight (assign discovery_index + first_seen_s) and refresh exits on
        later sights, keeping discovery order / first-seen / visit count. Returns (entry, is_new)."""
        is_new = cell not in self._j
        if is_new:
            self._order += 1
            self._j[cell] = {
                'cell': [cell[0], cell[1]],
                'center_xy': [round(cell_size_m * cell[0], 3), round(cell_size_m * cell[1], 3)],
                'first_seen_s': round(t, 2),
                'discovery_index': self._order,
                'visits': self._visits.get(cell, 0),
            }
        e = self._j[cell]
        e['exits'] = list(exits)
        e['exit_count'] = len(exits)
        return e, is_new

    def to_dict(self) -> dict:
        return {
            'junction_count': len(self._j),
            'junctions': sorted(self._j.values(), key=lambda r: r['discovery_index']),
        }

    def flush(self, path: str) -> None:
        d = os.path.dirname(path)
        if d:
            os.makedirs(d, exist_ok=True)
        tmp = path + '.tmp'
        with open(tmp, 'w') as f:
            json.dump(self.to_dict(), f, indent=2)
        os.replace(tmp, path)   # atomic


def update_junctions(jlog: JunctionLog, brain, cell: Cell, prev_cell: Optional[Cell],
                     sensed, t: float) -> Tuple[Optional[Cell], Optional[dict]]:
    """Per-tick junction bookkeeping (ROS-free, fully unit-testable). Counts a visit on cell
    change; if `cell` is sensed with >= 3 in-grid open exits, observe() it. Returns
    (new_prev_cell, newly_recorded_entry_or_None) -- the node logs a JUNCTION line iff non-None."""
    if cell != prev_cell:
        jlog.visit(cell)
        prev_cell = cell
    new_entry = None
    if cell in sensed:
        exits = open_exits(brain, cell)
        if len(exits) >= 3:
            entry, is_new = jlog.observe(cell, exits, t)
            if is_new:
                new_entry = entry
    return prev_cell, new_entry
```

### Node wiring (`flood_fill_solver.py`)

- **Imports:** add `from tugbot_maze.junction_log import JunctionLog, update_junctions`;
  add `import os`. (No `DIRS`/`in_grid` import needed — `update_junctions` owns the predicate.)
- **`__init__`:**
  ```python
  self.junction_log_dir = str(self.declare_parameter('junction_log_dir', '').value)
  self.junction_log_path = os.path.join(self.junction_log_dir or 'log', 'junctions.json')
  self.junctions = JunctionLog()
  self._prev_motion_cell = None
  ```
- **A guarded flush helper** (so artifact IO can never crash the long-running solver — `flush` runs
  inside the 5 s rclpy timer, whose unhandled exceptions would tear down `spin()`):
  ```python
  def _flush_junctions(self):
      try:
          self.junctions.flush(self.junction_log_path)
      except Exception as e:                       # never let artifact IO kill the solver
          self.get_logger().warning('junction log flush failed: %r' % e)
  ```
- **`_control_tick`, `driving` branch (after `self.motion.step(...)`):**
  ```python
  self._prev_motion_cell, j = update_junctions(
      self.junctions, self.brain, self.motion.cell, self._prev_motion_cell, self.motion.sensed, t)
  if j is not None:
      self.get_logger().info('JUNCTION cell=%s exits=%s order=%d visits=%d'
                             % (tuple(j['cell']), j['exits'], j['discovery_index'], j['visits']))
  ```
- **`EXIT_REACHED` branch:** after publishing the goal event, `self._flush_junctions()`.
- **`_diag_tick`:** at the end, `self._flush_junctions()` (periodic 5 s flush so a killed run still
  leaves data).

### Artifact location

The node's runtime CWD is the workspace root (`tools/run_flood_fill_maze.sh` does `cd "$WS"`). The
run script already creates `ART="log/flood_fill_run_${STAMP}"`. Passing `junction_log_dir:=$ART`
lands `junctions.json` beside `launch.log`. If the param is empty (node run directly),
`junction_log_path` falls back to `log/junctions.json` under CWD (`flush` creates the dir).

### Launch + run-script changes

- **`tugbot_maze_explore.launch.py`:** add `DeclareLaunchArgument('junction_log_dir', default_value='')`
  and pass `{'junction_log_dir': LaunchConfiguration('junction_log_dir')}` in the `flood_fill_solver`
  Node `parameters` list (following the existing `pose_source` param pattern).
- **`run_flood_fill_maze.sh`:** add `junction_log_dir:=$ART` to the `ros2 launch ...` args; add
  `JUNCTION` to the `flood_fill_tail.txt` grep alternation.

---

## Testing & validation

**Unit — `test/test_flood_fill_brain.py` (or test_junction_log.py): `open_exits`:**
- ≥3-open interior cell → all open in-grid dirs; a border cell → out-of-grid side excluded; a fully
  walled-but-one cell → single exit.

**Unit — `test/test_hop_controller.py` (add `backout_command`):**
- `backout_command(0.0, 0.0)` → `v == -0.30`, `|w|` ≈ 0 (aligned → straight reverse).
- heading error → `w` sign reduces the error toward `hold_cardinal` (`yaw=0.2, hold=0.0` → `w < 0`).
- custom `backout_v` honoured (returned `v == -backout_v`).

**Unit — `test/test_maze_motion.py` (add):**
- `test_deadend_triggers_backout`: `D=(5,5)` walled N/E/W, open S to `P=(5,4)`; set `motion.cell=D`
  and `motion.hop_dir=(0,1)` (entered going N, so `came_from='S'`); call `_route` → `phase ==
  'backout'`, `backout_target == (5,4)`, `backout_cardinal == atan2(1, 0)` (≈ +π/2, faces N),
  `backout_count == 1`.
- `test_backout_reverses_then_advances`: from the above, `_backout` with a pose still inside `D` →
  `v < 0` (≈ −0.30) and small `|w|`; then a pose moved one cell south (into `P`) → `motion.cell ==
  (5,4)`, `phase == 'center'`.
- `test_entrance_does_not_backout`: `motion.cell=ENTRANCE_CELL=(1,0)`, `hop_dir=None`, E walled / N
  open (S,W out of grid) → `_route` does **not** enter backout (`phase == 'turn'`, `hop_target is
  not None`); guards the existing `test_center_senses_then_turns_toward_a_chosen_neighbor`.
- `test_single_open_exit_not_came_from_does_not_backout`: a cell whose lone open exit is **not** the
  `came_from` edge → no backout (normal route).
- `test_backout_timeout_escalates`: enter backout, then `_backout` with poses that never reach the
  arrival threshold while advancing `t` past `backout_deadline`, repeated `max_backout_attempts`
  times → after the cap, `_route` no longer enters backout for that cell (falls to `phase ==
  'turn'`); asserts no infinite re-arm.

**Unit — `test/test_junction_log.py` (new): `JunctionLog` + `update_junctions`:**
- `observe` records on first sight, returns `is_new=True`; second call same cell → `is_new=False`,
  `count == 1`, and **refreshes exits** if changed.
- `discovery_index` increments across distinct cells; `to_dict()['junctions']` sorted by it.
- `visit` increments `visits`; a `visit` before `observe` is reflected; repeated visits update an
  already-recorded entry's `visits`.
- `flush(tmp_path/'junctions.json')` writes parseable JSON of the documented shape and creates a
  missing parent dir.
- `update_junctions` (the node glue, ROS-free): cell-change → `visit`; a sensed cell with ≥3 open
  exits → recorded once (returns the entry only when new); a non-sensed or <3-exit cell → nothing;
  `_prev_cell` threading counts entries correctly (no double-count while stationary).

**Offline regression — `test/test_maze_motion_sim.py` (must stay green AND prove back-out fired):**
- The end-to-end `maze_sim` solve (inertia + collision + odom drift + cmd latency) must still reach
  `EXIT_CELL` collision-free. **Add assertions** that Component 1 actually ran end-to-end (the
  project has a history of outcome-only green tests masking failures): assert `m.backout_count > 0`
  across the parametrized runs, and that at least one back-out advanced `m.cell` to the parent (e.g.
  capture phase/cell history in `_run`, assert a `backout → center` transition with a cell change).
  This makes a disabled/broken back-out fail the build even though the fallback path could still
  reach the exit. Component 2 is node-only and not exercised here (its glue is covered by the
  `update_junctions` unit test).

**Gazebo (one confirmation run):**
`tools/run_flood_fill_maze.sh 1800 false true odom_locked true` (GUI, 1800 s, odom_locked,
sense_debug). Clean stray sims first via the `!`-prefix pkill. Watch for: decisive reversals at
dead-ends (no 3 s stalls, no body clips), and a populated `log/flood_fill_run_<stamp>/junctions.json`
plus `JUNCTION` lines in `launch.log`.

**Success criteria:**
- All new unit tests pass; offline regression stays green **and** asserts back-out fired.
- In Gazebo: dead-end exits are firm and immediate (no stall/clip), and `junctions.json` contains
  the discovered junctions with coordinates, exits, discovery order, and visit counts.

## Constraints

- Local only — **never push** to origin.
- Commit messages end with `Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>`.
- Agent `pkill`/`kill` is hook-blocked; stray Gazebo sims are cleared by the user via the `!`
  prefix: `! pkill -9 -f "gz sim|ruby.*gz|parameter_bridge|flood_fill_solver|ros2 launch"`.
- Foreground `sleep` is blocked; use background runs.
- Shell cwd is not stable between Bash calls — use absolute paths / `git -C`.
- Avoid backticks inside `git commit -m` strings (bash command-substitution can drop a word).

## Revisions from the adversarial review (2026-06-20)

The first draft was reviewed by 4 lenses with per-finding adversarial verification; 11 confirmed
findings are folded in:

1. **MUST-FIX — entrance/forward-stub misfire → startup livelock.** Topological `len(open_exits)==1`
   fired at `(1,0)` (lone in-grid exit is forward), reverse-arced into a wall, never arrived, and
   re-armed forever → broke the offline regression. **Fixed** by the *came-from gate*
   (`open_exits(cell) == [OPP[_dir_name(hop_dir)]]`, `hop_dir is not None`, `cell != ENTRANCE_CELL`),
   making the reverse geometry provably valid and needing no yaw plumbing.
2. **MUST-FIX — back-out timeout livelock.** The old timeout path re-armed back-out with no
   escalation (the claimed "wedge recover/unstick takes over" was unreachable from `center`).
   **Fixed** with `backout_attempts`/`max_backout_attempts`: after the cap, `_route` falls through
   to the proven turn+drive toward the parent, whose wedge/hop-attempt machinery escalates.
3. **MUST-FIX (test) — regression couldn't prove back-out ran.** **Fixed** by adding `backout_count`
   and asserting `> 0` plus a `backout→center` cell advance in `test_maze_motion_sim`.
4. **Should — prose/code "committed" inconsistency for detection.** **Fixed**: detection is sensed-
   gated (not committed); rationale documented (a real dead-end never commits).
5. **Should — unguarded `flush()` crash path** in the 5 s rclpy timer. **Fixed** with `_flush_junctions`
   try/except.
6. **Should — committed gate under-recorded single-pass junctions.** **Fixed**: Component 2 gates on
   `sensed`; `observe()` refreshes exits on revisit.
7. **Should — node glue untested + privates access.** **Fixed**: extracted `open_exits` and
   `update_junctions` pure helpers (unit-tested); the JUNCTION log line uses the returned entry, not
   `JunctionLog` privates.
8. Plus the matching new unit tests for the entrance/forward-open geometry and the timeout branch.
