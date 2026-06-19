# Decisive Dead-End Back-Out + Passive Junction Log — Design

**Date:** 2026-06-20
**Workspace:** `ros2_ws_tugbot_nav_20260614`
**Branch:** `deadend-backout-junction-log` (off `main`; local only, **never pushed**)
**Status:** Approved (brainstorm + scope), ready for adversarial review then implementation plan.

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

1. **Decisive dead-end back-out** — detect a dead-end the moment its walls are committed and back
   **straight out one cell** to the parent, firmly and immediately (no 180° turn-in-place, no 3 s
   wedge wait), then resume the proven FSM. Removes both the hesitation and the dead-end wall-clip.
2. **Passive junction record** — detect each junction (≥ 3 open exits) and log its coordinate,
   open exits, discovery order, and visit count to a per-run `junctions.json` artifact (plus a
   stdout line). **Routing is untouched** — this is a memory asset / analyzable dataset, not a
   change to the proven flood-fill + Trémaux exploration.

## Scope (decided)

**IN:** decisive dead-end back-out (Component 1) and passive junction logging (Component 2).

**OUT (deferred, separate effort):** the broader doorway/turn-transition precision controller for
collisions that occur outside dead-ends (e.g. clipping on normal turns). Component 1 eliminates the
*dead-end* collision class; the general transition-precision rebuild is explicitly not in scope.

**Files touched:**
- `tugbot_maze/hop_controller.py` — add one pure function `backout_command`.
- `tugbot_maze/maze_motion.py` — add a `backout` FSM phase: dead-end detection in `_route`, a new
  `_backout` method, new `__init__` tunables/state. `_recover` stays unchanged (coexists).
- `tugbot_maze/junction_log.py` — **new** small, ROS-free, testable `JunctionLog` class.
- `tugbot_maze/flood_fill_solver.py` (node) — own a `JunctionLog`, detect+record junctions each
  tick from `motion.brain`/`motion.committed`/`motion.cell`, flush JSON on `EXIT_REACHED` and on
  the existing 5 s diag timer. **No `maze_motion` change is needed for Component 2.**
- `src/tugbot_bringup/launch/tugbot_maze_explore.launch.py` — add a `junction_log_dir` launch arg
  passed to the node.
- `tools/run_flood_fill_maze.sh` — pass `junction_log_dir:=$ART`; add `JUNCTION` to the tail grep.

**Do NOT touch:** `flood_fill_brain.py`, `cell_walls.py`, `wall_localize.py`, and within
`maze_motion.py`: `_center` sensing/commit/corroboration, the brain routing in `_route` (only a
dead-end branch is *added*), `_unstick`, dcell re-anchoring, the wedge detector, `_recover`, the
corridor follower, and the arrival test.

---

## Component 1 — Decisive dead-end back-out

### Geometry (why a straight reverse is correct, no turn)

The robot enters dead-end cell `D` from parent `P`, travelling `P→D`, so after `_center` it faces
the cardinal `d_enter` (into the dead-end). A dead-end has exactly one open exit, which is
necessarily `D→P = −d_enter` (behind the robot). Therefore **reversing straight (negative linear
velocity) while holding the current cardinal moves the robot back out along the only open
direction** — no rotation, no risk of sweeping the body into the three surrounding walls.

### New pure function (`hop_controller.py`)

Add after `profiled_turn_command`:

```python
def backout_command(yaw: float, hold_cardinal: float, *, backout_v: float = 0.30,
                    w_max: float = 0.5, kp_ang: float = 1.5) -> Tuple[float, float]:
    """Firm straight reverse out of a dead-end while holding `hold_cardinal`. Returns (v, w) with
    v = -backout_v (decisive, ~cruise speed -- NOT the 0.15 un-wedge crawl). Proportional steering
    on the heading error keeps the robot backing straight along the cardinal; steering sign is
    correct for reverse travel because w rotates the body toward hold_cardinal regardless of the
    sign of v. No cross-track term -- the reverse is a single ~2 m cell retracing a path just
    driven centered, so heading-hold suffices."""
    err = _norm(hold_cardinal - yaw)
    w = max(-w_max, min(w_max, kp_ang * err))
    return (-backout_v, w)
```

### FSM integration (`maze_motion.py`)

**New `__init__` tunables (defaults):** `backout_v = 0.30` (m/s, firm reverse),
`backout_timeout_s = 12.0` (s, safety valve).

**New `__init__` state:** `self.backout_target = None`, `self.backout_cardinal = 0.0`,
`self.backout_start = None`, `self.backout_deadline = 0.0`.

**Dispatch (`step`, `maze_motion.py:108-126`):** add a branch
```python
if self.phase == 'backout':
    return self._backout(pose, t)
```

**Dead-end detection (`_route`, `maze_motion.py:223-243`):** after the existing unstick guard
(`if nxt is None or not reachable: return self._unstick(t)`) and **before** the normal hop setup,
insert:
```python
# Dead-end: exactly one in-grid open exit -> decisive straight back-out to the parent,
# instead of routing back (which would rotate 180 deg in this cramped cell and wedge).
open_dirs = [d for d, (dx, dy) in DIRS.items()
             if in_grid((self.cell[0] + dx, self.cell[1] + dy))
             and not self.brain.is_wall(self.cell, d)]
if len(open_dirs) == 1:
    d_open = open_dirs[0]
    dx, dy = DIRS[d_open]
    self.backout_target = (self.cell[0] + dx, self.cell[1] + dy)
    self.backout_cardinal = math.atan2(-dy, -dx)   # face INTO the dead-end; reverse toward parent
    self.backout_start = (x, y)
    self.backout_deadline = t + self.backout_timeout_s
    self.center_start = None
    self.phase = 'backout'
    return (0.0, 0.0, False)
```
(`open_dirs` counts only **in-grid** neighbours, so a border cell's out-of-grid direction is never
miscounted as an exit. When `len(open_dirs) == 1`, `nxt` is necessarily that parent, so this branch
never starves real routing.)

**New `_backout` method:**
```python
def _backout(self, pose, t):
    """Decisive straight reverse out of a dead-end to the parent cell (one cell). Holds the
    cardinal and backs at backout_v -- no turn-in-place, no wedge_detect_s wait. On arrival
    (~one cell) advance self.cell to the parent and resume the FSM (which then turns/drives in
    the roomy parent cell). backout_deadline is a safety valve: on timeout, hand back to center
    and let the normal machinery (wedge recover / unstick) take over."""
    x, y, yaw = pose
    moved = math.hypot(x - self.backout_start[0], y - self.backout_start[1])
    arrived = moved >= CELL_SIZE_M - self.hop_arrive_slack_m
    if arrived or t >= self.backout_deadline:
        if arrived:
            self.cell = self.backout_target
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

- **Decisive:** fires the instant the dead-end is detected (after sensing/commit in `_center →
  _route`), reverses at 0.30 m/s — no 3 s wedge wait, no 0.15 m/s crawl.
- **No collision:** never attempts a 180° rotation inside the 3-walled cell; it backs straight out
  along the only open axis. The 180° (toward the onward route) happens later in the parent cell,
  which has open front+back — the same room a normal corridor turn uses.
- **"Back to the junction":** the 1-cell reverse returns the robot to the parent; the proven,
  drift-immune **forward** corridor follower then carries it onward toward the junction. (Flood
  distance + Trémaux never route back into the now-known dead-end.)
- **Coexists with `_recover`:** the wedge `_recover` (reactive, physical-pin net) is unchanged and
  triggers only from the `drive` phase; `backout` is anticipatory and topology-triggered. They do
  not overlap.
- **Termination:** `backout` ends on a 1-cell displacement or `backout_deadline`. A successful
  back-out sets `self.cell = parent` (not a dead-end), so it cannot immediately re-trigger.

---

## Component 2 — Passive junction record

### Detection

A committed cell with **≥ 3 in-grid open exits** is a junction. The **node** computes this from the
brain it already holds — no `maze_motion` change:
```python
open_exits = [d for d, (dx, dy) in DIRS.items()
              if in_grid((cell[0] + dx, cell[1] + dy)) and not brain.is_wall(cell, d)]
# len(open_exits) >= 3  ->  junction
```
Restricting to **committed** cells (`cell in motion.committed`) ensures the walls are finalized
(2× corroborated) before the junction is recorded, so exits are stable.

### New class (`tugbot_maze/junction_log.py`, ROS-free)

```python
import json
import os
from typing import Dict, List, Tuple

Cell = Tuple[int, int]


class JunctionLog:
    """Per-run record of discovered junctions (cells with >= 3 open exits). Pure data + JSON
    persistence; no ROS. record() dedupes by cell and stamps discovery order + first-seen time;
    visit() counts cell entries. flush() writes atomically so a killed run still leaves a file."""

    def __init__(self):
        self._j: Dict[Cell, dict] = {}
        self._order = 0
        self._visits: Dict[Cell, int] = {}

    def visit(self, cell: Cell) -> None:
        self._visits[cell] = self._visits.get(cell, 0) + 1
        if cell in self._j:
            self._j[cell]['visits'] = self._visits[cell]

    def record(self, cell: Cell, exits: List[str], t: float, cell_size_m: float = 2.0) -> bool:
        """Record a junction the first time it is seen. Returns True if newly added."""
        if cell in self._j:
            return False
        self._order += 1
        self._j[cell] = {
            'cell': [cell[0], cell[1]],
            'center_xy': [round(cell_size_m * cell[0], 3), round(cell_size_m * cell[1], 3)],
            'exits': list(exits),
            'exit_count': len(exits),
            'first_seen_s': round(t, 2),
            'discovery_index': self._order,
            'visits': self._visits.get(cell, 0),
        }
        return True

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
```

### Node wiring (`flood_fill_solver.py`)

- **Imports:** add `DIRS, in_grid` to the `flood_fill_brain` import; add
  `from tugbot_maze.junction_log import JunctionLog`; add `import os`.
- **`__init__`:** declare param and init state:
  ```python
  self.junction_log_dir = str(self.declare_parameter('junction_log_dir', '').value)
  self.junction_log_path = os.path.join(self.junction_log_dir or 'log', 'junctions.json')
  self.junctions = JunctionLog()
  self._prev_motion_cell = None
  ```
- **`_control_tick`, `driving` branch (after `self.motion.step(...)`):**
  ```python
  cur = self.motion.cell
  if cur != self._prev_motion_cell:
      self.junctions.visit(cur)
      self._prev_motion_cell = cur
  if cur in self.motion.committed:
      exits = [d for d, (dx, dy) in DIRS.items()
               if in_grid((cur[0] + dx, cur[1] + dy)) and not self.brain.is_wall(cur, d)]
      if len(exits) >= 3 and self.junctions.record(cur, exits, t):
          self.get_logger().info('JUNCTION cell=%s exits=%s order=%d visits=%d'
                                 % (cur, exits, self.junctions._order,
                                    self.junctions._j[cur]['visits']))
  ```
- **`EXIT_REACHED` branch:** after publishing the goal event, `self.junctions.flush(self.junction_log_path)`.
- **`_diag_tick`:** at the end, `self.junctions.flush(self.junction_log_path)` (periodic 5 s flush so
  a killed run still leaves data).

### Artifact location

The node's runtime CWD is the workspace root (`tools/run_flood_fill_maze.sh` does `cd "$WS"`). The
run script already creates `ART="log/flood_fill_run_${STAMP}"`. Passing `junction_log_dir:=$ART`
lands `junctions.json` beside `launch.log`. If the param is empty (node run directly),
`JunctionLog.flush` writes/creates `log/junctions.json` under CWD.

### Launch + run-script changes

- **`tugbot_maze_explore.launch.py`:** add a `DeclareLaunchArgument('junction_log_dir', default_value='')`
  and pass `{'junction_log_dir': LaunchConfiguration('junction_log_dir')}` in the `flood_fill_solver`
  Node `parameters` list.
- **`run_flood_fill_maze.sh`:** add `junction_log_dir:=$ART` to the `ros2 launch ...` args; add
  `JUNCTION` to the `flood_fill_tail.txt` grep alternation.

---

## Testing & validation

**Unit — `test/test_hop_controller.py` (add `backout_command`):**
- `backout_command(0.0, 0.0)` → `v == -0.30`, `|w|` ≈ 0 (aligned → straight reverse).
- heading error → `w` sign reduces the error toward `hold_cardinal` (e.g. `yaw=0.2, hold=0.0` → `w < 0`).
- custom `backout_v` honoured (returned `v == -backout_v`).

**Unit — `test/test_maze_motion.py` (add):**
- `test_deadend_triggers_backout`: brain with `D=(5,5)` walled N/E/W, open S to `P=(5,4)`; set
  `motion.cell=D`; call `_route` → `phase == 'backout'`, `backout_target == (5,4)`,
  `backout_cardinal == atan2(+1, 0)` (≈ +π/2, facing N into the dead-end).
- `test_backout_reverses_then_advances`: from the above, call `_backout` with a pose still inside
  `D` → `v < 0` (firm, ≈ −0.30) and small `|w|`; then with a pose moved one cell south (into `P`) →
  `motion.cell == (5,4)` and `phase == 'center'`.
- `test_non_deadend_does_not_backout`: a cell with 2 open exits routes normally (`phase == 'turn'`,
  not `backout`); a junction (3 open exits) also routes normally.

**Unit — `test/test_junction_log.py` (new):**
- `record` dedupes (second call same cell → `False`; `junction_count == 1`).
- `discovery_index` increments across distinct cells; `to_dict()['junctions']` sorted by it.
- `visit` increments `visits`, and a `visit` before `record` is reflected in the recorded entry.
- `flush(tmp_path/'junctions.json')` writes parseable JSON with the documented shape and creates
  missing parent dirs.

**Offline regression — `test/test_maze_motion_sim.py` (must stay green):**
- The end-to-end `maze_sim` solve (inertia + collision + odom drift + cmd latency) must still reach
  `EXIT_CELL`. Back-out is anticipatory and now active in the offline solve (the explorer enters
  real dead-ends), so this run also exercises Component 1 end-to-end. Component 2 is node-only and
  not exercised here (zero regression risk to the solver).

**Gazebo (one confirmation run):**
`tools/run_flood_fill_maze.sh 1800 false true odom_locked true` (GUI, 1800 s, odom_locked,
sense_debug). Clean stray sims first via the `!`-prefix pkill. Watch for: decisive reversals at
dead-ends (no 3 s stalls, no body clips), and a populated `log/flood_fill_run_<stamp>/junctions.json`
plus `JUNCTION` lines in `launch.log`.

**Success criteria:**
- All new unit tests pass; offline regression stays green.
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
