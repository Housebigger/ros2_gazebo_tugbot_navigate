# P3 Routing Fix Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Eliminate the P3 exploration-graph-exhaustion TIMEOUT class by replacing UNSTICK's whole-tier mass re-open with a single exit-nearest edge re-open, and by diverting zero-growth escapes straight to UNSTICK under an adaptive fast watchdog window.

**Architecture:** All product changes live in ONE file (`maze_motion.py`) inside a new workspace `ros2_ws_tugbot_nav_20260722` (full clone of 20260721). Forensics (Task 2) pins the fast-window constant before implementation. The navigation chain outside `maze_motion.py` stays frozen (spec: 明确不做).

**Tech Stack:** Python 3 (ROS-free offline core + pytest), ROS2 Jazzy + Gazebo 8 for the statistical gate, colcon symlink build.

**Spec:** `docs/superpowers/specs/2026-07-19-p3-routing-fix-design.md` (读附记前置事实)。

---

## PROCESS CONSTRAINTS (verbatim, apply to every task)

- Repo root: `/home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate`. Every bash block starts with an explicit `cd` (cwd drifts between calls).
- Git commits: message via heredoc `git commit -F - <<'EOF' ... EOF`; NO backtick characters anywhere in the message; end with the trailer line `Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>`; `git add` EXPLICIT paths only (never `-A`).
- pytest in the workspace REQUIRES the env: `set +u; source install/setup.bash 2>/dev/null;` before `python3 -m pytest ...` (unsourced single-file targets die with ModuleNotFoundError). Only `python3` exists.
- `colcon build --symlink-install` is MANDATORY (plain build breaks the maze_sim data path).
- Foreground `sleep` is blocked in direct tool calls (allowed inside script files). Any pkill/pgrep whose pattern contains "gz sim" or "ros2 launch" must live in a script file, with WRITE and EXECUTE in two separate calls.
- Do NOT launch GUI Gazebo yourself — Task 7 is user-gated.
- Gazebo runs: `bash tools/run_flood_fill_maze.sh <max_s> <headless> <rviz> online_slam` from the workspace root (PRIME offload is inside the script).

**Workspace:** `$WS = ros2_ws_tugbot_nav_20260722` (created in Task 1). Branch: `p3-routing-fix`.

---

### Task 1: Workspace clone 20260722 + branch + baseline

**Files:**
- Create: `ros2_ws_tugbot_nav_20260722/` (full clone of `ros2_ws_tugbot_nav_20260721`, minus build artifacts)

- [ ] **Step 1: Branch off main and clone the workspace**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git checkout main && git pull --ff-only 2>/dev/null; git checkout -b p3-routing-fix
rsync -a --exclude build --exclude install --exclude log \
  ros2_ws_tugbot_nav_20260721/ ros2_ws_tugbot_nav_20260722/
ls ros2_ws_tugbot_nav_20260722/src/tugbot_maze/tugbot_maze/maze_motion.py
```

Expected: file listed; no build/install/log in the clone.

- [ ] **Step 2: Build**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260722
set +u; source /opt/ros/jazzy/setup.bash; colcon build --symlink-install 2>&1 | tail -3
```

Expected: `Summary: N packages finished` with 0 failures.

- [ ] **Step 3: Capture the suite baseline (failure NAMES, not counts)**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260722
set +u; source install/setup.bash 2>/dev/null
python3 -m pytest src/tugbot_maze/test -q 2>&1 | tee /tmp/p3_suite_baseline.txt | tail -3
grep '^FAILED' /tmp/p3_suite_baseline.txt | sort > /tmp/p3_baseline_failures.txt
wc -l /tmp/p3_baseline_failures.txt
```

Expected: `7 failed, 453 passed, 3 xfailed` (the 20260721 merged-main baseline); 7 names = test_maze_asset_and_config_are_present + 6 test_wall_follow_maze_sim items. If names differ from the 20260721 record, STOP and report (BLOCKED).

- [ ] **Step 4: Commit the clone**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git add ros2_ws_tugbot_nav_20260722
git commit -F - <<'EOF'
chore: clone workspace 20260722 from 20260721 for the p3-routing-fix phase

Full source clone (build/install/log excluded); suite baseline re-verified at
7 failed / 453 passed / 3 xfailed with the archived failure-name list.

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
EOF
```

---

### Task 2: Forensics-first (pins NO_PROGRESS_FAST_S; validates the dexit metric)

**Files:**
- Create: `ros2_ws_tugbot_nav_20260722/tools/p3_forensics.py`
- Modify: `docs/superpowers/specs/2026-07-19-p3-routing-fix-design.md` (附记 gets the findings)

Run artifacts to analyze (they live in the PREDECESSOR workspace):
`ros2_ws_tugbot_nav_20260721/log/flood_fill_run_20260719_175751` (pure P3) and
`.../flood_fill_run_20260719_201541` (MIXED P1+P3, cross-check).

- [ ] **Step 1: Write the forensics script**

```python
#!/usr/bin/env python3
"""P3 forensics: cadence + timeline analysis of ESCAPE/UNSTICK events in a run artifact.

Usage: python3 tools/p3_forensics.py <artifact_dir> [<artifact_dir> ...]

Parses launch.log. Reports, per run:
  - every ESCAPE/UNSTICK event with its ROS stamp;
  - inter-escape intervals (the no-progress cadence: expect ~90s clusters);
  - per-window distinct-cell footprint and phase mix (from DIAG lines between escapes);
  - each UNSTICK's reopen count n (old format) and the recovery span until the next
    escape/exit (pins NO_PROGRESS_FAST_S: the fast window must cover one full
    reopen -> re-sense -> re-route cycle);
  - dexit (geometric cell distance to EXIT_CELL=(10,9)) of every event cell, to sanity-check
    the exit-directed selection metric against where the run actually got locked.
"""
import math
import re
import sys

EXIT_CELL = (10, 9)
STAMP = re.compile(r'\[(\d+)\.(\d+)\]')
ESC = re.compile(r'ESCAPE tier=(\d+) count=(\d+) cell=\((\d+), (\d+)\) prev=(\S+ ?\S*?) '
                 r'can_reverse=(\w+) gave_up_edge=(\w+)')
UNS = re.compile(r'UNSTICK reopen cell=\((\d+), (\d+)\) n=(\d+)')
CELL = re.compile(r'cell=\((\d+), (\d+)\)')
PHASE = re.compile(r'phase=(\w+)')


def _t(line):
    m = STAMP.search(line)
    return float(m.group(1)) + float('0.' + m.group(2)) if m else None


def _dexit(cx, cy):
    return math.hypot(cx - EXIT_CELL[0], cy - EXIT_CELL[1])


def analyze(path):
    events = []          # (t, 'ESC'|'UNS', cell, raw)
    ticks = []           # (t, cell, phase) from generic DIAG lines
    with open(path + '/launch.log', errors='replace') as f:
        for line in f:
            t = _t(line)
            if t is None:
                continue
            me, mu = ESC.search(line), UNS.search(line)
            if me:
                events.append((t, 'ESC', (int(me.group(3)), int(me.group(4))), me.group(0)))
                continue
            if mu:
                events.append((t, 'UNS', (int(mu.group(1)), int(mu.group(2))), mu.group(0)))
                continue
            mc, mp = CELL.search(line), PHASE.search(line)
            if mc and mp:
                ticks.append((t, (int(mc.group(1)), int(mc.group(2))), mp.group(1)))
    print('==== %s: %d events, %d DIAG ticks ====' % (path, len(events), len(ticks)))
    esc_ts = [t for (t, k, _, _) in events if k == 'ESC']
    gaps = [b - a for a, b in zip(esc_ts, esc_ts[1:])]
    if gaps:
        gaps_s = sorted(gaps)
        print('inter-escape gaps: n=%d min=%.1f p50=%.1f p90=%.1f max=%.1f' %
              (len(gaps), gaps_s[0], gaps_s[len(gaps_s) // 2],
               gaps_s[int(len(gaps_s) * 0.9)], gaps_s[-1]))
    for i, (t, k, cell, raw) in enumerate(events):
        nxt = events[i + 1][0] if i + 1 < len(events) else None
        span = ('%.1f' % (nxt - t)) if nxt is not None else 'END'
        win = [(tt, c, p) for (tt, c, p) in ticks if t <= tt and (nxt is None or tt < nxt)]
        cells = {c for (_, c, _) in win}
        phases = {}
        for (_, _, p) in win:
            phases[p] = phases.get(p, 0) + 1
        print('%9.1f %s cell=%s dexit=%.2f span=%s cells=%d phases=%s' %
              (t, k, cell, _dexit(*cell), span, len(cells),
               sorted(phases.items(), key=lambda kv: -kv[1])[:4]))
        print('          %s' % raw)


if __name__ == '__main__':
    for p in sys.argv[1:]:
        analyze(p)
```

Write to `ros2_ws_tugbot_nav_20260722/tools/p3_forensics.py`.

NOTE: before relying on the CELL/PHASE regexes, check what the solver DIAG lines actually
look like (`grep -m3 'phase=' <artifact>/launch.log`) and adjust the two regexes to the real
format — the ESCAPE/UNSTICK regexes are exact (they match the maze_motion event strings).

- [ ] **Step 2: Run on both P3 runs**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260722
python3 tools/p3_forensics.py \
  ../ros2_ws_tugbot_nav_20260721/log/flood_fill_run_20260719_175751 \
  ../ros2_ws_tugbot_nav_20260721/log/flood_fill_run_20260719_201541
```

Expected: 25 resp. 23 ESCAPE events; 2 UNSTICK each; inter-escape p50 near 90s.

- [ ] **Step 3: Judge the two spec questions and pin the constant**

From the output, answer in writing (goes to 附记):
1. Cadence: what fraction of the run is inter-escape dead time? What phase mix fills the
   windows (confirming the reverse-and-return ping-pong)?
2. UNSTICK recovery span: how long from each UNSTICK (esp. the n=16 mass re-open) until the
   next event — this is the reopen -> re-sense -> re-route cycle. Pin
   `NO_PROGRESS_FAST_S = 30.0` if one cycle fits in ~30s; otherwise round the observed p90
   cycle UP to the next 10s (floor 20s) and use THAT value in Task 4.
3. dexit sanity: are the locked-region event cells' dexit values consistent with the
   exit-approach edge ranking FIRST under the metric (dexit of exit-adjacent far endpoints
   is 1.0-1.5 vs >=3 for the ping-pong cells)? A full cut-candidate replay is out of scope;
   the metric is mechanically pinned by Task 3 unit tests.

- [ ] **Step 4: Record findings in the spec 附记 and commit**

Append to the `## 附记` section of `docs/superpowers/specs/2026-07-19-p3-routing-fix-design.md`
a numbered entry "1. Task 2 取证" with the three answers + the pinned constant value.

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git add ros2_ws_tugbot_nav_20260722/tools/p3_forensics.py docs/superpowers/specs/2026-07-19-p3-routing-fix-design.md
git commit -F - <<'EOF'
feat: p3_forensics tool + Task 2 findings (fast-window pinned, cadence quantified)

Cadence, per-window phase mix and UNSTICK recovery spans measured on runs
175751/201541; NO_PROGRESS_FAST_S pinned from the observed recovery cycle;
findings archived in the spec addendum.

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
EOF
```

---

### Task 3: UNSTICK single-edge re-open (TDD)

**Files:**
- Create: `ros2_ws_tugbot_nav_20260722/src/tugbot_maze/test/test_p3_routing_fix.py`
- Modify: `ros2_ws_tugbot_nav_20260722/src/tugbot_maze/tugbot_maze/maze_motion.py` (`_unstick`, new helper `_edge_exit_dist`)
- Modify: `ros2_ws_tugbot_nav_20260722/src/tugbot_maze/test/test_maze_motion.py` (ONE superseded assertion — see Step 5)

Geometry cheat-sheet for the fixtures (EXIT_CELL = (10, 9), distances = hypot of cell deltas):
pocket {(5,5),(5,6)} cut-edge far endpoints: ((5,6),'E')→(6,6) d=5.00 (MIN); ((5,6),'N')→(5,7)
d=5.39; ((5,5),'E')→(6,5) d=5.66; ((5,6),'W')→(4,6) d=6.71; ((5,5),'S')→(5,4) d=7.07;
((5,5),'W')→(4,5) d=7.21.

- [ ] **Step 1: Write the failing tests**

Create `test_p3_routing_fix.py`:

```python
"""P3 routing fix: single-edge exit-directed UNSTICK + zero-growth escape escalation.

Offline TDD for docs/superpowers/specs/2026-07-19-p3-routing-fix-design.md. The whole-tier
mass re-open and the degenerate 90s tier-2 reverse cadence are the two mechanisms behind
the P3 exploration-graph-exhaustion TIMEOUTs (runs 20260719_175751 / 20260719_201541)."""
import math
from tugbot_maze.maze_motion import MazeMotion
from tugbot_maze.maze_sim import MazeSim, load_segments
from tugbot_maze.flood_fill_brain import (
    ENTRANCE_CELL, EXIT_CELL, DIRS, OPP, in_grid, cell_center)


def _pocket_walls():
    # isolate the pocket {(5,5),(5,6)}: open only the (5,5)<->(5,6) edge, wall everything else.
    return [((5, 5), 'E'), ((5, 5), 'W'), ((5, 5), 'S'),
            ((5, 6), 'N'), ((5, 6), 'E'), ((5, 6), 'W')]


def _seal(m, walls, loco=True):
    for (c, d) in walls:
        m.brain.mark(c, d, is_wall=True)
    if loco:
        m.locomotion_walls.update(walls)


# ---- Task 3: UNSTICK single-edge, exit-nearest ----

def test_unstick_reopens_exactly_one_edge_exit_nearest():
    # OLD code re-opened the whole tier at once (the n=16 committed mass-rollback of the P3
    # runs). NEW contract: ONE edge per invocation, the dexit-minimal one.
    m = MazeMotion(); m.cell = (5, 5)
    _seal(m, _pocket_walls())
    m._unstick(0.0)
    assert m.phase == 'center'
    assert len(m.reopened) == 2                        # ONE undirected edge = both directed reps
    assert ((5, 6), 'E') in m.reopened and ((6, 6), 'W') in m.reopened   # dexit argmin (5.00)
    assert m.brain._state((5, 6), 'E') != 'wall'
    assert m.brain._state((5, 5), 'E') == 'wall'       # the rest of the tier stays sealed


def test_unstick_second_call_picks_next_best_edge():
    # With the best edge already spent (reopened bound), the NEXT dexit-min edge is chosen.
    m = MazeMotion(); m.cell = (5, 5)
    _seal(m, _pocket_walls())
    m.reopened.update({((5, 6), 'E'), ((6, 6), 'W')})  # best edge already consumed
    m._unstick(0.0)
    assert ((5, 6), 'N') in m.reopened                 # next best: nb (5,7), d=5.39
    assert m.brain._state((5, 6), 'N') != 'wall'
    assert m.brain._state((5, 6), 'E') == 'wall'       # consumed edge NOT re-touched


def test_unstick_tier_order_beats_dexit():
    # Trust tiers still come first: a loco-tier edge is picked even when a committed-tier
    # edge has the globally minimal dexit.
    m = MazeMotion(); m.cell = (5, 5)
    _seal(m, _pocket_walls(), loco=False)
    m.locomotion_walls.update([((5, 5), 'E'), ((5, 5), 'W'), ((5, 5), 'S')])  # loco = (5,5) edges
    m.committed.add((5, 6))                            # (5,6) edges (incl. global dexit-min) tier-3
    m._unstick(0.0)
    assert ((5, 5), 'E') in m.reopened                 # best WITHIN the loco tier (d=5.66)
    assert ((5, 6), 'E') not in m.reopened             # global min (d=5.00) outranked by tier


def test_unstick_incident_tiebreak_then_deterministic():
    # Three cut edges tie at dexit=5.00; two are incident to self.cell; 'E' < 'N' settles it.
    m = MazeMotion(); m.cell = (6, 5)
    walls = [((5, 5), 'W'), ((5, 5), 'S'),
             ((6, 5), 'N'), ((6, 5), 'E'), ((6, 5), 'S'),
             ((5, 6), 'N'), ((5, 6), 'E'), ((5, 6), 'W')]   # R = {(5,5),(6,5),(5,6)}
    _seal(m, walls)
    m._unstick(0.0)
    assert ((6, 5), 'E') in m.reopened                 # incident tie-winner, deterministic 'E'
    assert ((5, 6), 'E') not in m.reopened             # equal dexit but NOT incident


def test_unstick_exhaustion_still_terminates_stuck():
    # The reopened bound is untouched: all cut edges consumed -> terminal 'stuck'.
    m = MazeMotion(); m.cell = (5, 5)
    base = _pocket_walls()
    _seal(m, base)
    m.reopened.update(base)                            # every cut edge already re-opened once
    m._unstick(0.0)
    assert m.phase == 'stuck'


def test_unstick_diag_single_edge_format():
    m = MazeMotion(); m.cell = (5, 5)
    _seal(m, _pocket_walls())
    m._unstick(0.0)
    ev = m.events[-1]
    assert ev.startswith('UNSTICK reopen')
    assert ' edge=' in ev and ' tier=loco' in ev and ' dexit=5.00' in ev
```

- [ ] **Step 2: Run to verify RED**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260722
set +u; source install/setup.bash 2>/dev/null
python3 -m pytest src/tugbot_maze/test/test_p3_routing_fix.py -q 2>&1 | tail -3
```

Expected: 6 failed (old code mass-reopens: len(reopened)==12 not 2; old DIAG has n=, no edge=).

- [ ] **Step 3: Implement**

In `maze_motion.py`, add the helper right BEFORE `_unstick`:

```python
    def _edge_exit_dist(self, edge):
        """Single-edge re-open metric: geometric distance (cell units) from the edge's FAR
        endpoint (the cell outside the reachable component) to EXIT_CELL. Routing continues
        from the far endpoint after crossing the re-opened edge, so nearer-to-exit edges are
        re-verified first -- a falsely-sealed exit approach ranks first."""
        (c, d) = edge
        nb = (c[0] + DIRS[d][0], c[1] + DIRS[d][1])
        return math.hypot(nb[0] - EXIT_CELL[0], nb[1] - EXIT_CELL[1])
```

Replace the whole `_unstick` body with:

```python
    def _unstick(self, t):
        """Boxed (next_cell None) or exit-unreachable -> a false WALL cuts the robot's
        reachable component off from the exit. Re-open ONE cut edge (component -> outside)
        per invocation, in ASCENDING trust tiers -- locomotion (hop-failure) -> non-committed
        sensed (poor reads) -> committed (2x-corroborated, trusted last). Within the first
        non-empty tier pick the single edge whose far endpoint is geometrically nearest
        EXIT_CELL (falsely-sealed exit approaches rank first); ties prefer edges INCIDENT to
        self.cell (re-sensable on the very next center tick), then a deterministic order.
        Single-edge replaces the former whole-tier mass re-open (20260719 P3 forensics: one
        n=16 committed mass-rollback cost minutes of re-sensing + repeated optimism). The
        re-open is OPTIMISTIC -> OPEN; the cell is un-committed but KEPT in `sensed`, so the
        gate re-WALLs it only from a GOOD re-sense. Bounded by self.reopened (both reps):
        when no un-reopened cut edge remains, the map is disconnected -> 'stuck'."""
        R = self._reachable_component(self.cell)
        cut = [(c, d) for c in R for d, (dx, dy) in DIRS.items()
               if self.brain.is_wall(c, d) and (c, d) not in self.reopened
               and in_grid((c[0] + dx, c[1] + dy)) and (c[0] + dx, c[1] + dy) not in R]
        tiers = (('loco', lambda e: e in self.locomotion_walls),
                 ('sensed', lambda e: e not in self.locomotion_walls
                                      and e[0] not in self.committed),
                 ('committed', lambda e: e[0] in self.committed))
        for tier, pick in tiers:
            cand = [e for e in cut if pick(e)]
            if not cand:
                continue
            c, d = min(cand, key=lambda e: (self._edge_exit_dist(e),
                                            0 if e[0] == self.cell else 1, e))
            nb = (c[0] + DIRS[d][0], c[1] + DIRS[d][1])
            self.brain.mark(c, d, is_wall=False)     # re-open optimistically; a GOOD re-sense re-confirms
            self.reopened.add((c, d)); self.reopened.add((nb, OPP[d]))   # both reps -> clean 1x bound
            self.locomotion_walls.discard((c, d)); self.locomotion_walls.discard((nb, OPP[d]))
            self.failed_hops.pop((c, d), None); self.failed_hops.pop((nb, OPP[d]), None)
            self.committed.discard(c); self.corrob.pop(c, None)          # keep `sensed`: re-WALL needs good
            self.center_start = None
            self.align_start = None; self.latched_cardinal = None
            self.settle_until = t + self.settle_s
            self.escape_tier = 0                      # map mutation -> retry the cheap Tier-1 first
            self.events.append("UNSTICK reopen cell=%s edge=(%s,%s) tier=%s dexit=%.2f"  # DIAG
                               % (self.cell, c, d, tier, self._edge_exit_dist((c, d))))
            self.phase = 'center'                      # explicit: re-route / re-sense next tick
            return (0.0, 0.0, False)
        self.events.append("UNSTICK exhausted -> stuck cell=%s" % (self.cell,))  # DIAG
        self.phase = 'stuck'
        return (0.0, 0.0, False)
```

- [ ] **Step 4: Run the new tests to verify GREEN**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260722
set +u; source install/setup.bash 2>/dev/null
python3 -m pytest src/tugbot_maze/test/test_p3_routing_fix.py -q 2>&1 | tail -3
```

Expected: 6 passed.

- [ ] **Step 5: Upgrade the ONE superseded assertion (authorized contract change)**

`test_maze_motion.py::test_disconnecting_false_wall_triggers_unstick` pinned the mass-reopen:
it asserts `(cell, 'E') in m.reopened` and `m.brain._state((5, 5), 'E') != 'wall'` after an
unstick on the same pocket. Under single-edge the reopened edge is the dexit-min ((5,6),'E').
Replace those two final assertions with:

```python
    assert m.phase != 'stuck'                                # unstick fired (no silent wander/false-stuck)
    assert len(m.reopened) == 2                              # single-edge contract (P3 fix, 20260720)
    assert ((5, 6), 'E') in m.reopened                       # dexit-min cut edge
    assert m.brain._state((5, 6), 'E') != 'wall'             # the chosen cut wall was re-opened
```

Then run the WHOLE existing motion test file and triage: any OTHER failure that pins
whole-tier reopen semantics is a superseded contract — report it (BLOCKED) before editing;
this plan pre-authorizes ONLY the one assertion block above.

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260722
set +u; source install/setup.bash 2>/dev/null
python3 -m pytest src/tugbot_maze/test/test_maze_motion.py -q 2>&1 | tail -3
```

Expected: all pass (the unstick-escalation and stuck-termination tests hold: they assert
len(reopened) > 0 and the bound, both preserved).

- [ ] **Step 6: Commit**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git add ros2_ws_tugbot_nav_20260722/src/tugbot_maze/tugbot_maze/maze_motion.py \
        ros2_ws_tugbot_nav_20260722/src/tugbot_maze/test/test_p3_routing_fix.py \
        ros2_ws_tugbot_nav_20260722/src/tugbot_maze/test/test_maze_motion.py
git commit -F - <<'EOF'
feat: UNSTICK single-edge exit-directed re-open (kills the whole-tier mass rollback)

Within the first non-empty trust tier, exactly ONE cut edge is re-opened per
invocation: dexit-minimal (far endpoint nearest EXIT_CELL), ties prefer
incident-to-cell then deterministic order. Tier order, reopened bound and
stuck termination unchanged. DIAG line now carries edge/tier/dexit. One
superseded mass-reopen assertion in test_maze_motion upgraded per plan
authorization.

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
EOF
```

---

### Task 4: ESCAPE zero-growth escalation + adaptive window (TDD)

**Files:**
- Modify: `ros2_ws_tugbot_nav_20260722/src/tugbot_maze/tugbot_maze/maze_motion.py` (module constant, `__init__`, `step` watchdog, `_confined`, `_track_cell`, `_escape`)
- Modify: `ros2_ws_tugbot_nav_20260722/src/tugbot_maze/test/test_p3_routing_fix.py` (append tests)
- Possibly modify: `ros2_ws_tugbot_nav_20260722/src/tugbot_maze/test/test_maze_motion.py` (Step 5 triage)

Use the Task-2-pinned value everywhere this plan writes `30.0` for `NO_PROGRESS_FAST_S`
(if Task 2 pinned a different value, substitute it consistently, incl. the constants test).

- [ ] **Step 1: Append the failing tests to test_p3_routing_fix.py**

```python
# ---- Task 4: ESCAPE zero-growth escalation + adaptive watchdog window ----

def test_escape_first_fire_growth_positive_keeps_ladder():
    # _esc_visited_n starts at 0, so the FIRST escape always sees growth>0 -> the Tier-1
    # reverse ladder is behavior-identical to the old code.
    m = MazeMotion(); m.cell = (5, 5); m.prev_cell = (5, 4)
    m.visited.update({(5, 4), (5, 5)})
    m._escape((10.0, 10.0, 0.0), 100.0)
    assert m._escape_backout is True and m.phase == 'backout'
    assert m.backout_target == (5, 4)
    ev = [e for e in m.events if e.startswith('ESCAPE')][-1]
    assert ' growth=' in ev and ' win=' in ev


def test_escape_zero_growth_diverts_to_unstick_and_fast_window():
    m = MazeMotion(); m.cell = (5, 5); m.prev_cell = (5, 6)   # (5,5)<->(5,6) is the OPEN edge
    m.visited.update({(5, 5), (5, 6)})
    _seal(m, _pocket_walls())
    m._escape((10.0, 10.0, 0.0), 100.0)                # growth>0 -> normal reverse
    assert m.phase == 'backout'
    m.phase = 'center'; m._escape_backout = False
    m._escape((10.0, 10.0, 0.0), 200.0)                # visited unchanged -> growth==0
    assert m._no_progress_win == m.no_progress_fast_s  # fast cadence armed
    assert m.phase == 'center'                         # diverted to _unstick: single-edge reopen
    assert len(m.reopened) == 2
    assert m._escape_backout is False                  # NO degenerate reverse
    esc = [e for e in m.events if e.startswith('ESCAPE')][-1]
    assert 'growth=0' in esc


def test_escape_growth_restores_calm_window():
    m = MazeMotion(); m.cell = (5, 5); m.prev_cell = (5, 6)
    m.visited.update({(5, 5), (5, 6)})
    _seal(m, _pocket_walls())
    m._escape((10.0, 10.0, 0.0), 100.0)
    m.phase = 'center'; m._escape_backout = False
    m._escape((10.0, 10.0, 0.0), 200.0)                # zero growth -> fast window
    assert m._no_progress_win == m.no_progress_fast_s
    m.cell = (6, 6)                                    # NEW GROUND
    m._track_cell(300.0)
    assert m._no_progress_win == m.no_progress_s       # calm window restored
    assert m.escape_tier == 0                          # existing growth-clears-escalation kept


def test_watchdog_fires_on_adaptive_window():
    # The step() gate reads _no_progress_win, not no_progress_s.
    m = MazeMotion()
    fired = []
    m._escape = lambda pose, t: (fired.append(t), (0.0, 0.0, False))[1]
    m._no_progress_win = 5.0
    m.explore_t = 0.0
    m.recent = [(5.5, ENTRANCE_CELL)]                  # confined footprint
    sim = MazeSim(load_segments(), cell_center(ENTRANCE_CELL), 0.0)
    scan = sim.scan(n_beams=360, fov_rad=2 * math.pi)
    m.step(sim.pose, scan, 6.0)                        # 6.0 > 5.0 fast window
    assert fired == [6.0]
    m2 = MazeMotion()
    m2._escape = lambda pose, t: (fired.append(-t), (0.0, 0.0, False))[1]
    m2.explore_t = 0.0
    m2.recent = [(5.5, ENTRANCE_CELL)]
    m2.step(sim.pose, scan, 6.0)                       # 6.0 < default 90.0 -> silent
    assert fired == [6.0]


def test_p3_constants_pinned():
    m = MazeMotion()
    assert m.no_progress_s == 90.0
    assert m.no_progress_fast_s == 30.0
    assert m._no_progress_win == 90.0
    assert m._esc_visited_n == 0
```

- [ ] **Step 2: Run to verify RED**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260722
set +u; source install/setup.bash 2>/dev/null
python3 -m pytest src/tugbot_maze/test/test_p3_routing_fix.py -q 2>&1 | tail -3
```

Expected: the 5 new tests fail (AttributeError no_progress_fast_s / missing growth fields);
the 6 Task-3 tests still pass.

- [ ] **Step 3: Implement**

(a) Module constant, after the `_dir_name` definition near the top of `maze_motion.py`:

```python
NO_PROGRESS_FAST_S = 30.0   # exhausted-state watchdog window (pinned by the 175751 forensics, Task 2)
```

(b) In `__init__`, directly after `self.max_escape_tier = 2`:

```python
        self.no_progress_fast_s = NO_PROGRESS_FAST_S  # exhausted-state fast window (P3 fix)
        self._no_progress_win = self.no_progress_s    # current adaptive watchdog window
        self._esc_visited_n = 0                       # len(visited) at the previous escape
```

(c) In `step()`, the watchdog condition: replace
`and (t - self.explore_t) > self.no_progress_s` with
`and (t - self.explore_t) > self._no_progress_win`.

(d) In `_confined()`: replace `if t2 >= t - self.no_progress_s` with
`if t2 >= t - self._no_progress_win` (and mention the adaptive window in its docstring).

(e) In `_track_cell()`, inside the `if self.cell not in self.visited:` NEW GROUND branch,
after `self.escape_tier = 0` add:

```python
            self._no_progress_win = self.no_progress_s   # growth -> restore the calm window
```

(f) Replace the whole `_escape` body with:

```python
    def _escape(self, pose, t):
        """No-progress escape (top-of-step watchdog fired: no new ground for the adaptive
        window while confined). growth = visited cells gained since the PREVIOUS escape.
        While the maze still yields new ground the escalation ladder is unchanged: Tier 1 =
        decisive one-cell reverse to the adjacent known-open prev_cell; Tier 2 = ALSO give
        up the blocked forward edge. ZERO growth between escapes is the exploration-
        exhausted signal (20260719 P3 forensics: 23-25 escapes, near-all tier=2
        gave_up_edge=False -- every neighbour VISITED, the reverse changes nothing): divert
        straight to _unstick (single-edge re-open = a guaranteed map change) and drop the
        watchdog to the fast window until visited grows again. No valid reverse -> hand to
        _unstick AT MOST ONCE this tick (terminal; C2 revives next tick)."""
        x, y, yaw = pose
        self.escape_count += 1
        self.escape_tier = min(self.escape_tier + 1, self.max_escape_tier)
        self.explore_t = t                                   # reset on EVERY entry (no busy-re-fire)
        growth = len(self.visited) - self._esc_visited_n
        self._esc_visited_n = len(self.visited)
        pc = self.prev_cell
        man = None if pc is None else abs(pc[0] - self.cell[0]) + abs(pc[1] - self.cell[1])
        d_prev = (_dir_name((pc[0] - self.cell[0], pc[1] - self.cell[1]))
                  if (pc is not None and man == 1) else None)
        can_reverse = d_prev is not None and not self.brain.is_wall(self.cell, d_prev)
        if growth == 0:                                      # EXHAUSTED: no new ground since last escape
            self._no_progress_win = self.no_progress_fast_s  # fast cadence until visited grows
            self.events.append("ESCAPE tier=%d count=%d cell=%s prev=%s can_reverse=%s "
                               "gave_up_edge=%s growth=%d win=%.0f"  # DIAG
                               % (self.escape_tier, self.escape_count, self.cell,
                                  self.prev_cell, can_reverse, False, growth,
                                  self._no_progress_win))
            return self._unstick(t)                          # map-changing action, not a degenerate reverse
        gave_up = self.escape_tier >= 2 and self.hop_dir is not None
        if gave_up:                                          # Tier 2+: GIVE UP the blocked edge
            dirn = _dir_name(self.hop_dir)
            nb = (self.cell[0] + DIRS[dirn][0], self.cell[1] + DIRS[dirn][1])
            if nb not in self.visited:                       # never give up an edge to a VISITED cell
                self.brain.mark(self.cell, dirn, is_wall=True)   # the real routing change (symmetric)
                self._stamp_loco_wall(self.cell, dirn)           # provenance: _unstick reopens loco first
                self.committed.discard(self.cell)
                self.failed_hops.pop((self.cell, dirn), None)
            else:
                gave_up = False                              # visited edge stays open (accurate logging)
        self.events.append("ESCAPE tier=%d count=%d cell=%s prev=%s can_reverse=%s "
                           "gave_up_edge=%s growth=%d win=%.0f"  # DIAG
                           % (self.escape_tier, self.escape_count, self.cell, self.prev_cell,
                              can_reverse, gave_up, growth, self._no_progress_win))
        if can_reverse:                                      # Tier 1 & 2: one-cell reverse to prev
            dx, dy = DIRS[d_prev]
            self.backout_target = pc
            self.backout_cardinal = math.atan2(-dy, -dx)     # face away from prev -> reverse into it
            self.backout_start = (x, y)
            self.backout_deadline = t + self.backout_timeout_s
            self.center_start = None
            self._escape_backout = True
            self.phase = 'backout'
            return (0.0, 0.0, False)
        return self._unstick(t)                              # no reverse -> _unstick once (MF5)
```

- [ ] **Step 4: Run to verify GREEN**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260722
set +u; source install/setup.bash 2>/dev/null
python3 -m pytest src/tugbot_maze/test/test_p3_routing_fix.py -q 2>&1 | tail -3
```

Expected: 11 passed.

- [ ] **Step 5: Triage the existing escape tests (zero-growth collisions)**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260722
set +u; source install/setup.bash 2>/dev/null
python3 -m pytest src/tugbot_maze/test/test_maze_motion.py -q 2>&1 | tail -5
```

Any test that calls `_escape` REPEATEDLY without growing `m.visited` between calls (e.g.
`test_escape_tier_caps_at_max`) now diverts to `_unstick` on the 2nd+ call. Its INTENT
(tier capping / ladder pinning) is preserved by seeding growth between calls: before each
subsequent `_escape`, add a fresh cell, e.g. `m.visited.add((8, 8)); ...; m.visited.add((8, 7))`
(any never-before-used cells). Read each failing test, apply the minimal visited-seeding
edit that preserves its original assertion intent, and record the list in the commit
message. If a failing test's intent is genuinely the OLD repeated-zero-growth reverse
behavior, that contract is superseded — report BLOCKED with the test name before editing.

- [ ] **Step 6: Commit**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git add ros2_ws_tugbot_nav_20260722/src/tugbot_maze/tugbot_maze/maze_motion.py \
        ros2_ws_tugbot_nav_20260722/src/tugbot_maze/test/test_p3_routing_fix.py \
        ros2_ws_tugbot_nav_20260722/src/tugbot_maze/test/test_maze_motion.py
git commit -F - <<'EOF'
feat: escape zero-growth escalation + adaptive fast watchdog window

growth (visited gained since the previous escape) == 0 is the exploration-
exhausted signal: the escape diverts straight to the single-edge _unstick
instead of the degenerate one-cell reverse, and the no-progress window drops
to NO_PROGRESS_FAST_S until visited grows again (restored in _track_cell).
growth>0 keeps the old ladder byte-identical. ESCAPE DIAG line gains
growth=/win= fields. Existing repeated-escape tests seeded with visited
growth to preserve their ladder-pinning intent.

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
EOF
```

---

### Task 5: Integration (sealed exit approach) + full-suite gate

**Files:**
- Modify: `ros2_ws_tugbot_nav_20260722/src/tugbot_maze/test/test_p3_routing_fix.py` (append)

- [ ] **Step 1: Append the end-to-end test**

```python
# ---- Task 5: integration -- sealed exit approach recovers and solves ----

def test_sealed_exit_approach_recovers_and_solves():
    # P3 end-to-end: every interior approach edge of EXIT_CELL starts falsely WALLed and
    # COMMITTED (worst-trust tier: the old code answered this class with an n=16 mass
    # rollback in runs 175751/201541). The fix must recover via single-edge re-opens and
    # still reach the exit, with EVERY UNSTICK event single-edge.
    sim = MazeSim(load_segments(), cell_center(ENTRANCE_CELL), 0.0, inertia=True)
    m = MazeMotion()
    m.no_progress_s = 9.0              # x10 test-scaled cadence (product 90s)
    m._no_progress_win = 9.0           # post-construction override must set BOTH
    m.no_progress_fast_s = 3.0         # x10 test-scaled fast window (product 30s)
    for d, (dx, dy) in DIRS.items():
        nb = (EXIT_CELL[0] + dx, EXIT_CELL[1] + dy)
        if in_grid(nb):
            m.brain.mark(nb, OPP[d], is_wall=True)     # seal from the interior side
            m.committed.add(nb)
    t, done = 0.0, False
    for _ in range(60000):
        scan = sim.scan(n_beams=360, fov_rad=2 * math.pi)
        v, w, done = m.step(sim.pose, scan, t)
        if done:
            break
        sim.step(v, w, 0.1)
        t += 0.1
    assert done, "sealed exit approach not recovered (P3 signature)"
    for e in m.events:
        if e.startswith("UNSTICK reopen"):
            assert " edge=" in e                       # single-edge format = no mass re-open
```

- [ ] **Step 2: Run the new test (GREEN on the fixed code)**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260722
set +u; source install/setup.bash 2>/dev/null
python3 -m pytest src/tugbot_maze/test/test_p3_routing_fix.py -q 2>&1 | tail -3
```

Expected: 12 passed (this test may take ~1-2 min: full offline solve).
If the solve does not complete, this is a REAL finding — debug before proceeding
(the escape/unstick interplay is the product behavior under test).

- [ ] **Step 3: RED evidence on the OLD stack (historical cross-check)**

Run the same test in the PREDECESSOR workspace (old code) to certify the red:

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260721
set +u; source install/setup.bash 2>/dev/null
cp ../ros2_ws_tugbot_nav_20260722/src/tugbot_maze/test/test_p3_routing_fix.py /tmp/p3_red_check.py
python3 -m pytest /tmp/p3_red_check.py::test_sealed_exit_approach_recovers_and_solves -q 2>&1 | tail -3
```

Expected: FAIL — either the " edge=" format assertion (old mass-reopen logs n=) or the solve
budget. Record which. (Do NOT commit anything in 20260721.)

- [ ] **Step 4: Regression sims + full suite vs baseline**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260722
set +u; source install/setup.bash 2>/dev/null
python3 -m pytest src/tugbot_maze/test/test_maze_motion_sim.py src/tugbot_maze/test/test_online_slam_sim.py -q 2>&1 | tail -3
python3 -m pytest src/tugbot_maze/test -q 2>&1 | tee /tmp/p3_suite_after.txt | tail -3
grep '^FAILED' /tmp/p3_suite_after.txt | sort > /tmp/p3_after_failures.txt
diff /tmp/p3_baseline_failures.txt /tmp/p3_after_failures.txt && echo BASELINE-CLEAN
```

Expected: sims pass; full suite = 7 failed (SAME names, diff empty -> BASELINE-CLEAN) /
passed grows by the new test count / 3 xfailed.

- [ ] **Step 5: Commit**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git add ros2_ws_tugbot_nav_20260722/src/tugbot_maze/test/test_p3_routing_fix.py
git commit -F - <<'EOF'
test: sealed-exit-approach end-to-end integration (single-edge recovery solves)

All interior approach edges of EXIT_CELL falsely committed-WALLed at start;
the motion layer recovers via single-edge exit-directed re-opens and reaches
the exit in the offline inertia+collision sim; every UNSTICK event verified
single-edge. Red certified on the 20260721 stack. Full suite: baseline
failure names unchanged.

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
EOF
```

---

### Task 6: Headless statistical gate ×8 (3+3+2)

**Files:** none (run artifacts under `ros2_ws_tugbot_nav_20260722/log/`; findings -> spec 附记)

- [ ] **Step 1: Batch 1 (×3)** — write a batch script (background), runs sequential:

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260722
cat > /tmp/p3_batch.sh <<'EOF'
#!/bin/bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260722
for i in 1 2 3; do
  bash tools/run_flood_fill_maze.sh 3600 true false online_slam
  echo "=== RUN $i DONE ==="
done
EOF
chmod +x /tmp/p3_batch.sh
```

Then EXECUTE in a SEPARATE call (background): `bash /tmp/p3_batch.sh` (~3h worst case).

- [ ] **Step 2: Judge each run against the taxonomy**

Per run: result (EXIT_REACHED/TIMEOUT), `python3 tools/replay_collision_oracle.py <artifact>`
(official + live_rate), and classification: P1 = central-junction localization signature
(stale-yaw MATCH forensics, self-recovered) -> EXEMPT; P2 = clamp-lock (must stay zero);
P3 = exploration-exhaustion routing signature (zero-growth escape cadence, UNSTICK
patterns) -> GATE FAILURE. Also verify the NEW DIAG fields appear (growth=/win=; UNSTICK
edge=/tier=/dexit=) and note yaw_only/clamp_escape counts (localization stack must look
unchanged).

- [ ] **Step 3: Batches 2 (×3) and 3 (×2)** — same script pattern. STOP the batch sequence
on any P3-class bad run: forensics FIRST (p3_forensics.py + launch.log timeline) before any
parameter change; document, then decide (fix within spec / amend spec with user).

- [ ] **Step 4: Gate verdict + record**

Gate = ZERO P3-class bad runs across the 8 + oracle official <= 0.719% per run + P2 == 0.
Append the per-run table + verdict as the next numbered 附记 entry in the spec; commit:

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git add docs/superpowers/specs/2026-07-19-p3-routing-fix-design.md
git commit -F - <<'EOF'
docs: Task 6 statistical-gate record (headless x8, per-class verdicts)

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
EOF
```

---

### Task 7: GUI + RViz acceptance (USER-GATED — do not launch yourself)

- [ ] **Step 1:** Report the Task 6 tally and ASK THE USER to trigger:
`cd ros2_ws_tugbot_nav_20260722 && bash tools/run_flood_fill_maze.sh 3600 false true online_slam`
- [ ] **Step 2:** Judge same as Task 6 (EXIT + oracle <= 0.719% official, live_rate reported,
class check); user confirms visuals. Record as a 附记 entry (committed with Task 8 docs).

---

### Task 8: Docs + final review + merge

**Files:**
- Modify: `ros2_ws_tugbot_nav_20260722/README.md`
- Modify: `docs/superpowers/specs/2026-07-19-p3-routing-fix-design.md` (final 附记)

- [ ] **Step 1: README** — rename every this-workspace `ros2_ws_tugbot_nav_20260721`
reference to `ros2_ws_tugbot_nav_20260722` (title + run commands; verify with grep, keep
historical-predecessor mentions); add a routing subsection (after the localization section)
describing: single-edge exit-directed UNSTICK (tier order kept, dexit metric, reopened
bound), zero-growth escape escalation (growth signal, fast window 30s, restore-on-growth),
and the new DIAG observability (ESCAPE growth=/win=, UNSTICK edge=/tier=/dexit=), citing
the P3 forensics numbers (25 escapes x 90s, n=16 mass reopen). Append the final 附记 entry
(GUI result + disposition). Commit both files (heredoc, no backticks, trailer).

- [ ] **Step 2: Final whole-branch review** (fable subagent): diff `main...p3-routing-fix`;
UNFREEZE-BOUNDARY AUDIT — within `ros2_ws_tugbot_nav_20260722/src/tugbot_maze/tugbot_maze/`
ONLY `maze_motion.py` may differ from the 20260721 predecessor; workspace-level expected
diffs: README.md, tools/p3_forensics.py, test_p3_routing_fix.py, test_maze_motion.py
(authorized assertions only); repo-level: the spec only. Constants vs spec; suite gate re-run
with baseline names. APPROVE required before merge.

- [ ] **Step 3: Finishing flow** (superpowers:finishing-a-development-branch): full suite on
branch (baseline names) -> merge to main `git merge --no-ff p3-routing-fix -F <tempfile>`
(message: no backticks, trailer) -> full suite re-run on main -> `git push origin main` ->
`git branch -d p3-routing-fix` -> memory file (`p3-routing-fix-solved.md`, cross-link
[[yaw-only-fallback-solved]]) + MEMORY.md index line.

---

## Self-Review (done at authoring)

- Spec coverage: workspace/branch (T1), 取证先行 + fast-window pinning + dexit validation
  (T2), 组件 1 single-edge (T3), 组件 2 zero-growth + adaptive window (T4), TDD (a)-(d)
  (T3/T4/T5: unit red evidence via new-test-on-old-code, integration red certified on the
  20260721 stack), 统计门 zero-P3 (T6), GUI (T7), docs/merge (T8). No gaps.
- Known superseded contracts pre-authorized: ONE assertion block in
  test_disconnecting_false_wall_triggers_unstick (T3 Step 5); visited-seeding in repeated
  `_escape` tests (T4 Step 5). Anything beyond -> BLOCKED report first.
- Type consistency: `no_progress_fast_s` / `_no_progress_win` / `_esc_visited_n` /
  `_edge_exit_dist` / tier names 'loco'|'sensed'|'committed' used identically in T3/T4/T5
  code and tests. Fixture geometry double-checked against EXIT_CELL=(10,9)
  (dexit values 5.00/5.39/5.66/6.71/7.07/7.21; the 3-way 5.00 tie in the incident test).
