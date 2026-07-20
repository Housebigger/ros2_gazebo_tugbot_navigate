# Localization Root-Cause Fix Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Confirm-or-refute the along-track pose-lag verdict with a measurement harness, ship a map-poisoning leading indicator, then fork on the measurement to fix the dominant cause (expected: the ICP locality gate + a 3-DOF rms ceiling).

**Architecture:** New workspace `ros2_ws_tugbot_nav_20260723` (clone of 20260722). Task A is a default-OFF measurement scaffold (a ground-truth bridge + a dumb three-pose logger in the solver + an offline decomposition tool that owns ALL frame reconciliation). Task B ships an online believed-track edge-legality detector (zero ground truth). Task C forks on Task A's verdict; the expected ICP branch fixes `_has_local_interior` then, sequentially, adds a `residual_rms` ceiling. Ground truth / odom NEVER enter any control or localization loop.

**Tech Stack:** Python 3 (ROS-free offline core + pytest), ROS2 Jazzy + Gazebo 8, ros_gz_bridge, colcon symlink build.

**Spec:** `docs/superpowers/specs/2026-07-20-localization-root-cause-design.md` (读红线与关键事实前置).

---

## PROCESS CONSTRAINTS (verbatim, apply to every task)

- Repo root: `/home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate`. Every bash block starts with an explicit `cd` (cwd drifts).
- Git commits: message via heredoc `git commit -F - <<'EOF' ... EOF`; NO backtick characters in the message; end with the trailer `Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>`; `git add` EXPLICIT paths only (never `-A`; the repo has untracked `tmp_resources/` that must stay untracked).
- pytest in the workspace REQUIRES: `set +u; source install/setup.bash 2>/dev/null;` before `python3 -m pytest ...` from the workspace root. Only `python3` exists.
- `colcon build --symlink-install` is MANDATORY (plain build breaks the maze_sim data path).
- Foreground `sleep` is blocked in direct tool calls (allowed inside script files). pkill/pgrep whose pattern contains "gz sim"/"ros2 launch" must live in a script file, WRITE and EXECUTE in two separate calls.
- Do NOT launch GUI Gazebo yourself — the GUI acceptance task is user-gated.
- Gazebo runs: `bash tools/run_flood_fill_maze.sh <max_s> <headless> <rviz> online_slam` from the workspace root (PRIME offload is inside the script).
- **RED LINE (spec):** ground-truth pose and `/odom` may ONLY be logged and analysed offline. They must NEVER be read by `correct()`, the controller, or any pose input. Task A's logger is default-OFF (`pose_diag:=false`); product runs (the statistical gate body, GUI acceptance) run with it OFF.

**Workspace:** `$WS = ros2_ws_tugbot_nav_20260723` (created in Task 1). Branch: `localization-root-cause`.
**gz world name:** `tugbot_maze_world_20260528_clean_scaled2x`.
**Map origin (coordinate trap):** entrance `(11.011, 9.025)`; passed to the solver as params `entrance_x/entrance_y/entrance_yaw`.

---

### Task 1: Workspace clone 20260723 + branch + baseline

**Files:** Create `ros2_ws_tugbot_nav_20260723/` (clone of 20260722, minus build artifacts).

- [ ] **Step 1: Branch off main and clone**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git checkout main && git checkout -b localization-root-cause
rsync -a --exclude build --exclude install --exclude log \
  ros2_ws_tugbot_nav_20260722/ ros2_ws_tugbot_nav_20260723/
ls ros2_ws_tugbot_nav_20260723/src/tugbot_maze/tugbot_maze/online_scan_match_localizer.py
```
Expected: file listed; no build/install/log in the clone.

- [ ] **Step 2: Build**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260723
set +u; source /opt/ros/jazzy/setup.bash; colcon build --symlink-install 2>&1 | tail -3
```
Expected: `Summary: N packages finished` with 0 failures.

- [ ] **Step 3: Capture the suite baseline (failure NAMES)**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260723
set +u; source install/setup.bash 2>/dev/null
python3 -m pytest src/tugbot_maze/test -q 2>&1 | tee /tmp/loc_suite_baseline.txt | tail -3
grep '^FAILED' /tmp/loc_suite_baseline.txt | sort > /tmp/loc_baseline_failures.txt
wc -l /tmp/loc_baseline_failures.txt
```
Expected: `7 failed, 470 passed, 3 xfailed` (the 20260722 merged-main baseline); 7 names = test_maze_asset_and_config_are_present + 6 test_wall_follow_maze_sim items. If names differ, STOP and report BLOCKED with the actual list.

- [ ] **Step 4: Commit**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git add ros2_ws_tugbot_nav_20260723
git status --short | head -3
git commit -F - <<'EOF'
chore: clone workspace 20260723 from 20260722 for the localization-root-cause phase

Full source clone (build/install/log excluded); suite baseline re-verified at
7 failed / 470 passed / 3 xfailed with the archived failure-name list.

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
EOF
```

---

### Task 2: Task A instrumentation — ground-truth bridge + dumb three-pose logger + offline decomposer

**Files:**
- Modify: `ros2_ws_tugbot_nav_20260723/src/tugbot_gazebo/config/tugbot_bridge.yaml` (add ground-truth pose bridge)
- Modify: `ros2_ws_tugbot_nav_20260723/src/tugbot_maze/tugbot_maze/flood_fill_solver.py` (default-off POSEDIAG logger)
- Create: `ros2_ws_tugbot_nav_20260723/tools/pose_decompose.py`
- Create: `ros2_ws_tugbot_nav_20260723/src/tugbot_maze/test/test_pose_decompose.py`

The logger is DUMB: it logs three raw poses verbatim. ALL frame reconciliation and error decomposition lives in the offline tool, where it is testable and cannot touch the control loop.

- [ ] **Step 1: Add the ground-truth bridge entry**

Append to `ros2_ws_tugbot_nav_20260723/src/tugbot_gazebo/config/tugbot_bridge.yaml` (a NEW top-level list entry; match the file's existing 2-space list style):

```yaml
- ros_topic_name: "/gt/dynamic_pose"
  gz_topic_name: "/world/tugbot_maze_world_20260528_clean_scaled2x/dynamic_pose/info"
  ros_type_name: "tf2_msgs/msg/TFMessage"
  gz_type_name: "gz.msgs.Pose_V"
  direction: "GZ_TO_ROS"
```
This republishes every dynamic entity's world pose as a `TFMessage` on `/gt/dynamic_pose`; the robot base appears with `child_frame_id` containing `anymal_c` / `base_link`. This is eval scaffolding (config only); no control code reads it.

- [ ] **Step 2: Write the failing test for the offline decomposer**

Create `ros2_ws_tugbot_nav_20260723/src/tugbot_maze/test/test_pose_decompose.py`:

```python
"""Offline pose-decomposition for the localization-root-cause measurement (Task A).

POSEDIAG log lines carry three raw world/frame poses per tick (gt, odom, solver). This tool
reconciles frames and decomposes the solver-vs-ground-truth error into along-track / lateral /
yaw so the along-track-lag verdict can be confirmed or refuted. Pure Python, no ROS."""
import math
from tugbot_maze.pose_decompose import decompose_error, parse_posediag_line


def test_parse_posediag_line():
    line = ("[1784500000.500000000] [flood_fill_solver]: POSEDIAG "
            "gt=(1.000, 2.000, 0.500) odom=(1.100, 2.050, 0.480) solver=(3.000, 2.100, 0.520)")
    r = parse_posediag_line(line)
    assert r is not None
    assert r["t"] == 1784500000.5
    assert r["gt"] == (1.0, 2.0, 0.5)
    assert r["odom"] == (1.1, 2.05, 0.48)
    assert r["solver"] == (3.0, 2.1, 0.52)


def test_decompose_pure_alongtrack_error():
    # Solver believes it is 2.0 m BEHIND the true pose along the travel heading (yaw=0 -> +x).
    # gt at x=5, solver at x=3, same y and yaw. Along-track error (gt - solver, in gt body
    # frame) = +2.0; lateral = 0; yaw = 0.
    gt = (5.0, 1.0, 0.0)
    solver = (3.0, 1.0, 0.0)
    d = decompose_error(gt, solver)
    assert d["along"] == __import__("pytest").approx(2.0, abs=1e-9)
    assert d["lateral"] == __import__("pytest").approx(0.0, abs=1e-9)
    assert d["yaw"] == __import__("pytest").approx(0.0, abs=1e-9)


def test_decompose_alongtrack_respects_heading():
    # Same 2 m gap but heading = +90 deg (north): the gap is now along +y, so it is still
    # 'along-track' in the body frame, lateral stays 0.
    gt = (1.0, 5.0, math.pi / 2)
    solver = (1.0, 3.0, math.pi / 2)
    d = decompose_error(gt, solver)
    assert d["along"] == __import__("pytest").approx(2.0, abs=1e-9)
    assert d["lateral"] == __import__("pytest").approx(0.0, abs=1e-9)


def test_decompose_lateral_error():
    # Heading +x, gt offset in +y from solver -> pure lateral.
    gt = (3.0, 1.5, 0.0)
    solver = (3.0, 1.0, 0.0)
    d = decompose_error(gt, solver)
    assert d["along"] == __import__("pytest").approx(0.0, abs=1e-9)
    assert d["lateral"] == __import__("pytest").approx(0.5, abs=1e-9)


def test_decompose_yaw_error_wraps():
    gt = (0.0, 0.0, 3.0)
    solver = (0.0, 0.0, -3.0)
    d = decompose_error(gt, solver)
    # 3 - (-3) = 6 rad -> wrapped to 6 - 2pi = -0.283
    assert d["yaw"] == __import__("pytest").approx(6.0 - 2 * math.pi, abs=1e-9)
```

- [ ] **Step 3: Run to verify it fails**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260723
set +u; source install/setup.bash 2>/dev/null
python3 -m pytest src/tugbot_maze/test/test_pose_decompose.py -q 2>&1 | tail -3
```
Expected: ImportError (no module `tugbot_maze.pose_decompose`).

- [ ] **Step 4: Implement the offline decomposer module**

Create `ros2_ws_tugbot_nav_20260723/src/tugbot_maze/tugbot_maze/pose_decompose.py`:

```python
"""Frame reconciliation + error decomposition for Task A POSEDIAG lines (offline, ROS-free).

The solver logs three raw poses per tick; this module turns them into along-track / lateral /
yaw error so the along-track-lag verdict is testable. See the localization-root-cause spec."""
from __future__ import annotations
import math
import re
from typing import Dict, Optional, Tuple

Pose2D = Tuple[float, float, float]

_STAMP = re.compile(r"\[(\d+\.\d+)\]")
_TRIP = re.compile(
    r"POSEDIAG gt=\(([-\d.]+), ([-\d.]+), ([-\d.]+)\) "
    r"odom=\(([-\d.]+), ([-\d.]+), ([-\d.]+)\) "
    r"solver=\(([-\d.]+), ([-\d.]+), ([-\d.]+)\)")


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def parse_posediag_line(line: str) -> Optional[Dict]:
    ms, mt = _STAMP.search(line), _TRIP.search(line)
    if not (ms and mt):
        return None
    g = [float(x) for x in mt.groups()]
    return {"t": float(ms.group(1)),
            "gt": (g[0], g[1], g[2]),
            "odom": (g[3], g[4], g[5]),
            "solver": (g[6], g[7], g[8])}


def decompose_error(gt: Pose2D, other: Pose2D) -> Dict[str, float]:
    """Error (gt - other) expressed in gt's body frame: +along = gt is ahead of `other` along
    gt's heading; +lateral = gt is to gt's left of `other`; yaw = wrapped(gt_yaw - other_yaw)."""
    dx, dy = gt[0] - other[0], gt[1] - other[1]
    c, s = math.cos(gt[2]), math.sin(gt[2])
    return {"along": c * dx + s * dy,
            "lateral": -s * dx + c * dy,
            "yaw": _wrap(gt[2] - other[2])}
```

- [ ] **Step 5: Run to verify it passes**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260723
set +u; source install/setup.bash 2>/dev/null
python3 -m pytest src/tugbot_maze/test/test_pose_decompose.py -q 2>&1 | tail -3
```
Expected: 5 passed.

- [ ] **Step 6: Add the report CLI to pose_decompose.py**

Append to `ros2_ws_tugbot_nav_20260723/tools/pose_decompose.py` (a thin CLI wrapping the module):

```python
#!/usr/bin/env python3
"""CLI: decompose Task A POSEDIAG lines in a run's launch.log into per-segment / per-axis error.

Usage: python3 tools/pose_decompose.py <artifact_dir> [<artifact_dir> ...]

Reports, per run: how many POSEDIAG lines; the (gt - odom) segment (does /odom itself drift?)
and the (odom - solver) segment (ICP quality), each decomposed into along / lateral / yaw with
p10/p50/p90 and sign consistency; and, within the (gt - solver) total error, the split between
dead-reckoning windows (ticks whose most recent MATCH was rejected) and accepted-correction
ticks. The verdict predicts (gt - odom) along ~ 0 and (odom - solver) along = +2..+6 m."""
import sys
from tugbot_maze.pose_decompose import parse_posediag_line, decompose_error


def _pctl(xs, q):
    if not xs:
        return float("nan")
    ys = sorted(xs)
    return ys[min(len(ys) - 1, int(q * len(ys)))]


def analyze(path):
    rows = []
    for line in open(path + "/launch.log", errors="replace"):
        r = parse_posediag_line(line)
        if r:
            rows.append(r)
    print("==== %s: %d POSEDIAG lines ====" % (path, len(rows)))
    if not rows:
        print("  (no POSEDIAG -- was the run launched with pose_diag:=true?)")
        return
    for seg_name, a, b in (("gt-odom", "gt", "odom"), ("odom-solver", "odom", "solver"),
                           ("gt-solver", "gt", "solver")):
        al = [decompose_error(r[a], r[b])["along"] for r in rows]
        la = [abs(decompose_error(r[a], r[b])["lateral"]) for r in rows]
        yw = [abs(decompose_error(r[a], r[b])["yaw"]) for r in rows]
        pos = sum(1 for v in al if v > 0)
        print("  %-11s along p10/p50/p90 = %+.2f/%+.2f/%+.2f  |lat| p50=%.2f  |yaw| p50=%.2f  "
              "along>0: %d/%d" % (seg_name, _pctl(al, 0.1), _pctl(al, 0.5), _pctl(al, 0.9),
                                  _pctl(la, 0.5), _pctl(yw, 0.5), pos, len(al)))


if __name__ == "__main__":
    for p in sys.argv[1:]:
        analyze(p)
```

- [ ] **Step 7: Add the default-off POSEDIAG logger to the solver**

In `flood_fill_solver.py` `__init__` (near the other `declare_parameter` calls, ~line 62), add:

```python
        self.pose_diag = bool(self.declare_parameter('pose_diag', False).value)
        self._gt_pose = None          # latest ground-truth world pose (eval only; NEVER used for control)
```

Add a subscription guarded by the flag (near the other `create_subscription` calls, ~line 100). Import `TFMessage` at the top with the other message imports (`from tf2_msgs.msg import TFMessage`):

```python
        if self.pose_diag:            # eval scaffold: ground-truth pose, logged only
            self.create_subscription(TFMessage, '/gt/dynamic_pose', self._gt_cb, 10)
```

Add the callback and the log emit. The callback (place with the other `_cb` methods):

```python
    def _gt_cb(self, msg):            # eval only -- stores ground truth for POSEDIAG, never for control
        for tr in msg.transforms:
            if 'anymal_c' in tr.child_frame_id or tr.child_frame_id == 'base_link':
                t, q = tr.transform.translation, tr.transform.rotation
                self._gt_pose = (t.x, t.y, quat_to_yaw(q.x, q.y, q.z, q.w))
                return
```

In `_control_tick`, right after `self._sm_corrected` is refreshed for the tick (i.e. after the `_lookup_pose()` result is in hand; place it next to the events-drain block ~line 351), add:

```python
            if self.pose_diag and self._gt_pose is not None and self._sm_corrected is not None:
                odom_base = self._lookup_tf(self.odom_frame, self.base_frame)
                odom_world = (compose_2d(self._entrance_anchor, odom_base)
                              if odom_base is not None else self._sm_corrected)
                self.get_logger().info(
                    'POSEDIAG gt=(%.3f, %.3f, %.3f) odom=(%.3f, %.3f, %.3f) '
                    'solver=(%.3f, %.3f, %.3f)' % (self._gt_pose + odom_world + self._sm_corrected))
```

NOTE: `quat_to_yaw` and `compose_2d` are already imported (`flood_fill_solver.py:28`). The odom-world pose reuses the entrance anchor exactly as the solver's own bootstrap does (`:176`), so `odom` and `solver` share the map frame and `gt` is world; pose_decompose treats each pair independently, and any fixed map<->world offset cancels in the along/lateral decomposition because it is applied to both members of the `odom-solver` pair. The `gt-odom` / `gt-solver` pairs carry the (fixed, small, entrance-calibrated) offset; the tool reports it and the analyst subtracts the t0 bias (documented in the tool's verdict step, Task 4).

- [ ] **Step 8: Build and verify default-off = zero POSEDIAG**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260723
set +u; source /opt/ros/jazzy/setup.bash; colcon build --symlink-install 2>&1 | tail -2
source install/setup.bash 2>/dev/null
python3 -m pytest src/tugbot_maze/test/test_pose_decompose.py -q 2>&1 | tail -2
```
Expected: build clean; 5 passed. (Runtime default-off behavior is verified in Task 4's first run by grepping POSEDIAG=0 on a `pose_diag:=false` run.)

- [ ] **Step 9: Commit**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git add ros2_ws_tugbot_nav_20260723/src/tugbot_gazebo/config/tugbot_bridge.yaml \
        ros2_ws_tugbot_nav_20260723/src/tugbot_maze/tugbot_maze/flood_fill_solver.py \
        ros2_ws_tugbot_nav_20260723/tools/pose_decompose.py \
        ros2_ws_tugbot_nav_20260723/src/tugbot_maze/tugbot_maze/pose_decompose.py \
        ros2_ws_tugbot_nav_20260723/src/tugbot_maze/test/test_pose_decompose.py
git commit -F - <<'EOF'
feat: Task A measurement scaffold - ground-truth bridge + default-off POSEDIAG logger + decomposer

Eval-only, default-off (pose_diag:=false). A ground-truth dynamic_pose bridge,
a dumb three-pose (gt/odom/solver) logger in the solver guarded by the flag,
and an offline pose_decompose tool that owns all frame reconciliation and
splits solver-vs-truth error into along-track / lateral / yaw. Ground truth
and /odom are logged only; nothing in the control or localization path reads
them.

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
EOF
```

---

### Task 3: Task B — believed-track edge-legality detector (ships, TDD)

**Files:**
- Modify: `ros2_ws_tugbot_nav_20260723/src/tugbot_maze/tugbot_maze/maze_motion.py` (`_track_cell` cell-change branch)
- Create: `ros2_ws_tugbot_nav_20260723/src/tugbot_maze/test/test_edge_legality.py`

Facts: `_track_cell` (~line 301) has a cell-change branch at ~line 314 (`if self.cell != self.last_seen_cell:` → sets `self.prev_cell`, appends to `self.recent`). `self.events` is the structured-event list the solver drains (`flood_fill_solver.py:351`). `_dir_name((dx, dy))` maps a unit step to `'N'/'S'/'E'/'W'` (already defined in maze_motion). `brain.is_wall(cell, dir)` exists. `self.committed` is the frozen-cell set.

- [ ] **Step 1: Write the failing tests**

Create `ros2_ws_tugbot_nav_20260723/src/tugbot_maze/test/test_edge_legality.py`:

```python
"""Task B: believed-track edge-legality detector. On each believed cell change, the edge just
crossed must not be a known wall; a violation means the pose belief has slipped across a wall
(the earliest observable symptom of the mislocalization episode). Pure observation -- it must
NOT change any routing decision."""
from tugbot_maze.maze_motion import MazeMotion


def test_legal_transition_emits_no_event():
    m = MazeMotion(); m.cell = (5, 5); m.last_seen_cell = (5, 5)
    m.cell = (6, 5)                       # move E; edge (5,5)-E is open by default (unknown)
    m._track_cell(1.0)
    assert not any(e.startswith('ILLEGAL_EDGE') for e in m.events)


def test_illegal_transition_across_known_wall_emits_event():
    m = MazeMotion(); m.cell = (5, 5); m.last_seen_cell = (5, 5)
    m.brain.mark((5, 5), 'E', is_wall=True)   # (5,5)-E is a KNOWN wall
    m.cell = (6, 5)                            # ...yet the belief crossed it
    m._track_cell(2.0)
    ev = [e for e in m.events if e.startswith('ILLEGAL_EDGE')]
    assert len(ev) == 1
    assert 'cell=(6, 5)' in ev[0] and 'prev=(5, 5)' in ev[0] and 'dir=W' in ev[0]


def test_illegal_event_reports_committed_state():
    m = MazeMotion(); m.cell = (5, 5); m.last_seen_cell = (5, 5)
    m.brain.mark((5, 5), 'N', is_wall=True)
    m.committed.add((5, 6))
    m.cell = (5, 6)                            # crossed (5,5)-N into a committed cell
    m._track_cell(3.0)
    ev = [e for e in m.events if e.startswith('ILLEGAL_EDGE')][0]
    assert 'committed=True' in ev


def test_non_adjacent_jump_does_not_crash_or_falsely_fire():
    # A re-anchor can teleport the cell by >1 (diagonal / multi-cell). No single edge exists;
    # the detector must skip (no direction) rather than fire or raise.
    m = MazeMotion(); m.cell = (5, 5); m.last_seen_cell = (5, 5)
    m.cell = (7, 8)
    m._track_cell(4.0)
    assert not any(e.startswith('ILLEGAL_EDGE') for e in m.events)


def test_detector_does_not_change_routing_state():
    # Observation only: prev_cell / last_seen_cell / recent bookkeeping is exactly as before.
    m = MazeMotion(); m.cell = (5, 5); m.last_seen_cell = (5, 5)
    m.brain.mark((5, 5), 'E', is_wall=True)
    m.cell = (6, 5)
    m._track_cell(5.0)
    assert m.prev_cell == (5, 5) and m.last_seen_cell == (6, 5)
```

- [ ] **Step 2: Run to verify RED**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260723
set +u; source install/setup.bash 2>/dev/null
python3 -m pytest src/tugbot_maze/test/test_edge_legality.py -q 2>&1 | tail -3
```
Expected: the 2 event-expecting tests fail (no ILLEGAL_EDGE emitted); the 3 negative/no-op tests pass.

- [ ] **Step 3: Implement**

In `maze_motion.py` `_track_cell`, inside the `if self.cell != self.last_seen_cell:` branch, BEFORE `self.prev_cell = self.last_seen_cell` is overwritten (so prev is still the pre-move cell), add the legality check. The current branch is:

```python
        if self.cell != self.last_seen_cell:             # cell-change bookkeeping
            self.prev_cell = self.last_seen_cell
            self.last_seen_cell = self.cell
            self.recent.append((t, self.cell))
```

Replace it with:

```python
        if self.cell != self.last_seen_cell:             # cell-change bookkeeping
            src = self.last_seen_cell
            dx, dy = self.cell[0] - src[0], self.cell[1] - src[1]
            if (dx, dy) in DIRS_INV:                      # adjacent step -> a single edge was crossed
                d = DIRS_INV[(dx, dy)]
                if self.brain.is_wall(src, d):            # believed track crossed a KNOWN wall
                    self.events.append(                   # DIAG: earliest mislocalization symptom (Task B)
                        "ILLEGAL_EDGE cell=%s prev=%s dir=%s committed=%s t=%.1f"
                        % (self.cell, src, OPP[d], src in self.committed, t))
            self.prev_cell = self.last_seen_cell
            self.last_seen_cell = self.cell
            self.recent.append((t, self.cell))
```

`DIRS_INV` maps a unit step to a direction name. If it does not already exist in maze_motion, add it near the top next to `_dir_name` (which does the same mapping via a dict literal). Reuse the existing mapping: add module-level `DIRS_INV = {(1, 0): 'E', (-1, 0): 'W', (0, 1): 'N', (0, -1): 'S'}` if absent. (`_dir_name` already contains this exact dict; you may lift it to a module constant and have `_dir_name` reference it to stay DRY.) The event reports `dir=` as `OPP[d]` — the direction of the crossed edge as seen FROM the destination cell (`cell`), matching the test's `dir=W` when moving E from (5,5) into (6,5). `OPP` is already imported.

- [ ] **Step 4: Run to verify GREEN**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260723
set +u; source install/setup.bash 2>/dev/null
python3 -m pytest src/tugbot_maze/test/test_edge_legality.py -q 2>&1 | tail -3
```
Expected: 5 passed.

- [ ] **Step 5: Regression — full suite baseline**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260723
set +u; source install/setup.bash 2>/dev/null
python3 -m pytest src/tugbot_maze/test -q 2>&1 | tee /tmp/loc_after_b.txt | tail -3
grep '^FAILED' /tmp/loc_after_b.txt | sort > /tmp/loc_after_b_failures.txt
diff /tmp/loc_baseline_failures.txt /tmp/loc_after_b_failures.txt && echo BASELINE-CLEAN
```
Expected: 7 failed (same names -> BASELINE-CLEAN) / 475 passed / 3 xfailed.

- [ ] **Step 6: Commit**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git add ros2_ws_tugbot_nav_20260723/src/tugbot_maze/tugbot_maze/maze_motion.py \
        ros2_ws_tugbot_nav_20260723/src/tugbot_maze/test/test_edge_legality.py
git commit -F - <<'EOF'
feat: believed-track edge-legality detector (Task B, ships) - map-poisoning leading indicator

On each believed cell change, if the crossed edge is a known wall the pose
belief has slipped across it -- the earliest observable symptom of the
mislocalization episode (fired at t+490 in run 110327, 3100 s before the
timeout). Emits ILLEGAL_EDGE with cell/prev/dir/committed; pure observation,
changes no routing decision. Zero ground truth. Full suite baseline unchanged.

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
EOF
```

---

### Task 4: Task A measurement campaign + fork decision (CONTROLLER CHECKPOINT)

**Files:** none (run artifacts under `$WS/log/`; verdict -> spec 附记). This task GATES Task 5.

- [ ] **Step 1: Generate diagnostic runs (pose_diag on)** — write a batch script (WRITE then EXECUTE in separate calls):

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260723
cat > /tmp/loc_diag_batch.sh <<'EOF'
#!/bin/bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260723
for i in 1 2 3 4; do
  MAX_SECONDS=3600 HEADLESS=true USE_RVIZ=false POSE_SOURCE=online_slam POSE_DIAG=true \
    bash tools/run_flood_fill_maze.sh 3600 true false online_slam
  echo "=== DIAG RUN $i DONE ==="
done
EOF
chmod +x /tmp/loc_diag_batch.sh
```

NOTE: `run_flood_fill_maze.sh` must pass `pose_diag:=true` to the launch when `POSE_DIAG=true`. If the run script does not forward it, FIRST add a one-line pass-through (env `POSE_DIAG` -> launch arg `pose_diag`) in `tools/run_flood_fill_maze.sh` and in `tugbot_maze_explore.launch.py` (declare `pose_diag` arg, thread to the solver node params). Commit that plumbing with message `chore: thread pose_diag launch arg to the solver (eval scaffold)`. Then EXECUTE `bash /tmp/loc_diag_batch.sh` in a separate background call. Aim for >=2 runs that carry the mislocalization episode (ILLEGAL_EDGE fires); if 4 runs are all clean, run more (the trap is ~1/3 stochastic).

- [ ] **Step 2: Verify default-off first** — on ONE run, confirm the red line: a `POSE_DIAG` unset run logs zero POSEDIAG lines.

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260723
bash tools/run_flood_fill_maze.sh 300 true false online_slam    # POSE_DIAG unset -> off
A=$(ls -dt log/flood_fill_run_* | head -1)
echo "POSEDIAG lines (must be 0): $(grep -c POSEDIAG $A/launch.log)"
```
Expected: 0. If non-zero, the flag default is wrong — STOP/BLOCKED.

- [ ] **Step 3: Decompose + correlate with ILLEGAL_EDGE**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260723
for A in $(ls -dt log/flood_fill_run_* | head -6); do
  echo "== $A: result=$(grep -o 'result=[A-Z_]*' $A/result.txt 2>/dev/null) ILLEGAL_EDGE=$(grep -c ILLEGAL_EDGE $A/launch.log) =="
  python3 tools/pose_decompose.py $A
done
```

- [ ] **Step 4: Judge the fork (write the verdict to 附记, then CHECKPOINT the controller)**

Read the decomposition. Decide:
- **ICP-dominant** (`gt-odom` along ~0; `odom-solver` along large & positive, matching +2..+6 m): proceed to Task 5. Within ICP, note which sub-mechanism dominates — dead-reckoning windows (error grows during rejected-MATCH streaks) vs bad accepted corrections (error jumps on an accepted high-rms tick) — this selects Task 5's first lever.
- **odom-dominant** (`gt-odom` along large): the verdict is REFUTED; STOP. Do NOT start Task 5. Report to the controller for a re-scoped odom sub-plan (spec C2).
- **all-small** (no segment explains the error): verdict REFUTED differently; STOP, report for raw-scan capture (spec C3).

Append the verdict as 附记 item 1 (decomposition numbers per segment/axis, the ILLEGAL_EDGE correlation, and the fork taken). Commit:

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git add docs/superpowers/specs/2026-07-20-localization-root-cause-design.md
git commit -F - <<'EOF'
docs: Task A verdict - three-pose decomposition and the fork taken

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
EOF
```

**STOP HERE and report the verdict + chosen fork to the controller before Task 5.** Tasks 5-6 below assume the ICP fork; if the fork is odom or all-small, the controller re-plans.

---

### Task 5: Task C (ICP fork) — `_has_local_interior` nearest-distance fix (TDD)

**Files:**
- Modify: `ros2_ws_tugbot_nav_20260723/src/tugbot_maze/tugbot_maze/online_scan_match_localizer.py` (`_has_local_interior`, ~line 316)
- Create: `ros2_ws_tugbot_nav_20260723/src/tugbot_maze/test/test_local_interior_gate.py`

Current `_has_local_interior` tests only 3 sampled points per segment (both endpoints + midpoint) against `_local_radius=1.0`. On a corridor centreline the nearest point of a wall segment is a foot of perpendicular that is NOT one of those 3 points, so the gate reads "no local interior" and the ICP is switched off exactly where the robot drives. Fix: test the true point-to-segment distance.

- [ ] **Step 1: Write the failing tests**

Create `ros2_ws_tugbot_nav_20260723/src/tugbot_maze/test/test_local_interior_gate.py`:

```python
"""The locality gate must use true point-to-segment distance, not a 3-sample proxy. On a corridor
centreline the nearest wall point is a perpendicular foot, missed by endpoint/midpoint sampling,
which switched the ICP off exactly along the driven path (localization-root-cause verdict)."""
from tugbot_maze.online_scan_match_localizer import OnlineScanMatchLocalizer


def _loc():
    return OnlineScanMatchLocalizer()


def test_centreline_pose_sees_the_side_wall():
    # A 4 m wall segment running E-W at y=1.0 from x=0 to x=4. Robot near the wall end on the
    # corridor floor at (0.7, 0.1): the perpendicular foot is (0.7, 1.0) at distance 0.9 < 1.0,
    # but the three SAMPLED points are all > 1.0 away -- endpoint (0,1) = sqrt(0.49+0.81) = 1.14,
    # endpoint (4,1) = 3.5, midpoint (2,1) = sqrt(1.69+0.81) = 1.58. Old 3-sample proxy: False;
    # true point-to-segment distance: True.
    seg = (0.0, 1.0, 4.0, 1.0)
    loc = _loc()
    assert loc._has_local_interior((0.7, 0.1, 0.0), [seg]) is True


def test_far_pose_still_gated_off():
    seg = (0.0, 1.0, 4.0, 1.0)
    loc = _loc()
    assert loc._has_local_interior((2.0, -3.0, 0.0), [seg]) is False   # 4 m away, no local ref


def test_endpoint_still_counts():
    seg = (0.0, 1.0, 4.0, 1.0)
    loc = _loc()
    assert loc._has_local_interior((0.2, 1.2, 0.0), [seg]) is True     # 0.28 m from endpoint (0,1)


def test_no_segments_is_false():
    assert _loc()._has_local_interior((0.0, 0.0, 0.0), []) is False
```

- [ ] **Step 2: Run to verify RED**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260723
set +u; source install/setup.bash 2>/dev/null
python3 -m pytest src/tugbot_maze/test/test_local_interior_gate.py -q 2>&1 | tail -3
```
Expected: `test_centreline_pose_sees_the_side_wall` FAILS (old 3-sample code returns False); the other 3 pass.

- [ ] **Step 3: Implement point-to-segment distance**

Replace `_has_local_interior` in `online_scan_match_localizer.py` with:

```python
    def _has_local_interior(self, prior_pose, interior_segments) -> bool:
        """Return True iff at least one committed interior segment passes within local_radius of
        the prior pose, by TRUE point-to-segment distance (not a 3-sample endpoint/midpoint proxy
        -- that proxy read False on corridor centrelines, where the nearest wall point is a
        perpendicular foot, and switched the ICP off exactly along the driven path)."""
        px, py = float(prior_pose[0]), float(prior_pose[1])
        r2 = self._local_radius * self._local_radius
        for seg in interior_segments:
            x0, y0, x1, y1 = seg
            vx, vy = x1 - x0, y1 - y0
            wx, wy = px - x0, py - y0
            vv = vx * vx + vy * vy
            u = 0.0 if vv == 0.0 else max(0.0, min(1.0, (wx * vx + wy * vy) / vv))
            fx, fy = x0 + u * vx, y0 + u * vy       # nearest point on the segment (foot / endpoint)
            if (fx - px) * (fx - px) + (fy - py) * (fy - py) <= r2:
                return True
        return False
```

- [ ] **Step 4: Run to verify GREEN**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260723
set +u; source install/setup.bash 2>/dev/null
python3 -m pytest src/tugbot_maze/test/test_local_interior_gate.py -q 2>&1 | tail -3
```
Expected: 4 passed.

- [ ] **Step 5: Regression — full suite baseline**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260723
set +u; source install/setup.bash 2>/dev/null
python3 -m pytest src/tugbot_maze/test -q 2>&1 | tee /tmp/loc_after_c1.txt | tail -3
grep '^FAILED' /tmp/loc_after_c1.txt | sort > /tmp/loc_after_c1_failures.txt
diff /tmp/loc_baseline_failures.txt /tmp/loc_after_c1_failures.txt && echo BASELINE-CLEAN
```
Expected: BASELINE-CLEAN (same 7 names). NOTE: the offline localization sims (`test_online_slam_sim`) run through this gate — if any NEW failure appears, the gate change altered offline solve behavior; investigate before proceeding (do NOT weaken the tests).

- [ ] **Step 6: Commit**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git add ros2_ws_tugbot_nav_20260723/src/tugbot_maze/tugbot_maze/online_scan_match_localizer.py \
        ros2_ws_tugbot_nav_20260723/src/tugbot_maze/test/test_local_interior_gate.py
git commit -F - <<'EOF'
feat: locality gate uses true point-to-segment distance (Task C, ICP fork)

The 3-sample endpoint/midpoint proxy read False on corridor centrelines --
where the nearest wall point is a perpendicular foot -- switching the ICP off
exactly along the path the centring controller drives, which is the
dead-reckoning window that seeds the along-track slip. Now the gate uses the
exact point-to-segment distance. Full suite baseline unchanged.

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
EOF
```

---

### Task 6: Task C (ICP fork) — sequential `residual_rms` ceiling IF the indicator is not yet zeroed

**Files:**
- Modify: `ros2_ws_tugbot_nav_20260723/src/tugbot_maze/tugbot_maze/scan_match_localizer.py` (accept path, ~line 143-170)
- Create: `ros2_ws_tugbot_nav_20260723/src/tugbot_maze/test/test_rms_ceiling.py`

**GATE:** Only do this task if, after Task 5, a short headless A/B check (Task 7 method, ~3 runs) shows the ILLEGAL_EDGE indicator is NOT yet zeroed. If Task 5 alone zeroes it, SKIP Task 6 and record "single-lever sufficient" in 附记. (Sequential-lever discipline — spec Task C.)

- [ ] **Step 1: Write the failing tests**

Create `ros2_ws_tugbot_nav_20260723/src/tugbot_maze/test/test_rms_ceiling.py`:

```python
"""A 3-DOF ICP correction whose residual_rms exceeds a ceiling is rejected (returns the prior),
symmetric with the 1-DOF YAW_RMS_CEILING. The worst accepted corrections in the trap runs
(rms 0.157/0.169) landed exactly on the poisoning junctions while the accepted p90 was 0.089."""
from tugbot_maze.scan_match_localizer import ScanMatchLocalizer, RMS_CEILING_3DOF


def test_ceiling_constant_between_p90_and_trap():
    # Must reject the trap-onset corrections (>=0.157) while never touching the accepted p90 (0.089).
    assert 0.089 < RMS_CEILING_3DOF < 0.157


def test_high_rms_converged_pose_is_rejected(monkeypatch):
    loc = ScanMatchLocalizer()
    prior = (1.0, 2.0, 0.3)
    # Force the inner solve to converge to a pose within the translation clamp but with a high rms.
    def fake_solve_one(pose, pts, max_corr):
        return (0.05, 0.05, 0.0), {"n_inliers": 500, "residual_rms": 0.30,
                                   "eig_min": 200.0, "eig_max": 400.0, "under_inliers": False}
    monkeypatch.setattr(loc, "_solve_one", fake_solve_one)
    monkeypatch.setattr(loc, "_beams_to_points", lambda *a, **k: [(0.0, 0.0)])
    est, info = loc.correct(prior, [1.0], -3.14, 0.01)
    assert info["rejected"] is True
    assert est == prior                       # high-rms correction discarded -> prior returned


def test_low_rms_correction_is_accepted(monkeypatch):
    loc = ScanMatchLocalizer()
    prior = (1.0, 2.0, 0.3)
    def fake_solve_one(pose, pts, max_corr):
        return (0.02, 0.0, 0.0), {"n_inliers": 500, "residual_rms": 0.05,
                                  "eig_min": 200.0, "eig_max": 400.0, "under_inliers": False}
    monkeypatch.setattr(loc, "_solve_one", fake_solve_one)
    monkeypatch.setattr(loc, "_beams_to_points", lambda *a, **k: [(0.0, 0.0)])
    est, info = loc.correct(prior, [1.0], -3.14, 0.01)
    assert info["rejected"] is False
    assert est != prior                       # good correction applied
```

- [ ] **Step 2: Run to verify RED**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260723
set +u; source install/setup.bash 2>/dev/null
python3 -m pytest src/tugbot_maze/test/test_rms_ceiling.py -q 2>&1 | tail -3
```
Expected: ImportError (`RMS_CEILING_3DOF` undefined) or reject assertion fails.

- [ ] **Step 3: Implement**

In `scan_match_localizer.py`, add a module constant near the top:

```python
RMS_CEILING_3DOF = 0.12   # reject a converged 3-DOF correction fitting worse than this (localization-root-cause);
                          # between accepted p90 0.089 and trap-onset 0.157/0.169. Symmetric with YAW_RMS_CEILING.
```

In `correct()`, at the post-loop clamp check (~line 163, currently `if hypot(...) > trans_clamp_m or abs(...) > yaw_clamp_rad:`), add the rms ceiling as an additional reject condition. Change:

```python
        if (math.hypot(pose[0] - x0, pose[1] - y0) > self.trans_clamp_m
                or abs(_wrap(pose[2] - t0)) > self.yaw_clamp_rad):
```
to:

```python
        rms = info.get("residual_rms", float("nan"))
        if (math.hypot(pose[0] - x0, pose[1] - y0) > self.trans_clamp_m
                or abs(_wrap(pose[2] - t0)) > self.yaw_clamp_rad
                or (rms == rms and rms > RMS_CEILING_3DOF)):   # rms==rms: not NaN
```
The `conv_pose` diagnostic line and `rejected=True` / `return (x0,y0,t0), info` below stay unchanged (a high-rms reject also carries `conv_pose`, which is correct — the clamp-lock escape may still consider it).

- [ ] **Step 4: Run to verify GREEN**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260723
set +u; source install/setup.bash 2>/dev/null
python3 -m pytest src/tugbot_maze/test/test_rms_ceiling.py -q 2>&1 | tail -3
```
Expected: 3 passed.

- [ ] **Step 5: Regression — full suite baseline** (the ceiling changes ICP accept behavior; watch the offline sims closely)

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260723
set +u; source install/setup.bash 2>/dev/null
python3 -m pytest src/tugbot_maze/test -q 2>&1 | tee /tmp/loc_after_c2.txt | tail -3
grep '^FAILED' /tmp/loc_after_c2.txt | sort > /tmp/loc_after_c2_failures.txt
diff /tmp/loc_baseline_failures.txt /tmp/loc_after_c2_failures.txt && echo BASELINE-CLEAN
```
Expected: BASELINE-CLEAN. If `test_online_slam_sim` or `test_scan_match_localizer` newly fail, the ceiling is too aggressive (rejecting good corrections) — investigate the value against real accepted-rms data before weakening anything; a value tweak within [0.089, 0.157] is authorized, weakening a test is not.

- [ ] **Step 6: Commit**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git add ros2_ws_tugbot_nav_20260723/src/tugbot_maze/tugbot_maze/scan_match_localizer.py \
        ros2_ws_tugbot_nav_20260723/src/tugbot_maze/test/test_rms_ceiling.py
git commit -F - <<'EOF'
feat: 3-DOF ICP residual_rms ceiling (Task C, sequential second lever)

A converged correction fitting worse than RMS_CEILING_3DOF (0.12, between the
accepted p90 of 0.089 and the trap-onset 0.157/0.169) is rejected and the
prior is kept -- symmetric with the existing 1-DOF YAW_RMS_CEILING. The two
worst accepted corrections of the trap runs landed exactly on the poisoning
junctions; this declines them. Added only because the Task 5 A/B check showed
the poisoning indicator was not yet zeroed. Full suite baseline unchanged.

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
EOF
```

---

### Task 7: Statistical gate — poisoning indicator primary, EXIT-rate secondary (A/B)

**Files:** none (artifacts under `$WS/log/`; findings -> 附记).

- [ ] **Step 1: A/B batches** — run the amended stack headless (pose_diag OFF; the ILLEGAL_EDGE indicator is on unconditionally since Task B ships) and compare against the 20260722 predecessor's known runs. Write a batch script (WRITE then EXECUTE separately), 3+3+2 like prior phases, stop-batch-on-anomaly:

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260723
cat > /tmp/loc_gate_batch.sh <<'EOF'
#!/bin/bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260723
for i in 1 2 3; do
  bash tools/run_flood_fill_maze.sh 3600 true false online_slam
  echo "=== GATE RUN $i DONE ==="
done
EOF
chmod +x /tmp/loc_gate_batch.sh
```
Then EXECUTE in a separate background call.

- [ ] **Step 2: Judge each run**
Per run: result; `python3 tools/replay_collision_oracle.py <artifact>` (official + live_rate); **primary indicator**: `grep -c ILLEGAL_EDGE <artifact>/launch.log` and the timestamp of the first ILLEGAL_EDGE (`grep -m1 ILLEGAL_EDGE ... | grep -o 't=[0-9.]*'`). Compare against the 20260722 predecessor runs decomposed with the same detector (re-run the detector's logic offline on their logs is not possible — the detector is new; instead compare against the FORECAST from forensics: pre-fix runs poison at t~+400-490 with tens of ILLEGAL_EDGE-equivalent illegal transitions). The gate: **first ILLEGAL_EDGE time significantly later / per-run count significantly lower (ideally zero)** on the amended stack.

- [ ] **Step 3: Batches 2-3 + verdict** — same pattern; stop-on-anomaly with forensics first. Gate = poisoning indicator improved (primary) AND EXIT rate not below 4/8 domain (secondary) AND oracle <= 0.719%/run. Append the per-run table + verdict as the next 附记 item; commit (docs only).

---

### Task 8: GUI + RViz acceptance (USER-GATED — do not launch yourself)

- [ ] **Step 1:** Report the Task 7 tally and ASK THE USER to trigger:
`cd ros2_ws_tugbot_nav_20260723 && bash tools/run_flood_fill_maze.sh 3600 false true online_slam`
- [ ] **Step 2:** Judge: EXIT + oracle <= 0.719% + ILLEGAL_EDGE count + live_rate; user confirms visuals — SPECIFICALLY whether the accumulated radar map now stays registered with the live point cloud (the drift the user caught last phase is the direct visual pass/fail for this fix). Record as a 附记 item (committed with Task 9 docs).

---

### Task 9: Docs + final review + merge

**Files:**
- Modify: `ros2_ws_tugbot_nav_20260723/README.md`
- Modify: `docs/superpowers/specs/2026-07-20-localization-root-cause-design.md` (final 附记)

- [ ] **Step 1: README** — rename this-workspace `ros2_ws_tugbot_nav_20260722` references to `...20260723` (title + run commands; grep, keep historical mentions); add a localization-fix subsection describing: the measure-first method + Task A verdict, the locality-gate point-to-segment fix, the rms ceiling (if shipped), the ILLEGAL_EDGE indicator, and the HONEST result (indicator improvement; EXIT-rate framing). Append the final 附记 (GUI result + disposition). Commit both.

- [ ] **Step 2: Final whole-branch review** (fable subagent): diff `main...localization-root-cause`; UNFREEZE-BOUNDARY AUDIT — within `tugbot_maze/tugbot_maze/` only `online_scan_match_localizer.py` (`_has_local_interior`), `scan_match_localizer.py` (rms ceiling), `maze_motion.py` (Task B detector), `flood_fill_solver.py` (default-off POSEDIAG), `pose_decompose.py` (new) may differ; plus the bridge yaml, launch pose_diag arg, run script pass-through, README, tools/pose_decompose.py, and the four new test files. **RED-LINE AUDIT (weighted):** verify ground truth / `/odom` are read ONLY inside the `pose_diag`-guarded logger and NEVER by `correct()` / the controller / any pose input. Constants vs spec; suite gate re-run with baseline names.

- [ ] **Step 3: Finishing flow** (superpowers:finishing-a-development-branch): full suite on branch (baseline names) -> `git merge --no-ff localization-root-cause -F <tempfile>` -> full suite on main -> `git push origin main` -> `git branch -d localization-root-cause` -> memory file (`localization-root-cause-solved.md`, cross-link [[localization-alongtrack-root-cause]] and [[p3-routing-fix-solved]]) + MEMORY.md index.

---

## Self-Review (done at authoring)

- **Spec coverage:** workspace/branch (T1); Task A scaffold — bridge + default-off logger + decomposer + tests (T2) and the measurement campaign + fork checkpoint (T4); Task B detector ships (T3); red line enforced (T2 default-off, T4 Step-2 verify, T9 red-line audit); Task C ICP fork — `_has_local_interior` (T5) + sequential rms ceiling gated on the indicator (T6); C2 odom / C3 all-small handled as explicit STOP+re-plan at the T4 checkpoint; gate poisoning-primary + EXIT-secondary (T7); GUI with the radar-map-registration visual pass/fail (T8); docs/review/merge with red-line audit (T9). No gaps.
- **Fork honesty:** T4 is a hard controller checkpoint; T5/T6 are written for the EXPECTED (ICP) branch only, and the plan says so — if the fork is odom/all-small the controller re-plans rather than executing T5/T6.
- **Sequential-lever discipline:** T6 is explicitly gated on a post-T5 A/B showing the indicator not yet zeroed; if T5 suffices, T6 is skipped and recorded.
- **Placeholder scan:** none — every code step carries full code; the two decomposition/detector modules and all four test files are complete.
- **Type consistency:** `pose_diag` / `_gt_pose` / `_gt_cb` / `parse_posediag_line` / `decompose_error` / `ILLEGAL_EDGE` / `DIRS_INV` / `RMS_CEILING_3DOF` used identically across tasks and tests. Fixture geometry in T5 re-checked after catching a bug (the original (0.4,0.1) put endpoint (0,1) at 0.985<1.0, so the old proxy would also pass and the RED test would not be red); corrected to (0.7,0.1): perpendicular foot (0.7,1.0) dist 0.9<1.0, endpoints 1.14/3.5 and midpoint 1.58 all >1.0 -> old proxy False, new True.
