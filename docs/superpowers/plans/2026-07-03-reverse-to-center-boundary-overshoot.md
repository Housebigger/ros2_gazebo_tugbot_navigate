# Reverse-to-center for along-heading overshoot — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Eliminate the residual ~0.33 % rear-gripper graze at cell (3,9) by making `centering_command` null an along-heading overshoot with a **reverse translation** instead of a ~180° in-place rotation, without regressing the 16/16 autonomous completion.

**Architecture:** Single-function change in the ROS-free `hop_controller.centering_command`. When the axis to correct is anti-parallel to the current heading (cell centre is *behind*), drive in reverse along the heading; keep the existing forward-drive (aligned) and rotate-to-face (perpendicular/lateral) branches unchanged. Reverse is inherently safe: it only fires on an overshoot in the travel direction, so it retreats into the just-traversed, clear corridor.

**Tech Stack:** Python 3, pytest, ROS-free (`tugbot_maze` package). Validation via the offline true-footprint `maze_sim` oracle and a controlled Gazebo batch.

**Design:** `docs/superpowers/specs/2026-07-03-reverse-to-center-boundary-overshoot-design.md`

**Branch:** create `reverse-to-center-overshoot` off `main` before Task 1 (do NOT implement on `main`):
```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git checkout -b reverse-to-center-overshoot
```

---

## File Structure

- `src/tugbot_maze/tugbot_maze/hop_controller.py` — MODIFY `centering_command`; add module constant `_PARALLEL_COS`. The only behavioral change: anti-parallel heading → reverse.
- `src/tugbot_maze/test/test_hop_controller.py` — ADD a reverse unit test; UPDATE the one existing test (`test_centering_faces_axis_cardinal_before_driving`, lines 117–120) that encoded the old 180°-rotate for the now-reversing configuration.
- `src/tugbot_maze/test/test_maze_motion_sim.py` — end-to-end regression: drift=0/0.03 must stay collision-free; re-evaluate the drift=0.05 `xfail(strict=True)` cases (the fix may flip them to passing).
- `tools/replay_collision_oracle.py` — ADD: committed offline oracle replay over run-dir DIAG samples (promotes the ad-hoc `/tmp/replay_graze_diag.py`), for reproducible Gazebo acceptance.

---

## Task 1: Reverse-to-center in `centering_command`

**Files:**
- Modify: `src/tugbot_maze/tugbot_maze/hop_controller.py:19-53`
- Test: `src/tugbot_maze/test/test_hop_controller.py:117-120` (update) + new test

- [ ] **Step 1: Write the failing test (new reverse case) and update the one stale test**

In `src/tugbot_maze/test/test_hop_controller.py`, REPLACE the existing test (lines 117–120):

```python
def test_centering_faces_axis_cardinal_before_driving():
    # 0.3 m EAST of centre (ox>0) while facing east (yaw 0): must turn to face WEST (pi) first
    v, w, done = centering_command((2.3, 2.0, 0.0), 0.3, None, tol=0.12, yaw_tol=0.10)
    assert done is False and v == 0.0 and abs(w) > 0.0      # turning in place, not driving
```

with these two tests (the configuration now reverses instead of rotating ~180°, and add the (3,9)-style case):

```python
def test_centering_reverses_instead_of_turning_180():
    # 0.3 m EAST of centre (ox>0) while facing EAST (yaw 0): the centre is directly BEHIND.
    # Reverse-translate west; do NOT rotate ~180deg to face west (that off-centre rotation is
    # what swept the rear gripper into the wall).
    v, w, done = centering_command((2.3, 2.0, 0.0), 0.3, None, tol=0.12, yaw_tol=0.10)
    assert done is False and v < 0.0 and abs(w) < 1e-6


def test_centering_reverses_north_overshoot_at_boundary():
    # (3,9)-style arrival overshoot: 0.4 m NORTH of centre while still facing NORTH (+pi/2).
    # Centre is behind along the heading -> reverse south (retreats into the cleared corridor),
    # not a 180deg in-place rotation.
    v, w, done = centering_command((6.04, 18.40, math.pi / 2), None, 0.40, tol=0.10, yaw_tol=0.10)
    assert done is False and v < 0.0 and abs(w) < 1e-6
```

- [ ] **Step 2: Run the tests to verify they FAIL**

Run: `cd src/tugbot_maze && python3 -m pytest test/test_hop_controller.py -q`
Expected: the two new tests FAIL — the current code rotates (`v == 0.0, abs(w) > 0`) for these anti-parallel configurations, so `v < 0.0` is not met.

- [ ] **Step 3: Implement the reverse branch**

In `src/tugbot_maze/tugbot_maze/hop_controller.py`, after `_norm` (line 19-20) add the module constant:

```python
_PARALLEL_COS = math.cos(math.radians(45.0))   # |heading·axis| >= this -> heading ~parallel to the axis
```

Then REPLACE the body of `centering_command` (current lines 36-53, from `cands = []` to the final `return`) with:

```python
    cands = []
    if ox is not None and abs(ox) > tol:
        cands.append(('x', ox))
    if oy is not None and abs(oy) > tol:
        cands.append(('y', oy))
    if not cands:
        return (0.0, 0.0, True)
    axis, off = max(cands, key=lambda c: abs(c[1]))
    # Unit vector of the motion that reduces `off` along the correction axis (toward the centre).
    if axis == 'x':
        dx, dy = (-1.0 if off > 0 else 1.0), 0.0
    else:
        dx, dy = 0.0, (-1.0 if off > 0 else 1.0)
    yaw = pose[2]
    a = math.cos(yaw) * dx + math.sin(yaw) * dy      # +1 centre ahead, -1 centre behind, 0 perpendicular
    speed = min(v_max, max(v_min, kp_lin * abs(off)))
    # Anti-parallel: the centre is behind along the heading (an along-travel overshoot). Reverse-
    # translate to null it -- retreats into the just-traversed, clear corridor -- instead of the
    # ~180deg in-place rotation that sweeps the asymmetric rear gripper into a near wall (the (3,9)
    # graze). A diff-drive robot cannot strafe, but it can reverse.
    if a <= -_PARALLEL_COS:
        return (-speed, 0.0, False)
    # Perpendicular / not-yet-aligned: face the axis cardinal first, then drive forward (unchanged).
    want = math.atan2(dy, dx)
    dyaw = _norm(want - yaw)
    if abs(dyaw) > yaw_tol:
        w = kp_ang * dyaw - kd_ang * yaw_rate                         # PD: damp latency overshoot
        return (0.0, max(-w_max, min(w_max, w)), False)
    return (speed, 0.0, False)                                        # aligned -> drive forward to null
```

Also update the docstring (lines 29-35) to describe the reverse behavior:

```python
    """Re-center the robot to the current cell centre one cardinal axis at a time.

    (ox, oy) is the robot's offset from the cell centre in MAP axes (+x=E, +y=N), as
    returned by cell_center_offset (a component is None for an open axis with no wall to
    reference). Returns (v, w, done). Corrects the larger out-of-tol axis by driving ALONG
    that axis: forward when the centre is ahead, and in REVERSE when the centre is behind
    (an along-heading overshoot) -- reversing retreats into the just-traversed, clear
    corridor and avoids the ~180deg in-place rotation that sweeps the asymmetric rear
    gripper into a near wall. Rotate-to-face is used only when the axis is perpendicular to
    the heading (lateral centering). done=True when every referenced axis is within tol."""
```

- [ ] **Step 4: Run the tests to verify they PASS (and no regression in the file)**

Run: `cd src/tugbot_maze && python3 -m pytest test/test_hop_controller.py -q`
Expected: PASS — including the unchanged cases `test_centering_drives_forward_once_aligned`, `test_centering_y_axis_faces_north_south`, `test_centering_corrects_larger_axis_first` (perpendicular → still rotates), and `test_centering_within_envelope`.

- [ ] **Step 5: Commit**

```bash
git add src/tugbot_maze/tugbot_maze/hop_controller.py src/tugbot_maze/test/test_hop_controller.py
git commit -m "$(cat <<'EOF'
feat: reverse-to-center for along-heading overshoot (kill (3,9) graze)

centering_command now nulls an along-heading overshoot by reversing along
the current heading instead of rotating ~180deg to face the reduce
direction. The 180deg in-place rotation, done while ~0.4m off-centre toward
the north perimeter wall at (3,9), swept the asymmetric rear gripper through
the wall (the 0.33% residual). Reverse retreats into the just-traversed,
clear corridor; perpendicular lateral centering is unchanged.

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 2: End-to-end offline regression (true-footprint sim)

**Files:**
- Test: `src/tugbot_maze/test/test_maze_motion_sim.py:57-69`

**Context:** `_run(drift, latency)` drives `MazeMotion` through the inertia+collision `maze_sim` and reports `collided` (true-footprint `sim.collides`). The clean cases (drift=0.0, 0.03) assert **no collision**; the drift=0.05 cases are `xfail(strict=True)` ("genuine true-footprint collision the circle oracle hid"). This task confirms the fix does not regress the clean solves and re-evaluates the drift=0.05 grazes.

- [ ] **Step 1: Run the end-to-end suite on the fixed code**

Run: `cd src/tugbot_maze && python3 -m pytest test/test_maze_motion_sim.py -q`
Expected: drift=0.0 and drift=0.03 PASS (reaches exit, `not collided`, desync ≤ 1, esc == 0). Record the drift=0.05 outcome (XFAIL = still collides, or XPASS = now collision-free).

- [ ] **Step 2: Handle the drift=0.05 result**

- If the drift=0.05 cases are now **collision-free** (pytest reports XPASS, which FAILS under `strict=True`): the reverse-to-center fix eliminated them offline. REMOVE the `xfail` markers so they assert no collision — replace the three `pytest.param(...)` lines (58-62 region) so the parametrize list reads:

```python
@pytest.mark.parametrize("drift,latency", [
    (0.0, 0),
    (0.03, 0),
    (0.05, 0),
    (0.05, 2),
    (0.05, 3),
])
```

  Re-run `python3 -m pytest test/test_maze_motion_sim.py -q` → expect all PASS.

- If drift=0.05 **still collides** (XFAIL holds): leave the markers as-is (the residual there is a different mechanism — e.g. a backout graze — out of this plan's scope) and update the three `reason=` strings to note it was re-checked after the reverse-to-center fix and persists. Do NOT weaken the drift=0/0.03 assertions.

- [ ] **Step 3: Run the full offline suite (regression guard)**

Run:
```bash
cd src/tugbot_maze && python3 -m pytest \
  test/test_hop_controller.py test/test_maze_motion_sim.py test/test_maze_sim.py \
  test/test_flood_fill_brain.py test/test_cell_walls.py test/test_scan_match_localizer.py \
  test/test_pose_tracking.py test/test_wall_localize.py -q
```
Expected: all green (no regressions). If any previously-passing test fails, STOP and report.

- [ ] **Step 4: Commit**

```bash
git add src/tugbot_maze/test/test_maze_motion_sim.py
git commit -m "$(cat <<'EOF'
test: re-evaluate drift=0.05 offline grazes after reverse-to-center

Confirms the clean drift=0/0.03 solves stay collision-free and records the
drift=0.05 true-footprint outcome after the centering fix.

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 3: Committed offline collision-oracle replay tool

**Files:**
- Create: `tools/replay_collision_oracle.py`

**Context:** The 0.33 % baseline was computed ad-hoc. Promote it to a committed tool so Gazebo acceptance is reproducible: it replays run-dir DIAG samples through `maze_sim.collides` and reports the collision rate + per-cell/per-phase graze geometry.

- [ ] **Step 1: Create the tool**

Create `tools/replay_collision_oracle.py`:

```python
#!/usr/bin/env python3
"""Offline true-footprint collision-oracle replay over Gazebo DIAG samples.

Replays the (x,y,yaw) DIAG poses logged by flood_fill_solver through the true asymmetric
footprint oracle (maze_sim.collides) and reports the collision rate plus the geometry of
every grazing sample. Gazebo does NOT log physical collisions; this is how the collision
rate is measured. Usage:

    python3 tools/replay_collision_oracle.py <run_dir_or_batch_dir> [more ...]

A batch dir is one containing manifest.txt (one run dir per line); a run dir is one
containing launch.log. Reproduces the documented (3,9) 0.33% baseline on the clean batches.
"""
import argparse
import math
import os
import re
import sys
from collections import Counter

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "src", "tugbot_maze"))
from tugbot_maze.maze_sim import MazeSim, load_segments          # noqa: E402
from tugbot_maze.flood_fill_brain import pose_to_cell, cell_center  # noqa: E402

DIAG_RE = re.compile(
    r"DIAG pose=\(([-\d.eE+]+), ([-\d.eE+]+), ([-\d.eE+]+)\) "
    r"dcell=\((\d+), (\d+)\) odomcell=\((\d+), (\d+)\) "
    r"dist_to_exit=([-\d.]+) phase=(\S+)"
)


def run_logs_from(path):
    """Expand a batch dir (has manifest.txt) or a run dir (has launch.log) to launch.log paths."""
    mani = os.path.join(path, "manifest.txt")
    if os.path.exists(mani):
        out = []
        with open(mani) as f:
            for line in f:
                d = line.strip()
                if d and os.path.exists(os.path.join(d, "launch.log")):
                    out.append(os.path.join(d, "launch.log"))
        return out
    lg = os.path.join(path, "launch.log")
    return [lg] if os.path.exists(lg) else []


def main(argv):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("paths", nargs="+", help="run dirs and/or batch dirs")
    args = ap.parse_args(argv)

    sim = MazeSim(load_segments(), (0.0, 0.0), 0.0)
    logs = []
    for p in args.paths:
        logs.extend(run_logs_from(p))
    if not logs:
        print("no launch.log found under the given paths", file=sys.stderr)
        return 2

    total = coll = 0
    grazes = []
    for lg in logs:
        with open(lg, errors="ignore") as f:
            for line in f:
                m = DIAG_RE.search(line)
                if not m:
                    continue
                x, y, yaw = float(m.group(1)), float(m.group(2)), float(m.group(3))
                phase = m.group(9)
                total += 1
                if sim.collides(x, y, yaw):
                    coll += 1
                    cell = pose_to_cell(x, y)
                    cx, cy = cell_center(cell)
                    grazes.append((cell, x, y, math.degrees(yaw), phase, x - cx, y - cy))

    rate = (100.0 * coll / total) if total else 0.0
    print(f"runs={len(logs)} samples={total} collide={coll} rate={rate:.3f}%")
    for cell, x, y, yd, phase, offx, offy in sorted(grazes):
        print(f"  cell={cell} pose=({x:.2f},{y:.2f}) yaw={yd:+.0f}deg "
              f"phase={phase} off=(x{offx:+.2f},y{offy:+.2f})")
    print("by cell :", dict(Counter(g[0] for g in grazes)))
    print("by phase:", dict(Counter(g[4] for g in grazes)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
```

- [ ] **Step 2: Verify it reproduces the documented 0.33 % on the clean batches**

Run:
```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260614
python3 tools/replay_collision_oracle.py log/batch_diag_20260628_232253 log/batch_diag_20260629_071328
```
Expected: `samples=1808 collide=6 rate=0.332%`, `by cell : {(3, 9): 6}`. (Confirms the tool matches the baseline the fix is measured against.)

- [ ] **Step 3: Commit**

```bash
git add tools/replay_collision_oracle.py
git commit -m "$(cat <<'EOF'
tools: committed offline collision-oracle replay (reproduces (3,9) 0.33%)

Replays run-dir DIAG poses through the true-footprint maze_sim oracle and
reports the collision rate + per-cell/per-phase graze geometry, so the
Gazebo acceptance number is reproducible instead of computed ad-hoc.

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Post-implementation: Gazebo acceptance gate (with the user)

Not an autonomous task — requires the user to clear stray Gazebo sims and say "set up the run". After all three tasks are committed on `reverse-to-center-overshoot`:

1. Build: `cd ros2_ws_tugbot_nav_20260614 && source /opt/ros/jazzy/setup.bash && colcon build --symlink-install && source install/setup.bash`.
2. Controlled batch (≥ 8 runs): `bash tools/batch_diagnose_floodfill.sh 8 1200 false true`.
3. Replay the oracle over the new batch: `python3 tools/replay_collision_oracle.py log/batch_diag_<new_stamp>`.

**Acceptance (the honest gate — same standard that retired the prior guard):**
- Completion stays **16/16-equivalent** (every run `EXIT_REACHED`).
- Oracle collision rate **≤ 0.33 %**, target **0 grazes at (3,9)**.
- If either regresses → abandon and park the branch; update the spec/README/memory with the honest outcome. Do NOT merge to `main` until this gate passes.

Merge only after the gate passes (via `superpowers:finishing-a-development-branch`).

---

## Self-Review

**Spec coverage:** reverse-to-center primitive (Task 1); offline true-footprint regression incl. drift=0.05 re-eval (Task 2); committed replay tool (Task 3); Gazebo acceptance ≤0.33 % / 16-16 (post-impl gate). All spec sections covered.

**Placeholder scan:** none — every code step shows complete code; every run step shows the exact command and expected output.

**Type/name consistency:** `_PARALLEL_COS`, `a`, `speed`, `want`, `dyaw` consistent between the implementation and the test expectations; `centering_command` signature unchanged (only internal logic + docstring change), so its sole production caller (`maze_motion._center`) and all other unit tests are unaffected except the one updated case.
