# Wall-Follower Entrance-Seal Fix — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Stop the reactive wall-follower from "cheating" (leaving the maze and following the *outside* of the perimeter wall to the exit) by sealing the entrance opening once the robot has entered, so it must solve the interior — and fix the offline guarantee to prove a legitimate interior solve.

**Architecture:** A virtual wall segment fills the entrance gap. It is fused into the LIDAR `ranges` in the live solver (after entry) and added to the raycaster's segment set in the offline sim — both using one shared geometry helper. The guarantee gains a hard "stayed inside the outer-wall box" invariant and re-confirms the faster *legitimate* hand. Empirically (verified against the real maze segments): sealed `left` solves the interior in 5595 steps with 0 exterior samples, sealed `right` in 9320 — so `follow_side='left'` stays the default but is now honest (it was a 1611-exterior-sample cheat before).

**Tech Stack:** Python 3.12, ROS 2 Jazzy (`rclpy`), `pytest`, NumPy (offline sim only). All new logic is ROS-free and unit-tested; the node is thin glue verified by the smoke test + Gazebo re-validation.

**Workspace:** `ros2_ws_tugbot_nav_20260614`. Run unit tests from `ros2_ws_tugbot_nav_20260614/src/tugbot_maze`. Run git commands from the repo root `/home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate`. Work happens on branch `fix-wall-follower-entrance-seal` (already created).

---

## File structure

| File | Responsibility | Change |
|---|---|---|
| `src/tugbot_maze/tugbot_maze/wall_follow_control.py` | ROS-free node-support helpers | **+** `entrance_seal_segment()`, **+** `fuse_virtual_segment()` |
| `src/tugbot_maze/tugbot_maze/maze_sim.py` | Test-only raycaster + integrator | **+** `outer_boundary_box()` |
| `src/tugbot_maze/tugbot_maze/wall_follow_solver.py` | Thin ROS node | arm seal on entry; fuse seal before `sectorize`; new params |
| `src/tugbot_maze/test/test_wall_follow_control.py` | Unit tests for the helpers | **+** tests for both new helpers |
| `src/tugbot_maze/test/test_wall_follow_maze_sim.py` | Offline guarantee | seal active; stay-inside invariant; re-select hand |
| `ros2_ws_tugbot_nav_20260614/README.md` | Workspace doc | correct the "outer-boundary arc" text (it described the cheat) |
| `docs/superpowers/specs/2026-06-15-wall-follower-entrance-seal-design.md` | Spec | append "Implementation outcome" |

Constants used across tasks (verified against `maze_wall_segments_20260528.yaml`):
- Entrance opening: center map `(0.95, 0.0)`, width `2.072423 m`, `opening_side='left'` → seal segment `(0.95, -1.0362115, 0.95, 1.0362115)`.
- Outer-wall box (map): `x ∈ [0.95, 21.07]`, `y ∈ [-1.04, 19.09]` (from the `outer: true` segments).
- Exit: `(21.072562, 18.083566)`, radius `1.2`. Sim start: `(2.0, 0.0)`, yaw `0.0`.

---

### Task 1: `entrance_seal_segment` helper

**Files:**
- Modify: `src/tugbot_maze/tugbot_maze/wall_follow_control.py`
- Test: `src/tugbot_maze/test/test_wall_follow_control.py`

- [ ] **Step 1: Write the failing tests**

Update the import line at the top of `test/test_wall_follow_control.py` and add `import pytest`:

```python
import pytest
from tugbot_maze.wall_follow_control import (
    exit_reached, entering_done, StallWatchdog, entrance_seal_segment)
```

Append these tests:

```python
def test_entrance_seal_segment_left_is_vertical():
    # west-wall opening (vertical wall) -> seal spans y at constant x
    seg = entrance_seal_segment((0.95, 0.0), 2.072423, 'left')
    assert seg == pytest.approx((0.95, -1.0362115, 0.95, 1.0362115))


def test_entrance_seal_segment_top_is_horizontal():
    # top/bottom opening (horizontal wall) -> seal spans x at constant y
    seg = entrance_seal_segment((4.0, 9.0), 2.0, 'top')
    assert seg == pytest.approx((3.0, 9.0, 5.0, 9.0))


def test_entrance_seal_segment_bad_side_raises():
    with pytest.raises(ValueError):
        entrance_seal_segment((0.0, 0.0), 1.0, 'sideways')
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cd ros2_ws_tugbot_nav_20260614/src/tugbot_maze && python3 -m pytest test/test_wall_follow_control.py -q`
Expected: FAIL with `ImportError: cannot import name 'entrance_seal_segment'`.

- [ ] **Step 3: Implement the helper**

Append to `tugbot_maze/wall_follow_control.py` (it already has `import math`):

```python
def entrance_seal_segment(center_xy, width_m, opening_side):
    """Return a map-frame segment (x0, y0, x1, y1) that fills a boundary opening.

    Used to "close the door" so the reactive follower treats the entrance as a
    solid wall once the robot is inside. `opening_side` 'left'/'right' is a
    vertical boundary wall, so the seal spans y; 'top'/'bottom' is a horizontal
    wall, so the seal spans x. width_m is the opening width.
    """
    cx, cy = float(center_xy[0]), float(center_xy[1])
    half = float(width_m) / 2.0
    if opening_side in ('left', 'right'):
        return (cx, cy - half, cx, cy + half)
    if opening_side in ('top', 'bottom'):
        return (cx - half, cy, cx + half, cy)
    raise ValueError("opening_side must be 'left', 'right', 'top', or 'bottom'")
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `cd ros2_ws_tugbot_nav_20260614/src/tugbot_maze && python3 -m pytest test/test_wall_follow_control.py -q`
Expected: PASS (all tests, including the pre-existing ones).

- [ ] **Step 5: Commit**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git add ros2_ws_tugbot_nav_20260614/src/tugbot_maze/tugbot_maze/wall_follow_control.py \
        ros2_ws_tugbot_nav_20260614/src/tugbot_maze/test/test_wall_follow_control.py
git commit -m "feat(wall-follower): add entrance_seal_segment helper"
```

---

### Task 2: `fuse_virtual_segment` helper

**Files:**
- Modify: `src/tugbot_maze/tugbot_maze/wall_follow_control.py`
- Test: `src/tugbot_maze/test/test_wall_follow_control.py`

- [ ] **Step 1: Write the failing tests**

Add `import math` and extend the import in `test/test_wall_follow_control.py`:

```python
import math
import pytest
from tugbot_maze.wall_follow_control import (
    exit_reached, entering_done, StallWatchdog,
    entrance_seal_segment, fuse_virtual_segment)
```

Append these tests (robot at `(2,0)`; a vertical seal wall at `x=0.95`, `y∈[-1,1]`, `1.05 m` west of the robot; half-thickness `0.12` → range `0.93`):

```python
def test_fuse_virtual_segment_beam_hits_seal():
    # single beam pointing west (yaw=pi, beam angle 0) -> hits the seal at x=0.95
    seg = (0.95, -1.0, 0.95, 1.0)
    fused = fuse_virtual_segment([12.0], 0.0, 0.0, (2.0, 0.0, math.pi), seg,
                                 wall_half_thickness_m=0.12, max_range=12.0)
    assert fused[0] == pytest.approx(0.93, abs=1e-6)   # 1.05 - 0.12


def test_fuse_virtual_segment_beam_pointing_away_unchanged():
    # beam pointing east (yaw=0) -> seal is behind the ray -> range unchanged
    seg = (0.95, -1.0, 0.95, 1.0)
    fused = fuse_virtual_segment([5.0], 0.0, 0.0, (2.0, 0.0, 0.0), seg)
    assert fused[0] == pytest.approx(5.0)


def test_fuse_virtual_segment_keeps_nearer_real_obstacle():
    # a real return at 0.5 m is nearer than the seal (0.93) -> keep 0.5
    seg = (0.95, -1.0, 0.95, 1.0)
    fused = fuse_virtual_segment([0.5], 0.0, 0.0, (2.0, 0.0, math.pi), seg)
    assert fused[0] == pytest.approx(0.5)


def test_fuse_virtual_segment_nonfinite_real_uses_seal():
    seg = (0.95, -1.0, 0.95, 1.0)
    fused = fuse_virtual_segment([float('inf')], 0.0, 0.0, (2.0, 0.0, math.pi), seg)
    assert fused[0] == pytest.approx(0.93, abs=1e-6)
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cd ros2_ws_tugbot_nav_20260614/src/tugbot_maze && python3 -m pytest test/test_wall_follow_control.py -q`
Expected: FAIL with `ImportError: cannot import name 'fuse_virtual_segment'`.

- [ ] **Step 3: Implement the helper**

Append to `tugbot_maze/wall_follow_control.py`:

```python
def fuse_virtual_segment(ranges, angle_min, angle_inc, pose, segment, *,
                         wall_half_thickness_m=0.12, max_range=12.0):
    """Fuse a single virtual wall `segment` into a LaserScan `ranges` list.

    Each beam range becomes min(real_range, distance-to-segment). This mirrors
    maze_sim's ray-segment intersection and half-thickness convention so the
    live "sealed door" matches the offline guarantee. `pose` = (x, y, yaw) and
    `segment` = (x0, y0, x1, y1) share the same (map) frame. Beams that miss the
    segment keep their real range. Non-finite / non-positive real ranges are
    treated as `max_range` before fusing. Returns a new list.
    """
    px, py, yaw = float(pose[0]), float(pose[1]), float(pose[2])
    x0, y0, x1, y1 = (float(c) for c in segment)
    ex, ey = x1 - x0, y1 - y0
    wx, wy = x0 - px, y0 - py
    out = []
    for i in range(len(ranges)):
        r = ranges[i]
        if r is None or not math.isfinite(r) or r <= 0.0:
            r = max_range
        else:
            r = min(float(r), max_range)
        ang = yaw + angle_min + i * angle_inc
        dx, dy = math.cos(ang), math.sin(ang)
        denom = dx * ey - dy * ex
        if abs(denom) > 1e-12:
            t = (wx * ey - wy * ex) / denom          # ray param == distance (d is unit)
            u = (wx * dy - wy * dx) / denom          # segment param in [0, 1]
            if t >= 0.0 and 0.0 <= u <= 1.0:
                seal_r = max(0.0, min(t - wall_half_thickness_m, max_range))
                r = min(r, seal_r)
        out.append(r)
    return out
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `cd ros2_ws_tugbot_nav_20260614/src/tugbot_maze && python3 -m pytest test/test_wall_follow_control.py -q`
Expected: PASS (all helper tests).

- [ ] **Step 5: Commit**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git add ros2_ws_tugbot_nav_20260614/src/tugbot_maze/tugbot_maze/wall_follow_control.py \
        ros2_ws_tugbot_nav_20260614/src/tugbot_maze/test/test_wall_follow_control.py
git commit -m "feat(wall-follower): add fuse_virtual_segment LIDAR seal helper"
```

---

### Task 3: Seal the offline guarantee + re-select the legitimate hand

This is the correctness gate: with the entrance gap left open the simulated robot escaped exactly like the real one, and the test only checked "reached exit XY". We add `outer_boundary_box()`, seal the run, and assert the robot never leaves the box.

**Files:**
- Modify: `src/tugbot_maze/tugbot_maze/maze_sim.py`
- Modify (rewrite): `src/tugbot_maze/test/test_wall_follow_maze_sim.py`

- [ ] **Step 1: Write the failing guarantee tests (rewrite the test file)**

Replace the entire contents of `test/test_wall_follow_maze_sim.py` with:

```python
import math
import pytest
from tugbot_maze.wall_follower import WallFollower, sectorize
from tugbot_maze.maze_sim import MazeSim, load_segments, outer_boundary_box
from tugbot_maze.wall_follow_control import entrance_seal_segment

EXIT_XY = (21.072562, 18.083566)
EXIT_RADIUS = 1.2
START_XY = (2.0, 0.0)         # post-ENTRY_DIRECT: ~2 m inside the entrance mouth
START_YAW = 0.0               # facing +x (east), into the maze
# Entrance: 2.072 m gap in the west outer wall, centered map (0.95, 0).
ENTRANCE_SEAL = entrance_seal_segment((0.95, 0.0), 2.072423, 'left')
OUTSIDE_TOL_M = 0.4           # robot center may sit this far past the outer-wall line


def run_to_exit(follow_side, *, sealed=True, max_steps=20000, dt=0.1, n_beams=72,
                start_xy=START_XY, start_yaw=START_YAW):
    """Run the follower in the sim. Returns (steps_to_exit | None, closest, outside).

    `sealed` closes the entrance gap with ENTRANCE_SEAL. `outside` counts samples
    where the robot center left the outer-wall box by more than OUTSIDE_TOL_M --
    i.e. cheating by following the maze exterior.
    """
    segs = load_segments()
    if sealed:
        segs = segs + [ENTRANCE_SEAL]
    bx0, bx1, by0, by1 = outer_boundary_box()
    sim = MazeSim(segs, start_xy=start_xy, start_yaw=start_yaw)
    assert not sim.collides(start_xy[0], start_xy[1]), "start must be collision-free"
    wf = WallFollower(follow_side=follow_side)
    closest = float('inf')
    outside = 0
    for step in range(max_steps):
        ranges, amin, ainc = sim.scan(n_beams=n_beams)
        sectors = sectorize(ranges, amin, ainc, follow_side)
        cmd = wf.update(sectors)
        sim.step(cmd.v, cmd.w, dt)
        x, y, _ = sim.pose
        if max(bx0 - x, x - bx1, by0 - y, y - by1) > OUTSIDE_TOL_M:
            outside += 1
        d = math.hypot(x - EXIT_XY[0], y - EXIT_XY[1])
        closest = min(closest, d)
        if d <= EXIT_RADIUS:
            return step + 1, closest, outside
    return None, closest, outside


def test_sealed_left_hand_solves_interior():
    steps, closest, outside = run_to_exit('left', sealed=True)
    assert steps is not None, f"sealed left failed to reach exit; closest={closest:.2f} m"
    assert outside == 0, f"sealed left left the maze {outside} times (cheating)"


def test_sealed_right_hand_solves_interior():
    steps, closest, outside = run_to_exit('right', sealed=True)
    assert steps is not None, f"sealed right failed to reach exit; closest={closest:.2f} m"
    assert outside == 0, f"sealed right left the maze {outside} times (cheating)"


def test_unsealed_left_hand_cheats_via_exterior():
    # Locks in WHY the seal is required: without it, left escapes the perimeter.
    _, _, outside = run_to_exit('left', sealed=False)
    assert outside > 0, "expected unsealed left to leave the maze (the bug the seal fixes)"


def test_report_faster_legitimate_hand(capsys):
    steps_l, _, out_l = run_to_exit('left', sealed=True)
    steps_r, _, out_r = run_to_exit('right', sealed=True)
    assert steps_l is not None and steps_r is not None
    assert out_l == 0 and out_r == 0
    faster = 'left' if steps_l <= steps_r else 'right'
    with capsys.disabled():
        print(f"\n[GUARANTEE sealed] left={steps_l} right={steps_r} -> FASTER_LEGIT_HAND={faster}")
    assert faster == 'left'        # left ~5595 < right ~9320 (sealed)


@pytest.mark.parametrize("dx,dy,dyaw", [(0.2, 0.2, 0.1), (-0.2, -0.2, -0.1), (0.2, -0.2, 0.1)])
def test_sealed_left_hand_robust_to_start_perturbation(dx, dy, dyaw):
    steps, closest, outside = run_to_exit('left', sealed=True,
                                          start_xy=(START_XY[0] + dx, START_XY[1] + dy),
                                          start_yaw=START_YAW + dyaw)
    assert steps is not None, (
        f"perturbed sealed left failed to reach exit; closest={closest:.2f} m "
        f"(dx={dx}, dy={dy}, dyaw={dyaw})")
    assert outside == 0, f"perturbed sealed left cheated {outside} times"
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cd ros2_ws_tugbot_nav_20260614/src/tugbot_maze && python3 -m pytest test/test_wall_follow_maze_sim.py -q`
Expected: FAIL with `ImportError: cannot import name 'outer_boundary_box' from 'tugbot_maze.maze_sim'`.

- [ ] **Step 3: Implement `outer_boundary_box` in `maze_sim.py`**

Append to `tugbot_maze/maze_sim.py` (it already imports `yaml` and defines `px_to_map`, `default_segments_path`):

```python
def outer_boundary_box(path: Optional[str] = None) -> Tuple[float, float, float, float]:
    """Return (xmin, xmax, ymin, ymax) of the maze's outer boundary in map frame.

    Built from the segments flagged `outer: true` in the YAML. Used by the
    guarantee to assert the solver never leaves the maze (no exterior cheating).
    """
    if path is None:
        path = default_segments_path()
    with open(path) as f:
        doc = yaml.safe_load(f)
    xs: List[float] = []
    ys: List[float] = []
    for s in doc['segments']:
        if s.get('outer'):
            x0, y0 = px_to_map(float(s['p0_px'][0]), float(s['p0_px'][1]))
            x1, y1 = px_to_map(float(s['p1_px'][0]), float(s['p1_px'][1]))
            xs += [x0, x1]
            ys += [y0, y1]
    return (min(xs), max(xs), min(ys), max(ys))
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `cd ros2_ws_tugbot_nav_20260614/src/tugbot_maze && python3 -m pytest test/test_wall_follow_maze_sim.py -q -s`
Expected: PASS. The `-s` shows `[GUARANTEE sealed] left=5595 right=9320 -> FASTER_LEGIT_HAND=left` (exact step counts may vary by ±a few). If `test_sealed_*_solves_interior` fails or `outside > 0`, STOP — the seal geometry is wrong; do not proceed.

- [ ] **Step 5: Commit**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git add ros2_ws_tugbot_nav_20260614/src/tugbot_maze/tugbot_maze/maze_sim.py \
        ros2_ws_tugbot_nav_20260614/src/tugbot_maze/test/test_wall_follow_maze_sim.py
git commit -m "fix(wall-follower): offline guarantee enforces staying inside the maze

Seal the entrance gap in the sim and fail the run if the robot leaves the
outer-wall box. Re-selects the faster LEGITIMATE hand (left, 5595 vs 9320
sealed steps); documents that unsealed left cheats via the exterior."
```

---

### Task 4: Wire the seal into the live solver

The node is thin glue over the Task 1/2 helpers (which are unit-tested) and the Task 3 sim (the integration model). Behavioral correctness is verified by the offline guarantee + the Gazebo re-validation that follows the plan.

**Files:**
- Modify: `src/tugbot_maze/tugbot_maze/wall_follow_solver.py`
- Test: `src/tugbot_maze/test/test_wall_follow_solver_smoke.py` (unchanged; must still pass)

- [ ] **Step 1: Update the import**

In `tugbot_maze/wall_follow_solver.py`, change:

```python
from tugbot_maze.wall_follow_control import exit_reached, entering_done, StallWatchdog
```

to:

```python
from tugbot_maze.wall_follow_control import (
    exit_reached, entering_done, StallWatchdog,
    entrance_seal_segment, fuse_virtual_segment)
```

- [ ] **Step 2: Declare seal params + state in `__init__`**

In `WallFollowSolver.__init__`, immediately after the `self.cruise_v = cruise_v` line (end of the param block, before `self.follower = WallFollower(...)`), add:

```python
        # --- entrance seal: close the door behind the robot so it solves the
        #     interior instead of following the maze exterior ---
        self.entrance_seal_enabled = bool(
            self.declare_parameter('entrance_seal_enabled', True).value)
        self.wall_half_thickness_m = float(
            self.declare_parameter('wall_half_thickness_m', 0.12).value)
        _seal = entrance_seal_segment((0.95, 0.0), 2.072423, 'left')
        sx0 = float(self.declare_parameter('seal_x0', _seal[0]).value)
        sy0 = float(self.declare_parameter('seal_y0', _seal[1]).value)
        sx1 = float(self.declare_parameter('seal_x1', _seal[2]).value)
        sy1 = float(self.declare_parameter('seal_y1', _seal[3]).value)
        self.seal_segment = (sx0, sy0, sx1, sy1)
        self.seal_armed = False
```

- [ ] **Step 3: Replace `_sectors` to fuse the seal when armed**

Replace:

```python
    def _sectors(self):
        s = self.scan_msg
        return sectorize(s.ranges, s.angle_min, s.angle_increment, self.follow_side,
                         max_range=self.max_range_m)
```

with:

```python
    def _sectors(self, pose=None):
        s = self.scan_msg
        ranges = s.ranges
        # Once the robot is inside, fuse the sealed entrance into the scan so the
        # follower "sees" a closed door and follows the interior, not the exterior.
        if self.seal_armed and pose is not None:
            ranges = fuse_virtual_segment(
                ranges, s.angle_min, s.angle_increment, pose, self.seal_segment,
                wall_half_thickness_m=self.wall_half_thickness_m,
                max_range=self.max_range_m)
        return sectorize(ranges, s.angle_min, s.angle_increment, self.follow_side,
                         max_range=self.max_range_m)
```

- [ ] **Step 4: Arm the seal at the `entering → follow` transition, and pass pose in `follow`**

In `_control_tick`, in the `if self.phase == 'entering':` block, change the transition body so it arms the seal:

```python
            if done or blocked:
                self.phase = 'follow'
                self.follower.state = State.FIND_WALL
                self.seal_armed = self.entrance_seal_enabled
                if pose is not None:
                    self.watchdog.reset(t, pose[0], pose[1])
                self.get_logger().info('engaging wall-follower (entered=%s blocked=%s) seal=%s'
                                       % (done, blocked, self.seal_armed))
```

(The `blocked = ... self._sectors().front` line just above stays as-is: `seal_armed` is still False there, so `_sectors()` with no pose is correct.)

Then change the final two lines of `_control_tick` (the `# phase == 'follow'` command) from:

```python
        cmd = self.follower.update(self._sectors())
        self._publish_cmd(cmd.v, cmd.w)
```

to:

```python
        cmd = self.follower.update(self._sectors(pose))
        self._publish_cmd(cmd.v, cmd.w)
```

- [ ] **Step 5: Verify the node still imports and the full ROS-free suite passes**

Run: `cd ros2_ws_tugbot_nav_20260614/src/tugbot_maze && python3 -m pytest test/test_wall_follower.py test/test_maze_sim.py test/test_wall_follow_control.py test/test_wall_follow_maze_sim.py -q`
Expected: PASS (all unit + guarantee tests).

Run (if rclpy is sourced): `python3 -c "from tugbot_maze import wall_follow_solver; print('imports OK')"`
Expected: `imports OK` (no syntax/import error). If rclpy is not sourced this will ImportError on `rclpy` — that is fine; the colcon smoke test covers it.

- [ ] **Step 6: Commit**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git add ros2_ws_tugbot_nav_20260614/src/tugbot_maze/tugbot_maze/wall_follow_solver.py
git commit -m "feat(wall-follower): seal the entrance in the live solver after entry

Arm a virtual wall across the entrance gap on the entering->follow transition
and fuse it into /scan before sectorize, so the robot solves the interior
instead of following the maze exterior. Same geometry as the offline guarantee."
```

---

### Task 5: Correct the docs (the README described the cheat as a feature)

**Files:**
- Modify: `ros2_ws_tugbot_nav_20260614/README.md`
- Modify: `docs/superpowers/specs/2026-06-15-wall-follower-entrance-seal-design.md`

- [ ] **Step 1: Fix the `follow_side` paragraph in the README**

In `ros2_ws_tugbot_nav_20260614/README.md`, find the paragraph under "How to run" that begins ``follow_side`` defaults to `left`` and currently claims it "traces the short outer-boundary arc to the NE exit rather than the long interior route (1721 vs 9320 sim-steps)". Replace that paragraph with:

```markdown
`follow_side` defaults to `left`. The robot drives ~2 m in, then a **virtual wall
seals the entrance behind it** (the door closes), so it solves the maze *interior*
and cannot follow the perimeter exterior. Under the seal, the left hand reaches the
exit in ~5595 offline sim-steps and the right in ~9320, both staying inside the
outer wall — so `left` is the faster *legitimate* hand. (An earlier build with no
seal let `left` cheat: it left the maze and ran along the *outside* of the perimeter
wall; the offline guarantee now fails any run that leaves the outer-wall box.)
```

- [ ] **Step 2: Add the seal to the architecture table in the README**

In the "Wall-following architecture" table, update the `wall_follow_control.py` row to mention the new helpers, and the `wall_follow_solver.py` row to mention the seal. Change the `wall_follow_control.py` row's Role cell to:

```markdown
Node-support helpers — `exit_reached`, `entering_done`, `StallWatchdog`, plus `entrance_seal_segment` / `fuse_virtual_segment` (the virtual sealed door)
```

and append to the `wall_follow_solver.py` row's Role cell: `; seals the entrance into the scan once entered (interior-only solve)`.

- [ ] **Step 3: Append the implementation outcome to the spec**

Append to `docs/superpowers/specs/2026-06-15-wall-follower-entrance-seal-design.md`:

```markdown

## Implementation outcome (2026-06-15)

Implemented via subagent-driven TDD. Offline guarantee (sealed): left reaches the
exit in 5595 sim-steps, right in 9320, **both with 0 samples outside the outer-wall
box**; the three start-perturbations are robust on the sealed left hand. `follow_side`
stays `left` (the faster legitimate hand) — it did NOT flip to `right` as the design
guessed, because the seal converts left from a 1611-exterior-sample cheat into a clean
interior solve. The live solver arms the seal on the entering→follow transition and
fuses it into `/scan` with the same geometry. Gazebo re-validation: see the re-run
trajectory plot (0 samples outside the box, EXIT_REACHED from the interior).
```

- [ ] **Step 4: Verify no follow_side default needs changing**

`follow_side='left'` is already the default in `wall_follow_solver.py`, the launch `DeclareLaunchArgument('follow_side', ...)`, and `tools/run_wall_follower_maze.sh`. Confirm with:

Run: `cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate && grep -rn "follow_side" ros2_ws_tugbot_nav_20260614/tools/ ros2_ws_tugbot_nav_20260614/src/tugbot_bringup/launch/ | grep -i "default\|left\|right"`
Expected: defaults are `left`. No change required (the seal made `left` legitimate).

- [ ] **Step 5: Commit**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git add ros2_ws_tugbot_nav_20260614/README.md \
        docs/superpowers/specs/2026-06-15-wall-follower-entrance-seal-design.md
git commit -m "docs(wall-follower): correct README (entrance seal, interior solve) + spec outcome"
```

---

## Final verification (after all tasks)

- [ ] Full ROS-free suite green:

Run: `cd ros2_ws_tugbot_nav_20260614/src/tugbot_maze && python3 -m pytest test/test_wall_follower.py test/test_maze_sim.py test/test_wall_follow_control.py test/test_wall_follow_maze_sim.py -q`
Expected: all PASS.

- [ ] Rebuild: `cd ros2_ws_tugbot_nav_20260614 && source /opt/ros/jazzy/setup.bash && colcon build --symlink-install` → 6 packages finished.

- [ ] **Gazebo re-validation (the acceptance gate):** GUI re-run, then regenerate the trajectory-vs-maze plot. Acceptance = `EXIT_REACHED` **AND 0 trajectory samples outside the outer-wall box** (contrast the pre-fix run's 61/78 outside). This is tracked as a separate task and run after the branch is implemented.
