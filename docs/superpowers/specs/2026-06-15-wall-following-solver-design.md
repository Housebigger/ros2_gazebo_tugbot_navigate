# Reactive Wall-Following Maze Solver — Design Spec

**Date:** 2026-06-15
**Workspace:** `ros2_ws_tugbot_nav_20260614`
**Status:** Approved (design); ready for implementation planning

## Goal

Give the Tugbot a **provably-correct, wedge-proof** autonomous maze solver that reliably
reaches the exit at map `(21.07, 18.08)` from the entrance at map `(0, 0)` — targeting
**≥ 4/5 `EXIT_REACHED`** within a 1500 s headless budget, without any prior map knowledge
in the control loop.

## Background & rationale (why this approach)

The prior frontier-coverage solver (`20260614`) explores untrapped but never finishes:
recent runs show ~85–120 *wedge* events per run (~1.5 per goal), the robot reaching ~8.7 m
from the exit then wandering back, and the exit-dash firing 0–3 times. The root cause is
**navigation/control, not planning**: Nav2 point-goals + reactive fallback jam in the
1.76 m corridors. GCN (guided corridor navigation) succeeds on the same corridors because
it follows a corridor *centerline* continuously.

A ground-truth topology analysis of the `20260528` maze (reconstructed from
`maze_wall_segments_20260528.yaml`) establishes the decisive fact:

```
100 cells, 99 edges, fully connected  →  TREE (no loops; a "perfect maze")
entrance (SW) and exit (NE) are both gaps in the OUTER wall
shortest entrance→exit: 30 cell-steps
wall-following reaches the exit: one hand ≈72 steps, the other ≈126 — BOTH guaranteed
```

For a perfect maze with entrance and exit on the outer boundary, the **right/left-hand
rule (wall-following) is mathematically guaranteed to reach the exit.** Wall-following
maintains a *fixed lateral offset* from one wall via continuous LIDAR control, so it
**cannot wedge** (no point-goal is ever aimed into a corner) and **cannot loop forever**
(a tree has no cycles). This directly removes both documented blockers.

Reference inspiration: the Bug-algorithm family (Bug2 / TangentBug / DistBug — drive-to-goal
plus boundary-following) and standard ROS2 reactive wall-followers (LIDAR + PID on lateral
distance). We adopt the simplest member that is *guaranteed here*: pure wall-following. The
known exit location is used only for the success check in v1 (Bug2's m-line bias is
explicitly out of scope — see Scope).

## Architecture

Thin ROS node delegates *all control* to a ROS-free reactive core, validated offline by a
ROS-free maze simulator:

```
/scan (LaserScan) ──► sectorize ──► WallFollower.update(sectors) ──► (v, ω) ──► /cmd_vel_nav (10 Hz)
map→base_link TF  ──► pose ──► exit-distance check ──► EXIT_REACHED / DONE
```

- **No Nav2 action client, no costmap, no `/maze_boundary_map`, no frontier/Trémaux** in the
  control loop. (Those remain available behind their existing `explorer_type` flags.)
- SLAM (`slam_toolbox`) stays running **only** for `map→base_link` localization (exit check +
  DIAG) — never for planning. The control loop itself needs only `/scan`.

## Components

### 1. `wall_follower.py` (ROS-free core — the brain)

Pure, deterministic, fully unit-testable. No ROS imports.

**Sector abstraction** — the node reduces a `LaserScan` to a small struct so the core never
touches ROS types:

```python
@dataclass
class Sectors:
    front: float          # min range in FRONT  (±front_half_angle of heading)
    side:  float          # min range in SIDE   (followed side, −90°±side_half_angle for right-hand)
    front_side: float     # min range in FRONT-SIDE diagonal (−45°±diag_half_angle for right-hand)
    # ranges are meters; inf/NaN already sanitized to a large finite value by the node
```

**State machine** (`enum State: FIND_WALL, FOLLOW, TURN_AWAY, CORNER`), right-hand rule
(left-hand = mirror via `follow_side` param). The core is a *stateful but pure* policy —
it keeps `self.state` + a hysteresis dwell counter, and `update` is a deterministic function
of `(self.state, Sectors)` with no ROS, time, or progress input (stall recovery lives in the
node, so the core stays trivially testable and the offline guarantee test exercises it
directly). One method:

```python
def update(self, s: Sectors) -> Command  # Command(v: float, w: float)
```

Decision order each tick (right-hand; `+w` = turn left / counter-clockwise):

1. **`s.front < front_block_m`** → blocked ahead → **TURN_AWAY**: rotate left in place
   (`v≈0, w=+turn_w`) until front clears. (Inside corner / T-stem.)
2. **elif `s.side > wall_lost_m`** → followed wall vanished → **CORNER**: arc toward the side
   (`v=corner_v, w=−corner_w`) to round into the opening. (Outside corner / junction.)
3. **else** → **FOLLOW**: PID on `(s.side − target_wall_m)` → `w = clamp(kp*err + kd*derr, ±w_max)`,
   `v = cruise_v` reduced by `|w|`. (Corridor centering.)
4. **FIND_WALL** (initial, until any range < engage_m): drive forward `v=cruise_v, w=0`; on
   engage, transition to FOLLOW.

Tunable params (constructor args, with defaults; the node passes ROS params through):
`target_wall_m=0.6`, `front_block_m=0.7`, `wall_lost_m=1.2`, `engage_m=1.0`, `cruise_v=0.3`,
`corner_v=0.18`, `corner_w=0.6`, `turn_w=0.7`, `w_max=0.8`, `kp=1.5`, `kd=0.4`,
`follow_side='right'`. Hysteresis: a small dwell (`min_state_ticks`) before leaving CORNER/
TURN_AWAY back to FOLLOW, to damp oscillation at junctions.

Geometry justification for defaults: corridor clear width 1.76 m, robot radius 0.35 m →
centered offset 0.88 m; following at 0.6 m keeps ≥0.25 m off the followed wall and ~1.16 m on
the open side. `front_block_m=0.7` starts the turn with >0.35 m margin to the wall face.
`wall_lost_m=1.2` separates "still beside the wall (~0.6 m)" from "opening to the next
corridor (≥1.76 m)".

### 2. `maze_sim.py` (test-only, ROS-free — the proof harness)

A minimal 2-D raycaster + unicycle integrator over the **real** `20260528` wall segments
(loaded from `maze_wall_segments_20260528.yaml`, same px→world transform the world file uses,
2× scaled, in the **map frame** so entrance=(0,0), exit=(21.07,18.08)):

```python
class MazeSim:
    def __init__(self, segments, start_xy, start_yaw): ...
    def scan(self, n_beams, fov_rad, max_range) -> list[float]   # ranges from current pose
    def step(self, v: float, w: float, dt: float) -> None        # integrate + clamp at walls
    @property
    def pose(self) -> tuple[float, float, float]                 # x, y, yaw (map frame)
```

Walls are line segments; `scan` ray-casts each beam against all segments (closest hit).
`step` integrates `x+=v cosθ dt; y+=v sinθ dt; θ+=w dt`, and if the new center is within
robot_radius of any wall, the move is rejected/clamped (so the sim robot can't pass through
walls — a stuck sim robot models a real stall).

This lets a unit test run the **actual** `wall_follower` (via the same `Sectors`
abstraction) through the **actual** maze and assert exit-reaching — proving the guarantee
offline and objectively selecting the faster hand.

### 3. `wall_follow_solver` node (ROS — thin wiring)

A new control path selected by `explorer_type:=wall_follower`. Responsibilities:

- Subscribe `/scan`; sanitize (inf/NaN → `max_range`); compute `Sectors` for `follow_side`.
- Look up `map→base_link` (TF) for pose; compute `made_progress` (moved > `progress_eps` in
  the last `progress_window` s) and exit distance. (`made_progress` feeds only the node's
  stall watchdog — not the core.)
- 10 Hz timer: `cmd = wall_follower.update(sectors)`; the stall watchdog may override `cmd`
  with a back-up; publish the resulting `Twist` on `/cmd_vel_nav`.
- **Entry:** reuse existing ENTRY_DIRECT (drive ~2 m inward) before engaging FIND_WALL, so the
  robot is inside the mouth with the south outer wall on its right.
- **Stall watchdog:** if `not made_progress` for `stall_s` (default 4 s) → issue a brief
  back-up (`v=−0.15` for `backup_s`) then re-engage FIND_WALL. Bounded retry counter only
  guards sensor/control glitches (a tree maze cannot trap a wall-follower).
- **Exit:** when exit distance < `exit_radius_m` (1.2) → log `EXIT_REACHED`, publish zero
  velocity, transition DONE.
- **DIAG** every 5 s: `pose`, `yaw`, `dist_to_exit`, `state`, `made_progress`.
- Reuses: the maze startup/entry scaffolding, exit self-check, and DIAG style from
  `maze_solver.py`. (Implementation may extend `maze_solver.py` with the new branch or add a
  sibling node — the plan will choose; either way the control loop is the wall-follower.)

## Data flow

1. Startup → ENTRY_DIRECT drives the robot ~2 m into the maze mouth.
2. Control timer (10 Hz): `/scan` → `Sectors` → `wall_follower.update` → `/cmd_vel_nav`.
3. Localization timer: `map→base_link` → exit distance + progress.
4. Exit distance < 1.2 m → `EXIT_REACHED` → stop.

## Error handling

| Condition | Handling |
|---|---|
| `inf`/`NaN` beams | node sanitizes to `max_range` before sectorizing |
| All-side scan dropout | `Sectors` all = `max_range` → FIND_WALL creeps forward slowly |
| Pose/TF unavailable | skip exit check that tick; keep driving (control loop needs only `/scan`) |
| Stall (no progress `stall_s`) | brief back-up + re-engage FIND_WALL; bounded retries |
| Wall-follower oscillation | hysteresis dwell + `w_max` clamp + speed-down-while-turning |

## Testing strategy (TDD)

1. **Unit — `wall_follower` branches** (`test_wall_follower.py`): front-blocked → turn away
   (`w>0, v≈0`); side-open → corner arc (`w<0, v>0`); corridor → PID centering (sign of `w`
   matches offset error; `|w|≤w_max`); find-wall → forward until engage; hysteresis dwell.
2. **Integration — guarantee proof** (`test_wall_follow_maze_sim.py`): `MazeSim` over the real
   `20260528` segments + real `wall_follower`, from entrance, `dt=0.1`, step budget; **assert
   the robot reaches within `exit_radius_m` of the exit.** Run for `follow_side='right'`
   (primary) and assert it is no slower than `'left'` (records the objective hand choice).
   A second case starts the robot mid-corridor to confirm FIND_WALL engagement.
3. **Sim — reliability** (`tools/run_solver_maze.sh ... explorer_type:=wall_follower`): single
   run reaches `EXIT_REACHED`; then `run_solver_reliability.sh 5 1500` targeting ≥ 4/5.

## Scope / YAGNI

- **In:** pure right-hand reactive wall-following; ROS-free core + maze-sim proof; node wiring;
  entry reuse; stall recovery; exit check; reliability run.
- **Out (v1):** Bug2 m-line / exit-bias (graft only if pure wall-following underperforms);
  frontier/Trémaux in the loop (kept behind existing flags); Nav2/costmap in the loop;
  dynamic-obstacle handling (maze is static).

## Success criteria

1. `wall_follower` + `maze_sim` integration test proves exit-reaching on the real maze graph
   (the guarantee, offline) and fixes the faster `follow_side`.
2. A headless Gazebo run logs `EXIT_REACHED` via `explorer_type:=wall_follower`.
3. Reliability batch ≥ 4/5 `EXIT_REACHED` within 1500 s — the standing bar for "reliable
   autonomous 通关."
