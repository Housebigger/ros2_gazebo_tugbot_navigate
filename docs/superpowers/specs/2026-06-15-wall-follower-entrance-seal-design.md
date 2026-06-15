# Wall-Follower Entrance-Seal Fix — Design

**Date:** 2026-06-15
**Workspace:** `ros2_ws_tugbot_nav_20260614`
**Status:** Approved (design); supersedes the `follow_side='left'` default and the
exit-only offline guarantee from `2026-06-15-wall-following-solver-design.md`.

## Problem

The reactive wall-follower (`explorer_type:=wall_follower`) reaches the exit by
**cheating**: instead of solving the maze interior, it slips back out the entrance
opening and follows the **outside** of the perimeter wall to the exit. Both the
entrance and the exit are openings in the outer boundary, so the building's exterior
is a short, wall-followable path between them.

**Evidence (GUI run `wall_follower_run_20260615_201314`, `follow_side=left`):**
- Outer-wall box (map frame): x ∈ [0.95, 21.07], y ∈ [−1.04, 19.09].
- **61 of 78 trajectory samples (78%) lay *outside* that box** (beyond the outer walls
  by > 0.3 m). The path reached x = 22.64 (1.6 m east of the east wall) and y = −3.19
  (2.1 m south of the south wall).
- The robot made a brief ~2.4 m incursion at the entrance, then reversed back out the
  opening, rounded the SW corner, and ran along the exterior (south wall → east wall →
  NE exit). The entire interior was never entered.

## Root cause

1. **The entrance opening is a real gap** in the outer wall. Nothing stops the robot
   from traversing back out through it after the entry drive; the left-hand rule near
   the opening leads it straight back out.
2. **The offline "guarantee" did not detect this.** `test_wall_follow_maze_sim.py`
   only asserted `distance_to_exit ≤ exit_radius` — it never checked the robot stayed
   inside the maze. The sim's segment set included the open entrance gap, so the
   simulated robot escaped exactly like the real one and still "passed."
3. **The guarantee selected the cheat.** `test_report_faster_hand` chose `left`
   because the exterior arc (1721 steps) is far shorter than the interior route (9320
   steps). The documented "left-hand is ~5.4× faster" was the cheat, mislabeled a
   feature. The launch/solver default `follow_side='left'` is therefore wrong.

## Goal & success criteria

The robot must **solve the interior**: reach the exit zone having stayed inside the
maze perimeter the whole time.

- **Success = `EXIT_REACHED` (within `exit_radius` = 1.2 m of the exit) AND zero
  trajectory samples outside the outer-wall box** (within a small tolerance).
- Reaching the exit zone from the interior is sufficient; the robot need not drive
  fully out through the exit opening.
- Sealing only the **entrance** is sufficient: this perfect maze's outer boundary has
  exactly two openings (entrance + exit); once the entrance is closed, the only
  remaining boundary gap is the goal.

## Approach: virtual sealed door in the perception layer

"Close the door behind the robot." Once the robot has entered, a virtual wall fills
the entrance gap. The reactive follower needs no new logic — it simply sees a closed
wall where the open door used to be and follows the interior. The **same seal
geometry** is used by the live solver and the offline sim, so the offline guarantee
proves what actually runs.

### Seal geometry (single source of truth)

The entrance is a `2.072 m` gap in the **west** outer wall (vertical wall), centered
at map `(0.95, 0.0)` (derived from the instance YAML: world opening center
`(−10.061281, −9.025070)` + map offset `(11.011281, 9.025070)`; `opening_side: left`).
The seal segment fills it:

```
seal = (0.95, −1.036) → (0.95, +1.036)   # map frame, vertical
```

A new ROS-free helper derives it so live and sim cannot drift apart:

```python
# wall_follow_control.py  (ROS-free)
def entrance_seal_segment(center_xy, width_m, opening_side):
    """Return (x0,y0,x1,y1) map-frame segment filling a boundary opening.
    opening_side 'left'/'right' -> vertical seal (gap spans y);
    'top'/'bottom' -> horizontal seal (gap spans x)."""
```

Default args (this maze): `center_xy=(0.95, 0.0)`, `width_m=2.072423`,
`opening_side='left'`.

### Live solver changes (`wall_follow_solver.py`)

New ROS-free fusion helper (in `wall_follow_control.py`):

```python
def fuse_virtual_segment(ranges, angle_min, angle_inc, pose, segment, *,
                         wall_half_thickness_m=0.12, max_range=12.0):
    """Return a new ranges list where each beam range is min(real, seal_range).
    For beam i: world angle = pose.yaw + angle_min + i*angle_inc; ray origin =
    (pose.x, pose.y); intersect the one segment; subtract wall_half_thickness;
    clip to max_range; inf (no hit) leaves the real range unchanged.
    Matches maze_sim's ray-segment + half-thickness convention."""
```

Control-loop integration:
- **Arming:** seal is **off** during `startup` / `entering` (the robot must be able to
  drive in); armed at the `entering → follow` transition. A `bool self.seal_armed`
  set when the phase becomes `follow`.
- In `follow` / `backup`, when `seal_armed` and pose is available, fuse the seal into
  `ranges` before `sectorize`. If pose is momentarily unavailable, skip fusion that
  tick (harmless — the seal is a safety net, not the only thing keeping it inside once
  it commits to an interior wall).
- New params: `entrance_seal_enabled` (default `true`), seal endpoints
  (`seal_x0,seal_y0,seal_x1,seal_y1`, defaults from `entrance_seal_segment`).

`_sectors()` gains an optional fused-ranges path; the `WallFollower` policy and
`sectorize` are unchanged.

### Offline guarantee changes (`maze_sim.py`, `test_wall_follow_maze_sim.py`)

- Add the seal segment to the sim's segment set. The sim start is `(2.0, 0.0)`
  (already inside, post-entry), so the seal is simply always present in the model.
  Provide it via `load_segments(..., include_entrance_seal=True)` or by appending
  `entrance_seal_segment(...)` in the test — the seal must use the identical helper.
- **New hard invariant in `run_to_exit`:** track every pose; fail (return `None` with a
  reason, or assert) if any pose is outside the outer-wall box
  `x∈[0.95,21.07], y∈[−1.04,19.09]` by more than a tolerance (e.g. 0.4 m). The box is
  derived from the `outer: true` segments, not hard-coded magic.
- **Re-select `follow_side`:** with the seal active, re-run both hands.
  - `test_<hand>_reaches_exit` now also requires "stayed inside".
  - `test_report_faster_hand` reports the legitimate faster hand.
  - Set the launch/solver/script default `follow_side` to the hand the sealed sim
    proves (expected to flip from `left`). The start-perturbation robustness test runs
    on the chosen hand.

### follow_side default propagation

Update the default `follow_side` in: `wall_follow_solver.py` param default, the launch
`DeclareLaunchArgument('follow_side', ...)`, and `tools/run_wall_follower_maze.sh` /
`run_wall_follower_reliability.sh`. Value = whatever the sealed sim selects.

## Components / files

| File | Change |
|---|---|
| `tugbot_maze/wall_follow_control.py` | + `entrance_seal_segment(...)`, + `fuse_virtual_segment(...)` (both ROS-free) |
| `tugbot_maze/wall_follow_solver.py` | arm seal on `entering→follow`; fuse seal into `ranges` before `sectorize`; new params |
| `tugbot_maze/maze_sim.py` | optionally include the entrance seal in the segment set (via the shared helper) |
| `test/test_wall_follow_control.py` | + unit tests: `entrance_seal_segment`, `fuse_virtual_segment` |
| `test/test_wall_follow_maze_sim.py` | seal active; stay-inside invariant; re-select & report hand; perturbation on chosen hand |
| `bringup launch` + `tools/run_wall_follower_*.sh` | `follow_side` default := sealed-sim choice |

## Testing

- **Unit (ROS-free, ~seconds):**
  - `entrance_seal_segment`: correct endpoints for `opening_side='left'` (and a
    `'top'` case for the horizontal branch).
  - `fuse_virtual_segment`: a beam aimed at the sealed gap from inside now reads the
    seal wall (≈ distance to the gap − half-thickness); beams aimed elsewhere are
    unchanged; the no-pose / disabled path returns ranges unchanged; a beam pointing
    away from the segment is unchanged (inf hit).
- **Integration gate (offline sim):** both/one hand reach the exit **and stay inside**;
  faster legitimate hand reported; perturbation-robust on the chosen hand.
- **Re-validation (Gazebo GUI):** rebuild + GUI run; regenerate the trajectory-vs-maze
  plot; acceptance = `EXIT_REACHED` AND 0 samples outside the box.

## Risks

1. **A hand may not solve the sealed interior.** Wall-following reaches the exit only
   if the exit is on the same connected wall the robot follows. For a simply-connected
   perfect maze (walls attached to the boundary), both hands qualify — but the **sim is
   the gate**: a hand that can't solve the sealed interior fails loudly and we use the
   other.
2. **Neither hand solves it** (e.g. a detached interior wall island) would be a real
   finding requiring a rethink — unlikely given the verified perfect-maze property, and
   it would surface immediately in the offline sim before any Gazebo time is spent.
3. **Live/sim seal divergence.** Mitigated by sharing `entrance_seal_segment` and
   matching `fuse_virtual_segment` to the sim's ray-segment + half-thickness math.

## Out of scope

- Trémaux / Pledge strategies (deferred to a future workspace, per prior decision).
- Changing the Gazebo world or the robot spawn pose (the perception seal makes physical
  world edits unnecessary).
- The `20260522` GCN solver (already accepted; it follows a known interior route).
