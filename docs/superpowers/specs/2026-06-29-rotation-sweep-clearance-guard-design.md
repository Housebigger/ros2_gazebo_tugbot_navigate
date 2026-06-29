# Rotation-Sweep Clearance Guard — Design

**Date:** 2026-06-29
**Status:** design approved (user confirmed 2026-06-29)
**Topic:** Eliminate the residual rear-corner rotation-sweep graze (cell (3,9)-type) by guaranteeing that the in-place turn never sweeps the true footprint into a known wall; if it would, make room (translate toward cell-center) first.

---

## 1. Problem & ground truth

scan_match localization solved the *pervasive* wall-hitting (Gazebo true-footprint-oracle replay: odom baseline ~25%+ of sampled poses in collision → scan_match **0.33%**, 6/1808). The residual 6 samples are all at **one spot**: cell **(3,9)** (north-boundary row), pose ≈ (6.04, 18.40), while **turning**.

Investigated with the true asymmetric footprint (`FOOT_X_FRONT=0.262`, `FOOT_X_REAR=-0.468`, `FOOT_HALF_W=0.292`) against the known wall map (`wall_half_thickness=0.12`):

- **At the cell center (6, 18): 0/72 headings collide** — the footprint can rotate through a full circle freely. The cell is NOT too tight.
- Off-center toward the north wall: `y=18.2 → 0/72`, **`y=18.4 → 20/72`**, `y=18.6 → 67/72` (and symmetric along x: `x=6.2 → 0`, `x=6.4 → 16`). Threshold ≈ 0.2–0.4 m off-center.
- **Culprit:** the **REAR corners** (`REAR-L`/`REAR-R`) sweep into the **north boundary wall** (segment `(0.95, 19.02, 21.01, 19.02)`); at y=18.4 the rear corner reaches 0.068 m from the wall centerline (< 0.12 m).

**Conclusion:** the cause is **rotating while not centered** (arrival overshoot ~0.4 m toward a wall). Rotating at the (known) cell center is always safe. This matches the long-deferred "rear-corner under drift" xfails; it is a geometry/maneuver issue, independent of localization.

## 2. Goal

The motion FSM must **never execute an in-place rotation whose swept true footprint intersects a known wall.** When a planned turn would graze, the robot first makes room (translates toward the known cell center, collision-safely), then turns. Route discovery and the rest of the FSM are unchanged.

**Success:** offline tests prove a collision-free turn from the (3,9) off-center pose; the Gazebo scan_match batch, replayed through the true-footprint oracle, shows **0** collision samples (was 0.33%), still 8/8 EXIT_REACHED.

## 3. Approach

A swept-clearance **invariant** enforced at the single turn-entry point, plus a bounded, collision-safe make-room maneuver.

### Invariant (the safety contract)
At `maze_motion._route()` (`maze_motion.py:355–361`), after `target_cardinal` is computed and before `self.phase = 'turn'`: if `swept_collides(x, y, current_yaw, target_cardinal)` is true (the true footprint intersects a known wall anywhere along the **shortest** rotation arc), do NOT enter `turn`; enter the make-room phase instead.

### Make-room maneuver (`preturn`)
Drive toward the **known cell center** `cell_center(self.cell)` (= `(CELL_SIZE_M*cx, CELL_SIZE_M*cy)`), using **pure translation along the current heading** (no rotation → cannot sweep → collision-safe by construction):
- Compute the signed along-heading distance to the cell center: `s = (cx_ctr - x)·cos(yaw) + (cy_ctr - y)·sin(yaw)`. Drive forward if `s > 0`, reverse if `s < 0`, at low speed (`preturn_v`), holding heading (`w = 0`).
- Re-check `swept_collides(...)` every tick. When it clears → `phase = 'turn'` (set `turn_start`, `turn_in_tol=0`). 
- Bounded by `preturn_timeout_s` (and a max travel). On timeout (cannot clear by translating along this heading — e.g. the rare perpendicular-axis off-center case that pure translation can't fix), fall back to the existing stuck/recovery path (`_unstick`/escape) rather than force a grazing turn. **The invariant — never sweep into a wall — holds regardless of whether make-room succeeds.**

This fixes the observed case (arrival overshoot along the corridor axis: reversing along the heading directly undoes the overshoot) and is provably collision-free during make-room (no rotation). It does not claim to resolve every off-center geometry; it guarantees no grazing turn is ever executed.

## 4. Components & interfaces

### New: `src/tugbot_maze/tugbot_maze/collision_geometry.py` (ROS-free, NumPy)
Single source of footprint-vs-wall collision truth, shared by the offline sim and the runtime guard.

```python
def rect_hits_segments(segs, x, y, yaw, x0, x1, y0, y1, half_thickness) -> bool:
    """True iff the rectangle [x0,x1]x[y0,y1] (base_link) at pose (x,y,yaw) comes
    within half_thickness of any wall-segment centerline. segs: (S,4) ndarray.
    Exact: Liang-Barsky segment-vs-rectangle intersection (dist 0), else min of
    endpoint-to-box and box-corner-to-segment distances. (Moved verbatim from
    maze_sim.collides' yaw-given branch.)"""

class FootprintClearance:
    def __init__(self, segments, *, wall_half_thickness_m=0.12,
                 x0=FOOT_X_REAR, x1=FOOT_X_FRONT, y0=-FOOT_HALF_W, y1=FOOT_HALF_W):
        # precompute self._segs = np.asarray(segments).reshape(-1,4)
    def collides(self, x, y, yaw) -> bool:
        # rect_hits_segments(self._segs, x, y, yaw, x0,x1,y0,y1, m)
    def swept_collides(self, x, y, yaw0, yaw1, *, step_rad=math.radians(5.0)) -> bool:
        # sample the SHORTEST arc yaw0 -> yaw1 inclusive at step_rad; any collides
```

### Modify: `src/tugbot_maze/tugbot_maze/maze_sim.py`
`MazeSim.collides(x, y, yaw)` (the yaw-given branch, lines 197–234) → call `rect_hits_segments(self.segs, x, y, yaw, FOOT_X_REAR, FOOT_X_FRONT, -FOOT_HALF_W, FOOT_HALF_W, self.wall_half_thickness_m)`. The `yaw=None` bounding-circle branch stays. **Behavior must be byte-identical** (refactor only).

### Modify: `src/tugbot_maze/tugbot_maze/maze_motion.py`
- `__init__(..., segments=None)` → `self.clearance = FootprintClearance(segments) if segments is not None else None`. New params: `preturn_v` (e.g. 0.12 m/s), `preturn_timeout_s` (e.g. 4.0 s), `sweep_step_rad` (≈5°).
- `_route()` turn-entry (line ~361): gate as in §3; if blocked, `self.phase = 'preturn'` and record `preturn_start = t`.
- New `_preturn(self, pose, t)`: the make-room maneuver (§3); dispatched from `step()` like the other phases.
- When `self.clearance is None` (segments not supplied), behavior is unchanged (gate is a no-op) — preserves existing tests/offline callers that construct `MazeMotion()` without segments.

### Modify: `src/tugbot_maze/tugbot_maze/flood_fill_solver.py`
Pass the known map into motion: `self.motion = MazeMotion(self.brain, ..., mem=self.mem, segments=load_segments())` (lines 60–66). `load_segments` is already imported (for the localizer).

## 5. Data flow

Known wall segments (`load_segments()`) → `MazeMotion.clearance` (built once). Each turn-entry: `swept_collides(pose, target_cardinal)` over the shortest arc → turn (clear) or `preturn` (make room → re-gate → turn). Localization is accurate (scan_match rms 0.02–0.04 m), so the map-based check reflects the true footprint position.

## 6. Testing (TDD)

### `test/test_collision_geometry.py` (new)
1. **Refactor equivalence:** for a grid of random `(x,y,yaw)` near several cells, `FootprintClearance(load_segments()).collides(x,y,yaw)` equals `MazeSim(load_segments(),(0,0),0).collides(x,y,yaw)` (same truth, post-refactor).
2. **Ground-truth match (the investigation):** at cell (3,9) center `(6.0,18.0)`, `swept_collides` over a full turn is **False** for any target; at `(6.04,18.40)` a turn to a south-ish heading (e.g. target `-2.4 rad`) is **True**. Spot-check `y=18.2 → clear`, `y=18.4 → collides`.
3. **Swept sampling:** `swept_collides` returns True iff some sampled heading on the shortest arc collides (construct a case where only a mid-arc heading collides).

### `test/test_maze_motion_sim.py` (modify)
4. **Reproduce + fix:** `MazeSim(load_segments(), start_xy=(6.04, 18.40), start_yaw=math.pi/2)` (north — the arrival heading after driving up into the boundary cell, overshooting center by 0.4 m), `MazeMotion(segments=load_segments())`, `m.cell=(3,9)`, with a target hop whose `target_cardinal` lands the shortest arc in the grazing band (e.g. a south/west hop, `target_cardinal ≈ -2.4 rad`). FIRST assert the unguarded sweep grazes: `FootprintClearance(load_segments()).swept_collides(6.04, 18.40, math.pi/2, -2.4)` is True. THEN run the guarded FSM loop and assert: `sim.collides(sim.x, sim.y, sim.yaw)` is **False at every tick**, the robot reaches `target_cardinal` (within `yaw_tol`), and ends within ~0.2 m of the cell center. (For this north-arrival the make-room formula gives `s = -0.4 < 0` → reverse south, directly undoing the overshoot.)
5. **Regression:** the full clean solve (`test_reaches_exit_without_collision_or_desync` non-xfail params) and the scan_match closed-loop tests still pass (normal centered turns pass the gate immediately; `segments=None` callers unaffected).

### Refactor safety
6. The existing `test_maze_sim.py` suite stays green (collides unchanged in behavior).

## 7. Acceptance criteria

- All new + existing ROS-free tests green; the 3 `[0.05-*]` xfails remain xfailed.
- Gazebo: re-run the scan_match controlled batch (default pose source) and replay the recorded trajectories through the true-footprint oracle (`/tmp/replay_collisions.py`-style, now using `collision_geometry`): **0 collision samples** (was 6/1808; specifically 0 at cell (3,9)), still **8/8 EXIT_REACHED**, time unchanged (~560 s).

## 8. Out of scope / non-goals

- Resolving arbitrary perpendicular-axis off-centering by translation (rare; the invariant still prevents collision, falling back to existing recovery).
- Changing routing, sensing, or the turn profile itself.
- Sensor-based (live-LIDAR) clearance — the known-map check is used (consistent with scan_match; localization is accurate). Noted as a future alternative if map/true divergence ever appears.

## 9. Risks

- **maze_sim is "test-only"; extracting its collision core to a runtime module:** mitigated by the equivalence test (Test 1) — the runtime module is the same math, just shared. NumPy at runtime is already used (localizer).
- **Make-room can't clear (perpendicular off-center):** bounded timeout → existing recovery; invariant still holds (no grazing turn).
- **Per-tick swept check cost:** O(headings × segments), ~tens × tens, sub-ms; only at turn-entry, not every tick.
- **`segments=None` path:** the gate is a no-op so all existing `MazeMotion()` constructions and offline tests are unaffected; only the node (which passes segments) gets the guard.
