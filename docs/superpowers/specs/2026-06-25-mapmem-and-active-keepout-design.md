# Map-Memory Manager + Active Keep-Out — Design

> Run `flood_fill_run_20260625_082913` (directional keep-out, branch `no-progress-watchdog`): TIMEOUT, but a **project record** — closest approach **8.39 m** (first time below the historic 8.7 m floor), 32 cells explored, correct NE approach. Two objective failures, both confirmed by the user watching Gazebo/RViz:
> - **Collision rate 33.1 %** (117/354 DIAG poses inside the 0.47 m robot+wall contact envelope, `MazeSim` oracle). The robot's wheels physically caught in the `(4,9)/(5,9)` doorway corner.
> - **Cell desync 54.2 %** (192/354 ticks `dcell≠odomcell`). The planner operates on the wrong cell over half the time, so wall-marks and routing are mis-attributed → the "garbled explored-map memory" the user reported.
>
> Both share one mechanism: a **lateral pin at a doorway** (body drifts off-centerline, grazes the divider wall, the discrete `dcell` lags the body by one cell, the planner fights to backtrack down a column the body is no longer in). User directives: (1) increase the safety radius; (2) add a dedicated module to maintain/manage the explored-map memory.

**Goal:** Stop the doorway-corner collisions at the source (active wall repulsion) and stop the explored-map from being corrupted by control-failure pins (a dedicated `MapMemory` gateway that guards failure-marks and reconciles the planner cell to odom).

**Architecture:** Two independent components, both ROS-free and offline-sim-testable.
1. **Active keep-out** — add a *repulsion* term to the corridor follower so the robot steers *away* from a near side wall (today the keep-out only caps cross-steer and slows; it never repels), plus bump the safety radius `0.60 → 0.70`.
2. **`MapMemory` (targeted gateway)** — a new module wrapping `FloodFillBrain` that owns the two integrity behaviors the scattered marking lacks: (a) **persistent-desync odom reconcile** (snap `dcell→odomcell` when stuck-desynced), and (b) **mark-guard** (suppress failure-driven WALL marks that are lateral-pin control failures, not real walls). Sensing-driven marks in `_center` stay as-is (already quality-gated).

**Tech Stack:** Python 3.12, ROS 2 Jazzy (node is a thin adapter), pytest. Branch `no-progress-watchdog`. Diagnostics stay IN for the next validation run; the diagnostic-instrumentation commit `2fe9e39` is reverted only before any merge. Commit trailer ends with `Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>`. No backticks inside `git commit -m`.

---

## Component 1 — Active keep-out (repulsion + radius 0.70)

### Problem (confirmed by the keepout reader + 33.1 % oracle)
`corridor_drive_command` (`hop_controller.py:138-165`) is the keep-out. When `near_wall_m < safety_radius` it only:
- raises the cross-steer cap `0.35 → 0.8` (`keepout_max_cross_steer`), and
- slows toward `wedge_v_floor` (never zero).

Both act on **heading toward the centerline**; neither pushes the robot *past* center, *away* from the near wall. So at a 2.0 m doorway the body still reaches `near=0.31 m` (inside the 0.47 m contact envelope) while `perp_front≈4 m`. Raising the radius alone (the literal request) does **not** fix this — it only widens the reactive zone. **Decision (user-approved): repulsion + radius 0.70.**

### Design
Add a repulsion bias to the lateral target inside `corridor_follow_command` (`hop_controller.py:168-191`), which already has `d_left, d_right`. When the nearer side wall is within `safety_radius`, shift the centerline target *away* from it, proportional to the intrusion depth, then clamp as today. `centerline_cross` only converges to the geometric center; this adds an away-from-wall push so the robot doesn't graze the corner while creeping through.

Sign convention (from `hop_controller.py`): `cross` is `+` when the robot is LEFT of center; `corridor_drive_command` steers to *reduce* `cross` (so `+cross` ⇒ steer right). Therefore: near wall on the **left** (`d_left < d_right`) ⇒ add `+bias` (steer right, away); near wall on the **right** ⇒ add `−bias`.

New kwarg `keepout_repulse_gain: float = 0.6` on `corridor_follow_command` (and threaded as a `MazeMotion` attribute). Repulsion is computed *before* the existing `max_cross_track_m` clamp, so it can never exceed the proven 0.6 m lateral authority:

```python
# inside corridor_follow_command, replacing the current `cross = max(...clamp...)`:
cross = centerline_cross(d_left, d_right, fallback_cross=fallback_cross,
                         wall_seen_m=wall_seen_m, half_corridor_m=half_corridor_m)
# Active keep-out repulsion: a side wall within safety_radius pushes the centerline target
# AWAY from it (toward the open side), proportional to intrusion depth. centerline_cross
# only converges to CENTER; this repels so the robot won't graze the corner while creeping.
near_side = min(d_left, d_right)
if near_side < safety_radius:
    bias = keepout_repulse_gain * (safety_radius - near_side)
    cross += bias if d_left < d_right else -bias        # push toward the farther (open) side
cross = max(-max_cross_track_m, min(max_cross_track_m, cross))
```

**Why it won't destabilize normal driving:** in a centered 1.76 m corridor `d_left≈d_right≈0.88 > 0.70`, so `near_side ≥ safety_radius` and the bias is zero. Repulsion engages only within 0.70 m of a wall (off-center by >0.18 m, or a narrow doorway). It nudges *heading*; the pure-pursuit lookahead and the 0.6 m clamp bound the deviation.

**Radius bump:** `MazeMotion.__init__` `self.safety_radius = 0.60 → 0.70`; defaults in `corridor_drive_command` and `corridor_follow_command` `safety_radius: float = 0.60 → 0.70`. Leave `wedge_slow_m=0.60` (the slow-down profile is layered *inside* the repulsion zone: repel at 0.70, slow at 0.60, floor near 0.40).

### Files
- Modify: `tugbot_maze/hop_controller.py` — `corridor_follow_command` (add repulsion + `keepout_repulse_gain` kwarg); `safety_radius` default `0.60→0.70` in both command fns.
- Modify: `tugbot_maze/tugbot_maze/maze_motion.py` — `__init__`: `self.safety_radius = 0.70`, add `self.keepout_repulse_gain = 0.6`; thread `keepout_repulse_gain` into the `corridor_follow_command(...)` call (`maze_motion.py:582-591`).
- Test: `test/test_hop_controller.py`.

---

## Component 2 — `MapMemory` targeted gateway

### Responsibilities
A new `tugbot_maze/map_memory.py` with class `MapMemory(brain)`. It is the gateway for **failure-driven** map mutations and for **planner↔odom reconciliation**. It does NOT take over sensing-driven marks (`_center` `brain.mark`, `maze_motion.py:229-230`), which are already quality-gated (`good = aligned and pos_ok and not_straddling`) and trustworthy.

### 2a — Persistent-desync odom reconcile (kills the 54 % desync)
The bounded 1-cell hysteretic `_reanchor` (`maze_motion.py:250-262`) is the gentle path and stays. But at `(4,9)` the body sits at `x=9.23` — a boundary straddle (0.77 m from the `(5,9)` center, outside the 0.6 m core), so neither `_reanchor` nor a core-gated snap will ever update `dcell`. `MapMemory` adds a **forced fallback**: track how long `pose_to_cell(x,y)` has disagreed with `dcell`; once the disagreement persists `reconcile_persist_s` (default 8.0 s — far below the 90 s no-progress escape), snap `dcell` to the odom cell at the next safe settled point. Safe because `odom_locked` pose was rock-steady (not drifting) all run; the persistence gate guarantees only *stuck* desync triggers (normal hops resolve desync in <8 s).

```python
def observe(self, dcell, x, y, t):
    """Call every tick. Tracks how long the planner cell has disagreed with the odom cell."""
    odom = pose_to_cell(x, y)
    if (not in_grid(odom)) or odom == dcell:
        self._desync_since = None
    elif self._desync_since is None:
        self._desync_since = t

def reconcile_target(self, dcell, x, y, t):
    """Return the odom cell to adopt when the desync has persisted >= reconcile_persist_s,
    else dcell unchanged. Call only at a safe settled point (center phase). The bounded 1-cell
    hysteretic re-anchor remains the gentle path; this is the forced fallback for a stuck
    boundary-straddle (the (4,9) trap)."""
    odom = pose_to_cell(x, y)
    if (in_grid(odom) and odom != dcell and self._desync_since is not None
            and (t - self._desync_since) >= self.reconcile_persist_s):
        self._desync_since = None
        self.reconciles += 1
        return odom
    return dcell
```

Integration:
- `MazeMotion.step` (`maze_motion.py:145`, after `_track_cell`): `self.mem.observe(self.cell, pose[0], pose[1], t)`.
- `MazeMotion._center`, at the **very top** (right after `x, y, yaw = pose`, *before* the committed-cell fast-path at `maze_motion.py:174` — otherwise a committed straddle cell early-returns and never reconciles):
  ```python
  target = self.mem.reconcile_target(self.cell, x, y, t)
  if target != self.cell:
      self.events.append("RECONCILE %s -> %s (desync>%.0fs)" % (self.cell, target, self.mem.reconcile_persist_s))
      self.cell = target                                  # relabel only; committed wall-knowledge is unchanged
      self.hop_attempts.clear(); self.hop_dir = None
      self.center_start = None; self.align_start = None; self.latched_cardinal = None
  ```
  (Falls through to the existing fast-path / exit / committed / sense / route logic from the corrected cell.)

### 2b — Mark-guard on failure-driven WALL marks (protects the map)
The two `_drive` giveup sites (`maze_motion.py:535` wedge, `:563` front_block/deadline) mark `brain.mark(cell, dirn, True)` after `max_hop_attempts`. They already skip edges to *visited* cells, but an edge toward an *unvisited* cell the robot is straddling into (e.g. `(4,9)-E` while the body is in `(5,9)`) gets falsely walled. `MapMemory.mark_wall_on_failure` adds a **lateral-pin filter**: a failure with an *open front* but the robot *jammed against a side wall well off-centerline* is a control failure, not a wall — suppress it.

```python
def is_lateral_pin(self, perp_front, near, cross_track, safety_radius):
    """A hop failure is a lateral pin (control failure, NOT a wall) when the forward path is
    open but the robot is jammed against a side wall well off the corridor centerline."""
    return (perp_front is not None and perp_front > self.front_open_m
            and near is not None and near < safety_radius
            and abs(cross_track) > self.pin_cross_m)

def mark_wall_on_failure(self, cell, dirn, *, perp_front, near, cross_track, safety_radius):
    """Gateway for hop-failure WALL marks. Suppress (return False) when it's a lateral pin;
    otherwise mark the wall (return True)."""
    if self.is_lateral_pin(perp_front, near, cross_track, safety_radius):
        self.suppressed += 1
        return False
    self.brain.mark(cell, dirn, is_wall=True)
    return True
```

Defaults: `front_open_m=1.3` (an "open" forward reading; matches `wall_seen_m`), `pin_cross_m=0.5` (off-centerline beyond the normal `max_cross_track_m=0.6` regime), `reconcile_persist_s=8.0`.

Integration at both `_drive` giveup sites — replace the guarded `brain.mark(...) + _stamp_loco_wall(...)` block with a call through the gateway, computing `cross_track` the same way `_stall_event` does (`grid_cross_track`, `maze_motion.py:464`):
```python
nb = (self.cell[0] + DIRS[dirn][0], self.cell[1] + DIRS[dirn][1])
if nb not in self.visited:                                   # never wall an edge to a VISITED cell
    ct = grid_cross_track(x, y, self.cell, self.hop_dir, cell_size_m=CELL_SIZE_M)
    if self.mem.mark_wall_on_failure(self.cell, dirn, perp_front=perp[dirn], near=near,
                                     cross_track=ct, safety_radius=self.safety_radius):
        self._stamp_loco_wall(self.cell, dirn)               # re-openable by _unstick (both reps)
        self.committed.discard(self.cell)                    # un-commit; cell stays in `sensed`
```
Note the front_block site naturally passes the guard: `perp[dirn] < front_block_m (0.7) < front_open_m (1.3)` ⇒ `is_lateral_pin` is False ⇒ a genuine front wall is still marked. Only the *open-front* deadline/wedge giveups are suppressed.

**Out of scope (kept tight):** the `_escape` tier-2 mark (`maze_motion.py:413`) has no scan in scope and fires rarely (last-resort, reopenable by `_unstick`); leave it unchanged. The mark-guard + reconcile together resolve the `(4,9)` trap: suppress the false wall, then within 8 s snap `dcell→(5,9)` and re-plan eastward.

### Ownership
`FloodFillSolver.__init__` (`flood_fill_solver.py:57-58`) creates the brain, then `MapMemory(brain)`, and passes **both** into `MazeMotion(brain, mem=...)`. `MazeMotion.__init__` stores `self.mem` (constructs a default `MapMemory(self.brain)` if `None`, mirroring the brain default). Observables `mem.suppressed`, `mem.reconciles` are logged in DIAG.

### Files
- Create: `tugbot_maze/tugbot_maze/map_memory.py` — `MapMemory`.
- Modify: `tugbot_maze/tugbot_maze/maze_motion.py` — `__init__` (accept/store `mem`); `step` (`observe`); `_center` (`reconcile_target` + snap); both `_drive` giveup sites (route marks through `mark_wall_on_failure`).
- Modify: `tugbot_maze/tugbot_maze/flood_fill_solver.py` — create `MapMemory`, pass to `MazeMotion`; log `suppressed`/`reconciles` in DIAG.
- Test: `test/test_map_memory.py` (new), `test/test_maze_motion.py`, `test/test_maze_motion_sim.py`.

---

## Testing strategy (TDD)

**Unit — `MapMemory` (`test/test_map_memory.py`):**
- `is_lateral_pin` True for `(perp_front=4.0, near=0.31, cross=1.22, safety=0.70)`; False when front is short (`perp_front=0.6` real wall), when not near (`near=0.85`), and when on-centerline (`cross=0.1`).
- `mark_wall_on_failure` suppresses the lateral pin (brain edge stays non-WALL, `suppressed==1`) and marks a genuine front-block (brain edge == WALL).
- `observe`/`reconcile_target`: no snap before `reconcile_persist_s`; snap to the odom cell at/after it; counter resets when `dcell==odomcell`; `reconciles` increments on snap.

**Unit — repulsion (`test/test_hop_controller.py`):**
- Near right wall (`d_left=0.85, d_right=0.30`, safety 0.70) ⇒ returned `w` steers LEFT (more than the no-repulsion baseline). Symmetric for near-left.
- Both walls open/centered (`d_left=d_right=0.88`) ⇒ identical to no-repulsion (`bias==0`).
- Repulsion + clamp never exceeds the `max_cross_track_m` envelope.

**Integration — `MazeMotion` (`test/test_maze_motion.py`):**
- Reconcile snap: drive a straddle scenario (`cell=(4,9)`, pose `x=9.23`), step ≥ `reconcile_persist_s` of sim-time in center phase ⇒ `self.cell` becomes `(5,9)`, hop state cleared.
- Mark-guard: a wedge giveup with open front + side pin ⇒ `brain.is_wall(cell, dirn)` stays False; a wedge giveup with a real short front ⇒ marked True.

**Regression — offline sim (`test/test_maze_motion_sim.py`):** the existing `test_reaches_exit_without_collision_or_desync[...]` drift cases must still pass (esc/desync invariants). Run the full motion+sim+brain+hop+junction+cell+sim suite.

**Gazebo (user-initiated, diagnostics IN):** expect collision rate well below 33 %, desync well below 54 %, the `(4,9)`-class doorway resolved via reconcile within ~8 s instead of a multi-minute pin, and depth at least matching the 8.39 m record (ideally `EXIT_REACHED`). Watch `mem.suppressed` (false walls avoided) and `mem.reconciles` (forced snaps).

## Risks
- **Repulsion into an open junction:** at a junction the "open side" may be a perpendicular corridor; repulsion could nudge the robot toward it. Mitigated by the 0.6 m clamp + lookahead; the offline sim is the gate. If it appears, condition repulsion on both side walls being *seen* (`< wall_seen_m`).
- **Reconcile to a wrong cell under real odom corruption:** `odom_locked` keeps odom truthful in sim; the 8 s persistence + in-grid gate bound it. If a future run shows true odom drift, add a "pose moved consistently" check.
- **Suppressing a real wall:** the lateral-pin filter requires an *open* front (`perp_front>1.3`); a real wall reads short, so it is never suppressed.
