# `_center` Phase Sensing-Convergence — Design (hardened)

**Date:** 2026-06-19
**Workspace:** `ros2_ws_tugbot_nav_20260614`
**Branch:** `tighter-corridor-controller` (continues the corridor-controller effort; local only, not pushed)
**Status:** Approved (brainstorm + adversarial design-hardening workflow), ready for implementation plan.

> This design was hardened by a 5-lens adversarial workflow
> (`harden-center-phase-design`, run `wf_2f764fac-27b`). The workflow **overturned the
> original intuition** (the cardinal-gate is *not* the cure) and surfaced a **critical
> regression risk** (naive commit can permanently brick the solve). Both are addressed below.

## 1. Problem & verified root cause

The tighter corridor controller fixed in-corridor wedging but the robot then **churns at
junction cell (3,4)**: 18 recover episodes, regressed to (1,5), never near the exit. From the
run's `SENSE`/`DIAG` logs the loop is:

`_drive`/`_recover` do `self.sensed.discard(self.cell)` → `_center` re-senses (3,4) from a
*different pose* → `sense_cell_walls` returns *different* N/S walls → `brain.mark` overwrites the
map → `next_cell` flips → the robot turns/drives a different direction → fails → recovers → repeat
(the ~1.2 m pose swings are the flip-flopping drives).

**Verified mechanism (corrected from the original framing):**

- The read instability is **position-driven, not yaw-driven.** `cell_wall_perp_dist`
  (projection-median) is **yaw-invariant** given accurate odom yaw — the yaw cancels in the
  projection. The off-cardinal yaws in the evidence (−1.27/−3.06/2.86) are *not* what flips N/S.
  The flicker is the **travel-axis position**: pose `y` swung 7.0–8.2, straddling the **y=7.0
  (3,3)/(3,4) cell boundary** (`pose_to_cell` uses `round(y/2.0)`); E/W stay stable, N/S flip —
  the y-axis fingerprint.
- **Bug enabler in code:** `_center` shares one `center_start` between the centering gate
  (`maze_motion.py` ~L117) and the yaw-snap gate (~L126), both guarding
  `(t - center_start) < center_timeout_s`. When centering can't converge (`off≈0.3 > center_tol
  0.10`) it burns the whole 4 s, the snap branch is skipped, and `sense_cell_walls` runs at the
  off-pose arrival — and `sensed.discard` lets it repeat from new poses.
- **Permanence hazard:** `brain.mark` and the `_drive` hop-failure path only ever *add* WALL
  edges (monotonic); a WALL edge has **no locomotion self-correction** (the robot never drives
  into a wall to learn it's open). `next_cell==None` → `phase='stuck'` returns `(0,0,False)`
  forever. So any mechanism that *freezes* a wrong WALL read can make the maze unsolvable.

## 2. Decision

**Proceed with the sense-commit fix, plus position-gating and map-integrity backstops.**

- **Sense-once / commit** stops the churn *loop* (verified: making `sensed.discard` inert freezes
  the map → `next_cell` deterministic). This is necessary and is the load-bearing change.
- It is **not sufficient for read correctness**: committing must be gated on **position** (the
  binding criterion), not yaw, and never on `centered==True` alone (degenerate at open
  junctions, where `cell_center_offset` returns `None` and `centering_command` reports done
  trivially).
- Commit must be **reversible under stuck** or it can brick the solve. Backstops added.

Rejected: *proceed-as-is* (position-blind commit + permanent false-wall) and *rethink* (the core
mechanism is sound once gated + backstopped).

## 3. `_center` redesign

All changes are in `MazeMotion._center` plus a small, justified touch to `_drive` (§4b) and new
`__init__` state/params. `_turn`, `_recover`, `corridor_follow_command`, `profiled_turn_command`,
sensing primitives, and the brain are otherwise unchanged.

Per-tick stages:

**3a. Committed fast-path.** If `self.cell in self.committed`: do the hysteresis re-anchor (3e),
then route (3h) — **skip centering, align, and sensing entirely** (no re-run of the 4 s
centering, no added motion on revisits).

**3b. Lateral centering.** Unchanged: `ox,oy = cell_center_offset(...)`;
`v,w,centered = centering_command(...)`; if `not centered and (t-center_start) < center_timeout_s`
→ `settle_until = t+settle_s`, return `(v,w,False)`.

**3c. Cardinal align (pose-freeze, not the commit criterion).** Latch the nearest cardinal
**once** at align entry into `self.latched_cardinal` (avoids 45° basin jitter from recomputing
`round(yaw/(pi/2))` each tick). Use an **independent** `align_start` (separate from
`center_start`). Rotate **in place**:
`w = profiled_turn_command(yaw, self.latched_cardinal, self.yaw_rate, ang_decel=self.ang_decel,
turn_w_max=self.turn_w_max, kd=self.kd_turn)`; if `|norm(latched_cardinal-yaw)| > yaw_tol_rad and
(t-align_start) < align_timeout_s` → `settle_until = t+settle_s`, return `(0.0, w, False)`.
Value: rotating in place (v=0) **freezes position** during the read; it does not change the
computed walls (yaw cancels) — it is pose-freeze + envelope insurance.

**3d. Settle.** If `t < settle_until` → return `(0,0,False)` (let motion stop before sensing).

**3e. Re-anchor with hysteresis.** `anchored = pose_to_cell(x,y)`. Adopt it only if
`in_grid(anchored)` and Manhattan distance to `self.cell` ≤ 1 **and** (`anchored==self.cell` **or**
the pose is *clearly inside* `anchored` — on the differing axis, `|coord − 2*anchored[axis]| ≤
(1.0 − boundary_margin_m)`). This prevents `self.cell` (and thus `next_cell`) flipping while the
robot sits on a cell boundary. If the re-anchor lands on a committed cell → committed fast-path.
If `self.cell == EXIT_CELL` → done.

**3f. Quality gate (binding = POSITION).** Compute `aligned = |norm(latched_cardinal−yaw)| ≤
yaw_tol_rad`. Recompute `ox,oy = cell_center_offset(...)` *at the sense tick* (atomic with the
sense). Define:
- `pos_ok` = every **referenced** axis (non-`None` from `cell_center_offset`) has `|offset| ≤
  commit_offset_tol`. (Open axes — `None` — impose no constraint; they cannot be referenced.)
- `not_straddling` = on the **travel axis** (`'x'` if `hop_dir[0]≠0` else `'y'`), the pose is
  `≥ boundary_margin_m` from the nearest cell boundary, i.e. `|coord − 2*cell[axis]| ≤ (1.0 −
  boundary_margin_m)`.
- `good = aligned and pos_ok and not_straddling`.

**3g. Sense / commit / corroborate / poor.** Re-sense eligibility:
`should_sense = (self.cell not in committed) and (self.cell not in sensed or good)`.
- If `should_sense`: `walls = sense_cell_walls(...)`; `brain.mark` each dir; `sensed.add(cell)`;
  record dbg (§6).
  - If `good`: corroboration across reads — if `walls == self.corrob_walls and self.corrob_cell
    == cell` then `corrob_count += 1` else `(corrob_cell, corrob_walls, corrob_count) = (cell,
    walls, 1)`. If `corrob_count ≥ K_corroborate` → `committed.add(cell)`.
  - If not `good`: record the read (the brain is marked so the robot can route) but do **not**
    commit. The cell stays re-sensable **only via a future good read**.
- **Crucially:** a **poor re-entry of an already-sensed cell does NOT re-sense** (`should_sense`
  is False because the cell is in `sensed` and the read is not `good`) — it routes on the existing
  map. This is what bounds churn: bad poses cannot repeatedly overwrite the map. A wrong poor read
  that *disconnects* the map is caught by the §4a deadlock backstop; a false-OPEN self-corrects via
  the locomotion wall-mark (§4b). A good re-entry (rare second chance) re-senses and corroborates.
  (No separate poor-attempt cap is needed — the skip-on-poor-re-entry rule already bounds it.)

**3h. Route.** `nxt = brain.next_cell(self.cell)`. If `nxt is None` → §4a deadlock backstop. Else
`if nxt != self.hop_target: brain.mark_traversal(self.cell, nxt)`; set `hop_target/hop_dir/
target_cardinal/hop_start`, reset turn/center/align state, `phase='turn'`.

## 4. Map-integrity safeguards

**4a. Deadlock backstop (before terminal `stuck`).** When `next_cell(self.cell) is None`: if the
cell is in `committed` **or** has locomotion-marked walls (§4b), then **evict** it from
`committed` and `sensed`, **re-open** its locomotion-marked WALL edges (`brain.mark(cell,dir,
is_wall=False)` for each recorded `(cell,dir)` — optimistic; the forced re-sense overwrites with
truth), clear its corroboration/poor state, reset `center_start`/`align_start` so the next tick
re-senses, and return `(0,0,False)`. Declare `phase='stuck'` only if `next_cell` is still `None`
*after* a fresh good re-sense (track an `evicted_once` guard per cell to avoid infinite eviction).

**4b. Un-commit on hop-failure wall-mark.** Record every WALL stamped by locomotion failure (the
`_drive` `brain.mark(cell,dir,is_wall=True)` after `max_hop_attempts`, ~L213/L234) in
`self.locomotion_walls` as `(cell,dir)`; and if `cell in committed`, remove it from `committed`
so a fresh sense can confirm/deny that edge. (Small touch to the two `_drive` mark sites.)

**4c. Invariant.** Commit freezes **sensing only**, never the map: locomotion may still overwrite
a committed-OPEN edge (§4b handles the un-commit). Unit-test pins this.

## 5. State & lifecycle

New `MazeMotion` state (in `__init__`): `self.committed = set()`,
`self.align_start = None`, `self.latched_cardinal = None`, `self.corrob_cell = None`,
`self.corrob_walls = None`, `self.corrob_count = 0`,
`self.locomotion_walls = set()`, `self.evicted = set()`.

**Reset discipline:** `align_start` and `latched_cardinal` follow the exact lifecycle of
`center_start` — set lazily on align entry, **cleared (`None`) at every site `center_start` is
cleared**: `__init__` and the five `self.center_start = None` sites (end of `_center` route, the
`_turn`→drive handoff, `_drive` arrival→center, `_drive` front-block→center, `_recover`→center).
(Verify the exact lines during implementation; the invariant is "wherever `center_start` resets,
`align_start`/`latched_cardinal` reset too.") `sensed` is retained as the re-sensable layer the
three `discard` sites clear; `committed` is the freeze layer the sense gate now keys on.

## 6. Diagnostics

`self.dbg` at the sense tick records: `good` (bool), `committed`/`resensed` status, `|yaw_err|`,
the per-axis offsets `(ox,oy)`, `pos_ok`/`not_straddling`, and the committed wall-set. The node
logs these under `sense_debug`, so the Gazebo (3,4) audit can confirm the cell was committed via
the **good** path with the **ground-truth** wall-set (not repeatedly re-sensed via the poor path).

## 7. Validation plan

**Offline 91-test suite stays green** — necessary but **NOT sufficient**: offline dynamics let
centering converge, so the bug never reproduces (the suite is already green *without* this fix).

**New unit tests** (drive `MazeMotion._center` with crafted poses/scans, or via a synthetic
`maze_sim` corridor):
1. A `good` (aligned + in-position + non-straddling) read commits after `K_corroborate` agreeing
   reads; a subsequent `sensed.discard(cell)` is **inert** (walls frozen, `next_cell` stable).
2. A `poor` (off-position / straddling) read is **not** committed and the cell stays re-sensable;
   a poor re-entry of an already-sensed cell does **not** re-sense.
3. Sensing across `±yaw_tol` and small along-travel-axis offsets asserts **no false WALL**.
4. An injected single bad junction read does **not** permanently lock a false wall: when it makes
   `next_cell==None`, the §4a backstop evicts + re-senses (backstop test).
5. Invariant: commit freezes sensing only — a locomotion last-resort wall-mark still un-commits
   the cell (§4b/§4c).

**Gazebo:** run **multiple seeded runs** (a single run can pass by luck given the documented
run-to-run non-determinism). Each: `tools/run_flood_fill_maze.sh 1800 false true odom_locked
true`. Assert: the committed (3,4) wall-set matches ground truth, the churn is gone (recover
episodes near zero at (3,4)), the robot progresses past (3,4), and log the odom-yaw error to
confirm/deny Change-1's (minor) contribution. Clear stray sims via the `!`-prefix pkill first.

## 8. Tuning knobs & defaults

| param | default | rationale |
|---|---|---|
| `yaw_tol_rad` | 0.10 | existing |
| `align_timeout_s` | 6.0 | generous heading-alignment backstop (separate from center) |
| `commit_offset_tol` | 0.40 m | inside the 0.45 m no-false-wall sensing envelope, yet passes the observed 0.30–0.34 m so it doesn't deadlock |
| `boundary_margin_m` | 0.40 m | straddle + re-anchor hysteresis margin from the cell boundary |
| `K_corroborate` | 2 | guards the ±yaw_tol / borderline-offset band the offline suite never tests |
| `center_timeout_s` | 4.0 | existing |

## Open risks (watch in the Gazebo runs)

- If interior odom yaw is accurate (memory: drift ~0.25 m, yaw small), Change-1 contributes little
  to read *correctness* (yaw cancels) — the position gate + corroboration are the real levers.
- Run-to-run non-determinism: only multi-run + ground-truth assertion on the committed (3,4)
  wall-set is conclusive that the commit is *correct*, not just lucky.
- Position-gate tuning: too tight → chronic poor-path/deadlock; too loose → frozen-wrong commit.
  0.40 m is the starting compromise; confirm from runs.
- Cumulative junction dwell (align ~6 s on top of center ~4 s per first visit) can pressure
  `hop_timeout_s`/episode budgets; watch for new timeout-driven stalls.

## Constraints

- Local only — **never push** to origin.
- Commit trailer: `Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>`.
- Agent `pkill`/`kill` is hook-blocked; the user clears stray Gazebo sims via the `!` prefix.
- Foreground `sleep` blocked; background only. Shell cwd not stable — use absolute paths / `git -C`.
