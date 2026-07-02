# Reverse-to-center for along-heading overshoot — design

**Date:** 2026-07-03
**Workspace:** `ros2_ws_tugbot_nav_20260614`
**Goal:** Eliminate the residual ~0.33 % true-footprint graze at cell **(3,9)** *without regressing* the 16/16 autonomous completion — by fixing the centering primitive to null an along-heading overshoot with a **reverse translation** instead of a 180° in-place rotation.

---

## 1. Problem & diagnosis (measured, not assumed)

The lone residual collision is a ~0.33 % rear-gripper graze, all at cell **(3,9)**. A controlled offline replay of the true-footprint oracle (`maze_sim.collides(x,y,yaw)`) over every DIAG sample of the two clean 16/16 Gazebo batches (`batch_diag_20260628_232253`, `batch_diag_20260629_071328`) reproduced it exactly:

- **6 / 1808 samples collide = 0.332 %** (matches the documented 0.33 %). **All 6 at (3,9).**
- The trajectory is identical across **all 16 runs**:
  1. Robot hops **north** from (3,8) into (3,9) and **consistently overshoots to y ≈ 18.38–18.41** (cell centre y = 18.0; north perimeter-wall face y = 18.88). Cross-track (x) is perfectly centred (6.04).
  2. `centering_command` must correct this **+0.40 m north** offset. Its rule is: face the map cardinal that lets it drive *forward* to reduce the offset, then drive. To reduce a **north** offset it picks `want = south (−90°)` and **rotates ~180° in place** (arrival heading is north, +90°).
  3. That in-place ~180° rotation, executed while still ~0.4 m north of centre, sweeps the **asymmetric rear gripper** (corner radius √(0.468²+0.292²) ≈ **0.552 m**) through the north wall face (0.48 m away). Transient swept graze.

### Oracle confirmation of the mechanism (`maze_sim.collides`)

| pose | collides |
|---|---|
| y=18.40, heading **north** (+90°) — rotation start | **False** |
| y=18.40, heading **south** (−90°) — rotation end | **False** |
| y=18.40, **mid-rotation** (yaw in the wall-facing arc) | **True** ← the graze |
| **reverse** straight south 18.40→18.00 at fixed north heading | **False all the way** |
| y=18.00 (centred), **any** heading rotate-in-place | **False for all headings** |

**Root cause:** an along-travel arrival **overshoot** that the centering primitive corrects by **rotating 180°** — and rotating while off-centre toward a near wall is exactly the grazing maneuver. It is a **circular dependency**: centering needs to face the axis, but facing-the-axis-by-rotating (while off-centre) is what grazes.

This also explains why the earlier **rotation-sweep-clearance-guard** *worsened* collisions (0.33 %→4.43 %, branch parked): it gated in-place rotations, but the culprit rotation *is the one centering performs to correct the overshoot* — gating it pinned the robot off-centre and rotated it more. The recorded lesson ("target centering, not rotation-gating") is refined here to: **center via reverse translation, not via rotation.**

## 2. The fix

A differential-drive robot cannot strafe, **but it can reverse**. When the axis to be corrected is (anti-)parallel to the current heading, drive **backward** along the heading to null the offset — a pure translation with no rear-gripper sweep — instead of rotating 180° to face the other way.

### Change (single function): `hop_controller.centering_command`

Today (`hop_controller.py:23–53`) it:
1. picks the larger out-of-tol axis `(axis, off)`;
2. computes `want` = the heading to drive **forward** and reduce `off`;
3. if `|_norm(want − yaw)| > yaw_tol` → **rotate toward `want`** (returns `(0, w, False)`);
4. else → **drive forward** `v = min(v_max, max(v_min, kp_lin·|off|))`.

New rule — choose the drive direction by the robot's alignment to the correction axis, using
`d` = unit vector of desired motion toward centre (axis-x: `(−sign(off),0)`; axis-y: `(0,−sign(off))`),
`f` = heading unit vector `(cos yaw, sin yaw)`, and `a = f·d`:

- **`|a| ≥ cos(perp_tol)` (heading ~parallel to the axis):** drive **`v = sign(a)·speed`, `w = 0`** — forward when `a>0`, **reverse when `a<0`**. No rotation.
- **else (heading ~perpendicular to the axis):** rotate to face `d` (i.e. face `want`), then drive — **unchanged** lateral-centering behavior.

`speed = min(v_max, max(v_min, kp_lin·|off|))`. `perp_tol` splits parallel-vs-perpendicular; **`perp_tol = 45°` (`|a| ≥ 0.707` ⇒ drive)**. Centering runs after a cardinal drive, so the heading is near-cardinal and `a` is near ±1 (parallel ⇒ drive/reverse) or near 0 (perpendicular ⇒ rotate) — the 45° split is unambiguous in practice.

The `done` criterion (all referenced axes within `tol`) and the perpendicular/lateral path are unchanged. The only behavioral change is: **the "offset is behind me along my heading" case reverses instead of rotating 180°.**

### Why this is safe (cannot walk into a wall)

Reverse triggers **only** when centre is *behind* the robot along its heading — i.e. the robot **overshot in its travel direction**. Reversing therefore retreats into the corridor it *just drove through*, which is clear by construction. Oracle-verified above (reverse 18.40→18.00 is collision-free; rotation at 18.00 is collision-free for all headings). It is self-limiting: each tick recomputes `(ox,oy)`; the axis drops out of `cands` at `tol` and centering returns `done`. The reverse (~0.4 m) is **faster** than the old 180°-rotate-then-drive, so `center_timeout_s = 4.0` is ample (the old path sometimes *timed out mid-rotation*, leaking the overshoot into the turn — this removes that failure too). No runtime collision oracle is introduced.

## 3. Scope

**In scope:** `centering_command` drive-direction selection (`hop_controller.py`), and its unit tests.

**Out of scope / non-goals:**
- The committed-cell fast-path (`_center` lines 186–192) that skips centering — **not implicated**: all 6 grazes are on the first productive, *uncommitted* pass through (3,9); a committed boundary cell was already sensed centred.
- Changing arrival deceleration / hop-stop tuning (the "kill the overshoot at source" alternative) — broader blast radius on every hop, higher regression risk to 16/16; rejected.
- Adding a runtime true-footprint oracle / swept-collision guard (the parked-branch approach) — unnecessary given the traversed-corridor safety invariant.

## 4. Testing & validation (each gates the next)

1. **Unit (`test_hop_controller.py`):**
   - **New:** the reverse case — heading anti-parallel to the correction axis returns `v < 0, w ≈ 0` (e.g. `centering_command((2.3,2.0,0.0), 0.3, None)` — facing east, centre to the west → **reverse**, not rotate).
   - **Update:** the one existing case that encodes the old 180°-rotate behavior for exactly this configuration (`(2.3,2.0,0.0), ox=0.3`) to assert the reverse. The other 6 cases are unaffected (verified: forward-aligned and perpendicular cases unchanged).
2. **Offline true-footprint regression (`maze_sim` + `MazeMotion`, `test_maze_motion_sim.py`):** drive a (3,9)-style *arrival-overshoot → center → turn* end-to-end through the offline oracle and assert **no `collides` sample** (the pre-fix arc grazed). Full existing offline suite stays green, including the end-to-end maze solve.
3. **Committed offline replay tool (`tools/replay_collision_oracle.py`):** promote the ad-hoc replay (`/tmp/replay_graze_diag.py`) to a committed tool — parses run-dir DIAG samples, runs `maze_sim.collides`, reports rate + per-cell/per-phase graze geometry. Makes the acceptance number reproducible (the 0.33 % was previously computed ad-hoc).
4. **Gazebo acceptance (controlled batch, ≥ 8 runs, user sets up the run):**
   - **Completion must stay 16/16-equivalent** (every run `EXIT_REACHED`).
   - **Oracle-replay collision rate ≤ 0.33 %**, target **0 grazes at (3,9)**.
   - **Abandon the fix if either regresses** — same honest gate that retired the previous guard.

## 5. Files

- **Modify:** `src/tugbot_maze/tugbot_maze/hop_controller.py` — `centering_command` drive-direction selection.
- **Modify:** `src/tugbot_maze/test/test_hop_controller.py` — add reverse test; update the one rotate-expecting case.
- **Modify/add:** `src/tugbot_maze/test/test_maze_motion_sim.py` — (3,9) overshoot→turn no-collision regression.
- **Add:** `tools/replay_collision_oracle.py` — committed offline oracle replay for acceptance.

## 6. Success criteria

Autonomous completion remains **16/16**; the (3,9) rear-gripper graze goes to **0** (overall oracle collision rate ≤ 0.33 %, targeting 0); change confined to the centering drive-direction rule; offline suite green. If Gazebo shows any completion or collision regression, the change is abandoned and parked (as with the prior guard), and this document updated with the honest outcome.
