# Wedge-Detector Heading Gate — Design

**Status:** approved (design confirmed by instrumented diagnostic run `20260623_233508`).
**Scope:** one surgical change in `MazeMotion._drive`'s wedge detector + tests. Builds on the `no-progress-watchdog` branch (the watchdog is the backstop; the diagnostic instrumentation stays in for validation).

## 1. Confirmed diagnosis (from the instrumented run)

Across 35 stalls: **29 `wedge` / 6 `front_block`**. The `wedge` stalls (the dominant failure) had side `near` **median 1.30 m** (open junctions — only 2/29 < 0.50, so NOT tight/pinned), `|yaw_err|` **median 0.70 rad (~40°)**, and `last_v` **median 0.00**. Lateral centering was fine (`cross_track` median 0.08). Stalls clustered at the open junctions (1,3)/(2,3)/(1,4); the robot never escaped that cluster (best dist 18.29 m, identical to every SW-path run); 9 false-wall stamps + heavy `_unstick` churn; 8 watchdog escapes, no latch.

**Root cause (in `hop_controller.corridor_drive_command`):**
```
throttle = max(0, 1 - |yaw_err|/slow_angle)   # slow_angle=0.6 → v=0 once |yaw_err| >= 0.6 rad
v = v_max * throttle * wedge_factor ;  w = clamp(kp_ang * yaw_err)
```
When heading drifts past ~34°, the follower **correctly** sets `v=0` and turns in place to re-align. But `_drive`'s wedge detector measures **only positional movement**:
```
if hypot(x - progress_pose) > wedge_move_eps: reset baseline   # only POSITION resets it
elif (t - progress_t) > wedge_detect_s: -> WEDGE               # in-place rotation never resets -> fires at 3 s
```
So a legitimate in-place re-alignment (no positional progress) is **misread as a physical pin** after 3 s → false-wall giveup → map churn → escape cascade → never progresses. The fingerprint is unambiguous: `last_v≈0` + large `yaw_err` + open sides.

## 2. Fix

Gate the wedge timer so it accumulates **only when the robot is aligned and commanded forward** (a true pin), not when the follower is turning in place to re-align. In `_drive`'s wedge detector, add a middle branch keyed on the **heading error** (the direct cause of `v=0`):
```python
if hypot(x - self.progress_pose) > self.wedge_move_eps:                  # positional progress (unchanged)
    self.progress_pose = (x, y); self.progress_t = t
elif abs(_norm(self.target_cardinal - yaw)) >= self.wedge_realign_yaw:   # NEW: heading far off cardinal ->
    self.progress_pose = (x, y); self.progress_t = t                      #   follower turns in place (v~=0), not a pin
elif (t - self.progress_t) > self.wedge_detect_s:                        # aligned but not moving -> real pin
    ... wedge (unchanged) ...
```
- New tunable `wedge_realign_yaw: float = 0.5` (rad, ~29°) — at/above this the follower's heading term throttles `v` toward 0 (`slow_angle=0.6`), so no positional progress is *expected* (turning, not pinned); below it the robot is essentially aligned and a non-moving state IS a pin.
- **Signal choice (refined during TDD):** an earlier draft gated on the commanded velocity `self._last_drive_v < 0.05`, but that field is `0.0` until the first corridor command runs, so it spuriously fired on an *aligned* real pin whose drive hadn't yet computed `v` (caught by the existing `test_commit_freezes_sensing_not_map`). The heading error is the direct, lag-free signal and correctly separates an aligned pin (`|yaw_err|≈0` → wedge) from a mis-aligned re-align (`|yaw_err|` large → gate). `_last_drive_v` is retained only for the diagnostic `STALL` events.

## 3. Why this is correct / safe
- **Real pins preserved:** commanded `v ≥ wedge_min_drive_v` + no positional progress → wedges exactly as today.
- **Misfire eliminated:** legitimate in-place re-alignment no longer trips the timer → no false wall → no map corruption → no cascade.
- **No "turns forever" hole:** if the heading never converges (e.g., a bad pure-pursuit setpoint at the junction — the deferred approach-C cause), the no-progress watchdog (`no_progress_s=90` + confined) fires and escapes — bounded, and crucially WITHOUT corrupting the map. The diagnostic events stay in so the validation run reveals if spinning becomes the new bottleneck (→ then address approach C).

## 4. Tests
- `test_wedge_gate_skips_when_rotating_in_place`: in `drive`, MIS-ALIGNED (`target_cardinal` far from `yaw`, `|yaw_err| ≥ wedge_realign_yaw`), no positional progress, advance `t` past `wedge_detect_s` → wedge does NOT fire (`phase` stays `drive`, no `brain.mark`, `progress_t` advanced).
- `test_wedge_still_fires_on_real_pin`: ALIGNED (`yaw == target_cardinal`), no positional progress, advance past `wedge_detect_s` → wedge DOES fire (recover/giveup as today). (The existing `test_commit_freezes_sensing_not_map` also guards this: it is an aligned pin and must still wedge.)
- Offline regression `test_maze_motion_sim.py` stays green (the gate only suppresses a misfire the clean solve never triggers; the solve must still reach the exit, `escape_count==0`).

## 5. Out of scope (deferred)
- The heading-drift *cause* at junctions (approach C — likely the `grid_cross_track` fallback injecting a spurious pure-pursuit setpoint when side walls vanish). Addressed only if the validation run shows persistent in-place spinning after this gate.
- Reverting the diagnostic instrumentation (`2fe9e39`) — kept for the validation run; revert before any merge.
