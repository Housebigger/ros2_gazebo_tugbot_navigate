# Wedge-Detector Heading Gate — Implementation Plan

> **For agentic workers:** single tightly-coupled task → execute inline with TDD (superpowers:executing-plans), not subagent-driven. Steps use checkbox (`- [ ]`).

**Goal:** Stop `_drive`'s wedge detector from misreading a legitimate in-place re-alignment (follower commands `v≈0` to turn) as a physical pin. Pairs with spec `docs/superpowers/specs/2026-06-24-wedge-detector-heading-gate-design.md`.

**Conventions:**
- `PKG = /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260614/src/tugbot_maze`
- `REPO = /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate`
- Branch `no-progress-watchdog` (already checked out; the wedge-gate builds on the watchdog backstop + the diagnostic instrumentation). **Never push.** Stage only this task's files. Commit trailer `Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>`; no backticks in `-m`.
- Test: `cd "$PKG" && PYTHONPATH="$PKG" python3 -m pytest <path> -v`. Helper `_two_exit_brain` already exists in `test/test_maze_motion.py`.

---

## Task 1: Gate the wedge detector on commanded forward velocity

**Files:** Modify `tugbot_maze/maze_motion.py`; Test `test/test_maze_motion.py`.

- [ ] **Step 1: Write the failing tests** — append to `test/test_maze_motion.py`:

```python
def test_wedge_gate_skips_when_rotating_in_place():
    # MIS-ALIGNED (yaw far from cardinal): the follower zeroes v to turn in place and re-align;
    # no positional progress must NOT be misread as a pin.
    b = _two_exit_brain((5, 5), 'N', 'S')
    m = MazeMotion(b)
    m.cell = (5, 5); m.hop_dir = (0, 1); m.hop_target = (5, 6)
    m.target_cardinal = math.pi / 2                        # hop is N; robot points E (yaw=0) -> |yaw_err|=pi/2
    m.phase = 'drive'; m.hop_start = (10.0, 10.0); m.hop_deadline = 1e12
    m.progress_pose = (10.0, 10.0); m.progress_t = 0.0
    sim = MazeSim(load_segments(), cell_center((5, 5)), 0.0)
    m._drive((10.0, 10.0, 0.0), _scan_at(sim), m.wedge_detect_s + 5.0)  # moved=0 (<0.3) -> no front_block
    assert m.phase == 'drive'                               # NOT wedged (no recover/center)
    assert not b.is_wall((5, 5), 'N')                       # no false wall stamped
    assert m.progress_t == m.wedge_detect_s + 5.0           # baseline reset (re-aligning is legit)


def test_wedge_still_fires_on_real_pin():
    # ALIGNED (yaw == cardinal) but not moving -> a genuine pin must still wedge.
    b = _two_exit_brain((5, 5), 'N', 'S')
    m = MazeMotion(b)
    m.cell = (5, 5); m.hop_dir = (1, 0); m.hop_target = (6, 5)
    m.target_cardinal = 0.0                                # hop is E; robot points E (yaw=0) -> |yaw_err|=0
    m.phase = 'drive'; m.hop_start = (10.0, 10.0); m.hop_deadline = 1e12
    m.progress_pose = (10.0, 10.0); m.progress_t = 0.0
    sim = MazeSim(load_segments(), cell_center((5, 5)), 0.0)
    m._drive((10.0, 10.0, 0.0), _scan_at(sim), m.wedge_detect_s + 5.0)
    assert m.phase in ('recover', 'center')                # wedge fired (recover; center if it gave up)
```

- [ ] **Step 2: Run to verify they fail**

Run: `cd "$PKG" && PYTHONPATH="$PKG" python3 -m pytest test/test_maze_motion.py -k "wedge_gate or wedge_still" -v`
Expected: `test_wedge_gate_skips_when_rotating_in_place` FAILS (current code wedges on the rotating-in-place case → `phase` becomes `recover`, baseline not reset, possibly a wall). `test_wedge_still_fires_on_real_pin` likely already PASSES (real pin wedges today) — that's fine, it guards the regression.

- [ ] **Step 3: Implement** — in `tugbot_maze/maze_motion.py`:
  (a) Add the tunable to `__init__`, next to the diagnostic `self._last_drive_v` line:
  ```python
          self.wedge_realign_yaw = 0.5          # |yaw_err|>=this => follower turns in place (v~=0), not a pin
  ```
  (b) In `_drive`'s wedge detector, insert the gating branch (keyed on HEADING ERROR — the direct cause of `v=0`; `_last_drive_v` is `0.0`-until-first-command and misfires on an aligned pin) between the positional-progress reset and the timeout:
  ```python
          if math.hypot(x - self.progress_pose[0], y - self.progress_pose[1]) > self.wedge_move_eps:
              self.progress_pose = (x, y); self.progress_t = t
          elif abs(_norm(self.target_cardinal - yaw)) >= self.wedge_realign_yaw:  # heading far off cardinal =>
              self.progress_pose = (x, y); self.progress_t = t  # follower turns in place (v~=0), not a pin -> no false wedge
          elif (t - self.progress_t) > self.wedge_detect_s:
              ...  # unchanged wedge giveup/recover
  ```

- [ ] **Step 4: Run to verify they pass + no regression**

Run: `cd "$PKG" && PYTHONPATH="$PKG" python3 -m pytest test/test_maze_motion.py test/test_maze_motion_sim.py -q`
Expected: PASS (2 new + all prior unit + the offline regression — the gate only suppresses a misfire the clean solve never triggers; the solve still reaches the exit with `escape_count==0`).

- [ ] **Step 5: Commit**

```bash
git -C "$REPO" add ros2_ws_tugbot_nav_20260614/src/tugbot_maze/tugbot_maze/maze_motion.py ros2_ws_tugbot_nav_20260614/src/tugbot_maze/test/test_maze_motion.py
git -C "$REPO" commit -m "fix: wedge detector ignores in-place re-alignment (v~=0), not a pin

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Validation (after the task)
1. Full ROS-free suite green: `cd "$PKG" && PYTHONPATH="$PKG" python3 -m pytest test/ -q`.
2. Rebuild + Gazebo (user-initiated; diagnostic STALL/ESCAPE events kept IN for this run): `colcon build --packages-select tugbot_maze`, then `tools/run_flood_fill_maze.sh 1800 false true odom_locked true`. Watch: the (1,3)/(2,3)/(1,4) junction cluster no longer produces `wedge` stalls / false-wall stamps; the robot threads the junctions and progresses past 18.29 m. If `STALL reason=wedge` recurs with `last_v≈0` it means the gate missed; if the robot now SPINS in place at junctions (no wedge, no progress, watchdog fires) → the heading-drift cause (approach C) is the next target.
3. Do not merge/push — banking is a separate user-approved step; revert the diagnostic commit `2fe9e39` before any merge.
