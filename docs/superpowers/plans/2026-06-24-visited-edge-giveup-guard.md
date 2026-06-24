# Visited-Edge Giveup Guard (+ cap loosen) — Implementation Plan

> Execute inline with TDD. Pairs with spec `docs/superpowers/specs/2026-06-24-visited-edge-giveup-guard-design.md`. Branch `no-progress-watchdog` (on it). `PKG`/`REPO` as before. Never push; stage only each task's files; trailer `Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>`; no backticks in `-m`.

## Task 1: Fix 1 — never wall an edge to a visited cell

**Files:** `tugbot_maze/maze_motion.py`; Test `test/test_maze_motion.py`.

- [ ] **Step 1: Failing test** — append to `test/test_maze_motion.py`:
```python
def test_giveup_does_not_wall_edge_to_visited_cell():
    # The robot came from (2,4); a failed hop back toward it must NOT wall the (visited) edge.
    b = _two_exit_brain((3, 4), 'W', 'S')                  # (3,4) open W/S (W -> (2,4))
    m = MazeMotion(b)
    m.cell = (3, 4); m.hop_dir = (-1, 0); m.hop_target = (2, 4); m.target_cardinal = math.pi
    m.visited = {(2, 4), (3, 4)}                            # (2,4) already visited
    m.max_hop_attempts = 1                                  # this failure reaches the giveup cap
    m.phase = 'drive'; m.hop_start = (6.0, 8.0); m.hop_deadline = 0.0
    sim = MazeSim(load_segments(), cell_center((3, 4)), 0.0)
    m._drive((6.4, 8.0, 0.0), _scan_at(sim), 1.0)          # moved 0.4 (<arrive), t>=deadline -> giveup
    assert not b.is_wall((3, 4), 'W')                       # edge to the VISITED cell must stay OPEN


def test_giveup_still_walls_edge_to_unvisited_cell():
    b = _two_exit_brain((3, 4), 'W', 'S')
    m = MazeMotion(b)
    m.cell = (3, 4); m.hop_dir = (-1, 0); m.hop_target = (2, 4); m.target_cardinal = math.pi
    m.visited = {(3, 4)}                                    # (2,4) NOT visited
    m.max_hop_attempts = 1
    m.phase = 'drive'; m.hop_start = (6.0, 8.0); m.hop_deadline = 0.0
    sim = MazeSim(load_segments(), cell_center((3, 4)), 0.0)
    m._drive((6.4, 8.0, 0.0), _scan_at(sim), 1.0)
    assert b.is_wall((3, 4), 'W')                          # unvisited-cell edge still walled (unchanged)
```
- [ ] **Step 2: Run → FAIL** (`test_giveup_does_not_wall...` fails: current code walls regardless): `cd "$PKG" && PYTHONPATH="$PKG" python3 -m pytest test/test_maze_motion.py -k "giveup_does_not_wall or giveup_still_walls" -v`
- [ ] **Step 3: Implement** — in `maze_motion.py`, guard the wall-mark in all THREE giveup sites with a visited check. Pattern (define `nb` then guard `brain.mark`+`_stamp_loco_wall`+`committed.discard`):
  (a) `_drive` wedge-giveup `if marked:` block — wrap the three wall lines:
  ```python
              if marked:
                  nb = (self.cell[0] + DIRS[dirn][0], self.cell[1] + DIRS[dirn][1])
                  if nb not in self.visited:                           # never wall an edge to a visited cell
                      self.brain.mark(self.cell, dirn, is_wall=True)
                      self._stamp_loco_wall(self.cell, dirn)           # re-openable by _unstick (both reps)
                      self.committed.discard(self.cell)                # un-commit; cell stays in `sensed`
                  self.hop_attempts.pop(key, None)                     # reset per-edge counter (re-route/re-approach)
                  self.failed_hops.pop(key, None)
                  self.phase = 'center'
              else:
                  self.recover_until = t + self.recover_s
                  self.phase = 'recover'
  ```
  (b) `_drive` front_block/deadline-giveup `if marked:` block — same `nb` guard around the three wall lines; keep `hop_attempts.pop`/`failed_hops.pop` after.
  (c) `_escape` Tier-2 `if gave_up:` block:
  ```python
          if gave_up:
              dirn = _dir_name(self.hop_dir)
              nb = (self.cell[0] + DIRS[dirn][0], self.cell[1] + DIRS[dirn][1])
              if nb not in self.visited:                           # never give up an edge to a visited cell
                  self.brain.mark(self.cell, dirn, is_wall=True)
                  self._stamp_loco_wall(self.cell, dirn)
                  self.committed.discard(self.cell)
                  self.failed_hops.pop((self.cell, dirn), None)
              else:
                  gave_up = False                                  # visited edge stays open (accurate logging)
  ```
- [ ] **Step 4: Run → PASS + regression**: `cd "$PKG" && PYTHONPATH="$PKG" python3 -m pytest test/test_maze_motion.py test/test_maze_motion_sim.py -q`
- [ ] **Step 5: Commit** `maze_motion.py`, `test/test_maze_motion.py` — `fix: never wall a locomotion edge to a visited cell (no self-walling / severed reverse)`

## Task 2: Fix 2 — loosen max_cross_steer 0.25 -> 0.35

**Files:** `tugbot_maze/hop_controller.py`, `tugbot_maze/maze_motion.py`; Test `test/test_hop_controller.py`.

- [ ] **Step 1: Failing test** — append to `test/test_hop_controller.py`:
```python
def test_max_cross_steer_default_loosened():
    import inspect
    from tugbot_maze.hop_controller import corridor_drive_command, corridor_follow_command
    assert inspect.signature(corridor_drive_command).parameters['max_cross_steer'].default == 0.35
    assert inspect.signature(corridor_follow_command).parameters['max_cross_steer'].default == 0.35
```
- [ ] **Step 2: Run → FAIL** (default still 0.25): `... -k max_cross_steer_default -v`
- [ ] **Step 3: Implement** — change `max_cross_steer: float = 0.25` to `= 0.35` in `corridor_drive_command` and `corridor_follow_command`; change `self.max_cross_steer = 0.25` to `= 0.35` in `maze_motion.py.__init__`.
- [ ] **Step 4: Run → PASS + regression**: `cd "$PKG" && PYTHONPATH="$PKG" python3 -m pytest test/test_hop_controller.py test/test_maze_motion.py test/test_maze_motion_sim.py -q`
- [ ] **Step 5: Commit** `hop_controller.py`, `maze_motion.py`, `test/test_hop_controller.py` — `tune: loosen max_cross_steer 0.25->0.35 (recover genuine offsets)`

## Validation (after both tasks)
1. Full ROS-free suite green: `cd "$PKG" && PYTHONPATH="$PKG" python3 -m pytest test/test_maze_motion.py test/test_maze_motion_sim.py test/test_hop_controller.py test/test_flood_fill_brain.py test/test_junction_log.py test/test_cell_walls.py test/test_maze_sim.py -q`.
2. Rebuild + Gazebo (user-initiated, diagnostics IN): watch the robot reverse out of (3,4)-class pockets and re-approach (no self-walling, no 18-escape livelock); progress past 18 m.
3. No merge/push; revert `2fe9e39` before any merge.
