import math
from tugbot_maze.maze_motion import MazeMotion, within_commit_offset, within_cell_core
from tugbot_maze.maze_sim import MazeSim, load_segments
from tugbot_maze.flood_fill_brain import ENTRANCE_CELL, EXIT_CELL, cell_center
from tugbot_maze.cell_walls import sense_cell_walls


def _scan_at(sim):
    return sim.scan(n_beams=360, fov_rad=2 * math.pi)


def test_starts_in_center_at_entrance():
    m = MazeMotion()
    assert m.cell == ENTRANCE_CELL and m.phase == 'center'


def test_center_senses_then_turns_toward_a_chosen_neighbor():
    # At the entrance cell centre, MazeMotion should sense, pick next_cell, and enter 'turn'.
    sim = MazeSim(load_segments(), cell_center(ENTRANCE_CELL), 0.0)
    m = MazeMotion()
    t = 0.0
    for _ in range(60):                       # a few ticks to settle + sense + plan
        v, w, done = m.step(sim.pose, _scan_at(sim), t)
        t += 0.1
        if m.phase in ('turn', 'drive'):
            break
    assert m.phase in ('turn', 'drive')
    assert m.hop_target is not None and m.hop_dir is not None


def test_turn_rotates_then_settles_to_drive():
    # Force a turn target 90 deg from current heading; stepping with a rotating yaw must
    # eventually settle into 'drive'.
    sim = MazeSim(load_segments(), cell_center(ENTRANCE_CELL), 0.0)
    m = MazeMotion()
    m.phase = 'turn'
    m.hop_dir = (0, 1)
    m.target_cardinal = math.pi / 2
    m.hop_start = cell_center(ENTRANCE_CELL)
    yaw = 0.0
    t = 0.0
    for _ in range(200):
        v, w, _ = m.step((0.0, 0.0, yaw), _scan_at(sim), t)
        assert abs(v) < 1e-9                  # no translation during a turn
        yaw = math.atan2(math.sin(yaw + w * 0.1), math.cos(yaw + w * 0.1))
        t += 0.1
        if m.phase == 'drive':
            break
    assert m.phase == 'drive'
    assert abs(math.atan2(math.sin(math.pi / 2 - yaw), math.cos(math.pi / 2 - yaw))) <= 0.11


def test_done_when_cell_is_exit():
    m = MazeMotion()
    m.cell = EXIT_CELL
    sim = MazeSim(load_segments(), cell_center(EXIT_CELL), 0.0)
    v, w, done = m.step(sim.pose, _scan_at(sim), 0.0)
    assert done is True and (v, w) == (0.0, 0.0)


# ---- hardened _center: position-gate predicates + sense-commit behaviors ----

def test_within_commit_offset():
    assert within_commit_offset(0.3, 0.34, 0.40) is True       # both referenced, within tol
    assert within_commit_offset(0.5, 0.0, 0.40) is False       # x axis too far
    assert within_commit_offset(None, 0.9, 0.40) is False      # open x ignored, y too far
    assert within_commit_offset(None, None, 0.40) is True      # both open -> no constraint


def test_within_cell_core():
    # cell index 4 -> centre at 8.0 (CELL_SIZE_M=2). Core = within (1.0 - margin) of centre.
    assert within_cell_core(8.0, 4, 0.40) is True
    assert within_cell_core(8.55, 4, 0.40) is True             # 0.55 <= 0.60 core
    assert within_cell_core(8.7, 4, 0.40) is False             # 0.70 > 0.60 -> near boundary
    assert within_cell_core(7.0, 4, 0.40) is False             # the (.,3)/(.,4) boundary at y=7


def _force_commit(m, sim, max_ticks=400, dt=0.1):
    """Step _center at sim's pose; on leaving 'center', simulate a re-entry of the SAME cell so
    corroboration can accumulate. Returns True once the cell commits."""
    t = 0.0
    for _ in range(max_ticks):
        m.step(sim.pose, sim.scan(n_beams=360, fov_rad=2 * math.pi), t)
        t += dt
        if m.cell in m.committed:
            return True
        if m.phase != 'center':
            m.phase = 'center'; m.center_start = None
            m.align_start = None; m.latched_cardinal = None
    return False


def test_good_read_commits_then_discard_is_inert():
    # From a centred, cardinal pose the entrance cell commits; a later sensed.discard (the churn
    # trigger) must NOT change its walls -- the committed cell is frozen against re-sensing.
    sim = MazeSim(load_segments(), cell_center(ENTRANCE_CELL), 0.0)
    m = MazeMotion()
    assert _force_commit(m, sim), "cell never committed from a centred pose"
    cell = m.cell
    before = {d: m.brain._state(cell, d) for d in ('N', 'S', 'E', 'W')}
    m.sensed.discard(cell)                          # would trigger a re-sense in the old code
    m.phase = 'center'; m.center_start = None
    m.step(sim.pose, sim.scan(n_beams=360, fov_rad=2 * math.pi), 100.0)
    after = {d: m.brain._state(cell, d) for d in ('N', 'S', 'E', 'W')}
    assert after == before                          # walls frozen despite the discard
    assert cell in m.committed                       # still committed (fast-path didn't un-commit)


def test_poor_read_not_committed_and_poor_reentry_skips():
    # A boundary-straddling pose (travel axis y) fails the position gate -> the read is NOT
    # committed, and a second straddling visit does NOT re-sense (so bad poses can't churn).
    cell = (1, 3); cx, cy = cell_center(cell)
    sim = MazeSim(load_segments(), (cx, cy + 0.85), math.pi / 2)   # ~0.85 from centre -> straddling
    m = MazeMotion(); m.cell = cell; m.hop_dir = (0, 1); m.phase = 'center'
    t = 0.0
    for _ in range(200):
        m.step(sim.pose, sim.scan(n_beams=360, fov_rad=2 * math.pi), t); t += 0.1
        if m.phase != 'center':
            break
    assert cell in m.sensed and cell not in m.committed     # sensed once, NOT committed (poor)
    snap = {d: m.brain._state(cell, d) for d in ('N', 'S', 'E', 'W')}
    m.phase = 'center'; m.center_start = None; m.align_start = None; m.latched_cardinal = None
    for _ in range(200):
        m.step(sim.pose, sim.scan(n_beams=360, fov_rad=2 * math.pi), t); t += 0.1
        if m.phase != 'center':
            break
    assert {d: m.brain._state(cell, d) for d in ('N', 'S', 'E', 'W')} == snap   # no re-sense churn


def test_no_false_wall_across_yaw_tol_and_small_offset():
    # An edge OPEN at the true centre must never read as a WALL under +/-0.3 m offset & +/-0.1 rad.
    cell = (1, 3); cx, cy = cell_center(cell)
    base = MazeSim(load_segments(), (cx, cy), 0.0)
    r0 = sense_cell_walls(*base.scan(n_beams=360, fov_rad=2 * math.pi), 0.0)
    for dx, dy, dyaw in [(0.3, 0, 0.1), (-0.3, 0, -0.1), (0, 0.3, 0.1), (0, -0.3, -0.1)]:
        s = MazeSim(load_segments(), (cx + dx, cy + dy), dyaw)
        r = sense_cell_walls(*s.scan(n_beams=360, fov_rad=2 * math.pi), dyaw)
        for d in ('N', 'S', 'E', 'W'):
            if not r0[d]:
                assert not r[d], f"false WALL on {d} from offset=({dx},{dy}) yaw={dyaw}"


def test_committed_dead_end_reopened_not_stuck():
    # A committed cell whose every edge is WALLed (disconnected) must have its cut edges RE-OPENED for
    # a corrective re-sense -- not declared terminally 'stuck' (a permanent false WALL would brick it).
    m = MazeMotion(); cell = (5, 5); m.cell = cell
    for d in ('N', 'S', 'E', 'W'):
        m.brain.mark(cell, d, is_wall=True)
    m.committed.add(cell)
    m.locomotion_walls.update({(cell, d) for d in ('N', 'S', 'E', 'W')})
    sim = MazeSim(load_segments(), cell_center(cell), 0.0)
    m.step(sim.pose, sim.scan(n_beams=360, fov_rad=2 * math.pi), 0.0)
    assert m.phase != 'stuck'
    assert cell not in m.committed and (cell, 'E') in m.reopened
    assert all(m.brain._state(cell, d) != 'wall' for d in ('N', 'S', 'E', 'W'))   # re-opened


def test_commit_freezes_sensing_not_map():
    # Commit freezes SENSING only: a locomotion (hop-failure) wall-mark still un-commits the cell
    # and records the edge so the deadlock backstop can later re-open it.
    m = MazeMotion(); cell = (3, 3); m.cell = cell
    m.committed.add(cell)
    m.phase = 'drive'; m.hop_dir = (1, 0); m.hop_target = (4, 3)
    m.hop_start = cell_center(cell)
    m.progress_pose = cell_center(cell); m.progress_t = 0.0
    m.hop_deadline = 1e9
    m.hop_attempts[(cell, 'E')] = m.max_hop_attempts - 1          # this attempt reaches the cap
    sim = MazeSim(load_segments(), cell_center(cell), 0.0)
    m.step(sim.pose, sim.scan(n_beams=360, fov_rad=2 * math.pi), 10.0)   # no progress -> wedge giveup
    assert cell not in m.committed                                # un-committed by the locomotion mark
    assert (cell, 'E') in m.locomotion_walls
    assert m.brain.is_wall(cell, 'E')


def test_disconnecting_false_wall_triggers_unstick():
    # A false WALL can disconnect the exit while leaving an OPEN neighbour -- next_cell is then
    # non-None (an inf-distance cell), so the robot would wander forever. _route must detect
    # exit-unreachability and run _unstick (re-open the blamed locomotion walls), not wander.
    m = MazeMotion(); cell = (5, 5)
    m.cell = cell
    walls = [((5, 5), 'E'), ((5, 5), 'W'), ((5, 5), 'S'),
             ((5, 6), 'N'), ((5, 6), 'E'), ((5, 6), 'W')]    # isolate the pocket {(5,5),(5,6)}
    for (c, d) in walls:
        m.brain.mark(c, d, is_wall=True)
    m.locomotion_walls.update(walls)
    m.committed.add(cell)                                    # fast-path to _route (skip the real re-sense)
    assert m.brain.next_cell(cell) is not None               # open N neighbour -> NOT fully boxed
    assert m.brain.flood().get(cell) is None                 # ...yet the pocket is exit-unreachable
    sim = MazeSim(load_segments(), cell_center(cell), 0.0)
    m.step(sim.pose, sim.scan(n_beams=360, fov_rad=2 * math.pi), 0.0)
    assert m.phase != 'stuck'                                # unstick fired (no silent wander/false-stuck)
    assert (cell, 'E') in m.reopened
    assert m.brain._state((5, 5), 'E') != 'wall'             # a cut wall was re-opened


def _pocket_walls():
    # isolate the pocket {(5,5),(5,6)}: open only the (5,5)<->(5,6) edge, wall everything else.
    return [((5, 5), 'E'), ((5, 5), 'W'), ((5, 5), 'S'),
            ((5, 6), 'N'), ((5, 6), 'E'), ((5, 6), 'W')]


def test_unstick_escalates_noncommitted_then_committed():
    # No locomotion walls. Tier 1 (locomotion) is empty, so _unstick must escalate.
    # uncommitted cut cells -> tier 2 (non-committed) re-open; committed cut cells -> tier 3.
    m = MazeMotion(); m.cell = (5, 5)
    for (c, d) in _pocket_walls():
        m.brain.mark(c, d, is_wall=True)
    m._unstick(0.0)                                   # call directly (no _center re-sense overwrite)
    assert m.phase != 'stuck' and len(m.reopened) > 0  # tier-2 (non-committed) recovery

    m2 = MazeMotion(); m2.cell = (5, 5)
    for (c, d) in _pocket_walls():
        m2.brain.mark(c, d, is_wall=True)
    m2.committed.update({(5, 5), (5, 6)})             # all cut cells committed -> only tier 3 applies
    m2._unstick(0.0)
    assert m2.phase != 'stuck' and len(m2.reopened) > 0  # tier-3 (committed) last-resort recovery


def test_unstick_terminates_when_all_cut_edges_reopened():
    # Once every cut edge has already had its one re-open, _unstick declares terminal 'stuck'
    # (the self.reopened bound guarantees termination).
    m = MazeMotion(); m.cell = (5, 5)
    base = _pocket_walls()
    for (c, d) in base:
        m.brain.mark(c, d, is_wall=True)
    m.reopened.update(base)                           # every cut edge already re-opened once
    m._unstick(0.0)
    assert m.phase == 'stuck'


def test_reopened_edge_survives_poor_resense():
    # After _unstick re-opens a (false) wall, the cell is un-committed but KEPT in `sensed`, so a
    # POOR (straddling) re-entry must NOT re-sense and re-WALL the re-opened edge (re-WALL needs a
    # GOOD pose). Otherwise a single poor read re-seals the false wall -> premature 'stuck'.
    cell = (1, 3); cx, cy = cell_center(cell)
    m = MazeMotion(); m.cell = cell; m.hop_dir = (0, 1)
    m.sensed.add(cell)                                   # sensed-but-uncommitted, as _unstick leaves it
    m.brain.mark(cell, 'N', is_wall=False)               # an optimistically re-opened edge
    m.reopened.add((cell, 'N'))
    snap = {d: m.brain._state(cell, d) for d in ('N', 'S', 'E', 'W')}
    sim = MazeSim(load_segments(), (cx, cy + 0.85), math.pi / 2)   # straddling -> poor pose
    m.phase = 'center'; t = 0.0
    for _ in range(120):
        m.step(sim.pose, sim.scan(n_beams=360, fov_rad=2 * math.pi), t); t += 0.1
        if m.phase != 'center':
            break
    assert {d: m.brain._state(cell, d) for d in ('N', 'S', 'E', 'W')} == snap   # no poor re-sense/re-WALL


def _deadend_brain(d_cell, open_dir):
    """Brain where d_cell is walled on every in-grid side except open_dir."""
    from tugbot_maze.flood_fill_brain import FloodFillBrain, DIRS, in_grid
    b = FloodFillBrain()
    for d, (dx, dy) in DIRS.items():
        nb = (d_cell[0] + dx, d_cell[1] + dy)
        if in_grid(nb) and d != open_dir:
            b.mark(d_cell, d, True)
    return b


def test_deadend_triggers_backout():
    b = _deadend_brain((5, 5), 'S')            # open only S -> parent (5,4)
    m = MazeMotion(b)
    m.cell = (5, 5)
    m.hop_dir = (0, 1)                          # entered going N => came_from = 'S'
    m._route(10.0, 10.0, 0.0)
    assert m.phase == 'backout'
    assert m.backout_target == (5, 4)
    assert abs(m.backout_cardinal - math.pi / 2) < 1e-9   # face N (into the dead-end)
    assert m.backout_count == 1


def test_backout_reverses_then_advances():
    b = _deadend_brain((5, 5), 'S')
    m = MazeMotion(b)
    m.cell = (5, 5); m.hop_dir = (0, 1)
    m._route(10.0, 10.0, 0.0)                   # -> phase 'backout', start (10,10)
    v, w, _ = m._backout((10.0, 10.0, math.pi / 2), 0.1)
    assert v < 0.0 and abs(w) < 0.05            # firm straight reverse
    m._backout((10.0, 8.0, math.pi / 2), 0.2)   # moved one cell south into (5,4)
    assert m.cell == (5, 4) and m.phase == 'center'


def test_entrance_does_not_backout():
    from tugbot_maze.flood_fill_brain import FloodFillBrain
    b = FloodFillBrain()
    b.mark((1, 0), 'E', True)                   # entrance: only in-grid open exit is N (forward)
    m = MazeMotion(b)
    m.cell = (1, 0); m.hop_dir = None           # no came-from at the start
    m._route(2.0, 0.0, 0.0)
    assert m.phase == 'turn' and m.hop_target is not None


def test_backout_requires_exit_equals_came_from():
    b = _deadend_brain((5, 5), 'S')             # lone open exit is 'S'
    m = MazeMotion(b)
    m.cell = (5, 5); m.hop_dir = (0, -1)        # entered going S => came_from = 'N' != 'S'
    m._route(10.0, 10.0, 0.0)
    assert m.phase != 'backout'


def test_backout_timeout_escalates():
    b = _deadend_brain((5, 5), 'S')
    m = MazeMotion(b)
    m.cell = (5, 5); m.hop_dir = (0, 1)
    for _ in range(m.max_backout_attempts):     # each: enter backout, then time out (no arrival)
        m._route(10.0, 10.0, 0.0)
        assert m.phase == 'backout'
        m._backout((10.0, 10.0, math.pi / 2), m.backout_deadline + 0.1)
        assert m.phase == 'center'
    m._route(10.0, 10.0, 0.0)                   # attempts exhausted -> normal turn, no re-arm
    assert m.phase == 'turn'


def test_mark_traversal_counts_on_arrival_not_on_pick():
    from tugbot_maze.flood_fill_brain import FloodFillBrain
    b = FloodFillBrain()
    b.mark((5, 5), 'E', True); b.mark((5, 5), 'W', True)   # open N/S -> routes toward exit (N)
    m = MazeMotion(b)
    m.cell = (5, 5); m.hop_dir = (0, -1)          # arrived from S; not a dead-end (N also open)
    m._route(10.0, 10.0, 0.0)
    assert m.phase == 'turn' and m.hop_target is not None
    tgt = m.hop_target
    assert b._edge_traversal_count((5, 5), tgt) == 0          # NOT counted at pick
    m.phase = 'drive'; m.hop_start = (10.0, 10.0)
    arrive = (10.0 + 2.0 * m.hop_dir[0], 10.0 + 2.0 * m.hop_dir[1], 0.0)
    m._drive(arrive, _scan_at(MazeSim(load_segments(), cell_center(tgt), 0.0)), 0.1)
    assert b._edge_traversal_count((5, 5), tgt) == 1          # counted on arrival


def test_failed_hop_does_not_inflate_traversal():
    from tugbot_maze.flood_fill_brain import FloodFillBrain
    b = FloodFillBrain()
    b.mark((5, 5), 'E', True); b.mark((5, 5), 'W', True)
    m = MazeMotion(b)
    m.cell = (5, 5); m.hop_dir = (0, -1)
    m._route(10.0, 10.0, 0.0)
    tgt = m.hop_target
    m.phase = 'drive'; m.hop_start = (10.0, 10.0); m.hop_deadline = 0.0; m.max_hop_attempts = 99
    m.progress_pose = (10.0, 10.0); m.progress_t = 1.0       # wedge baseline (not crossing wedge_detect_s)
    sim = MazeSim(load_segments(), cell_center((5, 5)), 0.0)
    m._drive((10.4, 10.0, 0.0), _scan_at(sim), 1.0)           # moved 0.4 (<1.95), t>=deadline -> fail
    assert m.phase == 'center'                                 # bounced back, did NOT arrive
    assert b._edge_traversal_count((5, 5), tgt) == 0           # failed hop must NOT inflate


# ---- C1: exploration-progress clock + confinement ----

def _two_exit_brain(cell, open_a, open_b):
    """cell open on exactly open_a/open_b (in-grid), walled elsewhere."""
    from tugbot_maze.flood_fill_brain import FloodFillBrain, DIRS, in_grid
    b = FloodFillBrain()
    for d, (dx, dy) in DIRS.items():
        nb = (cell[0] + dx, cell[1] + dy)
        if in_grid(nb) and d not in (open_a, open_b):
            b.mark(cell, d, True)
    return b


def test_track_cell_seeds_unconditionally_at_startup():
    m = MazeMotion()                       # cell == last_seen_cell == ENTRANCE_CELL, NO cell change
    m._track_cell(1.78e9)                  # large absolute clock, tick 1
    assert m.explore_t == 1.78e9           # seeded even with no cell change (MF2)
    assert ENTRANCE_CELL in m.visited      # entrance counts as visited ground


def test_track_cell_new_ground_resets_explore_t_and_tier():
    m = MazeMotion(); m._track_cell(1000.0)   # seed at entrance
    m.escape_tier = 2
    m.cell = (1, 1)                            # advance to NEW ground
    m._track_cell(1005.0)
    assert (1, 1) in m.visited and m.explore_t == 1005.0
    assert m.prev_cell == ENTRANCE_CELL and m.escape_tier == 0   # progress clears escalation


def test_track_cell_revisit_does_not_reset_explore_t():
    m = MazeMotion(); m._track_cell(1000.0)    # visit entrance
    m.cell = (1, 1); m._track_cell(1005.0)     # new ground -> explore_t = 1005
    m.cell = ENTRANCE_CELL; m._track_cell(1010.0)   # REVISIT (already in visited)
    assert m.explore_t == 1005.0               # revisit must NOT advance the clock (F1 fix)


def test_confined_true_for_small_footprint_false_for_large():
    m = MazeMotion(); m.no_progress_s = 90.0; m.confine_k = 6
    m.cell = (5, 5)
    m.recent = [(100.0, c) for c in [(5, 5), (5, 6), (5, 5), (5, 6)]]  # 2 distinct in-window
    assert m._confined(120.0) is True
    m.recent = [(100.0, c) for c in [(1, 1), (2, 2), (3, 3), (4, 4), (5, 5), (6, 6), (7, 7)]]  # 7 distinct
    assert m._confined(120.0) is False
    m.recent = []                              # frozen single cell -> footprint {self.cell} size 1
    assert m._confined(120.0) is True


def test_confined_prunes_out_of_window_entries():
    m = MazeMotion(); m.no_progress_s = 90.0; m.confine_k = 2
    m.cell = (5, 5)
    # old, out-of-window distinct cells must be pruned so they don't inflate the footprint
    m.recent = [(10.0, (1, 1)), (10.0, (2, 2)), (10.0, (3, 3)), (200.0, (5, 6))]
    assert m._confined(250.0) is True          # only (5,6)+(5,5) within [160,250] -> 2 <= K


# ---- C3: escalating escape (_escape) + _backout guard ----

def test_escape_tier1_reverses_to_prev_keeps_edge_open():
    b = _two_exit_brain((5, 5), 'N', 'S')      # came from S=(5,4); forward N=(5,6)
    m = MazeMotion(b)
    m.cell = (5, 5); m.last_seen_cell = (5, 5); m.prev_cell = (5, 4)
    m.hop_dir = (0, 1); m.hop_target = (5, 6); m.visited = {(5, 4), (5, 5)}
    m._escape((10.0, 10.0, 0.0), 100.0)
    assert m.phase == 'backout' and m.backout_target == (5, 4)
    assert m.escape_tier == 1 and m.escape_count == 1 and m._escape_backout is True
    assert not b.is_wall((5, 5), 'N')          # Tier-1 leaves the forward edge OPEN (re-approach)
    assert abs(m.backout_cardinal - math.pi / 2) < 1e-9   # face N -> reverse S into prev


def test_escape_tier2_gives_up_edge_then_reverses():
    b = _two_exit_brain((5, 5), 'N', 'S')
    m = MazeMotion(b)
    m.cell = (5, 5); m.last_seen_cell = (5, 5); m.prev_cell = (5, 4)
    m.hop_dir = (0, 1); m.hop_target = (5, 6); m.visited = {(5, 4), (5, 5)}
    m.escape_tier = 1                          # already escalated once
    m._escape((10.0, 10.0, 0.0), 100.0)
    assert m.escape_tier == 2
    assert b.is_wall((5, 5), 'N')              # Tier-2 GIVES UP the forward edge (real brain wall)
    assert (5, 5) not in m.committed
    assert m.phase == 'backout' and m.backout_target == (5, 4)


def test_escape_tier_caps_at_max():
    b = _two_exit_brain((5, 5), 'N', 'S')
    m = MazeMotion(b)
    m.cell = (5, 5); m.last_seen_cell = (5, 5); m.prev_cell = (5, 4)
    m.hop_dir = (0, 1); m.hop_target = (5, 6); m.visited = {(5, 4), (5, 5)}
    m.escape_tier = 2
    m._escape((10.0, 10.0, 0.0), 100.0)
    assert m.escape_tier == 2                   # capped at max_escape_tier


def test_escape_resets_explore_t_every_entry():
    b = _two_exit_brain((5, 5), 'N', 'S')
    m = MazeMotion(b)
    m.cell = (5, 5); m.last_seen_cell = (5, 5); m.prev_cell = (5, 4)
    m.hop_dir = (0, 1); m.hop_target = (5, 6); m.visited = {(5, 4), (5, 5)}
    m.explore_t = 10.0
    m._escape((10.0, 10.0, 0.0), 500.0)
    assert m.explore_t == 500.0                 # reset on entry (no busy-re-fire)


def test_escape_no_reverse_falls_to_unstick():
    b = _two_exit_brain((5, 5), 'N', 'S')       # (5,5) has E/W walled, N/S open
    m = MazeMotion(b)
    m.cell = (5, 5); m.last_seen_cell = (5, 5); m.prev_cell = (1, 1)   # NOT adjacent
    m.hop_dir = (0, 1); m.hop_target = (5, 6); m.visited = {(5, 5)}
    m._escape((10.0, 10.0, 0.0), 100.0)
    assert m.phase in ('center', 'stuck')       # handed to _unstick (reopened->center, or stuck)


def test_escape_backout_timeout_does_not_charge_dead_end_budget():
    m = MazeMotion()
    m.cell = (5, 5); m._escape_backout = True
    m.backout_start = (10.0, 10.0); m.backout_deadline = 0.0   # immediate timeout, no arrival
    bc0 = m.backout_count
    m._backout((10.0, 10.0, 0.0), 1.0)
    assert m.backout_attempts.get((5, 5), 0) == 0     # escape-reverse timeout must NOT charge it
    assert m.backout_count == bc0                     # nor inflate the dead-end metric
    assert m._escape_backout is False                 # cleared on leaving backout (re-enables C2)


def test_churn_escape_escalates_and_reroutes():
    """MF8 churn-escape: under repeated no-progress fires in a confined region, the escape escalates
    Tier-1 -> Tier-2, gives up the blocked edge, and next_cell then routes to a DIFFERENT cell
    (flood reroute) -- so the robot provably leaves the churn instead of looping. Deterministic
    FSM-level fixture (full physical liveness is validated in Gazebo)."""
    from tugbot_maze.flood_fill_brain import FloodFillBrain
    b = FloodFillBrain()                              # open map; (3,3) routes N toward the exit
    m = MazeMotion(b)
    m.cell = (3, 3); m.last_seen_cell = (3, 3); m.prev_cell = (3, 2)   # came from S=(3,2)
    m.hop_dir = (0, 1); m.hop_target = (3, 4)         # forward N toward exit
    m.visited = {(3, 2), (3, 3)}
    assert b.next_cell((3, 3)) == (3, 4)              # baseline: routes forward N
    m._escape((6.0, 6.0, 0.0), 100.0)                 # fire 1 -> Tier 1 (edge stays open)
    assert m.escape_tier == 1 and not b.is_wall((3, 3), 'N')
    m._escape((6.0, 6.0, 0.0), 200.0)                 # fire 2 (no new ground) -> Tier 2 gives up edge
    assert m.escape_tier == 2 and b.is_wall((3, 3), 'N')
    assert b.next_cell((3, 3)) != (3, 4)              # liveness: flood reroutes off the walled edge
    assert m.escape_count == 2


def test_watchdog_fires_from_center_when_confined_and_no_progress():
    b = _two_exit_brain((5, 5), 'N', 'S')
    m = MazeMotion(b)
    m.cell = (5, 5); m.last_seen_cell = (5, 5); m.prev_cell = (5, 4)
    m.hop_dir = (0, 1); m.hop_target = (5, 6); m.visited = {(5, 4), (5, 5)}; m.explore_t = 100.0
    sim = MazeSim(load_segments(), cell_center((5, 5)), 0.0)
    m.step(sim.pose, _scan_at(sim), 100.0 + m.no_progress_s + 1.0)
    assert m.escape_count == 1                  # fired (no new ground + confined + past window)


def test_watchdog_fires_from_stuck():
    m = MazeMotion()
    m.cell = (3, 1); m.last_seen_cell = (3, 1); m.prev_cell = (2, 1)
    m.phase = 'stuck'; m.visited = {(2, 1), (3, 1)}; m.explore_t = 50.0
    sim = MazeSim(load_segments(), cell_center((3, 1)), 0.0)
    m.step(sim.pose, _scan_at(sim), 50.0 + m.no_progress_s + 1.0)
    assert m.escape_count == 1                  # the F2 fix: stuck is no longer a permanent freeze


def test_watchdog_silent_at_exit_cell():
    m = MazeMotion()
    m.cell = EXIT_CELL; m.last_seen_cell = EXIT_CELL; m.visited = {EXIT_CELL}; m.explore_t = 0.0
    sim = MazeSim(load_segments(), cell_center(EXIT_CELL), 0.0)
    v, w, done = m.step(sim.pose, _scan_at(sim), 1.0 + m.no_progress_s + 5.0)
    assert done is True and m.escape_count == 0  # never escapes at the exit (guard non-vacuous)


def test_watchdog_silent_when_not_confined():
    m = MazeMotion(); m.confine_k = 6
    m.cell = (5, 5); m.last_seen_cell = (5, 5); m.visited = {(5, 5)}; m.explore_t = 10.0
    t = 10.0 + m.no_progress_s + 1.0
    m.recent = [(t - 1, c) for c in [(1, 1), (2, 2), (3, 3), (4, 4), (5, 5), (6, 6), (7, 7)]]  # 7 > K
    sim = MazeSim(load_segments(), cell_center((5, 5)), 0.0)
    m.step(sim.pose, _scan_at(sim), t)
    assert m.escape_count == 0                  # large footprint -> not confined -> silent


def test_watchdog_suppressed_during_escape_reverse():
    b = _two_exit_brain((5, 5), 'N', 'S')
    m = MazeMotion(b)
    m.cell = (5, 5); m.last_seen_cell = (5, 5); m.prev_cell = (5, 4)
    m.hop_dir = (0, 1); m.hop_target = (5, 6); m.visited = {(5, 4), (5, 5)}; m.explore_t = 10.0
    m.phase = 'backout'; m._escape_backout = True            # an escape reverse is executing
    m.backout_start = (10.0, 10.0); m.backout_deadline = 1e12
    sim = MazeSim(load_segments(), cell_center((5, 5)), 0.0)
    m.step(sim.pose, _scan_at(sim), 10.0 + m.no_progress_s + 5.0)
    assert m.escape_count == 0                  # not re-fired mid-escape (still in backout)


def test_failed_hops_persists_across_arrivals_and_walls_edge():
    b = _two_exit_brain((5, 5), 'N', 'S')
    m = MazeMotion(b); m.max_hop_attempts = 99      # isolate failed_hops from the per-dwell cap
    m.cell = (5, 5); m.hop_dir = (0, 1); m.hop_target = (5, 6)
    sim = MazeSim(load_segments(), cell_center((5, 5)), 0.0)
    for _ in range(m.failed_hop_limit):
        m.hop_attempts.clear()                      # simulate the wholesale clear that any arrival does
        m.phase = 'drive'; m.hop_start = (10.0, 10.0); m.hop_deadline = 0.0
        m.progress_pose = (10.0, 10.0); m.progress_t = 1.0   # wedge baseline (not crossing wedge_detect_s)
        m._drive((10.4, 10.0, 0.0), _scan_at(sim), 1.0)   # moved 0.4 (<arrive), t>=deadline -> fail
    assert b.is_wall((5, 5), 'N')                   # failed_hops survived the clears -> edge given up (F3)


def test_single_failed_hop_does_not_wall():
    b = _two_exit_brain((5, 5), 'N', 'S')
    m = MazeMotion(b); m.max_hop_attempts = 99
    m.cell = (5, 5); m.hop_dir = (0, 1); m.hop_target = (5, 6)
    sim = MazeSim(load_segments(), cell_center((5, 5)), 0.0)
    m.phase = 'drive'; m.hop_start = (10.0, 10.0); m.hop_deadline = 0.0
    m.progress_pose = (10.0, 10.0); m.progress_t = 1.0       # wedge baseline (not crossing wedge_detect_s)
    m._drive((10.4, 10.0, 0.0), _scan_at(sim), 1.0)
    assert not b.is_wall((5, 5), 'N') and m.failed_hops[((5, 5), 'N')] == 1


def test_successful_hop_clears_failed_hops_for_edge():
    b = _two_exit_brain((5, 5), 'N', 'S')
    m = MazeMotion(b); m.max_hop_attempts = 99
    m.cell = (5, 5); m.hop_dir = (0, 1); m.hop_target = (5, 6)
    m.failed_hops[((5, 5), 'N')] = 2
    m.phase = 'drive'; m.hop_start = (10.0, 10.0)
    sim = MazeSim(load_segments(), cell_center((5, 6)), 0.0)
    m._drive((10.0, 12.0, 0.0), _scan_at(sim), 0.1)         # moved ~2.0 -> arrival
    assert m.cell == (5, 6)
    assert ((5, 5), 'N') not in m.failed_hops              # cleared on success, pre-advance key (MF6)


def test_unstick_reopen_clears_failed_hops_and_resets_tier():
    from tugbot_maze.flood_fill_brain import FloodFillBrain
    b = FloodFillBrain()
    # box (5,5) in with WALLs on all four sides so _unstick must reopen a cut edge
    for d in ('N', 'S', 'E', 'W'):
        b.mark((5, 5), d, True)
    m = MazeMotion(b)
    m.cell = (5, 5); m.escape_tier = 2
    m.failed_hops[((5, 5), 'N')] = 3
    m._unstick(1.0)
    assert m.escape_tier == 0                              # map mutation (reopen) resets escalation
    # at least one incident edge was reopened and its failed_hops cleared
    assert all(k[0] != (5, 5) or k not in m.failed_hops for k in [((5, 5), 'N')]) or \
           ((5, 5), 'N') not in m.failed_hops


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
