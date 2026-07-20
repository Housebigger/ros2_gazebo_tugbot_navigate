"""Task B: believed-track edge-legality detector. On each believed cell change, the edge just
crossed must not be a known wall; a violation means the pose belief has slipped across a wall
(the earliest observable symptom of the mislocalization episode). Pure observation -- it must
NOT change any routing decision."""
from tugbot_maze.maze_motion import MazeMotion


def test_legal_transition_emits_no_event():
    m = MazeMotion(); m.cell = (5, 5); m.last_seen_cell = (5, 5)
    m.cell = (6, 5)                       # move E; edge (5,5)-E is open by default (unknown)
    m._track_cell(1.0)
    assert not any(e.startswith('ILLEGAL_EDGE') for e in m.events)


def test_illegal_transition_across_known_wall_emits_event():
    m = MazeMotion(); m.cell = (5, 5); m.last_seen_cell = (5, 5)
    m.brain.mark((5, 5), 'E', is_wall=True)   # (5,5)-E is a KNOWN wall
    m.cell = (6, 5)                            # ...yet the belief crossed it
    m._track_cell(2.0)
    ev = [e for e in m.events if e.startswith('ILLEGAL_EDGE')]
    assert len(ev) == 1
    assert 'cell=(6, 5)' in ev[0] and 'prev=(5, 5)' in ev[0] and 'dir=W' in ev[0]


def test_illegal_event_reports_committed_state():
    m = MazeMotion(); m.cell = (5, 5); m.last_seen_cell = (5, 5)
    m.brain.mark((5, 5), 'N', is_wall=True)
    m.committed.add((5, 6))
    m.cell = (5, 6)                            # crossed (5,5)-N into a committed cell
    m._track_cell(3.0)
    ev = [e for e in m.events if e.startswith('ILLEGAL_EDGE')][0]
    assert 'committed=True' in ev


def test_non_adjacent_jump_does_not_crash_or_falsely_fire():
    # A re-anchor can teleport the cell by >1 (diagonal / multi-cell). No single edge exists;
    # the detector must skip (no direction) rather than fire or raise.
    m = MazeMotion(); m.cell = (5, 5); m.last_seen_cell = (5, 5)
    m.cell = (7, 8)
    m._track_cell(4.0)
    assert not any(e.startswith('ILLEGAL_EDGE') for e in m.events)


def test_detector_does_not_change_routing_state():
    m = MazeMotion(); m.cell = (5, 5); m.last_seen_cell = (5, 5)
    m.brain.mark((5, 5), 'E', is_wall=True)
    m.cell = (6, 5)
    m._track_cell(5.0)
    assert m.prev_cell == (5, 5) and m.last_seen_cell == (6, 5)
