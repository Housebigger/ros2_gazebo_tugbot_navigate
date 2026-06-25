from tugbot_maze.flood_fill_brain import FloodFillBrain
from tugbot_maze.map_memory import MapMemory


def test_is_lateral_pin_true_for_open_front_side_pin():
    m = MapMemory(FloodFillBrain())
    assert m.is_lateral_pin(perp_front=4.0, near=0.31, cross_track=1.22, safety_radius=0.70) is True


def test_is_lateral_pin_false_for_real_front_wall():
    m = MapMemory(FloodFillBrain())
    assert m.is_lateral_pin(perp_front=0.6, near=0.31, cross_track=1.22, safety_radius=0.70) is False


def test_is_lateral_pin_false_when_not_near_a_wall():
    m = MapMemory(FloodFillBrain())
    assert m.is_lateral_pin(perp_front=4.0, near=0.85, cross_track=1.22, safety_radius=0.70) is False


def test_is_lateral_pin_false_when_on_centerline():
    m = MapMemory(FloodFillBrain())
    assert m.is_lateral_pin(perp_front=4.0, near=0.31, cross_track=0.10, safety_radius=0.70) is False


def test_mark_wall_on_failure_suppresses_lateral_pin():
    b = FloodFillBrain(); m = MapMemory(b)
    marked = m.mark_wall_on_failure((4, 9), 'E', perp_front=4.0, near=0.31,
                                    cross_track=1.22, safety_radius=0.70)
    assert marked is False
    assert b.is_wall((4, 9), 'E') is False
    assert m.suppressed == 1


def test_mark_wall_on_failure_marks_genuine_front_block():
    b = FloodFillBrain(); m = MapMemory(b)
    marked = m.mark_wall_on_failure((4, 9), 'N', perp_front=0.5, near=0.31,
                                    cross_track=1.22, safety_radius=0.70)
    assert marked is True
    assert b.is_wall((4, 9), 'N') is True
    assert m.suppressed == 0


def test_reconcile_no_snap_before_persist():
    m = MapMemory(FloodFillBrain(), reconcile_persist_s=8.0)
    m.observe((4, 9), 9.23, 18.0, 100.0)
    assert m.reconcile_target((4, 9), 9.23, 18.0, 105.0) == (4, 9)
    assert m.reconciles == 0


def test_reconcile_snaps_after_persist():
    m = MapMemory(FloodFillBrain(), reconcile_persist_s=8.0)
    m.observe((4, 9), 9.23, 18.0, 100.0)
    m.observe((4, 9), 9.23, 18.0, 108.5)
    assert m.reconcile_target((4, 9), 9.23, 18.0, 108.5) == (5, 9)
    assert m.reconciles == 1


def test_observe_resets_desync_when_synced():
    m = MapMemory(FloodFillBrain(), reconcile_persist_s=8.0)
    m.observe((4, 9), 9.23, 18.0, 100.0)
    m.observe((5, 9), 9.23, 18.0, 101.0)
    assert m.reconcile_target((5, 9), 9.23, 18.0, 110.0) == (5, 9)
    assert m.reconciles == 0


def test_reconcile_resets_clock_when_pose_moves():
    m = MapMemory(FloodFillBrain(), reconcile_persist_s=8.0, reconcile_move_eps=0.5)
    m.observe((4, 9), 9.23, 18.0, 100.0)                 # desync begins at pose x=9.23 (odom (5,9))
    m.observe((4, 9), 9.85, 18.0, 104.0)                 # still odom (5,9) but moved 0.62m -> clock restarts
    assert m.reconcile_target((4, 9), 9.85, 18.0, 108.5) == (4, 9)   # 4.5s since restart < 8 -> no snap
    assert m.reconciles == 0


def test_reconcile_snaps_when_pose_stays_pinned():
    m = MapMemory(FloodFillBrain(), reconcile_persist_s=8.0, reconcile_move_eps=0.5)
    m.observe((4, 9), 9.23, 18.0, 100.0)
    m.observe((4, 9), 9.25, 18.02, 104.0)                # pose within eps -> clock keeps running
    assert m.reconcile_target((4, 9), 9.25, 18.02, 108.5) == (5, 9)  # ~8.5s pinned -> snap
    assert m.reconciles == 1
