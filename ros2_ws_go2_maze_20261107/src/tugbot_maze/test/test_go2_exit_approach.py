"""Reaching the discrete exit cell must not stop short of the actual exit region."""
import math

from tugbot_maze.go2_exit_approach import exit_approach_command

EXIT = (21.072562, 18.083566)


def test_reported_exit_cell_just_outside_region_keeps_approaching():
    # Real first-run failure: EXIT_REACHED but 1.223 m from the 1.2 m region.
    v, w, reached = exit_approach_command((19.851786, 18.006441, 0.024166), EXIT, 0.85, 3.0)
    assert 0 < v <= 0.12
    assert abs(w) <= 0.35
    assert not reached


def test_final_approach_has_position_margin_and_zero_velocity():
    assert exit_approach_command((20.30, 18.083566, 0.0), EXIT, 0.85, 3.0) == (0.0, 0.0, True)


def test_exit_approach_stops_on_blocked_or_missing_scan():
    for clearance in (0.10, float('nan')):
        assert exit_approach_command((19.85, 18.0, 0.0), EXIT, 0.85, clearance) == (0.0, 0.0, False)


def test_exit_approach_aligns_before_translating():
    v, w, reached = exit_approach_command((19.85, 18.0, math.pi), EXIT, 0.85, 3.0)
    assert v == 0.0 and abs(w) <= 0.35 and not reached
