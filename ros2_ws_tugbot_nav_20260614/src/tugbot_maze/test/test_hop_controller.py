import math
from tugbot_maze.hop_controller import hop_command


def test_arrived_when_within_tolerance():
    v, w, arrived = hop_command((2.0, 0.0, 0.0), (2.0, 0.0))
    assert arrived is True
    assert (v, w) == (0.0, 0.0)


def test_drives_forward_when_aligned():
    v, w, arrived = hop_command((0.0, 0.0, 0.0), (2.0, 0.0))   # target straight ahead
    assert arrived is False
    assert v > 0.0
    assert abs(w) < 1e-6


def test_turns_toward_target_and_slows():
    # target to the left (90 deg) -> turn left (+w), and v throttled near 0 while mis-aligned
    v, w, arrived = hop_command((0.0, 0.0, 0.0), (0.0, 2.0))
    assert w > 0.0
    assert v < 0.1


def test_commands_within_envelope():
    v, w, _ = hop_command((0.0, 0.0, 0.0), (0.0, 2.0))   # large heading error
    assert -0.5 <= v <= 0.5
    assert -0.5 <= w <= 0.5
