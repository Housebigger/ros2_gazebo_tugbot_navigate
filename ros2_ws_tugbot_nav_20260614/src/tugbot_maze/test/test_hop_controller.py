import math
from tugbot_maze.hop_controller import (
    hop_command, centering_command, hop_drive_command, cross_track_offset)


def test_cross_track_sign_by_direction():
    # N travel: robot east of centre (ox>0) is to its RIGHT -> negative cross-track
    assert cross_track_offset(0.3, None, (0, 1)) == -0.3
    # S travel: east is to its LEFT -> positive
    assert cross_track_offset(0.3, None, (0, -1)) == 0.3
    # E travel: north (oy>0) is to its LEFT -> positive
    assert cross_track_offset(None, 0.3, (1, 0)) == 0.3
    # W travel: north is to its RIGHT -> negative
    assert cross_track_offset(None, 0.3, (-1, 0)) == -0.3


def test_cross_track_open_axis_is_zero():
    # N travel but the x axis is open (no side wall) -> no lateral reference
    assert cross_track_offset(None, 0.5, (0, 1)) == 0.0
    assert cross_track_offset(0.5, None, (1, 0)) == 0.0


def test_hop_drive_steers_right_when_left_of_centre():
    # facing north, on the centerline (dyaw=0) but 0.3 m LEFT of centre (cross>0):
    # steer RIGHT (negative w) to come back, still driving forward
    v, w = hop_drive_command((1.5, 4.0, math.pi / 2), math.pi / 2, 0.3)
    assert w < 0.0 and v > 0.0


def test_hop_drive_cross_track_within_envelope():
    v, w = hop_drive_command((1.5, 4.0, math.pi / 2), math.pi / 2, -2.0)   # large cross-track
    assert -0.5 <= v <= 0.5 and -0.5 <= w <= 0.5


def test_centering_done_when_within_tol_or_open():
    # both axes within tol -> done, stop
    assert centering_command((2.0, 2.0, 0.0), 0.05, -0.04, tol=0.12) == (0.0, 0.0, True)
    # both axes open (None) -> nothing to reference -> done
    assert centering_command((2.0, 2.0, 0.0), None, None, tol=0.12) == (0.0, 0.0, True)


def test_centering_faces_axis_cardinal_before_driving():
    # 0.3 m EAST of centre (ox>0) while facing east (yaw 0): must turn to face WEST (pi) first
    v, w, done = centering_command((2.3, 2.0, 0.0), 0.3, None, tol=0.12, yaw_tol=0.10)
    assert done is False and v == 0.0 and abs(w) > 0.0      # turning in place, not driving


def test_centering_drives_forward_once_aligned():
    # 0.3 m EAST of centre, already facing WEST (yaw pi): drive forward to null the offset
    v, w, done = centering_command((2.3, 2.0, math.pi), 0.3, None, tol=0.12, yaw_tol=0.10)
    assert done is False and v > 0.0 and abs(w) < 1e-6


def test_centering_y_axis_faces_north_south():
    # 0.3 m NORTH of centre (oy>0): drive SOUTH -> face -pi/2; already facing south -> drive
    v, w, done = centering_command((2.0, 2.3, -math.pi / 2), None, 0.3, tol=0.12, yaw_tol=0.10)
    assert done is False and v > 0.0 and abs(w) < 1e-6


def test_centering_corrects_larger_axis_first():
    # ox=0.2 (east), oy=0.5 (north): the y axis is larger -> correct it first (face N/S)
    v, w, done = centering_command((2.2, 2.5, 0.0), 0.2, 0.5, tol=0.12, yaw_tol=0.10)
    # facing east (yaw 0), needs to face south (-pi/2) for the y correction -> turning
    assert done is False and v == 0.0 and abs(w) > 0.0


def test_centering_within_envelope():
    v, w, _ = centering_command((2.0, 2.9, 0.0), None, 0.9, tol=0.12)   # large offset
    assert -0.5 <= v <= 0.5 and -0.5 <= w <= 0.5


def test_hop_drive_holds_heading_straight():
    # aligned with the cardinal -> drive forward, little/no steering
    v, w = hop_drive_command((1.5, 4.0, math.pi / 2), math.pi / 2)
    assert v > 0.0 and abs(w) < 1e-6


def test_hop_drive_steers_back_toward_heading_while_moving():
    # yaw drifted +0.2 rad off the north cardinal -> steer negative to correct, still moving
    v, w = hop_drive_command((1.5, 4.0, math.pi / 2 + 0.2), math.pi / 2)
    assert w < 0.0 and v > 0.0


def test_hop_drive_slows_when_badly_misaligned():
    # 90 deg off -> throttle forward speed toward 0 while turning hard
    v, w = hop_drive_command((1.5, 4.0, 0.0), math.pi / 2)
    assert v < 0.1 and w > 0.0


def test_hop_drive_within_envelope():
    v, w = hop_drive_command((1.5, 4.0, 0.0), math.pi)
    assert -0.5 <= v <= 0.5 and -0.5 <= w <= 0.5


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
