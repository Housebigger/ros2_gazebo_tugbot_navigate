import math
import pytest
from tugbot_maze.hop_controller import (
    hop_command, centering_command, hop_drive_command, cross_track_offset)
from tugbot_maze.hop_controller import corridor_drive_command
from tugbot_maze.hop_controller import side_distances
from tugbot_maze.hop_controller import centerline_cross
from tugbot_maze.hop_controller import corridor_follow_command
from tugbot_maze.hop_controller import profiled_turn_command
from tugbot_maze.hop_controller import backout_command
from tugbot_maze.hop_controller import grid_cross_track


def test_corridor_follow_centres_between_walls():
    v, w = corridor_follow_command(math.pi / 2, math.pi / 2, 0.88, 0.88)
    assert v > 0.0 and abs(w) < 1e-9


def test_corridor_follow_steers_toward_farther_wall():
    # facing N, closer to the RIGHT (east) wall (d_right small) -> right of centre -> steer LEFT (+w)
    v, w = corridor_follow_command(math.pi / 2, math.pi / 2, 1.2, 0.5)
    assert w > 0.0 and v > 0.0


def test_corridor_follow_single_wall_holds_offset():
    # only LEFT wall seen, robot off-center toward it (0.6) -> left of centre -> steer RIGHT (-w),
    # still driving (a modest offset stays under corridor_drive_command's turn-first throttle)
    v, w = corridor_follow_command(math.pi / 2, math.pi / 2, 0.6, 2.0)
    assert w < 0.0 and v > 0.0


def test_corridor_follow_no_walls_uses_fallback():
    _, w = corridor_follow_command(math.pi / 2, math.pi / 2, 2.0, 2.0, fallback_cross=0.3)
    assert w < 0.0
    _, w0 = corridor_follow_command(math.pi / 2, math.pi / 2, 2.0, 2.0, fallback_cross=0.0)
    assert abs(w0) < 1e-9


def test_corridor_follow_slows_near_wall_never_zero():
    v, _ = corridor_follow_command(math.pi / 2, math.pi / 2, 0.88, 0.88, near_wall_m=0.40)
    assert 0.0 < v <= 0.12


def test_corridor_follow_within_envelope():
    v, w = corridor_follow_command(0.0, math.pi / 2, 0.3, 0.3, near_wall_m=0.35)
    assert -0.5 <= v <= 0.5 and -0.5 <= w <= 0.5


def test_centerline_cross_both_walls_balances():
    assert centerline_cross(0.88, 0.88) == 0.0
    assert centerline_cross(0.5, 1.2) > 0.0                   # near left wall -> left of centre (+)
    assert centerline_cross(0.6, 1.0) == pytest.approx(0.2)   # (d_right - d_left)/2


def test_centerline_cross_single_wall_holds_half_corridor():
    assert centerline_cross(2.0, 1.2, half_corridor_m=0.88) == pytest.approx(1.2 - 0.88)
    assert centerline_cross(0.5, 2.0, half_corridor_m=0.88) == pytest.approx(0.88 - 0.5)


def test_centerline_cross_neither_wall_uses_fallback():
    assert centerline_cross(2.0, 2.0, fallback_cross=0.25) == 0.25
    assert centerline_cross(2.0, 2.0) == 0.0


def test_side_distances_maps_left_right_by_direction():
    # robot-frame left = +90 deg from heading
    perp = {'E': 1.0, 'W': 2.0, 'N': 3.0, 'S': 4.0}
    assert side_distances(perp, (0, 1)) == (2.0, 1.0)    # N: left=W, right=E
    assert side_distances(perp, (0, -1)) == (1.0, 2.0)   # S: left=E, right=W
    assert side_distances(perp, (1, 0)) == (3.0, 4.0)    # E: left=N, right=S
    assert side_distances(perp, (-1, 0)) == (4.0, 3.0)   # W: left=S, right=N


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


def test_hop_drive_large_cross_does_not_kill_forward_speed():
    # a huge (likely bogus) cross-track must NOT stall the drive: heading is aligned, so
    # the capped cross-track only nudges w and forward speed stays near full.
    v, w = hop_drive_command((1.5, 4.0, math.pi / 2), math.pi / 2, 5.0)
    assert v > 0.2 and abs(w) <= 0.25 + 1e-9


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


def test_corridor_drive_steers_toward_centerline():
    # facing the N cardinal, 0.3 m LEFT of centre -> steer RIGHT (negative w), still driving
    v, w = corridor_drive_command(math.pi / 2, math.pi / 2, 0.3)
    assert w < 0.0 and v > 0.0


def test_corridor_drive_goes_straight_on_centerline():
    v, w = corridor_drive_command(math.pi / 2, math.pi / 2, 0.0)
    assert v > 0.0 and abs(w) < 1e-9


def test_corridor_drive_turns_in_place_when_badly_misaligned():
    # heading 90 deg off the cardinal -> v throttled to ~0 (turn first), w turns toward cardinal
    v, w = corridor_drive_command(0.0, math.pi / 2, 0.0)
    assert v < 0.05 and w > 0.0


def test_corridor_drive_slows_near_wall_but_never_zero():
    # aligned & centred but a side wall at the stop distance -> v floored, not zero (no freeze)
    v, w = corridor_drive_command(math.pi / 2, math.pi / 2, 0.0, near_wall_m=0.40)
    assert 0.0 < v <= 0.12        # ~wedge_v_floor, never 0
    v2, _ = corridor_drive_command(math.pi / 2, math.pi / 2, 0.0, near_wall_m=0.30)  # past stop
    assert 0.0 < v2 <= 0.12


def test_corridor_drive_full_speed_when_far_from_walls():
    v, _ = corridor_drive_command(math.pi / 2, math.pi / 2, 0.0, near_wall_m=1.5)
    assert v > 0.25               # ~cruise


def test_corridor_drive_within_envelope():
    v, w = corridor_drive_command(0.0, math.pi / 2, 2.0, near_wall_m=0.35)  # extreme inputs
    assert -0.5 <= v <= 0.5 and -0.5 <= w <= 0.5


def test_profiled_turn_full_rate_when_far():
    w = profiled_turn_command(0.0, math.pi / 2, ang_decel=1.2, turn_w_max=0.35)
    assert abs(w - 0.35) < 1e-9


def test_profiled_turn_decelerates_when_close():
    w_small = profiled_turn_command(math.pi / 2 - 0.05, math.pi / 2, ang_decel=1.2, turn_w_max=0.35)
    w_tiny = profiled_turn_command(math.pi / 2 - 0.01, math.pi / 2, ang_decel=1.2, turn_w_max=0.35)
    assert 0.0 < w_tiny < w_small <= 0.35


def test_profiled_turn_sign_toward_target():
    assert profiled_turn_command(0.0, 0.3) > 0.0
    assert profiled_turn_command(0.0, -0.3) < 0.0


def test_profiled_turn_within_cap():
    w = profiled_turn_command(0.0, math.pi, turn_w_max=0.35)
    assert -0.35 <= w <= 0.35


def test_backout_command_reverses_straight_when_aligned():
    v, w = backout_command(math.pi / 2, math.pi / 2)
    assert v == -0.30 and abs(w) < 1e-9


def test_backout_command_holds_heading():
    # yaw ahead of hold_cardinal -> steer back (negative w), still reversing
    v, w = backout_command(0.2, 0.0)
    assert v < 0.0 and w < 0.0


def test_backout_command_custom_speed():
    v, _ = backout_command(0.0, 0.0, backout_v=0.22)
    assert v == -0.22


def test_grid_cross_track_all_dirs():
    # cell (5,5) centre (10,10); robot 0.4 m off-centre on the lateral axis
    assert abs(grid_cross_track(9.6, 10.0, (5, 5), (0, 1)) - 0.4) < 1e-9    # N: west=left  => +
    assert abs(grid_cross_track(10.4, 10.0, (5, 5), (0, 1)) + 0.4) < 1e-9   # N: east=right => -
    assert abs(grid_cross_track(10.4, 10.0, (5, 5), (0, -1)) - 0.4) < 1e-9  # S: east=left  => +
    assert abs(grid_cross_track(10.0, 10.4, (5, 5), (1, 0)) - 0.4) < 1e-9   # E: north=left => +
    assert abs(grid_cross_track(10.0, 10.4, (5, 5), (-1, 0)) + 0.4) < 1e-9  # W: north=right=> -


def test_grid_cross_track_on_centerline_is_zero():
    assert grid_cross_track(10.0, 10.0, (5, 5), (0, 1)) == 0.0
    assert grid_cross_track(10.0, 10.0, (5, 5), (1, 0)) == 0.0


def test_cross_track_steer_is_capped():
    # huge cross_track, robot aligned to cardinal (yaw=0): steering capped to kp_ang*max_cross_steer
    v, w = corridor_drive_command(0.0, 0.0, 2.0, None, kp_ang=1.5, lookahead_m=0.7, max_cross_steer=0.25)
    assert abs(w - (-1.5 * 0.25)) < 1e-6          # capped (uncapped would be atan2(-2,0.7)*1.5 -> clamp -0.5)


def test_small_cross_track_unaffected_by_cap():
    w = corridor_drive_command(0.0, 0.0, 0.10, None, kp_ang=1.5, lookahead_m=0.7, max_cross_steer=0.25)[1]
    assert abs(w - 1.5 * math.atan2(-0.10, 0.7)) < 1e-6   # below cap -> unchanged


def test_max_cross_steer_default_loosened():
    import inspect
    assert inspect.signature(corridor_drive_command).parameters['max_cross_steer'].default == 0.35
    assert inspect.signature(corridor_follow_command).parameters['max_cross_steer'].default == 0.35


def test_keepout_overrides_cross_steer_cap_when_close():
    # near a wall (clearance < safety_radius): emergency cap allows stronger centering than normal.
    # kp_ang=1.0, w_max=2.0 so w doesn't saturate and the two caps are distinguishable.
    w_close = corridor_drive_command(0.0, 0.0, 2.0, 0.5, kp_ang=1.0, w_max=2.0, lookahead_m=0.7,
                                     max_cross_steer=0.35, safety_radius=0.60, keepout_max_cross_steer=0.8)[1]
    w_far = corridor_drive_command(0.0, 0.0, 2.0, 1.0, kp_ang=1.0, w_max=2.0, lookahead_m=0.7,
                                   max_cross_steer=0.35, safety_radius=0.60, keepout_max_cross_steer=0.8)[1]
    assert abs(w_close - (-0.8)) < 1e-6     # clearance 0.5 < 0.60 -> emergency cap 0.8 -> w=-0.8
    assert abs(w_far - (-0.35)) < 1e-6      # clearance 1.0 >= 0.60 -> normal cap 0.35 -> w=-0.35


def test_repulsion_steers_away_from_near_right_wall():
    import math
    from tugbot_maze.hop_controller import corridor_follow_command
    yaw = math.pi / 2; cardinal = math.pi / 2                      # N travel; for N: left=W, right=E
    _, w0 = corridor_follow_command(yaw, cardinal, 0.85, 0.85, 0.85, safety_radius=0.70)  # centered
    _, w1 = corridor_follow_command(yaw, cardinal, 0.85, 0.30, 0.30, safety_radius=0.70)  # near right (E)
    assert w1 > w0                                                 # near right -> steer LEFT (+w, CCW toward W)


def test_repulsion_steers_away_from_near_left_wall():
    import math
    from tugbot_maze.hop_controller import corridor_follow_command
    yaw = math.pi / 2; cardinal = math.pi / 2
    _, w0 = corridor_follow_command(yaw, cardinal, 0.85, 0.85, 0.85, safety_radius=0.70)
    _, w1 = corridor_follow_command(yaw, cardinal, 0.30, 0.85, 0.30, safety_radius=0.70)  # near left (W)
    assert w1 < w0                                                 # near left -> steer RIGHT (-w)


def test_repulsion_noop_when_centered_and_clear():
    import math
    from tugbot_maze.hop_controller import corridor_follow_command
    yaw = math.pi / 2; cardinal = math.pi / 2
    v, w = corridor_follow_command(yaw, cardinal, 0.88, 0.88, 0.88, safety_radius=0.70)
    assert abs(w) < 1e-9 and v > 0.0                               # both sides > radius -> bias 0


def test_repulsion_stays_within_cross_track_envelope():
    import math
    from tugbot_maze.hop_controller import corridor_follow_command
    yaw = math.pi / 2; cardinal = math.pi / 2
    _, w = corridor_follow_command(yaw, cardinal, 0.85, 0.05, 0.05, safety_radius=0.70,
                                   keepout_repulse_gain=5.0, max_cross_track_m=0.6, w_max=0.5)
    assert abs(w) <= 0.5 + 1e-9
