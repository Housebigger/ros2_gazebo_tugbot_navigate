import math

from tugbot_maze.pose_tracking import (
    compose_2d, inverse_2d, odom_prior, quat_to_yaw,
    yaw_to_quat, map_to_odom)


def test_compose_identity_parent_returns_child():
    # parent->mid is identity -> parent->child == mid->child
    assert compose_2d((0.0, 0.0, 0.0), (3.0, 4.0, 1.0)) == (3.0, 4.0, 1.0)


def test_compose_pure_translation_adds():
    assert compose_2d((1.0, 2.0, 0.0), (3.0, 4.0, 0.0)) == (4.0, 6.0, 0.0)


def test_compose_rotated_parent_rotates_child_translation():
    # parent->mid rotated +90deg: the child's +x offset maps onto parent +y.
    x, y, yaw = compose_2d((0.0, 0.0, math.pi / 2), (1.0, 0.0, 0.0))
    assert math.isclose(x, 0.0, abs_tol=1e-9)
    assert math.isclose(y, 1.0, abs_tol=1e-9)
    assert math.isclose(yaw, math.pi / 2, abs_tol=1e-9)


def test_compose_full_rigid_transform():
    x, y, yaw = compose_2d((5.0, 5.0, math.pi / 2), (2.0, 0.0, math.pi / 2))
    assert math.isclose(x, 5.0, abs_tol=1e-9)
    assert math.isclose(y, 7.0, abs_tol=1e-9)
    assert math.isclose(yaw, math.pi, abs_tol=1e-9)


def test_freeze_and_track_round_trips_world_pose():
    # The odom_locked mechanism: a frozen map->odom offset composed with a live
    # odom->base_link reading yields the pose in the map (world) frame.
    map_to_odom = (2.0, 0.0, 0.0)
    odom_to_base = (1.5, 0.0, math.pi / 2)
    assert compose_2d(map_to_odom, odom_to_base) == (3.5, 0.0, math.pi / 2)


def test_quat_to_yaw_zero():
    assert math.isclose(quat_to_yaw(0.0, 0.0, 0.0, 1.0), 0.0, abs_tol=1e-9)


def test_quat_to_yaw_ninety_degrees():
    h = math.sqrt(2) / 2
    assert math.isclose(quat_to_yaw(0.0, 0.0, h, h), math.pi / 2, abs_tol=1e-6)


def test_quat_to_yaw_negative_ninety_degrees():
    h = math.sqrt(2) / 2
    assert math.isclose(quat_to_yaw(0.0, 0.0, -h, h), -math.pi / 2, abs_tol=1e-6)


def _close(a, b, tol=1e-9):
    return all(abs(x - y) < tol for x, y in zip(a, b))


def test_inverse_2d_is_group_inverse():
    p = (1.3, -2.1, 0.7)
    assert _close(compose_2d(inverse_2d(p), p), (0.0, 0.0, 0.0))
    assert _close(compose_2d(p, inverse_2d(p)), (0.0, 0.0, 0.0))


def test_odom_prior_no_motion_returns_last_corrected():
    corrected = (5.0, 3.0, 1.0)
    odom = (2.0, 2.0, 0.5)
    assert _close(odom_prior(corrected, odom, odom), corrected)


def test_odom_prior_applies_odom_delta_in_body_frame():
    # corrected pose faces +y (yaw=pi/2); odom advances +1 along its own x.
    corrected = (0.0, 0.0, math.pi / 2)
    last_odom = (0.0, 0.0, 0.0)
    cur_odom = (1.0, 0.0, 0.0)            # +1 m forward in odom/base x
    prior = odom_prior(corrected, last_odom, cur_odom)
    # forward in body frame (yaw=pi/2) maps to +y in map
    assert _close(prior, (0.0, 1.0, math.pi / 2), tol=1e-9)


def test_yaw_to_quat_known_angles():
    for yaw, (ez, ew) in [(0.0, (0.0, 1.0)),
                          (math.pi / 2, (math.sqrt(0.5), math.sqrt(0.5))),
                          (math.pi, (1.0, 0.0)),
                          (-math.pi / 2, (-math.sqrt(0.5), math.sqrt(0.5)))]:
        x, y, z, w = yaw_to_quat(yaw)
        assert (x, y) == (0.0, 0.0)
        assert math.isclose(z, ez, abs_tol=1e-9)
        assert math.isclose(w, ew, abs_tol=1e-9)


def test_map_to_odom_roundtrip():
    # For any map->base and odom->base, compose(map_to_odom, odom->base) == map->base
    map_base = (3.0, -2.0, 1.1)
    odom_base = (0.7, 0.4, 0.3)
    m2o = map_to_odom(map_base, odom_base)
    back = compose_2d(m2o, odom_base)
    for a, b in zip(back, map_base):
        assert math.isclose(a, b, abs_tol=1e-9)
