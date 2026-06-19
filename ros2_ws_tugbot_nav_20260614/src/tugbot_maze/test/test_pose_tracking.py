import math

from tugbot_maze.pose_tracking import compose_2d, quat_to_yaw


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
