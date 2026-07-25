"""Unit tests for the gt parallel accumulator's PURE helpers (the node wraps
them; sim integration is exercised by the Task 4 diag runs). Pins the A/B
fairness rules: nearest-stamp pairing with the production TF timeout as the
max pairing distance, and the exact world->map compose with the lidar mount.

Note: pairing is against the gt sample's RECEIPT stamp, not a stamp read out
of /gt/dynamic_pose -- measured (2026-07-25 gtstamp smoke) that the bridged
transforms[0].header.stamp is constant sec:0/nanosec:0, unusable. The
nearest_gt_sample/trim_gt_buffer helpers below are stamp-AGNOSTIC (they take
plain floats), so this receipt-time fix needed no helper-level test change."""
import math

import numpy as np
import pytest

from tugbot_maze.gt_cloud_accumulator import (
    nearest_gt_sample, compose_map_sensor, MAX_PAIR_DT_S,
    WORLD_TO_MAP_X, WORLD_TO_MAP_Y, LIDAR_MOUNT_Z)


def test_pairing_constants_mirror_production():
    # Production accumulator: TF lookup timeout 0.05 s at the cloud stamp.
    # The gt instrument must not be allowed a LOOSER pairing window.
    assert MAX_PAIR_DT_S == 0.05
    # Coordinate anchors from maze_20260528_scaled_instance.yaml.
    assert WORLD_TO_MAP_X == pytest.approx(11.011281)
    assert WORLD_TO_MAP_Y == pytest.approx(9.02507)
    assert LIDAR_MOUNT_Z == pytest.approx(0.55)   # tugbot contract mount


def test_nearest_gt_sample_picks_closest():
    buf = [(1.00, 'a'), (1.10, 'b'), (1.20, 'c')]
    assert nearest_gt_sample(buf, 1.12) == (1.10, 'b')
    assert nearest_gt_sample(buf, 1.19) == (1.20, 'c')


def test_nearest_gt_sample_rejects_beyond_max_dt():
    buf = [(1.00, 'a')]
    assert nearest_gt_sample(buf, 1.00 + MAX_PAIR_DT_S + 0.001) is None
    assert nearest_gt_sample(buf, 1.00 + MAX_PAIR_DT_S - 0.001) == (1.00, 'a')


def test_nearest_gt_sample_empty_buffer():
    assert nearest_gt_sample([], 1.0) is None


def test_compose_identity_pose_maps_origin():
    # Robot at world origin, identity rotation: a sensor-frame point (0,0,0)
    # lands at map (offset_x, offset_y, LIDAR_MOUNT_Z).
    T = compose_map_sensor(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
    p = T @ np.array([0.0, 0.0, 0.0, 1.0])
    assert p[0] == pytest.approx(WORLD_TO_MAP_X)
    assert p[1] == pytest.approx(WORLD_TO_MAP_Y)
    assert p[2] == pytest.approx(LIDAR_MOUNT_Z)


def test_compose_yawed_pose_rotates_sensor_points():
    # Robot at world (1, 2, 0), yaw +90deg: sensor +x becomes map +y.
    yaw = math.pi / 2
    qz, qw = math.sin(yaw / 2), math.cos(yaw / 2)
    T = compose_map_sensor(1.0, 2.0, 0.0, 0.0, 0.0, qz, qw)
    p = T @ np.array([2.0, 0.0, 0.0, 1.0])          # 2 m ahead of the sensor
    assert p[0] == pytest.approx(1.0 + WORLD_TO_MAP_X, abs=1e-6)
    assert p[1] == pytest.approx(2.0 + WORLD_TO_MAP_Y + 2.0, abs=1e-6)
    assert p[2] == pytest.approx(LIDAR_MOUNT_Z, abs=1e-6)


def test_trim_gt_buffer_drops_old_samples():
    from tugbot_maze.gt_cloud_accumulator import trim_gt_buffer, GT_BUFFER_S
    buf = [(0.0, 'old'), (1.0, 'mid'), (2.5, 'new')]
    out = trim_gt_buffer(buf, 2.5)
    assert (0.0, 'old') not in out and (2.5, 'new') in out
    assert all(t >= 2.5 - GT_BUFFER_S for t, _ in out)


def test_compose_passes_gt_z_through():
    # Task 1 review I2 documentation: gt z rides through the compose; the
    # two maps' z datums may differ and the metric must be datum-relative.
    T = compose_map_sensor(0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 1.0)
    p = T @ np.array([0.0, 0.0, 0.0, 1.0])
    assert p[2] == pytest.approx(0.5 + LIDAR_MOUNT_Z)
