"""Guard: the 3D lidar must exist centered on the back (the nav chain
assumes SCAN_OFFSET_X=0), the original CERBERUS front_laser pose
(-0.364, z 0.142, yaw -1.571 — off-center AND rotated) must NOT be
copied, and the legacy 2D scan_omni must be gone."""
import math
import re
from pathlib import Path


def _model_sdf_text():
    ws_src = Path(__file__).resolve().parents[2]
    return (ws_src / 'tugbot_description' / 'models' / 'anymal_c' / 'model.sdf').read_text()


def _lidar_block():
    sdf = _model_sdf_text()
    m = re.search(r'<sensor name="lidar_3d" type="gpu_lidar">(.*?)</sensor>', sdf, re.S)
    assert m is not None, 'lidar_3d sensor missing from model.sdf'
    return m.group(1)


def test_scan_omni_is_gone():
    assert 'scan_omni' not in _model_sdf_text()


def test_lidar_3d_pose_is_centered_and_unrotated():
    block = _lidar_block()
    m = re.search(r'<pose>([^<]+)</pose>', block)
    assert m is not None, 'lidar_3d has no pose'
    x, y, z, roll, pitch, yaw = (float(v) for v in m.group(1).split())
    assert abs(x) < 0.05 and abs(y) < 0.05, f'lidar off-center ({x},{y}): breaks SCAN_OFFSET_X=0'
    assert abs(yaw) < 0.05, f'lidar yaw={yaw}: the CERBERUS -1.571 must not be copied'
    assert abs(roll) < 0.01 and abs(pitch) < 0.01
    assert 0.2 < z < 0.5, f'lidar z={z}: expected the old scan_omni height (~0.35)'


def test_lidar_3d_spec_matches_cerberus_front_laser():
    block = _lidar_block()
    assert '<update_rate>10</update_rate>' in block
    h = re.search(r'<horizontal>(.*?)</horizontal>', block, re.S).group(1)
    v = re.search(r'<vertical>(.*?)</vertical>', block, re.S).group(1)
    assert '<samples>1800</samples>' in h
    assert '<samples>16</samples>' in v
    vmin = float(re.search(r'<min_angle>([^<]+)</min_angle>', v).group(1))
    vmax = float(re.search(r'<max_angle>([^<]+)</max_angle>', v).group(1))
    assert vmin == -vmax and math.isclose(vmax, 0.261799, abs_tol=1e-5)
    assert '<topic>/lidar/points</topic>' in block
