"""Guard: the forward observation camera must exist and face +x (the travel
direction). The 7 original CERBERUS cameras all face -x (CERBERUS calls -x
"front"); copying any of their poses points the camera backward — that exact
mistake is what this file pins down. See the 2026-07-17 dog-front-camera spec."""
import math
import re
from pathlib import Path


def _model_sdf_text():
    # workspace layout: <ws>/src/tugbot_maze/test/  ->  <ws>/src/tugbot_description/...
    ws_src = Path(__file__).resolve().parents[2]
    return (ws_src / 'tugbot_description' / 'models' / 'anymal_c' / 'model.sdf').read_text()


def _camera_block():
    sdf = _model_sdf_text()
    m = re.search(r'<sensor name="camera_front" type="camera">(.*?)</sensor>', sdf, re.S)
    assert m is not None, 'camera_front sensor missing from model.sdf'
    return m.group(1)


def test_camera_front_faces_travel_direction():
    block = _camera_block()
    m = re.search(r'<pose>([^<]+)</pose>', block)
    assert m is not None, 'camera_front has no pose'
    x, y, z, roll, pitch, yaw = (float(v) for v in m.group(1).split())
    assert x > 0.3, f'camera x={x}: must sit at the +x (travel) end, not the CERBERUS head end'
    assert abs(yaw) < 0.3, f'camera yaw={yaw}: must face +x; yaw~pi means the original -x pose was copied'
    assert abs(pitch) < 0.3 and abs(roll) < 0.1, f'camera tilted: roll={roll} pitch={pitch}'
    assert 0.05 < z < 0.4, f'camera z={z}: expected just above the shell'


def test_camera_front_spec_matches_cerberus_camera_4():
    block = _camera_block()
    assert '<update_rate>20</update_rate>' in block
    assert '<width>720</width>' in block
    assert '<height>540</height>' in block
    hfov = float(re.search(r'<horizontal_fov>([^<]+)</horizontal_fov>', block).group(1))
    assert math.isclose(hfov, 2.19911, abs_tol=1e-4)
    assert '<topic>/camera/front/image</topic>' in block
