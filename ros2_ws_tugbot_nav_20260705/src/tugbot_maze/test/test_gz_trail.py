"""Unit tests for tools/gz_trail.py (pure functions only -- no Gazebo needed).
The script lives in the workspace tools/ dir, outside the package; import via path."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'tools'))
import gz_trail


# Two known `gz model -m <name> -p` output styles (separator differs across gz versions).
_POSE_OUTPUT_SPACES = """\
Requesting state for world [tugbot_maze_world_20260528_clean_scaled2x]...

Model: [8]
  - Name: tugbot
  - Pose [ XYZ (m) ] [ RPY (rad) ]:
    [10.500000 2.000000 0.100000]
    [0.000000 -0.000000 1.570000]
"""

_POSE_OUTPUT_PIPES = """\
Requesting state for world [tugbot_maze_world_20260528_clean_scaled2x]...

Model: [8]
  - Name: tugbot
  - Pose [ XYZ (m) ] [ RPY (rad) ]:
    [10.500000 | 2.000000 | 0.100000]
    [0.000000 | -0.000000 | 1.570000]
"""


def test_parse_pose_space_separated():
    assert gz_trail.parse_model_pose(_POSE_OUTPUT_SPACES) == (10.5, 2.0, 0.1)


def test_parse_pose_pipe_separated():
    assert gz_trail.parse_model_pose(_POSE_OUTPUT_PIPES) == (10.5, 2.0, 0.1)


def test_parse_pose_bad_input_returns_none():
    assert gz_trail.parse_model_pose('') is None
    assert gz_trail.parse_model_pose('Error: model not found') is None
    assert gz_trail.parse_model_pose('Pose but no numbers') is None


def test_should_record_first_point_always():
    assert gz_trail.should_record((1.0, 2.0, 0.0), None, 0.10) is True


def test_should_record_min_distance():
    last = (1.0, 2.0, 0.0)
    assert gz_trail.should_record((1.05, 2.0, 0.0), last, 0.10) is False   # 0.05 < 0.10
    assert gz_trail.should_record((1.20, 2.0, 0.0), last, 0.10) is True    # 0.20 >= 0.10
    # xy-plane distance only (z ignored)
    assert gz_trail.should_record((1.0, 2.0, 9.9), last, 0.10) is False
    # boundary: exactly min_dist away counts as recorded (>=, not >).
    # Zero-origin so dx*dx and min_dist*min_dist are the identical float computation.
    assert gz_trail.should_record((0.10, 0.0, 0.0), (0.0, 0.0, 0.0), 0.10) is True


def test_marker_request_line_strip_red():
    pts = [(1.0, 2.0, 0.0), (3.0, 4.0, 0.0)]
    req = gz_trail.marker_request(pts)
    assert 'type: LINE_STRIP' in req
    assert 'action: ADD_MODIFY' in req
    assert 'ns: "tugbot_trail"' in req
    assert 'id: 1' in req
    assert req.count('point {') == 2
    # red material, both diffuse and emissive so it reads bright regardless of lighting
    assert 'diffuse { r: 1 a: 1 }' in req
    assert 'emissive { r: 1 a: 1 }' in req
    # z lifted off the floor to avoid z-fighting
    assert 'z: 0.050' in req


def test_marker_request_point_coordinates():
    req = gz_trail.marker_request([(1.5, -2.25, 0.0)])
    assert 'point { x: 1.500 y: -2.250 z: 0.050 }' in req
