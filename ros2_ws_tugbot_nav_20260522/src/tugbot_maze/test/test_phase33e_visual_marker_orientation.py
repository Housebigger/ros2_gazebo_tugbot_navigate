import hashlib
import json
import math
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[3]
SCALED_WORLD = ROOT / 'src' / 'tugbot_gazebo' / 'worlds' / 'tugbot_maze_world_20260528_clean_scaled2x.sdf'
UNSCALED_WORLD = ROOT / 'src' / 'tugbot_gazebo' / 'worlds' / 'tugbot_maze_world_20260528_clean.sdf'
SCAFFOLD_WORLD = ROOT / 'src' / 'tugbot_gazebo' / 'worlds' / 'tugbot_maze_world.sdf'
METADATA = ROOT / 'src' / 'tugbot_maze' / 'config' / 'maze_20260528_scaled_instance.yaml'
TOOL = ROOT / 'tools' / 'correct_phase33e_visual_marker_orientation.py'
ARTIFACT_DIR = ROOT / 'log' / 'phase33e_visual_marker_orientation'

_GENERATED = False

ENTRANCE_XYZ_BEFORE = [-10.661, -9.025, 0.005]
ENTRANCE_YAW_BEFORE = 0.0
ENTRANCE_YAW_AFTER = math.pi
FINISH_XYZ_BEFORE = [10.061, 9.058, 0.005]
FINISH_YAW_BEFORE = 1.571
FINISH_YAW_AFTER = -math.pi  # normalized from 1.571 + pi/2 ~= pi
TUGBOT_POSE_EXPECTED = [-11.011, -9.025, 0.0, 0.0, 0.0, 0.0]


def _sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _floats(text: str):
    return [float(v) for v in text.split()]


def _angle_close(a: float, b: float, tol: float = 0.003) -> bool:
    d = math.atan2(math.sin(a - b), math.cos(a - b))
    return abs(d) <= tol


def _world_models(path: Path):
    tree = ET.parse(path)
    world = tree.getroot().find('.//world')
    assert world is not None
    return world, {m.get('name'): m for m in world.findall('model')}


def _wall_pose_size_map(path: Path):
    _world, models = _world_models(path)
    out = {}
    for name, model in models.items():
        if not name or not name.startswith('maze_wall'):
            continue
        out[name] = {
            'pose': _floats(model.findtext('pose') or ''),
            'size': _floats(model.findtext('./link/collision/geometry/box/size') or ''),
        }
    return out


def _ensure_generated():
    global _GENERATED
    if _GENERATED:
        return
    assert TOOL.exists(), 'Phase33E marker orientation correction tool must exist'
    scaffold_before = _sha(SCAFFOLD_WORLD)
    unscaled_before = _sha(UNSCALED_WORLD)
    result = subprocess.run(
        [sys.executable, str(TOOL), '--world', str(SCALED_WORLD), '--metadata', str(METADATA), '--artifact-dir', str(ARTIFACT_DIR)],
        cwd=ROOT,
        check=True,
        text=True,
        capture_output=True,
    )
    assert 'orientation_corrected' in result.stdout
    assert _sha(SCAFFOLD_WORLD) == scaffold_before
    assert _sha(UNSCALED_WORLD) == unscaled_before
    _GENERATED = True


def test_phase33e_wall_geometry_tugbot_and_marker_xyz_preserved():
    _ensure_generated()
    base = _wall_pose_size_map(UNSCALED_WORLD)
    scaled = _wall_pose_size_map(SCALED_WORLD)
    assert len(scaled) == len(base) == 53
    for name, b in base.items():
        s = scaled[name]
        assert math.isclose(s['pose'][0], b['pose'][0] * 2.0, abs_tol=0.002)
        assert math.isclose(s['pose'][1], b['pose'][1] * 2.0, abs_tol=0.002)
        assert math.isclose(s['pose'][2], b['pose'][2], abs_tol=0.001)
        assert math.isclose(max(s['size'][0], s['size'][1]), max(b['size'][0], b['size'][1]) * 2.0, abs_tol=0.003)
        assert math.isclose(min(s['size'][0], s['size'][1]), min(b['size'][0], b['size'][1]), abs_tol=0.001)
        assert math.isclose(s['size'][2], b['size'][2], abs_tol=0.001)
    world, models = _world_models(SCALED_WORLD)
    for inc in world.findall('include'):
        if (inc.findtext('uri') or '').strip() == 'model://tugbot':
            assert all(math.isclose(a, b, abs_tol=0.001) for a, b in zip(_floats(inc.findtext('pose') or ''), TUGBOT_POSE_EXPECTED))
            break
    else:
        raise AssertionError('missing tugbot include')
    entrance_pose = _floats(models['maze_entrance_arrow_visual'].findtext('pose') or '')
    finish_pose = _floats(models['maze_exit_finish_band_visual'].findtext('pose') or '')
    assert all(math.isclose(a, b, abs_tol=0.001) for a, b in zip(entrance_pose[:3], ENTRANCE_XYZ_BEFORE))
    assert all(math.isclose(a, b, abs_tol=0.001) for a, b in zip(finish_pose[:3], FINISH_XYZ_BEFORE))


def test_phase33e_marker_yaws_rotated_by_requested_amounts():
    _ensure_generated()
    _world, models = _world_models(SCALED_WORLD)
    entrance_pose = _floats(models['maze_entrance_arrow_visual'].findtext('pose') or '')
    finish_pose = _floats(models['maze_exit_finish_band_visual'].findtext('pose') or '')
    assert _angle_close(entrance_pose[5], ENTRANCE_YAW_AFTER)
    assert _angle_close(finish_pose[5], FINISH_YAW_AFTER)
    summary = json.loads((ARTIFACT_DIR / 'marker_orientation_summary.json').read_text())
    assert _angle_close(summary['entrance_arrow']['yaw_before'], ENTRANCE_YAW_BEFORE)
    assert _angle_close(summary['entrance_arrow']['yaw_after'], ENTRANCE_YAW_AFTER)
    assert _angle_close(summary['exit_finish_band']['yaw_before'], FINISH_YAW_BEFORE)
    assert _angle_close(summary['exit_finish_band']['yaw_after'], FINISH_YAW_AFTER)
    assert _angle_close(summary['entrance_arrow']['yaw_delta'], math.pi)
    assert _angle_close(summary['exit_finish_band']['yaw_delta'], math.pi / 2.0)


def test_phase33e_markers_remain_visual_only_and_styles_preserved():
    _ensure_generated()
    _world, models = _world_models(SCALED_WORLD)
    expected_styles = {
        'maze_entrance_arrow_visual': 'triangle_arrow',
        'maze_exit_finish_band_visual': 'rotated_checker_finish_band',
        'maze_exit_marker': 'hidden_semantic_marker',
    }
    for name, style in expected_styles.items():
        model = models[name]
        assert model.get('phase33d_style') == style
        assert model.find('.//collision') is None
        assert len(model.findall('./link/visual')) >= 1
    exit_marker_pose = _floats(models['maze_exit_marker'].findtext('pose') or '')
    assert all(math.isclose(a, b, abs_tol=0.001) for a, b in zip(exit_marker_pose, [10.061, 9.058, 0.010, 0.0, 0.0, 0.0]))


def test_phase33e_metadata_records_before_after_yaws_without_coordinate_changes():
    _ensure_generated()
    meta = yaml.safe_load(METADATA.read_text())
    assert meta['scale_factor'] == 2.0
    assert meta['markers']['entrance_arrow']['x_m'] == -10.661281
    assert meta['markers']['entrance_arrow']['y_m'] == -9.02507
    assert meta['markers']['exit_finish_band']['x_m'] == 10.061281
    assert meta['markers']['exit_finish_band']['y_m'] == 9.058496
    correction = meta['markers']['phase33e_orientation_correction']
    assert correction['status'] == 'orientation_corrected_not_promoted'
    assert correction['coordinates_modified'] is False
    assert correction['wall_geometry_modified'] is False
    assert _angle_close(correction['entrance_arrow']['yaw_before'], ENTRANCE_YAW_BEFORE)
    assert _angle_close(correction['entrance_arrow']['yaw_after'], ENTRANCE_YAW_AFTER)
    assert _angle_close(correction['exit_finish_band']['yaw_before'], FINISH_YAW_BEFORE)
    assert _angle_close(correction['exit_finish_band']['yaw_after'], FINISH_YAW_AFTER)
    assert _angle_close(meta['markers']['entrance_arrow']['yaw_rad'], ENTRANCE_YAW_AFTER)
    assert _angle_close(meta['markers']['exit_finish_band']['yaw_rad'], FINISH_YAW_AFTER)


def test_phase33e_artifacts_and_preservation_guards():
    _ensure_generated()
    expected = ['marker_orientation_summary.json', 'entrance_arrow_orientation_preview.png', 'exit_finish_band_orientation_preview.png']
    missing = [name for name in expected if not (ARTIFACT_DIR / name).exists()]
    assert not missing
    summary = json.loads((ARTIFACT_DIR / 'marker_orientation_summary.json').read_text())
    assert summary['status'] == 'orientation_corrected_not_promoted'
    assert summary['wall_count'] == 53
    assert summary['wall_geometry_unchanged'] is True
    assert summary['marker_xyz_unchanged'] is True
    assert summary['tugbot_pose_unchanged'] is True
    assert summary['markers_visual_only'] is True
    assert summary['candidate_not_promoted'] is True
    assert summary['navigation_started'] is False
    assert summary['scaffold_sha256_before'] == summary['scaffold_sha256_after']
    assert summary['unscaled_clean_sha256_before'] == summary['unscaled_clean_sha256_after']
    assert SCAFFOLD_WORLD.exists() and UNSCALED_WORLD.exists()
