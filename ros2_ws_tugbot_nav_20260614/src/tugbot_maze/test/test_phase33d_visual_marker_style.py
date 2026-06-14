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
TOOL = ROOT / 'tools' / 'refine_phase33d_visual_marker_style.py'
ARTIFACT_DIR = ROOT / 'log' / 'phase33d_visual_marker_style'

_GENERATED = False

EXPECTED_TUGBOT = [-11.011, -9.025, 0.0, 0.0, 0.0, 0.0]
EXPECTED_ARROW = [-10.661, -9.025, 0.005, 0.0, 0.0, 0.0]
EXPECTED_FINISH = [10.061, 9.058, 0.005, 0.0, 0.0, math.pi / 2.0]
EXPECTED_EXIT_CENTER = [10.061, 9.058]


def _sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _floats(text: str):
    return [float(v) for v in text.split()]


def _world_models(path: Path):
    tree = ET.parse(path)
    world = tree.getroot().find('.//world')
    assert world is not None
    return world, {m.get('name'): m for m in world.findall('model')}


def _wall_pose_size_map(path: Path):
    world, models = _world_models(path)
    out = {}
    for name, model in models.items():
        if not name or not name.startswith('maze_wall'):
            continue
        out[name] = {
            'pose': _floats(model.findtext('pose') or ''),
            'size': _floats(model.findtext('./link/collision/geometry/box/size') or ''),
        }
    return out


def _visual_by_name(model: ET.Element, name: str):
    for visual in model.findall('./link/visual'):
        if visual.get('name') == name:
            return visual
    raise AssertionError(f'missing visual {name}')


def _box_size(visual: ET.Element):
    return _floats(visual.findtext('./geometry/box/size') or '')


def _ensure_generated():
    global _GENERATED
    if _GENERATED:
        return
    assert TOOL.exists(), 'Phase33D marker style tool must exist'
    scaffold_before = _sha(SCAFFOLD_WORLD)
    unscaled_before = _sha(UNSCALED_WORLD)
    result = subprocess.run(
        [sys.executable, str(TOOL), '--world', str(SCALED_WORLD), '--metadata', str(METADATA), '--artifact-dir', str(ARTIFACT_DIR)],
        cwd=ROOT,
        check=True,
        text=True,
        capture_output=True,
    )
    assert 'triangle_arrow' in result.stdout
    assert _sha(SCAFFOLD_WORLD) == scaffold_before
    assert _sha(UNSCALED_WORLD) == unscaled_before
    _GENERATED = True


def test_phase33d_wall_geometry_and_coordinates_preserved():
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
            assert all(math.isclose(a, b, abs_tol=0.001) for a, b in zip(_floats(inc.findtext('pose') or ''), EXPECTED_TUGBOT))
            break
    else:
        raise AssertionError('missing tugbot include')
    assert all(math.isclose(a, b, abs_tol=0.001) for a, b in zip(_floats(models['maze_entrance_arrow_visual'].findtext('pose') or ''), EXPECTED_ARROW))


def test_phase33d_entrance_marker_is_visual_only_triangle_arrow_not_square_block():
    _ensure_generated()
    _world, models = _world_models(SCALED_WORLD)
    arrow = models['maze_entrance_arrow_visual']
    assert arrow.find('.//collision') is None
    assert arrow.get('phase33d_style') == 'triangle_arrow'
    visuals = arrow.findall('./link/visual')
    assert len(visuals) == 3
    names = {v.get('name') for v in visuals}
    assert names == {'triangle_arrow_tip', 'triangle_arrow_left_edge', 'triangle_arrow_right_edge'}
    sizes = [_box_size(v) for v in visuals]
    assert all(size[2] <= 0.012 for size in sizes)
    edge_yaws = []
    for name in ['triangle_arrow_left_edge', 'triangle_arrow_right_edge']:
        pose = _floats(_visual_by_name(arrow, name).findtext('pose') or '')
        edge_yaws.append(pose[5])
    assert edge_yaws[0] > 0.25
    assert edge_yaws[1] < -0.25
    tip_size = _box_size(_visual_by_name(arrow, 'triangle_arrow_tip'))
    assert tip_size[0] < 0.70
    assert tip_size[1] < 0.35
    # No old square-like shaft/head visuals remain.
    assert 'shaft' not in names and 'head' not in names


def test_phase33d_finish_band_rotated_visual_only_and_spans_exit_width():
    _ensure_generated()
    _world, models = _world_models(SCALED_WORLD)
    finish = models['maze_exit_finish_band_visual']
    assert finish.find('.//collision') is None
    assert finish.get('phase33d_style') == 'rotated_checker_finish_band'
    pose = _floats(finish.findtext('pose') or '')
    assert math.isclose(pose[0], EXPECTED_EXIT_CENTER[0], abs_tol=0.001)
    assert math.isclose(pose[1], EXPECTED_EXIT_CENTER[1], abs_tol=0.001)
    assert math.isclose(abs(pose[5]), math.pi / 2.0, abs_tol=0.002)
    visuals = finish.findall('./link/visual')
    assert len(visuals) == 8
    xs = []
    ys = []
    for visual in visuals:
        assert visual.find('./geometry/box') is not None
        vpose = _floats(visual.findtext('pose') or '')
        size = _box_size(visual)
        xs.append(vpose[0])
        ys.append(vpose[1])
        assert size[2] <= 0.012
    # Local Y span becomes world/opening width after model yaw=90deg.
    assert max(ys) - min(ys) >= 1.45
    assert max(ys) - min(ys) <= 2.20
    assert max(xs) - min(xs) <= 0.80


def test_phase33d_exit_marker_no_visible_green_disk_covering_finish_band():
    _ensure_generated()
    _world, models = _world_models(SCALED_WORLD)
    exit_marker = models.get('maze_exit_marker')
    if exit_marker is not None:
        assert exit_marker.get('phase33d_style') in {'hidden_semantic_marker', 'removed_visual_marker'}
        assert exit_marker.find('.//collision') is None
        visuals = exit_marker.findall('./link/visual')
        assert len(visuals) <= 1
        for visual in visuals:
            transparency = float(visual.findtext('./material/transparency') or '0')
            size_text = visual.findtext('./geometry/cylinder/radius') or visual.findtext('./geometry/box/size') or ''
            tiny = '0.001' in size_text or '0.000' in size_text
            assert transparency >= 0.99 or tiny
    summary = json.loads((ARTIFACT_DIR / 'marker_style_summary.json').read_text())
    assert summary['exit_marker_visible_ground_disk_removed'] is True


def test_phase33d_artifacts_metadata_and_preservation_guards():
    _ensure_generated()
    expected = ['entrance_triangle_arrow_preview.png', 'exit_finish_band_rotated_preview.png', 'marker_style_summary.json']
    missing = [name for name in expected if not (ARTIFACT_DIR / name).exists()]
    assert not missing
    summary = json.loads((ARTIFACT_DIR / 'marker_style_summary.json').read_text())
    assert summary['status'] == 'visual_marker_style_refined_not_promoted'
    assert summary['wall_geometry_unchanged'] is True
    assert summary['coordinates_unchanged'] is True
    assert summary['markers_visual_only'] is True
    assert summary['candidate_not_promoted'] is True
    assert summary['navigation_started'] is False
    meta = yaml.safe_load(METADATA.read_text())
    assert meta['entrance']['x_m'] == -11.011281
    assert meta['exit']['x_m'] == 10.061281
    assert meta['markers']['entrance_arrow']['style'] == 'green_isosceles_triangle_arrow'
    assert meta['markers']['exit_finish_band']['style'] == 'rotated_black_white_checker_band'
    assert meta['markers']['exit_marker']['visible_ground_disk'] is False
    assert SCAFFOLD_WORLD.exists() and UNSCALED_WORLD.exists()
