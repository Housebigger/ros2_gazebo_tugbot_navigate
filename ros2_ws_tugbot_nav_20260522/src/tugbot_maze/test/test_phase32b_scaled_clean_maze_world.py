import hashlib
import json
import math
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[3]
BASE_WORLD = ROOT / 'src' / 'tugbot_gazebo' / 'worlds' / 'tugbot_maze_world_20260528_clean.sdf'
SCAFFOLD_WORLD = ROOT / 'src' / 'tugbot_gazebo' / 'worlds' / 'tugbot_maze_world.sdf'
SCALED_WORLD = ROOT / 'src' / 'tugbot_gazebo' / 'worlds' / 'tugbot_maze_world_20260528_clean_scaled2x.sdf'
SEGMENTS_YAML = ROOT / 'src' / 'tugbot_maze' / 'config' / 'maze_wall_segments_20260528.yaml'
METADATA = ROOT / 'src' / 'tugbot_maze' / 'config' / 'maze_20260528_scaled_instance.yaml'
GENERATOR = ROOT / 'tools' / 'generate_phase32b_scaled_clean_maze_world.py'
ARTIFACT_DIR = ROOT / 'log' / 'phase32b_scaled_clean_maze_world'


def _sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _world_models(path):
    tree = ET.parse(path)
    world = tree.getroot().find('.//world')
    assert world is not None
    return world, {m.get('name'): m for m in world.findall('model')}


def _wall_pose_size_map(path):
    _world, models = _world_models(path)
    out = {}
    for name, model in models.items():
        if not name or not name.startswith('maze_wall'):
            continue
        pose = [float(v) for v in (model.findtext('pose') or '').split()]
        size = [float(v) for v in (model.findtext('./link/collision/geometry/box/size') or '').split()]
        out[name] = {'pose': pose, 'size': size}
    return out


def _load_yaml(path):
    return yaml.safe_load(path.read_text())


def test_phase32b_scaled_metadata_schema():
    assert METADATA.exists()
    data = _load_yaml(METADATA)
    assert data['source_image'] == 'package://tugbot_maze/assets/maze_20260528.png'
    assert data['base_world'] == 'src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean.sdf'
    assert data['output_world'] == 'src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean_scaled2x.sdf'
    assert float(data['scale_factor']) == 2.0
    assert data['original_bounds_m'] == {'xmin': -6.0, 'xmax': 6.0, 'ymin': -6.0, 'ymax': 6.0}
    assert data['scaled_bounds_m'] == {'xmin': -12.0, 'xmax': 12.0, 'ymin': -12.0, 'ymax': 12.0}
    assert data['wall_thickness_policy']['mode'] == 'keep_original_thickness'
    assert float(data['wall_thickness_policy']['wall_thickness_m']) == 0.24
    assert data['wall_height_policy']['mode'] == 'keep_original_height'
    assert float(data['wall_height_policy']['wall_height_m']) == 1.2
    assert data['markers']['visual_only'] is True
    assert data['markers']['collision'] == 'none'
    assert data['entrance']['x_m'] == -12.0
    assert data['entrance']['y_m'] == -2.0
    assert data['exit']['x_m'] == 0.0
    assert data['exit']['y_m'] == 12.0


def test_phase32b_generator_creates_scaled_world_without_overwriting_inputs():
    assert GENERATOR.exists()
    base_before = _sha(BASE_WORLD)
    scaffold_before = _sha(SCAFFOLD_WORLD)
    result = subprocess.run(
        [
            sys.executable,
            str(GENERATOR),
            '--metadata', str(METADATA),
            '--segments-yaml', str(SEGMENTS_YAML),
            '--base-world', str(BASE_WORLD),
            '--output-world', str(SCALED_WORLD),
            '--artifact-dir', str(ARTIFACT_DIR),
        ],
        cwd=ROOT,
        check=True,
        text=True,
        capture_output=True,
    )
    assert 'scaled_wall_count' in result.stdout
    assert _sha(BASE_WORLD) == base_before
    assert _sha(SCAFFOLD_WORLD) == scaffold_before
    assert SCALED_WORLD.exists()
    assert SCALED_WORLD != BASE_WORLD
    assert SCALED_WORLD != SCAFFOLD_WORLD


def test_phase32b_wall_count_scale_and_thickness_contract():
    base = _wall_pose_size_map(BASE_WORLD)
    scaled = _wall_pose_size_map(SCALED_WORLD)
    assert len(scaled) == len(base) == 53
    for name, b in base.items():
        assert name in scaled
        s = scaled[name]
        assert math.isclose(s['pose'][0], b['pose'][0] * 2.0, abs_tol=0.002)
        assert math.isclose(s['pose'][1], b['pose'][1] * 2.0, abs_tol=0.002)
        assert math.isclose(s['pose'][2], b['pose'][2], abs_tol=0.001)
        b_long = max(b['size'][0], b['size'][1])
        s_long = max(s['size'][0], s['size'][1])
        b_thick = min(b['size'][0], b['size'][1])
        s_thick = min(s['size'][0], s['size'][1])
        assert math.isclose(s_long, b_long * 2.0, abs_tol=0.003)
        assert math.isclose(s_thick, b_thick, abs_tol=0.001)
        assert math.isclose(s['size'][2], b['size'][2], abs_tol=0.001)


def test_phase32b_entrance_exit_and_visual_only_markers():
    world, models = _world_models(SCALED_WORLD)
    includes = world.findall('include')
    tugbot_include = [inc for inc in includes if (inc.findtext('uri') or '').strip() == 'model://tugbot']
    assert tugbot_include
    pose = [float(v) for v in (tugbot_include[0].findtext('pose') or '').split()]
    assert pose[:3] == [-12.0, -2.0, 0.0]
    assert math.isclose(pose[5], 0.0, abs_tol=0.001)

    marker_names = {'maze_entrance_arrow_visual', 'maze_exit_finish_band_visual'}
    assert marker_names.issubset(models.keys())
    for name in marker_names:
        model = models[name]
        assert model.find('.//collision') is None, f'{name} must be visual-only without collision'
        pose_values = [float(v) for v in (model.findtext('pose') or '').split()]
        assert pose_values[2] >= 0.004
        assert model.find('.//visual') is not None
    exit_pose = [float(v) for v in (models['maze_exit_finish_band_visual'].findtext('pose') or '').split()]
    assert math.isclose(exit_pose[0], 0.0, abs_tol=0.001)
    assert math.isclose(exit_pose[1], 12.0, abs_tol=0.001)


def test_phase32b_artifacts_summary_and_sdf_parse():
    ET.parse(SCALED_WORLD)
    expected = [
        'scaled_sdf_plan_view.png',
        'entrance_arrow_exit_finish_overlay.png',
        'scale_comparison_summary.json',
    ]
    missing = [name for name in expected if not (ARTIFACT_DIR / name).exists()]
    assert not missing
    summary = json.loads((ARTIFACT_DIR / 'scale_comparison_summary.json').read_text())
    assert summary['status'] == 'scaled_candidate_generated_not_promoted'
    assert summary['scale_factor'] == 2.0
    assert summary['base_wall_count'] == 53
    assert summary['scaled_wall_count'] == 53
    assert summary['wall_thickness_preserved'] is True
    assert summary['wall_height_preserved'] is True
    assert summary['base_clean_world_preserved'] is True
    assert summary['scaffold_world_preserved'] is True
    assert summary['markers']['visual_only'] is True
    assert summary['markers']['collision_free'] is True
    assert summary['guardrails']['runtime_navigation_started'] is False
    assert summary['guardrails']['candidate_not_promoted'] is True
