import hashlib
import importlib.util
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[3]
YAML_PATH = ROOT / 'src' / 'tugbot_maze' / 'config' / 'maze_wall_segments_20260522.yaml'
GENERATOR = ROOT / 'tools' / 'generate_phase29_image_faithful_world.py'
CANDIDATE_WORLD = ROOT / 'src' / 'tugbot_gazebo' / 'worlds' / 'tugbot_maze_world_image_faithful.sdf'
SCAFFOLD_WORLD = ROOT / 'src' / 'tugbot_gazebo' / 'worlds' / 'tugbot_maze_world.sdf'
ARTIFACT_DIR = ROOT / 'log' / 'phase29_maze_world_reconstruction'


def _load_generator():
    spec = importlib.util.spec_from_file_location('phase29_generator', GENERATOR)
    assert spec is not None
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _load_yaml():
    with YAML_PATH.open('r', encoding='utf-8') as f:
        return yaml.safe_load(f)


def _run_generator():
    result = subprocess.run(
        [
            sys.executable,
            str(GENERATOR),
            '--segments-yaml',
            str(YAML_PATH),
            '--output-world',
            str(CANDIDATE_WORLD),
            '--artifact-dir',
            str(ARTIFACT_DIR),
        ],
        cwd=ROOT,
        check=True,
        text=True,
        capture_output=True,
    )
    assert result.stdout.strip()


def _wall_models(path):
    root = ET.parse(path).getroot()
    models = []
    for model in root.findall('.//world/model'):
        name = model.get('name') or ''
        if name.startswith('maze_wall'):
            models.append(model)
    return models


def test_phase29_wall_segment_yaml_schema_and_review_contract():
    data = _load_yaml()
    assert data['schema_version'] == 1
    assert data['source_image'].endswith('maze_20260522.jpg')
    assert data['review_status'] == 'hybrid_manual_reviewed_candidate'
    assert data['generation_policy']['do_not_overwrite_scaffold_world'] is True
    assert data['transform']['image_width_px'] == 1000
    assert data['transform']['image_height_px'] == 1000
    assert data['transform']['world_bounds_m'] == {'xmin': -6.0, 'xmax': 6.0, 'ymin': -6.0, 'ymax': 6.0}
    assert data['wall_defaults']['height_m'] == 1.2
    assert 0.15 <= data['wall_defaults']['thickness_m'] <= 0.35
    assert len(data['segments']) >= 35
    inner = [s for s in data['segments'] if not s.get('outer', False)]
    assert len(inner) > 5
    for segment in data['segments']:
        assert {'id', 'orientation', 'p0_pct', 'p1_pct'} <= set(segment)
        assert segment['orientation'] in {'horizontal', 'vertical'}
        assert len(segment['p0_pct']) == 2
        assert len(segment['p1_pct']) == 2
        for value in [*segment['p0_pct'], *segment['p1_pct']]:
            assert 0.0 <= float(value) <= 100.0


def test_phase29_image_world_transform_entrance_exit_consistency():
    module = _load_generator()
    data = _load_yaml()
    transform = module.ImageWorldTransform.from_config(data['transform'])
    assert transform.pixel_pct_to_world_xy((50.0, 50.0)) == (0.0, 0.0)
    assert transform.pixel_pct_to_world_xy((0.0, 100.0)) == (-6.0, -6.0)
    assert transform.pixel_pct_to_world_xy((100.0, 0.0)) == (6.0, 6.0)
    entrance = data['entrance']
    exit_cfg = data['exit']
    assert transform.pixel_pct_to_world_xy(tuple(entrance['pixel_pct'])) == (entrance['x_m'], entrance['y_m'])
    assert transform.pixel_pct_to_world_xy(tuple(exit_cfg['pixel_pct'])) == (exit_cfg['x_m'], exit_cfg['y_m'])


def test_phase29_generator_creates_candidate_world_with_many_inner_walls_and_artifacts():
    _run_generator()
    assert CANDIDATE_WORLD.exists()
    walls = _wall_models(CANDIDATE_WORLD)
    assert len(walls) >= 35
    inner = [m for m in walls if not (m.get('name') or '').startswith('maze_wall_outer_')]
    assert len(inner) >= 30
    world_text = CANDIDATE_WORLD.read_text()
    assert '<world name="tugbot_maze_world_image_faithful">' in world_text
    assert 'maze_exit_marker' in world_text
    for artifact in [
        'phase29_source_with_wall_segment_overlay.png',
        'phase29_generated_sdf_plan_view.png',
        'phase29_sdf_overlay_on_source_image.png',
        'phase29_entrance_exit_overlay.png',
        'phase29_generation_summary.json',
    ]:
        assert (ARTIFACT_DIR / artifact).exists(), artifact


def test_phase29_generator_does_not_overwrite_scaffold_world():
    before = hashlib.sha256(SCAFFOLD_WORLD.read_bytes()).hexdigest()
    _run_generator()
    after = hashlib.sha256(SCAFFOLD_WORLD.read_bytes()).hexdigest()
    assert after == before
    assert CANDIDATE_WORLD != SCAFFOLD_WORLD


def test_phase29_generated_world_segment_pose_size_contract():
    _run_generator()
    data = _load_yaml()
    wall_defaults = data['wall_defaults']
    root = ET.parse(CANDIDATE_WORLD).getroot()
    generated = [m for m in root.findall('.//world/model') if (m.get('name') or '').startswith('maze_wall_seg_')]
    assert generated
    for model in generated:
        pose = [float(v) for v in (model.findtext('pose') or '').split()]
        size = [float(v) for v in (model.findtext('.//collision/geometry/box/size') or '').split()]
        assert len(pose) == 6
        assert len(size) == 3
        assert abs(size[2] - wall_defaults['height_m']) < 1e-9
        assert max(size[0], size[1]) > min(size[0], size[1])
        assert abs(min(size[0], size[1]) - wall_defaults['thickness_m']) < 1e-9
