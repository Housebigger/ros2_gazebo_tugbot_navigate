import hashlib
import json
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[3]
ASSET = ROOT / 'src' / 'tugbot_maze' / 'assets' / 'maze_20260528.png'
METADATA = ROOT / 'src' / 'tugbot_maze' / 'config' / 'maze_20260528_instance.yaml'
SEGMENTS_YAML = ROOT / 'src' / 'tugbot_maze' / 'config' / 'maze_wall_segments_20260528.yaml'
GENERATOR = ROOT / 'tools' / 'generate_phase31_clean_maze_world.py'
CANDIDATE_WORLD = ROOT / 'src' / 'tugbot_gazebo' / 'worlds' / 'tugbot_maze_world_20260528_clean.sdf'
SCAFFOLD_WORLD = ROOT / 'src' / 'tugbot_gazebo' / 'worlds' / 'tugbot_maze_world.sdf'
ARTIFACT_DIR = ROOT / 'log' / 'phase31_clean_maze_world_generation'


def _load_yaml(path):
    with path.open('r', encoding='utf-8') as f:
        return yaml.safe_load(f)


def test_phase31_clean_source_asset_and_metadata_schema():
    assert ASSET.exists(), 'clean source image must be archived into package assets'
    meta = _load_yaml(METADATA)
    assert meta['source_image'] == 'package://tugbot_maze/assets/maze_20260528.png'
    assert meta['source_image_kind'] == 'clean_2d_binary_maze'
    assert meta['conversion']['mode'] == 'clean_binary_image_to_wall_segments'
    assert meta['conversion']['candidate_not_promoted'] is True
    assert meta['conversion']['do_not_overwrite_scaffold_world'] is True
    assert meta['world_bounds_m'] == {'xmin': -6.0, 'xmax': 6.0, 'ymin': -6.0, 'ymax': 6.0}
    for key in ('entrance', 'exit'):
        assert meta[key]['status'] == 'candidate'
        assert -6.0 <= float(meta[key]['x_m']) <= 6.0
        assert -6.0 <= float(meta[key]['y_m']) <= 6.0
    assert meta['runtime_policy']['image_allowed_for_world_generation'] is True
    assert meta['runtime_policy']['image_allowed_for_runtime_path_planning'] is False


def test_phase31_generator_creates_outputs_without_overwriting_scaffold():
    assert GENERATOR.exists(), 'generator must exist'
    before_hash = hashlib.sha256(SCAFFOLD_WORLD.read_bytes()).hexdigest()
    result = subprocess.run(
        [
            sys.executable,
            str(GENERATOR),
            '--metadata',
            str(METADATA),
            '--output-segments',
            str(SEGMENTS_YAML),
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
    assert 'inner_wall_count' in result.stdout
    after_hash = hashlib.sha256(SCAFFOLD_WORLD.read_bytes()).hexdigest()
    assert before_hash == after_hash
    assert CANDIDATE_WORLD.exists()
    assert CANDIDATE_WORLD != SCAFFOLD_WORLD
    assert SEGMENTS_YAML.exists()


def test_phase31_wall_segment_yaml_schema_and_geometry():
    data = _load_yaml(SEGMENTS_YAML)
    assert data['schema_version'] == 1
    assert data['source_image_kind'] == 'clean_2d_binary_maze'
    assert data['conversion']['mode'] == 'clean_binary_image_to_wall_segments'
    assert data['generation_policy']['candidate_output'].endswith('tugbot_maze_world_20260528_clean.sdf')
    segments = data['segments']
    assert len(segments) >= 12
    inner = [seg for seg in segments if not seg.get('outer', False)]
    assert len(inner) >= 8
    ids = set()
    for seg in segments:
        assert seg['id'] not in ids
        ids.add(seg['id'])
        assert seg['orientation'] in {'horizontal', 'vertical'}
        p0 = seg['p0_px']
        p1 = seg['p1_px']
        assert len(p0) == 2 and len(p1) == 2
        if seg['orientation'] == 'horizontal':
            assert p0[1] == p1[1]
            assert abs(p1[0] - p0[0]) >= data['extraction']['min_segment_length_px']
        else:
            assert p0[0] == p1[0]
            assert abs(p1[1] - p0[1]) >= data['extraction']['min_segment_length_px']


def test_phase31_generated_sdf_contract():
    tree = ET.parse(CANDIDATE_WORLD)
    root = tree.getroot()
    world = root.find('.//world')
    assert world is not None
    assert world.get('name') == 'tugbot_maze_world_20260528_clean'
    model_names = [m.get('name') for m in world.findall('model')]
    wall_models = [name for name in model_names if name and name.startswith('maze_wall')]
    assert len(wall_models) >= 12
    assert 'maze_exit_marker' in model_names
    includes = world.findall('include')
    assert any((inc.findtext('uri') or '').strip() == 'model://tugbot' for inc in includes)
    for model in world.findall('model'):
        name = model.get('name') or ''
        if not name.startswith('maze_wall'):
            continue
        size_text = model.findtext('./link/collision/geometry/box/size')
        pose_text = model.findtext('pose')
        assert size_text and pose_text
        sx, sy, sz = [float(v) for v in size_text.split()]
        assert sz == 1.2
        assert (sx >= 0.20 and sy >= 0.20)
        assert (sx > sy) or (sy > sx)


def test_phase31_debug_artifacts_and_summary():
    expected = [
        '00_source_image.png',
        '01_binary_wall_mask.png',
        '02_detected_wall_segments_overlay.png',
        '03_generated_sdf_plan_view.png',
        '04_sdf_overlay_on_source_image.png',
        '05_entrance_exit_overlay.png',
        'phase31_generation_summary.json',
    ]
    missing = [name for name in expected if not (ARTIFACT_DIR / name).exists()]
    assert not missing
    summary = json.loads((ARTIFACT_DIR / 'phase31_generation_summary.json').read_text())
    assert summary['status'] == 'candidate_generated_not_promoted'
    assert summary['source_image_kind'] == 'clean_2d_binary_maze'
    assert summary['wall_model_count_total'] >= 12
    assert summary['inner_wall_count'] >= 8
    assert summary['baseline_scaffold_preserved'] is True
    assert summary['guardrails']['runtime_navigation_started'] is False
    assert summary['guardrails']['scaffold_world_overwritten'] is False
    assert summary['guardrails']['candidate_not_promoted'] is True
    for key in ('entrance', 'exit'):
        pose = summary[key]
        assert pose['status'] == 'candidate'
        assert -6.0 <= float(pose['x_m']) <= 6.0
        assert -6.0 <= float(pose['y_m']) <= 6.0
