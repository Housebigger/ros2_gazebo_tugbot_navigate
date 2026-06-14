import hashlib
import json
import math
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[3]
SEGMENTS_YAML = ROOT / 'src' / 'tugbot_maze' / 'config' / 'maze_wall_segments_20260528.yaml'
METADATA = ROOT / 'src' / 'tugbot_maze' / 'config' / 'maze_20260528_scaled_instance.yaml'
SCALED_WORLD = ROOT / 'src' / 'tugbot_gazebo' / 'worlds' / 'tugbot_maze_world_20260528_clean_scaled2x.sdf'
UNSCALED_WORLD = ROOT / 'src' / 'tugbot_gazebo' / 'worlds' / 'tugbot_maze_world_20260528_clean.sdf'
SCAFFOLD_WORLD = ROOT / 'src' / 'tugbot_gazebo' / 'worlds' / 'tugbot_maze_world.sdf'
TOOL = ROOT / 'tools' / 'align_phase33b_entrance_exit_markers.py'
ARTIFACT_DIR = ROOT / 'log' / 'phase33b_entrance_exit_marker_alignment'

_GENERATED = False


def _sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _load_yaml(path):
    return yaml.safe_load(path.read_text())


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


def _detect_openings():
    data = _load_yaml(SEGMENTS_YAML)
    segs = data['segments']
    width = data['transform']['image_width_px']
    height = data['transform']['image_height_px']
    outer = [s for s in segs if s.get('outer')]
    xs = []
    ys = []
    for seg in outer:
        xs.extend([seg['p0_px'][0], seg['p1_px'][0]])
        ys.extend([seg['p0_px'][1], seg['p1_px'][1]])
    xmin, xmax, ymin, ymax = min(xs), max(xs), min(ys), max(ys)

    sides = {'top': [], 'bottom': [], 'left': [], 'right': []}
    tol = 3
    for seg in segs:
        p0 = seg['p0_px']
        p1 = seg['p1_px']
        if seg['orientation'] == 'horizontal':
            y = p0[1]
            interval = sorted([p0[0], p1[0]])
            if abs(y - ymin) <= tol:
                sides['top'].append(interval)
            if abs(y - ymax) <= tol:
                sides['bottom'].append(interval)
        else:
            x = p0[0]
            interval = sorted([p0[1], p1[1]])
            if abs(x - xmin) <= tol:
                sides['left'].append(interval)
            if abs(x - xmax) <= tol:
                sides['right'].append(interval)

    def merge(intervals, gap=2):
        out = []
        for a, b in sorted(intervals):
            if not out or a > out[-1][1] + gap:
                out.append([a, b])
            else:
                out[-1][1] = max(out[-1][1], b)
        return out

    def gaps(intervals, start, end, min_gap=4):
        cur = start
        out = []
        for a, b in merge(intervals):
            if a - cur >= min_gap:
                out.append([cur, a])
            cur = max(cur, b)
        if end - cur >= min_gap:
            out.append([cur, end])
        return out

    def px_to_world(x, y):
        # Phase33B works on the 2x scaled world: original clean transform was x/y=-6..6.
        x_m = (-6.0 + x / (width - 1) * 12.0) * 2.0
        y_m = (6.0 - y / (height - 1) * 12.0) * 2.0
        return [x_m, y_m]

    openings = []
    for side in ['top', 'bottom']:
        y = ymin if side == 'top' else ymax
        inward = '-Y' if side == 'top' else '+Y'
        for a, b in gaps(sides[side], xmin, xmax):
            center = px_to_world((a + b) / 2.0, y)
            width_m = abs(px_to_world(b, y)[0] - px_to_world(a, y)[0])
            openings.append({'side': side, 'center_world': center, 'width_m': width_m, 'inward': inward})
    for side in ['left', 'right']:
        x = xmin if side == 'left' else xmax
        inward = '+X' if side == 'left' else '-X'
        for a, b in gaps(sides[side], ymin, ymax):
            center = px_to_world(x, (a + b) / 2.0)
            width_m = abs(px_to_world(x, b)[1] - px_to_world(x, a)[1])
            openings.append({'side': side, 'center_world': center, 'width_m': width_m, 'inward': inward})
    return openings


def _expected_openings():
    openings = _detect_openings()
    entrance = next(o for o in openings if o['side'] == 'left')
    exit_ = next(o for o in openings if o['side'] == 'right')
    return entrance, exit_


def _ensure_generated():
    global _GENERATED
    if _GENERATED:
        return
    assert TOOL.exists(), 'Phase33B alignment tool must exist'
    scaffold_before = _sha(SCAFFOLD_WORLD)
    unscaled_before = _sha(UNSCALED_WORLD)
    result = subprocess.run(
        [
            sys.executable,
            str(TOOL),
            '--metadata', str(METADATA),
            '--segments-yaml', str(SEGMENTS_YAML),
            '--scaled-world', str(SCALED_WORLD),
            '--artifact-dir', str(ARTIFACT_DIR),
        ],
        cwd=ROOT,
        check=True,
        text=True,
        capture_output=True,
    )
    assert 'selected_entrance_opening' in result.stdout
    assert _sha(SCAFFOLD_WORLD) == scaffold_before
    assert _sha(UNSCALED_WORLD) == unscaled_before
    _GENERATED = True


def test_phase33b_detected_openings_and_metadata_candidates():
    _ensure_generated()
    openings = _detect_openings()
    assert len(openings) >= 2
    sides = {o['side'] for o in openings}
    assert {'left', 'right'}.issubset(sides)
    entrance, exit_ = _expected_openings()
    meta = _load_yaml(METADATA)
    assert meta['entrance']['status'] == 'candidate_aligned_to_detected_boundary_opening'
    assert meta['exit']['status'] == 'candidate_aligned_to_detected_boundary_opening'
    assert math.isclose(meta['entrance']['opening_center_x_m'], entrance['center_world'][0], abs_tol=0.05)
    assert math.isclose(meta['entrance']['opening_center_y_m'], entrance['center_world'][1], abs_tol=0.05)
    assert math.isclose(meta['exit']['x_m'], exit_['center_world'][0], abs_tol=0.05)
    assert math.isclose(meta['exit']['y_m'], exit_['center_world'][1], abs_tol=0.05)
    assert meta['entrance']['inward_direction'] == '+X'
    assert meta['exit']['inward_direction'] == '-X'


def test_phase33b_wall_count_poses_and_sizes_unchanged_by_marker_alignment():
    _ensure_generated()
    base = _wall_pose_size_map(UNSCALED_WORLD)
    scaled = _wall_pose_size_map(SCALED_WORLD)
    assert len(scaled) == len(base) == 53
    for name, b in base.items():
        assert name in scaled
        s = scaled[name]
        assert math.isclose(s['pose'][0], b['pose'][0] * 2.0, abs_tol=0.002)
        assert math.isclose(s['pose'][1], b['pose'][1] * 2.0, abs_tol=0.002)
        assert math.isclose(s['pose'][2], b['pose'][2], abs_tol=0.001)
        assert math.isclose(max(s['size'][0], s['size'][1]), max(b['size'][0], b['size'][1]) * 2.0, abs_tol=0.003)
        assert math.isclose(min(s['size'][0], s['size'][1]), min(b['size'][0], b['size'][1]), abs_tol=0.001)
        assert math.isclose(s['size'][2], b['size'][2], abs_tol=0.001)


def test_phase33b_markers_visual_only_and_aligned_to_openings():
    _ensure_generated()
    entrance, exit_ = _expected_openings()
    _world, models = _world_models(SCALED_WORLD)
    arrow = models['maze_entrance_arrow_visual']
    finish = models['maze_exit_finish_band_visual']
    for model in (arrow, finish):
        assert model.find('.//collision') is None
        assert model.find('.//visual') is not None
        pose = [float(v) for v in (model.findtext('pose') or '').split()]
        assert pose[2] >= 0.004
    arrow_pose = [float(v) for v in (arrow.findtext('pose') or '').split()]
    finish_pose = [float(v) for v in (finish.findtext('pose') or '').split()]
    # Arrow is outside the left opening and points +X into the maze.
    assert arrow_pose[0] < entrance['center_world'][0]
    assert 0.25 <= entrance['center_world'][0] - arrow_pose[0] <= 1.25
    assert math.isclose(arrow_pose[1], entrance['center_world'][1], abs_tol=0.10)
    assert math.isclose(arrow_pose[5], 0.0, abs_tol=0.001)
    # Finish band is centered on the detected right-side exit opening.
    assert math.isclose(finish_pose[0], exit_['center_world'][0], abs_tol=0.10)
    assert math.isclose(finish_pose[1], exit_['center_world'][1], abs_tol=0.10)
    assert -12.0 <= finish_pose[0] <= 12.0
    assert -12.0 <= finish_pose[1] <= 12.0


def test_phase33b_summary_and_artifacts():
    _ensure_generated()
    expected = [
        'boundary_openings_overlay.png',
        'corrected_marker_overlay.png',
        'corrected_scaled_sdf_plan_view.png',
        'phase33b_marker_alignment_summary.json',
    ]
    missing = [name for name in expected if not (ARTIFACT_DIR / name).exists()]
    assert not missing
    summary = json.loads((ARTIFACT_DIR / 'phase33b_marker_alignment_summary.json').read_text())
    assert summary['status'] == 'marker_alignment_corrected_not_promoted'
    assert summary['wall_count_unchanged'] is True
    assert summary['wall_geometry_unchanged'] is True
    assert summary['markers']['visual_only'] is True
    assert summary['markers']['collision_free'] is True
    assert summary['selected_entrance_opening']['side'] == 'left'
    assert summary['selected_exit_opening']['side'] == 'right'
    assert summary['guardrails']['runtime_navigation_started'] is False
    assert summary['guardrails']['scaffold_world_overwritten'] is False
    assert summary['guardrails']['unscaled_clean_world_overwritten'] is False


def test_phase33b_tugbot_pose_and_preservation_guards():
    _ensure_generated()
    entrance, _exit = _expected_openings()
    _world, _models = _world_models(SCALED_WORLD)
    includes = _world.findall('include')
    tugbot = [inc for inc in includes if (inc.findtext('uri') or '').strip() == 'model://tugbot']
    assert tugbot
    pose = [float(v) for v in (tugbot[0].findtext('pose') or '').split()]
    assert pose[0] < entrance['center_world'][0]
    assert math.isclose(pose[1], entrance['center_world'][1], abs_tol=0.10)
    assert math.isclose(pose[5], 0.0, abs_tol=0.001)
    assert SCAFFOLD_WORLD.exists()
    assert UNSCALED_WORLD.exists()
