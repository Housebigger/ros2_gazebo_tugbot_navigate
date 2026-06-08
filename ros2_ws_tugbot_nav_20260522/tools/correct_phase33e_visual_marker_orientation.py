#!/usr/bin/env python3
"""Phase33E visual marker orientation correction.

Yaw-only correction for the refined scaled clean maze marker styles:
- entrance triangle arrow yaw += pi;
- exit checker finish band yaw += pi/2;
- marker x/y/z, Tugbot pose, wall geometry, scale factor, and collision policy
  are preserved.
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import yaml

ENTRANCE_MODEL = 'maze_entrance_arrow_visual'
FINISH_MODEL = 'maze_exit_finish_band_visual'
EXIT_MARKER_MODEL = 'maze_exit_marker'
TUGBOT_URI = 'model://tugbot'

# Phase33D audited yaw baselines. Phase33E is intentionally idempotent:
# repeated invocations restore the requested corrected yaw instead of accumulating
# another +pi/+pi/2 rotation.
ENTRANCE_YAW_BEFORE_BASELINE = 0.0
FINISH_YAW_BEFORE_BASELINE = 1.571


def parse_floats(text: str) -> list[float]:
    return [float(v) for v in text.split()]


def fmt_pose(values: list[float]) -> str:
    return ' '.join(f'{v:.3f}' for v in values)


def normalize_angle(angle: float) -> float:
    # Normalize to [-pi, pi]. Use +pi rather than -pi at the boundary for the
    # entrance yaw so the human-readable report matches requested +pi rotation.
    normalized = math.atan2(math.sin(angle), math.cos(angle))
    if math.isclose(normalized, -math.pi, abs_tol=1e-9) and angle > 0:
        return math.pi
    return normalized


def angle_delta(after: float, before: float) -> float:
    return normalize_angle(after - before)


def sha256(path: Path) -> str | None:
    if not path.exists():
        return None
    return hashlib.sha256(path.read_bytes()).hexdigest()


def world_from_tree(tree: ET.ElementTree) -> ET.Element:
    root = tree.getroot()
    if root is None:
        raise ValueError('missing root')
    world = root.find('.//world')
    if world is None:
        raise ValueError('missing world')
    return world


def wall_geometry(world: ET.Element) -> dict[str, dict[str, list[float]]]:
    out: dict[str, dict[str, list[float]]] = {}
    for model in world.findall('model'):
        name = model.get('name') or ''
        if not name.startswith('maze_wall'):
            continue
        out[name] = {
            'pose': parse_floats(model.findtext('pose') or ''),
            'size': parse_floats(model.findtext('./link/collision/geometry/box/size') or ''),
        }
    return out


def pose_of(model: ET.Element) -> list[float]:
    pose_text = model.findtext('pose')
    if not pose_text:
        raise ValueError(f'model {model.get("name")} missing pose')
    values = parse_floats(pose_text)
    if len(values) != 6:
        raise ValueError(f'model {model.get("name")} pose has {len(values)} fields')
    return values


def set_pose(model: ET.Element, pose: list[float]) -> None:
    pose_el = model.find('pose')
    if pose_el is None:
        pose_el = ET.SubElement(model, 'pose')
    pose_el.text = fmt_pose(pose)


def almost_equal_list(a: list[float], b: list[float], tol: float = 1e-9) -> bool:
    return len(a) == len(b) and all(abs(x - y) <= tol for x, y in zip(a, b))


def geometries_equal(a: dict[str, Any], b: dict[str, Any], tol: float = 1e-9) -> bool:
    if a.keys() != b.keys():
        return False
    for key in a:
        for field in ('pose', 'size'):
            if not almost_equal_list(a[key][field], b[key][field], tol):
                return False
    return True


def get_tugbot_pose(world: ET.Element) -> list[float] | None:
    for inc in world.findall('include'):
        if (inc.findtext('uri') or '').strip() == TUGBOT_URI:
            return parse_floats(inc.findtext('pose') or '')
    return None


def markers_visual_only(models: dict[str, ET.Element]) -> bool:
    for name in [ENTRANCE_MODEL, FINISH_MODEL, EXIT_MARKER_MODEL]:
        model = models.get(name)
        if model is None or model.find('.//collision') is not None:
            return False
    return True


def update_metadata(path: Path, entrance_before: float, entrance_after: float, finish_before: float, finish_after: float) -> dict[str, Any]:
    data = yaml.safe_load(path.read_text())
    out = copy.deepcopy(data)
    markers = out.setdefault('markers', {})
    markers.setdefault('entrance_arrow', {})['yaw_rad'] = round(entrance_after, 6)
    markers.setdefault('exit_finish_band', {})['yaw_rad'] = round(finish_after, 6)
    markers['phase33e_orientation_correction'] = {
        'status': 'orientation_corrected_not_promoted',
        'coordinates_modified': False,
        'tugbot_pose_modified': False,
        'wall_geometry_modified': False,
        'scale_factor_modified': False,
        'collision_policy_modified': False,
        'entrance_arrow': {
            'yaw_before': round(entrance_before, 6),
            'yaw_delta': round(math.pi, 6),
            'yaw_after': round(entrance_after, 6),
            'normalization': '[-pi, pi]',
        },
        'exit_finish_band': {
            'yaw_before': round(finish_before, 6),
            'yaw_delta': round(math.pi / 2.0, 6),
            'yaw_after': round(finish_after, 6),
            'normalization': '[-pi, pi]',
        },
    }
    path.write_text(yaml.safe_dump(out, sort_keys=False), encoding='utf-8')
    return out


def indent_xml(elem: ET.Element, level: int = 0) -> None:
    i = '\n' + level * '  '
    if len(elem):
        if not elem.text or not elem.text.strip():
            elem.text = i + '  '
        children = list(elem)
        for child in children:
            indent_xml(child, level + 1)
        if children and (not children[-1].tail or not children[-1].tail.strip()):
            children[-1].tail = i
    if level and (not elem.tail or not elem.tail.strip()):
        elem.tail = i


def draw_entrance_preview(path: Path, before: float, after: float) -> None:
    img = np.full((440, 700, 3), 245, dtype=np.uint8)
    cv2.putText(img, 'Phase33E entrance arrow yaw correction', (35, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 90, 0), 2, cv2.LINE_AA)
    cv2.putText(img, f'before yaw={before:.3f} rad', (70, 95), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (60, 60, 60), 1, cv2.LINE_AA)
    cv2.putText(img, f'after yaw={after:.3f} rad (before + pi)', (370, 95), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (60, 60, 60), 1, cv2.LINE_AA)
    before_pts = np.array([[270, 200], [100, 125], [100, 275]], np.int32)
    after_pts = np.array([[430, 200], [600, 125], [600, 275]], np.int32)
    cv2.polylines(img, [before_pts], True, (0, 150, 0), 10, cv2.LINE_AA)
    cv2.polylines(img, [after_pts], True, (0, 150, 0), 10, cv2.LINE_AA)
    cv2.arrowedLine(img, (290, 200), (395, 200), (0, 0, 180), 3, tipLength=0.20)
    cv2.putText(img, 'rotate 180 deg about marker center; x/y/z unchanged', (80, 385), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (40, 40, 40), 1, cv2.LINE_AA)
    path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(path), img)


def draw_finish_preview(path: Path, before: float, after: float) -> None:
    img = np.full((440, 700, 3), 245, dtype=np.uint8)
    cv2.putText(img, 'Phase33E finish band yaw correction', (45, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (20, 20, 20), 2, cv2.LINE_AA)
    cv2.putText(img, f'before yaw={before:.3f}', (85, 95), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (60, 60, 60), 1, cv2.LINE_AA)
    cv2.putText(img, f'after yaw={after:.3f} (before + pi/2)', (365, 95), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (60, 60, 60), 1, cv2.LINE_AA)
    def checker(cx, cy, w, h):
        for c in range(4):
            for r in range(2):
                color = (25, 25, 25) if (c + r) % 2 == 0 else (245, 245, 245)
                x0 = int(cx - w/2 + c*w/4); x1 = int(cx - w/2 + (c+1)*w/4)
                y0 = int(cy - h/2 + r*h/2); y1 = int(cy - h/2 + (r+1)*h/2)
                cv2.rectangle(img, (x0,y0), (x1,y1), color, -1)
                cv2.rectangle(img, (x0,y0), (x1,y1), (80,80,80), 1)
    checker(180, 210, 210, 70)
    # After: 90-degree visual preview.
    checker(505, 210, 70, 210)
    cv2.arrowedLine(img, (300, 210), (370, 210), (0, 0, 180), 3, tipLength=0.25)
    cv2.putText(img, 'rotate 90 deg about marker center; x/y/z unchanged', (90, 385), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (40, 40, 40), 1, cv2.LINE_AA)
    path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(path), img)


def generate(world_path: Path, metadata_path: Path, artifact_dir: Path) -> dict[str, Any]:
    scaffold = Path('src/tugbot_gazebo/worlds/tugbot_maze_world.sdf')
    unscaled = Path('src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean.sdf')
    scaffold_before = sha256(scaffold)
    unscaled_before = sha256(unscaled)

    tree = ET.parse(world_path)
    world = world_from_tree(tree)
    models = {m.get('name') or '': m for m in world.findall('model')}
    walls_before = wall_geometry(world)
    tugbot_before = get_tugbot_pose(world)
    entrance_pose_current = pose_of(models[ENTRANCE_MODEL])
    finish_pose_current = pose_of(models[FINISH_MODEL])
    exit_marker_pose_before = pose_of(models[EXIT_MARKER_MODEL])

    entrance_yaw_before = ENTRANCE_YAW_BEFORE_BASELINE
    finish_yaw_before = FINISH_YAW_BEFORE_BASELINE
    entrance_yaw_after = normalize_angle(entrance_yaw_before + math.pi)
    finish_yaw_after = normalize_angle(finish_yaw_before + math.pi / 2.0)

    entrance_pose_before = entrance_pose_current.copy()
    finish_pose_before = finish_pose_current.copy()
    entrance_pose_before[5] = entrance_yaw_before
    finish_pose_before[5] = finish_yaw_before
    entrance_pose_after = entrance_pose_current.copy()
    finish_pose_after = finish_pose_current.copy()
    entrance_pose_after[5] = entrance_yaw_after
    finish_pose_after[5] = finish_yaw_after
    set_pose(models[ENTRANCE_MODEL], entrance_pose_after)
    set_pose(models[FINISH_MODEL], finish_pose_after)

    walls_after = wall_geometry(world)
    tugbot_after = get_tugbot_pose(world)
    exit_marker_pose_after = pose_of(models[EXIT_MARKER_MODEL])
    wall_same = geometries_equal(walls_before, walls_after)
    marker_xyz_same = almost_equal_list(entrance_pose_before[:5], entrance_pose_after[:5]) and almost_equal_list(finish_pose_before[:5], finish_pose_after[:5]) and almost_equal_list(exit_marker_pose_before, exit_marker_pose_after)
    tugbot_same = tugbot_before == tugbot_after
    visual_only = markers_visual_only(models)
    if not wall_same:
        raise RuntimeError('wall geometry changed')
    if not marker_xyz_same:
        raise RuntimeError('marker position or non-yaw fields changed')
    if not tugbot_same:
        raise RuntimeError('tugbot pose changed')
    if not visual_only:
        raise RuntimeError('marker collision policy changed')

    indent_xml(tree.getroot())
    tree.write(world_path, encoding='unicode', xml_declaration=False)
    update_metadata(metadata_path, entrance_pose_before[5], entrance_pose_after[5], finish_pose_before[5], finish_pose_after[5])

    artifact_dir.mkdir(parents=True, exist_ok=True)
    draw_entrance_preview(artifact_dir / 'entrance_arrow_orientation_preview.png', entrance_pose_before[5], entrance_pose_after[5])
    draw_finish_preview(artifact_dir / 'exit_finish_band_orientation_preview.png', finish_pose_before[5], finish_pose_after[5])

    summary = {
        'phase': 'Phase33E',
        'status': 'orientation_corrected_not_promoted',
        'world': str(world_path),
        'metadata': str(metadata_path),
        'normalization': '[-pi, pi]',
        'wall_count': len(walls_after),
        'wall_geometry_unchanged': wall_same,
        'marker_xyz_unchanged': marker_xyz_same,
        'tugbot_pose_unchanged': tugbot_same,
        'markers_visual_only': visual_only,
        'marker_collision_count': 0,
        'entrance_arrow': {
            'pose_before': entrance_pose_before,
            'pose_after': entrance_pose_after,
            'yaw_before': entrance_pose_before[5],
            'yaw_delta': math.pi,
            'yaw_after': entrance_pose_after[5],
        },
        'exit_finish_band': {
            'pose_before': finish_pose_before,
            'pose_after': finish_pose_after,
            'yaw_before': finish_pose_before[5],
            'yaw_delta': math.pi / 2.0,
            'yaw_after': finish_pose_after[5],
        },
        'exit_marker_pose_unchanged': almost_equal_list(exit_marker_pose_before, exit_marker_pose_after),
        'scaffold_sha256_before': scaffold_before,
        'scaffold_sha256_after': sha256(scaffold),
        'unscaled_clean_sha256_before': unscaled_before,
        'unscaled_clean_sha256_after': sha256(unscaled),
        'candidate_not_promoted': True,
        'navigation_started': False,
        'nav2_mppi_params_modified': False,
        'scale_factor_modified': False,
        'wall_segment_geometry_modified': False,
        'artifacts': ['marker_orientation_summary.json', 'entrance_arrow_orientation_preview.png', 'exit_finish_band_orientation_preview.png'],
    }
    (artifact_dir / 'marker_orientation_summary.json').write_text(json.dumps(summary, indent=2, sort_keys=True), encoding='utf-8')
    return summary


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--world', type=Path, required=True)
    parser.add_argument('--metadata', type=Path, required=True)
    parser.add_argument('--artifact-dir', type=Path, required=True)
    args = parser.parse_args()
    summary = generate(args.world, args.metadata, args.artifact_dir)
    print(json.dumps({
        'status': summary['status'],
        'orientation_corrected': True,
        'entrance_yaw_before': summary['entrance_arrow']['yaw_before'],
        'entrance_yaw_after': summary['entrance_arrow']['yaw_after'],
        'exit_yaw_before': summary['exit_finish_band']['yaw_before'],
        'exit_yaw_after': summary['exit_finish_band']['yaw_after'],
        'wall_geometry_unchanged': summary['wall_geometry_unchanged'],
    }, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
