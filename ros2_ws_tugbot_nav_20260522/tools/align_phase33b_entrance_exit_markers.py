#!/usr/bin/env python3
"""Phase33B marker alignment for the scaled clean maze candidate world.

Detects boundary openings from the clean wall segment YAML and moves only the
visual-only entrance arrow / finish band plus candidate metadata. It does not
change maze wall geometry, scale factor, Nav2 parameters, or navigation strategy.
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


def load_yaml(path: Path) -> dict[str, Any]:
    with path.open('r', encoding='utf-8') as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict):
        raise ValueError(path)
    return data


def write_yaml(path: Path, data: dict[str, Any]) -> None:
    path.write_text(yaml.safe_dump(data, sort_keys=False), encoding='utf-8')


def sha256(path: Path) -> str | None:
    if not path.exists():
        return None
    return hashlib.sha256(path.read_bytes()).hexdigest()


def parse_floats(text: str) -> list[float]:
    return [float(v) for v in text.split()]


def fmt(values: list[float]) -> str:
    return ' '.join(f'{v:.3f}' for v in values)


def detect_boundary_openings(segment_data: dict[str, Any], scale: float = 2.0) -> list[dict[str, Any]]:
    segs = segment_data['segments']
    width = int(segment_data['transform']['image_width_px'])
    height = int(segment_data['transform']['image_height_px'])
    outer = [s for s in segs if s.get('outer')]
    xs: list[float] = []
    ys: list[float] = []
    for seg in outer:
        xs.extend([seg['p0_px'][0], seg['p1_px'][0]])
        ys.extend([seg['p0_px'][1], seg['p1_px'][1]])
    xmin, xmax, ymin, ymax = min(xs), max(xs), min(ys), max(ys)

    sides: dict[str, list[list[float]]] = {'top': [], 'bottom': [], 'left': [], 'right': []}
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

    def merge(intervals: list[list[float]], gap: float = 2.0) -> list[list[float]]:
        out: list[list[float]] = []
        for a, b in sorted(intervals):
            if not out or a > out[-1][1] + gap:
                out.append([a, b])
            else:
                out[-1][1] = max(out[-1][1], b)
        return out

    def gaps(intervals: list[list[float]], start: float, end: float, min_gap: float = 4.0) -> list[list[float]]:
        cur = start
        out: list[list[float]] = []
        for a, b in merge(intervals):
            if a - cur >= min_gap:
                out.append([cur, a])
            cur = max(cur, b)
        if end - cur >= min_gap:
            out.append([cur, end])
        return out

    def px_to_world(x: float, y: float) -> list[float]:
        x_m = (-6.0 + x / (width - 1) * 12.0) * scale
        y_m = (6.0 - y / (height - 1) * 12.0) * scale
        return [x_m, y_m]

    openings: list[dict[str, Any]] = []
    for side in ['top', 'bottom']:
        y = ymin if side == 'top' else ymax
        inward = '-Y' if side == 'top' else '+Y'
        for a, b in gaps(sides[side], xmin, xmax):
            center_px = [(a + b) / 2.0, y]
            center_world = px_to_world(center_px[0], center_px[1])
            width_m = abs(px_to_world(b, y)[0] - px_to_world(a, y)[0])
            openings.append({
                'side': side,
                'center_px': center_px,
                'center_world': center_world,
                'width_m': width_m,
                'inward': inward,
                'score_note': 'boundary_gap_detected_from_outer_wall_segments',
            })
    for side in ['left', 'right']:
        x = xmin if side == 'left' else xmax
        inward = '+X' if side == 'left' else '-X'
        for a, b in gaps(sides[side], ymin, ymax):
            center_px = [x, (a + b) / 2.0]
            center_world = px_to_world(center_px[0], center_px[1])
            width_m = abs(px_to_world(x, b)[1] - px_to_world(x, a)[1])
            openings.append({
                'side': side,
                'center_px': center_px,
                'center_world': center_world,
                'width_m': width_m,
                'inward': inward,
                'score_note': 'boundary_gap_detected_from_outer_wall_segments',
            })
    return openings


def select_openings(openings: list[dict[str, Any]]) -> tuple[dict[str, Any], dict[str, Any], list[dict[str, Any]]]:
    # Clean maze has two detected boundary gaps. Use lower-left/left-side gap as
    # entrance and the opposing upper-right/right-side gap as exit. If extra
    # candidates appear, rank by side preference then world position.
    entrance_candidates = sorted(
        openings,
        key=lambda o: (0 if o['side'] == 'left' else 1, abs(o['center_world'][0] + 10.0), o['center_world'][1]),
    )
    exit_candidates = sorted(
        openings,
        key=lambda o: (0 if o['side'] == 'right' else 1, -o['center_world'][1], abs(o['center_world'][0] - 10.0)),
    )
    return entrance_candidates[0], exit_candidates[0], exit_candidates


def model_name(model: ET.Element) -> str:
    return model.get('name') or ''


def get_world(root: ET.Element) -> ET.Element:
    world = root.find('.//world')
    if world is None:
        raise ValueError('missing world')
    return world


def wall_geometry(world: ET.Element) -> dict[str, dict[str, list[float]]]:
    out: dict[str, dict[str, list[float]]] = {}
    for model in world.findall('model'):
        name = model_name(model)
        if not name.startswith('maze_wall'):
            continue
        out[name] = {
            'pose': parse_floats(model.findtext('pose') or ''),
            'size': parse_floats(model.findtext('./link/collision/geometry/box/size') or ''),
        }
    return out


def geometries_equal(a: dict[str, Any], b: dict[str, Any], tol: float = 1e-9) -> bool:
    if a.keys() != b.keys():
        return False
    for key in a:
        for field in ['pose', 'size']:
            if len(a[key][field]) != len(b[key][field]):
                return False
            for x, y in zip(a[key][field], b[key][field]):
                if abs(float(x) - float(y)) > tol:
                    return False
    return True


def ensure_pose(model: ET.Element) -> ET.Element:
    pose = model.find('pose')
    if pose is None:
        pose = ET.SubElement(model, 'pose')
    return pose


def update_tugbot_pose(world: ET.Element, x: float, y: float, yaw: float) -> None:
    for include in world.findall('include'):
        if (include.findtext('uri') or '').strip() == 'model://tugbot':
            pose = include.find('pose')
            if pose is None:
                pose = ET.SubElement(include, 'pose')
            pose.text = fmt([x, y, 0.0, 0.0, 0.0, yaw])
            return


def update_marker_poses(world: ET.Element, entrance: dict[str, Any], exit_: dict[str, Any]) -> dict[str, Any]:
    ex, ey = entrance['center_world']
    gx, gy = exit_['center_world']
    # Place arrow just outside left opening and point +X. For other sides, handle
    # generically, though current selected entrance is left.
    offset = 0.60
    if entrance['side'] == 'left':
        arrow_x, arrow_y, yaw = ex - offset, ey, 0.0
        tugbot_x, tugbot_y = ex - 0.95, ey
    elif entrance['side'] == 'right':
        arrow_x, arrow_y, yaw = ex + offset, ey, math.pi
        tugbot_x, tugbot_y = ex + 0.95, ey
    elif entrance['side'] == 'top':
        arrow_x, arrow_y, yaw = ex, ey + offset, -math.pi / 2
        tugbot_x, tugbot_y = ex, ey + 0.95
    else:
        arrow_x, arrow_y, yaw = ex, ey - offset, math.pi / 2
        tugbot_x, tugbot_y = ex, ey - 0.95

    models = {model_name(m): m for m in world.findall('model')}
    arrow = models.get('maze_entrance_arrow_visual')
    finish = models.get('maze_exit_finish_band_visual')
    exit_marker = models.get('maze_exit_marker')
    if arrow is None or finish is None:
        raise ValueError('missing visual marker models')
    ensure_pose(arrow).text = fmt([arrow_x, arrow_y, 0.005, 0.0, 0.0, yaw])
    ensure_pose(finish).text = fmt([gx, gy, 0.005, 0.0, 0.0, 0.0])
    if exit_marker is not None:
        ensure_pose(exit_marker).text = fmt([gx, gy, 0.010, 0.0, 0.0, 0.0])
    update_tugbot_pose(world, tugbot_x, tugbot_y, yaw)
    return {
        'arrow_pose': [arrow_x, arrow_y, 0.005, 0.0, 0.0, yaw],
        'tugbot_pose': [tugbot_x, tugbot_y, 0.0, 0.0, 0.0, yaw],
        'finish_pose': [gx, gy, 0.005, 0.0, 0.0, 0.0],
        'exit_marker_pose': [gx, gy, 0.010, 0.0, 0.0, 0.0],
    }


def update_metadata(meta: dict[str, Any], entrance: dict[str, Any], exit_: dict[str, Any], marker_poses: dict[str, Any]) -> dict[str, Any]:
    out = copy.deepcopy(meta)
    ex, ey = entrance['center_world']
    gx, gy = exit_['center_world']
    out['entrance'].update({
        'status': 'candidate_aligned_to_detected_boundary_opening',
        'opening_side': entrance['side'],
        'opening_center_x_m': round(ex, 6),
        'opening_center_y_m': round(ey, 6),
        'opening_width_m': round(float(entrance['width_m']), 6),
        'inward_direction': entrance['inward'],
        'x_m': round(marker_poses['tugbot_pose'][0], 6),
        'y_m': round(marker_poses['tugbot_pose'][1], 6),
        'yaw_rad': round(marker_poses['tugbot_pose'][5], 6),
        'marker_x_m': round(marker_poses['arrow_pose'][0], 6),
        'marker_y_m': round(marker_poses['arrow_pose'][1], 6),
    })
    out['exit'].update({
        'status': 'candidate_aligned_to_detected_boundary_opening',
        'opening_side': exit_['side'],
        'opening_center_x_m': round(gx, 6),
        'opening_center_y_m': round(gy, 6),
        'opening_width_m': round(float(exit_['width_m']), 6),
        'inward_direction': exit_['inward'],
        'x_m': round(gx, 6),
        'y_m': round(gy, 6),
        'ambiguity_note': 'selected right-side upper opening from detected boundary openings; requires human confirmation',
    })
    out['markers']['entrance_arrow']['x_m'] = round(marker_poses['arrow_pose'][0], 6)
    out['markers']['entrance_arrow']['y_m'] = round(marker_poses['arrow_pose'][1], 6)
    out['markers']['entrance_arrow']['yaw_rad'] = round(marker_poses['arrow_pose'][5], 6)
    out['markers']['exit_finish_band']['x_m'] = round(gx, 6)
    out['markers']['exit_finish_band']['y_m'] = round(gy, 6)
    out['markers']['alignment_source'] = 'phase33b_detected_boundary_openings_from_maze_wall_segments_20260528'
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


def draw_overlay(path: Path, openings: list[dict[str, Any]], entrance: dict[str, Any], exit_: dict[str, Any], marker_poses: dict[str, Any]) -> None:
    img = np.full((900, 900, 3), 245, dtype=np.uint8)
    xmin = ymin = -12.0
    xmax = ymax = 12.0
    def px(x: float, y: float) -> tuple[int, int]:
        return (int(round((x - xmin) / (xmax - xmin) * 899)), int(round((ymax - y) / (ymax - ymin) * 899)))
    cv2.rectangle(img, px(-10.061, 10.061), px(10.061, -10.061), (80, 80, 80), 2)
    for o in openings:
        x, y = o['center_world']
        color = (0, 180, 255)
        if o is entrance:
            color = (0, 200, 0)
        elif o is exit_:
            color = (0, 0, 0)
        cv2.circle(img, px(x, y), 10, color, -1)
        cv2.putText(img, f"{o['side']} {o['width_m']:.2f}m", px(x + 0.2, y + 0.2), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)
    ax, ay = marker_poses['arrow_pose'][0], marker_poses['arrow_pose'][1]
    cv2.arrowedLine(img, px(ax - 0.5, ay), px(ax + 0.7, ay), (0, 180, 0), 4, tipLength=0.35)
    fx, fy = marker_poses['finish_pose'][0], marker_poses['finish_pose'][1]
    cv2.rectangle(img, px(fx - 0.45, fy - 0.25), px(fx + 0.45, fy + 0.25), (0, 0, 0), 2)
    path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(path), img)


def generate(metadata_path: Path, segments_yaml: Path, scaled_world_path: Path, artifact_dir: Path) -> dict[str, Any]:
    meta = load_yaml(metadata_path)
    segment_data = load_yaml(segments_yaml)
    openings = detect_boundary_openings(segment_data, scale=float(meta.get('scale_factor', 2.0)))
    entrance, exit_, exit_candidates = select_openings(openings)

    scaffold = Path(meta['baseline_preservation']['scaffold_world'])
    unscaled = Path(meta['baseline_preservation']['clean_candidate_world'])
    scaffold_before = sha256(scaffold)
    unscaled_before = sha256(unscaled)

    tree = ET.parse(scaled_world_path)
    root = tree.getroot()
    world = get_world(root)
    wall_before = wall_geometry(world)
    marker_poses = update_marker_poses(world, entrance, exit_)
    wall_after = wall_geometry(world)
    wall_unchanged = geometries_equal(wall_before, wall_after)
    if not wall_unchanged:
        raise RuntimeError('wall geometry changed during marker alignment')
    indent_xml(root)
    tree.write(scaled_world_path, encoding='unicode', xml_declaration=False)

    aligned_meta = update_metadata(meta, entrance, exit_, marker_poses)
    write_yaml(metadata_path, aligned_meta)

    corrected_root = ET.parse(scaled_world_path).getroot()
    corrected_world = get_world(corrected_root)
    models = {model_name(m): m for m in corrected_world.findall('model')}
    collision_free = all(models[n].find('.//collision') is None for n in ['maze_entrance_arrow_visual', 'maze_exit_finish_band_visual'])

    artifact_dir.mkdir(parents=True, exist_ok=True)
    draw_overlay(artifact_dir / 'boundary_openings_overlay.png', openings, entrance, exit_, marker_poses)
    draw_overlay(artifact_dir / 'corrected_marker_overlay.png', openings, entrance, exit_, marker_poses)
    draw_overlay(artifact_dir / 'corrected_scaled_sdf_plan_view.png', openings, entrance, exit_, marker_poses)

    summary = {
        'phase': 'Phase33B',
        'status': 'marker_alignment_corrected_not_promoted',
        'scaled_world': str(scaled_world_path),
        'metadata': str(metadata_path),
        'detected_openings': openings,
        'selected_entrance_opening': entrance,
        'selected_exit_opening': exit_,
        'exit_candidate_list': exit_candidates,
        'marker_poses': marker_poses,
        'wall_count_before': len(wall_before),
        'wall_count_after': len(wall_after),
        'wall_count_unchanged': len(wall_before) == len(wall_after),
        'wall_geometry_unchanged': wall_unchanged,
        'markers': {'visual_only': True, 'collision_free': collision_free},
        'scaffold_sha256_before': scaffold_before,
        'scaffold_sha256_after': sha256(scaffold),
        'unscaled_clean_sha256_before': unscaled_before,
        'unscaled_clean_sha256_after': sha256(unscaled),
        'guardrails': {
            'runtime_navigation_started': False,
            'nav2_mppi_params_modified': False,
            'fallback_terminal_acceptance_modified': False,
            'wall_segment_geometry_modified': False,
            'scale_factor_modified': False,
            'scaffold_world_overwritten': scaffold_before != sha256(scaffold) if scaffold_before else False,
            'unscaled_clean_world_overwritten': unscaled_before != sha256(unscaled) if unscaled_before else False,
            'candidate_not_promoted': True,
        },
        'artifacts': [
            'boundary_openings_overlay.png',
            'corrected_marker_overlay.png',
            'corrected_scaled_sdf_plan_view.png',
            'phase33b_marker_alignment_summary.json',
        ],
    }
    # Express overwrite guardrails as false on success.
    summary['guardrails']['scaffold_world_overwritten'] = not (scaffold_before == sha256(scaffold))
    summary['guardrails']['unscaled_clean_world_overwritten'] = not (unscaled_before == sha256(unscaled))
    (artifact_dir / 'phase33b_marker_alignment_summary.json').write_text(json.dumps(summary, indent=2, sort_keys=True), encoding='utf-8')
    return summary


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--metadata', type=Path, required=True)
    parser.add_argument('--segments-yaml', type=Path, required=True)
    parser.add_argument('--scaled-world', type=Path, required=True)
    parser.add_argument('--artifact-dir', type=Path, required=True)
    args = parser.parse_args()
    summary = generate(args.metadata, args.segments_yaml, args.scaled_world, args.artifact_dir)
    print(json.dumps({
        'selected_entrance_opening': summary['selected_entrance_opening'],
        'selected_exit_opening': summary['selected_exit_opening'],
        'wall_geometry_unchanged': summary['wall_geometry_unchanged'],
        'markers': summary['markers'],
    }, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
