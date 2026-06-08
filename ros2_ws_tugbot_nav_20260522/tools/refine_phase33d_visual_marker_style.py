#!/usr/bin/env python3
"""Phase33D visual-only entrance/exit marker style refinement.

Style-only edit for the corrected 2x clean maze candidate world:
- replace entrance box arrow with an isosceles-triangle-like green arrow made of
  thin visual-only boxes;
- rotate/reshape finish band to span the detected exit opening width;
- hide the green circular exit marker so it no longer covers the finish band.

No maze wall geometry, scale factor, entrance/exit coordinates, Nav2 parameters,
or navigation behavior are changed.
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

ARROW_POSE = [-10.661, -9.025, 0.005, 0.0, 0.0, 0.0]
TUGBOT_POSE = [-11.011, -9.025, 0.0, 0.0, 0.0, 0.0]
FINISH_POSE = [10.061, 9.058, 0.005, 0.0, 0.0, math.pi / 2.0]
EXIT_MARKER_POSE = [10.061, 9.058, 0.010, 0.0, 0.0, 0.0]
EXIT_OPENING_WIDTH_M = 2.005571


def fmt(values: list[float]) -> str:
    return ' '.join(f'{v:.3f}' for v in values)


def sha256(path: Path) -> str | None:
    if not path.exists():
        return None
    return hashlib.sha256(path.read_bytes()).hexdigest()


def parse_floats(text: str) -> list[float]:
    return [float(v) for v in text.split()]


def ensure(parent: ET.Element, tag: str) -> ET.Element:
    child = parent.find(tag)
    if child is None:
        child = ET.SubElement(parent, tag)
    return child


def remove_children(parent: ET.Element, tag: str) -> None:
    for child in list(parent.findall(tag)):
        parent.remove(child)


def model_name(model: ET.Element) -> str:
    return model.get('name') or ''


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
        for field in ('pose', 'size'):
            if len(a[key][field]) != len(b[key][field]):
                return False
            for x, y in zip(a[key][field], b[key][field]):
                if abs(float(x) - float(y)) > tol:
                    return False
    return True


def mat(parent: ET.Element, rgba: str, transparency: float | None = None) -> None:
    material = ET.SubElement(parent, 'material')
    ambient = ET.SubElement(material, 'ambient')
    ambient.text = rgba
    diffuse = ET.SubElement(material, 'diffuse')
    diffuse.text = rgba
    if transparency is not None:
        trans = ET.SubElement(material, 'transparency')
        trans.text = f'{transparency:.3f}'


def add_box_visual(link: ET.Element, name: str, pose: list[float], size: list[float], rgba: str) -> ET.Element:
    visual = ET.SubElement(link, 'visual', {'name': name})
    pose_el = ET.SubElement(visual, 'pose')
    pose_el.text = fmt(pose)
    geometry = ET.SubElement(visual, 'geometry')
    box = ET.SubElement(geometry, 'box')
    size_el = ET.SubElement(box, 'size')
    size_el.text = fmt(size)
    mat(visual, rgba)
    return visual


def replace_entrance_arrow(model: ET.Element) -> None:
    model.set('phase33d_style', 'triangle_arrow')
    ensure(model, 'pose').text = fmt(ARROW_POSE)
    link = ensure(model, 'link')
    link.set('name', link.get('name') or 'link')
    remove_children(link, 'collision')
    remove_children(link, 'visual')
    green = '0.0 0.9 0.0 1.0'
    # Isosceles triangle outline: one short nose plus two angled sides, all very thin
    # and visual-only. The tip points +X from the left entrance into the maze.
    add_box_visual(link, 'triangle_arrow_tip', [0.42, 0.0, 0.002, 0.0, 0.0, 0.0], [0.42, 0.09, 0.010], green)
    add_box_visual(link, 'triangle_arrow_left_edge', [0.05, 0.20, 0.002, 0.0, 0.0, 0.55], [0.92, 0.08, 0.010], green)
    add_box_visual(link, 'triangle_arrow_right_edge', [0.05, -0.20, 0.002, 0.0, 0.0, -0.55], [0.92, 0.08, 0.010], green)


def replace_finish_band(model: ET.Element) -> None:
    model.set('phase33d_style', 'rotated_checker_finish_band')
    ensure(model, 'pose').text = fmt(FINISH_POSE)
    link = ensure(model, 'link')
    link.set('name', link.get('name') or 'link')
    remove_children(link, 'collision')
    remove_children(link, 'visual')
    # Local Y is the long direction. With model yaw=90deg the band appears横向
    # across the right-side opening in world view while preserving the same center.
    total_width = 2.00
    total_depth = 0.54
    cols = 4
    rows = 2
    tile_y = total_width / cols
    tile_x = total_depth / rows
    z = 0.002
    black = '0.02 0.02 0.02 1.0'
    white = '0.95 0.95 0.95 1.0'
    for col in range(cols):
        for row in range(rows):
            x = -total_depth / 2.0 + tile_x * (row + 0.5)
            y = -total_width / 2.0 + tile_y * (col + 0.5)
            rgba = black if (col + row) % 2 == 0 else white
            add_box_visual(link, f'tile_{col}_{row}', [x, y, z, 0.0, 0.0, 0.0], [tile_x, tile_y, 0.010], rgba)


def hide_exit_marker(model: ET.Element) -> None:
    model.set('phase33d_style', 'hidden_semantic_marker')
    ensure(model, 'pose').text = fmt(EXIT_MARKER_POSE)
    link = ensure(model, 'link')
    link.set('name', link.get('name') or 'link')
    remove_children(link, 'collision')
    remove_children(link, 'visual')
    visual = ET.SubElement(link, 'visual', {'name': 'hidden_semantic_marker'})
    pose_el = ET.SubElement(visual, 'pose')
    pose_el.text = fmt([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    geometry = ET.SubElement(visual, 'geometry')
    box = ET.SubElement(geometry, 'box')
    size_el = ET.SubElement(box, 'size')
    size_el.text = fmt([0.001, 0.001, 0.001])
    mat(visual, '0.0 0.0 0.0 0.0', transparency=1.0)


def update_metadata(path: Path) -> dict[str, Any]:
    data = yaml.safe_load(path.read_text())
    out = copy.deepcopy(data)
    markers = out.setdefault('markers', {})
    markers.setdefault('entrance_arrow', {})['style'] = 'green_isosceles_triangle_arrow'
    markers['entrance_arrow']['geometry'] = 'three_thin_visual_boxes_triangle_outline'
    markers['entrance_arrow']['x_m'] = -10.661281
    markers['entrance_arrow']['y_m'] = -9.02507
    markers['entrance_arrow']['yaw_rad'] = 0.0
    markers.setdefault('exit_finish_band', {})['style'] = 'rotated_black_white_checker_band'
    markers['exit_finish_band']['x_m'] = 10.061281
    markers['exit_finish_band']['y_m'] = 9.058496
    markers['exit_finish_band']['yaw_rad'] = round(math.pi / 2.0, 6)
    markers['exit_finish_band']['spans_exit_opening_width_m'] = round(EXIT_OPENING_WIDTH_M, 6)
    markers['exit_marker'] = {
        'name': 'maze_exit_marker',
        'semantic_only': True,
        'visible_ground_disk': False,
        'style': 'hidden_semantic_marker',
        'x_m': 10.061281,
        'y_m': 9.058496,
    }
    markers['phase33d_style_refinement'] = {
        'status': 'visual_marker_style_refined_not_promoted',
        'wall_geometry_modified': False,
        'scale_factor_modified': False,
        'coordinates_modified': False,
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


def draw_arrow_preview(path: Path) -> None:
    img = np.full((420, 700, 3), 245, dtype=np.uint8)
    pts = np.array([[520, 210], [180, 95], [180, 325]], np.int32)
    cv2.polylines(img, [pts], True, (0, 170, 0), 14, cv2.LINE_AA)
    cv2.arrowedLine(img, (90, 210), (560, 210), (0, 120, 0), 4, tipLength=0.12)
    cv2.putText(img, 'Phase33D entrance: green triangle arrow, +X inward', (35, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 90, 0), 2, cv2.LINE_AA)
    cv2.putText(img, 'visual-only, collision-free, same pose', (35, 390), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (40, 40, 40), 1, cv2.LINE_AA)
    path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(path), img)


def draw_finish_preview(path: Path) -> None:
    img = np.full((500, 700, 3), 245, dtype=np.uint8)
    x0, y0 = 120, 190
    w, h = 460, 130
    cols, rows = 4, 2
    for c in range(cols):
        for r in range(rows):
            color = (20, 20, 20) if (c + r) % 2 == 0 else (245, 245, 245)
            p0 = (x0 + int(c*w/cols), y0 + int(r*h/rows))
            p1 = (x0 + int((c+1)*w/cols), y0 + int((r+1)*h/rows))
            cv2.rectangle(img, p0, p1, color, -1)
            cv2.rectangle(img, p0, p1, (70, 70, 70), 1)
    cv2.putText(img, 'Phase33D exit: rotated checker band spans opening', (35, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (20, 20, 20), 2, cv2.LINE_AA)
    cv2.putText(img, 'green circular ground marker hidden', (35, 445), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (40, 40, 40), 1, cv2.LINE_AA)
    path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(path), img)


def generate(world_path: Path, metadata_path: Path, artifact_dir: Path) -> dict[str, Any]:
    scaffold = Path('src/tugbot_gazebo/worlds/tugbot_maze_world.sdf')
    unscaled = Path('src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean.sdf')
    scaffold_before = sha256(scaffold)
    unscaled_before = sha256(unscaled)

    tree = ET.parse(world_path)
    world = world_from_tree(tree)
    walls_before = wall_geometry(world)
    models = {model_name(m): m for m in world.findall('model')}
    replace_entrance_arrow(models['maze_entrance_arrow_visual'])
    replace_finish_band(models['maze_exit_finish_band_visual'])
    if 'maze_exit_marker' in models:
        hide_exit_marker(models['maze_exit_marker'])
    walls_after = wall_geometry(world)
    wall_same = geometries_equal(walls_before, walls_after)
    if not wall_same:
        raise RuntimeError('wall geometry changed during Phase33D marker style refinement')
    indent_xml(tree.getroot())
    tree.write(world_path, encoding='unicode', xml_declaration=False)

    metadata = update_metadata(metadata_path)
    artifact_dir.mkdir(parents=True, exist_ok=True)
    draw_arrow_preview(artifact_dir / 'entrance_triangle_arrow_preview.png')
    draw_finish_preview(artifact_dir / 'exit_finish_band_rotated_preview.png')

    summary = {
        'phase': 'Phase33D',
        'status': 'visual_marker_style_refined_not_promoted',
        'world': str(world_path),
        'metadata': str(metadata_path),
        'wall_count': len(walls_after),
        'wall_geometry_unchanged': wall_same,
        'coordinates_unchanged': True,
        'entrance_pose': ARROW_POSE,
        'tugbot_pose': TUGBOT_POSE,
        'finish_band_pose': FINISH_POSE,
        'exit_marker_pose': EXIT_MARKER_POSE,
        'entrance_marker_style': 'green_isosceles_triangle_arrow',
        'finish_band_style': 'rotated_black_white_checker_band',
        'finish_band_opening_width_target_m': EXIT_OPENING_WIDTH_M,
        'exit_marker_visible_ground_disk_removed': True,
        'markers_visual_only': True,
        'marker_collision_count': 0,
        'scaffold_sha256_before': scaffold_before,
        'scaffold_sha256_after': sha256(scaffold),
        'unscaled_clean_sha256_before': unscaled_before,
        'unscaled_clean_sha256_after': sha256(unscaled),
        'candidate_not_promoted': True,
        'navigation_started': False,
        'nav2_mppi_params_modified': False,
        'scale_factor_modified': False,
        'wall_segment_geometry_modified': False,
        'artifacts': ['entrance_triangle_arrow_preview.png', 'exit_finish_band_rotated_preview.png', 'marker_style_summary.json'],
    }
    (artifact_dir / 'marker_style_summary.json').write_text(json.dumps(summary, indent=2, sort_keys=True), encoding='utf-8')
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
        'triangle_arrow': summary['entrance_marker_style'],
        'finish_band_style': summary['finish_band_style'],
        'wall_geometry_unchanged': summary['wall_geometry_unchanged'],
    }, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
