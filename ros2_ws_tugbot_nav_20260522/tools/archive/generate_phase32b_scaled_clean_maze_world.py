#!/usr/bin/env python3
"""Generate Phase32B scaled clean maze candidate world with visual-only markers.

This script scales the already accepted clean 2D candidate world in XY by a fixed
factor while preserving wall thickness/height, writes a separate candidate SDF,
and adds non-colliding floor visuals for entrance/exit. It never promotes or
overwrites the scaffold or unscaled clean candidate world.
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
        raise ValueError(f'YAML root must be a mapping: {path}')
    return data


def sha256(path: Path) -> str | None:
    if not path.exists():
        return None
    return hashlib.sha256(path.read_bytes()).hexdigest()


def parse_floats(text: str) -> list[float]:
    return [float(v) for v in text.split()]


def fmt(values: list[float]) -> str:
    return ' '.join(f'{v:.3f}' for v in values)


def model_name(model: ET.Element) -> str:
    return model.get('name') or ''


def is_wall_model(model: ET.Element) -> bool:
    return model_name(model).startswith('maze_wall')


def get_world(root: ET.Element) -> ET.Element:
    world = root.find('.//world')
    if world is None:
        raise ValueError('SDF has no world element')
    return world


def scale_wall_model(model: ET.Element, scale: float, keep_thickness: float, keep_height: float) -> None:
    pose_text = model.findtext('pose')
    if not pose_text:
        raise ValueError(f'wall missing pose: {model_name(model)}')
    pose = parse_floats(pose_text)
    pose[0] *= scale
    pose[1] *= scale
    pose[2] = keep_height / 2.0
    pose_elem = model.find('pose')
    if pose_elem is None:
        raise ValueError(f'wall missing pose element: {model_name(model)}')
    pose_elem.text = fmt(pose)

    for size_elem in model.findall('./link/collision/geometry/box/size') + model.findall('./link/visual/geometry/box/size'):
        sx, sy, sz = parse_floats(size_elem.text or '')
        if sx >= sy:
            sx *= scale
            sy = keep_thickness
        else:
            sx = keep_thickness
            sy *= scale
        sz = keep_height
        size_elem.text = fmt([sx, sy, sz])


def scale_tugbot_include(world: ET.Element, entrance: dict[str, Any]) -> None:
    for include in world.findall('include'):
        if (include.findtext('uri') or '').strip() != 'model://tugbot':
            continue
        pose_elem = include.find('pose')
        if pose_elem is None:
            pose_elem = ET.SubElement(include, 'pose')
        pose_elem.text = fmt([
            float(entrance['x_m']),
            float(entrance['y_m']),
            0.0,
            0.0,
            0.0,
            float(entrance.get('yaw_rad', 0.0)),
        ])
        return
    include = ET.SubElement(world, 'include')
    ET.SubElement(include, 'uri').text = 'model://tugbot'
    ET.SubElement(include, 'name').text = 'tugbot'
    ET.SubElement(include, 'pose').text = fmt([
        float(entrance['x_m']), float(entrance['y_m']), 0.0, 0.0, 0.0, float(entrance.get('yaw_rad', 0.0))
    ])


def update_exit_marker(world: ET.Element, exit_cfg: dict[str, Any]) -> None:
    for model in world.findall('model'):
        if model_name(model) != 'maze_exit_marker':
            continue
        pose_elem = model.find('pose')
        if pose_elem is None:
            pose_elem = ET.SubElement(model, 'pose')
        pose_elem.text = fmt([float(exit_cfg['x_m']), float(exit_cfg['y_m']), 0.01, 0.0, 0.0, 0.0])
        radius_elem = model.find('.//cylinder/radius')
        if radius_elem is not None:
            radius_elem.text = f"{float(exit_cfg.get('radius_m', 1.2)):.3f}"
        return


def make_box_visual(name: str, size: tuple[float, float, float], pose: tuple[float, float, float, float, float, float], rgba: str) -> ET.Element:
    model = ET.Element('model', {'name': name})
    ET.SubElement(model, 'static').text = 'true'
    ET.SubElement(model, 'pose').text = fmt(list(pose))
    link = ET.SubElement(model, 'link', {'name': 'link'})
    visual = ET.SubElement(link, 'visual', {'name': 'visual'})
    geom = ET.SubElement(visual, 'geometry')
    box = ET.SubElement(geom, 'box')
    ET.SubElement(box, 'size').text = fmt(list(size))
    material = ET.SubElement(visual, 'material')
    ET.SubElement(material, 'ambient').text = rgba
    ET.SubElement(material, 'diffuse').text = rgba
    return model


def remove_marker_models(world: ET.Element) -> None:
    for model in list(world.findall('model')):
        if model_name(model) in {'maze_entrance_arrow_visual', 'maze_exit_finish_band_visual'} or model_name(model).startswith('maze_exit_finish_tile_'):
            world.remove(model)


def add_entrance_arrow(world: ET.Element, marker_cfg: dict[str, Any]) -> None:
    cfg = marker_cfg['entrance_arrow']
    z = float(marker_cfg.get('z_m', 0.005))
    x = float(cfg['x_m'])
    y = float(cfg['y_m'])
    yaw = float(cfg.get('yaw_rad', 0.0))
    # Arrow points +X into the maze: shaft plus triangular head, all visual-only.
    parent = ET.Element('model', {'name': cfg.get('name', 'maze_entrance_arrow_visual')})
    ET.SubElement(parent, 'static').text = 'true'
    ET.SubElement(parent, 'pose').text = fmt([x, y, z, 0.0, 0.0, yaw])
    link = ET.SubElement(parent, 'link', {'name': 'link'})
    for name, dx, sx, sy in [
        ('shaft', -0.15, 1.00, 0.16),
        ('head', 0.45, 0.35, 0.45),
    ]:
        visual = ET.SubElement(link, 'visual', {'name': name})
        ET.SubElement(visual, 'pose').text = fmt([dx, 0.0, 0.0, 0.0, 0.0, 0.0])
        geom = ET.SubElement(visual, 'geometry')
        box = ET.SubElement(geom, 'box')
        ET.SubElement(box, 'size').text = fmt([sx, sy, 0.006])
        mat = ET.SubElement(visual, 'material')
        ET.SubElement(mat, 'ambient').text = '0.0 0.85 0.1 0.85'
        ET.SubElement(mat, 'diffuse').text = '0.0 0.85 0.1 0.85'
    world.append(parent)


def add_finish_band(world: ET.Element, marker_cfg: dict[str, Any]) -> None:
    cfg = marker_cfg['exit_finish_band']
    z = float(marker_cfg.get('z_m', 0.005))
    x0 = float(cfg['x_m'])
    y0 = float(cfg['y_m'])
    yaw = float(cfg.get('yaw_rad', 0.0))
    parent = ET.Element('model', {'name': cfg.get('name', 'maze_exit_finish_band_visual')})
    ET.SubElement(parent, 'static').text = 'true'
    ET.SubElement(parent, 'pose').text = fmt([x0, y0, z, 0.0, 0.0, yaw])
    link = ET.SubElement(parent, 'link', {'name': 'link'})
    tile = 0.30
    nx = 4
    ny = 2
    for ix in range(nx):
        for iy in range(ny):
            visual = ET.SubElement(link, 'visual', {'name': f'tile_{ix}_{iy}'})
            px = (ix - (nx - 1) / 2.0) * tile
            py = (iy - (ny - 1) / 2.0) * tile
            ET.SubElement(visual, 'pose').text = fmt([px, py, 0.0, 0.0, 0.0, 0.0])
            geom = ET.SubElement(visual, 'geometry')
            box = ET.SubElement(geom, 'box')
            ET.SubElement(box, 'size').text = fmt([tile, tile, 0.006])
            color = '0.02 0.02 0.02 0.95' if (ix + iy) % 2 == 0 else '0.95 0.95 0.95 0.95'
            mat = ET.SubElement(visual, 'material')
            ET.SubElement(mat, 'ambient').text = color
            ET.SubElement(mat, 'diffuse').text = color
    world.append(parent)


def indent_xml(elem: ET.Element, level: int = 0) -> None:
    i = '\n' + level * '  '
    if len(elem):
        if not elem.text or not elem.text.strip():
            elem.text = i + '  '
        children = list(elem)
        for child in children:
            indent_xml(child, level + 1)
        last_child = children[-1]
        if not last_child.tail or not last_child.tail.strip():
            last_child.tail = i
    if level and (not elem.tail or not elem.tail.strip()):
        elem.tail = i


def wall_pose_size_map(world: ET.Element) -> dict[str, dict[str, list[float]]]:
    out: dict[str, dict[str, list[float]]] = {}
    for model in world.findall('model'):
        if not is_wall_model(model):
            continue
        pose = parse_floats(model.findtext('pose') or '')
        size = parse_floats(model.findtext('./link/collision/geometry/box/size') or '')
        out[model_name(model)] = {'pose': pose, 'size': size}
    return out


def draw_plan(summary_walls: dict[str, dict[str, list[float]]], markers: dict[str, Any], path: Path, bounds: dict[str, float]) -> None:
    width = height = 1000
    img = np.full((height, width, 3), 245, dtype=np.uint8)
    xmin, xmax, ymin, ymax = bounds['xmin'], bounds['xmax'], bounds['ymin'], bounds['ymax']

    def to_px(x: float, y: float) -> tuple[int, int]:
        px = int(round((x - xmin) / (xmax - xmin) * (width - 1)))
        py = int(round((ymax - y) / (ymax - ymin) * (height - 1)))
        return px, py

    for name, wall in summary_walls.items():
        x, y = wall['pose'][0], wall['pose'][1]
        sx, sy = wall['size'][0], wall['size'][1]
        p0 = to_px(x - sx / 2, y - sy / 2)
        p1 = to_px(x + sx / 2, y + sy / 2)
        color = (230, 60, 60) if 'outer' not in name else (40, 80, 230)
        cv2.rectangle(img, (min(p0[0], p1[0]), min(p0[1], p1[1])), (max(p0[0], p1[0]), max(p0[1], p1[1])), color, -1)
    ent = markers['entrance_arrow']
    ext = markers['exit_finish_band']
    cv2.arrowedLine(img, to_px(ent['x_m'] - 0.7, ent['y_m']), to_px(ent['x_m'] + 0.7, ent['y_m']), (0, 180, 20), 6, tipLength=0.35)
    cv2.putText(img, 'entrance arrow', to_px(ent['x_m'], ent['y_m'] - 0.8), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 130, 20), 2, cv2.LINE_AA)
    ex = to_px(ext['x_m'], ext['y_m'])
    cv2.rectangle(img, (ex[0] - 24, ex[1] - 12), (ex[0] + 24, ex[1] + 12), (0, 0, 0), 2)
    cv2.putText(img, 'finish band', (max(0, ex[0] - 70), min(height - 20, ex[1] + 42)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 2, cv2.LINE_AA)
    path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(path), img)


def generate(metadata_path: Path, segments_yaml: Path, base_world_path: Path, output_world: Path, artifact_dir: Path) -> dict[str, Any]:
    meta = load_yaml(metadata_path)
    scale = float(meta['scale_factor'])
    if scale <= 0:
        raise ValueError('scale_factor must be positive')
    scaffold = Path(meta['baseline_preservation']['scaffold_world'])
    clean_candidate = Path(meta['baseline_preservation']['clean_candidate_world'])
    if output_world.resolve() in {scaffold.resolve(), clean_candidate.resolve()}:
        raise ValueError('refusing to overwrite scaffold or unscaled clean candidate world')
    base_before = sha256(base_world_path)
    scaffold_before = sha256(scaffold)

    tree = ET.parse(base_world_path)
    root = tree.getroot()
    world = get_world(root)
    world.set('name', 'tugbot_maze_world_20260528_clean_scaled2x')

    keep_thickness = float(meta['wall_thickness_policy']['wall_thickness_m'])
    keep_height = float(meta['wall_height_policy']['wall_height_m'])
    for model in world.findall('model'):
        if is_wall_model(model):
            scale_wall_model(model, scale, keep_thickness, keep_height)

    scale_tugbot_include(world, meta['entrance'])
    update_exit_marker(world, meta['exit'])
    remove_marker_models(world)
    add_entrance_arrow(world, meta['markers'])
    add_finish_band(world, meta['markers'])

    indent_xml(root)
    output_world.parent.mkdir(parents=True, exist_ok=True)
    tree.write(output_world, encoding='unicode', xml_declaration=False)

    scaled_root = ET.parse(output_world).getroot()
    scaled_world = get_world(scaled_root)
    base_walls = wall_pose_size_map(get_world(ET.parse(base_world_path).getroot()))
    scaled_walls = wall_pose_size_map(scaled_world)

    artifact_dir.mkdir(parents=True, exist_ok=True)
    draw_plan(scaled_walls, meta['markers'], artifact_dir / 'scaled_sdf_plan_view.png', meta['scaled_bounds_m'])
    draw_plan(scaled_walls, meta['markers'], artifact_dir / 'entrance_arrow_exit_finish_overlay.png', meta['scaled_bounds_m'])

    thickness_ok = all(math.isclose(min(w['size'][0], w['size'][1]), keep_thickness, abs_tol=0.002) for w in scaled_walls.values())
    height_ok = all(math.isclose(w['size'][2], keep_height, abs_tol=0.002) for w in scaled_walls.values())
    marker_models = {model_name(m): m for m in scaled_world.findall('model')}
    collision_free = all(marker_models[name].find('.//collision') is None for name in ['maze_entrance_arrow_visual', 'maze_exit_finish_band_visual'])
    summary = {
        'phase': 'Phase32B',
        'status': 'scaled_candidate_generated_not_promoted',
        'source_image': meta['source_image'],
        'base_world': str(base_world_path),
        'output_world': str(output_world),
        'scale_factor': scale,
        'original_bounds_m': meta['original_bounds_m'],
        'scaled_bounds_m': meta['scaled_bounds_m'],
        'base_wall_count': len(base_walls),
        'scaled_wall_count': len(scaled_walls),
        'wall_thickness_m': keep_thickness,
        'wall_height_m': keep_height,
        'wall_thickness_preserved': thickness_ok,
        'wall_height_preserved': height_ok,
        'base_clean_world_sha256_before': base_before,
        'base_clean_world_sha256_after': sha256(base_world_path),
        'base_clean_world_preserved': base_before == sha256(base_world_path),
        'scaffold_world': str(scaffold),
        'scaffold_world_sha256_before': scaffold_before,
        'scaffold_world_sha256_after': sha256(scaffold),
        'scaffold_world_preserved': scaffold_before == sha256(scaffold),
        'entrance': meta['entrance'],
        'exit': meta['exit'],
        'markers': {
            'visual_only': bool(meta['markers']['visual_only']),
            'collision_free': collision_free,
            'z_m': float(meta['markers']['z_m']),
            'entrance_arrow': meta['markers']['entrance_arrow'],
            'exit_finish_band': meta['markers']['exit_finish_band'],
        },
        'guardrails': {
            'runtime_navigation_started': False,
            'nav2_mppi_params_modified': False,
            'fallback_terminal_acceptance_modified': False,
            'navigation_strategy_modified': False,
            'scaffold_world_overwritten': False,
            'base_clean_world_overwritten': False,
            'candidate_not_promoted': True,
        },
        'artifacts': [
            'scaled_sdf_plan_view.png',
            'entrance_arrow_exit_finish_overlay.png',
            'scale_comparison_summary.json',
        ],
    }
    (artifact_dir / 'scale_comparison_summary.json').write_text(json.dumps(summary, indent=2, sort_keys=True), encoding='utf-8')
    return summary


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--metadata', type=Path, required=True)
    parser.add_argument('--segments-yaml', type=Path, required=True)
    parser.add_argument('--base-world', type=Path, required=True)
    parser.add_argument('--output-world', type=Path, required=True)
    parser.add_argument('--artifact-dir', type=Path, required=True)
    args = parser.parse_args()
    # Load segments to fail early on missing/invalid input, even though scaling uses base SDF geometry.
    load_yaml(args.segments_yaml)
    summary = generate(args.metadata, args.segments_yaml, args.base_world, args.output_world, args.artifact_dir)
    print(json.dumps({
        'output_world': summary['output_world'],
        'scale_factor': summary['scale_factor'],
        'base_wall_count': summary['base_wall_count'],
        'scaled_wall_count': summary['scaled_wall_count'],
        'wall_thickness_preserved': summary['wall_thickness_preserved'],
        'base_clean_world_preserved': summary['base_clean_world_preserved'],
        'scaffold_world_preserved': summary['scaffold_world_preserved'],
    }, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
