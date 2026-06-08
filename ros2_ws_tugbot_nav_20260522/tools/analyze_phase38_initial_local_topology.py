#!/usr/bin/env python3
"""Phase38 initial local-topology diagnostics from Phase37 bounded-smoke artifacts.

Read-only analyzer: parses saved YAML/text/JSONL artifacts, recomputes local
map/topology evidence offline, and writes JSON + PNG overlays. It does not start
ROS, Gazebo, Nav2, or maze_explorer.
"""
from __future__ import annotations

import argparse
import json
import math
import re
from collections import Counter
from pathlib import Path
from typing import Any

import yaml
from PIL import Image, ImageDraw

PHASE38_ALLOWED_CONCLUSIONS = {
    'INSUFFICIENT_MAP_AT_START',
    'START_POSE_OUTSIDE_MAZE_OR_TOO_CLOSE_TO_BOUNDARY',
    'TOPOLOGY_SAMPLING_SCALE_MISMATCH',
    'UNKNOWN_CELL_POLICY_TOO_STRICT',
    'TF_OR_POSE_ALIGNMENT_ISSUE',
    'INSUFFICIENT_EVIDENCE',
}

RUN_ID = 'phase37_scaled_clean_world_maze_explorer_bounded_smoke'


def _load_yaml_text(path: Path) -> dict[str, Any]:
    if not path.exists() or path.stat().st_size == 0:
        return {}
    docs = list(yaml.safe_load_all(path.read_text(encoding='utf-8', errors='replace')))
    for doc in docs:
        if isinstance(doc, dict):
            return doc
    return {}


def _load_json(path: Path) -> dict[str, Any]:
    if not path.exists() or path.stat().st_size == 0:
        return {}
    return json.loads(path.read_text(encoding='utf-8'))


def _load_jsonl(path: Path) -> list[dict[str, Any]]:
    if not path.exists() or path.stat().st_size == 0:
        return []
    rows = []
    for line in path.read_text(encoding='utf-8', errors='replace').splitlines():
        line = line.strip()
        if not line:
            continue
        rows.append(json.loads(line))
    return rows


def _grid_from_yaml_doc(doc: dict[str, Any]) -> dict[str, Any]:
    info = doc.get('info') or {}
    origin = ((info.get('origin') or {}).get('position') or {})
    data = doc.get('data') or []
    parsed_data = []
    truncated = False
    for raw in data:
        try:
            parsed_data.append(int(raw))
        except Exception:
            truncated = True
            break
    return {
        'frame_id': (doc.get('header') or {}).get('frame_id'),
        'stamp': (doc.get('header') or {}).get('stamp'),
        'width': int(info.get('width') or 0),
        'height': int(info.get('height') or 0),
        'resolution': float(info.get('resolution') or 0.0),
        'origin_x': float(origin.get('x') or 0.0),
        'origin_y': float(origin.get('y') or 0.0),
        'data': parsed_data,
        'data_expected_len': int(info.get('width') or 0) * int(info.get('height') or 0),
        'data_observed_len': len(parsed_data),
        'data_truncated': truncated or len(parsed_data) < int(info.get('width') or 0) * int(info.get('height') or 0),
    }


def _grid_value(grid: dict[str, Any], cell_x: int, cell_y: int) -> int:
    width = grid['width']
    height = grid['height']
    if cell_x < 0 or cell_y < 0 or cell_x >= width or cell_y >= height:
        return 100
    idx = cell_x + cell_y * width
    data = grid['data']
    if idx < 0 or idx >= len(data):
        return -1
    return int(data[idx])


def _world_to_cell(grid: dict[str, Any], x: float, y: float) -> tuple[int, int]:
    res = grid['resolution']
    return (int(math.floor((x - grid['origin_x']) / res)), int(math.floor((y - grid['origin_y']) / res)))


def _cell_to_world(grid: dict[str, Any], cell_x: int, cell_y: int) -> tuple[float, float]:
    res = grid['resolution']
    return (grid['origin_x'] + (cell_x + 0.5) * res, grid['origin_y'] + (cell_y + 0.5) * res)


def _has_clearance(grid: dict[str, Any], cell: tuple[int, int], radius_m: float, unknown_is_safe: bool = False) -> bool:
    cx, cy = cell
    value = _grid_value(grid, cx, cy)
    if value < 0 or value >= 65:
        return False
    radius_cells = int(math.ceil(max(radius_m, 0.0) / grid['resolution']))
    for y in range(cy - radius_cells, cy + radius_cells + 1):
        for x in range(cx - radius_cells, cx + radius_cells + 1):
            if math.hypot(x - cx, y - cy) * grid['resolution'] > radius_m + 1e-9:
                continue
            v = _grid_value(grid, x, y)
            if v >= 65:
                return False
            if v < 0 and not unknown_is_safe:
                return False
    return True


def _world_has_clearance(grid: dict[str, Any], x: float, y: float, radius_m: float, unknown_is_safe: bool = False) -> bool:
    return _has_clearance(grid, _world_to_cell(grid, x, y), radius_m, unknown_is_safe=unknown_is_safe)


def _safe_distance_along_ray(
    grid: dict[str, Any],
    pose_xy: tuple[float, float],
    direction_rad: float,
    lookahead_m: float,
    clearance_radius_m: float,
    unknown_is_safe: bool = False,
) -> tuple[float, str, int, float, tuple[float, float]]:
    sample_step_m = max(grid['resolution'] * 0.5, 0.05)
    best = 0.0
    distance = sample_step_m
    first_block_reason = 'lookahead_complete'
    first_block_value = 0
    first_block_distance = lookahead_m
    first_block_xy = (pose_xy[0] + math.cos(direction_rad) * lookahead_m, pose_xy[1] + math.sin(direction_rad) * lookahead_m)
    while distance <= lookahead_m + 1e-9:
        x = pose_xy[0] + math.cos(direction_rad) * distance
        y = pose_xy[1] + math.sin(direction_rad) * distance
        cell = _world_to_cell(grid, x, y)
        value = _grid_value(grid, *cell)
        if not _world_has_clearance(grid, x, y, clearance_radius_m, unknown_is_safe=unknown_is_safe):
            if cell[0] < 0 or cell[1] < 0 or cell[0] >= grid['width'] or cell[1] >= grid['height']:
                first_block_reason = 'out_of_bounds_or_map_edge'
            elif value < 0:
                first_block_reason = 'unknown_cell_or_unknown_clearance'
            elif value >= 65:
                first_block_reason = 'occupied_or_inflated_clearance'
            else:
                first_block_reason = 'clearance_radius_blocked'
            first_block_value = value
            first_block_distance = distance
            first_block_xy = (x, y)
            break
        best = distance
        distance += sample_step_m
    return (min(best, lookahead_m), first_block_reason, first_block_value, first_block_distance, first_block_xy)


def _sample_topology(grid: dict[str, Any], pose: dict[str, float]) -> dict[str, Any]:
    angle_step_deg = 90.0
    lookahead_m = 1.5
    clearance_radius_m = 0.38
    min_open_distance_m = 0.45
    yaw = float(pose['yaw'])
    samples = []
    open_dirs = []
    unknown_safe_open = []
    for i in range(int(round(360.0 / angle_step_deg))):
        angle = math.atan2(math.sin(yaw + math.radians(angle_step_deg) * i), math.cos(yaw + math.radians(angle_step_deg) * i))
        safe, reason, value, block_dist, block_xy = _safe_distance_along_ray(
            grid, (pose['x'], pose['y']), angle, lookahead_m, clearance_radius_m, unknown_is_safe=False
        )
        safe_unknown, _, _, _, _ = _safe_distance_along_ray(
            grid, (pose['x'], pose['y']), angle, lookahead_m, clearance_radius_m, unknown_is_safe=True
        )
        row = {
            'index': i,
            'angle_rad': angle,
            'angle_deg': math.degrees(angle),
            'safe_distance_m': safe,
            'safe_distance_unknown_is_safe_m': safe_unknown,
            'first_block_reason': reason,
            'first_block_value': value,
            'first_block_distance_m': block_dist,
            'first_block_xy': {'x': block_xy[0], 'y': block_xy[1]},
            'is_open': safe >= min_open_distance_m,
            'would_open_if_unknown_safe': safe_unknown >= min_open_distance_m,
        }
        samples.append(row)
        if row['is_open']:
            open_dirs.append(row)
        if row['would_open_if_unknown_safe']:
            unknown_safe_open.append(row)
    if len(open_dirs) <= 0:
        kind = 'unknown'
    elif len(open_dirs) == 1:
        kind = 'dead_end'
    elif len(open_dirs) == 2:
        kind = 'corridor'
    else:
        kind = 'junction'
    return {
        'parameters': {
            'angle_step_deg': angle_step_deg,
            'lookahead_m': lookahead_m,
            'clearance_radius_m': clearance_radius_m,
            'min_open_distance_m': min_open_distance_m,
        },
        'sampled_direction_count': len(samples),
        'open_direction_count_recomputed': len(open_dirs),
        'kind_recomputed': kind,
        'unknown_safe_open_direction_count': len(unknown_safe_open),
        'samples': samples,
        'block_reason_counts': dict(Counter(s['first_block_reason'] for s in samples)),
    }


def _pose_from_tf_or_odom(tf_doc: dict[str, Any], odom_doc: dict[str, Any]) -> dict[str, Any]:
    # Phase37 tf_once captured odom->base_link, while /map had identity-ish origin around odom.
    transforms = tf_doc.get('transforms') or []
    if transforms:
        t = (transforms[0].get('transform') or {}).get('translation') or {}
        r = (transforms[0].get('transform') or {}).get('rotation') or {}
        yaw = math.atan2(2.0 * (float(r.get('w', 1.0)) * float(r.get('z', 0.0)) + float(r.get('x', 0.0)) * float(r.get('y', 0.0))), 1.0 - 2.0 * (float(r.get('y', 0.0)) ** 2 + float(r.get('z', 0.0)) ** 2))
        return {'available': True, 'x': float(t.get('x') or 0.0), 'y': float(t.get('y') or 0.0), 'yaw': yaw, 'source': 'tf_once odom->base_link'}
    pose = (((odom_doc.get('pose') or {}).get('pose') or {}).get('position') or {})
    orient = (((odom_doc.get('pose') or {}).get('pose') or {}).get('orientation') or {})
    if pose:
        yaw = math.atan2(2.0 * (float(orient.get('w', 1.0)) * float(orient.get('z', 0.0)) + float(orient.get('x', 0.0)) * float(orient.get('y', 0.0))), 1.0 - 2.0 * (float(orient.get('y', 0.0)) ** 2 + float(orient.get('z', 0.0)) ** 2))
        return {'available': True, 'x': float(pose.get('x') or 0.0), 'y': float(pose.get('y') or 0.0), 'yaw': yaw, 'source': 'odom_once pose'}
    return {'available': False, 'x': 0.0, 'y': 0.0, 'yaw': 0.0, 'source': 'unavailable'}


def _cell_stats_near(grid: dict[str, Any], x: float, y: float, radius_m: float) -> dict[str, Any]:
    cx, cy = _world_to_cell(grid, x, y)
    radius_cells = int(math.ceil(radius_m / grid['resolution'])) if grid['resolution'] else 0
    counts = Counter()
    total = 0
    observed_len = len(grid.get('data') or [])
    for yy in range(cy - radius_cells, cy + radius_cells + 1):
        for xx in range(cx - radius_cells, cx + radius_cells + 1):
            wx, wy = _cell_to_world(grid, xx, yy)
            if math.hypot(wx - x, wy - y) > radius_m:
                continue
            cell_in_bounds = 0 <= xx < grid['width'] and 0 <= yy < grid['height']
            idx = xx + yy * grid['width'] if cell_in_bounds else -1
            total += 1
            if not cell_in_bounds:
                counts['out_of_bounds'] += 1
                counts['unknown'] += 1
                continue
            if idx >= observed_len:
                counts['truncated_missing'] += 1
                counts['unknown'] += 1
                continue
            v = _grid_value(grid, xx, yy)
            if v < 0:
                counts['unknown'] += 1
            elif v >= 65:
                counts['occupied'] += 1
            else:
                counts['free'] += 1
    known = counts['free'] + counts['occupied']
    return {
        'center_cell': {'x': cx, 'y': cy},
        'radius_m': radius_m,
        'total_cells': total,
        'free_cells': counts['free'],
        'occupied_cells': counts['occupied'],
        'unknown_cells': counts['unknown'],
        'out_of_bounds_cells': counts['out_of_bounds'],
        'truncated_missing_cells': counts['truncated_missing'],
        'known_ratio': (known / total) if total else 0.0,
        'free_ratio': (counts['free'] / total) if total else 0.0,
        'unknown_ratio': (counts['unknown'] / total) if total else 0.0,
        'out_of_bounds_ratio': (counts['out_of_bounds'] / total) if total else 0.0,
        'truncated_missing_ratio': (counts['truncated_missing'] / total) if total else 0.0,
    }


def _scan_diagnostics(scan_doc: dict[str, Any]) -> dict[str, Any]:
    ranges = scan_doc.get('ranges') or []
    finite = []
    for r in ranges:
        try:
            v = float(r)
        except Exception:
            continue
        if math.isfinite(v):
            finite.append(v)
    return {
        'sample_count': len(ranges),
        'finite_count': len(finite),
        'finite_ratio': (len(finite) / len(ranges)) if ranges else 0.0,
        'nearest_obstacle_m': min(finite) if finite else None,
        'range_min': scan_doc.get('range_min'),
        'range_max': scan_doc.get('range_max'),
    }


def _entrance_alignment(metadata: dict[str, Any], pose: dict[str, Any]) -> dict[str, Any]:
    ent = metadata.get('entrance') or {}
    ex = metadata.get('exit') or {}
    dx = float(ent.get('x_m', 0.0)) - pose['x']
    dy = float(ent.get('y_m', 0.0)) - pose['y']
    bearing = math.atan2(dy, dx)
    yaw_delta = math.atan2(math.sin(float(ent.get('yaw_rad', 0.0)) - pose['yaw']), math.cos(float(ent.get('yaw_rad', 0.0)) - pose['yaw']))
    return {
        'entrance_pose': {'x': float(ent.get('x_m', 0.0)), 'y': float(ent.get('y_m', 0.0)), 'yaw': float(ent.get('yaw_rad', 0.0))},
        'exit_pose': {'x': float(ex.get('x_m', 0.0)), 'y': float(ex.get('y_m', 0.0)), 'radius': float(ex.get('radius_m', 0.0))},
        'robot_pose': {'x': pose['x'], 'y': pose['y'], 'yaw': pose['yaw']},
        'dx_m': dx,
        'dy_m': dy,
        'distance_m': math.hypot(dx, dy),
        'bearing_from_robot_to_entrance_rad': bearing,
        'entrance_yaw_minus_robot_yaw_rad': yaw_delta,
    }


def _grid_bounds(grid: dict[str, Any]) -> dict[str, float]:
    return {
        'xmin': grid['origin_x'],
        'ymin': grid['origin_y'],
        'xmax': grid['origin_x'] + grid['width'] * grid['resolution'],
        'ymax': grid['origin_y'] + grid['height'] * grid['resolution'],
    }


def _classify(map_diag: dict[str, Any], local_diag: dict[str, Any], topology: dict[str, Any], alignment: dict[str, Any], grid: dict[str, Any], pose: dict[str, Any]) -> tuple[str, list[str]]:
    reasons = []
    bounds = _grid_bounds(grid)
    in_bounds = bounds['xmin'] <= pose['x'] <= bounds['xmax'] and bounds['ymin'] <= pose['y'] <= bounds['ymax']
    if not pose.get('available'):
        return 'TF_OR_POSE_ALIGNMENT_ISSUE', ['robot pose unavailable from saved tf/odom artifacts']
    if not in_bounds:
        reasons.append('robot pose is outside saved /map bounds')
        return 'START_POSE_OUTSIDE_MAZE_OR_TOO_CLOSE_TO_BOUNDARY', reasons
    if alignment['distance_m'] > 2.0:
        reasons.append('robot pose from tf/odom is far from active entrance pose')
    if map_diag.get('known_ratio_near_robot', map_diag.get('known_ratio', 0.0)) < 0.5:
        reasons.append('saved /map around robot is mostly unknown')
    if local_diag.get('known_ratio', 0.0) < 0.5:
        reasons.append('local neighborhood around robot is mostly unknown/unknown-dominated')
    if topology['open_direction_count_recomputed'] == 0 and topology['unknown_safe_open_direction_count'] > 0:
        reasons.append('strict unknown-cell clearance policy blocks directions that would open if unknown were treated as safe')
        if map_diag.get('known_ratio_near_robot', map_diag.get('known_ratio', 0.0)) < 0.5:
            return 'INSUFFICIENT_MAP_AT_START', reasons
        return 'UNKNOWN_CELL_POLICY_TOO_STRICT', reasons
    if map_diag.get('known_ratio_near_robot', map_diag.get('known_ratio', 0.0)) < 0.5 or local_diag.get('known_ratio', 0.0) < 0.5:
        return 'INSUFFICIENT_MAP_AT_START', reasons
    if alignment['distance_m'] > 2.0:
        return 'TF_OR_POSE_ALIGNMENT_ISSUE', reasons
    if topology['open_direction_count_recomputed'] == 0:
        reasons.append('topology recomputation found no safe rays despite non-empty map evidence')
        return 'TOPOLOGY_SAMPLING_SCALE_MISMATCH', reasons
    return 'INSUFFICIENT_EVIDENCE', reasons or ['artifacts do not isolate one allowed classification']


def _render_grid(grid: dict[str, Any], scale: int = 4) -> Image.Image:
    width, height = grid['width'], grid['height']
    img = Image.new('RGB', (max(1, width) * scale, max(1, height) * scale), 'white')
    pix = img.load()
    for y in range(height):
        for x in range(width):
            v = _grid_value(grid, x, y)
            if v < 0:
                color = (170, 170, 170)
            elif v >= 65:
                color = (20, 20, 20)
            else:
                shade = 255 - int(max(0, min(v, 64)) * 2)
                color = (shade, shade, shade)
            yy = height - 1 - y
            for dy in range(scale):
                for dx in range(scale):
                    pix[x * scale + dx, yy * scale + dy] = color
    return img


def _world_to_px(grid: dict[str, Any], x: float, y: float, scale: int = 4) -> tuple[int, int]:
    cx, cy = _world_to_cell(grid, x, y)
    return (int((cx + 0.5) * scale), int((grid['height'] - cy - 0.5) * scale))


def _draw_cross(draw: ImageDraw.ImageDraw, p: tuple[int, int], color: tuple[int, int, int], r: int = 5) -> None:
    x, y = p
    draw.line((x - r, y, x + r, y), fill=color, width=2)
    draw.line((x, y - r, x, y + r), fill=color, width=2)


def _write_overlays(output_dir: Path, grid: dict[str, Any], pose: dict[str, Any], alignment: dict[str, Any], topology: dict[str, Any]) -> None:
    scale = 6 if grid['width'] < 120 else 3
    base = _render_grid(grid, scale=scale)
    robot_px = _world_to_px(grid, pose['x'], pose['y'], scale=scale)
    ent = alignment['entrance_pose']
    exit_pose = alignment['exit_pose']
    entrance_px = _world_to_px(grid, ent['x'], ent['y'], scale=scale)
    exit_px = _world_to_px(grid, exit_pose['x'], exit_pose['y'], scale=scale)

    img = base.copy()
    draw = ImageDraw.Draw(img)
    _draw_cross(draw, robot_px, (255, 0, 0), 7)
    _draw_cross(draw, entrance_px, (0, 180, 0), 7)
    _draw_cross(draw, exit_px, (0, 0, 255), 7)
    draw.line((*robot_px, *entrance_px), fill=(255, 180, 0), width=2)
    img.save(output_dir / 'initial_pose_map_overlay.png')

    img = base.copy()
    draw = ImageDraw.Draw(img)
    _draw_cross(draw, robot_px, (255, 0, 0), 7)
    for sample in topology['samples']:
        block = sample['first_block_xy']
        px = _world_to_px(grid, block['x'], block['y'], scale=scale)
        color = (0, 200, 0) if sample['is_open'] else (255, 0, 0)
        draw.line((*robot_px, *px), fill=color, width=2)
        _draw_cross(draw, px, color, 3)
    img.save(output_dir / 'local_topology_sampling_overlay.png')

    img = base.copy()
    draw = ImageDraw.Draw(img)
    _draw_cross(draw, robot_px, (255, 0, 0), 7)
    _draw_cross(draw, entrance_px, (0, 180, 0), 7)
    dx = math.cos(ent['yaw']) * 1.0
    dy = math.sin(ent['yaw']) * 1.0
    arrow_end = _world_to_px(grid, ent['x'] + dx, ent['y'] + dy, scale=scale)
    draw.line((*entrance_px, *arrow_end), fill=(0, 180, 0), width=3)
    draw.line((*robot_px, *entrance_px), fill=(255, 180, 0), width=2)
    img.save(output_dir / 'entrance_alignment_overlay.png')


def analyze(workspace_root: Path, phase37_artifact_dir: Path, output_dir: Path) -> dict[str, Any]:
    output_dir.mkdir(parents=True, exist_ok=True)
    metadata_path = workspace_root / 'src/tugbot_maze/config/maze_20260528_scaled_instance.yaml'
    active_world = workspace_root / 'src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean_scaled2x.sdf'
    metadata = yaml.safe_load(metadata_path.read_text(encoding='utf-8'))

    summary = _load_json(phase37_artifact_dir / f'{RUN_ID}_summary.json')
    states = _load_jsonl(phase37_artifact_dir / f'{RUN_ID}_explorer_state.jsonl')
    goal_events = _load_jsonl(phase37_artifact_dir / f'{RUN_ID}_goal_events.jsonl')
    launch_log = (phase37_artifact_dir / f'{RUN_ID}_launch.log').read_text(encoding='utf-8', errors='replace')
    map_grid = _grid_from_yaml_doc(_load_yaml_text(phase37_artifact_dir / f'{RUN_ID}_map_once.txt'))
    local_grid = _grid_from_yaml_doc(_load_yaml_text(phase37_artifact_dir / f'{RUN_ID}_local_costmap_once.txt'))
    global_grid = _grid_from_yaml_doc(_load_yaml_text(phase37_artifact_dir / f'{RUN_ID}_global_costmap_once.txt'))
    scan_doc = _load_yaml_text(phase37_artifact_dir / f'{RUN_ID}_scan_once.txt')
    odom_doc = _load_yaml_text(phase37_artifact_dir / f'{RUN_ID}_odom_once.txt')
    tf_doc = _load_yaml_text(phase37_artifact_dir / f'{RUN_ID}_tf_once.txt')

    final_state = summary.get('final_state') or ((states[-1].get('state') or {}) if states else {})
    pose = _pose_from_tf_or_odom(tf_doc, odom_doc)
    alignment = _entrance_alignment(metadata, pose)
    map_near = _cell_stats_near(map_grid, pose['x'], pose['y'], radius_m=1.5) if map_grid['data'] else {}
    local_near = _cell_stats_near(local_grid, pose['x'], pose['y'], radius_m=1.0) if local_grid['data'] else {}
    global_near = _cell_stats_near(global_grid, pose['x'], pose['y'], radius_m=1.5) if global_grid['data'] else {}
    topology = _sample_topology(map_grid, pose) if map_grid['data'] and pose['available'] else {
        'sampled_direction_count': 0,
        'open_direction_count_recomputed': 0,
        'unknown_safe_open_direction_count': 0,
        'samples': [],
        'block_reason_counts': {},
        'kind_recomputed': 'unknown',
        'parameters': {},
    }
    scan = _scan_diagnostics(scan_doc)
    classification, reasons = _classify(map_near or {'known_ratio_near_robot': 0.0}, local_near or {'known_ratio': 0.0}, topology, alignment, map_grid, pose)

    strict_unknown_block_count = sum(1 for s in topology.get('samples', []) if s.get('first_block_reason') == 'unknown_cell_or_unknown_clearance')
    why = list(reasons)
    if topology.get('open_direction_count_recomputed') == 0:
        why.append(f"all {topology.get('sampled_direction_count')} sampled rays failed min_open_distance_m; strict unknown blocks={strict_unknown_block_count}")
    if alignment['distance_m'] > 2.0:
        why.append('robot pose used by Phase37 artifacts is near map origin while active entrance is far away')

    result = {
        'phase': 'Phase38 Initial Local Topology Diagnostics on Scaled Clean World',
        'inputs': {
            'phase37_dir': str(phase37_artifact_dir),
            'active_world': str(active_world.relative_to(workspace_root)),
            'active_metadata': str(metadata_path.relative_to(workspace_root)),
            'phase37_report': 'doc/doc_report/phase37_scaled_clean_world_maze_explorer_bounded_smoke_report.md',
        },
        'source_artifacts': {
            'phase37_dir': str(phase37_artifact_dir),
            'explorer_state_jsonl': str(phase37_artifact_dir / f'{RUN_ID}_explorer_state.jsonl'),
            'goal_events_jsonl': str(phase37_artifact_dir / f'{RUN_ID}_goal_events.jsonl'),
            'map_once': str(phase37_artifact_dir / f'{RUN_ID}_map_once.txt'),
            'scan_once': str(phase37_artifact_dir / f'{RUN_ID}_scan_once.txt'),
            'odom_once': str(phase37_artifact_dir / f'{RUN_ID}_odom_once.txt'),
            'tf_once': str(phase37_artifact_dir / f'{RUN_ID}_tf_once.txt'),
            'local_costmap_once': str(phase37_artifact_dir / f'{RUN_ID}_local_costmap_once.txt'),
            'global_costmap_once': str(phase37_artifact_dir / f'{RUN_ID}_global_costmap_once.txt'),
            'launch_log': str(phase37_artifact_dir / f'{RUN_ID}_launch.log'),
        },
        'phase37_conclusion': 'BOUNDED_SMOKE_PARTIAL_FAIL_NO_DISPATCH',
        'autonomous_success_claimed': False,
        'guardrails': {
            'read_only_artifact_analysis': True,
            'runtime_started': False,
            'nav2_params_modified': False,
            'strategy_modified': False,
            'autonomous_success_claimed': False,
        },
        'active_truth': {
            'entrance': {'x_m': float((metadata.get('entrance') or {}).get('x_m', 0.0)), 'y_m': float((metadata.get('entrance') or {}).get('y_m', 0.0)), 'yaw_rad': float((metadata.get('entrance') or {}).get('yaw_rad', 0.0))},
            'exit': {'x_m': float((metadata.get('exit') or {}).get('x_m', 0.0)), 'y_m': float((metadata.get('exit') or {}).get('y_m', 0.0)), 'radius_m': float((metadata.get('exit') or {}).get('radius_m', 0.0))},
        },
        'phase37_failure_signature': {
            'mode': final_state.get('mode'),
            'last_local_topology_kind': final_state.get('last_local_topology_kind'),
            'last_open_direction_count': final_state.get('last_open_direction_count'),
            'last_candidate_count': final_state.get('last_candidate_count'),
            'goal_count': final_state.get('goal_count'),
            'last_terminal_reason': final_state.get('last_terminal_reason'),
            'explorer_state_samples': len(states),
            'goal_events_samples': len(goal_events),
        },
        'robot_pose_map': pose,
        'entrance_alignment': alignment,
        'map_analysis': {
            'sample_exists': bool(map_grid.get('width')),
            'data_truncated': map_grid.get('data_truncated'),
            'data_observed_len': map_grid.get('data_observed_len'),
            'data_expected_len': map_grid.get('data_expected_len'),
            'frame_id': map_grid.get('frame_id'),
            'width': map_grid['width'],
            'height': map_grid['height'],
            'resolution': map_grid['resolution'],
            'bounds': _grid_bounds(map_grid),
            'robot_cell': map_near.get('center_cell'),
            'known_ratio_near_robot': map_near.get('known_ratio', 0.0),
            'unknown_ratio_near_robot': map_near.get('unknown_ratio', 0.0),
            'free_ratio_near_robot': map_near.get('free_ratio', 0.0),
            'cell_stats_near_robot': map_near,
        },
        'local_costmap_diagnostics': local_near,
        'global_costmap_diagnostics': global_near,
        'scan_analysis': {
            'sample_exists': bool(scan_doc),
            **scan,
        },
        'topology_sampling': {
            'attempted_from_artifact_map': bool(map_grid.get('data')) and bool(pose.get('available')),
            **topology,
        },
        'why_open_direction_count_zero': why,
        'dispatch_analysis': {
            'goal_events_samples': len(goal_events),
            'dispatch_events': sum(1 for row in goal_events if (row.get('state') or row).get('event') == 'dispatch'),
            'outcome_events': sum(1 for row in goal_events if (row.get('state') or row).get('event') == 'outcome'),
            'exit_reached_events': sum(1 for row in goal_events if (row.get('state') or row).get('event') == 'exit_reached'),
        },
        'conclusion': classification,
        'allowed_conclusions': sorted(PHASE38_ALLOWED_CONCLUSIONS),
        'classification': classification,
        'classification_options': sorted(PHASE38_ALLOWED_CONCLUSIONS),
        'launch_log_active_truth': {
            'maze_explorer_active_truth_seen': 'entrance=(-11.011, -9.025, 0.000) exit=(10.061, 9.058) radius=1.200' in launch_log,
            'legacy_truth_seen': any(tok in launch_log for tok in ['entrance_x:=-4.0', 'entrance_y:=-3.0', 'exit_x:=4.0', 'exit_y:=3.0', 'exit_radius:=0.6', 'maze_instance.yaml']),
        },
        'runtime_diagnostic_needed': False,
        'runtime_diagnostic_reason': 'Phase37 artifacts were sufficient to classify initial topology failure.' if classification != 'INSUFFICIENT_EVIDENCE' else 'Artifacts did not isolate a class.',
        'artifacts': {
            'initial_pose_map_overlay': str(output_dir / 'initial_pose_map_overlay.png'),
            'local_topology_sampling_overlay': str(output_dir / 'local_topology_sampling_overlay.png'),
            'entrance_alignment_overlay': str(output_dir / 'entrance_alignment_overlay.png'),
        },
    }
    analysis_path = output_dir / 'phase38_initial_topology_analysis.json'
    _write_overlays(output_dir, map_grid, pose, alignment, topology)
    analysis_path.write_text(json.dumps(result, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace-root', type=Path, default=Path.cwd())
    parser.add_argument('--phase37-artifact-dir', '--phase37-artifacts', dest='phase37_artifact_dir', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, default=Path('log/phase38_initial_local_topology'))
    args = parser.parse_args()
    result = analyze(args.workspace_root.resolve(), args.phase37_artifact_dir.resolve(), args.output_dir.resolve())
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
