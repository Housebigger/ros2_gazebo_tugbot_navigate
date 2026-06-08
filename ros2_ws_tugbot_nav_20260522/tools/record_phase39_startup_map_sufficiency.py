#!/usr/bin/env python3
"""Phase39 bounded startup map sufficiency and frame-alignment recorder.

Runtime mode uses rclpy subscriptions to serialize complete startup evidence from:
/map, /local_costmap/costmap, /global_costmap/costmap, /scan, /tf,
/maze/explorer_state, and /maze/goal_events. It also attempts tf2 lookup
for map->base_link, odom->base_link, and map->odom at every bounded snapshot.

Offline mode (--analyze-existing) analyzes an already serialized full-data JSON
without ROS, which keeps contract tests fast and avoids relying on truncated CLI
samples for map/costmap evidence.
"""
from __future__ import annotations

import argparse
import json
import math
import signal
import sys
import time
from collections import Counter
from pathlib import Path
from typing import Any

import yaml
from PIL import Image, ImageDraw

try:  # Runtime-only imports; offline analysis must work without a sourced ROS shell.
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from nav_msgs.msg import OccupancyGrid
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String
    from tf2_msgs.msg import TFMessage
    import tf2_ros
except Exception:  # pragma: no cover - exercised only when ROS is unsourced/unavailable.
    rclpy = None
    Node = object
    QoSProfile = None
    ReliabilityPolicy = None
    DurabilityPolicy = None
    HistoryPolicy = None
    qos_profile_sensor_data = None
    OccupancyGrid = None
    LaserScan = None
    String = None
    TFMessage = None
    tf2_ros = None

PHASE39_ALLOWED_CONCLUSIONS = {
    'MAP_SUFFICIENCY_DELAY_REQUIRED',
    'FRAME_ALIGNMENT_ISSUE_CONFIRMED',
    'START_POSE_ALIGNMENT_OK_BUT_MAP_NOT_READY',
    'SENSOR_FLOW_NOT_READY_AT_TOPOLOGY_TIME',
    'INSUFFICIENT_EVIDENCE',
}

TF_LOOKUP_PAIRS = ('map->base_link', 'odom->base_link', 'map->odom')
TOPICS = (
    '/map',
    '/local_costmap/costmap',
    '/global_costmap/costmap',
    '/scan',
    '/tf',
    '/maze/explorer_state',
    '/maze/goal_events',
)

DEFAULT_ENTRANCE = {'x_m': -11.011281, 'y_m': -9.025070, 'yaw_rad': 0.0}
DEFAULT_EXIT = {'x_m': 10.061281, 'y_m': 9.058496, 'radius_m': 1.2}


def _stamp_dict(stamp: Any) -> dict[str, int]:
    return {'sec': int(getattr(stamp, 'sec', 0)), 'nanosec': int(getattr(stamp, 'nanosec', 0))}


def _header_dict(header: Any) -> dict[str, Any]:
    return {'stamp': _stamp_dict(header.stamp), 'frame_id': str(header.frame_id)}


def _position_dict(p: Any) -> dict[str, float]:
    return {'x': float(p.x), 'y': float(p.y), 'z': float(p.z)}


def _quaternion_dict(q: Any) -> dict[str, float]:
    return {'x': float(q.x), 'y': float(q.y), 'z': float(q.z), 'w': float(q.w)}


def _quat_yaw(q: dict[str, float]) -> float:
    return math.atan2(
        2.0 * (q.get('w', 1.0) * q.get('z', 0.0) + q.get('x', 0.0) * q.get('y', 0.0)),
        1.0 - 2.0 * (q.get('y', 0.0) ** 2 + q.get('z', 0.0) ** 2),
    )


def _transform_to_dict(transform_stamped: Any) -> dict[str, Any]:
    t = transform_stamped.transform.translation
    r = transform_stamped.transform.rotation
    return {
        'header': _header_dict(transform_stamped.header),
        'child_frame_id': str(transform_stamped.child_frame_id),
        'translation': _position_dict(t),
        'rotation': _quaternion_dict(r),
        'yaw_rad': _quat_yaw(_quaternion_dict(r)),
    }


def _lookup_transform_dict(buffer: Any, parent: str, child: str) -> dict[str, Any]:
    if buffer is None or rclpy is None:
        return {'available': False, 'error': 'tf2 unavailable'}
    try:
        # tf2_ros.Buffer lookup: map->base_link, odom->base_link, map->odom.
        msg = buffer.lookup_transform(parent, child, rclpy.time.Time())
        row = _transform_to_dict(msg)
        row['available'] = True
        return row
    except Exception as exc:
        return {'available': False, 'error': str(exc), 'parent': parent, 'child': child}


def _grid_msg_to_dict(msg: Any | None) -> dict[str, Any] | None:
    if msg is None:
        return None
    origin = msg.info.origin
    return {
        'header': _header_dict(msg.header),
        'info': {
            'map_load_time': _stamp_dict(msg.info.map_load_time),
            'resolution': float(msg.info.resolution),
            'width': int(msg.info.width),
            'height': int(msg.info.height),
            'origin': {
                'position': _position_dict(origin.position),
                'orientation': _quaternion_dict(origin.orientation),
            },
        },
        'data': [int(v) for v in msg.data],
    }


def _scan_msg_to_dict(msg: Any | None) -> dict[str, Any] | None:
    if msg is None:
        return None
    return {
        'header': _header_dict(msg.header),
        'angle_min': float(msg.angle_min),
        'angle_max': float(msg.angle_max),
        'angle_increment': float(msg.angle_increment),
        'time_increment': float(msg.time_increment),
        'scan_time': float(msg.scan_time),
        'range_min': float(msg.range_min),
        'range_max': float(msg.range_max),
        'ranges': [float(v) for v in msg.ranges],
        'intensities': [float(v) for v in msg.intensities],
    }


def _tf_message_to_dict(msg: Any | None) -> dict[str, Any] | None:
    if msg is None:
        return None
    return {'transforms': [_transform_to_dict(t) for t in msg.transforms]}


def _json_string_payload(msg: Any, elapsed_sec: float) -> dict[str, Any]:
    try:
        payload = json.loads(msg.data)
    except Exception as exc:
        payload = {'_parse_error': str(exc), 'raw': msg.data}
    return {'elapsed_sec': elapsed_sec, 'wall_time': time.time(), 'state': payload}


def _load_metadata(path: Path | None) -> dict[str, Any]:
    if path and path.exists():
        return yaml.safe_load(path.read_text(encoding='utf-8')) or {}
    return {'entrance': dict(DEFAULT_ENTRANCE), 'exit': dict(DEFAULT_EXIT)}


def _grid_norm(grid: dict[str, Any] | None) -> dict[str, Any]:
    if not grid:
        return {'available': False, 'width': 0, 'height': 0, 'resolution': 0.0, 'origin_x': 0.0, 'origin_y': 0.0, 'data': []}
    info = grid.get('info') or {}
    origin = ((info.get('origin') or {}).get('position') or {})
    data = [int(v) for v in (grid.get('data') or [])]
    width = int(info.get('width') or 0)
    height = int(info.get('height') or 0)
    return {
        'available': width > 0 and height > 0 and len(data) == width * height,
        'frame_id': (grid.get('header') or {}).get('frame_id'),
        'width': width,
        'height': height,
        'resolution': float(info.get('resolution') or 0.0),
        'origin_x': float(origin.get('x') or 0.0),
        'origin_y': float(origin.get('y') or 0.0),
        'data': data,
        'expected_len': width * height,
        'observed_len': len(data),
    }


def _grid_value(grid: dict[str, Any], cell_x: int, cell_y: int) -> int | None:
    width = int(grid.get('width') or 0)
    height = int(grid.get('height') or 0)
    if cell_x < 0 or cell_y < 0 or cell_x >= width or cell_y >= height:
        return None
    idx = cell_x + cell_y * width
    data = grid.get('data') or []
    if idx < 0 or idx >= len(data):
        return -1
    return int(data[idx])


def _world_to_cell(grid: dict[str, Any], x: float, y: float) -> tuple[int, int]:
    res = float(grid.get('resolution') or 0.0)
    if res <= 0.0:
        return (0, 0)
    return (int(math.floor((x - grid.get('origin_x', 0.0)) / res)), int(math.floor((y - grid.get('origin_y', 0.0)) / res)))


def _cell_to_world(grid: dict[str, Any], cell_x: int, cell_y: int) -> tuple[float, float]:
    res = float(grid.get('resolution') or 0.0)
    return (grid.get('origin_x', 0.0) + (cell_x + 0.5) * res, grid.get('origin_y', 0.0) + (cell_y + 0.5) * res)


def _pose_from_snapshot(snapshot: dict[str, Any]) -> dict[str, Any]:
    lookup = ((snapshot.get('tf_lookups') or {}).get('map->base_link') or {})
    if lookup.get('available'):
        t = lookup.get('translation') or {}
        r = lookup.get('rotation') or {}
        return {
            'available': True,
            'x': float(t.get('x') or 0.0),
            'y': float(t.get('y') or 0.0),
            'yaw': float(lookup.get('yaw_rad', _quat_yaw({k: float(r.get(k, 0.0 if k != 'w' else 1.0)) for k in ('x', 'y', 'z', 'w')}))),
            'source': 'tf2 map->base_link',
        }
    return {'available': False, 'x': 0.0, 'y': 0.0, 'yaw': 0.0, 'source': lookup.get('error', 'map->base_link unavailable')}


def _scan_diag(scan: dict[str, Any] | None) -> dict[str, Any]:
    ranges = (scan or {}).get('ranges') or []
    finite = []
    for raw in ranges:
        try:
            v = float(raw)
        except Exception:
            continue
        if math.isfinite(v):
            finite.append(v)
    return {
        'available': scan is not None,
        'sample_count': len(ranges),
        'finite_count': len(finite),
        'finite_ratio': (len(finite) / len(ranges)) if ranges else 0.0,
        'nearest_obstacle_m': min(finite) if finite else None,
        'range_min': (scan or {}).get('range_min'),
        'range_max': (scan or {}).get('range_max'),
    }


def _cell_stats_near(grid: dict[str, Any], pose: dict[str, Any], radius_m: float) -> dict[str, Any]:
    if not grid.get('available') or not pose.get('available') or float(grid.get('resolution') or 0.0) <= 0.0:
        return {
            'available': False,
            'radius_m': radius_m,
            'total_cells': 0,
            'known_ratio_near_robot': 0.0,
            'free_ratio_near_robot': 0.0,
            'occupied_ratio_near_robot': 0.0,
            'unknown_ratio_near_robot': 1.0,
        }
    cx, cy = _world_to_cell(grid, float(pose['x']), float(pose['y']))
    radius_cells = int(math.ceil(radius_m / grid['resolution']))
    counts = Counter()
    total = 0
    for yy in range(cy - radius_cells, cy + radius_cells + 1):
        for xx in range(cx - radius_cells, cx + radius_cells + 1):
            wx, wy = _cell_to_world(grid, xx, yy)
            if math.hypot(wx - pose['x'], wy - pose['y']) > radius_m:
                continue
            total += 1
            value = _grid_value(grid, xx, yy)
            if value is None:
                counts['out_of_bounds'] += 1
                counts['unknown'] += 1
            elif value < 0:
                counts['unknown'] += 1
            elif value >= 65:
                counts['occupied'] += 1
            else:
                counts['free'] += 1
    known = counts['free'] + counts['occupied']
    return {
        'available': True,
        'center_cell': {'x': cx, 'y': cy},
        'radius_m': radius_m,
        'total_cells': total,
        'free_cells': counts['free'],
        'occupied_cells': counts['occupied'],
        'unknown_cells': counts['unknown'],
        'out_of_bounds_cells': counts['out_of_bounds'],
        'known_ratio_near_robot': (known / total) if total else 0.0,
        'free_ratio_near_robot': (counts['free'] / total) if total else 0.0,
        'occupied_ratio_near_robot': (counts['occupied'] / total) if total else 0.0,
        'unknown_ratio_near_robot': (counts['unknown'] / total) if total else 1.0,
    }


def _has_clearance(grid: dict[str, Any], cell: tuple[int, int], radius_m: float, unknown_is_safe: bool = False) -> bool:
    if not grid.get('available') or grid.get('resolution', 0.0) <= 0.0:
        return False
    cx, cy = cell
    value = _grid_value(grid, cx, cy)
    if value is None or value >= 65 or (value < 0 and not unknown_is_safe):
        return False
    radius_cells = int(math.ceil(max(radius_m, 0.0) / grid['resolution']))
    for y in range(cy - radius_cells, cy + radius_cells + 1):
        for x in range(cx - radius_cells, cx + radius_cells + 1):
            if math.hypot(x - cx, y - cy) * grid['resolution'] > radius_m + 1e-9:
                continue
            v = _grid_value(grid, x, y)
            if v is None or v >= 65 or (v < 0 and not unknown_is_safe):
                return False
    return True


def _sample_topology(grid: dict[str, Any], pose: dict[str, Any]) -> dict[str, Any]:
    params = {
        'angle_step_deg': 90.0,
        'lookahead_m': 1.6,
        'clearance_radius_m': 0.38,
        'min_open_distance_m': 0.55,
    }
    samples = []
    if not grid.get('available') or not pose.get('available'):
        return {'attempted': False, 'parameters': params, 'sampled_direction_count': 0, 'open_direction_count_recomputed': 0, 'samples': []}
    step = max(float(grid['resolution']) * 0.5, 0.05)
    for i in range(4):
        angle = math.atan2(math.sin(pose['yaw'] + math.radians(params['angle_step_deg']) * i), math.cos(pose['yaw'] + math.radians(params['angle_step_deg']) * i))
        best = 0.0
        reason = 'lookahead_complete'
        block_xy = (pose['x'] + math.cos(angle) * params['lookahead_m'], pose['y'] + math.sin(angle) * params['lookahead_m'])
        dist = step
        while dist <= params['lookahead_m'] + 1e-9:
            x = pose['x'] + math.cos(angle) * dist
            y = pose['y'] + math.sin(angle) * dist
            cell = _world_to_cell(grid, x, y)
            value = _grid_value(grid, *cell)
            if not _has_clearance(grid, cell, params['clearance_radius_m']):
                if value is None:
                    reason = 'out_of_bounds_or_map_edge'
                elif value < 0:
                    reason = 'unknown_cell_or_unknown_clearance'
                elif value >= 65:
                    reason = 'occupied_or_inflated_clearance'
                else:
                    reason = 'clearance_radius_blocked'
                block_xy = (x, y)
                break
            best = dist
            dist += step
        samples.append({
            'index': i,
            'angle_rad': angle,
            'angle_deg': math.degrees(angle),
            'safe_distance_m': min(best, params['lookahead_m']),
            'first_block_reason': reason,
            'first_block_xy': {'x': block_xy[0], 'y': block_xy[1]},
            'is_open': best >= params['min_open_distance_m'],
        })
    return {
        'attempted': True,
        'parameters': params,
        'sampled_direction_count': len(samples),
        'open_direction_count_recomputed': sum(1 for s in samples if s['is_open']),
        'samples': samples,
        'block_reason_counts': dict(Counter(s['first_block_reason'] for s in samples)),
    }


def _is_map_sufficient(map_stats: dict[str, Any], local_stats: dict[str, Any], scan: dict[str, Any], pose: dict[str, Any]) -> bool:
    return bool(
        pose.get('available')
        and scan.get('finite_count', 0) > 0
        and map_stats.get('known_ratio_near_robot', 0.0) >= 0.50
        and map_stats.get('free_ratio_near_robot', 0.0) >= 0.25
        and local_stats.get('known_ratio_near_robot', 0.0) >= 0.30
    )


def _active_truth(full: dict[str, Any]) -> dict[str, Any]:
    truth = full.get('active_truth') or {}
    entrance = truth.get('entrance') or (full.get('metadata') or {}).get('entrance') or DEFAULT_ENTRANCE
    exit_pose = truth.get('exit') or (full.get('metadata') or {}).get('exit') or DEFAULT_EXIT
    return {
        'entrance': {'x_m': float(entrance.get('x_m', DEFAULT_ENTRANCE['x_m'])), 'y_m': float(entrance.get('y_m', DEFAULT_ENTRANCE['y_m'])), 'yaw_rad': float(entrance.get('yaw_rad', 0.0))},
        'exit': {'x_m': float(exit_pose.get('x_m', DEFAULT_EXIT['x_m'])), 'y_m': float(exit_pose.get('y_m', DEFAULT_EXIT['y_m'])), 'radius_m': float(exit_pose.get('radius_m', DEFAULT_EXIT['radius_m']))},
    }


def _first_explorer_state(full: dict[str, Any]) -> dict[str, Any] | None:
    samples = []
    for snap in full.get('snapshots') or []:
        samples.extend(snap.get('explorer_state_samples') or [])
    samples.extend(full.get('explorer_state_samples') or [])
    samples = sorted(samples, key=lambda r: float(r.get('elapsed_sec', 1e18)))
    return samples[0] if samples else None


def _first_failed_exhausted(full: dict[str, Any]) -> dict[str, Any] | None:
    samples = []
    for snap in full.get('snapshots') or []:
        samples.extend(snap.get('explorer_state_samples') or [])
    samples.extend(full.get('explorer_state_samples') or [])
    rows = []
    for row in samples:
        state = row.get('state') or row
        if state.get('mode') == 'FAILED_EXHAUSTED':
            rows.append(row)
    rows = sorted(rows, key=lambda r: float(r.get('elapsed_sec', 1e18)))
    return rows[0] if rows else None


def _timeline(full: dict[str, Any]) -> list[dict[str, Any]]:
    rows = []
    for snap in full.get('snapshots') or []:
        pose = _pose_from_snapshot(snap)
        map_grid = _grid_norm(snap.get('map'))
        local_grid = _grid_norm(snap.get('local_costmap'))
        global_grid = _grid_norm(snap.get('global_costmap'))
        map_stats = _cell_stats_near(map_grid, pose, 1.5)
        local_stats = _cell_stats_near(local_grid, pose, 1.0)
        global_stats = _cell_stats_near(global_grid, pose, 1.5)
        scan = _scan_diag(snap.get('scan'))
        topology = _sample_topology(map_grid, pose)
        sufficient = _is_map_sufficient(map_stats, local_stats, scan, pose)
        rows.append({
            'elapsed_sec': float(snap.get('elapsed_sec', 0.0)),
            'robot_pose_map': pose,
            'map': map_stats,
            'local_costmap': local_stats,
            'global_costmap': global_stats,
            'scan': scan,
            'topology_sampling': topology,
            'sufficient_for_initial_topology': sufficient,
        })
    return rows


def _snapshot_at_or_before(rows: list[dict[str, Any]], t: float | None) -> dict[str, Any] | None:
    if t is None or not rows:
        return rows[0] if rows else None
    before = [r for r in rows if r['elapsed_sec'] <= t]
    if before:
        return before[-1]
    return rows[0]


def _snapshot_after(rows: list[dict[str, Any]], t: float | None) -> dict[str, Any] | None:
    if t is None or not rows:
        return rows[-1] if rows else None
    after = [r for r in rows if r['elapsed_sec'] >= t]
    if after:
        return after[0]
    return rows[-1]


def _classify(full: dict[str, Any], rows: list[dict[str, Any]], truth: dict[str, Any]) -> tuple[str, list[str]]:
    if not rows:
        return 'INSUFFICIENT_EVIDENCE', ['no bounded snapshots were captured']
    first_state = _first_explorer_state(full)
    first_state_time = float(first_state.get('elapsed_sec')) if first_state else None
    before = _snapshot_at_or_before(rows, first_state_time)
    after = _snapshot_after(rows, first_state_time)
    first_sufficient = next((r for r in rows if r.get('sufficient_for_initial_topology')), None)
    failed = _first_failed_exhausted(full)
    failed_time = float(failed.get('elapsed_sec')) if failed else None
    ent = truth['entrance']
    latest_pose = next((r['robot_pose_map'] for r in rows if r['robot_pose_map'].get('available')), {'available': False})
    distance = None
    if latest_pose.get('available'):
        distance = math.hypot(latest_pose['x'] - ent['x_m'], latest_pose['y'] - ent['y_m'])
    reasons = []
    if before is None or not before['robot_pose_map'].get('available'):
        return 'INSUFFICIENT_EVIDENCE', ['map->base_link pose unavailable at startup evidence time']
    if distance is not None and distance > 2.0:
        reasons.append(f'robot pose is {distance:.3f} m from active entrance')
        return 'FRAME_ALIGNMENT_ISSUE_CONFIRMED', reasons
    if first_state and (before['scan'].get('finite_count', 0) <= 0 or not before['map'].get('available') or not before['local_costmap'].get('available')):
        return 'SENSOR_FLOW_NOT_READY_AT_TOPOLOGY_TIME', ['scan/map/local-costmap flow was not ready at first explorer_state evidence time']
    if failed and first_sufficient and failed_time is not None and failed_time < first_sufficient['elapsed_sec']:
        return 'MAP_SUFFICIENCY_DELAY_REQUIRED', [
            f'FAILED_EXHAUSTED appeared at {failed_time:.3f}s before first sufficient snapshot at {first_sufficient["elapsed_sec"]:.3f}s'
        ]
    if first_state and before and not before.get('sufficient_for_initial_topology') and distance is not None and distance <= 0.75:
        return 'START_POSE_ALIGNMENT_OK_BUT_MAP_NOT_READY', ['robot/entrance alignment is within 0.75 m but startup map evidence is not sufficient']
    if after and not after.get('sufficient_for_initial_topology') and distance is not None and distance <= 0.75:
        return 'START_POSE_ALIGNMENT_OK_BUT_MAP_NOT_READY', ['bounded window ended with aligned pose but insufficient map/costmap/scan evidence']
    return 'INSUFFICIENT_EVIDENCE', reasons or ['bounded evidence did not match a stronger Phase39 classification']


def _render_grid(grid: dict[str, Any], scale: int = 4) -> Image.Image:
    width = max(1, int(grid.get('width') or 1))
    height = max(1, int(grid.get('height') or 1))
    img = Image.new('RGB', (width * scale, height * scale), 'white')
    pix = img.load()
    for y in range(height):
        for x in range(width):
            v = _grid_value(grid, x, y)
            if v is None:
                color = (220, 220, 220)
            elif v < 0:
                color = (160, 160, 160)
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


def _world_to_px(grid: dict[str, Any], x: float, y: float, scale: int) -> tuple[int, int]:
    cx, cy = _world_to_cell(grid, x, y)
    return (int((cx + 0.5) * scale), int((grid.get('height', 1) - cy - 0.5) * scale))


def _draw_cross(draw: ImageDraw.ImageDraw, p: tuple[int, int], color: tuple[int, int, int], r: int = 6) -> None:
    x, y = p
    draw.line((x - r, y, x + r, y), fill=color, width=2)
    draw.line((x, y - r, x, y + r), fill=color, width=2)


def _write_overlays(output_dir: Path, full: dict[str, Any], rows: list[dict[str, Any]], truth: dict[str, Any]) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    snap_with_map = None
    for snap in full.get('snapshots') or []:
        if snap.get('map'):
            snap_with_map = snap
            break
    if snap_with_map is None:
        Image.new('RGB', (300, 220), 'white').save(output_dir / 'robot_vs_entrance_frame_alignment_overlay.png')
        Image.new('RGB', (300, 220), 'white').save(output_dir / 'local_map_known_free_overlay.png')
        Image.new('RGB', (300, 220), 'white').save(output_dir / 'topology_sampling_after_full_map_overlay.png')
        return
    map_grid = _grid_norm(snap_with_map.get('map'))
    local_grid = _grid_norm(snap_with_map.get('local_costmap') or snap_with_map.get('map'))
    scale = 6 if map_grid.get('width', 0) <= 120 else 3
    base = _render_grid(map_grid, scale=scale)
    pose = _pose_from_snapshot(snap_with_map)
    ent = truth['entrance']
    img = base.copy()
    draw = ImageDraw.Draw(img)
    if pose.get('available'):
        robot_px = _world_to_px(map_grid, pose['x'], pose['y'], scale)
        entrance_px = _world_to_px(map_grid, ent['x_m'], ent['y_m'], scale)
        _draw_cross(draw, robot_px, (255, 0, 0), 7)
        _draw_cross(draw, entrance_px, (0, 180, 0), 7)
        draw.line((*robot_px, *entrance_px), fill=(255, 180, 0), width=2)
    img.save(output_dir / 'robot_vs_entrance_frame_alignment_overlay.png')

    scale_local = 6 if local_grid.get('width', 0) <= 120 else 3
    img = _render_grid(local_grid, scale=scale_local)
    draw = ImageDraw.Draw(img)
    if pose.get('available'):
        _draw_cross(draw, _world_to_px(local_grid, pose['x'], pose['y'], scale_local), (255, 0, 0), 7)
    img.save(output_dir / 'local_map_known_free_overlay.png')

    topo_row = next((r for r in rows if r.get('sufficient_for_initial_topology')), rows[-1] if rows else None)
    topo_snap = None
    if topo_row is not None:
        target_t = topo_row['elapsed_sec']
        topo_snap = min(full.get('snapshots') or [snap_with_map], key=lambda s: abs(float(s.get('elapsed_sec', 0.0)) - target_t))
    topo_grid = _grid_norm((topo_snap or snap_with_map).get('map'))
    topo_pose = _pose_from_snapshot(topo_snap or snap_with_map)
    topo = _sample_topology(topo_grid, topo_pose)
    img = _render_grid(topo_grid, scale=scale)
    draw = ImageDraw.Draw(img)
    if topo_pose.get('available'):
        robot_px = _world_to_px(topo_grid, topo_pose['x'], topo_pose['y'], scale)
        _draw_cross(draw, robot_px, (255, 0, 0), 7)
        for sample in topo.get('samples') or []:
            block = sample['first_block_xy']
            px = _world_to_px(topo_grid, block['x'], block['y'], scale)
            color = (0, 180, 0) if sample.get('is_open') else (255, 0, 0)
            draw.line((*robot_px, *px), fill=color, width=2)
            _draw_cross(draw, px, color, 3)
    img.save(output_dir / 'topology_sampling_after_full_map_overlay.png')


def analyze_full_data(full: dict[str, Any], output_dir: Path) -> dict[str, Any]:
    output_dir.mkdir(parents=True, exist_ok=True)
    truth = _active_truth(full)
    rows = _timeline(full)
    conclusion, reasons = _classify(full, rows, truth)
    first_state = _first_explorer_state(full)
    first_failed = _first_failed_exhausted(full)
    first_sufficient = next((r for r in rows if r.get('sufficient_for_initial_topology')), None)
    last_pose = next((r['robot_pose_map'] for r in reversed(rows) if r['robot_pose_map'].get('available')), {'available': False, 'x': 0.0, 'y': 0.0, 'yaw': 0.0})
    ent = truth['entrance']
    distance = math.hypot(last_pose.get('x', 0.0) - ent['x_m'], last_pose.get('y', 0.0) - ent['y_m']) if last_pose.get('available') else None
    before = _snapshot_at_or_before(rows, float(first_state.get('elapsed_sec')) if first_state else None)
    result = {
        'phase': 'Phase39 Startup Map Sufficiency and Entrance Frame Alignment Evidence',
        'allowed_conclusions': sorted(PHASE39_ALLOWED_CONCLUSIONS),
        'conclusion': conclusion,
        'classification_reasons': reasons,
        'autonomous_success_claimed': False,
        'phase37_semantics_preserved': 'BOUNDED_SMOKE_PARTIAL_FAIL_NO_DISPATCH',
        'phase38_conclusion_preserved': 'INSUFFICIENT_MAP_AT_START',
        'guardrails': {
            'bounded_startup_diagnostics_only': True,
            'nav2_mppi_controller_params_modified': False,
            'maze_explorer_strategy_modified': False,
            'fallback_or_terminal_acceptance_continued': False,
            'old_scaffold_world_used': False,
            'long_run_performed': False,
            'relied_on_ros2_topic_echo_truncated_map': False,
        },
        'active_truth': truth,
        'metadata': full.get('metadata') or {},
        'snapshot_count': len(full.get('snapshots') or []),
        'map_sufficiency_timeline': rows,
        'first_explorer_state': first_state,
        'first_failed_exhausted': first_failed,
        'first_sufficient_snapshot': first_sufficient,
        'failed_exhausted_before_map_sufficient': bool(first_failed and (first_sufficient is None or float(first_failed.get('elapsed_sec', 1e18)) < first_sufficient['elapsed_sec'])),
        'robot_pose_map': last_pose,
        'frame_alignment': {
            'active_entrance': truth['entrance'],
            'robot_pose_map': last_pose,
            'robot_to_active_entrance_distance_m': distance,
            'alignment_ok_threshold_m': 0.75,
            'frame_issue_threshold_m': 2.0,
            'map_odom_lookup_samples': [((snap.get('tf_lookups') or {}).get('map->odom') or {}) for snap in full.get('snapshots') or []],
            'odom_base_link_lookup_samples': [((snap.get('tf_lookups') or {}).get('odom->base_link') or {}) for snap in full.get('snapshots') or []],
            'map_base_link_lookup_samples': [((snap.get('tf_lookups') or {}).get('map->base_link') or {}) for snap in full.get('snapshots') or []],
        },
        'sensor_flow': {
            'first_explorer_state_time_scan_finite_count': (before or {}).get('scan', {}).get('finite_count', 0),
            'first_explorer_state_time_map_available': bool((before or {}).get('map', {}).get('available')),
            'first_explorer_state_time_local_costmap_available': bool((before or {}).get('local_costmap', {}).get('available')),
        },
        'artifacts': {
            'summary_json': str(output_dir / 'phase39_startup_map_sufficiency.json'),
            'full_data_json': str(output_dir / 'phase39_startup_full_data.json'),
            'robot_vs_entrance_frame_alignment_overlay': str(output_dir / 'robot_vs_entrance_frame_alignment_overlay.png'),
            'local_map_known_free_overlay': str(output_dir / 'local_map_known_free_overlay.png'),
            'topology_sampling_after_full_map_overlay': str(output_dir / 'topology_sampling_after_full_map_overlay.png'),
        },
    }
    _write_overlays(output_dir, full, rows, truth)
    (output_dir / 'phase39_startup_map_sufficiency.json').write_text(json.dumps(result, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    return result


if rclpy is not None:
    class Phase39Recorder(Node):
        def __init__(self, args: argparse.Namespace, metadata: dict[str, Any]) -> None:
            super().__init__('phase39_startup_map_sufficiency_recorder')
            self.args = args
            self.metadata_doc = metadata
            self.started_at = time.time()
            self.latest_map = None
            self.latest_local = None
            self.latest_global = None
            self.latest_scan = None
            self.latest_tf = None
            self.explorer_state_samples: list[dict[str, Any]] = []
            self.goal_event_samples: list[dict[str, Any]] = []
            self.snapshots: list[dict[str, Any]] = []
            self.stop_requested = False
            self.first_explorer_state_snapshot_taken = False
            self.first_failed_exhausted_snapshot_taken = False
            self.snapshot_times = [float(v) for v in args.snapshots.split(',') if v.strip()]
            self.next_snapshot_idx = 0

            grid_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=5, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            normal_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=50, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
            self.create_subscription(OccupancyGrid, '/map', lambda m: setattr(self, 'latest_map', m), grid_qos)
            self.create_subscription(OccupancyGrid, '/local_costmap/costmap', lambda m: setattr(self, 'latest_local', m), grid_qos)
            self.create_subscription(OccupancyGrid, '/global_costmap/costmap', lambda m: setattr(self, 'latest_global', m), grid_qos)
            self.create_subscription(LaserScan, '/scan', lambda m: setattr(self, 'latest_scan', m), qos_profile_sensor_data)
            self.create_subscription(TFMessage, '/tf', lambda m: setattr(self, 'latest_tf', m), normal_qos)
            self.create_subscription(String, '/maze/explorer_state', self._on_explorer_state, normal_qos)
            self.create_subscription(String, '/maze/goal_events', self._on_goal_event, normal_qos)
            self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=120.0))
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
            self.timer = self.create_timer(0.1, self._on_timer)
            self.get_logger().info('Phase39 recorder subscribed to full /map, costmap, /scan, /tf, explorer_state, and goal_events evidence')

        def elapsed(self) -> float:
            return time.time() - self.started_at

        def _on_explorer_state(self, msg: Any) -> None:
            row = _json_string_payload(msg, self.elapsed())
            self.explorer_state_samples.append(row)
            state = row.get('state') or {}
            if not self.first_explorer_state_snapshot_taken:
                self.first_explorer_state_snapshot_taken = True
                self.take_snapshot(float(row.get('elapsed_sec', self.elapsed())))
            if state.get('mode') == 'FAILED_EXHAUSTED' and not self.first_failed_exhausted_snapshot_taken:
                self.first_failed_exhausted_snapshot_taken = True
                self.take_snapshot(float(row.get('elapsed_sec', self.elapsed())))

        def _on_goal_event(self, msg: Any) -> None:
            self.goal_event_samples.append(_json_string_payload(msg, self.elapsed()))

        def _on_timer(self) -> None:
            elapsed = self.elapsed()
            if self.next_snapshot_idx < len(self.snapshot_times) and elapsed >= self.snapshot_times[self.next_snapshot_idx]:
                self.take_snapshot(self.snapshot_times[self.next_snapshot_idx])
                self.next_snapshot_idx += 1
            if elapsed >= self.args.duration_sec:
                self.stop_requested = True

        def take_snapshot(self, target_elapsed: float | None = None) -> None:
            lookup_specs = {'map->base_link': ('map', self.args.base_frame), 'odom->base_link': ('odom', self.args.base_frame), 'map->odom': ('map', 'odom')}
            snap = {
                'elapsed_sec': self.elapsed(),
                'target_elapsed_sec': target_elapsed,
                'wall_time': time.time(),
                'map': _grid_msg_to_dict(self.latest_map),
                'local_costmap': _grid_msg_to_dict(self.latest_local),
                'global_costmap': _grid_msg_to_dict(self.latest_global),
                'scan': _scan_msg_to_dict(self.latest_scan),
                'tf': _tf_message_to_dict(self.latest_tf),
                'tf_lookups': {name: _lookup_transform_dict(self.tf_buffer, parent, child) for name, (parent, child) in lookup_specs.items()},
                'explorer_state_samples': list(self.explorer_state_samples),
                'goal_event_samples': list(self.goal_event_samples),
            }
            self.snapshots.append(snap)
            self.get_logger().info(f'captured Phase39 snapshot target={target_elapsed} actual={snap["elapsed_sec"]:.3f}s')

        def full_data(self) -> dict[str, Any]:
            return {
                'phase': 'Phase39 Startup Map Sufficiency and Entrance Frame Alignment Evidence',
                'metadata': {
                    'active_world': str(self.args.active_world),
                    'active_metadata': str(self.args.maze_config) if self.args.maze_config else None,
                    'guardrails': {'bounded_startup_diagnostics_only': True},
                    'topics': list(TOPICS),
                    'snapshot_targets_sec': self.snapshot_times,
                    'duration_sec': self.args.duration_sec,
                },
                'active_truth': {
                    'entrance': self.metadata_doc.get('entrance') or DEFAULT_ENTRANCE,
                    'exit': self.metadata_doc.get('exit') or DEFAULT_EXIT,
                },
                'snapshots': self.snapshots,
                'explorer_state_samples': self.explorer_state_samples,
                'goal_event_samples': self.goal_event_samples,
            }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, default=Path('log/phase39_startup_map_sufficiency'))
    parser.add_argument('--duration-sec', type=float, default=95.0)
    parser.add_argument('--snapshots', default='30,60,90')
    parser.add_argument('--maze-config', type=Path, default=Path('src/tugbot_maze/config/maze_20260528_scaled_instance.yaml'))
    parser.add_argument('--active-world', default='src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean_scaled2x.sdf')
    parser.add_argument('--base-frame', default='base_link')
    parser.add_argument('--analyze-existing', type=Path, help='Analyze a previously captured full-data JSON without starting ROS subscriptions.')
    return parser.parse_args()


def _run_offline(args: argparse.Namespace) -> int:
    full = json.loads(args.analyze_existing.read_text(encoding='utf-8'))
    result = analyze_full_data(full, args.output_dir.resolve())
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


def _run_runtime(args: argparse.Namespace) -> int:
    if rclpy is None:
        raise RuntimeError('rclpy is unavailable; source /opt/ros/jazzy/setup.bash and workspace install/setup.bash for runtime recording')
    args.output_dir.mkdir(parents=True, exist_ok=True)
    metadata = _load_metadata(args.maze_config)
    rclpy.init()
    node = Phase39Recorder(args, metadata)

    def _sigterm(_signum: int, _frame: Any) -> None:
        node.stop_requested = True

    signal.signal(signal.SIGTERM, _sigterm)
    try:
        while rclpy.ok() and not node.stop_requested:
            try:
                rclpy.spin_once(node, timeout_sec=0.2)
            except rclpy.executors.ExternalShutdownException:
                break
        if not node.snapshots or (node.snapshot_times and len(node.snapshots) < len(node.snapshot_times)):
            node.take_snapshot(None)
        full = node.full_data()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    full_path = args.output_dir / 'phase39_startup_full_data.json'
    full_path.write_text(json.dumps(full, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    result = analyze_full_data(full, args.output_dir)
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


def main() -> int:
    args = parse_args()
    if args.analyze_existing:
        return _run_offline(args)
    return _run_runtime(args)


if __name__ == '__main__':
    sys.exit(main())
