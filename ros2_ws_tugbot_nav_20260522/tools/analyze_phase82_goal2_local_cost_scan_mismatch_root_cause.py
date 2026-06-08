#!/usr/bin/env python3
"""Phase82 Goal2 local-cost / scan mismatch root-cause analyzer.

Diagnostics only: read Phase81 raw scan/local-costmap/footprint/pose evidence and
classify the likely source of the split where physical forward scan clearance is
open but the Nav2 local-cost execution corridor is high/lethal.

This script does not modify navigation strategy, Nav2 parameters, costmap
parameters, branch scoring, centerline gates, directional readiness, or fallback
/ terminal acceptance.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

RUN_ID = 'phase82_goal2_local_cost_scan_mismatch_root_cause'

INFLATION = 'INFLATION_SPILLOVER_SUSPECTED'
FOOTPRINT = 'FOOTPRINT_OR_WEDGE_PROJECTION_SUSPECTED'
SAMPLING = 'CORRIDOR_SAMPLING_TOO_WIDE_SUSPECTED'
POSE_TF = 'POSE_TF_PROJECTION_MISMATCH_SUSPECTED'
NARROW = 'REAL_EXECUTION_CLEARANCE_TOO_NARROW'
STALE = 'COSTMAP_STALE_OR_OBSERVATION_MISMATCH'
INSUFFICIENT = 'ROOT_CAUSE_INSUFFICIENT_EVIDENCE'

ALLOWED_ROOT_CAUSE_CLASSIFICATIONS = [
    INFLATION,
    FOOTPRINT,
    SAMPLING,
    POSE_TF,
    NARROW,
    STALE,
    INSUFFICIENT,
]

GUARDRAILS = [
    'diagnostics-only root-cause classification',
    'No maze_explorer strategy changed',
    'No branch scoring changed',
    'No centerline gate changed',
    'No directional readiness override changed',
    'No fallback/terminal acceptance changed',
    'No Nav2/MPPI/controller tuning',
    'No inflation/robot_radius/clearance_radius_m/map threshold tuning',
    'No autonomous exploration success claimed',
    'No exit success claimed',
    'Phase83 not entered',
]

HIGH_COST_THRESHOLD = 70
LETHAL_COST_THRESHOLD = 99
STALE_DELTA_SEC = 2.0
SCAN_OPEN_CLEARANCE_M = 0.80
SCAN_NARROW_CLEARANCE_M = 0.72
DEFAULT_CORRIDOR_LENGTH_M = 1.0
DEFAULT_CORRIDOR_HALF_WIDTH_M = 0.30
NARROW_EXECUTION_WIDTH_M = 0.72


def _num(value: Any) -> float | None:
    try:
        out = float(value)
    except (TypeError, ValueError):
        return None
    return out if math.isfinite(out) else None


def _load_json(path: str | Path) -> dict[str, Any]:
    p = Path(path)
    if not p.exists() or not p.stat().st_size:
        return {}
    return json.loads(p.read_text(encoding='utf-8', errors='replace'))


def _yaw_from_quat(orientation: dict[str, Any]) -> float | None:
    x = _num(orientation.get('x')) or 0.0
    y = _num(orientation.get('y')) or 0.0
    z = _num(orientation.get('z')) or 0.0
    w = _num(orientation.get('w'))
    if w is None:
        return None
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _pose(value: Any) -> list[float] | None:
    if isinstance(value, (list, tuple)) and len(value) >= 3:
        x, y, yaw = _num(value[0]), _num(value[1]), _num(value[2])
        if x is not None and y is not None and yaw is not None:
            return [x, y, yaw]
    if isinstance(value, dict):
        x, y, yaw = _num(value.get('x')), _num(value.get('y')), _num(value.get('yaw'))
        if x is not None and y is not None and yaw is not None:
            return [x, y, yaw]
    return None


def _angle_norm(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def _local_costmap_geometry(costmap: dict[str, Any]) -> dict[str, Any] | None:
    info = costmap.get('info') if isinstance(costmap.get('info'), dict) else {}
    origin = info.get('origin') if isinstance(info.get('origin'), dict) else {}
    pos = origin.get('position') if isinstance(origin.get('position'), dict) else {}
    width = int(info.get('width') or 0)
    height = int(info.get('height') or 0)
    resolution = _num(info.get('resolution'))
    ox, oy = _num(pos.get('x')), _num(pos.get('y'))
    yaw = _num(origin.get('orientation_yaw'))
    if yaw is None:
        ori = origin.get('orientation') if isinstance(origin.get('orientation'), dict) else {}
        yaw = _yaw_from_quat(ori)
    if not width or not height or resolution is None or resolution <= 0 or ox is None or oy is None:
        return None
    return {
        'width': width,
        'height': height,
        'resolution': resolution,
        'origin_x': ox,
        'origin_y': oy,
        'origin_yaw': yaw or 0.0,
        'frame_id': costmap.get('frame_id'),
    }


def _select_costmap_pose(capture: dict[str, Any], frame_id: str | None) -> tuple[list[float] | None, str | None]:
    by_frame = capture.get('robot_pose_by_frame') if isinstance(capture.get('robot_pose_by_frame'), dict) else {}
    if frame_id and frame_id in by_frame:
        pose = _pose(by_frame.get(frame_id))
        if pose is not None:
            return pose, frame_id
    for key in ('terminal_pose', 'robot_pose', 'odom_pose'):
        pose = _pose(capture.get(key))
        if pose is not None:
            return pose, key
    for key in ('odom', 'map'):
        pose = _pose(by_frame.get(key))
        if pose is not None:
            return pose, key
    return None, None


def _body_coords(wx: float, wy: float, pose: list[float]) -> tuple[float, float]:
    dx, dy = wx - pose[0], wy - pose[1]
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return c * dx + s * dy, -s * dx + c * dy


def _cell_world(ix: int, iy: int, geom: dict[str, Any]) -> tuple[float, float]:
    gx = (ix + 0.5) * geom['resolution']
    gy = (iy + 0.5) * geom['resolution']
    c, s = math.cos(geom['origin_yaw']), math.sin(geom['origin_yaw'])
    return geom['origin_x'] + c * gx - s * gy, geom['origin_y'] + s * gx + c * gy


def _scan_sector_stats(scan: dict[str, Any] | None, *, half_angle_deg: float) -> dict[str, Any]:
    if not isinstance(scan, dict):
        return {'available': False, 'reason': 'missing_raw_scan'}
    ranges = scan.get('ranges')
    if not isinstance(ranges, list) or not ranges:
        return {'available': False, 'reason': 'missing_scan_ranges'}
    angle_min = _num(scan.get('angle_min'))
    angle_increment = _num(scan.get('angle_increment'))
    range_min = _num(scan.get('range_min')) or 0.0
    range_max = _num(scan.get('range_max'))
    if angle_min is None or angle_increment is None or angle_increment <= 0:
        return {'available': False, 'reason': 'missing_scan_angle_geometry'}
    half = math.radians(half_angle_deg)
    selected: list[float] = []
    invalid = 0
    for index, raw in enumerate(ranges):
        angle = _angle_norm(angle_min + index * angle_increment)
        if abs(angle) > half:
            continue
        value = _num(raw)
        if value is None or value < range_min:
            invalid += 1
            continue
        if range_max is not None and value > range_max:
            value = range_max
        selected.append(value)
    return {
        'available': True,
        'half_angle_deg': half_angle_deg,
        'sample_count': len(selected) + invalid,
        'finite_count': len(selected),
        'invalid_count': invalid,
        'min_clearance_m': min(selected) if selected else None,
        'mean_clearance_m': (sum(selected) / len(selected)) if selected else None,
    }


def _corridor_cells(capture: dict[str, Any], *, half_width_m: float, length_m: float) -> dict[str, Any]:
    costmap = capture.get('local_costmap')
    if not isinstance(costmap, dict):
        return {'available': False, 'reason': 'missing_raw_local_costmap'}
    data = costmap.get('data')
    geom = _local_costmap_geometry(costmap)
    if not isinstance(data, list) or not data:
        return {'available': False, 'reason': 'missing_costmap_data'}
    if geom is None:
        return {'available': False, 'reason': 'missing_costmap_geometry'}
    width, height = geom['width'], geom['height']
    if len(data) < width * height:
        return {'available': False, 'reason': 'truncated_costmap_data'}
    pose, pose_source = _select_costmap_pose(capture, geom.get('frame_id'))
    if pose is None:
        return {'available': False, 'reason': 'missing_robot_pose_for_costmap_frame'}

    values: list[int] = []
    high_cells: list[dict[str, Any]] = []
    lethal_cells: list[dict[str, Any]] = []
    gradient_cells = 0
    for iy in range(height):
        for ix in range(width):
            wx, wy = _cell_world(ix, iy, geom)
            forward, lateral = _body_coords(wx, wy, pose)
            if 0.0 <= forward <= length_m and abs(lateral) <= half_width_m:
                value = int(data[iy * width + ix])
                values.append(value)
                cell = {
                    'ix': ix,
                    'iy': iy,
                    'value': value,
                    'forward_m': forward,
                    'lateral_m': lateral,
                    'world_x': wx,
                    'world_y': wy,
                }
                if 1 <= value < HIGH_COST_THRESHOLD:
                    gradient_cells += 1
                if value >= HIGH_COST_THRESHOLD:
                    high_cells.append(cell)
                if value >= LETHAL_COST_THRESHOLD:
                    lethal_cells.append(cell)
    if not values:
        return {
            'available': True,
            'sufficient': False,
            'reason': 'front_corridor_has_no_costmap_cells',
            'frame_id': geom.get('frame_id'),
            'pose_source': pose_source,
            'robot_pose_used': pose,
            'half_width_m': half_width_m,
            'length_m': length_m,
            'sample_count': 0,
        }
    total = len(values)
    unknown = sum(1 for v in values if v < 0)
    high = len(high_cells)
    lethal = len(lethal_cells)
    free = total - unknown - high
    laterals = [c['lateral_m'] for c in high_cells]
    forwards = [c['forward_m'] for c in high_cells]
    return {
        'available': True,
        'sufficient': True,
        'frame_id': geom.get('frame_id'),
        'pose_source': pose_source,
        'robot_pose_used': pose,
        'half_width_m': half_width_m,
        'length_m': length_m,
        'sample_count': total,
        'unknown_count': unknown,
        'free_count': free,
        'gradient_count': gradient_cells,
        'high_count': high,
        'lethal_count': lethal,
        'high_cost_ratio': high / total,
        'lethal_ratio': lethal / total,
        'unknown_ratio': unknown / total,
        'free_ratio': free / total,
        'gradient_ratio': gradient_cells / total,
        'max_cost': max(values),
        'mean_known_cost': (sum(v for v in values if v >= 0) / max(1, sum(1 for v in values if v >= 0))),
        'high_lateral_min_m': min(laterals) if laterals else None,
        'high_lateral_max_m': max(laterals) if laterals else None,
        'high_forward_min_m': min(forwards) if forwards else None,
        'high_forward_max_m': max(forwards) if forwards else None,
        'high_cells': high_cells[:25],
        'lethal_cells': lethal_cells[:25],
    }


def _footprint_extents(capture: dict[str, Any], pose: list[float] | None) -> dict[str, Any]:
    footprint = capture.get('footprint') if isinstance(capture.get('footprint'), dict) else {}
    points = footprint.get('points')
    if pose is None or not isinstance(points, list) or len(points) < 3:
        return {'available': False, 'reason': 'missing_footprint_or_pose'}
    body_points: list[tuple[float, float]] = []
    for point in points:
        if isinstance(point, (list, tuple)) and len(point) >= 2:
            x, y = _num(point[0]), _num(point[1])
            if x is not None and y is not None:
                body_points.append(_body_coords(x, y, pose))
    if len(body_points) < 3:
        return {'available': False, 'reason': 'insufficient_footprint_points'}
    f_vals = [p[0] for p in body_points]
    l_vals = [p[1] for p in body_points]
    return {
        'available': True,
        'point_count': len(body_points),
        'forward_min_m': min(f_vals),
        'forward_max_m': max(f_vals),
        'lateral_min_m': min(l_vals),
        'lateral_max_m': max(l_vals),
        'width_m': max(l_vals) - min(l_vals),
        'length_m': max(f_vals) - min(f_vals),
        'stamp_sec': _num(footprint.get('stamp_sec')),
        'frame_id': footprint.get('frame_id'),
    }


def _stamp_diagnostics(capture: dict[str, Any]) -> dict[str, Any]:
    stamps = {}
    for key in ('scan', 'local_costmap', 'footprint', 'odom'):
        item = capture.get(key)
        if isinstance(item, dict):
            value = _num(item.get('stamp_sec'))
            if value is not None:
                stamps[key] = value
    sensor_stamps = [stamps[k] for k in ('scan', 'footprint', 'odom') if k in stamps]
    cost_stamp = stamps.get('local_costmap')
    deltas = []
    if cost_stamp is not None:
        deltas = [abs(s - cost_stamp) for s in sensor_stamps]
    return {
        'stamps_sec': stamps,
        'max_sensor_costmap_stamp_delta_sec': max(deltas) if deltas else None,
        'costmap_stale_by_threshold': bool(deltas and max(deltas) > STALE_DELTA_SEC),
        'threshold_sec': STALE_DELTA_SEC,
    }


def _pose_projection_diagnostics(capture: dict[str, Any]) -> dict[str, Any]:
    by_frame = capture.get('robot_pose_by_frame') if isinstance(capture.get('robot_pose_by_frame'), dict) else {}
    map_pose = _pose(by_frame.get('map'))
    odom_pose = _pose(by_frame.get('odom'))
    transforms = capture.get('tf') or capture.get('transforms') or {}
    out = {
        'map_pose_available': map_pose is not None,
        'odom_pose_available': odom_pose is not None,
        'tf_transform_records_available': bool(transforms),
        'map_odom_xy_delta_m': None,
        'map_odom_yaw_delta_rad': None,
        'supported': False,
        'reason': None,
    }
    if map_pose and odom_pose:
        dx, dy = map_pose[0] - odom_pose[0], map_pose[1] - odom_pose[1]
        out['map_odom_xy_delta_m'] = math.hypot(dx, dy)
        out['map_odom_yaw_delta_rad'] = abs(_angle_norm(map_pose[2] - odom_pose[2]))
        if out['map_odom_xy_delta_m'] > 0.20 or out['map_odom_yaw_delta_rad'] > 0.20:
            out['supported'] = True
            out['reason'] = 'map_odom_pose_delta_large'
    if not map_pose or not odom_pose:
        out['reason'] = 'pose_pair_missing_for_projection_check'
    return out


def _scan_width_estimate(scan: dict[str, Any] | None, *, forward_distance_m: float) -> dict[str, Any]:
    """Estimate free lateral width at a forward slice using polar scan points.

    This is only a conservative diagnostic.  If no points fall near the requested
    slice, the result is an evidence gap for the narrow-corridor candidate, not a
    proof of clearance.
    """
    if not isinstance(scan, dict) or not isinstance(scan.get('ranges'), list):
        return {'available': False, 'reason': 'missing_raw_scan'}
    angle_min = _num(scan.get('angle_min'))
    inc = _num(scan.get('angle_increment'))
    rmin = _num(scan.get('range_min')) or 0.0
    rmax = _num(scan.get('range_max'))
    if angle_min is None or inc is None or inc <= 0:
        return {'available': False, 'reason': 'missing_scan_geometry'}
    lateral_hits: list[float] = []
    for i, raw in enumerate(scan['ranges']):
        r = _num(raw)
        if r is None or r < rmin:
            continue
        if rmax is not None and r > rmax:
            r = rmax
        angle = _angle_norm(angle_min + i * inc)
        x = r * math.cos(angle)
        y = r * math.sin(angle)
        if abs(x - forward_distance_m) <= 0.15:
            lateral_hits.append(y)
    if len(lateral_hits) < 2:
        return {'available': False, 'reason': 'insufficient_scan_points_for_width_slice', 'forward_distance_m': forward_distance_m}
    neg = [v for v in lateral_hits if v < 0]
    pos = [v for v in lateral_hits if v > 0]
    if not neg or not pos:
        return {'available': False, 'reason': 'scan_width_slice_missing_two_sides', 'forward_distance_m': forward_distance_m}
    width = min(pos) - max(neg)
    return {
        'available': True,
        'forward_distance_m': forward_distance_m,
        'estimated_free_width_m': width,
        'left_nearest_m': min(pos),
        'right_nearest_m': max(neg),
    }


def _candidate_checks(capture: dict[str, Any], phase81: dict[str, Any], corridor: dict[str, Any], footprint: dict[str, Any], stamps: dict[str, Any], pose_diag: dict[str, Any]) -> dict[str, Any]:
    scan = capture.get('scan') if isinstance(capture.get('scan'), dict) else None
    scan20 = _scan_sector_stats(scan, half_angle_deg=20.0)
    scan45 = _scan_sector_stats(scan, half_angle_deg=45.0)
    scan90 = _scan_sector_stats(scan, half_angle_deg=90.0)
    physical_open = (_num(scan20.get('min_clearance_m')) or 0.0) >= SCAN_OPEN_CLEARANCE_M
    local_blocked = bool(corridor.get('sufficient') and (corridor.get('high_cost_ratio', 0.0) >= 0.35 or corridor.get('lethal_ratio', 0.0) >= 0.15))
    high_one_sided = False
    if corridor.get('high_lateral_min_m') is not None and corridor.get('high_lateral_max_m') is not None:
        high_one_sided = bool(corridor['high_lateral_max_m'] < 0.05 or corridor['high_lateral_min_m'] > -0.05)
    gradient_present = bool(corridor.get('gradient_count', 0) > 0 and corridor.get('lethal_count', 0) > 0)

    footprint_width = _num(footprint.get('width_m'))
    corridor_width = 2.0 * (_num(corridor.get('half_width_m')) or DEFAULT_CORRIDOR_HALF_WIDTH_M)
    footprint_wider_than_corridor = bool(footprint_width is not None and footprint_width > corridor_width + 0.02)
    footprint_nearly_corridor = bool(footprint_width is not None and footprint_width >= corridor_width * 0.95)

    narrow_width = _scan_width_estimate(scan, forward_distance_m=0.75)
    real_narrow = bool(narrow_width.get('available') and (_num(narrow_width.get('estimated_free_width_m')) or 999.0) < NARROW_EXECUTION_WIDTH_M)
    scan_min45 = _num(scan45.get('min_clearance_m'))
    wide_scan_gets_close = bool(scan_min45 is not None and scan_min45 < SCAN_NARROW_CLEARANCE_M)

    sampling_wide = bool(
        local_blocked
        and physical_open
        and high_one_sided
        and not footprint_wider_than_corridor
        and (scan45.get('available') and (scan45.get('min_clearance_m') or 0.0) >= SCAN_NARROW_CLEARANCE_M)
        and corridor_width > (footprint_width or 0.0) + 0.20
    )

    stale_supported = bool(stamps.get('costmap_stale_by_threshold'))

    return {
        'inflation_spillover': {
            'supported': bool(physical_open and local_blocked and gradient_present and not stale_supported),
            'physical_scan_forward_open': physical_open,
            'local_cost_corridor_blocked': local_blocked,
            'gradient_cells_present': gradient_present,
            'gradient_count': corridor.get('gradient_count'),
            'lethal_count': corridor.get('lethal_count'),
            'high_cost_ratio': corridor.get('high_cost_ratio'),
            'lethal_ratio': corridor.get('lethal_ratio'),
            'one_sided_high_cost_band': high_one_sided,
            'note': 'Open forward scan plus smooth local-cost gradient/lethal band suggests cost inflation spilling into the execution corridor; this is suspected, not a parameter-change authorization.',
        },
        'footprint_or_wedge_projection': {
            'supported': bool(local_blocked and footprint_wider_than_corridor),
            'footprint_available': footprint.get('available', False),
            'footprint_width_m': footprint_width,
            'footprint_forward_extent_m': footprint.get('forward_max_m'),
            'sampled_corridor_width_m': corridor_width,
            'footprint_nearly_as_wide_as_corridor': footprint_nearly_corridor,
            'note': 'Supported only if the captured footprint/wedge envelope is wider than the sampled execution corridor or directly explains the high-cost region.',
        },
        'corridor_sampling_too_wide': {
            'supported': sampling_wide,
            'sampled_corridor_half_width_m': corridor.get('half_width_m'),
            'sampled_corridor_width_m': corridor_width,
            'footprint_width_m': footprint_width,
            'high_cost_lateral_min_m': corridor.get('high_lateral_min_m'),
            'high_cost_lateral_max_m': corridor.get('high_lateral_max_m'),
            'one_sided_high_cost_band': high_one_sided,
            'note': 'Supported only if blockage is mainly at the sampled lateral fringe while scan/footprint evidence indicates the executable center is wider than needed.',
        },
        'pose_tf_projection_mismatch': pose_diag,
        'real_execution_clearance_too_narrow': {
            'supported': bool(real_narrow or (wide_scan_gets_close and not physical_open)),
            'scan_width_estimate': narrow_width,
            'scan45_min_clearance_m': scan45.get('min_clearance_m'),
            'scan90_min_clearance_m': scan90.get('min_clearance_m'),
            'threshold_width_m': NARROW_EXECUTION_WIDTH_M,
            'note': 'Supported only by scan geometry showing less free width than the robot execution envelope; Phase81 forward sector alone is not enough.',
        },
        'costmap_stale_or_observation_mismatch': {
            'supported': stale_supported,
            'stamps_sec': stamps.get('stamps_sec'),
            'max_sensor_costmap_stamp_delta_sec': stamps.get('max_sensor_costmap_stamp_delta_sec'),
            'threshold_sec': STALE_DELTA_SEC,
            'note': 'Supported if local costmap timestamp is stale relative to scan/odom/footprint or required observation streams are inconsistent.',
        },
    }


def _required_gaps(capture: dict[str, Any], phase81: dict[str, Any]) -> list[str]:
    gaps: list[str] = []
    if not isinstance(capture.get('scan'), dict):
        gaps.append('missing_raw_scan')
    if not isinstance(capture.get('local_costmap'), dict):
        gaps.append('missing_raw_local_costmap')
    if not isinstance(capture.get('footprint'), dict):
        gaps.append('missing_footprint')
    if not isinstance(capture.get('robot_pose_by_frame'), dict) and _pose(capture.get('terminal_pose')) is None:
        gaps.append('missing_robot_pose')
    if not isinstance(phase81, dict) or not phase81:
        gaps.append('missing_phase81_analysis')
    return gaps


def _choose_classification(checks: dict[str, Any], gaps: list[str]) -> tuple[str, list[str]]:
    if gaps:
        return INSUFFICIENT, [f'evidence_gap: {gap}' for gap in gaps]
    # Stale/mismatched observations trump geometric interpretations because the
    # channels may not describe the same instant.
    if checks['costmap_stale_or_observation_mismatch']['supported']:
        return STALE, ['costmap timestamp is stale relative to scan/odom/footprint']
    if checks['pose_tf_projection_mismatch']['supported']:
        return POSE_TF, ['map/odom pose projection delta is large enough to explain the mismatch']
    if checks['real_execution_clearance_too_narrow']['supported']:
        return NARROW, ['scan geometry indicates the executable corridor may be narrower than the robot envelope']
    if checks['footprint_or_wedge_projection']['supported']:
        return FOOTPRINT, ['captured footprint/wedge projection is wider than the sampled execution corridor']
    if checks['corridor_sampling_too_wide']['supported']:
        return SAMPLING, ['blocked cells are mainly at the sampled corridor fringe; sampling width may be too broad']
    if checks['inflation_spillover']['supported']:
        return INFLATION, ['scan is forward-open while local-cost corridor has gradient/high/lethal band consistent with inflation spillover']
    return INSUFFICIENT, ['candidate checks did not produce enough discriminating evidence for a conservative root-cause label']


def analyze_phase82(raw_capture_path: str | Path, phase81_analysis_path: str | Path) -> dict[str, Any]:
    capture = _load_json(raw_capture_path)
    phase81 = _load_json(phase81_analysis_path)
    gaps = _required_gaps(capture, phase81)

    corridor = _corridor_cells(capture, half_width_m=DEFAULT_CORRIDOR_HALF_WIDTH_M, length_m=DEFAULT_CORRIDOR_LENGTH_M)
    if corridor.get('available') is False:
        gaps.append(corridor.get('reason') or 'local_cost_corridor_unavailable')
    elif corridor.get('sufficient') is False:
        gaps.append(corridor.get('reason') or 'local_cost_corridor_insufficient')
    pose = corridor.get('robot_pose_used') if isinstance(corridor.get('robot_pose_used'), list) else _pose(capture.get('terminal_pose'))
    footprint = _footprint_extents(capture, pose)
    stamps = _stamp_diagnostics(capture)
    pose_diag = _pose_projection_diagnostics(capture)
    checks = _candidate_checks(capture, phase81, corridor, footprint, stamps, pose_diag) if not gaps or capture else {
        'inflation_spillover': {'supported': False},
        'footprint_or_wedge_projection': {'supported': False},
        'corridor_sampling_too_wide': {'supported': False},
        'pose_tf_projection_mismatch': pose_diag,
        'real_execution_clearance_too_narrow': {'supported': False},
        'costmap_stale_or_observation_mismatch': {'supported': False},
    }
    classification, reasons = _choose_classification(checks, gaps)

    phase81_scan = phase81.get('front_corridor_scan') if isinstance(phase81.get('front_corridor_scan'), dict) else {}
    phase81_cost = phase81.get('front_corridor_local_cost') if isinstance(phase81.get('front_corridor_local_cost'), dict) else {}
    scan_min = _num(phase81_scan.get('min_clearance_m'))
    high_ratio = _num(phase81_cost.get('high_cost_ratio'))
    lethal_ratio = _num(phase81_cost.get('lethal_ratio'))
    physical_open = bool(scan_min is not None and scan_min >= SCAN_OPEN_CLEARANCE_M)
    local_blocked = bool((high_ratio is not None and high_ratio >= 0.35) or (lethal_ratio is not None and lethal_ratio >= 0.15))

    return {
        'run_id': RUN_ID,
        'source_raw_capture': str(raw_capture_path),
        'source_phase81_analysis': str(phase81_analysis_path),
        'source_phase81_forward_open_classification': phase81.get('forward_open_classification'),
        'root_cause_classification': classification,
        'allowed_root_cause_classifications': ALLOWED_ROOT_CAUSE_CLASSIFICATIONS,
        'classification_reasons': reasons,
        'evidence_sufficient': classification != INSUFFICIENT,
        'evidence_gaps': sorted(set(gaps)),
        'scan_local_cost_mismatch': {
            'physical_scan_forward_open': physical_open,
            'local_cost_corridor_blocked': local_blocked,
            'scan_min_clearance_m': scan_min,
            'scan_mean_clearance_m': phase81_scan.get('mean_clearance_m'),
            'local_high_cost_ratio': high_ratio,
            'local_lethal_ratio': lethal_ratio,
            'local_unknown_ratio': phase81_cost.get('unknown_ratio'),
        },
        'phase82_recomputed_corridor': {k: v for k, v in corridor.items() if k not in ('high_cells', 'lethal_cells')},
        'high_cost_cell_samples': corridor.get('high_cells', []),
        'lethal_cell_samples': corridor.get('lethal_cells', []),
        'footprint_projection': footprint,
        'timestamp_diagnostics': stamps,
        'candidate_checks': checks,
        'semantic_caveats': {
            'diagnostics_only': True,
            'no_fabricated_missing_evidence': True,
            'not_a_repair': True,
            'not_nav2_parameter_final_diagnosis': True,
        },
        'guardrails': GUARDRAILS,
        'autonomous_success_claimed': False,
        'exit_success_claimed': False,
        'phase83_entered': False,
    }


def write_minimal_summary(result: dict[str, Any], path: str | Path) -> None:
    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    mismatch = result.get('scan_local_cost_mismatch', {})
    inflation = result.get('candidate_checks', {}).get('inflation_spillover', {})
    stale = result.get('candidate_checks', {}).get('costmap_stale_or_observation_mismatch', {})
    pose = result.get('candidate_checks', {}).get('pose_tf_projection_mismatch', {})
    text = f"""# Phase82 minimal summary

Status: {result.get('root_cause_classification')}
Source Phase81 classification: {result.get('source_phase81_forward_open_classification')}
Source raw capture: {result.get('source_raw_capture')}

Mismatch evidence:
- physical scan forward open: {mismatch.get('physical_scan_forward_open')}
- scan min clearance m: {mismatch.get('scan_min_clearance_m')}
- local-cost corridor blocked: {mismatch.get('local_cost_corridor_blocked')}
- local high-cost ratio: {mismatch.get('local_high_cost_ratio')}
- local lethal ratio: {mismatch.get('local_lethal_ratio')}

Candidate diagnosis:
- inflation spillover supported: {inflation.get('supported')}
- costmap stale/mismatch supported: {stale.get('supported')}
- pose/TF projection mismatch supported: {pose.get('supported')}
- evidence gaps: {result.get('evidence_gaps')}
- classification reasons: {result.get('classification_reasons')}

Guardrails: No strategy/config tuning; no autonomous success claim; no exit success claim; Phase83 not entered.
"""
    p.write_text(text, encoding='utf-8')


def main() -> int:
    parser = argparse.ArgumentParser(description='Analyze Phase82 Goal2 local-cost / scan mismatch root cause.')
    parser.add_argument('--raw-capture', required=True)
    parser.add_argument('--phase81-analysis', required=True)
    parser.add_argument('--output-json', required=True)
    parser.add_argument('--minimal-summary', required=True)
    args = parser.parse_args()

    result = analyze_phase82(args.raw_capture, args.phase81_analysis)
    out = Path(args.output_json)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(result, indent=2, sort_keys=True), encoding='utf-8')
    write_minimal_summary(result, args.minimal_summary)
    print(f"root_cause_classification={result['root_cause_classification']}")
    print(f"evidence_sufficient={str(result['evidence_sufficient']).lower()}")
    print(f"evidence_gaps={result['evidence_gaps']}")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
