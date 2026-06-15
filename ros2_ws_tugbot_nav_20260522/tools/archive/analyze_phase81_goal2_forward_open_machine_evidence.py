#!/usr/bin/env python3
"""Phase81 Goal2 forward-open machine evidence analyzer.

Diagnostics-only: classify whether raw /scan and raw /local_costmap/costmap
support a forward-open corridor at the held Goal2 timeout scene.  This analyzer
must not tune Nav2 or alter maze_explorer behavior.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

RUN_ID = 'phase81_goal2_forward_open_machine_evidence_capture'
CONFIRMED = 'FORWARD_OPEN_CORRIDOR_CONFIRMED'
BLOCKED = 'FORWARD_OPEN_CORRIDOR_BLOCKED'
INSUFFICIENT = 'FORWARD_OPEN_EVIDENCE_INSUFFICIENT'

DEFAULT_CORRIDOR_LENGTH_M = 1.0
DEFAULT_CORRIDOR_HALF_WIDTH_M = 0.30
DEFAULT_SCAN_SECTOR_HALF_ANGLE_DEG = 20.0
DEFAULT_MIN_OPEN_CLEARANCE_M = 0.80
DEFAULT_BLOCKED_CLEARANCE_M = 0.50
DEFAULT_MAX_HIGH_COST_RATIO_FOR_OPEN = 0.10
DEFAULT_MAX_LETHAL_RATIO_FOR_OPEN = 0.02
DEFAULT_BLOCKED_HIGH_COST_RATIO = 0.35
DEFAULT_BLOCKED_LETHAL_RATIO = 0.15
HIGH_COST_THRESHOLD = 70
LETHAL_COST_THRESHOLD = 99

GUARDRAILS = [
    'diagnostics-only evidence capture',
    'No maze_explorer strategy changed',
    'No branch scoring changed',
    'No centerline gate changed',
    'No directional readiness override changed',
    'No fallback/terminal acceptance changed',
    'No Nav2/MPPI/controller tuning',
    'No inflation/robot_radius/clearance_radius_m/map threshold tuning',
    'No autonomous exploration success claimed',
    'No exit success claimed',
    'Phase82 not entered',
]


def _number(value: Any) -> float | None:
    try:
        out = float(value)
    except (TypeError, ValueError):
        return None
    return out if math.isfinite(out) else None


def _pose(value: Any) -> list[float] | None:
    if isinstance(value, (list, tuple)) and len(value) >= 3:
        x = _number(value[0])
        y = _number(value[1])
        yaw = _number(value[2])
        if x is not None and y is not None and yaw is not None:
            return [x, y, yaw]
    if isinstance(value, dict):
        x = _number(value.get('x'))
        y = _number(value.get('y'))
        yaw = _number(value.get('yaw'))
        if x is not None and y is not None and yaw is not None:
            return [x, y, yaw]
    return None


def _angle_norm(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def _load_capture(path: str | Path) -> dict[str, Any]:
    p = Path(path)
    if not p.exists() or not p.stat().st_size:
        return {}
    return json.loads(p.read_text(encoding='utf-8', errors='replace'))


def analyze_scan(scan: dict[str, Any] | None, *, sector_half_angle_deg: float = DEFAULT_SCAN_SECTOR_HALF_ANGLE_DEG) -> dict[str, Any]:
    if not isinstance(scan, dict):
        return {'available': False, 'reason': 'missing_raw_scan'}
    ranges = scan.get('ranges')
    if not isinstance(ranges, list) or not ranges:
        return {'available': False, 'reason': 'missing_scan_ranges'}
    angle_min = _number(scan.get('angle_min'))
    angle_increment = _number(scan.get('angle_increment'))
    range_min = _number(scan.get('range_min')) or 0.0
    range_max = _number(scan.get('range_max'))
    if angle_min is None or angle_increment is None or angle_increment <= 0:
        return {'available': False, 'reason': 'missing_scan_angle_geometry'}

    half = math.radians(sector_half_angle_deg)
    selected: list[float] = []
    invalid_count = 0
    for index, raw in enumerate(ranges):
        angle = _angle_norm(angle_min + index * angle_increment)
        if abs(angle) > half:
            continue
        value = _number(raw)
        if value is None:
            invalid_count += 1
            continue
        if value < range_min:
            invalid_count += 1
            continue
        if range_max is not None and value > range_max:
            value = range_max
        selected.append(value)

    finite_count = len(selected)
    min_clearance = min(selected) if selected else None
    mean_clearance = sum(selected) / finite_count if finite_count else None
    return {
        'available': True,
        'frame_id': scan.get('frame_id'),
        'sector_half_angle_deg': sector_half_angle_deg,
        'range_count_total': len(ranges),
        'sector_sample_count': finite_count + invalid_count,
        'finite_sector_sample_count': finite_count,
        'invalid_sector_sample_count': invalid_count,
        'min_clearance_m': min_clearance,
        'mean_clearance_m': mean_clearance,
        'range_min_m': range_min,
        'range_max_m': range_max,
    }


def _costmap_origin(costmap: dict[str, Any]) -> tuple[float, float, float] | None:
    info = costmap.get('info') if isinstance(costmap.get('info'), dict) else {}
    origin = info.get('origin') if isinstance(info.get('origin'), dict) else {}
    pos = origin.get('position') if isinstance(origin.get('position'), dict) else {}
    ox = _number(pos.get('x'))
    oy = _number(pos.get('y'))
    yaw = _number(origin.get('orientation_yaw'))
    if yaw is None:
        ori = origin.get('orientation') if isinstance(origin.get('orientation'), dict) else {}
        x = _number(ori.get('x')) or 0.0
        y = _number(ori.get('y')) or 0.0
        z = _number(ori.get('z')) or 0.0
        w = _number(ori.get('w'))
        if w is not None:
            yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    if ox is None or oy is None:
        return None
    return (ox, oy, yaw or 0.0)


def _select_robot_pose_for_costmap(capture: dict[str, Any], frame_id: str | None) -> tuple[list[float] | None, str | None]:
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


def analyze_local_costmap(
    capture: dict[str, Any],
    *,
    corridor_length_m: float = DEFAULT_CORRIDOR_LENGTH_M,
    corridor_half_width_m: float = DEFAULT_CORRIDOR_HALF_WIDTH_M,
) -> dict[str, Any]:
    costmap = capture.get('local_costmap')
    if not isinstance(costmap, dict):
        return {'available': False, 'reason': 'missing_raw_local_costmap'}
    info = costmap.get('info') if isinstance(costmap.get('info'), dict) else {}
    data = costmap.get('data')
    width = int(info.get('width') or 0)
    height = int(info.get('height') or 0)
    resolution = _number(info.get('resolution'))
    frame_id = costmap.get('frame_id')
    origin = _costmap_origin(costmap)
    if not isinstance(data, list) or not data:
        return {'available': False, 'reason': 'missing_costmap_data'}
    if not width or not height or resolution is None or resolution <= 0 or origin is None:
        return {'available': False, 'reason': 'missing_costmap_geometry'}
    if len(data) < width * height:
        return {'available': False, 'reason': 'truncated_costmap_data'}
    robot_pose, pose_source = _select_robot_pose_for_costmap(capture, frame_id)
    if robot_pose is None:
        return {'available': False, 'reason': 'missing_robot_pose_for_costmap_frame'}

    ox, oy, origin_yaw = origin
    cos_o = math.cos(origin_yaw)
    sin_o = math.sin(origin_yaw)
    cos_r = math.cos(robot_pose[2])
    sin_r = math.sin(robot_pose[2])

    sampled: list[int] = []
    in_bounds = 0
    out_of_bounds = 0
    for iy in range(height):
        for ix in range(width):
            gx = (ix + 0.5) * resolution
            gy = (iy + 0.5) * resolution
            wx = ox + cos_o * gx - sin_o * gy
            wy = oy + sin_o * gx + cos_o * gy
            dx = wx - robot_pose[0]
            dy = wy - robot_pose[1]
            forward = cos_r * dx + sin_r * dy
            lateral = -sin_r * dx + cos_r * dy
            if 0.0 <= forward <= corridor_length_m and abs(lateral) <= corridor_half_width_m:
                in_bounds += 1
                sampled.append(int(data[iy * width + ix]))

    # The rectangular corridor is in world coordinates. If the local costmap does
    # not cover it, record an evidence gap instead of guessing.
    if not sampled:
        return {
            'available': True,
            'sufficient': False,
            'reason': 'front_corridor_has_no_costmap_cells',
            'frame_id': frame_id,
            'pose_source': pose_source,
            'sample_count': 0,
            'out_of_bounds_count': out_of_bounds,
            'corridor_length_m': corridor_length_m,
            'corridor_half_width_m': corridor_half_width_m,
        }

    total = len(sampled)
    unknown = sum(1 for v in sampled if v < 0)
    lethal = sum(1 for v in sampled if v >= LETHAL_COST_THRESHOLD)
    high = sum(1 for v in sampled if v >= HIGH_COST_THRESHOLD)
    occupied = sum(1 for v in sampled if v >= HIGH_COST_THRESHOLD)
    free = total - unknown - occupied
    max_cost = max(sampled)
    known_values = [v for v in sampled if v >= 0]
    mean_cost = sum(known_values) / len(known_values) if known_values else None
    return {
        'available': True,
        'sufficient': True,
        'frame_id': frame_id,
        'pose_source': pose_source,
        'robot_pose_used': robot_pose,
        'corridor_length_m': corridor_length_m,
        'corridor_half_width_m': corridor_half_width_m,
        'sample_count': total,
        'unknown_count': unknown,
        'free_count': free,
        'occupied_or_high_count': occupied,
        'lethal_count': lethal,
        'unknown_ratio': unknown / total,
        'free_ratio': free / total,
        'high_cost_ratio': high / total,
        'lethal_ratio': lethal / total,
        'max_cost': max_cost,
        'mean_known_cost': mean_cost,
    }


def classify(scan_summary: dict[str, Any], cost_summary: dict[str, Any]) -> tuple[str, list[str], list[str]]:
    gaps: list[str] = []
    reasons: list[str] = []

    if not scan_summary.get('available'):
        gaps.append(scan_summary.get('reason') or 'missing_raw_scan')
    elif not scan_summary.get('finite_sector_sample_count'):
        gaps.append('no_finite_scan_samples_in_front_sector')

    if not cost_summary.get('available'):
        gaps.append(cost_summary.get('reason') or 'missing_raw_local_costmap')
    elif not cost_summary.get('sufficient', True):
        gaps.append(cost_summary.get('reason') or 'insufficient_local_costmap_corridor_coverage')

    if gaps:
        return INSUFFICIENT, gaps, ['Required raw scan/local-costmap/pose evidence is incomplete; no forward-open claim fabricated.']

    min_clearance = _number(scan_summary.get('min_clearance_m'))
    high_ratio = _number(cost_summary.get('high_cost_ratio'))
    lethal_ratio = _number(cost_summary.get('lethal_ratio'))
    unknown_ratio = _number(cost_summary.get('unknown_ratio'))

    if min_clearance is not None and min_clearance < DEFAULT_BLOCKED_CLEARANCE_M:
        reasons.append(f'front_scan_min_clearance_below_blocked_threshold: {min_clearance:.3f}m')
    if lethal_ratio is not None and lethal_ratio >= DEFAULT_BLOCKED_LETHAL_RATIO:
        reasons.append(f'front_corridor_lethal_ratio_high: {lethal_ratio:.3f}')
    if high_ratio is not None and high_ratio >= DEFAULT_BLOCKED_HIGH_COST_RATIO:
        reasons.append(f'front_corridor_high_cost_ratio_high: {high_ratio:.3f}')
    if reasons:
        return BLOCKED, [], reasons

    if (
        min_clearance is not None and min_clearance >= DEFAULT_MIN_OPEN_CLEARANCE_M
        and high_ratio is not None and high_ratio <= DEFAULT_MAX_HIGH_COST_RATIO_FOR_OPEN
        and lethal_ratio is not None and lethal_ratio <= DEFAULT_MAX_LETHAL_RATIO_FOR_OPEN
        and (unknown_ratio is None or unknown_ratio <= 0.10)
    ):
        return CONFIRMED, [], [
            f'front_scan_min_clearance_m={min_clearance:.3f} >= {DEFAULT_MIN_OPEN_CLEARANCE_M:.3f}',
            f'front_corridor_high_cost_ratio={high_ratio:.3f} <= {DEFAULT_MAX_HIGH_COST_RATIO_FOR_OPEN:.3f}',
            f'front_corridor_lethal_ratio={lethal_ratio:.3f} <= {DEFAULT_MAX_LETHAL_RATIO_FOR_OPEN:.3f}',
        ]

    return INSUFFICIENT, [], [
        'Raw evidence exists, but scan clearance / local-cost distribution does not meet confirmed-open or blocked gates.',
        f'min_clearance_m={min_clearance}',
        f'high_cost_ratio={high_ratio}',
        f'lethal_ratio={lethal_ratio}',
        f'unknown_ratio={unknown_ratio}',
    ]


def analyze_capture(capture_path: str | Path) -> dict[str, Any]:
    capture_path = Path(capture_path)
    capture = _load_capture(capture_path)
    scan_summary = analyze_scan(capture.get('scan'))
    cost_summary = analyze_local_costmap(capture)
    classification, gaps, reasons = classify(scan_summary, cost_summary)

    return {
        'run_id': RUN_ID,
        'source_capture': str(capture_path),
        'source_phase80_classification': capture.get('source_phase80_classification'),
        'source_phase80_forward_open_status': capture.get('source_phase80_forward_open_status'),
        'goal_sequence': capture.get('goal_sequence', 2),
        'target': capture.get('target'),
        'terminal_pose': capture.get('terminal_pose') or capture.get('robot_pose'),
        'raw_capture': {
            'scan_available': bool(isinstance(capture.get('scan'), dict)),
            'local_costmap_available': bool(isinstance(capture.get('local_costmap'), dict)),
            'footprint_available': bool(isinstance(capture.get('footprint'), dict)),
            'odom_available': bool(isinstance(capture.get('odom'), dict)),
            'tf_available': bool(isinstance(capture.get('tf'), dict)),
        },
        'front_corridor_definition': {
            'scan_sector_half_angle_deg': DEFAULT_SCAN_SECTOR_HALF_ANGLE_DEG,
            'corridor_length_m': DEFAULT_CORRIDOR_LENGTH_M,
            'corridor_half_width_m': DEFAULT_CORRIDOR_HALF_WIDTH_M,
            'high_cost_threshold': HIGH_COST_THRESHOLD,
            'lethal_cost_threshold': LETHAL_COST_THRESHOLD,
            'confirmed_min_clearance_m': DEFAULT_MIN_OPEN_CLEARANCE_M,
            'blocked_clearance_m': DEFAULT_BLOCKED_CLEARANCE_M,
        },
        'front_corridor_scan': scan_summary,
        'front_corridor_local_cost': cost_summary,
        'forward_open_classification': classification,
        'evidence_sufficient': classification != INSUFFICIENT,
        'evidence_gaps': gaps,
        'classification_reasons': reasons,
        'semantic_caveats': [
            'classification is evidence-only and does not repair navigation',
            'classification is not exit success',
            'classification is not autonomous exploration success',
            'classification is not fallback/terminal acceptance',
            'classification is not a Nav2 parameter final diagnosis',
        ],
        'guardrails': GUARDRAILS,
        'complete_autonomous_success_claimed': False,
        'exit_success_claimed': False,
        'phase82_entered': False,
    }


def write_minimal_summary(result: dict[str, Any], output: str | Path) -> None:
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    scan = result.get('front_corridor_scan', {})
    cost = result.get('front_corridor_local_cost', {})
    lines = [
        '# Phase81 minimal field summary',
        '',
        f"Status: {result.get('forward_open_classification')}",
        'Trigger: Goal2 timeout / near-goal lateral residual / Phase80 forward-open evidence insufficient',
        f"Run/artifacts: {output.parent}",
        f"Source capture: {result.get('source_capture')}",
        f"Terminal pose: {result.get('terminal_pose')}",
        f"Target: {result.get('target')}",
        '',
        'Machine evidence:',
        f"- scan min front clearance m: {scan.get('min_clearance_m')}",
        f"- scan finite front-sector samples: {scan.get('finite_sector_sample_count')}",
        f"- local-cost front corridor samples: {cost.get('sample_count')}",
        f"- local-cost high ratio: {cost.get('high_cost_ratio')}",
        f"- local-cost lethal ratio: {cost.get('lethal_ratio')}",
        f"- local-cost unknown ratio: {cost.get('unknown_ratio')}",
        f"- evidence gaps: {result.get('evidence_gaps')}",
        f"- classification reasons: {result.get('classification_reasons')}",
        '',
        'Guardrails: No strategy/config tuning; no autonomous success claim; no exit success claim; Phase82 not entered.',
    ]
    output.write_text('\n'.join(lines) + '\n', encoding='utf-8')


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--capture-json', type=Path, required=True)
    parser.add_argument('--output-json', type=Path, default=Path('log/phase81_goal2_forward_open_machine_evidence_capture/phase81_goal2_forward_open_machine_evidence_capture_analysis.json'))
    parser.add_argument('--minimal-summary', type=Path, default=Path('log/phase81_goal2_forward_open_machine_evidence_capture/phase81_goal2_forward_open_machine_evidence_capture_minimal_field_summary.md'))
    args = parser.parse_args()

    result = analyze_capture(args.capture_json)
    args.output_json.parent.mkdir(parents=True, exist_ok=True)
    args.output_json.write_text(json.dumps(result, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    write_minimal_summary(result, args.minimal_summary)
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
