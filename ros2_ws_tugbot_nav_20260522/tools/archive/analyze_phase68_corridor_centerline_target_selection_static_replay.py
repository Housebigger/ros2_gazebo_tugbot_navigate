#!/usr/bin/env python3
"""Phase68 Corridor Centerline Target Selection Design / Static Replay.

static/log replay only.  This analyzer reuses accepted Phase66/Phase67
Goal 1 timeout evidence plus the active clean scaled2x SDF/metadata/Nav2
geometry config to ask a design-only question: if the Goal 1 target were moved
toward the local corridor centerline / maximum-clearance region, would static
local-cost proxy, footprint risk, and front-wedge risk decrease?

Guardrails: no runtime dispatch integration; no Nav2/MPPI/controller parameter
edits; no inflation/robot_radius/clearance_radius_m/map threshold tuning; no
branch scoring change; no corridor-following cmd_vel control; no fallback/
terminal acceptance change; no autonomous exploration success claim; no exit
success claim.
"""
from __future__ import annotations

import argparse
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

try:
    import yaml
except Exception:  # pragma: no cover
    yaml = None

RUN_ID = 'phase68_corridor_centerline_target_selection_static_replay'
PHASE = 'Phase68 Corridor Centerline Target Selection Design / Static Replay'
PHASE67_RUN_ID = 'phase67_goal1_timeout_visual_replay'
ACTIVE_WORLD = Path('src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean_scaled2x.sdf')
ACTIVE_METADATA = Path('src/tugbot_maze/config/maze_20260528_scaled_instance.yaml')
ACTIVE_NAV2_CONFIG = Path('src/tugbot_navigation/config/nav2_slam_params.yaml')
DEFAULT_PHASE67_ARTIFACT = Path('log') / PHASE67_RUN_ID / f'{PHASE67_RUN_ID}.json'
DEFAULT_OUTPUT = Path('log') / RUN_ID / f'{RUN_ID}.json'
ALLOWED_CLASSIFICATIONS = [
    'CENTERLINE_REPLAY_IMPROVES_LOCAL_COST',
    'CENTERLINE_REPLAY_NO_SAFE_CANDIDATE',
    'TARGET_ALREADY_CENTERED',
    'INSUFFICIENT_EVIDENCE',
]
GUARDRAILS = [
    'static/log replay only',
    'no runtime dispatch integration',
    'no Nav2/MPPI/controller parameter edits',
    'no inflation/robot_radius/clearance_radius_m/map threshold tuning',
    'no branch scoring change',
    'no corridor-following cmd_vel control',
    'no fallback/terminal acceptance change',
    'no autonomous exploration success claim',
    'no exit success claim',
]


def _load_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding='utf-8', errors='replace'))


def _write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding='utf-8')


def _round(value: float | None, digits: int = 6) -> float | None:
    if value is None or not math.isfinite(float(value)):
        return None
    return round(float(value), digits)


def _read_yaml(path: Path) -> dict[str, Any]:
    if yaml is None:
        raise RuntimeError('PyYAML is required for Phase68 static replay')
    data = yaml.safe_load(path.read_text(encoding='utf-8'))
    if not isinstance(data, dict):
        raise ValueError(f'YAML did not parse as a mapping: {path}')
    return data


def _nested(data: dict[str, Any], *keys: str) -> Any:
    cur: Any = data
    for key in keys:
        if not isinstance(cur, dict):
            return None
        cur = cur.get(key)
    return cur


def load_nav2_geometry(path: Path = ACTIVE_NAV2_CONFIG) -> dict[str, Any]:
    data = _read_yaml(path)
    local = _nested(data, 'local_costmap', 'local_costmap', 'ros__parameters') or {}
    inflation = local.get('inflation_layer') or {}
    return {
        'config_file': str(path),
        'robot_radius_m': float(local.get('robot_radius', 0.35)),
        'inflation_radius_m': float(inflation.get('inflation_radius', 0.70)),
        'cost_scaling_factor': float(inflation.get('cost_scaling_factor', 3.0)),
        'resolution_m': float(local.get('resolution', 0.05)),
    }


def map_world_offset(metadata_path: Path = ACTIVE_METADATA) -> tuple[float, float]:
    data = _read_yaml(metadata_path)
    world_entrance = _nested(data, 'world_frame_truth', 'entrance') or {}
    map_entrance = _nested(data, 'map_frame_truth', 'entrance') or {}
    return (
        float(world_entrance.get('x_m', 0.0)) - float(map_entrance.get('x_m', 0.0)),
        float(world_entrance.get('y_m', 0.0)) - float(map_entrance.get('y_m', 0.0)),
    )


def wall_rectangles(world_path: Path = ACTIVE_WORLD, metadata_path: Path = ACTIVE_METADATA) -> list[dict[str, Any]]:
    """Parse active SDF walls and return axis-aligned rectangles in map frame."""
    offset_x, offset_y = map_world_offset(metadata_path)
    root = ET.parse(world_path).getroot()
    rectangles: list[dict[str, Any]] = []
    for model in root.findall('.//world/model'):
        name = model.get('name') or ''
        if not name.startswith('maze_wall'):
            continue
        pose = (model.findtext('pose') or '0 0 0 0 0 0').split()
        x_world, y_world = float(pose[0]), float(pose[1])
        size_text = model.findtext('.//collision/geometry/box/size')
        if not size_text:
            continue
        sx, sy, _sz = [float(part) for part in size_text.split()]
        xmin_world = x_world - sx / 2.0
        xmax_world = x_world + sx / 2.0
        ymin_world = y_world - sy / 2.0
        ymax_world = y_world + sy / 2.0
        rectangles.append({
            'name': name,
            'xmin': xmin_world - offset_x,
            'xmax': xmax_world - offset_x,
            'ymin': ymin_world - offset_y,
            'ymax': ymax_world - offset_y,
            'sx': sx,
            'sy': sy,
        })
    if not rectangles:
        raise ValueError(f'No maze wall rectangles parsed from {world_path}')
    return rectangles


def point_in_rect(point: tuple[float, float], rect: dict[str, Any]) -> bool:
    x, y = point
    return float(rect['xmin']) <= x <= float(rect['xmax']) and float(rect['ymin']) <= y <= float(rect['ymax'])


def distance_to_rect(point: tuple[float, float], rect: dict[str, Any]) -> float:
    x, y = point
    dx = max(float(rect['xmin']) - x, x - float(rect['xmax']), 0.0)
    dy = max(float(rect['ymin']) - y, y - float(rect['ymax']), 0.0)
    return math.hypot(dx, dy)


def occupancy_at(point: tuple[float, float], walls: list[dict[str, Any]]) -> str:
    return 'occupied' if any(point_in_rect(point, wall) for wall in walls) else 'free'


def nearest_wall_distance(point: tuple[float, float], walls: list[dict[str, Any]]) -> float:
    if any(point_in_rect(point, wall) for wall in walls):
        return 0.0
    return min(distance_to_rect(point, wall) for wall in walls)


def _ray_rect_intersection(point: tuple[float, float], direction: tuple[float, float], rect: dict[str, Any]) -> float | None:
    x, y = point
    dx, dy = direction
    if point_in_rect(point, rect):
        return 0.0
    tmin = -math.inf
    tmax = math.inf
    for origin, delta, low, high in [
        (x, dx, float(rect['xmin']), float(rect['xmax'])),
        (y, dy, float(rect['ymin']), float(rect['ymax'])),
    ]:
        if abs(delta) < 1e-12:
            if origin < low or origin > high:
                return None
            continue
        t1 = (low - origin) / delta
        t2 = (high - origin) / delta
        near = min(t1, t2)
        far = max(t1, t2)
        tmin = max(tmin, near)
        tmax = min(tmax, far)
        if tmin > tmax:
            return None
    if tmax < 0.0:
        return None
    return max(tmin, 0.0)


def cast_wall_clearance(point: tuple[float, float], angle_rad: float, walls: list[dict[str, Any]], max_range_m: float = 6.0) -> float | None:
    direction = (math.cos(angle_rad), math.sin(angle_rad))
    hits = [
        t for wall in walls
        for t in [_ray_rect_intersection(point, direction, wall)]
        if t is not None and 0.0 <= t <= max_range_m
    ]
    return min(hits) if hits else None


def _static_inflation_cost(distance_m: float | None, robot_radius_m: float, inflation_radius_m: float, cost_scaling_factor: float) -> int | None:
    if distance_m is None:
        return None
    distance = max(0.0, float(distance_m))
    if distance <= robot_radius_m:
        return 100
    if distance > robot_radius_m + inflation_radius_m:
        return 0
    scaled = math.exp(-cost_scaling_factor * (distance - robot_radius_m))
    return max(1, min(99, int(round(99.0 * scaled))))


def _summary(values: list[int | None]) -> dict[str, Any]:
    clean = [int(v) for v in values if v is not None]
    if not clean:
        return {'sample_count': len(values), 'in_bounds_sample_count': 0, 'max': None, 'mean': None, 'high_cost_count': 0, 'lethal_count': 0}
    return {
        'sample_count': len(values),
        'in_bounds_sample_count': len(clean),
        'max': max(clean),
        'mean': sum(clean) / len(clean),
        'high_cost_count': sum(1 for v in clean if v >= 70),
        'lethal_count': sum(1 for v in clean if v >= 99),
    }


def _sample_disk(center: tuple[float, float], radius_m: float, step_m: float) -> list[tuple[float, float]]:
    samples: list[tuple[float, float]] = []
    cells = int(math.ceil(radius_m / max(step_m, 1e-6)))
    for ix in range(-cells, cells + 1):
        for iy in range(-cells, cells + 1):
            x = center[0] + ix * step_m
            y = center[1] + iy * step_m
            if math.hypot(x - center[0], y - center[1]) <= radius_m + 1e-9:
                samples.append((x, y))
    return samples


def _sample_front_wedge(pose: tuple[float, float, float], length_m: float, half_angle_rad: float, step_m: float) -> list[tuple[float, float]]:
    x, y, yaw = pose
    samples: list[tuple[float, float]] = []
    radial_steps = int(math.ceil(length_m / max(step_m, 1e-6)))
    angle_steps = max(3, int(math.ceil((2.0 * half_angle_rad) / 0.10)))
    for ri in range(1, radial_steps + 1):
        radius = ri * step_m
        if radius > length_m + 1e-9:
            continue
        for ai in range(angle_steps + 1):
            angle = yaw - half_angle_rad + (2.0 * half_angle_rad) * ai / max(angle_steps, 1)
            samples.append((x + math.cos(angle) * radius, y + math.sin(angle) * radius))
    return samples


def _cost_summary_for_points(points: list[tuple[float, float]], walls: list[dict[str, Any]], geometry: dict[str, Any]) -> dict[str, Any]:
    values = [
        _static_inflation_cost(
            nearest_wall_distance(point, walls),
            float(geometry['robot_radius_m']),
            float(geometry['inflation_radius_m']),
            float(geometry['cost_scaling_factor']),
        )
        for point in points
    ]
    return _summary(values)


def load_phase67_goal1_evidence(phase67_artifact: Path = DEFAULT_PHASE67_ARTIFACT) -> dict[str, Any]:
    data = _load_json(phase67_artifact)
    goal = data.get('goal1') or {}
    target = goal.get('target') or []
    dispatch_pose = goal.get('dispatch_pose') or []
    if len(target) < 2 or len(dispatch_pose) < 2:
        raise ValueError(f'Phase67 artifact lacks Goal 1 target/dispatch pose: {phase67_artifact}')
    open_direction = goal.get('dispatch_event', {}).get('branch_angle')
    if open_direction is None:
        open_direction = math.atan2(float(target[1]) - float(dispatch_pose[1]), float(target[0]) - float(dispatch_pose[0]))
    return {
        'phase67_artifact': str(phase67_artifact),
        'phase67_classification': data.get('classification'),
        'phase66_classification_preserved': data.get('phase66_classification_preserved'),
        'candidate_visual_hypotheses_for_human_check': data.get('candidate_visual_hypotheses_for_human_check'),
        'target': [float(target[0]), float(target[1])],
        'dispatch_pose': [float(dispatch_pose[0]), float(dispatch_pose[1]), float(dispatch_pose[2]) if len(dispatch_pose) > 2 else 0.0],
        'terminal_pose': goal.get('terminal_pose'),
        'open_direction_rad': float(open_direction),
        'phase67_timeout_local_cost': goal.get('local_cost') or {},
        'phase67_cmd_vel': goal.get('cmd_vel') or {},
        'phase67_nav2_feedback': goal.get('nav2_feedback') or {},
    }


def sample_centerline_candidates(
    evidence: dict[str, Any],
    forward_offsets_m: list[float] | None = None,
    lateral_offsets_m: list[float] | None = None,
) -> list[dict[str, Any]]:
    forward_offsets_m = forward_offsets_m if forward_offsets_m is not None else [-0.20, -0.10, 0.0, 0.10, 0.20]
    lateral_offsets_m = lateral_offsets_m if lateral_offsets_m is not None else [-0.60, -0.45, -0.30, -0.20, -0.10, 0.0, 0.10, 0.20, 0.30, 0.45, 0.60]
    target = evidence['target']
    dispatch = evidence['dispatch_pose']
    angle = float(evidence['open_direction_rad'])
    ux, uy = math.cos(angle), math.sin(angle)
    lx, ly = -math.sin(angle), math.cos(angle)
    candidates: list[dict[str, Any]] = []
    for forward_offset in forward_offsets_m:
        for lateral_offset in lateral_offsets_m:
            x = float(target[0]) + forward_offset * ux + lateral_offset * lx
            y = float(target[1]) + forward_offset * uy + lateral_offset * ly
            forward_progress = (x - float(dispatch[0])) * ux + (y - float(dispatch[1])) * uy
            candidates.append({
                'target': [x, y],
                'forward_offset_m': float(forward_offset),
                'lateral_offset_m': float(lateral_offset),
                'open_direction_rad': angle,
                'forward_progress_m': forward_progress,
                'projection_model': 'same_open_direction_lateral_centerline_static_replay',
            })
    return candidates


def evaluate_centerline_candidate(
    candidate: dict[str, Any],
    walls: list[dict[str, Any]],
    robot_radius_m: float = 0.35,
    inflation_radius_m: float = 0.70,
    cost_scaling_factor: float = 3.0,
    resolution_m: float = 0.05,
) -> dict[str, Any]:
    target = candidate['target']
    angle = float(candidate['open_direction_rad'])
    point = (float(target[0]), float(target[1]))
    side_max_range_m = 6.0
    left_raw = cast_wall_clearance(point, angle + math.pi / 2.0, walls, max_range_m=side_max_range_m)
    right_raw = cast_wall_clearance(point, angle - math.pi / 2.0, walls, max_range_m=side_max_range_m)
    # Keep left/right clearance numeric for centerline balance scoring.  If a
    # finite SDF wall segment is not hit within the bounded ray range, use the
    # max range as conservative "open/no hit" evidence and expose hit flags.
    left = left_raw if left_raw is not None else side_max_range_m
    right = right_raw if right_raw is not None else side_max_range_m
    nearest = nearest_wall_distance(point, walls)
    min_clearance = min(v for v in [left, right, nearest] if v is not None)
    balance_error = abs(left - right) if left is not None and right is not None else None
    geometry = {
        'robot_radius_m': robot_radius_m,
        'inflation_radius_m': inflation_radius_m,
        'cost_scaling_factor': cost_scaling_factor,
        'resolution_m': resolution_m,
    }
    static_local_cost = _static_inflation_cost(min_clearance, robot_radius_m, inflation_radius_m, cost_scaling_factor)
    footprint_points = _sample_disk(point, robot_radius_m, max(0.05, resolution_m))
    front_wedge_points = _sample_front_wedge((point[0], point[1], angle), 0.80, 0.45, max(0.05, resolution_m))
    footprint_cost_summary = _cost_summary_for_points(footprint_points, walls, geometry)
    front_wedge_cost_summary = _cost_summary_for_points(front_wedge_points, walls, geometry)
    occupancy = occupancy_at(point, walls)
    score = (
        (balance_error if balance_error is not None else 99.0)
        + 0.8 * (static_local_cost if static_local_cost is not None else 100.0) / 100.0
        + 0.015 * float(footprint_cost_summary['high_cost_count'])
        + 0.020 * float(front_wedge_cost_summary['high_cost_count'])
        + 2.0 * float(footprint_cost_summary['lethal_count'])
        + 1.5 * float(front_wedge_cost_summary['lethal_count'])
        + (10.0 if occupancy != 'free' else 0.0)
    )
    safe_by_static_replay = (
        occupancy == 'free'
        and min_clearance > robot_radius_m
        and footprint_cost_summary['lethal_count'] == 0
        and front_wedge_cost_summary['lethal_count'] == 0
    )
    return {
        **candidate,
        'left_wall_clearance_m': _round(left),
        'right_wall_clearance_m': _round(right),
        'left_wall_hit': left_raw is not None,
        'right_wall_hit': right_raw is not None,
        'side_clearance_no_hit_value_m': side_max_range_m,
        'balance_error_m': _round(balance_error),
        'min_clearance_m': _round(min_clearance),
        'nearest_wall_distance_m': _round(nearest),
        'occupancy': occupancy,
        'static_local_cost': static_local_cost,
        'footprint_cost_summary': footprint_cost_summary,
        'front_wedge_cost_summary': front_wedge_cost_summary,
        'forward_progress_m': _round(float(candidate.get('forward_progress_m', 0.0))),
        'safe_by_static_replay': safe_by_static_replay,
        'score': _round(score),
    }


def _is_better_than_original(candidate: dict[str, Any], original: dict[str, Any]) -> bool:
    """Return True when a centerline candidate improves the static replay risk.

    Phase68 must tolerate incomplete left/right ray coverage from finite SDF wall
    segments.  A candidate with finite balanced side clearances is considered a
    centerline improvement when it is static-safe and reduces at least one local
    risk metric while keeping footprint/front-wedge lethal counts no worse.
    """
    if candidate.get('safe_by_static_replay') is not True:
        return False
    if candidate.get('static_local_cost') is None or original.get('static_local_cost') is None:
        return False
    candidate_balance = candidate.get('balance_error_m')
    original_balance = original.get('balance_error_m')
    balance_improved = (
        candidate_balance is not None
        and (
            original_balance is None
            or float(candidate_balance) < float(original_balance) - 0.02
        )
    )
    static_cost_improved = int(candidate['static_local_cost']) < int(original['static_local_cost'])
    footprint_lethal_no_worse = candidate['footprint_cost_summary']['lethal_count'] <= original['footprint_cost_summary']['lethal_count']
    front_wedge_lethal_no_worse = candidate['front_wedge_cost_summary']['lethal_count'] <= original['front_wedge_cost_summary']['lethal_count']
    footprint_high_no_worse = candidate['footprint_cost_summary']['high_cost_count'] <= original['footprint_cost_summary']['high_cost_count']
    front_wedge_high_improved = candidate['front_wedge_cost_summary']['high_cost_count'] < original['front_wedge_cost_summary']['high_cost_count']
    return (
        footprint_lethal_no_worse
        and front_wedge_lethal_no_worse
        and footprint_high_no_worse
        and (balance_improved or static_cost_improved or front_wedge_high_improved)
    )


def classify_phase68(original: dict[str, Any], best: dict[str, Any] | None, candidates: list[dict[str, Any]]) -> str:
    if not candidates or best is None:
        return 'INSUFFICIENT_EVIDENCE'
    if original.get('balance_error_m') is not None and original['balance_error_m'] <= 0.05 and original.get('safe_by_static_replay'):
        return 'TARGET_ALREADY_CENTERED'
    if best.get('improves_over_original'):
        return 'CENTERLINE_REPLAY_IMPROVES_LOCAL_COST'
    if not any(row.get('safe_by_static_replay') for row in candidates):
        return 'CENTERLINE_REPLAY_NO_SAFE_CANDIDATE'
    if original.get('balance_error_m') is not None and original['balance_error_m'] <= 0.12:
        return 'TARGET_ALREADY_CENTERED'
    return 'CENTERLINE_REPLAY_NO_SAFE_CANDIDATE'


def analyze_phase68(
    phase67_artifact: Path = DEFAULT_PHASE67_ARTIFACT,
    output: Path = DEFAULT_OUTPUT,
    world_path: Path = ACTIVE_WORLD,
    metadata_path: Path = ACTIVE_METADATA,
    nav2_config_path: Path = ACTIVE_NAV2_CONFIG,
) -> dict[str, Any]:
    evidence = load_phase67_goal1_evidence(phase67_artifact)
    walls = wall_rectangles(world_path, metadata_path)
    geometry = load_nav2_geometry(nav2_config_path)
    original_candidate = {
        'target': evidence['target'],
        'forward_offset_m': 0.0,
        'lateral_offset_m': 0.0,
        'open_direction_rad': evidence['open_direction_rad'],
        'forward_progress_m': math.hypot(evidence['target'][0] - evidence['dispatch_pose'][0], evidence['target'][1] - evidence['dispatch_pose'][1]),
        'projection_model': 'original_goal1_target',
    }
    eval_geometry = {
        'robot_radius_m': geometry['robot_radius_m'],
        'inflation_radius_m': geometry['inflation_radius_m'],
        'cost_scaling_factor': geometry['cost_scaling_factor'],
        'resolution_m': geometry['resolution_m'],
    }
    original = evaluate_centerline_candidate(original_candidate, walls, **eval_geometry)
    candidates = [
        evaluate_centerline_candidate(row, walls, **eval_geometry)
        for row in sample_centerline_candidates(evidence)
    ]
    for row in candidates:
        row['improves_over_original'] = _is_better_than_original(row, original)
    safe = [row for row in candidates if row.get('safe_by_static_replay')]
    best_pool = safe if safe else candidates
    best = min(best_pool, key=lambda row: (row.get('score') if row.get('score') is not None else 9999.0, abs(float(row.get('forward_offset_m', 0.0))))) if best_pool else None
    if best is not None:
        best['improves_over_original'] = _is_better_than_original(best, original)
    classification = classify_phase68(original, best, candidates)
    payload = {
        'run_id': RUN_ID,
        'phase': PHASE,
        'classification': classification,
        'allowed_classifications': ALLOWED_CLASSIFICATIONS,
        'guardrails': GUARDRAILS,
        'active_world': str(world_path),
        'active_metadata': str(metadata_path),
        'active_nav2_config': str(nav2_config_path),
        'phase67_artifact': str(phase67_artifact),
        'phase67_classification_preserved': evidence.get('phase67_classification'),
        'phase66_classification_preserved': evidence.get('phase66_classification_preserved'),
        'phase67_human_observation_input': 'robot drifted off corridor centerline toward one wall; static replay tests centerline / max-clearance target design only',
        'runtime_dispatch_changed': False,
        'runtime_connected': False,
        'branch_scoring_changed': False,
        'cmd_vel_control_added': False,
        'complete_autonomous_success_claimed': False,
        'exit_success_claimed': False,
        'nav2_geometry': geometry,
        'original_goal1_target': {
            **original,
            'phase67_timeout_local_cost_reference': evidence.get('phase67_timeout_local_cost'),
        },
        'best_centerline_candidate': best,
        'centerline_candidates': candidates,
        'comparison': {
            'original_balance_error_m': original.get('balance_error_m'),
            'best_balance_error_m': best.get('balance_error_m') if best else None,
            'balance_error_delta_m': _round((best.get('balance_error_m') - original.get('balance_error_m')) if best and best.get('balance_error_m') is not None and original.get('balance_error_m') is not None else None),
            'original_static_local_cost': original.get('static_local_cost'),
            'best_static_local_cost': best.get('static_local_cost') if best else None,
            'original_footprint_lethal_count': original['footprint_cost_summary']['lethal_count'],
            'best_footprint_lethal_count': best['footprint_cost_summary']['lethal_count'] if best else None,
            'original_front_wedge_high_cost_count': original['front_wedge_cost_summary']['high_cost_count'],
            'best_front_wedge_high_cost_count': best['front_wedge_cost_summary']['high_cost_count'] if best else None,
            'candidate_count': len(candidates),
            'safe_candidate_count': len(safe),
        },
        'interpretation_limits': [
            'SDF wall + Nav2 inflation proxy is a design/static replay, not runtime local_costmap proof',
            'No runtime dispatch was modified or enabled for centerline projection',
            'A later phase would need explicit implementation and bounded runtime validation before any behavior claim',
        ],
    }
    _write_json(output, payload)
    return payload


def main() -> None:
    parser = argparse.ArgumentParser(description=PHASE)
    parser.add_argument('--phase67-artifact', type=Path, default=DEFAULT_PHASE67_ARTIFACT)
    parser.add_argument('--output', type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument('--world', type=Path, default=ACTIVE_WORLD)
    parser.add_argument('--metadata', type=Path, default=ACTIVE_METADATA)
    parser.add_argument('--nav2-config', type=Path, default=ACTIVE_NAV2_CONFIG)
    args = parser.parse_args()
    result = analyze_phase68(args.phase67_artifact, args.output, args.world, args.metadata, args.nav2_config)
    print(json.dumps({'run_id': RUN_ID, 'classification': result['classification'], 'output': str(args.output)}, sort_keys=True))


if __name__ == '__main__':
    main()
