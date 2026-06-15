#!/usr/bin/env python3
"""Phase64 Corridor Width / Robot Footprint Feasibility Decision.

Static geometry/config/log replay only.  This script reads the accepted active
scaled2x world, active Nav2 geometry configuration, Tugbot base mesh, and
Phase62/Phase63 replay artifacts.  It does not modify runtime dispatch, Nav2
parameters, branch selection, entrance strategy, fallback, or terminal behavior.
"""

from __future__ import annotations

import argparse
import json
import math
import re
import struct
import xml.etree.ElementTree as ET
from itertools import combinations
from pathlib import Path
from typing import Any

try:
    import yaml
except Exception:  # pragma: no cover - PyYAML is expected in this workspace
    yaml = None

RUN_ID = 'phase64_corridor_width_robot_footprint_feasibility_decision'
PHASE = 'Phase64 Corridor Width / Robot Footprint Feasibility Decision'
ALLOWED_CLASSIFICATIONS = [
    'GEOMETRY_FEASIBILITY_BLOCKED',
    'GEOMETRY_FEASIBILITY_MARGINAL',
    'GEOMETRY_FEASIBILITY_OK',
    'INSUFFICIENT_EVIDENCE',
]
GUARDRAILS = [
    'static geometry/config/log replay only',
    'no runtime dispatch integration',
    'no Nav2/MPPI/controller parameter edits',
    'no clearance_radius_m tuning',
    'no map sufficiency threshold tuning',
    'no branch selection scoring change',
    'no entrance/fallback/terminal acceptance change',
    'no autonomous exploration success claim',
    'first dispatch is not exit success',
    'no long run',
]

ACTIVE_WORLD = Path('src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean_scaled2x.sdf')
ACTIVE_METADATA = Path('src/tugbot_maze/config/maze_20260528_scaled_instance.yaml')
ACTIVE_NAV2_CONFIG = Path('src/tugbot_navigation/config/nav2_slam_params.yaml')
TUGBOT_BASE_STL = Path('src/tugbot_description/models/tugbot/meshes/base/tugbot_simp.stl')
DEFAULT_PHASE62_ARTIFACT = Path(
    'log/phase62_first_dispatch_local_cost_traversability_diagnostics/'
    'phase62_first_dispatch_local_cost_traversability_diagnostics.json'
)
DEFAULT_PHASE63_ARTIFACT = Path(
    'log/phase63_first_dispatch_target_safety_projection_static_replay/'
    'phase63_first_dispatch_target_safety_projection_static_replay.json'
)
DEFAULT_ARTIFACT_DIR = Path('log') / RUN_ID
DEFAULT_REPORT = Path('doc/doc_report/phase64_corridor_width_robot_footprint_feasibility_decision_report.md')


def _round(value: float | None, digits: int = 6) -> float | None:
    if value is None:
        return None
    return round(float(value), digits)


def _read_yaml(path: Path) -> dict[str, Any]:
    if yaml is None:
        raise RuntimeError('PyYAML is required for Phase64 analyzer')
    data = yaml.safe_load(path.read_text())
    if not isinstance(data, dict):
        raise ValueError(f'YAML did not parse as mapping: {path}')
    return data


def _nested(data: dict[str, Any], *keys: str) -> Any:
    cur: Any = data
    for key in keys:
        if not isinstance(cur, dict):
            return None
        cur = cur.get(key)
    return cur


def load_nav2_geometry_config(path: Path) -> dict[str, Any]:
    data = _read_yaml(path)
    local_params = _nested(data, 'local_costmap', 'local_costmap', 'ros__parameters') or {}
    global_params = _nested(data, 'global_costmap', 'global_costmap', 'ros__parameters') or {}

    def costmap_block(params: dict[str, Any]) -> dict[str, Any]:
        inflation = params.get('inflation_layer') or {}
        return {
            'robot_radius_m': float(params.get('robot_radius')) if params.get('robot_radius') is not None else None,
            'resolution_m': float(params.get('resolution')) if params.get('resolution') is not None else None,
            'width_m': float(params.get('width')) if params.get('width') is not None else None,
            'height_m': float(params.get('height')) if params.get('height') is not None else None,
            'inflation_radius_m': float(inflation.get('inflation_radius')) if inflation.get('inflation_radius') is not None else None,
            'cost_scaling_factor': float(inflation.get('cost_scaling_factor')) if inflation.get('cost_scaling_factor') is not None else None,
            'plugins': list(params.get('plugins') or []),
        }

    return {
        'config_file': str(path),
        'local_costmap': costmap_block(local_params),
        'global_costmap': costmap_block(global_params),
    }


def stl_xy_turning_diameter(path: Path) -> float:
    data = path.read_bytes()
    vertices: list[tuple[float, float, float]] = []
    if len(data) >= 84:
        tri_count = struct.unpack('<I', data[80:84])[0]
        if 84 + tri_count * 50 == len(data):
            offset = 84
            for _ in range(tri_count):
                values = struct.unpack('<12fH', data[offset:offset + 50])
                vertices.extend([values[3:6], values[6:9], values[9:12]])
                offset += 50
    if not vertices:
        text = data.decode('utf-8', errors='ignore')
        for match in re.finditer(r'vertex\s+([-+\deE.]+)\s+([-+\deE.]+)\s+([-+\deE.]+)', text):
            vertices.append(tuple(map(float, match.groups())))
    if not vertices:
        raise ValueError(f'No STL vertices parsed from {path}')
    return 2.0 * max(math.hypot(x, y) for x, y, _z in vertices)


def compute_required_widths(config: dict[str, Any], mesh_turning_diameter_m: float) -> dict[str, Any]:
    local = config['local_costmap']
    robot_radius = float(local['robot_radius_m'])
    inflation_radius = float(local['inflation_radius_m'])
    mesh_radius = mesh_turning_diameter_m / 2.0
    robot_radius_diameter = 2.0 * robot_radius
    robot_body_width = max(robot_radius_diameter, mesh_turning_diameter_m)
    return {
        'robot_radius_m': _round(robot_radius),
        'robot_radius_diameter_m': _round(robot_radius_diameter, 2),
        'mesh_radius_m': _round(mesh_radius),
        'mesh_turning_outer_diameter_m': _round(mesh_turning_diameter_m),
        'robot_body_width_required_m': _round(robot_body_width),
        'comfort_contract_width_m': _round(2.0 * mesh_turning_diameter_m),
        'local_inflation_radius_m': _round(inflation_radius),
        'inflation_full_envelope_width_m': _round(2.0 * (robot_radius + inflation_radius), 2),
        'mesh_inflation_full_envelope_width_m': _round(2.0 * (mesh_radius + inflation_radius)),
    }


def wall_rectangles(world_path: Path) -> list[dict[str, Any]]:
    root = ET.parse(world_path).getroot()
    rectangles: list[dict[str, Any]] = []
    for model in root.findall('.//world/model'):
        name = model.get('name') or ''
        if not name.startswith('maze_wall'):
            continue
        pose = (model.findtext('pose') or '0 0 0 0 0 0').split()
        x, y = float(pose[0]), float(pose[1])
        size_text = model.findtext('.//collision/geometry/box/size')
        if size_text is None:
            raise ValueError(f'{name}: missing collision box size')
        sx, sy, sz = [float(part) for part in size_text.split()]
        rectangles.append({
            'name': name,
            'x': x,
            'y': y,
            'sx': sx,
            'sy': sy,
            'sz': sz,
            'xmin': x - sx / 2.0,
            'xmax': x + sx / 2.0,
            'ymin': y - sy / 2.0,
            'ymax': y + sy / 2.0,
        })
    if len(rectangles) < 2:
        raise ValueError(f'Expected at least 2 maze wall rectangles in {world_path}')
    return rectangles


def rectangle_gap(first: dict[str, Any], second: dict[str, Any]) -> float:
    dx = max(second['xmin'] - first['xmax'], first['xmin'] - second['xmax'], 0.0)
    dy = max(second['ymin'] - first['ymax'], first['ymin'] - second['ymax'], 0.0)
    return math.hypot(dx, dy)


def point_to_rect_distance(point: tuple[float, float], rect: dict[str, Any]) -> float:
    px, py = point
    dx = max(rect['xmin'] - px, px - rect['xmax'], 0.0)
    dy = max(rect['ymin'] - py, py - rect['ymax'], 0.0)
    return math.hypot(dx, dy)


def analyze_world_corridors(world_path: Path) -> dict[str, Any]:
    rects = wall_rectangles(world_path)
    gaps: list[tuple[float, str, str]] = []
    for first, second in combinations(rects, 2):
        gap = rectangle_gap(first, second)
        if gap > 1e-6:
            gaps.append((gap, first['name'], second['name']))
    gaps.sort(key=lambda item: item[0])
    if not gaps:
        raise ValueError('No positive wall-to-wall gaps found')
    return {
        'world_file': str(world_path),
        'wall_count': len(rects),
        'global_min_positive_wall_gap_m': _round(gaps[0][0]),
        'global_min_positive_wall_gap_pair': [gaps[0][1], gaps[0][2]],
        'smallest_positive_wall_gaps': [
            {'gap_m': _round(gap), 'wall_a': first, 'wall_b': second}
            for gap, first, second in gaps[:12]
        ],
    }


def _first_dispatch_event(phase62: dict[str, Any]) -> dict[str, Any]:
    for replay in phase62.get('replays', []):
        event = replay.get('first_dispatch_event')
        if isinstance(event, dict):
            return event
    nav2_results = _nested(phase62, 'metrics', 'nav2_result_summary') or []
    if nav2_results and isinstance(nav2_results[0], dict):
        return nav2_results[0]
    raise ValueError('Phase62 artifact does not contain first dispatch event')


def _phase62_footprint_summary(phase62: dict[str, Any], event: dict[str, Any]) -> dict[str, Any]:
    footprint_cost = event.get('phase62_target_footprint_cost')
    if isinstance(footprint_cost, dict) and isinstance(footprint_cost.get('summary'), dict):
        return footprint_cost['summary']
    if isinstance(event.get('phase62_target_footprint_cost_summary'), dict):
        return event['phase62_target_footprint_cost_summary']
    summaries = _nested(phase62, 'metrics', 'target_footprint_cost_summary') or []
    if summaries and isinstance(summaries[0], dict):
        summary = summaries[0].get('summary') if isinstance(summaries[0].get('summary'), dict) else summaries[0]
        return summary
    raw_max = event.get('phase62_target_footprint_cost_max')
    if isinstance(raw_max, dict):
        raw_max = raw_max.get('max')
    return {
        'max': raw_max,
        'lethal_count': event.get('phase62_target_footprint_lethal_count'),
        'high_cost_count': event.get('phase62_target_footprint_high_cost_count'),
    }


def first_dispatch_corridor_evidence(
    phase62: dict[str, Any],
    phase63: dict[str, Any],
    active_metadata: dict[str, Any],
    world_rects: list[dict[str, Any]],
) -> dict[str, Any]:
    event = _first_dispatch_event(phase62)
    footprint = _phase62_footprint_summary(phase62, event)
    raw_local_cell = event.get('phase62_local_cell_state') or {}
    if isinstance(raw_local_cell, dict):
        local_cell = dict(raw_local_cell)
    else:
        local_cell = {'state': raw_local_cell} if raw_local_cell is not None else {}
    target_map = event.get('target') or event.get('goal_pose') or []
    dispatch_pose_map = event.get('dispatch_pose') or []
    entrance = _nested(active_metadata, 'world_frame_truth', 'entrance') or {}
    entrance_x = float(entrance.get('x_m', 0.0))
    entrance_y = float(entrance.get('y_m', 0.0))
    target_world = None
    nearest_sdf_wall_distance = None
    nearest_sdf_wall = None
    if isinstance(target_map, list) and len(target_map) >= 2:
        target_world = [float(target_map[0]) + entrance_x, float(target_map[1]) + entrance_y]
        nearest = sorted(
            (point_to_rect_distance((target_world[0], target_world[1]), rect), rect['name'])
            for rect in world_rects
        )[0]
        nearest_sdf_wall_distance = nearest[0]
        nearest_sdf_wall = nearest[1]

    target_clearance = event.get('target_clearance_m')
    path_clearance = event.get('path_corridor_min_clearance_m')
    effective_width = None
    if path_clearance is not None:
        effective_width = 2.0 * float(path_clearance)

    radius_max = None
    if isinstance(local_cell, dict):
        radius_max = local_cell.get('max_radius_cost')
    if radius_max is None:
        radius_max = event.get('dispatch_target_local_cost_max_radius')
    if radius_max is None:
        radius_max = event.get('phase62_local_radius_cost_max')
    if not local_cell:
        local_cell = {
            'state': event.get('phase62_local_cell_state') or event.get('dispatch_target_local_cost_state'),
            'value': event.get('dispatch_target_local_cost'),
            'max_radius_cost': radius_max,
        }

    return {
        'phase61_classification_preserved': phase62.get('phase61_classification_preserved') or 'SINGLE_OPEN_EXCEPTION_APPLIED_AND_DISPATCHED',
        'phase62_classification_preserved': phase62.get('classification'),
        'phase62_secondary_factor_preserved': phase62.get('secondary_factor') or 'LOCAL_COSTMAP_INFLATION_DOMINANT',
        'phase63_classification_preserved': phase63.get('classification'),
        'target_map': target_map,
        'dispatch_pose_map': dispatch_pose_map,
        'target_world_from_metadata_offset': target_world,
        'nearest_sdf_wall_distance_m': _round(nearest_sdf_wall_distance),
        'nearest_sdf_wall': nearest_sdf_wall,
        'target_clearance_m': _round(float(target_clearance)) if target_clearance is not None else None,
        'path_corridor_min_clearance_m': _round(float(path_clearance)) if path_clearance is not None else None,
        'first_dispatch_effective_width_from_replay_clearance_m': _round(effective_width),
        'target_crosses_narrow_passage': bool(event.get('target_crosses_narrow_passage')),
        'branch_angle_rad': event.get('branch_angle'),
        'local_topology': event.get('local_topology'),
        'selected_due_to_context': event.get('selected_due_to_context'),
        'candidate_branch_count': event.get('candidate_branch_count'),
        'local_radius_max_cost': radius_max,
        'local_cell_state': local_cell.get('state') if isinstance(local_cell, dict) else None,
        'footprint_max_cost': footprint.get('max'),
        'footprint_lethal_count': footprint.get('lethal_count'),
        'footprint_high_cost_count': footprint.get('high_cost_count'),
        'phase63_safe_projection_found': phase63.get('safe_projection_found'),
        'phase63_best_projection_safe': (phase63.get('best_projection') or {}).get('safe_projection'),
        'phase63_candidate_count': len(phase63.get('projection_candidates') or []),
    }


def classify_geometry(widths: dict[str, Any], evidence: dict[str, Any], world_geometry: dict[str, Any]) -> tuple[str, list[str]]:
    reasons: list[str] = []
    required_inflation = float(widths['inflation_full_envelope_width_m'])
    required_body = float(widths['robot_body_width_required_m'])
    required_comfort = float(widths['comfort_contract_width_m'])
    global_gap = float(world_geometry['global_min_positive_wall_gap_m'])
    path_clearance = evidence.get('path_corridor_min_clearance_m')
    effective_width = evidence.get('first_dispatch_effective_width_from_replay_clearance_m')
    footprint_max = evidence.get('footprint_max_cost')
    radius_max = evidence.get('local_radius_max_cost')
    no_projection = evidence.get('phase63_classification_preserved') == 'NO_SAFE_PROJECTION_IN_CORRIDOR'

    if path_clearance is None or effective_width is None or footprint_max is None or radius_max is None:
        return 'INSUFFICIENT_EVIDENCE', ['missing first-dispatch replay geometry/cost evidence']

    if global_gap < required_body:
        reasons.append('active_world_global_min_gap_below_robot_body_width')
    if global_gap < required_comfort:
        reasons.append('active_world_global_min_gap_below_comfort_contract')
    if global_gap < required_inflation:
        reasons.append('active_world_global_min_gap_below_local_inflation_full_envelope')
    if float(effective_width) < required_body:
        reasons.append('first_dispatch_effective_width_below_robot_body_width')
    if float(effective_width) < required_inflation:
        reasons.append('first_dispatch_effective_width_below_local_inflation_full_envelope')
    if float(path_clearance) - float(widths['mesh_radius_m']) < 0.05:
        reasons.append('first_dispatch_mesh_clearance_margin_under_5cm')
    if int(footprint_max) >= 99 or int(radius_max) >= 99:
        reasons.append('first_dispatch_local_cost_or_footprint_hits_lethal_radius')
    if no_projection:
        reasons.append('phase63_no_safe_projection_in_corridor')

    blocked_markers = {
        'first_dispatch_effective_width_below_robot_body_width',
        'first_dispatch_effective_width_below_local_inflation_full_envelope',
        'first_dispatch_local_cost_or_footprint_hits_lethal_radius',
        'phase63_no_safe_projection_in_corridor',
    }
    if blocked_markers.issubset(set(reasons)) or (
        'first_dispatch_effective_width_below_local_inflation_full_envelope' in reasons
        and 'first_dispatch_local_cost_or_footprint_hits_lethal_radius' in reasons
        and no_projection
    ):
        return 'GEOMETRY_FEASIBILITY_BLOCKED', reasons
    if reasons:
        return 'GEOMETRY_FEASIBILITY_MARGINAL', reasons
    return 'GEOMETRY_FEASIBILITY_OK', ['world/body/inflation margins satisfy static checks']


def analyze_phase64(root: Path, phase62_artifact: Path, phase63_artifact: Path) -> dict[str, Any]:
    root = root.resolve()
    active_metadata_path = root / ACTIVE_METADATA
    active_world_path = root / ACTIVE_WORLD
    nav2_config_path = root / ACTIVE_NAV2_CONFIG
    stl_path = root / TUGBOT_BASE_STL
    phase62_path = phase62_artifact if phase62_artifact.is_absolute() else root / phase62_artifact
    phase63_path = phase63_artifact if phase63_artifact.is_absolute() else root / phase63_artifact

    active_metadata = _read_yaml(active_metadata_path)
    nav2_config = load_nav2_geometry_config(nav2_config_path)
    mesh_diameter = stl_xy_turning_diameter(stl_path)
    required_widths = compute_required_widths(nav2_config, mesh_diameter)
    world_geometry = analyze_world_corridors(active_world_path)
    rects = wall_rectangles(active_world_path)
    phase62 = json.loads(phase62_path.read_text())
    phase63 = json.loads(phase63_path.read_text())
    evidence = first_dispatch_corridor_evidence(phase62, phase63, active_metadata, rects)

    classification, reasons = classify_geometry(required_widths, evidence, world_geometry)
    margins = {
        'active_world_min_gap_minus_robot_body_width_m': _round(
            float(world_geometry['global_min_positive_wall_gap_m']) - float(required_widths['robot_body_width_required_m'])
        ),
        'active_world_min_gap_minus_comfort_contract_m': _round(
            float(world_geometry['global_min_positive_wall_gap_m']) - float(required_widths['comfort_contract_width_m'])
        ),
        'active_world_min_gap_minus_inflation_full_envelope_m': _round(
            float(world_geometry['global_min_positive_wall_gap_m']) - float(required_widths['inflation_full_envelope_width_m'])
        ),
        'first_dispatch_clearance_minus_robot_radius_m': _round(
            float(evidence['path_corridor_min_clearance_m']) - float(required_widths['robot_radius_m'])
        ) if evidence.get('path_corridor_min_clearance_m') is not None else None,
        'first_dispatch_clearance_minus_mesh_radius_m': _round(
            float(evidence['path_corridor_min_clearance_m']) - float(required_widths['mesh_radius_m'])
        ) if evidence.get('path_corridor_min_clearance_m') is not None else None,
        'first_dispatch_effective_width_minus_robot_body_width_m': _round(
            float(evidence['first_dispatch_effective_width_from_replay_clearance_m']) - float(required_widths['robot_body_width_required_m'])
        ) if evidence.get('first_dispatch_effective_width_from_replay_clearance_m') is not None else None,
        'first_dispatch_effective_width_minus_inflation_full_envelope_m': _round(
            float(evidence['first_dispatch_effective_width_from_replay_clearance_m']) - float(required_widths['inflation_full_envelope_width_m'])
        ) if evidence.get('first_dispatch_effective_width_from_replay_clearance_m') is not None else None,
    }

    return {
        'phase': PHASE,
        'run_id': RUN_ID,
        'allowed_classifications': ALLOWED_CLASSIFICATIONS,
        'classification': classification,
        'classification_reasons': reasons,
        'artifact_dir': str(DEFAULT_ARTIFACT_DIR),
        'active_world': str(ACTIVE_WORLD),
        'active_metadata': str(ACTIVE_METADATA),
        'nav2_geometry_config': nav2_config,
        'robot_geometry': {
            'base_mesh': str(TUGBOT_BASE_STL),
            'mesh_turning_outer_diameter_m': _round(mesh_diameter),
            'mesh_radius_m': _round(mesh_diameter / 2.0),
        },
        'required_widths': required_widths,
        'world_corridor_geometry': world_geometry,
        'first_dispatch_corridor_evidence': evidence,
        'feasibility_margins': margins,
        'geometry_mismatch_escalated': classification in {'GEOMETRY_FEASIBILITY_BLOCKED', 'GEOMETRY_FEASIBILITY_MARGINAL'},
        'runtime_dispatch_changed': False,
        'complete_autonomous_success_claimed': False,
        'first_dispatch_is_not_exit_success': True,
        'guardrails': GUARDRAILS,
        'phase61_classification_preserved': 'SINGLE_OPEN_EXCEPTION_APPLIED_AND_DISPATCHED',
        'phase62_classification_preserved': 'CORRIDOR_TOO_NARROW',
        'phase62_secondary_factor_preserved': 'LOCAL_COSTMAP_INFLATION_DOMINANT',
        'phase63_classification_preserved': 'NO_SAFE_PROJECTION_IN_CORRIDOR',
    }


def render_markdown(result: dict[str, Any]) -> str:
    e = result['first_dispatch_corridor_evidence']
    w = result['required_widths']
    m = result['feasibility_margins']
    world = result['world_corridor_geometry']
    lines = [
        '# Phase64 Corridor Width / Robot Footprint Feasibility Decision Report',
        '',
        f"Run id: `{RUN_ID}`",
        f"Artifact dir: `{DEFAULT_ARTIFACT_DIR}/`",
        'Status: complete; stop for human acceptance. Do not enter Phase65. 不进入 Phase65.',
        '',
        '## Preserved conclusions',
        '',
        '- Phase61: `SINGLE_OPEN_EXCEPTION_APPLIED_AND_DISPATCHED`.',
        '- Phase62: `CORRIDOR_TOO_NARROW`; secondary `LOCAL_COSTMAP_INFLATION_DOMINANT`.',
        '- Phase63: `NO_SAFE_PROJECTION_IN_CORRIDOR`.',
        '- First dispatch is not autonomous exploration success and not exit success.',
        '',
        '## Guardrails',
        '',
    ]
    lines.extend(f'- {g}.' for g in result['guardrails'])
    lines.extend([
        '',
        '## Classification',
        '',
        f"`{result['classification']}`",
        '',
        'Reasons:',
        '',
    ])
    lines.extend(f"- `{reason}`" for reason in result['classification_reasons'])
    lines.extend([
        '',
        'Interpretation: Phase64 elevates the issue from exploration-candidate repair to a world/robot scale/footprint/inflation geometry mismatch decision. Runtime dispatch was not changed.',
        '',
        '## Active geometry/config evidence',
        '',
        f"- active_world: `{result['active_world']}`",
        f"- active_metadata: `{result['active_metadata']}`",
        f"- Nav2 config: `{result['nav2_geometry_config']['config_file']}`",
        f"- local robot_radius_m: `{result['nav2_geometry_config']['local_costmap']['robot_radius_m']}`",
        f"- local inflation_radius_m: `{result['nav2_geometry_config']['local_costmap']['inflation_radius_m']}`",
        f"- local cost_scaling_factor: `{result['nav2_geometry_config']['local_costmap']['cost_scaling_factor']}`",
        f"- Tugbot mesh turning_outer_diameter_m: `{result['robot_geometry']['mesh_turning_outer_diameter_m']}`",
        '',
        'Required/envelope widths:',
        '',
        f"- robot_body_width_required_m: `{w['robot_body_width_required_m']}`",
        f"- comfort_contract_width_m: `{w['comfort_contract_width_m']}`",
        f"- inflation_full_envelope_width_m: `{w['inflation_full_envelope_width_m']}`",
        f"- mesh_inflation_full_envelope_width_m: `{w['mesh_inflation_full_envelope_width_m']}`",
        '',
        'World SDF wall-gap evidence:',
        '',
        f"- wall_count: `{world['wall_count']}`",
        f"- global_min_positive_wall_gap_m: `{world['global_min_positive_wall_gap_m']}`",
        f"- global_min_positive_wall_gap_pair: `{world['global_min_positive_wall_gap_pair']}`",
        '',
        '## First-dispatch corridor replay evidence',
        '',
        f"- target_map: `{e['target_map']}`",
        f"- target_world_from_metadata_offset: `{e['target_world_from_metadata_offset']}`",
        f"- target_clearance_m: `{e['target_clearance_m']}`",
        f"- path_corridor_min_clearance_m: `{e['path_corridor_min_clearance_m']}`",
        f"- first_dispatch_effective_width_from_replay_clearance_m: `{e['first_dispatch_effective_width_from_replay_clearance_m']}`",
        f"- nearest_sdf_wall_distance_m: `{e['nearest_sdf_wall_distance_m']}` (`{e['nearest_sdf_wall']}`)",
        f"- target_crosses_narrow_passage: `{e['target_crosses_narrow_passage']}`",
        f"- local_radius_max_cost: `{e['local_radius_max_cost']}`",
        f"- local_cell_state: `{e['local_cell_state']}`",
        f"- footprint_max_cost: `{e['footprint_max_cost']}`",
        f"- footprint_lethal_count: `{e['footprint_lethal_count']}`",
        f"- phase63_safe_projection_found: `{e['phase63_safe_projection_found']}`",
        '',
        'Margins:',
        '',
    ])
    lines.extend(f"- {key}: `{value}`" for key, value in m.items())
    lines.extend([
        '',
        '## Validation placeholders',
        '',
        '- `python3 -m py_compile tools/analyze_phase64_corridor_width_robot_footprint_feasibility_decision.py`: run in final validation.',
        '- `pytest -q src/tugbot_maze/test/test_phase64_corridor_width_robot_footprint_feasibility_decision.py`: run in final validation.',
        '- `colcon build --packages-select tugbot_maze tugbot_bringup --symlink-install`: run in final validation.',
        '- `git diff -- src/tugbot_navigation/config | wc -c`: must remain `0`.',
        '- cleanup check: must remain empty.',
        '',
        '## Stop condition',
        '',
        'Phase64 complete. Stop for human acceptance. Do not enter Phase65. 不进入 Phase65.',
        '',
    ])
    return '\n'.join(lines)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--root', type=Path, default=Path.cwd())
    parser.add_argument('--phase62-artifact', type=Path, default=DEFAULT_PHASE62_ARTIFACT)
    parser.add_argument('--phase63-artifact', type=Path, default=DEFAULT_PHASE63_ARTIFACT)
    parser.add_argument('--artifact-dir', type=Path, default=DEFAULT_ARTIFACT_DIR)
    parser.add_argument('--report', type=Path, default=DEFAULT_REPORT)
    args = parser.parse_args()

    result = analyze_phase64(args.root, args.phase62_artifact, args.phase63_artifact)
    artifact_dir = args.artifact_dir if args.artifact_dir.is_absolute() else args.root / args.artifact_dir
    artifact_dir.mkdir(parents=True, exist_ok=True)
    json_path = artifact_dir / f'{RUN_ID}.json'
    md_path = artifact_dir / f'{RUN_ID}.md'
    markdown = render_markdown(result)
    json_path.write_text(json.dumps(result, indent=2, sort_keys=True) + '\n')
    md_path.write_text(markdown)
    report_path = args.report if args.report.is_absolute() else args.root / args.report
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(markdown)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
