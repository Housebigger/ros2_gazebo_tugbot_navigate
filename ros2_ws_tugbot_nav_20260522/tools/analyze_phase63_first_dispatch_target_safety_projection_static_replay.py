#!/usr/bin/env python3
"""Phase63 First Dispatch Target Safety Projection Design / Static Replay.

Static/log replay only. This analyzer reuses accepted Phase62 artifacts to design
a narrow target safety projection scheme. It does not integrate with runtime
dispatch, send ROS goals, tune Nav2/MPPI/controller parameters, tune
clearance_radius_m or map thresholds, change branch selection scoring, entrance
strategy, fallback, or terminal acceptance. First dispatch is not autonomous
exploration success and not exit success.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

PHASE = 'Phase63 First Dispatch Target Safety Projection Design / Static Replay'
RUN_ID = 'phase63_first_dispatch_target_safety_projection_static_replay'
PHASE62_RUN_ID = 'phase62_first_dispatch_local_cost_traversability_diagnostics'
PHASE62_CLASSIFICATION = 'CORRIDOR_TOO_NARROW'
PHASE62_SECONDARY_FACTOR = 'LOCAL_COSTMAP_INFLATION_DOMINANT'
PHASE61_CLASSIFICATION = 'SINGLE_OPEN_EXCEPTION_APPLIED_AND_DISPATCHED'
ACTIVE_WORLD = 'tugbot_maze_world_20260528_clean_scaled2x.sdf'
ACTIVE_METADATA = 'maze_20260528_scaled_instance.yaml'
HIGH_COST_THRESHOLD = 70
LETHAL_COST_THRESHOLD = 99
SAFE_RADIUS_MAX_COST = 70
SAFE_FOOTPRINT_MAX_COST = 90
SAFE_FOOTPRINT_LETHAL_COUNT = 0
SAFE_FRONT_WEDGE_MAX_COST = 90
ALLOWED_CLASSIFICATIONS = {
    'SAFE_PROJECTION_FOUND',
    'NO_SAFE_PROJECTION_IN_CORRIDOR',
    'INSUFFICIENT_REPLAY_EVIDENCE',
}
GUARDRAILS = [
    'static/log replay only',
    'no runtime dispatch integration',
    'no Nav2/MPPI/controller parameter edits',
    'no clearance_radius_m tuning',
    'no map sufficiency threshold tuning',
    'no branch selection scoring change',
    'no entrance/fallback/terminal acceptance change',
    'no autonomous exploration success claim',
    'first dispatch is not exit success',
]


def _load_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding='utf-8'))


def _write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding='utf-8')


def _number(value: Any) -> float | None:
    try:
        out = float(value)
    except (TypeError, ValueError):
        return None
    return out if math.isfinite(out) else None


def _summary(values: list[int | None]) -> dict[str, Any]:
    clean = [int(v) for v in values if v is not None]
    if not clean:
        return {'sample_count': len(values), 'in_bounds_sample_count': 0, 'max': None, 'mean': None, 'high_cost_count': 0, 'lethal_count': 0}
    return {
        'sample_count': len(values),
        'in_bounds_sample_count': len(clean),
        'max': max(clean),
        'mean': float(sum(clean) / len(clean)),
        'high_cost_count': sum(1 for v in clean if v >= HIGH_COST_THRESHOLD),
        'lethal_count': sum(1 for v in clean if v >= LETHAL_COST_THRESHOLD),
    }


def _patch_value_grid(patch: dict[str, Any]) -> dict[tuple[int, int], int | None]:
    rows = patch.get('rows') if isinstance(patch, dict) else None
    if not isinstance(rows, list):
        return {}
    center = patch.get('center_cell') if isinstance(patch.get('center_cell'), list) else None
    radius = int(patch.get('radius_cells', 0) or 0)
    center_x = int(center[0]) if center and len(center) >= 2 else None
    grid: dict[tuple[int, int], int | None] = {}
    for row in rows:
        if not isinstance(row, dict):
            continue
        cell_y = row.get('cell_y')
        values = row.get('values')
        if cell_y is None or not isinstance(values, list):
            continue
        if 'cell_x_start' in row:
            start_x = int(row['cell_x_start'])
        elif center_x is not None:
            start_x = center_x - radius
        else:
            start_x = 0
        for offset, value in enumerate(values):
            grid[(start_x + offset, int(cell_y))] = int(value) if value is not None else None
    return grid


class ReplayCostPatch:
    """Small local replay grid reconstructed from Phase62 local-cost patches."""

    def __init__(self, grid: dict[tuple[int, int], int | None], center_cell: tuple[int, int], center_xy: tuple[float, float], resolution: float = 0.05) -> None:
        self.grid = grid
        self.center_cell = center_cell
        self.center_xy = center_xy
        self.resolution = resolution

    @classmethod
    def from_phase62(cls, dispatch: dict[str, Any], replay: dict[str, Any]) -> 'ReplayCostPatch | None':
        patches: list[dict[str, Any]] = []
        for source in [
            dispatch.get('phase62_local_costmap_patch'),
            replay.get('local_costmap_patch_summary'),
            (replay.get('local_costmap_target_evidence') or {}).get('latest', {}).get('patch') if isinstance(replay.get('local_costmap_target_evidence'), dict) else None,
        ]:
            if isinstance(source, dict):
                patches.append(source)
        merged: dict[tuple[int, int], int | None] = {}
        center_cell: tuple[int, int] | None = None
        for patch in patches:
            center = patch.get('center_cell')
            if center_cell is None and isinstance(center, list) and len(center) >= 2:
                center_cell = (int(center[0]), int(center[1]))
            merged.update(_patch_value_grid(patch))
        target = dispatch.get('target')
        if center_cell is None or not isinstance(target, list) or len(target) < 2 or not merged:
            return None
        return cls(merged, center_cell, (float(target[0]), float(target[1])))

    def world_to_cell(self, xy: tuple[float, float]) -> tuple[int, int]:
        dx = int(round((xy[0] - self.center_xy[0]) / self.resolution))
        dy = int(round((xy[1] - self.center_xy[1]) / self.resolution))
        return (self.center_cell[0] + dx, self.center_cell[1] + dy)

    def value_at(self, xy: tuple[float, float]) -> int | None:
        return self.grid.get(self.world_to_cell(xy))

    def radius_values(self, xy: tuple[float, float], radius_m: float = 0.30) -> list[int | None]:
        cx, cy = self.world_to_cell(xy)
        radius_cells = int(math.ceil(radius_m / max(self.resolution, 1e-9)))
        values: list[int | None] = []
        for yy in range(cy - radius_cells, cy + radius_cells + 1):
            for xx in range(cx - radius_cells, cx + radius_cells + 1):
                wx = self.center_xy[0] + (xx - self.center_cell[0]) * self.resolution
                wy = self.center_xy[1] + (yy - self.center_cell[1]) * self.resolution
                if math.hypot(wx - xy[0], wy - xy[1]) <= radius_m + 1e-9:
                    values.append(self.grid.get((xx, yy)))
        return values

    def footprint_values(self, pose: tuple[float, float, float], length_m: float = 0.60, width_m: float = 0.50) -> list[int | None]:
        x, y, yaw = pose
        step = max(self.resolution, 0.05)
        half_l = length_m / 2.0
        half_w = width_m / 2.0
        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)
        nx = int(math.ceil(length_m / step))
        ny = int(math.ceil(width_m / step))
        values: list[int | None] = []
        for i in range(-nx, nx + 1):
            local_x = half_l * i / max(nx, 1)
            for j in range(-ny, ny + 1):
                local_y = half_w * j / max(ny, 1)
                wx = x + local_x * cos_y - local_y * sin_y
                wy = y + local_x * sin_y + local_y * cos_y
                values.append(self.value_at((wx, wy)))
        return values

    def front_wedge_values(self, pose: tuple[float, float, float], radius_m: float = 0.75, half_angle_rad: float = math.radians(35.0)) -> list[int | None]:
        x, y, yaw = pose
        values: list[int | None] = []
        cell_radius = int(math.ceil(radius_m / max(self.resolution, 1e-9)))
        cx, cy = self.world_to_cell((x, y))
        for yy in range(cy - cell_radius, cy + cell_radius + 1):
            for xx in range(cx - cell_radius, cx + cell_radius + 1):
                wx = self.center_xy[0] + (xx - self.center_cell[0]) * self.resolution
                wy = self.center_xy[1] + (yy - self.center_cell[1]) * self.resolution
                dx = wx - x
                dy = wy - y
                dist = math.hypot(dx, dy)
                if dist > radius_m or dist <= 1e-9:
                    continue
                delta = math.atan2(math.sin(math.atan2(dy, dx) - yaw), math.cos(math.atan2(dy, dx) - yaw))
                if abs(delta) <= half_angle_rad:
                    values.append(self.grid.get((xx, yy)))
        return values


def extract_phase62_first_dispatch(phase62: dict[str, Any]) -> tuple[dict[str, Any] | None, dict[str, Any] | None]:
    for replay in phase62.get('replays', []):
        if not isinstance(replay, dict):
            continue
        dispatch = replay.get('first_dispatch_event')
        if isinstance(dispatch, dict) and replay.get('dispatch_observed'):
            return dispatch, replay
    return None, None


def _direction_angle(dispatch: dict[str, Any]) -> float | None:
    value = _number(dispatch.get('branch_angle'))
    if value is not None:
        return value
    target = dispatch.get('target')
    pose = dispatch.get('dispatch_pose')
    if isinstance(target, list) and len(target) >= 2 and isinstance(pose, list) and len(pose) >= 2:
        return math.atan2(float(target[1]) - float(pose[1]), float(target[0]) - float(pose[0]))
    return None


def project_candidates_along_corridor(dispatch: dict[str, Any], distances: list[float] | None = None) -> list[dict[str, Any]]:
    distances = distances if distances is not None else [-0.30, -0.20, -0.10, 0.0, 0.10, 0.20, 0.30]
    target = dispatch.get('target')
    angle = _direction_angle(dispatch)
    if not isinstance(target, list) or len(target) < 2 or angle is None:
        return []
    unit = (math.cos(angle), math.sin(angle))
    rows: list[dict[str, Any]] = []
    for distance in distances:
        x = float(target[0]) + float(distance) * unit[0]
        y = float(target[1]) + float(distance) * unit[1]
        rows.append({
            'target': [x, y],
            'projection_distance_m': float(distance),
            'open_direction_rad': angle,
            'same_open_direction': True,
            'projection_model': 'same_open_direction_corridor_centerline_static_replay',
        })
    return rows


def _original_evidence(dispatch: dict[str, Any]) -> dict[str, Any]:
    target = dispatch.get('target')
    target_cell = dispatch.get('phase62_target_cell_state') or {}
    footprint = dispatch.get('phase62_target_footprint_cost') or {}
    front = dispatch.get('phase62_front_wedge_cost') or {}
    patch = dispatch.get('phase62_local_costmap_patch') or {}
    return {
        'target': target,
        'projection_distance_m': 0.0,
        'occupancy': dispatch.get('target_cell_occupancy'),
        'target_clearance_m': dispatch.get('target_clearance_m'),
        'path_corridor_min_clearance_m': dispatch.get('path_corridor_min_clearance_m'),
        'target_crosses_narrow_passage': dispatch.get('target_crosses_narrow_passage'),
        'local_cell_state': target_cell,
        'local_radius_cost_summary': {'max': target_cell.get('max_radius_cost'), 'value': target_cell.get('value'), 'state': target_cell.get('state')},
        'footprint_cost_summary': footprint.get('summary') if isinstance(footprint, dict) else None,
        'front_wedge_cost_summary': front if isinstance(front, dict) else None,
        'local_costmap_patch_summary': patch.get('summary') if isinstance(patch, dict) else None,
    }


def _score(radius_summary: dict[str, Any], footprint_summary: dict[str, Any], front_summary: dict[str, Any]) -> float:
    # Missing replay coverage must not look artificially safe.  Penalize None so
    # a partial projection patch is reported as incomplete rather than preferred.
    radius_max = _number(radius_summary.get('max'))
    footprint_max = _number(footprint_summary.get('max'))
    front_max = _number(front_summary.get('max'))
    radius_term = radius_max if radius_max is not None else 999.0
    footprint_term = footprint_max if footprint_max is not None else 999.0
    front_term = front_max if front_max is not None else 999.0
    footprint_lethal = _number(footprint_summary.get('lethal_count')) or 0.0
    footprint_high = _number(footprint_summary.get('high_cost_count')) or 0.0
    return radius_term * 1.0 + footprint_term * 1.5 + footprint_lethal * 20.0 + footprint_high * 2.0 + front_term * 0.25


def evaluate_projection_candidate(candidate: dict[str, Any], dispatch: dict[str, Any], replay_patch: ReplayCostPatch | None, original: dict[str, Any]) -> dict[str, Any]:
    out = dict(candidate)
    target = candidate.get('target')
    angle = _number(candidate.get('open_direction_rad')) or _direction_angle(dispatch) or 0.0
    pose = (float(target[0]), float(target[1]), angle) if isinstance(target, list) and len(target) >= 2 else None
    original_target = dispatch.get('target')
    distance_to_original = None
    if pose and isinstance(target, list) and len(target) >= 2 and isinstance(original_target, list) and len(original_target) >= 2:
        distance_to_original = math.hypot(float(target[0]) - float(original_target[0]), float(target[1]) - float(original_target[1]))
    within_small_projection_window = distance_to_original is not None and distance_to_original <= 0.35 + 1e-9
    same_corridor = bool(candidate.get('same_open_direction')) and within_small_projection_window
    if replay_patch is None or pose is None:
        out.update({
            'occupancy': None,
            'replay_evidence_available': False,
            'same_corridor': same_corridor,
            'distance_to_original_target_m': distance_to_original,
            'within_small_projection_window': within_small_projection_window,
            'local_radius_cost_summary': {'max': None},
            'footprint_cost_summary': {'max': None, 'lethal_count': 0, 'high_cost_count': 0},
            'front_wedge_cost_summary': {'max': None},
            'safety_score': None,
            'improves_over_original': False,
            'safe_projection': False,
            'rejection_reason': 'insufficient_replay_cost_patch',
        })
        return out
    radius_summary = _summary(replay_patch.radius_values((pose[0], pose[1]), 0.30))
    footprint_summary = _summary(replay_patch.footprint_values(pose))
    front_summary = _summary(replay_patch.front_wedge_values(pose))
    original_radius = original.get('local_radius_cost_summary') or {}
    original_footprint = original.get('footprint_cost_summary') or {}
    original_front = original.get('front_wedge_cost_summary') or {}
    original_score = _score(
        {'max': original_radius.get('max') or original_radius.get('max_radius_cost') or original_radius.get('value')},
        original_footprint if isinstance(original_footprint, dict) else {},
        original_front if isinstance(original_front, dict) else {},
    )
    safety_score = _score(radius_summary, footprint_summary, front_summary)
    improves = safety_score < original_score and (footprint_summary.get('lethal_count') or 0) <= (original_footprint.get('lethal_count') or 0)
    radius_safe = (radius_summary.get('max') is not None and int(radius_summary['max']) < SAFE_RADIUS_MAX_COST)
    footprint_safe = (footprint_summary.get('max') is not None and int(footprint_summary['max']) < SAFE_FOOTPRINT_MAX_COST and int(footprint_summary.get('lethal_count') or 0) <= SAFE_FOOTPRINT_LETHAL_COUNT)
    front_has_coverage = bool((front_summary.get('in_bounds_sample_count') or 0) > 0)
    front_safe = (front_has_coverage and front_summary.get('max') is not None and int(front_summary['max']) < SAFE_FRONT_WEDGE_MAX_COST)
    occupancy = 0 if (radius_summary.get('in_bounds_sample_count') or 0) > 0 else None
    safe_projection = bool(same_corridor and improves and radius_safe and footprint_safe and front_safe and occupancy == 0)
    reasons = []
    if not same_corridor:
        reasons.append('not_same_corridor_or_too_far')
    if not improves:
        reasons.append('not_safer_than_original')
    if not radius_safe:
        reasons.append('radius_cost_not_safe')
    if not footprint_safe:
        reasons.append('footprint_cost_not_safe')
    if not front_safe:
        reasons.append('front_wedge_not_safe')
    if occupancy != 0:
        reasons.append('occupancy_not_replay_clear')
    out.update({
        'occupancy': occupancy,
        'replay_evidence_available': True,
        'same_corridor': same_corridor,
        'distance_to_original_target_m': distance_to_original,
        'within_small_projection_window': within_small_projection_window,
        'local_radius_cost_summary': radius_summary,
        'footprint_cost_summary': footprint_summary,
        'front_wedge_cost_summary': front_summary,
        'safety_score': safety_score,
        'original_safety_score': original_score,
        'improves_over_original': improves,
        'safe_projection': safe_projection,
        'rejection_reason': None if safe_projection else ','.join(reasons),
    })
    return out


def classify_phase63(candidates: list[dict[str, Any]], dispatch: dict[str, Any] | None, replay_patch: ReplayCostPatch | None) -> str:
    if dispatch is None or replay_patch is None or not candidates:
        return 'INSUFFICIENT_REPLAY_EVIDENCE'
    if any(c.get('safe_projection') for c in candidates):
        return 'SAFE_PROJECTION_FOUND'
    if any(c.get('replay_evidence_available') and c.get('same_corridor') for c in candidates):
        return 'NO_SAFE_PROJECTION_IN_CORRIDOR'
    return 'INSUFFICIENT_REPLAY_EVIDENCE'


def analyze_phase63(phase62_artifact: Path, artifact_dir: Path, output: Path | None = None) -> dict[str, Any]:
    phase62 = _load_json(phase62_artifact)
    dispatch, replay = extract_phase62_first_dispatch(phase62)
    original = _original_evidence(dispatch or {}) if dispatch else {'target': None}
    replay_patch = ReplayCostPatch.from_phase62(dispatch or {}, replay or {}) if dispatch and replay else None
    raw_candidates = project_candidates_along_corridor(dispatch or {}) if dispatch else []
    projection_candidates = [evaluate_projection_candidate(c, dispatch or {}, replay_patch, original) for c in raw_candidates]
    safe = [c for c in projection_candidates if c.get('safe_projection')]
    best_projection = None
    if safe:
        best_projection = sorted(safe, key=lambda c: (float(c.get('safety_score') or 1e9), abs(float(c.get('projection_distance_m') or 0.0))))[0]
    else:
        scored = [c for c in projection_candidates if c.get('safety_score') is not None]
        best_projection = sorted(scored, key=lambda c: float(c.get('safety_score') or 1e9))[0] if scored else None
    classification = classify_phase63(projection_candidates, dispatch, replay_patch)
    result = {
        'phase': PHASE,
        'run_id': RUN_ID,
        'artifact_dir': str(artifact_dir),
        'source_phase62_artifact': str(phase62_artifact),
        'active_world': ACTIVE_WORLD,
        'active_metadata': ACTIVE_METADATA,
        'phase62_run_id': PHASE62_RUN_ID,
        'phase62_classification_preserved': PHASE62_CLASSIFICATION,
        'phase62_secondary_factor_preserved': PHASE62_SECONDARY_FACTOR,
        'phase61_classification_preserved': PHASE61_CLASSIFICATION,
        'classification': classification,
        'allowed_classifications': sorted(ALLOWED_CLASSIFICATIONS),
        'guardrails': GUARDRAILS,
        'runtime_dispatch_changed': False,
        'runtime_dispatch_integrated': False,
        'complete_autonomous_success_claimed': False,
        'first_dispatch_is_not_exit_success': True,
        'dispatch_source': {
            'observed_in_phase62': dispatch is not None,
            'goal_sequence': (dispatch or {}).get('goal_sequence'),
            'selected_due_to_context': (dispatch or {}).get('selected_due_to_context'),
            'single_open_exception_applied': (dispatch or {}).get('single_open_exception_applied'),
            'local_topology': (dispatch or {}).get('local_topology'),
            'branch_angle': (dispatch or {}).get('branch_angle'),
        },
        'projection_design': {
            'scope': 'design_static_replay_only',
            'eligibility_note': 'Runtime integration would be limited to post-ingress single-open exception candidates with map-clear target and high local footprint/radius cost; Phase63 does not integrate it.',
            'projection_axis': 'same open direction / corridor centerline',
            'projection_distances_m': [-0.30, -0.20, -0.10, 0.0, 0.10, 0.20, 0.30],
            'small_projection_window_m': 0.35,
            'safety_thresholds': {
                'radius_max_cost_lt': SAFE_RADIUS_MAX_COST,
                'footprint_max_cost_lt': SAFE_FOOTPRINT_MAX_COST,
                'footprint_lethal_count_lte': SAFE_FOOTPRINT_LETHAL_COUNT,
                'front_wedge_max_cost_lt': SAFE_FRONT_WEDGE_MAX_COST,
            },
        },
        'original_target_evidence': original,
        'replay_patch_available': replay_patch is not None,
        'projection_candidates': projection_candidates,
        'best_projection': best_projection,
        'safe_projection_found': classification == 'SAFE_PROJECTION_FOUND',
        'recommendations': [
            'Keep this as design/static replay evidence only; do not alter runtime dispatch in Phase63.',
            'If later accepted, implement behind a narrow gate limited to post-ingress single-open exception candidates and revalidate with TDD plus bounded max_goals=1 runtime.',
        ],
    }
    if output is not None:
        _write_json(output, result)
        md = output.with_suffix('.md')
        md.write_text(render_markdown(result), encoding='utf-8')
    return result


def render_markdown(result: dict[str, Any]) -> str:
    best = result.get('best_projection') or {}
    lines = [
        f"# {PHASE}",
        '',
        f"run_id: `{RUN_ID}`",
        f"classification: `{result.get('classification')}`",
        '',
        '## Preserved conclusions',
        '',
        f"- Phase62: `{PHASE62_CLASSIFICATION}`; secondary `{PHASE62_SECONDARY_FACTOR}`.",
        f"- Phase61: `{PHASE61_CLASSIFICATION}`.",
        '- First dispatch remains not autonomous exploration success and not exit success.',
        '',
        '## Guardrails',
        '',
    ]
    lines.extend(f"- {g}" for g in GUARDRAILS)
    lines.extend([
        '',
        '## Static replay result',
        '',
        f"- replay_patch_available: `{result.get('replay_patch_available')}`",
        f"- safe_projection_found: `{result.get('safe_projection_found')}`",
        f"- candidate_count: `{len(result.get('projection_candidates') or [])}`",
        '',
        '## Best projection candidate',
        '',
        f"- projection_distance_m: `{best.get('projection_distance_m')}`",
        f"- target: `{best.get('target')}`",
        f"- same_corridor: `{best.get('same_corridor')}`",
        f"- improves_over_original: `{best.get('improves_over_original')}`",
        f"- safe_projection: `{best.get('safe_projection')}`",
        f"- rejection_reason: `{best.get('rejection_reason')}`",
        f"- local_radius_cost_summary: `{best.get('local_radius_cost_summary')}`",
        f"- footprint_cost_summary: `{best.get('footprint_cost_summary')}`",
        f"- front_wedge_cost_summary: `{best.get('front_wedge_cost_summary')}`",
        '',
        '## Stop condition',
        '',
        'Phase63 stops for human acceptance. Do not enter Phase64. 不进入 Phase64.',
        '',
    ])
    return '\n'.join(lines)


def main() -> int:
    parser = argparse.ArgumentParser(description=PHASE)
    parser.add_argument('--phase62-artifact', default=f'log/{PHASE62_RUN_ID}/{PHASE62_RUN_ID}.json')
    parser.add_argument('--artifact-dir', default=f'log/{RUN_ID}')
    parser.add_argument('--output', default=None)
    args = parser.parse_args()
    artifact_dir = Path(args.artifact_dir)
    output = Path(args.output) if args.output else artifact_dir / f'{RUN_ID}.json'
    result = analyze_phase63(Path(args.phase62_artifact), artifact_dir, output)
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
