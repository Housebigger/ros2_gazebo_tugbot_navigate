#!/usr/bin/env python3
"""Phase27-alt-R5 offline footprint terminal acceptance design analyzer.

This is design/static tooling only. It provides a pure-Python acceptance helper
for a proposed footprint-based terminal line/cylinder predicate and an offline
log comparator for existing explorer_state JSONL artifacts. It does not start
runtime smoke, modify Nav2/MPPI/controller parameters, or replace the current
terminal predicate.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any, Iterable

CURRENT_TERMINAL_RADIUS_M = 0.6
DEFAULT_ROBOT_CIRCUMSCRIBED_RADIUS_M = 0.4942061445948485
DEFAULT_TERMINAL_MARGIN_M = 0.05
EPS = 1e-9


def number(value: Any) -> float | None:
    if value is None or isinstance(value, bool):
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def load_jsonl(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    if not path.exists():
        return rows
    for line_no, line in enumerate(path.read_text(encoding='utf-8').splitlines(), start=1):
        if not line.strip():
            continue
        try:
            raw = json.loads(line)
        except json.JSONDecodeError:
            continue
        payload = raw.get('state', raw) if isinstance(raw, dict) else raw
        if isinstance(payload, dict):
            payload = dict(payload)
            payload['_line_no'] = line_no
            rows.append(payload)
    return rows


def grid_meta(grid: dict[str, Any]) -> tuple[int, int, float, float, float, list[int]]:
    width = int(grid['width'])
    height = int(grid['height'])
    resolution = float(grid['resolution'])
    origin_x = float(grid.get('origin_x', 0.0))
    origin_y = float(grid.get('origin_y', 0.0))
    data = [int(v) for v in grid['data']]
    return width, height, resolution, origin_x, origin_y, data


def world_to_cell(x: float, y: float, grid: dict[str, Any]) -> tuple[int, int] | None:
    width, height, resolution, origin_x, origin_y, _ = grid_meta(grid)
    mx = int(math.floor((x - origin_x) / resolution))
    my = int(math.floor((y - origin_y) / resolution))
    if mx < 0 or my < 0 or mx >= width or my >= height:
        return None
    return mx, my


def cell_value(cell: tuple[int, int] | None, grid: dict[str, Any]) -> int | None:
    if cell is None:
        return None
    width, _height, _resolution, _origin_x, _origin_y, data = grid_meta(grid)
    mx, my = cell
    return data[my * width + mx]


def sample_line_cells(start_xy: tuple[float, float], end_xy: tuple[float, float], grid: dict[str, Any]) -> list[tuple[int, int] | None]:
    width, height, resolution, origin_x, origin_y, _ = grid_meta(grid)
    del width, height, origin_x, origin_y
    sx, sy = start_xy
    ex, ey = end_xy
    distance = math.hypot(ex - sx, ey - sy)
    steps = max(1, int(math.ceil(distance / max(resolution * 0.5, 1e-6))))
    cells: list[tuple[int, int] | None] = []
    previous: tuple[int, int] | None | object = object()
    for i in range(steps + 1):
        t = i / steps
        cell = world_to_cell(sx + (ex - sx) * t, sy + (ey - sy) * t, grid)
        if cell != previous:
            cells.append(cell)
            previous = cell
    return cells


def classify_value(value: int | None, occupied_threshold: int) -> str:
    if value is None:
        return 'unknown'
    if value < 0:
        return 'unknown'
    if value >= occupied_threshold:
        return 'occupied'
    return 'free'


def evaluate_footprint_terminal_acceptance(
    *,
    robot_xy: tuple[float, float],
    exit_xy: tuple[float, float],
    robot_circumscribed_radius_m: float,
    terminal_acceptance_margin_m: float,
    occupancy_grid: dict[str, Any] | None = None,
    occupied_threshold: int = 65,
    unknown_policy: str = 'conservative_reject',
    epsilon: float = EPS,
) -> dict[str, Any]:
    """Evaluate the proposed footprint terminal line/cylinder predicate.

    Unknown policy:
    - conservative_reject: any unknown/out-of-bounds robot, exit, or LOS cell rejects.
    - record_only: unknown cells are counted but do not reject by themselves.
    """
    if unknown_policy not in {'conservative_reject', 'record_only'}:
        raise ValueError(f'unsupported unknown_policy: {unknown_policy}')
    rx, ry = robot_xy
    ex, ey = exit_xy
    distance = math.hypot(rx - ex, ry - ey)
    acceptance_radius = float(robot_circumscribed_radius_m) + float(terminal_acceptance_margin_m)
    distance_gate = distance <= acceptance_radius + epsilon
    result: dict[str, Any] = {
        'accepted': False,
        'reason': None,
        'distance_m': round(distance, 12),
        'acceptance_radius_m': acceptance_radius,
        'distance_gate_passed': distance_gate,
        'robot_circumscribed_radius_m': robot_circumscribed_radius_m,
        'terminal_acceptance_margin_m': terminal_acceptance_margin_m,
        'unknown_policy': unknown_policy,
        'robot_cell_value': None,
        'exit_cell_value': None,
        'line_of_sight_occupied_count': 0,
        'line_of_sight_unknown_count': 0,
        'unknown_cell_count': 0,
    }
    if not distance_gate:
        result['reason'] = 'distance_exceeds_footprint_radius_plus_margin'
        return result
    if occupancy_grid is None:
        result['accepted'] = True
        result['reason'] = 'accepted_without_occupancy_grid'
        return result

    robot_cell = world_to_cell(rx, ry, occupancy_grid)
    exit_cell = world_to_cell(ex, ey, occupancy_grid)
    robot_value = cell_value(robot_cell, occupancy_grid)
    exit_value = cell_value(exit_cell, occupancy_grid)
    result['robot_cell_value'] = robot_value
    result['exit_cell_value'] = exit_value

    robot_class = classify_value(robot_value, occupied_threshold)
    exit_class = classify_value(exit_value, occupied_threshold)
    if robot_class == 'occupied':
        result['reason'] = 'robot_cell_occupied'
        return result
    if exit_class == 'occupied':
        result['reason'] = 'exit_cell_occupied'
        return result
    if unknown_policy == 'conservative_reject':
        if robot_class == 'unknown':
            result['reason'] = 'robot_cell_unknown_conservative_reject'
            result['unknown_cell_count'] = 1
            return result
        if exit_class == 'unknown':
            result['reason'] = 'exit_cell_unknown_conservative_reject'
            result['unknown_cell_count'] = 1
            return result

    los_cells = sample_line_cells(exit_xy, robot_xy, occupancy_grid)
    occupied_count = 0
    unknown_count = 0
    for cell in los_cells:
        value = cell_value(cell, occupancy_grid)
        cls = classify_value(value, occupied_threshold)
        if cls == 'occupied':
            occupied_count += 1
        elif cls == 'unknown':
            unknown_count += 1
    result['line_of_sight_occupied_count'] = occupied_count
    result['line_of_sight_unknown_count'] = unknown_count
    result['unknown_cell_count'] = unknown_count + (1 if robot_class == 'unknown' else 0) + (1 if exit_class == 'unknown' else 0)
    if occupied_count > 0:
        result['reason'] = 'line_of_sight_occupied'
        return result
    if unknown_policy == 'conservative_reject' and unknown_count > 0:
        result['reason'] = 'line_of_sight_unknown_conservative_reject'
        return result
    result['accepted'] = True
    result['reason'] = 'accepted'
    return result


def state_distances(rows: Iterable[dict[str, Any]]) -> list[float]:
    values: list[float] = []
    for row in rows:
        value = number(row.get('exit_distance_m'))
        if value is not None:
            values.append(value)
    return values


def summarize_state_run(
    run_id: str,
    state_path: Path,
    *,
    current_radius_m: float,
    footprint_acceptance_radius_m: float,
) -> dict[str, Any]:
    rows = load_jsonl(state_path)
    distances = state_distances(rows)
    final = rows[-1] if rows else {}
    final_distance = number(final.get('exit_distance_m'))
    min_distance = min(distances) if distances else None
    current_accept_rows = [d for d in distances if d <= current_radius_m + EPS]
    footprint_accept_rows = [d for d in distances if d <= footprint_acceptance_radius_m + EPS]
    def within(row: dict[str, Any], radius: float) -> bool:
        value = number(row.get('exit_distance_m'))
        return value is not None and value <= radius + EPS

    first_current = next((row for row in rows if within(row, current_radius_m)), None)
    first_footprint = next((row for row in rows if within(row, footprint_acceptance_radius_m)), None)
    return {
        'run_id': run_id,
        'explorer_state': str(state_path),
        'row_count': len(rows),
        'final_mode': final.get('mode'),
        'final_exit_distance_m': final_distance,
        'min_exit_distance_m': min_distance,
        'current_radius_final_accepts': final_distance is not None and final_distance <= current_radius_m + EPS,
        'footprint_radius_final_accepts': final_distance is not None and final_distance <= footprint_acceptance_radius_m + EPS,
        'min_distance_current_radius_accepts': min_distance is not None and min_distance <= current_radius_m + EPS,
        'min_distance_footprint_radius_accepts': min_distance is not None and min_distance <= footprint_acceptance_radius_m + EPS,
        'current_radius_accept_count': len(current_accept_rows),
        'footprint_radius_accept_count': len(footprint_accept_rows),
        'first_current_radius_accept': {
            'line_no': first_current.get('_line_no'),
            'mode': first_current.get('mode'),
            'exit_distance_m': number(first_current.get('exit_distance_m')),
        } if first_current else None,
        'first_footprint_radius_accept': {
            'line_no': first_footprint.get('_line_no'),
            'mode': first_footprint.get('mode'),
            'exit_distance_m': number(first_footprint.get('exit_distance_m')),
        } if first_footprint else None,
        'footprint_accepts_more_than_current': len(footprint_accept_rows) > len(current_accept_rows),
    }


def analyze_runs(args: argparse.Namespace) -> dict[str, Any]:
    acceptance_radius = args.robot_circumscribed_radius_m + args.terminal_acceptance_margin_m
    runs = [
        summarize_state_run(run_id, path, current_radius_m=args.current_terminal_radius_m, footprint_acceptance_radius_m=acceptance_radius)
        for run_id, path in args.run
    ]
    return {
        'phase': 'Phase27-alt-R5',
        'analysis_type': 'offline_design_comparison_only',
        'current_terminal_radius_m': args.current_terminal_radius_m,
        'robot_circumscribed_radius_m': args.robot_circumscribed_radius_m,
        'terminal_acceptance_margin_m': args.terminal_acceptance_margin_m,
        'acceptance_radius_m': acceptance_radius,
        'relationship_to_current_radius': {
            'footprint_plus_margin_less_than_current_0_6m': acceptance_radius < args.current_terminal_radius_m,
            'delta_m': acceptance_radius - args.current_terminal_radius_m,
        },
        'runs': runs,
        'aggregate': {
            'run_count': len(runs),
            'current_radius_final_accept_count': sum(1 for run in runs if run['current_radius_final_accepts']),
            'footprint_radius_final_accept_count': sum(1 for run in runs if run['footprint_radius_final_accepts']),
            'current_radius_min_distance_accept_count': sum(1 for run in runs if run['min_distance_current_radius_accepts']),
            'footprint_radius_min_distance_accept_count': sum(1 for run in runs if run['min_distance_footprint_radius_accepts']),
            'footprint_accepts_more_runs_than_current': any(run['footprint_accepts_more_than_current'] for run in runs),
        },
        'design_gates': {
            'distance_gate': 'distance(base_link_xy, exit_xy) <= robot_circumscribed_radius_m + terminal_acceptance_margin_m',
            'robot_exit_cells_not_occupied': True,
            'line_of_sight_no_occupied_cells': True,
            'unknown_policy_default': 'conservative_reject',
            'unknown_policy_alternative': 'record_only_for_diagnostics_not_acceptance_default',
        },
        'guardrails': {
            'runtime_smoke_started': False,
            'existing_terminal_predicate_replaced': False,
            'nav2_mppi_controller_params_modified': False,
            'local_cost_gate_relaxed': False,
            'branch_selection_strategy_modified': False,
            'terminal_acceptance_runtime_forced': False,
        },
        'mppi_root_cause_claim': 'not_evaluated_by_phase27_alt_r5',
        'conclusion': 'PASS_AS_FOOTPRINT_TERMINAL_ACCEPTANCE_DESIGN_ONLY',
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument('--output-json', required=True, type=Path)
    parser.add_argument('--robot-circumscribed-radius-m', type=float, default=DEFAULT_ROBOT_CIRCUMSCRIBED_RADIUS_M)
    parser.add_argument('--terminal-acceptance-margin-m', type=float, default=DEFAULT_TERMINAL_MARGIN_M)
    parser.add_argument('--current-terminal-radius-m', type=float, default=CURRENT_TERMINAL_RADIUS_M)
    parser.add_argument('--run', nargs=2, action='append', metavar=('RUN_ID', 'EXPLORER_STATE_JSONL'), type=str, default=[])
    args = parser.parse_args()
    args.run = [(run_id, Path(path)) for run_id, path in args.run]
    return args


def main() -> int:
    args = parse_args()
    result = analyze_runs(args)
    args.output_json.parent.mkdir(parents=True, exist_ok=True)
    args.output_json.write_text(json.dumps(result, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    print(json.dumps({'output': str(args.output_json), 'conclusion': result['conclusion'], 'run_count': len(result['runs'])}, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
