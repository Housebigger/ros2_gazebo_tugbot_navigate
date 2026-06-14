import importlib.util
import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / 'tools' / 'analyze_phase27_alt_r5_footprint_acceptance_offline.py'


def _load_module():
    spec = importlib.util.spec_from_file_location('phase27_alt_r5_footprint', SCRIPT)
    assert spec is not None
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _grid(width=12, height=9, resolution=0.1, origin_x=-0.4, origin_y=-0.4, fill=0):
    return {
        'width': width,
        'height': height,
        'resolution': resolution,
        'origin_x': origin_x,
        'origin_y': origin_y,
        'data': [fill for _ in range(width * height)],
    }


def _set_cell(grid, mx, my, value):
    grid['data'][my * grid['width'] + mx] = value


def test_footprint_contact_and_base_not_at_exit_accept_when_known_clear_los():
    mod = _load_module()
    grid = _grid()

    contact = mod.evaluate_footprint_terminal_acceptance(
        robot_xy=(0.49, 0.0),
        exit_xy=(0.0, 0.0),
        robot_circumscribed_radius_m=0.49,
        terminal_acceptance_margin_m=0.0,
        occupancy_grid=grid,
    )
    assert contact['accepted'] is True
    assert contact['distance_gate_passed'] is True
    assert contact['reason'] == 'accepted'

    covered_not_origin = mod.evaluate_footprint_terminal_acceptance(
        robot_xy=(0.53, 0.0),
        exit_xy=(0.0, 0.0),
        robot_circumscribed_radius_m=0.49,
        terminal_acceptance_margin_m=0.05,
        occupancy_grid=grid,
    )
    assert covered_not_origin['accepted'] is True
    assert covered_not_origin['acceptance_radius_m'] == 0.54
    assert covered_not_origin['distance_m'] == 0.53


def test_distance_boundary_is_stable_and_exceeding_radius_fails():
    mod = _load_module()
    grid = _grid(width=30, height=5, origin_x=-0.2, origin_y=-0.2)

    boundary = mod.evaluate_footprint_terminal_acceptance(
        robot_xy=(0.5400000001, 0.0),
        exit_xy=(0.0, 0.0),
        robot_circumscribed_radius_m=0.49,
        terminal_acceptance_margin_m=0.05,
        occupancy_grid=grid,
    )
    assert boundary['accepted'] is True
    assert boundary['distance_gate_passed'] is True

    outside = mod.evaluate_footprint_terminal_acceptance(
        robot_xy=(0.56, 0.0),
        exit_xy=(0.0, 0.0),
        robot_circumscribed_radius_m=0.49,
        terminal_acceptance_margin_m=0.05,
        occupancy_grid=grid,
    )
    assert outside['accepted'] is False
    assert outside['reason'] == 'distance_exceeds_footprint_radius_plus_margin'


def test_occupied_wall_between_robot_and_exit_rejects_even_when_distance_passes():
    mod = _load_module()
    grid = _grid(width=11, height=5, resolution=0.1, origin_x=-0.1, origin_y=-0.2)
    _set_cell(grid, 3, 2, 100)  # line between exit at x=0 and robot at x=0.5

    result = mod.evaluate_footprint_terminal_acceptance(
        robot_xy=(0.5, 0.0),
        exit_xy=(0.0, 0.0),
        robot_circumscribed_radius_m=0.5,
        terminal_acceptance_margin_m=0.02,
        occupancy_grid=grid,
    )
    assert result['distance_gate_passed'] is True
    assert result['accepted'] is False
    assert result['line_of_sight_occupied_count'] > 0
    assert result['reason'] == 'line_of_sight_occupied'


def test_robot_or_exit_occupied_and_unknown_cells_are_conservative_rejects():
    mod = _load_module()

    exit_occupied = _grid()
    _set_cell(exit_occupied, 4, 4, 100)
    result = mod.evaluate_footprint_terminal_acceptance(
        robot_xy=(0.3, 0.0),
        exit_xy=(0.0, 0.0),
        robot_circumscribed_radius_m=0.5,
        terminal_acceptance_margin_m=0.0,
        occupancy_grid=exit_occupied,
    )
    assert result['accepted'] is False
    assert result['reason'] == 'exit_cell_occupied'

    unknown = _grid()
    _set_cell(unknown, 4, 4, -1)
    result = mod.evaluate_footprint_terminal_acceptance(
        robot_xy=(0.3, 0.0),
        exit_xy=(0.0, 0.0),
        robot_circumscribed_radius_m=0.5,
        terminal_acceptance_margin_m=0.0,
        occupancy_grid=unknown,
        unknown_policy='conservative_reject',
    )
    assert result['accepted'] is False
    assert result['reason'] == 'exit_cell_unknown_conservative_reject'

    recorded = mod.evaluate_footprint_terminal_acceptance(
        robot_xy=(0.3, 0.0),
        exit_xy=(0.0, 0.0),
        robot_circumscribed_radius_m=0.5,
        terminal_acceptance_margin_m=0.0,
        occupancy_grid=unknown,
        unknown_policy='record_only',
    )
    assert recorded['accepted'] is True
    assert recorded['unknown_cell_count'] > 0
    assert recorded['unknown_policy'] == 'record_only'


def test_cli_offline_comparison_reports_r4_final_distances_not_accepted(tmp_path):
    state = tmp_path / 'state.jsonl'
    state.write_text('\n'.join([
        json.dumps({'state': {'mode': 'NAVIGATING', 'exit_distance_m': 0.782}}),
        json.dumps({'state': {'mode': 'FAILED_EXHAUSTED', 'exit_distance_m': 0.703}}),
    ]) + '\n')
    output = tmp_path / 'out.json'
    subprocess.run([
        sys.executable,
        str(SCRIPT),
        '--output-json', str(output),
        '--robot-circumscribed-radius-m', '0.4942061445948485',
        '--terminal-acceptance-margin-m', '0.05',
        '--run', 'synthetic_r4', str(state),
    ], check=True)
    data = json.loads(output.read_text())
    run = data['runs'][0]
    assert data['acceptance_radius_m'] == 0.5442061445948485
    assert run['current_radius_final_accepts'] is False
    assert run['footprint_radius_final_accepts'] is False
    assert run['min_distance_current_radius_accepts'] is False
    assert run['min_distance_footprint_radius_accepts'] is False
    assert data['guardrails']['runtime_smoke_started'] is False
    assert data['mppi_root_cause_claim'] == 'not_evaluated_by_phase27_alt_r5'
