from pathlib import Path
import importlib.util
import json
import sys

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'src' / 'tugbot_maze'))

from tugbot_maze.grid_utils import OccupancyGridInfo, OccupancyGridView

MAZE_EXPLORER = ROOT / 'src' / 'tugbot_maze' / 'tugbot_maze' / 'maze_explorer.py'
MAZE_PERCEPTION = ROOT / 'src' / 'tugbot_maze' / 'tugbot_maze' / 'maze_perception.py'
WRAPPER = ROOT / 'tools' / 'run_phase70_centerline_gate_relaxation_balance_first_runtime_validation.sh'
ANALYZER = ROOT / 'tools' / 'analyze_phase70_centerline_gate_relaxation_balance_first_runtime_validation.py'
REPORT = ROOT / 'doc' / 'doc_report' / 'phase70_centerline_gate_relaxation_balance_first_runtime_validation_report.md'
RUN_ID = 'phase70_centerline_gate_relaxation_balance_first_runtime_validation'


def _read(path: Path) -> str:
    assert path.exists(), f'missing {path}'
    return path.read_text(encoding='utf-8')


def _load_analyzer():
    spec = importlib.util.spec_from_file_location('phase70_analyzer', ANALYZER)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _corridor_grid(width_m=4.0, height_m=4.0, resolution=0.05):
    width = int(width_m / resolution)
    height = int(height_m / resolution)
    origin_x = -width_m / 2.0
    origin_y = -height_m / 2.0
    data = [0] * (width * height)
    info = OccupancyGridInfo(width=width, height=height, resolution=resolution, origin_x=origin_x, origin_y=origin_y)
    for y in range(height):
        for x in range(width):
            wx = origin_x + (x + 0.5) * resolution
            # vertical corridor along +Y, walls at x ~= +/-1.0
            if abs(wx) >= 0.95:
                data[x + y * width] = 100
    return OccupancyGridView(info, data, occupied_threshold=65)


def _local_cost_grid(width_m=4.0, height_m=4.0, resolution=0.05, lethal_x_min=None):
    width = int(width_m / resolution)
    height = int(height_m / resolution)
    origin_x = -width_m / 2.0
    origin_y = -height_m / 2.0
    data = []
    for y in range(height):
        wy = origin_y + (y + 0.5) * resolution
        for x in range(width):
            wx = origin_x + (x + 0.5) * resolution
            # Keep local-cost mildly asymmetric so a centered candidate can improve balance
            # while not necessarily improving every conservative Phase69 cost metric.
            cost = min(98, int(max(0.0, abs(wx) - 0.05) * 80.0))
            if lethal_x_min is not None and wx >= lethal_x_min and 0.4 <= wy <= 1.6:
                cost = 100
            data.append(cost)
    return OccupancyGridView(OccupancyGridInfo(width=width, height=height, resolution=resolution, origin_x=origin_x, origin_y=origin_y), data, occupied_threshold=99)


def test_phase70_balance_first_gate_applies_when_balance_improves_and_safety_floor_holds():
    from tugbot_maze.maze_perception import refine_corridor_centerline_target

    result = refine_corridor_centerline_target(
        map_grid=_corridor_grid(),
        local_cost_grid=_local_cost_grid(),
        dispatch_pose=(0.0, 0.0, 1.57079632679),
        original_target=(0.35, 1.0),
        direction_rad=1.57079632679,
        clearance_radius_m=0.35,
        side_probe_m=1.5,
        forward_offsets_m=(0.0,),
        lateral_offsets_m=(-0.35, -0.20, 0.0, 0.20, 0.35),
        gate_mode='balance_first',
        balance_improvement_epsilon_m=0.04,
        min_clearance_floor_m=0.45,
        forward_progress_tolerance_m=0.05,
    )

    assert result['applied'] is True
    assert result['gate_mode'] == 'balance_first'
    assert result['reason'] == 'balance_first_applied'
    assert result['branch_scoring_changed'] is False
    assert result['original_target'] == [0.35, 1.0]
    assert abs(result['refined_target'][0]) < 0.10
    assert result['selected_metrics']['balance_error_m'] <= result['original_metrics']['balance_error_m'] - 0.04
    assert result['selected_metrics']['same_corridor'] is True
    assert result['selected_metrics']['two_side_wall_evidence'] is True
    assert result['selected_metrics']['target_has_clearance'] is True
    assert result['selected_metrics']['occupancy_free'] is True
    assert result['selected_metrics']['safety_floor_ok'] is True
    assert result['selected_metrics']['footprint_lethal_not_increased'] is True
    assert result['selected_metrics']['front_wedge_lethal_not_increased'] is True
    assert result['selected_metrics']['forward_progress_not_obviously_lowered'] is True
    assert result['selected_metrics']['balance_first_eligible'] is True
    assert result['balance_first_eligible_candidate_count'] >= 1
    assert result['gate_conditions']['balance_error_improved'] is True
    assert result['gate_conditions']['footprint_lethal_not_increased'] is True
    assert result['gate_conditions']['front_wedge_lethal_not_increased'] is True


def test_phase70_balance_first_rejects_when_safety_floor_or_lethal_cost_regresses():
    from tugbot_maze.maze_perception import refine_corridor_centerline_target

    too_close_to_wall = refine_corridor_centerline_target(
        map_grid=_corridor_grid(),
        local_cost_grid=_local_cost_grid(),
        dispatch_pose=(0.0, 0.0, 1.57079632679),
        original_target=(0.35, 1.0),
        direction_rad=1.57079632679,
        clearance_radius_m=0.35,
        side_probe_m=1.5,
        forward_offsets_m=(0.0,),
        lateral_offsets_m=(-0.45,),
        gate_mode='balance_first',
        min_clearance_floor_m=0.65,
    )
    assert too_close_to_wall['applied'] is False
    assert too_close_to_wall['reason'] in {'safety_floor_blocked', 'no_balance_first_candidate'}
    assert too_close_to_wall['refined_target'] == too_close_to_wall['original_target']

    lethal_regression = refine_corridor_centerline_target(
        map_grid=_corridor_grid(),
        local_cost_grid=_local_cost_grid(lethal_x_min=0.15),
        dispatch_pose=(0.0, 0.0, 1.57079632679),
        original_target=(-0.35, 1.0),
        direction_rad=1.57079632679,
        clearance_radius_m=0.35,
        side_probe_m=1.5,
        forward_offsets_m=(0.0,),
        lateral_offsets_m=(-0.55,),
        gate_mode='balance_first',
        min_clearance_floor_m=0.45,
    )
    assert lethal_regression['applied'] is False
    assert lethal_regression['reason'] in {'lethal_cost_regression', 'no_balance_first_candidate'}
    assert lethal_regression['refined_target'] == lethal_regression['original_target']


def test_phase70_maze_explorer_contract_for_balance_first_runtime_gate_and_dual_target_diagnostics():
    source = _read(MAZE_EXPLORER)
    for token in [
        'centerline_target_refinement_gate_mode',
        "balance_first",
        'centerline_target_refinement_min_clearance_floor_m',
        'centerline_target_refinement_forward_progress_tolerance_m',
        'balance_first_eligible_candidate_count',
        'footprint_lethal_not_increased',
        'front_wedge_lethal_not_increased',
        'original_target',
        'refined_target',
        'branch_scoring_changed',
    ]:
        assert token in source
    assert 'self.topology.choose_next_branch(node.node_id, exit_xy=(self.exit_x, self.exit_y))' in source


def test_phase70_wrapper_and_analyzer_contracts_for_bounded_runtime_smoke():
    wrapper = _read(WRAPPER)
    analyzer = _read(ANALYZER)
    assert f'RUN_ID="{RUN_ID}"' in wrapper
    assert f"RUN_ID = '{RUN_ID}'" in analyzer
    assert 'phase65_inner_ingress_waypoint_map=x=2.0,y=0.0,yaw=0.0' in wrapper
    assert 'MAX_GOALS="${PHASE70_MAX_GOALS:-1}"' in wrapper
    assert 'PHASE70_MAX_GOALS must be exactly 1' in wrapper
    assert '-p max_goals:="$MAX_GOALS"' in wrapper
    assert '-p centerline_target_refinement_enabled:=true' in wrapper
    assert '-p centerline_target_refinement_gate_mode:=balance_first' in wrapper
    assert 'phase70_runtime_timeline.jsonl' in wrapper
    assert 'phase70_controller_dynamics.jsonl' in wrapper
    assert 'phase70_local_costmap_samples.jsonl' in wrapper
    assert 'git diff -- src/tugbot_navigation/config' in wrapper
    assert 'cleanup_processes_after' in wrapper
    for guardrail in [
        'no Nav2/MPPI/controller parameter edits',
        'no inflation/robot_radius/clearance_radius_m/map threshold tuning',
        'no branch scoring change',
        'no corridor-following cmd_vel control',
        'no fallback/terminal acceptance change',
        'no autonomous exploration success claim',
        'no exit success claim',
        'bounded runtime only',
    ]:
        assert guardrail in wrapper
        assert guardrail in analyzer
    for classification in [
        'BALANCE_FIRST_CENTERLINE_APPLIED_AND_IMPROVES',
        'BALANCE_FIRST_APPLIED_TIMEOUT_REMAINS',
        'BALANCE_FIRST_GATE_NO_APPLY',
        'BALANCE_FIRST_REGRESSION',
        'INSUFFICIENT_EVIDENCE',
        'GUARDRAIL_VIOLATION_STRATEGY_CHANGED',
    ]:
        assert classification in analyzer


def test_phase70_analyzer_classifies_synthetic_applied_timeout_and_preserves_no_success_claims(tmp_path):
    module = _load_analyzer()
    replay_dir = tmp_path / 'replay_01'
    replay_dir.mkdir()
    events = [
        {
            'event': 'dispatch',
            'goal_sequence': 1,
            'target': [0.0, 1.0],
            'original_target': [0.35, 1.0],
            'refined_target': [0.0, 1.0],
            'centerline_refinement_applied': True,
            'centerline_refinement_reason': 'balance_first_applied',
            'centerline_target_refinement': {
                'applied': True,
                'gate_mode': 'balance_first',
                'selected_metrics': {
                    'balance_error_m': 0.0,
                    'min_clearance_m': 0.9,
                    'footprint_lethal_count': 0,
                    'front_wedge_lethal_count': 0,
                },
                'original_metrics': {
                    'balance_error_m': 0.7,
                    'min_clearance_m': 0.55,
                    'footprint_lethal_count': 0,
                    'front_wedge_lethal_count': 0,
                },
            },
        },
        {'event': 'timeout', 'goal_sequence': 1, 'result_reason': 'timeout'},
    ]
    (replay_dir / f'{RUN_ID}_replay_01_goal_events.jsonl').write_text('\n'.join(json.dumps(e) for e in events), encoding='utf-8')
    (replay_dir / f'{RUN_ID}_replay_01_inner_ingress_navigate_to_pose_action_result.json').write_text('{"success": true}', encoding='utf-8')
    (tmp_path / f'{RUN_ID}_preflight.txt').write_text('MAX_GOALS=1\ncenterline_target_refinement_gate_mode=balance_first\n', encoding='utf-8')
    (tmp_path / f'{RUN_ID}_nav2_config_diff.txt').write_text('', encoding='utf-8')
    (tmp_path / f'{RUN_ID}_cleanup_processes_after.txt').write_text('', encoding='utf-8')
    result = module.analyze_phase70(tmp_path, output=tmp_path / 'summary.json')
    assert result['run_id'] == RUN_ID
    assert result['complete_autonomous_success_claimed'] is False
    assert result['exit_success_claimed'] is False
    assert result['branch_scoring_changed'] is False
    assert result['balance_first_applied_count'] == 1
    assert result['classification'] == 'BALANCE_FIRST_APPLIED_TIMEOUT_REMAINS'


def test_phase70_report_contract_stop_condition_and_guardrails():
    report = _read(REPORT)
    assert 'Phase70' in report
    assert RUN_ID in report
    assert 'CENTERLINE_GATE_NO_APPLY' in report
    assert 'balance-first' in report or 'balance_first' in report
    assert 'original_target' in report and 'refined_target' in report
    assert '不宣称 autonomous exploration success' in report or 'no autonomous exploration success claim' in report
    assert '不宣称 exit success' in report or 'no exit success claim' in report
    assert '不进入 Phase71' in report or 'Do not enter Phase71' in report
