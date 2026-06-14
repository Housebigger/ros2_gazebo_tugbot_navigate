from pathlib import Path
import importlib.util
import sys

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'src' / 'tugbot_maze'))

from tugbot_maze.grid_utils import OccupancyGridInfo, OccupancyGridView

MAZE_EXPLORER = ROOT / 'src' / 'tugbot_maze' / 'tugbot_maze' / 'maze_explorer.py'
MAZE_PERCEPTION = ROOT / 'src' / 'tugbot_maze' / 'tugbot_maze' / 'maze_perception.py'
WRAPPER = ROOT / 'tools' / 'run_phase69_corridor_centerline_target_selection_runtime_gate.sh'
ANALYZER = ROOT / 'tools' / 'analyze_phase69_corridor_centerline_target_selection_runtime_gate.py'
REPORT = ROOT / 'doc' / 'doc_report' / 'phase69_corridor_centerline_target_selection_runtime_gate_report.md'
RUN_ID = 'phase69_corridor_centerline_target_selection_runtime_gate'


def _read(path: Path) -> str:
    assert path.exists(), f'missing {path}'
    return path.read_text(encoding='utf-8')


def _load_analyzer():
    spec = importlib.util.spec_from_file_location('phase69_analyzer', ANALYZER)
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
        wy = origin_y + (y + 0.5) * resolution
        for x in range(width):
            wx = origin_x + (x + 0.5) * resolution
            # vertical corridor along +Y, walls at x ~= +/-1.0
            if abs(wx) >= 0.95:
                data[x + y * width] = 100
    return OccupancyGridView(info, data, occupied_threshold=65)


def _local_cost_grid(width_m=4.0, height_m=4.0, resolution=0.05):
    width = int(width_m / resolution)
    height = int(height_m / resolution)
    origin_x = -width_m / 2.0
    origin_y = -height_m / 2.0
    data = []
    for y in range(height):
        wy = origin_y + (y + 0.5) * resolution
        for x in range(width):
            wx = origin_x + (x + 0.5) * resolution
            # Runtime local-cost proxy: cost increases near side walls.
            cost = min(100, int(max(0.0, abs(wx) - 0.05) * 80.0))
            data.append(cost)
    return OccupancyGridView(OccupancyGridInfo(width=width, height=height, resolution=resolution, origin_x=origin_x, origin_y=origin_y), data, occupied_threshold=99)


def test_phase69_maze_perception_has_pure_runtime_refinement_gate_and_recovers_centerline_candidate():
    source = _read(MAZE_PERCEPTION)
    assert 'def refine_corridor_centerline_target' in source
    assert 'same_corridor' in source
    assert 'two_side_wall_evidence' in source
    assert 'forward_progress_not_lowered' in source
    assert 'front_wedge_high_cost_count' in source

    from tugbot_maze.maze_perception import refine_corridor_centerline_target

    result = refine_corridor_centerline_target(
        map_grid=_corridor_grid(),
        local_cost_grid=_local_cost_grid(),
        dispatch_pose=(0.0, 0.0, 1.57079632679),
        original_target=(0.45, 1.0),
        direction_rad=1.57079632679,
        clearance_radius_m=0.35,
        side_probe_m=1.5,
        forward_offsets_m=(0.0, 0.2),
        lateral_offsets_m=(-0.45, 0.0, 0.45),
    )
    assert result['applied'] is True
    assert result['original_target'] == [0.45, 1.0]
    assert abs(result['refined_target'][0]) < 0.08
    assert result['refined_target'][1] >= result['original_metrics']['forward_progress_m'] - 1e-6
    assert result['selected_metrics']['same_corridor'] is True
    assert result['selected_metrics']['two_side_wall_evidence'] is True
    assert result['selected_metrics']['balance_error_m'] < result['original_metrics']['balance_error_m']
    assert result['selected_metrics']['min_clearance_m'] >= result['original_metrics']['min_clearance_m']
    assert result['selected_metrics']['local_cost_max_radius'] <= result['original_metrics']['local_cost_max_radius']
    assert result['selected_metrics']['front_wedge_high_cost_count'] <= result['original_metrics']['front_wedge_high_cost_count']


def test_phase69_refinement_gate_rejects_missing_two_side_wall_or_forward_regression():
    from tugbot_maze.maze_perception import refine_corridor_centerline_target

    open_grid = _corridor_grid()
    # Remove the left wall to make two-side-wall evidence false.
    open_grid.data = [0 if value == 100 and idx % open_grid.info.width < open_grid.info.width // 2 else value for idx, value in enumerate(open_grid.data)]
    no_wall_result = refine_corridor_centerline_target(
        map_grid=open_grid,
        local_cost_grid=_local_cost_grid(),
        dispatch_pose=(0.0, 0.0, 1.57079632679),
        original_target=(0.45, 1.0),
        direction_rad=1.57079632679,
        clearance_radius_m=0.35,
        side_probe_m=1.5,
        forward_offsets_m=(0.0, 0.2),
        lateral_offsets_m=(-0.45, 0.0, 0.45),
    )
    assert no_wall_result['applied'] is False
    assert no_wall_result['reason'] == 'missing_two_side_wall_evidence'
    assert no_wall_result['refined_target'] == no_wall_result['original_target']

    forward_regression_result = refine_corridor_centerline_target(
        map_grid=_corridor_grid(),
        local_cost_grid=_local_cost_grid(),
        dispatch_pose=(0.0, 0.0, 1.57079632679),
        original_target=(0.45, 1.0),
        direction_rad=1.57079632679,
        clearance_radius_m=0.35,
        side_probe_m=1.5,
        forward_offsets_m=(-0.4,),
        lateral_offsets_m=(-0.45, 0.0, 0.45),
    )
    assert forward_regression_result['applied'] is False
    assert forward_regression_result['reason'] in {'no_improving_candidate', 'forward_progress_regression'}


def test_phase69_maze_explorer_runtime_gate_contract_and_dual_target_diagnostics():
    source = _read(MAZE_EXPLORER)
    for param in [
        "centerline_target_refinement_enabled', True",
        'centerline_target_refinement_side_probe_m',
        'centerline_target_refinement_forward_offsets_m',
        'centerline_target_refinement_lateral_offsets_m',
    ]:
        assert param in source
    for token in [
        'refine_corridor_centerline_target',
        '_maybe_refine_corridor_centerline_dispatch_target',
        'centerline_target_refinement',
        'original_target',
        'refined_target',
        'centerline_refinement_applied',
        'centerline_refinement_reason',
        'branch_scoring_changed',
        'forward_progress_not_lowered',
    ]:
        assert token in source
    assert 'self.topology.choose_next_branch(node.node_id, exit_xy=(self.exit_x, self.exit_y))' in source
    assert 'score_for_exit' not in _read(WRAPPER) if WRAPPER.exists() else True


def test_phase69_wrapper_and_analyzer_contracts_for_bounded_runtime_smoke():
    wrapper = _read(WRAPPER)
    analyzer = _read(ANALYZER)
    assert f'RUN_ID="{RUN_ID}"' in wrapper
    assert f"RUN_ID = '{RUN_ID}'" in analyzer
    assert 'phase65_inner_ingress_waypoint_map=x=2.0,y=0.0,yaw=0.0' in wrapper
    assert 'MAX_GOALS="${PHASE69_MAX_GOALS:-1}"' in wrapper
    assert 'PHASE69_MAX_GOALS must be 1..2' in wrapper
    assert '-p max_goals:="$MAX_GOALS"' in wrapper
    assert '-p centerline_target_refinement_enabled:=true' in wrapper
    assert 'phase69_runtime_timeline.jsonl' in wrapper
    assert 'phase69_controller_dynamics.jsonl' in wrapper
    assert 'phase69_local_costmap_samples.jsonl' in wrapper
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
        'CENTERLINE_RUNTIME_GOAL1_IMPROVES',
        'CENTERLINE_RUNTIME_TIMEOUT_REMAINS',
        'CENTERLINE_GATE_NO_APPLY',
        'CENTERLINE_RUNTIME_REGRESSION',
        'INSUFFICIENT_EVIDENCE',
        'GUARDRAIL_VIOLATION_STRATEGY_CHANGED',
    ]:
        assert classification in analyzer


def test_phase69_analyzer_classifies_synthetic_replays_and_preserves_no_success_claims(tmp_path):
    module = _load_analyzer()
    replay_dir = tmp_path / 'replay_01'
    replay_dir.mkdir()
    events = [
        {
            'event': 'dispatch',
            'goal_sequence': 1,
            'target': [0.0, 1.2],
            'original_target': [0.45, 1.0],
            'refined_target': [0.0, 1.2],
            'centerline_refinement_applied': True,
            'centerline_refinement_reason': 'applied',
            'centerline_target_refinement': {'applied': True},
            'dispatch_target_local_cost_max_radius': 20,
            'phase62_front_wedge_cost': {'max': 30, 'high_cost_count': 0, 'lethal_count': 0},
            'phase62_target_footprint_cost': {'summary': {'max': 20, 'high_cost_count': 0, 'lethal_count': 0}},
        },
        {'event': 'success', 'goal_sequence': 1, 'result_reason': 'succeeded'},
    ]
    (replay_dir / f'{RUN_ID}_replay_01_goal_events.jsonl').write_text('\n'.join(__import__('json').dumps(e) for e in events), encoding='utf-8')
    (replay_dir / f'{RUN_ID}_replay_01_inner_ingress_navigate_to_pose_action_result.json').write_text('{"success": true}', encoding='utf-8')
    result = module.analyze_phase69(tmp_path, output=tmp_path / 'summary.json')
    assert result['run_id'] == RUN_ID
    assert result['complete_autonomous_success_claimed'] is False
    assert result['exit_success_claimed'] is False
    assert result['branch_scoring_changed'] is False
    assert result['centerline_refinement_applied_count'] == 1
    assert result['classification'] == 'CENTERLINE_RUNTIME_GOAL1_IMPROVES'


def test_phase69_report_contract_stop_condition_and_guardrails():
    report = _read(REPORT)
    assert 'Phase69' in report
    assert RUN_ID in report
    assert 'CENTERLINE_' in report
    assert 'original_target' in report and 'refined_target' in report
    assert '不宣称 autonomous exploration success' in report or 'no autonomous exploration success claim' in report
    assert '不宣称 exit success' in report or 'no exit success claim' in report
    assert '不进入 Phase70' in report or 'Do not enter Phase70' in report
