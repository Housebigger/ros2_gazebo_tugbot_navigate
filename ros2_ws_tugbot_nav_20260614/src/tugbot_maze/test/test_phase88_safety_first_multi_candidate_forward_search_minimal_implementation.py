from __future__ import annotations

from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'src' / 'tugbot_maze'))

from tugbot_maze.grid_utils import OccupancyGridInfo, OccupancyGridView
from tugbot_maze.maze_perception import refine_corridor_centerline_target

MAZE_EXPLORER = ROOT / 'src' / 'tugbot_maze' / 'tugbot_maze' / 'maze_explorer.py'
MAZE_PERCEPTION = ROOT / 'src' / 'tugbot_maze' / 'tugbot_maze' / 'maze_perception.py'
PHASE88_REPORT = ROOT / 'doc' / 'doc_report' / 'phase88_safety_first_multi_candidate_forward_search_minimal_implementation_report.md'


def _read(path: Path) -> str:
    assert path.exists(), f'missing {path}'
    return path.read_text(encoding='utf-8')


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
            if abs(wx) >= 0.95:
                data[x + y * width] = 100
    return OccupancyGridView(info, data, occupied_threshold=65)


def _one_sided_grid():
    grid = _corridor_grid()
    width = grid.info.width
    grid.data = [0 if value == 100 and idx % width < width // 2 else value for idx, value in enumerate(grid.data)]
    return grid


def _flat_local_cost_grid(width_m=4.0, height_m=4.0, resolution=0.05, default_cost=10):
    width = int(width_m / resolution)
    height = int(height_m / resolution)
    origin_x = -width_m / 2.0
    origin_y = -height_m / 2.0
    data = [int(default_cost)] * (width * height)
    return OccupancyGridView(
        OccupancyGridInfo(width=width, height=height, resolution=resolution, origin_x=origin_x, origin_y=origin_y),
        data,
        occupied_threshold=99,
    )


def _center_high_cost_grid(width_m=4.0, height_m=4.0, resolution=0.05):
    width = int(width_m / resolution)
    height = int(height_m / resolution)
    origin_x = -width_m / 2.0
    origin_y = -height_m / 2.0
    data = []
    for y in range(height):
        wy = origin_y + (y + 0.5) * resolution
        for x in range(width):
            wx = origin_x + (x + 0.5) * resolution
            # Non-lethal execution-risk band on the exact centerline.  It must
            # lose to a less-centered low-cost candidate once Phase88 ranks
            # clearance / local-cost risk before balance error.
            if abs(wx) <= 0.08 and 0.2 <= wy <= 1.8:
                data.append(85)
            else:
                data.append(10)
    return OccupancyGridView(
        OccupancyGridInfo(width=width, height=height, resolution=resolution, origin_x=origin_x, origin_y=origin_y),
        data,
        occupied_threshold=99,
    )


def test_phase88_candidate_family_generation_contract_includes_offsets_and_heading_variants():
    result = refine_corridor_centerline_target(
        map_grid=_corridor_grid(),
        local_cost_grid=_flat_local_cost_grid(),
        dispatch_pose=(0.0, 0.0, 1.57079632679),
        original_target=(0.35, 1.0),
        direction_rad=1.57079632679,
        clearance_radius_m=0.35,
        side_probe_m=1.5,
        forward_offsets_m=(0.0, 0.1),
        lateral_offsets_m=(-0.15, 0.0, 0.15),
        heading_offsets_rad=(-0.05, 0.0, 0.05),
        gate_mode='balance_first',
        min_clearance_floor_m=0.35,
        local_cost_radius_m=0.04,
        front_wedge_radius_m=0.05,
    )

    assert result['candidate_count'] == 18
    family = result['candidate_family']
    assert family['centerline_projection'] is True
    assert family['bounded_local_search'] is True
    assert family['lateral_offsets_m'] == [-0.15, 0.0, 0.15]
    assert family['forward_offsets_m'] == [0.0, 0.1]
    assert family['heading_offsets_rad'] == [-0.05, 0.0, 0.05]
    assert result['multi_candidate_forward_search']['candidate_family'] == family
    assert {round(candidate['heading_offset_rad'], 2) for candidate in result['candidates']} == {-0.05, 0.0, 0.05}
    assert all('target_yaw' in candidate for candidate in result['candidates'])


def test_phase88_safety_first_selection_makes_best_balance_yield_to_lower_risk_candidate():
    result = refine_corridor_centerline_target(
        map_grid=_corridor_grid(),
        local_cost_grid=_center_high_cost_grid(),
        dispatch_pose=(0.0, 0.0, 1.57079632679),
        original_target=(0.35, 1.0),
        direction_rad=1.57079632679,
        clearance_radius_m=0.35,
        side_probe_m=1.5,
        forward_offsets_m=(0.0,),
        # lateral_offset 0.35 is exact centerline x ~= 0.0 with best balance but high local cost.
        # lateral_offset 0.15 gives x ~= 0.20 with worse balance but lower local-cost risk.
        lateral_offsets_m=(0.35, 0.15),
        heading_offsets_rad=(0.0,),
        gate_mode='balance_first',
        min_clearance_floor_m=0.35,
        forward_progress_tolerance_m=0.05,
        local_cost_radius_m=0.04,
        front_wedge_radius_m=0.05,
        high_cost_threshold=70,
    )

    assert result['refinement_applied'] is True
    selected = result['selected_metrics']
    assert selected['hard_safety_pass'] is True
    assert selected['local_cost_max_radius'] == 10
    assert abs(selected['target'][0] - 0.20) < 0.08
    assert selected['balance_error_m'] > 0.0
    best_balance = min(result['candidates'], key=lambda row: row['balance_error_m'])
    assert abs(best_balance['target'][0]) < 0.08
    assert best_balance['local_cost_max_radius'] == 85
    assert result['selected_candidate_index'] == selected['candidate_index']
    assert result['selected_candidate_target'] == selected['target']
    assert 'clearance better' in ' -> '.join(result['selection_priority_trace'])
    assert 'balance error smaller' in ' -> '.join(result['selection_priority_trace'])


def test_phase88_reject_preserves_original_target_when_no_candidate_passes_hard_safety():
    result = refine_corridor_centerline_target(
        map_grid=_one_sided_grid(),
        local_cost_grid=_flat_local_cost_grid(),
        dispatch_pose=(0.0, 0.0, 1.57079632679),
        original_target=(0.35, 1.0),
        direction_rad=1.57079632679,
        clearance_radius_m=0.35,
        side_probe_m=1.5,
        forward_offsets_m=(0.0, 0.1),
        lateral_offsets_m=(-0.15, 0.0, 0.15),
        heading_offsets_rad=(-0.05, 0.0, 0.05),
        gate_mode='balance_first',
        min_clearance_floor_m=0.35,
    )

    assert result['refinement_applied'] is False
    assert result['refined_target'] == result['original_target']
    assert result['centerline_projected_target'] is None
    assert result['selected_candidate_index'] is None
    assert result['selected_candidate_target'] is None
    assert result['selected_candidate_yaw'] is None
    assert result['original_target_preserved_on_reject'] is True
    assert result['refinement_reject_reason'] == 'missing_two_side_wall_evidence'
    assert result['hard_safety_pass_candidate_count'] == 0
    assert result['multi_candidate_forward_search']['original_target_preserved_on_reject'] is True
    assert result['rejected_candidate_summaries']


def test_phase88_goal_events_contract_is_exposed_at_dispatch_top_level():
    source = _read(MAZE_EXPLORER)
    send_goal = source[source.index('def _send_goal'):source.index('def _goal_feedback_callback')]
    empty = source[source.index('def _empty_centerline_target_refinement_diagnostics'):source.index('def _maybe_refine_corridor_centerline_dispatch_target')]
    maybe = source[source.index('def _maybe_refine_corridor_centerline_dispatch_target'):source.index('def _send_goal')]

    for token in [
        'multi_candidate_forward_search',
        'candidate_family',
        'candidate_count',
        'hard_safety_pass_candidate_count',
        'selected_candidate_index',
        'selected_candidate_target',
        'selected_candidate_yaw',
        'selection_priority_trace',
        'rejected_candidate_summaries',
        'original_target_preserved_on_reject',
        'branch_scoring_changed',
        'fallback_terminal_acceptance_used',
    ]:
        assert token in send_goal
        assert token in empty
        assert token in maybe
    assert 'goal_kind != \'explore\'' in source
    assert 'self.topology.choose_next_branch(node.node_id, exit_xy=(self.exit_x, self.exit_y))' in source


def test_phase88_guardrails_and_report_contract():
    perception = _read(MAZE_PERCEPTION)
    report = _read(PHASE88_REPORT)

    for required_source_token in [
        'hard_safety_pass',
        'footprint_lethal_not_increased',
        'front_wedge_lethal_not_increased',
        'safety_floor_ok',
        'forward_progress_ok',
        'clearance better',
        'balance error smaller',
    ]:
        assert required_source_token in perception

    for report_token in [
        'Phase88',
        'Safety-first multi-candidate forward search minimal implementation',
        'No inflation/robot_radius/clearance_radius_m/MPPI/controller/map threshold tuning',
        'No branch scoring changed',
        'No exploration order changed',
        'No centerline gate changed',
        'No directional readiness changed',
        'No fallback/terminal acceptance changed',
        'No autonomous exploration success claimed',
        'No exit success claimed',
        'Stop: do not enter Phase89',
    ]:
        assert report_token in report
