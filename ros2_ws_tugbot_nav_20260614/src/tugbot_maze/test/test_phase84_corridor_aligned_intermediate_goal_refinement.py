from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'src' / 'tugbot_maze'))

from tugbot_maze.grid_utils import OccupancyGridInfo, OccupancyGridView
from tugbot_maze.maze_perception import refine_corridor_centerline_target

MAZE_EXPLORER = ROOT / 'src' / 'tugbot_maze' / 'tugbot_maze' / 'maze_explorer.py'
MAZE_PERCEPTION = ROOT / 'src' / 'tugbot_maze' / 'tugbot_maze' / 'maze_perception.py'
REPORT = ROOT / 'doc' / 'doc_report' / 'phase84_corridor_aligned_intermediate_goal_refinement_minimal_implementation_report.md'
CLEANUP_SUMMARY = ROOT / 'log' / 'phase84_corridor_aligned_intermediate_goal_refinement' / 'phase84_process_cleanup_summary.md'


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
        wy = origin_y + (y + 0.5) * resolution
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
            # Execution-cost proxy: lower near corridor centerline, high near side walls.
            cost = min(100, int(max(0.0, abs(wx) - 0.05) * 80.0))
            data.append(cost)
    return OccupancyGridView(OccupancyGridInfo(width=width, height=height, resolution=resolution, origin_x=origin_x, origin_y=origin_y), data, occupied_threshold=99)


def test_phase84_refinement_applies_projected_target_heading_and_forward_check():
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
        gate_mode='balance_first',
        min_clearance_floor_m=0.35,
        forward_progress_tolerance_m=0.05,
    )

    assert result['refinement_applied'] is True
    assert result['refinement_reject_reason'] is None
    assert result['original_target'] == [0.45, 1.0]
    assert result['centerline_projected_target'] == result['refined_target']
    assert abs(result['centerline_projected_target'][0]) < 0.08
    assert abs(result['corridor_heading_yaw'] - 1.57079632679) < 1e-9
    assert result['forward_executability_check']['checked'] is True
    assert result['forward_executability_check']['passed'] is True
    assert result['forward_executability_check']['same_corridor'] is True
    assert result['forward_executability_check']['target_has_clearance'] is True
    assert result['branch_scoring_changed'] is False
    assert result['fallback_terminal_acceptance_used'] is False


def test_phase84_refinement_rejects_and_preserves_original_target_when_evidence_missing():
    result = refine_corridor_centerline_target(
        map_grid=_one_sided_grid(),
        local_cost_grid=_local_cost_grid(),
        dispatch_pose=(0.0, 0.0, 1.57079632679),
        original_target=(0.45, 1.0),
        direction_rad=1.57079632679,
        clearance_radius_m=0.35,
        side_probe_m=1.5,
        forward_offsets_m=(0.0, 0.2),
        lateral_offsets_m=(-0.45, 0.0, 0.45),
        gate_mode='balance_first',
        min_clearance_floor_m=0.35,
        forward_progress_tolerance_m=0.05,
    )

    assert result['refinement_applied'] is False
    assert result['refined_target'] == result['original_target']
    assert result['centerline_projected_target'] is None
    assert result['refinement_reject_reason'] == 'missing_two_side_wall_evidence'
    assert result['forward_executability_check']['checked'] is True
    assert result['forward_executability_check']['passed'] is False
    assert result['forward_executability_check']['reason'] == 'missing_two_side_wall_evidence'
    assert result['branch_scoring_changed'] is False
    assert result['fallback_terminal_acceptance_used'] is False


def test_phase84_maze_explorer_goal_events_publish_required_refinement_fields():
    source = _read(MAZE_EXPLORER)
    send_goal = source[source.index('def _send_goal'):source.index('def _goal_feedback_callback')]
    for token in [
        'centerline_projected_target',
        'corridor_heading_yaw',
        'refinement_applied',
        'refinement_reject_reason',
        'forward_executability_check',
        'branch_scoring_changed',
        'fallback_terminal_acceptance_used',
    ]:
        assert token in send_goal
    assert "goal_kind != 'explore'" in source
    assert 'self.topology.choose_next_branch(node.node_id, exit_xy=(self.exit_x, self.exit_y))' in source
    assert 'goal_msg.pose = self._make_pose_stamped(target_xy, goal_yaw)' in send_goal


def test_phase84_guardrails_no_strategy_or_parameter_tuning_contract():
    explorer = _read(MAZE_EXPLORER)
    perception = _read(MAZE_PERCEPTION)
    cleanup = _read(CLEANUP_SUMMARY)

    assert 'score_for_exit' in explorer
    assert 'Phase26E deliberately calls this only after MazeTopology.choose_next_branch()' in explorer
    assert 'branch_scoring_changed' in perception and "'branch_scoring_changed': False" in perception
    assert 'fallback_terminal_acceptance_used' in perception and "'fallback_terminal_acceptance_used': False" in perception
    forbidden_runtime_tokens = [
        'evaluate_footprint_terminal_acceptance',
        'terminal_acceptance_margin_m',
        'near_exit_fallback_enabled',
    ]
    for token in forbidden_runtime_tokens:
        assert token not in perception
    assert 'gz sim' in cleanup and 'rviz2' in cleanup and 'controller_server' in cleanup
    assert 'remaining_matches: 0' in cleanup


def test_phase84_report_records_cleanup_validation_and_stop_condition():
    report = _read(REPORT)
    for token in [
        'Phase84',
        'Corridor-aligned intermediate goal refinement minimal implementation',
        'phase84_process_cleanup_summary.md',
        'original_target',
        'centerline_projected_target',
        'corridor_heading_yaw',
        'refinement_applied',
        'refinement_reject_reason',
        'forward_executability_check',
        'branch_scoring_changed=false',
        'fallback_terminal_acceptance_used=false',
        'No Nav2/MPPI/controller parameter tuning',
        'No branch scoring change',
        'No centerline gate change',
        'No directional readiness override change',
        'No fallback or terminal acceptance change',
        'No autonomous exploration success claim',
        'No exit success claim',
        'Stop: do not enter Phase85',
    ]:
        assert token in report
