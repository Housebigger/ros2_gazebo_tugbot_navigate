import ast
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'src' / 'tugbot_maze'))

from tugbot_maze.maze_topology import BranchOption, MazeTopology

MAZE_EXPLORER = ROOT / 'src' / 'tugbot_maze' / 'tugbot_maze' / 'maze_explorer.py'
WRAPPER = ROOT / 'tools' / 'run_phase21_controller_diagnostics_smoke.sh'
COVERAGE_TOOL = ROOT / 'tools' / 'check_phase26e_branch_diagnostics_coverage.py'


def test_phase26e_diagnostics_rank_matches_existing_maze_topology_choice_formula():
    topology = MazeTopology(junction_merge_radius_m=0.5, exit_bias_weight=0.5)
    node = topology.find_or_create_node(0.0, 0.0, node_type='junction')
    topology.set_branch_options(
        node.node_id,
        [
            BranchOption(angle_rad=3.14, target_xy=(-1.0, 0.0)),
            BranchOption(angle_rad=0.0, target_xy=(1.0, 0.0)),
            BranchOption(angle_rad=1.57, target_xy=(0.0, 1.0)),
        ],
    )
    chosen = topology.choose_next_branch(node.node_id, exit_xy=(4.0, 0.0))
    assert chosen is not None
    ranked = sorted(
        [branch for branch in node.branches if branch.state == 'untried'],
        key=lambda branch: branch.score_for_exit(node.xy, (4.0, 0.0), topology.exit_bias_weight),
        reverse=True,
    )
    assert chosen is ranked[0]
    assert chosen.target_xy == (1.0, 0.0)


def test_phase26e_goal_event_payload_contains_branch_choice_diagnostics_fields():
    source = MAZE_EXPLORER.read_text(encoding='utf-8')
    ast.parse(source)
    publish_body = source[source.index('payload = {'):source.index('self.goal_events_pub.publish')]
    required_payload_fields = [
        "'chosen_branch_rank': context.get('chosen_branch_rank')",
        "'chosen_branch_score_components': context.get('chosen_branch_score_components')",
        "'candidate_branch_count': context.get('candidate_branch_count')",
        "'candidate_branches': context.get('candidate_branches')",
        "'selected_due_to_context': context.get('selected_due_to_context')",
    ]
    for snippet in required_payload_fields:
        assert snippet in publish_body


def test_phase26e_branch_choice_diagnostics_are_computed_before_dispatch_without_changing_choice_order():
    source = MAZE_EXPLORER.read_text(encoding='utf-8')
    analyze_body = source[source.index('def _analyze_and_dispatch'):source.index('def _send_goal')]
    assert '_build_branch_choice_diagnostics' in source
    assert '_branch_option_payload' in source
    assert 'chosen_branch_rank' in source
    assert 'candidate_branches' in source
    assert 'selected_due_to_context' in source

    choose_pos = analyze_body.index('chosen = self.topology.choose_next_branch')
    diagnostics_pos = analyze_body.index('self.pending_branch_choice_diagnostics = self._build_branch_choice_diagnostics')
    send_pos = analyze_body.index("self._send_goal(chosen.target_xy")
    assert choose_pos < diagnostics_pos < send_pos

    # The Phase26E diagnostics must observe the already selected branch; they must
    # not replace MazeTopology.choose_next_branch with a new scoring/selection path.
    assert analyze_body.count('self.topology.choose_next_branch') == 1
    assert 'max(branch_options' not in analyze_body
    assert 'sorted(branch_options' not in analyze_body


def test_phase26e_candidate_branch_payload_schema_documents_required_fields():
    source = MAZE_EXPLORER.read_text(encoding='utf-8')
    helper_body = source[source.index('def _branch_option_payload'):source.index('def _send_goal')]
    required = [
        "'branch_angle'",
        "'target'",
        "'target_exit_dist'",
        "'exit_progress_delta_m'",
        "'target_clearance_m'",
        "'path_corridor_min_clearance_m'",
        "'dispatch_path_local_cost_max'",
        "'dispatch_path_local_cost_mean'",
        "'target_local_cost'",
        "'target_local_cost_max_radius'",
        "'is_reverse_candidate'",
        "'is_backtrack_context'",
        "'is_near_exit_candidate'",
        "'rejection_reason'",
    ]
    for token in required:
        assert token in helper_body


def test_phase26e_smoke_wrapper_accepts_diagnostics_only_run_id():
    source = WRAPPER.read_text(encoding='utf-8')
    assert 'phase26e_branch_diagnostics_smoke' in source
    assert 'phase26e_branch_diagnostics_smoke' in source[source.index('RUN_ID_PATTERN'):source.index('ROOT_DIR=')]


def test_phase26e_coverage_tool_is_diagnostics_only_and_checks_required_fields():
    source = COVERAGE_TOOL.read_text(encoding='utf-8')
    ast.parse(source)
    for field in [
        'chosen_branch_rank',
        'chosen_branch_score_components',
        'candidate_branch_count',
        'candidate_branches',
        'selected_due_to_context',
        'target_local_cost_max_radius',
        'path_corridor_min_clearance_m',
    ]:
        assert field in source
    assert 'coverage check only' in source
    assert 'do not use this smoke to promote or reject' in source
