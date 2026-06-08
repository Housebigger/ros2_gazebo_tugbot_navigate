from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
MAZE_EXPLORER = ROOT / 'src' / 'tugbot_maze' / 'tugbot_maze' / 'maze_explorer.py'
WRAPPER = ROOT / 'tools' / 'run_phase61_post_ingress_single_open_candidate_exception.sh'
ANALYZER = ROOT / 'tools' / 'analyze_phase61_post_ingress_single_open_candidate_exception.py'
REPORT = ROOT / 'doc' / 'doc_report' / 'phase61_post_ingress_single_open_candidate_exception_report.md'
RUN_ID = 'phase61_post_ingress_single_open_candidate_exception'


def _read(path: Path) -> str:
    assert path.exists(), f'missing {path}'
    return path.read_text(encoding='utf-8')


def test_phase61_maze_explorer_declares_bounded_single_open_exception_parameters_and_state():
    source = _read(MAZE_EXPLORER)
    assert "declare_parameter('post_ingress_single_open_exception_enabled', True)" in source
    assert "declare_parameter('post_ingress_single_open_exception_return_angle_min_deg'" in source
    assert 'self.post_ingress_single_open_exception_consumed' in source
    assert 'self.last_post_ingress_single_open_exception_diagnostics' in source
    assert 'post_ingress_context_active' in source
    assert 'first_post_ingress_topology_node' in source


def test_phase61_exception_scoped_to_dead_end_single_open_after_consistency_and_first_node_only():
    source = _read(MAZE_EXPLORER)
    assert '_maybe_apply_post_ingress_single_open_exception' in source
    assert '_build_post_ingress_single_open_exception_diagnostics' in source
    assert '_post_ingress_single_open_context_active' in source
    assert '_single_open_exception_multi_frame_confirmed' in source
    exception_block = source[
        source.index('def _maybe_apply_post_ingress_single_open_exception'):
        source.index('def _record_topology_consistency_frame')
    ]
    for required in [
        'local.kind == DEAD_END',
        'len(local.open_directions) == 1',
        'self.post_ingress_single_open_exception_consumed',
        'self._single_open_exception_multi_frame_confirmed()',
        'angle_to_return_entrance',
        'candidate_map_cell_state',
        'candidate_local_costmap_cell_state',
        'make_centered_branch_goal',
        'BranchOption',
    ]:
        assert required in exception_block
    assert "single_open_exception_reason = 'not_first_post_ingress_topology_node'" in exception_block
    assert "single_open_exception_reason = 'not_multi_frame_confirmed'" in exception_block
    assert "single_open_exception_reason = 'aligned_with_return_to_entrance'" in exception_block
    assert "single_open_exception_reason = 'candidate_map_not_clear'" in exception_block


def test_phase61_exception_enters_existing_dispatch_path_without_branch_scoring_changes():
    source = _read(MAZE_EXPLORER)
    dead_end_block = source[
        source.index('if local.kind == DEAD_END:'):
        source.index('branch_options = []', source.index('if local.kind == DEAD_END:'))
    ]
    assert '_maybe_apply_post_ingress_single_open_exception(' in dead_end_block
    assert 'candidate_selected_for_dispatch' in dead_end_block
    assert '_send_goal(chosen.target_xy' in dead_end_block
    assert 'self.topology.choose_next_branch(node.node_id, exit_xy=(self.exit_x, self.exit_y))' in dead_end_block
    topology = _read(ROOT / 'src' / 'tugbot_maze' / 'tugbot_maze' / 'maze_topology.py')
    assert 'def choose_next_branch(self, node_id: int, exit_xy: Point) -> Optional[BranchOption]:' in topology
    assert 'return max(candidates, key=lambda branch: branch.score_for_exit(node.xy, exit_xy, self.exit_bias_weight))' in topology


def test_phase61_structured_diagnostics_schema_in_state_and_goal_events():
    source = _read(MAZE_EXPLORER)
    for field in [
        'post_ingress_single_open_exception',
        'post_ingress_context_active',
        'first_post_ingress_topology_node',
        'single_open_exception_eligible',
        'single_open_exception_applied',
        'single_open_exception_reason',
        'angle_to_return_entrance',
        'multi_frame_single_open_confirmed',
        'single_open_exception_candidate_recovered_during_consistency_window',
        'candidate_map_cell_state',
        'candidate_local_costmap_cell_state',
        'single_open_exception_dispatch_produced',
    ]:
        assert field in source


def test_phase61_wrapper_and_analyzer_contract():
    wrapper = _read(WRAPPER)
    analyzer = _read(ANALYZER)
    joined = wrapper + '\n' + analyzer
    assert f'RUN_ID="{RUN_ID}"' in wrapper
    assert f"RUN_ID = '{RUN_ID}'" in analyzer
    assert f'log/${{RUN_ID}}' in wrapper
    assert 'tugbot_maze_world_20260528_clean_scaled2x.sdf' in joined
    assert 'maze_20260528_scaled_instance.yaml' in joined
    assert 'ingress_waypoint_map=x=${INGRESS_X},y=${INGRESS_Y},yaw=${INGRESS_YAW}' in wrapper
    assert '-p max_goals:=1' in wrapper
    assert '-p near_exit_fallback_enabled:=false' in wrapper
    assert '-p post_ingress_single_open_exception_enabled:=true' in wrapper
    assert 'PHASE61_REPLAY_COUNT' in wrapper
    assert 'REPLAY_COUNT' in wrapper
    assert 'bounded replay only' in joined
    assert 'no Nav2/MPPI/controller parameter edits' in joined
    assert 'no clearance_radius_m tuning' in joined
    assert 'no map sufficiency threshold tuning' in joined
    assert 'no ordinary branch selection scoring change' in joined
    assert 'no fallback/terminal acceptance' in joined
    assert 'no autonomous exploration success claim' in joined
    assert 'first dispatch is not exit success' in joined


def test_phase61_analyzer_classifications_and_metrics():
    analyzer = _read(ANALYZER)
    for classification in [
        'SINGLE_OPEN_EXCEPTION_IMPLEMENTED_STATIC_ONLY',
        'SINGLE_OPEN_EXCEPTION_APPLIED_AND_DISPATCHED',
        'SINGLE_OPEN_EXCEPTION_NOT_ELIGIBLE_CONFIRMED_DEAD_END',
        'SINGLE_OPEN_EXCEPTION_INCONCLUSIVE_RUNTIME_VARIANCE',
        'GUARDRAIL_VIOLATION_STRATEGY_CHANGED',
    ]:
        assert classification in analyzer
    for metric in [
        'post_ingress_context_active',
        'first_post_ingress_topology_node',
        'single_open_exception_eligible',
        'single_open_exception_applied',
        'single_open_exception_reason',
        'angle_to_return_entrance',
        'multi_frame_single_open_confirmed',
        'candidate_map_cell_state',
        'candidate_local_costmap_cell_state',
        'single_open_exception_dispatch_produced',
        'dispatch_observed',
        'first_dispatch_observed_not_exit_success',
        'complete_autonomous_success_claimed',
    ]:
        assert metric in analyzer


def test_phase61_report_contract_preserves_prior_conclusions_and_stop_condition():
    report = _read(REPORT)
    assert 'Phase61' in report
    assert 'Post-Ingress Single-Open-Direction Candidate Exception' in report
    assert RUN_ID in report
    assert 'SINGLE_OPEN_DIRECTION_MISCLASSIFIED_AS_DEAD_END' in report
    assert 'CANDIDATE_FORMATION_UNSTABLE_DEAD_END_POLICY_SENSITIVE' in report
    assert 'TOPOLOGY_CONSISTENCY_CONFIRMED_NO_CANDIDATE' in report
    assert 'does not claim autonomous exploration success' in report or '不声明 autonomous exploration success' in report or '不得写成自主探索成功' in report
    assert 'first dispatch' in report and ('not exit success' in report or '不是出口成功' in report)
    assert '不进入 Phase62' in report or 'Do not enter Phase62' in report
