from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
ANALYZER = ROOT / 'tools' / 'analyze_phase60_single_open_direction_dead_end_policy_review.py'
REPORT = ROOT / 'doc' / 'doc_report' / 'phase60_single_open_direction_dead_end_policy_review_report.md'
MAZE_EXPLORER = ROOT / 'src' / 'tugbot_maze' / 'tugbot_maze' / 'maze_explorer.py'
MAZE_PERCEPTION = ROOT / 'src' / 'tugbot_maze' / 'tugbot_maze' / 'maze_perception.py'
RUN_ID = 'phase60_single_open_direction_dead_end_policy_review'


def _read(path: Path) -> str:
    assert path.exists(), f'missing {path}'
    return path.read_text(encoding='utf-8')


def test_phase60_analyzer_exists_and_is_read_only_artifact_review():
    analyzer = _read(ANALYZER)
    assert f"RUN_ID = '{RUN_ID}'" in analyzer
    assert f"ARTIFACT_DIR = Path('log') / RUN_ID" in analyzer
    assert 'phase58_post_ingress_candidate_formation_stability_replay' in analyzer
    assert 'phase59_post_ingress_topology_consistency_guard' in analyzer
    assert 'NO_RUNTIME_LAUNCH' in analyzer
    assert 'read_only_review' in analyzer
    forbidden_runtime_terms = [
        'ros2 launch',
        'NavigateToPose.Goal',
        'send_goal_async',
        'gz sim',
        'startup_warmup_no_dispatch:=false',
    ]
    for term in forbidden_runtime_terms:
        assert term not in analyzer


def test_phase60_static_policy_review_names_single_open_dead_end_chain():
    analyzer = _read(ANALYZER)
    perception = _read(MAZE_PERCEPTION)
    explorer = _read(MAZE_EXPLORER)
    assert 'elif count == 1:' in perception
    assert 'kind = DEAD_END' in perception
    assert "local.kind == DEAD_END" in explorer
    assert "filtered_open_directions=[]" in explorer
    assert "rejection_reason='dead_end_policy_no_branch_options'" in explorer
    for review_field in [
        'raw_open_direction_generation',
        'filtered_open_direction_filtering',
        'single_open_direction_dead_end_classification',
        'dead_end_policy_no_branch_options_trigger',
        'post_ingress_context_absent',
    ]:
        assert review_field in analyzer


def test_phase60_offline_replay_schema_includes_ingress_geometry_and_candidate_context():
    analyzer = _read(ANALYZER)
    for field in [
        'robot_pose_map',
        'robot_yaw_rad',
        'accepted_open_direction_angle_rad',
        'accepted_open_direction_vector',
        'angle_to_entrance_yaw_deg',
        'angle_to_ingress_vector_deg',
        'angle_to_return_to_entrance_deg',
        'roughly_points_into_maze',
        'opposite_return_to_entrance',
        'candidate_goal_point',
        'candidate_map_cell_state',
        'candidate_local_costmap_cell_state',
        'candidate_clearance_result',
        'dead_end_policy_state',
    ]:
        assert field in analyzer


def test_phase60_analyzer_classifications_and_candidate_strategy_options():
    analyzer = _read(ANALYZER)
    for classification in [
        'SINGLE_OPEN_DIRECTION_POLICY_REVIEW_COMPLETED',
        'SINGLE_OPEN_DIRECTION_MISCLASSIFIED_AS_DEAD_END',
        'SINGLE_OPEN_DIRECTION_POLICY_CURRENTLY_JUSTIFIED',
        'SINGLE_OPEN_DIRECTION_REVIEW_INCONCLUSIVE_NEEDS_RUNTIME_EVIDENCE',
        'GUARDRAIL_VIOLATION_STRATEGY_CHANGED',
    ]:
        assert classification in analyzer
    for option in [
        'keep_current_policy',
        'allow_single_open_direction_only_at_post_ingress',
        'allow_single_open_direction_if_not_backtracking',
        'require_multi_frame_single_open_direction_then_candidate',
        'require entrance/ingress context tag before first node',
    ]:
        assert option in analyzer


def test_phase60_report_contract_preserves_phase59_conclusions_and_guardrails():
    report = _read(REPORT)
    assert 'Phase60' in report
    assert 'Single Open Direction Dead-End Policy Review' in report
    assert RUN_ID in report
    assert 'CANDIDATE_FORMATION_UNSTABLE_DEAD_END_POLICY_SENSITIVE' in report
    assert 'TOPOLOGY_CONSISTENCY_CONFIRMED_NO_CANDIDATE' in report
    assert 'does not claim autonomous exploration success' in report or '不声明 autonomous exploration success' in report or '不得写成自主探索成功' in report
    assert 'first dispatch' in report and ('not exit success' in report or '不是出口成功' in report)
    assert 'No Gazebo/SLAM/Nav2 runtime was launched' in report or '未启动 Gazebo/SLAM/Nav2 runtime' in report
    assert 'Phase61' in report


def test_phase60_analyzer_writes_expected_json_and_markdown_outputs():
    analyzer = _read(ANALYZER)
    assert "phase60_single_open_direction_dead_end_policy_review.json" in analyzer
    assert "phase60_single_open_direction_dead_end_policy_review.md" in analyzer
    assert 'static_policy_review' in analyzer
    assert 'offline_replay_review' in analyzer
    assert 'candidate_strategy_review' in analyzer
    assert 'guardrails' in analyzer
