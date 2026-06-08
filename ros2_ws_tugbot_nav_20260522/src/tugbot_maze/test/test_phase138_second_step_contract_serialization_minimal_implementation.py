import importlib.util
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
MAZE_EXPLORER = ROOT / 'src' / 'tugbot_maze' / 'tugbot_maze' / 'maze_explorer.py'
ANALYZER = ROOT / 'tools' / 'analyze_phase138_second_step_contract_serialization_static.py'
REPORT = ROOT / 'doc' / 'doc_report' / 'phase138_second_step_contract_serialization_minimal_implementation_report.md'


def _read(path: Path) -> str:
    assert path.exists(), f'missing file: {path}'
    return path.read_text(encoding='utf-8')


def _load(path: Path, module_name: str):
    assert path.exists(), f'missing module: {path}'
    spec = importlib.util.spec_from_file_location(module_name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_phase138_pending_second_step_snapshot_is_runtime_serialized_with_required_origin_fields():
    text = _read(MAZE_EXPLORER)
    required = [
        'def _serialize_pending_corridor_alignment_second_step',
        "'exists': True",
        "'runtime_serialized': True",
        "'original_goal_kind': pending.get('original_goal_kind')",
        "'original_target': pending.get('original_target')",
        "'direction_rad': pending.get('direction_rad')",
        "'start_node_id': pending.get('start_node_id')",
        "'active_branch': self._branch_option_to_payload",
        "'staging_goal_pose': self._staging_plan_goal_pose",
        "'staging_plan': pending.get('staging_plan')",
        "'staging_result_status_label': pending.get('staging_result_status_label')",
        "'staging_succeeded_wall_time_sec': pending.get('staging_succeeded_wall_time_sec')",
    ]
    for phrase in required:
        assert phrase in text


def test_phase138_fresh_evidence_timestamps_are_recorded_after_staging_success():
    text = _read(MAZE_EXPLORER)
    required = [
        "pending['fresh_scan_sample_time_sec'] = self._now_wall_time_sec()",
        "pending['fresh_local_costmap_sample_time_sec'] = self._now_wall_time_sec()",
        "pending['fresh_tf_sample_time_sec'] = second_step_generation_wall_time_sec",
        "'fresh_scan_received': bool(pending.get('fresh_scan_received', False))",
        "'fresh_local_costmap_received': bool(pending.get('fresh_local_costmap_received', False))",
        "'fresh_tf_received': bool(pending.get('fresh_tf_received', False))",
        "'fresh_scan_sample_time_sec': pending.get('fresh_scan_sample_time_sec')",
        "'fresh_local_costmap_sample_time_sec': pending.get('fresh_local_costmap_sample_time_sec')",
        "'fresh_tf_sample_time_sec': pending.get('fresh_tf_sample_time_sec')",
        "'generation_wall_time_sec': second_step_generation_wall_time_sec",
    ]
    for phrase in required:
        assert phrase in text


def test_phase138_second_step_forward_goal_payload_records_validity_and_candidate_diagnostics():
    text = _read(MAZE_EXPLORER)
    required = [
        'def _serialize_second_step_forward_goal',
        "'valid': self._second_step_forward_goal_valid(second_step)",
        "'map_frame_id': self.map_frame",
        "'selected_candidate_target': second_step.get('selected_candidate_target')",
        "'selected_candidate_yaw': second_step.get('selected_candidate_yaw')",
        "'candidate_count': source_forward.get('candidate_count')",
        "'hard_safety_pass_candidate_count': second_step.get('hard_safety_pass_candidate_count')",
        "'selected_candidate_index': source_forward.get('selected_candidate_index')",
        "'selection_priority_trace': source_forward.get('selection_priority_trace')",
        "'rejected_candidate_summaries': source_forward.get('rejected_candidate_summaries')",
        "'front_wedge_risk_after_staging': self._front_wedge_risk_payload(second_step)",
        "'lateral_residual_after': self._second_step_lateral_residual_after(second_step)",
        "'generated_after_fresh_evidence': bool(second_step.get('generated_after_fresh_evidence', False))",
    ]
    for phrase in required:
        assert phrase in text


def test_phase138_outgoing_second_step_dispatch_records_recursion_guard_without_reapplying_staging():
    text = _read(MAZE_EXPLORER)
    required = [
        "'skip_two_step_staging': bool(skip_two_step_staging)",
        "'phase138_recursion_guard': bool(skip_two_step_staging)",
        "'phase136_recursion_guard': bool(skip_two_step_staging)",
        "'recursion_guard': bool(skip_two_step_staging)",
        "'two_step_stage_dispatch_requested': False",
        "'staging_applied': False",
        "'prior_staging_applied': True",
        "self._send_goal((float(selected[0]), float(selected[1])), float(yaw if isinstance(yaw, (int, float)) else direction_rad), 'explore', skip_two_step_staging=True)",
    ]
    for phrase in required:
        assert phrase in text


def test_phase138_static_analyzer_accepts_updated_source_and_rejects_missing_contract():
    analyzer = _load(ANALYZER, 'phase138_static_analyzer')
    result = analyzer.analyze_source(MAZE_EXPLORER)
    assert result['valid'] is True
    assert result['classification'] == 'PHASE138_SECOND_STEP_SERIALIZATION_CONTRACT_PRESENT'
    for key, value in result['checks'].items():
        assert value is True, key

    missing = analyzer.analyze_text('def _send_goal(self):\n    pass\n')
    assert missing['valid'] is False
    assert missing['classification'] == 'PHASE138_SECOND_STEP_SERIALIZATION_CONTRACT_INCOMPLETE'
    assert missing['checks']['pending_snapshot_helper'] is False


def test_phase138_report_records_minimal_implementation_boundaries_and_no_runtime():
    text = _read(REPORT)
    required = [
        'PHASE138_SECOND_STEP_CONTRACT_SERIALIZATION_MINIMAL_IMPLEMENTATION_COMPLETE_STOP_BEFORE_PHASE139',
        'minimal artifact/serialization/diagnostic fields only',
        'No Gazebo/RViz/Nav2 runtime was started',
        'No NavigateToPose goal was sent',
        'No maze_explorer runtime was started',
        'No staging/explore/third goal was sent',
        'No Nav2/MPPI/controller/goal checker/config tuning',
        'No branch scoring/centerline/fallback/terminal acceptance behavior change',
        'No staging disablement',
        'No target selection or goal timing change',
        'No autonomous exploration success or exit success is claimed',
        'Phase139 not entered',
        'py_compile',
        'pytest',
        'static analyzer',
    ]
    for phrase in required:
        assert phrase in text
