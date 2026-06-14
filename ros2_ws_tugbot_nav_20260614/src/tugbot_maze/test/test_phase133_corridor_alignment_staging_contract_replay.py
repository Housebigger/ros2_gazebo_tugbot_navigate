import importlib.util
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
ANALYZER = ROOT / 'tools' / 'analyze_phase133_corridor_alignment_staging_contract_replay.py'
PHASE129_ARTIFACT = ROOT / 'log' / 'phase129_instrumented_first_goal_timeout_diagnosis' / 'phase129_instrumented_first_goal_timeout_diagnosis_rerun.json'
PHASE131_ANALYSIS = ROOT / 'log' / 'phase131_first_dispatch_kind_artifact_replay' / 'phase131_first_dispatch_kind_artifact_replay_analysis.json'
PHASE129_STDERR = ROOT / 'log' / 'phase129_instrumented_first_goal_timeout_diagnosis' / 'phase129_instrumented_first_goal_timeout_diagnosis_rerun_maze_explorer_stderr.log'
REPORT = ROOT / 'doc' / 'doc_report' / 'phase133_corridor_alignment_staging_contract_replay_report.md'

EXPECTED_CLASSIFICATIONS = [
    'FIRST_DISPATCH_EXPLORE_ACCEPTED_STOP',
    'FIRST_DISPATCH_STAGING_ACCEPTED_STOP',
    'FIRST_DISPATCH_STAGING_REJECTED_DIAGNOSTIC_FAIL',
    'FIRST_DISPATCH_STAGING_TIMEOUT_DIAGNOSTIC_FAIL',
    'STAGING_SECOND_STEP_NOT_ATTEMPTED_STOP',
    'DISPATCH_KIND_CONTRACT_AMBIGUOUS',
]


def load_analyzer():
    assert ANALYZER.exists(), 'Phase133 analyzer must exist'
    spec = importlib.util.spec_from_file_location('phase133_analyzer', ANALYZER)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_phase133_classification_vocabulary_is_phase132_contract():
    module = load_analyzer()
    assert module.CLASSIFICATIONS == EXPECTED_CLASSIFICATIONS


def test_phase133_analyzer_classifies_phase129_literal_staging_without_calling_it_explore():
    module = load_analyzer()
    result = module.analyze_artifacts(
        phase129_artifact_path=PHASE129_ARTIFACT,
        phase131_analysis_path=PHASE131_ANALYSIS,
        phase129_stderr_path=PHASE129_STDERR,
    )

    assert result['phase'] == 'Phase133'
    assert result['mode'] == 'corridor_alignment_staging_contract_replay'
    assert result['valid'] is True
    assert result['classification'] == 'STAGING_SECOND_STEP_NOT_ATTEMPTED_STOP'
    assert result['first_literal_dispatch']['goal_kind'] == 'corridor_alignment_staging'
    assert result['first_literal_dispatch']['is_first_exploration_goal'] is False
    assert result['first_exploration_goal']['attempted'] is False
    assert result['first_exploration_goal']['second_step_dispatched'] is False
    assert result['claims']['staging_is_exploration_success'] is False
    assert result['claims']['staging_is_exit_success'] is False
    assert result['claims']['mixed_with_phase127_timeout_local_cost_replay'] is False


def test_phase133_staging_relationship_preserves_required_artifact_fields():
    module = load_analyzer()
    result = module.analyze_artifacts(
        phase129_artifact_path=PHASE129_ARTIFACT,
        phase131_analysis_path=PHASE131_ANALYSIS,
        phase129_stderr_path=PHASE129_STDERR,
    )
    rel = result['staging_relationship']

    assert rel['staging_applied']['value'] is True
    assert rel['two_step_stage_dispatch_requested']['value'] is True
    assert rel['staging_reason']['value'] == 'reduce_lateral_residual_and_front_wedge_risk_before_second_step_forward_goal'
    assert rel['original_target']['available'] is True
    assert rel['staging_target']['available'] is True
    assert rel['refined_target']['available'] is True
    assert rel['original_target']['value'] != rel['staging_target']['value']
    assert rel['staging_goal_pose']['available'] is True
    assert rel['staging_lateral_residual_before_m']['value'] > rel['staging_lateral_residual_after_m']['value']
    assert rel['front_wedge_risk']['available'] is True


def test_phase133_pending_second_step_evidence_is_explicit_not_silently_false():
    module = load_analyzer()
    result = module.analyze_artifacts(
        phase129_artifact_path=PHASE129_ARTIFACT,
        phase131_analysis_path=PHASE131_ANALYSIS,
        phase129_stderr_path=PHASE129_STDERR,
    )
    pending = result['pending_second_step_evidence']

    assert pending['pending_corridor_alignment_second_step']['available'] is False
    assert pending['pending_corridor_alignment_second_step']['value'] == 'missing'
    assert pending['second_step_forward_goal']['available'] is False
    assert pending['second_step_forward_goal']['value'] == 'missing'
    assert result['classification_reasons']
    assert any('second-step explore goal was not attempted' in reason for reason in result['classification_reasons'])


def test_phase133_rejects_old_phase129_non_explore_rejection_as_nav2_staging_rejection():
    module = load_analyzer()
    result = module.analyze_artifacts(
        phase129_artifact_path=PHASE129_ARTIFACT,
        phase131_analysis_path=PHASE131_ANALYSIS,
        phase129_stderr_path=PHASE129_STDERR,
    )

    assert result['legacy_phase129_contract']['result_status_label'] == 'REJECTED_NON_EXPLORE_GOAL_KIND'
    assert result['nav2_outcome_evidence']['nav2_rejection_evidence']['available'] is False
    assert result['classification'] != 'FIRST_DISPATCH_STAGING_REJECTED_DIAGNOSTIC_FAIL'
    assert result['classification'] != 'FIRST_DISPATCH_STAGING_TIMEOUT_DIAGNOSTIC_FAIL'
    assert result['classification'] != 'FIRST_DISPATCH_STAGING_ACCEPTED_STOP'


def test_phase133_cli_writes_json_and_markdown_outputs(tmp_path):
    module = load_analyzer()
    output_json = tmp_path / 'analysis.json'
    output_md = tmp_path / 'summary.md'

    exit_code = module.main([
        '--phase129-artifact', str(PHASE129_ARTIFACT),
        '--phase131-analysis', str(PHASE131_ANALYSIS),
        '--phase129-stderr', str(PHASE129_STDERR),
        '--output-json', str(output_json),
        '--output-md', str(output_md),
    ])

    assert exit_code == 0
    data = json.loads(output_json.read_text(encoding='utf-8'))
    summary = output_md.read_text(encoding='utf-8')
    assert data['classification'] == 'STAGING_SECOND_STEP_NOT_ATTEMPTED_STOP'
    assert 'STAGING_SECOND_STEP_NOT_ATTEMPTED_STOP' in summary
    assert 'not exploration success' in summary
    assert 'Phase134 boundary' in summary


def test_phase133_report_records_boundary_and_stop_condition_after_completion():
    assert REPORT.exists(), 'Phase133 report must be written after analyzer run'
    text = REPORT.read_text(encoding='utf-8')

    required = [
        'PHASE133_CORRIDOR_ALIGNMENT_STAGING_CONTRACT_REPLAY_COMPLETE_STOP_BEFORE_PHASE134',
        'STAGING_SECOND_STEP_NOT_ATTEMPTED_STOP',
        'first literal dispatch',
        'corridor_alignment_staging',
        'not the first exploration goal',
        'not exploration success',
        'not exit success',
        'not Phase127 FIRST_GOAL_TIMEOUT_LOCAL_COST_BLOCKED',
        'Phase134',
        'bounded staging-contract smoke',
        'not full autonomous exploration',
        'Phase134 not entered',
    ]
    for phrase in required:
        assert phrase in text


def test_phase133_analyzer_source_is_offline_only_and_has_no_runtime_goal_path():
    source = ANALYZER.read_text(encoding='utf-8')
    forbidden = [
        'subprocess',
        'Popen',
        'ros2 launch',
        'ros2 run',
        'NavigateToPose.Goal',
        'send_goal_async',
        'rclpy.init',
        'maze_explorer --ros-args',
        'max_goals:=',
        'controller_server',
        'mppi',
    ]
    for token in forbidden:
        assert token not in source
