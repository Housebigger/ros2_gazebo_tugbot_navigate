from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
PROPOSAL = ROOT / 'doc' / 'doc_proposal' / 'phase108_preflight_false_negative_reduction_design.md'
REPORT = ROOT / 'doc' / 'doc_report' / 'phase108_preflight_false_negative_reduction_design_report.md'


def _text(path: Path) -> str:
    assert path.exists(), f'missing required Phase108 document: {path}'
    return path.read_text(encoding='utf-8')


def test_phase108_proposal_exists_and_is_design_only():
    text = _text(PROPOSAL)
    required = [
        'Phase108',
        'DESIGN_ONLY',
        'fail-closed',
        'false-negative reduction',
        'do not send ingress goal',
        'do not start maze_explorer',
        'do not remove preflight',
        'do not tune Nav2',
        'does not prove the bug is fixed',
        'does not prove it is safe to send a goal',
        'Phase109 not entered',
    ]
    for phrase in required:
        assert phrase in text


def test_phase108_proposal_covers_required_design_topics():
    text = _text(PROPOSAL)
    required_topics = [
        'lifecycle_multi_source_confirmation',
        'tf_consecutive_sampling_stable_window',
        'scan_wait_for_first_sample',
        'per_sample_failure_reason_history',
        'raw_style_snapshot_cross_check',
        'lifecycle_ambiguous_not_inactive',
        'startup_grace_plus_stable_window',
    ]
    for topic in required_topics:
        assert topic in text


def test_phase108_artifact_schema_records_sample_level_evidence_and_ambiguity():
    text = _text(PROPOSAL)
    required_schema_terms = [
        'sample_history',
        'sample_index',
        'sample_wall_time_sec',
        'failed_gates',
        'failure_reasons_by_gate',
        'lifecycle_sources',
        'lifecycle_ambiguous',
        'raw_style_snapshot_before_reject',
        'startup_grace_sec',
        'stable_window_sec',
        'first_scan_wait_elapsed_sec',
    ]
    for term in required_schema_terms:
        assert term in text


def test_phase108_report_summarizes_no_runtime_validation_or_repair_claim():
    text = _text(REPORT)
    required = [
        'COMPLETE_DESIGN_REVIEW_ONLY_STOP_BEFORE_PHASE109',
        'No implementation was performed',
        'No Phase106 rerun was performed',
        'No ingress goal was sent',
        'No Nav2/MPPI/controller tuning was performed',
        'does not prove the Phase105 bug is fixed',
        'does not prove it is safe to send an ingress goal',
        'waiting for human acceptance before Phase109',
    ]
    for phrase in required:
        assert phrase in text


def test_phase108_report_lists_expected_future_reject_and_ambiguity_tokens():
    text = _text(REPORT)
    expected_tokens = [
        'ingress_lifecycle_ambiguous',
        'ingress_first_scan_timeout',
        'ingress_tf_stable_window_not_met',
        'ingress_raw_snapshot_cross_check_failed',
        'ingress_preflight_timeout',
    ]
    for token in expected_tokens:
        assert token in text
