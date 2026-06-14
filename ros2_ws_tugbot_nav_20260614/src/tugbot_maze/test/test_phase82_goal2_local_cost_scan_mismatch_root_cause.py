from __future__ import annotations

import importlib.util
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
ANALYZER = ROOT / 'tools' / 'analyze_phase82_goal2_local_cost_scan_mismatch_root_cause.py'
PHASE81_DIR = ROOT / 'log' / 'phase81_goal2_forward_open_machine_evidence_capture'
PHASE82_DIR = ROOT / 'log' / 'phase82_goal2_local_cost_scan_mismatch_root_cause'
PHASE82_JSON = PHASE82_DIR / 'phase82_goal2_local_cost_scan_mismatch_root_cause.json'
PHASE82_SUMMARY = PHASE82_DIR / 'phase82_goal2_local_cost_scan_mismatch_root_cause_minimal_summary.md'
PHASE82_REPORT = ROOT / 'doc' / 'doc_report' / 'phase82_goal2_local_cost_scan_mismatch_root_cause_report.md'

ALLOWED = {
    'INFLATION_SPILLOVER_SUSPECTED',
    'FOOTPRINT_OR_WEDGE_PROJECTION_SUSPECTED',
    'CORRIDOR_SAMPLING_TOO_WIDE_SUSPECTED',
    'POSE_TF_PROJECTION_MISMATCH_SUSPECTED',
    'REAL_EXECUTION_CLEARANCE_TOO_NARROW',
    'COSTMAP_STALE_OR_OBSERVATION_MISMATCH',
    'ROOT_CAUSE_INSUFFICIENT_EVIDENCE',
}


def load_module():
    assert ANALYZER.exists(), f'missing Phase82 analyzer: {ANALYZER}'
    spec = importlib.util.spec_from_file_location('phase82_analyzer', ANALYZER)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_phase82_declares_allowed_root_cause_tokens_and_guardrails():
    module = load_module()
    assert set(module.ALLOWED_ROOT_CAUSE_CLASSIFICATIONS) == ALLOWED
    assert 'ROOT_CAUSE_INSUFFICIENT_EVIDENCE' in module.ALLOWED_ROOT_CAUSE_CLASSIFICATIONS
    guardrail_text = '\n'.join(module.GUARDRAILS)
    assert 'No maze_explorer strategy changed' in guardrail_text
    assert 'No branch scoring changed' in guardrail_text
    assert 'No centerline gate changed' in guardrail_text
    assert 'No directional readiness override changed' in guardrail_text
    assert 'No fallback/terminal acceptance changed' in guardrail_text
    assert 'No Nav2/MPPI/controller tuning' in guardrail_text
    assert 'No inflation/robot_radius/clearance_radius_m/map threshold tuning' in guardrail_text
    assert 'No autonomous exploration success claimed' in guardrail_text
    assert 'No exit success claimed' in guardrail_text


def test_phase82_actual_phase81_artifacts_classify_footprint_or_wedge_projection_suspected():
    module = load_module()
    raw_capture = PHASE81_DIR / 'phase81_goal2_forward_open_machine_evidence_capture_raw_capture.json'
    phase81_analysis = PHASE81_DIR / 'phase81_goal2_forward_open_machine_evidence_capture_analysis.json'
    assert raw_capture.exists()
    assert phase81_analysis.exists()

    result = module.analyze_phase82(raw_capture, phase81_analysis)

    assert result['source_phase81_forward_open_classification'] == 'FORWARD_OPEN_CORRIDOR_BLOCKED'
    assert result['scan_local_cost_mismatch']['physical_scan_forward_open'] is True
    assert result['scan_local_cost_mismatch']['local_cost_corridor_blocked'] is True
    assert result['root_cause_classification'] == 'FOOTPRINT_OR_WEDGE_PROJECTION_SUSPECTED'
    assert result['root_cause_classification'] in ALLOWED
    assert result['evidence_sufficient'] is True
    assert result['candidate_checks']['footprint_or_wedge_projection']['supported'] is True
    assert result['candidate_checks']['inflation_spillover']['supported'] is True
    assert result['candidate_checks']['pose_tf_projection_mismatch']['supported'] is False
    assert result['candidate_checks']['costmap_stale_or_observation_mismatch']['supported'] is False
    assert result['autonomous_success_claimed'] is False
    assert result['exit_success_claimed'] is False
    assert result['phase83_entered'] is False


def test_phase82_missing_required_raw_evidence_stays_insufficient(tmp_path: Path):
    module = load_module()
    raw_capture = json.loads((PHASE81_DIR / 'phase81_goal2_forward_open_machine_evidence_capture_raw_capture.json').read_text())
    phase81_analysis = json.loads((PHASE81_DIR / 'phase81_goal2_forward_open_machine_evidence_capture_analysis.json').read_text())
    raw_capture.pop('scan', None)
    raw_path = tmp_path / 'raw_capture_missing_scan.json'
    analysis_path = tmp_path / 'phase81_analysis.json'
    raw_path.write_text(json.dumps(raw_capture))
    analysis_path.write_text(json.dumps(phase81_analysis))

    result = module.analyze_phase82(raw_path, analysis_path)

    assert result['root_cause_classification'] == 'ROOT_CAUSE_INSUFFICIENT_EVIDENCE'
    assert result['evidence_sufficient'] is False
    assert 'missing_raw_scan' in result['evidence_gaps']
    assert result['semantic_caveats']['no_fabricated_missing_evidence'] is True


def test_phase82_stale_costmap_observation_mismatch_synthetic_path(tmp_path: Path):
    module = load_module()
    raw_capture = json.loads((PHASE81_DIR / 'phase81_goal2_forward_open_machine_evidence_capture_raw_capture.json').read_text())
    phase81_analysis = json.loads((PHASE81_DIR / 'phase81_goal2_forward_open_machine_evidence_capture_analysis.json').read_text())
    raw_capture['local_costmap']['stamp_sec'] = 10.0
    raw_capture['scan']['stamp_sec'] = 20.0
    raw_capture['odom']['stamp_sec'] = 20.0
    raw_capture['footprint']['stamp_sec'] = 20.0
    raw_path = tmp_path / 'raw_capture_stale_costmap.json'
    analysis_path = tmp_path / 'phase81_analysis.json'
    raw_path.write_text(json.dumps(raw_capture))
    analysis_path.write_text(json.dumps(phase81_analysis))

    result = module.analyze_phase82(raw_path, analysis_path)

    assert result['root_cause_classification'] == 'COSTMAP_STALE_OR_OBSERVATION_MISMATCH'
    assert result['candidate_checks']['costmap_stale_or_observation_mismatch']['supported'] is True
    assert result['candidate_checks']['costmap_stale_or_observation_mismatch']['max_sensor_costmap_stamp_delta_sec'] >= 9.0


def test_phase82_output_artifacts_and_report_are_written_after_real_run():
    assert PHASE82_JSON.exists(), f'missing Phase82 JSON artifact: {PHASE82_JSON}'
    assert PHASE82_SUMMARY.exists(), f'missing Phase82 minimal summary: {PHASE82_SUMMARY}'
    assert PHASE82_REPORT.exists(), f'missing Phase82 report: {PHASE82_REPORT}'
    data = json.loads(PHASE82_JSON.read_text())
    report = PHASE82_REPORT.read_text()
    assert data['root_cause_classification'] == 'FOOTPRINT_OR_WEDGE_PROJECTION_SUSPECTED'
    assert 'FOOTPRINT_OR_WEDGE_PROJECTION_SUSPECTED' in report
    assert 'INFLATION_SPILLOVER_SUSPECTED' in report
    assert 'ROOT_CAUSE_INSUFFICIENT_EVIDENCE' in report
    assert 'Phase83 not entered' in report
    assert 'No maze_explorer strategy changed' in report
    assert 'No Nav2/MPPI/controller tuning' in report
    assert 'No autonomous exploration success claimed' in report
    assert 'No exit success claimed' in report
