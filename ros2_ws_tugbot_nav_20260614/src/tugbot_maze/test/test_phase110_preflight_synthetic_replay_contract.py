from __future__ import annotations

import importlib.util
import json
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
ANALYZER = ROOT / 'tools' / 'analyze_phase110_preflight_synthetic_replay_contract.py'
REPORT = ROOT / 'doc' / 'doc_report' / 'phase110_preflight_synthetic_replay_contract_report.md'
LOG_DIR = ROOT / 'log' / 'phase110_preflight_synthetic_replay_contract'


def _load_analyzer():
    assert ANALYZER.exists(), f'missing {ANALYZER}'
    spec = importlib.util.spec_from_file_location('phase110_replay_analyzer', ANALYZER)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_phase110_synthetic_replay_covers_lifecycle_states(tmp_path):
    module = _load_analyzer()
    artifacts = module.generate_synthetic_replay_artifacts(tmp_path / 'artifacts')
    analysis = module.analyze_replay_dir(tmp_path / 'artifacts')

    assert set(artifacts) >= {
        'lifecycle_active_confirmed',
        'lifecycle_inactive_confirmed',
        'lifecycle_ambiguous',
        'lifecycle_query_error',
    }
    states = analysis['lifecycle_replay_states']
    assert states['lifecycle_active_confirmed'] == 'active_confirmed'
    assert states['lifecycle_inactive_confirmed'] == 'inactive_confirmed'
    assert states['lifecycle_ambiguous'] == 'ambiguous'
    assert states['lifecycle_query_error'] == 'query_error'
    assert analysis['contract_valid'] is True


def test_phase110_scan_and_tf_replay_classifications_are_distinct(tmp_path):
    module = _load_analyzer()
    module.generate_synthetic_replay_artifacts(tmp_path / 'artifacts')
    analysis = module.analyze_replay_dir(tmp_path / 'artifacts')
    cases = analysis['cases']

    waiting = cases['first_scan_waiting_only']
    timeout = cases['first_scan_timeout']
    recovered = cases['early_tf_miss_recovered']
    stable_short = cases['stable_window_not_met']

    assert waiting['classification'] == 'WAITING_FOR_FIRST_SCAN_OBSERVED_NO_FINAL_SCAN_TIMEOUT'
    assert waiting['sample_history_tokens'][0] == ['waiting_for_first_scan']
    assert timeout['classification'] == 'INGRESS_FIRST_SCAN_TIMEOUT_REPLAYED'
    assert timeout['reject_reason'] == 'ingress_first_scan_timeout'
    assert recovered['classification'] == 'EARLY_TF_MISS_RECOVERED_STABLE_WINDOW_PASS'
    assert recovered['passed'] is True
    assert recovered['final_failure_not_early_missing'] is True
    assert stable_short['classification'] == 'INGRESS_TF_STABLE_WINDOW_NOT_MET_REPLAYED'
    assert stable_short['reject_reason'] == 'ingress_tf_stable_window_not_met'
    assert stable_short['final_failure_not_early_missing'] is True


def test_phase110_raw_snapshot_contradiction_is_ambiguous_and_fail_closed(tmp_path):
    module = _load_analyzer()
    module.generate_synthetic_replay_artifacts(tmp_path / 'artifacts')
    analysis = module.analyze_replay_dir(tmp_path / 'artifacts')
    raw_case = analysis['cases']['raw_snapshot_contradiction']

    assert raw_case['classification'] == 'RAW_SNAPSHOT_CONTRADICTION_AMBIGUOUS_FAIL_CLOSED'
    assert raw_case['passed'] is False
    assert raw_case['ingress_goal_sent'] is False
    assert raw_case['maze_explorer_started'] is False
    assert raw_case['raw_style_snapshot_cross_check']['ambiguous'] is True
    assert 'ingress_raw_snapshot_cross_check_failed' in raw_case['failed_gates']


def test_phase110_sample_history_contract_is_complete_for_every_sample(tmp_path):
    module = _load_analyzer()
    module.generate_synthetic_replay_artifacts(tmp_path / 'artifacts')
    analysis = module.analyze_replay_dir(tmp_path / 'artifacts')

    assert analysis['sample_history_contract']['all_samples_have_failed_gates'] is True
    assert analysis['sample_history_contract']['all_samples_have_failure_reasons_by_gate'] is True
    assert analysis['sample_history_contract']['missing_fields'] == []
    for case in analysis['cases'].values():
        assert case['sample_count'] == len(case['sample_history'])
        for sample in case['sample_history']:
            assert 'failed_gates' in sample
            assert 'failure_reasons_by_gate' in sample


def test_phase110_cli_writes_analysis_and_human_readable_summary(tmp_path):
    output_dir = tmp_path / 'phase110'
    proc = subprocess.run(
        ['python3', str(ANALYZER), '--output-dir', str(output_dir)],
        cwd=ROOT,
        text=True,
        capture_output=True,
        check=False,
    )
    assert proc.returncode == 0, proc.stderr
    analysis_path = output_dir / 'phase110_preflight_synthetic_replay_contract_analysis.json'
    summary_path = output_dir / 'phase110_preflight_synthetic_replay_contract_minimal_summary.md'
    assert analysis_path.exists()
    assert summary_path.exists()

    analysis = json.loads(analysis_path.read_text())
    summary = summary_path.read_text()
    assert analysis['contract_valid'] is True
    assert 'Phase110 synthetic replay contract summary' in summary
    assert 'active_confirmed' in summary
    assert 'waiting_for_first_scan' in summary
    assert 'ingress_tf_stable_window_not_met' in summary
    assert 'RAW_SNAPSHOT_CONTRADICTION_AMBIGUOUS_FAIL_CLOSED' in summary
    assert 'No runtime stack was started' in summary


def test_phase110_report_guardrails_present_after_completion():
    assert REPORT.exists(), f'missing {REPORT}'
    text = REPORT.read_text(encoding='utf-8')
    for phrase in [
        'PHASE110_SYNTHETIC_REPLAY_CONTRACT_VALIDATED_STOP_BEFORE_PHASE111',
        'No Gazebo/RViz/SLAM/Nav2/maze_explorer was started',
        'No Phase106 rerun was performed',
        'No ingress goal was sent',
        'No Nav2/MPPI/controller/config tuning was performed',
        'does not prove it is safe to send an ingress goal',
        'Phase111 not entered',
    ]:
        assert phrase in text
