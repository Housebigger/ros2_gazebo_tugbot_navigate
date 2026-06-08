import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / 'tools' / 'analyze_timeout_subtypes.py'
LOG = ROOT / 'log'


def _run_classifier(input_json: Path, output_json: Path) -> dict:
    result = subprocess.run([
        sys.executable,
        str(SCRIPT),
        '--failure-windows', str(input_json),
        '--output-json', str(output_json),
    ], text=True, capture_output=True, check=False)
    assert result.returncode == 0, result.stderr
    assert 'goal_sequence,outcome,controller_subtype,timing_subtype,combined_subtype' in result.stdout
    return json.loads(output_json.read_text(encoding='utf-8'))


def test_phase24a_synthetic_timeout_subtype_classifier(tmp_path):
    failure_windows = tmp_path / 'failure_windows.json'
    output = tmp_path / 'subtypes.json'
    failure_windows.write_text(json.dumps({
        'failure_windows': [
            {
                'goal_sequence': 1,
                'outcome': 'timeout',
                'classification': 'healthy_motion_but_late_stall',
                'late_controller_silent': True,
                'timeout_footprint_cost_max': 99,
                'timeout_footprint_lethal_cell_count': 5,
                'timeout_front_wedge_cost_max': 100,
                'timeout_path_ahead_0_5m_cost_max': 99,
                'timeout_path_ahead_1_0m_cost_max': 100,
                'near_zero_cmd_to_first_progress_failure_sec': -2.5,
            },
            {
                'goal_sequence': 2,
                'outcome': 'timeout',
                'classification': 'healthy_motion_but_late_stall',
                'late_controller_silent': True,
                'timeout_robot_local_cost_max': 100,
                'timeout_right_side_cost_max': 99,
                'timeout_footprint_cost_max': 0,
                'timeout_front_wedge_cost_max': 0,
                'timeout_path_ahead_0_5m_cost_max': 0,
                'timeout_path_ahead_1_0m_cost_max': 0,
                'near_zero_cmd_to_first_progress_failure_sec': 8.0,
            },
            {
                'goal_sequence': 3,
                'outcome': 'failure',
                'classification': 'healthy_motion',
                'late_controller_silent': False,
            },
        ]
    }), encoding='utf-8')

    data = _run_classifier(failure_windows, output)
    by_seq = {row['goal_sequence']: row for row in data['timeout_subtypes']}

    assert by_seq[1]['controller_subtype'] == 'footprint_path_blocked_late_silent'
    assert by_seq[1]['timing_subtype'] == 'cmd_silent_before_progress_failure'
    assert by_seq[1]['combined_subtype'] == 'footprint_path_blocked_late_silent+cmd_silent_before_progress_failure'

    assert by_seq[2]['controller_subtype'] == 'side_cost_or_timing_late_silent'
    assert by_seq[2]['timing_subtype'] == 'cmd_silent_after_recovery_abort'
    assert by_seq[2]['combined_subtype'] == 'side_cost_or_timing_late_silent+cmd_silent_after_recovery_abort'

    assert 3 not in by_seq
    assert data['summary']['timeout_count'] == 2
    assert data['summary']['controller_subtype_counts']['footprint_path_blocked_late_silent'] == 1
    assert data['summary']['controller_subtype_counts']['side_cost_or_timing_late_silent'] == 1
    assert data['summary']['timing_subtype_counts']['cmd_silent_before_progress_failure'] == 1
    assert data['summary']['timing_subtype_counts']['cmd_silent_after_recovery_abort'] == 1


def test_phase24a_phase23b_fixture_timeout_subtypes(tmp_path):
    fixture = LOG / 'phase23_run1_failure_windows_after_rebuild.json'
    assert fixture.exists(), 'Phase 23B fixture is required for Phase 24A regression'
    data = _run_classifier(fixture, tmp_path / 'phase23b_subtypes.json')
    by_seq = {row['goal_sequence']: row for row in data['timeout_subtypes']}

    assert set(by_seq) == {2, 4, 8, 11}
    assert by_seq[8]['controller_subtype'] == 'footprint_path_blocked_late_silent'
    assert by_seq[11]['controller_subtype'] == 'footprint_path_blocked_late_silent'
    assert by_seq[2]['controller_subtype'] == 'side_cost_or_timing_late_silent'
    assert by_seq[4]['controller_subtype'] == 'side_cost_or_timing_late_silent'

    assert by_seq[8]['timing_subtype'] == 'cmd_silent_before_progress_failure'
    assert by_seq[2]['timing_subtype'] == 'cmd_silent_after_recovery_abort'
    assert by_seq[4]['timing_subtype'] == 'cmd_silent_after_recovery_abort'
    assert by_seq[11]['timing_subtype'] == 'cmd_silent_after_recovery_abort'

    assert data['summary']['controller_subtype_counts'] == {
        'footprint_path_blocked_late_silent': 2,
        'side_cost_or_timing_late_silent': 2,
    }
    assert data['summary']['timing_subtype_counts'] == {
        'cmd_silent_after_recovery_abort': 3,
        'cmd_silent_before_progress_failure': 1,
    }
    assert data['summary']['severe_footprint_path_ratio'] == 0.5
