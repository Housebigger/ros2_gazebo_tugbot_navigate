import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
ANALYZER = ROOT / 'tools' / 'analyze_phase26x_selected_control_debug.py'
REPORT = ROOT / 'doc' / 'doc_report' / 'phase26x_mppi_selected_control_diagnostics.md'


def run_analyzer(input_path: Path, output_path: Path):
    return subprocess.run([
        sys.executable,
        str(ANALYZER),
        '--input-jsonl', str(input_path),
        '--output-json', str(output_path),
    ], text=True, capture_output=True, check=False)


def write_jsonl(path: Path, rows):
    path.write_text('\n'.join(json.dumps(row) for row in rows) + '\n', encoding='utf-8')


def test_phase26x_analyzer_classifies_control_sequence_and_returned_twist_states(tmp_path):
    input_path = tmp_path / 'phase26x_mppi_selected_control_debug.jsonl'
    output_path = tmp_path / 'phase26x_selected_control_analysis.json'
    write_jsonl(input_path, [
        {
            'phase': '26X',
            'event': 'mppi_selected_control_cycle',
            'cycle_index': 1,
            'near_zero': True,
            'returned_twist': {'vx': 0.0, 'vy': 0.0, 'wz': 0.0},
            'control_sequence_head': [
                {'vx': 0.0, 'vy': 0.0, 'wz': 0.0},
                {'vx': 0.01, 'vy': 0.0, 'wz': 0.0},
            ],
            'fallback_used': False,
            'update_summary': {
                'first_before': {'vx': 0.18, 'vy': 0.0, 'wz': 0.12},
                'first_after': {'vx': 0.0, 'vy': 0.0, 'wz': 0.0},
            },
        },
        {
            'phase': '26X',
            'event': 'mppi_selected_control_cycle',
            'cycle_index': 2,
            'near_zero': True,
            'returned_twist': {'vx': 0.0, 'vy': 0.0, 'wz': 0.0},
            'control_sequence_head': [
                {'vx': 0.24, 'vy': 0.0, 'wz': 0.11},
                {'vx': 0.20, 'vy': 0.0, 'wz': 0.08},
            ],
            'fallback_used': False,
        },
        {
            'phase': '26X',
            'event': 'mppi_selected_control_cycle',
            'cycle_index': 3,
            'near_zero': True,
            'returned_twist': {'vx': 0.0, 'vy': 0.0, 'wz': 0.0},
            'control_sequence_head': [
                {'vx': 0.0, 'vy': 0.0, 'wz': 0.0},
            ],
            'fallback': {'used': True, 'failed': False},
        },
    ])

    result = run_analyzer(input_path, output_path)
    assert result.returncode == 0, result.stderr
    data = json.loads(output_path.read_text(encoding='utf-8'))

    assert data['phase'] == '26X'
    assert data['analysis_only'] is True
    assert data['summary']['cycle_count'] == 3
    assert data['summary']['near_zero_cycle_count'] == 3
    assert data['summary']['control_sequence_first_near_zero_count'] == 2
    assert data['summary']['returned_twist_near_zero_count'] == 3
    assert data['summary']['fallback_used_count'] == 1
    assert data['summary']['control_nonzero_but_returned_zero_count'] == 1
    assert data['summary']['optimizer_update_collapsed_control_count'] == 1

    by_cycle = {row['cycle_index']: row for row in data['cycles']}
    assert by_cycle[1]['control_sequence_first_is_near_zero'] is True
    assert by_cycle[1]['returned_twist_is_near_zero'] is True
    assert by_cycle[1]['optimizer_update_collapsed_control'] is True
    assert by_cycle[2]['control_nonzero_but_returned_zero'] is True
    assert by_cycle[3]['fallback_used'] is True
    assert data['decision']['phase27_allowed'] is False
    assert data['decision']['candidate_promotion_or_rejection_allowed'] is False


def test_phase26x_analyzer_reports_missing_evidence_without_inventing_conclusion(tmp_path):
    input_path = tmp_path / 'empty_phase26x.jsonl'
    output_path = tmp_path / 'empty_phase26x.json'
    input_path.write_text('', encoding='utf-8')

    result = run_analyzer(input_path, output_path)
    assert result.returncode == 0, result.stderr
    data = json.loads(output_path.read_text(encoding='utf-8'))

    assert data['summary']['cycle_count'] == 0
    assert data['conclusion']['near_zero_reason'] == 'insufficient_evidence'
    assert data['conclusion']['evidence_complete'] is False
    assert 'do_not_tune_nav2_controller_params_from_phase26x' in data['decision']['guardrails']


def test_phase26x_report_documents_overlay_infeasibility_and_source_level_plan():
    assert REPORT.exists()
    text = REPORT.read_text(encoding='utf-8')
    assert 'Phase26X' in text
    assert 'diagnostics-only' in text
    assert 'apt-get source ros-jazzy-nav2-mppi-controller' in text
    assert 'E: You must put some' in text and 'deb-src' in text
    assert 'Optimizer::getControlFromSequenceAsTwist' in text
    assert 'MPPIController::computeVelocityCommands' in text
    assert 'do not modify branch selection' in text
    assert 'do not modify Nav2/controller parameter semantics' in text
    assert 'phase26x_mppi_selected_control_debug.jsonl' in text
