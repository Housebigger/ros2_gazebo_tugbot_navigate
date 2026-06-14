import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
COMPARE = ROOT / 'tools' / 'compare_phase25b_metrics.py'


def _write(path: Path, final_mode: str, successes: int, timeouts: int, footprint: int, exit_distance: float):
    path.write_text(json.dumps({
        'run_summary': {
            'timeout_cancel_count': timeouts,
            'goal_success_count': successes,
            'blocked_branch_count': 0,
            'blacklisted_goal_count': 0,
            'final_mode': final_mode,
            'exit_distance_m': exit_distance,
        },
        'timeout_subtypes': {
            'summary': {
                'controller_subtype_counts': {'footprint_path_blocked_late_silent': footprint},
            }
        }
    }), encoding='utf-8')


def test_phase25c_compare_reports_exit_distance_advisory(tmp_path):
    baseline = tmp_path / 'baseline.json'
    experiment = tmp_path / 'experiment.json'
    output = tmp_path / 'decision.json'
    _write(baseline, 'FAILED_EXHAUSTED', successes=8, timeouts=4, footprint=2, exit_distance=0.75)
    _write(experiment, 'FAILED_EXHAUSTED', successes=9, timeouts=3, footprint=0, exit_distance=1.70)

    result = subprocess.run([
        sys.executable,
        str(COMPARE),
        '--baseline', str(baseline),
        '--experiment', str(experiment),
        '--output-json', str(output),
    ], text=True, capture_output=True, check=False)

    assert result.returncode == 0, result.stderr
    data = json.loads(output.read_text(encoding='utf-8'))
    assert data['accepted'] is True
    assert data['advisory']['exit_distance_m']['baseline'] == 0.75
    assert data['advisory']['exit_distance_m']['experiment'] == 1.70
    assert data['advisory']['exit_distance_m']['worsened_materially'] is True
    assert data['advisory']['exit_distance_m']['acceptable_or_exit_reached'] is False
    assert data['recommendation'] == 'repeat_or_smaller_delta_before_baseline_promotion'


def test_phase25c_compare_accepts_exit_reached_despite_distance(tmp_path):
    baseline = tmp_path / 'baseline.json'
    experiment = tmp_path / 'experiment.json'
    output = tmp_path / 'decision.json'
    _write(baseline, 'FAILED_EXHAUSTED', successes=8, timeouts=4, footprint=2, exit_distance=0.75)
    _write(experiment, 'EXIT_REACHED', successes=9, timeouts=3, footprint=0, exit_distance=1.70)

    result = subprocess.run([
        sys.executable,
        str(COMPARE),
        '--baseline', str(baseline),
        '--experiment', str(experiment),
        '--output-json', str(output),
    ], text=True, capture_output=True, check=False)

    assert result.returncode == 0, result.stderr
    data = json.loads(output.read_text(encoding='utf-8'))
    assert data['accepted'] is True
    assert data['advisory']['exit_distance_m']['acceptable_or_exit_reached'] is True
