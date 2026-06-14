import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
WRAPPER = ROOT / 'tools' / 'run_phase21_controller_diagnostics_smoke.sh'
AGGREGATE = ROOT / 'tools' / 'aggregate_phase26b_variance.py'


def test_phase26b_wrapper_supports_baseline_and_candidate_run_ids_and_artifacts():
    wrapper = WRAPPER.read_text(encoding='utf-8')
    assert 'phase26b_baseline_run[0-9]+' in wrapper
    assert 'phase26b_candidate_run[0-9]+' in wrapper
    assert 'PHASE26B_CANDIDATE_PROFILE=true' in wrapper
    assert 'FAILURE_WINDOWS="$LOG_DIR/${RUN_ID}_failure_windows.json"' in wrapper
    assert 'TIMEOUT_SUBTYPES="$LOG_DIR/${RUN_ID}_timeout_subtypes.json"' in wrapper
    assert 'POST_RECOVERY_ENRICHED="$LOG_DIR/${RUN_ID}_post_recovery_enriched.json"' in wrapper
    assert 'tools/analyze_failure_windows.py' in wrapper
    assert 'tools/analyze_timeout_subtypes.py' in wrapper
    assert 'tools/enrich_post_recovery_snapshots.py' in wrapper


def test_phase26b_aggregator_summarizes_runtime_params_and_variance(tmp_path):
    log_dir = tmp_path / 'log'
    log_dir.mkdir()
    for run_id, profile, mode, distance, timeouts, footprint in [
        ('phase26b_baseline_run1', 'canonical_baseline', 'FAILED_EXHAUSTED', 1.0, 4, 2),
        ('phase26b_baseline_run2', 'canonical_baseline', 'EXIT_REACHED', 0.5, 2, 1),
        ('phase26b_candidate_run1', 'candidate_costcritic_275', 'EXIT_REACHED', 0.6, 3, 1),
    ]:
        (log_dir / f'{run_id}_params_fingerprint.json').write_text(json.dumps({
            'selected_profile': profile,
            'params_file': {'costcritic_cost_weight': 2.75 if 'candidate' in run_id else 3.81},
        }), encoding='utf-8')
        (log_dir / f'{run_id}_runtime_params').mkdir()
        (log_dir / f'{run_id}_runtime_params' / 'controller_server_summary.json').write_text(json.dumps({
            'controller_server': {'FollowPath': {'CostCritic': {'cost_weight': 2.75 if 'candidate' in run_id else 3.81}}},
        }), encoding='utf-8')
        (log_dir / f'{run_id}_explorer_state.jsonl').write_text(json.dumps({'state': {
            'mode': mode,
            'goal_count': 12,
            'goal_success_count': 8,
            'timeout_cancel_count': timeouts,
            'blocked_branch_count': 0,
            'blacklisted_goal_count': 0,
            'exit_distance_m': distance,
        }}) + '\n', encoding='utf-8')
        (log_dir / f'{run_id}_timeout_subtypes.json').write_text(json.dumps({
            'summary': {
                'controller_subtype_counts': {'footprint_path_blocked_late_silent': footprint},
                'side_or_timing_count': 1,
                'unclassified_count': 0,
            }
        }), encoding='utf-8')
    output = tmp_path / 'summary.json'
    result = subprocess.run([
        sys.executable,
        str(AGGREGATE),
        '--log-dir', str(log_dir),
        '--runs', 'phase26b_baseline_run1,phase26b_baseline_run2,phase26b_candidate_run1',
        '--output-json', str(output),
    ], text=True, capture_output=True, check=False)
    assert result.returncode == 0, result.stderr
    data = json.loads(output.read_text(encoding='utf-8'))
    assert data['groups']['canonical_baseline']['run_count'] == 2
    assert data['groups']['canonical_baseline']['exit_reached_count'] == 1
    assert data['groups']['canonical_baseline']['timeout_cancel_count']['median'] == 3.0
    assert data['groups']['candidate_costcritic_275']['runtime_cost_weight_values'] == [2.75]
    assert data['groups']['candidate_costcritic_275']['source_cost_weight_values'] == [2.75]
    assert data['recommendation'] in {
        'insufficient_repeat_data',
        'candidate_variance_characterized_not_promotion_ready',
        'candidate_repeat_metrics_favorable_but_requires_review',
    }
