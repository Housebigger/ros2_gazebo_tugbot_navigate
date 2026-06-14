import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
AGGREGATOR = ROOT / 'tools' / 'aggregate_phase26s_summary_repeats.py'


def write_json(path: Path, data):
    path.write_text(json.dumps(data, indent=2), encoding='utf-8')


def run_aggregate(tmp_path: Path, run_ids: list[str], *, phase: str = '26T'):
    output = tmp_path / f'phase{phase.lower()}_aggregate.json'
    cmd = [
        sys.executable,
        str(AGGREGATOR),
        '--log-dir', str(tmp_path),
        '--run-ids', *run_ids,
        '--phase', phase,
        '--output-json', str(output),
    ]
    result = subprocess.run(cmd, text=True, capture_output=True, check=False)
    return result, output


def write_case_artifacts(root: Path, run_id: str, *, condition='trajectory_evidence_present_without_critic_stats', degenerate=0, cost_weight=3.81):
    coverage = {
        'phase': '26R',
        'pass_coverage': True,
        'summary': {
            'case_count': 1,
            'evidence_size_bytes': 1000,
            'all_required_topics_covered_case_count': 1,
            'trajectory_summary_metrics_present_case_count': 1,
        },
        'cases': [{
            'run_id': run_id,
            'goal_sequence': 2,
            'all_required_topics_covered': True,
            'required_topics': {
                '/trajectories': {'covered': True, 'sample_count': 2},
                '/optimal_trajectory': {'covered': True, 'sample_count': 2},
                '/transformed_global_plan': {'covered': True, 'sample_count': 2},
            },
            'trajectory_summary_metrics': {
                'sample_count': 2,
                'marker_count_max': 7000,
                'degenerate_trajectory_count_max': degenerate,
                'representative_path_length_max': 0.9,
            },
            'condition_hypothesis': condition,
        }],
        'decision': {'phase27_candidate_signal': 'not_supported', 'intervention_allowed': False},
    }
    analysis = {
        'summary': {'condition_hypothesis_counts': {condition: 1}, 'real_mppi_evidence_case_count': 1},
        'cases': [{
            'run_id': run_id,
            'goal_sequence': 2,
            'condition_hypothesis': condition,
            'trajectory_summary': coverage['cases'][0]['trajectory_summary_metrics'],
        }],
        'decision': {'phase27_candidate_signal': 'not_supported'},
    }
    params_dir = root / f'{run_id}_runtime_params'
    params_dir.mkdir(parents=True, exist_ok=True)
    write_json(root / f'{run_id}_phase26r_summary_evidence_coverage.json', coverage)
    write_json(root / f'{run_id}_phase26p_mppi_evidence_analysis.json', analysis)
    write_json(params_dir / 'controller_server_summary.json', {'controller_server': {'FollowPath': {'CostCritic': {'cost_weight': cost_weight}}}})
    (root / f'{run_id}_mppi_evidence_summary.jsonl').write_text('{"event":"summary"}\n', encoding='utf-8')


def test_phase26t_candidate_comparison_is_evidence_only_and_never_promotion(tmp_path):
    write_case_artifacts(tmp_path, 'phase26r_baseline_summary_run1', condition='trajectory_evidence_present_without_critic_stats', degenerate=0, cost_weight=3.81)
    write_case_artifacts(tmp_path, 'phase26r_baseline_summary_run2', condition='trajectory_evidence_present_without_critic_stats', degenerate=0, cost_weight=3.81)
    write_case_artifacts(tmp_path, 'phase26r_candidate_summary_run1', condition='trajectory_evidence_present_without_critic_stats', degenerate=0, cost_weight=2.75)
    result, output = run_aggregate(
        tmp_path,
        ['phase26r_baseline_summary_run1', 'phase26r_baseline_summary_run2', 'phase26r_candidate_summary_run1'],
    )
    assert result.returncode == 0, result.stderr
    data = json.loads(output.read_text(encoding='utf-8'))
    assert data['phase'] == '26T'
    assert data['summary']['baseline_run_count'] == 2
    assert data['summary']['candidate_run_count'] == 1
    assert data['summary']['candidate_comparison']['mode'] == 'evidence_only_not_promotion'
    assert data['summary']['candidate_comparison']['candidate_condition_counts'] == {'trajectory_evidence_present_without_critic_stats': 1}
    assert data['summary']['candidate_comparison']['candidate_has_specific_condition'] is False
    assert data['summary']['candidate_comparison']['candidate_condition_differs_from_baseline'] is False
    assert data['decision']['phase27_candidate_signal'] == 'not_supported'
    assert data['decision']['intervention_allowed'] is False
    assert data['decision']['candidate_promotion_allowed'] is False
    assert data['decision']['candidate_rejection_allowed'] is False
    assert data['decision']['reason'] == 'candidate_condition_not_specific_or_not_different'
    assert 'do_not_promote_or_reject_candidate_from_phase26t' in data['decision']['guardrails']


def test_phase26t_candidate_specific_difference_is_review_only_not_intervention(tmp_path):
    write_case_artifacts(tmp_path, 'phase26r_baseline_summary_run1', condition='trajectory_evidence_present_without_critic_stats', cost_weight=3.81)
    write_case_artifacts(tmp_path, 'phase26r_baseline_summary_run2', condition='trajectory_evidence_present_without_critic_stats', cost_weight=3.81)
    write_case_artifacts(tmp_path, 'phase26r_candidate_summary_run1', condition='trajectory_summary_degenerate_without_critic_stats', degenerate=5, cost_weight=2.75)
    result, output = run_aggregate(
        tmp_path,
        ['phase26r_baseline_summary_run1', 'phase26r_baseline_summary_run2', 'phase26r_candidate_summary_run1'],
    )
    assert result.returncode == 0, result.stderr
    data = json.loads(output.read_text(encoding='utf-8'))
    assert data['summary']['candidate_comparison']['candidate_has_specific_condition'] is True
    assert data['summary']['candidate_comparison']['candidate_condition_differs_from_baseline'] is True
    assert data['decision']['phase27_candidate_signal'] == 'review_only'
    assert data['decision']['intervention_allowed'] is False
    assert data['decision']['reason'] == 'candidate_specific_difference_requires_human_review_before_phase27'
