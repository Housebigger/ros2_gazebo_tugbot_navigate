import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
WRAPPER = ROOT / 'tools' / 'run_phase21_controller_diagnostics_smoke.sh'
CHECKER = ROOT / 'tools' / 'check_phase26r_summary_evidence_coverage.py'


def write_json(path: Path, data):
    path.write_text(json.dumps(data, indent=2), encoding='utf-8')


def write_jsonl(path: Path, rows):
    path.write_text('\n'.join(json.dumps(row) for row in rows) + '\n', encoding='utf-8')


def test_phase26r_wrapper_accepts_summary_smoke_run_ids_without_new_intervention_profile():
    source = WRAPPER.read_text(encoding='utf-8')
    assert 'phase26r_baseline_summary_run[0-9]+' in source
    assert 'phase26r_candidate_summary_run[0-9]+' in source
    assert 'PHASE26R_COVERAGE="$LOG_DIR/${RUN_ID}_phase26r_summary_evidence_coverage.json"' in source
    assert 'tools/check_phase26r_summary_evidence_coverage.py' in source
    assert 'PHASE26P_BASELINE_DIAG_PROFILE=true' in source
    assert 'PHASE26P_CANDIDATE_DIAG_PROFILE=true' in source
    assert 'phase26r_summary_smoke' in source
    assert 'phase27' not in source.lower()[source.index('phase26r_baseline_summary_run'):source.index('NAV2_PARAMS_FILE=')]


def test_phase26r_coverage_checker_requires_all_summary_topics_in_first_cmd_window(tmp_path):
    timeline = tmp_path / 'timeline.json'
    evidence = tmp_path / 'summary.jsonl'
    analysis = tmp_path / 'analysis.json'
    output = tmp_path / 'coverage.json'
    timeline_time = 100.0
    write_json(timeline, {
        'cases': [{
            'run_id': 'synthetic_phase26r',
            'goal_sequence': 2,
            'cmd_near_zero_relation': {'first_cmd_near_zero_time': timeline_time},
        }]
    })
    write_jsonl(evidence, [
        {
            'event': 'message',
            'wall_time': 99.4,
            'topic': '/trajectories',
            'msg_type': 'visualization_msgs/msg/MarkerArray',
            'data_summary': {
                'summary_kind': 'marker_array_trajectory_summary',
                'marker_count': 3,
                'point_count': 12,
                'trajectory_count': 3,
                'representative_path_length': 0.9,
                'degenerate_trajectory_count': 0,
            },
        },
        {
            'event': 'message',
            'wall_time': 100.1,
            'topic': '/optimal_trajectory',
            'msg_type': 'nav_msgs/msg/Path',
            'data_summary': {'summary_kind': 'path_summary', 'point_count': 5, 'path_displacement': 0.4, 'path_length': 0.5},
        },
        {
            'event': 'message',
            'wall_time': 100.8,
            'topic': '/transformed_global_plan',
            'msg_type': 'nav_msgs/msg/Path',
            'data_summary': {'summary_kind': 'path_summary', 'point_count': 8, 'path_displacement': 1.2, 'path_length': 1.4},
        },
    ])
    write_json(analysis, {
        'phase': '26Q',
        'cases': [{
            'run_id': 'synthetic_phase26r',
            'goal_sequence': 2,
            'trajectory_summary': {
                'sample_count': 1,
                'marker_count_max': 3,
                'point_count_max': 12,
                'degenerate_trajectory_count_max': 0,
                'representative_path_length_max': 0.9,
            },
            'condition_hypothesis': 'trajectory_evidence_present_without_critic_stats',
        }],
        'decision': {'phase27_candidate_signal': 'not_supported'},
    })
    result = subprocess.run([
        sys.executable,
        str(CHECKER),
        '--timeline-json', str(timeline),
        '--mppi-evidence', str(evidence),
        '--analysis-json', str(analysis),
        '--output-json', str(output),
        '--max-evidence-bytes', '200000',
    ], text=True, capture_output=True, check=False)
    assert result.returncode == 0, result.stderr
    data = json.loads(output.read_text(encoding='utf-8'))
    assert data['phase'] == '26R'
    assert data['analysis_only'] is True
    assert data['pass_coverage'] is True
    assert data['summary']['evidence_size_bytes'] < 200000
    assert data['summary']['all_required_topics_covered_case_count'] == 1
    case = data['cases'][0]
    assert case['window']['start_offset_sec'] == -1.0
    assert case['window']['end_offset_sec'] == 1.0
    assert case['required_topics']['/trajectories']['covered'] is True
    assert case['required_topics']['/optimal_trajectory']['covered'] is True
    assert case['required_topics']['/transformed_global_plan']['covered'] is True
    assert case['trajectory_summary_metrics']['sample_count'] == 1
    assert data['decision']['phase27_candidate_signal'] == 'not_supported'
    assert data['decision']['intervention_allowed'] is False


def test_phase26r_coverage_checker_fails_when_required_topic_missing(tmp_path):
    timeline = tmp_path / 'timeline.json'
    evidence = tmp_path / 'summary.jsonl'
    analysis = tmp_path / 'analysis.json'
    output = tmp_path / 'coverage.json'
    write_json(timeline, {'cases': [{'run_id': 'synthetic_phase26r', 'goal_sequence': 2, 'cmd_near_zero_relation': {'first_cmd_near_zero_time': 100.0}}]})
    write_jsonl(evidence, [
        {'event': 'message', 'wall_time': 100.0, 'topic': '/trajectories', 'data_summary': {'summary_kind': 'marker_array_trajectory_summary'}},
        {'event': 'message', 'wall_time': 100.1, 'topic': '/optimal_trajectory', 'data_summary': {'summary_kind': 'path_summary'}},
    ])
    write_json(analysis, {'cases': [{'run_id': 'synthetic_phase26r', 'goal_sequence': 2, 'trajectory_summary': {'sample_count': 1}}]})
    result = subprocess.run([
        sys.executable,
        str(CHECKER),
        '--timeline-json', str(timeline),
        '--mppi-evidence', str(evidence),
        '--analysis-json', str(analysis),
        '--output-json', str(output),
    ], text=True, capture_output=True, check=False)
    assert result.returncode == 1
    data = json.loads(output.read_text(encoding='utf-8'))
    assert data['pass_coverage'] is False
    assert data['cases'][0]['required_topics']['/transformed_global_plan']['covered'] is False
