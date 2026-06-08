import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
RECORDER = ROOT / 'tools' / 'record_phase26p_mppi_evidence.py'
AUDIT = ROOT / 'tools' / 'audit_phase26q_mppi_debug_capabilities.py'
WRAPPER = ROOT / 'tools' / 'run_phase21_controller_diagnostics_smoke.sh'


def write_jsonl(path: Path, rows):
    path.write_text('\n'.join(json.dumps(row) for row in rows) + '\n', encoding='utf-8')


def marker(marker_id, points, namespace='Candidate Trajectories'):
    return {
        'id': marker_id,
        'ns': namespace,
        'points': [{'x': x, 'y': y, 'z': 0.0} for x, y in points],
    }


def test_phase26q_recorder_summarizes_marker_arrays_without_raw_payload():
    source = RECORDER.read_text(encoding='utf-8')
    assert 'summarize_message' in source
    assert 'summarize_marker_array' in source
    assert 'raw_topics_regex' in source
    assert "'data_summary': summary" in source
    assert "'data': data" not in source
    assert "'/trajectories'" in source


def test_phase26q_analyzer_uses_trajectory_summary_metrics(tmp_path):
    timeline = tmp_path / 'timeline.json'
    evidence = tmp_path / 'mppi_summary.jsonl'
    output = tmp_path / 'phase26q.json'
    timeline.write_text(json.dumps({
        'cases': [{
            'run_id': 'synthetic_phase26q',
            'goal_sequence': 2,
            'cmd_near_zero_relation': {
                'first_cmd_near_zero_time': 10.0,
                'first_cmd_near_zero_after_recovery_sec': 0.5,
            },
            'local_cost_windows': {
                'first_high_cost_window_after_recovery_sec': 0.1,
                'high_cost_window_count_after_recovery': 1,
            },
        }]
    }), encoding='utf-8')
    write_jsonl(evidence, [
        {
            'event': 'message',
            'wall_time': 9.7,
            'topic': '/trajectories',
            'msg_type': 'visualization_msgs/msg/MarkerArray',
            'data_summary': {
                'summary_kind': 'marker_array_trajectory_summary',
                'marker_count': 2,
                'point_count': 5,
                'trajectory_count': 2,
                'trajectory_displacement_min': 0.0,
                'trajectory_displacement_max': 0.5,
                'trajectory_displacement_mean': 0.25,
                'representative_path_length': 0.5,
                'degenerate_trajectory_count': 1,
                'near_zero_trajectory_count': 1,
            },
        },
        {
            'event': 'message',
            'wall_time': 9.9,
            'topic': '/optimal_trajectory',
            'msg_type': 'nav_msgs/msg/Path',
            'data_summary': {
                'summary_kind': 'path_summary',
                'point_count': 3,
                'path_displacement': 0.2,
                'path_length': 0.22,
            },
        },
    ])
    result = subprocess.run([
        sys.executable,
        str(ROOT / 'tools' / 'analyze_phase26p_mppi_evidence.py'),
        '--timeline-json', str(timeline),
        '--mppi-evidence', str(evidence),
        '--output-json', str(output),
    ], text=True, capture_output=True, check=False)
    assert result.returncode == 0, result.stderr
    data = json.loads(output.read_text(encoding='utf-8'))
    assert data['phase'] == '26Q'
    case = data['cases'][0]
    assert case['trajectory_summary']['sample_count'] == 1
    assert case['trajectory_summary']['marker_count_max'] == 2
    assert case['trajectory_summary']['point_count_max'] == 5
    assert case['trajectory_summary']['degenerate_trajectory_count_max'] == 1
    assert case['trajectory_summary']['representative_path_length_max'] == 0.5
    assert case['optimal_trajectory']['path_sample_count'] == 1
    assert case['condition_hypothesis'] == 'trajectory_summary_degenerate_without_critic_stats'
    assert data['decision']['phase27_candidate_signal'] == 'not_supported'


def test_phase26q_critics_stats_audit_marks_jazzy_mppi_unavailable(tmp_path):
    output = tmp_path / 'audit.json'
    result = subprocess.run([
        sys.executable,
        str(AUDIT),
        '--ros-prefix', '/opt/ros/jazzy',
        '--output-json', str(output),
    ], text=True, capture_output=True, check=False)
    assert result.returncode == 0, result.stderr
    data = json.loads(output.read_text(encoding='utf-8'))
    assert data['phase'] == '26Q'
    assert data['analysis_only'] is True
    assert data['critics_stats']['status'] == 'unavailable_in_installed_jazzy_mppi'
    assert data['critics_stats']['message_type_available'] is False
    assert data['critics_stats']['library_symbol_available'] is False
    assert data['critics_stats']['parameter_name_supported'] is False
    assert data['recommendation'] == 'do_not_block_on_critics_stats_use_summarized_trajectories_and_existing_logs'


def test_phase26q_wrapper_uses_summarized_mppi_evidence_filename():
    source = WRAPPER.read_text(encoding='utf-8')
    assert 'PHASE26P_MPPI_EVIDENCE="$LOG_DIR/${RUN_ID}_mppi_evidence_summary.jsonl"' in source
    assert '--raw-topics-regex' in source
    assert '^$' in source
