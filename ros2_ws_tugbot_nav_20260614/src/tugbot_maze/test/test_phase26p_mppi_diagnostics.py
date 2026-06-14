import json
import subprocess
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[3]
CONFIG_DIR = ROOT / 'src' / 'tugbot_navigation' / 'config'
BASELINE = CONFIG_DIR / 'nav2_slam_params.yaml'
DIAG_BASELINE = CONFIG_DIR / 'nav2_slam_phase26p_mppi_diagnostics_params.yaml'
CANDIDATE = CONFIG_DIR / 'nav2_slam_candidate_costcritic_275_params.yaml'
DIAG_CANDIDATE = CONFIG_DIR / 'nav2_slam_phase26p_candidate_mppi_diagnostics_params.yaml'
WRAPPER = ROOT / 'tools' / 'run_phase21_controller_diagnostics_smoke.sh'
TOPIC_RECORDER = ROOT / 'tools' / 'record_phase26p_mppi_evidence.py'
ANALYZER = ROOT / 'tools' / 'analyze_phase26p_mppi_evidence.py'


def load_yaml(path: Path):
    return yaml.safe_load(path.read_text(encoding='utf-8'))


def follow_path(data):
    return data['controller_server']['ros__parameters']['FollowPath']


def without_debug_keys(data):
    clone = json.loads(json.dumps(data))
    fp = follow_path(clone)
    fp.pop('publish_critics_stats', None)
    fp.pop('publish_optimal_trajectory', None)
    return clone


def test_phase26p_diagnostics_profiles_only_enable_mppi_debug_evidence():
    base = load_yaml(BASELINE)
    diag = load_yaml(DIAG_BASELINE)
    candidate = load_yaml(CANDIDATE)
    diag_candidate = load_yaml(DIAG_CANDIDATE)

    assert follow_path(diag)['publish_critics_stats'] is True
    assert follow_path(diag)['publish_optimal_trajectory'] is True
    assert follow_path(diag_candidate)['publish_critics_stats'] is True
    assert follow_path(diag_candidate)['publish_optimal_trajectory'] is True

    assert without_debug_keys(diag) == without_debug_keys(base)
    assert without_debug_keys(diag_candidate) == without_debug_keys(candidate)

    assert follow_path(diag)['CostCritic'] == follow_path(base)['CostCritic']
    assert follow_path(diag)['PathAlignCritic'] == follow_path(base)['PathAlignCritic']
    assert follow_path(diag)['PathFollowCritic'] == follow_path(base)['PathFollowCritic']
    assert follow_path(diag)['GoalCritic'] == follow_path(base)['GoalCritic']


def test_phase26p_wrapper_accepts_diagnostics_run_ids_and_records_mppi_topics():
    source = WRAPPER.read_text(encoding='utf-8')
    assert 'phase26p_baseline_diag_run[0-9]+' in source
    assert 'phase26p_candidate_diag_run[0-9]+' in source
    assert 'nav2_slam_phase26p_mppi_diagnostics_params.yaml' in source
    assert 'nav2_slam_phase26p_candidate_mppi_diagnostics_params.yaml' in source
    assert 'PHASE26P_MPPI_EVIDENCE="$LOG_DIR/${RUN_ID}_mppi_evidence_summary.jsonl"' in source
    assert 'tools/record_phase26p_mppi_evidence.py' in source
    assert '--topic-regex' in source
    assert 'critics_stats|optimal_trajectory|transformed_global_plan|trajectories|trajectory' in source
    assert 'phase26p_mppi_diagnostics_profile' in source
    assert 'PHASE_RUN_MAX_GOALS' in source
    assert 'MAX_GOALS="${PHASE_RUN_MAX_GOALS:-12}"' in source


def test_phase26p_topic_recorder_and_analyzer_contract_with_synthetic_critic_stats(tmp_path):
    assert TOPIC_RECORDER.exists()
    recorder_source = TOPIC_RECORDER.read_text(encoding='utf-8')
    assert 'ExternalShutdownException' in recorder_source
    assert 'except ExternalShutdownException' in recorder_source
    evidence = tmp_path / 'mppi_evidence.jsonl'
    timeline = tmp_path / 'timeline.json'
    output = tmp_path / 'phase26p.json'
    timeline.write_text(json.dumps({
        'phase': '26N',
        'cases': [{
            'run_id': 'synthetic',
            'goal_sequence': 7,
            'cmd_near_zero_relation': {
                'recovery_time': 100.0,
                'first_cmd_near_zero_time': 100.6,
                'first_cmd_near_zero_after_recovery_sec': 0.6,
            },
            'local_cost_windows': {
                'first_high_cost_window_after_recovery_sec': 0.1,
                'high_cost_window_count_after_recovery': 2,
            },
        }],
    }), encoding='utf-8')
    rows = [
        {
            'wall_time': 99.8,
            'topic': '/controller_server/FollowPath/critics_stats',
            'msg_type': 'nav2_msgs/msg/CriticsStats',
            'data': {
                'critics': [
                    {'name': 'CostCritic', 'changed': True, 'cost': 9000.0},
                    {'name': 'PathAlignCritic', 'changed': False, 'cost': 0.0},
                    {'name': 'PathFollowCritic', 'changed': True, 'cost': 20.0},
                ]
            },
        },
        {
            'wall_time': 100.4,
            'topic': '/controller_server/FollowPath/critics_stats',
            'msg_type': 'nav2_msgs/msg/CriticsStats',
            'data': {
                'critics': [
                    {'name': 'CostCritic', 'changed': True, 'cost': 1200000.0},
                    {'name': 'PathAlignCritic', 'changed': True, 'cost': 80.0},
                    {'name': 'PathFollowCritic', 'changed': False, 'cost': 0.0},
                ]
            },
        },
        {
            'wall_time': 100.5,
            'topic': '/optimal_trajectory',
            'msg_type': 'nav_msgs/msg/Path',
            'data': {
                'poses': [
                    {'pose': {'position': {'x': 1.0, 'y': 1.0}}},
                    {'pose': {'position': {'x': 1.0, 'y': 1.0}}},
                    {'pose': {'position': {'x': 1.0, 'y': 1.0}}},
                ]
            },
        },
    ]
    evidence.write_text('\n'.join(json.dumps(row) for row in rows) + '\n', encoding='utf-8')
    result = subprocess.run([
        sys.executable,
        str(ANALYZER),
        '--timeline-json', str(timeline),
        '--mppi-evidence', str(evidence),
        '--output-json', str(output),
    ], text=True, capture_output=True, check=False)
    assert result.returncode == 0, result.stderr
    data = json.loads(output.read_text(encoding='utf-8'))
    case = data['cases'][0]
    assert data['phase'] == '26P'
    assert data['analysis_only'] is True
    assert data['decision']['phase27_candidate_signal'] == 'not_supported'
    assert data['decision']['intervention_signal'] == 'blocked_until_real_run_evidence'
    assert case['join_window']['start_offset_sec'] == -1.0
    assert case['join_window']['end_offset_sec'] == 1.0
    assert case['critic_stats']['sample_count'] == 2
    assert case['critic_stats']['top_critic_by_max_cost']['name'] == 'CostCritic'
    assert case['critic_stats']['top_critic_by_max_cost']['max_cost'] == 1200000.0
    assert case['optimal_trajectory']['sample_count'] == 1
    assert case['optimal_trajectory']['path_sample_count'] == 1
    assert case['optimal_trajectory']['zero_displacement_path_sample_count'] == 1
    assert case['condition_hypothesis'] == 'costcritic_high_cost_with_degenerate_optimal_path'
