import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
LOG_PARSER = ROOT / 'tools' / 'analyze_phase26u_controller_log_signals.py'
SEMANTICS_AUDIT = ROOT / 'tools' / 'audit_phase26u_mppi_topic_semantics.py'
LOCAL_JOIN = ROOT / 'tools' / 'analyze_phase26u_optimal_path_local_cost_join.py'


def write_json(path: Path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2), encoding='utf-8')


def write_jsonl(path: Path, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text('\n'.join(json.dumps(row) for row in rows) + '\n', encoding='utf-8')


def test_phase26u_controller_log_parser_extracts_windowed_signals(tmp_path):
    run_id = 'phase26r_baseline_summary_run9'
    log_dir = tmp_path / 'log'
    log_dir.mkdir()
    write_json(log_dir / f'{run_id}_phase26p_single_goal_timeline.json', {
        'cases': [{
            'run_id': run_id,
            'goal_sequence': 2,
            'cmd_near_zero_relation': {
                'recovery_time': 100.0,
                'first_cmd_near_zero_time': 101.0,
                'progress_failure_time': 100.5,
                'controller_abort_time': 100.6,
            },
            'timeline': [
                {'label': 'recovery_clear_costmap', 'time': 100.0},
                {'label': 'progress_failure', 'time': 100.5},
                {'label': 'controller_abort', 'time': 100.6},
                {'label': 'first_cmd_near_zero_after_recovery', 'time': 101.0},
            ],
        }],
    })
    write_jsonl(log_dir / f'{run_id}_goal_events.jsonl', [
        {'event': 'dispatch', 'goal_sequence': 2, 'wall_time': 90.0},
        {'event': 'timeout_cancel_result', 'goal_sequence': 2, 'wall_time': 110.0},
    ])
    (log_dir / f'{run_id}_launch.log').write_text(
        '[controller_server-7] [ERROR] [100.500000000] [controller_server]: Failed to make progress\n'
        '[controller_server-7] [WARN] [100.600000000] [controller_server]: [follow_path] [ActionServer] Aborting handle.\n'
        '[controller_server-7] [WARN] [101.100000000] [MPPIController]: No valid trajectories found; zero velocity command selected\n'
        '[controller_server-7] [WARN] [101.200000000] [MPPIController]: near collision constraint from CostCritic and PathAlignCritic\n',
        encoding='utf-8',
    )
    write_json(log_dir / f'{run_id}_phase26p_mppi_evidence_analysis.json', {
        'summary': {'condition_hypothesis_counts': {'trajectory_evidence_present_without_critic_stats': 1}},
        'cases': [{'run_id': run_id, 'goal_sequence': 2, 'condition_hypothesis': 'trajectory_evidence_present_without_critic_stats'}],
    })
    output = tmp_path / 'signals.json'
    result = subprocess.run([
        sys.executable, str(LOG_PARSER),
        '--log-dir', str(log_dir),
        '--run-ids', run_id,
        '--output-json', str(output),
    ], text=True, capture_output=True, check=False)
    assert result.returncode == 0, result.stderr
    data = json.loads(output.read_text(encoding='utf-8'))
    assert data['phase'] == '26U'
    assert data['analysis_only'] is True
    case = data['cases'][0]
    assert case['window']['first_cmd_near_zero_time'] == 101.0
    assert case['signals']['progress_failure_count'] == 1
    assert case['signals']['controller_abort_count'] == 1
    assert case['signals']['no_valid_control_count'] == 1
    assert case['signals']['near_collision_count'] == 1
    assert case['signals']['critic_keyword_counts']['CostCritic'] == 1
    assert case['condition_hypothesis'] == 'controller_no_valid_control_or_invalid_trajectory_log_signal'
    assert data['decision']['intervention_allowed'] is False
    assert data['decision']['phase27_candidate_signal'] == 'review_only'


def test_phase26u_mppi_topic_semantics_audit_explains_markerarray_limitations(tmp_path):
    ros_prefix = tmp_path / 'ros'
    header = ros_prefix / 'include/nav2_mppi_controller/tools/trajectory_visualizer.hpp'
    header.parent.mkdir(parents=True)
    header.write_text(
        'std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>> trajectories_publisher_;\n'
        'std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> transformed_path_pub_;\n'
        'std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> optimal_path_pub_;\n'
        'void add(const models::Trajectories & trajectories, const std::string & marker_namespace);\n'
        'void add(const xt::xtensor<float, 2> & trajectory, const std::string & marker_namespace, const builtin_interfaces::msg::Time & cmd_stamp);\n'
        'void visualize(const nav_msgs::msg::Path & plan);\n',
        encoding='utf-8',
    )
    lib = ros_prefix / 'lib/libmppi_controller.so'
    lib.parent.mkdir(parents=True)
    lib.write_text('/trajectories\noptimal_trajectory\ntransformed_global_plan\nvisualize\n', encoding='utf-8')
    evidence = tmp_path / 'summary.jsonl'
    write_jsonl(evidence, [{
        'event': 'message',
        'topic': '/trajectories',
        'msg_type': 'visualization_msgs/msg/MarkerArray',
        'data_summary': {
            'summary_kind': 'marker_array_trajectory_summary',
            'marker_count': 7656,
            'point_count': 0,
            'representative_path_length': None,
        },
    }])
    output = tmp_path / 'semantics.json'
    result = subprocess.run([
        sys.executable, str(SEMANTICS_AUDIT),
        '--ros-prefix', str(ros_prefix),
        '--sample-evidence', str(evidence),
        '--output-json', str(output),
    ], text=True, capture_output=True, check=False)
    assert result.returncode == 0, result.stderr
    data = json.loads(output.read_text(encoding='utf-8'))
    assert data['phase'] == '26U'
    assert data['topics']['/trajectories']['message_type'] == 'visualization_msgs/msg/MarkerArray'
    assert data['topics']['/trajectories']['sample_marker_count'] == 7656
    assert data['topics']['/trajectories']['sample_point_count'] == 0
    assert data['topics']['/trajectories']['geometry_limit'] == 'markerarray_points_empty_in_compact_summary'
    assert data['topics']['/optimal_trajectory']['message_type'] == 'nav_msgs/msg/Path'
    assert data['topics']['/transformed_global_plan']['message_type'] == 'nav_msgs/msg/Path'
    assert data['decision']['phase27_candidate_signal'] == 'not_supported'


def test_phase26u_local_cost_join_flags_high_cost_choke_but_geometry_gap(tmp_path):
    run_id = 'phase26r_candidate_summary_run9'
    log_dir = tmp_path / 'log'
    write_json(log_dir / f'{run_id}_phase26p_single_goal_timeline.json', {
        'cases': [{
            'run_id': run_id,
            'goal_sequence': 2,
            'cmd_near_zero_relation': {'recovery_time': 100.0, 'first_cmd_near_zero_time': 101.0},
        }],
    })
    write_jsonl(log_dir / f'{run_id}_mppi_evidence_summary.jsonl', [
        {'event': 'message', 'topic': '/optimal_trajectory', 'wall_time': 100.9, 'data_summary': {'summary_kind': 'path_summary', 'point_count': 56, 'path_displacement': 0.00005, 'path_length': 0.00006}},
        {'event': 'message', 'topic': '/transformed_global_plan', 'wall_time': 100.95, 'data_summary': {'summary_kind': 'path_summary', 'point_count': 20, 'path_displacement': 0.8, 'path_length': 1.0}},
    ])
    write_json(log_dir / f'{run_id}_phase26p_mppi_evidence_analysis.json', {
        'cases': [{
            'run_id': run_id,
            'goal_sequence': 2,
            'optimal_trajectory': {'sample_count': 1, 'path_sample_count': 1, 'zero_displacement_path_sample_count': 1, 'path_displacement_min': 0.00005},
            'trajectory_summary': {'sample_count': 1, 'degenerate_trajectory_count_max': 0.0},
        }],
    })
    write_json(log_dir / f'{run_id}_post_recovery_enriched.json', {
        'enriched_recovery_snapshots': [{
            'goal_sequence': 2,
            'near_zero_age_sec': 0.02,
            'near_zero_path_ahead_1_0m_cost_max': 99,
            'near_zero_robot_to_path_distance_m': 0.04,
            'near_zero_snapshot': {
                'wall_time': 101.02,
                'path_ahead_1_0m_cost_max': 99,
                'path_ahead_1_0m_high_cost_count': 6,
                'path_ahead_1_0m_nearest_high_cost_point': [0.5, 0.8],
                'robot_pose': [0.0, 0.0, 0.0],
                'robot_to_path_distance_m': 0.04,
            },
        }],
    })
    output = tmp_path / 'join.json'
    result = subprocess.run([
        sys.executable, str(LOCAL_JOIN),
        '--log-dir', str(log_dir),
        '--run-ids', run_id,
        '--output-json', str(output),
    ], text=True, capture_output=True, check=False)
    assert result.returncode == 0, result.stderr
    data = json.loads(output.read_text(encoding='utf-8'))
    case = data['cases'][0]
    assert case['spatial_join']['optimal_path_geometry_available'] is False
    assert case['spatial_join']['spatial_join_possible'] is False
    assert case['local_cost_choke_evidence']['high_cost_choke_near_zero'] is True
    assert case['optimal_trajectory_summary']['zero_displacement_path_sample_count'] == 1
    assert case['condition_hypothesis'] == 'near_zero_with_local_path_high_cost_but_optimal_path_geometry_missing'
    assert data['decision']['phase27_candidate_signal'] == 'not_supported'
    assert 'raw_or_sampled_path_points_needed' in data['decision']['missing_evidence']
