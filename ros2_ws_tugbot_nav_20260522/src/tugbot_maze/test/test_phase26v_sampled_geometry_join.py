import importlib.util
import json
import subprocess
import sys
import types
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
RECORDER = ROOT / 'tools' / 'record_phase26p_mppi_evidence.py'
JOIN = ROOT / 'tools' / 'analyze_phase26v_sampled_path_local_cost_join.py'
WRAPPER = ROOT / 'tools' / 'run_phase21_controller_diagnostics_smoke.sh'


def write_json(path: Path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2), encoding='utf-8')


def write_jsonl(path: Path, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text('\n'.join(json.dumps(row) for row in rows) + '\n', encoding='utf-8')


def import_recorder_with_stubs():
    rclpy = types.ModuleType('rclpy')
    rclpy.init = lambda: None
    rclpy.ok = lambda: False
    rclpy.spin_once = lambda *args, **kwargs: None
    rclpy.shutdown = lambda: None
    executors = types.ModuleType('rclpy.executors')
    class ExternalShutdownException(Exception):
        pass
    executors.ExternalShutdownException = ExternalShutdownException
    node_mod = types.ModuleType('rclpy.node')
    class Node:
        pass
    node_mod.Node = Node
    convert = types.ModuleType('rosidl_runtime_py.convert')
    convert.message_to_ordereddict = lambda msg: msg
    utilities = types.ModuleType('rosidl_runtime_py.utilities')
    utilities.get_message = lambda name: object
    rosidl = types.ModuleType('rosidl_runtime_py')
    sys.modules['rclpy'] = rclpy
    sys.modules['rclpy.executors'] = executors
    sys.modules['rclpy.node'] = node_mod
    sys.modules['rosidl_runtime_py'] = rosidl
    sys.modules['rosidl_runtime_py.convert'] = convert
    sys.modules['rosidl_runtime_py.utilities'] = utilities
    spec = importlib.util.spec_from_file_location('phase26p_recorder_under_test', RECORDER)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_phase26v_recorder_bounded_path_and_marker_geometry_summaries():
    module = import_recorder_with_stubs()
    path_data = {
        'poses': [
            {'pose': {'position': {'x': float(i), 'y': float(i % 2), 'z': 0.0}}}
            for i in range(10)
        ]
    }
    summary = module.summarize_path(path_data, sample_limit=4)
    assert summary['summary_kind'] == 'path_summary'
    assert summary['point_count'] == 10
    assert len(summary['sampled_points']) == 4
    assert summary['sampled_points'][0] == [0.0, 0.0, 0.0]
    assert summary['sampled_points'][-1] == [9.0, 1.0, 0.0]
    assert summary['sampled_point_count'] == 4

    marker_data = {
        'markers': [
            {
                'ns': 'candidate',
                'id': 1,
                'type': 4,
                'action': 0,
                'scale': {'x': 0.03, 'y': 0.04, 'z': 0.05},
                'color': {'r': 1.0, 'g': 0.5, 'b': 0.25, 'a': 0.8},
                'points': [{'x': 0.0, 'y': 0.0, 'z': 0.0}, {'x': 0.2, 'y': 0.0, 'z': 0.0}],
            },
            {'ns': 'delete', 'id': 2, 'type': 4, 'action': 2, 'points': []},
        ]
    }
    marker_summary = module.summarize_marker_array(marker_data, sample_limit=3)
    assert marker_summary['marker_count'] == 2
    assert marker_summary['marker_type_counts'] == {'4': 2}
    assert marker_summary['marker_action_counts'] == {'0': 1, '2': 1}
    assert marker_summary['marker_namespace_counts'] == {'candidate': 1, 'delete': 1}
    assert marker_summary['sampled_marker_point_count'] == 2
    assert marker_summary['sampled_marker_points'][1]['point'] == [0.2, 0.0, 0.0]
    assert marker_summary['scale_summary']['x']['max'] == 0.03
    assert marker_summary['color_alpha_summary']['max'] == 0.8


def test_phase26v_join_detects_sampled_optimal_path_high_cost_intersection(tmp_path):
    run_id = 'phase26v_baseline_geometry_run9'
    log_dir = tmp_path / 'log'
    write_json(log_dir / f'{run_id}_phase26p_single_goal_timeline.json', {
        'cases': [{
            'run_id': run_id,
            'goal_sequence': 2,
            'cmd_near_zero_relation': {'first_cmd_near_zero_time': 101.0, 'recovery_time': 100.0},
        }],
    })
    write_jsonl(log_dir / f'{run_id}_mppi_evidence_summary.jsonl', [
        {
            'event': 'message',
            'topic': '/optimal_trajectory',
            'wall_time': 100.9,
            'data_summary': {
                'summary_kind': 'path_summary',
                'path_length': 0.4,
                'path_displacement': 0.4,
                'sampled_points': [[0.0, 0.0, 0.0], [0.5, 0.5, 0.0], [1.0, 1.0, 0.0]],
            },
        },
        {
            'event': 'message',
            'topic': '/transformed_global_plan',
            'wall_time': 100.95,
            'data_summary': {
                'summary_kind': 'path_summary',
                'path_length': 1.2,
                'path_displacement': 1.0,
                'sampled_points': [[2.0, 2.0, 0.0], [3.0, 3.0, 0.0]],
            },
        },
    ])
    write_json(log_dir / f'{run_id}_post_recovery_enriched.json', {
        'enriched_recovery_snapshots': [{
            'goal_sequence': 2,
            'near_zero_snapshot': {
                'wall_time': 101.02,
                'path_ahead_1_0m_cost_max': 99,
                'path_ahead_1_0m_high_cost_points': [[0.52, 0.49], [0.6, 0.6]],
                'robot_pose': [0.0, 0.0, 0.0],
                'robot_to_path_distance_m': 0.05,
            },
        }],
    })
    output = tmp_path / 'join.json'
    result = subprocess.run([
        sys.executable, str(JOIN),
        '--log-dir', str(log_dir),
        '--run-ids', run_id,
        '--output-json', str(output),
    ], text=True, capture_output=True, check=False)
    assert result.returncode == 0, result.stderr
    data = json.loads(output.read_text(encoding='utf-8'))
    case = data['cases'][0]
    assert case['optimal_path_intersects_high_cost'] is True
    assert case['transformed_plan_intersects_high_cost'] is False
    assert case['optimal_path_min_distance_to_high_cost'] < 0.05
    assert case['condition_hypothesis'] == 'sampled_optimal_path_intersects_high_cost_choke'
    assert data['decision']['intervention_allowed'] is False
    assert data['decision']['phase27_candidate_signal'] == 'review_only'


def test_phase26v_wrapper_run_ids_are_diagnostics_only_aliases():
    source = WRAPPER.read_text(encoding='utf-8')
    assert 'phase26v_baseline_geometry_run[0-9]+' in source
    assert 'phase26v_candidate_geometry_run[0-9]+' in source
    assert 'PHASE26V_BASELINE_GEOMETRY_PROFILE' in source
    assert 'PHASE26V_CANDIDATE_GEOMETRY_PROFILE' in source
    assert 'tools/analyze_phase26v_sampled_path_local_cost_join.py' in source
    assert '--sample-points' in source
