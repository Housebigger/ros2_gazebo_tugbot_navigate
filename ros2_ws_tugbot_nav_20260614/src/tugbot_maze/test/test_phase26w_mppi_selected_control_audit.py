import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
AUDIT = ROOT / 'tools' / 'audit_phase26w_mppi_selected_control_path.py'


def test_phase26w_audit_reports_selected_control_and_visualization_path(tmp_path):
    output = tmp_path / 'phase26w_audit.json'
    result = subprocess.run([
        sys.executable,
        str(AUDIT),
        '--ros-prefix', '/opt/ros/jazzy',
        '--workspace-root', str(ROOT),
        '--output-json', str(output),
    ], text=True, capture_output=True, check=False)
    assert result.returncode == 0, result.stderr
    data = json.loads(output.read_text(encoding='utf-8'))

    assert data['phase'] == '26W'
    assert data['analysis_only'] is True
    assert data['decision']['intervention_allowed'] is False
    assert data['decision']['phase27_candidate_signal'] == 'not_supported'
    assert data['decision']['phase26x_instrumentation_needed'] is True

    selected = data['selected_control_path']
    assert selected['controller_entrypoint']['symbol_or_declaration_present'] is True
    assert selected['optimizer_entrypoint']['symbol_or_declaration_present'] is True
    assert selected['control_sequence_to_twist']['symbol_or_declaration_present'] is True
    assert selected['selected_control_near_zero_origin_located'] is True
    assert selected['near_zero_reason_observable_from_installed_artifacts'] is False

    relation = data['topic_to_command_relation']
    assert relation['optimal_trajectory_relation_to_cmd_vel'] == 'visualization_of_optimized_trajectory_not_the_cmd_vel_publish_path'
    assert relation['cmd_vel_origin'] == 'Optimizer::getControlFromSequenceAsTwist'

    marker = data['trajectories_marker_count_explanation']
    assert marker['observed_marker_count'] == 7656
    assert marker['derived_from_batch_size_time_steps_stride'] is True
    assert marker['likely_formula'] == 'ceil(batch_size / trajectory_step) * ceil(time_steps / time_step)'


def test_phase26w_audit_finds_fallback_constraints_and_instrumentation_points(tmp_path):
    output = tmp_path / 'phase26w_audit.json'
    subprocess.run([
        sys.executable,
        str(AUDIT),
        '--ros-prefix', '/opt/ros/jazzy',
        '--workspace-root', str(ROOT),
        '--output-json', str(output),
    ], text=True, capture_output=True, check=True)
    data = json.loads(output.read_text(encoding='utf-8'))

    evidence = data['installed_evidence']
    assert evidence['headers']['optimizer.hpp']['exists'] is True
    assert 'fallback' in evidence['headers']['optimizer.hpp']['hits']
    assert 'getControlFromSequenceAsTwist' in evidence['library_strings']['hits']
    assert 'Optimizer fail to compute path' in evidence['library_strings']['hits']
    assert 'retry_attempt_limit' in evidence['library_strings']['hits']

    controls = data['control_constraints_and_failure_paths']
    assert controls['constraints_declared'] is True
    assert controls['fallback_declared'] is True
    assert controls['retry_attempt_limit_declared'] is True
    assert 'vx_max' in controls['runtime_param_names']
    assert 'wz_max' in controls['runtime_param_names']

    plan = data['phase26x_instrumentation_plan']
    assert plan['recommended'] is True
    assert plan['scope'] == 'diagnostics_only_near_zero_cycles'
    assert any('getControlFromSequenceAsTwist' in point for point in plan['minimal_points'])
    assert any('evalControl' in point for point in plan['minimal_points'])
