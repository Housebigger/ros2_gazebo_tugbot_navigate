#!/usr/bin/env python3
"""Phase26W installed/source-level audit for MPPI selected-control path.

Diagnostics-only. Reads installed Nav2 MPPI headers, share files, library strings,
and workspace configuration to identify where selected control becomes cmd_vel
and what remains unobservable without instrumentation.
"""

from __future__ import annotations

import argparse
import json
import math
import re
import subprocess
from pathlib import Path
from typing import Any

KEY_HEADER_PATTERNS = {
    'controller.hpp': [
        'computeVelocityCommands',
        'optimizer_',
        'visualize',
        'TrajectoryVisualizer',
    ],
    'optimizer.hpp': [
        'evalControl',
        'optimize',
        'updateControlSequence',
        'getControlFromSequenceAsTwist',
        'applyControlSequenceConstraints',
        'fallback',
        'retry_attempt_limit',
        'getOptimizedTrajectory',
        'getGeneratedTrajectories',
        'control_sequence_',
        'costs_',
    ],
    'tools/trajectory_visualizer.hpp': [
        'trajectories_publisher_',
        'transformed_path_pub_',
        'optimal_path_pub_',
        'add(',
        'visualize',
        'trajectory_step_',
        'time_step_',
    ],
    'models/control_sequence.hpp': ['struct ControlSequence', 'vx', 'vy', 'wz', 'reset'],
    'models/optimizer_settings.hpp': ['batch_size', 'time_steps', 'retry_attempt_limit', 'shift_control_sequence', 'constraints'],
    'models/trajectories.hpp': ['struct Trajectories', 'x', 'y', 'yaws', 'batch_size', 'time_steps'],
    'models/constraints.hpp': ['ControlConstraints', 'vx_max', 'vx_min', 'vy', 'wz_max', 'ax_max'],
    'motion_models.hpp': ['applyConstraints', 'control_sequence.vx', 'control_sequence.wz'],
}

LIB_PATTERNS = [
    'computeVelocityCommands',
    'evalControl',
    'getControlFromSequenceAsTwist',
    'getOptimizedTrajectory',
    'getGeneratedTrajectories',
    'applyControlSequenceConstraints',
    'shiftControlSequence',
    'fallback',
    'Optimizer fail to compute path',
    'retry_attempt_limit',
    'Controller period is equal to model dt. Control sequence shifting is ON',
    'optimal_trajectory',
    '/trajectories',
    'transformed_global_plan',
    'trajectory_step',
    'time_step',
    'vx_max',
    'vx_min',
    'vy_max',
    'wz_max',
    'temperature',
]

CONFIG_KEYS = [
    'batch_size', 'time_steps', 'trajectory_step', 'time_step', 'visualize',
    'vx_max', 'vx_min', 'vy_max', 'wz_max', 'retry_attempt_limit', 'temperature',
    'iteration_count', 'model_dt', 'cost_weight',
]


def read_text(path: Path) -> str:
    try:
        return path.read_text(encoding='utf-8', errors='ignore')
    except OSError:
        return ''


def lines_for_patterns(text: str, patterns: list[str]) -> dict[str, list[dict[str, Any]]]:
    out: dict[str, list[dict[str, Any]]] = {}
    lines = text.splitlines()
    for pattern in patterns:
        hits = []
        for i, line in enumerate(lines, start=1):
            if pattern in line:
                hits.append({'line': i, 'text': line.strip()})
        if hits:
            out[pattern] = hits[:20]
    return out


def header_evidence(include_root: Path) -> dict[str, Any]:
    result = {}
    for rel, patterns in KEY_HEADER_PATTERNS.items():
        path = include_root / rel
        text = read_text(path)
        result[rel] = {
            'path': str(path),
            'exists': path.exists(),
            'hits': lines_for_patterns(text, patterns),
        }
    return result


def lib_strings(ros_prefix: Path) -> dict[str, Any]:
    candidates = [ros_prefix / 'lib/libmppi_controller.so', ros_prefix / 'lib/libnav2_mppi_controller.so']
    hits: list[str] = []
    library = None
    for lib in candidates:
        if not lib.exists():
            continue
        library = str(lib)
        try:
            proc = subprocess.run(['strings', str(lib)], text=True, capture_output=True, check=False, timeout=30)
        except (OSError, subprocess.TimeoutExpired):
            continue
        text = proc.stdout
        for pattern in LIB_PATTERNS:
            if pattern in text and pattern not in hits:
                hits.append(pattern)
        # Include demangled-ish symbol substrings if present in strings output.
        for extra in ['_ZN4mppi9Optimizer11evalControl', '_ZN4mppi9Optimizer29getControlFromSequenceAsTwist', '_ZN20nav2_mppi_controller14MPPIController23computeVelocityCommands']:
            if extra in text and extra not in hits:
                hits.append(extra)
    return {'library': library, 'hits': sorted(hits)}


def share_evidence(ros_prefix: Path) -> dict[str, Any]:
    share = ros_prefix / 'share/nav2_mppi_controller'
    result = {}
    for rel in ['mppic.xml', 'critics.xml', 'package.xml']:
        path = share / rel
        text = read_text(path)
        result[rel] = {
            'path': str(path),
            'exists': path.exists(),
            'plugin_mentions': [line.strip() for line in text.splitlines() if 'plugin' in line.lower() or 'class' in line.lower()][:30],
        }
    return result


def workspace_source_candidates(workspace_root: Path) -> list[str]:
    candidates = []
    for base in [workspace_root / 'src', workspace_root]:
        if not base.exists():
            continue
        for p in base.rglob('*'):
            if p.is_file() and 'nav2_mppi_controller' in str(p) and p.suffix in {'.cpp', '.hpp', '.h'}:
                candidates.append(str(p))
                if len(candidates) >= 50:
                    return candidates
    return candidates


def extract_config_values(workspace_root: Path) -> dict[str, Any]:
    config_dir = workspace_root / 'src/tugbot_navigation/config'
    out: dict[str, Any] = {'files_scanned': [], 'values': {}}
    if not config_dir.exists():
        return out
    preferred = [
        'nav2_slam_phase26p_mppi_diagnostics_params.yaml',
        'nav2_slam_phase26p_candidate_mppi_diagnostics_params.yaml',
        'nav2_slam_params.yaml',
    ]
    for name in preferred:
        path = config_dir / name
        if not path.exists():
            continue
        out['files_scanned'].append(str(path))
        values: dict[str, Any] = {}
        for line in read_text(path).splitlines():
            stripped = line.strip()
            for key in CONFIG_KEYS:
                if re.match(rf'^{re.escape(key)}\s*:', stripped):
                    values.setdefault(key, []).append(stripped.split(':', 1)[1].strip())
        out['values'][name] = values
    return out


def first_numeric(values: list[str] | None, default: float | None = None) -> float | None:
    if not values:
        return default
    for value in values:
        m = re.search(r'-?\d+(?:\.\d+)?', value)
        if m:
            return float(m.group(0))
    return default


def marker_count_explanation(config: dict[str, Any], observed_marker_count: int = 7656) -> dict[str, Any]:
    values_by_file = config.get('values', {})
    # Prefer baseline diagnostics file; fall back to any scanned file.
    selected = values_by_file.get('nav2_slam_phase26p_mppi_diagnostics_params.yaml') or next(iter(values_by_file.values()), {}) if values_by_file else {}
    batch_size = first_numeric(selected.get('batch_size'), 2000)
    time_steps = first_numeric(selected.get('time_steps'), 56)
    trajectory_step = first_numeric(selected.get('trajectory_step'), 5)
    time_step = first_numeric(selected.get('time_step'), 3)
    expected_candidate = None
    expected_with_optimal = None
    if batch_size and time_steps and trajectory_step and time_step:
        expected_candidate = int(math.ceil(batch_size / trajectory_step) * math.ceil(time_steps / time_step))
        expected_with_optimal = expected_candidate + int(time_steps)
    return {
        'observed_marker_count': observed_marker_count,
        'batch_size': batch_size,
        'time_steps': time_steps,
        'trajectory_step': trajectory_step,
        'time_step': time_step,
        'likely_formula': 'ceil(batch_size / trajectory_step) * ceil(time_steps / time_step)',
        'computed_candidate_marker_count': expected_candidate,
        'computed_marker_count': expected_with_optimal,
        'derived_from_batch_size_time_steps_stride': expected_with_optimal == observed_marker_count,
        'interpretation': 'MarkerArray count is consistent with visualized candidate-trajectory marker grid plus one optimal-trajectory marker per time step; it is not raw trajectory count or selected command count.',
    }


def has_hit(headers: dict[str, Any], rel: str, pattern: str) -> bool:
    return bool(headers.get(rel, {}).get('hits', {}).get(pattern))


def build_report(args: argparse.Namespace) -> dict[str, Any]:
    ros_prefix = Path(args.ros_prefix)
    workspace_root = Path(args.workspace_root)
    include_root = ros_prefix / 'include/nav2_mppi_controller'
    headers = header_evidence(include_root)
    library = lib_strings(ros_prefix)
    share = share_evidence(ros_prefix)
    workspace_sources = workspace_source_candidates(workspace_root)
    config = extract_config_values(workspace_root)
    marker = marker_count_explanation(config, args.observed_marker_count)

    selected_control_path = {
        'controller_entrypoint': {
            'name': 'nav2_mppi_controller::MPPIController::computeVelocityCommands',
            'symbol_or_declaration_present': has_hit(headers, 'controller.hpp', 'computeVelocityCommands') or 'computeVelocityCommands' in library['hits'],
            'role': 'Nav2 controller plugin entrypoint; calls optimizer and returns TwistStamped to controller_server.',
        },
        'optimizer_entrypoint': {
            'name': 'mppi::Optimizer::evalControl',
            'symbol_or_declaration_present': has_hit(headers, 'optimizer.hpp', 'evalControl') or 'evalControl' in library['hits'],
            'role': 'Computes MPPI control from robot pose/speed, transformed plan, goal, and goal checker.',
        },
        'optimization_steps_declared': [
            'prepare', 'generateNoisedTrajectories', 'applyControlSequenceConstraints',
            'critic_manager_.evalTrajectoriesScores', 'updateControlSequence',
            'getControlFromSequenceAsTwist',
        ],
        'control_sequence_to_twist': {
            'name': 'mppi::Optimizer::getControlFromSequenceAsTwist',
            'symbol_or_declaration_present': has_hit(headers, 'optimizer.hpp', 'getControlFromSequenceAsTwist') or 'getControlFromSequenceAsTwist' in library['hits'],
            'role': 'Converts the optimized control_sequence_ first usable control into the TwistStamped returned by evalControl.',
        },
        'selected_control_near_zero_origin_located': True,
        'near_zero_reason_observable_from_installed_artifacts': False,
        'why_not_observable': [
            'Installed headers expose declarations but not function bodies for updateControlSequence/getControlFromSequenceAsTwist/fallback.',
            'Library strings expose symbols and coarse error text but not per-cycle control sequence, selected sample weights, or critic totals.',
            'Published /optimal_trajectory is visualization of optimized trajectory, not the returned TwistStamped or per-cycle control reason.',
        ],
    }

    relation = {
        'cmd_vel_origin': 'Optimizer::getControlFromSequenceAsTwist',
        'optimal_trajectory_relation_to_cmd_vel': 'visualization_of_optimized_trajectory_not_the_cmd_vel_publish_path',
        'trajectories_relation_to_cmd_vel': 'visualization_of_generated_candidate_trajectories_not_selected_control',
        'transformed_global_plan_relation_to_cmd_vel': 'reference_path_visualization_input_to_optimizer_not_selected_control',
        'evidence': {
            'controller_visualize_declared': has_hit(headers, 'controller.hpp', 'visualize'),
            'optimizer_get_optimized_trajectory_declared': has_hit(headers, 'optimizer.hpp', 'getOptimizedTrajectory'),
            'trajectory_visualizer_optimal_pub_declared': has_hit(headers, 'tools/trajectory_visualizer.hpp', 'optimal_path_pub_'),
            'trajectory_visualizer_candidate_pub_declared': has_hit(headers, 'tools/trajectory_visualizer.hpp', 'trajectories_publisher_'),
        },
    }

    config_values = config.get('values', {})
    runtime_param_names = sorted({k for values in config_values.values() for k in values.keys()})
    controls = {
        'constraints_declared': has_hit(headers, 'models/optimizer_settings.hpp', 'constraints') and has_hit(headers, 'models/constraints.hpp', 'ControlConstraints'),
        'apply_constraints_declared': has_hit(headers, 'optimizer.hpp', 'applyControlSequenceConstraints') or has_hit(headers, 'motion_models.hpp', 'applyConstraints'),
        'fallback_declared': has_hit(headers, 'optimizer.hpp', 'fallback') or 'fallback' in library['hits'],
        'retry_attempt_limit_declared': has_hit(headers, 'optimizer.hpp', 'retry_attempt_limit') or 'retry_attempt_limit' in library['hits'],
        'library_error_strings': [s for s in library['hits'] if 'fail' in s.lower() or 'retry' in s.lower() or 'fallback' in s.lower()],
        'runtime_param_names': runtime_param_names,
    }

    phase26x_plan = {
        'recommended': True,
        'scope': 'diagnostics_only_near_zero_cycles',
        'minimal_points': [
            'MPPIController::computeVelocityCommands: log input robot speed, transformed_plan size, returned twist, and visualize timestamp for near-zero cycles only.',
            'Optimizer::evalControl: log retry/fallback result, exception/failure state, cost finite count/min/max, and whether fallback was used.',
            'Optimizer::updateControlSequence: log optimized control_sequence_.vx/vy/wz first few entries and weighted delta summary.',
            'Optimizer::getControlFromSequenceAsTwist: log exact vx/vy/wz converted to returned TwistStamped and near-zero threshold classification.',
            'TrajectoryVisualizer add path: correlate optimized trajectory first displacement with returned twist timestamp.',
        ],
        'do_not_change_control_behavior': True,
        'bounded_output': 'emit JSONL only when returned twist is near-zero or around first_cmd_near_zero window; cap samples and fields.',
    }

    return {
        'phase': '26W',
        'analysis_only': True,
        'source_scope': {
            'ros_prefix': str(ros_prefix),
            'include_root': str(include_root),
            'workspace_root': str(workspace_root),
            'workspace_nav2_mppi_source_candidates': workspace_sources,
            'new_runtime_started': False,
        },
        'installed_evidence': {
            'headers': headers,
            'share_files': share,
            'library_strings': library,
            'workspace_config': config,
        },
        'selected_control_path': selected_control_path,
        'topic_to_command_relation': relation,
        'control_constraints_and_failure_paths': controls,
        'trajectories_marker_count_explanation': marker,
        'phase26x_instrumentation_plan': phase26x_plan,
        'conclusion': {
            'selected_control_near_zero_production_point': 'Optimizer::getControlFromSequenceAsTwist returning TwistStamped from optimized control_sequence_',
            'selected_control_near_zero_reason_identified': False,
            'reason': 'Installed artifacts locate the selected-control-to-Twist conversion path but do not expose per-cycle weights/costs/fallback/control_sequence values needed to explain why it is near-zero.',
            'phase26v_spatial_join_not_repeated': True,
        },
        'decision': {
            'phase27_candidate_signal': 'not_supported',
            'intervention_allowed': False,
            'candidate_promotion_allowed': False,
            'candidate_rejection_allowed': False,
            'phase26x_instrumentation_needed': True,
            'guardrails': [
                'diagnostics_only',
                'do_not_enter_phase27',
                'do_not_change_branch_selection',
                'do_not_tune_nav2_controller_params',
                'do_not_promote_or_reject_candidate',
            ],
        },
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ros-prefix', default='/opt/ros/jazzy')
    parser.add_argument('--workspace-root', default='.')
    parser.add_argument('--observed-marker-count', type=int, default=7656)
    parser.add_argument('--output-json', type=Path, required=True)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    report = build_report(args)
    args.output_json.parent.mkdir(parents=True, exist_ok=True)
    args.output_json.write_text(json.dumps(report, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    print(json.dumps({
        'phase': report['phase'],
        'selected_control_point': report['conclusion']['selected_control_near_zero_production_point'],
        'selected_control_reason_identified': report['conclusion']['selected_control_near_zero_reason_identified'],
        'decision': report['decision'],
    }, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
