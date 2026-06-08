#!/usr/bin/env python3
"""Phase45 Nav2 bringup/runtime recovery diagnostics analyzer.

Diagnostics-only: no Nav2 parameter tuning, no maze_explorer strategy work.
"""

from __future__ import annotations

import argparse
import json
import re
import subprocess
from pathlib import Path
from typing import Any

RUN_ID = 'phase45_nav2_bringup_runtime_recovery_diagnostics'
CLASSIFICATIONS = [
    'NAV2_BRINGUP_NOT_INCLUDED',
    'NAV2_NODES_NOT_STARTED',
    'BT_NAVIGATOR_MISSING',
    'NAVIGATE_TO_POSE_SERVER_MISSING',
    'LIFECYCLE_NOT_ACTIVE',
    'PARAMS_OR_LAUNCH_ARGUMENT_ERROR',
    'NAV2_BASELINE_RECOVERED',
    'INCONCLUSIVE_NEEDS_MANUAL_LOG_REVIEW',
]
REQUIRED_NAV2_NODES = [
    '/controller_server',
    '/planner_server',
    '/bt_navigator',
    '/behavior_server',
    '/waypoint_follower',
    '/velocity_smoother',
    '/smoother_server',
]
EXPECTED_LIFECYCLE_NODES = REQUIRED_NAV2_NODES + ['/slam_toolbox']
FORBIDDEN_NODES = ['maze_explorer', 'maze_goal_monitor', 'frontier_explorer']
LOG_ERROR_PATTERNS = [
    'import error',
    'missing package',
    'params file',
    'parameter file',
    'lifecycle transition',
    'process has died',
    'exception',
    'traceback',
    'failed to load',
    'no such file',
    'could not',
    'error',
    'fatal',
]


def read_text(path: Path) -> str:
    try:
        return path.read_text(errors='replace')
    except FileNotFoundError:
        return ''


def run_cmd(command: list[str], cwd: Path | None = None, timeout: int = 10) -> dict[str, Any]:
    try:
        proc = subprocess.run(command, cwd=str(cwd) if cwd else None, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=timeout)
        return {'cmd': command, 'exit_code': proc.returncode, 'output': proc.stdout}
    except Exception as exc:  # pragma: no cover - defensive runtime path
        return {'cmd': command, 'exit_code': 999, 'output': f'{type(exc).__name__}: {exc}'}


def static_analyze(root: Path) -> dict[str, Any]:
    manual_launch = root / 'src' / 'tugbot_bringup' / 'launch' / 'tugbot_maze_slam_nav.launch.py'
    phase44_wrapper = root / 'tools' / 'run_phase44_manual_nav2_baseline_recovery.sh'
    nav2_params = root / 'src' / 'tugbot_navigation' / 'config' / 'nav2_slam_params.yaml'
    slam_params = root / 'src' / 'tugbot_navigation' / 'config' / 'slam_toolbox_params.yaml'
    world = root / 'src' / 'tugbot_gazebo' / 'worlds' / 'tugbot_maze_world_20260528_clean_scaled2x.sdf'

    launch_text = read_text(manual_launch)
    wrapper_text = read_text(phase44_wrapper)
    forbidden = [name for name in FORBIDDEN_NODES if name in launch_text]
    nav2_launch_file = None
    if 'navigation_launch.py' in launch_text:
        nav2_launch_file = 'navigation_launch.py'
    elif 'bringup_launch.py' in launch_text:
        nav2_launch_file = 'bringup_launch.py'

    return {
        'run_id': RUN_ID,
        'manual_launch': {
            'path': str(manual_launch),
            'exists': manual_launch.exists(),
            'includes_nav2_bringup': 'nav2_bringup' in launch_text and nav2_launch_file is not None,
            'nav2_launch_file': nav2_launch_file,
            'mentions_navigation_launch.py': 'navigation_launch.py' in launch_text,
            'mentions_bringup_launch.py': 'bringup_launch.py' in launch_text,
            'passes_params_file': "'params_file': nav2_params_file" in launch_text or 'params_file' in launch_text,
            'passes_autostart': "'autostart': autostart" in launch_text or 'autostart' in launch_text,
            'passes_use_sim_time': "'use_sim_time': use_sim_time" in launch_text or 'use_sim_time' in launch_text,
            'passes_namespace': 'namespace' in launch_text,
            'passes_slam_argument_to_nav2': "'slam'" in launch_text,
            'uses_active_scaled2x_world': 'tugbot_maze_world_20260528_clean_scaled2x.sdf' in launch_text,
            'uses_online_slam_launch': 'online_async_launch.py' in launch_text,
            'forbidden_explorer_references': forbidden,
            'timer_nav2_delay_sec': _extract_timer_period_before(launch_text, 'nav2_navigation_launch'),
        },
        'phase44_wrapper': {
            'path': str(phase44_wrapper),
            'exists': phase44_wrapper.exists(),
            'passes_params_file': 'params_file:=' in wrapper_text,
            'passes_slam_params_file': 'slam_params_file:=' in wrapper_text,
            'passes_autostart_true': 'autostart:=true' in wrapper_text,
            'passes_use_sim_time_true': 'use_sim_time:=true' in wrapper_text,
            'uses_manual_launch': 'tugbot_maze_slam_nav.launch.py' in wrapper_text,
        },
        'files': {
            'active_world_exists': world.exists(),
            'nav2_params_exists': nav2_params.exists(),
            'slam_params_exists': slam_params.exists(),
            'active_world': str(world),
            'nav2_params': str(nav2_params),
            'slam_params': str(slam_params),
        },
    }


def _extract_timer_period_before(text: str, action_name: str) -> float | None:
    pattern = re.compile(r'TimerAction\(period=([0-9.]+),\s*actions=\[([^\]]+)\]\)')
    for match in pattern.finditer(text):
        if action_name in match.group(2):
            try:
                return float(match.group(1))
            except ValueError:
                return None
    return None


def parse_action_server_count(text: str) -> int | None:
    match = re.search(r'Action servers:\s*(\d+)', text)
    return int(match.group(1)) if match else None


def parse_goal_pose_subscribers(text: str) -> int | None:
    match = re.search(r'Subscription count:\s*(\d+)', text)
    return int(match.group(1)) if match else None


def parse_nodes(text: str) -> list[str]:
    return [line.strip() for line in text.splitlines() if line.strip().startswith('/')]


def parse_lifecycle_states(text: str) -> dict[str, str]:
    states: dict[str, str] = {}
    current = None
    for raw in text.splitlines():
        line = raw.strip()
        if line.startswith('## /'):
            current = line[3:].strip()
            states[current] = 'missing'
        elif current and line:
            if 'active [3]' in line:
                states[current] = 'active'
            elif 'Node not found' in line:
                states[current] = 'missing'
            elif 'inactive' in line:
                states[current] = 'inactive'
            elif 'unconfigured' in line:
                states[current] = 'unconfigured'
            elif 'errorprocessing' in line or 'finalized' in line:
                states[current] = line
    return states


def extract_log_errors(log_text: str) -> list[dict[str, Any]]:
    rows = []
    for idx, line in enumerate(log_text.splitlines(), start=1):
        lower = line.lower()
        if any(pattern in lower for pattern in LOG_ERROR_PATTERNS):
            # Treat known RViz GLSL and inflation-size warnings as evidence but not primary launch arg errors.
            rows.append({'line': idx, 'text': line[:500]})
    return rows[:80]


def analyze_artifacts(root: Path, artifact_dir: Path) -> dict[str, Any]:
    static = static_analyze(root)
    nodes_text = read_text(artifact_dir / 'nodes.txt')
    actions_text = read_text(artifact_dir / 'actions.txt')
    lifecycle_text = read_text(artifact_dir / 'lifecycle_states.txt')
    action_info_text = read_text(artifact_dir / 'navigate_to_pose_action_info.txt')
    goal_pose_info_text = read_text(artifact_dir / 'goal_pose_topic_info.txt')
    launch_log_text = read_text(artifact_dir / 'launch.log')
    process_text = read_text(artifact_dir / 'nav2_process_tree.txt')
    params_snapshot_dir = artifact_dir / 'params_snapshot'

    nodes = parse_nodes(nodes_text)
    lifecycle = parse_lifecycle_states(lifecycle_text)
    action_servers = parse_action_server_count(action_info_text)
    goal_pose_subscribers = parse_goal_pose_subscribers(goal_pose_info_text)
    forbidden = [node for node in nodes if any(token in node for token in FORBIDDEN_NODES)]
    required_presence = {node: node in nodes for node in REQUIRED_NAV2_NODES}
    lifecycle_active = {node: lifecycle.get(node) == 'active' for node in EXPECTED_LIFECYCLE_NODES}
    errors = extract_log_errors(launch_log_text)

    classification, reasons = classify(static, nodes, lifecycle, action_servers, goal_pose_subscribers, errors, action_info_text, process_text)
    summary = {
        'run_id': RUN_ID,
        'classification': classification,
        'classification_reasons': reasons,
        'autonomous_exploration_success_claimed': False,
        'maze_explorer_involved': bool(forbidden),
        'forbidden_explorer_nodes_seen': forbidden,
        'static_analysis': static,
        'nodes': {
            'count': len(nodes),
            'required_nav2_present': required_presence,
            'lifecycle_manager_navigation_present': '/lifecycle_manager_navigation' in nodes,
            'slam_toolbox_present': '/slam_toolbox' in nodes,
        },
        'actions': {
            'navigate_to_pose_listed': '/navigate_to_pose' in actions_text,
            'navigate_to_pose_action_server_count': action_servers,
            'navigate_to_pose_action_info': action_info_text,
            'goal_pose_subscription_count': goal_pose_subscribers,
            'goal_pose_topic_info': goal_pose_info_text,
        },
        'lifecycle': {
            'states': lifecycle,
            'expected_active': lifecycle_active,
        },
        'params_snapshot': {
            'dir': str(params_snapshot_dir),
            'files': sorted(p.name for p in params_snapshot_dir.glob('*.yaml')) if params_snapshot_dir.exists() else [],
        },
        'process_evidence': process_text,
        'launch_log_errors': errors,
        'artifacts': sorted(p.name for p in artifact_dir.iterdir()) if artifact_dir.exists() else [],
    }
    return summary


def classify(static: dict[str, Any], nodes: list[str], lifecycle: dict[str, str], action_servers: int | None, goal_pose_subscribers: int | None, errors: list[dict[str, Any]], action_info: str, process_text: str) -> tuple[str, list[str]]:
    reasons: list[str] = []
    manual = static['manual_launch']
    if not manual['includes_nav2_bringup']:
        return 'NAV2_BRINGUP_NOT_INCLUDED', ['manual launch does not include nav2_bringup navigation_launch.py or bringup_launch.py']
    actionable_errors = [row['text'] for row in errors if _is_launch_arg_error(row['text'])]
    if actionable_errors:
        reasons.append('launch.log contains parameter/import/missing-file/lifecycle error patterns')
        reasons.extend(actionable_errors[:5])
        return 'PARAMS_OR_LAUNCH_ARGUMENT_ERROR', reasons
    present = {node: node in nodes for node in REQUIRED_NAV2_NODES}
    if not any(present.values()):
        return 'NAV2_NODES_NOT_STARTED', ['none of the required Nav2 nodes are present in ros2 node list']
    if '/bt_navigator' not in nodes:
        reasons.append('/bt_navigator missing from ros2 node list')
        if 'bt_navigator' not in process_text:
            reasons.append('bt_navigator process not present in process tree')
        return 'BT_NAVIGATOR_MISSING', reasons
    inactive = [node for node in REQUIRED_NAV2_NODES if lifecycle.get(node) != 'active']
    if inactive:
        return 'LIFECYCLE_NOT_ACTIVE', [f'lifecycle not active for: {inactive}']
    if action_servers != 1:
        reasons.append(f'/navigate_to_pose Action servers: {action_servers}')
        if goal_pose_subscribers == 0:
            reasons.append('/goal_pose subscription count is 0')
        return 'NAVIGATE_TO_POSE_SERVER_MISSING', reasons
    if action_servers == 1 and '/bt_navigator' in nodes and not inactive:
        return 'NAV2_BASELINE_RECOVERED', ['Nav2 nodes active and /navigate_to_pose action server is available']
    return 'INCONCLUSIVE_NEEDS_MANUAL_LOG_REVIEW', ['evidence did not match a more specific classification']


def _is_launch_arg_error(text: str) -> bool:
    lower = text.lower()
    benign = [
        'rviz/glsl',
        'active samplers',
        'inflation radius',
        'egl: failed to create',
        'libegl warning',
        'waiting on external lifecycle transitions',
        'node lifecycle',
    ]
    if any(token in lower for token in benign):
        return False
    return any(token in lower for token in ['import error', 'missing package', 'params file', 'parameter file', 'process has died', 'exception', 'traceback', 'failed to load', 'no such file', 'lifecycle transition', 'fatal'])


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--root', type=Path, default=Path.cwd())
    parser.add_argument('--artifact-dir', type=Path, default=Path('log') / RUN_ID)
    parser.add_argument('--static-output', type=Path)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--static-only', action='store_true')
    args = parser.parse_args()

    root = args.root.resolve()
    artifact_dir = (root / args.artifact_dir).resolve() if not args.artifact_dir.is_absolute() else args.artifact_dir
    static = static_analyze(root)
    if args.static_output:
        args.static_output.parent.mkdir(parents=True, exist_ok=True)
        args.static_output.write_text(json.dumps(static, indent=2, sort_keys=True) + '\n')
    if args.static_only:
        print(json.dumps(static, sort_keys=True))
        return 0
    summary = analyze_artifacts(root, artifact_dir)
    output = args.output or artifact_dir / 'phase45_nav2_bringup_diagnostics.json'
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(summary, indent=2, sort_keys=True) + '\n')
    print(json.dumps({'classification': summary['classification'], 'reasons': summary['classification_reasons']}, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
