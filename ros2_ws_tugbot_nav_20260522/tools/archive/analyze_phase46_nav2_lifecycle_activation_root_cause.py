#!/usr/bin/env python3
"""Phase46 Nav2 lifecycle activation root-cause diagnostics analyzer.

Diagnostics-only: no Nav2 parameter tuning and no maze_explorer strategy work.
"""

from __future__ import annotations

import argparse
import json
import re
from pathlib import Path
from typing import Any

RUN_ID = 'phase46_nav2_lifecycle_activation_root_cause'
ACTIVE_WORLD = 'tugbot_maze_world_20260528_clean_scaled2x.sdf'
FORBIDDEN = ['maze_explorer', 'maze_goal_monitor', 'frontier_explorer']
CLASSIFICATIONS = [
    'LIFECYCLE_MANAGER_NAMESPACE_MISMATCH',
    'LIFECYCLE_MANAGER_EXITED',
    'MANAGED_NODE_LIST_MISMATCH',
    'LIFECYCLE_TRANSITION_BLOCKED_BY_NODE',
    'AUTOSTART_NOT_EFFECTIVE',
    'NAV2_LIFECYCLE_ACTIVE_RECOVERED',
    'INCONCLUSIVE',
]
CORE_MANAGED = [
    '/controller_server',
    '/planner_server',
    '/bt_navigator',
    '/behavior_server',
    '/waypoint_follower',
    '/velocity_smoother',
    '/smoother_server',
]
OPTIONAL_OR_NEW_NAV2 = ['/route_server', '/collision_monitor', '/docking_server']


def read_text(path: Path) -> str:
    try:
        return path.read_text(errors='replace')
    except FileNotFoundError:
        return ''


def root_default() -> Path:
    return Path(__file__).resolve().parents[1]


def static_analysis(root: Path) -> dict[str, Any]:
    launch = root / 'src/tugbot_bringup/launch/tugbot_maze_slam_nav.launch.py'
    nav2_params = root / 'src/tugbot_navigation/config/nav2_slam_params.yaml'
    launch_text = read_text(launch)
    params_text = read_text(nav2_params)
    declared_sections = []
    for line in params_text.splitlines():
        match = re.match(r'^([A-Za-z0-9_]+):\s*$', line)
        if match:
            declared_sections.append(match.group(1))
    forbidden_refs = [token for token in FORBIDDEN if token in launch_text]
    return {
        'run_id': RUN_ID,
        'manual_launch': {
            'path': str(launch),
            'exists': launch.exists(),
            'includes_nav2_bringup': 'nav2_bringup' in launch_text,
            'mentions_navigation_launch.py': 'navigation_launch.py' in launch_text,
            'mentions_bringup_launch.py': 'bringup_launch.py' in launch_text,
            'passes_autostart': 'autostart' in launch_text,
            'passes_use_sim_time': 'use_sim_time' in launch_text,
            'passes_params_file': 'params_file' in launch_text,
            'uses_active_scaled2x_world': ACTIVE_WORLD in launch_text,
            'forbidden_explorer_references': forbidden_refs,
        },
        'nav2_params': {
            'path': str(nav2_params),
            'exists': nav2_params.exists(),
            'declared_sections': declared_sections,
            'has_lifecycle_manager_navigation_section': 'lifecycle_manager_navigation:' in params_text,
            'has_node_names_key': bool(re.search(r'\bnode_names\s*:', params_text)),
            'declares_docking_server': 'docking_server' in declared_sections,
            'declares_route_server': 'route_server' in declared_sections,
            'declares_collision_monitor': 'collision_monitor' in declared_sections,
        },
    }


def parse_node_list(text: str) -> list[str]:
    return sorted({line.strip() for line in text.splitlines() if line.strip().startswith('/')})


def parse_action_server_count(text: str) -> int | None:
    match = re.search(r'Action servers:\s*(\d+)', text)
    return int(match.group(1)) if match else None


def parse_goal_pose_subscription_count(text: str) -> int | None:
    match = re.search(r'Subscription count:\s*(\d+)', text)
    return int(match.group(1)) if match else None


def parse_lifecycle_states(text: str) -> dict[str, dict[str, str]]:
    # Expected file format contains sections:
    # ## before
    # /node: active
    sections: dict[str, dict[str, str]] = {'before': {}, 'after': {}}
    current = 'before'
    for raw in text.splitlines():
        line = raw.strip()
        if not line:
            continue
        lower = line.lower()
        if lower.startswith('##') and 'after' in lower:
            current = 'after'
            continue
        if lower.startswith('##') and 'before' in lower:
            current = 'before'
            continue
        if line.startswith('/') and ':' in line:
            node, state = line.split(':', 1)
            sections.setdefault(current, {})[node.strip()] = state.strip()
    return sections


def parse_manager_params(text: str) -> dict[str, Any]:
    params: dict[str, Any] = {'raw': text, 'node_names': [], 'autostart': None, 'namespace': None}
    for line in text.splitlines():
        stripped = line.strip()
        if stripped.startswith('node_names:') or 'node_names:' in stripped or stripped.startswith('String values are:'):
            if stripped.startswith('String values are:'):
                value = stripped
            else:
                value = stripped.split('node_names:', 1)[1].strip()
            if not value:
                continue
            params['node_names_raw'] = value
            params['node_names'] = _parse_string_list(value)
        elif stripped.startswith('autostart:') or 'autostart:' in stripped or stripped.startswith('Boolean value is:'):
            if stripped.startswith('Boolean value is:'):
                value = stripped.split(':', 1)[1].strip()
            else:
                value = stripped.split('autostart:', 1)[1].strip()
            params['autostart_raw'] = value
            params['autostart'] = value.lower() in {'true', '1'}
        elif stripped.startswith('namespace:') or 'namespace:' in stripped:
            params['namespace'] = stripped.split('namespace:', 1)[1].strip()
    return params


def _parse_string_list(value: str) -> list[str]:
    value = value.strip()
    if not value:
        return []
    # ros2 param get usually prints: String values are: ['a', 'b']
    match = re.search(r'\[(.*)\]', value)
    if match:
        inner = match.group(1)
        return [part.strip().strip('"\'') for part in inner.split(',') if part.strip()]
    colon_match = re.search(r':\s*([^:]+)$', value)
    if colon_match and value.lower().startswith(('string values are', 'parameter value is')):
        tail = colon_match.group(1).strip()
        if tail:
            return [tail.strip().strip('"\'')]
    return [value.strip().strip('"\'')]


def parse_transition_service_results(text: str) -> dict[str, Any]:
    result: dict[str, Any] = {'raw': text, 'change_state_services': [], 'get_state_services': [], 'errors': []}
    for line in text.splitlines():
        stripped = line.strip()
        if stripped.endswith('/change_state'):
            result['change_state_services'].append(stripped)
        if stripped.endswith('/get_state'):
            result['get_state_services'].append(stripped)
        if any(token in stripped.lower() for token in ['error', 'failed', 'not found', 'timeout']):
            result['errors'].append(stripped)
    return result


def parse_launch_log(text: str) -> dict[str, Any]:
    interesting = []
    for idx, line in enumerate(text.splitlines(), 1):
        lower = line.lower()
        if any(token in lower for token in ['lifecycle manager', 'configuring', 'activating', 'activated', 'failed', 'error', 'exception', 'transition', 'docking_server', 'route_server', 'collision_monitor']):
            interesting.append({'line': idx, 'text': line})
    blocked_nodes = []
    for row in interesting:
        lower = row['text'].lower()
        if 'configuring' in lower:
            for node in CORE_MANAGED + OPTIONAL_OR_NEW_NAV2:
                if node.strip('/') in lower:
                    blocked_nodes.append(node)
    return {'interesting': interesting[-120:], 'blocked_or_last_configuring_nodes': blocked_nodes[-10:]}


def classify(summary: dict[str, Any]) -> tuple[str, list[str]]:
    reasons: list[str] = []
    static = summary['static_analysis']
    nodes = summary['nodes']['nodes']
    lifecycle = summary['lifecycle']['states']
    after = lifecycle.get('after') or lifecycle.get('before') or {}
    manager_params = summary['manager_params']
    managed = summary['managed_nodes']['managed_nodes']
    transition = summary['transition_service_results']
    launch = summary['launch_log']

    manager_nodes = [node for node in nodes if node.endswith('/lifecycle_manager_navigation') or node == '/lifecycle_manager_navigation']
    if '/lifecycle_manager_navigation' not in nodes and manager_nodes:
        reasons.append(f'lifecycle manager appears namespaced or renamed: {manager_nodes}')
        return 'LIFECYCLE_MANAGER_NAMESPACE_MISMATCH', reasons

    nav2_present = any(node in nodes for node in CORE_MANAGED)
    if nav2_present and '/lifecycle_manager_navigation' not in nodes:
        reasons.append('Nav2 lifecycle nodes are present but /lifecycle_manager_navigation is absent from node list')
        return 'LIFECYCLE_MANAGER_EXITED', reasons

    if manager_params.get('autostart') is False:
        reasons.append('lifecycle_manager_navigation autostart parameter is false')
        return 'AUTOSTART_NOT_EFFECTIVE', reasons

    if not manager_params.get('raw') and '/lifecycle_manager_navigation' in nodes:
        reasons.append('lifecycle_manager_navigation exists but manager params could not be read')
        return 'AUTOSTART_NOT_EFFECTIVE', reasons

    node_names = manager_params.get('node_names') or managed
    if node_names:
        normalized = ['/' + item.lstrip('/') for item in node_names]
        missing_from_graph = [node for node in normalized if node not in nodes]
        if missing_from_graph:
            reasons.append(f'managed node_names include nodes missing from graph: {missing_from_graph}')
            return 'MANAGED_NODE_LIST_MISMATCH', reasons

    core_after = {node: after.get(node, 'missing') for node in CORE_MANAGED}
    if core_after and all(state.startswith('active') for state in core_after.values()):
        action_servers = summary['actions'].get('navigate_to_pose_action_server_count')
        if action_servers and action_servers > 0:
            reasons.append('core Nav2 lifecycle nodes are active and /navigate_to_pose action server exists')
            return 'NAV2_LIFECYCLE_ACTIVE_RECOVERED', reasons

    inactive_or_unconfigured = {node: state for node, state in core_after.items() if state != 'active'}
    if inactive_or_unconfigured:
        blocker = _infer_blocker(launch, transition, after)
        reasons.append(f'lifecycle activation blocked or incomplete; inactive core states: {inactive_or_unconfigured}; inferred blocker: {blocker}')
        return 'LIFECYCLE_TRANSITION_BLOCKED_BY_NODE', reasons

    if manager_params.get('autostart') is not True:
        reasons.append('autostart parameter was not confirmed true')
        return 'AUTOSTART_NOT_EFFECTIVE', reasons

    reasons.append('evidence did not match a specific lifecycle activation root-cause bucket')
    return 'INCONCLUSIVE', reasons


def _infer_blocker(launch: dict[str, Any], transition: dict[str, Any], after: dict[str, str]) -> str:
    for node in ['/docking_server', '/route_server', '/collision_monitor']:
        state = after.get(node)
        if state in {'missing', 'unconfigured', 'inactive'}:
            return f'{node} state={state}'
    blocked = launch.get('blocked_or_last_configuring_nodes') or []
    if blocked:
        return blocked[-1]
    errors = transition.get('errors') or []
    if errors:
        return errors[-1]
    return 'unknown'


def runtime_summary(root: Path, artifact_dir: Path) -> dict[str, Any]:
    static = static_analysis(root)
    nodes_text = read_text(artifact_dir / 'nodes.txt')
    actions_text = read_text(artifact_dir / 'actions.txt')
    lifecycle_text = read_text(artifact_dir / 'lifecycle_states_before_after.txt')
    manager_params_text = read_text(artifact_dir / 'manager_params.txt')
    managed_nodes_text = read_text(artifact_dir / 'managed_nodes.txt')
    transition_text = read_text(artifact_dir / 'transition_service_results.txt')
    launch_text = read_text(artifact_dir / 'launch.log')
    navigate_info = read_text(artifact_dir / 'navigate_to_pose_action_info.txt')
    goal_pose_info = read_text(artifact_dir / 'goal_pose_topic_info.txt')

    nodes = parse_node_list(nodes_text)
    manager_params = parse_manager_params(manager_params_text)
    managed_nodes = [line.strip().lstrip('- ').strip('"\'') for line in managed_nodes_text.splitlines() if line.strip() and not line.startswith('#')]
    summary: dict[str, Any] = {
        'run_id': RUN_ID,
        'static_analysis': static,
        'nodes': {
            'count': len(nodes),
            'nodes': nodes,
            'lifecycle_manager_navigation_present': '/lifecycle_manager_navigation' in nodes,
            'manager_like_nodes': [n for n in nodes if 'lifecycle_manager' in n],
            'forbidden_explorer_nodes_seen': [n for n in nodes if any(token in n for token in FORBIDDEN)],
        },
        'actions': {
            'actions_raw': actions_text,
            'navigate_to_pose_action_info': navigate_info,
            'navigate_to_pose_action_server_count': parse_action_server_count(navigate_info),
            'goal_pose_topic_info': goal_pose_info,
            'goal_pose_subscription_count': parse_goal_pose_subscription_count(goal_pose_info),
        },
        'lifecycle': {'raw': lifecycle_text, 'states': parse_lifecycle_states(lifecycle_text)},
        'manager_params': manager_params,
        'managed_nodes': {'raw': managed_nodes_text, 'managed_nodes': managed_nodes},
        'transition_service_results': parse_transition_service_results(transition_text),
        'launch_log': parse_launch_log(launch_text),
        'cleanup_empty': read_text(artifact_dir / 'cleanup_processes_after.txt') == '',
        'nav2_config_diff_empty': read_text(artifact_dir / 'nav2_config_diff.txt') == '',
        'maze_explorer_involved': any(token in (nodes_text + launch_text) for token in FORBIDDEN),
        'autonomous_exploration_success_claimed': False,
    }
    classification, reasons = classify(summary)
    summary['classification'] = classification
    summary['classification_reasons'] = reasons
    summary['allowed_classifications'] = CLASSIFICATIONS
    return summary


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--root', default=str(root_default()))
    parser.add_argument('--artifact-dir', default=f'log/{RUN_ID}')
    parser.add_argument('--static-output')
    parser.add_argument('--output')
    args = parser.parse_args()
    root = Path(args.root).resolve()
    artifact_dir = (root / args.artifact_dir).resolve() if not Path(args.artifact_dir).is_absolute() else Path(args.artifact_dir)
    artifact_dir.mkdir(parents=True, exist_ok=True)
    static = static_analysis(root)
    static_path = Path(args.static_output) if args.static_output else artifact_dir / 'nav2_launch_static_analysis.json'
    static_path.write_text(json.dumps(static, indent=2, sort_keys=True))
    summary = runtime_summary(root, artifact_dir)
    output_path = Path(args.output) if args.output else artifact_dir / 'diagnostics.json'
    output_path.write_text(json.dumps(summary, indent=2, sort_keys=True))
    print(json.dumps({'classification': summary['classification'], 'reasons': summary['classification_reasons']}, ensure_ascii=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
