#!/usr/bin/env python3
"""Phase36 static readiness check for bounded autonomous maze exploration.

This is a read-only analyzer.  It does not launch ROS/Gazebo, does not start
maze_explorer, and does not modify navigation strategy or Nav2 parameters.
"""
from __future__ import annotations

import argparse
import ast
import json
import subprocess
from pathlib import Path
from typing import Any

import yaml

ACTIVE_WORLD_NAME = 'tugbot_maze_world_20260528_clean_scaled2x.sdf'
OLD_SCAFFOLD_WORLD_NAME = 'tugbot_maze_world.sdf'
LEGACY_STATIC_MAP_NAME = 'map_1725111373.yaml'
DECORATIVE_ROUTE_NAME = 'maze_20260522.jpg'
ACTIVE_METADATA_REL = 'src/tugbot_maze/config/maze_20260528_scaled_instance.yaml'
CURRENT_MAZE_LAUNCHES = [
    'src/tugbot_bringup/launch/tugbot_maze_slam.launch.py',
    'src/tugbot_bringup/launch/tugbot_maze_slam_nav.launch.py',
    'src/tugbot_bringup/launch/tugbot_maze_explore.launch.py',
]


def load_yaml(path: Path) -> dict[str, Any]:
    data = yaml.safe_load(path.read_text())
    if not isinstance(data, dict):
        raise ValueError(f'expected YAML mapping: {path}')
    return data


def parse_launch_defaults(path: Path) -> dict[str, str]:
    tree = ast.parse(path.read_text())
    defaults: dict[str, str] = {}
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        if not isinstance(node.func, ast.Name) or node.func.id != 'DeclareLaunchArgument':
            continue
        if not node.args or not isinstance(node.args[0], ast.Constant):
            continue
        name = str(node.args[0].value)
        default_text: str | None = None
        for keyword in node.keywords:
            if keyword.arg != 'default_value':
                continue
            default_text = launch_default_to_text(keyword.value)
            break
        if default_text is not None:
            defaults[name] = default_text
    return defaults


def launch_default_to_text(node: ast.AST) -> str:
    if isinstance(node, ast.Constant):
        return str(node.value)
    if isinstance(node, ast.Call):
        if isinstance(node.func, ast.Attribute) and node.func.attr == 'join':
            parts = [launch_default_to_text(arg) for arg in node.args]
            return '/'.join(part.strip('/') for part in parts if part)
        if isinstance(node.func, ast.Name) and node.func.id == 'LaunchConfiguration':
            if node.args and isinstance(node.args[0], ast.Constant):
                return f'LaunchConfiguration({node.args[0].value})'
    if isinstance(node, ast.Name):
        return node.id
    if isinstance(node, ast.Subscript):
        return ast.unparse(node)
    return ast.unparse(node)


def expected_truth_from_metadata(metadata: dict[str, Any]) -> dict[str, float]:
    truth = metadata.get('map_frame_truth') or {}
    entrance = truth.get('entrance') or metadata['entrance']
    exit_truth = truth.get('exit') or metadata['exit']
    return {
        'entrance_x': float(entrance['x_m']),
        'entrance_y': float(entrance['y_m']),
        'entrance_yaw': float(entrance['yaw_rad']),
        'exit_x': float(exit_truth['x_m']),
        'exit_y': float(exit_truth['y_m']),
        'exit_radius': float(exit_truth['radius_m']),
    }


def default_matches(defaults: dict[str, str], key: str, expected: float, tol: float = 1e-6) -> bool:
    try:
        return abs(float(defaults[key]) - expected) <= tol
    except (KeyError, TypeError, ValueError):
        return False


def check_launch_defaults(root: Path) -> dict[str, dict[str, bool]]:
    result: dict[str, dict[str, bool]] = {}
    for rel in CURRENT_MAZE_LAUNCHES:
        path = root / rel
        text = path.read_text()
        defaults = parse_launch_defaults(path)
        world_default = defaults.get('world_sdf', '')
        result[rel] = {
            'active_world_default_present': ACTIVE_WORLD_NAME in world_default or ACTIVE_WORLD_NAME in text,
            'old_scaffold_default_present': OLD_SCAFFOLD_WORLD_NAME in world_default and ACTIVE_WORLD_NAME not in world_default,
            'legacy_static_map_default_present': LEGACY_STATIC_MAP_NAME in world_default,
        }
    return result


def check_explorer_truth(root: Path, metadata: dict[str, Any]) -> dict[str, Any]:
    launch_rel = 'src/tugbot_bringup/launch/tugbot_maze_explore.launch.py'
    launch_path = root / launch_rel
    defaults = parse_launch_defaults(launch_path)
    expected = expected_truth_from_metadata(metadata)
    active_keys = ['entrance_x', 'entrance_y', 'entrance_yaw', 'exit_x', 'exit_y', 'exit_radius']
    uses_active_metadata_defaults = all(default_matches(defaults, key, expected[key]) for key in active_keys)
    metadata_default = defaults.get('maze_config')
    metadata_file_declared = Path(metadata_default).name if metadata_default else None
    metadata_file_exists = False
    if metadata_file_declared is not None:
        metadata_file_exists = (root / 'src' / 'tugbot_maze' / 'config' / metadata_file_declared).exists()
    return {
        'launch_file': launch_rel,
        'source_status': 'launch_arguments_not_metadata_loaded',
        'metadata_file_declared': metadata_file_declared,
        'metadata_file_exists': metadata_file_exists,
        'uses_active_metadata_defaults': uses_active_metadata_defaults,
        'launch_defaults': {key: defaults.get(key) for key in active_keys},
        'expected_from_metadata': expected,
    }


def text_has_all(text: str, fields: list[str]) -> tuple[bool, list[str]]:
    missing = [field for field in fields if field not in text]
    return not missing, missing


def check_goal_events_and_state_schema(root: Path) -> dict[str, Any]:
    explorer_text = (root / 'src/tugbot_maze/tugbot_maze/maze_explorer.py').read_text()
    launch_text = (root / 'src/tugbot_bringup/launch/tugbot_maze_explore.launch.py').read_text()
    required_goal_event_fields = [
        "'event'", "'goal_sequence'", "'dispatch_pose'", "'target'",
        "'target_exit_dist'", "'robot_exit_dist_at_dispatch'", "'goal_kind'",
        "'effective_timeout_sec'", "'result_status'", "'result_reason'",
        "'elapsed_sec'", "'near_exit'", "'candidate_branches'",
        "'chosen_branch_rank'", "'dispatch_local_cost_sample_age_sec'",
        "'dispatch_target_in_local_costmap_bounds'", "'dispatch_robot_in_local_costmap_bounds'",
        "'dispatch_path_local_cost_sample_count'", "'dispatch_local_cost_sample_coverage_ratio'",
        "'timeout_local_cost_sample_age_sec'", "'timeout_robot_in_local_costmap_bounds'",
        "'timeout_footprint_cost_max'", "'timeout_front_wedge_cost_max'",
        "'timeout_path_ahead_0_5m_cost_max'", "'blocked_branch_count'",
        "'blacklisted_goal_count'", "'mode'",
    ]
    required_state_fields = [
        "'mode'", "'goal_active'", "'goal_count'", "'goal_sequence_id'",
        "'effective_goal_timeout_sec'", "'goal_success_count'", "'goal_failure_count'",
        "'nav2_failure_count'", "'blocked_branch_count'", "'blacklisted_goal_count'",
        "'known_junctions'", "'edges'", "'exit_distance_m'",
        "'near_exit_fallback_enabled'", "'goal_settle_active'",
    ]
    event_ok, missing_event = text_has_all(explorer_text, required_goal_event_fields)
    state_ok, missing_state = text_has_all(explorer_text, required_state_fields)
    return {
        'goal_events_topic_configurable': 'goal_events_topic' in launch_text and '/maze/goal_events' in launch_text,
        'state_topic_configurable': 'state_topic' in launch_text and '/maze/explorer_state' in launch_text,
        'required_goal_event_fields_present': event_ok,
        'missing_goal_event_fields': missing_event,
        'required_state_fields_present': state_ok,
        'missing_state_fields': missing_state,
    }


def git_diff_empty(root: Path, rel_path: str) -> bool:
    result = subprocess.run(
        ['git', 'diff', '--', rel_path],
        cwd=root,
        check=False,
        text=True,
        capture_output=True,
    )
    return result.returncode == 0 and result.stdout == ''


def check_legacy_references(root: Path) -> dict[str, Any]:
    active_source_legacy_paths = [
        'src/tugbot_gazebo/worlds/tugbot_maze_world.sdf',
        'src/tugbot_maze/assets/maze_20260522.jpg',
        'src/tugbot_navigation/maps/map_1725111373.yaml',
        'src/tugbot_navigation/maps/explored/tugbot_nav_world_slam_phase14_perimeter_no_spin.yaml',
    ]
    current_launch_legacy_default_refs: list[str] = []
    for rel in CURRENT_MAZE_LAUNCHES:
        defaults = parse_launch_defaults(root / rel)
        for name, value in defaults.items():
            if any(token in value for token in [DECORATIVE_ROUTE_NAME, LEGACY_STATIC_MAP_NAME]):
                current_launch_legacy_default_refs.append(f'{rel}:{name}={value}')
            if OLD_SCAFFOLD_WORLD_NAME in value and ACTIVE_WORLD_NAME not in value:
                current_launch_legacy_default_refs.append(f'{rel}:{name}={value}')
    return {
        'active_source_legacy_files_absent': all(not (root / rel).exists() for rel in active_source_legacy_paths),
        'current_launch_legacy_default_refs': current_launch_legacy_default_refs,
        'nav2_config_diff_empty': git_diff_empty(root, 'src/tugbot_navigation/config'),
        'decorative_route_archived': (root / 'archive/deprecated_maze_worlds/assets/decorative_route_20260522/maze_20260522.jpg').exists(),
        'old_scaffold_archived': (root / 'archive/deprecated_maze_worlds/worlds/manual_simplified_first_pass_scaffold/tugbot_maze_world.sdf').exists(),
        'legacy_static_maps_archived': (root / 'archive/deprecated_maze_worlds/maps/static_legacy/map_1725111373.yaml').exists(),
    }


def check_metadata(metadata: dict[str, Any]) -> dict[str, Any]:
    expected = expected_truth_from_metadata(metadata)
    truth = metadata.get('map_frame_truth') or {}
    expected_values = {
        'entrance_x': float((truth.get('entrance') or {}).get('x_m', 0.0)),
        'entrance_y': float((truth.get('entrance') or {}).get('y_m', 0.0)),
        'entrance_yaw': float((truth.get('entrance') or {}).get('yaw_rad', 0.0)),
        'exit_x': float((truth.get('exit') or {}).get('x_m', 21.072562)),
        'exit_y': float((truth.get('exit') or {}).get('y_m', 18.083566)),
        'exit_radius': float((truth.get('exit') or {}).get('radius_m', 1.2)),
    }
    mismatches = []
    for key, value in expected_values.items():
        if abs(expected[key] - value) > 1e-6:
            mismatches.append({'key': key, 'expected': value, 'actual': expected[key]})
    return {
        'active_metadata_path': ACTIVE_METADATA_REL,
        'source_image': metadata.get('source_image'),
        'output_world': metadata.get('output_world'),
        'scale_factor': metadata.get('scale_factor'),
        'expected_truth': expected,
        'matches_phase36_expected_values': not mismatches,
        'mismatches': mismatches,
        'runtime_image_planning_disabled': metadata.get('runtime_policy', {}).get('image_allowed_for_runtime_path_planning') is False,
    }


def build_report(root: Path) -> dict[str, Any]:
    metadata = load_yaml(root / ACTIVE_METADATA_REL)
    launch_defaults = check_launch_defaults(root)
    explorer_truth = check_explorer_truth(root, metadata)
    schema = check_goal_events_and_state_schema(root)
    legacy = check_legacy_references(root)
    metadata_check = check_metadata(metadata)
    blockers: list[dict[str, str]] = []
    if not explorer_truth['uses_active_metadata_defaults']:
        blockers.append({
            'id': 'MAZE_EXPLORE_ENTRANCE_EXIT_DEFAULTS_NOT_ACTIVE_SCALED2X',
            'detail': 'tugbot_maze_explore.launch.py still defaults entrance/exit to the old first-pass scaffold values instead of active scaled2x metadata.',
        })
    if explorer_truth['metadata_file_declared'] != Path(ACTIVE_METADATA_REL).name or not explorer_truth['metadata_file_exists']:
        blockers.append({
            'id': 'MAZE_EXPLORE_METADATA_DEFAULT_POINTS_TO_MISSING_LEGACY_FILE',
            'detail': 'maze_config default is maze_instance.yaml and the file is not present in active config; active metadata is maze_20260528_scaled_instance.yaml.',
        })
    if not all(item['active_world_default_present'] and not item['old_scaffold_default_present'] for item in launch_defaults.values()):
        blockers.append({'id': 'CURRENT_MAZE_LAUNCH_DEFAULT_NOT_ACTIVE_WORLD', 'detail': 'One or more current maze launch defaults is not aligned to active scaled2x world.'})
    if not metadata_check['matches_phase36_expected_values']:
        blockers.append({'id': 'ACTIVE_METADATA_COORDINATES_MISMATCH', 'detail': 'Active metadata entrance/exit/radius do not match Phase36 expected coordinates.'})
    if not schema['required_goal_event_fields_present'] or not schema['required_state_fields_present']:
        blockers.append({'id': 'SMOKE_DIAGNOSTIC_SCHEMA_INCOMPLETE', 'detail': 'goal_events or explorer_state source schema lacks fields needed for bounded smoke diagnosis.'})
    if legacy['current_launch_legacy_default_refs']:
        blockers.append({'id': 'LEGACY_REFERENCE_IN_CURRENT_WORKFLOW_DEFAULT', 'detail': 'Current workflow launch defaults include deprecated world/map/image references.'})
    return {
        'phase': 'Phase36 Scaled Clean World Autonomous Exploration Readiness Review',
        'workspace_root': str(root),
        'active_world': f'src/tugbot_gazebo/worlds/{ACTIVE_WORLD_NAME}',
        'active_metadata': metadata_check,
        'launch_defaults': launch_defaults,
        'explorer_active_truth': explorer_truth,
        'goal_events_and_state_schema': schema,
        'legacy_references': legacy,
        'guardrails': {
            'autonomous_exploration_started': False,
            'maze_explorer_started': False,
            'nav2_params_modified': False,
            'strategy_modified': False,
            'long_run_started': False,
        },
        'blockers': blockers,
        'decision': 'NOT_READY_WITH_BLOCKERS' if blockers else 'READY_FOR_BOUNDED_AUTONOMOUS_EXPLORATION_SMOKE',
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace-root', type=Path, default=Path.cwd())
    parser.add_argument('--json', action='store_true', help='Print machine-readable JSON only.')
    parser.add_argument('--output', type=Path, help='Optional JSON output path.')
    args = parser.parse_args()
    root = args.workspace_root.resolve()
    report = build_report(root)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2, sort_keys=True) + '\n')
    if args.json:
        print(json.dumps(report, sort_keys=True))
    else:
        print(json.dumps(report, indent=2, sort_keys=True))


if __name__ == '__main__':
    main()
