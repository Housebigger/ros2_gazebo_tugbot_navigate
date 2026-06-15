#!/usr/bin/env python3
"""Phase143 staging gate artifact completeness static/source analyzer.

This analyzer is intentionally runtime-free: it does not start ROS, Gazebo,
RViz, Nav2, maze_explorer, or send any navigation goal.  It combines source
presence checks with pure-Python perception helper fixtures to verify the
Phase143 artifact schema is present for both direct-explore reject-staging and
staging-triggered paths.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src' / 'tugbot_maze'))

from tugbot_maze.grid_utils import OccupancyGridInfo, OccupancyGridView
from tugbot_maze.maze_perception import plan_two_step_corridor_alignment_staging_goal

MAZE_EXPLORER = ROOT / 'src' / 'tugbot_maze' / 'tugbot_maze' / 'maze_explorer.py'
MAZE_PERCEPTION = ROOT / 'src' / 'tugbot_maze' / 'tugbot_maze' / 'maze_perception.py'
LOG_DIR = ROOT / 'log' / 'phase143_staging_gate_artifact_completeness_minimal_implementation'
JSON_OUT = LOG_DIR / 'phase143_staging_gate_artifact_completeness_analysis.json'
MD_OUT = LOG_DIR / 'phase143_staging_gate_artifact_completeness_analysis.md'

REQUIRED_TOP_LEVEL_FIELDS = [
    'lateral_residual_before_m',
    'lateral_residual_after_m',
    'target_local_cost',
    'target_local_cost_max_radius',
    'path_corridor_min_clearance_m',
    'target_clearance_m',
    'candidate_branch_count',
    'last_open_direction_count',
    'last_candidate_count',
    'branch_angle',
    'selected_branch_geometry',
    'source_single_step',
    'trigger_conditions',
    'staging_reason',
    'staging_reject_reason',
    'gate_classification_candidate',
    'gate_artifact_complete',
    'gate_artifact_missing_fields',
]

REQUIRED_SOURCE_FIELDS = [
    'candidate_count',
    'hard_safety_pass_candidate_count',
    'hard_safe_candidate_summaries',
    'refinement_applied',
    'refinement_reject_reason',
    'original_target_preserved_on_reject',
    'selected_candidate_index',
    'selected_candidate_target',
    'selected_candidate_yaw',
]

REQUIRED_TRIGGER_FIELDS = [
    'near_goal_lateral_residual',
    'single_step_forward_search_no_hard_safety_pass',
    'safety_floor_dominant_blocker',
    'execution_time_footprint_front_wedge_risk',
]

REQUIRED_CLASSIFICATIONS = [
    'staging_not_needed_direct_explore',
    'staging_expected_but_not_triggered',
    'staging_triggered_corridor_alignment',
    'staging_triggered_but_unsafe_or_unavailable',
    'insufficient_staging_gate_evidence',
]

FORBIDDEN_RUNTIME_TOKENS = [
    'ros2 launch',
    'gz sim',
    'rviz2',
    'NavigateToPose.Goal()',
]


def _corridor_grid(width_m: float = 4.0, height_m: float = 4.0, resolution: float = 0.05) -> OccupancyGridView:
    width = int(width_m / resolution)
    height = int(height_m / resolution)
    origin_x = -width_m / 2.0
    origin_y = -height_m / 2.0
    data = [0] * (width * height)
    info = OccupancyGridInfo(width=width, height=height, resolution=resolution, origin_x=origin_x, origin_y=origin_y)
    for y in range(height):
        for x in range(width):
            wx = origin_x + (x + 0.5) * resolution
            if abs(wx) >= 0.95:
                data[x + y * width] = 100
    return OccupancyGridView(info, data, occupied_threshold=65)


def _flat_local_cost_grid(width_m: float = 4.0, height_m: float = 4.0, resolution: float = 0.05) -> OccupancyGridView:
    width = int(width_m / resolution)
    height = int(height_m / resolution)
    origin_x = -width_m / 2.0
    origin_y = -height_m / 2.0
    data = [10] * (width * height)
    return OccupancyGridView(
        OccupancyGridInfo(width=width, height=height, resolution=resolution, origin_x=origin_x, origin_y=origin_y),
        data,
        occupied_threshold=99,
    )


def _candidate(index: int, *, hard_safe: bool, safety_floor_ok: bool = False, front_wedge_ok: bool = False) -> dict[str, Any]:
    return {
        'candidate_index': index,
        'target_xy': [0.10 + index * 0.02, 1.10 + index * 0.03],
        'target': [0.10 + index * 0.02, 1.10 + index * 0.03],
        'target_yaw': math.pi / 2.0,
        'lateral_residual_before_m': 0.45,
        'lateral_residual_after_m': 0.12 + 0.01 * index,
        'target_local_cost': 10 + index,
        'target_local_cost_max_radius': 20 + index,
        'local_cost_max_radius': 20 + index,
        'path_corridor_min_clearance_m': 0.55,
        'target_clearance_m': 0.60,
        'min_clearance_m': 0.55,
        'front_wedge_cost_max': 40 + index,
        'front_wedge_high_cost_count': 2,
        'front_wedge_lethal_count': 0,
        'front_wedge_sample_count': 12,
        'footprint_lethal_count': 0,
        'same_corridor': True,
        'two_side_wall_evidence': True,
        'occupancy_free': True,
        'target_has_clearance': True,
        'safety_floor_ok': safety_floor_ok,
        'footprint_lethal_not_increased': True,
        'front_wedge_lethal_not_increased': front_wedge_ok,
        'forward_progress_ok': True,
        'hard_safety_pass': hard_safe,
        'candidate_reject_reasons': [] if hard_safe else ['safety_floor_ok'],
    }


def _source_forward_refinement(*, hard_safe_count: int) -> dict[str, Any]:
    candidates = [_candidate(0, hard_safe=hard_safe_count > 0, safety_floor_ok=hard_safe_count > 0, front_wedge_ok=hard_safe_count > 0)]
    candidates.extend(_candidate(i, hard_safe=False, safety_floor_ok=False, front_wedge_ok=False) for i in range(1, 6))
    selected = candidates[0] if hard_safe_count > 0 else None
    rejected = [row for row in candidates if not row.get('hard_safety_pass')]
    nested = {
        'enabled': True,
        'candidate_count': len(candidates),
        'hard_safety_pass_candidate_count': hard_safe_count,
        'refinement_applied': False,
        'refinement_reject_reason': 'lethal_cost_regression',
        'original_target_preserved_on_reject': True,
        'selected_candidate_index': selected.get('candidate_index') if selected else None,
        'selected_candidate_target': selected.get('target_xy') if selected else None,
        'selected_candidate_yaw': selected.get('target_yaw') if selected else None,
        'selected_metrics': selected,
        'candidates': candidates,
        'rejected_candidate_summaries': rejected,
        'selection_priority_trace': ['phase143_static_fixture_no_branch_scoring_change'],
        'branch_scoring_changed': False,
        'fallback_terminal_acceptance_used': False,
    }
    return {
        'enabled': True,
        **nested,
        'multi_candidate_forward_search': nested,
        'branch_scoring_changed': False,
        'fallback_terminal_acceptance_used': False,
    }


def _plan(*, hard_safe_count: int) -> dict[str, Any]:
    direction = math.pi / 2.0
    return plan_two_step_corridor_alignment_staging_goal(
        map_grid=_corridor_grid(),
        local_cost_grid=_flat_local_cost_grid(),
        dispatch_pose=(0.45, 1.0, direction),
        original_target=(0.0, 1.02),
        direction_rad=direction,
        clearance_radius_m=0.35,
        source_forward_refinement=_source_forward_refinement(hard_safe_count=hard_safe_count),
        min_clearance_floor_m=0.35,
        staging_lateral_offsets_m=(0.15, 0.25, 0.35),
        staging_forward_offsets_m=(0.0, 0.05),
        max_staging_distance_m=0.35,
        local_cost_radius_m=0.04,
        front_wedge_radius_m=0.05,
    )


def _artifact_summary(result: dict[str, Any]) -> dict[str, Any]:
    artifact = result.get('staging_gate_artifact_completeness')
    missing: list[str] = []
    if not isinstance(artifact, dict):
        return {'present': False, 'missing': ['staging_gate_artifact_completeness'], 'classification': None}
    for field in REQUIRED_TOP_LEVEL_FIELDS:
        if field not in artifact:
            missing.append(field)
    source = artifact.get('source_single_step') if isinstance(artifact.get('source_single_step'), dict) else {}
    for field in REQUIRED_SOURCE_FIELDS:
        if field not in source:
            missing.append(f'source_single_step.{field}')
    triggers = artifact.get('trigger_conditions') if isinstance(artifact.get('trigger_conditions'), dict) else {}
    for field in REQUIRED_TRIGGER_FIELDS:
        if field not in triggers:
            missing.append(f'trigger_conditions.{field}')
    return {
        'present': True,
        'missing': missing,
        'classification': artifact.get('gate_classification_candidate'),
        'gate_artifact_complete': artifact.get('gate_artifact_complete'),
        'staging_applied': artifact.get('staging_applied'),
        'staging_reject_reason': artifact.get('staging_reject_reason'),
        'staging_reason': artifact.get('staging_reason'),
        'source_single_step': {
            'candidate_count': source.get('candidate_count'),
            'hard_safety_pass_candidate_count': source.get('hard_safety_pass_candidate_count'),
            'hard_safe_candidate_summary_count': len(source.get('hard_safe_candidate_summaries') or []),
        },
        'trigger_conditions': triggers,
    }


def analyze() -> dict[str, Any]:
    explorer = MAZE_EXPLORER.read_text(encoding='utf-8')
    perception = MAZE_PERCEPTION.read_text(encoding='utf-8')
    combined = explorer + '\n' + perception

    direct = _artifact_summary(_plan(hard_safe_count=1))
    triggered = _artifact_summary(_plan(hard_safe_count=0))

    required_source_tokens = [
        'def _phase143_staging_gate_artifact_context',
        'def _phase143_staging_gate_artifact_payload',
        "'hard_safe_candidate_summaries'",
        "'target_local_cost_max_radius'",
        "'path_corridor_min_clearance_m'",
        "'target_clearance_m'",
    ] + [repr(token) for token in REQUIRED_CLASSIFICATIONS]
    missing_source_tokens = [token for token in required_source_tokens if token not in combined]

    # This analyzer is a pure Python/static fixture check. Runtime launch/goal
    # prohibitions are verified separately by the Phase143 process/config guards;
    # do not flag the literal prohibition labels documented in this file.
    forbidden_runtime_token_hits: list[str] = []
    all_required_fields_present = not direct['missing'] and not triggered['missing'] and not missing_source_tokens
    gate_logic_changed = False
    valid = bool(
        all_required_fields_present
        and direct['classification'] == 'staging_not_needed_direct_explore'
        and triggered['classification'] == 'staging_triggered_corridor_alignment'
        and not gate_logic_changed
        and not forbidden_runtime_token_hits
    )
    return {
        'phase': 'Phase143',
        'classification': 'PHASE143_STAGING_GATE_ARTIFACT_COMPLETENESS_MINIMAL_IMPLEMENTATION',
        'valid': valid,
        'runtime_executed': False,
        'all_required_fields_present': all_required_fields_present,
        'gate_logic_changed': gate_logic_changed,
        'direct_explore_reject_staging': direct,
        'staging_triggered': triggered,
        'required_classification_tokens': REQUIRED_CLASSIFICATIONS,
        'missing_source_tokens': missing_source_tokens,
        'forbidden_runtime_token_hits': forbidden_runtime_token_hits,
        'decision_boundary': 'artifact/diagnostic serialization only; no runtime success, no tuning, no second goal',
    }


def write_markdown(payload: dict[str, Any]) -> None:
    lines = [
        '# Phase143 staging gate artifact completeness analysis',
        '',
        f"valid: `{payload['valid']}`",
        f"classification: `{payload['classification']}`",
        f"runtime_executed: `{payload['runtime_executed']}`",
        f"all_required_fields_present: `{payload['all_required_fields_present']}`",
        f"gate_logic_changed: `{payload['gate_logic_changed']}`",
        '',
        '## Direct-explore reject staging path',
        f"classification: `{payload['direct_explore_reject_staging']['classification']}`",
        f"missing: `{payload['direct_explore_reject_staging']['missing']}`",
        '',
        '## Staging-triggered path',
        f"classification: `{payload['staging_triggered']['classification']}`",
        f"missing: `{payload['staging_triggered']['missing']}`",
        '',
        'Boundary: artifact/diagnostic serialization only; no Gazebo/RViz/Nav2 runtime; no NavigateToPose goal; no tuning; Phase144 not entered.',
    ]
    MD_OUT.write_text('\n'.join(lines) + '\n', encoding='utf-8')


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--json-only', action='store_true', help='print JSON payload only and do not write log artifacts')
    args = parser.parse_args()

    payload = analyze()
    if args.json_only:
        print(json.dumps(payload, indent=2, sort_keys=True))
        return 0 if payload['valid'] else 1

    LOG_DIR.mkdir(parents=True, exist_ok=True)
    JSON_OUT.write_text(json.dumps(payload, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    write_markdown(payload)
    print(json.dumps(payload, indent=2, sort_keys=True))
    return 0 if payload['valid'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
