#!/usr/bin/env python3
"""Phase131 first-dispatch-kind artifact replay diagnosis.

Offline analyzer only. It reads existing Phase124/125/129 artifacts and writes a
bounded diagnosis explaining why Phase129 observed a first dispatch kind of
``corridor_alignment_staging`` while Phase124/125 observed ``explore``.

It does not launch ROS/Gazebo/Nav2, send NavigateToPose goals, start
maze_explorer, tune configuration, or change navigation/exploration logic.
"""
from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path
from typing import Any

PHASE = 'Phase131'
MODE = 'first_dispatch_kind_artifact_replay'

CLASSIFICATIONS = [
    'FIRST_DISPATCH_KIND_STABLE_EXPLORE',
    'FIRST_DISPATCH_KIND_CHANGED_TO_STAGING_POST_INGRESS',
    'FIRST_DISPATCH_KIND_CHANGED_BY_POSE_TOPOLOGY_DRIFT',
    'FIRST_DISPATCH_KIND_CONTRACT_AMBIGUOUS',
    'INSUFFICIENT_DISPATCH_KIND_EVIDENCE',
]

FORBIDDEN_RUNTIME_ACTIONS = {
    'launch_gazebo_rviz_nav2': False,
    'send_navigate_to_pose_goal': False,
    'start_maze_explorer': False,
    'send_exploration_corridor_or_staging_goal': False,
    'tune_nav2_mppi_controller_goal_checker_config': False,
    'change_exploration_strategy_branch_scoring_centerline_fallback_terminal_acceptance': False,
    'claim_autonomous_or_exit_success': False,
}

DEFAULT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_PHASE124 = DEFAULT_ROOT / 'log' / 'phase124_first_exploration_goal_dispatch_smoke' / 'phase124_first_exploration_goal_dispatch_smoke_rerun_artifact.json'
DEFAULT_PHASE125 = DEFAULT_ROOT / 'log' / 'phase125_first_exploration_goal_execution_result_smoke' / 'phase125_first_exploration_goal_execution_result_smoke_artifact.json'
DEFAULT_PHASE129 = DEFAULT_ROOT / 'log' / 'phase129_instrumented_first_goal_timeout_diagnosis' / 'phase129_instrumented_first_goal_timeout_diagnosis_rerun.json'
DEFAULT_OUTPUT_JSON = DEFAULT_ROOT / 'log' / 'phase131_first_dispatch_kind_artifact_replay' / 'phase131_first_dispatch_kind_artifact_replay_analysis.json'
DEFAULT_OUTPUT_MD = DEFAULT_ROOT / 'log' / 'phase131_first_dispatch_kind_artifact_replay' / 'phase131_first_dispatch_kind_artifact_replay_summary.md'


MISSING = 'missing'


def _read_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding='utf-8'))


def _safe_get(obj: Any, path: list[Any], default: Any = None) -> Any:
    cur = obj
    for key in path:
        if isinstance(cur, dict):
            if key not in cur:
                return default
            cur = cur[key]
        elif isinstance(cur, list) and isinstance(key, int) and 0 <= key < len(cur):
            cur = cur[key]
        else:
            return default
    return cur


def _first_present(*values: Any) -> Any:
    for value in values:
        if value is not None:
            return value
    return None


def _as_float(value: Any, default: float | None = None) -> float | None:
    try:
        if value is None or isinstance(value, bool):
            return default
        f = float(value)
        if math.isnan(f) or math.isinf(f):
            return default
        return f
    except (TypeError, ValueError):
        return default


def _as_int(value: Any, default: int | None = None) -> int | None:
    try:
        if value is None or isinstance(value, bool):
            return default
        return int(value)
    except (TypeError, ValueError):
        return default


def _as_bool(value: Any) -> bool | None:
    if isinstance(value, bool):
        return value
    return None


def _xy(value: Any) -> list[float] | None:
    if isinstance(value, dict) and {'x', 'y'}.issubset(value):
        x = _as_float(value.get('x'))
        y = _as_float(value.get('y'))
        if x is not None and y is not None:
            return [x, y]
    if isinstance(value, (list, tuple)) and len(value) >= 2:
        x = _as_float(value[0])
        y = _as_float(value[1])
        if x is not None and y is not None:
            return [x, y]
    return None


def _pose(value: Any) -> list[float] | None:
    if isinstance(value, dict) and {'x', 'y'}.issubset(value):
        x = _as_float(value.get('x'))
        y = _as_float(value.get('y'))
        yaw = _as_float(value.get('yaw'), 0.0)
        if x is not None and y is not None and yaw is not None:
            return [x, y, yaw]
    if isinstance(value, (list, tuple)) and len(value) >= 3:
        x = _as_float(value[0])
        y = _as_float(value[1])
        yaw = _as_float(value[2])
        if x is not None and y is not None and yaw is not None:
            return [x, y, yaw]
    return None


def _distance_xy(a: Any, b: Any) -> float | None:
    aa = _xy(a)
    bb = _xy(b)
    if aa is None or bb is None:
        return None
    return float(math.hypot(aa[0] - bb[0], aa[1] - bb[1]))


def _yaw_delta(a: Any, b: Any) -> float | None:
    pa = _pose(a)
    pb = _pose(b)
    if pa is None or pb is None:
        return None
    return float(abs(math.atan2(math.sin(pa[2] - pb[2]), math.cos(pa[2] - pb[2]))))


def _value_field(value: Any, source: str | None = None) -> dict[str, Any]:
    return {
        'value': value if value is not None else MISSING,
        'available': value is not None,
        'source': source,
    }


def _find_dispatch_events(obj: Any) -> list[dict[str, Any]]:
    events: list[dict[str, Any]] = []

    def rec(cur: Any) -> None:
        if isinstance(cur, dict):
            if cur.get('event') == 'dispatch':
                events.append(cur)
            for value in cur.values():
                rec(value)
        elif isinstance(cur, list):
            for value in cur:
                rec(value)

    rec(obj)
    return events


def _first_dispatch_event(artifact: dict[str, Any]) -> dict[str, Any]:
    # Prefer the canonical goal_events arrays; they preserve the dispatch context.
    for path in (
        ['first_goal_artifact', 'goal_events'],
        ['first_goal_result_artifact', 'goal_events'],
        ['goal_events'],
    ):
        events = _safe_get(artifact, path, []) or []
        if isinstance(events, list):
            for event in events:
                if isinstance(event, dict) and event.get('event') == 'dispatch':
                    return event
    found = _find_dispatch_events(artifact)
    return found[0] if found else {}


def _first_goal(artifact: dict[str, Any]) -> dict[str, Any]:
    first = artifact.get('first_goal')
    return first if isinstance(first, dict) else {}


def _candidate(artifact: dict[str, Any]) -> dict[str, Any]:
    candidate = _safe_get(artifact, ['first_goal', 'candidate'], {})
    return candidate if isinstance(candidate, dict) else {}


def _frontier(artifact: dict[str, Any]) -> dict[str, Any]:
    frontier = _safe_get(artifact, ['first_goal', 'frontier_evidence'], {})
    return frontier if isinstance(frontier, dict) else {}


def _extract_robot_pose_after_ingress(artifact: dict[str, Any], dispatch_event: dict[str, Any]) -> tuple[list[float] | None, str | None]:
    # The first dispatch pose is the strongest cross-phase proxy when all runs
    # started maze_explorer immediately after Phase120 ingress/handoff.
    dispatch_pose = _pose(dispatch_event.get('dispatch_pose'))
    if dispatch_pose is not None:
        return dispatch_pose, 'first_dispatch_event.dispatch_pose'
    for path in (
        ['phase120_artifact', 'dispatch', 'final_pose'],
        ['phase120_ingress_artifact', 'dispatch', 'final_pose'],
        ['phase120_source_artifact', 'dispatch', 'final_pose'],
        ['handoff_artifact', 'ingress_goal_result', 'final_pose'],
    ):
        pose = _pose(_safe_get(artifact, path))
        if pose is not None:
            return pose, '.'.join(str(p) for p in path)
    return None, None


def _goal_count_max_goals(artifact: dict[str, Any], dispatch_event: dict[str, Any]) -> dict[str, Any]:
    max_goals = _first_present(artifact.get('maze_explorer_max_goals'), dispatch_event.get('max_goals'))
    goal_sequence = _first_present(dispatch_event.get('goal_sequence'), _safe_get(artifact, ['first_goal', 'goal_sequence']))
    goal_count_before = dispatch_event.get('goal_count_before_dispatch')
    if goal_count_before is None and _as_int(goal_sequence) is not None:
        goal_count_before = max(0, int(goal_sequence) - 1)
    return {
        'goal_sequence': goal_sequence,
        'goal_count_before_dispatch': goal_count_before,
        'max_goals': max_goals,
        'single_goal_budget': bool(_as_int(max_goals, 0) == 1),
    }


def _post_ingress_flags(dispatch_event: dict[str, Any]) -> dict[str, Any]:
    nested = dispatch_event.get('post_ingress_single_open_exception')
    if not isinstance(nested, dict):
        nested = {}
    keys = [
        'post_ingress_context_active',
        'first_post_ingress_topology_node',
        'single_open_exception_eligible',
        'single_open_exception_applied',
        'single_open_exception_reason',
        'post_ingress_single_open_exception_enabled',
    ]
    return {key: _first_present(dispatch_event.get(key), nested.get(key)) for key in keys}


def _active_edge_state_machine(artifact: dict[str, Any], dispatch_event: dict[str, Any]) -> dict[str, Any]:
    return {
        'active_edge_id': _first_present(dispatch_event.get('active_edge_id'), _safe_get(artifact, ['first_goal', 'active_edge_id'])),
        'active_goal_kind': _first_present(dispatch_event.get('active_goal_kind'), _safe_get(artifact, ['first_goal', 'goal_kind'])),
        'mode': _first_present(dispatch_event.get('mode'), artifact.get('mode')),
        'topology_consistency_guard': dispatch_event.get('topology_consistency_guard'),
        'topology_consistency_status': _first_present(
            dispatch_event.get('topology_consistency_status'),
            _safe_get(dispatch_event, ['topology_consistency_guard', 'topology_consistency_status']),
            _safe_get(dispatch_event, ['topology_consistency_guard', 'status']),
        ),
    }


def _staging_evidence(dispatch_event: dict[str, Any]) -> dict[str, Any]:
    centerline = dispatch_event.get('centerline_target_refinement')
    if not isinstance(centerline, dict):
        centerline = {}
    plan = dispatch_event.get('two_step_staging_plan')
    if not isinstance(plan, dict):
        plan = {}
    staging_goal_pose = dispatch_event.get('staging_goal_pose')
    return {
        'goal_kind': dispatch_event.get('goal_kind'),
        'staging_applied': bool(dispatch_event.get('staging_applied', False)),
        'two_step_stage_dispatch_requested': bool(dispatch_event.get('two_step_stage_dispatch_requested', False)),
        'staging_reason': dispatch_event.get('staging_reason'),
        'staging_reject_reason': dispatch_event.get('staging_reject_reason'),
        'staging_goal_pose': staging_goal_pose,
        'staging_lateral_residual_before_m': _safe_get(staging_goal_pose, ['lateral_residual_before_m']) if isinstance(staging_goal_pose, dict) else None,
        'staging_lateral_residual_after_m': _safe_get(staging_goal_pose, ['lateral_residual_after_m']) if isinstance(staging_goal_pose, dict) else None,
        'centerline_refinement_applied': bool(dispatch_event.get('centerline_refinement_applied', centerline.get('applied', False))),
        'centerline_refinement_reason': _first_present(dispatch_event.get('centerline_refinement_reason'), centerline.get('reason')),
        'centerline_projected_target': _first_present(dispatch_event.get('centerline_projected_target'), centerline.get('centerline_projected_target')),
        'corridor_heading_yaw': _first_present(dispatch_event.get('corridor_heading_yaw'), centerline.get('corridor_heading_yaw')),
        'source_forward_window_available': isinstance(dispatch_event.get('source_forward_window'), dict),
        'staging_window_available': isinstance(dispatch_event.get('staging_window'), dict),
        'staging_plan_enabled': plan.get('enabled'),
    }


def normalize_phase_artifact(phase_label: str, artifact_path: Path, artifact: dict[str, Any]) -> dict[str, Any]:
    dispatch_event = _first_dispatch_event(artifact)
    first_goal = _first_goal(artifact)
    candidate = _candidate(artifact)
    frontier = _frontier(artifact)
    pose_after_ingress, pose_source = _extract_robot_pose_after_ingress(artifact, dispatch_event)
    dispatch_pose = _pose(dispatch_event.get('dispatch_pose'))

    goal_kind = _first_present(candidate.get('goal_kind'), first_goal.get('goal_kind'), dispatch_event.get('goal_kind'))
    current_node = _first_present(candidate.get('current_node_id'), _safe_get(first_goal, ['topology_evidence', 'current_node_id']), dispatch_event.get('current_node_id'))
    start_node = _first_present(candidate.get('start_node_id'), _safe_get(first_goal, ['topology_evidence', 'start_node_id']), dispatch_event.get('start_node_id'))
    local_topology = _first_present(frontier.get('local_topology'), dispatch_event.get('local_topology'))
    candidate_family = _first_present(candidate.get('candidate_family'), dispatch_event.get('candidate_family'), local_topology)
    candidate_rank = _first_present(candidate.get('candidate_rank'), dispatch_event.get('candidate_rank'), dispatch_event.get('selected_candidate_index'))
    candidate_count = _first_present(candidate.get('candidate_count'), frontier.get('candidate_count'), dispatch_event.get('candidate_count'))
    candidate_branch_count = _first_present(frontier.get('candidate_branch_count'), dispatch_event.get('candidate_branch_count'))
    last_open_direction_count = _first_present(frontier.get('last_open_direction_count'), dispatch_event.get('last_open_direction_count'))
    last_candidate_count = _first_present(frontier.get('last_candidate_count'), dispatch_event.get('last_candidate_count'))
    near_exit = _first_present(candidate.get('near_exit'), dispatch_event.get('near_exit'))
    raw_target = _first_present(candidate.get('raw_target'), dispatch_event.get('target'))
    refined_target = _first_present(candidate.get('refined_target'), dispatch_event.get('refined_target'), dispatch_event.get('target'))
    original_target = _first_present(candidate.get('original_target'), dispatch_event.get('original_target'), raw_target)
    selection_reason = _first_present(first_goal.get('selection_reason'), dispatch_event.get('selection_reason'), dispatch_event.get('selected_due_to_context'))

    required_fields = {
        'robot_pose_after_ingress': _value_field(pose_after_ingress, pose_source),
        'dispatch_pose': _value_field(dispatch_pose, 'first_dispatch_event.dispatch_pose' if dispatch_pose is not None else None),
        'goal_kind': _value_field(goal_kind, 'first_goal.candidate.goal_kind/dispatch_event.goal_kind'),
        'current_node_id': _value_field(current_node, 'first_goal.candidate.current_node_id'),
        'start_node_id': _value_field(start_node, 'first_goal.candidate.start_node_id'),
        'topology_state': _value_field(local_topology, 'frontier_evidence.local_topology'),
        'local_topology': _value_field(local_topology, 'frontier_evidence.local_topology'),
        'candidate_family': _value_field(candidate_family, 'first_goal.candidate.candidate_family'),
        'candidate_rank': _value_field(candidate_rank, 'first_goal.candidate.candidate_rank'),
        'candidate_count': _value_field(candidate_count, 'first_goal.candidate.candidate_count'),
        'candidate_branch_count': _value_field(candidate_branch_count, 'frontier_evidence.candidate_branch_count'),
        'last_open_direction_count': _value_field(last_open_direction_count, 'frontier_evidence.last_open_direction_count'),
        'last_candidate_count': _value_field(last_candidate_count, 'frontier_evidence.last_candidate_count'),
        'near_exit': _value_field(near_exit, 'first_goal.candidate.near_exit'),
        'post_ingress_flags': _value_field(_post_ingress_flags(dispatch_event), 'dispatch_event.post_ingress_single_open_exception'),
        'active_edge_state_machine': _value_field(_active_edge_state_machine(artifact, dispatch_event), 'dispatch_event/state'),
        'goal_count_max_goals': _value_field(_goal_count_max_goals(artifact, dispatch_event), 'dispatch_event.goal_sequence + artifact.maze_explorer_max_goals'),
        'raw_target': _value_field(_xy(raw_target), 'first_goal.candidate.raw_target/dispatch_event.target'),
        'refined_target': _value_field(_xy(refined_target), 'first_goal.candidate.refined_target/dispatch_event.refined_target'),
        'original_target': _value_field(_xy(original_target), 'first_goal.candidate.original_target/dispatch_event.original_target'),
        'selection_reason': _value_field(selection_reason, 'first_goal.selection_reason/dispatch_event.selected_due_to_context'),
    }

    missing = [name for name, field in required_fields.items() if not field['available']]
    return {
        'phase': phase_label,
        'artifact_path': str(artifact_path),
        'artifact_classification': artifact.get('classification'),
        'dispatch_event_count': artifact.get('dispatch_event_count'),
        'first_dispatch_event_present': bool(dispatch_event),
        'required_fields': required_fields,
        'staging_evidence': _staging_evidence(dispatch_event),
        'candidate_branches': frontier.get('candidate_branches') or dispatch_event.get('candidate_branches') or [],
        'missing_required_fields': missing,
    }


def _required_value(phase: dict[str, Any], field: str) -> Any:
    return _safe_get(phase, ['required_fields', field, 'value'])


def _post_ingress_active(phase: dict[str, Any]) -> bool:
    flags = _required_value(phase, 'post_ingress_flags')
    if not isinstance(flags, dict):
        return False
    return bool(flags.get('post_ingress_context_active') or flags.get('first_post_ingress_topology_node'))


def _kind(phase: dict[str, Any]) -> Any:
    return _required_value(phase, 'goal_kind')


def _int_field(phase: dict[str, Any], field: str) -> int | None:
    return _as_int(_required_value(phase, field))


def _trigger_evidence(phases: list[dict[str, Any]]) -> dict[str, Any]:
    by_name = {phase['phase']: phase for phase in phases}
    p124 = by_name.get('Phase124', {})
    p125 = by_name.get('Phase125', {})
    p129 = by_name.get('Phase129', {})

    p125_pose = _required_value(p125, 'dispatch_pose')
    p129_pose = _required_value(p129, 'dispatch_pose')
    p125_robot_pose = _required_value(p125, 'robot_pose_after_ingress')
    p129_robot_pose = _required_value(p129, 'robot_pose_after_ingress')
    p125_yaw = _yaw_delta(p125_pose, p129_pose)
    p125_xy = _distance_xy(p125_pose, p129_pose)
    ingress_xy = _distance_xy(p125_robot_pose, p129_robot_pose)

    p125_branch = _int_field(p125, 'candidate_branch_count')
    p129_branch = _int_field(p129, 'candidate_branch_count')
    p125_open = _int_field(p125, 'last_open_direction_count')
    p129_open = _int_field(p129, 'last_open_direction_count')
    p125_candidate = _int_field(p125, 'last_candidate_count')
    p129_candidate = _int_field(p129, 'last_candidate_count')

    original_target = _required_value(p129, 'original_target')
    refined_target = _required_value(p129, 'refined_target')
    raw_target = _required_value(p129, 'raw_target')
    p129_dispatch_pose = _required_value(p129, 'dispatch_pose')
    original_to_refined = _distance_xy(original_target, refined_target)
    original_to_raw = _distance_xy(original_target, raw_target)
    refined_to_robot = _distance_xy(refined_target, p129_dispatch_pose)

    staging = p129.get('staging_evidence', {}) if isinstance(p129, dict) else {}
    post_flags = _required_value(p129, 'post_ingress_flags')
    if not isinstance(post_flags, dict):
        post_flags = {}
    active_state = _required_value(p129, 'active_edge_state_machine')
    if not isinstance(active_state, dict):
        active_state = {}
    goal_budget = _required_value(p129, 'goal_count_max_goals')
    if not isinstance(goal_budget, dict):
        goal_budget = {}

    return {
        'pose_yaw_drift': {
            'dispatch_pose_xy_delta_from_phase125_m': p125_xy,
            'robot_pose_after_ingress_xy_delta_from_phase125_m': ingress_xy,
            'dispatch_yaw_delta_from_phase125_rad': p125_yaw,
            'pose_yaw_drift_material': bool((p125_xy is not None and p125_xy >= 0.10) or (p125_yaw is not None and p125_yaw >= 0.35)),
        },
        'topology_or_candidate_drift': {
            'phase124_candidate_branch_count': _int_field(p124, 'candidate_branch_count'),
            'phase125_candidate_branch_count': p125_branch,
            'phase129_candidate_branch_count': p129_branch,
            'candidate_branch_count_delta_from_phase125': (p129_branch - p125_branch) if p129_branch is not None and p125_branch is not None else None,
            'phase125_last_open_direction_count': p125_open,
            'phase129_last_open_direction_count': p129_open,
            'last_open_direction_count_delta_from_phase125': (p129_open - p125_open) if p129_open is not None and p125_open is not None else None,
            'phase125_last_candidate_count': p125_candidate,
            'phase129_last_candidate_count': p129_candidate,
            'last_candidate_count_delta_from_phase125': (p129_candidate - p125_candidate) if p129_candidate is not None and p125_candidate is not None else None,
            'topology_state_phase125': _required_value(p125, 'topology_state'),
            'topology_state_phase129': _required_value(p129, 'topology_state'),
            'current_node_phase125': _required_value(p125, 'current_node_id'),
            'current_node_phase129': _required_value(p129, 'current_node_id'),
        },
        'centerline_corridor_alignment': {
            'staging_applied': bool(staging.get('staging_applied')),
            'two_step_stage_dispatch_requested': bool(staging.get('two_step_stage_dispatch_requested')),
            'staging_reason': staging.get('staging_reason'),
            'staging_lateral_residual_before_m': staging.get('staging_lateral_residual_before_m'),
            'staging_lateral_residual_after_m': staging.get('staging_lateral_residual_after_m'),
            'corridor_heading_yaw': staging.get('corridor_heading_yaw'),
        },
        'candidate_refinement': {
            'original_target': original_target,
            'raw_target': raw_target,
            'refined_target': refined_target,
            'original_to_refined_delta_m': original_to_refined,
            'original_to_raw_delta_m': original_to_raw,
            'refined_target_distance_from_robot_m': refined_to_robot,
            'transformed_forward_target_to_near_robot_staging_target': bool(
                original_to_refined is not None and original_to_refined > 0.5 and refined_to_robot is not None and refined_to_robot < 0.35
            ),
        },
        'post_ingress_context': {
            'post_ingress_context_active': bool(post_flags.get('post_ingress_context_active')),
            'first_post_ingress_topology_node': bool(post_flags.get('first_post_ingress_topology_node')),
            'single_open_exception_applied': bool(post_flags.get('single_open_exception_applied')),
            'single_open_exception_reason': post_flags.get('single_open_exception_reason'),
        },
        'state_machine_active_edge': {
            'active_edge_id': active_state.get('active_edge_id'),
            'active_goal_kind': active_state.get('active_goal_kind'),
            'mode': active_state.get('mode'),
            'topology_consistency_status': active_state.get('topology_consistency_status'),
        },
        'goal_budget_ordering': {
            'goal_sequence': goal_budget.get('goal_sequence'),
            'goal_count_before_dispatch': goal_budget.get('goal_count_before_dispatch'),
            'max_goals': goal_budget.get('max_goals'),
            'single_goal_budget': bool(goal_budget.get('single_goal_budget')),
        },
    }


def classify_dispatch_kind(phases: list[dict[str, Any]]) -> dict[str, Any]:
    if len(phases) < 3 or any(not phase.get('first_dispatch_event_present') for phase in phases):
        return {
            'classification': 'INSUFFICIENT_DISPATCH_KIND_EVIDENCE',
            'reasons': ['one or more first dispatch events are missing'],
        }
    by_name = {phase['phase']: phase for phase in phases}
    p124 = by_name.get('Phase124')
    p125 = by_name.get('Phase125')
    p129 = by_name.get('Phase129')
    if not p124 or not p125 or not p129:
        return {
            'classification': 'INSUFFICIENT_DISPATCH_KIND_EVIDENCE',
            'reasons': ['Phase124/Phase125/Phase129 normalized entries are all required'],
        }

    kinds = [_kind(p124), _kind(p125), _kind(p129)]
    missing_kind = [phase['phase'] for phase in (p124, p125, p129) if _kind(phase) == MISSING]
    if missing_kind:
        return {
            'classification': 'INSUFFICIENT_DISPATCH_KIND_EVIDENCE',
            'reasons': [f'missing goal_kind in {", ".join(missing_kind)}'],
        }
    if kinds == ['explore', 'explore', 'explore']:
        return {
            'classification': 'FIRST_DISPATCH_KIND_STABLE_EXPLORE',
            'reasons': ['Phase124/125/129 all observed first dispatch goal_kind=explore'],
        }
    if kinds[0] == 'explore' and kinds[1] == 'explore' and kinds[2] == 'corridor_alignment_staging':
        evidence = _trigger_evidence([p124, p125, p129])
        reasons: list[str] = ['Phase124/125 first dispatch remained explore, while Phase129 changed to corridor_alignment_staging']
        if bool(evidence['post_ingress_context']['post_ingress_context_active']):
            reasons.append('Phase129 artifact marks post_ingress_context_active=true')
            return {
                'classification': 'FIRST_DISPATCH_KIND_CHANGED_TO_STAGING_POST_INGRESS',
                'reasons': reasons,
            }
        topo = evidence['topology_or_candidate_drift']
        pose = evidence['pose_yaw_drift']
        candidate_refinement = evidence['candidate_refinement']
        if any(
            value not in (None, 0)
            for value in (
                topo.get('candidate_branch_count_delta_from_phase125'),
                topo.get('last_open_direction_count_delta_from_phase125'),
                topo.get('last_candidate_count_delta_from_phase125'),
            )
        ) or bool(pose.get('pose_yaw_drift_material')) or bool(candidate_refinement.get('transformed_forward_target_to_near_robot_staging_target')):
            if topo.get('candidate_branch_count_delta_from_phase125') is not None:
                reasons.append(
                    f"candidate_branch_count changed from {topo.get('phase125_candidate_branch_count')} to {topo.get('phase129_candidate_branch_count')}"
                )
            if candidate_refinement.get('transformed_forward_target_to_near_robot_staging_target'):
                reasons.append('Phase129 staging transformed an explore original_target into a near-robot staging target')
            if not pose.get('pose_yaw_drift_material'):
                reasons.append('dispatch pose/yaw drift is not material in the available artifacts')
            return {
                'classification': 'FIRST_DISPATCH_KIND_CHANGED_BY_POSE_TOPOLOGY_DRIFT',
                'reasons': reasons,
            }
        return {
            'classification': 'FIRST_DISPATCH_KIND_CONTRACT_AMBIGUOUS',
            'reasons': reasons + ['staging was observed but available artifacts do not expose post-ingress, pose, topology, or refinement trigger evidence'],
        }
    return {
        'classification': 'FIRST_DISPATCH_KIND_CONTRACT_AMBIGUOUS',
        'reasons': [f'unsupported dispatch kind sequence: {kinds}'],
    }


def _diagnosis_summary(classification: str, trigger_evidence: dict[str, Any], classification_reasons: list[str]) -> str:
    pieces = list(classification_reasons)
    if classification == 'FIRST_DISPATCH_KIND_CHANGED_BY_POSE_TOPOLOGY_DRIFT':
        topo = trigger_evidence['topology_or_candidate_drift']
        pieces.append(
            'Phase129 corridor_alignment_staging is supported by topology/candidate drift evidence: '
            f"candidate_branch_count {topo.get('phase125_candidate_branch_count')} -> {topo.get('phase129_candidate_branch_count')}, "
            f"last_open_direction_count {topo.get('phase125_last_open_direction_count')} -> {topo.get('phase129_last_open_direction_count')}, "
            f"last_candidate_count {topo.get('phase125_last_candidate_count')} -> {topo.get('phase129_last_candidate_count')}."
        )
        refine = trigger_evidence['candidate_refinement']
        if refine.get('transformed_forward_target_to_near_robot_staging_target'):
            pieces.append('Phase129 staging transformed an explore original_target into a near-robot staging target.')
        post = trigger_evidence['post_ingress_context']
        pieces.append(f"post_ingress_context_active={str(bool(post.get('post_ingress_context_active'))).lower()} in the dispatch event.")
    return ' '.join(str(p) for p in pieces if p)


def analyze_artifacts(*, phase124_path: Path, phase125_path: Path, phase129_path: Path) -> dict[str, Any]:
    artifacts = [
        normalize_phase_artifact('Phase124', phase124_path, _read_json(phase124_path)),
        normalize_phase_artifact('Phase125', phase125_path, _read_json(phase125_path)),
        normalize_phase_artifact('Phase129', phase129_path, _read_json(phase129_path)),
    ]
    classification_result = classify_dispatch_kind(artifacts)
    trigger = _trigger_evidence(artifacts)
    comparison = {phase['phase']: phase for phase in artifacts}
    kinds = [_kind(phase) for phase in artifacts]
    return {
        'phase': PHASE,
        'mode': MODE,
        'created_wall_time_sec': time.time(),
        'schema': 'phase131_first_dispatch_kind_artifact_replay.v1',
        'classification': classification_result['classification'],
        'classification_reasons': classification_result.get('reasons', []),
        'dispatch_kind_sequence': kinds,
        'phase_comparison': comparison,
        'trigger_evidence': trigger,
        'diagnosis_summary': _diagnosis_summary(
            classification_result['classification'],
            trigger,
            classification_result.get('reasons', []),
        ),
        'guardrails': {
            'artifact_replay_only': True,
            'source_artifacts': [str(phase124_path), str(phase125_path), str(phase129_path)],
            'forbidden_runtime_actions': FORBIDDEN_RUNTIME_ACTIONS,
            'phase132_entered': False,
        },
        'claims': {
            'autonomous_exploration_success': False,
            'exit_success': False,
            'timeout_repaired_or_reclassified_as_success': False,
        },
        'valid': classification_result['classification'] in CLASSIFICATIONS,
    }


def _fmt(value: Any) -> str:
    if isinstance(value, float):
        return f'{value:.6g}'
    if isinstance(value, (list, dict)):
        return json.dumps(value, sort_keys=True)
    return str(value)


def _comparison_table(result: dict[str, Any]) -> list[str]:
    fields = [
        'goal_kind',
        'dispatch_pose',
        'current_node_id',
        'start_node_id',
        'topology_state',
        'candidate_branch_count',
        'last_open_direction_count',
        'last_candidate_count',
        'near_exit',
        'raw_target',
        'original_target',
        'refined_target',
        'selection_reason',
    ]
    lines = ['| Field | Phase124 | Phase125 | Phase129 |', '| --- | --- | --- | --- |']
    comparison = result.get('phase_comparison', {})
    for field in fields:
        row = [field]
        for phase in ('Phase124', 'Phase125', 'Phase129'):
            value = _safe_get(comparison, [phase, 'required_fields', field, 'value'], MISSING)
            row.append(_fmt(value))
        lines.append('| ' + ' | '.join(row) + ' |')
    return lines


def write_outputs(result: dict[str, Any], output_json: Path, output_md: Path) -> None:
    output_json.parent.mkdir(parents=True, exist_ok=True)
    output_md.parent.mkdir(parents=True, exist_ok=True)
    output_json.write_text(json.dumps(result, indent=2, sort_keys=True) + '\n', encoding='utf-8')

    trigger = result['trigger_evidence']
    md: list[str] = []
    md.append('# Phase131 first dispatch kind artifact replay summary')
    md.append('')
    md.append(f"Classification: `{result['classification']}`")
    md.append('')
    md.append('## Guardrails')
    md.append('')
    md.append('- Artifact replay/analyzer only.')
    md.append('- No Gazebo/RViz/Nav2 runtime was launched.')
    md.append('- No NavigateToPose goal was sent.')
    md.append('- No maze_explorer was started.')
    md.append('- No exploration/corridor/staging goal was sent.')
    md.append('- No Nav2/MPPI/controller/goal checker/config tuning was performed.')
    md.append('- No exploration strategy/branch scoring/centerline/fallback/terminal acceptance change was made.')
    md.append('- No autonomous exploration success or exit success is claimed.')
    md.append('- Phase132 not entered.')
    md.append('')
    md.append('## Dispatch-kind sequence')
    md.append('')
    md.append(f"`{result['dispatch_kind_sequence']}`")
    md.append('')
    md.append('## Comparison')
    md.extend(_comparison_table(result))
    md.append('')
    md.append('## Trigger evidence')
    md.append('')
    md.append(f"- candidate_branch_count delta from Phase125: `{trigger['topology_or_candidate_drift']['candidate_branch_count_delta_from_phase125']}`")
    md.append(f"- last_open_direction_count delta from Phase125: `{trigger['topology_or_candidate_drift']['last_open_direction_count_delta_from_phase125']}`")
    md.append(f"- last_candidate_count delta from Phase125: `{trigger['topology_or_candidate_drift']['last_candidate_count_delta_from_phase125']}`")
    md.append(f"- dispatch pose XY delta from Phase125: `{trigger['pose_yaw_drift']['dispatch_pose_xy_delta_from_phase125_m']}`")
    md.append(f"- post_ingress_context_active: `{str(trigger['post_ingress_context']['post_ingress_context_active']).lower()}`")
    md.append(f"- staging_applied: `{str(trigger['centerline_corridor_alignment']['staging_applied']).lower()}`")
    md.append(f"- staging_reason: `{trigger['centerline_corridor_alignment']['staging_reason']}`")
    md.append(f"- transformed_forward_target_to_near_robot_staging_target: `{str(trigger['candidate_refinement']['transformed_forward_target_to_near_robot_staging_target']).lower()}`")
    md.append('')
    md.append('## Diagnostic summary')
    md.append('')
    md.append(result['diagnosis_summary'])
    md.append('')
    output_md.write_text('\n'.join(md) + '\n', encoding='utf-8')


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--phase124-artifact', type=Path, default=DEFAULT_PHASE124)
    parser.add_argument('--phase125-artifact', type=Path, default=DEFAULT_PHASE125)
    parser.add_argument('--phase129-artifact', type=Path, default=DEFAULT_PHASE129)
    parser.add_argument('--output-json', type=Path, default=DEFAULT_OUTPUT_JSON)
    parser.add_argument('--output-md', type=Path, default=DEFAULT_OUTPUT_MD)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    result = analyze_artifacts(
        phase124_path=args.phase124_artifact,
        phase125_path=args.phase125_artifact,
        phase129_path=args.phase129_artifact,
    )
    write_outputs(result, args.output_json, args.output_md)
    print(json.dumps({
        'classification': result['classification'],
        'dispatch_kind_sequence': result['dispatch_kind_sequence'],
        'output_json': str(args.output_json),
        'output_md': str(args.output_md),
    }, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
