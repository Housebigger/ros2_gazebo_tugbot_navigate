"""Local maze topology perception from occupancy grids.

The functions here classify the robot's local neighborhood into corridor,
junction, or dead-end by sampling safe rays in discrete directions. They are
pure Python so the behavior is testable without Gazebo/Nav2.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, List, Optional, Sequence, Tuple

from .grid_utils import OccupancyGridView

Point = Tuple[float, float]

DEAD_END = 'dead_end'
JUNCTION = 'junction'
CORRIDOR = 'corridor'
UNKNOWN = 'unknown'


@dataclass(frozen=True)
class OpenDirection:
    angle_rad: float
    target_xy: Point
    distance_m: float


@dataclass(frozen=True)
class LocalTopology:
    kind: str
    open_directions: List[OpenDirection]


def normalize_angle(angle_rad: float) -> float:
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def sample_open_directions(
    grid: OccupancyGridView,
    pose: Tuple[float, float, float],
    angle_step_deg: float = 30.0,
    lookahead_m: float = 1.5,
    clearance_radius_m: float = 0.35,
    min_open_distance_m: float = 0.45,
) -> List[OpenDirection]:
    x, y, yaw = pose
    step_rad = math.radians(angle_step_deg)
    if step_rad <= 0.0:
        raise ValueError('angle_step_deg must be positive')
    sample_count = max(1, int(round((2.0 * math.pi) / step_rad)))
    directions: List[OpenDirection] = []
    for index in range(sample_count):
        angle = normalize_angle(yaw + index * step_rad)
        safe_distance = _safe_distance_along_ray(
            grid,
            origin_xy=(x, y),
            direction_rad=angle,
            lookahead_m=lookahead_m,
            clearance_radius_m=clearance_radius_m,
        )
        if safe_distance >= min_open_distance_m:
            target = (x + math.cos(angle) * safe_distance, y + math.sin(angle) * safe_distance)
            directions.append(OpenDirection(angle_rad=angle, target_xy=target, distance_m=safe_distance))
    return _merge_similar_directions(directions, merge_angle_rad=max(step_rad * 0.75, math.radians(10.0)))


def classify_local_topology(
    grid: OccupancyGridView,
    pose: Tuple[float, float, float],
    angle_step_deg: float = 30.0,
    lookahead_m: float = 1.5,
    clearance_radius_m: float = 0.35,
    min_open_distance_m: float = 0.45,
) -> LocalTopology:
    open_directions = sample_open_directions(
        grid,
        pose,
        angle_step_deg=angle_step_deg,
        lookahead_m=lookahead_m,
        clearance_radius_m=clearance_radius_m,
        min_open_distance_m=min_open_distance_m,
    )
    count = len(open_directions)
    if count <= 0:
        kind = UNKNOWN
    elif count == 1:
        kind = DEAD_END
    elif count == 2:
        kind = CORRIDOR
    else:
        kind = JUNCTION
    return LocalTopology(kind=kind, open_directions=open_directions)


def filter_open_directions(
    directions: List[OpenDirection],
    robot_yaw: float,
    allow_reverse: bool = True,
    reverse_angle_threshold_deg: float = 135.0,
) -> List[OpenDirection]:
    if allow_reverse:
        return list(directions)
    threshold = math.radians(reverse_angle_threshold_deg)
    return [direction for direction in directions if abs(normalize_angle(direction.angle_rad - robot_yaw)) < threshold]


def make_branch_goal(
    grid: OccupancyGridView,
    pose_xy: Point,
    direction_rad: float,
    preferred_step_m: float = 1.0,
    min_step_m: float = 0.45,
    clearance_radius_m: float = 0.35,
) -> Optional[Point]:
    safe_distance = _safe_distance_along_ray(
        grid,
        origin_xy=pose_xy,
        direction_rad=direction_rad,
        lookahead_m=preferred_step_m,
        clearance_radius_m=clearance_radius_m,
    )
    if safe_distance < min_step_m:
        return None
    return (
        pose_xy[0] + math.cos(direction_rad) * safe_distance,
        pose_xy[1] + math.sin(direction_rad) * safe_distance,
    )


def make_centered_branch_goal(
    grid: OccupancyGridView,
    pose_xy: Point,
    direction_rad: float,
    preferred_step_m: float = 1.0,
    min_step_m: float = 0.45,
    clearance_radius_m: float = 0.35,
    lateral_search_m: float = 0.8,
) -> Optional[Point]:
    raw_goal = make_branch_goal(
        grid,
        pose_xy=pose_xy,
        direction_rad=direction_rad,
        preferred_step_m=preferred_step_m,
        min_step_m=min_step_m,
        clearance_radius_m=clearance_radius_m,
    )
    if raw_goal is None:
        return None

    lateral_angle = direction_rad + math.pi / 2.0
    left_clearance = _distance_to_blocked_boundary(
        grid,
        origin_xy=raw_goal,
        direction_rad=lateral_angle,
        lookahead_m=lateral_search_m,
        clearance_radius_m=clearance_radius_m,
    )
    right_clearance = _distance_to_blocked_boundary(
        grid,
        origin_xy=raw_goal,
        direction_rad=lateral_angle + math.pi,
        lookahead_m=lateral_search_m,
        clearance_radius_m=clearance_radius_m,
    )
    lateral_offset = (left_clearance - right_clearance) / 2.0
    centered = (
        raw_goal[0] + math.cos(lateral_angle) * lateral_offset,
        raw_goal[1] + math.sin(lateral_angle) * lateral_offset,
    )
    if grid.world_point_has_clearance(centered[0], centered[1], clearance_radius_m):
        return centered
    return raw_goal


def refine_corridor_centerline_target(
    *,
    map_grid: OccupancyGridView,
    local_cost_grid: Optional[OccupancyGridView],
    dispatch_pose: Tuple[float, float, float],
    original_target: Point,
    direction_rad: float,
    clearance_radius_m: float,
    side_probe_m: float = 1.5,
    forward_offsets_m: Sequence[float] = (0.0, 0.1, 0.2),
    lateral_offsets_m: Sequence[float] = (-0.3, -0.2, -0.1, 0.0, 0.1, 0.2, 0.3),
    heading_offsets_rad: Sequence[float] = (math.radians(-5.0), 0.0, math.radians(5.0)),
    local_cost_radius_m: float = 0.30,
    front_wedge_radius_m: float = 0.75,
    front_wedge_half_angle_rad: float = math.radians(35.0),
    high_cost_threshold: int = 70,
    balance_improvement_epsilon_m: float = 0.02,
    gate_mode: str = 'all_metrics',
    min_clearance_floor_m: Optional[float] = None,
    forward_progress_tolerance_m: float = 0.0,
) -> dict[str, Any]:
    """Narrow runtime gate for corridor-centerline dispatch-target refinement.

    This is intentionally pure evidence logic: it does not rank branches, tune
    Nav2, change clearance/map thresholds, or add fallback/terminal acceptance.
    It only evaluates the already-selected dispatch target and returns a target
    replacement when all centerline runtime gate conditions hold.
    """
    forward = (math.cos(direction_rad), math.sin(direction_rad))
    lateral = (math.cos(direction_rad + math.pi / 2.0), math.sin(direction_rad + math.pi / 2.0))
    dispatch_xy = (float(dispatch_pose[0]), float(dispatch_pose[1]))
    original_metrics = _centerline_candidate_metrics(
        map_grid=map_grid,
        local_cost_grid=local_cost_grid,
        dispatch_xy=dispatch_xy,
        target_xy=original_target,
        direction_rad=direction_rad,
        clearance_radius_m=clearance_radius_m,
        side_probe_m=side_probe_m,
        local_cost_radius_m=local_cost_radius_m,
        front_wedge_radius_m=front_wedge_radius_m,
        front_wedge_half_angle_rad=front_wedge_half_angle_rad,
        high_cost_threshold=high_cost_threshold,
    )

    normalized_gate_mode = str(gate_mode or 'all_metrics').strip().lower()
    if normalized_gate_mode not in {'all_metrics', 'balance_first'}:
        normalized_gate_mode = 'all_metrics'
    clearance_floor = float(clearance_radius_m if min_clearance_floor_m is None else min_clearance_floor_m)
    progress_tolerance = max(0.0, float(forward_progress_tolerance_m))

    normalized_forward_offsets = [float(value) for value in forward_offsets_m] or [0.0]
    normalized_lateral_offsets = [float(value) for value in lateral_offsets_m] or [0.0]
    normalized_heading_offsets = [float(value) for value in heading_offsets_rad] or [0.0]
    candidate_family = {
        'centerline_projection': True,
        'bounded_local_search': True,
        'forward_offsets_m': normalized_forward_offsets,
        'lateral_offsets_m': normalized_lateral_offsets,
        'heading_offsets_rad': normalized_heading_offsets,
    }
    selection_priority_trace = [
        'hard safety pass',
        'no footprint/front-wedge lethal regression',
        'safety_floor_ok',
        'forward_progress_ok',
        'clearance better',
        'balance error smaller',
    ]

    candidates: list[dict[str, Any]] = []
    forward_regression_seen = False
    safety_floor_blocked_seen = False
    lethal_regression_seen = False
    for forward_offset in normalized_forward_offsets:
        for lateral_offset in normalized_lateral_offsets:
            candidate = (
                float(original_target[0]) + forward[0] * float(forward_offset) + lateral[0] * float(lateral_offset),
                float(original_target[1]) + forward[1] * float(forward_offset) + lateral[1] * float(lateral_offset),
            )
            for heading_offset in normalized_heading_offsets:
                candidate_yaw = normalize_angle(direction_rad + float(heading_offset))
                metrics = _centerline_candidate_metrics(
                    map_grid=map_grid,
                    local_cost_grid=local_cost_grid,
                    dispatch_xy=dispatch_xy,
                    target_xy=candidate,
                    direction_rad=direction_rad,
                    footprint_yaw_rad=candidate_yaw,
                    clearance_radius_m=clearance_radius_m,
                    side_probe_m=side_probe_m,
                    local_cost_radius_m=local_cost_radius_m,
                    front_wedge_radius_m=front_wedge_radius_m,
                    front_wedge_half_angle_rad=front_wedge_half_angle_rad,
                    high_cost_threshold=high_cost_threshold,
                )
                metrics['candidate_index'] = len(candidates)
                metrics['target_xy'] = list(metrics['target'])
                metrics['target_yaw'] = float(candidate_yaw)
                metrics['forward_offset_m'] = float(forward_offset)
                metrics['lateral_offset_m'] = float(lateral_offset)
                metrics['heading_offset_rad'] = float(heading_offset)
                metrics['centerline_projection'] = True
                metrics['forward_progress_not_lowered'] = bool(
                    metrics['forward_progress_m'] + 1e-9 >= original_metrics['forward_progress_m']
                )
                metrics['forward_progress_not_obviously_lowered'] = bool(
                    metrics['forward_progress_m'] + progress_tolerance + 1e-9 >= original_metrics['forward_progress_m']
                )
                metrics['forward_progress_ok'] = bool(metrics['forward_progress_not_obviously_lowered'])
                metrics['improves_balance'] = bool(
                    metrics['balance_error_m'] <= max(0.0, original_metrics['balance_error_m'] - balance_improvement_epsilon_m)
                )
                metrics['balance_error_improved'] = metrics['improves_balance']
                metrics['improves_min_clearance'] = bool(
                    metrics['min_clearance_m'] + 1e-9 >= original_metrics['min_clearance_m']
                )
                metrics['safety_min_clearance_floor_m'] = clearance_floor
                metrics['safety_floor_ok'] = bool(metrics['min_clearance_m'] + 1e-9 >= clearance_floor)
                metrics['improves_local_cost'] = _optional_number_leq(
                    metrics.get('local_cost_max_radius'), original_metrics.get('local_cost_max_radius')
                )
                metrics['improves_front_wedge'] = bool(
                    int(metrics['front_wedge_high_cost_count']) <= int(original_metrics['front_wedge_high_cost_count'])
                    and int(metrics['front_wedge_lethal_count']) <= int(original_metrics['front_wedge_lethal_count'])
                )
                metrics['footprint_lethal_not_increased'] = bool(
                    int(metrics['footprint_lethal_count']) <= int(original_metrics['footprint_lethal_count'])
                )
                metrics['front_wedge_lethal_not_increased'] = bool(
                    int(metrics['front_wedge_lethal_count']) <= int(original_metrics['front_wedge_lethal_count'])
                )
                metrics['no_footprint_front_wedge_lethal_regression'] = bool(
                    metrics['footprint_lethal_not_increased'] and metrics['front_wedge_lethal_not_increased']
                )
                metrics['hard_safety_pass'] = bool(
                    metrics['same_corridor']
                    and metrics['two_side_wall_evidence']
                    and metrics['target_has_clearance']
                    and metrics['occupancy_free']
                    and metrics['no_footprint_front_wedge_lethal_regression']
                    and metrics['safety_floor_ok']
                    and metrics['forward_progress_ok']
                )
                metrics['eligible'] = bool(
                    metrics['same_corridor']
                    and metrics['two_side_wall_evidence']
                    and metrics['target_has_clearance']
                    and metrics['occupancy_free']
                    and metrics['forward_progress_not_lowered']
                    and metrics['improves_balance']
                    and metrics['improves_min_clearance']
                    and metrics['improves_local_cost']
                    and metrics['improves_front_wedge']
                )
                metrics['balance_first_eligible'] = bool(metrics['hard_safety_pass'] and metrics['balance_error_improved'])
                metrics['selection_rank_tuple'] = [
                    0 if metrics['hard_safety_pass'] else 1,
                    0 if metrics['no_footprint_front_wedge_lethal_regression'] else 1,
                    0 if metrics['safety_floor_ok'] else 1,
                    0 if metrics['forward_progress_ok'] else 1,
                    _number_or_inf(metrics.get('local_cost_max_radius')),
                    int(metrics['front_wedge_high_cost_count']),
                    int(metrics.get('footprint_high_cost_count', 0)),
                    -float(metrics['min_clearance_m']),
                    float(metrics['balance_error_m']),
                    abs(float(heading_offset)),
                    int(metrics['candidate_index']),
                ]
                metrics['candidate_reject_reasons'] = _candidate_reject_reasons(metrics)
                if metrics['same_corridor'] and metrics['two_side_wall_evidence'] and not metrics['forward_progress_not_lowered']:
                    forward_regression_seen = True
                if metrics['same_corridor'] and metrics['two_side_wall_evidence'] and not metrics['safety_floor_ok']:
                    safety_floor_blocked_seen = True
                if metrics['same_corridor'] and metrics['two_side_wall_evidence'] and (
                    not metrics['footprint_lethal_not_increased'] or not metrics['front_wedge_lethal_not_increased']
                ):
                    lethal_regression_seen = True
                candidates.append(metrics)

    two_side_candidates = [row for row in candidates if row['same_corridor'] and row['two_side_wall_evidence']]
    strict_eligible = [row for row in candidates if row['eligible']]
    balance_first_eligible = [row for row in candidates if row['balance_first_eligible']]
    safety_first_eligible = [row for row in candidates if row['hard_safety_pass']]
    eligible = safety_first_eligible
    selected = min(
        eligible,
        key=lambda row: tuple(row['selection_rank_tuple']),
        default=None,
    )
    reason = 'balance_first_applied' if (selected is not None and normalized_gate_mode == 'balance_first') else ('applied' if selected is not None else 'no_improving_candidate')
    original_two_side = bool(original_metrics.get('two_side_wall_evidence'))
    if selected is None and not candidates:
        reason = 'no_candidate_family_generated'
    elif selected is None and not bool(original_metrics.get('same_corridor')):
        reason = 'missing_same_corridor_evidence'
    elif selected is None and (not original_two_side or not two_side_candidates):
        reason = 'missing_two_side_wall_evidence'
    elif selected is None and forward_regression_seen:
        reason = 'forward_progress_regression'
    elif selected is None and lethal_regression_seen:
        # Preserve the Phase70/85 compatibility token while the rejected
        # candidate summaries carry the more detailed Phase88 subitem evidence.
        reason = 'lethal_cost_regression'
    elif selected is None and safety_floor_blocked_seen:
        reason = 'safety_floor_blocked'
    elif selected is None:
        reason = 'all_candidates_failed_hard_safety'

    applied = bool(selected is not None)
    original_target_payload = [float(original_target[0]), float(original_target[1])]
    selected_candidate_index = int(selected['candidate_index']) if selected is not None else None
    selected_candidate_target = [float(selected['target'][0]), float(selected['target'][1])] if selected is not None else None
    selected_candidate_yaw = float(selected['target_yaw']) if selected is not None else None
    centerline_projected_target = selected_candidate_target
    refined_target = centerline_projected_target if centerline_projected_target is not None else original_target_payload
    original_target_preserved_on_reject = bool(not applied and refined_target == original_target_payload)
    forward_check_source = selected if selected is not None else original_metrics
    forward_executability_check = {
        'checked': True,
        'passed': applied,
        'reason': None if applied else reason,
        'same_corridor': bool(forward_check_source.get('same_corridor', False)),
        'two_side_wall_evidence': bool(forward_check_source.get('two_side_wall_evidence', False)),
        'target_has_clearance': bool(forward_check_source.get('target_has_clearance', False)),
        'occupancy_free': bool(forward_check_source.get('occupancy_free', False)),
        'forward_progress_not_lowered': bool(forward_check_source.get('forward_progress_not_lowered', applied)),
        'forward_progress_not_obviously_lowered': bool(
            forward_check_source.get('forward_progress_not_obviously_lowered', applied)
        ),
        'forward_progress_ok': bool(forward_check_source.get('forward_progress_ok', applied)),
        'safety_floor_ok': bool(forward_check_source.get('safety_floor_ok', applied)),
        'footprint_lethal_not_increased': bool(forward_check_source.get('footprint_lethal_not_increased', applied)),
        'front_wedge_lethal_not_increased': bool(forward_check_source.get('front_wedge_lethal_not_increased', applied)),
        'hard_safety_pass': bool(forward_check_source.get('hard_safety_pass', applied)),
        'local_cost_sample_count': int(forward_check_source.get('local_cost_sample_count', 0)),
        'front_wedge_sample_count': int(forward_check_source.get('front_wedge_sample_count', 0)),
    }
    rejected_candidate_summaries = [_compact_candidate_reject_summary(row) for row in candidates if row is not selected]
    hard_safety_pass_candidate_count = len(safety_first_eligible)
    multi_candidate_forward_search = {
        'enabled': True,
        'candidate_family': candidate_family,
        'original_target': original_target_payload,
        'original_metrics': original_metrics,
        'candidates': candidates,
        'candidate_count': len(candidates),
        'hard_safety_pass_candidate_count': hard_safety_pass_candidate_count,
        'selected_candidate_index': selected_candidate_index,
        'selected_candidate_target': selected_candidate_target,
        'selected_candidate_yaw': selected_candidate_yaw,
        'selection_priority_trace': selection_priority_trace,
        'rejected_candidate_summaries': rejected_candidate_summaries,
        'refinement_applied': applied,
        'refinement_reject_reason': None if applied else reason,
        'original_target_preserved_on_reject': original_target_preserved_on_reject,
        'branch_scoring_changed': False,
        'fallback_terminal_acceptance_used': False,
    }

    return {
        'enabled': True,
        'gate_mode': normalized_gate_mode,
        'applied': applied,
        'reason': reason,
        'branch_scoring_changed': False,
        'fallback_terminal_acceptance_used': False,
        'original_target': original_target_payload,
        'refined_target': refined_target,
        'centerline_projected_target': centerline_projected_target,
        'corridor_heading_yaw': float(selected_candidate_yaw if selected_candidate_yaw is not None else normalize_angle(direction_rad)),
        'refinement_applied': applied,
        'refinement_reject_reason': None if applied else reason,
        'forward_executability_check': forward_executability_check,
        'original_metrics': original_metrics,
        'selected_metrics': selected,
        'candidate_family': candidate_family,
        'candidate_count': len(candidates),
        'eligible_candidate_count': len(eligible),
        'strict_eligible_candidate_count': len(strict_eligible),
        'balance_first_eligible_candidate_count': len(balance_first_eligible),
        'hard_safety_pass_candidate_count': hard_safety_pass_candidate_count,
        'two_side_wall_candidate_count': len(two_side_candidates),
        'selected_candidate_index': selected_candidate_index,
        'selected_candidate_target': selected_candidate_target,
        'selected_candidate_yaw': selected_candidate_yaw,
        'selection_priority_trace': selection_priority_trace,
        'rejected_candidate_summaries': rejected_candidate_summaries,
        'original_target_preserved_on_reject': original_target_preserved_on_reject,
        'multi_candidate_forward_search': multi_candidate_forward_search,
        'candidates': candidates,
        'gate_conditions': {
            'same_corridor': bool(selected and selected['same_corridor']),
            'two_side_wall_evidence': bool(selected and selected['two_side_wall_evidence']),
            'target_has_clearance': bool(selected and selected['target_has_clearance']),
            'occupancy_free': bool(selected and selected['occupancy_free']),
            'forward_progress_not_lowered': bool(selected and selected['forward_progress_not_lowered']),
            'forward_progress_not_obviously_lowered': bool(selected and selected['forward_progress_not_obviously_lowered']),
            'forward_progress_ok': bool(selected and selected['forward_progress_ok']),
            'improves_balance': bool(selected and selected['improves_balance']),
            'balance_error_improved': bool(selected and selected['balance_error_improved']),
            'improves_min_clearance': bool(selected and selected['improves_min_clearance']),
            'improves_local_cost': bool(selected and selected['improves_local_cost']),
            'improves_front_wedge': bool(selected and selected['improves_front_wedge']),
            'safety_floor_ok': bool(selected and selected['safety_floor_ok']),
            'footprint_lethal_not_increased': bool(selected and selected['footprint_lethal_not_increased']),
            'front_wedge_lethal_not_increased': bool(selected and selected['front_wedge_lethal_not_increased']),
            'hard_safety_pass': bool(selected and selected['hard_safety_pass']),
            'balance_first_eligible': bool(selected and selected['balance_first_eligible']),
        },
    }



def _empty_two_step_staging_result(
    *,
    reason: str,
    original_target: Point,
    trigger_conditions: Optional[dict[str, bool]] = None,
    source_forward_refinement: Optional[dict[str, Any]] = None,
    visual_handoff_mode: bool = False,
    dispatch_pose: Optional[Tuple[float, float, float]] = None,
    direction_rad: Optional[float] = None,
    selected_branch_geometry: Optional[dict[str, Any]] = None,
    candidate_branch_count: Optional[int] = None,
    last_open_direction_count: Optional[int] = None,
    last_candidate_count: Optional[int] = None,
) -> dict[str, Any]:
    source = _source_forward_search_payload(source_forward_refinement or {})
    triggers = trigger_conditions or {
        'near_goal_lateral_residual': False,
        'single_step_forward_search_no_hard_safety_pass': False,
        'safety_floor_dominant_blocker': False,
        'execution_time_footprint_front_wedge_risk': False,
    }
    result = {
        'enabled': False,
        'two_step_stage_dispatch_requested': False,
        'reason': reason,
        'original_target': [float(original_target[0]), float(original_target[1])],
        'refined_target': [float(original_target[0]), float(original_target[1])],
        'two_step_staging_plan': {
            'enabled': False,
            'trigger_conditions': triggers,
            'source_single_step': {
                'candidate_count': int(source.get('candidate_count', 0) or 0),
                'hard_safety_pass_candidate_count': int(source.get('hard_safety_pass_candidate_count', 0) or 0),
                'refinement_applied': bool(source.get('refinement_applied', False)),
                'refinement_reject_reason': source.get('refinement_reject_reason'),
                'original_target_preserved_on_reject': bool(source.get('original_target_preserved_on_reject', False)),
            },
            'visual_handoff_mode': bool(visual_handoff_mode),
        },
        'staging_goal_pose': None,
        'staging_reason': None,
        'staging_executability_check': {
            'checked': False,
            'hard_safety_pass': False,
            'reason': reason,
            'same_corridor': False,
            'two_side_wall_evidence': False,
            'target_has_clearance': False,
            'occupancy_free': False,
            'safety_floor_ok': False,
            'footprint_lethal_not_increased': False,
            'front_wedge_lethal_not_increased': False,
            'forward_progress_ok': False,
            'bounded_short_distance': False,
            'lateral_residual_reduced': False,
        },
        'second_step_forward_goal': None,
        'staging_applied': False,
        'staging_reject_reason': reason,
        'original_target_preserved_on_reject': True,
        'visual_diagnosis_wait_requested': bool(visual_handoff_mode),
        'branch_scoring_changed': False,
        'fallback_terminal_acceptance_used': False,
        'staging_candidates': [],
        'corridor_evidence_carry_over': {
            'evaluated': False,
            'eligible': False,
            'allowed_fields': list(ALLOWED_CORRIDOR_CARRY_OVER_FIELDS),
            'forbidden_fields': list(FORBIDDEN_CORRIDOR_CARRY_OVER_FIELDS),
        },
        'carry_over_source': None,
        'carry_over_applied': False,
        'carry_over_reject_reason': None,
        'source_forward_window': _source_forward_window_summary(source),
        'staging_window': _staging_window_summary([], max_staging_distance_m=0.0),
        'safety_evidence_recomputed': True,
    }
    result['staging_gate_artifact_completeness'] = _phase143_staging_gate_artifact_payload(
        result,
        dispatch_pose=dispatch_pose,
        direction_rad=direction_rad,
        source_forward_refinement=source_forward_refinement or {},
        selected_metrics=None,
        trigger_conditions=triggers,
        selected_branch_geometry=selected_branch_geometry,
        candidate_branch_count=candidate_branch_count,
        last_open_direction_count=last_open_direction_count,
        last_candidate_count=last_candidate_count,
    )
    return result


def plan_two_step_corridor_alignment_staging_goal(
    *,
    map_grid: OccupancyGridView,
    local_cost_grid: Optional[OccupancyGridView],
    dispatch_pose: Tuple[float, float, float],
    original_target: Point,
    direction_rad: float,
    clearance_radius_m: float,
    source_forward_refinement: dict[str, Any],
    side_probe_m: float = 1.5,
    staging_lateral_offsets_m: Sequence[float] = (0.10, 0.20, 0.30),
    staging_forward_offsets_m: Sequence[float] = (0.0, 0.05, 0.10),
    max_staging_distance_m: float = 0.35,
    local_cost_radius_m: float = 0.30,
    front_wedge_radius_m: float = 0.75,
    front_wedge_half_angle_rad: float = math.radians(35.0),
    high_cost_threshold: int = 70,
    min_clearance_floor_m: Optional[float] = None,
    near_goal_lateral_residual_min_m: float = 0.10,
    visual_handoff_mode: bool = False,
    current_goal_sequence: Optional[int] = None,
    current_branch_context_id: Optional[str] = None,
    current_timestamp_sec: Optional[float] = None,
    staging_frame_id: str = 'map',
    carry_over_source_max_age_sec: float = 30.0,
    carry_over_heading_tolerance_rad: float = math.radians(25.0),
) -> dict[str, Any]:
    """Plan a conservative corridor-alignment staging goal for Phase92.

    This evaluates only the already selected branch target. It must not alter
    branch scoring, exploration order, Nav2/controller parameters, fallback, or
    terminal acceptance semantics.
    """
    source = _source_forward_search_payload(source_forward_refinement)
    trigger_conditions = _two_step_staging_trigger_conditions(
        dispatch_pose=dispatch_pose,
        original_target=original_target,
        direction_rad=direction_rad,
        source_forward_refinement=source_forward_refinement,
        near_goal_lateral_residual_min_m=near_goal_lateral_residual_min_m,
    )
    reject_reason = _two_step_trigger_reject_reason(trigger_conditions, source)
    if reject_reason is not None:
        return _empty_two_step_staging_result(
            reason=reject_reason,
            original_target=original_target,
            trigger_conditions=trigger_conditions,
            source_forward_refinement=source_forward_refinement,
            visual_handoff_mode=visual_handoff_mode,
            dispatch_pose=dispatch_pose,
            direction_rad=direction_rad,
        )

    dispatch_xy = (float(dispatch_pose[0]), float(dispatch_pose[1]))
    forward = (math.cos(direction_rad), math.sin(direction_rad))
    lateral = (math.cos(direction_rad + math.pi / 2.0), math.sin(direction_rad + math.pi / 2.0))
    lateral_residual_before = _lateral_residual_m(dispatch_xy, original_target, direction_rad)
    residual_sign = 1.0 if lateral_residual_before >= 0.0 else -1.0
    clearance_floor = float(clearance_radius_m if min_clearance_floor_m is None else min_clearance_floor_m)
    baseline_metrics = _centerline_candidate_metrics(
        map_grid=map_grid,
        local_cost_grid=local_cost_grid,
        dispatch_xy=dispatch_xy,
        target_xy=dispatch_xy,
        direction_rad=direction_rad,
        footprint_yaw_rad=direction_rad,
        clearance_radius_m=clearance_radius_m,
        side_probe_m=side_probe_m,
        local_cost_radius_m=local_cost_radius_m,
        front_wedge_radius_m=front_wedge_radius_m,
        front_wedge_half_angle_rad=front_wedge_half_angle_rad,
        high_cost_threshold=high_cost_threshold,
    )
    normalized_lateral_offsets = [abs(float(value)) for value in staging_lateral_offsets_m if abs(float(value)) > 1e-9]
    normalized_forward_offsets = [max(0.0, float(value)) for value in staging_forward_offsets_m] or [0.0]
    candidates: list[dict[str, Any]] = []
    for lateral_offset in normalized_lateral_offsets:
        lateral_step = min(abs(lateral_residual_before), lateral_offset)
        if lateral_step <= 1e-9:
            continue
        for forward_offset in normalized_forward_offsets:
            candidate_xy = (
                dispatch_xy[0] + lateral[0] * residual_sign * lateral_step + forward[0] * forward_offset,
                dispatch_xy[1] + lateral[1] * residual_sign * lateral_step + forward[1] * forward_offset,
            )
            staging_distance = math.hypot(candidate_xy[0] - dispatch_xy[0], candidate_xy[1] - dispatch_xy[1])
            metrics = _centerline_candidate_metrics(
                map_grid=map_grid,
                local_cost_grid=local_cost_grid,
                dispatch_xy=dispatch_xy,
                target_xy=candidate_xy,
                direction_rad=direction_rad,
                footprint_yaw_rad=direction_rad,
                clearance_radius_m=clearance_radius_m,
                side_probe_m=side_probe_m,
                local_cost_radius_m=local_cost_radius_m,
                front_wedge_radius_m=front_wedge_radius_m,
                front_wedge_half_angle_rad=front_wedge_half_angle_rad,
                high_cost_threshold=high_cost_threshold,
            )
            residual_after = _lateral_residual_m(candidate_xy, original_target, direction_rad)
            metrics['candidate_index'] = len(candidates)
            metrics['target_xy'] = list(metrics['target'])
            metrics['target_yaw'] = float(normalize_angle(direction_rad))
            metrics['staging_lateral_offset_m'] = float(lateral_step * residual_sign)
            metrics['staging_forward_offset_m'] = float(forward_offset)
            metrics['staging_distance_m'] = float(staging_distance)
            metrics['bounded_short_distance'] = bool(staging_distance <= max_staging_distance_m + 1e-9)
            metrics['lateral_residual_before_m'] = float(lateral_residual_before)
            metrics['lateral_residual_after_m'] = float(residual_after)
            metrics['lateral_residual_reduced'] = bool(abs(residual_after) + 1e-9 < abs(lateral_residual_before))
            metrics['corridor_heading_yaw'] = float(normalize_angle(direction_rad))
            metrics['safety_min_clearance_floor_m'] = clearance_floor
            metrics['safety_floor_ok'] = bool(metrics['min_clearance_m'] + 1e-9 >= clearance_floor)
            metrics['footprint_lethal_not_increased'] = bool(
                int(metrics['footprint_lethal_count']) <= int(baseline_metrics['footprint_lethal_count'])
            )
            metrics['front_wedge_lethal_not_increased'] = bool(
                int(metrics['front_wedge_lethal_count']) <= int(baseline_metrics['front_wedge_lethal_count'])
            )
            metrics['forward_progress_ok'] = bool(metrics['forward_progress_m'] + 1e-9 >= 0.0)
            metrics['hard_safety_pass'] = bool(
                metrics['same_corridor']
                and metrics['two_side_wall_evidence']
                and metrics['target_has_clearance']
                and metrics['occupancy_free']
                and metrics['safety_floor_ok']
                and metrics['footprint_lethal_not_increased']
                and metrics['front_wedge_lethal_not_increased']
                and metrics['forward_progress_ok']
                and metrics['bounded_short_distance']
                and metrics['lateral_residual_reduced']
            )
            metrics['candidate_reject_reasons'] = _staging_candidate_reject_reasons(metrics)
            candidates.append(metrics)

    source_forward_window = _source_forward_window_summary(source)
    staging_window = _staging_window_summary(candidates, max_staging_distance_m=max_staging_distance_m)
    carry_over = _evaluate_corridor_evidence_carry_over(
        source=source,
        source_forward_window=source_forward_window,
        staging_window=staging_window,
        candidates=candidates,
        dispatch_pose=dispatch_pose,
        original_target=original_target,
        direction_rad=direction_rad,
        current_goal_sequence=current_goal_sequence,
        current_branch_context_id=current_branch_context_id,
        current_timestamp_sec=current_timestamp_sec,
        staging_frame_id=staging_frame_id,
        source_max_age_sec=carry_over_source_max_age_sec,
        heading_tolerance_rad=carry_over_heading_tolerance_rad,
    )
    if bool(carry_over.get('carry_over_applied', False)):
        for row in candidates:
            row['local_same_corridor'] = bool(row.get('same_corridor', False))
            row['local_two_side_wall_evidence'] = bool(row.get('two_side_wall_evidence', False))
            if not bool(row.get('same_corridor', False)):
                row['same_corridor'] = True
            if not bool(row.get('two_side_wall_evidence', False)):
                row['two_side_wall_evidence'] = True
                row['corridor_evidence_source'] = 'phase88_forward_window_carry_over'
            row['safety_evidence_recomputed'] = True
            row['hard_safety_pass'] = bool(
                row['same_corridor']
                and row['two_side_wall_evidence']
                and row['target_has_clearance']
                and row['occupancy_free']
                and row['safety_floor_ok']
                and row['footprint_lethal_not_increased']
                and row['front_wedge_lethal_not_increased']
                and row['forward_progress_ok']
                and row['bounded_short_distance']
                and row['lateral_residual_reduced']
            )
            row['candidate_reject_reasons'] = _staging_candidate_reject_reasons(row)
    else:
        for row in candidates:
            row['local_same_corridor'] = bool(row.get('same_corridor', False))
            row['local_two_side_wall_evidence'] = bool(row.get('two_side_wall_evidence', False))
            row['safety_evidence_recomputed'] = True

    selected = min(
        [row for row in candidates if row['hard_safety_pass']],
        key=lambda row: (
            abs(float(row['lateral_residual_after_m'])),
            _number_or_inf(row.get('front_wedge_cost_max')),
            int(row.get('front_wedge_high_cost_count', 0)),
            int(row.get('footprint_high_cost_count', 0)),
            float(row['staging_distance_m']),
            int(row['candidate_index']),
        ),
        default=None,
    )
    if selected is None:
        reason = _staging_family_reject_reason(candidates)
        if bool(carry_over.get('carry_over_applied', False)):
            reason = 'staging_safety_recompute_failed'
            carry_over['carry_over_reject_reason'] = 'staging_safety_recompute_failed'
        result = _empty_two_step_staging_result(
            reason=reason,
            original_target=original_target,
            trigger_conditions=trigger_conditions,
            source_forward_refinement=source_forward_refinement,
            visual_handoff_mode=visual_handoff_mode,
        )
        result['enabled'] = True
        result['two_step_staging_plan']['enabled'] = True
        result['staging_executability_check'].update(_staging_executability_check(None, reason=reason))
        best_failed: Optional[dict[str, Any]] = None
        if candidates:
            best_failed = min(
                candidates,
                key=lambda row: (
                    abs(float(row.get('lateral_residual_after_m', 0.0))),
                    float(row.get('staging_distance_m', 0.0)),
                    int(row.get('candidate_index', 0)),
                ),
            )
            failed_check = _staging_executability_check(best_failed, reason=reason)
            failed_check['hard_safety_pass'] = False
            failed_check['reason'] = reason
            result['staging_executability_check'].update(failed_check)
        result['staging_executability_check']['safety_evidence_recomputed'] = True
        result['staging_candidates'] = [_compact_staging_candidate_summary(row) for row in candidates]
        result.update(_carry_over_result_fields(carry_over, source_forward_window, staging_window))
        result['staging_gate_artifact_completeness'] = _phase143_staging_gate_artifact_payload(
            result,
            dispatch_pose=dispatch_pose,
            direction_rad=direction_rad,
            source_forward_refinement=source_forward_refinement,
            selected_metrics=best_failed if candidates else None,
            trigger_conditions=trigger_conditions,
            selected_branch_geometry=None,
            candidate_branch_count=None,
            last_open_direction_count=None,
            last_candidate_count=None,
        )
        return result

    pose = {
        'x': float(selected['target'][0]),
        'y': float(selected['target'][1]),
        'yaw': float(selected['target_yaw']),
        'lateral_residual_before_m': float(selected['lateral_residual_before_m']),
        'lateral_residual_after_m': float(selected['lateral_residual_after_m']),
        'staging_distance_m': float(selected['staging_distance_m']),
    }
    result = {
        'enabled': True,
        'two_step_stage_dispatch_requested': True,
        'reason': 'phase90_trigger_bundle_corridor_alignment_staging',
        'original_target': [float(original_target[0]), float(original_target[1])],
        'refined_target': [float(selected['target'][0]), float(selected['target'][1])],
        'two_step_staging_plan': {
            'enabled': True,
            'trigger_conditions': trigger_conditions,
            'source_single_step': {
                'candidate_count': int(source.get('candidate_count', 0) or 0),
                'hard_safety_pass_candidate_count': int(source.get('hard_safety_pass_candidate_count', 0) or 0),
                'refinement_applied': bool(source.get('refinement_applied', False)),
                'refinement_reject_reason': source.get('refinement_reject_reason'),
                'original_target_preserved_on_reject': bool(source.get('original_target_preserved_on_reject', False)),
            },
            'staging_candidate_family': {
                'short_distance': True,
                'corridor_heading': True,
                'same_corridor_required': True,
                'two_side_wall_evidence_required': True,
                'staging_lateral_offsets_m': normalized_lateral_offsets,
                'staging_forward_offsets_m': normalized_forward_offsets,
                'max_staging_distance_m': float(max_staging_distance_m),
            },
            'visual_handoff_mode': bool(visual_handoff_mode),
        },
        'staging_goal_pose': pose,
        'staging_reason': 'reduce_lateral_residual_and_front_wedge_risk_before_second_step_forward_goal',
        'staging_executability_check': _staging_executability_check(selected, reason=None),
        'second_step_forward_goal': None,
        'staging_applied': True,
        **_carry_over_result_fields(carry_over, source_forward_window, staging_window),
        'staging_reject_reason': None,
        'original_target_preserved_on_reject': False,
        'visual_diagnosis_wait_requested': False,
        'branch_scoring_changed': False,
        'fallback_terminal_acceptance_used': False,
        'staging_candidates': [_compact_staging_candidate_summary(row) for row in candidates if row is not selected],
    }
    result['staging_gate_artifact_completeness'] = _phase143_staging_gate_artifact_payload(
        result,
        dispatch_pose=dispatch_pose,
        direction_rad=direction_rad,
        source_forward_refinement=source_forward_refinement,
        selected_metrics=selected,
        trigger_conditions=trigger_conditions,
        selected_branch_geometry=None,
        candidate_branch_count=None,
        last_open_direction_count=None,
        last_candidate_count=None,
    )
    return result


def generate_second_step_forward_goal_after_staging(
    *,
    map_grid: OccupancyGridView,
    local_cost_grid: Optional[OccupancyGridView],
    staged_pose: Tuple[float, float, float],
    original_target: Point,
    direction_rad: float,
    clearance_radius_m: float,
    fresh_scan_received: bool,
    fresh_local_costmap_received: bool,
    fresh_tf_received: bool,
    side_probe_m: float = 1.5,
    forward_offsets_m: Sequence[float] = (0.0, 0.1, 0.2),
    lateral_offsets_m: Sequence[float] = (-0.3, -0.15, 0.0, 0.15, 0.3),
    heading_offsets_rad: Sequence[float] = (0.0,),
    local_cost_radius_m: float = 0.30,
    front_wedge_radius_m: float = 0.75,
    front_wedge_half_angle_rad: float = math.radians(35.0),
    high_cost_threshold: int = 70,
    min_clearance_floor_m: Optional[float] = None,
    forward_progress_tolerance_m: float = 0.0,
) -> dict[str, Any]:
    base: dict[str, Any] = {
        'generated_after_fresh_evidence': False,
        'fresh_scan_received': bool(fresh_scan_received),
        'fresh_local_costmap_received': bool(fresh_local_costmap_received),
        'fresh_tf_received': bool(fresh_tf_received),
        'selected_candidate_target': None,
        'selected_candidate_yaw': None,
        'hard_safety_pass_candidate_count': None,
        'forward_refinement_result': None,
        'reject_reason': None,
        'branch_scoring_changed': False,
        'fallback_terminal_acceptance_used': False,
    }
    if not (fresh_scan_received and fresh_local_costmap_received and fresh_tf_received):
        base['reject_reason'] = 'second_step_fresh_evidence_missing'
        return base
    refinement = refine_corridor_centerline_target(
        map_grid=map_grid,
        local_cost_grid=local_cost_grid,
        dispatch_pose=staged_pose,
        original_target=original_target,
        direction_rad=direction_rad,
        clearance_radius_m=clearance_radius_m,
        side_probe_m=side_probe_m,
        forward_offsets_m=forward_offsets_m,
        lateral_offsets_m=lateral_offsets_m,
        heading_offsets_rad=heading_offsets_rad,
        local_cost_radius_m=local_cost_radius_m,
        front_wedge_radius_m=front_wedge_radius_m,
        front_wedge_half_angle_rad=front_wedge_half_angle_rad,
        high_cost_threshold=high_cost_threshold,
        min_clearance_floor_m=min_clearance_floor_m,
        forward_progress_tolerance_m=forward_progress_tolerance_m,
    )
    base.update({
        'generated_after_fresh_evidence': bool(refinement.get('refinement_applied', False)),
        'selected_candidate_target': refinement.get('selected_candidate_target'),
        'selected_candidate_yaw': refinement.get('selected_candidate_yaw'),
        'hard_safety_pass_candidate_count': int(refinement.get('hard_safety_pass_candidate_count', 0) or 0),
        'forward_refinement_result': refinement,
        'reject_reason': None if refinement.get('refinement_applied', False) else 'second_step_no_hard_safe_forward_goal',
        'branch_scoring_changed': False,
        'fallback_terminal_acceptance_used': False,
    })
    return base


def _source_forward_search_payload(source_forward_refinement: dict[str, Any]) -> dict[str, Any]:
    nested = source_forward_refinement.get('multi_candidate_forward_search')
    if isinstance(nested, dict):
        return nested
    return source_forward_refinement


def _lateral_residual_m(dispatch_xy: Point, target_xy: Point, direction_rad: float) -> float:
    lateral = (math.cos(direction_rad + math.pi / 2.0), math.sin(direction_rad + math.pi / 2.0))
    return float((float(target_xy[0]) - float(dispatch_xy[0])) * lateral[0] + (float(target_xy[1]) - float(dispatch_xy[1])) * lateral[1])


PHASE143_STAGING_GATE_CLASSIFICATIONS = [
    'staging_not_needed_direct_explore',
    'staging_expected_but_not_triggered',
    'staging_triggered_corridor_alignment',
    'staging_triggered_but_unsafe_or_unavailable',
    'insufficient_staging_gate_evidence',
]

PHASE143_REQUIRED_STAGING_GATE_ARTIFACT_FIELDS = [
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
]

PHASE143_REQUIRED_TRIGGER_CONDITION_FIELDS = [
    'near_goal_lateral_residual',
    'single_step_forward_search_no_hard_safety_pass',
    'safety_floor_dominant_blocker',
    'execution_time_footprint_front_wedge_risk',
]


def _phase143_metric_value(metrics: Optional[dict[str, Any]], *keys: str) -> Any:
    if not isinstance(metrics, dict):
        return None
    for key in keys:
        value = metrics.get(key)
        if value is not None:
            return value
    return None


def _phase143_float_or_none(value: Any) -> Optional[float]:
    try:
        out = float(value)
    except (TypeError, ValueError):
        return None
    return out if math.isfinite(out) else None


def _phase143_int_or_none(value: Any) -> Optional[int]:
    if isinstance(value, bool):
        return None
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def _phase143_xy_payload(value: Any) -> Optional[list[float]]:
    if isinstance(value, (list, tuple)) and len(value) >= 2:
        try:
            return [float(value[0]), float(value[1])]
        except (TypeError, ValueError):
            return None
    return None


def _phase143_source_rows(source: dict[str, Any]) -> list[dict[str, Any]]:
    rows = source.get('candidates')
    if isinstance(rows, list):
        return [dict(row) for row in rows if isinstance(row, dict)]
    summaries = source.get('rejected_candidate_summaries')
    if isinstance(summaries, list):
        return [dict(row) for row in summaries if isinstance(row, dict)]
    return []


def _phase143_compact_candidate_summary(metrics: dict[str, Any]) -> dict[str, Any]:
    return {
        'candidate_index': metrics.get('candidate_index'),
        'target_xy': _phase143_xy_payload(metrics.get('target_xy', metrics.get('target'))),
        'yaw': _phase143_float_or_none(metrics.get('target_yaw', metrics.get('yaw'))),
        'lateral_residual_after_m': _phase143_float_or_none(metrics.get('lateral_residual_after_m')),
        'target_local_cost': _phase143_int_or_none(
            _phase143_metric_value(metrics, 'target_local_cost', 'local_cost_center', 'local_cost_max_radius')
        ),
        'target_local_cost_max_radius': _phase143_int_or_none(
            _phase143_metric_value(metrics, 'target_local_cost_max_radius', 'local_cost_max_radius')
        ),
        'path_corridor_min_clearance_m': _phase143_float_or_none(
            _phase143_metric_value(metrics, 'path_corridor_min_clearance_m', 'min_clearance_m')
        ),
        'target_clearance_m': _phase143_float_or_none(
            _phase143_metric_value(metrics, 'target_clearance_m', 'nearest_obstacle_distance_m', 'min_clearance_m')
        ),
        'front_wedge_cost_max': _phase143_int_or_none(metrics.get('front_wedge_cost_max')),
        'footprint_lethal_count': _phase143_int_or_none(metrics.get('footprint_lethal_count')),
        'front_wedge_lethal_count': _phase143_int_or_none(metrics.get('front_wedge_lethal_count')),
        'same_corridor': bool(metrics.get('same_corridor', False)),
        'two_side_wall_evidence': bool(metrics.get('two_side_wall_evidence', False)),
        'hard_safety_pass': bool(metrics.get('hard_safety_pass', False)),
    }


def _phase143_source_single_step_payload(source_forward_refinement: dict[str, Any]) -> dict[str, Any]:
    source = _source_forward_search_payload(source_forward_refinement or {})
    rows = _phase143_source_rows(source)
    hard_safe = [row for row in rows if bool(row.get('hard_safety_pass', False))]
    explicit_hard_safe = source.get('hard_safe_candidate_summaries')
    if isinstance(explicit_hard_safe, list) and explicit_hard_safe:
        hard_safe_summaries = [dict(row) for row in explicit_hard_safe if isinstance(row, dict)]
    else:
        hard_safe_summaries = [_phase143_compact_candidate_summary(row) for row in hard_safe[:5]]
    rejected = source.get('rejected_candidate_summaries')
    rejected_rows = [dict(row) for row in rejected if isinstance(row, dict)] if isinstance(rejected, list) else []
    return {
        'candidate_count': int(source.get('candidate_count', len(rows)) or 0),
        'hard_safety_pass_candidate_count': int(source.get('hard_safety_pass_candidate_count', len(hard_safe)) or 0),
        'refinement_applied': bool(source.get('refinement_applied', False)),
        'refinement_reject_reason': source.get('refinement_reject_reason'),
        'original_target_preserved_on_reject': bool(source.get('original_target_preserved_on_reject', False)),
        'selected_candidate_index': source.get('selected_candidate_index'),
        'selected_candidate_target': _phase143_xy_payload(source.get('selected_candidate_target')),
        'selected_candidate_yaw': _phase143_float_or_none(source.get('selected_candidate_yaw')),
        'hard_safe_candidate_summaries': hard_safe_summaries,
        'rejected_candidate_summaries': rejected_rows[:5],
        'selection_priority_trace': list(source.get('selection_priority_trace') or []),
    }


def _phase143_selected_source_metrics(source_forward_refinement: dict[str, Any]) -> Optional[dict[str, Any]]:
    source = _source_forward_search_payload(source_forward_refinement or {})
    selected = source.get('selected_metrics')
    if isinstance(selected, dict):
        return selected
    rows = _phase143_source_rows(source)
    for row in rows:
        if bool(row.get('hard_safety_pass', False)):
            return row
    if rows:
        return rows[0]
    return None


def _phase143_selected_branch_geometry(
    *,
    selected_branch_geometry: Optional[dict[str, Any]],
    selected_metrics: Optional[dict[str, Any]],
    original_target: Any,
    branch_angle: Optional[float],
) -> dict[str, Any]:
    geometry = dict(selected_branch_geometry or {})
    target_xy = _phase143_xy_payload(
        geometry.get('target_xy', geometry.get('target', _phase143_metric_value(selected_metrics, 'target_xy', 'target')))
    ) or _phase143_xy_payload(original_target)
    geometry.setdefault('branch_angle', branch_angle)
    geometry.setdefault('target_xy', target_xy)
    geometry.setdefault('chosen_branch_rank', geometry.get('rank'))
    geometry.setdefault('exit_progress_delta_m', geometry.get('exit_progress_delta_m'))
    geometry.setdefault('target_exit_dist', geometry.get('target_exit_dist'))
    geometry.setdefault('edge_id', geometry.get('edge_id'))
    geometry.setdefault('state', geometry.get('state'))
    return geometry


def _phase143_gate_artifact_missing_fields(artifact: dict[str, Any]) -> list[str]:
    missing: list[str] = []
    for field in PHASE143_REQUIRED_STAGING_GATE_ARTIFACT_FIELDS:
        if field not in artifact:
            missing.append(field)
    source_single_step = artifact.get('source_single_step') if isinstance(artifact.get('source_single_step'), dict) else {}
    for field in ['candidate_count', 'hard_safety_pass_candidate_count', 'hard_safe_candidate_summaries']:
        if field not in source_single_step:
            missing.append(f'source_single_step.{field}')
    trigger_conditions = artifact.get('trigger_conditions') if isinstance(artifact.get('trigger_conditions'), dict) else {}
    for field in PHASE143_REQUIRED_TRIGGER_CONDITION_FIELDS:
        if field not in trigger_conditions:
            missing.append(f'trigger_conditions.{field}')
    direct_hard_safe = (
        artifact.get('staging_reject_reason') == 'single_step_forward_search_had_hard_safe_candidate'
        and int(source_single_step.get('hard_safety_pass_candidate_count', 0) or 0) > 0
    )
    if direct_hard_safe and not source_single_step.get('hard_safe_candidate_summaries'):
        missing.append('source_single_step.hard_safe_candidate_summaries')
    return missing


def _phase143_gate_classification(artifact: dict[str, Any], missing_fields: list[str]) -> str:
    if missing_fields:
        return 'insufficient_staging_gate_evidence'
    source_single_step = artifact.get('source_single_step') if isinstance(artifact.get('source_single_step'), dict) else {}
    triggers = artifact.get('trigger_conditions') if isinstance(artifact.get('trigger_conditions'), dict) else {}
    hard_count = int(source_single_step.get('hard_safety_pass_candidate_count', 0) or 0)
    staging_applied = bool(artifact.get('staging_applied', False))
    stage_requested = bool(artifact.get('two_step_stage_dispatch_requested', False))
    reject_reason = artifact.get('staging_reject_reason')
    if staging_applied and stage_requested:
        return 'staging_triggered_corridor_alignment'
    if reject_reason == 'single_step_forward_search_had_hard_safe_candidate' and hard_count > 0:
        return 'staging_not_needed_direct_explore'
    if bool(triggers) and all(bool(triggers.get(field, False)) for field in PHASE143_REQUIRED_TRIGGER_CONDITION_FIELDS):
        if not staging_applied and (stage_requested or str(reject_reason or '').startswith('staging_')):
            return 'staging_triggered_but_unsafe_or_unavailable'
        return 'staging_expected_but_not_triggered'
    return 'insufficient_staging_gate_evidence'


def _phase143_staging_gate_artifact_payload(
    result: dict[str, Any],
    *,
    dispatch_pose: Optional[Tuple[float, float, float]],
    direction_rad: Optional[float],
    source_forward_refinement: dict[str, Any],
    selected_metrics: Optional[dict[str, Any]],
    trigger_conditions: dict[str, bool],
    selected_branch_geometry: Optional[dict[str, Any]],
    candidate_branch_count: Optional[int],
    last_open_direction_count: Optional[int],
    last_candidate_count: Optional[int],
) -> dict[str, Any]:
    source_single_step = _phase143_source_single_step_payload(source_forward_refinement)
    source_metrics = selected_metrics if isinstance(selected_metrics, dict) else _phase143_selected_source_metrics(source_forward_refinement)
    branch_angle = _phase143_float_or_none(direction_rad)
    lateral_before = _phase143_float_or_none(_phase143_metric_value(source_metrics, 'lateral_residual_before_m'))
    lateral_after = _phase143_float_or_none(_phase143_metric_value(source_metrics, 'lateral_residual_after_m'))
    if lateral_before is None and dispatch_pose is not None and direction_rad is not None:
        lateral_before = abs(_lateral_residual_m((dispatch_pose[0], dispatch_pose[1]), result.get('original_target', (0.0, 0.0)), direction_rad))
    if lateral_after is None:
        pose = result.get('staging_goal_pose')
        if isinstance(pose, dict):
            lateral_after = _phase143_float_or_none(pose.get('lateral_residual_after_m'))
    target_local_cost = _phase143_int_or_none(
        _phase143_metric_value(source_metrics, 'target_local_cost', 'local_cost_center', 'local_cost_max_radius')
    )
    target_local_cost_max_radius = _phase143_int_or_none(
        _phase143_metric_value(source_metrics, 'target_local_cost_max_radius', 'local_cost_max_radius')
    )
    path_corridor_min_clearance = _phase143_float_or_none(
        _phase143_metric_value(source_metrics, 'path_corridor_min_clearance_m', 'min_clearance_m')
    )
    target_clearance = _phase143_float_or_none(
        _phase143_metric_value(source_metrics, 'target_clearance_m', 'nearest_obstacle_distance_m', 'min_clearance_m')
    )
    inferred_candidate_count = source_single_step.get('candidate_count')
    artifact = {
        'lateral_residual_before_m': lateral_before,
        'lateral_residual_after_m': lateral_after,
        'lateral_residual_source': 'staging_goal_pose' if result.get('staging_applied') else 'source_single_step_selected_metrics',
        'lateral_residual_available': bool(lateral_before is not None and lateral_after is not None),
        'target_local_cost': target_local_cost,
        'target_local_cost_max_radius': target_local_cost_max_radius,
        'target_local_cost_source': 'selected_candidate_metrics',
        'target_in_local_costmap_bounds': None,
        'path_corridor_min_clearance_m': path_corridor_min_clearance,
        'target_clearance_m': target_clearance,
        'clearance_source': 'selected_candidate_metrics',
        'front_wedge_risk': {
            'max': _phase143_int_or_none(_phase143_metric_value(source_metrics, 'front_wedge_cost_max')),
            'mean': _phase143_float_or_none(_phase143_metric_value(source_metrics, 'front_wedge_cost_mean')),
            'high_cost_count': _phase143_int_or_none(_phase143_metric_value(source_metrics, 'front_wedge_high_cost_count')),
            'lethal_count': _phase143_int_or_none(_phase143_metric_value(source_metrics, 'front_wedge_lethal_count')),
            'sample_count': _phase143_int_or_none(_phase143_metric_value(source_metrics, 'front_wedge_sample_count')),
        },
        'candidate_branch_count': int(candidate_branch_count if candidate_branch_count is not None else inferred_candidate_count or 0),
        'last_open_direction_count': int(last_open_direction_count if last_open_direction_count is not None else inferred_candidate_count or 0),
        'last_candidate_count': int(last_candidate_count if last_candidate_count is not None else inferred_candidate_count or 0),
        'branch_angle': branch_angle,
        'selected_branch_geometry': _phase143_selected_branch_geometry(
            selected_branch_geometry=selected_branch_geometry,
            selected_metrics=source_metrics,
            original_target=result.get('original_target'),
            branch_angle=branch_angle,
        ),
        'two_step_stage_dispatch_requested': bool(result.get('two_step_stage_dispatch_requested', False)),
        'staging_applied': bool(result.get('staging_applied', False)),
        'source_single_step': source_single_step,
        'trigger_conditions': {field: bool(trigger_conditions.get(field, False)) for field in PHASE143_REQUIRED_TRIGGER_CONDITION_FIELDS},
        'trigger_evidence': {
            'near_goal_lateral_residual': {'value_m': lateral_before},
            'single_step_forward_search_no_hard_safety_pass': {
                'hard_safety_pass_candidate_count': source_single_step.get('hard_safety_pass_candidate_count'),
            },
            'safety_floor_dominant_blocker': {
                'path_corridor_min_clearance_m': path_corridor_min_clearance,
                'target_clearance_m': target_clearance,
            },
            'execution_time_footprint_front_wedge_risk': {
                'max': _phase143_int_or_none(_phase143_metric_value(source_metrics, 'front_wedge_cost_max')),
                'lethal_count': _phase143_int_or_none(_phase143_metric_value(source_metrics, 'front_wedge_lethal_count')),
            },
        },
        'staging_reason': result.get('staging_reason'),
        'staging_reject_reason': result.get('staging_reject_reason'),
        'branch_scoring_changed': False,
        'fallback_terminal_acceptance_used': False,
    }
    missing_fields = _phase143_gate_artifact_missing_fields(artifact)
    artifact['gate_artifact_missing_fields'] = missing_fields
    artifact['gate_artifact_complete'] = not missing_fields
    artifact['gate_classification_candidate'] = _phase143_gate_classification(artifact, missing_fields)
    return artifact


def _two_step_staging_trigger_conditions(
    *,
    dispatch_pose: Tuple[float, float, float],
    original_target: Point,
    direction_rad: float,
    source_forward_refinement: dict[str, Any],
    near_goal_lateral_residual_min_m: float,
) -> dict[str, bool]:
    source = _source_forward_search_payload(source_forward_refinement)
    rejected = list(source.get('rejected_candidate_summaries') or source_forward_refinement.get('rejected_candidate_summaries') or [])
    candidate_count = int(source.get('candidate_count', source_forward_refinement.get('candidate_count', 0)) or 0)
    hard_count = int(source.get('hard_safety_pass_candidate_count', source_forward_refinement.get('hard_safety_pass_candidate_count', 0)) or 0)
    refinement_applied = bool(source.get('refinement_applied', source_forward_refinement.get('refinement_applied', False)))
    original_preserved = bool(source.get('original_target_preserved_on_reject', source_forward_refinement.get('original_target_preserved_on_reject', False)))
    safety_floor_fail_count = sum(1 for row in rejected if not bool(row.get('safety_floor_ok', False)))
    same_corridor_supported = bool(rejected) and all(bool(row.get('same_corridor', False)) for row in rejected)
    two_side_supported = bool(rejected) and all(bool(row.get('two_side_wall_evidence', False)) for row in rejected)
    occupancy_not_primary = not rejected or all(bool(row.get('occupancy_free', True)) for row in rejected)
    forward_not_primary = not rejected or all(bool(row.get('forward_progress_ok', True)) for row in rejected)
    footprint_or_front_wedge_risk = any(
        (not bool(row.get('footprint_lethal_not_increased', True)))
        or (not bool(row.get('front_wedge_lethal_not_increased', True)))
        for row in rejected
    ) or bool(source_forward_refinement.get('execution_time_footprint_front_wedge_risk', False))
    lateral_residual = abs(_lateral_residual_m((dispatch_pose[0], dispatch_pose[1]), original_target, direction_rad))
    return {
        'near_goal_lateral_residual': bool(lateral_residual >= near_goal_lateral_residual_min_m),
        'single_step_forward_search_no_hard_safety_pass': bool(
            candidate_count > 0 and hard_count == 0 and not refinement_applied and original_preserved
        ),
        'safety_floor_dominant_blocker': bool(
            bool(rejected)
            and safety_floor_fail_count >= max(1, math.ceil(len(rejected) * 0.5))
            and same_corridor_supported
            and two_side_supported
            and occupancy_not_primary
            and forward_not_primary
        ),
        'execution_time_footprint_front_wedge_risk': bool(footprint_or_front_wedge_risk),
    }


def _two_step_trigger_reject_reason(trigger_conditions: dict[str, bool], source: dict[str, Any]) -> Optional[str]:
    if all(trigger_conditions.values()):
        return None
    if not trigger_conditions.get('near_goal_lateral_residual', False):
        return 'missing_near_goal_lateral_residual_evidence'
    if int(source.get('hard_safety_pass_candidate_count', 0) or 0) > 0:
        return 'single_step_forward_search_had_hard_safe_candidate'
    if int(source.get('candidate_count', 0) or 0) <= 0:
        return 'single_step_forward_search_not_attempted'
    return 'trigger_bundle_not_satisfied'


def _staging_candidate_reject_reasons(metrics: dict[str, Any]) -> list[str]:
    reasons: list[str] = []
    for key, reason in [
        ('same_corridor', 'missing_same_corridor_evidence'),
        ('two_side_wall_evidence', 'missing_two_side_wall_evidence'),
        ('target_has_clearance', 'staging_target_clearance_blocked'),
        ('occupancy_free', 'staging_target_occupied'),
        ('safety_floor_ok', 'staging_safety_floor_blocked'),
        ('footprint_lethal_not_increased', 'staging_footprint_lethal_regression'),
        ('front_wedge_lethal_not_increased', 'staging_front_wedge_lethal_regression'),
        ('forward_progress_ok', 'staging_forward_progress_regression'),
        ('bounded_short_distance', 'staging_not_bounded_short_distance'),
        ('lateral_residual_reduced', 'staging_lateral_residual_not_reduced'),
    ]:
        if not bool(metrics.get(key, False)):
            reasons.append(reason)
    return reasons


def _staging_family_reject_reason(candidates: list[dict[str, Any]]) -> str:
    if not candidates:
        return 'staging_candidate_family_empty'
    all_reasons = [reason for row in candidates for reason in row.get('candidate_reject_reasons', [])]
    for preferred in [
        'staging_safety_floor_blocked',
        'staging_footprint_lethal_regression',
        'staging_front_wedge_lethal_regression',
        'missing_same_corridor_evidence',
        'missing_two_side_wall_evidence',
        'staging_not_bounded_short_distance',
    ]:
        if preferred in all_reasons:
            return preferred
    return 'all_staging_candidates_failed_hard_safety'


def _staging_executability_check(metrics: Optional[dict[str, Any]], *, reason: Optional[str]) -> dict[str, Any]:
    if metrics is None:
        return {
            'checked': True,
            'hard_safety_pass': False,
            'reason': reason,
            'same_corridor': False,
            'two_side_wall_evidence': False,
            'target_has_clearance': False,
            'occupancy_free': False,
            'safety_floor_ok': False,
            'footprint_lethal_not_increased': False,
            'front_wedge_lethal_not_increased': False,
            'forward_progress_ok': False,
            'bounded_short_distance': False,
            'lateral_residual_reduced': False,
            'local_costmap_stamp_age_sec': None,
            'tf_stamp_age_sec': None,
        }
    return {
        'checked': True,
        'hard_safety_pass': bool(metrics.get('hard_safety_pass', False)),
        'reason': reason,
        'same_corridor': bool(metrics.get('same_corridor', False)),
        'two_side_wall_evidence': bool(metrics.get('two_side_wall_evidence', False)),
        'local_same_corridor': bool(metrics.get('local_same_corridor', metrics.get('same_corridor', False))),
        'local_two_side_wall_evidence': bool(metrics.get('local_two_side_wall_evidence', metrics.get('two_side_wall_evidence', False))),
        'corridor_evidence_source': metrics.get('corridor_evidence_source', 'local_staging_window'),
        'safety_evidence_recomputed': bool(metrics.get('safety_evidence_recomputed', True)),
        'target_has_clearance': bool(metrics.get('target_has_clearance', False)),
        'occupancy_free': bool(metrics.get('occupancy_free', False)),
        'safety_floor_ok': bool(metrics.get('safety_floor_ok', False)),
        'footprint_lethal_not_increased': bool(metrics.get('footprint_lethal_not_increased', False)),
        'front_wedge_lethal_not_increased': bool(metrics.get('front_wedge_lethal_not_increased', False)),
        'forward_progress_ok': bool(metrics.get('forward_progress_ok', False)),
        'bounded_short_distance': bool(metrics.get('bounded_short_distance', False)),
        'lateral_residual_reduced': bool(metrics.get('lateral_residual_reduced', False)),
        'local_cost_sample_count': int(metrics.get('local_cost_sample_count', 0)),
        'front_wedge_sample_count': int(metrics.get('front_wedge_sample_count', 0)),
        'local_costmap_stamp_age_sec': metrics.get('local_costmap_stamp_age_sec'),
        'tf_stamp_age_sec': metrics.get('tf_stamp_age_sec'),
    }


def _compact_staging_candidate_summary(metrics: dict[str, Any]) -> dict[str, Any]:
    return {
        'candidate_index': metrics.get('candidate_index'),
        'target_xy': metrics.get('target_xy', metrics.get('target')),
        'target_yaw': metrics.get('target_yaw'),
        'staging_lateral_offset_m': metrics.get('staging_lateral_offset_m'),
        'staging_forward_offset_m': metrics.get('staging_forward_offset_m'),
        'staging_distance_m': metrics.get('staging_distance_m'),
        'lateral_residual_before_m': metrics.get('lateral_residual_before_m'),
        'lateral_residual_after_m': metrics.get('lateral_residual_after_m'),
        'hard_safety_pass': bool(metrics.get('hard_safety_pass', False)),
        'same_corridor': bool(metrics.get('same_corridor', False)),
        'two_side_wall_evidence': bool(metrics.get('two_side_wall_evidence', False)),
        'local_same_corridor': bool(metrics.get('local_same_corridor', metrics.get('same_corridor', False))),
        'local_two_side_wall_evidence': bool(metrics.get('local_two_side_wall_evidence', metrics.get('two_side_wall_evidence', False))),
        'corridor_evidence_source': metrics.get('corridor_evidence_source', 'local_staging_window'),
        'safety_evidence_recomputed': bool(metrics.get('safety_evidence_recomputed', True)),
        'safety_floor_ok': bool(metrics.get('safety_floor_ok', False)),
        'footprint_lethal_not_increased': bool(metrics.get('footprint_lethal_not_increased', False)),
        'front_wedge_lethal_not_increased': bool(metrics.get('front_wedge_lethal_not_increased', False)),
        'forward_progress_ok': bool(metrics.get('forward_progress_ok', False)),
        'bounded_short_distance': bool(metrics.get('bounded_short_distance', False)),
        'candidate_reject_reasons': list(metrics.get('candidate_reject_reasons', [])),
    }


ALLOWED_CORRIDOR_CARRY_OVER_FIELDS = [
    'same_corridor',
    'two_side_wall_evidence',
    'corridor_heading',
    'wall_clearance_context',
    'source_forward_window',
]
FORBIDDEN_CORRIDOR_CARRY_OVER_FIELDS = [
    'hard_safety_pass',
    'safety_floor_ok',
    'occupancy_free',
    'target_has_clearance',
    'footprint_lethal_not_increased',
    'front_wedge_lethal_not_increased',
    'fresh_scan_local_costmap_tf',
]


def _carry_over_result_fields(
    carry_over: dict[str, Any],
    source_forward_window: dict[str, Any],
    staging_window: dict[str, Any],
) -> dict[str, Any]:
    return {
        'corridor_evidence_carry_over': carry_over.get('corridor_evidence_carry_over', {
            'evaluated': False,
            'eligible': False,
            'allowed_fields': list(ALLOWED_CORRIDOR_CARRY_OVER_FIELDS),
            'forbidden_fields': list(FORBIDDEN_CORRIDOR_CARRY_OVER_FIELDS),
        }),
        'carry_over_source': carry_over.get('carry_over_source'),
        'carry_over_applied': bool(carry_over.get('carry_over_applied', False)),
        'carry_over_reject_reason': carry_over.get('carry_over_reject_reason'),
        'source_forward_window': source_forward_window,
        'staging_window': staging_window,
        'safety_evidence_recomputed': bool(carry_over.get('safety_evidence_recomputed', True)),
        'branch_scoring_changed': False,
        'fallback_terminal_acceptance_used': False,
    }


def _phase101_optional_float(value: Any) -> Optional[float]:
    try:
        out = float(value)
    except (TypeError, ValueError):
        return None
    return out if math.isfinite(out) else None


def _xy_from_row(row: dict[str, Any]) -> Optional[list[float]]:
    target = row.get('target_xy', row.get('target'))
    if isinstance(target, (list, tuple)) and len(target) >= 2:
        try:
            return [float(target[0]), float(target[1])]
        except (TypeError, ValueError):
            return None
    return None


def _range_from_points(points: list[list[float]], index: int) -> dict[str, Optional[float]]:
    if not points:
        return {'min': None, 'max': None}
    values = [float(point[index]) for point in points]
    return {'min': float(min(values)), 'max': float(max(values))}


def _wall_clearance_context_from_rows(rows: list[dict[str, Any]], original_metrics: dict[str, Any]) -> dict[str, Any]:
    source_row = original_metrics if original_metrics else (rows[0] if rows else {})
    return {
        'left_wall_clearance_m': source_row.get('left_wall_clearance_m'),
        'right_wall_clearance_m': source_row.get('right_wall_clearance_m'),
        'left_wall_hit': source_row.get('left_wall_hit'),
        'right_wall_hit': source_row.get('right_wall_hit'),
    }


def _source_forward_window_summary(source: dict[str, Any]) -> dict[str, Any]:
    explicit = source.get('source_forward_window')
    if isinstance(explicit, dict):
        return explicit
    rows = [row for row in list(source.get('rejected_candidate_summaries') or source.get('candidates') or []) if isinstance(row, dict)]
    original_metrics = source.get('original_metrics') if isinstance(source.get('original_metrics'), dict) else {}
    points = [_xy_from_row(row) for row in rows]
    points = [point for point in points if point is not None]
    original_target = _xy_from_row(original_metrics) if original_metrics else None
    if original_target is not None:
        points.append(original_target)
    same_count = sum(1 for row in rows if bool(row.get('same_corridor', False)))
    two_side_count = sum(1 for row in rows if bool(row.get('two_side_wall_evidence', False)))
    if bool(original_metrics.get('same_corridor', False)):
        same_count += 1
    if bool(original_metrics.get('two_side_wall_evidence', False)):
        two_side_count += 1
    return {
        'target_xy': original_target,
        'candidate_count': int(source.get('candidate_count', len(rows)) or 0),
        'y_range': _range_from_points(points, 1),
        'x_range': _range_from_points(points, 0),
        'corridor_heading': source.get('corridor_heading_yaw', source.get('selected_candidate_yaw')),
        'same_corridor_count': int(same_count),
        'two_side_wall_count': int(two_side_count),
        'wall_clearance_context': _wall_clearance_context_from_rows(rows, original_metrics),
    }


def _staging_window_summary(candidates: list[dict[str, Any]], *, max_staging_distance_m: float) -> dict[str, Any]:
    points = [_xy_from_row(row) for row in candidates]
    points = [point for point in points if point is not None]
    return {
        'candidate_count': len(candidates),
        'y_range': _range_from_points(points, 1),
        'x_range': _range_from_points(points, 0),
        'max_staging_distance_m': float(max_staging_distance_m),
        'two_side_wall_count': int(sum(1 for row in candidates if bool(row.get('two_side_wall_evidence', False)))),
        'same_corridor_count': int(sum(1 for row in candidates if bool(row.get('same_corridor', False)))),
    }


def _source_value(source: dict[str, Any], key: str) -> Any:
    if key in source:
        return source.get(key)
    nested = source.get('multi_candidate_forward_search')
    if isinstance(nested, dict):
        return nested.get(key)
    return None


def _geometry_coherent_with_source_corridor(
    *,
    dispatch_pose: Tuple[float, float, float],
    original_target: Point,
    direction_rad: float,
    candidates: list[dict[str, Any]],
) -> bool:
    dispatch_xy = (float(dispatch_pose[0]), float(dispatch_pose[1]))
    target_progress = ((float(original_target[0]) - dispatch_xy[0]) * math.cos(direction_rad)
                       + (float(original_target[1]) - dispatch_xy[1]) * math.sin(direction_rad))
    if target_progress <= 1e-9:
        return False
    for row in candidates:
        point = _xy_from_row(row)
        if point is None:
            continue
        progress = ((point[0] - dispatch_xy[0]) * math.cos(direction_rad)
                    + (point[1] - dispatch_xy[1]) * math.sin(direction_rad))
        if progress < -1e-9 or progress > target_progress + 1e-9:
            return False
        if not bool(row.get('same_corridor', False)):
            return False
    return True


def _evaluate_corridor_evidence_carry_over(
    *,
    source: dict[str, Any],
    source_forward_window: dict[str, Any],
    staging_window: dict[str, Any],
    candidates: list[dict[str, Any]],
    dispatch_pose: Tuple[float, float, float],
    original_target: Point,
    direction_rad: float,
    current_goal_sequence: Optional[int],
    current_branch_context_id: Optional[str],
    current_timestamp_sec: Optional[float],
    staging_frame_id: str,
    source_max_age_sec: float,
    heading_tolerance_rad: float,
) -> dict[str, Any]:
    source_heading = _phase101_optional_float(
        source_forward_window.get('corridor_heading') if isinstance(source_forward_window, dict) else None
    )
    if source_heading is None:
        source_heading = _phase101_optional_float(source.get('corridor_heading_yaw', source.get('selected_candidate_yaw')))
    carry_source = {
        'goal_sequence': _source_value(source, 'goal_sequence'),
        'source_event_type': _source_value(source, 'source_event_type'),
        'source_timestamp': _source_value(source, 'source_timestamp'),
        'frame_id': _source_value(source, 'frame_id') or 'map',
        'same_corridor': bool((source_forward_window or {}).get('same_corridor_count') or 0),
        'two_side_wall_evidence': bool((source_forward_window or {}).get('two_side_wall_count') or 0),
        'corridor_heading': source_heading,
        'wall_clearance_context': (source_forward_window or {}).get('wall_clearance_context'),
        'source_forward_window': source_forward_window,
    }
    base = {
        'corridor_evidence_carry_over': {
            'evaluated': bool(staging_window.get('candidate_count', 0) and staging_window.get('two_side_wall_count', 0) == 0),
            'eligible': False,
            'allowed_fields': list(ALLOWED_CORRIDOR_CARRY_OVER_FIELDS),
            'forbidden_fields': list(FORBIDDEN_CORRIDOR_CARRY_OVER_FIELDS),
        },
        'carry_over_source': carry_source,
        'carry_over_applied': False,
        'carry_over_reject_reason': None,
        'safety_evidence_recomputed': True,
    }
    if not base['corridor_evidence_carry_over']['evaluated']:
        base['carry_over_reject_reason'] = None
        return base

    source_candidate_count = int((source_forward_window or {}).get('candidate_count') or 0)
    x_range = source_forward_window.get('x_range') if isinstance(source_forward_window.get('x_range'), dict) else None
    y_range = source_forward_window.get('y_range') if isinstance(source_forward_window.get('y_range'), dict) else None
    source_window_has_bounds = bool(
        x_range is not None
        and y_range is not None
        and x_range.get('min') is not None
        and x_range.get('max') is not None
        and y_range.get('min') is not None
        and y_range.get('max') is not None
    )
    if source_candidate_count <= 0:
        if int(source_forward_window.get('same_corridor_count') or 0) <= 0 or int(source_forward_window.get('two_side_wall_count') or 0) <= 0:
            base['carry_over_reject_reason'] = 'source_missing_same_corridor_or_two_side_wall'
            return base
        base['carry_over_reject_reason'] = 'forward_window_not_trustworthy'
        return base
    if not source_window_has_bounds:
        base['carry_over_reject_reason'] = 'forward_window_not_trustworthy'
        return base
    if int(source_forward_window.get('same_corridor_count') or 0) <= 0 or int(source_forward_window.get('two_side_wall_count') or 0) <= 0:
        base['carry_over_reject_reason'] = 'source_missing_same_corridor_or_two_side_wall'
        return base
    source_seq = _source_value(source, 'goal_sequence')
    if current_goal_sequence is not None and source_seq is not None and int(source_seq) != int(current_goal_sequence):
        base['carry_over_reject_reason'] = 'source_branch_or_goal_sequence_mismatch'
        return base
    source_branch = _source_value(source, 'branch_context_id')
    if current_branch_context_id is not None and source_branch is not None and str(source_branch) != str(current_branch_context_id):
        base['carry_over_reject_reason'] = 'source_branch_or_goal_sequence_mismatch'
        return base
    source_frame = str(_source_value(source, 'frame_id') or 'map')
    if source_frame != str(staging_frame_id or 'map'):
        base['carry_over_reject_reason'] = 'frame_mismatch'
        return base
    source_timestamp = _phase101_optional_float(_source_value(source, 'source_timestamp'))
    now = _phase101_optional_float(current_timestamp_sec)
    if source_timestamp is not None and now is not None and now - source_timestamp > max(0.0, float(source_max_age_sec)):
        base['carry_over_reject_reason'] = 'carry_over_source_stale'
        return base
    if source_heading is None:
        base['carry_over_reject_reason'] = 'insufficient_carry_over_evidence'
        return base
    if abs(normalize_angle(float(source_heading) - float(direction_rad))) > max(0.0, float(heading_tolerance_rad)):
        base['carry_over_reject_reason'] = 'heading_mismatch'
        return base
    if not _geometry_coherent_with_source_corridor(
        dispatch_pose=dispatch_pose,
        original_target=original_target,
        direction_rad=direction_rad,
        candidates=candidates,
    ):
        base['carry_over_reject_reason'] = 'staging_not_consistent_with_source_corridor'
        return base
    if not carry_source.get('wall_clearance_context'):
        base['carry_over_reject_reason'] = 'insufficient_carry_over_evidence'
        return base

    base['corridor_evidence_carry_over']['eligible'] = True
    base['carry_over_applied'] = True
    base['carry_over_reject_reason'] = None
    return base


def _centerline_candidate_metrics(
    *,
    map_grid: OccupancyGridView,
    local_cost_grid: Optional[OccupancyGridView],
    dispatch_xy: Point,
    target_xy: Point,
    direction_rad: float,
    clearance_radius_m: float,
    side_probe_m: float,
    local_cost_radius_m: float,
    front_wedge_radius_m: float,
    front_wedge_half_angle_rad: float,
    high_cost_threshold: int,
    footprint_yaw_rad: Optional[float] = None,
) -> dict[str, Any]:
    lateral_angle = direction_rad + math.pi / 2.0
    left_clearance, left_hit = _ray_clearance_to_blocked(
        map_grid, target_xy, lateral_angle, side_probe_m, clearance_radius_m
    )
    right_clearance, right_hit = _ray_clearance_to_blocked(
        map_grid, target_xy, lateral_angle + math.pi, side_probe_m, clearance_radius_m
    )
    nearest_clearance = map_grid.nearest_obstacle_distance(
        float(target_xy[0]), float(target_xy[1]), max_radius_m=max(side_probe_m, clearance_radius_m * 3.0)
    )
    map_cell_value = map_grid.cell_value(map_grid.world_to_cell(float(target_xy[0]), float(target_xy[1])))
    local_values = _grid_values_in_radius(local_cost_grid, target_xy, local_cost_radius_m)
    footprint_yaw = float(direction_rad if footprint_yaw_rad is None else footprint_yaw_rad)
    wedge_values = _grid_values_in_wedge(
        local_cost_grid, target_xy, footprint_yaw, front_wedge_radius_m, front_wedge_half_angle_rad
    )
    progress = ((float(target_xy[0]) - float(dispatch_xy[0])) * math.cos(direction_rad)
                + (float(target_xy[1]) - float(dispatch_xy[1])) * math.sin(direction_rad))
    return {
        'target': [float(target_xy[0]), float(target_xy[1])],
        'same_corridor': _line_is_free_known(map_grid, dispatch_xy, target_xy),
        'occupancy_cell_value': int(map_cell_value),
        'occupancy_free': bool(0 <= int(map_cell_value) < map_grid.occupied_threshold),
        'target_has_clearance': bool(map_grid.world_point_has_clearance(target_xy[0], target_xy[1], clearance_radius_m)),
        'left_wall_clearance_m': float(left_clearance),
        'right_wall_clearance_m': float(right_clearance),
        'left_wall_hit': bool(left_hit),
        'right_wall_hit': bool(right_hit),
        'two_side_wall_evidence': bool(left_hit and right_hit),
        'balance_error_m': float(abs(left_clearance - right_clearance)),
        'min_clearance_m': float(min(left_clearance, right_clearance, nearest_clearance)),
        'nearest_obstacle_distance_m': float(nearest_clearance),
        'forward_progress_m': float(progress),
        'local_cost_sample_count': len(local_values),
        'local_cost_max_radius': max(local_values) if local_values else None,
        'local_cost_mean_radius': float(sum(local_values) / len(local_values)) if local_values else None,
        'footprint_high_cost_count': int(sum(1 for value in local_values if value >= high_cost_threshold)),
        'footprint_lethal_count': int(sum(1 for value in local_values if value >= 99)),
        'front_wedge_sample_count': len(wedge_values),
        'front_wedge_cost_max': max(wedge_values) if wedge_values else None,
        'front_wedge_high_cost_count': int(sum(1 for value in wedge_values if value >= high_cost_threshold)),
        'front_wedge_lethal_count': int(sum(1 for value in wedge_values if value >= 99)),
    }


def _candidate_reject_reasons(metrics: dict[str, Any]) -> list[str]:
    reasons: list[str] = []
    for key in [
        'same_corridor',
        'two_side_wall_evidence',
        'target_has_clearance',
        'occupancy_free',
        'footprint_lethal_not_increased',
        'front_wedge_lethal_not_increased',
        'safety_floor_ok',
        'forward_progress_ok',
    ]:
        if not bool(metrics.get(key, False)):
            reasons.append(key)
    if bool(metrics.get('hard_safety_pass', False)) and not bool(metrics.get('balance_error_improved', False)):
        reasons.append('balance_error_not_improved')
    return reasons


def _compact_candidate_reject_summary(metrics: dict[str, Any]) -> dict[str, Any]:
    return {
        'candidate_index': metrics.get('candidate_index'),
        'target_xy': metrics.get('target_xy', metrics.get('target')),
        'target_yaw': metrics.get('target_yaw'),
        'lateral_offset_m': metrics.get('lateral_offset_m'),
        'forward_offset_m': metrics.get('forward_offset_m'),
        'heading_offset_rad': metrics.get('heading_offset_rad'),
        'hard_safety_pass': bool(metrics.get('hard_safety_pass', False)),
        'same_corridor': bool(metrics.get('same_corridor', False)),
        'two_side_wall_evidence': bool(metrics.get('two_side_wall_evidence', False)),
        'footprint_lethal_not_increased': bool(metrics.get('footprint_lethal_not_increased', False)),
        'front_wedge_lethal_not_increased': bool(metrics.get('front_wedge_lethal_not_increased', False)),
        'safety_floor_ok': bool(metrics.get('safety_floor_ok', False)),
        'forward_progress_ok': bool(metrics.get('forward_progress_ok', False)),
        'min_clearance_m': metrics.get('min_clearance_m'),
        'local_cost_max_radius': metrics.get('local_cost_max_radius'),
        'balance_error_m': metrics.get('balance_error_m'),
        'candidate_reject_reasons': list(metrics.get('candidate_reject_reasons', [])),
    }


def _ray_clearance_to_blocked(
    grid: OccupancyGridView,
    origin_xy: Point,
    direction_rad: float,
    lookahead_m: float,
    clearance_radius_m: float,
) -> tuple[float, bool]:
    sample_step_m = max(grid.info.resolution, 0.05)
    best_distance = 0.0
    distance = sample_step_m
    while distance <= lookahead_m + 1e-9:
        x = origin_xy[0] + math.cos(direction_rad) * distance
        y = origin_xy[1] + math.sin(direction_rad) * distance
        if not grid.world_point_has_clearance(x, y, clearance_radius_m):
            return float(max(best_distance, distance)), True
        best_distance = distance
        distance += sample_step_m
    return float(lookahead_m), False


def _line_is_free_known(grid: OccupancyGridView, start: Point, end: Point) -> bool:
    return all(0 <= int(value) < grid.occupied_threshold for value in grid.line_sample_values(start, end))


def _grid_values_in_radius(grid: Optional[OccupancyGridView], center: Point, radius_m: float) -> list[int]:
    if grid is None:
        return []
    center_cell = grid.world_to_cell(float(center[0]), float(center[1]))
    radius_cells = int(math.ceil(max(radius_m, 0.0) / grid.info.resolution))
    values: list[int] = []
    for cell_y in range(center_cell[1] - radius_cells, center_cell[1] + radius_cells + 1):
        for cell_x in range(center_cell[0] - radius_cells, center_cell[0] + radius_cells + 1):
            cell = (cell_x, cell_y)
            if not grid.in_bounds(cell):
                continue
            world_x, world_y = grid.cell_to_world(cell_x, cell_y)
            if math.hypot(world_x - float(center[0]), world_y - float(center[1])) <= radius_m + 1e-9:
                values.append(int(grid.cell_value(cell)))
    return values


def _grid_values_in_wedge(
    grid: Optional[OccupancyGridView],
    origin_xy: Point,
    yaw: float,
    radius_m: float,
    half_angle_rad: float,
) -> list[int]:
    if grid is None:
        return []
    values: list[int] = []
    radial_steps = max(2, int(math.ceil(radius_m / max(grid.info.resolution, 0.05))))
    angle_steps = 8
    for radial_index in range(1, radial_steps + 1):
        radius = radius_m * radial_index / radial_steps
        for angle_index in range(-angle_steps, angle_steps + 1):
            angle = yaw + half_angle_rad * angle_index / angle_steps
            cell = grid.world_to_cell(
                float(origin_xy[0]) + radius * math.cos(angle),
                float(origin_xy[1]) + radius * math.sin(angle),
            )
            if grid.in_bounds(cell):
                values.append(int(grid.cell_value(cell)))
    return values


def _number_or_inf(value: Any) -> float:
    try:
        out = float(value)
    except (TypeError, ValueError):
        return float('inf')
    return out if math.isfinite(out) else float('inf')


def _optional_number_leq(left: Any, right: Any) -> bool:
    left_num = _number_or_inf(left)
    right_num = _number_or_inf(right)
    if math.isinf(left_num) and math.isinf(right_num):
        return True
    return left_num <= right_num + 1e-9


def _distance_to_blocked_boundary(
    grid: OccupancyGridView,
    origin_xy: Point,
    direction_rad: float,
    lookahead_m: float,
    clearance_radius_m: float,
) -> float:
    sample_step_m = max(grid.info.resolution, 0.05)
    best_distance = 0.0
    distance = sample_step_m
    while distance <= lookahead_m + 1e-9:
        x = origin_xy[0] + math.cos(direction_rad) * distance
        y = origin_xy[1] + math.sin(direction_rad) * distance
        if not grid.world_point_has_clearance(x, y, clearance_radius_m):
            break
        best_distance = distance
        distance += sample_step_m
    return best_distance


def _safe_distance_along_ray(
    grid: OccupancyGridView,
    origin_xy: Point,
    direction_rad: float,
    lookahead_m: float,
    clearance_radius_m: float,
) -> float:
    sample_step_m = max(grid.info.resolution * 0.5, 0.05)
    best_distance = 0.0
    distance = sample_step_m
    while distance <= lookahead_m + 1e-9:
        x = origin_xy[0] + math.cos(direction_rad) * distance
        y = origin_xy[1] + math.sin(direction_rad) * distance
        if not grid.world_point_has_clearance(x, y, clearance_radius_m):
            break
        best_distance = distance
        distance += sample_step_m
    return min(best_distance, lookahead_m)


def _merge_similar_directions(directions: List[OpenDirection], merge_angle_rad: float) -> List[OpenDirection]:
    merged: List[OpenDirection] = []
    for direction in directions:
        existing_index: Optional[int] = None
        for index, existing in enumerate(merged):
            delta = abs(normalize_angle(direction.angle_rad - existing.angle_rad))
            if delta <= merge_angle_rad:
                existing_index = index
                break
        if existing_index is None:
            merged.append(direction)
        elif direction.distance_m > merged[existing_index].distance_m:
            merged[existing_index] = direction
    return merged
