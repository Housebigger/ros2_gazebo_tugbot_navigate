#!/usr/bin/env python3
"""Phase60 read-only single-open-direction dead-end policy review.

This analyzer performs a static/offline review only.  It does not start Gazebo,
SLAM, Nav2, or maze_explorer, and it does not change runtime strategy.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any, Iterable, Optional

RUN_ID = 'phase60_single_open_direction_dead_end_policy_review'
ARTIFACT_DIR = Path('log') / RUN_ID
PHASE58_RUN_ID = 'phase58_post_ingress_candidate_formation_stability_replay'
PHASE59_RUN_ID = 'phase59_post_ingress_topology_consistency_guard'
NO_RUNTIME_LAUNCH = True
REVIEW_MODE = 'read_only_review'

ALLOWED_CLASSIFICATIONS = [
    'SINGLE_OPEN_DIRECTION_POLICY_REVIEW_COMPLETED',
    'SINGLE_OPEN_DIRECTION_MISCLASSIFIED_AS_DEAD_END',
    'SINGLE_OPEN_DIRECTION_POLICY_CURRENTLY_JUSTIFIED',
    'SINGLE_OPEN_DIRECTION_REVIEW_INCONCLUSIVE_NEEDS_RUNTIME_EVIDENCE',
    'GUARDRAIL_VIOLATION_STRATEGY_CHANGED',
]

CANDIDATE_STRATEGY_OPTIONS = [
    'keep_current_policy',
    'allow_single_open_direction_only_at_post_ingress',
    'allow_single_open_direction_if_not_backtracking',
    'require_multi_frame_single_open_direction_then_candidate',
    'require entrance/ingress context tag before first node',
]

GUARDRAILS = [
    'active scaled2x world/map artifacts only',
    'read-only static/offline review',
    'NO_RUNTIME_LAUNCH',
    'no Nav2/MPPI/controller parameter edits',
    'no clearance_radius_m tuning',
    'no map sufficiency threshold tuning',
    'no branch selection runtime strategy change',
    'no fallback/terminal acceptance',
    'no old scaffold world/map',
    'no autonomous exploration success claim',
    'first dispatch is not exit success',
    'bounded artifacts only; no long run',
]


def _load_json(path: Path) -> dict[str, Any]:
    if not path.exists():
        return {}
    return json.loads(path.read_text(encoding='utf-8'))


def _read_text(path: Path) -> str:
    if not path.exists():
        return ''
    return path.read_text(encoding='utf-8', errors='replace')


def _jsonl_rows(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    if not path.exists():
        return rows
    for line in path.read_text(encoding='utf-8', errors='replace').splitlines():
        if not line.strip():
            continue
        try:
            rows.append(json.loads(line))
        except json.JSONDecodeError:
            continue
    return rows


def _state(row: dict[str, Any]) -> dict[str, Any]:
    state = row.get('state')
    return state if isinstance(state, dict) else row


def _norm_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _angle_delta_deg(a: Optional[float], b: Optional[float]) -> Optional[float]:
    if a is None or b is None:
        return None
    return abs(math.degrees(_norm_angle(float(a) - float(b))))


def _heading(a: Iterable[float], b: Iterable[float]) -> Optional[float]:
    aa = list(a)
    bb = list(b)
    if len(aa) < 2 or len(bb) < 2:
        return None
    return math.atan2(float(bb[1]) - float(aa[1]), float(bb[0]) - float(aa[0]))


def _distance(a: Iterable[float], b: Iterable[float]) -> Optional[float]:
    aa = list(a)
    bb = list(b)
    if len(aa) < 2 or len(bb) < 2:
        return None
    return math.hypot(float(aa[0]) - float(bb[0]), float(aa[1]) - float(bb[1]))


def _first_topology_state(root: Path, run_id: str, replay_id: str) -> dict[str, Any]:
    path = root / 'log' / run_id / replay_id / f'{run_id}_{replay_id}_explorer_state.jsonl'
    for row in _jsonl_rows(path):
        state = _state(row)
        p56 = state.get('phase56_open_direction_to_candidate_diagnostics')
        if isinstance(p56, dict) and p56.get('available'):
            return state
        nested = state.get('last_topology_sampling_diagnostics') or {}
        p56 = nested.get('phase56_open_direction_to_candidate_diagnostics') if isinstance(nested, dict) else None
        if isinstance(p56, dict) and p56.get('available'):
            state = dict(state)
            state['phase56_open_direction_to_candidate_diagnostics'] = p56
            return state
    return {}


def _extract_phase58_review(root: Path) -> dict[str, Any]:
    aggregate_path = root / 'log' / PHASE58_RUN_ID / f'{PHASE58_RUN_ID}.json'
    data = _load_json(aggregate_path)
    replays = []
    for replay in data.get('replays', []) or []:
        replay_id = str(replay.get('replay_id') or '')
        state = _first_topology_state(root, PHASE58_RUN_ID, replay_id)
        p56 = state.get('phase56_open_direction_to_candidate_diagnostics') or {}
        replays.append({
            'replay_id': replay_id,
            'dispatch_observed': bool(replay.get('dispatch_observed')),
            'raw_open_direction_count': replay.get('raw_open_direction_count') or p56.get('raw_open_direction_count'),
            'filtered_open_direction_count': replay.get('filtered_open_direction_count') or p56.get('filtered_open_direction_count'),
            'candidate_before_filter_count': replay.get('candidate_before_filter_count') or p56.get('candidate_before_filter_count'),
            'candidate_after_filter_count': replay.get('candidate_after_filter_count') or p56.get('candidate_after_filter_count'),
            'junction_or_dead_end_policy_filter': p56.get('junction_or_dead_end_policy_filter'),
            'branch_candidate_rejection_reason': p56.get('branch_candidate_rejection_reason'),
            'accepted_open_direction_angle_rad': p56.get('accepted_open_direction_angle_rad'),
            'accepted_open_direction_angle_deg': p56.get('accepted_open_direction_angle_deg'),
            'candidate_local_costmap_cell_state': p56.get('candidate_local_costmap_cell_state'),
            'candidate_map_cell_state': p56.get('candidate_map_cell_state'),
        })
    return {
        'artifact': str(aggregate_path),
        'classification': data.get('classification'),
        'phase57_conclusion_preserved': 'TIMEOUT_INCONCLUSIVE_DATA_GAP',
        'phase58_conclusion_preserved': 'CANDIDATE_FORMATION_UNSTABLE_DEAD_END_POLICY_SENSITIVE',
        'mixed_dispatch_and_no_dispatch': bool(data.get('mixed_dispatch_and_no_dispatch')),
        'replays': replays,
    }


def _extract_phase59_offline_review(root: Path) -> dict[str, Any]:
    aggregate_path = root / 'log' / PHASE59_RUN_ID / f'{PHASE59_RUN_ID}.json'
    data = _load_json(aggregate_path)
    entrance_xy = (0.0, 0.0)
    entrance_yaw = 0.0
    ingress_waypoint_map = (1.0, 0.0, 0.0)
    ingress_vector_angle = _heading(entrance_xy, ingress_waypoint_map[:2])
    replay_reviews = []
    for replay in data.get('replays', []) or []:
        replay_id = str(replay.get('replay_id') or '')
        state = _first_topology_state(root, PHASE59_RUN_ID, replay_id)
        p56 = state.get('phase56_open_direction_to_candidate_diagnostics') or {}
        consistency_frames = replay.get('consistency_frames') or []
        first_frame = consistency_frames[0] if consistency_frames else {}
        robot_pose_map = first_frame.get('robot_pose_map') or None
        robot_yaw_rad = robot_pose_map[2] if isinstance(robot_pose_map, list) and len(robot_pose_map) >= 3 else None
        accepted_angle = p56.get('accepted_open_direction_angle_rad')
        return_to_entrance_angle = _heading(robot_pose_map[:2], entrance_xy) if isinstance(robot_pose_map, list) else None
        opposite_return_angle = _norm_angle(return_to_entrance_angle + math.pi) if return_to_entrance_angle is not None else None
        candidate_goal_point = p56.get('candidate_goal_point')
        robot_to_entrance_dist = _distance(robot_pose_map[:2], entrance_xy) if isinstance(robot_pose_map, list) else None
        candidate_to_entrance_dist = _distance(candidate_goal_point, entrance_xy) if isinstance(candidate_goal_point, list) else None
        angle_to_entrance_yaw_deg = _angle_delta_deg(accepted_angle, entrance_yaw)
        angle_to_ingress_vector_deg = _angle_delta_deg(accepted_angle, ingress_vector_angle)
        angle_to_return_to_entrance_deg = _angle_delta_deg(accepted_angle, return_to_entrance_angle)
        angle_to_opposite_return_deg = _angle_delta_deg(accepted_angle, opposite_return_angle)
        replay_reviews.append({
            'replay_id': replay_id,
            'robot_pose_map': robot_pose_map,
            'robot_yaw_rad': robot_yaw_rad,
            'entrance_yaw_rad': entrance_yaw,
            'ingress_waypoint_map': list(ingress_waypoint_map),
            'ingress_vector_angle_rad': ingress_vector_angle,
            'accepted_open_direction_angle_rad': accepted_angle,
            'accepted_open_direction_angle_deg': p56.get('accepted_open_direction_angle_deg'),
            'accepted_open_direction_vector': p56.get('accepted_open_direction_vector'),
            'angle_to_entrance_yaw_deg': angle_to_entrance_yaw_deg,
            'angle_to_ingress_vector_deg': angle_to_ingress_vector_deg,
            'angle_to_return_to_entrance_deg': angle_to_return_to_entrance_deg,
            'angle_to_opposite_return_deg': angle_to_opposite_return_deg,
            'roughly_points_into_maze': bool(angle_to_ingress_vector_deg is not None and angle_to_ingress_vector_deg <= 45.0),
            'opposite_return_to_entrance': bool(angle_to_opposite_return_deg is not None and angle_to_opposite_return_deg <= 45.0),
            'not_aligned_with_return_to_entrance': bool(angle_to_return_to_entrance_deg is not None and angle_to_return_to_entrance_deg >= 80.0),
            'robot_distance_from_entrance_m': robot_to_entrance_dist,
            'candidate_distance_from_entrance_m': candidate_to_entrance_dist,
            'candidate_increases_entrance_distance': bool(
                robot_to_entrance_dist is not None and candidate_to_entrance_dist is not None and candidate_to_entrance_dist > robot_to_entrance_dist
            ),
            'raw_open_direction_count': p56.get('raw_open_direction_count') or replay.get('raw_open_direction_count'),
            'filtered_open_direction_count': p56.get('filtered_open_direction_count') or replay.get('filtered_open_direction_count'),
            'candidate_before_filter_count': p56.get('candidate_before_filter_count') or replay.get('candidate_before_filter_count'),
            'candidate_after_filter_count': p56.get('candidate_after_filter_count') or replay.get('candidate_after_filter_count'),
            'candidate_goal_point': candidate_goal_point,
            'raw_branch_goal_point': p56.get('raw_branch_goal_point'),
            'candidate_map_cell_state': p56.get('candidate_map_cell_state'),
            'candidate_local_costmap_cell_state': p56.get('candidate_local_costmap_cell_state'),
            'candidate_clearance_result': p56.get('candidate_clearance_result'),
            'min_travel_distance_check': p56.get('min_travel_distance_check'),
            'dead_end_policy_state': replay.get('dead_end_policy_state'),
            'junction_or_dead_end_policy_filter': p56.get('junction_or_dead_end_policy_filter') or first_frame.get('junction_or_dead_end_policy_filter'),
            'branch_candidate_rejection_reason': p56.get('branch_candidate_rejection_reason') or first_frame.get('branch_candidate_rejection_reason'),
            'confirmed_no_candidate': bool(replay.get('confirmed_no_candidate')),
            'terminalization_delayed': any(bool(frame.get('terminalization_delayed')) for frame in consistency_frames),
        })
    return {
        'artifact': str(aggregate_path),
        'classification': data.get('classification'),
        'phase59_runtime_classification_preserved': 'TOPOLOGY_CONSISTENCY_CONFIRMED_NO_CANDIDATE',
        'phase58_conclusion_preserved': 'CANDIDATE_FORMATION_UNSTABLE_DEAD_END_POLICY_SENSITIVE',
        'replays': replay_reviews,
    }


def _static_policy_review(root: Path) -> dict[str, Any]:
    perception_path = root / 'src' / 'tugbot_maze' / 'tugbot_maze' / 'maze_perception.py'
    explorer_path = root / 'src' / 'tugbot_maze' / 'tugbot_maze' / 'maze_explorer.py'
    topology_path = root / 'src' / 'tugbot_maze' / 'tugbot_maze' / 'maze_topology.py'
    perception = _read_text(perception_path)
    explorer = _read_text(explorer_path)
    topology = _read_text(topology_path)
    single_is_dead_end = "elif count == 1:" in perception and "kind = DEAD_END" in perception
    dead_end_short_circuit = "if local.kind == DEAD_END:" in explorer and "filtered_open_directions=[]" in explorer
    no_branch_options = "rejection_reason='dead_end_policy_no_branch_options'" in explorer
    post_ingress_context_absent = 'post_ingress' not in explorer and 'ingress_waypoint' not in explorer
    return {
        'raw_open_direction_generation': 'sample_open_directions samples yaw-relative rays, accepts rays whose safe distance is >= min_open_distance_m, then merges similar directions.',
        'filtered_open_direction_filtering': 'filter_open_directions can remove reverse headings only when allow_reverse is false; the dead_end short-circuit bypasses this filter with filtered_open_directions=[].',
        'single_open_direction_dead_end_classification': 'classify_local_topology maps count == 1 to DEAD_END before branch candidate generation.',
        'dead_end_policy_no_branch_options_trigger': 'maze_explorer local.kind == DEAD_END builds Phase56 diagnostics with branch_options=[] and rejection_reason=dead_end_policy_no_branch_options, then terminalizes/backtracks unless Phase59 consistency guard delays it.',
        'post_ingress_context_absent': post_ingress_context_absent,
        'single_open_direction_is_default_dead_end': bool(single_is_dead_end),
        'dead_end_short_circuits_candidate_generation': bool(dead_end_short_circuit),
        'dead_end_policy_no_branch_options_present': bool(no_branch_options),
        'choose_next_branch_requires_branch_options': 'MazeTopology.choose_next_branch returns None when node.branches has no selectable UNTRIED branch options.',
        'evidence_files': [str(perception_path), str(explorer_path), str(topology_path)],
    }


def _candidate_strategy_review(offline_review: dict[str, Any]) -> list[dict[str, Any]]:
    replays = offline_review.get('replays') or []
    all_single = bool(replays) and all((r.get('raw_open_direction_count') == 1 or r.get('raw_open_direction_count') == [1, 1]) for r in replays)
    all_no_candidate = bool(replays) and all((r.get('candidate_after_filter_count') == 0 or r.get('candidate_after_filter_count') == [0, 0]) for r in replays)
    not_return = bool(replays) and all(bool(r.get('not_aligned_with_return_to_entrance')) for r in replays)
    clear_map_candidate = bool(replays) and all(((r.get('candidate_map_cell_state') or {}).get('clearance_result') == 'clear') for r in replays)
    local_cost_high = any(((r.get('candidate_local_costmap_cell_state') or {}).get('max_radius_cost') in (99, 100) or (r.get('candidate_local_costmap_cell_state') or {}).get('state') == 'lethal_or_obstacle') for r in replays)
    return [
        {
            'option': 'keep_current_policy',
            'assessment': 'not_preferred_for_post_ingress_without_more_context' if all_single and all_no_candidate and clear_map_candidate else 'plausible_if_runtime_cost_evidence_is_decisive',
            'rationale': 'The current policy is simple and conservative, but Phase59 shows it terminalizes before candidate generation when a single open ray exists near ingress.',
        },
        {
            'option': 'allow_single_open_direction_only_at_post_ingress',
            'assessment': 'recommended_minimal_phase61_candidate',
            'rationale': 'Scope the exception to the first post-ingress topology node so ordinary dead-end handling elsewhere remains unchanged.',
        },
        {
            'option': 'allow_single_open_direction_if_not_backtracking',
            'assessment': 'plausible_but_needs_visited_edge_context',
            'rationale': 'Phase59 accepted direction is not aligned with return-to-entrance, but the current topology has no explicit incoming-edge/backtracking context at first node.' if not_return else 'Needs stronger heading evidence.',
        },
        {
            'option': 'require_multi_frame_single_open_direction_then_candidate',
            'assessment': 'compatible_with_phase59_guard',
            'rationale': 'Phase59 already provides consecutive-frame stability; Phase61 could use confirmed single-open evidence as a candidate-formation precondition rather than terminalization.',
        },
        {
            'option': 'require entrance/ingress context tag before first node',
            'assessment': 'recommended_guardrail_for_exception',
            'rationale': 'A post-ingress tag would distinguish entrance corridor bootstrap from true dead ends at later nodes and prevent a broad strategy change.',
        },
        {
            'option': 'local_cost_caveat_not_a_strategy_option',
            'assessment': 'needs_runtime_validation_before_dispatch_acceptance' if local_cost_high else 'no_high_local_cost_seen',
            'rationale': 'The candidate goal is map-clear, but local cost max radius is high; Phase60 must not convert this into a dispatch behavior change.',
        },
    ]


def _count_matches(value: Any, expected: int) -> bool:
    if value == expected:
        return True
    if isinstance(value, list) and value:
        return all(item == expected for item in value)
    return False


def _classify(static_review: dict[str, Any], offline_review: dict[str, Any]) -> str:
    replays = offline_review.get('replays') or []
    if not static_review.get('single_open_direction_is_default_dead_end'):
        return 'SINGLE_OPEN_DIRECTION_POLICY_REVIEW_INCONCLUSIVE_NEEDS_RUNTIME_EVIDENCE'
    if not replays:
        return 'SINGLE_OPEN_DIRECTION_REVIEW_INCONCLUSIVE_NEEDS_RUNTIME_EVIDENCE'
    all_single_confirmed = all(
        _count_matches(r.get('raw_open_direction_count'), 1)
        and _count_matches(r.get('candidate_after_filter_count'), 0)
        and r.get('confirmed_no_candidate')
        for r in replays
    )
    has_candidate_geometry = all(bool(r.get('candidate_goal_point')) for r in replays)
    map_clear = all(((r.get('candidate_map_cell_state') or {}).get('clearance_result') == 'clear') for r in replays)
    not_return = all(bool(r.get('not_aligned_with_return_to_entrance')) for r in replays)
    local_cost_caveat = any(
        ((r.get('candidate_local_costmap_cell_state') or {}).get('state') == 'lethal_or_obstacle')
        or ((r.get('candidate_local_costmap_cell_state') or {}).get('max_radius_cost') in (99, 100))
        for r in replays
    )
    if all_single_confirmed and has_candidate_geometry and map_clear and not_return:
        # This is a policy-review classification, not a runtime-dispatch approval.
        # The local-cost caveat is preserved in candidate_strategy_review and the
        # Phase61 recommendation, but the static chain still shows single-open
        # geometry is terminalized before candidate generation.
        return 'SINGLE_OPEN_DIRECTION_MISCLASSIFIED_AS_DEAD_END'
    if all_single_confirmed and local_cost_caveat:
        return 'SINGLE_OPEN_DIRECTION_POLICY_REVIEW_COMPLETED'
    if all_single_confirmed:
        return 'SINGLE_OPEN_DIRECTION_POLICY_REVIEW_COMPLETED'
    return 'SINGLE_OPEN_DIRECTION_REVIEW_INCONCLUSIVE_NEEDS_RUNTIME_EVIDENCE'


def _markdown(result: dict[str, Any]) -> str:
    lines = [
        '# Phase60 Single Open Direction Dead-End Policy Review Artifact',
        '',
        f"run_id: `{RUN_ID}`",
        f"classification: `{result['classification']}`",
        '',
        '## Guardrails',
    ]
    lines += [f'- {item}' for item in result['guardrails']]
    lines += [
        '',
        '## Static policy review',
    ]
    for key, value in result['static_policy_review'].items():
        if key == 'evidence_files':
            continue
        lines.append(f'- {key}: {value}')
    lines += [
        '',
        '## Offline replay review',
    ]
    for replay in result['offline_replay_review']['replays']:
        lines += [
            f"### {replay['replay_id']}",
            f"- robot_pose_map: {replay.get('robot_pose_map')}",
            f"- robot_yaw_rad: {replay.get('robot_yaw_rad')}",
            f"- raw/filtered/candidate_after: {replay.get('raw_open_direction_count')}/{replay.get('filtered_open_direction_count')}/{replay.get('candidate_after_filter_count')}",
            f"- accepted_open_direction_angle_deg: {replay.get('accepted_open_direction_angle_deg')}",
            f"- angle_to_entrance_yaw_deg: {replay.get('angle_to_entrance_yaw_deg')}",
            f"- angle_to_ingress_vector_deg: {replay.get('angle_to_ingress_vector_deg')}",
            f"- angle_to_return_to_entrance_deg: {replay.get('angle_to_return_to_entrance_deg')}",
            f"- roughly_points_into_maze: {replay.get('roughly_points_into_maze')}",
            f"- opposite_return_to_entrance: {replay.get('opposite_return_to_entrance')}",
            f"- candidate_goal_point: {replay.get('candidate_goal_point')}",
            f"- candidate_map_cell_state: {replay.get('candidate_map_cell_state')}",
            f"- candidate_local_costmap_cell_state: {replay.get('candidate_local_costmap_cell_state')}",
            f"- candidate_clearance_result: {replay.get('candidate_clearance_result')}",
            f"- dead_end_policy_state: {replay.get('dead_end_policy_state')}",
        ]
    lines += [
        '',
        '## Candidate strategy review',
    ]
    for option in result['candidate_strategy_review']:
        lines.append(f"- {option['option']}: {option['assessment']} — {option['rationale']}")
    lines += [
        '',
        '## Phase61 recommendation',
        result['phase61_recommendation'],
        '',
        'No autonomous exploration success is claimed. Any first dispatch evidence remains only first-dispatch/candidate-formation evidence, not exit success.',
    ]
    return '\n'.join(lines) + '\n'


def analyze(root: Path, artifact_dir: Path) -> dict[str, Any]:
    static_review = _static_policy_review(root)
    phase58_review = _extract_phase58_review(root)
    offline_review = _extract_phase59_offline_review(root)
    candidate_review = _candidate_strategy_review(offline_review)
    classification = _classify(static_review, offline_review)
    guardrail_violation = False
    result = {
        'run_id': RUN_ID,
        'mode': REVIEW_MODE,
        'NO_RUNTIME_LAUNCH': NO_RUNTIME_LAUNCH,
        'artifact_dir': str(artifact_dir),
        'classification': classification if not guardrail_violation else 'GUARDRAIL_VIOLATION_STRATEGY_CHANGED',
        'allowed_classifications': ALLOWED_CLASSIFICATIONS,
        'guardrails': GUARDRAILS,
        'guardrail_violation': guardrail_violation,
        'active_world': 'tugbot_maze_world_20260528_clean_scaled2x.sdf',
        'active_metadata': 'maze_20260528_scaled_instance.yaml',
        'phase58_conclusion_preserved': 'CANDIDATE_FORMATION_UNSTABLE_DEAD_END_POLICY_SENSITIVE',
        'phase59_runtime_classification_preserved': 'TOPOLOGY_CONSISTENCY_CONFIRMED_NO_CANDIDATE',
        'phase58_artifact_review': phase58_review,
        'static_policy_review': static_review,
        'offline_replay_review': offline_review,
        'candidate_strategy_options': CANDIDATE_STRATEGY_OPTIONS,
        'candidate_strategy_review': candidate_review,
        'phase61_recommendation': (
            'If Phase61 is approved, implement the smallest post-ingress single-open-direction candidate exception: only after dispatch-entry readiness, only at the first post-ingress topology node, only after Phase59-style multi-frame confirmation, only when the single open direction is not return/backtracking, and keep all Nav2/clearance/map thresholds unchanged.'
            if classification == 'SINGLE_OPEN_DIRECTION_MISCLASSIFIED_AS_DEAD_END'
            else 'Do not change runtime behavior until additional evidence resolves the single-open-direction policy ambiguity.'
        ),
        'outputs': {
            'json': str(artifact_dir / 'phase60_single_open_direction_dead_end_policy_review.json'),
            'markdown': str(artifact_dir / 'phase60_single_open_direction_dead_end_policy_review.md'),
        },
    }
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', default='.', help='workspace root')
    parser.add_argument('--artifact-dir', default=str(ARTIFACT_DIR), help='Phase60 artifact directory')
    args = parser.parse_args()
    root = Path(args.root).resolve()
    artifact_dir = Path(args.artifact_dir)
    if not artifact_dir.is_absolute():
        artifact_dir = root / artifact_dir
    artifact_dir.mkdir(parents=True, exist_ok=True)
    result = analyze(root=root, artifact_dir=artifact_dir)
    json_path = artifact_dir / 'phase60_single_open_direction_dead_end_policy_review.json'
    md_path = artifact_dir / 'phase60_single_open_direction_dead_end_policy_review.md'
    json_path.write_text(json.dumps(result, indent=2, sort_keys=True, ensure_ascii=False) + '\n', encoding='utf-8')
    md_path.write_text(_markdown(result), encoding='utf-8')
    print(json.dumps(result, indent=2, sort_keys=True, ensure_ascii=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
