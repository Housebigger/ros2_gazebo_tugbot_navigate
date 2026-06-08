#!/usr/bin/env python3
"""Phase67 Goal 1 Timeout Visual Replay / Terminal Pose Diagnosis.

Builds a visual-overlay payload from accepted Phase66 evidence.  This phase is
for human observation in Gazebo/RViz only: it does not tune Nav2/MPPI/controller,
does not change branch scoring, does not add projection/fallback/terminal
acceptance, and does not claim autonomous exploration success or exit success.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

PHASE = 'Phase67 Goal 1 Timeout Visual Replay / Terminal Pose Diagnosis'
RUN_ID = 'phase67_goal1_timeout_visual_replay'
PHASE66_RUN_ID = 'phase66_bounded_autonomous_rerun_from_inner_ingress'
PHASE66_CLASSIFICATION = 'INNER_INGRESS_TIMEOUT_REMAINS'
ACTIVE_WORLD = 'tugbot_maze_world_20260528_clean_scaled2x.sdf'
INNER_INGRESS_WAYPOINT_MAP = {'x_m': 2.0, 'y_m': 0.0, 'yaw_rad': 0.0}
ROBOT_RADIUS_M = 0.35
TARGET_TOLERANCE_REFERENCE_RADIUS_M = 0.25
FRONT_WEDGE_LENGTH_M = 0.80
FRONT_WEDGE_HALF_ANGLE_RAD = 0.45
ALLOWED_CLASSIFICATIONS = [
    'VISUALLY_TARGET_TOO_CLOSE_TO_WALL',
    'VISUALLY_TURNING_FOOTPRINT_COLLIDES',
    'VISUALLY_GOAL_TOLERANCE_ORIENTATION_BLOCKED',
    'VISUALLY_COSTMAP_RECOVERY_LOOP',
    'VISUAL_EVIDENCE_INCONCLUSIVE',
]
GUARDRAILS = [
    'visual replay / manual observation only',
    'bounded Goal 1 replay only; max_goals=1 or equivalent',
    'no Nav2/MPPI/controller parameter edits',
    'no inflation/robot_radius/clearance_radius_m/map threshold tuning',
    'no branch scoring change',
    'no target projection integration',
    'no fallback/terminal acceptance change',
    'no autonomous exploration success claim',
    'no exit success claim',
]


def _safe_json_load(path: Path | None, default: Any) -> Any:
    if not path or not path.exists() or not path.stat().st_size:
        return default
    try:
        return json.loads(path.read_text(encoding='utf-8', errors='replace'))
    except Exception:
        return default


def _safe_json_loads(text: str) -> Any | None:
    try:
        return json.loads(text)
    except Exception:
        return None


def _read_jsonl(path: Path | None) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    if not path or not path.exists() or not path.stat().st_size:
        return rows
    for line in path.read_text(encoding='utf-8', errors='replace').splitlines():
        if not line.strip():
            continue
        row = _safe_json_loads(line)
        if isinstance(row, dict):
            rows.append(row)
    return rows


def _number(value: Any) -> float | None:
    try:
        out = float(value)
    except (TypeError, ValueError):
        return None
    return out if math.isfinite(out) else None


def _dist(a: list[float] | tuple[float, ...], b: list[float] | tuple[float, ...]) -> float:
    return math.hypot(float(a[0]) - float(b[0]), float(a[1]) - float(b[1]))


def _angle_normalize(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _stats(values: list[float]) -> dict[str, Any]:
    clean = [float(v) for v in values if math.isfinite(float(v))]
    if not clean:
        return {'count': 0, 'min': None, 'max': None, 'mean': None, 'final': None}
    return {
        'count': len(clean),
        'min': min(clean),
        'max': max(clean),
        'mean': sum(clean) / len(clean),
        'final': clean[-1],
    }


def _summary_from_stats(stats: dict[str, Any] | None, key: str = 'max') -> Any:
    if isinstance(stats, dict):
        return stats.get(key)
    return None


def _repo_root_from_phase66_artifact(path: Path) -> Path:
    # <root>/log/phase66.../phase66....json
    try:
        return path.resolve().parents[2]
    except Exception:
        return Path.cwd()


def _phase66_replay_dir(phase66_artifact: Path, replay: dict[str, Any]) -> Path:
    artifact_dir = replay.get('artifact_dir')
    if artifact_dir:
        return Path(str(artifact_dir))
    return phase66_artifact.parent / 'replay_01'


def _extract_goal_events(replay_dir: Path) -> list[dict[str, Any]]:
    candidates = list(replay_dir.glob('*_goal_events.jsonl'))
    if not candidates:
        return []
    rows = _read_jsonl(candidates[0])
    out: list[dict[str, Any]] = []
    for row in rows:
        state = row.get('state') if isinstance(row.get('state'), dict) else row
        if isinstance(state, dict):
            state = dict(state)
            state.setdefault('_recorder_elapsed_sec', row.get('elapsed_sec'))
            state.setdefault('_recorder_seq', row.get('seq'))
            out.append(state)
    return out


def _trajectory_points(replay_dir: Path, max_points: int = 140) -> list[list[float]]:
    rows = _read_jsonl(replay_dir / 'phase66_controller_dynamics.jsonl')
    poses: list[list[float]] = []
    for row in rows:
        if row.get('source') != 'odom':
            continue
        x, y, yaw = _number(row.get('x')), _number(row.get('y')), _number(row.get('yaw'))
        if x is None or y is None:
            continue
        pose = [x, y, yaw if yaw is not None else 0.0]
        if not poses or _dist(poses[-1], pose) > 0.015:
            poses.append(pose)
    if len(poses) <= max_points:
        return poses
    stride = max(1, math.ceil(len(poses) / max_points))
    sampled = poses[::stride]
    if sampled[-1] != poses[-1]:
        sampled.append(poses[-1])
    return sampled


def _cmd_vel_window(replay_dir: Path, start: float | None, end: float | None) -> dict[str, Any]:
    rows = _read_jsonl(replay_dir / 'phase66_controller_dynamics.jsonl')
    selected: list[dict[str, Any]] = []
    for row in rows:
        if row.get('source') not in {'cmd_vel', 'cmd_vel_smoothed'}:
            continue
        elapsed = _number(row.get('elapsed_sec'))
        if elapsed is None:
            continue
        if start is not None and elapsed < start:
            continue
        if end is not None and elapsed > end:
            continue
        selected.append(row)
    lin = [abs(float(row.get('linear_x', 0.0))) for row in selected]
    ang = [abs(float(row.get('angular_z', 0.0))) for row in selected]
    return {
        'sample_count': len(selected),
        'linear_x_abs': _stats(lin),
        'angular_z_abs': _stats(ang),
        'nonzero_count': sum(1 for lx, az in zip(lin, ang) if lx > 0.01 or az > 0.05),
    }


def _final_local_cost_sample(replay_dir: Path) -> dict[str, Any]:
    rows = _read_jsonl(replay_dir / 'phase66_local_costmap_samples.jsonl')
    return rows[-1] if rows else {}


def _max_local_cost_sample(replay_dir: Path) -> dict[str, Any]:
    rows = _read_jsonl(replay_dir / 'phase66_local_costmap_samples.jsonl')
    if not rows:
        return {}
    def score(row: dict[str, Any]) -> tuple[int, int, int]:
        fw = row.get('front_wedge_cost') or {}
        fp = row.get('robot_footprint_cost') or {}
        return (
            int(fw.get('lethal_count') or 0),
            int(fw.get('high_cost_count') or 0),
            int(fp.get('lethal_count') or 0),
        )
    return max(rows, key=score)


def _timeout_goal_event(goal_events: list[dict[str, Any]]) -> dict[str, Any]:
    for row in goal_events:
        if row.get('result_reason') == 'goal_timeout' or row.get('event') == 'timeout':
            return row
    return {}


def _timeout_cancel_event(goal_events: list[dict[str, Any]]) -> dict[str, Any]:
    for row in goal_events:
        if row.get('result_reason') == 'goal_canceled_after_timeout' or row.get('event') == 'timeout_cancel_result':
            return row
    return {}


def _visual_classification(timeout_event: dict[str, Any], goal: dict[str, Any], human_observation: str | None = None) -> str:
    # Without human visual feedback, keep the classification conservative even if
    # costmap risk is strong.  Report candidate visual hypotheses separately.
    if human_observation:
        normalized = human_observation.strip().upper()
        if normalized in ALLOWED_CLASSIFICATIONS:
            return normalized
    return 'VISUAL_EVIDENCE_INCONCLUSIVE'


def _visual_hypotheses(timeout_event: dict[str, Any], goal: dict[str, Any]) -> list[str]:
    hypotheses: list[str] = []
    if _number(timeout_event.get('timeout_right_side_clearance_m')) == 0.0:
        hypotheses.append('VISUALLY_TARGET_TOO_CLOSE_TO_WALL')
    if int(timeout_event.get('timeout_footprint_lethal_cell_count') or 0) > 0:
        hypotheses.append('VISUALLY_TURNING_FOOTPRINT_COLLIDES')
    if int(goal.get('nav2_feedback_summary', {}).get('number_of_recoveries_max') or 0) > 0:
        hypotheses.append('VISUALLY_COSTMAP_RECOVERY_LOOP')
    final_pose = goal.get('robot_progress_summary', {}).get('final_pose') or []
    target = goal.get('target') or []
    if len(final_pose) >= 3 and len(target) >= 2:
        target_bearing = math.atan2(float(target[1]) - float(final_pose[1]), float(target[0]) - float(final_pose[0]))
        if abs(_angle_normalize(float(final_pose[2]) - target_bearing)) > 1.2:
            hypotheses.append('VISUALLY_GOAL_TOLERANCE_ORIENTATION_BLOCKED')
    return sorted(set(hypotheses)) or ['VISUAL_EVIDENCE_INCONCLUSIVE']


def build_phase67_payload(phase66_artifact: str | Path, human_observation: str | None = None) -> dict[str, Any]:
    phase66_path = Path(phase66_artifact)
    data = _safe_json_load(phase66_path, {})
    replay = (data.get('replays') or [{}])[0]
    replay_dir = _phase66_replay_dir(phase66_path, replay)
    goal = (replay.get('per_goal_summaries') or [{}])[0]
    goal_events = _extract_goal_events(replay_dir)
    timeout_event = _timeout_goal_event(goal_events)
    cancel_event = _timeout_cancel_event(goal_events)
    final_cost_sample = _final_local_cost_sample(replay_dir)
    max_cost_sample = _max_local_cost_sample(replay_dir)
    trajectory = _trajectory_points(replay_dir)

    target = [float(v) for v in (goal.get('target') or [2.083600873682933, 1.022924811693503])[:2]]
    dispatch_pose = [float(v) for v in (goal.get('dispatch_pose') or [1.853110128126024, 0.02417608567611149, -0.005494165745570537])[:3]]
    terminal_pose = [float(v) for v in (goal.get('robot_progress_summary', {}).get('final_pose') or trajectory[-1] if trajectory else [2.4175180046806055, 1.0220449273557242, 1.5843474169880594])[:3]]
    if not trajectory:
        trajectory = [dispatch_pose, terminal_pose]
    dispatch_elapsed = None
    timeout_elapsed = _number(timeout_event.get('_recorder_elapsed_sec'))
    if goal_events:
        dispatch_elapsed = _number(goal_events[0].get('_recorder_elapsed_sec'))
    cmd_before_timeout = _cmd_vel_window(replay_dir, None if timeout_elapsed is None else max(0.0, timeout_elapsed - 8.0), timeout_elapsed)
    cmd_after_timeout = _cmd_vel_window(replay_dir, timeout_elapsed, None if timeout_elapsed is None else timeout_elapsed + 8.0)

    local_cost = goal.get('local_cost_footprint_front_wedge_summary') or {}
    timeout_front_wedge_cost_max = timeout_event.get('timeout_front_wedge_cost_max')
    timeout_footprint_lethal_count = timeout_event.get('timeout_footprint_lethal_cell_count')
    timeout_robot_local_cost_max = timeout_event.get('timeout_robot_local_cost_max')

    risk_marker_pose = list(terminal_pose)
    clearance = _number(timeout_event.get('timeout_front_wedge_clearance_m'))
    if clearance is not None:
        risk_marker_pose = [terminal_pose[0] + math.cos(terminal_pose[2]) * clearance, terminal_pose[1] + math.sin(terminal_pose[2]) * clearance, terminal_pose[2]]

    payload = {
        'run_id': RUN_ID,
        'phase': PHASE,
        'source_phase66_artifact': str(phase66_path),
        'source_replay_dir': str(replay_dir),
        'active_world': ACTIVE_WORLD,
        'inner_ingress_waypoint_map': INNER_INGRESS_WAYPOINT_MAP,
        'phase66_classification_preserved': data.get('classification') or PHASE66_CLASSIFICATION,
        'phase66_secondary_observed_risk_preserved': 'LOCAL_COST_RISK_REMAINS' if data.get('metrics', {}).get('local_cost_risk_observed') else None,
        'classification': _visual_classification(timeout_event, goal, human_observation),
        'candidate_visual_hypotheses_for_human_check': _visual_hypotheses(timeout_event, goal),
        'allowed_classifications': ALLOWED_CLASSIFICATIONS,
        'guardrails': GUARDRAILS,
        'robot_radius_m': ROBOT_RADIUS_M,
        'target_tolerance_reference_radius_m': TARGET_TOLERANCE_REFERENCE_RADIUS_M,
        'front_wedge_length_m': FRONT_WEDGE_LENGTH_M,
        'front_wedge_half_angle_rad': FRONT_WEDGE_HALF_ANGLE_RAD,
        'complete_autonomous_success_claimed': False,
        'exit_success_claimed': False,
        'goal1': {
            'goal_sequence': int(goal.get('goal_sequence') or 1),
            'dispatch_observed': bool(goal.get('dispatch_observed')),
            'timeout': bool(goal.get('timeout')),
            'outcome_event': goal.get('outcome_event'),
            'result_reason': goal.get('result_reason'),
            'cancel_result_reason': cancel_event.get('result_reason'),
            'target': target,
            'dispatch_pose': dispatch_pose,
            'terminal_pose': terminal_pose,
            'trajectory_points': trajectory,
            'dispatch_elapsed_sec': dispatch_elapsed,
            'timeout_elapsed_sec': timeout_elapsed,
            'distance_to_target_final_m': goal.get('robot_progress_summary', {}).get('distance_to_target', {}).get('final'),
            'distance_to_target_improvement_m': goal.get('robot_progress_summary', {}).get('distance_improvement_m'),
            'robot_total_motion_m': goal.get('robot_progress_summary', {}).get('total_motion_m'),
            'robot_stuck': goal.get('robot_progress_summary', {}).get('robot_stuck'),
            'cmd_vel': {
                'summary': goal.get('cmd_vel_summary') or {},
                'timeout_minus_8s_to_timeout': cmd_before_timeout,
                'timeout_to_timeout_plus_8s': cmd_after_timeout,
                'nonzero_command_count': goal.get('cmd_vel_summary', {}).get('nonzero_command_count'),
            },
            'nav2_feedback': {
                **(goal.get('nav2_feedback_summary') or {}),
                'number_of_recoveries_max': goal.get('nav2_feedback_summary', {}).get('number_of_recoveries_max'),
                'final_distance_remaining_m': goal.get('nav2_feedback_summary', {}).get('distance_remaining', {}).get('final'),
            },
            'local_cost': {
                'summary': local_cost,
                'timeout_front_wedge_cost_max': timeout_front_wedge_cost_max,
                'timeout_front_wedge_cost_mean': timeout_event.get('timeout_front_wedge_cost_mean'),
                'timeout_front_wedge_clearance_m': timeout_event.get('timeout_front_wedge_clearance_m'),
                'timeout_footprint_cost_max': timeout_event.get('timeout_footprint_cost_max'),
                'timeout_footprint_lethal_cell_count': timeout_footprint_lethal_count,
                'timeout_footprint_inflated_cell_count': timeout_event.get('timeout_footprint_inflated_cell_count'),
                'timeout_robot_local_cost_max': timeout_robot_local_cost_max,
                'timeout_robot_local_cost_mean': timeout_event.get('timeout_robot_local_cost_mean'),
                'timeout_right_side_clearance_m': timeout_event.get('timeout_right_side_clearance_m'),
                'timeout_right_side_cost_max': timeout_event.get('timeout_right_side_cost_max'),
                'timeout_left_side_clearance_m': timeout_event.get('timeout_left_side_clearance_m'),
                'timeout_left_side_cost_max': timeout_event.get('timeout_left_side_cost_max'),
                'timeout_path_ahead_0_5m_cost_max': timeout_event.get('timeout_path_ahead_0_5m_cost_max'),
                'timeout_path_ahead_1_0m_cost_max': timeout_event.get('timeout_path_ahead_1_0m_cost_max'),
                'risk_marker_pose': risk_marker_pose,
                'final_local_cost_sample': final_cost_sample,
                'max_risk_local_cost_sample': max_cost_sample,
            },
            'dispatch_event': goal_events[0] if goal_events else {},
            'timeout_event': timeout_event,
            'timeout_cancel_event': cancel_event,
        },
        'rviz_marker_topic': '/phase67/goal1_timeout_visual_markers',
        'suggested_screenshot_moments': [
            'after inner ingress succeeds, before maze_explorer starts',
            'immediately after Goal 1 dispatch marker appears',
            'while robot turns near the Goal 1 target and front-wedge marker overlaps high-cost cells',
            'at timeout terminal pose after recoveries reach the recorded maximum',
            'after toggling RViz local_costmap/footprint/MarkerArray displays together',
        ],
    }
    return payload


def write_payload(phase66_artifact: str | Path, output: str | Path, human_observation: str | None = None) -> dict[str, Any]:
    payload = build_phase67_payload(phase66_artifact, human_observation=human_observation)
    out = Path(output)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding='utf-8')
    return payload


def main() -> None:
    parser = argparse.ArgumentParser(description=PHASE)
    parser.add_argument('--phase66-artifact', default=f'log/{PHASE66_RUN_ID}/{PHASE66_RUN_ID}.json')
    parser.add_argument('--output', default=f'log/{RUN_ID}/{RUN_ID}.json')
    parser.add_argument('--human-observation-classification', choices=ALLOWED_CLASSIFICATIONS)
    args = parser.parse_args()
    payload = write_payload(args.phase66_artifact, args.output, args.human_observation_classification)
    print(json.dumps({'run_id': RUN_ID, 'classification': payload['classification'], 'output': str(args.output)}, sort_keys=True))


if __name__ == '__main__':
    main()
