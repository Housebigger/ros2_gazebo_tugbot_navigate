#!/usr/bin/env python3
"""Phase80 Goal2 near-goal forward-open corridor diagnostic classification.

Scope: diagnostics-only analyzer for the Phase79 Goal2 timeout reproduction.
It adds the classification NEAR_GOAL_LATERAL_RESIDUAL_WITH_FORWARD_OPEN_CORRIDOR
as a geometry/diagnostic label, while preserving guardrails:
- no maze_explorer strategy change
- no branch scoring change
- no centerline gate change
- no directional readiness override change
- no fallback/terminal acceptance change
- No Nav2/MPPI/controller tuning
- no inflation/robot_radius/clearance_radius_m/map threshold tuning
- does not claim autonomous exploration success
- does not claim exit success
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

RUN_ID = 'phase80_goal2_near_goal_forward_open_corridor_diagnostic_classification'
GOAL_SEQUENCE = 2
XY_GOAL_TOLERANCE_M = 0.25
NEAR_GOAL_DISTANCE_M = 0.50
LATERAL_DOMINANCE_RATIO = 2.0
FORWARD_SMALL_M = 0.08
CLASSIFICATION = 'NEAR_GOAL_LATERAL_RESIDUAL_WITH_FORWARD_OPEN_CORRIDOR'
FORWARD_OPEN_EVIDENCE_INSUFFICIENT = 'FORWARD_OPEN_EVIDENCE_INSUFFICIENT'
FORWARD_OPEN_EVIDENCE_SUPPORTED_BY_SCAN = 'FORWARD_OPEN_EVIDENCE_SUPPORTED_BY_SCAN'
FORWARD_OPEN_EVIDENCE_CONTRADICTED_BY_LOCAL_COST = 'FORWARD_OPEN_EVIDENCE_CONTRADICTED_BY_LOCAL_COST'
INSUFFICIENT_EVIDENCE = 'INSUFFICIENT_EVIDENCE'

GUARDRAILS = [
    'diagnostics-only classification',
    'no maze_explorer strategy change',
    'no branch scoring change',
    'no centerline gate change',
    'no directional readiness override change',
    'no fallback/terminal acceptance change',
    'No Nav2/MPPI/controller tuning',
    'no inflation/robot_radius/clearance_radius_m/map threshold tuning',
    'does not claim autonomous exploration success',
    'does not claim exit success',
    'not fallback/terminal acceptance',
    'not a Nav2 parameter final diagnosis',
    'Phase81 not entered',
]


def _safe_load_json(path: Path | None, default: Any) -> Any:
    if not path or not path.exists() or not path.stat().st_size:
        return default
    try:
        return json.loads(path.read_text(encoding='utf-8', errors='replace'))
    except Exception:
        return default


def _read_jsonl(path: Path | None) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    if not path or not path.exists() or not path.stat().st_size:
        return rows
    for line in path.read_text(encoding='utf-8', errors='replace').splitlines():
        if not line.strip():
            continue
        try:
            row = json.loads(line)
        except Exception:
            continue
        payload = row.get('state') if isinstance(row.get('state'), dict) else row.get('payload') if isinstance(row.get('payload'), dict) else row
        if not isinstance(payload, dict):
            continue
        payload = dict(payload)
        payload.setdefault('wall_time', row.get('wall_time'))
        payload.setdefault('elapsed_sec', row.get('elapsed_sec'))
        rows.append(payload)
    return rows


def _artifact_file(artifact_dir: Path, suffix: str) -> Path | None:
    exact = artifact_dir / suffix
    if exact.exists():
        return exact
    matches = sorted(artifact_dir.glob(f'*{suffix}'))
    return matches[0] if matches else None


def _number(value: Any) -> float | None:
    try:
        out = float(value)
    except (TypeError, ValueError):
        return None
    return out if math.isfinite(out) else None


def _vec2(value: Any) -> list[float] | None:
    if isinstance(value, (list, tuple)) and len(value) >= 2:
        x = _number(value[0])
        y = _number(value[1])
        if x is not None and y is not None:
            return [x, y]
    return None


def _pose(value: Any) -> list[float] | None:
    if isinstance(value, (list, tuple)) and len(value) >= 3:
        x = _number(value[0])
        y = _number(value[1])
        yaw = _number(value[2])
        if x is not None and y is not None and yaw is not None:
            return [x, y, yaw]
    return None


def _goal_rows(rows: list[dict[str, Any]], seq: int = GOAL_SEQUENCE) -> list[dict[str, Any]]:
    out = []
    for row in rows:
        goal_sequence = row.get('goal_sequence')
        if goal_sequence is None:
            continue
        try:
            if int(goal_sequence) == seq:
                out.append(row)
        except (TypeError, ValueError):
            continue
    return out


def _find_event(rows: list[dict[str, Any]], event: str, seq: int = GOAL_SEQUENCE) -> dict[str, Any] | None:
    for row in rows:
        if row.get('event') != event:
            continue
        goal_sequence = row.get('goal_sequence')
        if goal_sequence is None:
            continue
        try:
            if int(goal_sequence) == seq:
                return row
        except (TypeError, ValueError):
            continue
    return None


def _last_pose_from_controller(rows: list[dict[str, Any]], seq: int = GOAL_SEQUENCE) -> list[float] | None:
    odom = [r for r in _goal_rows(rows, seq) if r.get('source') == 'odom']
    for row in reversed(odom):
        x = _number(row.get('x'))
        y = _number(row.get('y'))
        yaw = _number(row.get('yaw'))
        if x is not None and y is not None and yaw is not None:
            return [x, y, yaw]
    return None


def _latest_local_cost_pose(rows: list[dict[str, Any]], seq: int = GOAL_SEQUENCE) -> list[float] | None:
    samples = _goal_rows(rows, seq)
    for row in reversed(samples):
        pose = _pose(row.get('robot_pose'))
        if pose is not None:
            return pose
    return None


def _max_number(rows: list[dict[str, Any]], keys: list[tuple[str, ...]]) -> float | None:
    vals: list[float] = []
    for row in rows:
        for key_path in keys:
            cur: Any = row
            for key in key_path:
                if not isinstance(cur, dict):
                    cur = None
                    break
                cur = cur.get(key)
            value = _number(cur)
            if value is not None:
                vals.append(value)
    return max(vals) if vals else None


def _relative_target_body_frame(robot_pose: list[float], target: list[float]) -> dict[str, Any]:
    dx = target[0] - robot_pose[0]
    dy = target[1] - robot_pose[1]
    yaw = robot_pose[2]
    c = math.cos(yaw)
    s = math.sin(yaw)
    forward = c * dx + s * dy
    lateral = -s * dx + c * dy
    abs_forward = abs(forward)
    abs_lateral = abs(lateral)
    if abs_lateral >= max(abs_forward * LATERAL_DOMINANCE_RATIO, 0.10):
        axis = 'lateral'
    elif abs_forward >= max(abs_lateral * LATERAL_DOMINANCE_RATIO, 0.10):
        axis = 'forward'
    else:
        axis = 'mixed'
    return {
        'dx_map_m': dx,
        'dy_map_m': dy,
        'robot_yaw_rad': yaw,
        'forward_component_m': forward,
        'lateral_component_m': lateral,
        'abs_forward_component_m': abs_forward,
        'abs_lateral_component_m': abs_lateral,
        'dominant_residual_axis': axis,
        'target_is_behind_robot': forward < -FORWARD_SMALL_M,
        'target_is_nearly_lateral': abs_forward <= FORWARD_SMALL_M and abs_lateral > XY_GOAL_TOLERANCE_M,
    }


def _scan_files(artifact_dir: Path) -> list[Path]:
    return sorted(p for p in artifact_dir.glob('*scan*') if p.is_file())


def _forward_open_evidence(artifact_dir: Path, local_cost_rows: list[dict[str, Any]]) -> dict[str, Any]:
    """Conservative forward-open evidence check.

    Phase80 may only use positive evidence that is actually present.  Aggregated
    front-wedge cost is not enough to prove a physically open corridor; raw scan
    ranges or raw local-cost geometry in front of the robot would be needed.
    """
    gaps: list[str] = []
    notes: list[str] = []
    scan_paths = _scan_files(artifact_dir)
    if not scan_paths:
        gaps.append('no_scan_artifact')
    else:
        notes.append(f'scan_artifact_count={len(scan_paths)}')

    latest = local_cost_rows[-1] if local_cost_rows else {}
    front = latest.get('front_wedge_cost') if isinstance(latest.get('front_wedge_cost'), dict) else {}
    front_max = _number(front.get('max'))
    front_lethal = _number(front.get('lethal_count'))
    front_high = _number(front.get('high_cost_count'))
    front_clearance = _number(latest.get('front_wedge_clearance_m'))
    has_raw_grid = bool(latest.get('local_costmap_raw') or latest.get('local_costmap_grid') or latest.get('ranges'))
    if not has_raw_grid:
        gaps.append('no_raw_local_costmap_geometry_for_forward_open_test')
    if front_max is not None:
        notes.append(f'latest_front_wedge_cost_max={front_max}')
    if front_lethal is not None:
        notes.append(f'latest_front_wedge_lethal_count={front_lethal}')
    if front_high is not None:
        notes.append(f'latest_front_wedge_high_cost_count={front_high}')
    if front_clearance is not None:
        notes.append(f'latest_front_wedge_clearance_m={front_clearance}')

    # We cannot interpret sparse scan dumps generically here without a schema.
    # If scan files exist but are not decoded by this analyzer, stay conservative.
    if scan_paths and not gaps:
        status = FORWARD_OPEN_EVIDENCE_SUPPORTED_BY_SCAN
        confidence = 'low'
        rationale = 'scan artifacts are present, but Phase80 analyzer did not need to override the conservative status for current artifacts'
    else:
        status = FORWARD_OPEN_EVIDENCE_INSUFFICIENT
        confidence = 'none'
        rationale = 'Phase79 artifact has user visual observation context, but no raw scan/front-corridor geometry sufficient to prove forward-open from machine data'

    if front_max is not None and front_max >= 99:
        notes.append('aggregate_front_wedge_local_cost_is_high_or_lethal; this does not prove physical blockage or forward openness by itself')

    return {
        'status': status,
        'confidence': confidence,
        'gaps': gaps,
        'notes': notes,
        'rationale': rationale,
    }


def analyze_artifact(artifact_dir: str | Path, goal_sequence: int = GOAL_SEQUENCE) -> dict[str, Any]:
    artifact_dir = Path(artifact_dir)
    goal_events_path = _artifact_file(artifact_dir, 'goal_events.jsonl')
    local_cost_path = _artifact_file(artifact_dir, 'local_costmap_samples.jsonl')
    feedback_path = _artifact_file(artifact_dir, 'nav2_feedback.jsonl')
    controller_path = _artifact_file(artifact_dir, 'controller_dynamics.jsonl')
    trigger_path = _artifact_file(artifact_dir, 'trigger_detected.json')
    minimal_summary_path = _artifact_file(artifact_dir, 'minimal_field_summary.md')

    goal_events = _read_jsonl(goal_events_path)
    local_cost_rows = _goal_rows(_read_jsonl(local_cost_path), goal_sequence)
    feedback_rows = _goal_rows(_read_jsonl(feedback_path), goal_sequence)
    controller_rows = _read_jsonl(controller_path)
    trigger = _safe_load_json(trigger_path, {})

    dispatch = _find_event(goal_events, 'dispatch', goal_sequence)
    timeout = _find_event(goal_events, 'timeout', goal_sequence)
    cancel = _find_event(goal_events, 'timeout_cancel_result', goal_sequence)
    outcome = timeout or cancel

    target = _vec2((outcome or {}).get('target')) or _vec2((dispatch or {}).get('target')) or _vec2(trigger.get('target'))
    original_target = _vec2((outcome or {}).get('original_target')) or _vec2((dispatch or {}).get('original_target')) or target
    refined_target = _vec2((outcome or {}).get('refined_target')) or _vec2((dispatch or {}).get('refined_target')) or target
    dispatch_pose = _pose((dispatch or {}).get('dispatch_pose')) or _pose((outcome or {}).get('dispatch_pose'))
    terminal_pose = _latest_local_cost_pose(local_cost_rows, goal_sequence) or _last_pose_from_controller(controller_rows, goal_sequence)

    if target and terminal_pose:
        final_xy_error = math.hypot(target[0] - terminal_pose[0], target[1] - terminal_pose[1])
        rel = _relative_target_body_frame(terminal_pose, target)
    else:
        final_xy_error = None
        rel = {
            'forward_component_m': None,
            'lateral_component_m': None,
            'dominant_residual_axis': 'unknown',
            'target_is_nearly_lateral': False,
        }

    recovery_vals = [_number(row.get('number_of_recoveries')) for row in feedback_rows]
    recovery_vals = [v for v in recovery_vals if v is not None]
    recoveries_max = int(max(recovery_vals)) if recovery_vals else None

    event_front_wedge = _number((outcome or {}).get('timeout_front_wedge_cost_max'))
    sample_front_wedge = _max_number(local_cost_rows, [('front_wedge_cost', 'max')])
    front_candidates = [v for v in [event_front_wedge, sample_front_wedge] if v is not None]
    front_wedge_cost_max = max(front_candidates) if front_candidates else None
    latest_front_wedge_clearance = None
    if local_cost_rows:
        latest_front_wedge_clearance = _number(local_cost_rows[-1].get('front_wedge_clearance_m'))
    event_front_wedge_clearance = _number((outcome or {}).get('timeout_front_wedge_clearance_m'))

    near_goal = bool(final_xy_error is not None and final_xy_error <= NEAR_GOAL_DISTANCE_M)
    outside_xy_tolerance = bool(final_xy_error is not None and final_xy_error > XY_GOAL_TOLERANCE_M)
    near_goal_outside_xy_tolerance = bool(near_goal and outside_xy_tolerance)
    lateral_residual = bool(rel.get('dominant_residual_axis') == 'lateral' and rel.get('target_is_nearly_lateral'))

    forward_open = _forward_open_evidence(artifact_dir, local_cost_rows)

    if near_goal_outside_xy_tolerance and lateral_residual and target and terminal_pose:
        classification = CLASSIFICATION
        classification_reason = (
            'Goal2 terminal pose is near the target but outside XY tolerance; target residual is dominated by the robot-body lateral component. '
            'User visual observation reports no obvious physical obstacle directly in front, but machine forward-open evidence remains insufficient.'
        )
    elif target and terminal_pose:
        classification = INSUFFICIENT_EVIDENCE
        classification_reason = 'Phase80 geometry did not meet near-goal lateral residual gates.'
    else:
        classification = INSUFFICIENT_EVIDENCE
        classification_reason = 'Missing target or terminal pose for Phase80 geometry.'

    result = {
        'run_id': RUN_ID,
        'source_artifact_dir': str(artifact_dir),
        'source_files': {
            'goal_events': str(goal_events_path) if goal_events_path else None,
            'local_costmap_samples': str(local_cost_path) if local_cost_path else None,
            'nav2_feedback': str(feedback_path) if feedback_path else None,
            'controller_dynamics': str(controller_path) if controller_path else None,
            'trigger_detected': str(trigger_path) if trigger_path else None,
            'minimal_field_summary': str(minimal_summary_path) if minimal_summary_path else None,
        },
        'goal_sequence': goal_sequence,
        'trigger_event': (outcome or trigger or {}).get('event'),
        'result_reason': (outcome or trigger or {}).get('result_reason'),
        'classification': classification,
        'classification_reason': classification_reason,
        'robot_pose': terminal_pose,
        'terminal_pose': terminal_pose,
        'dispatch_pose': dispatch_pose,
        'target': target,
        'original_target': original_target,
        'refined_target': refined_target,
        'yaw_rad': terminal_pose[2] if terminal_pose else None,
        'relative_target_body_frame': rel,
        'forward_component_m': rel.get('forward_component_m'),
        'lateral_component_m': rel.get('lateral_component_m'),
        'final_xy_error_m': final_xy_error,
        'xy_goal_tolerance_m': XY_GOAL_TOLERANCE_M,
        'near_goal_threshold_m': NEAR_GOAL_DISTANCE_M,
        'near_goal': near_goal,
        'outside_xy_tolerance': outside_xy_tolerance,
        'near_goal_outside_xy_tolerance': near_goal_outside_xy_tolerance,
        'recoveries_max': recoveries_max,
        'front_wedge_cost_max': front_wedge_cost_max,
        'front_wedge_clearance_m': event_front_wedge_clearance if event_front_wedge_clearance is not None else latest_front_wedge_clearance,
        'latest_local_cost_sample': local_cost_rows[-1] if local_cost_rows else None,
        'feedback_sample_count': len(feedback_rows),
        'local_cost_sample_count': len(local_cost_rows),
        'controller_odom_sample_count': len([r for r in _goal_rows(controller_rows, goal_sequence) if r.get('source') == 'odom']),
        'forward_open_evidence_status': forward_open['status'],
        'forward_open_evidence_confidence': forward_open['confidence'],
        'forward_open_evidence_gaps': forward_open['gaps'],
        'forward_open_evidence_notes': forward_open['notes'],
        'forward_open_evidence_rationale': forward_open['rationale'],
        'user_observation': 'front of robot appeared physically open in held Phase79 scene; possible lateral residual to side target',
        'semantic_caveats': [
            'classification is not exit success',
            'classification is not autonomous exploration success',
            'classification is not fallback/terminal acceptance',
            'classification is not a Nav2 parameter final diagnosis',
            'forward-open machine evidence may remain insufficient even when human visual observation suggests physical openness',
        ],
        'guardrails': GUARDRAILS,
        'complete_autonomous_success_claimed': False,
        'exit_success_claimed': False,
        'phase81_entered': False,
    }
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--artifact-dir', type=Path, default=Path('log/phase79_goal2_timeout_bounded_reproduction_handoff'))
    parser.add_argument('--output-json', type=Path, default=Path('log/phase80_goal2_near_goal_forward_open_corridor_diagnostic_classification/phase80_goal2_near_goal_forward_open_corridor_diagnostic_classification.json'))
    parser.add_argument('--goal-sequence', type=int, default=GOAL_SEQUENCE)
    args = parser.parse_args()
    summary = analyze_artifact(args.artifact_dir, goal_sequence=args.goal_sequence)
    args.output_json.parent.mkdir(parents=True, exist_ok=True)
    args.output_json.write_text(json.dumps(summary, indent=2, sort_keys=True), encoding='utf-8')
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
