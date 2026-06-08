#!/usr/bin/env python3
"""Phase79 Goal2 bounded reproduction handoff summary helper.

This helper is intentionally lightweight: it reads live bounded-reproduction
artifacts and writes a minimal field summary for the user screenshot -> ChatGPT
handoff. It does not generate a lengthy human observation report and it does not
change navigation behavior.
"""
from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path
from typing import Any

RUN_ID = 'phase79_goal2_timeout_bounded_reproduction_handoff'
TRIGGER_LABEL = 'goal_timeout / local_cost_risk / recovery loop / near-goal outside tolerance'


def _load_json(path: Path | None, default: Any) -> Any:
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
        payload = row.get('state') if isinstance(row.get('state'), dict) else row
        if isinstance(payload, dict):
            merged = dict(payload)
            merged.setdefault('_elapsed_sec', row.get('elapsed_sec'))
            merged.setdefault('_wall_time', row.get('wall_time'))
            rows.append(merged)
    return rows


def _num(value: Any) -> float | None:
    try:
        if value is None:
            return None
        out = float(value)
        if math.isnan(out) or math.isinf(out):
            return None
        return out
    except Exception:
        return None


def _last_pose(controller_rows: list[dict[str, Any]], fallback: Any = None) -> list[float] | None:
    for row in reversed(controller_rows):
        if row.get('source') == 'odom' and _num(row.get('x')) is not None and _num(row.get('y')) is not None:
            return [float(row['x']), float(row['y']), float(row.get('yaw') or 0.0)]
    if isinstance(fallback, list) and len(fallback) >= 2:
        return [float(fallback[0]), float(fallback[1]), float(fallback[2] if len(fallback) > 2 else 0.0)]
    return None


def _max_recoveries(feedback_rows: list[dict[str, Any]]) -> int:
    values = []
    for row in feedback_rows:
        try:
            values.append(int(row.get('number_of_recoveries') or 0))
        except Exception:
            pass
    return max(values or [0])


def _max_front_wedge(local_rows: list[dict[str, Any]], timeline_rows: list[dict[str, Any]]) -> int | None:
    values: list[int] = []
    for row in local_rows + timeline_rows:
        for container_key in ('front_wedge_cost', 'local_cost_footprint_front_wedge_summary'):
            container = row.get(container_key)
            if isinstance(container, dict):
                for key in ('max', 'front_wedge_cost_max'):
                    val = _num(container.get(key))
                    if val is not None:
                        values.append(int(val))
        val = _num(row.get('timeout_front_wedge_cost_max'))
        if val is not None:
            values.append(int(val))
    return max(values) if values else None


def _goal2_events(events: list[dict[str, Any]]) -> list[dict[str, Any]]:
    out = []
    for event in events:
        try:
            if int(event.get('goal_sequence') or 0) == 2:
                out.append(event)
        except Exception:
            continue
    return out


def build_summary(args: argparse.Namespace) -> dict[str, Any]:
    artifact_dir = Path(args.artifact_dir).resolve()
    phase77_event = _load_json(Path(args.phase77_event) if args.phase77_event else None, {})
    phase75_summary = _load_json(Path(args.phase75_json) if args.phase75_json else None, {})
    goal_events = _read_jsonl(Path(args.goal_events)) if args.goal_events else []
    feedback_rows = _read_jsonl(Path(args.nav2_feedback)) if args.nav2_feedback else []
    local_rows = _read_jsonl(Path(args.local_costmap_samples)) if args.local_costmap_samples else []
    timeline_rows = _read_jsonl(Path(args.runtime_timeline)) if args.runtime_timeline else []
    controller_rows = _read_jsonl(Path(args.controller_dynamics)) if args.controller_dynamics else []

    goal2 = _goal2_events(goal_events)
    dispatch = next((e for e in goal2 if e.get('event') == 'dispatch'), None)
    timeout = next((e for e in goal2 if e.get('event') in {'timeout', 'timeout_cancel_result'}), None)
    target = (dispatch or timeout or phase77_event or phase75_summary).get('target')
    if target is None:
        target = phase77_event.get('dispatch_target') or phase75_summary.get('goal2_actual_target')
    pose = _last_pose(controller_rows, phase77_event.get('final_pose') or phase75_summary.get('final_pose'))
    recoveries = _max_recoveries(feedback_rows)
    wedge_max = _max_front_wedge(local_rows, timeline_rows)
    final_error = phase77_event.get('final_xy_error_m') or phase75_summary.get('goal_tolerance_band', {}).get('final_xy_error_m')

    trigger_seen = bool(timeout or recoveries > 0 or (wedge_max is not None and wedge_max >= 70))
    if args.status:
        status = args.status
    elif trigger_seen:
        status = 'SCENE_HELD_WAITING_FOR_USER_SCREENSHOT'
    else:
        status = 'REPRODUCTION_TRIGGER_NOT_REACHED'

    evidence_parts = []
    if timeout:
        evidence_parts.append(f"goal2 {timeout.get('event')} result_reason={timeout.get('result_reason')}")
    if recoveries > 0:
        evidence_parts.append(f'number_of_recoveries_max={recoveries}')
    if wedge_max is not None:
        evidence_parts.append(f'front_wedge_cost_max={wedge_max}')
    if final_error is not None:
        evidence_parts.append(f'final_xy_error_m={final_error}')
    if not evidence_parts:
        evidence_parts.append('bounded reproduction artifacts created; trigger evidence pending live observation')

    return {
        'run_id': RUN_ID,
        'status': status,
        'artifact_dir': str(artifact_dir),
        'trigger': TRIGGER_LABEL,
        'robot_pose': pose,
        'target': target,
        'key_evidence': '; '.join(str(x) for x in evidence_parts),
        'goal2_event_count': len(goal2),
        'goal2_timeout_observed': bool(timeout),
        'number_of_recoveries_max': recoveries,
        'front_wedge_cost_max': wedge_max,
        'handoff': 'automatic bounded reproduction -> user screenshot -> ChatGPT discussion -> Hermes executes confirmed plan',
        'guardrails': [
            'No navigation strategy changed',
            'No Nav2/MPPI/controller tuning',
            'No autonomous exploration success claimed',
            'No exit success claimed',
            'algorithm repair phase not entered',
            'Phase80 not entered',
        ],
        'updated_at_wall_time': time.time(),
    }


def write_outputs(summary: dict[str, Any], status_json: Path, minimal_md: Path) -> None:
    status_json.parent.mkdir(parents=True, exist_ok=True)
    minimal_md.parent.mkdir(parents=True, exist_ok=True)
    status_json.write_text(json.dumps(summary, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    pose = summary.get('robot_pose')
    target = summary.get('target')
    md = f"""# Phase79 Goal2 bounded reproduction handoff - minimal field summary

Status: {summary['status']}

Trigger: {summary['trigger']}

Run/artifacts: {summary['artifact_dir']}

Robot pose: {pose if pose is not None else 'pending live sample'}

Target: {target if target is not None else 'pending Goal2 dispatch sample'}

Key evidence: {summary['key_evidence']}

Screenshot suggestions (1-4 only):
- Gazebo wide: show Tugbot pose in the narrow corridor near Goal2.
- RViz local costmap / footprint / front wedge: show robot footprint and cost wedge around the nose.
- Goal tolerance: show Goal2 target and whether the robot is near but outside tolerance.
- Recovery loop: show behavior/recovery status if RViz or logs indicate repeated recovery.

No lengthy human observation report is required. Please send these screenshots and a brief judgment to ChatGPT discussion, then let Hermes execute the confirmed plan.
"""
    minimal_md.write_text(md, encoding='utf-8')


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description='Write Phase79 lightweight handoff summary.')
    parser.add_argument('--artifact-dir', required=True)
    parser.add_argument('--phase77-event')
    parser.add_argument('--phase75-json')
    parser.add_argument('--goal-events')
    parser.add_argument('--nav2-feedback')
    parser.add_argument('--local-costmap-samples')
    parser.add_argument('--runtime-timeline')
    parser.add_argument('--controller-dynamics')
    parser.add_argument('--status')
    parser.add_argument('--status-json', required=True)
    parser.add_argument('--minimal-md', required=True)
    args = parser.parse_args(argv)
    summary = build_summary(args)
    write_outputs(summary, Path(args.status_json), Path(args.minimal_md))
    print(json.dumps({'status': summary['status'], 'minimal_md': args.minimal_md}, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
