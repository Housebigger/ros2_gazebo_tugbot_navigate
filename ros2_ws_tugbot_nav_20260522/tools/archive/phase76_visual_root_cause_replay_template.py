#!/usr/bin/env python3
"""Phase76 Obstacle-Triggered Visual Root-Cause Workflow template.

This reusable tool converts a single obstacle/navigation event into a visual
root-cause replay package: replay command, RViz/Gazebo marker overlay specs,
screenshot checklist, and a human observation report template.

Guardrails:
- workflow/template only; no navigation strategy edits
- no Nav2/MPPI/controller tuning
- no inflation/robot footprint radius/clearance radius/map threshold tuning
- no branch scoring change
- no centerline gate change
- no directional readiness override change
- no fallback/terminal acceptance change
- no autonomous exploration success claim
- no exit success claim
- no long-running experiment requirement

Trigger tokens intentionally covered by this template:
- goal_timeout
- FAILED_EXHAUSTED
- no_candidate
- local_cost_risk
- recovery_count>0
- near-goal outside tolerance
- re-dispatch readiness blocked

Marker tokens intentionally covered by this template:
- dispatch_target_marker
- original_target_marker
- refined_target_marker
- robot_trajectory_marker
- terminal_pose_marker
- footprint_marker
- front_wedge_marker
- local_cost_high_lethal_marker
- nearest_wall_marker
- corridor_centerline_marker
- goal_tolerance_circle_marker

Screenshot moments intentionally covered by this template:
- dispatch
- near_goal
- recovery
- timeout
- final_pose
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

RUN_ID = 'phase76_obstacle_triggered_visual_root_cause_workflow'
PHASE = 'Phase76 Obstacle-Triggered Visual Root-Cause Workflow'
DEFAULT_MARKER_TOPIC = '/phase76/visual_root_cause_markers'
DEFAULT_OVERLAY_LAUNCH = 'phase76_visual_root_cause_overlay.launch.py'

TRIGGER_ORDER = [
    'goal_timeout',
    'FAILED_EXHAUSTED',
    'no_candidate',
    'local_cost_risk',
    'recovery_count>0',
    'near-goal outside tolerance',
    're-dispatch readiness blocked',
]

SCREENSHOT_MOMENTS = [
    {
        'id': 'dispatch',
        'title': 'Dispatch moment',
        'required_overlay': ['dispatch_target_marker', 'original_target_marker', 'refined_target_marker'],
        'check': 'Capture robot pose, dispatch target, original/refined target relationship, and current local costmap.',
    },
    {
        'id': 'near_goal',
        'title': 'Near-goal outside tolerance moment',
        'required_overlay': ['goal_tolerance_circle_marker', 'footprint_marker', 'front_wedge_marker'],
        'check': 'Capture final approach when robot is near the target but outside XY tolerance.',
    },
    {
        'id': 'recovery',
        'title': 'Nav2 recovery moment',
        'required_overlay': ['robot_trajectory_marker', 'local_cost_high_lethal_marker', 'nearest_wall_marker'],
        'check': 'Capture the first recovery/progress-failure loop with trajectory, high/lethal cost, and nearest wall evidence.',
    },
    {
        'id': 'timeout',
        'title': 'Timeout / exhausted moment',
        'required_overlay': ['terminal_pose_marker', 'footprint_marker', 'front_wedge_marker'],
        'check': 'Capture timeout or FAILED_EXHAUSTED state with terminal footprint and front wedge.',
    },
    {
        'id': 'final_pose',
        'title': 'Final pose after stop/cancel',
        'required_overlay': ['terminal_pose_marker', 'goal_tolerance_circle_marker', 'corridor_centerline_marker'],
        'check': 'Capture final pose, target tolerance circle, and corridor centerline after the run stops.',
    },
]

GUARDRAILS = {
    'scope': 'workflow/template only',
    'navigation_strategy_changed': False,
    'nav2_mppi_controller_tuned': False,
    'inflation_or_radius_or_clearance_or_map_threshold_tuned': False,
    'branch_scoring_changed': False,
    'centerline_gate_changed': False,
    'directional_readiness_override_changed': False,
    'fallback_or_terminal_acceptance_changed': False,
    'autonomous_success_claimed': False,
    'exit_success_claimed': False,
    'long_running_experiment_required': False,
}


def _number(value: Any) -> float | None:
    try:
        out = float(value)
    except (TypeError, ValueError):
        return None
    return out if math.isfinite(out) else None


def _as_pose(value: Any, default: list[float]) -> list[float]:
    if isinstance(value, (list, tuple)) and len(value) >= 2:
        x = _number(value[0])
        y = _number(value[1])
        yaw = _number(value[2]) if len(value) > 2 else 0.0
        if x is not None and y is not None:
            return [x, y, yaw or 0.0]
    if isinstance(value, dict):
        x = _number(value.get('x') or value.get('x_m'))
        y = _number(value.get('y') or value.get('y_m'))
        yaw = _number(value.get('yaw') or value.get('yaw_rad')) or 0.0
        if x is not None and y is not None:
            return [x, y, yaw]
    return list(default)


def _as_xy(value: Any, default: list[float]) -> list[float]:
    pose = _as_pose(value, [default[0], default[1], 0.0])
    return [pose[0], pose[1]]


def _dist_xy(a: list[float], b: list[float]) -> float:
    return math.hypot(float(a[0]) - float(b[0]), float(a[1]) - float(b[1]))


def _nested_get(data: dict[str, Any], *keys: str) -> Any:
    cur: Any = data
    for key in keys:
        if not isinstance(cur, dict):
            return None
        cur = cur.get(key)
    return cur


def _max_numeric_from_keys(data: dict[str, Any], keys: list[str]) -> float:
    values: list[float] = []
    for key in keys:
        value = data.get(key)
        if isinstance(value, dict):
            for nested in value.values():
                num = _number(nested)
                if num is not None:
                    values.append(num)
        else:
            num = _number(value)
            if num is not None:
                values.append(num)
    return max(values) if values else 0.0


def detect_visual_trigger(event: dict[str, Any]) -> dict[str, Any]:
    """Select the visual root-cause trigger class without changing strategy."""
    text_fields = ' '.join(
        str(event.get(key, ''))
        for key in ['event', 'result_reason', 'last_failure_reason', 'classification', 'mode', 'terminal_reason']
    )
    matched: list[str] = []
    if 'goal_timeout' in text_fields or event.get('event') == 'timeout':
        matched.append('goal_timeout')
    if 'FAILED_EXHAUSTED' in text_fields or event.get('failed_exhausted') is True:
        matched.append('FAILED_EXHAUSTED')
    if event.get('no_candidate') is True or event.get('last_candidate_count') == 0 or event.get('candidate_after_filter_count') == 0:
        matched.append('no_candidate')
    cost_score = _max_numeric_from_keys(event, [
        'local_cost_risk_score',
        'timeout_front_wedge_cost_max',
        'timeout_footprint_cost_max',
        'timeout_footprint_lethal_cell_count',
        'front_wedge_cost_max',
        'footprint_cost_max',
    ])
    if event.get('local_cost_risk') is True or cost_score >= 90.0:
        matched.append('local_cost_risk')
    recovery_count = max(
        _number(event.get('number_of_recoveries')) or 0.0,
        _number(event.get('recovery_count')) or 0.0,
        _number(event.get('number_of_recoveries_max')) or 0.0,
        _number(_nested_get(event, 'nav2_recovery_summary', 'number_of_recoveries_max')) or 0.0,
    )
    if recovery_count > 0:
        matched.append('recovery_count>0')
    if event.get('near_goal_outside_tolerance') is True or event.get('near_goal_but_outside_xy_tolerance') is True:
        matched.append('near-goal outside tolerance')
    else:
        target = _as_xy(event.get('target'), [0.0, 0.0])
        final_pose = _as_pose(event.get('final_pose') or event.get('terminal_pose'), [999.0, 999.0, 0.0])
        tolerance = _number(event.get('xy_goal_tolerance_m')) or 0.25
        near_band = _number(event.get('near_goal_visual_band_m')) or max(0.75, tolerance * 2.0)
        error = _dist_xy(target, final_pose)
        if tolerance < error <= near_band:
            matched.append('near-goal outside tolerance')
    readiness_blocked = event.get('re_dispatch_readiness_blocked') is True or event.get('redispatch_readiness_blocked') is True
    blocking = event.get('dispatch_readiness_blocking_reasons') or event.get('blocking_reasons')
    if readiness_blocked or (isinstance(blocking, list) and bool(blocking)):
        matched.append('re-dispatch readiness blocked')
    selected = next((token for token in TRIGGER_ORDER if token in matched), 'manual_visual_review')
    return {
        'selected': selected,
        'matched': matched,
        'requires_visual_replay': bool(matched),
        'policy': 'start visual root-cause replay before further algorithm edits when obstacle trigger appears',
    }


def _front_wedge_points(pose: list[float], length_m: float = 0.80, half_angle_rad: float = 0.45) -> list[list[float]]:
    x, y, yaw = pose
    return [
        [x, y, 0.08],
        [x + math.cos(yaw + half_angle_rad) * length_m, y + math.sin(yaw + half_angle_rad) * length_m, 0.08],
        [x + math.cos(yaw) * length_m, y + math.sin(yaw) * length_m, 0.08],
        [x + math.cos(yaw - half_angle_rad) * length_m, y + math.sin(yaw - half_angle_rad) * length_m, 0.08],
        [x, y, 0.08],
    ]


def build_marker_specs(event: dict[str, Any]) -> list[dict[str, Any]]:
    target = _as_xy(event.get('target') or event.get('dispatch_target'), [0.0, 0.0])
    original = _as_xy(event.get('original_target'), target)
    refined = _as_xy(event.get('refined_target'), target)
    dispatch_pose = _as_pose(event.get('dispatch_pose'), [target[0], target[1] - 1.0, 0.0])
    terminal_pose = _as_pose(event.get('terminal_pose') or event.get('final_pose'), dispatch_pose)
    trajectory = event.get('trajectory_points') or event.get('odom_trajectory') or [dispatch_pose, terminal_pose]
    if not isinstance(trajectory, list) or len(trajectory) < 2:
        trajectory = [dispatch_pose, terminal_pose]
    trajectory_points = [[float(_as_pose(p, dispatch_pose)[0]), float(_as_pose(p, dispatch_pose)[1]), 0.07] for p in trajectory]
    nearest_wall = _as_xy(event.get('nearest_wall_point') or event.get('nearest_wall'), terminal_pose[:2])
    centerline = event.get('corridor_centerline_points') or [dispatch_pose[:2], target]
    if not isinstance(centerline, list) or len(centerline) < 2:
        centerline = [dispatch_pose[:2], target]
    centerline_points = [[_as_xy(p, target)[0], _as_xy(p, target)[1], 0.045] for p in centerline]
    tolerance = _number(event.get('xy_goal_tolerance_m')) or 0.25
    footprint_radius = _number(event.get('visual_footprint_radius_m')) or 0.35
    high_cost_point = _as_xy(event.get('local_cost_high_lethal_point') or event.get('risk_marker_pose'), terminal_pose[:2])

    return [
        {
            'name': 'dispatch_target_marker',
            'type': 'sphere',
            'frame_id': 'map',
            'position': [target[0], target[1], 0.12],
            'scale': [0.22, 0.22, 0.22],
            'color': [1.0, 0.05, 0.05, 0.95],
            'purpose': 'dispatch target actually sent to Nav2',
        },
        {
            'name': 'original_target_marker',
            'type': 'sphere',
            'frame_id': 'map',
            'position': [original[0], original[1], 0.18],
            'scale': [0.16, 0.16, 0.16],
            'color': [1.0, 1.0, 0.0, 0.90],
            'purpose': 'original target before any recorded refinement, visual comparison only',
        },
        {
            'name': 'refined_target_marker',
            'type': 'sphere',
            'frame_id': 'map',
            'position': [refined[0], refined[1], 0.24],
            'scale': [0.16, 0.16, 0.16],
            'color': [0.0, 0.8, 1.0, 0.90],
            'purpose': 'refined target if present; does not imply a new centerline-gate behavior',
        },
        {
            'name': 'robot_trajectory_marker',
            'type': 'line_strip',
            'frame_id': 'map',
            'points': trajectory_points,
            'scale': [0.035, 0.0, 0.0],
            'color': [0.2, 1.0, 0.2, 0.95],
            'purpose': 'robot trajectory from dispatch to terminal pose',
        },
        {
            'name': 'terminal_pose_marker',
            'type': 'arrow',
            'frame_id': 'map',
            'position': [terminal_pose[0], terminal_pose[1], 0.10],
            'yaw': terminal_pose[2],
            'scale': [0.62, 0.09, 0.09],
            'color': [1.0, 0.65, 0.0, 0.95],
            'purpose': 'terminal/final pose at timeout, exhausted, or manual stop',
        },
        {
            'name': 'footprint_marker',
            'type': 'circle',
            'frame_id': 'map',
            'position': [terminal_pose[0], terminal_pose[1], 0.055],
            'radius_m': footprint_radius,
            'color': [1.0, 0.65, 0.0, 0.95],
            'purpose': 'visual footprint envelope at terminal pose',
        },
        {
            'name': 'front_wedge_marker',
            'type': 'line_strip',
            'frame_id': 'map',
            'points': _front_wedge_points(terminal_pose),
            'scale': [0.035, 0.0, 0.0],
            'color': [1.0, 0.2, 0.0, 0.95],
            'purpose': 'front wedge at terminal pose for recovery-loop observation',
        },
        {
            'name': 'local_cost_high_lethal_marker',
            'type': 'sphere',
            'frame_id': 'map',
            'position': [high_cost_point[0], high_cost_point[1], 0.20],
            'scale': [0.20, 0.20, 0.20],
            'color': [1.0, 0.0, 1.0, 0.95],
            'purpose': 'representative local cost high/lethal point near footprint or front wedge',
        },
        {
            'name': 'nearest_wall_marker',
            'type': 'sphere',
            'frame_id': 'map',
            'position': [nearest_wall[0], nearest_wall[1], 0.16],
            'scale': [0.18, 0.18, 0.18],
            'color': [0.8, 0.2, 1.0, 0.95],
            'purpose': 'nearest wall or wall-side clearance point from diagnostics',
        },
        {
            'name': 'corridor_centerline_marker',
            'type': 'line_strip',
            'frame_id': 'map',
            'points': centerline_points,
            'scale': [0.025, 0.0, 0.0],
            'color': [0.2, 0.7, 1.0, 0.90],
            'purpose': 'corridor centerline reference for visual diagnosis only',
        },
        {
            'name': 'goal_tolerance_circle_marker',
            'type': 'circle',
            'frame_id': 'map',
            'position': [target[0], target[1], 0.052],
            'radius_m': tolerance,
            'color': [0.3, 0.3, 1.0, 0.90],
            'purpose': 'goal tolerance circle around dispatch target',
        },
    ]


def build_screenshot_checklist(plan: dict[str, Any]) -> list[dict[str, Any]]:
    trigger = plan.get('trigger', {}).get('selected')
    return [
        {
            **moment,
            'trigger_context': trigger,
            'required': True,
            'capture_filename_hint': f"{plan.get('run_id', RUN_ID)}_{moment['id']}.png",
        }
        for moment in SCREENSHOT_MOMENTS
    ]


def build_replay_command(artifact_dir: Path, run_id: str) -> str:
    plan_json = artifact_dir / f'{run_id}_visual_root_cause_plan.json'
    return (
        'ros2 launch tugbot_bringup phase76_visual_root_cause_overlay.launch.py '
        f'phase76_plan:={plan_json} '
        'headless:=false use_rviz:=true '
        f'marker_topic:={DEFAULT_MARKER_TOPIC}'
    )


def build_visual_root_cause_plan(event: dict[str, Any], artifact_dir: str | Path, run_id: str = RUN_ID) -> dict[str, Any]:
    artifact_path = Path(artifact_dir)
    trigger = detect_visual_trigger(event)
    plan: dict[str, Any] = {
        'phase': PHASE,
        'run_id': run_id,
        'artifact_dir': str(artifact_path),
        'trigger': trigger,
        'marker_topic': DEFAULT_MARKER_TOPIC,
        'overlay_launch_template': DEFAULT_OVERLAY_LAUNCH,
        'replay_command': build_replay_command(artifact_path, run_id),
        'marker_specs': build_marker_specs(event),
        'guardrails': dict(GUARDRAILS),
        'source_event': event,
        'operator_note': 'When a trigger appears, run or stage visual replay before further algorithm edits.',
    }
    plan['screenshot_checklist'] = build_screenshot_checklist(plan)
    return plan


def render_screenshot_checklist(plan: dict[str, Any]) -> str:
    lines = [
        f"# Phase76 Screenshot Checklist: {plan.get('run_id', RUN_ID)}",
        '',
        f"Marker topic: `{plan.get('marker_topic', DEFAULT_MARKER_TOPIC)}`",
        f"Trigger: `{plan.get('trigger', {}).get('selected')}`",
        '',
        'Required screenshot moments:',
    ]
    for item in plan.get('screenshot_checklist', []):
        overlays = ', '.join(item.get('required_overlay', []))
        lines.extend([
            f"- [ ] {item['id']} — {item['title']}",
            f"  - filename hint: `{item['capture_filename_hint']}`",
            f"  - overlays: {overlays}",
            f"  - check: {item['check']}",
        ])
    lines.extend([
        '',
        'Do not mark this checklist as autonomous exploration success or exit success.',
        '',
    ])
    return '\n'.join(lines)


def render_human_observation_report_template(plan: dict[str, Any]) -> str:
    matched = ', '.join(plan.get('trigger', {}).get('matched', [])) or 'manual_visual_review'
    return f"""# Phase76 人工观察报告模板: {plan.get('run_id', RUN_ID)}

## Scope

- Workflow: Phase76 Obstacle-Triggered Visual Root-Cause Workflow.
- Trigger selected: `{plan.get('trigger', {}).get('selected')}`.
- Matched trigger evidence: `{matched}`.
- This report is for RViz/Gazebo visual root-cause observation only.
- 不调 Nav2/MPPI/controller；不改导航策略；不声明 autonomous exploration success；不声明 exit success。

## Replay command

```bash
{plan.get('replay_command')}
```

## Marker overlay items to verify

- dispatch target: `dispatch_target_marker`
- original/refined target: `original_target_marker`, `refined_target_marker`
- robot trajectory: `robot_trajectory_marker`
- terminal pose / final pose: `terminal_pose_marker`
- footprint: `footprint_marker`
- front wedge: `front_wedge_marker`
- local cost high/lethal: `local_cost_high_lethal_marker`
- nearest wall: `nearest_wall_marker`
- corridor centerline: `corridor_centerline_marker`
- goal tolerance circle: `goal_tolerance_circle_marker`

## Screenshot evidence checklist

{render_screenshot_checklist(plan)}

## Human observations

1. Dispatch moment: what is visible around dispatch target, original target, refined target, footprint, and local costmap?
2. Near-goal moment: is the robot outside the goal tolerance circle, and which side is constrained?
3. Recovery moment: does the robot/front wedge repeatedly face local cost high/lethal cells or nearest wall?
4. Timeout/exhausted moment: is the terminal footprint or front wedge visibly blocked?
5. Final pose moment: does final pose align with corridor centerline, target tolerance circle, or wall-side squeeze?

## Conservative conclusion

- Visual classification: `VISUAL_EVIDENCE_PENDING_HUMAN_OBSERVATION`
- Navigation strategy changed: `false`
- Autonomous exploration success claimed: `false`
- Exit success claimed: `false`
"""


def _load_event_json(path: Path) -> dict[str, Any]:
    data = json.loads(path.read_text(encoding='utf-8', errors='replace'))
    if isinstance(data, dict):
        return data
    raise ValueError(f'event JSON must be an object: {path}')


def _write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + '\n', encoding='utf-8')


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=PHASE)
    parser.add_argument('--event-json', required=True, help='Single JSON object describing the trigger event')
    parser.add_argument('--artifact-dir', required=True, help='Directory where plan/checklist/report artifacts are written')
    parser.add_argument('--run-id', default=RUN_ID)
    args = parser.parse_args(argv)

    event = _load_event_json(Path(args.event_json))
    artifact_dir = Path(args.artifact_dir)
    plan = build_visual_root_cause_plan(event, artifact_dir=artifact_dir, run_id=args.run_id)

    _write_json(artifact_dir / f'{args.run_id}_visual_root_cause_plan.json', plan)
    _write_json(artifact_dir / f'{args.run_id}_marker_specs.json', plan['marker_specs'])
    (artifact_dir / f'{args.run_id}_screenshot_checklist.md').write_text(render_screenshot_checklist(plan), encoding='utf-8')
    (artifact_dir / f'{args.run_id}_human_observation_report_template.md').write_text(
        render_human_observation_report_template(plan),
        encoding='utf-8',
    )
    print(json.dumps({
        'run_id': args.run_id,
        'trigger': plan['trigger'],
        'plan': str(artifact_dir / f'{args.run_id}_visual_root_cause_plan.json'),
        'checklist': str(artifact_dir / f'{args.run_id}_screenshot_checklist.md'),
        'report_template': str(artifact_dir / f'{args.run_id}_human_observation_report_template.md'),
        'marker_specs': str(artifact_dir / f'{args.run_id}_marker_specs.json'),
        'replay_command': plan['replay_command'],
    }, indent=2, sort_keys=True))
    return 0


if __name__ == '__main__':  # pragma: no cover
    raise SystemExit(main())
