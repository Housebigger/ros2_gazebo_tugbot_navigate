#!/usr/bin/env python3
"""Phase139 instrumented second-step contract runtime verification.

Scope guardrails:
- Use the visible-stack ingress + handoff + maze_explorer chain inherited from Phase136.
- Send only the explicit inner-ingress goal: map,x=2.0,y=0.0,yaw=0.0.
- Start maze_explorer with max_goals:=2 after handoff readiness.
- Allow first literal goal_kind=corridor_alignment_staging and exactly one second-step goal_kind=explore.
- Stop at accepted/result/diagnostic boundary; never allow a third goal or full exploration.
- Do not tune Nav2/MPPI/controller/goal checker/config and do not change exploration behavior.
- Do not claim autonomous exploration success, exit success, or Phase127 timeout repair.
"""
from __future__ import annotations

import argparse
import importlib.util
import json
import time
from argparse import Namespace
from pathlib import Path
from typing import Any

PHASE = 'Phase139'
MODE = 'instrumented_second_step_contract_runtime_verification'
LOCKED_GOAL = {'frame_id': 'map', 'x': 2.0, 'y': 0.0, 'yaw': 0.0}
LOG_DIR = Path(__file__).resolve().parents[1] / 'log' / 'phase139_instrumented_second_step_contract_runtime_verification'

CLASSIFICATIONS = [
    'SECOND_STEP_CONTRACT_VERIFIED_EXPLORE_ACCEPTED_STOP',
    'SECOND_STEP_CONTRACT_VERIFIED_EXPLORE_RESULT_SUCCEEDED_STOP',
    'SECOND_STEP_CONTRACT_VERIFIED_EXPLORE_TIMEOUT_DIAGNOSTIC_FAIL',
    'SECOND_STEP_CONTRACT_VERIFIED_EXPLORE_REJECTED_DIAGNOSTIC_FAIL',
    'SECOND_STEP_CONTRACT_STILL_AMBIGUOUS',
    'STAGING_SUCCEEDED_SECOND_STEP_NOT_READY_FAIL_CLOSED',
]

FORBIDDEN_ACTIONS = {
    'third_goal': False,
    'full_exploration': False,
    'manual_goal1_carry_over_branch_centerline_fallback_terminal_exit_goal': False,
    'nav2_mppi_controller_goal_checker_config_tuning': False,
    'exploration_strategy_branch_scoring_centerline_fallback_terminal_acceptance_change': False,
    'direct_staging_disablement': False,
    'autonomous_or_exit_success_claim': False,
    'phase127_timeout_fixed_claim': False,
}


def _load_module(path: Path, name: str) -> Any:
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f'cannot import {name} from {path}')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


_p136 = _load_module(Path(__file__).resolve().parent / 'run_phase136_bounded_second_step_explore_after_staging_smoke.py', 'phase136_runner_for_phase139')
_analyzer = _load_module(Path(__file__).resolve().parent / 'analyze_phase139_instrumented_second_step_contract_runtime_verification.py', 'phase139_analyzer_for_runner')


def _dict(value: Any) -> dict[str, Any]:
    return value if isinstance(value, dict) else {}


def _list(value: Any) -> list[Any]:
    return value if isinstance(value, list) else []


def _read_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding='utf-8'))


def write_json(path: Path, data: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2, sort_keys=True) + '\n', encoding='utf-8')


def _with_boundary_claims(artifact: dict[str, Any]) -> dict[str, Any]:
    claims = dict(_dict(artifact.get('claims')))
    claims.update({
        'second_step_is_autonomous_exploration_success': False,
        'second_step_is_exit_success': False,
        'mixed_with_phase127_timeout_local_cost_replay': False,
        'phase127_timeout_fixed': False,
        'autonomous_exploration_success': False,
        'exit_success': False,
    })
    artifact['claims'] = claims
    artifact['forbidden_actions'] = dict(FORBIDDEN_ACTIONS)
    return artifact


def build_phase139_artifact(*, run_id: str, source_artifact: dict[str, Any]) -> dict[str, Any]:
    """Normalize a Phase136-style runtime artifact into the Phase139 contract artifact."""
    base = dict(source_artifact)
    base['phase136_source_phase'] = source_artifact.get('phase')
    base['phase136_source_mode'] = source_artifact.get('mode')
    base['phase'] = PHASE
    base['run_id'] = run_id
    base['mode'] = MODE
    base['created_wall_time_sec'] = time.time()
    base['locked_goal'] = dict(LOCKED_GOAL)
    base['runtime_scope'] = {
        'visible_stack': True,
        'explicit_inner_ingress_goal': dict(LOCKED_GOAL),
        'maze_explorer_max_goals': 2,
        'first_literal_goal_kind_allowed': 'corridor_alignment_staging',
        'only_second_step_goal_kind_allowed': 'explore',
        'third_goal_forbidden': True,
        'full_exploration_forbidden': True,
        'nav2_config_tuning_forbidden': True,
        'strategy_change_forbidden': True,
    }
    _with_boundary_claims(base)
    analysis = _analyzer.analyze_artifact(base)
    base['analysis'] = analysis
    base['classification'] = analysis['classification']
    base['contract_checks'] = analysis['contract_checks']
    base['runtime_contract'] = analysis['runtime_contract']
    base['valid'] = analysis['valid']
    base['notes'] = _list(base.get('notes')) + [
        'Phase139 verifies Phase138 serialization fields in a bounded visible-stack runtime artifact.',
        'Second-step accepted/succeeded is not autonomous exploration success and not exit success.',
        'Second-step result does not repair or reinterpret Phase127 FIRST_GOAL_TIMEOUT_LOCAL_COST_BLOCKED.',
        'Phase139 stops before Phase140 and before any third goal/full exploration.',
    ]
    return base


def _phase136_args(args: argparse.Namespace, source_output: Path) -> Namespace:
    return Namespace(
        output=source_output,
        phase120_output=args.phase120_output,
        preflight_output=args.preflight_output,
        phase120_artifact=args.phase120_artifact,
        run_id=args.run_id + '_source_phase136',
        launch_log_path=args.launch_log_path,
        preflight_timeout_sec=args.preflight_timeout_sec,
        tf_stability_window_sec=args.tf_stability_window_sec,
        sample_period_sec=args.sample_period_sec,
        startup_grace_sec=args.startup_grace_sec,
        action_server_wait_sec=args.action_server_wait_sec,
        bounded_goal_result_wait_sec=args.bounded_goal_result_wait_sec,
        readiness_wait_timeout_sec=args.readiness_wait_timeout_sec,
        readiness_wait_sample_period_sec=args.readiness_wait_sample_period_sec,
        handoff_collection_sec=args.handoff_collection_sec,
        pose_tolerance_m=args.pose_tolerance_m,
        yaw_tolerance_rad=args.yaw_tolerance_rad,
        max_costmap_age_sec=args.max_costmap_age_sec,
        max_scan_age_sec=args.max_scan_age_sec,
        max_tf_age_sec=args.max_tf_age_sec,
        second_step_observation_sec=args.second_step_observation_sec,
    )


def run_phase139(args: argparse.Namespace) -> dict[str, Any]:
    args.output.parent.mkdir(parents=True, exist_ok=True)
    if args.source_artifact:
        source_artifact = _read_json(args.source_artifact)
        source_path = args.source_artifact
    else:
        source_path = args.output.with_name(args.output.stem + '_source_phase136.json')
        source_artifact = _p136.run_phase136(_phase136_args(args, source_path))
        write_json(source_path, source_artifact)
    artifact = build_phase139_artifact(run_id=args.run_id, source_artifact=source_artifact)
    artifact['phase136_source_artifact_path'] = str(source_path)
    write_json(args.output, artifact)
    return artifact


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=LOG_DIR / 'phase139_instrumented_second_step_contract_runtime_verification.json')
    parser.add_argument('--source-artifact', type=Path, help='test/debug only: analyze an existing runtime artifact instead of launching visible stack')
    parser.add_argument('--phase120-output', type=Path, default=LOG_DIR / 'phase139_phase120_inner_ingress.json')
    parser.add_argument('--preflight-output', type=Path, default=LOG_DIR / 'phase139_preflight.json')
    parser.add_argument('--phase120-artifact', type=Path, help='test/debug only: reuse existing Phase120 artifact instead of dispatching')
    parser.add_argument('--run-id', default='phase139_instrumented_second_step_contract_runtime_verification')
    parser.add_argument('--launch-log-path')
    parser.add_argument('--preflight-timeout-sec', type=float, default=35.0)
    parser.add_argument('--tf-stability-window-sec', type=float, default=2.0)
    parser.add_argument('--sample-period-sec', type=float, default=0.5)
    parser.add_argument('--startup-grace-sec', type=float, default=2.0)
    parser.add_argument('--action-server-wait-sec', type=float, default=10.0)
    parser.add_argument('--bounded-goal-result-wait-sec', type=float, default=45.0)
    parser.add_argument('--readiness-wait-timeout-sec', type=float, default=60.0)
    parser.add_argument('--readiness-wait-sample-period-sec', type=float, default=1.0)
    parser.add_argument('--handoff-collection-sec', type=float, default=4.0)
    parser.add_argument('--pose-tolerance-m', type=float, default=0.35)
    parser.add_argument('--yaw-tolerance-rad', type=float, default=0.35)
    parser.add_argument('--max-costmap-age-sec', type=float, default=5.0)
    parser.add_argument('--max-scan-age-sec', type=float, default=2.0)
    parser.add_argument('--max-tf-age-sec', type=float, default=1.5)
    parser.add_argument('--second-step-observation-sec', type=float, default=95.0)
    args = parser.parse_args()
    artifact = run_phase139(args)
    second = _dict(artifact.get('second_step_dispatch'))
    print(json.dumps({
        'classification': artifact.get('classification'),
        'valid': artifact.get('valid'),
        'handoff_allowed': artifact.get('handoff_allowed'),
        'maze_explorer_started': artifact.get('maze_explorer_started'),
        'maze_explorer_max_goals': artifact.get('maze_explorer_max_goals'),
        'second_step_goal_kind': second.get('goal_kind'),
        'second_step_result_status_label': second.get('result_status_label'),
        'third_goal_dispatched': artifact.get('third_goal_dispatched'),
        'artifact': str(args.output),
    }, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
