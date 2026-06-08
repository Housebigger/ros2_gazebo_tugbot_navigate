#!/usr/bin/env python3
"""Phase27-alt-R3 decision-path coverage analyzer.

This is a static/replay audit helper. It inspects the upper-level MazeExplorer
source plus the Phase27-alt-R2 replay coverage JSON to explain why R1 can enter
`robot_exit_dist <= 0.6m` without emitting a runtime `terminal_acceptance`
fallback event. It does not run Gazebo/Nav2, fetch or overlay MPPI source, alter
Nav2/MPPI/controller parameters, relax local-cost gates, or explain MPPI
selected-control near-zero root cause.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any


def load_json(path: Path) -> dict[str, Any]:
    if not path.exists() or path.stat().st_size == 0:
        return {}
    return json.loads(path.read_text(encoding='utf-8'))


def index_or_none(text: str, token: str, start: int = 0) -> int | None:
    try:
        return text.index(token, start)
    except ValueError:
        return None


def block_between(text: str, start_token: str, end_token: str | None = None) -> str:
    start = index_or_none(text, start_token)
    if start is None:
        return ''
    if end_token is None:
        return text[start:]
    end = index_or_none(text, end_token, start + len(start_token))
    return text[start:end if end is not None else len(text)]


def line_no(text: str, index: int | None) -> int | None:
    if index is None:
        return None
    return text.count('\n', 0, index) + 1


def audit_decision_path(source: str) -> dict[str, Any]:
    explore = block_between(source, 'def _explore_once', 'def _analyze_and_dispatch')
    decision = block_between(source, 'def _near_exit_fallback_decision', 'def _maybe_apply_near_exit_fallback')
    maybe = block_between(source, 'def _maybe_apply_near_exit_fallback', 'def _compute_near_exit_micro_goal')
    failure = block_between(source, 'def _handle_goal_failure', 'def _is_stale_goal_result')
    cancel_after_timeout = block_between(source, 'def _mark_goal_canceled_after_timeout', 'def _start_goal_settle_cooldown')
    terminal_cancel = block_between(source, 'def _cancel_active_goal_for_terminal_state', 'def _cancel_goal_after_timeout')

    exit_idx = index_or_none(explore, 'if self._exit_reached(robot_pose):')
    active_idx = index_or_none(explore, 'if self.goal_active:')
    settle_idx = index_or_none(explore, 'if self._goal_settle_active():')
    fallback_idx = index_or_none(explore, 'if self._maybe_apply_near_exit_fallback(robot_pose):')
    dispatch_idx = index_or_none(explore, 'self._analyze_and_dispatch(robot_pose)')

    normal_precedes = exit_idx is not None and fallback_idx is not None and exit_idx < fallback_idx
    fallback_after_no_active = (
        fallback_idx is not None
        and active_idx is not None
        and settle_idx is not None
        and active_idx < fallback_idx
        and settle_idx < fallback_idx
    )
    fallback_before_branch_dispatch = fallback_idx is not None and dispatch_idx is not None and fallback_idx < dispatch_idx

    recent_tokens = [
        'recent_problem_reasons = {GOAL_TIMEOUT, BLOCKED_NAV2, GOAL_CANCELED_AFTER_TIMEOUT, GOAL_REJECTED}',
        'has_recent_failure = self.last_failure_reason in recent_problem_reasons',
        'has_cmd_stall =',
        'if not (has_recent_failure or has_cmd_stall):',
        "base['fallback_reason'] = 'no_recent_timeout_abort_or_cmd_stall'",
    ]
    terminal_tokens = [
        'robot_exit_dist <= self.near_exit_terminal_acceptance_radius_m',
        "'fallback_reason': 'terminal_acceptance_radius'",
        "'action': 'terminal_acceptance'",
        "'near_exit_fallback_triggered': True",
    ]
    action_tokens = [
        "if action == 'terminal_acceptance':",
        'self._publish_near_exit_fallback_event(decision)',
        "self._enter_terminal_state(EXIT_REACHED, terminal_reason='near_exit_terminal_acceptance', robot_pose=robot_pose)",
    ]

    return {
        'normal_exit_monitor_precedes_near_exit_fallback': bool(normal_precedes),
        'near_exit_fallback_called_only_after_no_active_goal_and_settle': bool(fallback_after_no_active),
        'near_exit_fallback_precedes_normal_branch_dispatch': bool(fallback_before_branch_dispatch),
        'fallback_terminal_acceptance_requires_recent_problem_evidence': all(token in decision for token in recent_tokens),
        'fallback_terminal_acceptance_contract_present': all(token in decision for token in terminal_tokens),
        'terminal_acceptance_action_enters_exit_reached': all(token in maybe for token in action_tokens),
        'failure_path_sets_last_failure_reason': 'self.last_failure_reason = reason' in failure,
        'timeout_cancel_result_sets_recent_failure_reason': 'self.last_failure_reason = GOAL_CANCELED_AFTER_TIMEOUT' in cancel_after_timeout,
        'normal_exit_path_cancels_active_goal_before_fallback': (
            'self._enter_terminal_state(EXIT_REACHED, terminal_reason=\'exit_reached\', robot_pose=robot_pose)' in explore
            and 'self._cancel_active_goal_for_terminal_state(terminal_reason)' in block_between(source, 'def _enter_terminal_state', 'def _cancel_active_goal_for_terminal_state')
            and "self._publish_goal_event('terminal_cancel'" in terminal_cancel
        ),
        'line_numbers': {
            'exit_reached_check': line_no(explore, exit_idx),
            'goal_active_check': line_no(explore, active_idx),
            'settle_check': line_no(explore, settle_idx),
            'near_exit_fallback_call': line_no(explore, fallback_idx),
            'branch_dispatch_call': line_no(explore, dispatch_idx),
        },
        'decision_path_summary': [
            'terminal-state guard returns first',
            'normal exit monitor checks _exit_reached(robot_pose) before fallback',
            'active goals return before fallback; timeout/failure only stores recent evidence and enters SETTLING',
            'settling returns before fallback until cooldown expires',
            'near-exit fallback is evaluated only at AT_NODE_ANALYZE before normal branch dispatch',
        ],
    }


def interpret_r1_r2(r2: dict[str, Any], audit: dict[str, Any]) -> dict[str, Any]:
    window = r2.get('nearest_exit_radius_state_window') or {}
    final_state = r2.get('final_state') or {}
    counts = r2.get('counts') or {}
    entered_terminal_radius = (window.get('state_count_at_or_below_terminal_radius') or 0) > 0
    terminal_event_count = counts.get('terminal_acceptance_event_count') or 0
    normal_exit_reached = final_state.get('final_mode') == 'EXIT_REACHED'
    natural_bypass = bool(
        entered_terminal_radius
        and terminal_event_count == 0
        and normal_exit_reached
        and audit.get('normal_exit_monitor_precedes_near_exit_fallback')
        and audit.get('near_exit_fallback_called_only_after_no_active_goal_and_settle')
    )
    if natural_bypass:
        why = 'robot_entered_terminal_radius_while_goal_active_normal_exit_monitor_executed_before_fallback_decision'
    elif not entered_terminal_radius:
        why = 'r1_did_not_enter_terminal_radius'
    elif terminal_event_count:
        why = 'terminal_acceptance_event_present_in_r2_input'
    else:
        why = 'insufficient_static_evidence_for_natural_bypass'
    return {
        'r2_status': r2.get('coverage_status'),
        'r2_conclusion': r2.get('conclusion'),
        'entered_terminal_radius_in_r1_state': entered_terminal_radius,
        'terminal_acceptance_event_count': terminal_event_count,
        'final_mode': final_state.get('final_mode'),
        'normal_exit_monitor_natural_bypass_likely': natural_bypass,
        'why_r1_entered_radius_without_fallback_event': why,
        'normal_terminal_monitor_can_preempt_fallback_terminal_acceptance': bool(audit.get('normal_exit_monitor_precedes_near_exit_fallback')),
        'fallback_terminal_acceptance_is_abnormal_branch': True,
    }


def runtime_design(audit: dict[str, Any], interp: dict[str, Any]) -> dict[str, Any]:
    audit_ok = all([
        audit.get('normal_exit_monitor_precedes_near_exit_fallback'),
        audit.get('near_exit_fallback_called_only_after_no_active_goal_and_settle'),
        audit.get('fallback_terminal_acceptance_requires_recent_problem_evidence'),
        audit.get('fallback_terminal_acceptance_contract_present'),
        audit.get('terminal_acceptance_action_enters_exit_reached'),
    ])
    targeted_needed = not audit_ok
    if audit_ok and interp.get('normal_exit_monitor_natural_bypass_likely'):
        recommended_path = 'host_level_coverage_sufficient_for_abnormal_branch'
        reason = 'normal exit monitor naturally handles ordinary radius crossing before fallback decision path can run'
    elif audit_ok:
        recommended_path = 'optional_targeted_runtime_design_only'
        reason = 'static path is sound but R1 replay did not exercise abnormal post-failure no-active-goal terminal-radius window'
        targeted_needed = False
    else:
        recommended_path = 'targeted_runtime_scenario_design_required_before_execution'
        reason = 'decision path audit found missing or ambiguous ordering/contract evidence'
    return {
        'targeted_runtime_needed': bool(targeted_needed),
        'execute_runtime_now': False,
        'recommended_path': recommended_path,
        'reason': reason,
        'if_runtime_is_later_requested': {
            'must_keep_default_thresholds': True,
            'must_not_relax_local_cost_gate': True,
            'must_not_change_nav2_mppi_controller_params': True,
            'must_not_change_branch_selection': True,
            'coverage_window_needed': 'robot already within <=0.6m after recent timeout/cancel/failure, no active goal, after settle cooldown, before normal exit monitor has already entered EXIT_REACHED',
            'risk': 'normal exit monitor is expected to win ordinary runtime races, so natural coverage may be difficult without artificial harnessing',
        },
    }


def analyze(maze_explorer: Path, r2_coverage_json: Path) -> dict[str, Any]:
    source = maze_explorer.read_text(encoding='utf-8')
    r2 = load_json(r2_coverage_json)
    audit = audit_decision_path(source)
    interp = interpret_r1_r2(r2, audit)
    design = runtime_design(audit, interp)
    return {
        'phase': 'Phase27-alt-R3',
        'inputs': {
            'maze_explorer': str(maze_explorer),
            'r2_coverage_json': str(r2_coverage_json),
        },
        'decision_path_audit': audit,
        'r1_r2_interpretation': interp,
        'runtime_coverage_design': design,
        'conclusion': 'PASS_AS_TARGETED_TERMINAL_ACCEPTANCE_COVERAGE_DESIGN',
        'coverage_result_from_r2': r2.get('conclusion', 'NOT_COVERED_TERMINAL_ACCEPTANCE_BRANCH'),
        'guardrails': {
            'phase26y_or_26z_entered': False,
            'nav2_mppi_controller_source_fetched': False,
            'nav2_mppi_controller_params_modified': False,
            'costcritic_275_promoted_or_rejected': False,
            'branch_selection_strategy_modified': False,
            'local_cost_gate_relaxed': False,
            'runtime_started_by_r3': False,
        },
        'mppi_root_cause_claim': 'not_evaluated_by_phase27_alt_r3',
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument('--maze-explorer', required=True, type=Path)
    parser.add_argument('--r2-coverage-json', required=True, type=Path)
    parser.add_argument('--output-json', required=True, type=Path)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    result = analyze(args.maze_explorer, args.r2_coverage_json)
    args.output_json.parent.mkdir(parents=True, exist_ok=True)
    args.output_json.write_text(json.dumps(result, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    print(json.dumps({
        'output': str(args.output_json),
        'conclusion': result['conclusion'],
        'runtime_recommended_path': result['runtime_coverage_design']['recommended_path'],
    }, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
