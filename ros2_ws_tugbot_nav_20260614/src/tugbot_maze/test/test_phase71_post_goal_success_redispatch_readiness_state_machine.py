from pathlib import Path
import importlib.util
import json
import sys

ROOT = Path(__file__).resolve().parents[3]
ANALYZER = ROOT / 'tools' / 'analyze_phase71_post_goal_success_redispatch_readiness_state_machine.py'
REPORT = ROOT / 'doc' / 'doc_report' / 'phase71_post_goal_success_redispatch_readiness_state_machine_diagnosis_report.md'
PHASE70_ARTIFACT_DIR = ROOT / 'log' / 'phase70_centerline_gate_relaxation_balance_first_runtime_validation'
PHASE70_RUN_ID = 'phase70_centerline_gate_relaxation_balance_first_runtime_validation'
PHASE71_RUN_ID = 'phase71_post_goal_success_redispatch_readiness_state_machine_diagnosis'


def _load_analyzer():
    spec = importlib.util.spec_from_file_location('phase71_analyzer', ANALYZER)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _write_jsonl(path: Path, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text('\n'.join(json.dumps(row, sort_keys=True) for row in rows), encoding='utf-8')


def _make_replay(tmp_path: Path, replay_id: str, events, states, preflight='MAX_GOALS=2\n'):
    replay_dir = tmp_path / replay_id
    replay_dir.mkdir(parents=True)
    _write_jsonl(replay_dir / f'{PHASE70_RUN_ID}_{replay_id}_goal_events.jsonl', [{'state': e, 'elapsed_sec': i} for i, e in enumerate(events)])
    _write_jsonl(replay_dir / f'{PHASE70_RUN_ID}_{replay_id}_explorer_state.jsonl', [{'state': s, 'elapsed_sec': i} for i, s in enumerate(states)])
    (replay_dir / f'{PHASE70_RUN_ID}_{replay_id}_inner_ingress_navigate_to_pose_action_result.json').write_text('{"success": true}', encoding='utf-8')
    (tmp_path / f'{PHASE70_RUN_ID}_preflight.txt').write_text(preflight, encoding='utf-8')
    (tmp_path / f'{PHASE70_RUN_ID}_nav2_config_diff.txt').write_text('', encoding='utf-8')
    (tmp_path / f'{PHASE70_RUN_ID}_cleanup_processes_after.txt').write_text('', encoding='utf-8')
    return replay_dir


def _ready_gate(blocking_reasons=None, passed=True):
    blocking_reasons = blocking_reasons or []
    return {
        'passed': passed,
        'blocking_reasons': blocking_reasons,
        'checks': {
            'map_sufficient': passed and 'map_sufficient' not in blocking_reasons,
            'scan_sufficient': passed and 'scan_sufficient' not in blocking_reasons,
            'tf_sufficient': passed and 'tf_sufficient' not in blocking_reasons,
            'local_costmap_sufficient': passed and 'local_costmap_sufficient' not in blocking_reasons,
            'nav2_lifecycle_active': passed and 'nav2_lifecycle_active' not in blocking_reasons,
            'navigate_to_pose_action_ready': passed and 'navigate_to_pose_action_ready' not in blocking_reasons,
            'goal_pose_subscriber_ready': passed and 'goal_pose_subscriber_ready' not in blocking_reasons,
        },
        'map': {'sufficient': passed, 'known_ratio': 0.96, 'free_ratio': 0.94, 'map_stamp': 35.001},
        'scan': {'sufficient': passed, 'finite_count': 630},
        'tf': {'sufficient': passed},
        'local_costmap': {'sufficient': passed, 'sample_age_sec': 0.42, 'known_ratio': 1.0, 'free_ratio': 0.57, 'map_stamp': 35.4},
    }


def test_phase71_analyzer_contract_mentions_required_diagnostics_and_guardrails():
    source = ANALYZER.read_text(encoding='utf-8')
    assert PHASE71_RUN_ID in source
    assert 'Phase70' in source
    for token in [
        'goal outcome',
        'goal_count',
        'mode transitions',
        'entry readiness',
        'candidate_count',
        'open_direction_count',
        'local topology kind',
        'map/scan/costmap/TF ages',
        'blacklist/visited/frontier/exhausted reasons',
        're-dispatch gate',
    ]:
        assert token in source
    for classification in [
        'POST_SUCCESS_REDISTPATCH_BLOCKED_BY_READINESS',
        'POST_SUCCESS_NO_CANDIDATE',
        'POST_SUCCESS_EXHAUSTED_PREMATURELY',
        'SUCCESS_EVENT_SEMANTICS_MISMATCH',
        'INSUFFICIENT_EVIDENCE',
    ]:
        assert classification in source
    for forbidden in [
        'Nav2/MPPI/controller tuning',
        'inflation/robot_radius/clearance_radius_m/map threshold tuning',
        'branch scoring change',
        'centerline gate runtime behavior change',
        'fallback/terminal acceptance change',
        'autonomous exploration success claim',
        'exit success claim',
    ]:
        assert forbidden in source


def test_phase71_classifies_phase70_replay01_success_then_goal_budget_exhausted_as_premature_before_redispatch():
    module = _load_analyzer()
    assert PHASE70_ARTIFACT_DIR.exists(), 'Phase71 intentionally reuses Phase70 artifacts'
    result = module.analyze_phase71(PHASE70_ARTIFACT_DIR, output=None)

    assert result['run_id'] == PHASE71_RUN_ID
    assert result['source_phase70_artifact_dir'] == str(PHASE70_ARTIFACT_DIR)
    assert result['classification'] == 'POST_SUCCESS_EXHAUSTED_PREMATURELY'
    assert result['complete_autonomous_success_claimed'] is False
    assert result['exit_success_claimed'] is False

    replay01 = next(r for r in result['replays'] if r['replay_id'] == 'replay_01')
    assert replay01['goal1']['outcome_event'] == 'success'
    assert replay01['post_success']['final_mode'] == 'FAILED_EXHAUSTED'
    assert replay01['post_success']['final_goal_count'] == 1
    assert replay01['post_success']['max_goals'] == 1
    assert replay01['post_success']['last_terminal_reason'] == 'goal budget reached'
    assert replay01['post_success']['dispatch_after_success_count'] == 0
    assert replay01['post_success']['redispatch_gate_triggered'] is False
    assert replay01['post_success']['blocking_stage'] == 'goal_budget_exhausted_before_readiness_or_topology_resample'
    assert replay01['readiness_after_success']['passed'] is True
    assert replay01['candidate_after_success']['candidate_after_filter_count'] == 4
    assert replay01['candidate_after_success']['raw_open_direction_count'] == 4
    assert replay01['candidate_after_success']['local_topology_kind'] == 'junction'
    assert replay01['resource_ages_after_success']['local_costmap']['sample_age_sec'] is not None


def test_phase71_classifies_synthetic_post_success_readiness_blocked(tmp_path):
    module = _load_analyzer()
    _make_replay(
        tmp_path,
        'replay_01',
        events=[
            {'event': 'dispatch', 'goal_sequence': 1, 'target': [1.0, 0.0]},
            {'event': 'success', 'goal_sequence': 1, 'result_reason': 'succeeded'},
        ],
        states=[
            {'mode': 'NAVIGATING', 'goal_count': 1, 'goal_success_count': 0, 'dispatch_readiness_gate': _ready_gate()},
            {
                'mode': 'WAIT_FOR_DISPATCH_ENTRY_READINESS',
                'goal_count': 1,
                'goal_success_count': 1,
                'last_completed_goal_sequence_id': 1,
                'dispatch_readiness_gate_passed': False,
                'dispatch_readiness_gate': _ready_gate(['map_sufficient'], passed=False),
                'dispatch_readiness_blocking_reasons': ['map_sufficient'],
            },
        ],
        preflight='MAX_GOALS=3\n',
    )
    result = module.analyze_phase71(tmp_path, output=None)
    replay = result['replays'][0]
    assert result['classification'] == 'POST_SUCCESS_REDISTPATCH_BLOCKED_BY_READINESS'
    assert replay['post_success']['blocking_stage'] == 'dispatch_entry_readiness'
    assert replay['readiness_after_success']['blocking_reasons'] == ['map_sufficient']
    assert replay['post_success']['redispatch_gate_triggered'] is True


def test_phase71_classifies_synthetic_post_success_no_candidate_after_ready_topology_resample(tmp_path):
    module = _load_analyzer()
    _make_replay(
        tmp_path,
        'replay_01',
        events=[
            {'event': 'dispatch', 'goal_sequence': 1, 'target': [1.0, 0.0]},
            {'event': 'success', 'goal_sequence': 1, 'result_reason': 'succeeded'},
        ],
        states=[
            {'mode': 'NAVIGATING', 'goal_count': 1, 'goal_success_count': 0, 'dispatch_readiness_gate': _ready_gate()},
            {
                'mode': 'FAILED_EXHAUSTED',
                'goal_count': 1,
                'goal_success_count': 1,
                'last_completed_goal_sequence_id': 1,
                'last_terminal_reason': 'dead_end_policy_no_branch_options',
                'dispatch_readiness_gate_passed': True,
                'dispatch_readiness_gate': _ready_gate(),
                'last_local_topology_kind': 'dead_end',
                'last_candidate_count': 0,
                'last_open_direction_count': 1,
                'last_topology_sampling_diagnostics': {
                    'raw_open_direction_count': 1,
                    'filtered_open_direction_count': 1,
                    'candidate_before_filter_count': 1,
                    'candidate_after_filter_count': 0,
                    'local_topology_kind': 'dead_end',
                },
                'phase56_open_direction_to_candidate_diagnostics': {
                    'available': True,
                    'raw_open_direction_count': 1,
                    'filtered_open_direction_count': 1,
                    'candidate_before_filter_count': 1,
                    'candidate_after_filter_count': 0,
                    'branch_candidate_rejection_reason': 'duplicate_or_exhausted',
                },
            },
        ],
        preflight='MAX_GOALS=3\n',
    )
    result = module.analyze_phase71(tmp_path, output=None)
    replay = result['replays'][0]
    assert result['classification'] == 'POST_SUCCESS_NO_CANDIDATE'
    assert replay['post_success']['blocking_stage'] == 'post_success_candidate_formation'
    assert replay['candidate_after_success']['candidate_after_filter_count'] == 0
    assert replay['candidate_after_success']['rejection_reason'] == 'duplicate_or_exhausted'


def test_phase71_classifies_synthetic_success_event_semantics_mismatch(tmp_path):
    module = _load_analyzer()
    _make_replay(
        tmp_path,
        'replay_01',
        events=[
            {'event': 'dispatch', 'goal_sequence': 1, 'target': [1.0, 0.0]},
            {'event': 'success', 'goal_sequence': 1, 'result_reason': 'succeeded'},
        ],
        states=[
            {'mode': 'NAVIGATING', 'goal_count': 1, 'goal_success_count': 0, 'dispatch_readiness_gate': _ready_gate()},
            {'mode': 'FAILED_EXHAUSTED', 'goal_count': 1, 'goal_success_count': 0, 'last_completed_goal_sequence_id': None, 'dispatch_readiness_gate': _ready_gate()},
        ],
        preflight='MAX_GOALS=3\n',
    )
    result = module.analyze_phase71(tmp_path, output=None)
    replay = result['replays'][0]
    assert result['classification'] == 'SUCCESS_EVENT_SEMANTICS_MISMATCH'
    assert replay['goal1']['success_state_observed'] is False
    assert replay['post_success']['blocking_stage'] == 'success_event_state_counter_mismatch'


def test_phase71_report_contract_stop_condition_and_no_phase72():
    report = REPORT.read_text(encoding='utf-8')
    assert 'Phase71' in report
    assert PHASE71_RUN_ID in report
    assert 'Phase70' in report and 'replay_01' in report
    assert 'POST_SUCCESS_EXHAUSTED_PREMATURELY' in report
    assert 'goal budget reached' in report
    assert '不进入 Phase72' in report or 'Do not enter Phase72' in report
    assert '不宣称 autonomous exploration success' in report or 'no autonomous exploration success claim' in report
    assert '不宣称 exit success' in report or 'no exit success claim' in report
