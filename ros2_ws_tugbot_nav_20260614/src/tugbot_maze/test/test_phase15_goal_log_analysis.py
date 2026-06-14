import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / 'tools' / 'analyze_goal_events_with_nav2_log.py'


def _write_goal_events(path: Path) -> None:
    rows = [
        {
            'seq': 1,
            'wall_time': 100.0,
            'state': {
                'event': 'dispatch',
                'goal_sequence': 1,
                'dispatch_pose': [0.0, 0.0, 0.0],
                'target': [1.0, 0.0],
                'branch_angle': 0.0,
                'local_topology': 'junction',
                'goal_kind': 'explore',
                'target_exit_dist': 4.0,
                'effective_timeout_sec': 35.0,
                'near_exit': False,
            },
        },
        {
            'seq': 2,
            'wall_time': 108.0,
            'state': {
                'event': 'success',
                'goal_sequence': 1,
                'elapsed_sec': 8.0,
                'result_status': 4,
                'result_reason': 'succeeded',
            },
        },
        {
            'seq': 3,
            'wall_time': 120.0,
            'state': {
                'event': 'dispatch',
                'goal_sequence': 2,
                'dispatch_pose': [1.0, 0.0, 0.0],
                'target': [1.0, 1.0],
                'branch_angle': 1.57,
                'local_topology': 'corridor',
                'goal_kind': 'explore',
                'target_exit_dist': 2.0,
                'effective_timeout_sec': 35.0,
                'near_exit': False,
            },
        },
        {
            'seq': 4,
            'wall_time': 156.0,
            'state': {
                'event': 'timeout',
                'goal_sequence': 2,
                'elapsed_sec': 36.0,
                'result_reason': 'goal_timeout',
                'branch_failure_state': 'untried',
                'caused_branch_failure': False,
                'caused_blacklist': False,
            },
        },
    ]
    path.write_text('\n'.join(json.dumps(row) for row in rows) + '\n')


def test_phase15_script_correlates_controller_events_by_goal_interval(tmp_path):
    events = tmp_path / 'goal_events.jsonl'
    log = tmp_path / 'launch.log'
    output = tmp_path / 'analysis.json'
    _write_goal_events(events)
    log.write_text(
        '\n'.join(
            [
                '[controller_server-7] [INFO] [101.0] [controller_server]: Passing new path to controller.',
                '[controller_server-7] [ERROR] [130.0] [controller_server]: Failed to make progress',
                '[controller_server-7] [INFO] [131.0] [local_costmap.local_costmap]: Received request to clear entirely the local_costmap',
                '[controller_server-7] [WARN] [132.0] [controller_server]: [follow_path] [ActionServer] Aborting handle.',
            ]
        )
        + '\n'
    )

    result = subprocess.run(
        [sys.executable, str(SCRIPT), '--goal-events', str(events), '--log', str(log), '--output-json', str(output)],
        check=False,
        text=True,
        capture_output=True,
    )

    assert result.returncode == 0, result.stderr
    data = json.loads(output.read_text())
    by_seq = {row['goal_sequence']: row for row in data['goals']}
    assert by_seq[1]['outcome'] == 'success'
    assert by_seq[1]['progress_failure_count'] == 0
    assert by_seq[2]['outcome'] == 'timeout'
    assert by_seq[2]['progress_failure_count'] == 1
    assert by_seq[2]['recovery_count'] == 1
    assert by_seq[2]['controller_abort_count'] == 1
    assert data['summary']['timeout_with_progress_failure_count'] == 1
    assert data['summary']['success_with_progress_failure_count'] == 0


def test_phase15_script_prints_goal_sequence_table(tmp_path):
    events = tmp_path / 'goal_events.jsonl'
    log = tmp_path / 'launch.log'
    _write_goal_events(events)
    log.write_text('[controller_server-7] [ERROR] [130.0] [controller_server]: Failed to make progress\n')

    result = subprocess.run(
        [sys.executable, str(SCRIPT), '--goal-events', str(events), '--log', str(log)],
        check=False,
        text=True,
        capture_output=True,
    )

    assert result.returncode == 0, result.stderr
    assert 'goal_sequence' in result.stdout
    assert 'progress_failure_count' in result.stdout
    assert 'timeout' in result.stdout
