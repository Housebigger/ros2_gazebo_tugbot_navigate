import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / 'tools' / 'analyze_goal_events_with_nav2_log.py'


def _write_jsonl(path: Path, rows: list[dict]) -> None:
    path.write_text(''.join(json.dumps(row, sort_keys=True) + '\n' for row in rows), encoding='utf-8')


def test_phase23b_nav2_analyzer_exports_event_timestamps(tmp_path):
    goal_events = tmp_path / 'goal_events.jsonl'
    launch_log = tmp_path / 'launch.log'
    output = tmp_path / 'nav2.json'

    _write_jsonl(goal_events, [
        {'wall_time': 100.0, 'state': {'event': 'dispatch', 'goal_sequence': 1, 'goal_kind': 'branch'}},
        {'wall_time': 140.0, 'state': {'event': 'timeout', 'goal_sequence': 1, 'result_reason': 'goal_timeout'}},
    ])
    launch_log.write_text('\n'.join([
        '[controller_server-7] [ERROR] [105.500000000] [controller_server]: Failed to make progress',
        '[controller_server-7] [INFO] [106.000000000] [local_costmap.local_costmap]: Received request to clear entirely the local_costmap',
        '[planner_server-9] [INFO] [106.250000000] [global_costmap.global_costmap]: Received request to clear entirely the global_costmap',
        '[controller_server-7] [WARN] [106.500000000] [controller_server]: [follow_path] [ActionServer] Aborting handle.',
    ]), encoding='utf-8')

    result = subprocess.run([
        sys.executable,
        str(SCRIPT),
        '--goal-events', str(goal_events),
        '--log', str(launch_log),
        '--output-json', str(output),
    ], text=True, capture_output=True, check=False)

    assert result.returncode == 0, result.stderr
    data = json.loads(output.read_text())
    row = data['goals'][0]

    assert row['progress_failure_times'] == [105.5]
    assert row['clear_costmap_times'] == [106.0, 106.25]
    assert row['controller_abort_times'] == [106.5]
    assert row['first_progress_failure_time'] == 105.5
    assert row['last_clear_costmap_time'] == 106.25
    assert row['last_controller_abort_time'] == 106.5
    assert data['summary']['timeout_with_controller_abort_count'] == 1
