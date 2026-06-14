import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / 'tools' / 'compare_goal_diagnostics_runs.py'
LOG = ROOT / 'log'


def test_phase20_compare_goal_diagnostics_runs_contract_against_existing_artifacts(tmp_path):
    output = tmp_path / 'phase20_compare.json'
    args = [
        sys.executable,
        str(SCRIPT),
        '--run',
        'phase17',
        '--goal-events',
        str(LOG / 'phase17_goal_events_smoke.jsonl'),
        '--nav2-analysis',
        str(LOG / 'phase17_goal_nav2_analysis.json'),
        '--state',
        str(LOG / 'phase17_explorer_state_smoke.jsonl'),
        '--run',
        'phase18',
        '--goal-events',
        str(LOG / 'phase18_goal_events_smoke.jsonl'),
        '--nav2-analysis',
        str(LOG / 'phase18_goal_nav2_analysis.json'),
        '--state',
        str(LOG / 'phase18_explorer_state_smoke.jsonl'),
        '--local-cost-summary',
        str(LOG / 'phase18_local_costmap_summary.json'),
        '--run',
        'phase19',
        '--goal-events',
        str(LOG / 'phase19_goal_events_smoke.jsonl'),
        '--nav2-analysis',
        str(LOG / 'phase19_goal_nav2_analysis.json'),
        '--state',
        str(LOG / 'phase19_explorer_state_smoke.jsonl'),
        '--local-cost-summary',
        str(LOG / 'phase19_goal_event_cost_summary.json'),
        '--output-json',
        str(output),
    ]
    result = subprocess.run(args, text=True, capture_output=True, check=False)

    assert result.returncode == 0, result.stderr
    data = json.loads(output.read_text())
    assert data['summary']['run_count'] == 3
    assert data['summary']['exit_reached_count'] >= 2
    assert data['summary']['timeout_count_total'] >= 6
    assert data['summary']['blocked_or_blacklist_run_count'] >= 1

    runs = {run['run_id']: run for run in data['runs']}
    assert runs['phase17']['final_mode'] == 'EXIT_REACHED'
    assert runs['phase18']['final_mode'] == 'FAILED_EXHAUSTED'
    assert runs['phase19']['final_mode'] == 'EXIT_REACHED'
    assert runs['phase19']['blocked_branch_count'] == 0
    assert runs['phase19']['blacklisted_goal_count'] == 0

    timeout_rows = data['timeout_goals']
    assert any(row['run_id'] == 'phase19' and row['goal_sequence'] == 10 for row in timeout_rows)
    seq10 = next(row for row in timeout_rows if row['run_id'] == 'phase19' and row['goal_sequence'] == 10)
    assert 'high_dispatch_path_cost' in seq10['taxonomy_labels']
    assert 'high_timeout_robot_cost' in seq10['taxonomy_labels']
    assert 'timeout_squeezed' in seq10['taxonomy_labels']
    assert 'nav2_progress_cluster' in seq10['taxonomy_labels']

    assert data['taxonomy']['timeout']['nav2_progress_cluster']['count'] >= 5
    assert data['taxonomy']['timeout']['high_timeout_robot_cost']['count'] >= 2
    assert 'success' in data['taxonomy']
    assert 'high_dispatch_path_cost' in data['taxonomy']['success']
    assert 'run_id,final_mode,timeout_count' in result.stdout
