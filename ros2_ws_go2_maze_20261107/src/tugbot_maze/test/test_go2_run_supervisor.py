"""An old PASS file must never certify a new physical experiment."""
import json
from pathlib import Path
import subprocess
import sys

import pytest

WORKSPACE = Path(__file__).resolve().parents[3]


def test_watcher_does_not_accept_old_success(tmp_path):
    (tmp_path / 'status.json').write_text(json.dumps({'success': True, 'samples': 1000}))
    placeholder = subprocess.Popen(['sleep', '10'])
    try:
        with pytest.raises(subprocess.TimeoutExpired):
            subprocess.run([sys.executable, str(WORKSPACE / 'tools/watch_maze_run.py'),
                            str(tmp_path), str(placeholder.pid)],
                           capture_output=True, timeout=2, check=False)
    finally:
        placeholder.terminate()
        placeholder.wait(timeout=2)


def test_checked_launch_refuses_reused_evidence_directory(tmp_path):
    (tmp_path / 'status.json').write_text('{"success": true}')
    result = subprocess.run(['bash', str(WORKSPACE / 'tools/run_checked_maze.sh'), str(tmp_path)],
                            capture_output=True, text=True, timeout=15, check=False)
    assert result.returncode == 2
    assert 'Evidence directory must be empty' in result.stderr
    assert not (tmp_path / 'launch.log').exists()


def test_cleanup_finishes_when_shutdown_sends_another_interrupt(tmp_path):
    # A second shutdown signal during cleanup must not skip the replay step.
    child = tmp_path / 'child.py'
    ready = tmp_path / 'ready'
    child.write_text('''import os, signal, sys, time
from pathlib import Path
def stop(signum, frame):
    os.kill(os.getppid(), signal.SIGINT)
    raise SystemExit(0)
signal.signal(signal.SIGINT, stop)
Path(sys.argv[1]).touch()
while True:
    time.sleep(1)
''')
    wrapper = (WORKSPACE / 'tools/run_checked_maze.sh').read_text()
    cleanup = 'cleanup() {' + wrapper.split('cleanup() {', 1)[1].split('\n}\n', 1)[0] + '\n}\n'
    harness = tmp_path / 'harness.sh'
    harness.write_text('''#!/usr/bin/env bash
set -eo pipefail
trap 'exit 130' INT TERM
setsid python3 "$1" "$2" &
launch_pid=$!
while [ ! -f "$2" ]; do sleep 0.01; done
''' + cleanup + '\ncleanup\nprintf "REPLAY_REACHED\\n"\n')
    result = subprocess.run(['bash', str(harness), str(child), str(ready)],
                            capture_output=True, text=True, timeout=12,
                            start_new_session=True, check=False)
    assert result.returncode == 0, result.stderr
    assert 'REPLAY_REACHED' in result.stdout
