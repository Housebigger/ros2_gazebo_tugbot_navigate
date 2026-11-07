"""Bounded process supervisor; reads audit results without controlling the robot."""
import json
import os
import sys
import time
from pathlib import Path

run_dir = Path(sys.argv[1])
launch_pid = int(sys.argv[2])
started_ns = time.time_ns()
started = time.monotonic()
last_report = 0
while time.monotonic() - started < 3600:
    try:
        os.kill(launch_pid, 0)
    except ProcessLookupError:
        raise SystemExit('Launch exited before accepted completion')
    try:
        status_file = run_dir / 'status.json'
        status = (json.loads(status_file.read_text())
                  if status_file.stat().st_mtime_ns >= started_ns else {})
    except (FileNotFoundError, json.JSONDecodeError):
        status = {}
    elapsed = time.monotonic() - started
    if elapsed - last_report >= 30:
        print(json.dumps({'wall_s': round(elapsed), **status}), flush=True)
        last_report = elapsed
    if status.get('success'):
        print('GO2 MAZE PASS ' + json.dumps(status), flush=True)
        raise SystemExit(0)
    if status.get('collision_samples', 0) > 0 or status.get('exterior_samples', 0) > 0:
        raise SystemExit('Independent geometry audit failed: ' + json.dumps(status))
    if status.get('samples', 0) > 100 and status.get('max_pose_pair_error', 0) >= 0.05:
        raise SystemExit('Ground-truth and odometry pairing failed: ' + json.dumps(status))
    if elapsed > 90 and status.get('samples', 0) < 100:
        raise SystemExit('No usable independent pose feedback')
    if status.get('elapsed_sim_s', 0) > 1800:
        raise SystemExit('Simulation time budget exceeded')
    time.sleep(1)
raise SystemExit('Wall time budget exceeded')
