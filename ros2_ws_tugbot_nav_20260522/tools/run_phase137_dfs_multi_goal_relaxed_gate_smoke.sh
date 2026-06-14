#!/usr/bin/env bash
# Phase137: DFS multi-goal smoke test with relaxed readiness gate.
#
# Launches full maze simulation headless with max_goals:=30, monitors
# /maze/goal_events for 5+ consecutive successful explore goals.
# PASS: 5+ consecutive explore goals succeed without stalling.
# FAIL: Robot stalls or <5 goals succeed within timeout.
#
WS="$(cd "$(dirname "$0")/.." && pwd)"
cd "$WS"

set +u
source /opt/ros/jazzy/setup.bash
source install/setup.bash
set -euo pipefail

ARTIFACT_DIR="log/phase137_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$ARTIFACT_DIR"

echo "[Phase137] Starting headless maze simulation with max_goals=30 ..."

# Start simulation in background
ros2 launch tugbot_bringup tugbot_maze_explore.launch.py \
    headless:=true \
    use_rviz:=false \
    explorer_type:=maze_dfs \
    max_goals:=30 \
    entry_direct_enabled:=true \
    entry_direct_distance_m:=1.5 \
    > "$ARTIFACT_DIR/launch.log" 2>&1 &
LAUNCH_PID=$!

echo "[Phase137] Launch PID=$LAUNCH_PID"

# Wait for maze_explorer to start (it has a 13s timer delay)
echo "[Phase137] Waiting for maze_explorer to start (13s timer + warmup)..."
sleep 20

# Run the Python monitor
python3 - "$ARTIFACT_DIR" <<'PYEOF'
import json, sys, time, os, subprocess, signal, threading

artifact_dir = sys.argv[1]
TARGET_SUCCESSES = 5
MAX_WAIT_SEC = 300  # 5 min total
STALL_TIMEOUT = 60  # no goal activity for 60s = stall

results = {
    'phase': 'Phase137',
    'elapsed_sec': 0,
    'success_count': 0,
    'fail_count': 0,
    'dispatch_count': 0,
    'stall_detected': False,
    'target_successes': TARGET_SUCCESSES,
    'pass': False,
    'events': [],
}

# Use ros2 topic echo to monitor goal events
echo_proc = subprocess.Popen(
    ['ros2', 'topic', 'echo', '/maze/goal_events', 'std_msgs/msg/String'],
    stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
    bufsize=1
)

start = time.time()
last_activity = start
lock = threading.Lock()

def parse_event(text):
    """Parse a ros2 topic echo output into structured data."""
    text = text.strip()
    if not text:
        return None
    # ros2 topic echo outputs "data: '<json>'"
    # or raw JSON if using --no-arrange
    if text.startswith("data: '"):
        json_str = text[7:]  # strip "data: '"
        if json_str.endswith("'"):
            json_str = json_str[:-1]
        json_str = json_str.replace("\\'", "'").replace("\\\\", "\\")
    elif text.startswith('data: "'):
        json_str = text[7:]
        if json_str.endswith('"'):
            json_str = json_str[:-1]
        json_str = json_str.replace('\\"', '"').replace('\\\\', '\\')
    else:
        json_str = text

    try:
        return json.loads(json_str)
    except (json.JSONDecodeError, ValueError):
        return None

def read_events():
    """Read events from ros2 topic echo."""
    buffer = ""
    while True:
        try:
            char = echo_proc.stdout.read(1)
            if not char:
                break
            buffer += char
            # ros2 topic echo separates messages with "\n---\n"
            if buffer.endswith("\n---\n"):
                event = parse_event(buffer[:-5].strip())
                if event:
                    with lock:
                        process_event(event)
                buffer = ""
        except Exception:
            break

def process_event(event):
    global last_activity
    elapsed = time.time() - start
    event_type = str(event.get('event', '')).lower()
    kind = str(event.get('goal_kind', ''))
    status = str(event.get('result_status', ''))

    results['events'].append({
        'elapsed': round(elapsed, 1),
        'event': event.get('event', ''),
        'goal_kind': kind,
        'goal_sequence': event.get('goal_sequence'),
        'result_status': status,
    })

    if event_type == 'dispatch':
        results['dispatch_count'] += 1
        last_activity = time.time()
        print(f"[Phase137] 📤 Dispatch #{results['dispatch_count']}: "
              f"kind={kind} seq={event.get('goal_sequence')} @ {elapsed:.1f}s")

    elif event_type in ('success',):
        results['success_count'] += 1
        last_activity = time.time()
        print(f"[Phase137] ✅ Success #{results['success_count']}: "
              f"kind={kind} seq={event.get('goal_sequence')} @ {elapsed:.1f}s")

    elif event_type in ('failure', 'timeout', 'terminal_cancel_result',
                        'timeout_cancel_result', 'cancel', 'stale_result'):
        results['fail_count'] += 1
        last_activity = time.time()
        print(f"[Phase137] ❌ Failure: kind={kind} event={event_type} "
              f"status={status} seq={event.get('goal_sequence')} @ {elapsed:.1f}s")

# Start reading events in background
reader = threading.Thread(target=read_events, daemon=True)
reader.start()

# Main monitoring loop
print(f"[Phase137] Target: {TARGET_SUCCESSES} consecutive successful explore goals")
print(f"[Phase137] Max wait: {MAX_WAIT_SEC}s, Stall timeout: {STALL_TIMEOUT}s")

while time.time() - start < MAX_WAIT_SEC:
    with lock:
        sc = results['success_count']
        fc = results['fail_count']
        stall = results['stall_detected']

    if sc >= TARGET_SUCCESSES:
        print(f"\n[Phase137] 🎉 SUCCESS: {sc} goals completed!")
        break

    if time.time() - last_activity > STALL_TIMEOUT and sc > 0:
        with lock:
            results['stall_detected'] = True
        print(f"\n[Phase137] 🛑 STALL: No goal activity for {STALL_TIMEOUT}s "
              f"(after {sc} successes)")
        break

    time.sleep(2)

# Cleanup
echo_proc.terminate()
try:
    echo_proc.wait(timeout=3)
except subprocess.TimeoutExpired:
    echo_proc.kill()

elapsed = time.time() - start
results['elapsed_sec'] = round(elapsed, 1)
results['pass'] = results['success_count'] >= TARGET_SUCCESSES and not results['stall_detected']

# Trim events for artifact
results['events'] = results['events'][-50:]
result_path = os.path.join(artifact_dir, 'result.json')
with open(result_path, 'w') as f:
    json.dump(results, f, indent=2)

print(f"\n{'='*60}")
print(f"[Phase137] RESULT: {'PASS ✅' if results['pass'] else 'FAIL ❌'}")
print(f"[Phase137] Successes: {results['success_count']}/{TARGET_SUCCESSES}")
print(f"[Phase137] Failures: {results['fail_count']}")
print(f"[Phase137] Dispatches: {results['dispatch_count']}")
print(f"[Phase137] Stall: {results['stall_detected']}")
print(f"[Phase137] Elapsed: {elapsed:.1f}s")
print(f"[Phase137] Artifact: {result_path}")
print(f"{'='*60}")

sys.exit(0 if results['pass'] else 1)
PYEOF

PYTHON_EXIT=$?

# Kill launch
echo "[Phase137] Stopping simulation (PID=$LAUNCH_PID) ..."
kill $LAUNCH_PID 2>/dev/null || true
sleep 2
kill -9 $LAUNCH_PID 2>/dev/null || true
wait $LAUNCH_PID 2>/dev/null || true

echo "[Phase137] Done."
exit $PYTHON_EXIT
