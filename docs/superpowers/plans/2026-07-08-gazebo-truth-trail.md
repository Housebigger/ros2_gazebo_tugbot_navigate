# Gazebo Ground-Truth Red Trail Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Draw the tugbot's ground-truth path as a red LINE_STRIP marker directly in the Gazebo 3D scene during GUI runs.

**Architecture:** A standalone pure-Python script (`tools/gz_trail.py`) polls the model's ground-truth pose via `gz model -m tugbot -p` (~2 Hz), min-distance-filters the samples, and redraws a growing red `LINE_STRIP` via the gz-sim `/marker` service (`gz service` CLI). Zero ROS code, zero new dependencies, zero build steps (a `tools/` script). The run script auto-starts it only when `HEADLESS=false`.

**Tech Stack:** Python 3 stdlib (subprocess/re/argparse/signal), `gz` CLI (Gazebo Harmonic 8.11), bash (run-script integration), pytest.

**Spec:** `docs/superpowers/specs/2026-07-08-gazebo-truth-trail-design.md`

**Workspace root for all commands:** `ros2_ws_tugbot_nav_20260705/`. For running the unit tests, source once per shell:
```bash
cd ros2_ws_tugbot_nav_20260705
source /opt/ros/jazzy/setup.bash && source install/setup.bash
```
No `colcon build` is needed for any task in this plan (only `tools/` + test files change). Use `python3` (plain `python` does not exist on this machine).

---

## File structure

| File | Responsibility | Change |
|---|---|---|
| `tools/gz_trail.py` | The trail drawer: pure functions (pose parsing, distance filter, marker request building) + subprocess wrappers + main loop | **Create** |
| `src/tugbot_maze/test/test_gz_trail.py` | Unit tests for the pure functions (no Gazebo needed) | **Create** |
| `tools/run_flood_fill_maze.sh` | Auto-start/teardown of the trail drawer in GUI mode | **Modify** (~8 lines) |

**Testability note:** the pure functions (`parse_model_pose`, `should_record`, `marker_request`) are fully unit-tested. The subprocess wrappers and main loop are thin glue verified by an import/`--help` smoke check and the user-gated Gazebo acceptance (Task 6). The exact `gz model -p` output format could not be captured offline — the parser is written to tolerate both known separator styles (spaces and pipes), the unit test covers both, and Task 6 captures the real output to confirm.

---

### Task 1: `parse_model_pose` + `should_record` (pure functions, TDD)

**Files:**
- Create: `tools/gz_trail.py`
- Create: `src/tugbot_maze/test/test_gz_trail.py`

- [ ] **Step 1: Write the failing tests**

Create `src/tugbot_maze/test/test_gz_trail.py`:

```python
"""Unit tests for tools/gz_trail.py (pure functions only -- no Gazebo needed).
The script lives in the workspace tools/ dir, outside the package; import via path."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'tools'))
import gz_trail


# Two known `gz model -m <name> -p` output styles (separator differs across gz versions).
_POSE_OUTPUT_SPACES = """\
Requesting state for world [tugbot_maze_world_20260528_clean_scaled2x]...

Model: [8]
  - Name: tugbot
  - Pose [ XYZ (m) ] [ RPY (rad) ]:
    [10.500000 2.000000 0.100000]
    [0.000000 -0.000000 1.570000]
"""

_POSE_OUTPUT_PIPES = """\
Requesting state for world [tugbot_maze_world_20260528_clean_scaled2x]...

Model: [8]
  - Name: tugbot
  - Pose [ XYZ (m) ] [ RPY (rad) ]:
    [10.500000 | 2.000000 | 0.100000]
    [0.000000 | -0.000000 | 1.570000]
"""


def test_parse_pose_space_separated():
    assert gz_trail.parse_model_pose(_POSE_OUTPUT_SPACES) == (10.5, 2.0, 0.1)


def test_parse_pose_pipe_separated():
    assert gz_trail.parse_model_pose(_POSE_OUTPUT_PIPES) == (10.5, 2.0, 0.1)


def test_parse_pose_bad_input_returns_none():
    assert gz_trail.parse_model_pose('') is None
    assert gz_trail.parse_model_pose('Error: model not found') is None
    assert gz_trail.parse_model_pose('Pose but no numbers') is None


def test_should_record_first_point_always():
    assert gz_trail.should_record((1.0, 2.0, 0.0), None, 0.10) is True


def test_should_record_min_distance():
    last = (1.0, 2.0, 0.0)
    assert gz_trail.should_record((1.05, 2.0, 0.0), last, 0.10) is False   # 0.05 < 0.10
    assert gz_trail.should_record((1.20, 2.0, 0.0), last, 0.10) is True    # 0.20 >= 0.10
    # xy-plane distance only (z ignored)
    assert gz_trail.should_record((1.0, 2.0, 9.9), last, 0.10) is False
```

- [ ] **Step 2: Run to verify failure**

Run: `cd ros2_ws_tugbot_nav_20260705 && source /opt/ros/jazzy/setup.bash && source install/setup.bash && python3 -m pytest src/tugbot_maze/test/test_gz_trail.py -q`
Expected: FAIL — `ModuleNotFoundError: No module named 'gz_trail'`.

- [ ] **Step 3: Implement**

Create `tools/gz_trail.py`:

```python
#!/usr/bin/env python3
"""Draw the tugbot's ground-truth path as a red LINE_STRIP marker in the Gazebo scene.

Polls the model pose via the gz CLI (~2 Hz), min-distance filters the samples, and
redraws a growing /marker LINE_STRIP. Pure gz-side: no ROS, no extra dependencies.
Started automatically by tools/run_flood_fill_maze.sh when HEADLESS=false; also
standalone-runnable:  python3 tools/gz_trail.py [--model tugbot] [--period 0.5]
[--min-dist 0.10].
See docs/superpowers/specs/2026-07-08-gazebo-truth-trail-design.md.
"""
from __future__ import annotations
import argparse
import re
import signal
import subprocess
import sys
import time
from typing import List, Optional, Tuple

Point = Tuple[float, float, float]

# One bracketed numeric triple; separators are whitespace and/or a pipe (both styles
# of `gz model -p` output exist across gz versions).
_TRIPLE_RE = re.compile(
    r"\[\s*([-+\d.eE]+)\s*(?:\|\s*)?([-+\d.eE]+)\s*(?:\|\s*)?([-+\d.eE]+)\s*\]")


def parse_model_pose(text: str) -> Optional[Point]:
    """First bracketed [x y z] triple after a line containing 'Pose'; None if absent."""
    if not text:
        return None
    idx = text.find('Pose')
    if idx < 0:
        return None
    m = _TRIPLE_RE.search(text, idx)
    if not m:
        return None
    try:
        return (float(m.group(1)), float(m.group(2)), float(m.group(3)))
    except ValueError:
        return None


def should_record(new_point: Point, last_point: Optional[Point], min_dist: float) -> bool:
    """Record a sample only when it moved >= min_dist (xy-plane) from the last one."""
    if last_point is None:
        return True
    dx = new_point[0] - last_point[0]
    dy = new_point[1] - last_point[1]
    return (dx * dx + dy * dy) >= min_dist * min_dist
```

- [ ] **Step 4: Run to verify pass**

Run: `python3 -m pytest src/tugbot_maze/test/test_gz_trail.py -q`
Expected: PASS (6 tests).

- [ ] **Step 5: Commit**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git add ros2_ws_tugbot_nav_20260705/tools/gz_trail.py ros2_ws_tugbot_nav_20260705/src/tugbot_maze/test/test_gz_trail.py
git commit -F - <<'EOF'
feat: gz_trail pose parser + min-distance filter (TDD)

Pure functions for the Gazebo ground-truth trail drawer; parser tolerates
both space- and pipe-separated gz model -p output styles.

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>
EOF
```

---

### Task 2: `marker_request` builder (TDD)

**Files:**
- Modify: `tools/gz_trail.py` (append)
- Modify: `src/tugbot_maze/test/test_gz_trail.py` (append)

- [ ] **Step 1: Write the failing tests**

Append to `src/tugbot_maze/test/test_gz_trail.py`:

```python
def test_marker_request_line_strip_red():
    pts = [(1.0, 2.0, 0.0), (3.0, 4.0, 0.0)]
    req = gz_trail.marker_request(pts)
    assert 'type: LINE_STRIP' in req
    assert 'action: ADD_MODIFY' in req
    assert 'ns: "tugbot_trail"' in req
    assert 'id: 1' in req
    assert req.count('point {') == 2
    # red material, both diffuse and emissive so it reads bright regardless of lighting
    assert 'diffuse { r: 1 a: 1 }' in req
    assert 'emissive { r: 1 a: 1 }' in req
    # z lifted off the floor to avoid z-fighting
    assert 'z: 0.050' in req


def test_marker_request_point_coordinates():
    req = gz_trail.marker_request([(1.5, -2.25, 0.0)])
    assert 'point { x: 1.500 y: -2.250 z: 0.050 }' in req
```

- [ ] **Step 2: Run to verify failure**

Run: `python3 -m pytest src/tugbot_maze/test/test_gz_trail.py -q`
Expected: FAIL — `AttributeError: module 'gz_trail' has no attribute 'marker_request'`.

- [ ] **Step 3: Implement**

Append to `tools/gz_trail.py`:

```python
def marker_request(points: List[Point], ns: str = 'tugbot_trail', marker_id: int = 1,
                   z: float = 0.05) -> str:
    """Protobuf-text gz.msgs.Marker request: a red LINE_STRIP through the points.
    Re-sending with the same ns/id replaces the marker, so the line grows."""
    parts = [
        'ns: "%s"' % ns,
        'id: %d' % marker_id,
        'action: ADD_MODIFY',
        'type: LINE_STRIP',
        'material { ambient { r: 1 a: 1 } diffuse { r: 1 a: 1 } emissive { r: 1 a: 1 } }',
    ]
    for p in points:
        parts.append('point { x: %.3f y: %.3f z: %.3f }' % (p[0], p[1], z))
    return ', '.join(parts)
```

- [ ] **Step 4: Run to verify pass**

Run: `python3 -m pytest src/tugbot_maze/test/test_gz_trail.py -q`
Expected: PASS (8 tests).

- [ ] **Step 5: Commit**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git add ros2_ws_tugbot_nav_20260705/tools/gz_trail.py ros2_ws_tugbot_nav_20260705/src/tugbot_maze/test/test_gz_trail.py
git commit -F - <<'EOF'
feat: gz_trail red LINE_STRIP marker request builder (TDD)

Protobuf-text /marker request: ADD_MODIFY LINE_STRIP, red diffuse+emissive,
points lifted to z=0.05 against floor z-fighting.

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>
EOF
```

---

### Task 3: subprocess wrappers + main loop + CLI

**Files:**
- Modify: `tools/gz_trail.py` (append)

The glue below is deliberately thin (no unit test — verified by import + `--help` smoke here and by Gazebo in Task 6).

- [ ] **Step 1: Implement the wrappers and main loop**

Append to `tools/gz_trail.py`:

```python
def _poll_pose(model: str) -> Optional[Point]:
    """One ground-truth pose sample via the gz CLI; None while the sim isn't up."""
    try:
        r = subprocess.run(['gz', 'model', '-m', model, '-p'],
                           capture_output=True, text=True, timeout=5)
    except (subprocess.TimeoutExpired, OSError):
        return None
    if r.returncode != 0:
        return None
    return parse_model_pose(r.stdout)


def _redraw(points: List[Point], ns: str, marker_id: int,
            timeout_ms: int = 2000) -> bool:
    """Replace the trail marker with the full point list. False on failure."""
    req = marker_request(points, ns, marker_id)
    try:
        r = subprocess.run(
            ['gz', 'service', '-s', '/marker',
             '--reqtype', 'gz.msgs.Marker', '--reptype', 'gz.msgs.Empty',
             '--timeout', str(timeout_ms), '--req', req],
            capture_output=True, text=True, timeout=timeout_ms / 1000.0 + 3.0)
    except (subprocess.TimeoutExpired, OSError):
        return False
    return r.returncode == 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description='Red ground-truth trail in the Gazebo scene (poll + /marker).')
    ap.add_argument('--model', default='tugbot', help='Gazebo model name')
    ap.add_argument('--period', type=float, default=0.5, help='poll period, seconds')
    ap.add_argument('--min-dist', type=float, default=0.10,
                    help='min xy motion (m) before a new trail point is recorded')
    args = ap.parse_args(argv)

    stop = {'flag': False}

    def _sig(_signum, _frame):
        stop['flag'] = True

    signal.signal(signal.SIGINT, _sig)
    signal.signal(signal.SIGTERM, _sig)

    points: List[Point] = []
    fails = 0
    while not stop['flag']:
        p = _poll_pose(args.model)
        if p is not None and should_record(p, points[-1] if points else None,
                                           args.min_dist):
            points.append(p)
            if len(points) >= 2:                       # a strip needs 2+ points
                if _redraw(points, 'tugbot_trail', 1):
                    fails = 0
                else:
                    fails += 1
                    print('gz_trail: marker redraw failed (n=%d, consecutive=%d)'
                          % (len(points), fails), file=sys.stderr, flush=True)
        time.sleep(args.period)
    return 0


if __name__ == '__main__':
    sys.exit(main())
```

- [ ] **Step 2: Smoke-verify import and CLI**

Run:
```bash
python3 -c "import sys; sys.path.insert(0, 'tools'); import gz_trail; print('IMPORT_OK')"
python3 tools/gz_trail.py --help >/dev/null && echo HELP_OK
python3 -m pytest src/tugbot_maze/test/test_gz_trail.py -q
```
Expected: `IMPORT_OK`, `HELP_OK`, 8 tests PASS.

- [ ] **Step 3: Commit**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git add ros2_ws_tugbot_nav_20260705/tools/gz_trail.py
git commit -F - <<'EOF'
feat: gz_trail main loop (poll -> filter -> redraw, clean SIGTERM exit)

Silent retry while the sim is not up; redraw failures logged to stderr and
never fatal. Zero-arg runnable with tugbot defaults.

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>
EOF
```

---

### Task 4: run-script integration (GUI-only auto-start + clean teardown)

**Files:**
- Modify: `tools/run_flood_fill_maze.sh`

- [ ] **Step 1: Add `gz_trail` to the kill pattern list**

In `kill_all_sim()`, the pattern list currently ends with:
```bash
               flood_fill_solver \
               robot_state_publisher static_transform_publisher component_container rviz; do
```
Change those two lines to:
```bash
               flood_fill_solver gz_trail \
               robot_state_publisher static_transform_publisher component_container rviz; do
```

- [ ] **Step 2: Start the trail drawer in GUI mode**

Immediately AFTER the line
```bash
echo "[FLOODFILL] launch PID=$LAUNCH_PID DOMAIN=$ROS_DOMAIN_ID" | tee "$ART/run_meta.txt"
```
insert:
```bash
TRAIL_PID=""
if [ "$HEADLESS" = "false" ]; then
    # Ground-truth red trail in the Gazebo scene (GUI runs only; read-only wrt navigation).
    python3 "$WS/tools/gz_trail.py" > "$ART/gz_trail.log" 2>&1 &
    TRAIL_PID=$!
    echo "[FLOODFILL] gz_trail PID=$TRAIL_PID (red ground-truth trail)" | tee -a "$ART/run_meta.txt"
fi
```

- [ ] **Step 3: Kill it in teardown**

Immediately BEFORE the teardown line `kill -INT "$LAUNCH_PID" 2>/dev/null`, insert:
```bash
[ -n "$TRAIL_PID" ] && kill "$TRAIL_PID" 2>/dev/null
```

- [ ] **Step 4: Update the usage comment**

Change the usage header comment line
```bash
# Usage: tools/run_flood_fill_maze.sh [MAX_SECONDS] [HEADLESS] [USE_RVIZ] [POSE_SOURCE] [SENSE_DEBUG]
```
to
```bash
# Usage: tools/run_flood_fill_maze.sh [MAX_SECONDS] [HEADLESS] [USE_RVIZ] [POSE_SOURCE] [SENSE_DEBUG]
#   HEADLESS=false also auto-starts tools/gz_trail.py (red ground-truth trail in the Gazebo scene).
```

- [ ] **Step 5: Verify the script parses**

Run: `bash -n tools/run_flood_fill_maze.sh && echo SYNTAX_OK`
Expected: `SYNTAX_OK`.

- [ ] **Step 6: Commit**

```bash
cd /home/hyh/Desktop/agent_playground/playground_claude_code/ros2_gazebo_tugbot_navigate
git add ros2_ws_tugbot_nav_20260705/tools/run_flood_fill_maze.sh
git commit -F - <<'EOF'
feat: auto-start gz_trail in GUI runs, clean teardown

HEADLESS=false starts the red ground-truth trail drawer (log in the artifact
dir); teardown kills it and kill_all_sim covers strays.

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>
EOF
```

---

### Task 5: unit-suite regression gate

**Files:** none (validation only)

- [ ] **Step 1: Full suite**

Run:
```bash
cd ros2_ws_tugbot_nav_20260705
source /opt/ros/jazzy/setup.bash && source install/setup.bash
python3 -m pytest src/tugbot_maze/test/ -q 2>&1 | tail -3
```
Expected: `379 passed, 7 failed` — the 371 pre-existing passes + the 8 new gz_trail tests, and EXACTLY the 7 known pre-existing failures (6 × `test_wall_follow_maze_sim.py` + 1 × `test_maze_asset_and_config_are_present`). Zero new failures.

- [ ] **Step 2: Commit (only if a fix was needed)**

If nothing changed, skip.

---

### Task 6: Gazebo probe + acceptance (user-gated authority)

**Files:** none normally; `tools/gz_trail.py` only if the visibility fallback triggers.
**Do NOT auto-launch Gazebo — wait for the user's "set up the run".**

- [ ] **Step 1: Start a GUI run**

```bash
cd ros2_ws_tugbot_nav_20260705
tools/run_flood_fill_maze.sh 1000 false true online_slam
```
(The run script now auto-starts `gz_trail.py`; its log is `$ART/gz_trail.log`.)

- [ ] **Step 2: Probe the /marker service (the flagged risk)**

While the sim is up, in a second sourced shell:
```bash
# capture a REAL pose output sample to confirm the parser's format assumption
gz model -m tugbot -p
# hand-send a big red test sphere near the entrance; expect "data: true" reply
gz service -s /marker --reqtype gz.msgs.Marker --reptype gz.msgs.Empty --timeout 2000 \
  --req 'ns: "probe", id: 99, action: ADD_MODIFY, type: SPHERE, scale: {x: 0.3, y: 0.3, z: 0.3}, material: {ambient: {r: 1, a: 1}, diffuse: {r: 1, a: 1}}, pose: {position: {x: 2, y: 0, z: 0.3}}'
```
User confirms a red ball appears in the Gazebo GUI near the entrance. Then remove it:
```bash
gz service -s /marker --reqtype gz.msgs.Marker --reptype gz.msgs.Empty --timeout 2000 \
  --req 'ns: "probe", id: 99, action: DELETE_MARKER'
```
If the real `gz model -p` output format differs from both parser styles: update `_TRIPLE_RE` + add the real sample to the unit test (tools script — no build, instant effect).

- [ ] **Step 3: Watch the trail + collect acceptance evidence**

- User watches: a red line grows behind the tugbot from the entrance, through the maze, to the exit; it remains after `EXIT_REACHED`.
- Check `$ART/gz_trail.log` for repeated `redraw failed` lines (a few during startup are fine; persistent failure is not).
- After the run: `python3 tools/replay_collision_oracle.py "$(ls -td log/flood_fill_run_* | head -1)"` → expect `rate=0.000%`, EXIT_REACHED, duration ~545–575 s (trail is read-only; no regression).
- Confirm no orphans: `pgrep -af gz_trail` → empty after the script's teardown.

- [ ] **Step 4 (ONLY if the LINE_STRIP is invisible/too thin): sphere-chain fallback**

Append to `tools/gz_trail.py`:
```python
def sphere_request(point: Point, marker_id: int, ns: str = 'tugbot_trail',
                   z: float = 0.05, diameter: float = 0.08) -> str:
    """One small red sphere at the point (beaded-trail fallback when LINE_STRIP
    renders too thin). Unique marker_id per point -> constant-size requests."""
    return ', '.join([
        'ns: "%s"' % ns,
        'id: %d' % marker_id,
        'action: ADD_MODIFY',
        'type: SPHERE',
        'material { ambient { r: 1 a: 1 } diffuse { r: 1 a: 1 } emissive { r: 1 a: 1 } }',
        'scale { x: %.3f y: %.3f z: %.3f }' % (diameter, diameter, diameter),
        'pose { position { x: %.3f y: %.3f z: %.3f } }' % (point[0], point[1], z),
    ])
```
And in `main()`, replace the `if len(points) >= 2: ... _redraw(points, 'tugbot_trail', 1)` block with a per-point sphere send:
```python
            req = sphere_request(p, len(points))
            try:
                r = subprocess.run(
                    ['gz', 'service', '-s', '/marker',
                     '--reqtype', 'gz.msgs.Marker', '--reptype', 'gz.msgs.Empty',
                     '--timeout', '2000', '--req', req],
                    capture_output=True, text=True, timeout=5.0)
                ok = r.returncode == 0
            except (subprocess.TimeoutExpired, OSError):
                ok = False
            if ok:
                fails = 0
            else:
                fails += 1
                print('gz_trail: sphere send failed (n=%d, consecutive=%d)'
                      % (len(points), fails), file=sys.stderr, flush=True)
```
Add a unit test for `sphere_request` (type SPHERE, scale, pose position, red material), re-run the suite, re-run the Gazebo check, commit as
`fix: sphere-chain trail fallback (LINE_STRIP too thin in Harmonic GUI)` with the standard trailer.

- [ ] **Step 5: Report results to the user** (no commit for a pure-pass run — validation only).

---

## Self-Review

**1. Spec coverage:**
- Pure-gz polling script with parser/filter/builder as pure functions → Tasks 1–3. ✓
- Defaults (`--model tugbot --period 0.5 --min-dist 0.10`, z=0.05, ns/id, red diffuse+emissive) → Tasks 1–3 code. ✓
- GUI-only auto-start + artifact log + kill-pattern + teardown (orphan lesson) → Task 4. ✓
- Silent retry while sim down; redraw failures logged, never fatal; SIGINT/SIGTERM clean exit → Task 3 code. ✓
- Trail lifecycle (no DELETE; dies with the sim) → no DELETE anywhere in the drawer; probe sphere in Task 6 is deleted after use. ✓
- Unit tests (parser both formats, garbage input, filter, marker fields) → Tasks 1–2. ✓
- Flagged risk (marker availability/visibility) + ordered fallback → Task 6 Steps 2/4 with complete fallback code. ✓
- Success criteria (red line entrance→exit, headless unaffected, metrics unchanged, no orphans) → Task 6 Step 3 + Task 4. ✓

**2. Placeholder scan:** none — every code step shows complete code; commands have expected outputs.

**3. Type consistency:** `Point = Tuple[float, float, float]` used throughout; `marker_request(points, ns, marker_id, z)` defined in Task 2, called in Task 3 with matching args; `should_record(new_point, last_point, min_dist)` consistent between Tasks 1 and 3; `sphere_request` (Task 6 fallback) matches the same conventions. `HEADLESS`/`$WS`/`$ART` variables exist in the run script (verified). ✓
