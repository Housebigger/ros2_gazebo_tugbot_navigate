# Gazebo 真值红色轨迹（ground-truth trail in the Gazebo scene)

**Date:** 2026-07-08
**Workspace:** `ros2_ws_tugbot_nav_20260705`
**Status:** approved design → implementation plan next

## Goal

While the tugbot runs in the Gazebo simulation, draw a **red trail of its ground-truth
path directly in the Gazebo 3D scene**, so a visual run shows the complete route the
robot physically drove.

## User-selected decisions

1. **Pose source = Gazebo ground truth** (not the solver's ICP estimate). Division of
   labor: the Gazebo window shows physical truth (world + true trail); RViz shows the
   robot's belief (self-built map + ICP pose). The two windows become directly
   comparable by eye.
2. **Auto-start in GUI mode**: `tools/run_flood_fill_maze.sh` starts the trail drawer
   automatically when `HEADLESS=false`; headless batch runs never start it (nobody is
   watching). The script is also standalone-runnable by hand.

## Approach (chosen: pure gz CLI polling script)

A single standalone Python script polls the model's ground-truth pose via the `gz`
CLI and redraws a growing red `LINE_STRIP` marker in the Gazebo scene via the
standard gz-sim `/marker` service. **Zero new dependencies, zero ROS code changes.**

Verified feasibility facts (this machine, Gazebo Sim 8.11 Harmonic):
- `gz model -m tugbot -p` prints the model's world pose (CLI flags confirmed).
- `gz service -s /marker --reqtype gz.msgs.Marker --reptype gz.msgs.Boolean --req '…'`
  is the canonical scene-marker mechanism.
- The maze world (`tugbot_maze_world_20260528_clean_scaled2x`) includes the
  scene-broadcaster plugin; the robot model is named `tugbot`.
- gz-transport **Python bindings are NOT installed** (Jazzy vendor packages don't ship
  them) — which is fine for this approach and disqualifying for the alternatives.

Rejected alternatives:
- **gz-transport Python bindings**: requires installing a system package
  (`python3-gz-transport13`) for no real gain at ~2 Hz sampling.
- **C++ gz-transport node**: new package + CMake + gz dev headers to draw one line —
  massive overkill.
- **Spawning visual entities per segment** (`/world/<name>/create`): clutters the
  entity tree, slow; kept only as the last-resort fallback (see Risks).

## Architecture

```
Gazebo (running sim)                 tools/gz_trail.py (NEW, pure gz-side)
  model "tugbot" ──ground truth──>   poll:  gz model -m tugbot -p      (~2 Hz)
  GUI scene      <──red LINE_STRIP── redraw: gz service -s /marker
                                     (ns=tugbot_trail, same id ⇒ growing line)
```

No ROS node, launch file, or solver code is touched. Works identically for every
`pose_source` and every explorer — it visualizes the physical world, not the software.

## Components

### a. `tools/gz_trail.py` (NEW — the only code file)

Structured as **pure functions + a main loop** so the logic is unit-testable without
Gazebo:

- `parse_model_pose(text) -> Optional[Tuple[float, float, float]]`
  Parses `gz model -m <name> -p` textual output: locate the `Pose` header line, take
  the first bracketed triple `[x y z]`. Returns `None` on any malformed/empty input
  (sim not up, model missing) — the caller retries silently.
- `marker_request(points, ns='tugbot_trail', marker_id=1) -> str`
  Builds the protobuf-text request for `/marker`: `action: ADD_MODIFY`,
  `type: LINE_STRIP`, red material (diffuse + emissive 1,0,0,1), one `point` entry
  per sample with **z = 0.05** (lifted off the floor to avoid z-fighting).
- `should_record(new_point, last_point, min_dist) -> bool`
  Min-distance filter: record a new sample only when it moved ≥ `min_dist` from the
  last recorded point (default **0.10 m**) — turns keep their shape, straight lines
  stay sparse.
- Main loop: every `--period` (default **0.5 s**) poll pose → filter → append →
  redraw the full LINE_STRIP (same ns/id ⇒ Gazebo replaces the marker, so the line
  grows). Silent retry while the sim isn't up yet; clean exit on SIGINT/SIGTERM.
- CLI args (all with defaults, zero-arg runnable):
  `--model tugbot --period 0.5 --min-dist 0.10`.

Growth bound: a ~10-min run at 2 Hz with 0.10 m filtering ≈ hundreds of points; the
redraw request stays tens of KB — negligible.

### b. `tools/run_flood_fill_maze.sh` (MODIFY, ~6 lines)

- After the `ros2 launch … &` starts, **if `HEADLESS=false`**: start
  `python3 tools/gz_trail.py > "$ART/gz_trail.log" 2>&1 &` and record its PID.
- Add `gz_trail` to the `kill_all_sim` pattern list (lesson learned from the
  flood_fill_solver orphan-process leak), and kill it during teardown.
- Headless runs: never started. A trail-script crash **can never affect navigation**
  (independent process; the run script tolerates its absence).

## Behavior details

- **Trail lifecycle**: the trail lives with the sim session — after `EXIT_REACHED`
  the red line stays in the scene for inspection; closing the sim discards it; the
  next run starts a fresh GUI, so the trail resets automatically. No explicit
  DELETE action is sent.
- gz-transport uses its own default partition (independent of the randomized
  `ROS_DOMAIN_ID`), so the CLI calls reach the running sim without extra env setup.

## Error handling

- Sim not yet up / model not spawned → `parse_model_pose` returns `None` → silent
  retry next period.
- `gz service` call failure (e.g., GUI not ready) → log to the script's own log
  file, keep looping; never crash the run.
- Script killed mid-run → no cleanup needed (markers are scene state, discarded
  with the sim).

## Testing

- **Unit (`src/tugbot_maze/test/test_gz_trail.py` — plain pytest, no Gazebo):**
  - `parse_model_pose`: a captured real `gz model -p` output sample → correct
    (x, y, z); empty/garbage input → `None`.
  - `marker_request`: N input points → N `point` entries, `LINE_STRIP`,
    `ADD_MODIFY`, red material fields present, z lifted to 0.05.
  - `should_record`: below/above `min_dist` behaves correctly.
  (The script lives in `tools/`, so the test imports it via a path insert — same as
  any tools-script testing; if that proves awkward the logic functions live in the
  script and the test loads it with `importlib`.)
- **Gazebo acceptance (authority):** a GUI `online_slam` run — a red line grows
  behind the tugbot from entrance to exit, drawn on the floor of the Gazebo scene;
  the run's completion/collision metrics are unchanged (trail is read-only).

## Flagged risk (with fallback, resolved empirically in plan step 1)

**Marker availability/visibility in the Harmonic GUI** cannot be verified offline:
1. Whether the `/marker` service is served with the current GUI config (MinimalScene
   supports MarkerManager in Harmonic — expected yes, verify first).
2. Whether a `LINE_STRIP` renders visibly enough (GL line width may be 1 px).

Plan step 1 = empirical probe: with a running sim, send a hand-built marker request
and confirm it appears. Fallbacks, in order:
- Line too thin → switch the drawer to a **sphere-chain** (one small red `SPHERE`
  marker per recorded point, unique ids — beaded trail, clearly visible, and each
  update is a tiny constant-size request instead of a growing one).
- `/marker` unavailable → last resort: periodic thin red visual-only entities via
  `/world/<name>/create` (accepted as ugly; only if markers are truly absent).

## Out of scope

- Drawing the ICP-estimate trail (user chose ground-truth only).
- RViz-side trail (RViz already shows the belief; nav_msgs/Path can be a future nice-to-have).
- Any change to solver/launch/RViz configs — this feature is entirely `tools/`-side.

## Success criteria

1. GUI run: a red ground-truth trail grows behind the tugbot in the Gazebo scene,
   entrance → exit, and remains visible after completion.
2. Headless run: trail script not started; zero behavioral difference.
3. Navigation metrics (EXIT_REACHED, oracle collisions, duration) unchanged vs.
   baseline — the feature is read-only.
4. No orphan `gz_trail` processes after any run (clean teardown).
