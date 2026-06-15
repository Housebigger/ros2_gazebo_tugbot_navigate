# Phase81: Goal2 forward-open machine evidence capture

Status: completed / stopped at Phase81.

Forward-open machine evidence classification: `FORWARD_OPEN_CORRIDOR_BLOCKED`

Important nuance: raw `/scan` supports physical forward clearance, but raw `/local_costmap/costmap` in the robot-forward corridor is high/lethal enough that the combined machine evidence is classified as blocked for Nav2 execution.

## Scope and guardrails

Phase81 is evidence capture and classification only. It does not repair or tune the robot.

Guardrails preserved:

- No maze_explorer strategy changed.
- No branch scoring changed.
- No centerline gate changed.
- No directional readiness override changed.
- No fallback/terminal acceptance changed.
- No Nav2/MPPI/controller tuning.
- No inflation / robot_radius / clearance_radius_m / map threshold tuning.
- No autonomous exploration success claimed.
- No exit success claimed.
- This classification is not fallback/terminal acceptance and not a Nav2 parameter final diagnosis.

Phase82 not entered.

## Required source review

Read before implementation:

- `doc/doc_proposal/obstacle_reproduction_handoff_workflow.md`
- `doc/doc_report/phase80_goal2_near_goal_forward_open_corridor_diagnostic_classification_report.md`
- `log/phase79_goal2_timeout_bounded_reproduction_handoff/phase79_goal2_timeout_bounded_reproduction_handoff_minimal_field_summary.md`
- `log/phase79_goal2_timeout_bounded_reproduction_handoff/phase79_goal2_timeout_bounded_reproduction_handoff_trigger_detected.json`
- `log/phase80_goal2_near_goal_forward_open_corridor_diagnostic_classification/phase80_goal2_near_goal_forward_open_corridor_diagnostic_classification.json`
- Phase79 runbook reference: `tools/run_phase79_goal2_timeout_bounded_reproduction_handoff.sh`

## Reproduction / held-scene basis

Phase81 reused the already-held visible Phase79 Goal2 timeout scene rather than starting a new long autonomous run.

Live process check before capture showed the visible SLAM/Nav2/Gazebo/RViz scene still active, with no active `maze_explorer` process in the process list. The ROS graph exposed required topics:

- `/scan`
- `/local_costmap/costmap`
- `/local_costmap/published_footprint`
- `/local_costmap/footprint`
- `/odom`
- `/tf`
- `/goal_pose`

This satisfies the Phase81 requirement to capture raw scan/local-costmap/footprint/TF/odom evidence at the held Goal2 problem point without continuing exploration.

## New files

Analyzer:

- `tools/analyze_phase81_goal2_forward_open_machine_evidence.py`

Live raw evidence recorder:

- `tools/record_phase81_goal2_forward_open_machine_evidence.py`

Bounded capture runbook:

- `tools/run_phase81_goal2_forward_open_machine_evidence_capture.sh`

Focused tests:

- `src/tugbot_maze/test/test_phase81_goal2_forward_open_machine_evidence.py`

Artifacts:

- `log/phase81_goal2_forward_open_machine_evidence_capture/phase81_goal2_forward_open_machine_evidence_capture_raw_capture.json`
- `log/phase81_goal2_forward_open_machine_evidence_capture/phase81_goal2_forward_open_machine_evidence_capture_analysis.json`
- `log/phase81_goal2_forward_open_machine_evidence_capture/phase81_goal2_forward_open_machine_evidence_capture_minimal_field_summary.md`
- `log/phase81_goal2_forward_open_machine_evidence_capture/phase81_goal2_forward_open_machine_evidence_capture_preflight.txt`
- `log/phase81_goal2_forward_open_machine_evidence_capture/phase81_goal2_forward_open_machine_evidence_capture_ros_graph_snapshot.txt`

## Analyzer contract

The analyzer computes:

- front scan sector clearance using `/scan`, centered on robot-forward in scan frame.
- front rectangular corridor local-cost distribution from raw `/local_costmap/costmap`.
- corridor cost counts and ratios:
  - unknown/free/high/lethal counts.
  - high-cost ratio using threshold `>=70`.
  - lethal ratio using threshold `>=99`.
- footprint / odom / TF availability in the raw capture.
- final classification:
  - `FORWARD_OPEN_CORRIDOR_CONFIRMED`
  - `FORWARD_OPEN_CORRIDOR_BLOCKED`
  - `FORWARD_OPEN_EVIDENCE_INSUFFICIENT`

If raw scan, raw local costmap, or robot pose in the costmap frame is missing, the analyzer must output `FORWARD_OPEN_EVIDENCE_INSUFFICIENT` and record evidence gaps. It must not fabricate a positive proof.

## Phase81 raw machine evidence

Captured live at the held Goal2 problem scene:

- raw scan available: `true`
- raw local costmap available: `true`
- footprint available: `true`
- odom available: `true`
- TF available: `true`
- terminal pose from capture: `[2.425328078972796, 1.0314533025962633, 1.5956958930612246]`
- Goal2 target: `[2.057855221699651, 1.0261005743935105]`

Front scan sector evidence:

- scan frame: `tugbot/scan_omni/scan_omni`
- front sector half angle: `20 deg`
- finite front-sector samples: `100`
- min front clearance: `1.4389289617538452 m`
- mean front clearance: `5.6599742031097415 m`

Interpretation: the physical raw scan sector in front of the robot looks open by the Phase81 scan threshold.

Front local-cost corridor evidence:

- costmap frame: `odom`
- robot pose source for costmap projection: `odom`
- corridor length: `1.0 m`
- corridor half width: `0.3 m`
- sampled cells: `240`
- free cells: `120`
- high-cost cells: `120`
- lethal cells: `80`
- unknown cells: `0`
- high-cost ratio: `0.5`
- lethal ratio: `0.3333333333333333`
- max cost: `99`

Interpretation: although scan clearance is physically open, the local costmap directly in the robot-forward corridor is not execution-open; it contains high/lethal cost over a large fraction of the corridor.

## Classification

`FORWARD_OPEN_CORRIDOR_BLOCKED`

Reasons:

- `front_corridor_lethal_ratio_high: 0.333`
- `front_corridor_high_cost_ratio_high: 0.500`

Evidence gaps:

- none for this Phase81 capture.

This resolves Phase80's `FORWARD_OPEN_EVIDENCE_INSUFFICIENT` for the held Goal2 scene: the new raw machine evidence is sufficient, but it does not confirm a Nav2-execution-open corridor. It shows a split condition:

- physical scan forward clearance is open;
- local-cost execution corridor is blocked/high-risk.

This remains consistent with the Phase75/80 recovery-loop/local-cost diagnosis and does not authorize algorithm repair in Phase81.

## Verification

RED proof:

```text
PYTHONDONTWRITEBYTECODE=1 pytest -q src/tugbot_maze/test/test_phase81_goal2_forward_open_machine_evidence.py
```

Before the analyzer/recorder/runbook/report/artifacts existed, this failed as expected:

```text
4 failed
missing Phase81 analyzer
Phase81 live raw evidence recorder is missing
```

Capture command:

```bash
PHASE81_CAPTURE_DURATION_SEC=6 PYTHONDONTWRITEBYTECODE=1 tools/run_phase81_goal2_forward_open_machine_evidence_capture.sh
```

Observed capture output:

```text
scan_available=true
local_costmap_available=true
footprint_available=true
odom_available=true
robot_pose_frames=["map", "odom"]
forward_open_classification=FORWARD_OPEN_CORRIDOR_BLOCKED
```

Final verification command:

```bash
PYTHONDONTWRITEBYTECODE=1 pytest -q src/tugbot_maze/test/test_phase81_goal2_forward_open_machine_evidence.py && \
  printf '\n--- syntax ---\n' && \
  PYTHONDONTWRITEBYTECODE=1 python3 -m py_compile \
    tools/analyze_phase81_goal2_forward_open_machine_evidence.py \
    tools/record_phase81_goal2_forward_open_machine_evidence.py \
    src/tugbot_maze/test/test_phase81_goal2_forward_open_machine_evidence.py && \
  printf '\n--- strategy/config diff ---\n' && \
  git diff -- src/tugbot_maze/tugbot_maze/maze_explorer.py src/tugbot_navigation/config src/tugbot_bringup/launch src/tugbot_maze/tugbot_maze | cat
```

Observed output:

```text
....                                                                     [100%]
4 passed in 0.04s

--- syntax ---

--- strategy/config diff ---

--- phase81 artifact summary ---
classification FORWARD_OPEN_CORRIDOR_BLOCKED
scan_min_clearance_m 1.4389289617538452
local_high_cost_ratio 0.5
local_lethal_ratio 0.3333333333333333
phase82_entered False
```

Interpretation: focused tests passed, Python syntax passed, and the strategy/config guard diff was empty.

Final report token check also confirmed:

```text
FORWARD_OPEN_CORRIDOR_BLOCKED True
FORWARD_OPEN_CORRIDOR_CONFIRMED True
FORWARD_OPEN_EVIDENCE_INSUFFICIENT True
Phase82 not entered True
```

Final pycache cleanup after `py_compile`:

```text
removed_pycache_count 2
tools/__pycache__
src/tugbot_maze/test/__pycache__
remaining_pycache_count 0
```

## Stop condition

Phase81 stops after evidence capture, classification, artifacts, focused tests, and report. It does not enter Phase82 and does not implement a repair.
