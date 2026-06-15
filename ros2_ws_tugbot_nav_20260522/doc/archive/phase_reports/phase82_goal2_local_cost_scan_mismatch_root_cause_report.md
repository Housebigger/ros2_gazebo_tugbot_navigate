# Phase82: Goal2 local-cost / scan mismatch root-cause diagnosis

Status: completed / stopped at Phase82.

Primary conservative root-cause classification:

`FOOTPRINT_OR_WEDGE_PROJECTION_SUSPECTED`

Secondary supported signal:

`INFLATION_SPILLOVER_SUSPECTED`

Important nuance: Phase82 is a diagnosis-only phase. The result does not authorize tuning `inflation`, `robot_radius`, `clearance_radius_m`, MPPI/Nav2/controller parameters, branch scoring, centerline gates, directional readiness, or fallback/terminal acceptance. It only explains the Phase81 split between a physically open scan sector and a locally blocked Nav2 execution corridor.

## Scope and guardrails

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
- Phase83 not entered.

## Required source review

Read before implementation:

- `doc/doc_proposal/obstacle_reproduction_handoff_workflow.md`
- `doc/doc_report/phase80_goal2_near_goal_forward_open_corridor_diagnostic_classification_report.md`
- `doc/doc_report/phase81_goal2_forward_open_machine_evidence_capture_report.md`
- `log/phase81_goal2_forward_open_machine_evidence_capture/phase81_goal2_forward_open_machine_evidence_capture_raw_capture.json`
- `log/phase81_goal2_forward_open_machine_evidence_capture/phase81_goal2_forward_open_machine_evidence_capture_analysis.json`
- `log/phase81_goal2_forward_open_machine_evidence_capture/phase81_goal2_forward_open_machine_evidence_capture_minimal_field_summary.md`
- Phase81 skill reference `references/tugbot_maze_phase81_goal2_forward_open_machine_evidence_capture.md`

## New files

Analyzer:

- `tools/analyze_phase82_goal2_local_cost_scan_mismatch_root_cause.py`

Focused tests:

- `src/tugbot_maze/test/test_phase82_goal2_local_cost_scan_mismatch_root_cause.py`

Artifacts:

- `log/phase82_goal2_local_cost_scan_mismatch_root_cause/phase82_goal2_local_cost_scan_mismatch_root_cause.json`
- `log/phase82_goal2_local_cost_scan_mismatch_root_cause/phase82_goal2_local_cost_scan_mismatch_root_cause_minimal_summary.md`

Report:

- `doc/doc_report/phase82_goal2_local_cost_scan_mismatch_root_cause_report.md`

## Analyzer contract

The analyzer reads Phase81 raw capture plus Phase81 analysis and checks the requested candidate root causes:

- `INFLATION_SPILLOVER_SUSPECTED`
- `FOOTPRINT_OR_WEDGE_PROJECTION_SUSPECTED`
- `CORRIDOR_SAMPLING_TOO_WIDE_SUSPECTED`
- `POSE_TF_PROJECTION_MISMATCH_SUSPECTED`
- `REAL_EXECUTION_CLEARANCE_TOO_NARROW`
- `COSTMAP_STALE_OR_OBSERVATION_MISMATCH`
- `ROOT_CAUSE_INSUFFICIENT_EVIDENCE`

If raw scan, raw local costmap, footprint, robot pose, or Phase81 analysis are missing, the analyzer returns `ROOT_CAUSE_INSUFFICIENT_EVIDENCE` and records evidence gaps. It does not fabricate missing scan/local-cost/TF/footprint evidence.

## Phase81 split evidence reused

Phase81 forward-open evidence was sufficient but split:

- raw scan physical forward sector open;
- local costmap robot-forward corridor blocked/high-risk.

Phase81 key values:

- `source_phase81_forward_open_classification`: `FORWARD_OPEN_CORRIDOR_BLOCKED`
- scan min front clearance: `1.4389289617538452 m`
- local-cost high ratio: `0.5`
- local-cost lethal ratio: `0.3333333333333333`
- local-cost unknown ratio: `0.0`

Phase82 preserves that split instead of flattening it into a single physical-obstacle claim.

## Phase82 candidate checks

### 1. Footprint / front-wedge projection

Result: supported.

Key evidence:

- Captured footprint width: `0.720000183301383 m`
- Captured footprint forward extent: `0.35999999827337903 m`
- Phase82 sampled corridor width: `0.6 m`
- `footprint_nearly_as_wide_as_corridor`: `true`
- `footprint_width_m > sampled_corridor_width_m`

Interpretation:

The sampled execution corridor used for Phase81 local-cost classification is narrower than the captured footprint envelope. Therefore, a forward local-cost corridor can become high/lethal even while the raw scan sector directly ahead looks physically open. This supports `FOOTPRINT_OR_WEDGE_PROJECTION_SUSPECTED` as the primary conservative label.

This is not a decision to change footprint, wedge, robot radius, clearance radius, or any controller parameter. It is only a projection/geometry mismatch diagnosis.

### 2. Inflation spillover

Result: supported as a secondary signal.

Key evidence:

- physical scan forward open: `true`
- local-cost corridor blocked: `true`
- gradient cells present: `true`
- gradient cells in corridor: `100`
- lethal cells in corridor: `80`
- high-cost ratio: `0.5`
- lethal ratio: `0.3333333333333333`
- high-cost band appears one-sided in the sampled corridor.

Interpretation:

Open scan plus local-cost gradient/high/lethal band is compatible with inflation spillover into the execution corridor. However, because the captured footprint envelope is itself wider than the sampled Phase81 corridor, Phase82 uses the more specific primary label `FOOTPRINT_OR_WEDGE_PROJECTION_SUSPECTED` and records inflation spillover as a supported secondary signal.

This does not authorize inflation tuning in Phase82.

### 3. Pose / TF projection mismatch

Result: not supported.

Key evidence:

- map pose available: `true`
- odom pose available: `true`
- TF transform records available: `true`
- map/odom XY delta: `0.006198585070917654 m`
- map/odom yaw delta: `0.006979999999999764 rad`

Interpretation:

The available pose-frame discrepancy is small and does not explain the local-cost/scan mismatch.

### 4. Costmap stale / observation mismatch

Result: not supported.

Key evidence:

- scan stamp: `3994.602`
- local_costmap stamp: `3994.218`
- footprint stamp: `3994.596`
- odom stamp: `3994.632`
- max sensor-to-costmap delta: `0.41400000000021464 sec`
- threshold: `2.0 sec`

Interpretation:

The costmap is not stale enough by the Phase82 threshold to classify the mismatch as stale/observation mismatch.

### 5. Corridor sampling too wide

Result: not supported.

Key evidence:

- sampled corridor half width: `0.3 m`
- sampled corridor width: `0.6 m`
- footprint width: `0.720000183301383 m`
- high-cost band is one-sided, but the sampled corridor is not wider than the footprint envelope.

Interpretation:

The evidence does not support the claim that the Phase81 sampling corridor was too wide relative to the robot. If anything, the captured footprint is wider than the sampled corridor.

### 6. Real execution clearance too narrow

Result: not supported by available scan-width evidence.

Key evidence:

- estimated free scan width at forward slice `0.75 m`: `1.7594730571974257 m`
- threshold width: `0.72 m`
- scan45 min clearance: `0.6789765357971191 m`
- scan90 min clearance: `0.469503253698349 m`

Interpretation:

Wider scan sectors see nearby side geometry, but the forward slice width estimate is much larger than the robot envelope threshold. Phase82 therefore does not classify the root cause as real physical execution clearance too narrow.

## Final classification

`FOOTPRINT_OR_WEDGE_PROJECTION_SUSPECTED`

Rationale:

- Phase81 proved the direct forward scan sector was physically open by raw scan.
- Phase81 proved the robot-forward local-cost corridor was high/lethal.
- Phase82 found the captured footprint envelope width (`~0.72 m`) is wider than the Phase81 sampled corridor width (`0.6 m`).
- The local-cost band includes gradient/high/lethal structure consistent with inflation spillover, but the footprint/wedge projection relationship is the more specific conservative diagnosis available from the artifacts.
- Pose/TF mismatch and stale costmap mismatch are not supported by the captured data.
- Real physical clearance too narrow is not supported by the scan-width estimate.

## Verification

RED proof:

```text
PYTHONDONTWRITEBYTECODE=1 pytest -q src/tugbot_maze/test/test_phase82_goal2_local_cost_scan_mismatch_root_cause.py
```

Before the analyzer and artifacts existed, this failed as expected:

```text
5 failed
missing Phase82 analyzer
missing Phase82 JSON artifact
```

Analyzer execution:

```bash
PYTHONDONTWRITEBYTECODE=1 python3 tools/analyze_phase82_goal2_local_cost_scan_mismatch_root_cause.py \
  --raw-capture log/phase81_goal2_forward_open_machine_evidence_capture/phase81_goal2_forward_open_machine_evidence_capture_raw_capture.json \
  --phase81-analysis log/phase81_goal2_forward_open_machine_evidence_capture/phase81_goal2_forward_open_machine_evidence_capture_analysis.json \
  --output-json log/phase82_goal2_local_cost_scan_mismatch_root_cause/phase82_goal2_local_cost_scan_mismatch_root_cause.json \
  --minimal-summary log/phase82_goal2_local_cost_scan_mismatch_root_cause/phase82_goal2_local_cost_scan_mismatch_root_cause_minimal_summary.md
```

Observed analyzer output:

```text
root_cause_classification=FOOTPRINT_OR_WEDGE_PROJECTION_SUSPECTED
evidence_sufficient=true
evidence_gaps=[]
```

Final verification command:

```bash
PYTHONDONTWRITEBYTECODE=1 pytest -q src/tugbot_maze/test/test_phase82_goal2_local_cost_scan_mismatch_root_cause.py && \
  printf '\n--- syntax ---\n' && \
  PYTHONDONTWRITEBYTECODE=1 python3 -m py_compile \
    tools/analyze_phase82_goal2_local_cost_scan_mismatch_root_cause.py \
    src/tugbot_maze/test/test_phase82_goal2_local_cost_scan_mismatch_root_cause.py && \
  printf '\n--- strategy/config diff ---\n' && \
  git diff -- src/tugbot_maze/tugbot_maze/maze_explorer.py src/tugbot_navigation/config src/tugbot_bringup/launch src/tugbot_maze/tugbot_maze | cat && \
  printf '\n--- phase82 artifact summary ---\n' && \
  python3 - <<'PY'
import json
from pathlib import Path
p=Path('log/phase82_goal2_local_cost_scan_mismatch_root_cause/phase82_goal2_local_cost_scan_mismatch_root_cause.json')
data=json.loads(p.read_text())
print('classification', data['root_cause_classification'])
print('evidence_sufficient', data['evidence_sufficient'])
print('footprint_width_m', data['footprint_projection']['width_m'])
print('sampled_corridor_width_m', data['candidate_checks']['footprint_or_wedge_projection']['sampled_corridor_width_m'])
print('inflation_secondary_supported', data['candidate_checks']['inflation_spillover']['supported'])
print('phase83_entered', data['phase83_entered'])
PY
```

Observed output:

```text
.....                                                                    [100%]
5 passed in 0.07s

--- syntax ---

--- strategy/config diff ---

--- phase82 artifact summary ---
classification FOOTPRINT_OR_WEDGE_PROJECTION_SUSPECTED
evidence_sufficient True
footprint_width_m 0.720000183301383
sampled_corridor_width_m 0.6
inflation_secondary_supported True
phase83_entered False
```

Interpretation: focused tests passed, Python syntax passed, and the strategy/config diff guard was empty.

Final pycache cleanup after `py_compile`:

```text
removed_pycache_count 2
tools/__pycache__
src/tugbot_maze/test/__pycache__
remaining_pycache_count 0
```

## Stop condition

Phase82 stops after root-cause diagnosis, JSON artifact, minimal summary, focused tests, and report. It does not enter Phase83 and does not implement any repair.
