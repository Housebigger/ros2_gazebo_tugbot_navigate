# center-twist-root-cause: diagnose (then fix) the center-corridor map twist

**Date:** 2026-07-23
**Phase:** center-twist-root-cause (follows tugbot-return, merged `5298e15`)
**Workspace:** NEW `ros2_ws_tugbot_nav_20260728` (rsync clone of `ros2_ws_tugbot_nav_20260727`, excluding build/install/log/__pycache__/.pytest_cache)
**Branch:** `center-twist-root-cause`

## Goal

The user observes small-amplitude fan/oblique wall twisting and overlap in the
accumulated 3D map when the tugbot traverses the maze CENTER corridor.
Diagnose the root cause with a discriminating A/B instrument before touching
anything, then fix per evidence in the same phase.

## User adjudications (recorded)

1. **Phenomenology = B, fan/oblique twist.** Walls lean or fan (wider at
   range) rather than clean parallel double-shells — the signature of a YAW
   residual in the believed pose at accumulation time (small angle x distance
   amplifies at far walls). This sets the first forensic cut toward yaw, but
   the instruments stay two-sided (yaw AND position both measured).
2. **Phase boundary = A, diagnose + fix in one phase.** Measure first, fork on
   evidence, land the fix where the evidence points, re-verify with the same
   instruments. HONEST DOWNGRADE CLAUSE: if forensics rule the root cause
   unfixable in-phase (or requiring a redesign), the phase closes as a
   root-cause verdict document (precedent: P3-routing, alongtrack-lag).
3. Direct-fix-first (option C) explicitly declined — measure-first
   methodology holds (the localization-root-cause phase's lesson: levers
   built on unmeasured hypotheses have always failed here).

## Historical priors (why the center is special)

- The maze center is the project's weakest-observability region: open-region
  wobble, `_has_local_interior` disabling the localizer on corridor
  centerlines (21 probes / 18 off in the alongtrack forensics), the grid
  aliasing attractor, the historical central-trap yaw deviation.
- Both aliasing gates (yaw 0.5 rad, position 1.0 m) intercept LARGE
  deviations only. A "small-amplitude" twist is exactly the sub-threshold
  residual band the gates pass through by design.
- Odom is drift-free in this sim (world-anchored OdometryPublisher — measured
  at tugbot-return smoke: odom0 == gt0, err 0.0000 over an arc). If yaw
  residual exists, ICP accepts are ADDING error relative to a perfect prior —
  that asymmetry is itself diagnostic.
- gz gpu_lidar raycasts a full revolution at one sim instant (no intra-scan
  motion distortion in sim) — the classic rotating-lidar smear is excluded a
  priori; timing candidates are inter-message (stamp pairing, TF
  interpolation), not intra-scan.

## Chosen approach (of three)

**Discriminating A/B forensics, then fix per evidence** (chosen): a
ground-truth parallel accumulator splits the hypothesis space in half in one
run. Rejected: correlation-only forensics (POSEDIAG alone cannot cleanly
separate localization-layer from timing-layer causes — it becomes the
DECOMPOSITION layer inside approach one instead); direct-fix-first (declined
by adjudication 3).

## Design

### 1. Instruments (all EVAL-ONLY, default OFF, zero production contact)

**a. GT parallel accumulator — `tugbot_maze/gt_cloud_accumulator.py` (new
standalone node).** Subscribes `/lidar/points` + `/gt/dynamic_pose` and
accumulates the SAME clouds transformed by the ground-truth pose into a
separate voxel set. Rules:

- Red line: ground truth enters ONLY this instrument; it publishes nothing
  any control or navigation node consumes (`/maze/gt_cloud_map_3d` latched,
  visualization/analysis only).
- Same accumulation parameters as production (0.05 m voxel, range <= 8 m
  sensor-frame filter, map-frame z-clip) so the two maps are same-scale
  comparable. The gt pose is composed to MAP frame with the same entrance
  anchor the production chain uses (world -> map offset from the maze
  metadata), so both maps live in the same frame.
- Stamp-pairing fairness: production looks up TF AT the cloud stamp (the
  ackermann-phase 5d discipline). The gt instrument buffers gt samples
  (~58 Hz) and pairs each cloud with the nearest-stamp gt sample
  (interpolation unnecessary at 58 Hz vs 10 Hz cloud rate; nearest <= ~9 ms).
  This mirrors the production pairing discipline — the fairness premise of
  the A/B.
- `/gt/dynamic_pose` frame ids are EMPTY (bridge drops gz names —
  tugbot-return finding); the node uses the transforms[0] index convention
  with the same caveat comment as the smoke tool.
- Heisenberg guard: NO heavy publishing during the run (the historical
  ghost-forensics lesson: load alters behavior). Accumulate in memory, log
  `GTCLOUD voxels=N` lines at the production accumulator's cadence, dump the
  final voxel set to a file in the artifact dir at shutdown; latched publish
  at most once per 30 s for optional RViz inspection.
- Launched ONLY by the diagnostic run configuration (a launch arg
  `gt_cloud_diag:=true` on the explore launch, default false, wired like
  `pose_diag`); never in production runs.

**b. Twist metric — `tools/map_twist_metric.py` (offline analysis).** Input:
two dumped voxel-set files (production map dump + gt map dump) or exported
cloud files from the same run. For each configured CENTER-REGION wall band
(map-frame boxes derived at plan time from the maze instance metadata plus
the user's observed twist location), compute the wall-normal spread (voxel
count, thickness p50/p95, and a fitted-plane angle vs the true wall axis).
Output: per-band table production vs gt. This turns "small-amplitude twist"
into numbers and becomes the phase's improvement metric. (The production
accumulator needs a final-dump capability for this: reuse its existing
export path if one exists; otherwise capture the latched
`/maze/cloud_map_3d` topic once at run end from the run script — decided at
plan time, whichever avoids touching the frozen accumulator.)

**c. POSEDIAG decomposition (existing, unchanged).** `pose_diag:=true` runs +
`tools/pose_decompose.py` extended analysis (new flags or a sibling script —
no behavior change to the solver): yaw residual (solver vs gt) time series
bucketed by (i) region: center band vs rest, (ii) ICP event: accept / reject
/ localizer-off windows, (iii) motion phase. Position residual reported
alongside (two-sided per adjudication 1).

### 2. Forensic protocol (measure first, then fork)

Two instrumented headless runs (`pose_diag:=true gt_cloud_diag:=true`), then
the decision tree:

- **(a) GT map clean, production map twisted** -> the twist is 100%
  believed-pose error. POSEDIAG then pins the injection layer: sub-threshold
  ICP-accept yaw wobble concentrated in the center band? dead-reckoning
  windows while `_has_local_interior` holds the localizer off? something
  else? Fix candidates are ranked BY THE MEASURED mechanism; per project
  precedent the fix lives in the solver gate layer (localizer files stay
  byte-untouched).
- **(b) Both maps twisted** -> localization exonerated; the cause is in the
  sensing/timing/pairing layer (cloud stamp vs TF interpolation, publish
  cadence). Forensics shift to stamp deltas and TF-time alignment.
- **(c) Both maps clean** -> the phenomenon lives outside the accumulation
  path (rendering/export) or did not reproduce; re-observe (GUI run with the
  instrument on) before any further step.

The verdict (whichever branch) is written into the spec addendum with the
numbers BEFORE any fix lands.

### 3. Fix principles (branch (a)/(b) only, after the verdict)

- Smallest lever at the choke point the evidence names; solver gate layer
  preferred (both aliasing gates live there; localizers byte-frozen through
  three phases).
- Sim-scoped odom trust is sanctioned (drift-free premise measured), but any
  new gate/clamp records the real-drifting-odom caveat like its two
  predecessors.
- No tuning loops: one evidence-backed fix, re-run the same two instrumented
  runs, compare the twist metric. If the first fix misses, back to the
  verdict table — not parameter roulette.

### 4. Acceptance

- Hard gates unchanged and non-regressing: EXIT_REACHED, oracle 0.000%, both
  aliasing gates silent, zero ERROR, suite at the frozen baseline
  (10 failed / 544 passed, every delta adjudicated).
- Targeted metric: center-region wall thickness p95 (production map)
  improves significantly toward the gt map's value; global final voxels do
  NOT increase (twist removal should reduce or hold, ~420-450k band).
- GUI: user visual verdict on the center corridor, final keep/drop.
- Downgrade path: verdict-document closure per adjudication 2 if forensics
  say unfixable-in-phase.

### Non-goals

- No localizer file changes (three-phase byte-frozen precedent).
- No production accumulator changes unless branch (b) evidence NAMES it (the
  stamp-based TF fix from ackermann-physics stays).
- No motion-layer changes (the twist is a mapping/localization question; the
  tugbot's motion passed acceptance).
- No real-drifting-odom generalization work (recorded backlog stays parked).
