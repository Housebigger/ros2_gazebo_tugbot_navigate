---
title: Phase26-Final Diagnostic Closure Report
date: 2026-05-27
workspace: ros2_ws_tugbot_nav_20260522
phase: 26-Final
scope: diagnostics closure only
---

# Phase26-Final Diagnostic Closure Report

## 0. Scope

This is a diagnostics closure report for Phase20 through Phase26X. It does not start Gazebo/Nav2, does not fetch source, does not modify runtime behavior, and does not enter Phase27.

Guardrails:

- Phase27 remains blocked.
- Do not change branch selection.
- Do not tune Nav2/controller parameters.
- Do not promote or reject the `CostCritic.cost_weight=2.75` candidate from this closure.
- Do not change MPPI controller behavior.
- Treat the MPPI selected-control near-zero cause as unresolved until source-level diagnostics provide evidence.

## 1. Diagnostic chain summary: Phase20 to Phase26X

### Phase20: multi-run timeout taxonomy

Phase20 showed that timeout behavior was not a single stable dispatch-time geometry or branch-selection problem.

Key findings:

- Repeated baseline runs did not all reach the exit.
- Timeout counts varied across runs.
- One repeat shifted into blocked/blacklist behavior.
- Timeout goals often shared narrow-passage/high-cost features, but those features were not exclusive to failed goals.

Decision:

- Do not enter branch scoring or branch-selection intervention from Phase20.
- Move to controller/odom/cmd diagnostics.

### Phase21-22: controller dynamics and last-window taxonomy

Phase21 added odom/cmd_vel diagnostics. Phase22 added last-window classification.

Key findings:

- Timeout goals were not simple full-interval immobility.
- They usually had healthy motion earlier in the goal interval.
- The repeat-stable late signature was:
  - `healthy_motion_but_late_stall`
  - `late_controller_silent`
  - near-zero `cmd_vel_nav` and near-zero odom late in the timeout window
  - squeezed/high local-cost context

Decision:

- The problem moved from broad branch selection toward controller/local-cost late-window behavior.
- Continue diagnostics; do not tune from full-interval summaries alone.

### Phase23-24: failure-window local-cost and post-recovery alignment

Phase23 added runtime failure-window fields and Nav2 event timing. Phase24 added timeout subtype classification, post-recovery alignment, and enriched snapshots.

Key findings:

- Late-silent timeouts split into subtypes:
  - footprint/path blocked
  - side/timing
  - unclassified
- Most timeout windows had progress failure/recovery/controller abort timing.
- Post-recovery path updates continued in most rows.
- Robot-to-path distance was small in enriched post-recovery snapshots.
- The issue was not simply missing path refresh after recovery.

Decision:

- A narrow local path/footprint cost relief experiment was justified as a reversible Phase25 experiment.
- Branch-selection intervention still was not justified.

### Phase25: reversible local-cost / MPPI CostCritic experiments

Phase25 tested narrow, reversible intervention profiles.

Key findings:

- Local inflation relief (`phase25a`) was rejected: footprint/path blocked, timeout, and success metrics worsened despite one exit-reaching run.
- MPPI CostCritic lower weights showed mixed results:
  - `2.5` improved footprint/path blocked and timeouts but worsened exit distance.
  - `3.0` preserved exit distance but lost the timeout/subtype gains.
  - `2.75` looked promising in Phase25E/F after terminal-success-aware comparison.
- Candidate-baseline validation of `2.75` then failed twice in Phase25G/H.

Decision:

- Do not promote `2.75` directly.
- Do not reject it solely from one candidate-baseline failure either; characterize profile/runtime equivalence and run variance first.

### Phase26A-A1-B: params/profile/runtime equivalence and variance

Phase26A/A1 hardened parameter proof. Phase26B ran matched baseline/candidate repeats.

Key findings:

- Source YAML and runtime `/controller_server` parameter dumps matched expected profiles.
- Baseline runs loaded `CostCritic.cost_weight=3.81`.
- Candidate runs loaded `CostCritic.cost_weight=2.75`.
- Phase25E profile and candidate profile were semantically equivalent where relevant.
- Both baseline and candidate groups showed run-to-run variance:
  - baseline: 2/3 `EXIT_REACHED`, 1/3 `FAILED_EXHAUSTED`
  - candidate: 2/3 `EXIT_REACHED`, 1/3 `FAILED_EXHAUSTED`
- Candidate had a modest timeout median improvement but not stable final-mode / exit-distance superiority.

Decision:

- Params/profile/runtime load error is excluded as the primary explanation.
- `2.75` is not promotion-ready.
- Do not keep blind-testing nearby CostCritic values without deeper cause evidence.

### Phase26C-D-E-F-G-H-I-J-K-L: branch-choice diagnostics and spatial gating

These phases investigated whether route divergence or branch choice justified a direct branch-selection intervention.

Key findings:

- Some successful branch goals could still move away from the exit, so route divergence was real enough to monitor.
- However, candidate-branch diagnostics did not produce a stable, failed-only, clean-low-cost alternative branch pattern.
- Runtime spatial gates did not show a repeatable chosen-high / explored-clean signal sufficient for branch intervention.
- Phase26L real baseline/candidate smokes produced zero chosen-high/explored-clean signals.

Decision:

- Direct branch-selection intervention remains blocked.
- Continue controller/local-cost/MPPI evidence path instead.

### Phase26M-N-O: timing and controller/MPPI evidence availability

Phase26M/N refined timing around timeout goals. Phase26O audited controller/MPPI evidence availability.

Key findings:

- Timeout goals showed cmd-near-zero soon after recovery/controller abort clusters.
- Path updates continued while cmd was near-zero.
- Controller was active and receiving plans in timeout windows.
- Installed/runtime topics did not expose per-cycle selected control, critic cause, fallback, retry, or no-valid-control reason.

Decision:

- Continue MPPI diagnostics, but acknowledge installed artifacts expose only coarse evidence.

### Phase26P-Q-R-S-T-U-V: MPPI evidence hardening, summary repeats, and geometry join

These phases attempted to use available MPPI debug topics without changing behavior.

Key findings:

- Runtime discovered MPPI visualization topics such as `/optimal_trajectory`, `/trajectories`, and `/transformed_global_plan`.
- Installed Jazzy artifacts did not provide usable `critics_stats` evidence in this environment.
- Compact summaries could cover first cmd-near-zero windows.
- Candidate and baseline both showed broad `trajectory_evidence_present_without_critic_stats`-style conditions.
- Phase26V sampled `/optimal_trajectory` and `/transformed_global_plan` paths did not intersect captured high-cost choke points in the sampled near-zero windows.

Decision:

- A sampled optimal/transformed path crossing the captured high-cost choke is excluded for the Phase26V evidence windows.
- Visualization topics are insufficient to explain selected-control near-zero.
- Move to selected-control path audit.

### Phase26W-X: selected-control path audit and overlay feasibility

Phase26W located the selected-control production path in installed/source-level artifacts:

1. `MPPIController::computeVelocityCommands`
2. `Optimizer::evalControl`
3. optimizer internal steps including `updateControlSequence`
4. final conversion point: `Optimizer::getControlFromSequenceAsTwist`

Phase26W also showed that installed artifacts do not expose per-cycle values needed to explain why the selected command is near-zero.

Phase26X audited overlay feasibility:

- No `nav2_mppi_controller` source package exists in the current workspace.
- Installed binary/header/library artifacts exist under `/opt/ros/jazzy`.
- Authorized `apt-get source ros-jazzy-nav2-mppi-controller` failed because `deb-src` URIs are not enabled.
- Therefore a matching source overlay is not currently feasible without manual source preparation or source repository changes.
- Phase26X produced an analyzer contract for bounded JSONL but no real selected-control runtime rows.

Decision:

- Do not force-modify system packages.
- Do not infer the selected-control cause without source-level selected-control diagnostics.
- Current selected-control cause remains `insufficient_evidence`.

## 2. Excluded hypotheses

The following hypotheses are excluded or not supported by the Phase20-26X evidence chain.

### 2.1 Params/profile/runtime load error

Excluded by Phase26A/A1/B.

Evidence:

- Semantic YAML comparisons passed for the relevant profiles.
- Per-run source fingerprints existed.
- Runtime `/controller_server` dumps confirmed the expected `FollowPath.CostCritic.cost_weight` values.
- Baseline runs loaded `3.81`; candidate runs loaded `2.75`.

Decision impact:

- Phase25E vs candidate-baseline discrepancy should not be explained as wrapper selecting the wrong YAML or runtime failing to load candidate parameters.

### 2.2 Simple `CostCritic.cost_weight=2.75` promotion or rejection

Promotion is not supported; hard rejection is also not the central conclusion.

Evidence:

- Phase25E/F made `2.75` look promising under terminal-success-aware comparison.
- Phase25G/H candidate-baseline validation failed promotion.
- Phase26B matched repeats showed both baseline and candidate had 2/3 exit-reaching runs and 1/3 failed-exhausted runs.
- Candidate had some timeout-median improvement but did not show stable final-mode or exit-distance superiority.

Decision impact:

- Do not promote `2.75` to canonical baseline.
- Do not keep tuning around `2.75` until selected-control cause is better explained.
- Do not reject/erase the candidate artifact as if it had no signal; retain it as a studied candidate with unresolved variance.

### 2.3 Direct branch-selection intervention

Not supported by Phase20 and Phase26C-L.

Evidence:

- Phase20 failure mode was not stable enough for branch scoring.
- Branch-choice diagnostics did not reveal a stable failed-only, clean-low-cost alternative branch pattern.
- Runtime spatial gates did not produce repeatable chosen-high/explored-clean evidence.
- Phase26L real smokes found zero chosen-high/explored-clean signals.

Decision impact:

- Do not modify branch selection from current evidence.
- Continue to treat branch-choice changes as blocked unless future evidence isolates a stable route-divergence mechanism.

### 2.4 Sampled optimal/transformed path crossing captured high-cost choke

Excluded for the Phase26V sampled windows.

Evidence:

- Bounded sampled `/optimal_trajectory` and `/transformed_global_plan` path points were spatially joined against local high-cost points.
- Phase26V found sampled paths did not intersect the captured high-cost choke in the analyzed near-zero windows.
- The sampled optimal path was near-stationary, but its sampled geometry did not pass through the captured choke.

Decision impact:

- The observed near-zero command cannot be explained simply as sampled optimal/transformed path crossing the captured high-cost choke.
- The explanation must be closer to selected-control internals, critic scoring, fallback/retry, or update/conversion behavior.

### 2.5 Launch-log visible no-valid-control / collision / critic-specific reason

Not supported by installed/log evidence.

Evidence:

- Phase26U/W found only coarse installed strings and declarations such as `Optimizer fail to compute path`, `fallback`, and `retry_attempt_limit`.
- Launch logs did not expose per-cycle no-valid-control, collision-specific, critic-specific, fallback, retry, or control-sequence values sufficient to explain near-zero.
- Available MPPI topics were visualization/summary topics, not final selected-control cause topics.

Decision impact:

- Do not infer a critic-specific or collision-specific selected-control reason from `launch.log` alone.
- Source-level instrumentation is required for selected-control cause classification.

## 3. Still unexplained

The central unresolved question is:

```text
Why does MPPI selected control become near-zero in the late timeout/recovery windows?
```

Currently missing evidence:

- optimized `control_sequence_.vx/vy/wz` first entries per cycle
- returned twist immediately from `Optimizer::getControlFromSequenceAsTwist`
- whether fallback was called/used
- retry count and failure state in `Optimizer::evalControl`
- before/after values from `Optimizer::updateControlSequence`
- whether first control is already near-zero before conversion
- whether nonzero control is converted/clamped to near-zero
- whether updateControlSequence collapses control toward near-zero
- whether fallback returns near-zero

Phase26W located the selected-control production point. Phase26X showed that the current environment cannot build the needed overlay without matching source. Therefore the selected-control cause remains unresolved.

## 4. Current evidence conclusion

Current decision state:

```text
Phase27 remains blocked.
Do not tune Nav2/controller parameters.
Do not modify branch selection.
Do not promote or reject CostCritic=2.75 from this closure.
Selected-control cause: insufficient_evidence.
```

This is not a claim that MPPI is wrong, CostCritic is wrong, or branch selection is wrong. It is a claim that the current evidence chain has reached the limit of installed/runtime black-box diagnostics for selected-control near-zero.

## 5. Engineering decision point

At this point, there are two reasonable follow-up routes. They answer different questions and carry different risks.

## 6. Route A: Deep controller route

Name:

```text
Phase26Y / Phase26Z diagnostics-only source overlay
```

Purpose:

- Continue root-cause diagnostics inside MPPI.
- Explain selected-control near-zero using source-level instrumentation.

Prerequisite:

- A human prepares a matching `nav2_mppi_controller` source overlay for the installed Jazzy package version, or enables an approved source-repository path and verifies source/package compatibility.
- The overlay must be built in the workspace, not by editing `/opt/ros` system package files.

Diagnostics-only instrumentation targets:

- `Optimizer::getControlFromSequenceAsTwist`
  - returned `vx/vy/wz`
  - near-zero flag
  - first 3-5 `control_sequence_.vx/vy/wz`
- `Optimizer::evalControl`
  - fallback/retry/failure/exception state if available
- `Optimizer::updateControlSequence`
  - first control before/after and delta summary if available
- `MPPIController::computeVelocityCommands`
  - plan size
  - robot speed
  - returned twist

Bounded output:

```text
log/<run_id>_phase26x_mppi_selected_control_debug.jsonl
```

Expected decision value:

- Determines whether near-zero is caused by:
  - control sequence itself already near-zero
  - conversion/clamp after nonzero control
  - fallback
  - updateControlSequence collapse
  - still-missing critic/exception cause

Risks:

- Requires careful source/version matching.
- Overlay may diverge from installed binary behavior if source does not match exactly.
- Instrumentation in a real-time controller can perturb timing if logging is too heavy.
- Build and plugin resolution mistakes can create false evidence if the overlay is not actually loaded.

Mitigations:

- Keep instrumentation bounded and near-zero-only.
- Record overlay source/version/fingerprint.
- Verify plugin library resolution and runtime log token proving overlay loaded.
- Compare source/runtime params and preserve existing behavior semantics.
- No parameter or control behavior changes.

Workload:

- Medium to high.
- Requires source acquisition, build integration, plugin-load proof, bounded smoke, and analysis.

Rollback:

- High, if implemented as a workspace overlay and not a system package modification.
- Remove overlay from workspace / clean build / source only `/opt/ros` and existing workspace packages.

Best when:

- The engineering goal is to understand the root cause before any intervention.
- The team is willing to spend time on MPPI internals and source/version hygiene.

## 7. Route B: Engineering fallback route

Name:

```text
Phase27-alt near-exit recovery / terminal acceptance fallback
```

Purpose:

- Stop deep MPPI black-box investigation for now.
- Design a higher-level engineering fallback that accepts or recovers near-exit terminal behavior without modifying branch selection or MPPI internals blindly.

Possible design direction:

- Detect near-exit / late-silent situations using existing diagnostics:
  - exit distance
  - goal timeout subtype
  - post-recovery timing
  - near-zero cmd/odom window
  - no blocked/blacklist pollution
- Add a conservative terminal acceptance or near-exit recovery policy only when the robot is already inside or near the configured exit success region, or when a bounded local terminal maneuver is safer than continuing to dispatch normal DFS goals.
- Keep it separate from branch selection and MPPI tuning.

Important caveat:

- This is not Phase27 as originally blocked branch-selection/controller tuning. It would be a Phase27-alt engineering fallback proposal and still requires a design/test phase before implementation.
- It trades root-cause certainty for practical task completion.

Risks:

- Could mask the underlying MPPI selected-control problem.
- Poorly gated fallback could create false success or hide navigation failures.
- If applied too broadly, it could introduce state-machine complexity and brittle terminal behavior.
- It may not help failures far from the exit.

Mitigations:

- Gate only near the exit or in explicitly diagnosed terminal windows.
- Require structured event logging for every fallback activation.
- Add contract tests before runtime implementation.
- Keep fallback reversible with launch/config enable flag defaulting to off until validated.
- Acceptance criterion remains reaching the configured exit coordinates/region, not arbitrary Nav2 success.

Workload:

- Medium.
- Less source/toolchain risk than Route A, but requires careful state-machine design and regression tests.

Rollback:

- High if implemented behind a disabled-by-default flag and isolated from branch selection.

Best when:

- The engineering goal is to improve mission completion sooner.
- The team accepts that the selected-control cause remains black-box and wants a pragmatic terminal-layer fallback.

## 8. A/B comparison

| Dimension | Route A: deep controller diagnostics | Route B: engineering fallback |
| --- | --- | --- |
| Primary question | Why is selected control near-zero? | How can the mission succeed despite near-zero? |
| Root-cause value | High | Low to medium |
| Practical completion value | Indirect | Potentially high near exit |
| Required source overlay | Yes | No |
| Gazebo/Nav2 runs in next phase | Bounded diagnostics smoke only after overlay proof | Bounded validation only after design/tests |
| Behavior change | None in diagnostics phases | Yes, if later implemented |
| Risk of masking root cause | Low | Medium to high |
| Risk of wrong runtime evidence | Medium if overlay mismatch | Low for instrumentation, medium for policy design |
| Workload | Medium-high | Medium |
| Rollback | High if overlay is workspace-only | High if flag-gated and isolated |
| Best next artifact | Phase26Y source-overlay proof + selected-control JSONL | Phase27-alt design doc + contract tests |

## 9. Recommended closure decision

Recommended state after Phase26-Final:

1. Close Phase26 diagnostic chain as complete up to black-box limits.
2. Keep Phase27 blocked for original branch-selection/controller-tuning interventions.
3. Choose one of two explicit next routes:
   - Route A if root-cause certainty is required before intervention.
   - Route B if practical mission completion near the exit is more valuable than further MPPI internals work.
4. Do not silently mix A and B. Each route should start as a named phase with its own guardrails and acceptance tests.

## 10. Validation for this closure

This report is documentation-only. No Gazebo/Nav2 run was started. No source was fetched. No code behavior was modified.

Validation should be limited to file existence and markdown/static checks.
