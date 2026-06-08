---
title: Phase26X MPPI selected-control diagnostics
date: 2026-05-27
workspace: ros2_ws_tugbot_nav_20260522
phase: 26X
---

# Phase26X MPPI selected-control diagnostics

## Scope and guardrails

Phase26X remains diagnostics-only.

Guardrails held:

- do not enter Phase27
- do not modify branch selection
- do not modify Nav2/controller parameter semantics
- do not promote or reject any candidate
- do not change MPPI control behavior
- do not infer tuning from incomplete selected-control evidence

## Overlay feasibility audit

Current workspace audit:

- No `nav2_mppi_controller` source package exists under `src/` in this workspace.
- Installed binary package is present: `ros-jazzy-nav2-mppi-controller 1.3.11-1noble.20260412.062211`.
- Installed headers and library are present:
  - `/opt/ros/jazzy/include/nav2_mppi_controller/optimizer.hpp`
  - `/opt/ros/jazzy/include/nav2_mppi_controller/controller.hpp`
  - `/opt/ros/jazzy/lib/libmppi_controller.so`

Authorized source fetch feasibility check:

```bash
apt-get source ros-jazzy-nav2-mppi-controller
```

Result:

```text
Reading package lists...
E: You must put some 'deb-src' URIs in your sources.list
```

Conclusion: a matching source overlay is not currently feasible from this environment without enabling source repositories or manually providing the matching Nav2 source. Therefore Phase26X must not force-modify system packages. The safe Phase26X output is a source-level instrumentation plan plus an offline analyzer contract for the expected bounded JSONL.

## Source-level instrumentation plan

If matching source is later provided, instrument only logging/diagnostics. Do not alter control values, constraints, critics, sampling, fallback behavior, retry behavior, or parameter semantics.

Bounded output target:

```text
log/<run_id>_phase26x_mppi_selected_control_debug.jsonl
```

Record only near-zero cycles or at most the first N cycles. Recommended hard defaults:

- near-zero threshold: linear abs <= 0.03, angular abs <= 0.05
- max rows: 200
- emit one compact JSON object per selected-control cycle

### Optimizer::getControlFromSequenceAsTwist

Record:

- returned `vx`, `vy`, `wz`
- `near_zero` flag
- `control_sequence_.vx/vy/wz` first 3 to 5 entries
- cycle stamp / wall time if available

This is the primary selected-control conversion point located in Phase26W.

### Optimizer::evalControl

Record, if visible in source:

- fallback called/used
- retry count / retry attempt index
- failure flag from fallback or optimizer failure path
- exception class/message if an exception path is taken
- whether returned twist came from normal optimize path or fallback path

Do not catch-and-swallow new exceptions differently from upstream behavior.

### Optimizer::updateControlSequence

Record, if visible in source:

- first control before update
- first control after update
- delta summary for first 3 to 5 controls if cheap to compute
- collapsed flag: before nonzero and after near-zero

Do not modify softmax, weights, costs, constraints, or history shifting.

### MPPIController::computeVelocityCommands

Record:

- transformed plan size
- robot speed (`linear.x`, `linear.y`, `angular.z`)
- returned twist
- whether returned twist is near-zero

Do not change visualization behavior or final command behavior.

## Analyzer added

Added offline analyzer:

```text
tools/analyze_phase26x_selected_control_debug.py
```

It consumes the proposed bounded JSONL and reports:

- `control_sequence_first_is_near_zero`
- `returned_twist_is_near_zero`
- `fallback_used`
- `control_nonzero_but_returned_zero`
- `optimizer_update_collapsed_control`

It also returns a guarded conclusion:

- `control_sequence_itself_near_zero`
- `conversion_or_post_sequence_clamp_after_nonzero_control`
- `optimizer_update_collapsed_control_sequence`
- `fallback_used`
- `insufficient_evidence`
- `insufficient_or_mixed_evidence`

The analyzer decision block always keeps Phase27 blocked and forbids candidate promotion/rejection from Phase26X alone.

## Current artifacts

Created:

- `tools/analyze_phase26x_selected_control_debug.py`
- `src/tugbot_maze/test/test_phase26x_selected_control_debug.py`
- `log/phase26x_overlay_feasibility_audit.json`
- `log/phase26x_overlay_infeasible_phase26x_mppi_selected_control_debug.jsonl`
- `log/phase26x_overlay_infeasible_phase26x_selected_control_analysis.json`
- `doc/doc_report/phase26x_mppi_selected_control_diagnostics.md`

The JSONL file is intentionally empty for this run because overlay instrumentation could not be built without source. The analyzer output therefore records `insufficient_evidence` rather than inventing a selected-control cause.

## Verification

RED:

- `python3 -m pytest src/tugbot_maze/test/test_phase26x_selected_control_debug.py -q`
- observed `3 failed` before analyzer/report existed.

GREEN and regression:

- focused Phase26X tests: `3 passed`
- Phase24/25/26 regression selection: `99 passed`
- `python3 -m py_compile tools/analyze_phase26x_selected_control_debug.py src/tugbot_maze/test/test_phase26x_selected_control_debug.py`: passed
- `bash -n tools/run_phase21_controller_diagnostics_smoke.sh`: passed
- `colcon build --symlink-install`: 6 packages finished
- `colcon test --event-handlers console_direct+`: 170 tests passed
- `colcon test-result --verbose`: 170 tests, 0 errors, 0 failures, 0 skipped

No bounded baseline diagnostics run was started because overlay source instrumentation was infeasible; running the old installed binary would not produce Phase26X selected-control fields.

## Current evidence conclusion

No real Phase26X runtime selected-control JSONL was produced because source overlay is not currently feasible. Therefore the near-zero reason remains:

```text
insufficient_evidence
```

Phase26X has not proven whether near-zero is caused by:

- control_sequence itself near-zero,
- conversion/clamp after nonzero control,
- fallback,
- updateControlSequence collapsing the first control.

No Nav2/controller tuning is proposed.
