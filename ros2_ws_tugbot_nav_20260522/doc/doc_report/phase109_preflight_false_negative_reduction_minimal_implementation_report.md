# Phase109 ingress preflight false-negative reduction minimal implementation report

Status: PHASE109_IMPLEMENTED_STATIC_UNIT_VALIDATED_STOP_BEFORE_PHASE110

## Goal

Phase109 implements the minimal accepted part of the Phase108 design in the existing Phase105 ingress preflight tool, without runtime smoke.

The implementation preserves fail-closed semantics: ambiguous, missing, stale, unstable, or contradictory evidence still rejects and does not authorize the wrapper to send the unchanged inner-ingress goal.

## Scope performed

Modified:

- `tools/phase105_inner_ingress_tf_controller_preflight.py`
- `src/tugbot_maze/test/test_phase109_preflight_false_negative_reduction_minimal_implementation.py`
- `doc/doc_report/phase109_preflight_false_negative_reduction_minimal_implementation_report.md`

Not modified intentionally:

- maze_explorer strategy
- Phase88/92/101 carry-over or staging logic
- branch scoring, exploration order, centerline gate, directional readiness, fallback, terminal acceptance
- Nav2/MPPI/controller/inflation/robot_radius/clearance_radius_m/map threshold/config

## Implementation summary

1. Lifecycle multi-source confirmation
   - Added `derive_lifecycle_confirmation()`.
   - Records `lifecycle_query`, `node_graph`, and `action_availability` evidence.
   - Derives one of: `active_confirmed`, `inactive_confirmed`, `ambiguous`, `query_error`.
   - Ambiguous/query-error are fail-closed and no longer flattened into a definitive active state.

2. Startup grace + stable window
   - `PreflightConfig` now includes `startup_grace_sec` in addition to the existing `tf_stability_window_sec` and bounded `timeout_sec`.
   - Evaluation distinguishes startup grace samples from stable-window samples.
   - Passing still requires a continuous stable window with all hard gates clear.
   - A single early TF miss can recover; if later samples pass but do not cover a full stable window, the reject reason is `ingress_tf_stable_window_not_met` rather than the early miss.

3. First scan wait
   - Missing `/scan` during startup grace is recorded as sample-only `waiting_for_first_scan`.
   - If the bounded first-scan wait expires, the fail-closed token is `ingress_first_scan_timeout`.

4. Per-sample history
   - Artifacts now include full `sample_history`.
   - Each sample records `failed_gates`, `failure_reasons_by_gate`, lifecycle evidence, phase, and stable-window elapsed duration.

5. Raw-style snapshot cross-check
   - Before reject, the preflight artifact stores `raw_style_snapshot_cross_check`.
   - If raw-style evidence contradicts gated missing/inactive evidence, it sets ambiguity/contradiction fields and adds diagnostic token `ingress_raw_snapshot_cross_check_failed`.
   - This is diagnostic only; it does not cause pass.

6. CLI/static contract
   - Added `--startup-grace-sec` to the preflight CLI.
   - Synthetic/unit path remains available for static validation.

## Focused tests

Added:

```text
src/tugbot_maze/test/test_phase109_preflight_false_negative_reduction_minimal_implementation.py
```

Coverage:

- lifecycle multi-source states: active_confirmed, inactive_confirmed, ambiguous, query_error
- first-scan grace and `ingress_first_scan_timeout`
- transient early TF miss recovery
- stable-window-not-met classification
- sample_history and raw-style snapshot contradiction handling
- Phase109 report guardrails

## Safety and non-runtime guardrails

No Phase106 rerun was performed.
No ingress goal was sent.
No Gazebo/RViz/SLAM/Nav2/maze_explorer was started.
No Nav2/MPPI/controller/config tuning was performed.
Preflight was not removed.
Fail-closed behavior was preserved.
This implementation does not prove it is safe to send an ingress goal.
This implementation does not prove autonomous exploration success.
This implementation does not prove exit success.
Phase110 not entered.

## Verification

### RED step

Initial Phase109 focused test run before implementation:

```text
PYTHONDONTWRITEBYTECODE=1 pytest -q src/tugbot_maze/test/test_phase109_preflight_false_negative_reduction_minimal_implementation.py

FFFFFF                                                                   [100%]
6 failed in 0.06s
```

Expected missing behavior included `derive_lifecycle_confirmation`, `startup_grace_sec`, first-scan/stable-window/sample-history/raw-snapshot fields, and the report file.

### Static/unit verification

Command:

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -m py_compile \
  tools/phase105_inner_ingress_tf_controller_preflight.py \
  src/tugbot_maze/test/test_phase105_inner_ingress_tf_controller_preflight_hardening.py \
  src/tugbot_maze/test/test_phase109_preflight_false_negative_reduction_minimal_implementation.py
PYTHONDONTWRITEBYTECODE=1 pytest -q \
  src/tugbot_maze/test/test_phase105_inner_ingress_tf_controller_preflight_hardening.py \
  src/tugbot_maze/test/test_phase109_preflight_false_negative_reduction_minimal_implementation.py
python3 tools/phase105_inner_ingress_tf_controller_preflight.py \
  --output /tmp/phase109_synthetic_pass.json \
  --synthetic-pass \
  --startup-grace-sec 1.0
```

Observed result:

```text
............                                                             [100%]
12 passed in 0.04s
synthetic_pass_artifact_ok
EXIT 0
```

The synthetic artifact check confirmed:

```text
passed=true
sample_history present
startup_grace_sec=1.0
ingress_goal_sent=false
```

### Guard/cleanup verification

Command:

```bash
find tools src/tugbot_maze/test -name __pycache__ -type d -prune -exec rm -rf {} +
git diff -- src/tugbot_navigation/config
ps -eo pid,cmd | grep -E 'phase109|phase108|phase107_preflight|phase106_preflighted|record_explorer_state_series|maze_explorer|ros2 launch tugbot_bringup tugbot_maze_slam_nav|gz sim|rviz2|slam_toolbox|controller_server|planner_server|bt_navigator|ros_gz_bridge|parameter_bridge' | grep -v grep || true
find tools src/tugbot_maze/test -name __pycache__ -type d
git status --short -- tools/phase105_inner_ingress_tf_controller_preflight.py src/tugbot_maze/test/test_phase105_inner_ingress_tf_controller_preflight_hardening.py src/tugbot_maze/test/test_phase109_preflight_false_negative_reduction_minimal_implementation.py doc/doc_report/phase109_preflight_false_negative_reduction_minimal_implementation_report.md src/tugbot_navigation/config
```

Observed guard result:

```text
?? doc/doc_report/phase109_preflight_false_negative_reduction_minimal_implementation_report.md
?? src/tugbot_maze/test/test_phase105_inner_ingress_tf_controller_preflight_hardening.py
?? src/tugbot_maze/test/test_phase109_preflight_false_negative_reduction_minimal_implementation.py
?? src/tugbot_navigation/config/
?? tools/phase105_inner_ingress_tf_controller_preflight.py
EXIT 0
```

Interpretation:

- Nav2 config diff guard output was empty.
- Runtime process guard output was empty.
- Scoped `__pycache__` guard output was empty after cleanup.
- The shown `??` entries are this workspace's untracked-file status for phase artifacts/source paths; Phase109 did not tune Nav2 config and only edited the scoped preflight/tests/report files.

## Conclusion

Phase109 implemented the minimal false-negative reduction changes in the Phase105 ingress preflight tool and validated them with static/unit tests only.

The result preserves fail-closed behavior: lifecycle ambiguity, raw-snapshot contradiction, missing scan timeout, and stable-window failure all reject and do not send an ingress goal.

Phase109 stops here for human acceptance. Phase110 not entered.
