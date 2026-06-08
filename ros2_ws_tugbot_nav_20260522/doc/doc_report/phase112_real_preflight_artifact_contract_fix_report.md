# Phase112 real preflight artifact contract fix report

Status: PHASE112_REAL_PREFLIGHT_ARTIFACT_CONTRACT_FIX_COMPLETE_STOP_BEFORE_PHASE113

## Goal

Phase112 fixes only the artifact/runtime-sampling contract defect exposed by Phase111: some `sample_history` entries lacked `failed_gates` and `failure_reasons_by_gate`. The fix must keep all ambiguous/query_error/timeout/raw-contradiction states fail-closed and must not change navigation strategy or Nav2 configuration.

## Scope

Allowed:

- Modify `tools/phase105_inner_ingress_tf_controller_preflight.py`.
- Add/modify focused tests.
- Use synthetic replay and one real no-goal preflight-only rerun to validate the artifact contract.

Forbidden and guarded:

- No NavigateToPose goal was sent.
- `ingress_goal_sent=false` is required for real rerun artifacts.
- `maze_explorer_started=false` is required for real rerun artifacts.
- No maze_explorer was started.
- No Nav2/MPPI/controller/config tuning was performed.
- No exploration strategy was changed.
- Preflight was not removed.
- This phase does not prove autonomous exploration success.
- This phase does not prove exit success.
- This phase does not prove it is safe to send an ingress goal.
- Phase113 not entered.

## Deliverables

- Updated `tools/phase105_inner_ingress_tf_controller_preflight.py`
- `src/tugbot_maze/test/test_phase112_real_preflight_artifact_contract_fix.py`
- `log/phase112_real_preflight_artifact_contract_fix/`
- `doc/doc_report/phase112_real_preflight_artifact_contract_fix_report.md`

## Implementation summary

Added `_complete_sample_history_contract(samples, config)` and made `PreflightResult.to_artifact()` serialize the completed copy rather than the possibly early-returned `self.samples` list. This is intentionally artifact-only normalization: it annotates every sample with required diagnostic fields but does not alter `PreflightResult.passed`, reject reason, failed gates, wrapper guard state, or fail-closed semantics.

Required per-sample fields now completed for all serialization paths:

- `phase`
- `failed_gates`
- `failure_reasons_by_gate`
- `lifecycle_sources`
- `scan_transform_check`
- `map_base_tf_check`
- `map_base_tf_age_sec`
- `map_odom_tf_age_sec`
- `odom_base_tf_age_sec`
- `stable_window_elapsed_sec`

## Verification

### TDD RED

Focused Phase112 tests were added before implementation.

Command:

```bash
PYTHONDONTWRITEBYTECODE=1 pytest -q src/tugbot_maze/test/test_phase112_real_preflight_artifact_contract_fix.py
```

Observed RED result:

```text
F..F                                                                     [100%]
2 failed, 2 passed in 0.05s
```

Expected failing behavior was reproduced:

- `test_phase112_final_timeout_path_annotates_samples_after_first_scan_timeout` failed because sample 2 was missing:
  - `failed_gates`
  - `failure_reasons_by_gate`
  - `lifecycle_sources`
  - `phase`
  - `stable_window_elapsed_sec`
- report guard test failed because the Phase112 report did not exist yet.

### GREEN focused contract check

After adding `_complete_sample_history_contract()` and wiring `PreflightResult.to_artifact()` to serialize the completed sample copy, the implementation-focused tests passed:

```text
PYTHONDONTWRITEBYTECODE=1 python3 -m py_compile tools/phase105_inner_ingress_tf_controller_preflight.py src/tugbot_maze/test/test_phase112_real_preflight_artifact_contract_fix.py
PYTHONDONTWRITEBYTECODE=1 pytest -q src/tugbot_maze/test/test_phase112_real_preflight_artifact_contract_fix.py -k 'not report_guardrails'
...                                                                      [100%]
3 passed, 1 deselected in 0.02s
```

### Regression/static/synthetic verification

Command:

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -m py_compile \
  tools/phase105_inner_ingress_tf_controller_preflight.py \
  tools/analyze_phase110_preflight_synthetic_replay_contract.py \
  tools/analyze_phase111_real_preflight_no_goal_dispatch.py \
  src/tugbot_maze/test/test_phase109_preflight_false_negative_reduction_minimal_implementation.py \
  src/tugbot_maze/test/test_phase110_preflight_synthetic_replay_contract.py \
  src/tugbot_maze/test/test_phase111_real_preflight_no_goal_dispatch.py \
  src/tugbot_maze/test/test_phase112_real_preflight_artifact_contract_fix.py

PYTHONDONTWRITEBYTECODE=1 pytest -q \
  src/tugbot_maze/test/test_phase109_preflight_false_negative_reduction_minimal_implementation.py \
  src/tugbot_maze/test/test_phase110_preflight_synthetic_replay_contract.py \
  src/tugbot_maze/test/test_phase111_real_preflight_no_goal_dispatch.py \
  src/tugbot_maze/test/test_phase112_real_preflight_artifact_contract_fix.py

PYTHONDONTWRITEBYTECODE=1 python3 tools/analyze_phase110_preflight_synthetic_replay_contract.py \
  --output-dir log/phase112_real_preflight_artifact_contract_fix/phase110_synthetic_replay_regression

PYTHONDONTWRITEBYTECODE=1 python3 tools/phase105_inner_ingress_tf_controller_preflight.py \
  --output log/phase112_real_preflight_artifact_contract_fix/phase112_synthetic_pass_preflight.json \
  --synthetic-pass \
  --startup-grace-sec 1.0
```

Observed output:

```text
.....................                                                    [100%]
21 passed in 0.18s
{"contract_valid": true, ...}
{"failed_gates": [], "passed": true, "reject_reason": null}
{"phase110_replay_contract_valid": true, "real_artifact_contract_valid": true, "real_classification": "PHASE111_PREFLIGHT_REJECTED_NO_GOAL_DISPATCH", "real_no_goal_dispatch_guard_valid": true, "real_reject_reason": "ingress_first_scan_timeout", "synthetic_pass": true, "synthetic_sample_contract_fields": ["failed_gates", "failure_reasons_by_gate", "lifecycle_sources", "map_base_tf_check", "phase", "scan_transform_check", "stable_window_elapsed_sec"]}
```

Phase109/110/111 focused tests did not regress.

### Real no-goal preflight-only rerun

Initial cleanup summary:

- `log/phase112_real_preflight_artifact_contract_fix/phase112_real_preflight_artifact_contract_fix_initial_cleanup_summary.json`

Observed:

```json
{"before_count": 0, "after_count": 0}
```

A visible Gazebo/RViz/SLAM/Nav2 stack was started only to obtain real topics, TF, lifecycle, and action evidence. Readiness artifact:

- `log/phase112_real_preflight_artifact_contract_fix/phase112_real_preflight_artifact_contract_fix_ros_graph_ready.txt`

Real preflight command shape:

```bash
python3 tools/phase105_inner_ingress_tf_controller_preflight.py \
  --timeout-sec 30 \
  --startup-grace-sec 3.0 \
  --tf-stability-window-sec 2.0 \
  --sample-period-sec 0.5 \
  --tf-max-age-sec 1.5 \
  --scan-max-age-sec 1.5 \
  --output log/phase112_real_preflight_artifact_contract_fix/phase112_real_preflight_artifact_contract_fix_ingress_preflight.json

python3 tools/phase105_inner_ingress_tf_controller_preflight.py \
  --mark-wrapper-state \
  --output log/phase112_real_preflight_artifact_contract_fix/phase112_real_preflight_artifact_contract_fix_ingress_preflight.json \
  --ingress-goal-sent false \
  --maze-explorer-started false
```

Observed preflight stdout:

```json
{"failed_gates": ["ingress_lifecycle_ambiguous", "ingress_map_base_tf_missing", "ingress_map_odom_tf_stale", "ingress_odom_base_tf_stale", "ingress_first_scan_timeout", "ingress_controller_robot_pose_unavailable", "ingress_goal_pose_transform_unavailable"], "passed": false, "reject_reason": "ingress_first_scan_timeout"}
```

Analyzer command reused the Phase111 no-goal analyzer against the Phase112 artifact directory:

```bash
python3 tools/analyze_phase111_real_preflight_no_goal_dispatch.py \
  --artifact-dir log/phase112_real_preflight_artifact_contract_fix \
  --run-id phase112_real_preflight_artifact_contract_fix \
  --output-json log/phase112_real_preflight_artifact_contract_fix/phase112_real_preflight_artifact_contract_fix_analysis.json \
  --minimal-summary-output log/phase112_real_preflight_artifact_contract_fix/phase112_real_preflight_artifact_contract_fix_minimal_summary.md
```

Observed analyzer output:

```json
{"classification": "PHASE111_PREFLIGHT_REJECTED_NO_GOAL_DISPATCH", "no_goal_dispatch_guard_valid": true, "passed": false, "reject_reason": "ingress_first_scan_timeout"}
```

The classification label is from the reused Phase111 analyzer; for Phase112 interpretation, the important fields are:

```json
{
  "artifact_contract_valid": true,
  "no_goal_dispatch_guard_valid": true,
  "ingress_goal_sent": false,
  "maze_explorer_started": false,
  "passed": false,
  "reject_reason": "ingress_first_scan_timeout"
}
```

Real rerun artifacts:

- `log/phase112_real_preflight_artifact_contract_fix/phase112_real_preflight_artifact_contract_fix_ingress_preflight.json`
- `log/phase112_real_preflight_artifact_contract_fix/phase112_real_preflight_artifact_contract_fix_analysis.json`
- `log/phase112_real_preflight_artifact_contract_fix/phase112_real_preflight_artifact_contract_fix_minimal_summary.md`
- `log/phase112_real_preflight_artifact_contract_fix/phase112_real_preflight_artifact_contract_fix_post_preflight_ros_snapshot.txt`

### Phase112 findings

- The Phase111 artifact contract defect is fixed for the real no-goal rerun:
  - `sample_history_contract.all_samples_have_failed_gates=true`
  - `sample_history_contract.all_samples_have_failure_reasons_by_gate=true`
  - `sample_history_contract.missing_fields=[]`
  - `artifact_contract_valid=true`
- No-goal guard remained valid:
  - `ingress_goal_sent=false`
  - `maze_explorer_started=false`
  - `no_goal_dispatch_guard_valid=true`
- Safety policy remained fail-closed:
  - lifecycle ambiguous remained failing evidence;
  - raw snapshot contradiction remained ambiguous/failing;
  - first scan timeout remained rejecting;
  - no pass was inferred from ambiguous external evidence.
- The real preflight still rejected with `ingress_first_scan_timeout`; Phase112 did not tune, weaken, or bypass that behavior.

### Final runtime cleanup

Final cleanup summary:

- `log/phase112_real_preflight_artifact_contract_fix/phase112_real_preflight_artifact_contract_fix_final_cleanup_summary.json`

Observed:

```json
{"before_count": 21, "after_count": 0}
```

The cleaned processes were the visible launch, Gazebo, RViz, SLAM Toolbox, Nav2 servers, bridges, and static TF publishers. No `maze_explorer` process was started.

### Final guard summary

Final guard artifacts:

- `log/phase112_real_preflight_artifact_contract_fix/phase112_real_preflight_artifact_contract_fix_final_guard_summary.json`
- `log/phase112_real_preflight_artifact_contract_fix/phase112_real_preflight_artifact_contract_fix_final_nav2_config_diff.txt`
- `log/phase112_real_preflight_artifact_contract_fix/phase112_real_preflight_artifact_contract_fix_final_process_guard.txt`
- `log/phase112_real_preflight_artifact_contract_fix/phase112_real_preflight_artifact_contract_fix_final_cache_guard.txt`
- `log/phase112_real_preflight_artifact_contract_fix/phase112_real_preflight_artifact_contract_fix_final_git_status_guard.txt`

Observed final guard summary:

```json
{
  "nav2_config_tracked_diff_empty": true,
  "runtime_process_guard_empty": true,
  "scoped_pycache_guard_empty": true,
  "nav2_config_git_status_short": ["?? src/tugbot_navigation/config/"],
  "note": "git status may list src/tugbot_navigation/config as untracked repository state; tracked diff is empty and Phase112 did not edit Nav2 config files."
}
```

Interpretation:

- tracked Nav2 config diff is empty;
- no relevant Gazebo/RViz/SLAM/Nav2/maze_explorer/recorder/preflight runtime process remained;
- scoped Python cache cleanup guard is empty;
- repository still reports `src/tugbot_navigation/config/` as untracked state, but Phase112 did not edit Nav2 config files and the tracked diff guard is empty.

## Conclusion

Phase112 completed the artifact/runtime-sampling contract fix. Every serialized sample path now carries the required per-sample diagnostic fields, and the real no-goal preflight-only rerun produced `artifact_contract_valid=true` while preserving fail-closed rejection and no-goal guardrails.

No autonomous exploration success or exit success is claimed. Phase112 stops here for human acceptance. Phase113 not entered.
