# Phase110 preflight synthetic replay contract report

Status: PHASE110_SYNTHETIC_REPLAY_CONTRACT_VALIDATED_STOP_BEFORE_PHASE111

## Goal

Phase110 validates the Phase109 ingress preflight artifact contract by synthetic replay only. The objective is to prove the new fields are stable, diagnostic, and analyzer-readable before any future real preflight rerun.

## Scope

Phase110 added offline replay/analyzer tooling and focused tests only:

- `tools/analyze_phase110_preflight_synthetic_replay_contract.py`
- `src/tugbot_maze/test/test_phase110_preflight_synthetic_replay_contract.py`
- `log/phase110_preflight_synthetic_replay_contract/`
- `doc/doc_report/phase110_preflight_synthetic_replay_contract_report.md`

## Guardrails

No Gazebo/RViz/SLAM/Nav2/maze_explorer was started.
No Phase106 rerun was performed.
No ingress goal was sent.
No Nav2/MPPI/controller/config tuning was performed.
No exploration strategy was changed.
Preflight was not removed.
This synthetic replay does not prove it is safe to send an ingress goal.
This synthetic replay does not prove autonomous exploration success.
This synthetic replay does not prove exit success.
Phase111 not entered.

## Replay coverage

The synthetic analyzer constructs and consumes artifacts covering:

1. `active_confirmed`, `inactive_confirmed`, `ambiguous`, and `query_error` lifecycle replay states.
2. `waiting_for_first_scan` as a sample-only startup-grace observation.
3. `ingress_first_scan_timeout` as a bounded first-scan wait failure.
4. Single early TF miss followed by recovery and stable-window pass.
5. Stable-window shortfall classified as `ingress_tf_stable_window_not_met`, not final map-base missing.
6. `raw_style_snapshot_cross_check` contradiction classified as ambiguous and fail-closed.
7. Full `sample_history` contract: every sample has `failed_gates` and `failure_reasons_by_gate`.
8. Human-readable minimal summary output.

## Verification

### RED step

Initial Phase110 focused test run before analyzer/report implementation:

```text
PYTHONDONTWRITEBYTECODE=1 pytest -q src/tugbot_maze/test/test_phase110_preflight_synthetic_replay_contract.py

FFFFFF                                                                   [100%]
6 failed in 0.06s
```

Expected missing behavior: analyzer script, replay generation, analysis/minimal summary output, and report file.

### Static/unit and synthetic replay verification

Command:

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -m py_compile \
  tools/analyze_phase110_preflight_synthetic_replay_contract.py \
  src/tugbot_maze/test/test_phase110_preflight_synthetic_replay_contract.py
PYTHONDONTWRITEBYTECODE=1 pytest -q src/tugbot_maze/test/test_phase110_preflight_synthetic_replay_contract.py
PYTHONDONTWRITEBYTECODE=1 python3 tools/analyze_phase110_preflight_synthetic_replay_contract.py --output-dir log/phase110_preflight_synthetic_replay_contract
```

Observed result:

```text
......                                                                   [100%]
6 passed in 0.12s
{"analysis": "log/phase110_preflight_synthetic_replay_contract/phase110_preflight_synthetic_replay_contract_analysis.json", "artifacts_dir": "log/phase110_preflight_synthetic_replay_contract/synthetic_artifacts", "contract_valid": true, "summary": "log/phase110_preflight_synthetic_replay_contract/phase110_preflight_synthetic_replay_contract_minimal_summary.md"}
```

Generated artifacts:

- `log/phase110_preflight_synthetic_replay_contract/phase110_preflight_synthetic_replay_contract_analysis.json`
- `log/phase110_preflight_synthetic_replay_contract/phase110_preflight_synthetic_replay_contract_minimal_summary.md`
- `log/phase110_preflight_synthetic_replay_contract/synthetic_artifacts/`

Minimal summary result:

```text
contract_valid: True
lifecycle_active_confirmed: active_confirmed
lifecycle_inactive_confirmed: inactive_confirmed
lifecycle_ambiguous: ambiguous
lifecycle_query_error: query_error
WAITING_FOR_FIRST_SCAN_OBSERVED_NO_FINAL_SCAN_TIMEOUT
INGRESS_FIRST_SCAN_TIMEOUT_REPLAYED
EARLY_TF_MISS_RECOVERED_STABLE_WINDOW_PASS
INGRESS_TF_STABLE_WINDOW_NOT_MET_REPLAYED
RAW_SNAPSHOT_CONTRADICTION_AMBIGUOUS_FAIL_CLOSED
```

### Guard/cleanup verification

Command:

```bash
find tools src/tugbot_maze/test -name __pycache__ -type d -prune -exec rm -rf {} +
git diff -- src/tugbot_navigation/config
ps -eo pid,cmd | grep -E 'phase110|phase109|phase108|phase107_preflight|phase106_preflighted|record_explorer_state_series|maze_explorer|ros2 launch tugbot_bringup tugbot_maze_slam_nav|gz sim|rviz2|slam_toolbox|controller_server|planner_server|bt_navigator|ros_gz_bridge|parameter_bridge' | grep -v grep || true
find tools src/tugbot_maze/test -name __pycache__ -type d
git status --short -- tools/analyze_phase110_preflight_synthetic_replay_contract.py src/tugbot_maze/test/test_phase110_preflight_synthetic_replay_contract.py log/phase110_preflight_synthetic_replay_contract doc/doc_report/phase110_preflight_synthetic_replay_contract_report.md src/tugbot_navigation/config
```

Observed result:

```text
?? doc/doc_report/phase110_preflight_synthetic_replay_contract_report.md
?? log/phase110_preflight_synthetic_replay_contract/
?? src/tugbot_maze/test/test_phase110_preflight_synthetic_replay_contract.py
?? src/tugbot_navigation/config/
?? tools/analyze_phase110_preflight_synthetic_replay_contract.py
```

Interpretation:

- Nav2 config diff guard output was empty.
- Runtime process guard output was empty.
- Scoped `__pycache__` guard output was empty after cleanup.
- `??` entries are untracked workspace phase artifacts/source paths; Phase110 intentionally added analyzer/tests/report/log artifacts and did not tune Nav2 config.

## Conclusion

Phase110 synthetic replay contract validation is complete. The analyzer can generate and consume Phase109-style synthetic preflight artifacts, verify the new artifact contract, and produce a human-readable minimal summary.

The validated contract is diagnostic-only and fail-closed. It prepares for a future real preflight rerun but does not prove that a goal is safe to send.

Phase110 stops here for human acceptance. Phase111 not entered.
