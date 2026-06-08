# Phase113 preflight internal sampling discrepancy report

Status: PHASE113_PREFLIGHT_INTERNAL_SAMPLING_DISCREPANCY_DIAGNOSIS_COMPLETE_STOP_BEFORE_PHASE114

## Goal

Phase113 diagnoses why external ROS graph/snapshot can see `/scan` and `map->base_link`, while the preflight artifact still rejects fail-closed with `ingress_first_scan_timeout` and TF-missing gates.

This phase is diagnosis-only. It may add analyzer tooling and lightweight internal sampling instrumentation, but it must not alter safety policy or navigation behavior.

## Scope and guardrails

Allowed:

- Add `tools/analyze_phase113_preflight_internal_sampling_discrepancy.py`.
- Add focused tests.
- Add lightweight diagnostic fields under each runtime sample's `internal_sampling_diagnostics`.
- Run one real no-goal preflight-only rerun.

Forbidden and guarded:

- No NavigateToPose goal was sent.
- `ingress_goal_sent=false` is required.
- `maze_explorer_started=false` is required.
- No maze_explorer was started.
- No Nav2/MPPI/controller/config tuning was performed.
- No exploration strategy was changed.
- Preflight was not removed.
- This diagnosis-only phase does not prove autonomous exploration success.
- This diagnosis-only phase does not prove exit success.
- This diagnosis-only phase does not prove it is safe to send an ingress goal.
- Phase114 not entered.

## Deliverables

- `tools/analyze_phase113_preflight_internal_sampling_discrepancy.py`
- `src/tugbot_maze/test/test_phase113_preflight_internal_sampling_discrepancy.py`
- `log/phase113_preflight_internal_sampling_discrepancy/`
- `doc/doc_report/phase113_preflight_internal_sampling_discrepancy_report.md`

## Instrumentation summary

Runtime samples now include diagnosis-only `internal_sampling_diagnostics` with:

- executor spin count and spin timeout;
- node clock/use_sim_time, ROS time, wall time;
- `/scan` subscription topic, publisher/subscriber counts, callback count, latest frame, stamp/age, latest callback wall time;
- TF buffer can_transform/lookup_transform diagnostics for `map->base_link`, `map->odom`, `odom->base_link`, and `map-><scan_frame>`.

This instrumentation is serialized into artifacts only. It does not change `failure_reasons_for_sample()`, `tokens_from_failure_reasons()`, pass/reject evaluation, Nav2 config, or exploration strategy.

## Verification

### TDD RED

Focused Phase113 tests were added before implementation.

Command:

```bash
PYTHONDONTWRITEBYTECODE=1 pytest -q src/tugbot_maze/test/test_phase113_preflight_internal_sampling_discrepancy.py
```

Observed RED result:

```text
FF.FF                                                                    [100%]
4 failed, 1 passed in 0.06s
```

Expected failures were:

- missing `tools/analyze_phase113_preflight_internal_sampling_discrepancy.py`;
- CLI could not write analysis/summary because analyzer was missing;
- report guard failed because this report did not exist yet.

One existing preflight test passed, confirming the new test fixture could still load the Phase112 preflight module.

### GREEN focused diagnosis tests

After implementing the analyzer and diagnosis-only internal sampling fields:

```text
PYTHONDONTWRITEBYTECODE=1 python3 -m py_compile tools/phase105_inner_ingress_tf_controller_preflight.py tools/analyze_phase113_preflight_internal_sampling_discrepancy.py src/tugbot_maze/test/test_phase113_preflight_internal_sampling_discrepancy.py
PYTHONDONTWRITEBYTECODE=1 pytest -q src/tugbot_maze/test/test_phase113_preflight_internal_sampling_discrepancy.py -k 'not report_guardrails'
....                                                                     [100%]
4 passed, 1 deselected in 0.05s
```

After adding the sim-time mismatch classifier test:

```text
.....                                                                    [100%]
5 passed, 1 deselected in 0.05s
```

### Static/focused regression before real rerun

Command:

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -m py_compile \
  tools/phase105_inner_ingress_tf_controller_preflight.py \
  tools/analyze_phase110_preflight_synthetic_replay_contract.py \
  tools/analyze_phase111_real_preflight_no_goal_dispatch.py \
  tools/analyze_phase113_preflight_internal_sampling_discrepancy.py \
  src/tugbot_maze/test/test_phase109_preflight_false_negative_reduction_minimal_implementation.py \
  src/tugbot_maze/test/test_phase110_preflight_synthetic_replay_contract.py \
  src/tugbot_maze/test/test_phase111_real_preflight_no_goal_dispatch.py \
  src/tugbot_maze/test/test_phase112_real_preflight_artifact_contract_fix.py \
  src/tugbot_maze/test/test_phase113_preflight_internal_sampling_discrepancy.py

PYTHONDONTWRITEBYTECODE=1 pytest -q \
  src/tugbot_maze/test/test_phase109_preflight_false_negative_reduction_minimal_implementation.py \
  src/tugbot_maze/test/test_phase110_preflight_synthetic_replay_contract.py \
  src/tugbot_maze/test/test_phase111_real_preflight_no_goal_dispatch.py \
  src/tugbot_maze/test/test_phase112_real_preflight_artifact_contract_fix.py \
  src/tugbot_maze/test/test_phase113_preflight_internal_sampling_discrepancy.py -k 'not report_guardrails'
```

Observed:

```text
.....................                                                    [100%]
21 passed, 5 deselected in 0.23s
```

### Real no-goal instrumented preflight-only rerun

Initial cleanup:

- `log/phase113_preflight_internal_sampling_discrepancy/phase113_preflight_internal_sampling_discrepancy_initial_cleanup_summary.json`

Observed:

```json
{"before_count": 0, "after_count": 0}
```

Started visible Gazebo/RViz/SLAM/Nav2 stack only for real topics, TF, lifecycle, and action evidence. Readiness artifact:

- `log/phase113_preflight_internal_sampling_discrepancy/phase113_preflight_internal_sampling_discrepancy_ros_graph_ready.txt`

Readiness confirmed before preflight:

- `/navigate_to_pose` ready;
- `/map` ready;
- `/scan` ready;
- external `tf2_echo map base_link` saw `Translation:`;
- lifecycle/action graph visible.

Real preflight command shape:

```bash
python3 tools/phase105_inner_ingress_tf_controller_preflight.py \
  --timeout-sec 30 \
  --startup-grace-sec 3.0 \
  --tf-stability-window-sec 2.0 \
  --sample-period-sec 0.5 \
  --tf-max-age-sec 1.5 \
  --scan-max-age-sec 1.5 \
  --output log/phase113_preflight_internal_sampling_discrepancy/phase113_preflight_internal_sampling_discrepancy_ingress_preflight.json

python3 tools/phase105_inner_ingress_tf_controller_preflight.py \
  --mark-wrapper-state \
  --output log/phase113_preflight_internal_sampling_discrepancy/phase113_preflight_internal_sampling_discrepancy_ingress_preflight.json \
  --ingress-goal-sent false \
  --maze-explorer-started false
```

Observed preflight stdout:

```json
{"failed_gates": ["ingress_lifecycle_ambiguous", "ingress_map_base_tf_missing", "ingress_map_odom_tf_stale", "ingress_odom_base_tf_stale", "ingress_first_scan_timeout", "ingress_controller_robot_pose_unavailable", "ingress_goal_pose_transform_unavailable"], "passed": false, "reject_reason": "ingress_first_scan_timeout"}
```

External post-preflight snapshot artifact:

- `log/phase113_preflight_internal_sampling_discrepancy/phase113_preflight_internal_sampling_discrepancy_post_preflight_ros_snapshot.txt`

Analyzer artifacts:

- `log/phase113_preflight_internal_sampling_discrepancy/phase113_preflight_internal_sampling_discrepancy_analysis.json`
- `log/phase113_preflight_internal_sampling_discrepancy/phase113_preflight_internal_sampling_discrepancy_minimal_summary.md`

Analyzer observed:

```json
{
  "classification": "INTERNAL_SCAN_RECEIVED_BUT_REJECTED_BY_AGE_OR_TF",
  "contributing_findings": [
    "INTERNAL_TF_BUFFER_MISSING_WHILE_EXTERNAL_TF_AVAILABLE",
    "INTERNAL_NODE_USE_SIM_TIME_FALSE_WITH_SIM_STAMPED_SCAN",
    "SCAN_CALLBACK_RECEIVED_BUT_SCAN_GATE_UNAVAILABLE"
  ],
  "no_goal_dispatch_guard_valid": true,
  "reject_reason": "ingress_first_scan_timeout"
}
```

### Evidence summary

External evidence after preflight:

- `scan_available=true`
- `scan_frame_ids=["tugbot/scan_omni/scan_omni"]`
- `map_base_tf_available=true`
- launch active marker seen

Internal preflight evidence:

- executor spun: `spin_once_count_max=4`
- preflight node clock: `use_sim_time_values=["False"]`
- ROS time first/last tracked wall time, not sim time:
  - first: `1780498327.2427213`
  - last: `1780498355.3097396`
- `/scan` subscription saw a publisher: `publisher_count_max=1`
- `/scan` callback fired once: `callback_count_max=1`
- latest scan frame: `tugbot/scan_omni/scan_omni`
- latest scan age inside preflight: `1780498285.3194246` seconds
- scan gate still unavailable: `any_scan_available_gate=false`
- scan transform gate unavailable: `any_scan_transform_available_gate=false`
- internal TF buffer missing `map`:
  - `map->base_link any_can_transform=false`
  - `map->base_link any_lookup_success=false`
  - latest exception: `"map" passed to lookupTransform argument target_frame does not exist.`
  - same map-missing exception for `map->odom` and `map->tugbot/scan_omni/scan_omni`
- internal `odom->base_link` did become available in some samples.

### Diagnosis

Primary diagnosis:

- The preflight did not merely fail to subscribe to `/scan`.
- It did receive one `/scan` callback and saw topic publisher count 1.
- The scan was rejected because the preflight node was using wall time (`use_sim_time=false`) while the stack topics/TF are sim-time based.
- This made the scan age wall-time-sized (`~1.78e9 sec`), so `scan_available` stayed false and eventually became `ingress_first_scan_timeout`.
- The same clock/time mismatch is consistent with the internal TF buffer being unable to resolve `map`, while external CLI commands could resolve `map->base_link` outside the preflight process.

Contributing findings:

- `INTERNAL_NODE_USE_SIM_TIME_FALSE_WITH_SIM_STAMPED_SCAN`
- `SCAN_CALLBACK_RECEIVED_BUT_SCAN_GATE_UNAVAILABLE`
- `INTERNAL_TF_BUFFER_MISSING_WHILE_EXTERNAL_TF_AVAILABLE`

This is still diagnosis-only. Phase113 did not change the preflight clock policy, did not force `use_sim_time`, and did not weaken any fail-closed gates.

### Final runtime cleanup

Final cleanup summary:

- `log/phase113_preflight_internal_sampling_discrepancy/phase113_preflight_internal_sampling_discrepancy_final_cleanup_summary.json`

Observed:

```json
{"before_count": 21, "after_count": 0}
```

The cleaned processes were the visible launch, Gazebo, RViz, SLAM Toolbox, Nav2 servers, bridges, and static TF publishers. No `maze_explorer` process was started.

## Conclusion

Phase113 completed the diagnosis-only internal sampling discrepancy investigation. The strongest supported root-cause hypothesis is that the preflight node runs with `use_sim_time=false` while the stack publishes sim-time stamped scan/TF, causing huge scan age and internal TF-buffer lookup failures despite external `/scan` and `map->base_link` visibility.

No NavigateToPose goal was sent. `ingress_goal_sent=false`. `maze_explorer_started=false`. No autonomous exploration success or exit success is claimed. Phase113 stops here for human acceptance. Phase114 not entered.
