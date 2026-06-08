# Phase114 preflight sim-time alignment fix report

Status: PHASE114_PREFLIGHT_SIM_TIME_ALIGNMENT_FIX_COMPLETE_STOP_BEFORE_PHASE115

## Goal

Phase114 minimally fixes the Phase113 root-cause hypothesis: the preflight node used wall time while Gazebo/Nav2 topics and TF were sim-time based. The phase aligns the preflight node with the simulation stack and validates only internal `/scan` age and TF buffer recovery.

This is a no-goal validation phase. It does not send a NavigateToPose goal, does not start `maze_explorer`, and does not claim autonomous exploration or exit success.

## Scope and guardrails

Allowed:

- Modify `tools/phase105_inner_ingress_tf_controller_preflight.py`.
- Add CLI switches for sim-time selection.
- Add Phase114 focused tests, analyzer, artifacts, and report.
- Run a real visible-stack no-goal preflight-only rerun.

Guarded / not changed:

- No NavigateToPose goal was sent.
- `ingress_goal_sent=false`.
- `maze_explorer_started=false`.
- No maze_explorer was started.
- No Nav2/MPPI/controller/config tuning was performed.
- No Nav2/MPPI/controller/inflation/robot_radius/clearance_radius/map threshold/config tuning was performed.
- No exploration strategy, branch scoring, exploration order, centerline gate, readiness/fallback/terminal acceptance logic was changed.
- Preflight was not removed.
- This phase does not prove autonomous exploration success.
- This phase does not prove exit success.
- Phase115 not entered.

## Implementation summary

Updated preflight tool:

- `PreflightConfig` now includes `use_sim_time: bool = True`.
- Runtime sampler creates its ROS node with `parameter_overrides=[Parameter('use_sim_time', ...)]` from `config.use_sim_time`.
- CLI exposes mutually exclusive `--use-sim-time` / `--no-use-sim-time`; default is `--use-sim-time` for Gazebo/Nav2 simulation preflight.
- Artifacts record top-level `ingress_preflight.use_sim_time` and per-sample clock diagnostics.
- `/scan` subscription now uses sensor-compatible QoS: `ReliabilityPolicy.BEST_EFFORT`, `HistoryPolicy.KEEP_LAST`, `DurabilityPolicy.VOLATILE`.
- Runtime sampler now uses a bounded background `SingleThreadedExecutor` thread so `/clock`, `/scan`, and TF callbacks keep flowing while the main thread performs blocking lifecycle/action CLI checks.
- First-scan timeout classification was tightened: once a real scan callback/frame/stamp is observed, later stale/unavailable scan state is reported as scan/TF instability rather than the old `ingress_first_scan_timeout` token.

No pass/reject safety gate was weakened; the tool remains fail-closed.

## Focused TDD / static verification

RED evidence:

- Phase114 focused tests initially failed because the tool had no `use_sim_time` config/artifact contract, no CLI switches, no Phase114 analyzer/report completion token, and later still reported `ingress_first_scan_timeout` after a received scan.

GREEN evidence:

- `PYTHONDONTWRITEBYTECODE=1 python3 -m py_compile tools/phase105_inner_ingress_tf_controller_preflight.py tools/analyze_phase114_preflight_sim_time_alignment_fix.py src/tugbot_maze/test/test_phase114_preflight_sim_time_alignment_fix.py`
- `PYTHONDONTWRITEBYTECODE=1 pytest -q src/tugbot_maze/test/test_phase114_preflight_sim_time_alignment_fix.py -k 'not report_guardrails'`
- Result after implementation before final report guard: `7 passed, 1 deselected in 0.10s`.

Final regression bundle:

- Artifact: `phase114_preflight_sim_time_alignment_fix_final_guard_bundle.txt`
- `py_compile` over preflight/analyzers/tests passed.
- Focused Phase105/109/110/111/112/113/114 pytest result: `41 passed in 0.34s`.
- Final analyzer rerun on the accepted runtime artifact produced `SIM_TIME_ALIGNMENT_RECOVERED_FAIL_CLOSED_NEW_REASON`.
- Post-cleanup guard artifact: `phase114_preflight_sim_time_alignment_fix_post_cleanup_final_guards.txt`.
- Report guard pytest after cleanup: `1 passed in 0.02s`.
- Runtime process guard empty.
- Tracked Nav2 config diff guard empty.
- Scoped pycache guard empty.

Phase114 analyzer added:

- `tools/analyze_phase114_preflight_sim_time_alignment_fix.py`
- It classifies whether sim-time alignment recovered, whether wall-time-sized scan age remains, whether internal TF improved, and whether no-goal guardrails are intact.

## Runtime cleanup and launch evidence

Artifact directory:

- `log/phase114_preflight_sim_time_alignment_fix/`

Cleanup before reruns:

- Initial cleanup: `phase114_preflight_sim_time_alignment_fix_qos_rerun_cleanup_summary.json`, before_count=21, after_count=0.
- Final rerun cleanup before launch: `phase114_preflight_sim_time_alignment_fix_final_rerun_cleanup_summary.json`, before_count=21, after_count=0.

Visible stack was launched with:

- Gazebo visible (`headless:=false`).
- RViz enabled (`use_rviz:=true`).
- Stack `use_sim_time:=true`.
- No `maze_explorer` launch.
- No goal dispatch.

Readiness evidence:

- `phase114_preflight_sim_time_alignment_fix_final_ros_graph_ready.txt`
- External readiness confirmed `/navigate_to_pose`, `/map`, `/scan`, and `map->base_link` TF before the final preflight-only run.

## Runtime validation history

Phase114 intentionally kept intermediate reruns because they explain what the minimal fix needed:

1. Sim-time node parameter only
   - Artifact: `phase114_preflight_sim_time_alignment_fix_ingress_preflight.json`
   - `use_sim_time=true` recorded.
   - Still no effective internal scan callback in that run; classified `SIM_TIME_ALIGNMENT_NOT_EFFECTIVE`.

2. Sensor QoS rerun
   - Artifact: `phase114_preflight_sim_time_alignment_fix_qos_ingress_preflight.json`
   - Internal scan callback observed and wall-time-sized age disappeared in at least the first sample.
   - TF partially improved (`map->odom` observed), but callbacks stopped advancing while blocking runtime checks ran; still not sufficient.

3. Background executor final validation
   - Artifact: `phase114_preflight_sim_time_alignment_fix_background_executor_ingress_preflight.json`
   - Analysis: `phase114_preflight_sim_time_alignment_fix_background_executor_analysis.json`
   - Minimal summary: `phase114_preflight_sim_time_alignment_fix_background_executor_minimal_summary.md`
   - Classification: `SIM_TIME_ALIGNMENT_RECOVERED_FAIL_CLOSED_NEW_REASON`.

## Final no-goal preflight-only result

Final preflight stdout:

```json
{"failed_gates": ["ingress_lifecycle_ambiguous", "ingress_preflight_timeout"], "passed": false, "reject_reason": "ingress_preflight_timeout"}
```

Key final analyzer evidence from `phase114_preflight_sim_time_alignment_fix_background_executor_analysis.json`:

- classification: `SIM_TIME_ALIGNMENT_RECOVERED_FAIL_CLOSED_NEW_REASON`
- passed: `false`
- reject_reason: `ingress_preflight_timeout`
- artifact_use_sim_time: `true`
- all_samples_use_sim_time: `true`
- callback_count_max: `364`
- latest scan frame: `tugbot/scan_omni/scan_omni`
- scan age values: `[0.09300000000001774, 0.02400000000000091, 0.03900000000004411, 0.06599999999997408]`
- max_scan_age_sec: `0.09300000000001774`
- wall_time_sized_age_detected: `false`
- map->base_link internal TF can_transform: `true`
- map->odom internal TF can_transform: `true`
- odom->base_link internal TF can_transform: `true`
- no_goal_dispatch_guard_valid: `true`
- ingress_goal_sent: `false`
- maze_explorer_started: `false`

Sample-level final evidence:

- Sample 0: `/scan` callback count 87, scan age 0.093 sec, all core TF pairs available.
- Sample 1: `/scan` callback count 179, scan age 0.024 sec, all core TF pairs available.
- Sample 2: `/scan` callback count 272, scan age 0.039 sec, all core TF pairs available.
- Sample 3: `/scan` callback count 364, scan age 0.066 sec, all core TF pairs available.

## Required Phase114 judgments

1. Preflight artifact records `use_sim_time=true`:
   - Yes. Top-level artifact and all sample clock diagnostics record sim time.

2. `/scan` callback age no longer wall-time-sized:
   - Yes. Final max scan age is about 0.093 sec, not ~1.78e9 sec.

3. Internal TF buffer diagnostics improved:
   - Yes. Final analyzer reports internal `can_transform=true` for `map->base_link`, `map->odom`, and `odom->base_link`.

4. If still fail-closed, new true reject reason is not `ingress_first_scan_timeout`:
   - Yes. Final fail-closed reason is `ingress_preflight_timeout` with `ingress_lifecycle_ambiguous`, not `ingress_first_scan_timeout`.
   - This means Phase113's scan-age/TF-buffer root cause is fixed, but preflight did not pass in the bounded window because lifecycle confirmation remained ambiguous.

5. No goal dispatch / no explorer:
   - Yes. `ingress_goal_sent=false`, `maze_explorer_started=false`, `no_goal_dispatch_guard_valid=true`.

## Cleanup after runtime

Final cleanup artifact:

- `phase114_preflight_sim_time_alignment_fix_final_cleanup_summary.json`
- before_count=21
- after_count=0
- unrelated_processes_targeted=false

## Final conclusion

Phase114 completed the minimal sim-time alignment fix and verified it in a real visible-stack no-goal preflight-only rerun.

The Phase113 wall-time-sized scan age failure is no longer present. Internal `/scan` callbacks advance under sim time, scan age is fresh, and the internal TF buffer resolves the required core transforms. The preflight still fails closed, but with a new bounded-window reason (`ingress_preflight_timeout` plus lifecycle ambiguity), not the old `ingress_first_scan_timeout` signature.

No autonomous navigation was attempted. No ingress goal was sent. No `maze_explorer` was started. No Nav2 or exploration strategy tuning was performed.

Stop here for human acceptance. Phase115 not entered.
