# Phase26Q MPPI evidence hardening: summarized trajectories and critics_stats audit

Date: 2026-05-26
Workspace: `/home/hyh/Desktop/agent_playground/playground_hermes/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260522`

## Goal

Phase26Q remains analysis-first and does not enter intervention.

The phase addresses two Phase26P gaps:

1. Raw `/trajectories` MarkerArray evidence was too large for longer runs.
2. `publish_critics_stats: true` did not produce a runtime `critics_stats` topic, so we needed to confirm whether this Nav2 Jazzy MPPI build actually supports critic-score publication.

## Changes

### 1. Online evidence summarization

Updated:

- `tools/record_phase26p_mppi_evidence.py`

The recorder now writes compact `data_summary` rows by default instead of full raw message payloads.

For `visualization_msgs/msg/MarkerArray` / `/trajectories`, it records:

- `summary_kind: marker_array_trajectory_summary`
- `marker_count`
- `point_count`
- `trajectory_count`
- `trajectory_displacement_min`
- `trajectory_displacement_max`
- `trajectory_displacement_mean`
- `trajectory_path_length_min`
- `trajectory_path_length_max`
- `trajectory_path_length_mean`
- `representative_path_length`
- `representative_marker_id`
- `degenerate_trajectory_count`
- `near_zero_trajectory_count`

For `nav_msgs/msg/Path` topics such as `/optimal_trajectory` and `/transformed_global_plan`, it records:

- `summary_kind: path_summary`
- `point_count`
- `path_displacement`
- `path_length`

The recorder still supports optional raw payload capture via `--raw-topics-regex`, but the wrapper passes `--raw-topics-regex '^$'` so default Phase26Q diagnostics write summaries only.

### 2. Analyzer support for summarized evidence

Updated:

- `tools/analyze_phase26p_mppi_evidence.py`

The analyzer now accepts both old raw `data` rows and new `data_summary` rows.

New per-case field:

- `trajectory_summary`
  - `sample_count`
  - `marker_count_max`
  - `point_count_max`
  - `degenerate_trajectory_count_max`
  - `representative_path_length_max`

If summarized trajectories indicate degenerate trajectories but critic stats are absent, the analyzer can classify:

- `trajectory_summary_degenerate_without_critic_stats`

This is still not an intervention signal; it is evidence triage.

### 3. MPPI debug capability audit

Added:

- `tools/audit_phase26q_mppi_debug_capabilities.py`

It inspects `/opt/ros/jazzy` for:

- `publish_critics_stats` parameter support in installed `nav2_mppi_controller` headers/share files;
- `critics_stats` string/symbol in `libmppi_controller.so`;
- `CriticsStats` message availability in installed `nav2_msgs` interfaces.

Audit artifact:

- `log/phase26q_mppi_debug_capabilities_audit.json`

Result:

```json
{
  "critics_stats": {
    "status": "unavailable_in_installed_jazzy_mppi",
    "parameter_name_supported": false,
    "library_symbol_available": false,
    "message_type_available": false
  },
  "recommendation": "do_not_block_on_critics_stats_use_summarized_trajectories_and_existing_logs"
}
```

Installed evidence also confirmed available MPPI debug strings in `libmppi_controller.so`:

- `/trajectories`
- `optimal_trajectory`
- `transformed_global_plan`

### 4. Wrapper integration

Updated:

- `tools/run_phase21_controller_diagnostics_smoke.sh`

Phase26P/26Q diagnostics wrapper now writes:

- `${RUN_ID}_mppi_evidence_summary.jsonl`
- `${RUN_ID}_phase26q_mppi_debug_capabilities.json`

and invokes recorder with:

```bash
--raw-topics-regex '^$'
```

so future runs do not store full raw MarkerArray JSON by default.

## TDD and verification

New test:

- `src/tugbot_maze/test/test_phase26q_mppi_evidence_hardening.py`

RED was verified first:

- missing `summarize_message` / `summarize_marker_array`
- analyzer still reported `phase: 26P`
- missing audit tool
- wrapper still used raw `*_mppi_evidence.jsonl`

GREEN / verification:

- `python3 -m pytest src/tugbot_maze/test/test_phase26q_mppi_evidence_hardening.py src/tugbot_maze/test/test_phase26p_mppi_diagnostics.py -q`
  - `7 passed in 0.24s`

- `python3 -m py_compile tools/record_phase26p_mppi_evidence.py tools/analyze_phase26p_mppi_evidence.py tools/audit_phase26q_mppi_debug_capabilities.py`
  - passed

- `bash -n tools/run_phase21_controller_diagnostics_smoke.sh`
  - passed

- `python3 -m pytest src/tugbot_maze/test/test_phase24*.py src/tugbot_maze/test/test_phase25*.py src/tugbot_maze/test/test_phase26*.py -q`
  - `80 passed in 1.49s`

- `. /opt/ros/jazzy/setup.sh && . install/setup.sh && colcon build --symlink-install --packages-select tugbot_bringup tugbot_navigation tugbot_maze`
  - `3 packages finished [0.91s]`

- final process cleanup check found no lingering ROS/Gazebo/Nav2/recorder processes.

## Compression proof using Phase26P raw artifact

Existing Phase26P raw artifact:

- `log/phase26p_baseline_diag_run1_mppi_evidence.jsonl`
- size: `284,445,894 bytes` (~271 MB)

Phase26Q re-summarized a small representative subset from the raw artifact:

- `log/phase26q_reanalyzed_phase26p_baseline_diag_run1_mppi_evidence_summary.jsonl`
- rows written: `16`
- size: `5,702 bytes`

This demonstrates the desired order-of-magnitude payload reduction for MarkerArray evidence. Future real Phase26Q wrapper runs will produce compact summary rows online rather than post-processing raw rows.

The reanalysis artifact:

- `log/phase26q_reanalyzed_phase26p_baseline_diag_run1_analysis.json`

Because the representative subset did not overlap the first-cmd-near-zero window, it reports:

- `condition_hypothesis: insufficient_mppi_evidence`

This is expected and does not change the Phase26P conclusion.

## Critics stats conclusion

For this installed Nav2 Jazzy MPPI build, `critics_stats` should be treated as unavailable evidence.

Evidence:

- no installed `nav2_msgs` `CriticsStats` interface found;
- no `publish_critics_stats` parameter found in installed `nav2_mppi_controller` headers/share files;
- no `critics_stats` string/symbol found in `/opt/ros/jazzy/lib/libmppi_controller.so`;
- Phase26P runtime topic discovery also did not find a `critics_stats` topic.

Therefore, do not block the diagnostics roadmap on `critics_stats` for this environment.

## Decision

Phase26Q does not locate a specific actionable MPPI condition.

It does establish the evidence strategy going forward:

- use summarized `/trajectories` MarkerArray evidence;
- use summarized `/optimal_trajectory` and `/transformed_global_plan` path evidence;
- continue joining with `cmd_vel_nav`, local-cost snapshots, `/maze/goal_events`, Nav2/controller logs, and runtime param dumps;
- mark `critics_stats` as unavailable in this Jazzy MPPI installation.

Guardrails remain:

- Phase27 remains blocked;
- do not tune Nav2/controller params from Phase26Q;
- do not change branch selection from Phase26Q;
- do not promote/reject `2.75` from Phase26Q.

## Recommended next step

Run a bounded Phase26Q summary-evidence smoke after this hardening, e.g. `phase26p_baseline_diag_run2` or a new `phase26q_baseline_summary_run1` if we add a dedicated run ID alias.

Acceptance for that run should be evidence-quality only:

- compact `${RUN_ID}_mppi_evidence_summary.jsonl` remains small enough for longer runs;
- includes `/trajectories`, `/optimal_trajectory`, and `/transformed_global_plan` summaries near the first-cmd-near-zero windows;
- analyzer reports trajectory summary metrics in the join windows;
- no intervention unless those metrics identify a stable, specific condition across matched repeats.
