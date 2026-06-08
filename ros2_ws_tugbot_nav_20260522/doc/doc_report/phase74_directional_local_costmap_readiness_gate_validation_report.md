# Phase74 Directional Local Costmap Readiness Gate / Bounded Runtime Validation

## Status

Completed and stopped at Phase74. Phase75 not entered.

Classification: `DIRECTIONAL_GATE_ENABLES_GOAL2_DISPATCH`

This is bounded runtime evidence only. It does not claim autonomous exploration success and does not claim exit success.

## Goal

Validate a minimal direction-aware local costmap readiness override after Phase73 classified the full-window local costmap gate as too strict for post-success redispatch.

Phase74 reused the Phase65 inner ingress staging point:

- `x=2.0`
- `y=0.0`
- `yaw=0.0`

Bounded runtime:

- `replay=2`
- `max_goals=3`

## Guardrails verified

- No Nav2/MPPI/controller tuning.
- No inflation/robot_radius/clearance_radius_m/map threshold tuning.
- No branch scoring change.
- No centerline gate runtime behavior change.
- No fallback/terminal acceptance change.
- No autonomous exploration success claim.
- No exit success claim.

Analyzer guardrail checks:

```json
{
  "branch_scoring_changed": false,
  "centerline_gate_runtime_behavior_changed": false,
  "cleanup_processes_after_empty": true,
  "directional_local_costmap_readiness_override_enabled": true,
  "guardrail_violation": false,
  "max_goals_exactly_3": true,
  "nav2_config_diff_empty": true,
  "replay_count_exactly_2": true
}
```

## Files added/changed

Implementation:

- `src/tugbot_maze/tugbot_maze/maze_explorer.py`
  - Added minimal direction-aware local costmap readiness override.
  - Preserves the original full-window local costmap gate record.
  - Adds directional override diagnostics without changing branch scoring, centerline target refinement behavior, fallback, terminal acceptance, or Nav2 parameters.

Tests/analyzer/wrapper/report:

- `src/tugbot_maze/test/test_phase74_directional_local_costmap_readiness_gate_validation.py`
- `tools/analyze_phase74_directional_local_costmap_readiness_gate_validation.py`
- `tools/run_phase74_directional_local_costmap_readiness_gate_validation.sh`
- `doc/doc_report/phase74_directional_local_costmap_readiness_gate_validation_report.md`

Artifacts:

- `log/phase74_directional_local_costmap_readiness_gate_validation/phase74_directional_local_costmap_readiness_gate_validation.json`
- `log/phase74_directional_local_costmap_readiness_gate_validation/replay_01/`
- `log/phase74_directional_local_costmap_readiness_gate_validation/replay_02/`

## Evidence summary

Analyzer metrics:

```json
{
  "blacklisted_goal_count_total": 0,
  "blocked_branch_count_total": 0,
  "directional_override_applied_replay_count": 1,
  "dispatch_count_total": 3,
  "goal1_success_replay_count": 1,
  "goal2_dispatch_replay_count": 1,
  "outcome_count_total": 5,
  "redispatch_after_goal1_success_observed": true,
  "timeout_count_total": 4
}
```

### replay_01

- Inner ingress action: success.
- Goal1 dispatch observed: yes.
- Goal1 outcome: timeout, not success.
- Goal2 dispatch observed: no.
- Final mode: `WAIT_FOR_DISPATCH_ENTRY_READINESS`.
- Final goal count: `1`.
- Directional override considered: yes.
- Directional override applied: no.
- Max traversable direction count observed: `0`.
- Blocked branch count: `0`.
- Blacklisted goal count: `0`.

Key local costmap evidence at the readiness gate:

- Full-window `free_ratio`: about `0.468 < 0.50`.
- Full-window known ratio: `1.0`.
- Local topology: `junction`.
- Directional override reason: `no_directional_corridor_traversable`.

Interpretation: replay_01 did not exercise post-Goal1-success redispatch because Goal1 timed out. It is not evidence against post-success redispatch; it is a bounded-runtime timeout case with no override application.

### replay_02

- Inner ingress action: success.
- Goal1 dispatch observed: yes.
- Goal1 outcome: success.
- Goal2 dispatch observed after Goal1 success: yes.
- Final mode: `WAIT_FOR_DISPATCH_ENTRY_READINESS`.
- Final goal count: `2`.
- Directional override applied: yes.
- Override applied observations: `72`.
- Max traversable direction count observed: `2`.
- Max non-reverse traversable direction count observed: `1`.
- Blocked branch count: `0`.
- Blacklisted goal count: `0`.

Selected directional override evidence:

```json
{
  "candidate_direction_angle_deg": 91.36847485496499,
  "direction_corridor_traversable": true,
  "is_reverse_candidate": false,
  "min_clearance_m": 0.6500000096857548,
  "path_cost": {
    "coverage_ratio": 1.0,
    "in_bounds_sample_count": 22,
    "max": 46,
    "mean": 17.09090909090909,
    "sample_count": 22,
    "threshold": 70
  },
  "target_risk": {
    "target_in_local_costmap_bounds": true,
    "target_local_cost": 0,
    "target_local_cost_max_radius": 46
  },
  "footprint_risk": {
    "high_cost_count": 0,
    "lethal_count": 0,
    "max": 44,
    "mean": 9.208333333333334,
    "p95": 40.0,
    "sample_count": 120
  },
  "front_wedge_risk": {
    "clearance_m": 0.3399265798206598,
    "high_cost_count": 29,
    "lethal_count": 12,
    "max": 99,
    "mean": 41.25
  }
}
```

Full-window gate evidence retained for the same replay:

- Full-window `free_ratio`: about `0.465 < 0.50`.
- Full-window known ratio: `1.0`.
- Full-window reason: `local_costmap_ratio_or_bounds_insufficient`.
- Directional override reason: `directional_corridor_traversable`.

Interpretation: replay_02 directly answers Phase74's post-success redispatch question. After Goal1 success, the full-window local costmap readiness gate still failed, but the directional corridor evidence was sufficient and the override allowed the state machine to proceed into Goal2 dispatch.

Goal2 then timed out. This is recorded as a downstream bounded-runtime navigation outcome, not as exit success and not as autonomous exploration success.

## Verification commands and outputs

Focused Phase74 test:

```text
pytest -q src/tugbot_maze/test/test_phase74_directional_local_costmap_readiness_gate_validation.py
......                                                                   [100%]
6 passed in 0.01s
```

Analyzer rerun:

```text
python3 tools/analyze_phase74_directional_local_costmap_readiness_gate_validation.py --analyze --artifact-dir log/phase74_directional_local_costmap_readiness_gate_validation --output log/phase74_directional_local_costmap_readiness_gate_validation/phase74_directional_local_costmap_readiness_gate_validation.json
{"classification": "DIRECTIONAL_GATE_ENABLES_GOAL2_DISPATCH", "output": "log/phase74_directional_local_costmap_readiness_gate_validation/phase74_directional_local_costmap_readiness_gate_validation.json", "replay_count": 2}
```

Syntax checks:

```text
python3 -m py_compile src/tugbot_maze/tugbot_maze/maze_explorer.py tools/analyze_phase74_directional_local_costmap_readiness_gate_validation.py
bash -n tools/run_phase74_directional_local_costmap_readiness_gate_validation.sh
```

Output: no errors.

Build:

```text
colcon build --symlink-install --packages-select tugbot_maze tugbot_bringup
Starting >>> tugbot_maze
Finished <<< tugbot_maze [0.64s]
Starting >>> tugbot_bringup
Finished <<< tugbot_bringup [0.08s]

Summary: 2 packages finished [0.84s]
```

Nav2 config diff:

```text
git diff -- src/tugbot_navigation/config
```

Output: empty.

Cleanup check:

```text
pgrep -af '[r]os2 launch|[r]os2 run tugbot_maze maze_explorer|[g]z sim|[r]viz2|[s]lam_toolbox|[m]aze_explorer|[m]aze_goal_monitor|[f]rontier_explorer|[c]ontroller_server|[p]lanner_server|[b]t_navigator|[b]ehavior_server|[w]aypoint_follower|[v]elocity_smoother|[s]moother_server|[r]oute_server|[c]ollision_monitor|[d]ocking_server|[r]os_gz_bridge|[p]arameter_bridge|[s]tatic_transform_publisher' | grep -v 'hermes-snap' | grep -v 'pgrep -af' || true
```

Output: empty.

## Conclusion

Phase74 supports the classification:

`DIRECTIONAL_GATE_ENABLES_GOAL2_DISPATCH`

Reason:

- Phase72/73 failure mode was reproduced in the relevant condition: full-window local costmap readiness failed post-success because `free_ratio < 0.50` even though knownness/freshness/in-bounds were good.
- In replay_02, Goal1 succeeded.
- The Phase74 directional override found a non-reverse traversable candidate direction.
- The state machine proceeded to Goal2 dispatch.
- No branch scoring, centerline runtime behavior, fallback/terminal behavior, Nav2 config, blocked branch count, or blacklist regression was observed.

Important caveat:

- Goal2 timed out in replay_02.
- replay_01 timed out on Goal1.
- Therefore Phase74 validates post-success redispatch enablement only; it does not validate sustained autonomous exploration, next-goal safety, or exit success.

Stop here. Do not enter Phase75 without explicit acceptance.
