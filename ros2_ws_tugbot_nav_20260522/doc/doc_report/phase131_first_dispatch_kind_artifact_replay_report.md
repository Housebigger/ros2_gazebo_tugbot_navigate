# Phase131 first dispatch kind artifact replay report

Status: PHASE131_FIRST_DISPATCH_KIND_ARTIFACT_REPLAY_COMPLETE_STOP_BEFORE_PHASE132

## Scope

Phase131 was executed as an offline artifact replay/analyzer phase after Phase130.

Allowed work performed:

- Added `tools/analyze_phase131_first_dispatch_kind_artifact_replay.py`.
- Added focused static/analyzer tests in `src/tugbot_maze/test/test_phase131_first_dispatch_kind_artifact_replay.py`.
- Read existing Phase124/125/129 artifacts/logs only.
- Read `maze_explorer.py` source code only to understand field semantics for `corridor_alignment_staging`, `staging_applied`, post-ingress flags, topology/candidate fields, and target transformation fields.
- Wrote analysis outputs under `log/phase131_first_dispatch_kind_artifact_replay/`.

Guardrails preserved:

- No Gazebo/RViz/Nav2 runtime was launched.
- No NavigateToPose goal was sent.
- No maze_explorer was started.
- No exploration/corridor/staging goal was sent.
- No Nav2/MPPI/controller/goal checker/config tuning was performed.
- No exploration strategy/branch scoring/centerline/fallback/terminal acceptance change was made.
- No autonomous exploration success or exit success is claimed.
- No timeout was repaired or reclassified as success.
- Phase132 not entered.

## Inputs

Phase131 analyzer inputs:

- `log/phase124_first_exploration_goal_dispatch_smoke/phase124_first_exploration_goal_dispatch_smoke_rerun_artifact.json`
- `log/phase125_first_exploration_goal_execution_result_smoke/phase125_first_exploration_goal_execution_result_smoke_artifact.json`
- `log/phase129_instrumented_first_goal_timeout_diagnosis/phase129_instrumented_first_goal_timeout_diagnosis_rerun.json`

Source-code semantics inspected read-only:

- `src/tugbot_maze/tugbot_maze/maze_explorer.py`
  - `_send_goal()` records `original_target`, `refined_target`, `goal_kind`, `candidate_count`, `staging_applied`, `staging_reason`, `two_step_stage_dispatch_requested`, and post-ingress/topology diagnostics in the dispatch event context.
  - `corridor_alignment_staging` is assigned only when two-step staging is applied to an originally requested `goal_kind='explore'` dispatch.
  - `pending_corridor_alignment_second_step` records the original explore target and branch context for a later second step, but Phase129 stopped after the first non-explore dispatch and did not send a second goal.

## Analyzer output

Artifacts produced:

- `log/phase131_first_dispatch_kind_artifact_replay/phase131_first_dispatch_kind_artifact_replay_analysis.json`
- `log/phase131_first_dispatch_kind_artifact_replay/phase131_first_dispatch_kind_artifact_replay_summary.md`
- `log/phase131_first_dispatch_kind_artifact_replay/phase131_analyzer_stdout.log`

Analyzer command:

```bash
python3 tools/analyze_phase131_first_dispatch_kind_artifact_replay.py \
  --output-json log/phase131_first_dispatch_kind_artifact_replay/phase131_first_dispatch_kind_artifact_replay_analysis.json \
  --output-md log/phase131_first_dispatch_kind_artifact_replay/phase131_first_dispatch_kind_artifact_replay_summary.md
```

Observed stdout:

```json
{"classification": "FIRST_DISPATCH_KIND_CHANGED_BY_POSE_TOPOLOGY_DRIFT", "dispatch_kind_sequence": ["explore", "explore", "corridor_alignment_staging"], "output_json": "log/phase131_first_dispatch_kind_artifact_replay/phase131_first_dispatch_kind_artifact_replay_analysis.json", "output_md": "log/phase131_first_dispatch_kind_artifact_replay/phase131_first_dispatch_kind_artifact_replay_summary.md"}
```

## Classification

Final Phase131 classification:

`FIRST_DISPATCH_KIND_CHANGED_BY_POSE_TOPOLOGY_DRIFT`

This is diagnostic-only. It does not repair the timeout, does not authorize tuning, and does not turn the Phase125/127 timeout into success.

## Dispatch-kind comparison

Phase124/125 first dispatch remained goal_kind=explore. Phase129 first dispatch changed to goal_kind=corridor_alignment_staging.

| Field | Phase124 | Phase125 | Phase129 |
| --- | --- | --- | --- |
| goal_kind | explore | explore | corridor_alignment_staging |
| dispatch_pose | `[1.8538556568923468, 0.02183342995476324, -0.006870047456431466]` | `[1.8569191491890518, 0.024192576812237812, -0.005571541767227017]` | `[1.8502748183420978, 0.023517360243981468, -0.004911685743140483]` |
| current_node_id | 2 | 2 | 2 |
| start_node_id | 2 | 2 | 2 |
| topology_state/local_topology | junction | junction | junction |
| candidate_branch_count | 4 | 4 | 3 |
| last_open_direction_count | 4 | 4 | 3 |
| last_candidate_count | 4 | 4 | 3 |
| near_exit | false | false | false |
| selection_reason | topology_exit_bias_score | topology_exit_bias_score | topology_exit_bias_score |
| original_target | `[2.0857203439562952, 1.0202640827301475]` | `[2.0874871732589573, 1.0229234653829486]` | `[1.7051882914469514, 1.0242420478499465]` |
| raw/refined target | same as original explore target | same as original explore target | `[1.7005222087522947, 0.07425350703890773]` near-robot staging target |

## Trigger evidence by Phase130 hypothesis

### 1. robot pose after ingress / yaw / distance_to_ingress

The dispatch pose proxy for robot pose after ingress was materially stable:

- Phase125 to Phase129 dispatch pose XY delta: `0.006678551476018686 m`.
- Phase125 to Phase129 dispatch yaw delta: `0.0006598560240865343 rad`.
- Analyzer flag: `pose_yaw_drift_material=false`.

This does not support a simple large pose/yaw drift explanation.

### 2. current_node_id/start_node_id, local_topology, topology state

Node/topology labels were stable:

- `current_node_id=2` in Phase124/125/129.
- `start_node_id=2` in Phase124/125/129.
- `local_topology=junction` in Phase124/125/129.
- `near_exit=false` in Phase124/125/129.

The same coarse node/topology labels therefore do not explain the dispatch-kind change by themselves.

### 3. candidate_family/rank/count, candidate_branch_count, last_open_direction_count, last_candidate_count

Candidate/topology counts changed:

- candidate_branch_count changed from 4 to 3.
- last_open_direction_count changed from 4 to 3.
- last_candidate_count changed from 4 to 3.
- `candidate_branch_count_delta_from_phase125=-1`.
- `last_open_direction_count_delta_from_phase125=-1`.
- `last_candidate_count_delta_from_phase125=-1`.

This is the strongest artifact-level discrepancy explaining why Phase129 no longer matched the Phase124/125 first-dispatch contract.

### 4. near_exit, post_ingress flags, active_edge/state machine

Phase129 did not show post-ingress exception activation in the dispatch event:

- `post_ingress_context_active=false`.
- `first_post_ingress_topology_node=false`.
- `single_open_exception_applied=false`.
- `single_open_exception_reason=idle`.
- `active_edge_id=null`.
- `active_goal_kind=corridor_alignment_staging` at dispatch.
- `mode=NAVIGATING` after dispatch context was recorded.

Therefore Phase131 does not classify this as `FIRST_DISPATCH_KIND_CHANGED_TO_STAGING_POST_INGRESS`.

### 5. goal_count/max_goals and first dispatch event ordering

Phase129 remained a first-dispatch, single-goal-budget artifact:

- `goal_sequence=1`.
- `goal_count_before_dispatch=0`.
- `max_goals=1`.
- `single_goal_budget=true`.
- `dispatch_event_count=1`.
- `second_goal_dispatched=false`.

This supports the Phase129 fail-closed interpretation: the first budget slot was consumed by a staging dispatch, so Phase129 could not be used as a first `goal_kind=explore` timeout replay.

### 6. raw_target/refined_target/original_target, selection_reason

Phase129 preserved an explore-like forward `original_target`, but dispatched a near-robot staging target:

- Phase129 `original_target=[1.7051882914469514, 1.0242420478499465]`.
- Phase129 raw/refined/dispatch target: `[1.7005222087522947, 0.07425350703890773]`.
- `original_to_refined_delta_m=0.9500000000000001`.
- `refined_target_distance_from_robot_m=0.15811388512889157`.
- `transformed_forward_target_to_near_robot_staging_target=true`.
- `selection_reason=topology_exit_bias_score` remained unchanged.

This supports the source-code semantics: Phase129 selected an explore branch context, then two-step corridor-alignment staging transformed the outgoing dispatch to `goal_kind=corridor_alignment_staging`.

### 7. corridor_alignment_staging trigger evidence

Phase129 dispatch carried explicit staging evidence:

- `staging_applied=true`.
- `two_step_stage_dispatch_requested=true`.
- `staging_reason=reduce_lateral_residual_and_front_wedge_risk_before_second_step_forward_goal`.
- `staging_lateral_residual_before_m=0.15000000223517437`.
- `staging_lateral_residual_after_m≈0`.
- `corridor_heading_yaw=1.5658846410517562`.

This supports a corridor-alignment staging mechanism, not a Nav2 timeout result.

## Why not other classifications

- Not `FIRST_DISPATCH_KIND_STABLE_EXPLORE`: Phase129 first dispatch was `goal_kind=corridor_alignment_staging`, not `explore`.
- Not `FIRST_DISPATCH_KIND_CHANGED_TO_STAGING_POST_INGRESS`: Phase129 artifact has `post_ingress_context_active=false` and `single_open_exception_applied=false`.
- Not `FIRST_DISPATCH_KIND_CONTRACT_AMBIGUOUS`: enough evidence exists to show count drift plus explicit staging target transformation.
- Not `INSUFFICIENT_DISPATCH_KIND_EVIDENCE`: all three existing first-dispatch artifacts include required dispatch event fields.

## Phase127/129 boundary retained

Phase127 remains an artifact replay diagnosis of the Phase125 first explore-goal timeout as:

`FIRST_GOAL_TIMEOUT_LOCAL_COST_BLOCKED`

Phase129 remains:

`INSUFFICIENT_TIMEOUT_EVIDENCE`

Phase131 does not strengthen, repair, or override Phase127. It explains why Phase129 could not be used to strengthen Phase125/127: Phase129 did not produce the same first `goal_kind=explore` dispatch; it produced a first `goal_kind=corridor_alignment_staging` dispatch.

## Test-first evidence

Initial RED:

```text
7 failed in 0.08s
```

Expected initial RED reason: missing `tools/analyze_phase131_first_dispatch_kind_artifact_replay.py` and missing Phase131 report.

Intermediate GREEN for analyzer before report:

```text
6 passed, 1 failed in 0.09s
```

Expected remaining failure: missing report file.

Final verification is recorded after report generation in the final guard/test section below.

## Final verification

Focused tests:

```text
7 passed in 0.08s
```

Phase130/131 static bundle:

```text
15 passed in 0.07s
```

Analyzer syntax check:

```text
python3 -m py_compile tools/analyze_phase131_first_dispatch_kind_artifact_replay.py
py_compile ok
```

Final guard artifact:

- `log/phase131_first_dispatch_kind_artifact_replay/phase131_final_guard.txt`
- process guard: empty
- protected config diff guard: empty
- unexpected Phase131 runtime runner guard: `no runtime runner present`
- analyzer forbidden runtime token guard: `forbidden runtime tokens absent`
- pycache guard: clean after cleanup

## Stop condition

Phase131 is complete and stops here for human acceptance. Phase132 not entered.
