# Phase130 first dispatch kind discrepancy diagnosis design report

Status: PHASE130_FIRST_DISPATCH_KIND_DISCREPANCY_DESIGN_COMPLETE_STOP_BEFORE_PHASE131

## Summary

Phase130 completed a doc-only/design-only review for the dispatch-kind discrepancy discovered by Phase129.

The motivating discrepancy is:

- Phase124 first dispatch: `goal_kind=explore`
- Phase125 first dispatch: `goal_kind=explore`
- Phase129 first dispatch: `goal_kind=corridor_alignment_staging`

Phase129 therefore correctly classified its runtime as `INSUFFICIENT_TIMEOUT_EVIDENCE` and stopped fail-closed with `second_goal_dispatched=false`. Phase127's `FIRST_GOAL_TIMEOUT_LOCAL_COST_BLOCKED` remains only an artifact replay diagnosis from Phase125 timeout evidence; Phase129 did not strengthen, repair, or override it.

Phase130 does not fix or tune this behavior. It defines the evidence contract for a future Phase131 artifact replay/analyzer that should compare existing Phase124/125/129 artifacts offline.

## Scope compliance

No implementation was performed beyond docs and focused static tests.

Runtime and behavior guardrails preserved:

- No Gazebo/RViz/Nav2 runtime was launched.
- No NavigateToPose goal was sent.
- No maze_explorer was started.
- No exploration/corridor/staging goal was sent.
- No Nav2/MPPI/controller/goal checker/config tuning was performed.
- No exploration strategy was changed.
- No branch scoring was changed.
- No centerline gate was changed.
- No fallback was changed.
- No terminal acceptance was changed.
- No autonomous exploration success or exit success is claimed.
- Phase131 not entered.

## Delivered files

- Proposal: `doc/doc_proposal/phase130_first_dispatch_kind_discrepancy_design.md`
- Report: `doc/doc_report/phase130_first_dispatch_kind_discrepancy_design_report.md`
- Focused static tests: `src/tugbot_maze/test/test_phase130_first_dispatch_kind_discrepancy_design.py`

No Phase130 runtime runner/analyzer was added.

## Design decisions

### 1. Dispatch-kind comparison is separate from timeout diagnosis

Phase124/125 first-goal smoke only allows `goal_kind=explore`. A `corridor_alignment_staging` first dispatch is not a substitute for the Phase125 first explore-goal timeout and must not be mixed into timeout replay.

Therefore Phase130 keeps two diagnostic tracks separate:

- first dispatch kind discrepancy diagnosis; and
- Phase127 first explore-goal timeout/local-cost artifact replay diagnosis.

### 2. Phase124/125/129 comparison matrix is required

The proposal requires a future Phase131 analyzer to normalize and compare at least:

- robot pose after ingress
- current_node_id
- topology state
- candidate_family/rank/count
- candidate_branch_count
- last_open_direction_count
- last_candidate_count
- near_exit
- post_ingress flags
- active_edge/state machine
- goal_kind
- raw_target/refined_target/original_target
- selection_reason
- goal_count/max_goals

The design explicitly calls out the observed Phase124/125 `explore` vs Phase129 `corridor_alignment_staging` difference and the Phase129 branch-count/candidate-count drift from 4 to 3.

### 3. Staging trigger hypotheses are diagnosis-only

The proposal defines hypothesis buckets for future artifact replay:

- pose/yaw drift hypothesis
- centerline or corridor alignment hypothesis
- post-ingress context hypothesis
- topology consistency hypothesis
- candidate refinement hypothesis
- goal_count/max_goals interaction hypothesis
- state-machine or active-edge hypothesis

Each hypothesis is framed as diagnosis only; no repair, runtime rerun, strategy change, branch-scoring change, centerline change, or Nav2 tuning is authorized.

### 4. First dispatch kind contract is explicit

The proposal states:

- Phase124/125 first-goal smoke only allows `goal_kind=explore`.
- staging/corridor alignment must be classified separately.
- non-explore first dispatch must not be used as timeout-local-cost replay evidence.
- if older runners and newer runners observed different event streams, classify that as contract ambiguity rather than assuming a timeout replay.

### 5. Phase131 boundary is artifact replay/analyzer only

The proposal limits future Phase131 to:

- read existing Phase124/125/129 artifacts;
- compare existing Phase124/125/129 artifacts;
- optionally inspect existing source code only to understand field semantics and runner filters;
- create an offline analyzer/tests/report if requested.

Forbidden for Phase131 by this design:

- do not rerun runtime;
- do not launch Gazebo/RViz/Nav2;
- do not start maze_explorer;
- do not send any goal;
- do not tune or repair.

## Classification vocabulary defined for Phase131

Phase130 defines the following future Phase131 classifications:

- `FIRST_DISPATCH_KIND_STABLE_EXPLORE`
- `FIRST_DISPATCH_KIND_CHANGED_TO_STAGING_POST_INGRESS`
- `FIRST_DISPATCH_KIND_CHANGED_BY_POSE_TOPOLOGY_DRIFT`
- `FIRST_DISPATCH_KIND_CONTRACT_AMBIGUOUS`
- `INSUFFICIENT_DISPATCH_KIND_EVIDENCE`

These classifications are dispatch-kind classifications, not first-goal timeout classifications.

## Verification

Focused static tests were written before the design documents and verified RED:

```text
python3 -m pytest src/tugbot_maze/test/test_phase130_first_dispatch_kind_discrepancy_design.py -q
# 7 failed, 1 passed in 0.03s
```

Final verification after documents:

```text
python3 -m pytest src/tugbot_maze/test/test_phase130_first_dispatch_kind_discrepancy_design.py -q
# included in final focused/static bundle; Phase130 assertions passed
```

Static regression bundle:

```text
python3 -m pytest \
  src/tugbot_maze/test/test_phase128_first_goal_timeout_instrumentation_design.py \
  src/tugbot_maze/test/test_phase129_instrumented_first_goal_timeout_diagnosis.py \
  src/tugbot_maze/test/test_phase130_first_dispatch_kind_discrepancy_design.py \
  -q
# 25 passed in 0.07s
```

Final guard artifact:

- `log/phase130_first_dispatch_kind_discrepancy_design/phase130_final_guard.txt`

Final guard checks include:

- process guard
- protected config diff guard
- scoped pycache guard

Final guard summary after cleanup:

```text
phase130_process_guard: empty
phase130_protected_config_diff_guard: empty
phase130_doc_only_unexpected_runtime_files_guard: empty
phase130_pycache_guard: empty
```

## Stop condition

Phase130 completed the design review and stopped before Phase131.

Do not proceed to Phase131 without explicit human acceptance and a new phase request.
