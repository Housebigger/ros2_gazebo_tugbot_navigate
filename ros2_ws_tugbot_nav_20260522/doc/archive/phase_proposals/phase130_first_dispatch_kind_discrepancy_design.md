# Phase130 first dispatch kind discrepancy diagnosis design

Status: DESIGN_ONLY

## Purpose

Phase130 is a doc-only/design-only review after Phase129. The goal is to explain and define how a future offline diagnosis should determine why Phase124/125 first dispatch was `goal_kind=explore`, while Phase129 first dispatch changed to `goal_kind=corridor_alignment_staging`.

This phase does not diagnose by changing runtime behavior. It defines an evidence contract for a future Phase131 artifact replay/analyzer only.

## Scope and hard guardrails

Phase130 is doc-only/design-only.

Required negative guardrails:

- do not launch Gazebo/RViz/Nav2 runtime
- do not send NavigateToPose goal
- do not start maze_explorer
- do not send exploration/corridor/staging goal
- do not tune Nav2/MPPI/controller/goal checker/config
- do not change exploration strategy
- do not change branch scoring
- do not change centerline gate
- do not change fallback
- do not change terminal acceptance
- do not claim autonomous exploration success
- do not claim exit success
- Phase131 not entered

No implementation, runtime runner, runtime analyzer, launch file, Nav2 config, controller parameter, goal checker parameter, branch scoring code, centerline logic, fallback logic, terminal acceptance logic, or exploration behavior may be changed in Phase130.

## Inputs preserved from previous phases

### Phase124 first-dispatch smoke

Phase124 completed a visible-stack first exploration goal dispatch smoke and stopped at Nav2 acceptance.

Known evidence:

- classification: `FIRST_EXPLORATION_GOAL_DISPATCHED_ACCEPTED_STOP`
- same-run ingress and handoff: passed
- `maze_explorer max_goals=1`: started
- `dispatch_event_count=1`
- first dispatch `goal_kind=explore`
- accepted: `true`
- stop reason: `first_explore_goal_accepted_stop`
- robot pose after ingress: approximately `x=1.8538556568923468`, `y=0.02183342995476324`, `yaw=-0.006870047456431466`
- first target pose: approximately `x=2.0857203439562952`, `y=1.0202640827301475`, `yaw=1.5639262793384652`
- `current_node_id=2`
- `start_node_id=2`
- `candidate_family=junction`
- `candidate_rank=1`
- `candidate_count=4`
- `candidate_branch_count=4`
- `last_open_direction_count=4`
- `last_candidate_count=4`
- `near_exit=false`
- `selection_reason=topology_exit_bias_score`
- `local_topology=junction`
- topology consistency guard was idle in the report summary

### Phase125 first-goal execution-result smoke

Phase125 repeated the Phase124-style first explore goal and waited for the bounded result. It timed out.

Known evidence:

- classification: `FIRST_EXPLORATION_GOAL_RESULT_TIMEOUT_DIAGNOSTIC_FAIL`
- same-run ingress and handoff: passed
- `maze_explorer max_goals=1`: started
- `dispatch_event_count=1`
- `second_goal_dispatched=false`
- first dispatch `goal_kind=explore`
- accepted: `true`
- timeout: `true`
- stop reason: `first_explore_goal_result_timeout_stop`
- robot pose after ingress: approximately `x=1.8569191491890518`, `y=0.024192576812237812`, `yaw=-0.005571541767227017`
- first target pose: approximately `x=2.0874871732589573`, `y=1.0229234653829486`, `yaw=1.5652247850276695`
- `current_node_id=2`
- `start_node_id=2`
- `candidate_family=junction`
- `candidate_rank=1`
- `candidate_count=4`
- `candidate_branch_count=4`
- `last_open_direction_count=4`
- `last_candidate_count=4`
- `near_exit=false`
- `selection_reason=topology_exit_bias_score`
- `local_topology=junction`
- Phase23 timeout local-cost evidence was later replayed by Phase127

### Phase127 first-goal timeout artifact replay

Phase127 classified the Phase125 timeout artifact as `FIRST_GOAL_TIMEOUT_LOCAL_COST_BLOCKED` based on existing Phase125 evidence:

- `distance_remaining` plateau
- `recoveries=4`
- timeout line: `footprint_max=99 front_max=100 path_0_5m_max=73 path_1_0m_max=84`

This remains an artifact replay diagnosis. Phase129 did not strengthen, repair, or override Phase127.

### Phase129 instrumented first-goal runtime diagnosis

Phase129 repeated the visible-stack + ingress/handoff + `maze_explorer max_goals=1` chain with instrumentation. It did not reproduce the first explore goal timeout.

Known evidence:

- classification: `INSUFFICIENT_TIMEOUT_EVIDENCE`
- same-run ingress and handoff: passed in the bounded rerun
- `maze_explorer max_goals=1`: started
- `dispatch_event_count=1`
- `second_goal_dispatched=false`
- first dispatch `goal_kind=corridor_alignment_staging`
- result status: `REJECTED_NON_EXPLORE_GOAL_KIND`
- stop reason: `first_dispatch_not_explore_diagnostic_stop`
- first dispatch pose: approximately `x=1.7005222087522947`, `y=0.07425350703890773`, `yaw=1.5658846410517562`
- `current_node_id=2`
- `start_node_id=2`
- `candidate_family=junction`
- `candidate_rank=1`
- `candidate_count=3`
- `candidate_branch_count=3`
- `last_open_direction_count=3`
- `last_candidate_count=3`
- `near_exit=false`
- original target: approximately `[1.7051882914469514, 1.0242420478499465]`
- raw target: approximately `[1.7005222087522947, 0.07425350703890773]`
- refined target: approximately `[1.7005222087522947, 0.07425350703890773]`
- `target_local_cost=46`
- `target_local_cost_max_radius=99`
- `path_corridor_min_clearance_m=0.4301162697613631`
- `near_high_cost_band=true`

The Phase129 runner correctly stopped fail-closed and did not mix staging/corridor alignment with timeout replay. Future phases must explicitly do not mix staging/corridor alignment with timeout replay when interpreting Phase125/127 timeout artifacts.

## Phase124/125/129 artifact comparison matrix

A future Phase131 offline analyzer should produce a normalized table with at least these columns:

| Field | Phase124 expected extraction | Phase125 expected extraction | Phase129 expected extraction | Diagnostic use |
| --- | --- | --- | --- | --- |
| artifact path | Phase124 rerun artifact | Phase125 artifact | Phase129 rerun artifact | provenance |
| classification | dispatch accepted | timeout diagnostic fail | insufficient timeout evidence | phase result |
| handoff_allowed | true | true | true | same-run handoff comparability |
| robot pose after ingress | x/y/yaw near `(1.85, 0.02, 0)` | x/y/yaw near `(1.86, 0.02, 0)` | extract from Phase129 Phase120 artifact/preflight/first artifact | pose drift source |
| goal_kind | explore | explore | corridor_alignment_staging | first dispatch kind delta |
| current_node_id | 2 | 2 | 2 | topology node comparability |
| start_node_id | 2 | 2 | 2 | topology node comparability |
| topology state | junction | junction | junction | local topology comparability |
| local_topology | junction | junction | junction | normalized topology state |
| candidate_family | junction | junction | junction | candidate class comparability |
| candidate_rank | 1 | 1 | 1 | selected-rank comparability |
| candidate_count | 4 | 4 | 3 | topology/candidate drift |
| candidate_branch_count | 4 | 4 | 3 | branch-count drift |
| last_open_direction_count | 4 | 4 | 3 | open-direction drift |
| last_candidate_count | 4 | 4 | 3 | candidate-generation drift |
| near_exit | false | false | false | exclude near-exit terminal branch |
| post_ingress flags | extract from handoff/artifact if present | extract from handoff/artifact if present | extract from handoff/artifact if present | post-ingress context trigger check |
| active_edge/state machine | extract active edge, active goal kind, state-machine phase if present | same | same | staging state trigger check |
| goal_count/max_goals | first goal, max_goals=1 | first goal, max_goals=1 | first goal, max_goals=1 | goal budget interaction |
| raw_target | explore target | explore target | staging pose near robot | refinement/staging signature |
| refined_target | same as raw in 124/125 | same as raw in 125 | staging pose in 129 | refinement trigger signature |
| original_target | same as target in 124/125 | same as target in 125 | forward original target in 129 | staging-from-original-target evidence |
| selection_reason | topology_exit_bias_score | topology_exit_bias_score | extract if present/null | selection policy comparison |
| topology_consistency_guard_status | idle or absent | idle or absent | extract if present/null | consistency-guard trigger check |
| single_open_exception_applied | false or absent | false or absent | extract if present/null | post-ingress topology exception check |
| target_local_cost_max_radius | compare if available | compare if available | 99 | candidate risk/staging trigger check |
| path_corridor_min_clearance | compare if available | compare if available | 0.4301 | corridor risk trigger check |

The analyzer should also preserve missing fields explicitly as `missing`, not silently coerce them to `false` or `0`.

## Dispatch-kind discrepancy questions

Phase130 does not answer these questions by runtime replay. It defines how Phase131 should answer them offline:

1. Did the first dispatch kind truly change, or are Phase124/125 and Phase129 observing different event fields?
2. Did Phase129 start from a materially different robot pose after ingress, causing a staging pose near `(1.70, 0.07)` rather than an explore target near `(2.09, 1.02)`?
3. Did the topology state change from four open/candidate directions to three, making corridor alignment eligible?
4. Did the current node remain stable at `current_node_id=2`, or did hidden active-edge/state-machine context differ despite the same node id?
5. Did post_ingress flags, post-ingress context, or handoff metadata trigger a staging path that earlier smoke runners classified out of scope?
6. Did candidate refinement rewrite an original forward target into a staging/alignment pose?
7. Did `goal_count/max_goals` interact with staging so that a staging dispatch consumed the first budget slot under Phase129 but not Phase124/125?
8. Did Phase124/125 runner contracts filter or wait specifically for `goal_kind=explore`, while Phase129 captured the literal first dispatch event and therefore exposed a previously hidden staging event?

## corridor_alignment_staging trigger conditions to diagnose

Phase131 should test each hypothesis using only existing artifacts and source-code inspection if needed, not runtime replay.

### H1: pose/yaw drift hypothesis

`corridor_alignment_staging` may have triggered because the robot pose after ingress or immediate pre-dispatch pose differed enough from Phase124/125.

Evidence to compare:

- robot pose after ingress
- pre-dispatch robot pose if present
- first dispatch pose
- delta from ingress goal `(2.0, 0.0, 0.0)`
- yaw alignment to selected corridor
- body-frame lateral residual to original target

If Phase129 pose is materially different and topology/candidate counts also changed, this supports `FIRST_DISPATCH_KIND_CHANGED_BY_POSE_TOPOLOGY_DRIFT`.

### H2: centerline or corridor alignment hypothesis

`corridor_alignment_staging` may be a deliberate staging action generated by centerline/corridor-alignment logic before the actual exploration target.

Evidence to compare:

- original target vs raw_target vs refined_target
- whether Phase129 raw/refined target is close to robot while original target remains a forward explore target
- corridor heading yaw and lateral residual, if present
- centerline/corridor alignment diagnostic fields, if present
- target local cost and local-cost radius risk

This is diagnosis only; no repair and no centerline gate change is authorized.

### H3: post-ingress context hypothesis

A post-ingress flag or one-shot startup context may have enabled a staging path in Phase129 that was absent or hidden in Phase124/125.

Evidence to compare:

- post_ingress flags
- handoff artifact preconditions
- warmup/dry-start state
- startup context fields
- whether staging is marked as post-ingress or corridor-alignment entry behavior

This phase does not change post-ingress behavior.

### H4: topology consistency hypothesis

`corridor_alignment_staging` may have appeared because topology sampling changed from Phase124/125 four-branch evidence to Phase129 three-branch evidence.

Evidence to compare:

- topology state
- local_topology
- last_open_direction_count
- last_candidate_count
- candidate_branch_count
- topology consistency guard state/status
- single-open exception fields
- candidate branch angles and rejection reasons

If topology/candidate counts drift without an explicit staging trigger field, classify cautiously rather than inferring a cause.

### H5: candidate refinement hypothesis

Phase129 may have selected an explore candidate but then transformed it into a staging/alignment pose before dispatch.

Evidence to compare:

- original_target
- raw_target
- refined_target
- candidate source
- candidate rank and family
- target clearance and path corridor clearance
- selected branch vs lower-ranked branches
- staging-specific rejection/selection reason fields

Phase131 should distinguish `original_target` evidence from the actual dispatched target.

### H6: goal_count/max_goals interaction hypothesis

`max_goals=1` might count a staging dispatch as the first budgeted goal in Phase129, while Phase124/125 expected only explore dispatches to count.

Evidence to compare:

- goal_count before dispatch
- max_goals
- dispatch seq
- goal event count
- whether staging consumes `goal_count`
- whether Phase124/125 runner ignored non-explore events or never saw them

No change to `max_goals` semantics is authorized in Phase130.

### H7: state-machine or active-edge hypothesis

A hidden active_edge/state machine state may have differed across phases even when `current_node_id=2` remained stable.

Evidence to compare:

- active_edge/state machine
- active goal kind
- current node and start node
- previous edge traversal fields
- post-ingress handoff state
- exploration state topic snapshots if present

If fields are missing, Phase131 should mark `active_edge/state machine` evidence as missing and may classify `FIRST_DISPATCH_KIND_CONTRACT_AMBIGUOUS` or `INSUFFICIENT_DISPATCH_KIND_EVIDENCE`.

## first dispatch kind contract

Phase130 defines the contract for future diagnosis and report wording:

- Phase124/125 first-goal smoke only allows goal_kind=explore.
- A first `goal_kind=explore` dispatch can be used for Phase125/127 first-goal timeout replay.
- A first `goal_kind=corridor_alignment_staging`, corridor alignment, staging, fallback, terminal, carry-over, branch, centerline, or exit dispatch is not a valid substitute for the Phase124/125 first explore goal.
- staging/corridor alignment must be classified separately and must not be mixed into first explore timeout replay.
- If the first dispatch is staging, the correct diagnostic path is dispatch-kind discrepancy diagnosis, not timeout-local-cost classification.
- If the contract is ambiguous because older runners filtered events differently, classify that ambiguity explicitly.

## Classification vocabulary for Phase131

Phase131 should emit exactly one of the following classifications:

### FIRST_DISPATCH_KIND_STABLE_EXPLORE

Use only when Phase124, Phase125, and Phase129 normalized first dispatches are all `goal_kind=explore`, or when Phase129 staging evidence is shown to be non-first/non-dispatch noise and the first budgeted current-algorithm dispatch remains explore.

This is unlikely given current Phase129 evidence, but it preserves a complete taxonomy.

### FIRST_DISPATCH_KIND_CHANGED_TO_STAGING_POST_INGRESS

Use when Phase124/125 first dispatch is explore, Phase129 first dispatch is `corridor_alignment_staging`, and artifacts show post-ingress context, corridor alignment, centerline alignment, staging, or candidate refinement fields directly explain the staging conversion.

Evidence should include at least:

- Phase129 first dispatch `goal_kind=corridor_alignment_staging`
- Phase124/125 first dispatch `goal_kind=explore`
- same-run ingress/handoff pass in all comparable artifacts
- a post-ingress/staging/refinement field or original/raw/refined target signature supporting staging

### FIRST_DISPATCH_KIND_CHANGED_BY_POSE_TOPOLOGY_DRIFT

Use when Phase129 first dispatch changed to staging and the most supported explanation is a changed pose/topology/candidate landscape rather than an explicit post-ingress staging flag.

Evidence should include one or more of:

- robot pose after ingress or pre-dispatch pose drift
- changed `last_open_direction_count`
- changed `last_candidate_count`
- changed `candidate_count` or `candidate_branch_count`
- changed local topology/topology state
- branch geometry differences sufficient to alter first dispatch kind

### FIRST_DISPATCH_KIND_CONTRACT_AMBIGUOUS

Use when artifacts prove Phase124/125 and Phase129 used different observation contracts, for example if Phase124/125 ignored staging events while Phase129 captured them as first dispatch, and existing artifacts cannot determine whether staging existed before explore in Phase124/125.

This classification should not be used to claim a timeout replay.

### INSUFFICIENT_DISPATCH_KIND_EVIDENCE

Use when required artifact fields are missing or contradictory, so the analyzer cannot decide whether the kind change was caused by post-ingress staging, pose/topology drift, contract ambiguity, or stable explore behavior.

Missing evidence examples:

- no first dispatch event in one or more artifacts
- missing goal_kind
- missing ingress pose/pre-dispatch pose
- missing candidate/topology evidence
- missing original/raw/refined target fields needed to identify staging
- missing runner event-filtering contract

## Phase131 allowed scope

Phase131 allowed scope is artifact replay/analyzer only.

Allowed:

- read existing Phase124/125/129 artifacts
- compare existing Phase124/125/129 artifacts
- read existing reports and logs
- optionally inspect existing source code to understand field semantics and runner event filters
- create an offline analyzer and focused static tests
- write a bounded offline artifact and report

Forbidden in Phase131 unless a later human instruction explicitly changes scope:

- do not rerun runtime
- do not launch Gazebo/RViz/Nav2
- do not start maze_explorer
- do not send any goal
- do not tune or repair
- do not change Nav2/MPPI/controller/goal checker/config
- do not change exploration strategy, branch scoring, centerline, fallback, or terminal acceptance
- do not claim autonomous exploration success or exit success

Phase131 should produce a bounded offline artifact with:

- normalized Phase124/125/129 dispatch-kind comparison matrix
- per-field missingness flags
- selected classification from the Phase130 vocabulary
- evidence snippets with source artifact path and JSON pointer where possible
- explicit statement that timeout classification remains separate from dispatch-kind classification

## Acceptance criteria for Phase130

Phase130 is complete when:

1. The design document exists at `doc/doc_proposal/phase130_first_dispatch_kind_discrepancy_design.md`.
2. The report exists at `doc/doc_report/phase130_first_dispatch_kind_discrepancy_design_report.md`.
3. Focused static tests exist and pass.
4. No runtime runner/analyzer for Phase130 is added.
5. No Gazebo/RViz/Nav2/maze_explorer process is started.
6. Protected config/source diff guard is empty for Nav2 config and exploration strategy paths.
7. Phase131 is not entered.

## Stop condition

After Phase130 documents and static verification are complete, stop and wait for human acceptance. Do not enter Phase131.
