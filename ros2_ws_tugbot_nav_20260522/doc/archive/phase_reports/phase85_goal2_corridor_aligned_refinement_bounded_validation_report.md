# Phase85 Goal2 corridor-aligned refinement bounded validation report

Status: COMPLETE_BOUNDED_VALIDATION_STOPPED

## Scope

Phase85 validates whether the Phase84 corridor-aligned intermediate-goal refinement changes the old Goal2 near-goal lateral-residual target into a corridor-aligned Nav2-executable goal.

This phase is validation only.

No runtime navigation strategy, branch scoring, centerline gate, directional readiness override, fallback, terminal acceptance, Nav2/MPPI/controller, inflation, robot radius, clearance radius, or map threshold was tuned in Phase85.

No autonomous exploration success or exit success is claimed. Phase86 is not entered.

## Required inputs read

The following inputs were read before the bounded run:

- `doc/doc_report/phase84_corridor_aligned_intermediate_goal_refinement_minimal_implementation_report.md`
- `doc/doc_proposal/phase83_corridor_aligned_intermediate_goal_design_review.md`
- `doc/doc_report/phase79_goal2_timeout_bounded_reproduction_handoff_report.md`
- `doc/doc_report/phase80_goal2_near_goal_forward_open_corridor_diagnostic_classification_report.md`
- `doc/doc_report/phase81_goal2_forward_open_machine_evidence_capture_report.md`
- `doc/doc_report/phase82_goal2_local_cost_scan_mismatch_root_cause_report.md`
- `doc/doc_proposal/obstacle_reproduction_handoff_workflow.md`
- Phase79/81 recorder and analyzer artifacts/templates used to shape Phase85 capture.

## New Phase85 files

Tools:

- `tools/run_phase85_goal2_corridor_aligned_refinement_bounded_validation.sh`
- `tools/analyze_phase85_goal2_corridor_aligned_refinement_validation.py`
- `tools/record_phase85_goal2_corridor_aligned_refinement_evidence.py`

Focused tests:

- `src/tugbot_maze/test/test_phase85_goal2_corridor_aligned_refinement_bounded_validation.py`

Primary artifacts:

- `log/phase85_goal2_corridor_aligned_refinement_bounded_validation/phase85_process_cleanup_summary.md`
- `log/phase85_goal2_corridor_aligned_refinement_bounded_validation/phase85_goal2_corridor_aligned_refinement_bounded_validation_goal_events.jsonl`
- `log/phase85_goal2_corridor_aligned_refinement_bounded_validation/phase85_goal2_corridor_aligned_refinement_bounded_validation_nav2_feedback.jsonl`
- `log/phase85_goal2_corridor_aligned_refinement_bounded_validation/phase85_goal2_corridor_aligned_refinement_bounded_validation_local_costmap_samples.jsonl`
- `log/phase85_goal2_corridor_aligned_refinement_bounded_validation/phase85_goal2_corridor_aligned_refinement_bounded_validation_runtime_timeline.jsonl`
- `log/phase85_goal2_corridor_aligned_refinement_bounded_validation/phase85_goal2_corridor_aligned_refinement_bounded_validation_raw_capture.json`
- `log/phase85_goal2_corridor_aligned_refinement_bounded_validation/phase85_goal2_corridor_aligned_refinement_bounded_validation_analysis.json`
- `log/phase85_goal2_corridor_aligned_refinement_bounded_validation/phase85_goal2_corridor_aligned_refinement_bounded_validation_minimal_field_summary.md`
- `log/phase85_goal2_corridor_aligned_refinement_bounded_validation/phase85_goal2_corridor_aligned_refinement_bounded_validation_SCENE_HELD_WAITING_FOR_USER_SCREENSHOT.txt`

## Cleanup

Phase85 cleanup artifact:

- `log/phase85_goal2_corridor_aligned_refinement_bounded_validation/phase85_process_cleanup_summary.md`

Recorded cleanup summary:

- pre-cleanup scoped matches: 5
- mid-after-SIGTERM scoped matches: 0
- post-cleanup scoped matches: 0
- read-only reconfirmation remaining matches: 0

Guard: cleanup targeted only matching Gazebo/RViz/SLAM/Nav2/ros2 launch/maze_explorer processes scoped by workspace cwd/cmdline or active Tugbot maze launch/world.

## Bounded visible reproduction

Runbook:

```bash
PHASE85_HOLD_AFTER_TRIGGER_SEC=600 PHASE85_CLEANUP_ON_EXIT=0 PYTHONDONTWRITEBYTECODE=1 \
  tools/run_phase85_goal2_corridor_aligned_refinement_bounded_validation.sh
```

Run mode:

- visible Gazebo: enabled
- RViz: enabled
- bounded validation: yes
- long autonomous exploration: not run
- Phase86: not entered

The script sent an explicit inner-ingress Nav2 goal first, then started `maze_explorer` with bounded dispatch. Because the inner ingress goal was sent before `maze_explorer`, the old Phase79/80/81/82 Goal2 target appeared as `maze_explorer` validation goal sequence 1. The Phase85 analyzer therefore used target-match fallback to identify the old Goal2 segment.

Analyzer fields:

- `validation_goal_sequence`: 1
- `used_target_match_fallback`: true

The scene was held after capture for visual screenshot review. Maze explorer was stopped after analysis; Gazebo/RViz/Nav2/SLAM scene processes were intentionally left available for screenshots.

## Captured evidence

Captured artifacts cover:

- `/maze/goal_events`
- Nav2 feedback
- local costmap samples
- runtime timeline
- controller dynamics
- global plan samples
- collision monitor state
- raw `/scan`
- raw `/local_costmap/costmap`
- `/local_costmap/published_footprint`
- `/odom`
- TF lookup samples

Raw capture summary from `phase85_goal2_corridor_aligned_refinement_bounded_validation_analysis.json`:

- scan_available: true
- local_costmap_available: true
- odom_available: true
- tf_available: true
- footprint_available: true

## Goal2 dispatch refinement context

Classification:

```text
REFINEMENT_REJECTED_OR_NOT_TRIGGERED
```

Goal dispatch / refinement fields:

```text
original_target: [2.084224301395533, 1.023027384249055]
centerline_projected_target: null
nav2_goal_target: [2.084224301395533, 1.023027384249055]
corridor_heading_yaw: 1.5660394713676706
refinement_applied: false
refinement_reject_reason: lethal_cost_regression
forward_executability_check.passed: false
forward_executability_check.reason: lethal_cost_regression
branch_scoring_changed: false
fallback_terminal_acceptance_used: false
```

Forward executability evidence:

```text
checked: true
same_corridor: true
two_side_wall_evidence: true
target_has_clearance: true
occupancy_free: true
safety_floor_ok: false
footprint_lethal_not_increased: false
front_wedge_lethal_not_increased: false
forward_progress_not_lowered: false
forward_progress_not_obviously_lowered: false
local_cost_sample_count: 112
front_wedge_sample_count: 184
```

Candidate evidence:

```text
candidate_count: 21
two_side_wall_candidate_count: 21
strict_eligible_candidate_count: 0
eligible_candidate_count: 0
```

Original target metrics from dispatch context:

```text
local_cost_max_radius: 54
front_wedge_cost_max: 63
footprint_lethal_count: 0
front_wedge_lethal_count: 0
min_clearance_m: 0.42720019363165534
left_wall_clearance_m: 0.6000000089406967
right_wall_clearance_m: 0.5500000081956387
```

Interpretation:

- Phase84 refinement logic was present and evaluated the old Goal2 segment.
- It did not apply a refined target because no candidate passed the forward executability check.
- Reject reason was `lethal_cost_regression`.
- Nav2 received the original target, not a centerline-projected target.
- Therefore Phase85 cannot claim the old near-goal lateral residual target was improved by Phase84 refinement in this bounded run.

## Goal2 outcome

Observed outcome:

```text
event: timeout
result_reason: goal_timeout
timed_out: true
succeeded: false
```

Nav2 feedback summary:

```text
sample_count: 4585
recoveries_max: 4
recoveries_last: 4
distance_remaining_last_m: 0.3802814185619354
distance_remaining_min_m: 0.0
```

Note: `distance_remaining_min_m` includes early Nav2 feedback samples that can be zero before the active goal distance stabilizes. The terminal and last-distance evidence are the safer outcome signals for this run.

## Phase85 classification decision

Allowed classifications:

- `REFINEMENT_APPLIED_AND_GOAL2_IMPROVED`
- `REFINEMENT_APPLIED_BUT_GOAL2_STILL_TIMEOUT`
- `REFINEMENT_REJECTED_OR_NOT_TRIGGERED`
- `INSUFFICIENT_VALIDATION_EVIDENCE`

Selected classification:

```text
REFINEMENT_REJECTED_OR_NOT_TRIGGERED
```

Rationale:

- Goal2-equivalent dispatch was observed.
- Required refinement context was present either top-level or nested under `centerline_target_refinement`.
- `refinement_applied=false`.
- `refinement_reject_reason=lethal_cost_regression`.
- `branch_scoring_changed=false`.
- `fallback_terminal_acceptance_used=false`.
- The goal timed out after four recoveries, but because refinement was rejected, this is not evidence for `REFINEMENT_APPLIED_BUT_GOAL2_STILL_TIMEOUT`.

## Screenshot checklist for held scene

If the visible scene is still open, capture:

1. Gazebo wide view: robot stopped near old Goal2 corridor and nearest wall.
2. RViz local costmap: robot footprint and front wedge around the attempted approach.
3. RViz goal/tolerance view: dispatch target near `[2.084224301395533, 1.023027384249055]`; no refined projected target was dispatched.
4. Recovery/timeout pose: show local-cost obstruction or footprint/front-wedge interaction if visible.

Use the minimal field summary:

- `log/phase85_goal2_corridor_aligned_refinement_bounded_validation/phase85_goal2_corridor_aligned_refinement_bounded_validation_minimal_field_summary.md`

## Guardrails

Confirmed guardrails:

- No maze_explorer strategy changed.
- No branch scoring change accepted.
- No centerline gate tuning.
- No directional readiness override tuning.
- No fallback or terminal acceptance change.
- No Nav2/MPPI/controller tuning.
- No inflation, robot_radius, clearance_radius_m, or map threshold tuning.
- No autonomous exploration success claimed.
- No exit success claimed.
- Phase86 not entered.

Guard diff artifact:

- `log/phase85_goal2_corridor_aligned_refinement_bounded_validation/phase85_goal2_corridor_aligned_refinement_bounded_validation_nav2_config_diff.txt`

The recorded Nav2 config diff was empty.

## Validation commands

Focused tests were introduced in RED first: before Phase85 analyzer/report artifacts existed, the focused test failed as expected.

Final validation is recorded in the assistant final response after this report is written.

Expected final commands:

```bash
PYTHONDONTWRITEBYTECODE=1 pytest -q src/tugbot_maze/test/test_phase85_goal2_corridor_aligned_refinement_bounded_validation.py
PYTHONDONTWRITEBYTECODE=1 python3 -m py_compile tools/analyze_phase85_goal2_corridor_aligned_refinement_validation.py tools/record_phase85_goal2_corridor_aligned_refinement_evidence.py src/tugbot_maze/test/test_phase85_goal2_corridor_aligned_refinement_bounded_validation.py
git diff -- src/tugbot_navigation/config src/tugbot_maze/tugbot_maze/maze_explorer.py src/tugbot_maze/tugbot_maze/maze_perception.py | cat
```

## Stop condition

Phase85 is complete after final focused/static validation. Stop here. Do not enter Phase86.
