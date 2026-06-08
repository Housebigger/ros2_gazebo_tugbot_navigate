# Phase94 Applied single-step refinement execution outcome diagnosis report

Status: COMPLETE_DIAGNOSIS_ONLY_STOP_BEFORE_PHASE95

Scope: Phase94 diagnoses the Nav2 execution outcome after Phase93 observed an applied Phase88 single-step safety-first refinement in the Goal2-equivalent segment. This phase is read-only / diagnosis-only. It does not change navigation strategy or tune runtime configuration.

## Required cleanup completed

Before reading/analyzing Phase94 inputs, Phase93 held-scene processes were closed with a scoped cleanup that targeted only the known Phase93 process tree or workspace-related ROS/Gazebo/RViz/Nav2/maze_explorer commands.

Cleanup artifact:

```text
log/phase94_applied_single_step_refinement_execution_outcome_diagnosis/phase94_applied_single_step_refinement_execution_outcome_diagnosis_cleanup_summary.txt
```

Observed cleanup result:

```text
target_process_count_before=22
target_process_count_after=0
required_absence_process_count_after=1
residual_pid=873887 cmd=gz sim gui
required_absence_process_count_after_residual_cleanup=0
```

The residual process was the exact Phase93 `gz sim gui` process, then was killed separately. Final cleanup-summary required absence count was zero for:

```text
gz sim
rviz2
slam_toolbox
controller_server
bt_navigator
tugbot_maze_slam_nav.launch.py
maze_explorer
ros2 launch
```

A post-validation recheck also found:

```text
required_absence_process_count_current=0
```

## Required reading completed

Read before implementation/analysis:

1. `doc/doc_report/phase93_two_step_staging_bounded_goal2_validation_report.md`
2. `doc/doc_report/phase92_two_step_corridor_alignment_staging_goal_minimal_implementation_report.md`
3. `doc/doc_report/phase89_safety_first_refinement_bounded_goal2_validation_report.md`
4. `doc/doc_report/phase88_safety_first_multi_candidate_forward_search_minimal_implementation_report.md`
5. Phase93 log artifacts under `log/phase93_two_step_staging_bounded_goal2_validation/`

Also read for comparison context:

- `doc/doc_report/phase85_goal2_corridor_aligned_refinement_bounded_validation_report.md`
- Phase89/Phase85 analysis and runtime artifacts where available.

## New Phase94 files

Tooling:

```text
tools/analyze_phase94_applied_single_step_refinement_execution_outcome.py
```

Focused tests:

```text
src/tugbot_maze/test/test_phase94_applied_single_step_refinement_execution_outcome_diagnosis.py
```

Artifacts:

```text
log/phase94_applied_single_step_refinement_execution_outcome_diagnosis/phase94_applied_single_step_refinement_execution_outcome_diagnosis_analysis.json
log/phase94_applied_single_step_refinement_execution_outcome_diagnosis/phase94_applied_single_step_refinement_execution_outcome_diagnosis_minimal_field_summary.md
```

Report:

```text
doc/doc_report/phase94_applied_single_step_refinement_execution_outcome_diagnosis_report.md
```

## Analyzer inputs

Phase94 analyzer consumed:

```text
Phase93 goal_events:
log/phase93_two_step_staging_bounded_goal2_validation/phase93_two_step_staging_bounded_goal2_validation_goal_events.jsonl

Phase93 Nav2 feedback:
log/phase93_two_step_staging_bounded_goal2_validation/phase93_two_step_staging_bounded_goal2_validation_nav2_feedback.jsonl

Phase93 local costmap samples:
log/phase93_two_step_staging_bounded_goal2_validation/phase93_two_step_staging_bounded_goal2_validation_local_costmap_samples.jsonl

Phase93 raw capture:
log/phase93_two_step_staging_bounded_goal2_validation/phase93_two_step_staging_bounded_goal2_validation_raw_capture.json

Phase93 source analysis:
log/phase93_two_step_staging_bounded_goal2_validation/phase93_two_step_staging_bounded_goal2_validation_analysis.json

Comparison baselines:
log/phase89_safety_first_refinement_bounded_goal2_validation/
log/phase85_goal2_corridor_aligned_refinement_bounded_validation/
```

## Classification

Final Phase94 classification:

```text
APPLIED_REFINEMENT_HELD_BEFORE_TERMINAL_OUTCOME
```

Allowed Phase94 classifications:

```text
APPLIED_REFINEMENT_EXECUTION_IMPROVED
APPLIED_REFINEMENT_STILL_RECOVERY_DOMINANT
APPLIED_REFINEMENT_HELD_BEFORE_TERMINAL_OUTCOME
APPLIED_REFINEMENT_LOCAL_COST_RISK_PERSISTS
APPLIED_REFINEMENT_DIAGNOSIS_INSUFFICIENT_EVIDENCE
```

Evidence gap:

```text
missing_terminal_outcome_for_applied_refinement
```

Conservative decision rule: because Phase93 intentionally held/stopped the bounded window before a terminal outcome was captured, Phase94 must classify held-before-terminal rather than success, even though movement toward the selected target was observed.

## Selected applied single-step refinement extraction

Goal2-equivalent validation goal sequence:

```text
validation_goal_sequence=1
target_match_fallback=True
```

Extracted selected refinement fields:

```text
refinement_applied=True
selected_refined_target=[2.0854286327375418, 1.0229487083032263]
selected_candidate_index=10
selected_candidate_target=[2.0854286327375418, 1.0229487083032263]
selected_candidate_yaw=1.5653090147394948
candidate_count=63
hard_safety_pass_candidate_count=6
rejected_candidate_summary_count=62
branch_scoring_changed=False
fallback_terminal_acceptance_used=False
```

Selection priority trace:

```text
hard safety pass
no footprint/front-wedge lethal regression
safety_floor_ok
forward_progress_ok
clearance better
balance error smaller
```

Target shift:

```text
target_shift_from_original_m=0.0
nav2_target_shift_from_original_m=0.0
```

Interpretation: Phase93 observed Phase88 single-step refinement as applied with 6 hard-safety-pass candidates, but in this specific dispatch the selected candidate target equals the original/refined dispatch target. So the important change relative to Phase89 is not a geometric shift; it is that the Phase88 safety-first search found hard-safe candidates where Phase89 had none.

## Nav2 execution analysis

Feedback sample count:

```text
feedback_sample_count=14455
```

Distance remaining:

```text
distance_remaining_first_nonzero=1.0593633651733398
distance_remaining_last=0.3290790319442749
distance_remaining_progress_delta=0.7302843332290649
moved_toward_selected_target=True
```

Recoveries:

```text
recoveries_max=13
recoveries_last=13
recovery_dominant=True
```

Recovery timeline excerpt:

```text
recovery 1  at elapsed_sec=15.504855155944824  distance_remaining=1.0916498899459839
recovery 3  at elapsed_sec=36.17531991004944   distance_remaining=0.3290790319442749
recovery 4  at elapsed_sec=46.89474129676819   distance_remaining=0.3290790319442749
...
recovery 11 at elapsed_sec=114.96503567695618  distance_remaining=0.3290790319442749
recovery 12 at elapsed_sec=125.16426038742065  distance_remaining=0.3290790319442749
recovery 13 at elapsed_sec=140.2746376991272   distance_remaining=0.3290790319442749
```

Terminal outcome:

```text
terminal_observed=False
terminal_event=None
terminal_reason=None
terminal_succeeded=False
terminal_timed_out=False
held_before_terminal_outcome=True
```

Interpretation:

- The robot did move toward the selected target.
- Distance remaining dropped from about 1.059 m to 0.329 m.
- After reaching the 0.329 m plateau, recoveries continued accumulating up to 13.
- No terminal outcome was captured before the bounded scene hold: no terminal succeeded/timeout/cancel/failure event was observed.
- Therefore Phase94 cannot declare execution success or terminal timeout; it can only report held-before-terminal with recovery-dominant symptoms present inside the held window.

## Local cost / footprint / front-wedge evidence

Local-cost summary from Phase93 samples:

```text
sample_count=241
local_cost_risk_present=True
front_wedge_lethal_count_max=129.0
front_wedge_lethal_count_last=58.0
robot_footprint_lethal_count_max=52.0
robot_footprint_lethal_count_last=52.0
target_footprint_lethal_count_max=0.0
front_wedge_clearance_min_m=0.05
```

Interpretation:

- Local cost risk persisted around execution-time robot footprint/front wedge.
- Target footprint remained non-lethal in the sampled evidence, while robot/front-wedge costs became lethal during execution.
- This matches the Phase75/93 theme: dispatch target eligibility alone does not prove execution-time local-cost / footprint / front-wedge viability.
- Classification remains `APPLIED_REFINEMENT_HELD_BEFORE_TERMINAL_OUTCOME` because terminal outcome was missing; local-cost risk is a major supporting symptom, not a success/failure terminal result.

## Comparison with Phase89 and Phase85

Phase89 baseline:

```text
classification=PHASE88_REFINEMENT_STILL_REJECTED
refinement_applied=False
hard_safety_pass_candidate_count=0
recoveries_max=4
distance_remaining_last=0.33783894777297974
front_wedge_lethal_count_max=142.0
robot_footprint_lethal_count_max=52.0
terminal_observed=True
terminal_timed_out=True
```

Phase85 baseline:

```text
classification=REFINEMENT_REJECTED_OR_NOT_TRIGGERED
refinement_applied=False
recoveries_max=4
distance_remaining_last=0.3802814185619354
front_wedge_lethal_count_max=148.0
robot_footprint_lethal_count_max=54.0
terminal_observed=True
terminal_timed_out=True
```

Phase93/94 vs Phase89 deltas:

```text
refinement_applied_changed_false_to_true=True
hard_safety_pass_candidate_count_delta=6.0
recoveries_max_delta=9.0
distance_remaining_last_delta=-0.008759915828704834
front_wedge_lethal_count_max_delta=-13.0
robot_footprint_lethal_count_max_delta=0.0
```

Phase93/94 vs Phase85 deltas:

```text
refinement_applied_changed_false_to_true=True
recoveries_max_delta=9.0
distance_remaining_last_delta=-0.05120238661766052
front_wedge_lethal_count_max_delta=-19.0
robot_footprint_lethal_count_max_delta=-2.0
```

Comparison interpretation:

- Refinement status improved from rejected/not-applied to applied.
- Hard-safety-pass candidate count improved from Phase89 `0` to Phase93 `6`.
- Last distance remaining was slightly better than Phase89 and Phase85, but still near 0.329 m.
- Recoveries worsened from 4 to 13 inside the longer held Phase93 feedback window.
- Front-wedge lethal max was lower than Phase89/85 but still high/lethal; robot-footprint lethal max stayed effectively the same as Phase89.
- Because no terminal outcome exists, these deltas are diagnostic only and do not establish improved execution success.

## Guardrails preserved

- No maze_explorer strategy changed.
- No Phase88/92 logic changed.
- No branch scoring changed.
- No exploration order changed.
- No centerline gate changed.
- No directional readiness changed.
- No fallback/terminal acceptance changed.
- No Nav2/MPPI/controller tuning.
- No inflation/robot_radius/clearance_radius_m/map threshold tuning.
- Timeout was not treated as success.
- No autonomous exploration success claimed.
- No exit success claimed.
- Phase95 not entered.

## TDD and validation

Focused RED observed before analyzer/report existed:

```text
PYTHONDONTWRITEBYTECODE=1 pytest -q src/tugbot_maze/test/test_phase94_applied_single_step_refinement_execution_outcome_diagnosis.py
7 failed in 0.06s
```

Analyzer run:

```text
PYTHONDONTWRITEBYTECODE=1 python3 tools/analyze_phase94_applied_single_step_refinement_execution_outcome.py \
  --artifact-dir log/phase93_two_step_staging_bounded_goal2_validation \
  --phase89-dir log/phase89_safety_first_refinement_bounded_goal2_validation \
  --phase85-dir log/phase85_goal2_corridor_aligned_refinement_bounded_validation \
  --output-json log/phase94_applied_single_step_refinement_execution_outcome_diagnosis/phase94_applied_single_step_refinement_execution_outcome_diagnosis_analysis.json \
  --minimal-summary log/phase94_applied_single_step_refinement_execution_outcome_diagnosis/phase94_applied_single_step_refinement_execution_outcome_diagnosis_minimal_field_summary.md
```

Observed:

```text
classification=APPLIED_REFINEMENT_HELD_BEFORE_TERMINAL_OUTCOME
evidence_gaps=["missing_terminal_outcome_for_applied_refinement"]
```

Final GREEN/static/guard validation:

```text
PYTHONDONTWRITEBYTECODE=1 pytest -q src/tugbot_maze/test/test_phase94_applied_single_step_refinement_execution_outcome_diagnosis.py
.......                                                                  [100%]
7 passed in 0.06s

PYTHONDONTWRITEBYTECODE=1 python3 -m py_compile \
  tools/analyze_phase94_applied_single_step_refinement_execution_outcome.py \
  src/tugbot_maze/test/test_phase94_applied_single_step_refinement_execution_outcome_diagnosis.py
# exit 0

nav2_config_diff_lines=0
remaining_pycache_count=0
```

## Conclusion

Phase94 diagnoses the Phase93 applied single-step refinement execution as:

```text
APPLIED_REFINEMENT_HELD_BEFORE_TERMINAL_OUTCOME
```

Reason:

- Phase88 single-step refinement was applied.
- 6 hard-safety-pass candidates were available.
- The robot moved toward the selected target.
- However, the bounded Phase93 window did not capture terminal succeeded/timeout/cancel/failure.
- Recoveries accumulated to 13 and local-cost risk persisted around the robot footprint/front wedge.

Therefore Phase94 cannot claim the applied refinement improved execution success. It also cannot label the run terminal timeout because terminal outcome was not captured. The correct conservative result is held-before-terminal with recovery-dominant/local-cost-risk symptoms.

## Stop condition

Phase94 stops here. Phase95 not entered.
