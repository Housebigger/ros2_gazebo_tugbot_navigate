# Phase73 Local Costmap Sufficiency Gate Diagnosis / Threshold Semantics Review

Run id: `phase73_local_costmap_sufficiency_gate_diagnosis`

## Scope

Phase73 is a read-only semantic review of the Phase72 post-success readiness blocker. It reuses Phase72 artifacts and does not run a new autonomous navigation experiment.

Accepted Phase72 premise:

- Phase72 classification: `MULTIGOAL_NO_REDISPATCH_AFTER_SUCCESS`
- Phase72 sub-conclusion: `POST_SUCCESS_REDISPATCH_BLOCKED_BY_LOCAL_COSTMAP_SUFFICIENCY`
- Goal1 succeeded with `max_goals=3`; topology/candidate evidence remained available; Goal2 did not dispatch because `local_costmap_sufficient=false`.

Phase73 question: is the low post-success `free_ratio` evidence of a truly blocked local costmap, or is the full circular window denominator too strict for a narrow maze corridor where walls/inflation naturally occupy much of the window?

## Guardrails

- read-only Phase72 artifact analysis
- no Nav2/MPPI/controller tuning
- no inflation/robot_radius/clearance_radius_m/map threshold tuning
- no branch scoring change
- no centerline gate change
- no fallback/terminal acceptance change
- no autonomous exploration success claim
- no exit success claim
- 不进入 Phase74
- 不宣称 autonomous exploration success
- 不宣称 exit success

No Nav2/MPPI/controller tuning, no inflation/robot_radius/clearance_radius_m/map threshold tuning, no branch scoring change, no centerline gate change, and no fallback/terminal acceptance change were made.

## Classification

Overall classification: `LOCAL_DIRECTION_TRAVERSABLE_GATE_SHOULD_REPLACE_GLOBAL_RATIO`

Interpretation: the full-window local-costmap free-ratio gate appears too global for this post-success narrow-corridor state. The artifacts still show candidate/open-direction availability plus at least one non-reverse direction corridor slice with acceptable path-local-cost and clearance in both replays. This supports, for a later explicitly authorized phase, evaluating a replacement semantics based on robot-nearby footprint plus candidate-direction corridor traversability rather than tuning the existing 0.50 full-window threshold in Phase73.

Important caveat: selected target/front-wedge/target-footprint local-cost risk is still high in the Phase72 samples. Phase73 therefore does not claim the robot can safely navigate to the next goal, and it does not claim autonomous or exit success. It only diagnoses that the readiness blocker is caused by full-window ratio semantics before a candidate-direction gate can be evaluated.

## Aggregate metrics

- replay_count: 2
- goal1_success_replay_count: 2
- post_success_local_costmap_blocked_replay_count: 2
- direction_traversable_replay_count: 2
- full_window_free_ratio_values: [0.431637519872814, 0.4350597609561753]
- full_window_free_ratio_margin_values_vs_0.50: [-0.06836248012718599, -0.06494023904382468]

## Replay evidence

| Replay | classification | full-window free_ratio | margin vs 0.50 | occupied_ratio | unknown_ratio | candidate_count/open_direction_count | direction corridor traversable | non-reverse traversable | robot footprint lethal | front wedge lethal | target footprint lethal |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| replay_01 | `LOCAL_DIRECTION_TRAVERSABLE_GATE_SHOULD_REPLACE_GLOBAL_RATIO` | 0.431638 | -0.068362 | 0.568362 | 0.000000 | 4/4 | 2 | 1 | 22 | 197 | 77 |
| replay_02 | `LOCAL_DIRECTION_TRAVERSABLE_GATE_SHOULD_REPLACE_GLOBAL_RATIO` | 0.435060 | -0.064940 | 0.564940 | 0.000000 | 4/4 | 2 | 1 | 15 | 187 | 88 |

### Full-window distribution semantics

- replay_01: sample_count=1258, free_count=543, occupied_count=715, unknown_count=0, known_ratio=1.0, free_ratio=0.431638, min_free_ratio=0.5, margin=-0.068362, radius_m=1.0, sample_age_sec=0.123.
- replay_02: sample_count=1255, free_count=546, occupied_count=709, unknown_count=0, known_ratio=1.0, free_ratio=0.435060, min_free_ratio=0.5, margin=-0.064940, radius_m=1.0, sample_age_sec=0.159.

Phase72 gate payload records free/occupied/unknown counts, not raw per-cell bins. Therefore occupied currently combines lethal wall/obstacle cells and inflation-layer cells. The analyzer reports `inflated_count_estimate` conservatively as occupied for the full-window distribution and uses footprint/wedge/candidate summaries for more local semantics.

### Candidate direction corridor slices

#### replay_01

| rank | reverse | path max | path mean | min clearance m | target max radius | direction corridor traversable | target region traversable | note |
|---:|---:|---:|---:|---:|---:|---:|---:|---|
| 1 | False | 99.0 | 33.94117647058823 | 0.3500000052154064 | 99.0 | False | False | rejection=None |
| 2 | False | 53.0 | 14.80952380952381 | 0.4609772297337403 | 99.0 | True | False | rejection=lower_rank_not_selected |
| 4 | True | 0.0 | 0.0 | 0.7433034484420097 | 54.0 | True | True | rejection=lower_rank_not_selected |
| 3 | False | 99.0 | 45.38461538461539 | 0.3500000052154064 | 99.0 | False | False | rejection=lower_rank_not_selected |

Topology/candidate counts: raw_open_direction_count=4, filtered_open_direction_count=4, candidate_after_filter_count=4, result_counts={'inflated': 1, 'lethal_or_obstacle': 2, 'out_of_bounds': 1}.

#### replay_02

| rank | reverse | path max | path mean | min clearance m | target max radius | direction corridor traversable | target region traversable | note |
|---:|---:|---:|---:|---:|---:|---:|---:|---|
| 1 | False | 99.0 | 34.8235294117647 | 0.3500000052154064 | 99.0 | False | False | rejection=None |
| 2 | False | 51.0 | 11.157894736842104 | 0.47169906363169384 | 99.0 | True | False | rejection=lower_rank_not_selected |
| 4 | True | 0.0 | 0.0 | 0.6670832131466311 | 63.0 | True | True | rejection=lower_rank_not_selected |
| 3 | False | 99.0 | 45.38461538461539 | 0.3500000052154064 | 99.0 | False | False | rejection=lower_rank_not_selected |

Topology/candidate counts: raw_open_direction_count=4, filtered_open_direction_count=4, candidate_after_filter_count=4, result_counts={'clear': 1, 'inflated': 1, 'lethal_or_obstacle': 2}.

## Threshold semantics review

- Current gate semantics: local_costmap_sufficient requires full circular near-robot window known_ratio>=0.95 and free_ratio>=0.50 before candidate formation/dispatch
- Review question: Should the gate use robot-nearby footprint plus candidate-direction corridor slice traversability instead of a global full-window free_ratio?
- replace_global_ratio_with_directional_gate_candidate: True
- do_not_change_threshold_in_phase73: True

Phase73 recommendation is not to lower the full-window threshold immediately. The evidence points to a semantic mismatch: in a narrow corridor, a 1m circular local-costmap window can contain enough walls/inflation to fail `free_ratio>=0.50` even when topology has open directions and at least one non-reverse candidate-direction corridor slice remains locally traversable.

## Files

- Analyzer: `tools/analyze_phase73_local_costmap_sufficiency_gate_diagnosis.py`
- Tests: `src/tugbot_maze/test/test_phase73_local_costmap_sufficiency_gate_diagnosis.py`
- JSON output: `log/phase72_multigoal_bounded_rerun_from_inner_ingress/phase73_local_costmap_sufficiency_gate_diagnosis.json`
- Report: `doc/doc_report/phase73_local_costmap_sufficiency_gate_diagnosis_report.md`

## Verification

Completed verification:

- `python3 -m py_compile tools/analyze_phase73_local_costmap_sufficiency_gate_diagnosis.py src/tugbot_maze/test/test_phase73_local_costmap_sufficiency_gate_diagnosis.py` passed.
- Focused Phase73 pytest passed: `4 passed in 0.01s`.
- Analyzer execution passed and wrote `log/phase72_multigoal_bounded_rerun_from_inner_ingress/phase73_local_costmap_sufficiency_gate_diagnosis.json`.
- `colcon build --symlink-install --packages-select tugbot_maze tugbot_bringup` passed: `Summary: 2 packages finished [1.04s]`.
- Nav2 config diff remained empty: `nav2_config_diff_bytes=0`.
- Cleanup/process check remained empty: `cleanup_processes_after_bytes=0`.

## Stop condition

Phase73 completes after analyzer/tests/report verification. Do not enter Phase74 from this phase.
