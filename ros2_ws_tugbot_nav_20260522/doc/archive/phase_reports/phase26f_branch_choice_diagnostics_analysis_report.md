# Phase 26F post-run branch-choice diagnostics analysis

Date: 2026-05-25
Workspace: `/home/hyh/Desktop/agent_playground/playground_hermes/ros2_gazebo_tugbot_navigate/ros2_ws_tugbot_nav_20260522`

## Goal

Keep Phase26F analysis-only.

Use Phase26E candidate branch diagnostics to build a post-run analyzer that:

1. summarizes by run and goal sequence:
   - chosen rank;
   - candidate count;
   - chosen `exit_progress_delta_m`;
2. flags route-divergence cases where chosen target moves away from exit but a rejected candidate moves toward exit;
3. separates context flags:
   - reverse;
   - backtrack;
   - near-exit;
   - local-cost constrained;
   - blacklisted/rejected context;
4. analyzes available matched baseline/candidate repeats first;
5. does not tune parameters or change branch-selection behavior.

## Files added / changed

Added:

- `tools/analyze_phase26f_branch_choice_diagnostics.py`
- `src/tugbot_maze/test/test_phase26f_branch_choice_diagnostics_analysis.py`

Generated analysis artifact:

- `log/phase26f_branch_choice_diagnostics_analysis.json`

## Analyzer behavior

The analyzer consumes only existing JSONL artifacts:

- `*_goal_events.jsonl`
- optionally `*_explorer_state.jsonl` for final mode

It does not launch ROS/Gazebo and does not modify any runtime behavior.

Classification logic:

- `chosen_moves_toward_exit`: chosen target has positive exit progress.
- `chosen_moves_away_no_better_rejected_candidate`: chosen target moves away from exit, but no rejected candidate moves toward exit in available candidate diagnostics.
- `chosen_moves_away_rejected_moves_toward_exit`: route-divergence case; chosen target moves away while a rejected candidate moves toward exit.
- `chosen_exit_neutral_or_unknown`: small or missing exit-progress signal.

Context flags:

- `reverse`: any candidate marked `is_reverse_candidate`.
- `backtrack`: dispatch goal is backtrack or candidate has `is_backtrack_context`.
- `near_exit`: dispatch/candidate near-exit flag.
- `local_cost_constrained`: dispatch or candidate local-cost fields exceed threshold.
- `blacklisted_or_rejected`: any candidate has a rejection reason.
- `blacklisted`: any rejection reason contains blacklist.

## TDD evidence

RED verified first:

```bash
python3 -m pytest -q src/tugbot_maze/test/test_phase26f_branch_choice_diagnostics_analysis.py -v
```

Initial result before implementation:

- 2 failed because `tools/analyze_phase26f_branch_choice_diagnostics.py` did not exist.

GREEN after implementation:

```bash
python3 -m pytest -q src/tugbot_maze/test/test_phase26f_branch_choice_diagnostics_analysis.py -v
```

Result:

- 2 passed

The synthetic tests verify:

- route-divergence case detection;
- matched baseline/candidate grouping;
- analysis-only decision guardrails;
- reverse/backtrack/near-exit/local-cost/blacklist context separation.

## Analyzer run

Command:

```bash
python3 tools/analyze_phase26f_branch_choice_diagnostics.py \
  --log-dir log \
  --baseline-runs phase26b_baseline_run1,phase26b_baseline_run2,phase26b_baseline_run3 \
  --candidate-runs phase26b_candidate_run1,phase26b_candidate_run2,phase26b_candidate_run3 \
  --extra-runs phase26e_branch_diagnostics_smoke \
  --output-json log/phase26f_branch_choice_diagnostics_analysis.json
```

## Matched baseline/candidate results

Important limitation: Phase26B baseline/candidate artifacts predate Phase26E candidate-branch diagnostics. Therefore they have dispatch-level chosen target geometry, but no `candidate_branches[]`. The analyzer reports this as artifact gaps.

Artifact gap counts:

- `phase26b_baseline_run1`: 11 dispatches missing candidate diagnostics
- `phase26b_baseline_run2`: 12 dispatches missing candidate diagnostics
- `phase26b_baseline_run3`: 10 dispatches missing candidate diagnostics
- `phase26b_candidate_run1`: 8 dispatches missing candidate diagnostics
- `phase26b_candidate_run2`: 12 dispatches missing candidate diagnostics
- `phase26b_candidate_run3`: 10 dispatches missing candidate diagnostics
- `phase26e_branch_diagnostics_smoke`: 0 dispatches missing candidate diagnostics

Matched baseline summary:

- runs: `phase26b_baseline_run1`, `phase26b_baseline_run2`, `phase26b_baseline_run3`
- final modes:
  - run1: `EXIT_REACHED`
  - run2: `FAILED_EXHAUSTED`
  - run3: `EXIT_REACHED`
- dispatch count: 33
- chosen moves toward exit: 27
- chosen moves away: 5
- route-divergence cases with rejected-better evidence: 0

Matched candidate summary:

- runs: `phase26b_candidate_run1`, `phase26b_candidate_run2`, `phase26b_candidate_run3`
- final modes:
  - run1: `EXIT_REACHED`
  - run2: `FAILED_EXHAUSTED`
  - run3: `EXIT_REACHED`
- dispatch count: 30
- chosen moves toward exit: 25
- chosen moves away: 4
- route-divergence cases with rejected-better evidence: 0

Interpretation:

- Phase26B matched runs show some chosen-away dispatches in both baseline and candidate profiles.
- Because those runs lack `candidate_branches[]`, they cannot prove whether a rejected branch would have moved toward exit.
- Therefore they cannot justify Phase27 branch-selection intervention.

## Phase26E diagnostics smoke result

`phase26e_branch_diagnostics_smoke` does contain candidate diagnostics.

Summary:

- final mode: `EXIT_REACHED`
- dispatch count: 8
- missing candidate diagnostics: 0
- chosen moves toward exit: 8
- chosen moves away: 0
- route-divergence cases: 0
- reverse context count: 8
- near-exit dispatch count: 1
- local-cost constrained count: 6
- blacklisted/rejected context count: 8

First three dispatch examples:

1. seq 1
   - chosen rank: 1
   - candidate count: 4
   - chosen exit progress delta: 0.365078
   - best rejected exit progress delta: 0.237057
   - classification: `chosen_moves_toward_exit`
   - contexts: reverse, local_cost_constrained, rejected_candidate_context
2. seq 2
   - chosen rank: 1
   - candidate count: 3
   - chosen exit progress delta: 0.096557
   - best rejected exit progress delta: -0.725262
   - classification: `chosen_moves_toward_exit`
   - contexts: reverse, local_cost_clear, rejected_candidate_context
3. seq 3
   - chosen rank: 1
   - candidate count: 3
   - chosen exit progress delta: 0.130642
   - best rejected exit progress delta: -0.668435
   - classification: `chosen_moves_toward_exit`
   - contexts: reverse, local_cost_clear, rejected_candidate_context

Interpretation:

- The only run with full Phase26E candidate diagnostics does not show the requested route-divergence pattern.
- All dispatches selected rank 1 and moved toward exit.
- Rejected candidates were present, but none creates a stable case for changing branch choice.

## Decision

Phase26F acceptance: PASS as analysis-only.

Decision output:

- `stable_route_divergence_evidence`: false
- recommendation: `analysis_only_collect_more_or_repeat`
- guardrails:
  - `do_not_enter_phase27`
  - `do_not_change_branch_selection`
  - `do_not_tune_nav2_or_controller_params_from_phase26f`
  - `require_matched_repeats_before_intervention`

Therefore: do not enter Phase27 branch-selection intervention yet.

## Next recommendation

Continue analysis-only with matched repeats that include Phase26E candidate diagnostics in both baseline and candidate profiles.

Suggested Phase26G scope:

1. Run at least matched baseline/candidate repeats with Phase26E diagnostics enabled:
   - canonical baseline profile;
   - candidate profile only if still under investigation;
   - same run budget and smoke wrapper semantics;
   - no parameter changes.
2. Feed those repeats into `tools/analyze_phase26f_branch_choice_diagnostics.py`.
3. Require stable evidence before intervention:
   - route-divergence case count repeats across runs;
   - chosen-away dispatches have rejected-toward-exit alternatives;
   - context is not dominated by backtrack/reverse/local-cost constrained special cases;
   - failed runs differ from exit-reaching runs in branch-choice evidence.
4. Only if those criteria are met, design Phase27 as a narrow, reversible branch-selection experiment with explicit rollback and gates.
