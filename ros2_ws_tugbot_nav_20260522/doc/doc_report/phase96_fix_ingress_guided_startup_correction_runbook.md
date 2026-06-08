# Phase96-fix ingress-guided startup correction runbook

Status: READY_FOR_BOUNDED_RUNTIME_VALIDATION

Purpose: correct the Phase96 smoke workflow after the first Phase96 run stayed at the start pose with /maze/goal_events=0 and mode=WAIT_FOR_DISPATCH_ENTRY_READINESS blocked by map_sufficient=false. The correction is procedural only: send an explicit inner-ingress Nav2 goal before maze_explorer, then run the same bounded Phase88/92 refinement-chain smoke.

Scope
- Validation-only flow correction.
- explicit inner-ingress Nav2 goal before maze_explorer.
- max_goals:=2~3 only.
- Stop at dispatch observation, terminal outcome set, ingress failure, or bounded window expiry.
- No Phase88/92 refinement logic changed.
- No branch scoring/exploration order/centerline gate/directional readiness/fallback/terminal acceptance changed.
- No Nav2/MPPI/controller/inflation/robot_radius/clearance_radius_m/map threshold tuning.
- No autonomous exploration success claimed.
- No exit success claimed.
- Phase97 not entered.

Workflow
1. Inspect or cleanup only stale Phase96 maze_explorer/recorder workers. Do not kill unrelated ROS/Gazebo processes.
2. Launch or reuse the visible active scaled2x Gazebo/RViz/SLAM/Nav2 stack.
3. Wait for /navigate_to_pose, /bt_navigator, /controller_server, /map, and /local_costmap/costmap readiness.
4. Start read-only recorders.
5. Send explicit inner-ingress Nav2 goal:
   - target map pose: (2.0, 0.0, 0.0) unless overridden by PHASE96_FIX_INGRESS_* environment variables.
   - wait for succeeded / failed / timeout.
6. If ingress succeeds, start maze_explorer with max_goals 2 or 3 and the existing Phase88/92 parameters.
7. Wait only until maze_explorer dispatch is observed, max-goal terminal set is observed, explorer exits, or bounded smoke window expires.
8. Capture raw scan/map/local_costmap/odom/TF.
9. Analyze and classify.

Primary classification labels
- INGRESS_GUIDED_DISPATCH_OBSERVED
- INGRESS_GUIDED_DISPATCH_STILL_BLOCKED
- INGRESS_GUIDED_INGRESS_FAILED
- INGRESS_GUIDED_INSUFFICIENT_EVIDENCE

Required artifacts
- phase96_fix_initial_cleanup_summary.json
- phase96_fix_ingress_guided_startup_correction_preflight.txt
- phase96_fix_ingress_guided_startup_correction_ingress_result.json
- phase96_fix_ingress_guided_startup_correction_goal_events.jsonl
- phase96_fix_ingress_guided_startup_correction_explorer_state.jsonl
- phase96_fix_ingress_guided_startup_correction_nav2_feedback.jsonl
- phase96_fix_ingress_guided_startup_correction_local_costmap_samples.jsonl
- phase96_fix_ingress_guided_startup_correction_raw_capture.json
- phase96_fix_ingress_guided_startup_correction_analysis.json
- phase96_fix_ingress_guided_startup_correction_minimal_field_summary.md

Commands

From workspace root:

```bash
PYTHONDONTWRITEBYTECODE=1 pytest -q src/tugbot_maze/test/test_phase96_fix_ingress_guided_startup_correction.py
python3 -m py_compile tools/analyze_phase96_fix_ingress_guided_startup_correction.py tools/record_phase96_fix_smoke_evidence.py
bash -n tools/run_phase96_fix_ingress_guided_startup_correction.sh
git diff -- src/tugbot_navigation/config
```

Runtime examples

Reuse an already-held visible stack:

```bash
PHASE96_FIX_REUSE_VISIBLE_STACK=1 \
PHASE96_FIX_SKIP_BUILD=1 \
PHASE96_FIX_MAX_GOALS=3 \
PHASE96_FIX_CLEANUP_ON_EXIT=1 \
PYTHONDONTWRITEBYTECODE=1 \
tools/run_phase96_fix_ingress_guided_startup_correction.sh
```

Fresh visible stack:

```bash
PHASE96_FIX_REUSE_VISIBLE_STACK=0 \
PHASE96_FIX_MAX_GOALS=3 \
PYTHONDONTWRITEBYTECODE=1 \
tools/run_phase96_fix_ingress_guided_startup_correction.sh
```

Interpretation
- If ingress succeeds and any /maze/goal_events dispatch appears, the startup-flow fix is validated as INGRESS_GUIDED_DISPATCH_OBSERVED. This is not autonomous exploration success and not exit success.
- If ingress succeeds but no dispatch appears and the explorer remains readiness-blocked, classify INGRESS_GUIDED_DISPATCH_STILL_BLOCKED and report the blocker.
- If ingress fails or times out, do not start maze_explorer and classify INGRESS_GUIDED_INGRESS_FAILED.
