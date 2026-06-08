# Phase66: Bounded Autonomous Rerun From Inner Ingress

Status: COMPLETED - waiting for human acceptance

Classification: `INNER_INGRESS_TIMEOUT_REMAINS`

Secondary observed risk: `LOCAL_COST_RISK_REMAINS`

Run id: `phase66_bounded_autonomous_rerun_from_inner_ingress`

Artifact directory: `log/phase66_bounded_autonomous_rerun_from_inner_ingress/`

## Scope

Phase65 已人工验收通过，结论为
`INNER_INGRESS_FIX_IMPROVES_FIRST_DISPATCH`：使用 Phase65 inner ingress
waypoint map `(2.0, 0.0, 0.0)` 后，robot pose inside maze 为 true，first
dispatch target local cost 从 49 降到 0，target footprint lethal 从 8 降到 0。

Phase66 只做 active clean scaled2x world 上的 bounded autonomous smoke：先发送
inner ingress goal，再启动 `maze_explorer`，限制 `max_goals=4`（允许范围 3..5），
采集是否能连续产生多个 dispatch/outcome，以及 timeout / no-candidate /
local-cost / footprint / front-wedge 风险是否仍复现。

This phase does not claim autonomous exploration success and does not claim exit
success.

## Guardrails

Held:

- bounded runtime only;
- `max_goals=4`，且 wrapper 限制 `PHASE66_MAX_GOALS` 只能为 3..5；
- no Nav2/MPPI/controller parameter edits;
- no inflation/robot_radius/clearance_radius_m/map threshold tuning;
- no branch scoring change;
- no target projection integration;
- no fallback/terminal acceptance change;
- no old scaffold world/map;
- 不宣称 autonomous exploration success；
- 不宣称 exit success。

## Implemented artifacts

- Wrapper: `tools/run_phase66_bounded_autonomous_rerun_from_inner_ingress.sh`
- Analyzer/recorder: `tools/analyze_phase66_bounded_autonomous_rerun_from_inner_ingress.py`
- Focused tests: `src/tugbot_maze/test/test_phase66_bounded_autonomous_rerun_from_inner_ingress.py`
- Report: `doc/doc_report/phase66_bounded_autonomous_rerun_from_inner_ingress_report.md`
- Runtime summary: `log/phase66_bounded_autonomous_rerun_from_inner_ingress/phase66_bounded_autonomous_rerun_from_inner_ingress.json`

## Runtime protocol

1. Start Gazebo + SLAM + Nav2 with the active clean scaled2x world:
   `src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean_scaled2x.sdf`.
2. Send a single Nav2 goal to Phase65 inner ingress `(2.0, 0.0, 0.0)`.
3. After ingress success, start recorders for:
   - `/maze/goal_events`;
   - `/maze/explorer_state`;
   - Nav2 feedback/recoveries;
   - cmd_vel / cmd_vel_smoothed;
   - odom robot progress;
   - local costmap target evidence;
   - target footprint / robot footprint / front wedge;
   - collision monitor and plan samples.
4. Start `maze_explorer` with `max_goals=4` and existing Phase59/61 topology guards enabled.
5. Stop after bounded timeout; cleanup all ROS/Gazebo/Nav2/explorer processes.

Runtime executed one bounded replay.

## Evidence summary

From `phase66_bounded_autonomous_rerun_from_inner_ingress.json` / `replay_01`:

- Inner ingress Nav2 goal success: `true`.
- Max goals configured: `4`.
- Dispatch count: `1`.
- Goal event count: `3`.
- Outcome count: `2` (`timeout`, then `timeout_cancel_result`).
- Timeout count: `2`.
- No-candidate observed: `false`.
- Local-cost risk observed: `true`.
- Exit reached by existing state: `false`.
- Final explorer mode: `WAIT_FOR_DISPATCH_ENTRY_READINESS`.
- Final goal count: `1`.
- Explorer state samples: `253`.
- Runtime timeline samples: `155`.

Goal 1 details:

- Dispatch target: `(2.0836008737, 1.0229248117)`.
- Dispatch pose: `(1.8531101281, 0.0241760857, -0.0054941657)`.
- Dispatch event had `last_open_direction_count=4`, `last_candidate_count=4`.
- Outcome event: `timeout`.
- Timeout reason: `goal_timeout`.
- Cancel result after timeout: `timeout_cancel_result`, result status `5`, reason `goal_canceled_after_timeout`.
- Timeout-local-cost fields from goal event:
  - `timeout_front_wedge_cost_max=99`;
  - `timeout_footprint_lethal_cell_count=48`;
  - `timeout_robot_local_cost_max=100`.

Controller / progress evidence:

- cmd_vel samples: `1881`.
- nonzero cmd_vel count: `1056`.
- controller_no_cmd_vel: `false`.
- max absolute linear_x command: `0.16112233698`.
- max absolute angular_z command: `0.15034610033`.
- robot total motion: `1.73933648745 m`.
- robot_stuck: `false`.
- first odom pose: `(1.8531101281, 0.0241760857, -0.0054941657)`.
- final odom pose: `(2.4175180047, 1.0220449274, 1.5843474170)`.
- robot-to-target distance improvement: `0.69108171048 m`.
- final distance to target: `0.33391829026 m`.

Nav2 feedback / recovery evidence:

- Nav2 feedback samples: `4587`.
- max number_of_recoveries: `4`.
- final distance_remaining: `0.33090686798 m`.

Local-cost / footprint / front-wedge evidence:

- target cost max during Phase66 run: `63`.
- target footprint max: `63`.
- target footprint lethal count max: `0`.
- front wedge cost max: `100`.
- front wedge high-cost count max: `209`.
- local-cost risk observed: `true`.

## Classification rationale

Classification: `INNER_INGRESS_TIMEOUT_REMAINS`.

Reason: Phase66 preserved the Phase65 inner ingress improvement enough to dispatch
from an inside-maze pose, but the bounded autonomous run did not produce multiple
successful dispatch/outcome cycles. The first autonomous goal timed out and was
canceled after timeout. Runtime data also shows local-cost / front-wedge / robot
footprint risk at timeout (`timeout_front_wedge_cost_max=99`,
`timeout_footprint_lethal_cell_count=48`, `timeout_robot_local_cost_max=100`), so
`LOCAL_COST_RISK_REMAINS` is recorded as secondary observed risk.

This is not `NO_CANDIDATE_REMAINS`: dispatch occurred, and the dispatch event
reported `last_open_direction_count=4` and `last_candidate_count=4`.

This is not autonomous exploration success and not exit success: `exit_reached` was
not observed and final goal count was only `1`.

## Validation log

Completed after runtime:

- `python3 -m py_compile tools/analyze_phase66_bounded_autonomous_rerun_from_inner_ingress.py src/tugbot_maze/test/test_phase66_bounded_autonomous_rerun_from_inner_ingress.py`: PASS
- `bash -n tools/run_phase66_bounded_autonomous_rerun_from_inner_ingress.sh`: PASS
- `pytest -q src/tugbot_maze/test/test_phase66_bounded_autonomous_rerun_from_inner_ingress.py`: PASS (`4 passed in 0.00s`)
- `source /opt/ros/jazzy/setup.bash && colcon build --packages-select tugbot_maze tugbot_bringup --symlink-install`: PASS (`2 packages finished [0.94s]`)
- `git diff -- src/tugbot_navigation/config | wc -c`: `0`
- Cleanup check: empty; `phase66_bounded_autonomous_rerun_from_inner_ingress_cleanup_processes_after.txt` has 0 bytes and live `pgrep` check returned empty.

## Stop condition

Stop after Phase66 evidence and wait for human acceptance. 不进入 Phase67.
