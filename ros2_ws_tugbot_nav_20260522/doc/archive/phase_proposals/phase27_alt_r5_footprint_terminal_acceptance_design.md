# Phase27-alt-R5 Footprint-based Terminal Line Acceptance Design

Status: `DESIGN_ONLY_NOT_RUNTIME_ENABLED`
Date: 2026-05-28
Workspace: `ros2_ws_tugbot_nav_20260522`

## Goal

Design a supplemental terminal acceptance predicate based on whether the Tugbot
body footprint reaches the vertical terminal line/cylinder at the configured exit
coordinate. This is a design-only phase. It does not start runtime smoke and does
not replace the current terminal predicate.

## Guardrails

- Do not enter Phase26Y/Phase26Z.
- Do not fetch `nav2_mppi_controller` source.
- Do not modify Nav2/MPPI/controller parameter semantics.
- Do not promote or reject `CostCritic=2.75`.
- Do not change the general DFS/topology branch-selection strategy.
- Do not relax the local-cost safety gate.
- Do not claim MPPI selected-control near-zero root cause.
- Do not start runtime smoke in R5.
- Do not directly replace the existing terminal predicate.
- Do not use a fallback behavior success label.

## Footprint audit

Sources inspected:

- `src/tugbot_description/models/tugbot/model.sdf`
- `src/tugbot_description/models/tugbot/meshes/base/tugbot_simp.stl`
- `src/tugbot_navigation/config/nav2_slam_params.yaml`
- `src/tugbot_maze/tugbot_maze/maze_explorer.py`

Findings:

- Base collision mesh:
  - path: `meshes/base/tugbot_simp.stl`
  - referenced by `base_link_collision` in `model.sdf`
  - mesh-derived XY radius from `base_link` origin: `0.36912210068126217 m`
- Nav2 costmap model:
  - `robot_radius: 0.35`
  - no polygon footprint is configured in the current Nav2 params.
- Extra collision geometry in SDF includes gripper boxes behind the base.
  Approximate max collision radius from the `base_link` origin is:
  - `0.4942061445948485 m`
- R5 recommended design value:
  - `robot_circumscribed_radius_m = 0.4942061445948485`
  - derived conservatively from SDF collision geometry, not just Nav2 costmap radius.

Approximate collision radii:

```text
base_link/base_mesh                         0.36912210068126217 m
gripper_collision_box                       0.43289721643826723 m
gripper_hand_collision_box                  0.47066336165034134 m
gripper_hand_sideL/R_collision              0.4796342356421193 m
gripper_hand_sideL/R_slide_collision_approx 0.4942061445948485 m
```

## Current radius comparison

Current Phase27-alt terminal acceptance radius:

```text
near_exit_terminal_acceptance_radius_m = 0.6
```

Proposed footprint line/cylinder acceptance radius:

```text
robot_circumscribed_radius_m + terminal_acceptance_margin_m
= 0.4942061445948485 + 0.05
= 0.5442061445948485 m
```

Relationship:

```text
0.5442061445948485 m < 0.6 m
```

Therefore, with a conservative `0.05 m` margin, the footprint-based predicate is
stricter than the current 0.6m center-distance predicate. It would not explain R4
final distances of about `0.782 m` or `0.703 m` as terminal success.

## Proposed predicate

A candidate terminal-line acceptance should only pass if all gates pass:

```text
distance(base_link_xy, exit_xy)
  <= robot_circumscribed_radius_m + terminal_acceptance_margin_m
```

and:

```text
robot cell is not occupied
exit cell is not occupied
line-of-sight cells from robot_xy to exit_xy contain no occupied cells
unknown cell policy passes
```

Interpretation:

- The exit is treated as an infinite vertical line/cylinder along z at `exit_xy`.
- Since runtime navigation uses the map plane, the relevant distance is XY distance
  from `base_link` origin to that exit line.
- If XY distance is less than or equal to the robot circumscribed radius plus a
  small margin, the robot body footprint is considered to touch/cover the terminal
  line.

## Occupancy / known-space gates

Default unknown strategy:

```text
unknown_policy = conservative_reject
```

Meaning:

- unknown robot cell -> reject and record `robot_cell_unknown_conservative_reject`
- unknown exit cell -> reject and record `exit_cell_unknown_conservative_reject`
- unknown line-of-sight cell -> reject and record `line_of_sight_unknown_conservative_reject`

Alternative diagnostics-only mode:

```text
unknown_policy = record_only
```

This can be useful for offline analysis, but should not be the default runtime
acceptance policy because it could allow acceptance through unmapped space.

Occupied-cell strategy:

- occupied robot cell -> reject `robot_cell_occupied`
- occupied exit cell -> reject `exit_cell_occupied`
- occupied line-of-sight count > 0 -> reject `line_of_sight_occupied`

This specifically addresses the隔墙误判 risk: being close in Euclidean distance
is not enough if an occupied wall lies between robot and exit.

## Integration recommendation

R5 is design-only. If later promoted, implement as a supplemental terminal monitor
helper, not as a direct replacement in this phase:

```python
if self._exit_reached(robot_pose):
    ... existing behavior ...
elif self.footprint_terminal_acceptance_enabled and self._footprint_exit_reached(robot_pose):
    ... enter EXIT_REACHED with diagnostic reason footprint_terminal_line_acceptance ...
```

Recommended default:

```text
footprint_terminal_acceptance_enabled = false
```

Required event diagnostics if implemented later:

- `terminal_acceptance_mode = footprint_terminal_line`
- `robot_circumscribed_radius_m`
- `terminal_acceptance_margin_m`
- `footprint_acceptance_radius_m`
- `robot_exit_dist`
- `distance_gate_passed`
- `robot_cell_value`
- `exit_cell_value`
- `line_of_sight_occupied_count`
- `line_of_sight_unknown_count`
- `unknown_policy`
- `acceptance_reject_reason`

## Host-level tests required before implementation

The R5 test contract covers:

- footprint touching terminal line -> success
- base_link origin not at exit but footprint covers exit line -> success
- distance beyond footprint radius + margin -> fail
- wall / occupied line-of-sight between robot and exit -> fail
- occupied robot or exit cell -> fail
- unknown cells -> conservative reject by default, or record-only if explicitly selected
- margin boundary stability

## Offline analyzer

R5 adds:

- `tools/analyze_phase27_alt_r5_footprint_acceptance_offline.py`

It provides:

- pure helper `evaluate_footprint_terminal_acceptance(...)`
- offline comparison of existing explorer_state JSONL against:
  - current `0.6 m` center-distance predicate
  - footprint radius + margin predicate

This analyzer is design/static only. It does not prove occupancy gate pass/fail
for old runs because existing explorer_state logs do not contain map-cell line-of-
sight samples at every state. It only compares distance thresholds from existing
state samples.
