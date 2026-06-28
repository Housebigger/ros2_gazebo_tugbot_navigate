# Perimeter Absolute Re-anchor — Design

> **Context:** built on branch `footprint-oracle-baseline` (off `main`/`8661d4a`, commit `d63c88d`) = main's working navigation + the true-footprint collision oracle + yaw in DIAG. This spec attacks the long-standing NE central-junction plateau.

**Branch:** `footprint-oracle-baseline` (continue). Trailer ends `Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>`. **No backticks in `git commit -m`.** Diagnostics stay IN. Merge LOCALLY only.

---

## Root cause (diagnosed from run-2 SENSE-vs-ground-truth)

The robot reaches the NE near-exit region (~(9,7), dist 5.65 m — closest ever) but cannot complete the final ~3 cells to the exit (10,9). It thrashes in (8,5)/(8,6)/(9,5)/(9,4), gives up edges, and 1-cell-desyncs. The mechanism, confirmed by comparing logged `SENSE` walls to `ground_truth_edge_open`: **the wall sensing is pervasively wrong in the NE because the robot senses from 0.5–0.85 m OFF-CENTER** (e.g. cell (9,5) sensed at (18.60, 9.36) vs center (18,10) = 0.6 m off → sensed walls `[W]` vs ground truth `[E,N,S]`). Off-center beyond the ±0.3 m sensing tolerance → cardinal wall-windows hit the wrong walls → mis-sense → garbage flood routing.

It is a **vicious cycle**, not simple odom drift: the `SENSE` logs show the robot *knows* it is off-center (`off=(-0.32, …)`) but `good=False` — off-center → can't sense reliably → can't compute a trustworthy centering offset → can't re-center → stays off-center. Once entered (in the open-ish, high-drift NE), per-cell centering (4 s timeout, 0.40 m fallback clamp) cannot recover the 0.6 m error, and it cascades. The interior corridor walls are too short / mis-associated to break it.

## Key geometric enabler

The NE cells sit near the maze **perimeter**, whose walls are at **known absolute map coordinates** (drift-immune, long, unambiguous):
- **W** vertical x = 1.02 (y∈[1, 19])
- **E** vertical x = 21.01 (y∈[−1, 17])  ← note: ends at y≈17
- **N** horizontal y = 19.02 (x∈[1, 21])
- **S** horizontal y = −0.97 (x∈[1, 21])

Wall half-thickness 0.12 → the **near surface** is the centerline shifted 0.12 toward the interior. A boundary cell's center is ~0.88 m (perp, to surface) from its perimeter wall. In a boundary cell, the perimeter cardinal has **nothing beyond it**, so a return there *is* the perimeter wall — an absolute position reference. The NE approach is ideal: cells (10, 5..8) face the E wall; cells (·, 9) face the N wall; the exit corner (10,9) has the N wall ~1 m away (it uses N, since the E wall ends at y≈17).

---

## Design: perimeter absolute re-anchor

When the robot believes it is in a **boundary cell**, use the perimeter wall as an absolute reference to correct **both** the centering offset (continuous, drives it to true center) **and** `self.cell` (discrete, fixes the 1-cell desync) on the boundary axis — breaking the off-center→mis-sense cycle.

### Component A — `perimeter_offset` (new, `wall_localize.py`)

```python
# perimeter wall NEAR-SURFACE coordinate on each boundary axis (centerline -/+ wall_half_thickness)
PERIM = {  # boundary-cell index -> (cardinal, axis 'x'/'y', wall surface coord, sign)
    'E': (21.01 - 0.12),   # cx == GRID_MAX_X (10): wall to the +x; x_true = surf - perp_E
    'W': (1.02 + 0.12),    # cx == 1:               wall to the -x; x_true = surf + perp_W
    'N': (19.02 - 0.12),   # cy == 9:               wall to the +y; y_true = surf - perp_N
    'S': (-0.97 + 0.12),   # cy == 0:               wall to the -y; y_true = surf + perp_S
}
```
`perimeter_offset(ranges, angle_min, angle_inc, yaw, cell) -> dict` returns, for each boundary axis the `cell` touches, the **absolute true coordinate** on that axis and the implied **cell index**:
- Determine which perimeter(s) `cell` touches: `cx == 10 → 'E'`, `cx == 1 → 'W'`, `cy == 9 → 'N'`, `cy == 0 → 'S'`. (The exit-corner E wall ends at y≈17, so for `cy >= 9` do NOT use 'E'; use 'N'. Encode this: 'E' is valid only when `cell` center y ≤ 16.)
- For each valid perimeter, read the perpendicular distance in that cardinal from `cell_wall_perp_dist(...)` (it returns the raw median perpendicular distance, NOT the 1.3 m-thresholded wall flag, so it reports the perimeter even at ~1.7 m when off-center). If that distance is `< max_range` and `<= PERIM_MAX_M` (= 2.5 m; beyond that the perimeter isn't reliably the closest return), compute the absolute coord:
  - E: `x_true = PERIM['E'] - d_E`;  W: `x_true = PERIM['W'] + d_W`;  N: `y_true = PERIM['N'] - d_N`;  S: `y_true = PERIM['S'] + d_S`.
- Return `{axis: (true_coord, implied_cell_index)}` where `implied_cell_index = round(true_coord / CELL_SIZE_M)`.

### Component B — integration in `MazeMotion._center`

In `_center`, after computing `ox, oy = cell_center_offset(ranges, amin, ainc, yaw)`, if `self.cell` is a boundary cell call `perimeter_offset(...)`:
- **dcell self-consistency / correction:** for each returned axis, if `implied_cell_index != self.cell[axis]` AND the implied position is confidently within the implied cell core (`abs(true_coord - CELL_SIZE_M*implied_cell_index) <= CELL_SIZE_M/2 - boundary_margin_m`), correct `self.cell` on that axis to `implied_cell_index` (this is the absolute, unbounded version of `_reanchor`, gated by the perimeter being seen — it fixes the 1-cell desync the bounded `_reanchor` couldn't).
- **centering offset override:** for each returned axis, replace that axis's centering offset with the **absolute** offset `CELL_SIZE_M*self.cell[axis] - true_coord` (after any dcell correction). This is trustworthy even when the robot is 0.6 m off-center (the interior-wall offset is not), so the centering drive nulls the *real* error → the robot reaches true center → senses correctly.
- **precedence:** the perimeter offset takes precedence over the corridor-wall offset AND the clamped odom-grid fallback on the boundary axis when a perimeter wall is confirmed.

(The non-boundary axis, and all non-boundary cells, are unchanged — they keep the existing `cell_center_offset` + fallback.)

---

## Why this breaks the cycle

The cycle is off-center → mis-sense → bad offset → can't re-center. The perimeter wall gives an **absolute** offset that does NOT depend on correctly sensing the (mis-associated) interior walls, so even from 0.6 m off-center the robot computes the true error and drives to the real center. Once centered, the interior sensing is back in tolerance → correct walls → routing works. The dcell self-consistency check simultaneously repairs the 1-cell desync that misroutes.

## Testing strategy

- **A (`test_wall_localize.py`):** synthetic boundary-cell scans — robot in cell (10,5) placed 0.6 m off-center toward the interior; `perimeter_offset` returns `x_true` matching the true pose (NOT the cell center, NOT the drifted belief) and `implied_cell_index == 10`; a case where the robot is actually in (9,5) but `self.cell` says (10,5) returns `implied_cell_index == 9` (the desync-correction path).
- **B (`test_maze_motion_sim.py` / a focused test):** drive the offline robot into a boundary cell 0.6 m off-center with injected drift; assert that after a `_center` it is within `center_tol_m` of true center (recovered) and `self.cell` is correct — FAILS without the perimeter re-anchor (the existing centering can't recover 0.6 m), PASSES with it. This is the offline guard that exercises the real mechanism.
- **Regression (gate):** the full ROS-free suite stays green (the 3 `[0.05-*]` xfails remain — perimeter re-anchor does not target those rear-corner collisions). Non-boundary behavior unchanged.
- **Gazebo (user-initiated):** does the NE plateau break — robot reaches the exit (or gets materially past dist 5.65 m), with NE sensing correct (compare `SENSE` walls to ground truth at (10,5)/(10,8)/(·,9)) and the true-oracle collision rate in the NE down?

## Scope

**In:** `perimeter_offset` in `wall_localize.py`; its integration into `MazeMotion._center` (dcell correction + boundary-axis centering-offset override); tests. The perimeter constants live with the function.

**Out:** the rear-corner drift+latency collisions (the 3 xfails — separate); non-boundary navigation (unchanged); the LIDAR correction / footprint stop (dropped, not revisited here).

## Risks

- **Mis-identifying an interior wall as the perimeter.** Mitigated: only fires in a boundary cell, only on the perimeter-facing cardinal (nothing exists beyond it there), and gated by `PERIM_MAX_M` (2.5 m) + the dcell self-consistency check (an implausible implied position is rejected).
- **Severe off-center (> ~2.5 m perp to the perimeter).** Then the perimeter return exceeds `PERIM_MAX_M` and the re-anchor abstains — it does not fire on a bad reading, so it never makes things worse; it just doesn't help that tick (the robot may recover on a later tick as it drifts back into range).
- **dcell over-correction.** The self-consistency core gate (`<= CELL_SIZE_M/2 - boundary_margin_m`) prevents flipping on a boundary-straddling reading; only a confident implied-cell-core correction is applied.
- **Gazebo perimeter geometry vs the model coords.** The constants come from `load_segments` (the real maze YAML); confirm in Gazebo that the NE `SENSE` walls match ground truth after the fix (the headline check).
