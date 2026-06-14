"""Unit tests for the ROS-free frontier-coverage module.

Synthetic occupancy grids (0=free, -1=unknown, 100=occupied) exercise frontier
detection, clustering, goal selection (nearest / exit-bias / clearance / exclude)
and the end-to-end coverage_goal convenience wrapper.
"""

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import pytest

from tugbot_maze.grid_utils import OccupancyGridInfo, OccupancyGridView
from tugbot_maze.frontier_coverage import (
    Frontier,
    detect_frontier_cells,
    cluster_frontiers,
    select_frontier,
    coverage_goal,
)


# --------------------------------------------------------------------------- #
# Grid construction helpers
# --------------------------------------------------------------------------- #
def build_grid(width, height, *, resolution=1.0, origin=(0.0, 0.0),
               default=0, free=(), unknown=(), occupied=()):
    """Start every cell at `default`, then stamp free(0)/unknown(-1)/occupied(100)."""
    data = [default] * (width * height)

    def stamp(cells, value):
        for (x, y) in cells:
            data[x + y * width] = value

    stamp(free, 0)
    stamp(unknown, -1)
    stamp(occupied, 100)
    return OccupancyGridView(
        info=OccupancyGridInfo(width, height, resolution, origin[0], origin[1]),
        data=data,
    )


def half_grid():
    """10x10: left half (x<=4) free, right half (x>=5) unknown.

    The single explored-space boundary is the free column x=4.
    """
    data = [0 if x <= 4 else -1 for y in range(10) for x in range(10)]
    return OccupancyGridView(OccupancyGridInfo(10, 10, 1.0, 0.0, 0.0), data)


# --------------------------------------------------------------------------- #
# 1. Detection
# --------------------------------------------------------------------------- #
def test_detect_returns_free_boundary_column_adjacent_to_unknown():
    grid = half_grid()

    cells = detect_frontier_cells(grid)

    # Exactly the free column x=4 (each cell's right neighbor x=5 is unknown).
    assert set(cells) == {(4, y) for y in range(10)}
    for cell in cells:
        assert grid.is_free(cell)
        cx, cy = cell
        neighbors = [(cx + 1, cy), (cx - 1, cy), (cx, cy + 1), (cx, cy - 1)]
        assert any(grid.is_unknown(n) for n in neighbors)


# --------------------------------------------------------------------------- #
# 2. No frontier  (coverage complete)
# --------------------------------------------------------------------------- #
def test_no_frontier_on_all_free_grid():
    grid = build_grid(8, 8, default=0)

    assert detect_frontier_cells(grid) == []
    assert coverage_goal(grid, robot_xy=(1.0, 1.0), exit_xy=(7.0, 7.0)) is None


def test_no_frontier_with_free_and_occupied_but_no_unknown():
    # A free room walled by occupied cells; no unknown anywhere.
    occupied = [(x, 0) for x in range(8)] + [(x, 7) for x in range(8)] \
        + [(0, y) for y in range(8)] + [(7, y) for y in range(8)]
    grid = build_grid(8, 8, default=0, occupied=occupied)

    assert detect_frontier_cells(grid) == []
    assert coverage_goal(grid, robot_xy=(3.0, 3.0), exit_xy=(4.0, 4.0)) is None


# --------------------------------------------------------------------------- #
# 3. Clustering
# --------------------------------------------------------------------------- #
def test_clustering_groups_boundary_and_drops_tiny_isolated_frontier():
    # Pocket A: free column x=1 (rows 1..5) with unknown to its left (x=0).
    # Pocket B: a single free cell (8,8) with unknown to its right (9,8).
    pocket_a = [(1, y) for y in range(1, 6)]
    unknown_a = [(0, y) for y in range(1, 6)]
    pocket_b = [(8, 8)]
    unknown_b = [(9, 8)]
    grid = build_grid(
        10, 10, default=100,
        free=pocket_a + pocket_b,
        unknown=unknown_a + unknown_b,
    )

    cells = detect_frontier_cells(grid)
    assert set(cells) == set(pocket_a) | set(pocket_b)  # 5 + 1

    # min_size=4 keeps only the big boundary cluster, drops the size-1 pocket B.
    clusters = cluster_frontiers(cells, grid, min_size=4)
    assert len(clusters) == 1
    cluster = clusters[0]
    assert cluster.size == 5
    # Centroid sits on the boundary column x=1 (world x=1.5), mid-rows (y mean=3).
    assert cluster.centroid[0] == pytest.approx(1.5)
    assert cluster.centroid[1] == pytest.approx(3.5)
    assert grid.world_to_cell(*cluster.centroid) == (1, 3)

    # With min_size=1 both clusters survive (proves min_size is what dropped B).
    all_clusters = cluster_frontiers(cells, grid, min_size=1)
    assert len(all_clusters) == 2
    assert sorted(c.size for c in all_clusters) == [1, 5]


# --------------------------------------------------------------------------- #
# two-frontier fixture used by selection / exclude tests
# --------------------------------------------------------------------------- #
def two_frontier_grid():
    """10x10: left pocket L (col x=1, rows1..4) and right pocket R (col x=8, rows5..8),
    each free and bordered by unknown on the outer side; everything else occupied."""
    pocket_l = [(1, y) for y in range(1, 5)]      # frontier via unknown at x=0
    unknown_l = [(0, y) for y in range(1, 5)]
    pocket_r = [(8, y) for y in range(5, 9)]      # frontier via unknown at x=9
    unknown_r = [(9, y) for y in range(5, 9)]
    grid = build_grid(
        10, 10, default=100,
        free=pocket_l + pocket_r,
        unknown=unknown_l + unknown_r,
    )
    return grid, pocket_l, pocket_r


# --------------------------------------------------------------------------- #
# 4. Selection — nearest (exit_bias=0)
# --------------------------------------------------------------------------- #
def test_select_nearest_frontier_when_exit_bias_zero():
    grid, pocket_l, pocket_r = two_frontier_grid()
    frontiers = cluster_frontiers(detect_frontier_cells(grid), grid, min_size=4)
    assert len(frontiers) == 2

    robot = (2.0, 2.5)          # right next to pocket L
    exit_xy = (9.5, 9.5)        # far away; irrelevant with exit_bias=0

    goal = select_frontier(frontiers, robot, exit_xy, grid, exit_bias=0.0)

    # Nearest clearance-valid cell of L to the robot is the center of (1,2) = (1.5,2.5).
    assert goal == pytest.approx((1.5, 2.5))
    # Returned goal is a real free, clearance-valid world point.
    assert grid.is_free(grid.world_to_cell(*goal))
    assert grid.world_point_has_clearance(goal[0], goal[1], 0.35, unknown_is_safe=True)


# --------------------------------------------------------------------------- #
# 5. Selection — exit bias
# --------------------------------------------------------------------------- #
def test_select_exit_ward_frontier_when_exit_bias_high():
    # Two mirror-image frontiers equidistant from a centered robot; exit on the right.
    pocket_a = [(2, y) for y in range(4, 8)]
    unknown_a = [(1, y) for y in range(4, 8)]
    pocket_b = [(8, y) for y in range(4, 8)]
    unknown_b = [(9, y) for y in range(4, 8)]
    grid = build_grid(
        11, 11, default=100,
        free=pocket_a + pocket_b,
        unknown=unknown_a + unknown_b,
    )
    frontiers = cluster_frontiers(detect_frontier_cells(grid), grid, min_size=4)
    assert len(frontiers) == 2

    robot = (5.5, 5.5)          # equidistant: nearest cell of each pocket is 3.0 away
    exit_xy = (10.5, 5.5)       # to the right -> pocket B is exit-ward

    goal = select_frontier(frontiers, robot, exit_xy, grid, exit_bias=5.0)

    # Exit-ward pocket B wins; nearest cell to robot is center of (8,5) = (8.5,5.5).
    assert goal == pytest.approx((8.5, 5.5))


# --------------------------------------------------------------------------- #
# 6. Clearance filter
# --------------------------------------------------------------------------- #
def test_frontier_without_clearance_is_skipped():
    # A width-1 free corridor (rows of cells (2..5, 5)) with unknown along the top
    # (every corridor cell is a frontier) and occupied along the bottom + both ends.
    corridor = [(x, 5) for x in range(2, 6)]
    unknown_top = [(x, 4) for x in range(2, 6)]
    grid = build_grid(8, 11, default=100, free=corridor, unknown=unknown_top)

    frontiers = cluster_frontiers(detect_frontier_cells(grid), grid, min_size=4)
    assert len(frontiers) == 1 and frontiers[0].size == 4

    robot = (3.5, 5.5)
    exit_xy = (0.0, 0.0)

    # clearance_m=1.0 -> orthogonal neighbors are checked; every corridor cell has an
    # occupied orthogonal neighbor, so none has clearance -> frontier skipped -> None.
    assert select_frontier(frontiers, robot, exit_xy, grid, clearance_m=1.0) is None

    # With a tiny clearance only the (free) center cell is checked, so a goal IS found.
    assert select_frontier(frontiers, robot, exit_xy, grid, clearance_m=0.3) is not None


# --------------------------------------------------------------------------- #
# 7. Exclude list
# --------------------------------------------------------------------------- #
def test_excluded_frontier_is_skipped():
    grid, pocket_l, pocket_r = two_frontier_grid()
    frontiers = cluster_frontiers(detect_frontier_cells(grid), grid, min_size=4)
    left = min(frontiers, key=lambda f: f.centroid[0])    # pocket L (smaller x)

    robot = (2.0, 2.5)          # nearest to L
    exit_xy = (9.5, 9.5)

    # Without exclude, the nearer L is chosen.
    goal_plain = select_frontier(frontiers, robot, exit_xy, grid, exit_bias=0.0)
    assert goal_plain == pytest.approx((1.5, 2.5))

    # Excluding L's centroid (within default radius) forces selection of R.
    goal_excl = select_frontier(
        frontiers, robot, exit_xy, grid, exit_bias=0.0, exclude=(left.centroid,)
    )
    assert goal_excl == pytest.approx((8.5, 5.5))


# --------------------------------------------------------------------------- #
# 8. coverage_goal integration
# --------------------------------------------------------------------------- #
def test_coverage_goal_returns_goal_near_single_frontier():
    grid = half_grid()          # single boundary frontier on column x=4

    goal = coverage_goal(grid, robot_xy=(0.5, 4.5), exit_xy=(9.5, 4.5))

    assert goal is not None
    # Goal lies on the boundary column (world x = 4.5) and is a valid free point.
    assert goal[0] == pytest.approx(4.5)
    assert grid.is_free(grid.world_to_cell(*goal))
    assert grid.world_point_has_clearance(goal[0], goal[1], 0.35, unknown_is_safe=True)


def test_coverage_goal_returns_none_when_fully_known():
    grid = build_grid(8, 8, default=0)          # everything free, nothing unknown

    assert coverage_goal(grid, robot_xy=(1.0, 1.0), exit_xy=(6.0, 6.0)) is None
