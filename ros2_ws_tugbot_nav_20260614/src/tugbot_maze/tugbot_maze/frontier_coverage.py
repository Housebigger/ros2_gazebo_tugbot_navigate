"""SLAM-occupancy-grid frontier detection + selection for coverage exploration.

A frontier cell is a known-free cell adjacent to unknown space -- the boundary of
the explored region. Driving to frontiers and repeating maps the whole reachable
maze (coverage-complete), so the robot is guaranteed to reach the exit. Operates
directly on the occupancy grid (no topology graph -> no node aliasing).
"""
from __future__ import annotations
from dataclasses import dataclass, field
import math
from typing import List, Optional, Tuple

from .grid_utils import OccupancyGridView

Point = Tuple[float, float]
Cell = Tuple[int, int]

# 4-connectivity: a frontier cell is free and touches unknown across an edge.
_NEIGHBORS_4 = ((1, 0), (-1, 0), (0, 1), (0, -1))
# 8-connectivity: used to group adjacent frontier cells into one cluster.
_NEIGHBORS_8 = (
    (1, 0), (-1, 0), (0, 1), (0, -1),
    (1, 1), (1, -1), (-1, 1), (-1, -1),
)


@dataclass
class Frontier:
    centroid: Point          # world coords (cell_to_world of the cell-centroid)
    size: int                # number of frontier cells in the cluster
    cells: List[Cell] = field(default_factory=list)


def detect_frontier_cells(grid: OccupancyGridView) -> List[Cell]:
    """Return all free cells that have >=1 UNKNOWN 4-neighbor (the explored-space boundary).

    Single pass over the grid. ``is_unknown`` is False for out-of-bounds cells, so the
    grid border (treated as a wall, not as explorable space) never yields frontiers.
    """
    cells: List[Cell] = []
    width = grid.info.width
    height = grid.info.height
    for y in range(height):
        for x in range(width):
            cell = (x, y)
            if not grid.is_free(cell):
                continue
            for dx, dy in _NEIGHBORS_4:
                if grid.is_unknown((x + dx, y + dy)):
                    cells.append(cell)
                    break
    return cells


def cluster_frontiers(cells: List[Cell], grid: OccupancyGridView, min_size: int = 4) -> List[Frontier]:
    """Group adjacent frontier cells (8-connectivity) into clusters.

    Computes the world centroid (``cell_to_world`` of the rounded mean cell) and the
    size of each cluster, and drops clusters smaller than ``min_size`` (sensor noise).
    Uses an iterative flood fill (no recursion) so dense grids cannot overflow the stack.
    """
    cell_set = set(cells)
    visited: set = set()
    clusters: List[Frontier] = []

    for start in cells:
        if start in visited:
            continue
        # Iterative flood fill over 8-connected frontier cells.
        stack = [start]
        visited.add(start)
        component: List[Cell] = []
        while stack:
            cx, cy = stack.pop()
            component.append((cx, cy))
            for dx, dy in _NEIGHBORS_8:
                neighbor = (cx + dx, cy + dy)
                if neighbor in cell_set and neighbor not in visited:
                    visited.add(neighbor)
                    stack.append(neighbor)

        if len(component) < min_size:
            continue

        mean_x = sum(c[0] for c in component) / len(component)
        mean_y = sum(c[1] for c in component) / len(component)
        centroid_cell = (int(round(mean_x)), int(round(mean_y)))
        centroid = grid.cell_to_world(centroid_cell[0], centroid_cell[1])
        clusters.append(Frontier(centroid=centroid, size=len(component), cells=component))

    return clusters


def _best_goal_cell(
    frontier: Frontier,
    robot_xy: Point,
    grid: OccupancyGridView,
    clearance_m: float,
) -> Optional[Point]:
    """World point of the frontier's clearance-valid cell nearest the robot, or None.

    Frontier cells border unknown space, so ``unknown_is_safe=True`` -- only occupied
    (or out-of-bounds) cells within the clearance disk disqualify a candidate.
    """
    best_point: Optional[Point] = None
    best_dist: Optional[float] = None
    for cx, cy in frontier.cells:
        wx, wy = grid.cell_to_world(cx, cy)
        if not grid.world_point_has_clearance(wx, wy, clearance_m, unknown_is_safe=True):
            continue
        dist = math.dist(robot_xy, (wx, wy))
        if best_dist is None or dist < best_dist or (dist == best_dist and (wx, wy) < best_point):
            best_point = (wx, wy)
            best_dist = dist
    return best_point


def select_frontier(
    frontiers: List[Frontier],
    robot_xy: Point,
    exit_xy: Point,
    grid: OccupancyGridView,
    *,
    clearance_m: float = 0.35,
    exit_bias: float = 0.5,
    size_weight: float = 0.05,
    exclude: Tuple[Point, ...] = (),
    exclude_radius_m: float = 1.2,
) -> Optional[Point]:
    """Choose the best reachable frontier GOAL (a clearance-valid world point near a
    frontier), or None if there are no usable frontiers (coverage complete).

    Score each frontier:
        -distance_to_robot + exit_bias * exit_closeness_improvement + size_weight * size
    where distance_to_robot = dist(robot, goal) and
          exit_closeness_improvement = dist(robot, exit) - dist(goal, exit).

    Frontiers whose returned GOAL is within ``exclude_radius_m`` of any excluded point
    (recently-failed frontiers) are skipped, as are frontiers without a clearance-valid
    cell. The exclusion is tested against the actual returned goal (not the cluster
    centroid), so a failed goal is reliably suppressed even when goal != centroid.
    Returns the goal of the highest-scoring frontier, or None.
    """
    robot_to_exit = math.dist(robot_xy, exit_xy)
    best_goal: Optional[Point] = None
    best_score: Optional[float] = None

    for frontier in frontiers:
        goal = _best_goal_cell(frontier, robot_xy, grid, clearance_m)
        if goal is None:
            continue
        # Skip recently-failed frontiers (returned goal near an excluded point).
        if any(math.hypot(goal[0] - ex[0], goal[1] - ex[1]) <= exclude_radius_m for ex in exclude):
            continue

        distance_to_robot = math.dist(robot_xy, goal)
        exit_closeness_improvement = robot_to_exit - math.dist(goal, exit_xy)
        score = (
            -distance_to_robot
            + exit_bias * exit_closeness_improvement
            + size_weight * frontier.size
        )
        if best_score is None or score > best_score:
            best_score = score
            best_goal = goal

    return best_goal


def coverage_goal(
    grid: OccupancyGridView,
    robot_xy: Point,
    exit_xy: Point,
    *,
    clearance_m: float = 0.35,
    min_frontier_size: int = 4,
    exit_bias: float = 0.5,
    exclude: Tuple[Point, ...] = (),
) -> Optional[Point]:
    """Convenience: detect -> cluster -> select. Returns the next frontier goal, or
    None when no frontiers remain (coverage complete).
    """
    cells = detect_frontier_cells(grid)
    if not cells:
        return None
    frontiers = cluster_frontiers(cells, grid, min_size=min_frontier_size)
    if not frontiers:
        return None
    return select_frontier(
        frontiers,
        robot_xy,
        exit_xy,
        grid,
        clearance_m=clearance_m,
        exit_bias=exit_bias,
        exclude=exclude,
    )
