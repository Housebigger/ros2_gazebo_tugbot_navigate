"""Occupancy-grid helpers for pure Python maze perception tests.

The ROS node can construct this view from nav_msgs/OccupancyGrid, but this module
keeps the logic independent of ROS so branch detection can be tested quickly.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable, List, Tuple

Cell = Tuple[int, int]
Point = Tuple[float, float]


@dataclass(frozen=True)
class OccupancyGridInfo:
    width: int
    height: int
    resolution: float
    origin_x: float = 0.0
    origin_y: float = 0.0


class OccupancyGridView:
    def __init__(self, info: OccupancyGridInfo, data: Iterable[int], occupied_threshold: int = 65) -> None:
        self.info = info
        self.data: List[int] = list(data)
        self.occupied_threshold = int(occupied_threshold)
        expected = self.info.width * self.info.height
        if len(self.data) != expected:
            raise ValueError(f'occupancy data length {len(self.data)} does not match width*height {expected}')

    def in_bounds(self, cell: Cell) -> bool:
        x, y = cell
        return 0 <= x < self.info.width and 0 <= y < self.info.height

    def world_to_cell(self, x: float, y: float) -> Cell:
        cell_x = int(math.floor((x - self.info.origin_x) / self.info.resolution))
        cell_y = int(math.floor((y - self.info.origin_y) / self.info.resolution))
        return (cell_x, cell_y)

    def cell_to_world(self, cell_x: int, cell_y: int) -> Point:
        return (
            self.info.origin_x + (cell_x + 0.5) * self.info.resolution,
            self.info.origin_y + (cell_y + 0.5) * self.info.resolution,
        )

    def cell_value(self, cell: Cell) -> int:
        if not self.in_bounds(cell):
            return 100
        x, y = cell
        return int(self.data[x + y * self.info.width])

    def is_unknown(self, cell: Cell) -> bool:
        return self.in_bounds(cell) and self.cell_value(cell) < 0

    def is_occupied(self, cell: Cell) -> bool:
        if not self.in_bounds(cell):
            return True
        value = self.cell_value(cell)
        return value >= self.occupied_threshold

    def is_free(self, cell: Cell) -> bool:
        if not self.in_bounds(cell):
            return False
        value = self.cell_value(cell)
        return 0 <= value < self.occupied_threshold

    def has_clearance(self, cell: Cell, radius_m: float, unknown_is_safe: bool = False) -> bool:
        if not self.in_bounds(cell) or not self.is_free(cell):
            return False
        radius_cells = int(math.ceil(max(radius_m, 0.0) / self.info.resolution))
        cx, cy = cell
        for y in range(cy - radius_cells, cy + radius_cells + 1):
            for x in range(cx - radius_cells, cx + radius_cells + 1):
                candidate = (x, y)
                distance_m = math.hypot(x - cx, y - cy) * self.info.resolution
                if distance_m > radius_m + 1e-9:
                    continue
                if not self.in_bounds(candidate):
                    return False
                if self.is_occupied(candidate):
                    return False
                if self.is_unknown(candidate) and not unknown_is_safe:
                    return False
        return True

    def world_point_has_clearance(self, x: float, y: float, radius_m: float, unknown_is_safe: bool = False) -> bool:
        return self.has_clearance(self.world_to_cell(x, y), radius_m, unknown_is_safe=unknown_is_safe)

    def nearest_obstacle_distance(self, x: float, y: float, max_radius_m: float = 1.5, unknown_is_obstacle: bool = True) -> float:
        center = self.world_to_cell(x, y)
        if not self.in_bounds(center):
            return 0.0
        max_radius_m = max(0.0, float(max_radius_m))
        radius_cells = int(math.ceil(max_radius_m / self.info.resolution))
        best = max_radius_m
        cx, cy = center
        for cell_y in range(cy - radius_cells, cy + radius_cells + 1):
            for cell_x in range(cx - radius_cells, cx + radius_cells + 1):
                cell = (cell_x, cell_y)
                if not self.in_bounds(cell):
                    distance = math.hypot(cell_x - cx, cell_y - cy) * self.info.resolution
                    best = min(best, distance)
                    continue
                value = self.cell_value(cell)
                if value >= self.occupied_threshold or (unknown_is_obstacle and value < 0):
                    distance = math.hypot(cell_x - cx, cell_y - cy) * self.info.resolution
                    best = min(best, distance)
        return float(best)

    def line_sample_values(self, start: Point, end: Point, sample_step_m: float | None = None) -> List[int]:
        distance = math.hypot(end[0] - start[0], end[1] - start[1])
        if distance <= 1e-9:
            return [self.cell_value(self.world_to_cell(start[0], start[1]))]
        step = sample_step_m if sample_step_m is not None else max(self.info.resolution * 0.5, 0.05)
        steps = max(1, int(math.ceil(distance / max(step, 1e-6))))
        values: List[int] = []
        for index in range(steps + 1):
            ratio = index / steps
            x = start[0] + (end[0] - start[0]) * ratio
            y = start[1] + (end[1] - start[1]) * ratio
            values.append(self.cell_value(self.world_to_cell(x, y)))
        return values

    def line_min_clearance(self, start: Point, end: Point, max_radius_m: float = 1.5, sample_step_m: float | None = None) -> float:
        distance = math.hypot(end[0] - start[0], end[1] - start[1])
        if distance <= 1e-9:
            return self.nearest_obstacle_distance(start[0], start[1], max_radius_m=max_radius_m)
        step = sample_step_m if sample_step_m is not None else max(self.info.resolution * 2.0, 0.10)
        steps = max(1, int(math.ceil(distance / max(step, 1e-6))))
        best = max_radius_m
        for index in range(steps + 1):
            ratio = index / steps
            x = start[0] + (end[0] - start[0]) * ratio
            y = start[1] + (end[1] - start[1]) * ratio
            best = min(best, self.nearest_obstacle_distance(x, y, max_radius_m=max_radius_m))
        return float(best)
