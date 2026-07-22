"""Shared helpers for future maze image/world conversion code."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class MazePose:
    """Simple 2D pose used by maze metadata and tests."""

    x_m: float
    y_m: float
    yaw_rad: float = 0.0


@dataclass(frozen=True)
class MazeExitRegion:
    """Circular first-pass exit region."""

    x_m: float
    y_m: float
    radius_m: float

    def contains(self, x_m: float, y_m: float) -> bool:
        return (x_m - self.x_m) ** 2 + (y_m - self.y_m) ** 2 <= self.radius_m**2
