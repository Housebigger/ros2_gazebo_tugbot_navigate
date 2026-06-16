"""ROS-free micromouse flood-fill brain for the 2 m maze cell grid.

Memory = per-edge wall knowledge (WALL / OPEN / UNKNOWN). flood() computes
distance-to-exit by BFS from the exit over edges that are not WALL (UNKNOWN is
optimistically passable). next_cell() steps toward the exit along the lowest-
distance non-WALL neighbor. Deterministic; no ROS / time / I/O.
"""
from __future__ import annotations
import collections
import math
from typing import Dict, Optional, Tuple

Cell = Tuple[int, int]

CELL_SIZE_M = 2.0
CX_MIN, CX_MAX = 1, 10
CY_MIN, CY_MAX = 0, 9
ENTRANCE_CELL: Cell = (1, 0)
EXIT_CELL: Cell = (10, 9)
DIRS = {'N': (0, 1), 'S': (0, -1), 'E': (1, 0), 'W': (-1, 0)}
OPP = {'N': 'S', 'S': 'N', 'E': 'W', 'W': 'E'}

WALL, OPEN, UNKNOWN = 'wall', 'open', 'unknown'


def cell_center(cell: Cell) -> Tuple[float, float]:
    return (CELL_SIZE_M * cell[0], CELL_SIZE_M * cell[1])


def pose_to_cell(x: float, y: float) -> Cell:
    return (int(round(x / CELL_SIZE_M)), int(round(y / CELL_SIZE_M)))


def in_grid(cell: Cell) -> bool:
    return CX_MIN <= cell[0] <= CX_MAX and CY_MIN <= cell[1] <= CY_MAX


class FloodFillBrain:
    def __init__(self, exit_cell: Cell = EXIT_CELL):
        self.exit_cell = exit_cell
        self._edge: Dict[Tuple[Cell, str], str] = {}   # (cell, dir) -> WALL/OPEN (absent = UNKNOWN)
        self._traversals: Dict[frozenset, int] = {}     # undirected {A,B} edge -> traversal count

    def _state(self, cell: Cell, d: str) -> str:
        return self._edge.get((cell, d), UNKNOWN)

    def is_wall(self, cell: Cell, d: str) -> bool:
        return self._state(cell, d) == WALL

    def mark(self, cell: Cell, d: str, is_wall: bool) -> None:
        st = WALL if is_wall else OPEN
        self._edge[(cell, d)] = st
        dx, dy = DIRS[d]
        nb = (cell[0] + dx, cell[1] + dy)
        if in_grid(nb):
            self._edge[(nb, OPP[d])] = st        # keep knowledge symmetric (in-grid only)

    def _passable(self, cell: Cell, d: str) -> bool:
        return self._state(cell, d) != WALL       # OPEN or UNKNOWN are passable

    def _edge_traversal_count(self, a: Cell, b: Cell) -> int:
        return self._traversals.get(frozenset((a, b)), 0)

    def mark_traversal(self, a: Cell, b: Cell) -> None:
        """Record that the robot traversed the undirected edge a<->b (Trémaux count)."""
        e = frozenset((a, b))
        self._traversals[e] = self._traversals.get(e, 0) + 1

    def flood(self) -> Dict[Cell, int]:
        dist = {self.exit_cell: 0}
        q = collections.deque([self.exit_cell])
        while q:
            c = q.popleft()
            for d, (dx, dy) in DIRS.items():
                nb = (c[0] + dx, c[1] + dy)
                if in_grid(nb) and self._passable(c, d) and nb not in dist:
                    dist[nb] = dist[c] + 1
                    q.append(nb)
        return dist

    def next_cell(self, cur: Cell) -> Optional[Cell]:
        if cur == self.exit_cell:
            return None
        dist = self.flood()
        cands = []
        for d, (dx, dy) in DIRS.items():
            nb = (cur[0] + dx, cur[1] + dy)
            if in_grid(nb) and self._passable(cur, d):
                cands.append(nb)
        if not cands:
            return None
        # Trémaux: prefer the LEAST-traversed edge (unmarked over once-marked); never re-use
        # a twice-traversed edge unless it is the only option. This bounds each edge to <=2
        # traversals -> provably terminates -> escapes the multi-cell cycles that pure
        # flood-descent falls into under imperfect live sensing. Ties break toward the exit
        # (flood distance), then DIRS order. (Subsumes came-from anti-reversal: the just-used
        # edge is marked once, so an unmarked alternative is preferred over reversing.)
        legal = [nb for nb in cands if self._edge_traversal_count(cur, nb) < 2]
        pool = legal if legal else cands
        return min(pool, key=lambda nb: (self._edge_traversal_count(cur, nb),
                                         dist.get(nb, math.inf)))

    def is_done(self, cell: Cell) -> bool:
        return cell == self.exit_cell
