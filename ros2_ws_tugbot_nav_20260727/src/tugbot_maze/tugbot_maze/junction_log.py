"""ROS-free per-run record of discovered junctions (cells with >= 3 open exits).

A passive memory asset / analyzable dataset: the JunctionLog stores each junction's coordinate,
open exits, discovery order, first-seen time, and visit count, and persists to JSON. update_junctions
is the pure per-tick glue the node calls; it owns the detection predicate so the node stays thin.
Routing is NOT affected by anything here.
"""
import json
import os
from typing import Dict, List, Optional, Tuple

from tugbot_maze.flood_fill_brain import open_exits

Cell = Tuple[int, int]


class JunctionLog:
    def __init__(self):
        self._j: Dict[Cell, dict] = {}
        self._order = 0
        self._visits: Dict[Cell, int] = {}

    @property
    def count(self) -> int:
        return len(self._j)

    def visit(self, cell: Cell) -> None:
        self._visits[cell] = self._visits.get(cell, 0) + 1
        if cell in self._j:
            self._j[cell]['visits'] = self._visits[cell]

    def observe(self, cell: Cell, exits: List[str], t: float,
                cell_size_m: float = 2.0) -> Tuple[dict, bool]:
        """Record on first sight (assign discovery_index + first_seen_s); on later sights refresh
        exits/exit_count while keeping discovery order, first-seen time, and visit count. Returns
        (entry, is_new)."""
        is_new = cell not in self._j
        if is_new:
            self._order += 1
            self._j[cell] = {
                'cell': [cell[0], cell[1]],
                'center_xy': [round(cell_size_m * cell[0], 3), round(cell_size_m * cell[1], 3)],
                'first_seen_s': round(t, 2),
                'discovery_index': self._order,
                'visits': self._visits.get(cell, 0),
            }
        e = self._j[cell]
        e['exits'] = list(exits)
        e['exit_count'] = len(exits)
        return e, is_new

    def to_dict(self) -> dict:
        return {
            'junction_count': len(self._j),
            'junctions': sorted(self._j.values(), key=lambda r: r['discovery_index']),
        }

    def flush(self, path: str) -> None:
        d = os.path.dirname(path)
        if d:
            os.makedirs(d, exist_ok=True)
        tmp = path + '.tmp'
        with open(tmp, 'w') as f:
            json.dump(self.to_dict(), f, indent=2)
        os.replace(tmp, path)   # atomic


def update_junctions(jlog: JunctionLog, brain, cell: Cell, prev_cell: Optional[Cell],
                     sensed, t: float) -> Tuple[Optional[Cell], Optional[dict]]:
    """Per-tick junction bookkeeping (ROS-free, unit-testable). Counts a visit on cell change; if
    `cell` is sensed with >= 3 in-grid open exits, observe() it. Returns
    (new_prev_cell, newly_recorded_entry_or_None) -- the node logs a JUNCTION line iff non-None."""
    if cell != prev_cell:
        jlog.visit(cell)
        prev_cell = cell
    new_entry = None
    if cell in sensed:
        exits = open_exits(brain, cell)
        if len(exits) >= 3:
            entry, is_new = jlog.observe(cell, exits, t)
            if is_new:
                new_entry = entry
    return prev_cell, new_entry
