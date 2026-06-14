"""ROS-free Trémaux maze-solving brain.

Decides the next exploration action from the current local topology and a
persistent junction graph with per-edge traversal counts (Trémaux marks). A
corridor is never permanently discarded because of a navigation failure — only a
physically-traversed-twice edge or a geometry-confirmed wall is excluded. This is
the correctness core; it has no ROS dependencies and is fully unit-tested.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import List, Optional, Tuple

from . import maze_perception as perception
from .maze_topology import (
    MazeTopology, TopoNode, BranchOption,
    UNTRIED, IN_PROGRESS, EXPLORED,
    DEAD_END as BRANCH_DEAD_END,
)

Point = Tuple[float, float]

EXPLORE = 'explore'
REROUTE = 'reroute'
BACK_OUT = 'back_out'
DONE = 'done'

OUT_SUCCESS = 'success'
OUT_WALL = 'wall'
OUT_WEDGED = 'wedged'


@dataclass
class Action:
    kind: str
    target_xy: Optional[Point] = None
    path_xy: List[Point] = field(default_factory=list)
    yaw: Optional[float] = None
    reason: str = ''


class TremauxSolver:
    def __init__(
        self,
        exit_xy: Point,
        *,
        junction_merge_radius_m: float = 0.75,
        exit_bias_weight: float = 0.3,
        exploration_bonus_weight: float = 0.6,
        distance_to_exit_weight: float = 0.15,
        max_soft_failures: int = 3,
    ) -> None:
        self.exit_xy = (float(exit_xy[0]), float(exit_xy[1]))
        self.topology = MazeTopology(
            junction_merge_radius_m=junction_merge_radius_m,
            exit_bias_weight=exit_bias_weight,
            exploration_bonus_weight=exploration_bonus_weight,
            distance_to_exit_weight=distance_to_exit_weight,
        )
        self.max_soft_failures = int(max_soft_failures)
        self.active_start_node_id: Optional[int] = None
        self.active_branch: Optional[BranchOption] = None

    @staticmethod
    def _yaw(a: Point, b: Point) -> float:
        return math.atan2(b[1] - a[1], b[0] - a[0])

    def _edge_between(self, a_id: int, b_id: int):
        for edge in self.topology.edges.values():
            if {edge.start_node_id, edge.end_node_id} == {a_id, b_id}:
                return edge
        return None

    def _register_node(self, robot_xy: Point, local) -> TopoNode:
        if local.kind == perception.JUNCTION and len(local.open_directions) >= 3:
            center = perception.compute_junction_center(robot_xy, local.open_directions)
        else:
            center = (robot_xy[0], robot_xy[1])
        node_type = (
            'dead_end' if local.kind == perception.DEAD_END
            else 'junction' if local.kind == perception.JUNCTION
            else 'corridor'
        )
        return self.topology.find_or_create_node(center[0], center[1], node_type=node_type)

    def _set_branches(self, node: TopoNode, local) -> None:
        options = [
            BranchOption(angle_rad=d.angle_rad, target_xy=(d.target_xy[0], d.target_xy[1]))
            for d in local.open_directions
        ]
        self.topology.set_branch_options(node.node_id, options)

    def update(self, robot_xy: Point, robot_yaw: float, local) -> Action:
        node = self._register_node(robot_xy, local)
        edge = None
        if self.active_start_node_id is not None and node.node_id != self.active_start_node_id:
            edge = self.topology.connect_nodes(self.active_start_node_id, node.node_id, state=EXPLORED)
            edge.visit_count = max(edge.visit_count, 1)
            if self.active_branch is not None:
                self.active_branch.state = EXPLORED
        self.topology.visit_node(node.node_id)
        self._set_branches(node, local)

        if local.kind == perception.DEAD_END:
            if edge is not None:
                edge.visit_count = 2
            if self.active_branch is not None:
                self.active_branch.state = BRANCH_DEAD_END
            if self.active_start_node_id is not None:
                prev_xy = self.topology.nodes[self.active_start_node_id].xy
            else:
                prev_xy = (robot_xy[0] - math.cos(robot_yaw), robot_xy[1] - math.sin(robot_yaw))
            self.active_branch = None
            self.active_start_node_id = node.node_id
            return Action(BACK_OUT, target_xy=prev_xy, yaw=self._yaw(robot_xy, prev_xy),
                          reason='dead end: reverse out')

        self.active_branch = None
        chosen = self.topology.choose_next_branch(node.node_id, exit_xy=self.exit_xy)
        if chosen is not None:
            self.active_start_node_id = node.node_id
            self.active_branch = chosen
            self.topology.mark_branch_state(node.node_id, chosen, IN_PROGRESS)
            return Action(EXPLORE, target_xy=chosen.target_xy, yaw=chosen.angle_rad,
                          reason='explore untried branch')

        result = self.topology.dijkstra_nearest_unexplored(node.node_id)
        if result is not None:
            path_ids, _target = result
            path = [self.topology.nodes[i].xy for i in path_ids]
            # Reroute spans already-known edges; clear the active start so the
            # next update() does not add a spurious shortcut edge on arrival.
            self.active_start_node_id = None
            self.active_branch = None
            return Action(REROUTE, path_xy=path, reason='reroute to nearest untried junction')

        return Action(DONE, reason='full coverage: no untried branch reachable')

    def report_outcome(self, outcome: str) -> None:
        """Called by the node after a pilot action completes without arriving
        at a new node (wall confirmed, or wedged/timeout). SUCCESS is handled
        implicitly by the next update() connecting the edge.
        """
        branch = self.active_branch
        if branch is None:
            return
        if outcome == OUT_WALL:
            branch.state = BRANCH_DEAD_END
            self.active_branch = None
        elif outcome == OUT_WEDGED:
            branch.failures += 1
            if branch.failures >= self.max_soft_failures:
                branch.state = EXPLORED  # give up WITHOUT blacklisting the corridor
            else:
                branch.state = UNTRIED   # retry on a later visit
            self.active_branch = None
