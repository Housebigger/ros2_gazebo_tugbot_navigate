"""ROS-free Trémaux maze-solving brain.

Decides the next exploration action from the current local topology and a
persistent junction graph with per-edge traversal counts (Trémaux marks).

Corridor exclusion is state-based: a branch is skipped only when its state is
EXPLORED or DEAD_END; UNTRIED branches are the frontier.  A navigation failure
(wedge, timeout) never permanently blacklists a corridor — the branch retries
until max_soft_failures is reached, after which it is marked EXPLORED (give up,
no blacklist).  visit_count is the persisted per-edge Trémaux mark (1 = corridor
traversed once, 2 = traversed both ways or geometry-confirmed wall) maintained
for the coverage guarantee; it is not read to make exclusion decisions.

This is the correctness core; it has no ROS dependencies and is fully
unit-tested.
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
        # Idempotency guard: if update() is called again at the same node while
        # we are still committed to an in-progress branch (robot hasn't moved
        # yet), re-issue that branch rather than re-deciding — re-deciding would
        # detach active_branch from the branch actually being driven and let a
        # later dead-end/outcome mark the WRONG branch.
        if (self.active_start_node_id == node.node_id
                and self.active_branch is not None
                and local.kind != perception.DEAD_END):
            return Action(EXPLORE, target_xy=self.active_branch.target_xy,
                          yaw=self.active_branch.angle_rad,
                          reason='already committed to in-progress branch')
        edge = None
        if self.active_start_node_id is not None and node.node_id != self.active_start_node_id:
            edge = self.topology.connect_nodes(self.active_start_node_id, node.node_id, state=EXPLORED)
            edge.visit_count = max(edge.visit_count, 1)
            if self.active_branch is not None:
                self.active_branch.state = EXPLORED
        self.topology.visit_node(node.node_id)
        self._set_branches(node, local)

        # Mark the branch pointing back toward the node we just came from as
        # EXPLORED, so the robot commits to forward exploration instead of
        # retracing the edge it just traversed (the main cause of local
        # oscillation). Genuine backtracking is handled by REROUTE / dead-end.
        if (self.active_start_node_id is not None
                and self.active_start_node_id != node.node_id
                and self.active_start_node_id in self.topology.nodes):
            prev = self.topology.nodes[self.active_start_node_id]
            back_angle = math.atan2(prev.y - node.y, prev.x - node.x)
            for b in node.branches:
                if b.state == UNTRIED:
                    delta = abs(math.atan2(math.sin(b.angle_rad - back_angle),
                                           math.cos(b.angle_rad - back_angle)))
                    if delta < math.radians(35):
                        b.state = EXPLORED

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
            # path_xy[0] is the current node; consumer drives follow_path(path_xy[1:])
            return Action(REROUTE, path_xy=path, reason='reroute to nearest untried junction')

        return Action(DONE, reason='full coverage: no untried branch reachable')

    def report_outcome(self, outcome: str) -> None:
        """Called by the node after a pilot action completes without arriving
        at a new node (wall confirmed, wedged/timeout, or explicit success).
        OUT_SUCCESS marks the branch EXPLORED and clears active_branch so the
        idempotency guard does not re-commit to it on the next update().
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
        elif outcome == OUT_SUCCESS:
            # Branch was driven to its target successfully → mark traversed so the
            # brain advances (picks a new branch / reroutes) instead of the
            # idempotency guard re-committing to it forever.
            branch.state = EXPLORED
            self.active_branch = None
