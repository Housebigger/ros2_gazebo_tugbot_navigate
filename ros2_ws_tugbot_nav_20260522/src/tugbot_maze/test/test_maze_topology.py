import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from tugbot_maze.maze_topology import (
    BLOCKED,
    DEAD_END,
    EXPLORED,
    IN_PROGRESS,
    UNTRIED,
    BranchOption,
    MazeTopology,
    BLACKLISTED,
    BLOCKED_NAV2,
    TRUE_DEAD_END,
)


def test_new_junction_tracks_untried_branch_directions():
    topology = MazeTopology(junction_merge_radius_m=0.5)

    node = topology.find_or_create_node(0.0, 0.0, node_type='junction')
    topology.set_branch_options(
        node.node_id,
        [
            BranchOption(angle_rad=0.0, target_xy=(1.0, 0.0)),
            BranchOption(angle_rad=math.pi / 2.0, target_xy=(0.0, 1.0)),
            BranchOption(angle_rad=-math.pi / 2.0, target_xy=(0.0, -1.0)),
        ],
    )

    assert len(node.branches) == 3
    assert {branch.state for branch in node.branches} == {UNTRIED}


def test_find_or_create_node_reuses_nearby_junctions_and_creates_distant_nodes():
    topology = MazeTopology(junction_merge_radius_m=0.75)

    first = topology.find_or_create_node(1.0, 2.0, node_type='junction')
    nearby = topology.find_or_create_node(1.3, 2.2, node_type='junction')
    distant = topology.find_or_create_node(2.5, 2.0, node_type='junction')

    assert nearby.node_id == first.node_id
    assert distant.node_id != first.node_id
    assert len(topology.nodes) == 2


def test_dead_end_marks_incoming_edge_and_backtracks_to_recent_untried_junction():
    topology = MazeTopology(junction_merge_radius_m=0.5)
    a = topology.find_or_create_node(0.0, 0.0, node_type='junction')
    b = topology.find_or_create_node(1.0, 0.0, node_type='junction')
    c = topology.find_or_create_node(2.0, 0.0, node_type='dead_end')

    topology.set_branch_options(
        a.node_id,
        [
            BranchOption(angle_rad=0.0, target_xy=(1.0, 0.0)),
            BranchOption(angle_rad=math.pi / 2.0, target_xy=(0.0, 1.0)),
        ],
    )
    topology.set_branch_options(
        b.node_id,
        [
            BranchOption(angle_rad=0.0, target_xy=(2.0, 0.0)),
            BranchOption(angle_rad=-math.pi / 2.0, target_xy=(1.0, -1.0)),
        ],
    )
    edge_ab = topology.connect_nodes(a.node_id, b.node_id, state=EXPLORED)
    edge_bc = topology.connect_nodes(b.node_id, c.node_id, state=IN_PROGRESS)
    topology.visit_node(a.node_id)
    topology.visit_node(b.node_id)
    topology.visit_node(c.node_id)

    topology.mark_dead_end(c.node_id, incoming_edge_id=edge_bc.edge_id)
    backtrack = topology.next_backtrack_target(c.node_id)

    assert topology.nodes[c.node_id].node_type == 'dead_end'
    assert topology.edges[edge_bc.edge_id].state == DEAD_END
    assert topology.edges[edge_ab.edge_id].state == EXPLORED
    assert backtrack is not None
    assert backtrack.node_id == b.node_id


def test_backtracking_skips_exhausted_junctions():
    topology = MazeTopology(junction_merge_radius_m=0.5)
    a = topology.find_or_create_node(0.0, 0.0, node_type='junction')
    b = topology.find_or_create_node(1.0, 0.0, node_type='junction')
    c = topology.find_or_create_node(2.0, 0.0, node_type='dead_end')

    topology.set_branch_options(a.node_id, [BranchOption(angle_rad=math.pi / 2.0, target_xy=(0.0, 1.0))])
    topology.set_branch_options(b.node_id, [BranchOption(angle_rad=0.0, target_xy=(2.0, 0.0))])
    for branch in b.branches:
        branch.state = DEAD_END
    topology.visit_node(a.node_id)
    topology.visit_node(b.node_id)
    topology.visit_node(c.node_id)

    backtrack = topology.next_backtrack_target(c.node_id)

    assert backtrack is not None
    assert backtrack.node_id == a.node_id


def test_choose_next_branch_prefers_untried_branch_biased_toward_exit():
    topology = MazeTopology(junction_merge_radius_m=0.5, exit_bias_weight=1.0)
    node = topology.find_or_create_node(0.0, 0.0, node_type='junction')
    topology.set_branch_options(
        node.node_id,
        [
            BranchOption(angle_rad=math.pi, target_xy=(-1.0, 0.0)),
            BranchOption(angle_rad=0.0, target_xy=(1.0, 0.0)),
            BranchOption(angle_rad=math.pi / 2.0, target_xy=(0.0, 1.0), state=DEAD_END),
            BranchOption(angle_rad=-math.pi / 2.0, target_xy=(0.0, -1.0), state=BLOCKED),
        ],
    )

    chosen = topology.choose_next_branch(node.node_id, exit_xy=(4.0, 0.0))

    assert chosen is not None
    assert chosen.target_xy == (1.0, 0.0)
    assert chosen.state == UNTRIED


def test_choose_next_branch_returns_none_when_all_branches_terminal():
    topology = MazeTopology(junction_merge_radius_m=0.5)
    node = topology.find_or_create_node(0.0, 0.0, node_type='junction')
    topology.set_branch_options(
        node.node_id,
        [
            BranchOption(angle_rad=0.0, target_xy=(1.0, 0.0), state=EXPLORED),
            BranchOption(angle_rad=math.pi, target_xy=(-1.0, 0.0), state=DEAD_END),
        ],
    )

    assert topology.choose_next_branch(node.node_id, exit_xy=(4.0, 0.0)) is None
