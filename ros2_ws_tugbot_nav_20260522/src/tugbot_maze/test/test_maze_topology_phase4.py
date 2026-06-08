import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from tugbot_maze.maze_topology import (
    BLACKLISTED,
    BLOCKED,
    BLOCKED_NAV2,
    DEAD_END,
    TRUE_DEAD_END,
    BranchOption,
    MazeTopology,
)


def test_blacklisted_goal_is_not_selected_again():
    topology = MazeTopology(junction_merge_radius_m=0.5, blacklist_radius_m=0.4)
    node = topology.find_or_create_node(0.0, 0.0, node_type='junction')
    topology.set_branch_options(
        node.node_id,
        [
            BranchOption(angle_rad=0.0, target_xy=(1.0, 0.0)),
            BranchOption(angle_rad=math.pi / 2.0, target_xy=(0.0, 1.0)),
        ],
    )

    topology.blacklist_goal((1.05, 0.05), reason=BLOCKED_NAV2)
    chosen = topology.choose_next_branch(node.node_id, exit_xy=(4.0, 0.0))

    assert chosen is not None
    assert chosen.target_xy == (0.0, 1.0)
    assert node.branches[0].state == BLACKLISTED
    assert topology.blacklist[0].reason == BLOCKED_NAV2


def test_repeated_nav2_failures_block_branch_without_marking_true_dead_end():
    topology = MazeTopology(junction_merge_radius_m=0.5, max_failures_per_branch=2)
    node = topology.find_or_create_node(0.0, 0.0, node_type='junction')
    branch = BranchOption(angle_rad=0.0, target_xy=(1.0, 0.0))
    topology.set_branch_options(node.node_id, [branch])

    first_state = topology.record_branch_failure(node.node_id, branch, reason=BLOCKED_NAV2)
    second_state = topology.record_branch_failure(node.node_id, branch, reason=BLOCKED_NAV2)

    assert first_state != BLOCKED
    assert second_state == BLOCKED
    assert node.branches[0].state == BLOCKED
    assert node.branches[0].failure_reason == BLOCKED_NAV2
    assert node.node_type == 'junction'


def test_true_dead_end_failure_marks_node_and_edge_dead_end():
    topology = MazeTopology(junction_merge_radius_m=0.5)
    start = topology.find_or_create_node(0.0, 0.0, node_type='junction')
    end = topology.find_or_create_node(1.0, 0.0, node_type='dead_end')
    branch = BranchOption(angle_rad=0.0, target_xy=(1.0, 0.0))
    topology.set_branch_options(start.node_id, [branch])
    edge = topology.connect_nodes(start.node_id, end.node_id)

    topology.record_branch_failure(start.node_id, start.branches[0], reason=TRUE_DEAD_END, end_node_id=end.node_id, edge_id=edge.edge_id)

    assert start.branches[0].state == DEAD_END
    assert start.branches[0].failure_reason == TRUE_DEAD_END
    assert topology.edges[edge.edge_id].state == DEAD_END
    assert topology.nodes[end.node_id].node_type == 'dead_end'


def test_backtrack_attempt_limit_skips_repeatedly_failed_target():
    topology = MazeTopology(junction_merge_radius_m=0.5, max_backtrack_failures_per_node=2)
    a = topology.find_or_create_node(0.0, 0.0, node_type='junction')
    b = topology.find_or_create_node(1.0, 0.0, node_type='junction')
    c = topology.find_or_create_node(2.0, 0.0, node_type='dead_end')
    topology.set_branch_options(a.node_id, [BranchOption(angle_rad=math.pi / 2.0, target_xy=(0.0, 1.0))])
    topology.set_branch_options(b.node_id, [BranchOption(angle_rad=-math.pi / 2.0, target_xy=(1.0, -1.0))])
    topology.visit_node(a.node_id)
    topology.visit_node(b.node_id)
    topology.visit_node(c.node_id)

    topology.record_backtrack_failure(b.node_id)
    topology.record_backtrack_failure(b.node_id)
    target = topology.next_backtrack_target(c.node_id)

    assert target is not None
    assert target.node_id == a.node_id
    assert topology.nodes[b.node_id].backtrack_failures == 2


def test_goal_near_current_backtrack_target_is_treated_as_reached():
    topology = MazeTopology(junction_merge_radius_m=0.5, backtrack_goal_tolerance_m=0.35)
    node = topology.find_or_create_node(1.0, 1.0, node_type='junction')

    assert topology.backtrack_target_reached(node.node_id, (1.2, 1.1))
    assert not topology.backtrack_target_reached(node.node_id, (1.6, 1.0))
