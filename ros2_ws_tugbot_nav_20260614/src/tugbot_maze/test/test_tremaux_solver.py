from tugbot_maze.maze_topology import MazeTopology, IN_PROGRESS


def test_topoedge_has_visit_count_default_zero():
    topo = MazeTopology()
    a = topo.find_or_create_node(0.0, 0.0)
    b = topo.find_or_create_node(2.0, 0.0)
    edge = topo.connect_nodes(a.node_id, b.node_id, state=IN_PROGRESS)
    assert edge.visit_count == 0
    edge.visit_count = 2
    assert topo.edges[edge.edge_id].visit_count == 2
