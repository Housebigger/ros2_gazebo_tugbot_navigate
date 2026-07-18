import json


def test_observe_dedups_and_refreshes():
    from tugbot_maze.junction_log import JunctionLog
    jl = JunctionLog()
    e1, new1 = jl.observe((3, 4), ['N', 'E', 'W'], 2.0)
    assert new1 and e1['discovery_index'] == 1 and e1['exit_count'] == 3
    e2, new2 = jl.observe((3, 4), ['N', 'E', 'W', 'S'], 5.0)   # refresh exits, keep order/time
    assert not new2 and e2['discovery_index'] == 1 and e2['first_seen_s'] == 2.0
    assert e2['exit_count'] == 4 and jl.count == 1


def test_visit_counts_and_reflects_in_record():
    from tugbot_maze.junction_log import JunctionLog
    jl = JunctionLog()
    jl.visit((3, 4)); jl.visit((3, 4))         # 2 entries before record
    e, _ = jl.observe((3, 4), ['N', 'E', 'S'], 1.0)
    assert e['visits'] == 2
    jl.visit((3, 4))
    assert jl._j[(3, 4)]['visits'] == 3


def test_discovery_order_and_to_dict():
    from tugbot_maze.junction_log import JunctionLog
    jl = JunctionLog()
    jl.observe((1, 1), ['N', 'E', 'S'], 1.0)
    jl.observe((2, 2), ['N', 'E', 'W'], 2.0)
    d = jl.to_dict()
    assert d['junction_count'] == 2
    assert [j['cell'] for j in d['junctions']] == [[1, 1], [2, 2]]


def test_flush_writes_json(tmp_path):
    from tugbot_maze.junction_log import JunctionLog
    jl = JunctionLog()
    jl.observe((3, 4), ['N', 'E', 'W'], 1.0)
    path = str(tmp_path / 'sub' / 'junctions.json')
    jl.flush(path)
    data = json.loads(open(path).read())
    assert data['junction_count'] == 1 and data['junctions'][0]['cell'] == [3, 4]


def test_update_junctions_records_and_counts():
    from tugbot_maze.flood_fill_brain import FloodFillBrain
    from tugbot_maze.junction_log import JunctionLog, update_junctions
    b = FloodFillBrain()
    b.mark((5, 5), 'N', True)                  # (5,5): open S,E,W -> junction (3 exits)
    jl = JunctionLog()
    prev, j = update_junctions(jl, b, (5, 5), None, {(5, 5)}, 1.0)
    assert prev == (5, 5) and j is not None and j['exit_count'] == 3
    assert jl.count == 1 and jl._j[(5, 5)]['visits'] == 1
    prev, j2 = update_junctions(jl, b, (5, 5), prev, {(5, 5)}, 1.1)   # stationary: no re-record/visit
    assert j2 is None and jl._j[(5, 5)]['visits'] == 1


def test_update_junctions_ignores_non_junction_and_unsensed():
    from tugbot_maze.flood_fill_brain import FloodFillBrain
    from tugbot_maze.junction_log import JunctionLog, update_junctions
    jl = JunctionLog()
    b = FloodFillBrain()
    b.mark((5, 5), 'N', True); b.mark((5, 5), 'E', True)    # only 2 exits -> not a junction
    _, j = update_junctions(jl, b, (5, 5), None, {(5, 5)}, 1.0)
    assert j is None and jl.count == 0
    b2 = FloodFillBrain(); b2.mark((5, 5), 'N', True)        # junction, but NOT sensed
    _, j2 = update_junctions(jl, b2, (5, 5), None, set(), 1.0)
    assert j2 is None and jl.count == 0
