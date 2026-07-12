import math

from tugbot_maze.footprint import SCAN_OFFSET_X
from tugbot_maze.radar_occupancy import RadarOccupancyGrid

# 8x8 m world perimeter centerlines (same bbox convention as the old renderer's tests).
_PERIM = [(0.0, 0.0, 8.0, 0.0), (8.0, 0.0, 8.0, 8.0),
          (8.0, 8.0, 0.0, 8.0), (0.0, 8.0, 0.0, 0.0)]


def _cell_at(g, x, y):
    ix = int((x - g.info.origin.position.x) / g.info.resolution)
    iy = int((y - g.info.origin.position.y) / g.info.resolution)
    return g.data[iy * g.info.width + ix]


def test_grid_metadata_from_perimeter_bbox():
    r = RadarOccupancyGrid(_PERIM, resolution=0.1, margin_m=0.5)
    g = r.to_occupancy_grid()
    assert g.header.frame_id == 'map'
    assert g.info.resolution == 0.1
    assert (g.info.width, g.info.height) == (90, 90)   # [0,8]+/-0.5 -> 9 m -> 90 cells
    assert (g.info.origin.position.x, g.info.origin.position.y) == (-0.5, -0.5)
    assert g.info.origin.orientation.w == 1.0
    assert len(g.data) == 90 * 90


def test_unobserved_is_unknown():
    r = RadarOccupancyGrid(_PERIM, resolution=0.1)
    assert set(r.to_occupancy_grid().data) == {-1}     # nothing integrated -> all unknown


def test_hit_marks_endpoint_occupied():
    # Robot at (4,4) facing +x; wall hit straight ahead at range 2.0.
    r = RadarOccupancyGrid(_PERIM, resolution=0.1)
    r.integrate_scan((4.0, 4.0, 0.0), [2.0], angle_min=0.0, angle_inc=0.1)
    ex = 4.0 + SCAN_OFFSET_X + 2.0
    assert _cell_at(r.to_occupancy_grid(), ex, 4.0) == 100


def test_ray_path_marks_free():
    r = RadarOccupancyGrid(_PERIM, resolution=0.1)
    r.integrate_scan((4.0, 4.0, 0.0), [2.0], angle_min=0.0, angle_inc=0.1)
    # ~1 m in front of the sensor, well before the endpoint -> free
    assert _cell_at(r.to_occupancy_grid(), 4.0 + SCAN_OFFSET_X + 1.0, 4.0) == 0


def test_projection_applies_pose_rotation():
    # Facing +y (yaw=pi/2); hit at range 2 -> endpoint ~ (4, 4 + SCAN_OFFSET_X + 2).
    r = RadarOccupancyGrid(_PERIM, resolution=0.1)
    r.integrate_scan((4.0, 4.0, math.pi / 2), [2.0], angle_min=0.0, angle_inc=0.1)
    assert _cell_at(r.to_occupancy_grid(), 4.0, 4.0 + SCAN_OFFSET_X + 2.0) == 100


def test_accumulation_is_monotonic_and_clamped():
    r = RadarOccupancyGrid(_PERIM, resolution=0.1, l_occ=0.85, l_clamp=5.0)
    ex = 4.0 + SCAN_OFFSET_X + 2.0
    for _ in range(100):                                # hammer the same endpoint
        r.integrate_scan((4.0, 4.0, 0.0), [2.0], angle_min=0.0, angle_inc=0.1)
    iy = int((4.0 - r.y0) / r.resolution)
    ix = int((ex - r.x0) / r.resolution)
    assert abs(float(r._logodds[iy, ix]) - r.l_clamp) <= 1e-4   # saturated at clamp, no overflow
    assert r.to_occupancy_grid().data[iy * r.width + ix] == 100


def test_invalid_beams_produce_no_update():
    r = RadarOccupancyGrid(_PERIM, resolution=0.1, usable_range_m=8.0)
    r.integrate_scan((4.0, 4.0, 0.0),
                     [float('inf'), float('nan'), -1.0, 0.0, 100.0],  # inf/nan/neg/zero/over-range
                     angle_min=0.0, angle_inc=0.1)
    assert set(r.to_occupancy_grid().data) == {-1}


def test_oblique_hit_marks_endpoint_occupied_in_one_scan():
    # An oblique beam must mark its endpoint occupied in a single scan -- the free
    # ray-march must not decrement the endpoint's own cell.
    r = RadarOccupancyGrid(_PERIM, resolution=0.1)
    ang = 0.6
    r.integrate_scan((4.0, 4.0, 0.0), [3.0], angle_min=ang, angle_inc=0.1)
    ex = 4.0 + SCAN_OFFSET_X + 3.0 * math.cos(ang)
    ey = 4.0 + 3.0 * math.sin(ang)
    assert _cell_at(r.to_occupancy_grid(), ex, ey) == 100


def test_occupancy_values_are_three_valued():
    r = RadarOccupancyGrid(_PERIM, resolution=0.1)
    r.integrate_scan((4.0, 4.0, 0.0), [2.0, 3.0], angle_min=-0.2, angle_inc=0.2)
    assert set(r.to_occupancy_grid().data) <= {-1, 0, 100}
