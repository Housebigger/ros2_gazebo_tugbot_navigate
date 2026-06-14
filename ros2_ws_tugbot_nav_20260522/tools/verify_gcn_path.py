#!/usr/bin/env python3
"""Verify the corridor_navigator BFS path against the source maze image.

For each corridor segment we sample points along start->end (in map frame),
convert to image pixels, and check that a robot-radius clearance disk around
each sample is free (white) in the binary maze. Reports any collision.
"""
import sys, math
import numpy as np
from PIL import Image

sys.path.insert(0, 'src/tugbot_maze')
from tugbot_maze.corridor_navigator import MAZE_CORRIDORS

IMG = 'src/tugbot_maze/assets/maze_20260528.png'
W = H = 360
XMIN, XMAX = -6.0, 6.0
YMIN, YMAX = -6.0, 6.0
SCALE = 2.0
MAP_OFF_X = 11.011281
MAP_OFF_Y = 9.02507
ROBOT_R = 0.35           # robot radius, map metres
WALL_BLACK_THRESH = 128

im = np.asarray(Image.open(IMG).convert('L'))   # (H, W), 0=black wall
# free = white (>thresh)

def map_to_px(mx, my):
    # map -> original world
    ox = (mx - MAP_OFF_X) / SCALE
    oy = (my - MAP_OFF_Y) / SCALE
    px = (ox - XMIN) / (XMAX - XMIN) * (W - 1)
    py = (YMAX - oy) / (YMAX - YMIN) * (H - 1)
    return px, py

# clearance radius in pixels: ROBOT_R map -> original -> px
r_orig = ROBOT_R / SCALE
r_px = r_orig / (XMAX - XMIN) * (W - 1)

def is_clear(mx, my, r_px):
    px, py = map_to_px(mx, my)
    ipx, ipy = int(round(px)), int(round(py))
    rr = int(math.ceil(r_px))
    worst = 255
    hit = None
    for dy in range(-rr, rr + 1):
        for dx in range(-rr, rr + 1):
            if dx*dx + dy*dy > rr*rr:
                continue
            x, y = ipx + dx, ipy + dy
            if 0 <= x < W and 0 <= y < H:
                v = im[y, x]
                if v < worst:
                    worst = v
                    hit = (x, y)
            else:
                # outside image = exterior; treat as wall unless near exit gap
                worst = 0
                hit = (x, y)
    return worst >= WALL_BLACK_THRESH, worst, hit

print(f'robot clearance radius = {r_px:.1f} px ({ROBOT_R} m map)')
print(f'corridors: {len(MAZE_CORRIDORS)}')
any_fail = False
# Use a slightly reduced clearance for sampling so we measure true corridor fit
for c in MAZE_CORRIDORS:
    sx, sy = c['start']; ex, ey = c['end']
    dist = math.hypot(ex - sx, ey - sy)
    n = max(2, int(dist / 0.25) + 1)
    min_clear_worst = 255
    worst_pt = None
    collisions = 0
    for i in range(n + 1):
        t = i / n
        mx = sx + (ex - sx) * t
        my = sy + (ey - sy) * t
        ok, worst, hit = is_clear(mx, my, r_px)
        if worst < min_clear_worst:
            min_clear_worst = worst
            worst_pt = (round(mx,2), round(my,2), hit)
        if not ok:
            collisions += 1
    status = 'OK' if collisions == 0 else f'*** {collisions}/{n+1} COLLIDE ***'
    if collisions:
        any_fail = True
    print(f"C{c['id']:>2} {c['dir']} ({sx:>5.1f},{sy:>5.1f})->({ex:>5.1f},{ey:>5.1f}) "
          f"len={c['len']:.1f}  min_pixel={min_clear_worst:>3}  worst@{worst_pt}  {status}")

print()
print('RESULT:', 'PATH HAS WALL COLLISIONS' if any_fail else 'PATH CLEAR (robot radius fits)')
