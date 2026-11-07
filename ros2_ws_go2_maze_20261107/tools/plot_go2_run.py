#!/usr/bin/env python3
"""Plot an independently recorded Gazebo route. Full walls are evaluation-only."""
import argparse
import csv
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.patches import Circle
import numpy as np

from tugbot_maze.maze_sim import load_segments

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('run_dir', type=Path)
parser.add_argument('--output', type=Path)
args = parser.parse_args()
status = json.loads((args.run_dir / 'status.json').read_text())
replay = json.loads((args.run_dir / 'replay_audit.json').read_text())
with (args.run_dir / 'ground_truth.csv').open() as source:
    rows = list(csv.DictReader(source))
xy = np.array([[float(r['map_x']), float(r['map_y'])] for r in rows])
times = np.array([float(r['sim_time']) for r in rows])
times -= times[0]
tilts = np.array([math.degrees(float(r['tilt'])) for r in rows])
accepted = status['success'] and replay['success']
color = '#16845c' if accepted else '#bb5227'
fig = plt.figure(figsize=(12, 8), facecolor='white', layout='constrained')
grid = fig.add_gridspec(2, 2, width_ratios=[1.65, 1], height_ratios=[3, 1])
ax = fig.add_subplot(grid[:, 0])
ax.set_facecolor('#f5f7fa')
wall_segments = np.array(load_segments()).reshape(-1, 2, 2)
ax.add_collection(LineCollection(wall_segments, colors='#293846', linewidths=5, capstyle='butt'))
route = xy[::10]
ax.plot(route[:, 0], route[:, 1], color='#167bc1', linewidth=1.6, alpha=0.8, label='Actual Gazebo route')
ax.scatter(*xy[0], s=60, marker='o', c='#167bc1', zorder=5, label='Start')
ax.scatter(*xy[-1], s=120, marker='*', c=color, zorder=5, label='Final position')
ax.add_patch(Circle((21.072562, 18.083566), 1.2, color='#1c9f70', alpha=0.14))
ax.text(21.1, 19.5, 'EXIT', color='#16845c', ha='center', fontsize=9, weight='bold')
ax.set(xlim=(-1, 23), ylim=(-1.5, 20), xlabel='Map x (m)', ylabel='Map y (m)', aspect='equal')
ax.grid(alpha=0.12)
ax.legend(loc='lower right', framealpha=0.96, fontsize=9)
ax.set_title('Full wall geometry shown for evaluation only', loc='left', fontsize=10, color='#526373', pad=12)
info = fig.add_subplot(grid[0, 1])
info.axis('off')
info.text(0, 0.95, 'ACCEPTED' if accepted else 'NOT ACCEPTED', color=color, fontsize=22, weight='bold')
lines = [
    f"Simulation time     {status['elapsed_sim_s']:.1f} s",
    f"Actual route        {status['route_length_m']:.1f} m",
    f"Exit distance       {status['exit_distance_m']:.3f} m / 1.2 m",
    f"Final speed         {status['speed_m_s']:.4f} m/s",
    f"Raw pose samples    {replay['samples']:,}",
    f"Swept collisions    {replay['collision_samples']}",
    f"Illegal exits       {status['exterior_samples']}",
    f"Maximum tilt        {status['max_tilt_deg']:.2f} deg",
    f"Minimum body height {status['min_height']:.3f} m",
]
info.text(0, 0.79, '\n\n'.join(lines), va='top', family='monospace', fontsize=10, color='#293846')
chart = fig.add_subplot(grid[1, 1])
chart.plot(times[::5], tilts[::5], color='#167bc1', linewidth=0.8)
chart.axhline(math.degrees(0.1), color='#bb5227', linewidth=1, linestyle='--', label='Geometry envelope limit')
chart.set(xlabel='Simulation time (s)', ylabel='Total tilt (deg)', ylim=(0, 6.3))
chart.spines[['top', 'right']].set_visible(False)
chart.grid(alpha=0.16)
chart.legend(fontsize=7, loc='upper right')
fig.suptitle('Unitree Go2  |  Autonomous latest-maze run', fontsize=19, weight='bold', color='#203040')
output = args.output or args.run_dir / 'go2-maze-result.png'
output.parent.mkdir(parents=True, exist_ok=True)
fig.savefig(output, dpi=160, facecolor='white')
print(output)
