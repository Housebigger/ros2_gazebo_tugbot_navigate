#!/usr/bin/env python3
"""Recheck raw Gazebo trajectory with the currently installed Go2 footprint.

This evaluator reads the full wall map. It is not imported by navigation.
"""
import argparse
import csv
import json
import math
from pathlib import Path

from tugbot_maze.footprint import FOOT_X_FRONT, FOOT_X_REAR, FOOT_HALF_W, FOOTPRINT_MAX_TILT_RAD
from tugbot_maze.maze_sim import MazeSim, load_segments


def replay(path):
    oracle = MazeSim(load_segments(), (0.0, 0.0), 0.0)
    previous = None
    samples = swept = collisions = 0
    maximum_tilt = 0.0
    first_collision = None
    for row in csv.DictReader(path.open()):
        pose = tuple(float(row[name]) for name in ('map_x', 'map_y', 'yaw'))
        maximum_tilt = max(maximum_tilt, float(row['tilt']))
        x, y, yaw = pose
        px, py, old_yaw = previous or pose
        turn = math.remainder(yaw - old_yaw, math.tau)
        count = max(1, math.ceil(math.hypot(x-px, y-py)/0.01), math.ceil(abs(turn)/0.02))
        for index in range(1, count+1):
            fraction = index/count
            swept += 1
            if oracle.collides(px + fraction*(x-px), py + fraction*(y-py), old_yaw + fraction*turn):
                collisions += 1
                if first_collision is None:
                    first_collision = dict(row)
        samples += 1
        previous = pose
    return dict(samples=samples, swept_samples=swept, collision_samples=collisions,
                first_collision=first_collision, max_tilt_deg=math.degrees(maximum_tilt),
                final_pose=previous,
                footprint=[FOOT_X_REAR, FOOT_X_FRONT, FOOT_HALF_W])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run_dir', type=Path)
    parser.add_argument('--require-success', action='store_true')
    args = parser.parse_args()
    result = replay(args.run_dir / 'ground_truth.csv')
    status = json.loads((args.run_dir / 'status.json').read_text())
    result['online_success'] = status['success']
    result['tilt_envelope_valid'] = result['max_tilt_deg'] <= math.degrees(FOOTPRINT_MAX_TILT_RAD)
    result['dataset_complete'] = result['samples'] == status['samples']
    result['exit_distance_m'] = (math.hypot(result['final_pose'][0]-21.072562,
                                          result['final_pose'][1]-18.083566)
                                 if result['final_pose'] else math.inf)
    result['geometry_pass'] = bool(result['samples'] >= 100 and result['collision_samples'] == 0
                                   and result['tilt_envelope_valid'])
    result['success'] = bool(result['geometry_pass'] and status['success']
                             and result['dataset_complete'] and result['exit_distance_m'] <= 1.2)
    (args.run_dir / 'replay_audit.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))
    accepted = result['success'] if args.require_success else result['geometry_pass']
    raise SystemExit(0 if accepted else 1)


if __name__ == '__main__':
    main()
