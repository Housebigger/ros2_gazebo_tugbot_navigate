#!/usr/bin/env python3
"""Read-only checks of real camera streams during a standing or moving run."""
import argparse
from bisect import bisect_left
from collections import deque
import json
import math
from pathlib import Path
import time

import numpy as np
from PIL import Image
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image as RosImage
from tf2_msgs.msg import TFMessage


def stamp(message):
    return message.header.stamp.sec + message.header.stamp.nanosec * 1e-9


def rotation(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def check_image_times(times, end_sim_time, begin_sim_time):
    assert len(times) >= 30 and np.all(np.diff(times) > 0), 'No continuous sim-time images'
    assert -.2 <= end_sim_time-times[-1] <= .5, 'Camera stream is stale relative to final odometry'
    assert times[0] <= begin_sim_time+.5, 'Camera stream does not cover the observation window'
    assert times[-1]-times[0] > 3 and np.max(np.diff(times)) < 1, 'Camera stream stalled'


def scene_thumbnail(pixels):
    # Average 24x24 regions to suppress sensor noise before measuring scene change.
    h, w = pixels.shape[0]//24, pixels.shape[1]//24
    return pixels[:h*24, :w*24, :3].reshape(h, 24, w, 24, 3).mean(axis=(1, 3))


class CameraCheck(Node):
    def __init__(self):
        super().__init__('go2_camera_check', parameter_overrides=[rclpy.Parameter('use_sim_time', value=True)])
        self.images = {}
        self.infos = {}
        self.times = {name: [] for name in ('front', 'chase', 'overview')}
        self.samples = {name: [] for name in self.times}
        self.odom = []
        self.chase = []
        self.overview = None
        self.sync_spreads = deque(maxlen=10000)
        self.subscriptions_owned = []
        for name in self.times:
            self.subscriptions_owned.extend([
                self.create_subscription(RosImage, f'/camera/{name}/image',
                    lambda msg, name=name: self.image(msg, name), qos_profile_sensor_data),
                self.create_subscription(CameraInfo, f'/camera/{name}/camera_info',
                    lambda msg, name=name: self.infos.__setitem__(name, msg), qos_profile_sensor_data)])
        self.subscriptions_owned.extend([
            self.create_subscription(Odometry, '/odom', self.odom.append, 30),
            self.create_subscription(TFMessage, '/tf', self.transforms, 50),
            self.create_subscription(TFMessage, '/tf_static', self.transforms,
                QoSProfile(depth=30, durability=DurabilityPolicy.TRANSIENT_LOCAL))])

    def image(self, msg, name):
        self.images[name] = msg
        self.times[name].append(stamp(msg))
        if len(self.times[name]) % 20 == 1:
            # Large-scale image change, rather than sensor noise, indicates motion.
            channels = 4 if 'a8' in msg.encoding else 3
            pixels = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)
            pixels = pixels[:, :msg.width*channels].reshape(msg.height, msg.width, channels)
            self.samples[name].append(scene_thumbnail(pixels))
        if len(self.images) == 3:
            times = [stamp(m) for m in self.images.values()]
            self.sync_spreads.append(max(times)-min(times))

    def transforms(self, msg):
        for transform in msg.transforms:
            if transform.child_frame_id == 'camera_chase_optical':
                self.chase.append(transform)
            elif transform.child_frame_id == 'camera_overview_optical':
                self.overview = transform

    def save_images(self, destination, suffix):
        for name, msg in self.images.items():
            encoding = {'rgb8': ('RGB', 'RGB'), 'bgr8': ('RGB', 'BGR'),
                        'rgba8': ('RGBA', 'RGBA'), 'bgra8': ('RGBA', 'BGRA')}
            mode, raw = encoding[msg.encoding]
            Image.frombytes(mode, (msg.width, msg.height), bytes(msg.data),
                'raw', raw, msg.step, 1).save(destination/f'{name}-{suffix}.png')

    def evaluate(self, require_motion, begin_sim_time, follow_yaw=False):
        assert self.odom and self.chase and self.overview, 'Missing camera TF or odometry'
        dimensions = {'front': (720, 540), 'chase': (1280, 720), 'overview': (1024, 1024)}
        result = {'streams': {}, 'require_motion': require_motion,
                  'chase_mode': 'follow_yaw' if follow_yaw else 'fixed_world_direction'}
        for name, size in dimensions.items():
            assert name in self.images and name in self.infos, f'Missing {name} image/CameraInfo'
            image, info, times = self.images[name], self.infos[name], np.array(self.times[name])
            assert (image.width, image.height) == (info.width, info.height) == size
            assert image.header.frame_id == info.header.frame_id, f'{name}: inconsistent optical frame'
            check_image_times(times, stamp(self.odom[-1]), begin_sim_time)
            if name != 'front':
                hfov = .72 if name == 'chase' else 1.
                assert abs(info.k[0]-size[0]/(2*math.tan(hfov/2))) < 1
                assert abs(info.k[2]-size[0]/2) <= 1 and abs(info.k[5]-size[1]/2) <= 1
            changes = [float(np.mean(np.abs(s-self.samples[name][0]))) for s in self.samples[name][1:]]
            result['streams'][name] = dict(count=len(times), dimensions=list(size),
                frame=image.header.frame_id, simulated_hz=float((len(times)-1)/(times[-1]-times[0])),
                max_gap_sim_s=float(np.max(np.diff(times))), last_stamp=float(times[-1]),
                intrinsics=list(info.k), max_image_change=max(changes, default=0))
            result['streams'][name]['final_age_sim_s'] = float(stamp(self.odom[-1])-times[-1])
        spread = float(np.percentile(self.sync_spreads, 99))
        assert spread < .3, f'Images do not share a fresh simulation clock: {spread}'
        odom_times = [stamp(o) for o in self.odom]
        try:
            check_image_times(np.array([stamp(tf) for tf in self.chase]),
                              odom_times[-1], begin_sim_time)
        except AssertionError as error:
            raise AssertionError(f'Chase TF: {error}') from error
        positions, aims, horizon, world_direction_errors, matches = [], [], [], [], 0
        for tf in self.chase:
            t = stamp(tf)
            index = bisect_left(odom_times, t)
            candidates = [i for i in (index-1, index) if 0 <= i < len(odom_times)]
            nearest = min(candidates, key=lambda i: abs(odom_times[i]-t))
            if abs(odom_times[nearest]-t) > .05:
                continue
            assert tf.header.frame_id == 'odom'
            pose = self.odom[nearest].pose.pose
            body = np.array([pose.position.x, pose.position.y, pose.position.z])
            yaw = math.atan2(rotation(pose.orientation)[1, 0], rotation(pose.orientation)[0, 0])
            if not follow_yaw:
                yaw = 0.
            expected = body + np.array([-1.2*math.cos(yaw)+.9*math.sin(yaw),
                                        -1.2*math.sin(yaw)-.9*math.cos(yaw), 3.5])
            v = tf.transform.translation
            actual = np.array([v.x, v.y, v.z])
            optical = rotation(tf.transform.rotation)
            if not follow_yaw:
                fixed_direction = np.array([1.2, .9, -3.6])
                fixed_direction /= np.linalg.norm(fixed_direction)
                world_direction_errors.append(float(np.linalg.norm(optical[:, 2]-fixed_direction)))
            ray = body + np.array([0, 0, -.1])-actual
            positions.append(float(np.linalg.norm(actual-expected)))
            aims.append(float(math.acos(np.clip(np.dot(optical[:, 2], ray/np.linalg.norm(ray)), -1, 1))))
            horizon.append(float(abs(optical[2, 0])))
            matches += 1
        assert matches >= 100, 'Not enough time-aligned chase TF / odometry samples'
        assert max(positions) < .08, f'Camera is not following Go2: {max(positions)} m'
        assert max(aims) < .03 and max(horizon) < 1e-5, 'Chase aim/horizon drifted'
        if not follow_yaw:
            assert max(world_direction_errors) < 1e-6, 'Camera rotated instead of keeping its world direction'
        v = self.overview.transform.translation
        assert self.overview.header.frame_id == 'odom'
        assert np.allclose([v.x, v.y, v.z], [0, 0, 24])
        assert np.allclose(rotation(self.overview.transform.rotation), np.diag([1, -1, -1]))
        points = np.array([[o.pose.pose.position.x, o.pose.pose.position.y] for o in self.odom])
        yaws = np.unwrap([math.atan2(rotation(o.pose.pose.orientation)[1, 0], rotation(o.pose.pose.orientation)[0, 0]) for o in self.odom])
        travel = float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum())
        turn = float(np.ptp(yaws))
        if require_motion:
            assert travel > 2 and turn > .4, f'Insufficient walking/turning: {travel}, {turn}'
            assert result['streams']['front']['max_image_change'] > 2, 'Forward scene did not change'
            assert result['streams']['chase']['max_image_change'] > 2, 'Chase scene did not change'
        result.update(time_aligned_tf_samples=matches, max_follow_error_m=max(positions),
            max_aim_error_rad=max(aims), max_horizon_vertical_component=max(horizon),
            p99_image_stamp_spread_sim_s=spread, travel_m=travel, yaw_range_rad=turn,
            elapsed_sim_s=odom_times[-1]-odom_times[0], passed=True)
        if world_direction_errors:
            result['max_world_optical_direction_error'] = max(world_direction_errors)
        return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path, help='New evidence directory (must be empty)')
    parser.add_argument('--seconds', type=float, default=90, help='Observation duration in simulation seconds')
    parser.add_argument('--wall-timeout', type=float, default=360)
    parser.add_argument('--require-motion', action='store_true', help='Also require real walking and turning')
    parser.add_argument('--follow-yaw', action='store_true', help='Check optional yaw-following mode instead of the default fixed world direction')
    args = parser.parse_args()
    if args.output.exists() and any(args.output.iterdir()):
        parser.error('Evidence directory must be empty')
    args.output.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = CameraCheck()
    begin = None
    deadline = time.monotonic()+args.wall_timeout
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
            if len(node.images) != 3 or not node.odom:
                continue
            if begin is None:
                begin = stamp(node.odom[-1])
                node.save_images(args.output, 'start')
            if stamp(node.odom[-1])-begin >= args.seconds:
                break
        assert begin is not None and stamp(node.odom[-1])-begin >= args.seconds, 'Timed out waiting for live cameras/simulation'
        node.save_images(args.output, 'end')
        result = node.evaluate(args.require_motion, begin, args.follow_yaw)
        (args.output/'camera_check.json').write_text(json.dumps(result, indent=2)+'\n')
        print(json.dumps(result, indent=2), flush=True)
    except Exception as error:
        (args.output/'camera_check.json').write_text(json.dumps({'passed': False, 'error': str(error)}, indent=2)+'\n')
        raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
