#!/usr/bin/env python3
"""Physical Go2 motion ladder and real sensor smoke test."""
import json
import math
from pathlib import Path
import sys
from time import monotonic

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, LaserScan, PointCloud2, Image
from std_msgs.msg import Float64


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))


class Check(Node):
    def __init__(self):
        super().__init__('go2_motion_check')
        self.odom=None
        self.joints=None
        self.scan=None
        self.cloud=None
        self.image=None
        self.targets={}
        for leg in ('LF','RF','LH','RH'):
            for axis in ('HAA','HFE','KFE'):
                joint=f'{leg}_{axis}'
                self.create_subscription(Float64,f'/model/go2/joint/{joint}/cmd_pos',
                    lambda msg,joint=joint:self.targets.__setitem__(joint,msg.data),10)
        self.publisher=self.create_publisher(Twist,'/cmd_vel',10)
        for cls,topic,field in ((Odometry,'/odom','odom'),(JointState,'/joint_states','joints'),
                               (LaserScan,'/scan','scan'),(PointCloud2,'/lidar/points','cloud'),
                               (Image,'/camera/front/image','image')):
            self.create_subscription(cls,topic,lambda msg,field=field:setattr(self,field,msg),10)

    def time(self):
        stamp=self.odom.header.stamp
        return stamp.sec+stamp.nanosec*1e-9

    def run(self,v,w,seconds,publish=True):
        if self.odom is None:
            deadline=monotonic()+45
            while self.odom is None and monotonic()<deadline:
                rclpy.spin_once(self,timeout_sec=0.1)
            assert self.odom is not None,'No odometry'
        begin=self.time()
        deadline=monotonic()+seconds*8+10
        while self.time()-begin<seconds and monotonic()<deadline:
            if publish:
                message=Twist()
                message.linear.x=float(v)
                message.angular.z=float(w)
                self.publisher.publish(message)
            rclpy.spin_once(self,timeout_sec=0.025)
        assert self.time()-begin>=seconds,'Simulation clock did not progress'
        pose=self.odom.pose.pose
        record=dict(t=self.time(),x=pose.position.x,y=pose.position.y,z=pose.position.z,
                    yaw=yaw(pose.orientation))
        if self.joints is not None and self.targets:
            actual=dict(zip(self.joints.name,self.joints.position))
            record['joint_error']={j:round(self.targets[j]-actual[j],4)
                                   for j in self.targets if j in actual}
        print(json.dumps(record),flush=True)
        assert pose.position.z>0.26,record
        return record


def main():
    rclpy.init()
    node=Check()
    try:
        stand=node.run(0,0,6)
        assert 0.28<stand['z']<0.39,stand
        assert node.joints is not None and len(node.joints.name)==12,'Missing joint feedback'
        assert node.scan is not None and len(node.scan.ranges)==900,'Missing projected scan'
        assert node.cloud is not None and node.cloud.width*node.cloud.height>=1800*16,'Missing 3D lidar'
        assert node.image is not None and (node.image.width,node.image.height)==(720,540),'Missing camera'
        print('SENSORS PASS | 12 joints | 900 scan bins | 3D lidar | 720x540 camera',flush=True)
        forward=node.run(0.2,0,6)
        progress=(forward['x']-stand['x'])*math.cos(stand['yaw'])+(forward['y']-stand['y'])*math.sin(stand['yaw'])
        print(f"FORWARD_GAIN={progress/1.2:.3f}",flush=True)
        before=node.run(0,0,3)
        turned=node.run(0,0.35,7)
        rotation=math.remainder(turned['yaw']-before['yaw'],math.tau)
        print(f"TURN_GAIN={rotation/2.45:.3f}",flush=True)
        node.run(0,0,3)
        node.run(0.15,0,1)
        stopped=node.run(0,0,3,publish=False)
        speed=node.odom.twist.twist.linear
        assert math.hypot(speed.x,speed.y,speed.z)<0.03,'Command timeout did not stop the robot'
        assert progress > 0.45, f'Forward tracking too slow: {progress:.3f} m'
        assert rotation > 1.1, f'Turn response too slow: {rotation:.3f} rad'
        print(f'GO2 MOTION PASS | forward={progress:.3f} m | turn={rotation:.3f} rad | watchdog stopped',flush=True)
    finally:
        for _ in range(5):
            node.publisher.publish(Twist())
            rclpy.spin_once(node,timeout_sec=0.02)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__=='__main__':
    main()
