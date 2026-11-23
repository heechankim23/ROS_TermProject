"""Raycast from simulated physical pose, never from AMCL's own estimate."""
import math
import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from .sim_lidar import SimLidar
from .motion import yaw_from_quat
from .common import spin_single


class TruthPose:
    def __init__(self, node, gazebo, spawn):
        self.value = None
        self.spawn = spawn
        if gazebo:
            self.sub = node.create_subscription(PoseStamped, '/sim/robot_pose', self.on_truth, 10)
        else:
            self.sub = node.create_subscription(Odometry, '/odom', self.on_odom, 10)

    def on_truth(self, m):
        self.value = (m.pose.position.x,m.pose.position.y,yaw_from_quat(m.pose.orientation),m.header.stamp)

    def on_odom(self, m):
        p=m.pose.pose
        a=math.radians(self.spawn['yaw_deg'])
        c,s=math.cos(a),math.sin(a)
        self.value=(self.spawn['x']+c*p.position.x-s*p.position.y,
                    self.spawn['y']+s*p.position.x+c*p.position.y,
                    a+yaw_from_quat(p.orientation),m.header.stamp)

    def get_stamped(self):
        return self.value


class ExtendedLidar(SimLidar):
    def __init__(self):
        super().__init__()
        self.declare_parameter('gz_enabled',True)
        self.declare_parameter('map_config_file','')
        with open(self.get_parameter('map_config_file').value,encoding='utf-8') as f:
            spawn=yaml.safe_load(f)['robot_spawn']
        self.pose=TruthPose(self,self.get_parameter('gz_enabled').value,spawn)


def main():
    rclpy.init()
    spin_single(ExtendedLidar())
