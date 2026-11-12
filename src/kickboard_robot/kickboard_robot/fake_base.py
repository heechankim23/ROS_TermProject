"""[sim:=kinematic] Gazebo 없이 /cmd_vel 을 적분해 /odom 과 TF(odom->base_footprint)를 낸다.
로직(대기열·상태 머신·도킹 흐름)을 빠르게 검증할 때 사용."""
import math

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from .common import now_sec


class FakeBase(Node):
    def __init__(self):
        super().__init__('fake_base')
        self.declare_parameter('rate_hz', 30.0)
        self.x = self.y = self.th = 0.0
        self.v = self.w = 0.0
        self.last_cmd = 0.0
        self.last_t = None
        self.tf = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)
        self.create_timer(1.0 / self.get_parameter('rate_hz').value, self.tick)

    def on_cmd(self, m):
        self.v, self.w = m.linear.x, m.angular.z
        self.last_cmd = now_sec(self)

    def tick(self):
        t = now_sec(self)
        dt = 0.0 if self.last_t is None else t - self.last_t
        self.last_t = t
        if t - self.last_cmd > 0.5:
            self.v = self.w = 0.0
        self.th += self.w * dt
        self.x += self.v * math.cos(self.th) * dt
        self.y += self.v * math.sin(self.th) * dt
        stamp = self.get_clock().now().to_msg()
        qz, qw = math.sin(self.th / 2), math.cos(self.th / 2)
        tr = TransformStamped()
        tr.header.stamp, tr.header.frame_id, tr.child_frame_id = stamp, 'odom', 'base_footprint'
        tr.transform.translation.x, tr.transform.translation.y = self.x, self.y
        tr.transform.rotation.z, tr.transform.rotation.w = qz, qw
        self.tf.sendTransform(tr)
        o = Odometry()
        o.header.stamp, o.header.frame_id, o.child_frame_id = stamp, 'odom', 'base_footprint'
        o.pose.pose.position.x, o.pose.pose.position.y = self.x, self.y
        o.pose.pose.orientation.z, o.pose.pose.orientation.w = qz, qw
        o.twist.twist.linear.x, o.twist.twist.angular.z = self.v, self.w
        self.odom_pub.publish(o)


def main():
    rclpy.init()
    node = FakeBase()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
