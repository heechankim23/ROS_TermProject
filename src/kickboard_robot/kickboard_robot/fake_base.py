"""[sim:=kinematic] Gazebo 없이 /cmd_vel 을 적분해 /odom 과 TF(odom->base_footprint)를 낸다.
로직(대기열·상태 머신·도킹 흐름)을 빠르게 검증할 때 사용.
/sim/set_robot_pose (map 좌표 PoseStamped) 를 받으면 로봇을 그 자리로 옮긴다 (dock_controller snap 모드)."""
import math

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformBroadcaster, TransformListener

from .common import now_sec, spin_single


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
        self.tf_buf = Buffer()
        self.tf_listener = TransformListener(self.tf_buf, self)
        self.create_subscription(PoseStamped, '/sim/set_robot_pose', self.on_set_pose, 10)
        self.create_timer(1.0 / self.get_parameter('rate_hz').value, self.tick)

    def on_cmd(self, m):
        self.v, self.w = m.linear.x, m.angular.z
        self.last_cmd = now_sec(self)

    def on_set_pose(self, m):
        """map 좌표 목표 -> odom 좌표로 바꿔 로봇 위치를 바로 바꾼다 (map->odom 은 정적 TF)."""
        try:
            t = self.tf_buf.lookup_transform('odom', 'map', Time())
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f'set_robot_pose: map->odom 없음 ({e})')
            return
        q = t.transform.rotation
        a = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        tx, ty = t.transform.translation.x, t.transform.translation.y
        px, py = m.pose.position.x, m.pose.position.y
        o = m.pose.orientation
        yaw = math.atan2(2 * (o.w * o.z + o.x * o.y), 1 - 2 * (o.y * o.y + o.z * o.z))
        self.x = tx + math.cos(a) * px - math.sin(a) * py
        self.y = ty + math.sin(a) * px + math.cos(a) * py
        self.th = yaw + a
        self.v = self.w = 0.0

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
    spin_single(FakeBase())
