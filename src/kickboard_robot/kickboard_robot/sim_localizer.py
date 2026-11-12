"""[시뮬레이션 전용, sim:=gz nav:=simple] 정답 위치로 map -> odom 을 계속 보정한다 (= 완벽한 AMCL).

simple 모드는 원래 map -> odom 을 정적 TF 로 고정하고 '오도메트리 = 정답'이라고 가정했다.
그런데 1.7 m 로봇은 킥보드·벽에 닿거나 제자리 회전 중 바퀴가 미끄러지면 오도메트리가 수 m 까지
어긋났다 (실측 2.7 m) -> 도킹·정렬 위치가 틀어지고 로봇팔이 닿지 않음.
Gazebo 의 로봇 world pose(/sim/robot_pose, PosePublisher)와 /odom 으로
  map->odom = T(map, base) * T(odom, base)^-1
을 계산해 30 Hz 로 TF 를 낸다. nav:=nav2 에서는 AMCL 이 이 역할을 하므로 띄우지 않는다.
"""
import math

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from .common import spin_single
from .motion import quat_from_yaw, yaw_from_quat


def compose_map_odom(truth, odom):
    """truth = map 에서 base (x, y, yaw), odom = odom 에서 base (x, y, yaw) -> map 에서 odom (x, y, yaw)"""
    tx, ty, tth = truth
    ox, oy, oth = odom
    yaw = tth - oth
    c, s = math.cos(yaw), math.sin(yaw)
    return tx - (c * ox - s * oy), ty - (s * ox + c * oy), yaw


class SimLocalizer(Node):
    def __init__(self):
        super().__init__('sim_localizer')
        # map->odom 은 미끄러질 때만 천천히 변한다. 정답·odom 메시지 시각 차이로 생기는 떨림은 저역 통과로 없앰
        self.declare_parameter('smoothing', 0.1)
        self.truth = None
        self.est = None
        self.br = TransformBroadcaster(self)
        self.create_subscription(PoseStamped, '/sim/robot_pose', self.on_truth, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.get_logger().info('정답 위치로 map->odom 보정 (simple 모드 위치 추정)')

    def on_truth(self, m):
        p = m.pose
        self.truth = (p.position.x, p.position.y, yaw_from_quat(p.orientation))

    def on_odom(self, m):
        if self.truth is None:
            return
        p = m.pose.pose
        raw = compose_map_odom(self.truth, (p.position.x, p.position.y, yaw_from_quat(p.orientation)))
        if self.est is None or math.hypot(raw[0] - self.est[0], raw[1] - self.est[1]) > 0.5:
            self.est = raw                       # 처음 / 크게 어긋났으면 바로 맞춤
        else:
            a = self.get_parameter('smoothing').value
            dyaw = math.atan2(math.sin(raw[2] - self.est[2]), math.cos(raw[2] - self.est[2]))
            self.est = (self.est[0] + a * (raw[0] - self.est[0]), self.est[1] + a * (raw[1] - self.est[1]),
                        self.est[2] + a * dyaw)
        x, y, yaw = self.est
        t = TransformStamped()
        t.header.stamp = m.header.stamp
        t.header.frame_id, t.child_frame_id = 'map', 'odom'
        t.transform.translation.x, t.transform.translation.y = x, y
        q = quat_from_yaw(yaw)
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = q
        self.br.sendTransform(t)


def main():
    rclpy.init()
    spin_single(SimLocalizer())
