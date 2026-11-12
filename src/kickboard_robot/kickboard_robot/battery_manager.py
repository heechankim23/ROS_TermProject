"""F9: 가상 배터리. 소모 = 이동거리 x k1 + 경과시간 x k2. CHARGING 상태에서만 충전.

/battery_pct (Float32, 0~100), /battery_low (Bool) — return_battery_pct 이하이면 True
"""
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, Float32

from kickboard_interfaces.msg import RobotStatus

from .common import now_sec

# CHARGING 에서만 충전. IDLE(충전소 대기)은 소모도 충전도 없음 -> 시나리오 시작 배터리가 그대로 유지된다
NO_DRAIN_STATES = ('CHARGING', 'IDLE')


class BatteryManager(Node):
    def __init__(self):
        super().__init__('battery_manager')
        self.declare_parameter('initial_pct', 100.0)
        self.declare_parameter('k1_pct_per_m', 1.0)
        self.declare_parameter('k2_pct_per_s', 0.02)
        self.declare_parameter('charge_rate_pct_per_s', 2.0)   # 시연용으로 빠르게
        self.declare_parameter('return_battery_pct', 20.0)
        self.pct = float(self.get_parameter('initial_pct').value)
        self.state = 'IDLE'
        self.last_xy = None
        self.dist_acc = 0.0
        self.last_t = None
        self.create_subscription(Odometry, '/odom', self.on_odom, 20)
        self.create_subscription(RobotStatus, '/robot_status', lambda m: setattr(self, 'state', m.state), 10)
        self.pct_pub = self.create_publisher(Float32, '/battery_pct', 10)
        self.low_pub = self.create_publisher(Bool, '/battery_low', 10)
        self.create_timer(0.5, self.tick)
        self.get_logger().info(f'초기 배터리 {self.pct:.0f}%')

    def on_odom(self, m):
        p = m.pose.pose.position
        if self.last_xy is not None:
            d = math.hypot(p.x - self.last_xy[0], p.y - self.last_xy[1])
            if d < 1.0:                      # 순간이동(리셋) 무시
                self.dist_acc += d
        self.last_xy = (p.x, p.y)

    def tick(self):
        t = now_sec(self)
        if t <= 0.0:
            return
        dt = 0.0 if self.last_t is None else max(0.0, t - self.last_t)
        self.last_t = t
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        if self.state == 'CHARGING':
            self.pct += g('charge_rate_pct_per_s') * dt
        elif self.state in NO_DRAIN_STATES:
            pass
        else:
            self.pct -= g('k1_pct_per_m') * self.dist_acc + g('k2_pct_per_s') * dt
        self.dist_acc = 0.0
        self.pct = min(100.0, max(0.0, self.pct))
        self.pct_pub.publish(Float32(data=float(self.pct)))
        self.low_pub.publish(Bool(data=self.pct <= g('return_battery_pct')))


def main():
    rclpy.init()
    node = BatteryManager()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
