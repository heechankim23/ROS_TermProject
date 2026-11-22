"""Single /cmd_vel output for extended profiles, after all motion producers.

Simulation safety guard: stale command/scan/TF stops, swept footprint keepout,
scan emergency stop and acceleration-limited wet-road speed. Not safety certified.
"""
import math
import rclpy
import yaml
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from .common import PoseTracker, EventLog, now_sec, spin_single
from .occupancy import OccupancyMap
from .extended_policy import footprint_free, scan_collision, integrate, ramp


class ExtendedGuard(Node):
    def __init__(self):
        super().__init__('extended_guard')
        self.declare_parameter('map_yaml', '')
        self.declare_parameter('extended_file', '')
        with open(self.get_parameter('extended_file').value, encoding='utf-8') as f:
            self.cfg = yaml.safe_load(f).get('extended', {})
        self.omap = OccupancyMap(self.get_parameter('map_yaml').value)
        self.pose = PoseTracker(self)
        self.ev = EventLog(self)
        self.cmd = Twist()
        self.cmd_t = self.scan_t = -1e9
        self.points = []
        self.v = self.w = 0.0
        self.last = None
        self.reason = None
        self.latched_stop = ''
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Twist, '/extended/cmd_vel_raw', self.on_cmd, 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(String, '/extended/stop', self.on_stop, 10)
        self.create_timer(0.1, self.tick)

    def on_cmd(self, m):
        self.cmd, self.cmd_t = m, now_sec(self)

    def on_stop(self, m):
        self.latched_stop = m.data or 'external_stop'

    def on_scan(self, m):
        self.scan_t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        self.points = [(0.80 + r * math.cos(m.angle_min + i * m.angle_increment),
                        r * math.sin(m.angle_min + i * m.angle_increment))
                       for i, r in enumerate(m.ranges)
                       if math.isfinite(r) and m.range_min < r < m.range_max]

    def tick(self):
        now = now_sec(self)
        dt = 0.1 if self.last is None else max(0, min(0.2, now - self.last))
        self.last = now
        pose = self.pose.get_stamped()
        reason = ''
        if self.latched_stop:
            reason = self.latched_stop
        elif now - self.cmd_t > 0.4 or now < self.cmd_t:
            reason = 'stale_command'
        elif now - self.scan_t > 0.6 or now < self.scan_t:
            reason = 'stale_scan'
        elif pose is None or now - (pose[3].sec + pose[3].nanosec * 1e-9) > 0.6:
            reason = 'stale_tf'
        vmax = float(self.cfg.get('speed_limit', 0.5))
        amax = float(self.cfg.get('accel_limit', 0.4))
        v = ramp(self.v, max(-vmax, min(vmax, self.cmd.linear.x)), amax, dt)
        w = ramp(self.w, max(-0.6, min(0.6, self.cmd.angular.z)), 0.8, dt)
        if not all(math.isfinite(z) for z in (v, w)):
            reason = 'invalid_command'
        if not reason and scan_collision(self.points, v, w):
            reason = 'obstacle_stop'
        if not reason:
            # Include a braking horizon; sample each 0.1 s and the current footprint.
            horizon = 0.3 + abs(v) / max(amax, 0.05)
            steps = max(1, math.ceil(horizon / 0.1))
            for k in range(steps + 1):
                if not footprint_free(self.omap, integrate(pose[:3], v, w, horizon * k / steps)):
                    reason = 'keepout_stop'
                    break
        if reason:
            v = w = 0.0
        self.v, self.w = v, w
        msg = Twist()
        msg.linear.x, msg.angular.z = v, w
        self.pub.publish(msg)
        if reason != self.reason:
            self.ev.emit('guard', reason=reason or 'clear')
            self.reason = reason


def main():
    rclpy.init()
    spin_single(ExtendedGuard())
