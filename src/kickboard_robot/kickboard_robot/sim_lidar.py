"""[시뮬레이션 전용] 2D 라이다 흉내 -> /scan (sensor_msgs/LaserScan, frame base_scan).

이 VM(arm64 VMware, ogre1)에서는 Gazebo gpu_lidar 가 모든 빔에 최소값(0.12 m)만 내서 쓸 수 없다.
그래서 sim_tag_detector 가 카메라를 흉내 내듯, 정답 정보로 빔을 직접 쏜다:
  - 정적 지도 (maps/kickboard_world.yaml: 벽·선반·기둥·충전소)
  - 바닥에 있는 킥보드 (/sim/scooter_truth 위치·방향 + /scooter_states 기울기 -> 넘어졌으면 눕힌 크기)
라이다 위치 = 로봇(map->base_footprint TF) 앞 scan_offset_x. 360 빔, 0.12 ~ 3.5 m, 노이즈 σ 1 cm.
kinematic 모드에서도 돌아서 nav_bridge 의 장애물 회피가 같은 방식으로 동작한다.
"""
import math
import random

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from kickboard_interfaces.msg import ScooterState, ScooterTruthArray

from .common import PoseTracker, spin_single
from .motion import yaw_from_quat
from .occupancy import OccupancyMap

# 라이다 높이(0.22 m) 평면에서 본 킥보드 단면 (킥보드 좌표, 원점 = 바닥 중심)
UPRIGHT_BOX = (-0.30, 0.30, -0.10, 0.10)      # 흙받이·옆판·기둥
FALLEN_BOX = (-0.30, 0.30, -0.95, 0.12)       # 누우면 기둥·핸들이 -y 쪽 바닥에 깔림 (gz_utils: roll +90°)


class SimLidar(Node):
    def __init__(self):
        super().__init__('sim_lidar')
        self.declare_parameter('map_yaml', '')
        self.declare_parameter('rate_hz', 5.0)
        self.declare_parameter('samples', 360)
        self.declare_parameter('range_min', 0.12)
        self.declare_parameter('range_max', 3.5)
        self.declare_parameter('noise', 0.01)
        self.declare_parameter('scan_offset_x', 0.80)
        self.declare_parameter('tilt_threshold_deg', 45.0)
        g = self.get_parameter
        om = OccupancyMap(g('map_yaml').value)
        self.res, self.ox, self.oy = om.res, om.ox, om.oy
        self.static = np.frombuffer(bytes(om.occ), dtype=np.uint8).reshape(om.h, om.w)[::-1].astype(bool)
        self.truth, self.tilt = [], {}
        self.pose = PoseTracker(self)
        self.create_subscription(ScooterTruthArray, '/sim/scooter_truth', self.on_truth, 10)
        self.create_subscription(ScooterState, '/scooter_states',
                                 lambda m: self.tilt.__setitem__(m.id, m.tilt_deg), 20)
        self.pub = self.create_publisher(LaserScan, '/scan', 10)
        n = g('samples').value
        self.angles = -math.pi + 2 * math.pi * np.arange(n) / n
        self.steps = np.arange(g('range_min').value, g('range_max').value, self.res * 0.5)
        self.create_timer(1.0 / g('rate_hz').value, self.tick)

    def on_truth(self, m):
        self.truth = [(sid, p.position.x, p.position.y, yaw_from_quat(p.orientation)) for sid, p in zip(m.ids, m.poses)]

    def grid(self):
        """정적 지도 + 킥보드 상자"""
        g = self.static.copy()
        th = self.get_parameter('tilt_threshold_deg').value
        h, w = g.shape
        for sid, x, y, yaw in self.truth:
            x0, x1, y0, y1 = FALLEN_BOX if self.tilt.get(sid, 0.0) > th else UPRIGHT_BOX
            u = np.arange(x0, x1 + 1e-9, self.res * 0.5)
            v = np.arange(y0, y1 + 1e-9, self.res * 0.5)
            uu, vv = np.meshgrid(u, v)
            c, s = math.cos(yaw), math.sin(yaw)
            px, py = x + c * uu - s * vv, y + s * uu + c * vv
            jj = ((py - self.oy) / self.res).astype(int)
            ii = ((px - self.ox) / self.res).astype(int)
            ok = (jj >= 0) & (jj < h) & (ii >= 0) & (ii < w)
            g[jj[ok], ii[ok]] = True
        return g

    def tick(self):
        pose = self.pose.get()
        if pose is None:
            return
        gp = self.get_parameter
        off = gp('scan_offset_x').value
        sx, sy = pose[0] + off * math.cos(pose[2]), pose[1] + off * math.sin(pose[2])
        g = self.grid()
        h, w = g.shape
        a = pose[2] + self.angles
        px = sx + np.cos(a)[:, None] * self.steps[None, :]
        py = sy + np.sin(a)[:, None] * self.steps[None, :]
        jj = ((py - self.oy) / self.res).astype(int)
        ii = ((px - self.ox) / self.res).astype(int)
        inside = (jj >= 0) & (jj < h) & (ii >= 0) & (ii < w)
        hit = np.ones_like(inside)                         # 지도 밖 = 막힘
        hit[inside] = g[jj[inside], ii[inside]]
        first = np.where(hit.any(axis=1), hit.argmax(axis=1), -1)
        rng = np.where(first >= 0, self.steps[np.maximum(first, 0)], np.inf)
        noise = gp('noise').value
        ranges = [float(r) if math.isinf(r) else float(max(gp('range_min').value, r + random.gauss(0, noise)))
                  for r in rng]
        m = LaserScan()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'base_scan'
        m.angle_min, m.angle_max = float(self.angles[0]), float(self.angles[-1])
        m.angle_increment = float(self.angles[1] - self.angles[0])
        m.scan_time = 1.0 / gp('rate_hz').value
        m.range_min, m.range_max = float(gp('range_min').value), float(gp('range_max').value)
        m.ranges = ranges
        self.pub.publish(m)


def main():
    rclpy.init()
    spin_single(SimLidar())
