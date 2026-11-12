"""sim_lidar 가공부: 정적 지도 + 킥보드 상자에 빔을 쏴서 거리값을 만든다 (ROS 없음, numpy).

격자 g[j, i] 는 j = y 방향(아래->위), i = x 방향. True = 막힘.
"""
import math
import random

import numpy as np

# 라이다 높이(0.22 m) 평면에서 본 킥보드 단면 (킥보드 좌표, 원점 = 바닥 중심)
UPRIGHT_BOX = (-0.30, 0.30, -0.10, 0.10)      # 흙받이·옆판·기둥
FALLEN_BOX = (-0.30, 0.30, -0.95, 0.12)       # 누우면 기둥·핸들이 -y 쪽 바닥에 깔림 (gz_utils: roll +90°)


def static_grid_from(omap):
    """OccupancyMap -> (막힘 격자 bool[h, w], 해상도, 원점 x, 원점 y)"""
    g = np.frombuffer(bytes(omap.occ), dtype=np.uint8).reshape(omap.h, omap.w)[::-1].astype(bool)
    return g, omap.res, omap.ox, omap.oy


class LidarSim:
    def __init__(self, static_grid, res, ox, oy, samples=360, range_min=0.12, range_max=3.5,
                 tilt_threshold_deg=45.0):
        self.static, self.res, self.ox, self.oy = static_grid, res, ox, oy
        self.range_min, self.range_max = range_min, range_max
        self.tilt_th = tilt_threshold_deg
        self.angles = -math.pi + 2 * math.pi * np.arange(samples) / samples
        self.steps = np.arange(range_min, range_max, res * 0.5)

    def grid_with(self, scooters, tilts):
        """정적 지도 + 킥보드 상자. scooters = [(id, x, y, yaw)], tilts = {id: 기울기 deg}"""
        g = self.static.copy()
        h, w = g.shape
        for sid, x, y, yaw in scooters:
            x0, x1, y0, y1 = FALLEN_BOX if tilts.get(sid, 0.0) > self.tilt_th else UPRIGHT_BOX
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

    def cast(self, sx, sy, syaw, grid):
        """센서 위치 (sx, sy), 방향 syaw 에서 각 빔의 첫 충돌 거리 (없으면 inf). 지도 밖 = 막힘."""
        h, w = grid.shape
        a = syaw + self.angles
        px = sx + np.cos(a)[:, None] * self.steps[None, :]
        py = sy + np.sin(a)[:, None] * self.steps[None, :]
        jj = ((py - self.oy) / self.res).astype(int)
        ii = ((px - self.ox) / self.res).astype(int)
        inside = (jj >= 0) & (jj < h) & (ii >= 0) & (ii < w)
        hit = np.ones_like(inside)
        hit[inside] = grid[jj[inside], ii[inside]]
        first = np.where(hit.any(axis=1), hit.argmax(axis=1), -1)
        return np.where(first >= 0, self.steps[np.maximum(first, 0)], np.inf)

    def add_noise(self, rng_m, noise, rng=random):
        """거리값에 가우시안 노이즈 (inf 는 그대로, range_min 아래로는 안 내려감) -> float 리스트"""
        return [float(r) if math.isinf(r) else float(max(self.range_min, r + rng.gauss(0, noise))) for r in rng_m]

    def scan(self, robot_pose, offset_x, scooters, tilts, noise, rng=random):
        """로봇 pose (x, y, yaw) 에서 본 한 바퀴 스캔 (거리 리스트, 빔 순서 = self.angles)."""
        x, y, yaw = robot_pose
        sx, sy = x + offset_x * math.cos(yaw), y + offset_x * math.sin(yaw)
        return self.add_noise(self.cast(sx, sy, yaw, self.grid_with(scooters, tilts)), noise, rng)
