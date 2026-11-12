"""점유 격자 지도(map_server 형식 yaml + pgm)로 '빈 공간' 판정. nav_bridge 의 접근 목표 선정용."""
import math
import os

import yaml


class OccupancyMap:
    def __init__(self, yaml_path):
        with open(yaml_path) as f:
            m = yaml.safe_load(f)
        img = m['image'] if os.path.isabs(m['image']) else os.path.join(os.path.dirname(yaml_path), m['image'])
        self.res = float(m['resolution'])
        self.ox, self.oy = float(m['origin'][0]), float(m['origin'][1])
        occ_th = float(m.get('occupied_thresh', 0.65))
        negate = int(m.get('negate', 0))
        with open(img, 'rb') as f:
            data = f.read()
        # P5 헤더 파싱 (주석 없음 가정)
        parts, idx = [], 0
        while len(parts) < 4:
            while data[idx:idx + 1].isspace():
                idx += 1
            j = idx
            while not data[j:j + 1].isspace():
                j += 1
            parts.append(data[idx:j])
            idx = j
        assert parts[0] == b'P5', 'P5 PGM 만 지원'
        self.w, self.h = int(parts[1]), int(parts[2])
        px = data[idx + 1: idx + 1 + self.w * self.h]
        # 행 0 = 이미지 맨 위 = y 최대
        self.occ = bytearray(self.w * self.h)
        for k, v in enumerate(px):
            p = v / 255.0 if negate else (255 - v) / 255.0
            self.occ[k] = 1 if p > occ_th else 0

    def _cell(self, x, y):
        i = int((x - self.ox) / self.res)
        j = self.h - 1 - int((y - self.oy) / self.res)
        return i, j

    def is_free(self, x, y, clearance):
        ci, cj = self._cell(x, y)
        r = int(math.ceil(clearance / self.res))
        for dj in range(-r, r + 1):
            for di in range(-r, r + 1):
                if di * di + dj * dj > r * r:
                    continue
                i, j = ci + di, cj + dj
                if not (0 <= i < self.w and 0 <= j < self.h) or self.occ[j * self.w + i]:
                    return False
        return True


def free_standoff(omap, target, robot_xy, dist, clearance, step_deg=10, max_extra=1.2):
    """target 주변 반경 dist 원 위에서, 로봇 쪽 방향부터 좌우로 넓혀 가며 빈 점을 찾는다.
    (GPS 오차로 target 이 벽 근처·벽 밖에 찍히면) 반경을 0.2 m 씩 max_extra 까지 늘려 재탐색."""
    base = math.atan2(robot_xy[1] - target[1], robot_xy[0] - target[0])
    extra = 0.0
    while extra <= max_extra + 1e-9:
        r = dist + extra
        for k in range(0, 180 // step_deg + 1):
            for sgn in ((1,) if k == 0 else (1, -1)):
                a = base + sgn * math.radians(k * step_deg)
                gx, gy = target[0] + r * math.cos(a), target[1] + r * math.sin(a)
                if omap is None or omap.is_free(gx, gy, clearance):
                    return gx, gy, math.atan2(target[1] - gy, target[0] - gx)
        extra += 0.2
    return None
