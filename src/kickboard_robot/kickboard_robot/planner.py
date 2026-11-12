"""충돌 회피 경로 계획 (nav_bridge simple 백엔드용, ROS 무관 — 단위 테스트 가능).

Costmap
  정적 지도(벽·선반·기둥·충전소) + 라이다로 본 장애물(킥보드 등)을 0.1 m 격자로 합친다.
  라이다 빔이 지나간 칸은 비우고(raytrace clearing) 맞은 칸은 장애물로 기억한다 (decay_sec 동안).
  장애물까지 거리 d 로 비용을 매긴다:
    d < inscribed (로봇 반폭 0.47 m)      -> 통과 불가
    d < inflate   (운영값 1.1 m = 로봇을 덮는 원 0.98 m + 격자 오차)  -> 가까울수록 비쌈 (그래야 제자리 회전해도 안 부딪힘)
A* (8방향) 로 경로를 찾고, 보이는 곳까지 건너뛰어(line of sight) 꺾임을 줄인다.
"""
import heapq
import math
import warnings

import numpy as np

with warnings.catch_warnings():
    warnings.simplefilter('ignore')           # 이 VM 의 scipy 1.8 + numpy 1.26 버전 경고
    from scipy import ndimage

LETHAL = float('inf')


class Costmap:
    def __init__(self, x0, y0, width_m, height_m, res=0.1, inscribed=0.47, inflate=0.95,
                 inflate_weight=25.0, decay_sec=60.0):
        self.x0, self.y0, self.res = x0, y0, res
        self.w, self.h = int(round(width_m / res)), int(round(height_m / res))
        self.inscribed, self.inflate, self.weight, self.decay = inscribed, inflate, inflate_weight, decay_sec
        self.static = np.zeros((self.h, self.w), dtype=bool)       # [j(y), i(x)]
        self.seen = np.full((self.h, self.w), -np.inf)             # 라이다로 장애물을 마지막으로 본 시각
        self._dist = None
        self._t = None

    @classmethod
    def from_occupancy(cls, omap, **kw):
        """occupancy.OccupancyMap (map_server 형식) -> 같은 범위의 Costmap"""
        cm = cls(omap.ox, omap.oy, omap.w * omap.res, omap.h * omap.res, **kw)
        occ = np.frombuffer(bytes(omap.occ), dtype=np.uint8).reshape(omap.h, omap.w)[::-1]   # 행 0 = y 최소
        f = int(round(cm.res / omap.res))
        hh, ww = (omap.h // f) * f, (omap.w // f) * f
        coarse = occ[:hh, :ww].reshape(hh // f, f, ww // f, f).max(axis=(1, 3)) > 0
        cm.static[:coarse.shape[0], :coarse.shape[1]] = coarse
        return cm

    # ---------------- 좌표 ----------------
    def cell(self, x, y):
        return int((y - self.y0) / self.res), int((x - self.x0) / self.res)

    def center(self, j, i):
        return self.x0 + (i + 0.5) * self.res, self.y0 + (j + 0.5) * self.res

    def inside(self, j, i):
        return 0 <= j < self.h and 0 <= i < self.w

    # ---------------- 라이다 ----------------
    def update_scan(self, sx, sy, syaw, ranges, angle_min, angle_inc, range_min, range_max, t, ignore=None):
        """라이다 원점 (sx, sy, syaw) 의 한 스캔: 빔 경로는 비우고 끝점은 장애물로.
        ignore[k] = True 인 빔(자기 차체에 맞은 빔 등)은 정보 없음으로 친다"""
        r = np.asarray(ranges, dtype=float)
        a = syaw + angle_min + angle_inc * np.arange(len(r))
        valid = np.isfinite(r) & (r > range_min)          # 최소 거리 값 = 너무 가까움(정보 없음)
        if ignore is not None:
            valid &= ~np.asarray(ignore, dtype=bool)
        hit = valid & (r < range_max * 0.98)
        rr = np.where(valid, np.minimum(r, range_max), range_max)
        rr = np.where(np.isfinite(r), rr, range_max)            # inf = 끝까지 비어 있음
        rr = np.where(r <= range_min, 0.0, rr)                  # 너무 가까움 = 비우지도 않음
        if ignore is not None:
            rr = np.where(ignore, 0.0, rr)
        steps = np.arange(0.0, range_max, self.res * 0.5)
        d = steps[None, :]
        free = d < (rr[:, None] - self.res)                     # 끝점 바로 앞까지
        px = sx + np.cos(a)[:, None] * d
        py = sy + np.sin(a)[:, None] * d
        jj = ((py - self.y0) / self.res).astype(int)[free]
        ii = ((px - self.x0) / self.res).astype(int)[free]
        ok = (jj >= 0) & (jj < self.h) & (ii >= 0) & (ii < self.w)
        self.seen[jj[ok], ii[ok]] = -np.inf
        hx, hy = sx + np.cos(a[hit]) * r[hit], sy + np.sin(a[hit]) * r[hit]
        hj, hi = ((hy - self.y0) / self.res).astype(int), ((hx - self.x0) / self.res).astype(int)
        ok = (hj >= 0) & (hj < self.h) & (hi >= 0) & (hi < self.w)
        self.seen[hj[ok], hi[ok]] = t
        self._dist = None

    # ---------------- 거리 / 비용 ----------------
    def obstacles(self, t):
        return self.static | (self.seen > t - self.decay)

    def distance(self, t):
        """각 칸에서 가장 가까운 장애물까지 거리 (m)"""
        tk = math.floor(t)                       # 기억 만료(decay)는 초 단위면 충분 -> 매 주기 재계산 안 함
        if self._dist is None or self._t != tk:
            self._dist = ndimage.distance_transform_edt(~self.obstacles(t)) * self.res
            self._t = tk
        return self._dist

    def dist_at(self, x, y, t):
        j, i = self.cell(x, y)
        if not self.inside(j, i):
            return 0.0
        return float(self.distance(t)[j, i])

    def cost_grid(self, t):
        d = self.distance(t)
        c = 1.0 + self.weight * np.clip((self.inflate - d) / (self.inflate - self.inscribed), 0.0, 1.0) ** 2
        c[d < self.inscribed] = LETHAL
        return c

    # ---------------- 경로 ----------------
    def plan(self, start, goal, t, escape=0.6):
        """start, goal = (x, y). 경로 [(x, y), ...] (start 제외, goal 포함) 또는 None.
        출발점이 장애물에 너무 가까우면(정렬 직후 등) 반경 escape 안에서는 통과를 허용해 빠져나오게 한다."""
        cost = self.cost_grid(t)
        sj, si = self.cell(*start)
        gj, gi = self.cell(*goal)
        if not (self.inside(sj, si) and self.inside(gj, gi)):
            return None
        if math.isinf(cost[gj, gi]):
            return None
        er = int(math.ceil(escape / self.res))
        for dj in range(-er, er + 1):
            for di in range(-er, er + 1):
                j, i = sj + dj, si + di
                if self.inside(j, i) and math.isinf(cost[j, i]) and not self.static[j, i] \
                        and dj * dj + di * di <= er * er:
                    cost[j, i] = 1.0 + self.weight
        nbrs = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
                (-1, -1, 1.4142), (-1, 1, 1.4142), (1, -1, 1.4142), (1, 1, 1.4142)]
        g = {(sj, si): 0.0}
        parent = {}
        openq = [(0.0, sj, si)]
        while openq:
            f, j, i = heapq.heappop(openq)
            if (j, i) == (gj, gi):
                break
            gc = g[(j, i)]
            if f - math.hypot(gj - j, gi - i) > gc + 1e-9:
                continue                                   # 오래된 항목
            for dj, di, step in nbrs:
                nj, ni = j + dj, i + di
                if not self.inside(nj, ni):
                    continue
                c = cost[nj, ni]
                if math.isinf(c):
                    continue
                ng = gc + step * c
                if ng < g.get((nj, ni), float('inf')):
                    g[(nj, ni)] = ng
                    parent[(nj, ni)] = (j, i)
                    heapq.heappush(openq, (ng + math.hypot(gj - nj, gi - ni), nj, ni))
        if (gj, gi) not in g:
            return None
        cells = [(gj, gi)]
        while cells[-1] != (sj, si):
            cells.append(parent[cells[-1]])
        cells.reverse()
        pts = [self.center(j, i) for j, i in cells]
        pts[-1] = goal
        path = self.shortcut([start] + pts[1:], cost)
        return path or [goal]                           # 출발과 목표가 같은 칸이면 목표점 하나

    def segment_ok(self, a, b, cost, max_cost):
        n = max(2, int(math.hypot(b[0] - a[0], b[1] - a[1]) / (self.res * 0.5)))
        for k in range(n + 1):
            x, y = a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n
            j, i = self.cell(x, y)
            if not self.inside(j, i) or cost[j, i] > max_cost:
                return False
        return True

    def shortcut(self, pts, cost):
        """보이는 데까지 직선으로 건너뛴다. 단, 원래 경로보다 장애물에 더 붙지는 않게"""
        out, k = [pts[0]], 0
        while k < len(pts) - 1:
            nxt = k + 1
            for m in range(len(pts) - 1, k + 1, -1):
                worst = max(cost[self.cell(*p)] for p in pts[k:m + 1])
                if self.segment_ok(pts[k], pts[m], cost, max(worst, 1.0)):
                    nxt = m
                    break
            out.append(pts[nxt])
            k = nxt
        return out[1:]

    def path_blocked(self, path, t, from_xy=None):
        """경로 위에 통과 불가 칸이 생겼는지 (새 장애물)"""
        cost = self.cost_grid(t)
        pts = ([from_xy] if from_xy else []) + list(path)
        for a, b in zip(pts[:-1], pts[1:]):
            if not self.segment_ok(a, b, cost, 1e18):
                return True
        return False


def free_spot(cm, target, robot_xy, dist, clearance, t, step_deg=15, max_extra=0.6):
    """target 주변 반경 dist 원 위에서, 로봇 쪽 방향부터 좌우로 넓혀 가며
    장애물(정적 + 라이다)과 clearance 이상 떨어진 점을 찾는다. -> (x, y, target 을 보는 yaw) 또는 None"""
    base = math.atan2(robot_xy[1] - target[1], robot_xy[0] - target[0])
    extra = 0.0
    while extra <= max_extra + 1e-9:
        r = dist + extra
        for k in range(0, 180 // step_deg + 1):
            for sgn in ((1,) if k == 0 else (1, -1)):
                a = base + sgn * math.radians(k * step_deg)
                gx, gy = target[0] + r * math.cos(a), target[1] + r * math.sin(a)
                if cm.dist_at(gx, gy, t) >= clearance:
                    return gx, gy, math.atan2(target[1] - gy, target[0] - gx)
        extra += 0.2
    return None
