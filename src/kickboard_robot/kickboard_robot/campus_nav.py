"""캠퍼스(약 1.2 km) 같은 큰 지도용 경로 계획 (nav_bridge simple 백엔드, ROS 무관 — 단위 테스트 가능).

planner.Costmap 은 지도 전체를 한 격자로 잡고 A* 를 돈다. 캠퍼스 주행 지도는 0.25 m 격자 5000 x 5160 칸이라
그대로는 거리 변환 한 번에 수 초, 먼 목표 A* 는 수십 초가 걸린다. 그래서 두 단계로 나눈다.

  1. GridRouter  빈 공간 위 1 m 격자 그래프 (인도·횡단보도는 싸게, 차도는 비싸게: surface.py)에서 Dijkstra 로 큰 경로를 찾는다.
                 OSM 길 그래프는 꼭짓점이 건물 출입구 안쪽에 찍힌 곳이 많아 끊겨서 쓰지 않는다.
  2. RollingCostmap  로봇 주변 window_m x window_m 창만 planner.Costmap 으로 잡는다 (라이다 장애물 포함).
                 목표가 창 안이면 그대로 A*, 밖이면 큰 경로에서 창 안의 가장 먼 점까지 A* 하고 나머지는 큰 경로 점을 잇는다.
                 로봇이 움직이면 창을 다시 잡는다 (라이다로 본 장애물 기억은 겹치는 부분만 옮긴다).
nav_bridge 가 쓰는 Costmap 메서드(update_scan, plan, path_blocked, spot_ok, dist_at ...)를 같은 이름으로 제공한다.
"""
import copy
import json
import math
import warnings

import numpy as np

from .planner import Costmap
from .surface import FACTOR, build_surface

with warnings.catch_warnings():
    warnings.simplefilter('ignore')
    from scipy import ndimage

SUB_CLEARANCE = 1.5          # 창 안 중간 목표의 장애물 여유: 어느 방향으로 도착해도 차체(앞뒤 0.98 m)가 설 수 있게


class GridRouter:
    """빈 공간 위 1 m 격자 그래프에서 큰 경로를 찾는다 (scipy C 구현 Dijkstra, 캠퍼스 끝에서 끝까지 1초 안).
    칸 비용: 노면 종류(surface.py 의 FACTOR: 인도·횡단보도 1, 길가, 빈 땅, 차도), 장애물 1.2 m 안이면 x1.5.
    여유 거리(clear)가 min_clear 보다 작은 칸은 지나갈 수 없다 (로봇 반폭 + 격자 오차)."""

    def __init__(self, clear, ox, oy, res, surface=None, min_clear=0.6, cell=1.0):
        from scipy.sparse import coo_matrix
        f = max(1, int(round(cell / res)))
        H, W = (clear.shape[0] // f) * f, (clear.shape[1] // f) * f
        c = clear[:H, :W].reshape(H // f, f, W // f, f)[:, f // 2, :, f // 2]    # 칸 중심의 여유 거리
        self.ox, self.oy, self.cell = ox, oy, res * f
        self.h, self.w = c.shape
        free = c >= min_clear
        if surface is None:
            cost = np.ones(c.shape)
        else:                                      # 칸 중심의 노면 종류
            kind = surface[:H, :W].reshape(H // f, f, W // f, f)[:, f // 2, :, f // 2]
            cost = FACTOR[kind].astype(float)
        cost = np.where(c < 1.2, cost * 1.5, cost)
        self.free, self.cost = free, cost
        idx = -np.ones(c.shape, dtype=np.int64)
        idx[free] = np.arange(int(free.sum()))
        self.idx = idx
        self.n = int(free.sum())
        rows, cols, data = [], [], []
        h, w = self.h, self.w
        for dj, di, L in ((0, 1, 1.0), (1, 0, 1.0), (1, 1, 1.4142), (1, -1, 1.4142)):
            sa = (slice(0, h - dj), slice(max(0, -di), w - max(0, di)))      # 칸 (j, i)
            sb = (slice(dj, h), slice(max(0, di), w - max(0, -di)))          # 이웃 (j + dj, i + di)
            a, b, ca, cb = idx[sa], idx[sb], cost[sa], cost[sb]
            m = (a >= 0) & (b >= 0)
            rows.append(a[m])
            cols.append(b[m])
            data.append(L * self.cell * (ca[m] + cb[m]) / 2)
        r, cc, d = np.concatenate(rows), np.concatenate(cols), np.concatenate(data)
        self.G = coo_matrix((np.concatenate([d, d]), (np.concatenate([r, cc]), np.concatenate([cc, r]))),
                            shape=(self.n, self.n)).tocsr()
        self.ij = np.argwhere(free)            # 노드 번호 -> (j, i)
        self._cache = None

    def node(self, x, y, search=6):
        j, i = int((y - self.oy) / self.cell), int((x - self.ox) / self.cell)
        best = None
        for r in range(search + 1):            # 가장 가까운 빈 칸
            for dj in range(-r, r + 1):
                for di in range(-r, r + 1):
                    if max(abs(dj), abs(di)) != r:
                        continue
                    jj, ii = j + dj, i + di
                    if 0 <= jj < self.h and 0 <= ii < self.w and self.idx[jj, ii] >= 0:
                        d = dj * dj + di * di
                        if best is None or d < best[0]:
                            best = (d, int(self.idx[jj, ii]))
            if best is not None:
                return best[1]
        return None

    def xy(self, k):
        j, i = self.ij[k]
        return (self.ox + (i + 0.5) * self.cell, self.oy + (j + 0.5) * self.cell)

    def distances_from(self, start):
        """start 에서 모든 칸까지의 큰 경로를 한 번에 푼다 -> length(goal): 그 경로를 따라간 길이(m), 못 가면 None.
        비용(길 우선)이 가장 낮은 경로의 실제 길이라 nav_bridge 가 가는 거리와 같다"""
        from scipy.sparse.csgraph import dijkstra
        ss = self.node(*start)
        if ss is None:
            return lambda goal: None
        dist, pred = dijkstra(self.G, directed=False, indices=ss, return_predecessors=True)

        def length(goal):
            k = self.node(*goal)
            if k is None or not np.isfinite(dist[k]):
                return None
            total, prev = 0.0, tuple(goal)
            while k >= 0 and k != ss:
                p = self.xy(k)
                total += math.dist(prev, p)
                prev, k = p, pred[k]
            return total + math.dist(prev, start)
        return length

    def route(self, start, goal):
        """start -> goal 큰 경로 [(x, y), ...] (start 제외, goal 포함). 못 찾으면 None. 같은 목표면 결과를 재사용"""
        from scipy.sparse.csgraph import dijkstra
        gs = self.node(*goal)
        ss = self.node(*start)
        if gs is None or ss is None:
            return None
        if self._cache is None or self._cache[0] != gs:          # 목표에서 한 번 풀어 두면 어디서 출발해도 경로가 나온다
            dist, pred = dijkstra(self.G, directed=False, indices=gs, return_predecessors=True)
            self._cache = (gs, dist, pred)
        _, dist, pred = self._cache
        if not np.isfinite(dist[ss]):
            return None
        seq, k = [], ss
        while k != gs and k >= 0:
            seq.append(k)
            k = pred[k]
        seq.append(gs)
        pts = [self.xy(k) for k in seq[1:]]
        pts = _thin(pts, 4)
        return pts[:-1] + [tuple(goal)] if pts else [tuple(goal)]


def _thin(pts, every):
    """격자 경로 점을 every 칸마다 하나씩 (방향이 바뀌는 점은 남김)"""
    if len(pts) < 3:
        return pts
    out = [pts[0]]
    for k in range(1, len(pts) - 1):
        a, b, c = pts[k - 1], pts[k], pts[k + 1]
        turn = abs((b[0] - a[0]) * (c[1] - b[1]) - (b[1] - a[1]) * (c[0] - b[0])) > 1e-6
        if turn or k % every == 0:
            out.append(b)
    out.append(pts[-1])
    return out


class RollingCostmap:
    """큰 지도: 정적 지도는 전체, 경로 계획·라이다 기억은 로봇 주변 창(window)만"""

    def __init__(self, omap, route_json, window_m=70.0, res=0.25, inscribed=0.47, inflate=1.1,
                 decay_sec=60.0, logger=None):
        self.res, self.window_m = res, window_m
        self.kw = dict(res=res, inscribed=inscribed, inflate=inflate, decay_sec=decay_sec)
        self.log = logger
        grid = np.frombuffer(bytes(omap.occ), dtype=np.uint8).reshape(omap.h, omap.w)[::-1] > 0   # 행 0 = y 최소
        f = max(1, int(round(res / omap.res)))
        hh, ww = (omap.h // f) * f, (omap.w // f) * f
        self.full = grid[:hh, :ww].reshape(hh // f, f, ww // f, f).any(axis=(1, 3))
        self.ox, self.oy = omap.ox, omap.oy
        self.H, self.W = self.full.shape
        # 큰 경로: 0.25 m 여유 거리 -> 1 m 격자 그래프 (인도는 싸게, 차도는 비싸게)
        try:                                    # OpenCV 가 있으면 float32 로 (scipy 는 이 크기에서 메모리를 1 GB 가까이 씀)
            import cv2
            clear = cv2.distanceTransform((~self.full).astype(np.uint8), cv2.DIST_L2, 5) * np.float32(res)
        except ImportError:
            clear = ndimage.distance_transform_edt(~self.full).astype(np.float32) * res
        with open(route_json) as fjs:
            meta = json.load(fjs)
        # 노면 종류 (uint8): 창을 잡을 때 잘라 비용 배수로 바꾼다
        self.surface_kind = build_surface(meta, self.ox, self.oy, res, self.full.shape)
        self.graph = GridRouter(clear, self.ox, self.oy, res, surface=self.surface_kind)
        del clear
        self.win = None
        self.wc = None
        self.sub = None
        self._probe = None

    def snapshot(self):
        """계획용 복사본 (Costmap.snapshot 과 같은 뜻). 정적 지도·큰 경로 그래프는 함께 쓴다"""
        c = copy.copy(self)
        if self.win is not None:
            c.win = self.win.snapshot()
        c._probe = None
        return c

    # ---------- 창 ----------
    def _crop(self, cx, cy, size):
        half = size / 2
        x0 = min(max(cx - half, self.ox), self.ox + self.W * self.res - size)
        y0 = min(max(cy - half, self.oy), self.oy + self.H * self.res - size)
        i0, j0 = int(round((x0 - self.ox) / self.res)), int(round((y0 - self.oy) / self.res))
        x0, y0 = self.ox + i0 * self.res, self.oy + j0 * self.res
        cm = Costmap(x0, y0, size, size, **self.kw)
        cm.static[:, :] = self.full[j0:j0 + cm.h, i0:i0 + cm.w]
        cm.surface = FACTOR[self.surface_kind[j0:j0 + cm.h, i0:i0 + cm.w]]
        return cm, i0, j0

    def ensure(self, center):
        """center 가 창 중심에서 window/4 넘게 벗어나면 창을 다시 잡는다"""
        if self.win is not None and math.dist(center, self.wc) < self.window_m / 4:
            return
        new, i0, j0 = self._crop(center[0], center[1], self.window_m)
        if self.win is not None:          # 라이다로 본 장애물 기억을 겹치는 부분만 옮김
            old, oi, oj = self.win, self.wi, self.wj
            di, dj = i0 - oi, j0 - oj
            ys, ye = max(0, dj), min(old.h, dj + new.h)
            xs, xe = max(0, di), min(old.w, di + new.w)
            if ye > ys and xe > xs:
                new.seen[ys - dj:ye - dj, xs - di:xe - di] = old.seen[ys:ye, xs:xe]
        self.win, self.wi, self.wj = new, i0, j0
        self.wc = (new.x0 + self.window_m / 2, new.y0 + self.window_m / 2)

    def in_window(self, p, margin=0.0):
        w = self.win
        return w is not None and w.x0 + margin <= p[0] <= w.x0 + w.w * w.res - margin \
            and w.y0 + margin <= p[1] <= w.y0 + w.h * w.res - margin

    def view_for(self, target, margin=3.0):
        """target 주변 빈 자리 찾기용 Costmap: 창 안이면 창(라이다 포함), 밖이면 정적 지도만 잘라 만든 작은 격자"""
        if self.in_window(target, margin):
            return self.win
        key = (round(target[0]), round(target[1]))
        if self._probe is None or self._probe[0] != key:
            self._probe = (key, self._crop(target[0], target[1], 30.0)[0])
        return self._probe[1]

    # ---------- nav_bridge 가 쓰는 Costmap 메서드 ----------
    def update_scan(self, sx, sy, syaw, *a, **kw):
        self.ensure((sx, sy))
        self.win.update_scan(sx, sy, syaw, *a, **kw)

    def needs_replan(self, pose, now):
        """창 안 중간 목표에 가까워지면 다음 구간을 미리 계획한다 (창 밖 점으로 직진하지 않게). 0.5 초에 한 번까지"""
        return self.sub is not None and math.dist(pose[:2], self.sub) < 8.0 and now - self.sub_t > 0.5

    def plan(self, start, goal, t, escape=0.6):
        self.ensure(start)
        self.last_route = None
        self.sub = None
        if self.in_window(goal, 2.0):
            local = self.win.plan(start, goal, t, escape)
            if local is not None:
                return local
            # 창 안인데 길이 없음 = 건물을 돌아 창 밖으로 나갔다 와야 하는 경우 -> 큰 경로를 따라간다
        route = self.graph.route(start, goal)
        if route is None:
            if self.log:
                self.log(f'큰 경로 없음 ({start[0]:.0f}, {start[1]:.0f}) -> ({goal[0]:.0f}, {goal[1]:.0f})')
            return None
        self.last_route = route
        # 큰 경로를 1 m 간격으로 펴서, 창 안(가장자리 6 m 안쪽)이면서 지나갈 수 있는 가장 먼 점을 중간 목표로
        pts, prev = [], tuple(start)
        for q in route:
            n = max(1, int(math.dist(prev, q)))
            pts += [(prev[0] + (q[0] - prev[0]) * k / n, prev[1] + (q[1] - prev[1]) * k / n) for k in range(1, n + 1)]
            prev = q
        cost, dist = self.win.cost_grid(t), self.win.distance(t)
        cand = []
        for k, p in enumerate(pts):
            if not self.in_window(p, 6.0):
                break
            j, i = self.win.cell(*p)
            if self.win.inside(j, i) and not math.isinf(cost[j, i]):
                cand.append(p)
        # 창 안의 큰 경로 점 중 실제로 갈 수 있는 가장 먼 점 (먼 쪽부터 몇 개 시도)
        # 사방에 여유가 있는 점을 우선. 그런 점이 없는 좁은 길(또는 멀리 못 가는 경우)만 아무 점이나
        wide = [p for p in cand if dist[self.win.cell(*p)] >= SUB_CLEARANCE]
        if wide and math.dist(wide[-1], start) > 10.0:
            cand = wide
        local, sub = None, None
        step = max(1, len(cand) // 6)
        for p in list(reversed(cand))[::step][:7]:
            if math.dist(p, start) < 1.0:
                continue
            local = self.win.plan(start, p, t, escape)
            if local is not None:
                sub = p
                break
        if local is None:
            return None
        self.sub, self.sub_t = sub, t
        # 마지막 점 = 실제 목표: nav_bridge 가 중간 목표를 '도착'으로 보지 않게. 중간 목표 8 m 전에 다시 계획한다
        return local + [tuple(goal)]

    def path_blocked(self, path, t, from_xy=None):
        near = [p for p in path if self.in_window(p, 1.0)]
        if not near:
            return False
        return self.win.path_blocked(near, t, from_xy)

    def spot_ok(self, x, y, t, clearance, keep_clearance=None):
        return self.view_for((x, y), 3.0).spot_ok(x, y, t, clearance, keep_clearance)

    def __getattr__(self, name):          # cell, inside, static, dist_at, distance, cost_grid ... -> 창
        win = self.__dict__.get('win')
        if win is None:
            raise AttributeError(name)
        return getattr(win, name)
