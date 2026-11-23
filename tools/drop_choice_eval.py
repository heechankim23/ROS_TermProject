#!/usr/bin/env python3
"""주차구역 하역 자리 고르기 검증 (ROS 없음): 이 맵에만 맞는 최적화인지 무작위 더미 환경에서 확인한다.

  python3 tools/drop_choice_eval.py                  # 기본: 무작위 환경 300개 + 실제 맵
  python3 tools/drop_choice_eval.py --worlds 50 --seed 7

비교하는 정책 (drop_zones.choose_plan 의 거리 기준):
  w=0     예전 방식. 로봇 -> 하역 위치만 (가장 가까운 자리)
  w=1     지금 방식. 로봇 -> 하역 위치 + 하역 위치 -> 다음 갈 곳 (drop_next_points)
  w=0.5, 1.5, 2   두 번째 구간 가중치를 바꾼 변형
  oracle  실제 다음 목적지와 실제 경로 거리를 알고 고른 경우 (도달 가능한 최선)

정답 거리는 직선이 아니라 장애물을 돌아가는 경로 길이(0.2 m 격자 Dijkstra, 로봇 반폭 0.5 m 팽창)다.
'만재로 하역' 이면 실제 다음 대상은 queue_logic 의 점수식(사유·대기 시간·거리)이 하역 위치에서 고른다.
대기열 가중치(w_r, w_t, w_d)를 여러 조합으로 바꿔 '다음 대상 예측이 틀리는 경우'도 본다.
"""
import argparse
import heapq
import math
import os
import random
import statistics
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'src', 'kickboard_robot'))
from kickboard_robot import queue_logic as ql  # noqa: E402
from kickboard_robot.drop_zones import choose_plan, edge_plans, free_spots  # noqa: E402
from kickboard_robot.mission_logic import drop_next_points  # noqa: E402

NAV_RES = 0.2        # 경로 거리 격자
ROBOT_R = 0.5        # 경로 격자 팽창 (로봇 반폭 0.45 + 여유)
DROP_CLEARANCE = 1.05
POLICIES = [('w=0 (예전)', 0.0), ('w=0.5', 0.5), ('w=1 (지금)', 1.0), ('w=1.5', 1.5), ('w=2', 2.0)]
QUEUE_WEIGHTS = {'기본 10/1/5': ql.Weights(10, 1, 5), '거리 중시 10/1/20': ql.Weights(10, 1, 20),
                 '대기 중시 10/5/1': ql.Weights(10, 5, 1), '사유만 10/0/0': ql.Weights(10, 0, 0)}


class Grid:
    """사각 장애물 환경(무작위) 또는 실제 점유 지도. edge_plans 가 쓰는 is_free(x, y, clearance) 와 경로 거리장."""

    def __init__(self, w, h, rects):
        self.wm, self.hm = w, h
        self.rects = rects            # 장애물 사각형 (x0, y0, x1, y1). 방 밖도 막힌 것으로 본다
        self._build_nav()

    @classmethod
    def from_occupancy(cls, omap):
        g = cls.__new__(cls)
        g.omap = omap
        g.wm, g.hm = omap.w * omap.res, omap.h * omap.res
        g.ox, g.oy = omap.ox, omap.oy
        g._build_nav()
        return g

    def is_free(self, x, y, clearance):
        if hasattr(self, 'omap'):
            return self.omap.is_free(x, y, clearance)
        if not (clearance <= x <= self.wm - clearance and clearance <= y <= self.hm - clearance):
            return False
        for x0, y0, x1, y1 in self.rects:          # 사각형까지 거리 >= clearance
            dx, dy = max(x0 - x, 0.0, x - x1), max(y0 - y, 0.0, y - y1)
            if dx * dx + dy * dy < clearance * clearance:
                return False
        return True

    # 경로 거리 ----------------------------------------------------------
    def _origin(self):
        return (getattr(self, 'ox', 0.0), getattr(self, 'oy', 0.0))

    def _build_nav(self):
        ox, oy = self._origin()
        self.gw, self.gh = int(self.wm / NAV_RES), int(self.hm / NAV_RES)
        self.trav = [self.is_free(ox + (i + 0.5) * NAV_RES, oy + (j + 0.5) * NAV_RES, ROBOT_R)
                     for j in range(self.gh) for i in range(self.gw)]
        self._fields = {}

    def cell(self, p):
        ox, oy = self._origin()
        i = min(self.gw - 1, max(0, int((p[0] - ox) / NAV_RES)))
        j = min(self.gh - 1, max(0, int((p[1] - oy) / NAV_RES)))
        if self.trav[j * self.gw + i]:
            return j * self.gw + i
        best = None                      # 가장 가까운 통과 가능 칸 (목표가 벽에 살짝 붙은 경우)
        for dj in range(-4, 5):
            for di in range(-4, 5):
                ii, jj = i + di, j + dj
                if 0 <= ii < self.gw and 0 <= jj < self.gh and self.trav[jj * self.gw + ii]:
                    d = di * di + dj * dj
                    if best is None or d < best[0]:
                        best = (d, jj * self.gw + ii)
        return None if best is None else best[1]

    def field(self, p):
        key = (round(p[0], 3), round(p[1], 3))
        if key not in self._fields:
            dist = [math.inf] * (self.gw * self.gh)
            s = self.cell(p)
            if s is not None:
                dist[s] = 0.0
                pq = [(0.0, s)]
                steps = [(di, dj, NAV_RES * math.hypot(di, dj)) for di in (-1, 0, 1) for dj in (-1, 0, 1)
                         if di or dj]
                while pq:
                    d, c = heapq.heappop(pq)
                    if d > dist[c]:
                        continue
                    i, j = c % self.gw, c // self.gw
                    for di, dj, w in steps:
                        ii, jj = i + di, j + dj
                        if 0 <= ii < self.gw and 0 <= jj < self.gh:
                            n = jj * self.gw + ii
                            if self.trav[n] and d + w < dist[n]:
                                dist[n] = d + w
                                heapq.heappush(pq, (d + w, n))
            self._fields[key] = dist
        return self._fields[key]

    def path(self, a, b):
        c = self.cell(b)
        return math.inf if c is None else self.field(a)[c]


# 무작위 환경 ------------------------------------------------------------
def random_world(rng):
    w, h = rng.uniform(8, 20), rng.uniform(6, 15)
    rects = []
    for _ in range(rng.randint(2, 9)):                 # 선반·기둥
        if rng.random() < 0.5:
            sw, sh = rng.uniform(1.0, 4.0), rng.uniform(0.3, 0.8)
        else:
            sw, sh = rng.uniform(0.3, 0.8), rng.uniform(1.0, 4.0)
        x, y = rng.uniform(1.0, w - 1.0 - sw), rng.uniform(1.0, h - 1.0 - sh)
        rects.append((x, y, x + sw, y + sh))
    g = Grid(w, h, rects)
    zones = []
    for k in range(rng.randint(2, 4)):                 # 주차구역 (장애물과 겹치지 않는 바닥)
        for _ in range(50):
            zw, zh = rng.uniform(1.4, 2.5), rng.uniform(1.2, 2.0)
            x, y = rng.uniform(0.6, w - 0.6 - zw), rng.uniform(0.6, h - 0.6 - zh)
            poly = [[x, y], [x + zw, y], [x + zw, y + zh], [x, y + zh]]
            if g.is_free(x + zw / 2, y + zh / 2, max(zw, zh) / 2 + 0.2) and \
                    all(abs(x + zw / 2 - (z[0][0] + z[2][0]) / 2) > 2.5 or
                        abs(y + zh / 2 - (z[0][1] + z[2][1]) / 2) > 2.5 for _, z in zones):
                zones.append((chr(ord('A') + k), poly))
                break
    dock = None
    for _ in range(200):                               # 도킹 위치: 벽 쪽 빈 자리
        side = rng.randrange(4)
        p = [(rng.uniform(1.5, w - 1.5), 1.3), (rng.uniform(1.5, w - 1.5), h - 1.3),
             (1.3, rng.uniform(1.5, h - 1.5)), (w - 1.3, rng.uniform(1.5, h - 1.5))][side]
        if g.is_free(p[0], p[1], 1.0):
            dock = p
            break
    return g, zones, dock


def real_world():
    import yaml
    from kickboard_robot.occupancy import OccupancyMap
    cfg = os.path.join(ROOT, 'src', 'kickboard_sim')
    mc = yaml.safe_load(open(os.path.join(cfg, 'config', 'map_config.yaml')))
    g = Grid.from_occupancy(OccupancyMap(os.path.join(cfg, 'maps', 'kickboard_world.yaml')))
    zones = [(z['name'], z['polygon']) for z in mc['parking_zones']]
    return g, zones, (mc['robot_spawn']['x'], mc['robot_spawn']['y'])


def free_point(g, rng, clearance):
    ox, oy = g._origin()
    for _ in range(500):
        p = (ox + rng.uniform(0.5, g.wm - 0.5), oy + rng.uniform(0.5, g.hm - 0.5))
        if g.is_free(p[0], p[1], clearance):
            return p
    return None


# 정책 ------------------------------------------------------------------
def choose(plans, used, robot, need, next_xys, w):
    """실제 코드(drop_zones.choose_plan)에 두 번째 구간 가중치 w (= 파라미터 drop_next_weight) 를 넣어 고른다."""
    return choose_plan(plans, used, robot, need=need, next_xys=next_xys, next_weight=w)


def actual_next(g, drop_xy, queue_empty, waiting, dock, qw):
    """하역 뒤 실제로 가는 곳: 대기열이 비었으면 충전소, 아니면 대기열 점수식이 하역 위치에서 고른 킥보드."""
    if queue_empty or not waiting:
        return dock
    # now=100 s, t_enqueued 는 0~100 s 사이 무작위 -> 대기 시간 0~100 s
    return ql.ranked(waiting, 100.0, drop_xy, qw)[0]


def true_cost(g, robot, plan, nxt):
    r = plan['robot'][:2]
    return g.path(r, robot) + g.path(r, nxt)


# 실험 1: 한 번의 하역 결정 --------------------------------------------------
def one_shot(worlds, rng, samples):
    """정책별 실제 경로 길이 합. 결과: {(queue 이름, 상황): {정책: [비용...]}}"""
    out = {}
    for g, zones, dock, plans in worlds:
        for _ in range(samples):
            robot = free_point(g, rng, 0.6)
            if robot is None or math.isinf(g.path(dock, robot)):
                continue
            queue_empty = rng.random() < 0.5
            waiting = []
            for k in range(rng.randint(1, 6)):
                p = free_point(g, rng, 0.6)
                if p is not None and not math.isinf(g.path(dock, p)):
                    waiting.append(ql.Item(str(k), rng.choice([1, 2, 3]), p[0], p[1], 0, 0, rng.uniform(0, 100)))
            need = rng.randint(1, 4)
            wxy = [(i.x, i.y) for i in waiting]
            nexts = drop_next_points(queue_empty, wxy, dock)
            for qname, qw in QUEUE_WEIGHTS.items():
                key = (qname, '대기열 빈' if queue_empty else '만재')
                row = out.setdefault(key, {})
                costs = {}
                for pname, w in POLICIES:
                    plan, _ = choose(plans, [], robot, need, nexts, w)
                    nxt = actual_next(g, plan['robot'][:2], queue_empty, waiting, dock, qw)
                    nxt = dock if nxt is dock else (nxt.x, nxt.y)
                    costs[pname] = true_cost(g, robot, plan, nxt)
                orc = math.inf                    # oracle: 실제 다음 목적지·경로 거리를 알고 고름
                for p in plans:
                    if len(free_spots(p, [])) < need and any(len(free_spots(q, [])) >= need for q in plans):
                        continue
                    nxt = actual_next(g, p['robot'][:2], queue_empty, waiting, dock, qw)
                    nxt = dock if nxt is dock else (nxt.x, nxt.y)
                    orc = min(orc, true_cost(g, robot, p, nxt))
                costs['oracle'] = orc
                if all(math.isfinite(v) for v in costs.values()):
                    for k, v in costs.items():
                        row.setdefault(k, []).append(v)
    return out


# 실험 2: 하역을 여러 번 (자리가 차는 효과) ---------------------------------------
def episodes(worlds, rng, n_drops):
    """같은 환경에서 하역을 n_drops 번 이어서. 지금 방식이 가까운 구역을 먼저 채워 나중에 손해 보는지."""
    tot = {p: 0.0 for p, _ in POLICIES}
    fallback = {p: 0 for p, _ in POLICIES}
    for g, zones, dock, plans in worlds:
        seq = []
        for _ in range(n_drops):
            robot = free_point(g, rng, 0.6)
            if robot is None or math.isinf(g.path(dock, robot)):
                continue
            seq.append((robot, rng.randint(1, 4)))
        for pname, w in POLICIES:
            used = []
            for robot, need in seq:
                ch = choose(plans, used, robot, need, [dock], w)
                if ch is None:                    # 자리 없음 -> 충전소에서 하역 (로봇 -> 충전소)
                    fallback[pname] += 1
                    tot[pname] += g.path(dock, robot)
                    continue
                plan, fs = ch
                used += [(s[0], s[1]) for s in fs[:need]]
                tot[pname] += true_cost(g, robot, plan, dock)
    return tot, fallback


def build(g, zones, dock):
    """하역 후보. 충전소에서 경로가 닿지 않는 후보(선반에 갇힌 변 등)는 뺀다 — 실제 로봇도 갈 수 없다."""
    plans = []
    for name, poly in zones:
        plans += edge_plans(name, poly, g, DROP_CLEARANCE)
    return [p for p in plans if math.isfinite(g.path(dock, p['robot'][:2]))]


def report(title, out):
    print(f'\n## {title}')
    print('실제 경로 길이 (m). "예전 대비" = w=0 보다 줄어든 비율 (+ 가 좋음). "나빠진 경우" = w=0 보다 0.1 m 넘게 길어진 비율')
    for key in sorted(out):
        row = out[key]
        base = row['w=0 (예전)']
        n = len(base)
        print(f'\n{key[0]} · {key[1]}  (n={n})')
        print(f'  {"정책":12s} {"평균":>7s} {"예전 대비":>9s} {"나빠진 경우":>10s} {"최악 손해":>9s} {"oracle 과 차이":>13s}')
        for pname in [p for p, _ in POLICIES] + ['oracle']:
            v = row[pname]
            diff = [a - b for a, b in zip(v, base)]
            worse = sum(1 for d in diff if d > 0.1) / n
            gap = statistics.mean(a - b for a, b in zip(v, row['oracle']))
            print(f'  {pname:12s} {statistics.mean(v):7.2f} {-statistics.mean(diff) / statistics.mean(base):+9.1%} '
                  f'{worse:10.1%} {max(diff):9.2f} {gap:13.2f}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--worlds', type=int, default=300)
    ap.add_argument('--samples', type=int, default=20, help='환경마다 하역 상황 수')
    ap.add_argument('--drops', type=int, default=8, help='실험 2 에서 환경마다 이어서 하역하는 횟수')
    ap.add_argument('--seed', type=int, default=1)
    a = ap.parse_args()
    rng = random.Random(a.seed)

    worlds, skipped = [], 0
    while len(worlds) < a.worlds:
        g, zones, dock = random_world(rng)
        plans = build(g, zones, dock) if dock else []
        if dock is None or len({p['zone'] for p in plans}) < 2:      # 하역 후보 구역이 2개 이상인 환경만
            skipped += 1
            continue
        worlds.append((g, zones, dock, plans))
    n_plans = [len(w[3]) for w in worlds]
    print(f'# 하역 자리 고르기 검증 (seed {a.seed})')
    print(f'무작위 환경 {len(worlds)}개 (후보 구역 2개 미만이라 버린 것 {skipped}개), '
          f'환경당 하역 후보 {min(n_plans)}~{max(n_plans)}곳 (평균 {statistics.mean(n_plans):.1f})')

    report('실험 1. 무작위 환경 · 한 번의 하역', one_shot(worlds, rng, a.samples))

    g, zones, dock = real_world()
    real = [(g, zones, dock, build(g, zones, dock))]
    report('참고. 실제 맵 (kickboard_world) · 한 번의 하역', one_shot(real, rng, a.samples * 30))

    tot, fb = episodes(worlds, rng, a.drops)
    print(f'\n## 실험 2. 무작위 환경 · 하역 {a.drops}번 연속 (대기열 빈 경우, 자리가 차는 효과)')
    base = tot['w=0 (예전)']
    for pname, _ in POLICIES:
        print(f'  {pname:12s} 총 {tot[pname]:9.0f} m  예전 대비 {(base - tot[pname]) / base:+.1%}  '
              f'자리 없어 충전소 하역 {fb[pname]}회')


if __name__ == '__main__':
    main()
