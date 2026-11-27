"""운동학 제약을 지키는 경로 계획·추종 (nav_bridge simple 백엔드, ROS 무관 — 단위 테스트 가능).

로봇은 자동차처럼 움직인다: 제자리에서 못 돌고(|w| <= |v| / R), 회전 중심은 뒷바퀴 축이다.
**이 모듈의 모든 자세 (x, y, heading) 는 뒤축(rear_axle) 기준**이다. base_footprint 와의 변환은 motion.to_rear / from_rear.

  1. 최소 회전 반경 R_min = 1.8 m (최대 곡률 1/R_min): 경로의 모든 원호가 이 곡률 안
  2. 상태는 (x, y, heading), 움직임은 앞/뒤 + 원호뿐. 방향을 바꿀 때는 전·후진을 섞는다 (3점 회전)
  3. 직선·원호(곡률 0, ±k/2, ±k)를 이어 붙이고, 목표 자세는 Reeds-Shepp 곡선(reeds_shepp.py)으로 정확히 맞춘다.
     처음 찾은 경로를 바로 쓰지 않고, 길이·후진·전환 벌점이 가장 낮은 것을 고른다
  4. 충돌: 차체 직사각형을 원 3개로 덮어 경로 위 모든 자세(0.1 m 간격)에서 장애물(정적 지도 + 라이다)과 검사

방법: Hybrid A* (x, y, heading 격자 위에서 원호 움직임으로 탐색) + Reeds-Shepp 로 목표 자세에 바로 연결
(analytic expansion). 휴리스틱 = 장애물을 피한 2D 최단 거리(격자 Dijkstra)와 직선 거리 중 큰 값.
추종: 곡률을 제한한 Pure Pursuit (PathTracker). 전·후진 전환점에서는 멈췄다 간다.
"""
import heapq
import math
import time
from dataclasses import dataclass

import numpy as np

try:
    from scipy import ndimage
    from scipy.sparse import coo_matrix
    from scipy.sparse.csgraph import dijkstra
except ImportError:      # pragma: no cover  (package.xml 에 python3-scipy 있음)
    ndimage = None

from . import reeds_shepp
from .motion import MIN_TURN_RADIUS, REAR_BODY_X

TWO_PI = 2.0 * math.pi


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


@dataclass
class KinematicParams:
    min_turn_radius: float = MIN_TURN_RADIUS  # m. 로봇의 최소 회전 반경 (추종 곡률 한계 1/R, 절대 넘지 않음)
    plan_radius_factor: float = 1.1   # 계획은 R x 이 배수로 -> 추종이 오차를 바로잡을 곡률 여유 (계획 반경 1.98 m)
    step: float = 0.5                 # m. 탐색 한 걸음(원호 길이)
    heading_bins: int = 72            # 5° 간격
    xy_res: float = 0.25              # m. 탐색 상태 격자 (같은 칸·같은 방향이면 같은 상태)
    allow_reverse: bool = True
    reverse_penalty: float = 1.8      # 후진 거리 비용 배수
    switch_penalty: float = 2.0       # 전·후진 전환 1회 비용 (m 환산)
    steer_penalty: float = 0.15       # 곡선 주행 비용 (곡률 비례) -> 같은 거리면 직선 선호
    steer_change_penalty: float = 0.2  # 곡률이 바뀌는 비용 -> 조향을 자주 바꾸지 않음
    obstacle_weight: float = 0.6      # 장애물 근처 비용 (Costmap 팽창 비용 비례)
    goal_pos_tol: float = 0.15        # 도착 위치 허용
    smooth_margin: float = 0.15       # 다듬기(Dubins 건너뛰기) 때 장애물과 더 두는 여유 (m)
    blend_len: float = 0.8            # m. 곡률 연속화(G2): 곡률이 0 -> k 로 바뀌는 데 쓰는 길이 (완화 곡선)
    blend_max_shift: float = 0.15     # m. 연속화한 경로가 원래 경로에서 이보다 멀어지면 그 구간은 원래 경로 유지
    goal_yaw_tol_deg: float = 10.0    # 도착 방향 허용
    max_expansions: int = 30000
    time_budget_sec: float = 2.5      # 실제 시간
    refine_expansions: int = 150      # 첫 경로를 찾은 뒤 더 싼 경로를 찾아보는 확장 수
    heuristic_weight: float = 1.4     # weighted A* (빠르게, 최적에 가깝게)
    # 차체 (뒤축 기준): x 범위와 반폭, 여유
    body_x_min: float = REAR_BODY_X[0]
    body_x_max: float = REAR_BODY_X[1]
    body_half_width: float = 0.525
    safety_margin: float = 0.05
    n_circles: int = 3

    @property
    def plan_radius(self):
        return self.min_turn_radius * self.plan_radius_factor

    @property
    def kmax(self):
        """계획 경로의 최대 곡률"""
        return 1.0 / max(1e-3, self.plan_radius)


# ---------------------------------------------------------------- 차체 충돌 검사
class Footprint:
    """차체 직사각형을 원 n 개로 덮는다: 중심 (cx, 0), 반경 r. 원 중심에서 장애물까지 거리 >= r 이면 충돌 없음."""

    def __init__(self, p: KinematicParams):
        length = p.body_x_max - p.body_x_min
        seg = length / p.n_circles
        self.cx = [p.body_x_min + seg * (k + 0.5) for k in range(p.n_circles)]
        self.r = math.hypot(seg / 2.0, p.body_half_width) + p.safety_margin


SURFACE_CAP = 3.0
ESCAPE_TIGHT_RADIUS = 2.0     # 출발 자세가 막힌 판정일 때 차체 반폭만 지키며 움직일 수 있는 범위 (m)


class CollisionGrid:
    """Costmap 의 거리 격자로 자세(x, y, th) 충돌 검사. 출발점 근처(escape)는 라이다 장애물·진입 금지 구역을 봐준다.
    진입 금지 구역(인도 등, Costmap.set_physical 로 구분된 경우)은 벽이 아니다: 차체 원은 물리 장애물(벽·차·라이다)과만
    검사하고, 구동축 중심(바퀴)은 진입 금지 경계에서 keep_min 이상 떨어지게 한다 -> 차도 가장자리에 서서 앞부분이
    인도 위로 나가는 정렬 자세(팔을 뻗는 자리)가 가능하다."""

    def __init__(self, cm, t, fp: Footprint, start=None, escape=0.0, keep_min=0.47):
        self.cm, self.fp = cm, fp
        self.res, self.x0, self.y0, self.h, self.w = cm.res, cm.x0, cm.y0, cm.h, cm.w
        self.dist = cm.distance(t)                      # 정적 + 라이다 (진입 금지 포함)
        self.dist_keep, self.keep_min = None, keep_min
        if getattr(cm, 'keepout', None) is not None and getattr(cm, 'phys', None) is not None:
            phys = cm.phys | (cm.seen > t - cm.decay)
            self.dist = ndimage.distance_transform_edt(~phys) * self.res       # 물리 장애물만
            self.dist_keep = cm._dist_keep
        self.cost = cm.cost_grid(t, surface=False)      # 장애물 가까움만 (노면은 surface_at)
        self.surface = getattr(cm, 'surface', None)
        if self.surface is not None:                    # 곡선 계획은 배수를 낮춰 쓴다: 크면 최소 회전 반경 때문에
            self.surface = np.minimum(self.surface, SURFACE_CAP)  # 차도 모서리를 스칠 수밖에 없는 곳에서 탐색이 안 끝남
        self.start, self.escape = start, escape
        self.dist_loose = None
        if start is not None and escape > 0.0:
            sj, si = cm.cell(*start)
            keep = getattr(cm, 'keepout', None)
            off_road = keep is not None and cm.inside(sj, si) and bool(keep[sj, si])
            phys = getattr(cm, 'phys', None)
            base = phys if phys is not None else cm.static       # 탈출 반경 안: 정적 물리 장애물만 (라이다·진입 금지는 봐줌)
            self.dist_loose = ndimage.distance_transform_edt(~base) * self.res
            self.escape = max(escape, 1.5 if off_road else escape)
        self.margin = self.res * 0.5                    # 칸 중심 거리 -> 실제 거리 오차
        self.tight = fp.r + self.margin                 # 탈출 반경 안에서 원이 장애물과 두어야 하는 거리

    def _d(self, x, y, loose):
        i = int(math.floor((x - self.x0) / self.res))
        j = int(math.floor((y - self.y0) / self.res))
        if not (0 <= j < self.h and 0 <= i < self.w):
            return -1.0
        g = self.dist_loose if loose else self.dist
        return float(g[j, i])

    def free(self, x, y, th, extra=0.0):
        c, s = math.cos(th), math.sin(th)
        need = self.fp.r + self.margin + extra
        loose = (self.dist_loose is not None
                 and math.hypot(x - self.start[0], y - self.start[1]) < self.escape)
        for cx in self.fp.cx:
            if self._d(x + c * cx, y + s * cx, loose) < (self.tight if loose else need):
                return False
        if self.dist_keep is not None and not loose:
            i = int(math.floor((x - self.x0) / self.res))
            j = int(math.floor((y - self.y0) / self.res))
            if not (0 <= j < self.h and 0 <= i < self.w) or self.dist_keep[j, i] < self.keep_min:
                return False
        return True

    def surface_at(self, x, y):
        """(x, y) 노면 비용 배수 (1 = 인도)"""
        if self.surface is None:
            return 1.0
        i = int(math.floor((x - self.x0) / self.res))
        j = int(math.floor((y - self.y0) / self.res))
        if not (0 <= j < self.h and 0 <= i < self.w):
            return 1.0
        return float(self.surface[j, i])

    def surface_length(self, poses, prev):
        """자세 열을 노면 비용 배수로 잰 길이"""
        total = 0.0
        for q in poses:
            total += math.hypot(q[0] - prev[0], q[1] - prev[1]) * self.surface_at(q[0], q[1])
            prev = q
        return total

    def penalty(self, x, y):
        i = int(math.floor((x - self.x0) / self.res))
        j = int(math.floor((y - self.y0) / self.res))
        if not (0 <= j < self.h and 0 <= i < self.w):
            return 0.0
        c = self.cost[j, i]
        return 0.0 if math.isinf(c) else float(c - 1.0)


# ---------------------------------------------------------------- 움직임 (원호)
def arc(x, y, th, k, length, direction, ds=0.1):
    """(x, y, th) 에서 곡률 k, 길이 length 로 direction(+1 앞, -1 뒤) 주행한 자세들 [(x, y, th), ...] (시작 제외)"""
    n = max(1, int(math.ceil(length / ds)))
    out = []
    for m in range(1, n + 1):
        s = direction * length * m / n
        if abs(k) < 1e-9:
            out.append((x + s * math.cos(th), y + s * math.sin(th), th))
        else:
            nth = th + k * s
            out.append((x + (math.sin(nth) - math.sin(th)) / k, y - (math.cos(nth) - math.cos(th)) / k, wrap(nth)))
    return out


# ---------------------------------------------------------------- Dubins (전진만, 최소 반경 R)
def _mod2pi(a):
    return a - TWO_PI * math.floor(a / TWO_PI)


def dubins_shortest(q0, q1, R):
    """Dubins 최단 경로 -> [(곡률 부호 +1 L / 0 S / -1 R, 길이 m), ...] 또는 None"""
    dx, dy = q1[0] - q0[0], q1[1] - q0[1]
    D = math.hypot(dx, dy)
    d = D / R
    th = math.atan2(dy, dx)
    a = _mod2pi(q0[2] - th)
    b = _mod2pi(q1[2] - th)
    sa, sb, ca, cb, cab = math.sin(a), math.sin(b), math.cos(a), math.cos(b), math.cos(a - b)
    best = None

    def consider(word, t, p, q):
        nonlocal best
        if t is None:
            return
        L = t + p + q
        if best is None or L < best[0]:
            best = (L, word, (t, p, q))

    # LSL
    p2 = 2 + d * d - 2 * cab + 2 * d * (sa - sb)
    if p2 >= 0:
        tmp = math.atan2(cb - ca, d + sa - sb)
        consider('LSL', _mod2pi(-a + tmp), math.sqrt(p2), _mod2pi(b - tmp))
    # RSR
    p2 = 2 + d * d - 2 * cab + 2 * d * (sb - sa)
    if p2 >= 0:
        tmp = math.atan2(ca - cb, d - sa + sb)
        consider('RSR', _mod2pi(a - tmp), math.sqrt(p2), _mod2pi(-b + tmp))
    # LSR
    p2 = -2 + d * d + 2 * cab + 2 * d * (sa + sb)
    if p2 >= 0:
        p = math.sqrt(p2)
        tmp = math.atan2(-ca - cb, d + sa + sb) - math.atan2(-2.0, p)
        consider('LSR', _mod2pi(-a + tmp), p, _mod2pi(-_mod2pi(b) + tmp))
    # RSL
    p2 = d * d - 2 + 2 * cab - 2 * d * (sa + sb)
    if p2 >= 0:
        p = math.sqrt(p2)
        tmp = math.atan2(ca + cb, d - sa - sb) - math.atan2(2.0, p)
        consider('RSL', _mod2pi(a - tmp), p, _mod2pi(b - tmp))
    # RLR
    tmp = (6.0 - d * d + 2 * cab + 2 * d * (sa - sb)) / 8.0
    if abs(tmp) <= 1:
        p = _mod2pi(TWO_PI - math.acos(tmp))
        t = _mod2pi(a - math.atan2(ca - cb, d - sa + sb) + p / 2.0)
        consider('RLR', t, p, _mod2pi(a - b - t + p))
    # LRL
    tmp = (6.0 - d * d + 2 * cab + 2 * d * (sb - sa)) / 8.0
    if abs(tmp) <= 1:
        p = _mod2pi(TWO_PI - math.acos(tmp))
        t = _mod2pi(-a + math.atan2(-ca + cb, d + sa - sb) + p / 2.0)
        consider('LRL', t, p, _mod2pi(b - a - t + p))
    if best is None:
        return None
    sign = {'L': 1, 'S': 0, 'R': -1}
    return [(sign[c], seg * R) for c, seg in zip(best[1], best[2])]


def dubins_poses(q0, q1, R, ds=0.1):
    segs = dubins_shortest(q0, q1, R)
    if segs is None:
        return None
    x, y, th = q0
    out = []
    for sgn, L in segs:
        if L < 1e-6:
            continue
        pts = arc(x, y, th, sgn / R, L, +1, ds)
        out += [(p[0], p[1], p[2], 1) for p in pts]
        x, y, th = pts[-1]
    return out


# ---------------------------------------------------------------- Hybrid A*
class _Node:
    __slots__ = ('x', 'y', 'th', 'g', 'parent', 'poses', 'k', 'd', 's')

    def __init__(self, x, y, th, g, parent, poses, k, d, s=0.0):
        self.x, self.y, self.th, self.g, self.parent, self.poses, self.k, self.d = x, y, th, g, parent, poses, k, d
        self.s = s                                      # 출발부터 달린 거리


def _grid_distance_to_goal(cm, cg, goal_xy):
    """장애물을 피한 2D 최단 거리(m) 격자 (goal 에서 Dijkstra). 지나갈 수 없는 칸 = inf"""
    passable = cg.dist >= (cg.fp.r * 0.5)          # 거친 판단 (휴리스틱용): 차체 반폭 정도면 지나감
    if cg.dist_keep is not None:
        passable &= cg.dist_keep >= cg.keep_min * 0.5
    if cg.dist_loose is not None:
        passable |= cg.dist_loose >= (cg.fp.r * 0.5)
    h, w = passable.shape
    idx = np.arange(h * w).reshape(h, w)
    rows, cols, vals = [], [], []
    for dj, di, c in ((0, 1, 1.0), (1, 0, 1.0), (1, 1, math.sqrt(2)), (1, -1, math.sqrt(2))):
        a = passable[max(0, -dj):h - max(0, dj), max(0, -di):w - max(0, di)]
        b = passable[max(0, dj):h - max(0, -dj) if dj < 0 else h, max(0, di):w if di >= 0 else w + di]
        ia = idx[max(0, -dj):h - max(0, dj), max(0, -di):w - max(0, di)]
        ib = idx[max(0, dj):h - max(0, -dj) if dj < 0 else h, max(0, di):w if di >= 0 else w + di]
        both = a & b
        rows.append(ia[both])
        cols.append(ib[both])
        if cg.surface is None:
            vals.append(np.full(int(both.sum()), c * cm.res))
        else:                                       # 노면 비용: 두 칸 배수의 평균
            sa = cg.surface[max(0, -dj):h - max(0, dj), max(0, -di):w - max(0, di)]
            sb = cg.surface[max(0, dj):h - max(0, -dj) if dj < 0 else h, max(0, di):w if di >= 0 else w + di]
            vals.append(c * cm.res * 0.5 * (sa[both] + sb[both]))
    r = np.concatenate(rows)
    c = np.concatenate(cols)
    v = np.concatenate(vals)
    graph = coo_matrix((np.concatenate([v, v]), (np.concatenate([r, c]), np.concatenate([c, r]))),
                       shape=(h * w, h * w)).tocsr()
    gj, gi = cm.cell(*goal_xy)
    if not cm.inside(gj, gi):
        return None
    gidx = gj * w + gi
    if not passable[gj, gi]:
        # 목표 칸이 거칠게 봐도 막혀 있으면 가장 가까운 지나갈 수 있는 칸에서 시작
        jj, ii = np.nonzero(passable)
        if len(jj) == 0:
            return None
        k = int(np.argmin((jj - gj) ** 2 + (ii - gi) ** 2))
        gidx = int(jj[k]) * w + int(ii[k])
    d = dijkstra(graph, directed=False, indices=gidx, limit=np.inf)
    return d.reshape(h, w)


def rs_poses(q0, segs, R, ds=0.1):
    """Reeds-Shepp 구간 목록 -> 촘촘한 자세 [(x, y, th, dir)] (시작 제외)"""
    x, y, th = q0
    out = []
    for steer, length in segs:
        d = 1 if length > 0 else -1
        pts = arc(x, y, th, steer / R, abs(length), d, ds)
        out += [(p[0], p[1], p[2], d) for p in pts]
        x, y, th = pts[-1]
    return out


def reverses_early(segs, travelled, forward_first):
    """Reeds-Shepp 구간 목록이 출발부터 forward_first(m)를 다 가기 전에 후진하는가"""
    for _, length in segs:
        if length < 0 and travelled < forward_first - 1e-6:
            return True
        travelled += abs(length)
    return False


def plan_path(cm, start, goal, t, p: KinematicParams = None, goal_yaw_free=False, escape=0.6, log=None,
              forward_first=0.0):
    """start=(x, y, th), goal=(x, y, th) (뒤축 기준). -> 촘촘한 경로 [(x, y, th, dir), ...] (시작 제외) 또는 None.
    dir: +1 전진, -1 후진. goal_yaw_free: 도착 방향 상관없음(중간 목표).
    forward_first: 처음 이만큼(m)은 전진만 하는 경로 중에서 고른다 (뒤에 방금 내린 킥보드가 있을 때)."""
    p = p or KinematicParams()
    if ndimage is None:
        return None
    t_start = time.monotonic()
    fp = Footprint(p)
    cg = CollisionGrid(cm, t, fp, start=start[:2], escape=escape)
    sj, si = cm.cell(*start[:2])
    gj, gi = cm.cell(*goal[:2])
    if not (cm.inside(sj, si) and cm.inside(gj, gi)):
        return None
    if cg.dist_loose is not None and not cg.free(*start):
        # 벽·거치대에 바싹 붙어 출발 (제자리에서 못 돌므로 그냥 두면 첫 걸음부터 전부 막힘):
        # 근처에서는 원 반경 대신 실제 차체 반폭만 지키며 빠져나온다
        cg.tight, cg.escape = p.body_half_width, max(cg.escape, ESCAPE_TIGHT_RADIUS)
    if not goal_yaw_free and not cg.free(*goal):
        return None
    if goal_yaw_free and not any(cg.free(goal[0], goal[1], b * TWO_PI / 16) for b in range(16)):
        return None                                   # 목표 위치에 어느 방향으로도 설 수 없음
    hgrid = _grid_distance_to_goal(cm, cg, goal[:2])
    if hgrid is None or not np.isfinite(hgrid[sj, si]):
        if log:
            log('운동학 계획: 2D 로도 목표에 닿을 수 없음')
        return None

    R, kmax = p.plan_radius, p.kmax
    steers = [0.0, kmax, -kmax, kmax / 2, -kmax / 2]
    dirs = [1, -1] if p.allow_reverse else [1]
    nb = p.heading_bins
    yaw_tol = math.radians(p.goal_yaw_tol_deg)

    xyr = max(p.xy_res, cm.res)

    def key(x, y, th):
        return (int(math.floor((x - cm.x0) / xyr)), int(math.floor((y - cm.y0) / xyr)),
                int(round(_mod2pi(th) / TWO_PI * nb)) % nb)

    def heur(x, y, th):
        j, i = int(math.floor((y - cm.y0) / cm.res)), int(math.floor((x - cm.x0) / cm.res))
        hd = hgrid[j, i] if (0 <= j < cm.h and 0 <= i < cm.w) else np.inf
        e = math.hypot(goal[0] - x, goal[1] - y)
        h = max(e, float(hd) if np.isfinite(hd) else e * 3.0 + 5.0)
        if not goal_yaw_free:
            h += 0.3 * abs(wrap(goal[2] - th)) * R
        return h

    openq, best_g, counter = [(p.heuristic_weight * heur(*start), 0, _Node(*start, 0.0, None, [], 0.0, 0))], {}, 0
    best_g[key(*start)] = 0.0

    def finish(n, tail):
        out = list(tail)
        while n is not None:
            out = list(n.poses) + out
            n = n.parent
        out = smooth_path(start, out, cg, R, deadline=t_start + p.time_budget_sec + 1.0, margin=p.smooth_margin)
        return curvature_smooth(start, out, cg, kmax, p.blend_len, p.blend_max_shift)

    def try_shot(n):
        """Reeds-Shepp 로 목표에 바로 연결 -> (이 노드까지 비용 포함 총비용, 자세 목록) 또는 None.
        후보를 싼 것부터 충돌 검사해 처음 통과한 것."""
        q0 = (n.x, n.y, n.th)
        yaws = [goal[2]]
        if goal_yaw_free:
            approach = math.atan2(goal[1] - n.y, goal[0] - n.x)
            yaws = [approach, wrap(approach + math.pi), goal[2]]
        cands = []
        for gth in yaws:
            for segs in reeds_shepp.paths(q0, (goal[0], goal[1], gth), R):
                if not p.allow_reverse and any(length < 0 for _, length in segs):
                    continue
                if reverses_early(segs, n.s, forward_first):
                    continue
                c = reeds_shepp.cost(segs, p.reverse_penalty, p.switch_penalty)
                if n.d and segs and (1 if segs[0][1] > 0 else -1) != n.d:
                    c += p.switch_penalty
                cands.append((c, segs))
        cands.sort(key=lambda item: item[0])
        best = float(hgrid[int((n.y - cm.y0) // cm.res), int((n.x - cm.x0) // cm.res)])
        for c, segs in cands[:12]:
            poses = rs_poses(q0, segs, R)
            if not poses or not all(cg.free(q[0], q[1], q[2]) for q in poses):
                continue
            if cg.surface is not None:               # 바로 연결이 차도를 가로질러 가는 지름길이면 쓰지 않음
                length = cg.surface_length(poses, (n.x, n.y))
                if length > 1.25 * best + 3.0 + sum(abs(v) for _, v in segs) - math.hypot(goal[0] - n.x, goal[1] - n.y):
                    continue
                c += length - sum(abs(v) for _, v in segs)
            return n.g + c, poses
        return None

    expansions, found, found_at = 0, None, 0
    while openq:
        f, _, n = heapq.heappop(openq)
        if n.g > best_g.get(key(n.x, n.y, n.th), np.inf) + 1e-9:
            continue
        expansions += 1
        over = expansions > p.max_expansions or time.monotonic() - t_start > p.time_budget_sec
        if found is not None and (over or expansions - found_at > p.refine_expansions or n.g >= found[0]):
            break
        if over:
            if log:
                log(f'운동학 계획: 탐색 한도 ({expansions}회, {time.monotonic() - t_start:.1f} s)')
            return None
        dgoal = math.hypot(goal[0] - n.x, goal[1] - n.y)
        # 도착 판정 (위치 + 방향)
        if dgoal <= p.goal_pos_tol and n.parent is not None and (goal_yaw_free or abs(wrap(goal[2] - n.th)) <= yaw_tol):
            if found is None or n.g < found[0]:
                found = (n.g, n, [])
            break
        # Reeds-Shepp 바로 연결: 가까울수록 자주 시도
        if dgoal < 8.0 * R and (expansions == 1 or expansions % (2 if dgoal < 3.0 * R else 10) == 0):
            shot = try_shot(n)
            if shot is not None and (found is None or shot[0] < found[0]):
                if found is None:
                    found_at = expansions
                found = (shot[0], n, shot[1])
        for d in dirs:
            if d < 0 and n.s < forward_first - 1e-6:
                continue
            for k in steers:
                poses = arc(n.x, n.y, n.th, k, p.step, d)
                if not all(cg.free(q[0], q[1], q[2]) for q in poses):
                    continue
                x, y, th = poses[-1]
                cost = p.step * (1.0 if d > 0 else p.reverse_penalty)
                cost += p.step * p.steer_penalty * abs(k) / kmax
                cost += p.steer_change_penalty * abs(k - n.k) / kmax
                if n.d and d != n.d:
                    cost += p.switch_penalty
                cost += p.step * p.obstacle_weight * cg.penalty(x, y) / max(1.0, cm.weight)
                g = n.g + cost * cg.surface_at(x, y)
                kk = key(x, y, th)
                if g >= best_g.get(kk, np.inf):
                    continue
                best_g[kk] = g
                child = _Node(x, y, th, g, n, [(q[0], q[1], q[2], d) for q in poses], k, d, n.s + p.step)
                counter += 1
                heapq.heappush(openq, (g + p.heuristic_weight * heur(x, y, th), counter, child))
    if found is not None:
        if log:
            log(f'운동학 계획: {expansions}회 탐색, {time.monotonic() - t_start:.2f} s')
        return finish(found[1], found[2])
    if log:
        log(f'운동학 계획: 경로 없음 ({expansions}회 탐색)')
    return None


def _length(poses, prev):
    L = 0.0
    for q in poses:
        L += math.hypot(q[0] - prev[0], q[1] - prev[1])
        prev = q
    return L


def smooth_path(start, path, cg, R, deadline=None, margin=0.0):
    """전진 구간을 Dubins 곡선으로 '멀리 건너뛰어' 다듬는다 (격자 탐색의 흔들림 제거).
    건너뛴 곡선이 충돌 없고 원래보다 길지 않을 때만 바꾼다. 후진 구간은 그대로."""
    if not path:
        return path
    pts = [(start[0], start[1], start[2], path[0][3])] + list(path)
    out = [pts[0]]
    i = 0
    n = len(pts)
    while i < n - 1:
        if pts[i + 1][3] != 1 or (i > 0 and pts[i][3] != 1):
            out.append(pts[i + 1])
            i += 1
            continue
        # 같은 전진 구간의 끝
        e = i + 1
        while e + 1 < n and pts[e + 1][3] == 1:
            e += 1
        best = None
        j = e
        while j > i + 3:
            if deadline is not None and time.monotonic() > deadline:
                break
            cand = dubins_poses(pts[i][:3], pts[j][:3], R)
            if cand is not None and _length(cand, pts[i]) <= _length(pts[i + 1:j + 1], pts[i]) * 1.02 + 1e-6 \
                    and all(cg.free(q[0], q[1], q[2], margin) for q in cand) \
                    and (cg.surface is None or cg.surface_length(cand, pts[i])
                         <= cg.surface_length(pts[i + 1:j + 1], pts[i]) * 1.02 + 0.1):
                best = (j, cand)
                break
            j -= max(1, (j - i) // 4)
        if best is None:
            out.append(pts[i + 1])
            i += 1
        else:
            j, cand = best
            out += cand
            i = j
    return out[1:]


def curvature_smooth(start, path, cg, kmax, blend=0.8, max_shift=0.15):
    """직선·원호를 이은 경로(G1: 방향은 이어지지만 곡률이 계단처럼 0 -> k 로 튐)를 곡률도 이어지는 경로(G2)로 바꾼다.
    실제 차량은 조향을 한순간에 꺾을 수 없으니, 곡률이 blend 길이에 걸쳐 직선으로 변하게(완화 곡선, clothoid) 한다.
    방법: 같은 방향(전진/후진) 구간마다 곡률을 호 길이 blend 창으로 이동 평균 -> 다시 적분.
      - 이동 평균은 원래 곡률들의 볼록 결합이라 최대 곡률을 넘지 않는다 (아주 작은 보정분은 kmax 로 다시 확인)
      - 구간 끝 방향은 정확히 보존(전체 방향 변화를 맞추는 상수 보정). 완화 곡선마다 경로가 조금씩(~blend²k/24) 안쪽으로
        밀려 끝 위치가 어긋나는데, 이 오차는 구간 길이에 비례해 나눠 없앤다 (방향 어긋남 3% = 1.7° 이하일 때만)
      - 원래 경로에서 max_shift 넘게 벗어나거나, 곡률이 kmax 넘거나, 차체가 장애물과 겹치면 그 구간은 원래 경로 그대로
    전·후진 전환점(정지)은 그대로 둔다 — 멈춘 상태에선 곡률이 끊겨도 된다."""
    if not path or blend <= 0.0:
        return path
    pts = [(start[0], start[1], start[2], 0)] + list(path)
    out, k, n = [], 1, len(pts)
    while k < n:
        d = pts[k][3]
        e = k
        while e + 1 < n and pts[e + 1][3] == d:
            e += 1
        seg = pts[k:e + 1]
        for b in (blend, blend * 0.5):                 # 안 되면 완화 길이를 반으로 (밀림은 길이의 제곱에 비례 -> 1/4)
            new = _g2_segment(pts[k - 1], seg, d, cg, kmax, b, max_shift)
            if new is not seg:
                break
        out += new
        k = e + 1
    return out


def _g2_segment(p0, seg, d, cg, kmax, blend, max_shift):
    xs = np.array([p0[0]] + [q[0] for q in seg])
    ys = np.array([p0[1]] + [q[1] for q in seg])
    ths = [p0[2]] + [q[2] for q in seg]
    ds = np.hypot(np.diff(xs), np.diff(ys))
    dth = np.array([wrap(ths[i + 1] - ths[i]) for i in range(len(seg))])
    S = float(ds.sum())
    if len(seg) < 3 or S < blend or np.any(ds < 1e-6):
        return seg
    s = np.concatenate([[0.0], np.cumsum(ds)])            # 각 자세의 호 길이
    Th = np.concatenate([[0.0], np.cumsum(dth)])          # 누적 방향 변화 (곡률의 적분)
    mid = 0.5 * (s[:-1] + s[1:])
    lo, hi = np.clip(mid - blend / 2, 0.0, S), np.clip(mid + blend / 2, 0.0, S)
    ks = (np.interp(hi, s, Th) - np.interp(lo, s, Th)) / (hi - lo)   # 이동 평균 곡률 (구간 i)
    ks += (Th[-1] - float(np.sum(ks * ds))) / S          # 끝 방향 보존
    if np.max(np.abs(ks)) > kmax * 1.02 + 1e-6:
        return seg
    th = p0[2] + np.concatenate([[0.0], np.cumsum(ks * ds)])
    thm = 0.5 * (th[:-1] + th[1:])
    nx = p0[0] + np.concatenate([[0.0], np.cumsum(d * ds * np.cos(thm))])
    ny = p0[1] + np.concatenate([[0.0], np.cumsum(d * ds * np.sin(thm))])
    ex, ey = xs[-1] - nx[-1], ys[-1] - ny[-1]
    if math.hypot(ex, ey) > 0.03 * S:
        return seg
    nx += ex * s / S                                     # 끝 위치 맞춤 (선형으로 나눠서 -> 곡률 연속 유지)
    ny += ey * s / S
    if np.max(np.hypot(nx - xs, ny - ys)) > max_shift:
        return seg
    new = [(float(nx[i]), float(ny[i]), wrap(float(th[i])), d) for i in range(1, len(s))]
    if not all(cg.free(q[0], q[1], q[2]) for q in new):
        return seg
    return new


def path_collides(cm, path, t, p: KinematicParams = None, from_index=0, start=None, escape=0.6):
    """남은 경로(촘촘한 자세)가 지금 장애물과 부딪히는지"""
    p = p or KinematicParams()
    cg = CollisionGrid(cm, t, Footprint(p), start=start, escape=escape if start else 0.0)
    return any(not cg.free(q[0], q[1], q[2]) for q in path[from_index:])


def max_curvature(path):
    """경로 안 연속 자세 사이의 최대 곡률 — 검증용"""
    km = 0.0
    for a, b in zip(path[:-1], path[1:]):
        ds = math.hypot(b[0] - a[0], b[1] - a[1])
        if ds > 1e-6:
            km = max(km, abs(wrap(b[2] - a[2])) / ds)
    return km


# ---------------------------------------------------------------- 추종 (곡률 제한 Pure Pursuit)
class PathTracker:
    """촘촘한 경로를 따라간다. step(pose) -> (v, w, done). pose 는 뒤축 기준. 항상 |w| <= |v| / R.
    - 같은 방향(전진/후진) 구간은 Pure Pursuit. 곡률은 1/R 로 제한하고 곡률이 크면 속도를 줄인다 (v = w_max / k).
      lookahead 는 속도에 비례 (0.35 + 0.6 v, 최대 1.5 m)
    - 방향이 바뀌는 점(cusp)에서는 멈췄다가 반대로
    - 제자리 회전은 하지 않는다. 비정상으로 크게 벗어나 목표점이 옆·뒤(75° 이상)에 있으면 lost = True 로 알려
      호출한 쪽이 다시 계획하게 한다
    - 조향 속도 한계: 달리는 동안 명령 곡률은 초당 k_rate_max 이상 바뀌지 않는다 (조향을 한순간에 꺾지 않음).
      멈춘 상태(출발·cusp)에서는 제한 없이 새로 시작"""

    def __init__(self, path, v_max=0.22, w_max=1.0, min_turn_radius=MIN_TURN_RADIUS, lookahead=None, pos_tol=0.08,
                 k_rate_max=2.5):
        self.path = list(path)
        self.v_max, self.w_max = v_max, w_max
        self.kmax = 1.0 / max(1e-3, min_turn_radius)     # 추종도 최소 회전 반경을 지킨다
        self.look_fixed = lookahead
        self.pos_tol = pos_tol
        self.i = 0
        self.v_prev = 0.0
        self.lost = False                                # 경로를 크게 벗어남 -> 다시 계획해야 함
        self.k_rate_max = k_rate_max                     # 1/(m·s). 명령 곡률 변화율 한계
        self.k_prev = None                               # 직전 명령 곡률 (None = 멈춘 상태)
        self._t_prev = None
        # 경로 각 점의 곡률 (앞쪽 곡선을 미리 보고 감속하는 데 씀)
        self.curv = [0.0] * len(self.path)
        for k in range(1, len(self.path)):
            a, b = self.path[k - 1], self.path[k]
            ds = math.hypot(b[0] - a[0], b[1] - a[1])
            if ds > 1e-6:
                self.curv[k] = wrap(b[2] - a[2]) / ds        # 진행 방향 기준 왼쪽 + (후진도 같은 식)

    def _segment_end(self):
        d = self.path[self.i][3]
        e = self.i
        while e + 1 < len(self.path) and self.path[e + 1][3] == d:
            e += 1
        return e

    def remaining(self, pose):
        if not self.path:
            return 0.0
        return math.hypot(self.path[-1][0] - pose[0], self.path[-1][1] - pose[1])

    def cross_track(self, pose):
        if not self.path or self.i >= len(self.path):
            return 0.0
        lo, hi = max(0, self.i - 5), min(len(self.path), self.i + 30)
        return min(math.hypot(q[0] - pose[0], q[1] - pose[1]) for q in self.path[lo:hi])

    def step(self, pose, dt=None):
        """dt: 이전 호출과의 간격(s). None 이면 실제 시간으로 잰다 (곡률 변화율 제한에 씀)"""
        now = time.monotonic()
        if dt is None:
            dt = 0.05 if self._t_prev is None else min(0.2, max(0.01, now - self._t_prev))
        self._t_prev = now
        x, y, th = pose
        if not self.path:
            return 0.0, 0.0, True
        if self.i >= len(self.path):
            return 0.0, 0.0, True
        q = self.path[self.i]
        e = self._segment_end()
        d = q[3]
        best, bi = 1e18, self.i
        for k in range(self.i, min(e, self.i + 60) + 1):
            dd = (self.path[k][0] - x) ** 2 + (self.path[k][1] - y) ** 2
            if dd < best:
                best, bi = dd, k
        self.i = bi
        end = self.path[e]
        dist_end = math.hypot(end[0] - x, end[1] - y)
        # 구간 끝: 가까워졌거나, 마지막 점까지 왔는데 이미 지나쳤음
        passed = self.i >= e and ((end[0] - x) * math.cos(end[2]) + (end[1] - y) * math.sin(end[2])) * d <= 0.0
        if dist_end <= self.pos_tol or (passed and dist_end <= 6.0 * self.pos_tol):
            self.i = e + 1
            self.v_prev = 0.0
            self.k_prev = None
            return 0.0, 0.0, False                      # cusp / 끝: 다음 주기에 이어서
        look = self.look_fixed or min(1.5, 0.35 + 0.6 * abs(self.v_prev))
        tgt = end
        acc = 0.0
        for k in range(self.i, e):
            acc += math.hypot(self.path[k + 1][0] - self.path[k][0], self.path[k + 1][1] - self.path[k][1])
            if acc >= look:
                tgt = self.path[k + 1]
                break
        dx, dy = tgt[0] - x, tgt[1] - y
        hx, hy = math.cos(th) * d, math.sin(th) * d     # 진행 방향 (후진이면 뒤쪽)
        lx = dx * hx + dy * hy
        ly = -dx * hy + dy * hx
        L = math.hypot(dx, dy)
        ang = math.atan2(ly, lx)
        if L > 0.4 and abs(ang) > math.radians(75):      # 정상 추종에선 생기지 않음
            self.lost = True
            self.v_prev = 0.0
            self.k_prev = None
            return 0.0, 0.0, False
        # 경로 곡률(피드포워드) + 측면·방향 오차 피드백 (진행 방향 기준). 순수 pure pursuit 은 급한 원호에서 안쪽을 깎는다
        near = self.path[self.i]
        pth = near[2] + (math.pi if d < 0 else 0.0)       # 경로 진행 방향
        rth = th + (math.pi if d < 0 else 0.0)            # 로봇 진행 방향
        ex, ey = x - near[0], y - near[1]
        e_lat = -ex * math.sin(pth) + ey * math.cos(pth)  # 경로 왼쪽 +
        e_head = wrap(rth - pth)
        kff = self.curv[min(self.i + 1, len(self.curv) - 1)]
        k = kff - 1.6 * e_lat - 2.2 * math.sin(e_head)
        k = max(-self.kmax, min(self.kmax, k))
        if self.k_prev is not None and self.v_prev > 0.0:  # 조향 속도 한계 (달리는 중)
            dk = self.k_rate_max * dt
            k = max(self.k_prev - dk, min(self.k_prev + dk, k))
        self.k_prev = k
        v = self.v_max
        v = min(v, max(0.06, 1.0 * dist_end))          # 정지점(구간 끝)에서 감속
        if abs(k) > 1e-6:
            v = min(v, self.w_max / abs(k))
        v = min(v, self.v_max / (1.0 + 0.5 * abs(k)))  # 급한 곡선은 천천히
        # 앞쪽(제동 거리 + 0.5 m) 경로의 최대 곡률에 맞춰 미리 감속 (곡선에 빠르게 들어가 바깥으로 밀리지 않게)
        ahead = 0.5 + self.v_prev ** 2 / (2 * 0.5)
        acc, kmax_ahead = 0.0, 0.0
        for kk in range(self.i, e + 1):
            kmax_ahead = max(kmax_ahead, abs(self.curv[kk]))
            if kk > self.i:
                acc += math.hypot(self.path[kk][0] - self.path[kk - 1][0], self.path[kk][1] - self.path[kk - 1][1])
            if acc > ahead:
                break
        if kmax_ahead > 1e-6:
            v = min(v, max(0.08, 0.8 * self.w_max / kmax_ahead))
        if abs(e_lat) > 0.15:                            # 많이 벗어나 있으면 천천히 돌아온다
            v = min(v, max(0.1, self.v_max * 0.5))
        v = min(v, self.v_prev + 0.06 if self.v_prev >= 0 else v)   # 가속 제한 (주기당)
        self.v_prev = v
        return v * d, v * k, False
