"""collector 가공부: 태그 검출값 -> 킥보드 중심 추정, 도착 판정 (ROS 없음).

검출값은 (x, y, yaw) 튜플로 받는다. yaw = 태그 면의 바깥쪽 법선 방향.
"""
import math


def tag_id_of(scooter_id):
    """킥보드 ID -> AprilTag ID. 숫자 ID 는 그대로, 아니면 1~90 으로 해시."""
    try:
        return int(scooter_id)
    except ValueError:
        return abs(hash(scooter_id)) % 90 + 1


def arrived_by_tag(tag_distance, standoff, margin):
    """이동 중 태그가 standoff + margin 안에서 보이면 접근 완료로 본다 (GPS 오차로 목표가 너무 붙었을 때)."""
    return tag_distance <= standoff + margin


def center_from_tag(tx, ty, tyaw, half_depth):
    """태그 면 pose -> 킥보드 중심 (태그 면에서 법선 반대쪽으로 half_depth 안쪽). (x, y, yaw)"""
    return tx - half_depth * math.cos(tyaw), ty - half_depth * math.sin(tyaw), tyaw


def wrap_angle(a):
    """각도를 -pi ~ pi 로."""
    return math.atan2(math.sin(a), math.cos(a))


def retry_angles(base, offsets_deg, tried, min_sep_deg):
    """정렬 재시도 후보 각도(rad, 킥보드 중심에서 본 로봇 자리의 방향).
    base(처음 실패한 자리의 각도)에 offsets_deg 를 차례로 더한 순서를 지키고,
    이미 시도한 각도(tried)나 앞선 후보와 min_sep_deg 보다 가까운 것은 뺀다 (같은 자리 반복 방지)."""
    sep = math.radians(min_sep_deg) - 1e-9
    out = []
    for off in offsets_deg:
        a = wrap_angle(base + math.radians(off))
        if all(abs(wrap_angle(a - t)) >= sep for t in list(tried) + out):
            out.append(a)
    return out


def align_pose(cx, cy, r, angle):
    """킥보드 중심 (cx, cy) 둘레 반경 r 원 위, 방향 angle 인 자리와 중심을 바라보는 yaw. (x, y, yaw)"""
    return cx + r * math.cos(angle), cy + r * math.sin(angle), wrap_angle(angle + math.pi)


def align_next_step(waited, status, msg, wait_sec):
    """정렬 이동이 끝난 뒤 다음 단계.
    'done'        : 성공했거나, 재시도로 해결할 수 없는 결과(취소·거절·시간 초과)
    'wait_same'   : 잠깐 기다렸다 같은 자리로 다시 (사람·수레처럼 지나가는 장애물이면 비켜 있음)
    'change_angle': 같은 반경에서 각도를 바꾼 자리로 (기다려도 막혀 있음 = 나무·구조물 같은 고정 장애물)
    팔이 안 닿는 먼 자리('out of arm reach')는 자리 선택 문제라 기다려도 같으므로 바로 각도를 바꾼다."""
    if status != 'failed':
        return 'done'
    if not waited and wait_sec > 0 and not msg.startswith('out of arm reach'):
        return 'wait_same'
    return 'change_angle'


ARM_REACH_NEAR = 0.25     # align_distance 보다 이만큼 가까워도 팔이 닿음
ARM_REACH_SKIP_FAR = 0.15  # 정렬 주행을 생략할 때 허용하는 바깥쪽 여유 (적재 허용 여유 arm_reach_margin 보다 좁게)


def within_reach(r, dist, near, far):
    """킥보드 중심까지 거리 r 이 팔이 닿는 범위 [dist - near, dist + far] 안인가."""
    return dist - near <= r <= dist + far


def mean_center(centers, last_n=None):
    """중심 추정값들의 평균 (x, y)와 마지막 yaw. last_n 이면 최근 n 개만. 비어 있으면 None."""
    pts = centers[-last_n:] if last_n else centers
    if not pts:
        return None
    return (sum(c[0] for c in pts) / len(pts), sum(c[1] for c in pts) / len(pts), pts[-1][2])


# ---------------- 정렬 자리 채점 (align_spot_mode: scan) ----------------
# 킥보드 둘레 '팔이 닿는 띠' 위 후보 자리를 정적 지도 + 진입 금지 구역 + 라이다 점으로 검사·채점한다.
# 검사: 몸체(실제 직사각형)가 장애물과 안 겹침 / 경로 계획이 받아 주는 자리(장애물에서 로봇 반폭 이상)
#       / 팔 회전축 -> 킥보드 사이가 비어 있음 / 지금 자리에서 격자를 따라 갈 수 있음.
# 점수: 갈 길이 + 기본 반경에서 벗어난 정도 + 장애물에 붙은 정도. 낮을수록 좋다.
BODY_X = (-0.92, 0.89)       # 로봇 차체 x 범위 (base_footprint 기준, nav_bridge footprint 와 같게)
BODY_HALF_W = 0.47           # 차체 반폭 0.37 + 여유 0.1
ARM_PIVOT_X = 0.50           # 팔 회전축 (model.sdf arm_turret)
ARM_HALF_W = 0.15            # 팔이 지나가는 띠의 반폭
SCOOTER_KEEP = 0.6           # 킥보드 중심에서 이 안의 점은 킥보드 자신 (팔 경로 검사에서 뺌)


class SpotGrid:
    """킥보드 중심 둘레 정사각형 격자. 배열 [j(y 증가), i(x 증가)].
    wall: 물리 장애물(정적 지도), keep: 진입 금지 구역(인도 등, 벽 아님), scan: 라이다로 본 점."""

    def __init__(self, cx, cy, half=6.0, res=0.1):
        import numpy as np
        self.res = res
        self.n = int(round(2 * half / res))
        self.x0, self.y0 = cx - half, cy - half
        self.wall = np.zeros((self.n, self.n), dtype=bool)
        self.keep = np.zeros((self.n, self.n), dtype=bool)
        self.scan = np.zeros((self.n, self.n), dtype=bool)

    def cell(self, x, y):
        return int(math.floor((y - self.y0) / self.res)), int(math.floor((x - self.x0) / self.res))

    def inside(self, j, i):
        return 0 <= j < self.n and 0 <= i < self.n

    def centers(self):
        import numpy as np
        c = (np.arange(self.n) + 0.5) * self.res
        return self.x0 + c, self.y0 + c

    def add_points(self, xs, ys):
        import numpy as np
        i = np.floor((np.asarray(xs) - self.x0) / self.res).astype(int)
        j = np.floor((np.asarray(ys) - self.y0) / self.res).astype(int)
        ok = (i >= 0) & (i < self.n) & (j >= 0) & (j < self.n)
        self.scan[j[ok], i[ok]] = True

    def add_box(self, x, y, sx, sy, yaw=0.0, layer='wall'):
        """시험용: 회전한 직사각형을 채운다."""
        import numpy as np
        xs, ys = self.centers()
        X, Y = np.meshgrid(xs, ys)
        c, s = math.cos(yaw), math.sin(yaw)
        u, v = (X - x) * c + (Y - y) * s, -(X - x) * s + (Y - y) * c
        getattr(self, layer)[(np.abs(u) <= sx / 2) & (np.abs(v) <= sy / 2)] = True


def crop_occupancy(grid_tb, ox, oy, res_map, grid, outside=True):
    """지도 격자(행 0 = 위 = y 최대, OccupancyMap.grid 와 같은 배치)를 SpotGrid 칸 중심에서 읽는다.
    -> [j, i] bool 배열. 지도 밖 칸은 outside 값."""
    import numpy as np
    h, w = grid_tb.shape
    xs, ys = grid.centers()
    ii = np.floor((xs - ox) / res_map).astype(int)
    jj = h - 1 - np.floor((ys - oy) / res_map).astype(int)
    J, II = np.meshgrid(jj, ii, indexing='ij')
    ok = (II >= 0) & (II < w) & (J >= 0) & (J < h)
    out = np.full((grid.n, grid.n), bool(outside))
    out[ok] = grid_tb[J[ok], II[ok]]
    return out


def fill_static(grid, nav, phys=None):
    """grid 의 wall / keep 층을 지도에서 채운다. nav, phys = (grid_tb, ox, oy, res) 또는 None.
    물리 지도(phys)가 있으면 wall = 물리 장애물, keep = 주행 지도에만 있는 칸(인도 등 진입 금지).
    없으면 주행 지도 전체를 벽으로 본다."""
    if nav is None:
        return
    n = crop_occupancy(*nav, grid)
    if phys is None:
        grid.wall |= n
        return
    p = crop_occupancy(*phys, grid)
    grid.wall |= p
    grid.keep |= n & ~p


def _geodesic(passable, start, res):
    """8방향 Dijkstra 거리(m). 갈 수 없는 칸 = inf. 시작 칸은 막혀 있어도 출발점으로 쓴다."""
    import heapq
    import numpy as np
    n = passable.shape[0]
    d = np.full(passable.shape, np.inf)
    sj, si = start
    if not (0 <= sj < n and 0 <= si < n):
        return d
    d[sj, si] = 0.0
    pq = [(0.0, sj, si)]
    steps = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
             (-1, -1, 1.4142), (-1, 1, 1.4142), (1, -1, 1.4142), (1, 1, 1.4142)]
    while pq:
        c, j, i = heapq.heappop(pq)
        if c > d[j, i]:
            continue
        for dj, di, w in steps:
            nj, ni = j + dj, i + di
            if 0 <= nj < n and 0 <= ni < n and passable[nj, ni]:
                nc = c + w * res
                if nc < d[nj, ni]:
                    d[nj, ni] = nc
                    heapq.heappush(pq, (nc, nj, ni))
    return d


def plan_align_spots(grid, cx, cy, robot_xy, dist, radii, step_deg=10.0, inscribed=0.47,
                     keep_clear=None, w_radial=2.0, w_room=3.0, room=1.1):
    """팔이 닿는 자리 후보를 검사·채점한다.
    -> (후보 목록, 탈락 사유별 개수). 후보 = dict(x, y, yaw, a(킥보드에서 본 각도), r, score, geo, room), 점수 오름차순.
    keep_clear: 진입 금지 경계와 로봇 중심 사이 최소 거리 (None 이면 진입 금지 구역을 벽처럼 취급)."""
    import numpy as np
    from scipy import ndimage
    phys = grid.wall | grid.scan
    hard = phys | grid.keep if keep_clear is None else phys
    d_phys = ndimage.distance_transform_edt(~hard) * grid.res
    d_keep = ndimage.distance_transform_edt(~grid.keep) * grid.res if grid.keep.any() else None
    d_all = d_phys if d_keep is None else np.minimum(d_phys, d_keep)
    # 여유(room) 점수는 집으려는 킥보드 자신은 빼고 잰다 (안 그러면 늘 킥보드에서 먼 자리로 밀림)
    xs, ys = grid.centers()
    X, Y = np.meshgrid(xs, ys)
    others = hard & (np.hypot(X - cx, Y - cy) > SCOOTER_KEEP)
    d_room = ndimage.distance_transform_edt(~others) * grid.res
    geo = _geodesic(d_all > inscribed, grid.cell(*robot_xy), grid.res)
    # 몸체 검사용 표본점 (로봇 좌표)
    bx = np.arange(BODY_X[0], BODY_X[1] + 1e-9, grid.res)
    by = np.arange(-BODY_HALF_W, BODY_HALF_W + 1e-9, grid.res)
    BX, BY = [v.ravel() for v in np.meshgrid(bx, by)]
    out, why = [], {'map_edge': 0, 'unreachable': 0, 'too_close': 0, 'body': 0, 'arm': 0}
    for r in radii:
        k = int(round(360.0 / step_deg))
        for m in range(k):
            a = math.radians(m * step_deg)
            x, y, yaw = align_pose(cx, cy, r, a)
            j, i = grid.cell(x, y)
            if not grid.inside(j, i):
                why['map_edge'] += 1
                continue
            if d_phys[j, i] < inscribed + 0.03 or (keep_clear is not None and d_keep is not None
                                                   and d_keep[j, i] < keep_clear):
                why['too_close'] += 1
                continue
            if not math.isfinite(geo[j, i]):
                why['unreachable'] += 1
                continue
            c, s = math.cos(yaw), math.sin(yaw)
            pj = np.floor((y + BX * s + BY * c - grid.y0) / grid.res).astype(int)
            pi = np.floor((x + BX * c - BY * s - grid.x0) / grid.res).astype(int)
            ok = (pj >= 0) & (pj < grid.n) & (pi >= 0) & (pi < grid.n)
            if (~ok).any() or hard[pj, pi].any():
                why['body'] += 1
                continue
            # 팔 회전축 -> 킥보드 중심 사이 띠 (킥보드 자신의 점은 뺌)
            ax, ay = x + ARM_PIVOT_X * c, y + ARM_PIVOT_X * s
            L = math.hypot(cx - ax, cy - ay)
            ts = np.arange(0.0, L, grid.res / 2)
            ux, uy = (cx - ax) / max(L, 1e-9), (cy - ay) / max(L, 1e-9)
            blocked = False
            for off in (-ARM_HALF_W, 0.0, ARM_HALF_W):
                qx = ax + ts * ux - off * uy
                qy = ay + ts * uy + off * ux
                far = np.hypot(qx - cx, qy - cy) > SCOOTER_KEEP
                qj = np.floor((qy[far] - grid.y0) / grid.res).astype(int)
                qi = np.floor((qx[far] - grid.x0) / grid.res).astype(int)
                okq = (qj >= 0) & (qj < grid.n) & (qi >= 0) & (qi < grid.n)
                if phys[qj[okq], qi[okq]].any():
                    blocked = True
                    break
            if blocked:
                why['arm'] += 1
                continue
            rm = float(d_room[j, i])
            score = float(geo[j, i]) + w_radial * abs(r - dist) + w_room * max(0.0, room - rm)
            out.append({'x': x, 'y': y, 'yaw': yaw, 'a': wrap_angle(a), 'r': r, 'score': score,
                        'geo': float(geo[j, i]), 'room': rm})
    out.sort(key=lambda o: o['score'])
    return out, why


def next_spot(cands, failed, min_dist):
    """실패한 자리(failed: [(x, y)] 또는 [(x, y, 이 자리만의 거리)])에서 min_dist(m) 이상 떨어진 후보 중
    점수가 가장 좋은 것. 없으면 None. 각도로 거르지 않는다: 채점한 후보는 각도가 가까워도 반경·경로가 다른 자리다.
    (도착은 했는데 팔이 조금 안 닿은 자리는 작은 거리로 넣어, 같은 방향의 더 가까운 자리를 남긴다)"""
    for c in cands:
        if all(math.hypot(c['x'] - f[0], c['y'] - f[1]) >= (f[2] if len(f) > 2 else min_dist) for f in failed):
            return c
    return None
