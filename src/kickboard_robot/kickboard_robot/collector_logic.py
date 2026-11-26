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


# ---------------- 태그 면 구분 (킥보드 방향을 알아야 뒤로 붙을 수 있음) ----------------
# 킥보드 한 대에 태그 4장: 앞·뒤·왼·오른 면. ID = 기본 ID + FACE_ID_STEP * 면 번호 (AprilTag 36h11 은 0~586)
FACE_ID_STEP = 150
FACE_NORMAL = (0.0, math.pi, math.pi / 2, -math.pi / 2)   # 면 법선 (킥보드 기준 각)
FACE_DEPTH = (0.30, 0.30, 0.085, 0.085)                  # 면 -> 킥보드 중심 (gz_utils SCOOTER_L/2, SCOOTER_W/2)


def face_tag_id(base, face):
    return base + FACE_ID_STEP * face


def split_tag_id(tid):
    """태그 ID -> (기본 ID, 면 번호 0~3)"""
    face = tid // FACE_ID_STEP
    return (tid % FACE_ID_STEP, face) if face < 4 else (tid, 0)


def scooter_from_face(fx, fy, fyaw, face):
    """태그 면 pose + 면 번호 -> 킥보드 (중심 x, 중심 y, 진행 방향 yaw)"""
    d = FACE_DEPTH[face]
    return fx - d * math.cos(fyaw), fy - d * math.sin(fyaw), wrap_angle(fyaw - FACE_NORMAL[face])


# ---------------- v19 후진 적재 자리 ----------------
# 적재 애니메이션 시작 때 킥보드 앞바퀴 축은 로봇 기준 (-1.65, 레인 y), 킥보드 앞이 로봇 뒤를 봄 (같은 방향).
LOAD_AXLE_X = -1.65          # scootover_rig.json ref_scooter_l@robot 첫 프레임
G30_AXLE_X = 0.42            # 킥보드 모델 원점(바퀴 축 사이 가운데) -> 앞바퀴 축
LANE_Y = (0.27, -0.27)       # 레인 0 = 왼쪽, 1 = 오른쪽 (arm_loader)
ENTRY_DIST = G30_AXLE_X - LOAD_AXLE_X   # 킥보드 중심 -> 로봇 중심 (진행 방향으로) 2.07 m


def reverse_entry_pose(cx, cy, heading, lane=0, extra=0.0):
    """킥보드 (중심, 방향) -> 로봇이 서야 할 자리 (x, y, yaw). 로봇은 킥보드와 같은 방향, 킥보드 앞쪽에 서서 뒤를 댄다.
    extra: 진행 방향으로 더 떨어진 자리 (후보용)"""
    ax, ly = ENTRY_DIST + extra, -LANE_Y[lane]
    c, s = math.cos(heading), math.sin(heading)
    return cx + c * ax - s * ly, cy + s * ax + c * ly, wrap_angle(heading)


def entry_error(robot, cx, cy, heading, lane=0):
    """로봇 pose 와 후진 적재 자리의 차이 (앞뒤, 옆, yaw) — 적재 자리 기준 좌표"""
    tx, ty, tyaw = reverse_entry_pose(cx, cy, heading, lane)
    dx, dy = robot[0] - tx, robot[1] - ty
    c, s = math.cos(tyaw), math.sin(tyaw)
    return c * dx + s * dy, -s * dx + c * dy, wrap_angle(robot[2] - tyaw)


def dock_errors(pose, target):
    """적재 자리 좌표에서 본 로봇: (남은 거리 along: + = 아직 앞쪽, 옆 lat: 왼쪽 +, 방향 head)"""
    tx, ty, tyaw = target
    c, s = math.cos(tyaw), math.sin(tyaw)
    dx, dy = pose[0] - tx, pose[1] - ty
    return c * dx + s * dy, -s * dx + c * dy, wrap_angle(pose[2] - tyaw)


def dock_step(pose, target, goal_along=0.0, v_max=0.25, w_max=0.6, look=0.5, max_head=math.radians(25)):
    """적재 자리를 지나는 직선(방향 = target yaw)을 따라 along = goal_along 까지 가는 한 주기. -> (v, w, done)
    goal_along 이 지금보다 작으면 후진(도킹), 크면 전진(다시 빼기).
    옆 오차는 '선 쪽으로 기울인 방향'을 목표로 해 달리면서 줄인다 (제자리 회전 없음)."""
    ea, el, ey = dock_errors(pose, target)
    rem = ea - goal_along                       # + 면 후진해야 함
    if abs(rem) <= 0.02:
        return 0.0, 0.0, True
    back = rem > 0
    want = math.atan2(el, look)                 # 후진: el_dot = v sin(ey), v < 0 이므로 ey > 0 이 el 을 줄임
    want = max(-max_head, min(max_head, want if back else -want))
    w = max(-w_max, min(w_max, 2.0 * wrap_angle(want - ey)))
    v = max(0.05, min(v_max, 0.6 * abs(rem)))
    if abs(wrap_angle(want - ey)) > math.radians(20):
        v = 0.05                                # 방향이 많이 틀렸으면 거의 서서 돌림
    return (-v if back else v), w, False


def dock_ok(pose, target, pos_tol=0.04, lat_tol=0.07):
    ea, el, _ = dock_errors(pose, target)
    return abs(ea) <= pos_tol and abs(el) <= lat_tol


def fine_align_plan(robot, target, pos_tol, yaw_tol, max_move, lat_tol=None):
    """정렬 이동이 끝난 뒤 적재 자리(target = (x, y, yaw))에 정확히 서기 위한 짧은 동작 목록.
    차동 구동이라 옆으로는 못 가므로: 옆 오차가 lat_tol(없으면 pos_tol) 이하이면 [방향 맞춤, 앞뒤 직진],
    아니면 [자리 쪽(또는 반대쪽, 덜 도는 쪽)으로 회전, 직진(후진은 음수), 방향 맞춤].
    -> [('turn', 절대 yaw) | ('drive', 진행 방향 거리 m)] / [] (이미 허용 오차 안) / None (max_move 보다 멀어 직접 못 맞춤)"""
    tx, ty, tyaw = target
    c, s = math.cos(tyaw), math.sin(tyaw)
    dx, dy = robot[0] - tx, robot[1] - ty
    ea, el = c * dx + s * dy, -s * dx + c * dy
    ey = wrap_angle(robot[2] - tyaw)
    lat_tol = pos_tol if lat_tol is None else lat_tol     # 옆 맞춤은 두 번 크게 돌아야 해서 따로 느슨하게 둘 수 있음
    if abs(ea) <= pos_tol and abs(el) <= lat_tol and abs(ey) <= yaw_tol:
        return []
    d = math.hypot(dx, dy)
    if d > max_move:
        return None
    if abs(el) <= lat_tol:
        return ([('turn', tyaw)] if abs(ey) > yaw_tol else []) + ([('drive', -ea)] if abs(ea) > pos_tol else [])
    b = math.atan2(-dy, -dx)                       # 로봇 -> 자리 방향
    fwd, back = b, wrap_angle(b + math.pi)
    # 전진/후진 중 지금 방향에서 덜 도는 쪽 (어차피 끝에 tyaw 로 돌아오므로 tyaw 기준으로 고름)
    if abs(wrap_angle(fwd - tyaw)) <= abs(wrap_angle(back - tyaw)):
        return [('turn', fwd), ('drive', d), ('turn', tyaw)]
    return [('turn', back), ('drive', -d), ('turn', tyaw)]


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
BODY_X = (-0.98, 0.96)       # 로봇 차체 x 범위 (v19: 접힌 램프 포함, base_footprint 기준, nav_bridge footprint 와 같게)
BODY_HALF_W = 0.58           # 차체 반폭 0.525 + 여유
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
                     keep_clear=None, w_radial=2.0, w_room=3.0, room=1.1, entry=None):
    """팔이 닿는 자리 후보를 검사·채점한다.
    -> (후보 목록, 탈락 사유별 개수). 후보 = dict(x, y, yaw, a(킥보드에서 본 각도), r, score, geo, room), 점수 오름차순.
    keep_clear: 진입 금지 경계와 로봇 중심 사이 최소 거리 (None 이면 진입 금지 구역을 벽처럼 취급).
    entry=(heading, lane): v19 후진 적재 — 후보는 reverse_entry_pose 자리(radii = 진행 방향 추가 거리)만,
    팔 띠 대신 로봇 뒤끝 -> 킥보드 사이(램프가 펴지는 자리)를 검사한다."""
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
    if entry is None:
        poses = [(r, math.radians(m * step_deg), *align_pose(cx, cy, r, math.radians(m * step_deg)))
                 for r in radii for m in range(int(round(360.0 / step_deg)))]
        pivot = ARM_PIVOT_X
    else:
        poses = [(dist + ex, entry[0], *reverse_entry_pose(cx, cy, entry[0], entry[1], ex)) for ex in radii]
        pivot = BODY_X[0]
    for r, a, x, y, yaw in poses:
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
        ax, ay = x + pivot * c, y + pivot * s
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
