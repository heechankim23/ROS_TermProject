"""주차구역 하역 자리 계산 (ROS 무관 — 단위 테스트 가능).

주차구역(볼록 다각형)의 각 변 바깥에 로봇이 서서 구역 쪽을 바라보고, 로봇팔이 변 안쪽 INSET 위치에
킥보드를 SPACING 간격으로 나란히 세운다 (킥보드 앞이 구역 안쪽을 향함).
로봇이 설 자리는 지도에서 벽·고정 장애물과 clearance 이상 떨어져야 한다.
"""
import math

ROBOT_STANDOFF = 0.95   # 구역 변 -> 로봇 중심 (앞 범퍼 0.75 -> 변 밖 0.2 m)
INSET = 0.45            # 구역 변 -> 킥보드 중심 (구역 안쪽)
SPACING = 0.45          # 나란히 세운 킥보드 간격 (핸들 폭 0.42)
EDGE_MARGIN = 0.3       # 변 양 끝 여유
MIN_GAP = 0.40          # 이미 세운 킥보드와 이보다 가까운 자리는 쓰지 않음


def edge_plans(name, poly, omap=None, clearance=0.95, max_spots=4):
    """구역의 각 변마다 {'zone', 'robot': (x, y, yaw), 'spots': [(x, y, yaw), ...]} 후보 목록."""
    n = len(poly)
    cx, cy = sum(p[0] for p in poly) / n, sum(p[1] for p in poly) / n
    out = []
    for i in range(n):
        (x1, y1), (x2, y2) = poly[i], poly[(i + 1) % n]
        length = math.hypot(x2 - x1, y2 - y1)
        if length < 1e-6:
            continue
        tx, ty = (x2 - x1) / length, (y2 - y1) / length
        mx, my = (x1 + x2) / 2, (y1 + y2) / 2
        nx, ny = -ty, tx
        if (mx - cx) * nx + (my - cy) * ny < 0:          # 바깥 방향 법선
            nx, ny = -nx, -ny
        rx, ry = mx + nx * ROBOT_STANDOFF, my + ny * ROBOT_STANDOFF
        if omap is not None and not omap.is_free(rx, ry, clearance):
            continue
        k = min(max_spots, int((length - 2 * EDGE_MARGIN) / SPACING) + 1)
        if k <= 0:
            continue
        yaw_in = math.atan2(-ny, -nx)
        spots = []
        for j in range(k):
            off = (j - (k - 1) / 2) * SPACING
            spots.append((mx - nx * INSET + tx * off, my - ny * INSET + ty * off, yaw_in))
        out.append({'zone': name, 'robot': (rx, ry, yaw_in), 'spots': spots})
    return out


def free_spots(plan, used):
    """이미 킥보드를 세운 자리(used: [(x, y)])와 겹치지 않는 자리들"""
    return [s for s in plan['spots']
            if all(math.hypot(s[0] - u[0], s[1] - u[1]) >= MIN_GAP for u in used)]


def choose_plan(plans, used, robot_xy, need=1):
    """빈 자리가 있는 후보 중 가장 가까운 것. 빈 자리가 need 개 이상인 후보를 우선. 없으면 None."""
    best = None
    for p in plans:
        fs = free_spots(p, used)
        if not fs:
            continue
        d = math.hypot(p['robot'][0] - robot_xy[0], p['robot'][1] - robot_xy[1])
        key = (len(fs) < need, d)
        if best is None or key < best[0]:
            best = (key, p, fs)
    return None if best is None else (best[1], best[2])


# 충전소 충전 자리: 충전소 태그 면 앞, 도킹한 로봇의 양옆 (로봇팔이 닿는 곳)
CHARGER_FRONT = 0.25            # 태그 면 -> 킥보드 중심 (태그 법선 방향)
CHARGER_LATERAL = (0.8, -0.8, 1.25, -1.25)   # 좌우 (오른쪽부터 쓰려면 순서를 바꾼다)


def charger_spots(tag_x, tag_y, tag_yaw):
    """충전소 태그 pose(바깥 법선 yaw) -> 충전 자리 [(x, y, yaw)]. 킥보드 앞이 충전소를 향함"""
    nx, ny = math.cos(tag_yaw), math.sin(tag_yaw)
    tx, ty = -ny, nx
    bx, by = tag_x + nx * CHARGER_FRONT, tag_y + ny * CHARGER_FRONT
    return [(bx + tx * s, by + ty * s, tag_yaw + math.pi) for s in CHARGER_LATERAL]
