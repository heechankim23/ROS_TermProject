"""노면 종류와 주행 비용 (ROS 무관 — 단위 테스트 가능).

로봇은 평소에 인도(보행로·횡단보도, 없으면 차도 바깥 길가)로 다니고, 차도는 횡단보도로 건넌다.
차도 주행 비용을 크게 둬서, 차도 위의 킥보드처럼 목표가 차도에 있을 때만 그 근처에서 차도로 나간다.
"""
import math

import numpy as np

OPEN, VERGE, WALK, ROAD = 0, 1, 2, 3
# 1 m 당 비용 배수 (인도 = 1). 휴리스틱이 직선거리를 쓰므로 1 보다 작으면 안 된다
FACTOR = np.array([2.0, 1.25, 1.0, 8.0], dtype=np.float32)

ROAD_HALF = {'service': 3.0, 'residential': 3.0}     # 차도 반폭 (지도 그림과 같음)
WALK_HALF = {'pedestrian': 2.0, 'footway': 1.5}
VERGE_WIDTH = 2.5                                    # 차도 바깥 길가 (보행로가 없는 길에서 인도 대신)


def _stamp(grid, a, b, half, value, ox, oy, res):
    """선분 a-b 에서 half 안에 중심이 있는 칸을 value 로"""
    h, w = grid.shape
    x0, x1 = min(a[0], b[0]) - half, max(a[0], b[0]) + half
    y0, y1 = min(a[1], b[1]) - half, max(a[1], b[1]) + half
    i0, i1 = max(0, int((x0 - ox) / res)), min(w, int((x1 - ox) / res) + 2)
    j0, j1 = max(0, int((y0 - oy) / res)), min(h, int((y1 - oy) / res) + 2)
    if i1 <= i0 or j1 <= j0:
        return
    xs = ox + (np.arange(i0, i1) + 0.5) * res
    ys = oy + (np.arange(j0, j1) + 0.5) * res
    dx, dy = b[0] - a[0], b[1] - a[1]
    L2 = dx * dx + dy * dy
    X, Y = np.meshgrid(xs - a[0], ys - a[1])
    t = np.clip((X * dx + Y * dy) / L2, 0.0, 1.0) if L2 > 1e-12 else 0.0
    grid[j0:j1, i0:i1][np.hypot(X - t * dx, Y - t * dy) <= half] = value


def build_surface(meta, ox, oy, res, shape):
    """지도 json(roads, crosswalks) -> 노면 종류 격자 [j(y), i(x)] uint8.
    crosswalks: [{x, y, yaw_deg(건너는 방향), length(건너는 길이), width}]"""
    grid = np.full(shape, OPEN, dtype=np.uint8)
    segs = [(r['kind'], a, b) for r in meta.get('roads', []) for a, b in zip(r['pts'][:-1], r['pts'][1:])]
    for kind, a, b in segs:
        if kind in ROAD_HALF:
            _stamp(grid, a, b, ROAD_HALF[kind] + VERGE_WIDTH, VERGE, ox, oy, res)
    for kind, a, b in segs:
        if kind in WALK_HALF:
            _stamp(grid, a, b, WALK_HALF[kind], WALK, ox, oy, res)
    for kind, a, b in segs:                         # 차도는 보행로를 덮는다: 건너는 곳은 횡단보도만 싸다
        if kind in ROAD_HALF:
            _stamp(grid, a, b, ROAD_HALF[kind], ROAD, ox, oy, res)
    for c in meta.get('crosswalks', []):
        _fill_rect(grid, c, WALK, ox, oy, res)
    return grid


def _fill_rect(grid, c, value, ox, oy, res):
    th = math.radians(c['yaw_deg'])
    ct, st = math.cos(th), math.sin(th)
    hl, hw = c['length'] / 2, c['width'] / 2
    r = math.hypot(hl, hw)
    h, w = grid.shape
    i0, i1 = max(0, int((c['x'] - r - ox) / res)), min(w, int((c['x'] + r - ox) / res) + 2)
    j0, j1 = max(0, int((c['y'] - r - oy) / res)), min(h, int((c['y'] + r - oy) / res) + 2)
    if i1 <= i0 or j1 <= j0:
        return
    X, Y = np.meshgrid(ox + (np.arange(i0, i1) + 0.5) * res - c['x'], oy + (np.arange(j0, j1) + 0.5) * res - c['y'])
    grid[j0:j1, i0:i1][(np.abs(X * ct + Y * st) <= hl) & (np.abs(-X * st + Y * ct) <= hw)] = value
