import math
import random

import numpy as np

from kickboard_robot import queue_logic as ql
from kickboard_robot.geo import GeoConverter
from kickboard_robot.motion import GoToController, standoff_point


def test_geo_roundtrip_under_10cm():
    g = GeoConverter(37.5665, 126.9780)
    for _ in range(200):
        x, y = random.uniform(-200, 200), random.uniform(-200, 200)
        lat, lon = g.to_latlon(x, y)
        x2, y2 = g.to_map(lat, lon)
        assert math.hypot(x - x2, y - y2) < 0.1


def test_merge_keeps_aging_and_scores():
    w = ql.Weights(10, 1, 5)
    far = ql.Item('far', ql.LOW_BATTERY, 10, 0, 0, 0, t_enqueued=0)
    near = ql.Item('near', ql.FALLEN, 1, 0, 0, 0, t_enqueued=50)
    # t=50: near 이 우선
    assert ql.ranked([far, near], 50, (0, 0), w)[0].id == 'near'
    # 상한을 넘기면 far 가 어떤 새 항목에도 밀리지 않음
    tmax = ql.starvation_bound(w, d_max=10)
    fresh = ql.Item('new', ql.FALLEN, 0, 0, 0, 0, t_enqueued=tmax + 1)
    assert ql.ranked([far, fresh], tmax + 1, (0, 0), w)[0].id == 'far'


def test_standoff_and_controller_converge():
    gx, gy, gyaw = standoff_point((5, 0), (0, 0), 0.8)
    assert abs(gx - 4.2) < 1e-9 and abs(gy) < 1e-9 and abs(gyaw) < 1e-9
    c = GoToController()
    x, y, th = 0.0, 0.0, math.pi / 2
    for _ in range(4000):
        v, w, done = c.step((x, y, th), (gx, gy, gyaw))
        if done:
            break
        th += w * 0.05
        x += v * math.cos(th) * 0.05
        y += v * math.sin(th) * 0.05
    assert done and math.hypot(x - gx, y - gy) < 0.06


def test_free_standoff_avoids_walls():
    import os
    from kickboard_robot.occupancy import OccupancyMap, free_standoff
    here = os.path.dirname(__file__)
    m = OccupancyMap(os.path.join(here, '..', '..', 'kickboard_sim', 'maps', 'kickboard_world.yaml'))
    assert m.is_free(0.0, 0.0, 0.45)
    assert not m.is_free(4.8, 0.0, 0.45)          # 동쪽 벽(x=5) 근처
    # GPS 오차로 킥보드가 벽 밖(5.4, -0.6)에 찍힌 경우에도 목표는 벽에서 0.45 m 이상
    g = free_standoff(m, (5.4, -0.6), (-4.0, 0.0), 0.8, 0.45)
    assert g is not None and m.is_free(g[0], g[1], 0.45)
    # 선반(0, 3.6) 바로 아래 킥보드: 로봇 쪽 방향이 막히면 다른 각도로
    g = free_standoff(m, (0.0, 2.9), (0.0, 5.0), 0.8, 0.45)
    assert g is not None and m.is_free(g[0], g[1], 0.45)


def test_drop_zone_spots():
    from kickboard_robot.drop_zones import choose_plan, edge_plans, free_spots
    zone = [[-3.0, 2.3], [-1.0, 2.3], [-1.0, 3.7], [-3.0, 3.7]]
    plans = edge_plans('A', zone)
    assert len(plans) == 4
    south = min(plans, key=lambda p: p['robot'][1])
    assert abs(south['robot'][1] - (2.3 - 0.95)) < 1e-9
    assert abs(south['robot'][2] - math.pi / 2) < 1e-9          # 구역(북쪽)을 바라봄
    assert len(south['spots']) == 4
    for x, y, yaw in south['spots']:
        assert -3.0 < x < -1.0 and 2.3 < y < 3.7                  # 구역 안
        # 로봇팔 회전축(로봇 중심 앞 0.5 m)에서 닿는 거리
        assert math.hypot(x - south['robot'][0], y - (south['robot'][1] + 0.5)) < 1.2
    plan, fs = choose_plan(plans, [], (-2.0, 0.0))
    assert plan is south and len(fs) == 4
    used = [(s[0], s[1]) for s in south['spots']]
    assert free_spots(south, used) == []
    plan2, _ = choose_plan(plans, used, (-2.0, 0.0))
    assert plan2 is not south


def test_charger_spots_reachable_from_dock():
    from kickboard_robot.drop_zones import charger_spots
    spots = charger_spots(-4.548, 0.0, 0.0)
    assert len(spots) == 4
    turret = (-4.548 + 1.0 - 0.5, 0.0)          # 도킹 거리 1.0, 로봇 중심 앞 0.5 m (로봇은 -x 를 봄)
    for x, y, yaw in spots:
        assert abs(y) >= 0.8 - 1e-9                # 로봇(폭 ±0.45)과 충전소(±0.3) 옆
        assert math.hypot(x - turret[0], y - turret[1]) < 1.3


def test_planner_avoids_obstacle():
    from kickboard_robot.planner import Costmap, free_spot
    cm = Costmap(0.0, 0.0, 6.0, 4.0, res=0.1)
    cm.static[0, :] = cm.static[-1, :] = True
    cm.static[:, 0] = cm.static[:, -1] = True
    # 가운데 기둥 (킥보드처럼 라이다로 본 장애물): x 3.0, y 1.0~3.0
    cm.update_scan(1.0, 2.0, 0.0, [2.0] * 3, -0.1, 0.1, 0.12, 3.5, t=0.0)
    for y in np.arange(1.0, 3.01, 0.1):
        j, i = cm.cell(3.0, y)
        cm.seen[j, i] = 0.0
    path = cm.plan((1.0, 2.0), (5.0, 2.0), t=0.0)
    assert path is not None and abs(path[-1][0] - 5.0) < 1e-9
    d = cm.distance(0.0)
    pts = [(1.0, 2.0)] + path
    for a, b in zip(pts[:-1], pts[1:]):
        for k in range(21):
            x, y = a[0] + (b[0] - a[0]) * k / 20, a[1] + (b[1] - a[1]) * k / 20
            assert d[cm.cell(x, y)] >= 0.47 - 1e-6        # 로봇 반폭보다 가까이 가지 않음
    g = free_spot(cm, (3.0, 2.0), (1.0, 2.0), 1.3, 0.85, 0.0)
    assert g is not None and cm.dist_at(g[0], g[1], 0.0) >= 0.85


def test_sim_localizer_map_odom():
    from kickboard_robot.sim_localizer import compose_map_odom
    # odom 원점이 map (-3.548, 0) 에서 180° 돌아 있고, 로봇이 odom 에서 (1, 0.5, 0.3) 에 있을 때
    mx, my, myaw = -3.548, 0.0, math.pi
    ox, oy, oth = 1.0, 0.5, 0.3
    truth = (mx + math.cos(myaw) * ox - math.sin(myaw) * oy, my + math.sin(myaw) * ox + math.cos(myaw) * oy,
             myaw + oth)
    x, y, yaw = compose_map_odom(truth, (ox, oy, oth))
    assert abs(x - mx) < 1e-9 and abs(y - my) < 1e-9 and abs(math.atan2(math.sin(yaw - myaw), math.cos(yaw - myaw))) < 1e-9


def test_planner_same_cell_goal():
    from kickboard_robot.planner import Costmap
    cm = Costmap(0.0, 0.0, 4.0, 4.0, res=0.1)
    path = cm.plan((2.01, 2.01), (2.04, 2.03), t=0.0)
    assert path == [(2.04, 2.03)]
