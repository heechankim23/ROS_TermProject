"""운동학 제약 경로 계획·추종 (kinematic_planner.py) — ROS 없이 돈다."""
import math
import random

import numpy as np

from kickboard_robot.kinematic_planner import (KinematicParams, PathTracker, CollisionGrid, Footprint, dubins_poses,
                                               max_curvature, path_collides, plan_path, wrap)
from kickboard_robot.planner import Costmap


def corridor_map():
    """20 x 12 m, 가운데 벽에 2.4 m 틈 하나 (로봇은 틈을 지나 돌아 들어가야 함)"""
    cm = Costmap(0.0, 0.0, 20.0, 12.0, res=0.1, inscribed=0.47, inflate=1.1)
    cm.static[0, :] = cm.static[-1, :] = True
    cm.static[:, 0] = cm.static[:, -1] = True
    cm.static[:, 98:102] = True             # x = 9.8 ~ 10.2 벽
    cm.static[48:72, 98:102] = False        # y = 4.8 ~ 7.2 틈
    return cm


def mid_path_pivots(path):
    return sum(1 for k, q in enumerate(path) if q[3] == 0 and 0 < k < len(path) - 1
               and path[k - 1][3] != 0 and path[k + 1][3] != 0)


def test_dubins_reaches_pose_within_curvature():
    random.seed(0)
    for _ in range(100):
        q0 = (random.uniform(-4, 4), random.uniform(-4, 4), random.uniform(-3, 3))
        q1 = (random.uniform(-4, 4), random.uniform(-4, 4), random.uniform(-3, 3))
        ps = dubins_poses(q0, q1, 1.0)
        assert math.hypot(ps[-1][0] - q1[0], ps[-1][1] - q1[1]) < 1e-3
        assert abs(wrap(ps[-1][2] - q1[2])) < 1e-3
        assert max_curvature([(q0[0], q0[1], q0[2], 1)] + ps) <= 1.0 + 0.05


def test_plan_respects_curvature_and_footprint():
    cm = corridor_map()
    p = KinematicParams()
    start, goal = (3.0, 2.0, math.pi / 2), (17.0, 9.0, 0.0)
    path = plan_path(cm, start, goal, 0.0, p)
    assert path is not None
    assert max_curvature(path) <= p.kmax + 0.05            # 계획 경로 곡률 <= 1 / (최소 회전 반경 x 1.25)
    assert mid_path_pivots(path) == 0                      # 경로 중간에 제자리 회전(꺾임) 없음
    assert not path_collides(cm, path, 0.0, p)             # 차체(원 3개)가 벽과 안 겹침
    assert math.hypot(path[-1][0] - goal[0], path[-1][1] - goal[1]) <= p.goal_pos_tol
    # 벽 틈을 지난다 (x = 10 을 지날 때 y 는 틈 안)
    cross = [q for q in path if 9.7 < q[0] < 10.3]
    assert cross and all(4.8 < q[1] < 7.2 for q in cross)


def test_no_path_through_closed_wall():
    cm = corridor_map()
    cm.static[48:72, 98:102] = True                         # 틈 막음
    p = KinematicParams(time_budget_sec=1.0)
    assert plan_path(cm, (3.0, 6.0, 0.0), (17.0, 6.0, 0.0), 0.0, p) is None


def test_lidar_obstacle_on_route_is_avoided():
    cm = corridor_map()
    cm.seen[50:70, 60:64] = 5.0                            # x = 6 근처 라이다로 본 장애물 (2 m 폭)
    p = KinematicParams()
    path = plan_path(cm, (3.0, 6.0, 0.0), (17.0, 6.0, 0.0), 5.0, p)
    assert path is not None and not path_collides(cm, path, 5.0, p)


def test_tracker_follows_within_turn_radius():
    cm = corridor_map()
    p = KinematicParams()
    start, goal = (3.0, 2.0, math.pi / 2), (17.0, 9.0, 0.0)
    path = plan_path(cm, start, goal, 0.0, p)
    for v_max, w_max in ((0.22, 1.2), (1.5, 1.0)):
        tr = PathTracker(path, v_max=v_max, w_max=w_max, min_turn_radius=p.min_turn_radius)
        x, y, th = start
        worst_k, worst_dev = 0.0, 0.0
        for _ in range(20000):
            v, w, done = tr.step((x, y, th))
            if done:
                break
            assert abs(v) <= v_max + 1e-9 and abs(w) <= w_max + 1e-9
            if abs(v) > 0.05:
                worst_k = max(worst_k, abs(w / v))
            th = wrap(th + w * 0.05)
            x += v * math.cos(th) * 0.05
            y += v * math.sin(th) * 0.05
            worst_dev = max(worst_dev, tr.cross_track((x, y, th)))
        assert math.hypot(x - goal[0], y - goal[1]) < 0.2
        assert abs(wrap(th - goal[2])) < math.radians(10)
        assert worst_k <= 1.0 / p.min_turn_radius + 1e-6     # 주행 중에도 최소 회전 반경을 지킴
        assert worst_dev < 0.25


def test_footprint_circles_cover_body():
    p = KinematicParams()
    fp = Footprint(p)
    for u in np.linspace(p.body_x_min, p.body_x_max, 20):
        for v in np.linspace(-p.body_half_width, p.body_half_width, 7):
            assert any(math.hypot(u - cx, v) <= fp.r for cx in fp.cx)
    cg = CollisionGrid(corridor_map(), 0.0, fp)
    assert cg.free(5.0, 6.0, 0.0) and not cg.free(9.0, 2.0, 0.0)
