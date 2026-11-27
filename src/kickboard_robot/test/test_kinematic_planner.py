"""운동학 제약 경로 계획·추종 (kinematic_planner.py) — ROS 없이 돈다."""
import math
import random

import numpy as np

from kickboard_robot.kinematic_planner import (KinematicParams, PathTracker, CollisionGrid, Footprint, dubins_poses,
                                               dubins_shortest, max_curvature, path_collides, plan_path, wrap)
from kickboard_robot import reeds_shepp
from kickboard_robot.motion import car_limit, from_rear, to_rear
from kickboard_robot.planner import Costmap


def corridor_map():
    """20 x 12 m, 가운데 벽에 2.4 m 틈 하나 (로봇은 틈을 지나 돌아 들어가야 함)"""
    cm = Costmap(0.0, 0.0, 20.0, 12.0, res=0.1, inscribed=0.47, inflate=1.1)
    cm.static[0, :] = cm.static[-1, :] = True
    cm.static[:, 0] = cm.static[:, -1] = True
    cm.static[:, 98:102] = True             # x = 9.8 ~ 10.2 벽
    cm.static[48:72, 98:102] = False        # y = 4.8 ~ 7.2 틈
    return cm


def curvature_profile(path):
    """같은 방향으로 이어 달리는 연속 자세 사이 곡률들 (구간마다 리스트)"""
    segs, cur = [], []
    for a, b in zip(path[:-1], path[1:]):
        ds = math.hypot(b[0] - a[0], b[1] - a[1])
        if a[3] != b[3] or ds < 1e-6:
            if cur:
                segs.append(cur)
            cur = []
            continue
        cur.append(wrap(b[2] - a[2]) / ds)
    if cur:
        segs.append(cur)
    return segs


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
    assert max_curvature(path) <= p.kmax + 0.05            # 계획 경로 곡률 <= 1 / (최소 회전 반경 x 1.1)
    assert all(q[3] in (1, -1) for q in path)              # 제자리 회전 없음
    assert not path_collides(cm, path, 0.0, p)             # 차체(원 3개)가 벽과 안 겹침
    assert math.hypot(path[-1][0] - goal[0], path[-1][1] - goal[1]) <= p.goal_pos_tol
    # 벽 틈을 지난다 (x = 10 을 지날 때 y 는 틈 안)
    cross = [q for q in path if 9.7 < q[0] < 10.3]
    assert cross and all(4.8 < q[1] < 7.2 for q in cross)


def test_planned_curvature_is_continuous():
    """곡률이 계단처럼 튀지 않는다 (완화 곡선): 0.1 m 마다 곡률 변화 <= kmax * 0.1 / blend_len 의 1.5배 정도"""
    cm = corridor_map()
    p = KinematicParams()
    for start, goal in (((3.0, 2.0, math.pi / 2), (17.0, 9.0, 0.0)), ((5.0, 3.0, 0.0), (5.0, 9.0, math.pi))):
        path = plan_path(cm, start, goal, 0.0, p)
        assert path is not None and not path_collides(cm, path, 0.0, p)
        assert max_curvature(path) <= p.kmax + 0.05
        jumps = [abs(b - a) for seg in curvature_profile(path) for a, b in zip(seg[:-1], seg[1:])]
        assert not jumps or max(jumps) <= 0.2              # 연속화 전에는 0.5 (0 -> 1/R 한 번에)
        assert math.hypot(path[-1][0] - goal[0], path[-1][1] - goal[1]) <= p.goal_pos_tol


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
        worst_k, worst_dev, k_prev, worst_dk = 0.0, 0.0, None, 0.0
        for _ in range(20000):
            v, w, done = tr.step((x, y, th), dt=0.05)
            if done:
                break
            assert abs(v) <= v_max + 1e-9 and abs(w) <= w_max + 1e-9
            if abs(v) > 0.05:
                worst_k = max(worst_k, abs(w / v))
                k = w / v
                if k_prev is not None:
                    worst_dk = max(worst_dk, abs(k - k_prev))
                k_prev = k
            else:
                k_prev = None
            th = wrap(th + w * 0.05)
            x += v * math.cos(th) * 0.05
            y += v * math.sin(th) * 0.05
            worst_dev = max(worst_dev, tr.cross_track((x, y, th)))
        assert math.hypot(x - goal[0], y - goal[1]) < 0.2
        assert abs(wrap(th - goal[2])) < math.radians(10)
        assert worst_k <= 1.0 / p.min_turn_radius + 1e-6     # 주행 중에도 최소 회전 반경을 지킴
        assert worst_dev < 0.25
        assert worst_dk <= tr.k_rate_max * 0.05 + 1e-6     # 조향 속도 한계 (명령 곡률이 한 주기에 튀지 않음)


def test_footprint_circles_cover_body():
    p = KinematicParams()
    fp = Footprint(p)
    for u in np.linspace(p.body_x_min, p.body_x_max, 20):
        for v in np.linspace(-p.body_half_width, p.body_half_width, 7):
            assert any(math.hypot(u - cx, v) <= fp.r for cx in fp.cx)
    cg = CollisionGrid(corridor_map(), 0.0, fp)
    assert cg.free(5.0, 6.0, 0.0) and not cg.free(9.0, 2.0, 0.0)


def open_map():
    cm = Costmap(0.0, 0.0, 20.0, 14.0, res=0.1, inscribed=0.47, inflate=1.1)
    cm.static[0, :] = cm.static[-1, :] = True
    cm.static[:, 0] = cm.static[:, -1] = True
    return cm


def drive(path, start, p, dt=0.05):
    """뒤축 자세를 추종기로 몰아 끝 자세와 명령 목록을 돌려준다"""
    tr = PathTracker(path, v_max=1.0, w_max=1.0, min_turn_radius=p.min_turn_radius)
    x, y, th = start
    cmds = []
    for _ in range(6000):
        v, w, done = tr.step((x, y, th), dt=dt)
        assert not tr.lost
        if done:
            return (x, y, th), cmds
        cmds.append((v, w))
        x, y = x + v * math.cos(th + w * dt / 2) * dt, y + v * math.sin(th + w * dt / 2) * dt
        th = wrap(th + w * dt)
    raise AssertionError('끝까지 못 감')


def test_reeds_shepp_connects_random_poses():
    random.seed(1)
    for _ in range(300):
        q0 = (random.uniform(-5, 5), random.uniform(-5, 5), random.uniform(-3.1, 3.1))
        q1 = (q0[0] + random.uniform(-6, 6), q0[1] + random.uniform(-6, 6), random.uniform(-3.1, 3.1))
        found = reeds_shepp.paths(q0, q1, 1.8)
        assert found
        best = min(sum(abs(v) for _, v in segs) for segs in found)
        dub = sum(v for _, v in dubins_shortest(q0, q1, 1.8))
        assert best <= dub + 1e-6                          # 후진까지 쓰면 전진만(Dubins)보다 길 수 없다
        for segs in found:
            end = reeds_shepp.end_pose(q0, segs, 1.8)
            assert math.hypot(end[0] - q1[0], end[1] - q1[1]) < 2e-3 and abs(wrap(end[2] - q1[2])) < 2e-3


def test_turn_back_without_pivot():
    """계획 문서 3단계 완료 기준: 1.6 m 앞, 방향 163° 반대 목표를 제자리 회전 없이 도착"""
    cm, p = open_map(), KinematicParams()
    start = (8.0, 7.0, 0.0)
    goal = (9.6, 7.0, math.radians(163))
    path = plan_path(cm, start, goal, 0.0, p)
    assert path is not None and not path_collides(cm, path, 0.0, p)
    assert sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(path[:-1], path[1:])) < 14.0
    end, cmds = drive(path, start, p)
    assert math.hypot(end[0] - goal[0], end[1] - goal[1]) < 0.25 and abs(wrap(end[2] - goal[2])) < math.radians(12)
    assert all(abs(w) <= abs(v) / p.min_turn_radius + 1e-6 for v, w in cmds)    # 서서 도는 명령 없음


def test_car_limit_and_rear_axle():
    assert car_limit(0.0, 1.0) == (0.0, 0.0)
    v, w = car_limit(-0.36, 1.0)
    assert v == -0.36 and abs(w - 0.2) < 1e-9
    pose = (2.0, 1.0, math.pi / 2)
    rear = to_rear(pose)
    assert math.isclose(rear[0], 2.0, abs_tol=1e-9) and math.isclose(rear[1], 0.38)
    assert all(math.isclose(a, b) for a, b in zip(from_rear(rear), pose))


def test_forward_first_after_rear_drop():
    """뒤에 킥보드를 내린 뒤: 목표가 뒤·옆에 있어도 처음 1.5 m 는 전진으로 시작하는 경로를 고른다"""
    cm, p = open_map(), KinematicParams()
    start = (8.0, 7.0, 0.0)
    for goal in ((8.0, 7.16, 0.0), (6.0, 7.0, 0.0)):           # 옆 칸 (16 cm 옆), 2 m 뒤
        if goal[0] < start[0]:                                  # 바로 뒤 목표: 제한이 없으면 후진부터
            assert plan_path(cm, start, goal, 0.0, p)[0][3] == -1
        path = plan_path(cm, start, goal, 0.0, p, forward_first=1.5)
        assert path is not None
        travelled, prev = 0.0, start
        for q in path:
            if q[3] < 0:
                break
            travelled += math.hypot(q[0] - prev[0], q[1] - prev[1])
            prev = q
        assert travelled >= 1.4, (goal, travelled)
        end, _ = drive(path, start, p)
        assert math.hypot(end[0] - goal[0], end[1] - goal[1]) < 0.25
