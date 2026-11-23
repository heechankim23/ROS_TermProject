#!/usr/bin/env python3
"""노드 가공부(*_logic.py, planner) 오프라인 벤치마크. ROS 없이 돈다.

  python3 tools/bench_logic.py            # 전체
  python3 tools/bench_logic.py nav lidar  # 이름에 해당 단어가 들어간 항목만
  python3 tools/bench_logic.py --json out.json   # 결과 저장 (최적화 전후 비교용)

입력은 실제 지도(kickboard_sim/maps)와 시나리오 크기(킥보드 5~7대, 라이다 360빔)로 만든다.
값은 '1회 호출 평균'. 노드 안에서의 실제 비용은 KICKBOARD_PROFILE=1 로 측정한다 (docs/PIPELINE.md).
"""
import json
import math
import os
import random
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'src', 'kickboard_robot'))

from kickboard_robot import battery_logic, dock_logic, lidar_logic, localizer_logic  # noqa: E402
from kickboard_robot import mission_logic as ml, nav_logic, queue_book, queue_logic as ql, tag_sim_logic  # noqa: E402
from kickboard_robot.occupancy import OccupancyMap  # noqa: E402
from kickboard_robot.planner import Costmap, free_spot  # noqa: E402

MAP_YAML = os.path.join(ROOT, 'src', 'kickboard_sim', 'maps', 'kickboard_world.yaml')
SCOOTERS = [('1', 1.5, -1.0, 0.5), ('2', 0.5, 1.5, 0.8), ('3', 2.0, -1.0, 1.6), ('4', 3.0, 1.0, 0.0),
            ('5', -2.0, -1.5, 2.1), ('6', -2.6, 2.0, 0.0), ('7', -1.8, -2.8, 0.0)]
ROBOT = (-1.0, 0.0, 0.3)


def bench(fn, min_sec=0.3, min_n=5):
    """fn 을 min_sec 이상 반복 -> 1회 평균 (s)"""
    fn()                                   # 워밍업
    n, t0 = 0, time.perf_counter()
    while True:
        fn()
        n += 1
        dt = time.perf_counter() - t0
        if dt >= min_sec and n >= min_n:
            return dt / n, n


def cases():
    omap = OccupancyMap(MAP_YAML)
    rng = random.Random(0)

    # nav: 라이다 360빔 전처리 / 코스트맵 갱신 / A* / 빈 자리 찾기 / 추종 1 스텝
    grid, res, ox, oy = lidar_logic.static_grid_from(omap)
    sim = lidar_logic.LidarSim(grid, res, ox, oy)
    scan = sim.scan(ROBOT, 0.8, SCOOTERS, {'1': 90.0}, 0.01, rng)
    cm = Costmap.from_occupancy(omap, inscribed=0.47, inflate=1.1, decay_sec=60.0)
    fp = (-0.92, 0.89, 0.50)
    _, own, r = nav_logic.filter_scan(scan, sim.angles[0], sim.angles[1] - sim.angles[0], 0.12, 0.8, fp)
    sx, sy, syaw = nav_logic.sensor_pose(ROBOT, 0.8)
    t = [0.0]

    def cm_update():
        t[0] += 0.2
        cm.update_scan(sx, sy, syaw, r, float(sim.angles[0]), float(sim.angles[1] - sim.angles[0]), 0.12, 3.5,
                       t[0], ignore=own)

    cm_update()
    follower = nav_logic.PathFollower((3.0, -1.0, 0.0), nav_logic.FollowParams(), cm=cm)

    yield 'nav.filter_scan (360빔)', lambda: nav_logic.filter_scan(
        scan, sim.angles[0], sim.angles[1] - sim.angles[0], 0.12, 0.8, fp)
    yield 'nav.costmap_update_scan', cm_update
    yield 'nav.plan (A*, 방 대각선)', lambda: cm.plan((-3.0, 0.0), (3.5, -2.5), t[0])
    yield 'nav.free_spot (standoff 1.6 m)', lambda: free_spot(cm, (1.5, -1.0), ROBOT[:2], 1.6, 1.1, t[0])
    yield 'nav.follower_step (재계획 포함)', lambda: follower.step(ROBOT, t[0] + 2.0)

    # sim
    yield 'lidar.scan (7대, 360빔)', lambda: sim.scan(ROBOT, 0.8, SCOOTERS, {'1': 90.0}, 0.01, rng)
    cam = tag_sim_logic.camera_pose(ROBOT, 0.07)
    model = tag_sim_logic.CameraModel()
    yield 'tag_sim.detect (7대)', lambda: tag_sim_logic.detect(cam, SCOOTERS, (-4.548, 0.0, 0.0), 100, model, rng)
    f = localizer_logic.MapOdomFilter()
    yield 'localizer.update', lambda: f.update((1.0, 0.5, 0.2), (0.3, 0.1, 0.05))

    # queue / mission / battery / dock
    book, w = queue_book.QueueBook(), ql.Weights()
    for k in range(50):
        book.enqueue(str(k), 1 + k % 3, rng.uniform(-4, 4), rng.uniform(-3, 3), 0, 0, float(k))

    def select_and_requeue():
        it, _, _ = book.select(100.0, ROBOT[:2], w)
        book.update(it.id, 'REQUEUE')

    yield 'queue.select (50건)', select_and_requeue
    yield 'queue.rows (50건, 발행용)', lambda: book.rows(100.0, ROBOT[:2], w)
    cargo = ml.Cargo()
    cargo.add('1', ml.PARKING)
    p = ml.MissionParams(capacity=4)
    yield 'mission.decide_tick+select', lambda: (ml.decide_tick(ml.SELECTING, 80.0, p, 3, 0.0, False, False),
                                                 ml.decide_select(False, cargo, 80.0, p))
    b = battery_logic.BatteryModel()
    yield 'battery.add_odom+update', lambda: (b.add_odom(rng.random(), 0.0), b.update(time.monotonic(), 'NAVIGATING'))
    _, dock, _ = dock_logic.charger_poses({'tag_pose': {'x': -4.548, 'y': 0.0, 'yaw_deg': 0.0},
                                           'dock_distance': 1.0, 'pre_dock_distance': 2.0})
    df = dock_logic.DockGoalFilter(dock, 1.0)
    yield 'dock.goal_filter', lambda: df.add_detection(-4.55, 0.01, 0.0)


def main():
    args = sys.argv[1:]
    out = None
    if '--json' in args:
        i = args.index('--json')
        out = args[i + 1]
        args = args[:i] + args[i + 2:]
    results = {}
    print(f'{"항목":36s} {"1회 평균":>12s} {"반복":>7s}')
    for name, fn in cases():
        if args and not any(a in name for a in args):
            continue
        sec, n = bench(fn)
        results[name] = sec * 1e6
        unit = f'{sec * 1e3:9.3f} ms' if sec >= 1e-3 else f'{sec * 1e6:9.1f} µs'
        print(f'{name:36s} {unit:>12s} {n:7d}')
    if out:
        with open(out, 'w') as f:
            json.dump({'us_per_call': results}, f, ensure_ascii=False, indent=1)
        print(f'저장: {out}')


if __name__ == '__main__':
    main()
