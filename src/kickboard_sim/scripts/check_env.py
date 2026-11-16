#!/usr/bin/env python3
"""맵·시나리오 사전 점검 (시뮬레이션을 돌리기 전에 배치 실수를 잡는다).

  source install/setup.bash
  python3 src/kickboard_sim/scripts/check_env.py street                 # 맵만
  python3 src/kickboard_sim/scripts/check_env.py campus cp1_class_change  # 맵 + 시나리오
  python3 src/kickboard_sim/scripts/check_env.py --all                  # 모든 시나리오

점검 항목
  맵      : 도킹·pre-dock 지점이 비어 있는가 / 주차구역마다 로봇이 설 자리가 있는가 / 충전소에서 각 구역까지 길이 있는가
  시나리오 : 킥보드마다 (GPS 오차가 없다고 할 때) 로봇팔이 닿는 빈 자리가 있는가 / 거기까지 길이 있는가
             -> '도달 불가'가 의도한 것(진입 금지 구역 안 등)인지 확인하라는 뜻이지, 항상 오류는 아니다
"""
import glob
import math
import os
import sys

import yaml

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), 'kickboard_robot'))
from kickboard_robot.drop_zones import edge_plans  # noqa: E402
from kickboard_robot.occupancy import OccupancyMap  # noqa: E402
from kickboard_robot.planner import Costmap, free_spot  # noqa: E402

DEFAULT_MAP = 'kickboard_world'


def load_map(name):
    cfg = os.path.join(HERE, 'config', 'maps', name + '.yaml')
    if not os.path.exists(cfg):
        cfg = os.path.join(HERE, 'config', 'map_config.yaml')
    with open(cfg) as f:
        mc = yaml.safe_load(f)
    omap = OccupancyMap(os.path.join(HERE, 'maps', name + '.yaml'))
    cm = Costmap.from_occupancy(omap, inscribed=0.47, inflate=1.1)
    nav = (mc.get('params') or {}).get('nav_bridge', {})
    lidar = os.path.join(HERE, 'maps', name + '_lidar.yaml')
    if nav.get('keepout_goal_clearance', -1) >= 0 and os.path.exists(lidar):
        cm.set_physical(OccupancyMap(lidar))          # 진입 금지 구역(인도 등)을 벽과 구분
    return mc, omap, cm


def path_len(cm, a, b):
    p = cm.plan(a, b, 0.0)
    if p is None:
        return None
    pts = [a] + list(p)
    return sum(math.hypot(q[0] - r[0], q[1] - r[1]) for q, r in zip(pts[:-1], pts[1:]))


def check_map(name):
    mc, omap, cm = load_map(name)
    ok = True
    sp, c = mc['robot_spawn'], mc['charger']
    dock = (sp['x'], sp['y'])
    tyaw = math.radians(c['tag_pose']['yaw_deg'])
    pre = (c['tag_pose']['x'] + c['pre_dock_distance'] * math.cos(tyaw),
           c['tag_pose']['y'] + c['pre_dock_distance'] * math.sin(tyaw))
    print(f'[맵 {name}] {omap.w * omap.res:.0f} x {omap.h * omap.res:.0f} m, 주차구역 {len(mc.get("parking_zones", []))}곳')
    for label, p, cl in (('도킹 지점', dock, 0.6), ('pre-dock 지점', pre, 0.9)):
        free = omap.is_free(p[0], p[1], cl)
        ok &= free
        print(f'  {"OK " if free else "NG "} {label} ({p[0]:.2f}, {p[1]:.2f}) 반경 {cl} m 빈 공간')
    for z in mc.get('parking_zones', []):
        plans = edge_plans(z['name'], z['polygon'], omap,
                           (mc.get('params') or {}).get('mission_manager', {}).get('drop_clearance', 1.05))
        spots = sum(len(p['spots']) for p in plans)
        dists = [path_len(cm, pre, p['robot'][:2]) for p in plans]
        reach = [d for d in dists if d is not None]
        good = bool(plans) and bool(reach)
        ok &= good
        print(f'  {"OK " if good else "NG "} 구역 {z["name"]}: 로봇 자리 {len(plans)}곳, 킥보드 자리 {spots}개, '
              f'충전소에서 {"%.0f m" % min(reach) if reach else "길 없음"}')
    return ok, (mc, omap, cm, pre)


def check_scenario(path, cache):
    with open(path) as f:
        sc = yaml.safe_load(f)
    name = sc.get('map') or DEFAULT_MAP
    if name not in cache:
        cache[name] = check_map(name)
    _, (mc, omap, cm, pre) = cache[name]
    print(f'[시나리오 {os.path.basename(path)}] 맵 {name} — {sc.get("name")}')
    ok = True
    for s in sc.get('scooters', []):
        x, y = float(s['x']), float(s['y'])
        if not s.get('model', True):
            print(f'  --  킥보드 {s["id"]}: 실물 없음 (확인 실패 시나리오)')
            continue
        nav = (mc.get('params') or {}).get('nav_bridge', {})
        kc = nav.get('keepout_goal_clearance', -1)
        spot = free_spot(cm, (x, y), pre, 1.35, nav.get('goal_clearance', 1.1), 0.0,
                         keep_clearance=kc if kc >= 0 else None)
        d = path_len(cm, pre, spot[:2]) if spot else None
        good = spot is not None and d is not None
        ok &= good
        why = '' if good else (' <- 팔이 닿는 빈 자리 없음 (벽·진입 금지 구역·장애물과 너무 가까움)' if spot is None
                               else ' <- 길 없음')
        print(f'  {"OK " if good else "NG "} 킥보드 {s["id"]} ({x:.1f}, {y:.1f})' +
              (f' 충전소에서 {d:.0f} m' if d is not None else '') + why)
    return ok


def main():
    args = sys.argv[1:]
    cache = {}
    if args and args[0] == '--all':
        for p in sorted(glob.glob(os.path.join(HERE, 'config', 'scenarios', '*.yaml'))):
            check_scenario(p, cache)
        return
    if not args:
        print(__doc__)
        return
    cache[args[0]] = check_map(args[0])
    for sc in args[1:]:
        p = sc if os.path.isabs(sc) else os.path.join(HERE, 'config', 'scenarios', sc if sc.endswith('.yaml') else sc + '.yaml')
        check_scenario(p, cache)


if __name__ == '__main__':
    main()
