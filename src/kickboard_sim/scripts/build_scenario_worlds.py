#!/usr/bin/env python3
"""시나리오별 '작은 캠퍼스' world 생성기.

캠퍼스 전체 world(knu_campus_flat)는 지형·건물 메시가 커서 Gazebo 가 무겁고, 로봇이 본관 옆 도크에서 수백 m 를 오가야 한다.
시나리오마다 그 시나리오에 필요한 곳만 잘라 가볍게 만든다:
  - 범위: 킥보드 · 이벤트 위치 · 관련 구역(킥보드마다 가까운 일반·충전 구역 2곳씩 + 시나리오에 나오는 구역) · 로봇 도크 + 여유 40 m
  - 로봇 도크: 시나리오 킥보드들 가운데 근처 길가로 옮긴다 (원래 도크는 본관 옆)
  - 바닥: 충돌은 평면 하나, 보이는 바닥은 캠퍼스 텍스처를 잘라 쓴다. 건물·거치대는 범위 안의 것만
  - 주행 지도·경로 그래프는 캠퍼스 전체 것을 그대로 쓴다 (좌표 동일)

  python3 scripts/build_scenario_worlds.py            # config/scenarios_knu 의 k* 전부
  python3 scripts/build_scenario_worlds.py k08 k13

만드는 것 (맵 이름 knu_<시나리오>, 예: knu_k08)
  worlds/knu_k08.sdf, models/knu_campus/meshes/{terrain,buildings,racks}_k08.obj,
  config/maps/knu_k08.yaml, maps/knu_k08.yaml · knu_k08_lidar.yaml
실행: ros2 launch kickboard_sim bringup.launch.py scenario:=k08 map:=knu_k08
"""
import glob
import math
import os
import sys

import cv2
import numpy as np
import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import build_knu_campus as K  # noqa: E402

PKG = K.PKG
SC_DIR = os.path.join(PKG, 'config', 'scenarios_knu')
MESH = os.path.join(PKG, 'models', K.NAME, 'meshes')
MARGIN = 40.0



# 건물 보이는 크기 (로봇·킥보드와의 화면 비율용, 주행에는 영향 없음)
BLD_H_SCALE, BLD_H_MIN, BLD_H_MAX = 0.35, 2.5, 6.0
BLD_XY_SCALE = 0.8

def setup():
    L = K.load_layers()
    b = L['boundary']
    bounds = (math.floor(b[:, 0].min() / 10) * 10 - 40, math.floor(b[:, 1].min() / 10) * 10 - 40,
              math.ceil(b[:, 0].max() / 10) * 10 + 40, math.ceil(b[:, 1].max() / 10) * 10 + 40)
    T = K.Terrain()
    g = K.Grid(bounds, K.RES_GRID)
    M = K.build_masks(L, g)
    M['roads_wide'] = M['roads'].astype(np.uint8)
    M['pitch_u8'] = M['pitch'].astype(np.uint8)
    M['steps_u8'] = M['steps'].astype(np.uint8)
    M['cars_u8'] = M['cars'].astype(np.uint8)
    clear = cv2.distanceTransform((~M['solid']).astype(np.uint8), cv2.DIST_L2, 5) * K.RES_GRID
    dock0 = K.place_dock(L, g, M, clear)
    T.datum = float(T.elev(*dock0['dock']))
    zones = K.build_zones(L, g, M, clear)
    return L, bounds, T, g, M, clear, zones


def scenario_points(sc):
    pts = [(s['x'], s['y']) for s in sc.get('scooters') or []]
    for e in sc.get('events') or []:
        if 'x' in e and 'y' in e:
            pts.append((float(e['x']), float(e['y'])))
        for p in e.get('path') or []:
            pts.append(tuple(p))
    return pts


def pick_zones(sc, zones):
    by_id = {z['id']: z for z in zones if z['rack']}
    want = set()
    # 충전구역은 충전이 필요한 킥보드가 있을 때만 (일반 구역은 정상·고장 킥보드 하역과 '자리 없음' 대비로 2곳씩)
    need_charge = any(s.get('battery_pct', 80) < 15 and not s.get('fault') and not s.get('charging')
                      for s in sc.get('scooters') or [])
    for s in sc.get('scooters') or []:
        if s.get('slot'):
            want.add(s['slot'].split('-')[0])
        xy = np.array([s['x'], s['y']])
        for typ in (('general', 'charging') if need_charge else ('general',)):
            near = sorted((z for z in by_id.values() if z['type'] == typ),
                          key=lambda z: float(np.hypot(*(z['rack']['center'] - xy))))
            want |= {z['id'] for z in near[:2]}
    for ex in sc.get('expected') or []:
        if ex.get('zone'):
            want.add(ex['zone'])
    for e in sc.get('events') or []:
        for k in ('slot', 'zone'):
            if e.get(k):
                want.add(str(e[k]).split('-')[0])
    return [by_id[z] for z in sorted(want) if z in by_id]


def local_dock(L, g, M, clear, pts, zones_all):
    c = np.median(np.array(pts), axis=0)
    taken = [z['rack']['center'] for z in zones_all if z['rack']]
    for radius in (60, 120, 250):
        rack = K.place_rack(L, g, M, clear, c, 3.0, depth=1.2, radius=radius, taken=taken)
        if rack is not None:
            break
    out = rack['out']
    body = rack['center'] + out * 0.3
    normal = -out
    tag = body + normal * 0.152
    return {'body': body, 'yaw': math.atan2(normal[1], normal[0]), 'tag': tag, 'dock': tag + normal * 1.0}


def write_meshes(L, T, sid, zones, bbox, gbounds):
    x0, y0, x1, y1 = bbox
    gx0, gy0, gx1, gy1 = gbounds
    obj = K.Obj()                                   # 바닥: 캠퍼스 텍스처를 그대로 (UV 를 전체 범위 기준으로)
    xs = np.arange(x0, x1 + 1e-6, 4.0)
    ys = np.arange(y0, y1 + 1e-6, 4.0)
    ids = {}
    for j, y in enumerate(ys):
        for i, x in enumerate(xs):
            ids[i, j] = obj.add_v((x, y, 0.0))
            obj.vt.append(((x - gx0) / (gx1 - gx0), (y - gy0) / (gy1 - gy0)))
    for j in range(len(ys) - 1):
        for i in range(len(xs) - 1):
            a, b, c, d = ids[i, j], ids[i + 1, j], ids[i + 1, j + 1], ids[i, j + 1]
            obj.tri('ground', a, b, c, (a, b, c))
            obj.tri('ground', a, c, d, (a, c, d))
    obj.write(os.path.join(MESH, f'terrain_{sid}.obj'), f'terrain_{sid}.mtl',
              {'ground': ((1, 1, 1), 'campus_ground.png')}, smooth=True)
    obj = K.Obj()
    nb = 0
    for b in L['buildings']:
        p = b['poly']
        if p[:, 0].max() < x0 or p[:, 0].min() > x1 or p[:, 1].max() < y0 or p[:, 1].min() > y1:
            continue
        # 화면 비율: 실제 크기 그대로면 로봇(1.7 m)·킥보드(1.1 m)에 비해 건물이 너무 커 보여서
        # 높이를 낮추고(실제의 BLD_H_SCALE, BLD_H_MIN~BLD_H_MAX m) 바닥 윤곽도 중심 쪽으로 줄인다.
        # 주행·라이다는 2D 지도(원래 윤곽)를 쓰므로 로봇 경로는 그대로이고 보이는 모습만 바뀐다.
        c = p.mean(axis=0)
        ps = c + (p - c) * BLD_XY_SCALE
        h = min(max(b['height'] * BLD_H_SCALE, BLD_H_MIN), BLD_H_MAX)
        obj.prism(f'zone{b["zone"]}', ps, -0.5, h, roof_mat=f'roof{b["zone"]}')
        nb += 1
    mats = {}
    for k, (_, _, col) in K.ZONES.items():
        r, g_, bb = K.hex_rgb(col)
        mats[f'zone{k}'] = ((r, g_, bb), None)
        mats[f'roof{k}'] = ((r * 0.75, g_ * 0.75, bb * 0.75), None)
    if nb == 0:                                     # 빈 메시는 gz 가 싫어함
        obj.box('zone1', x0, y0, -5, 0.1, 0.1, 0.1)
    obj.write(os.path.join(MESH, f'buildings_{sid}.obj'), f'buildings_{sid}.mtl', mats)
    obj = K.Obj()
    gates = [gt for gt in L['gates'] if x0 <= gt['xy'][0] <= x1 and y0 <= gt['xy'][1] <= y1]
    K.rack_geometry(obj, zones, gates, None, T, True)
    obj.write(os.path.join(MESH, f'racks_{sid}.obj'), f'racks_{sid}.mtl',
              {k: (v, None) for k, v in K.RACK_COLORS.items()})
    return nb


def write_world(sid, name, zones, dock, bbox):
    x0, y0, x1, y1 = bbox
    cols = []
    for z in zones:
        r = z['rack']
        c = r['center']
        cols.append(f'        <collision name="{z["id"]}_rack"><pose>{c[0]:.3f} {c[1]:.3f} 0.4 0 0 {r["yaw"]:.4f}</pose>'
                    f'<geometry><box><size>{r["length"] + 0.2:.2f} 0.3 0.8</size></box></geometry></collision>')
    bx, by = dock['body']
    nl = '\n'
    sdf = f'''<?xml version="1.0"?>
<!-- 시나리오 {sid} 전용 작은 캠퍼스 ({x1 - x0:.0f} x {y1 - y0:.0f} m). scripts/build_scenario_worlds.py 가 만든 파일.
     좌표는 캠퍼스 전체 world 와 같다. 바닥 충돌은 평면, 건물·거치대는 이 범위 안의 것만. 맵 설정: config/maps/{name}.yaml -->
<sdf version="1.9">
  <world name="{name}">
    <physics name="default" type="ignored">
      <max_step_size>0.01</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre</render_engine>
    </plugin>
    <scene>
      <ambient>0.65 0.65 0.65 1</ambient>
      <background>0.70 0.80 0.92 1</background>
      <grid>false</grid>
    </scene>
    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 300 0 0 0</pose>
      <diffuse>0.9 0.9 0.88 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.4 0.3 -0.85</direction>
    </light>
    <model name="terrain">
      <static>true</static>
      <link name="link">
        <collision name="c"><pose>{(x0 + x1) / 2:.1f} {(y0 + y1) / 2:.1f} 0 0 0 0</pose><geometry><plane><normal>0 0 1</normal><size>{x1 - x0 + 40:.0f} {y1 - y0 + 40:.0f}</size></plane></geometry></collision>
        <visual name="v"><pose>0 0 0.005 0 0 0</pose><geometry><mesh><uri>model://{K.NAME}/meshes/terrain_{sid}.obj</uri></mesh></geometry></visual>
      </link>
    </model>
    <model name="buildings">
      <static>true</static>
      <link name="link">
        <collision name="c"><geometry><mesh><uri>model://{K.NAME}/meshes/buildings_{sid}.obj</uri></mesh></geometry></collision>
        <visual name="v"><geometry><mesh><uri>model://{K.NAME}/meshes/buildings_{sid}.obj</uri></mesh></geometry></visual>
      </link>
    </model>
    <model name="kickboard_racks">
      <static>true</static>
      <link name="link">
        <visual name="v"><geometry><mesh><uri>model://{K.NAME}/meshes/racks_{sid}.obj</uri></mesh></geometry></visual>
{nl.join(cols)}
      </link>
    </model>
    <include>
      <uri>model://charging_station</uri>
      <name>charging_station</name>
      <pose>{bx:.3f} {by:.3f} 0 0 0 {dock['yaw']:.4f}</pose>
    </include>
  </world>
</sdf>
'''
    open(os.path.join(PKG, 'worlds', f'{name}.sdf'), 'w').write(sdf)


def write_config(name, sid, zones, dock, base_cfg):
    cfg = yaml.safe_load(open(base_cfg))
    keep = {z['id'] for z in zones}
    cfg['parking_zones'] = [z for z in cfg.get('parking_zones', []) if z['name'] in keep]
    cfg['charging_zones'] = [z for z in cfg.get('charging_zones', []) if z['name'] in keep]
    yaw = math.degrees(dock['yaw'])
    cfg['robot_spawn'] = {'x': round(float(dock['dock'][0]), 3), 'y': round(float(dock['dock'][1]), 3),
                          'yaw_deg': round((yaw + 180) % 360, 1)}
    cfg['charger']['tag_pose'] = {'x': round(float(dock['tag'][0]), 3), 'y': round(float(dock['tag'][1]), 3),
                                  'yaw_deg': round(yaw, 1)}
    cfg['terrain'] = {'flat': True}
    with open(os.path.join(PKG, 'config', 'maps', f'{name}.yaml'), 'w') as f:
        f.write(f'# 시나리오 {sid} 전용 작은 캠퍼스 맵 설정. scripts/build_scenario_worlds.py 가 만든 파일.\n'
                f'# 구역은 이 시나리오에 필요한 {len(keep)}곳만, 로봇 도크는 시나리오 근처 길가.\n')
        yaml.safe_dump(cfg, f, allow_unicode=True, sort_keys=False, default_flow_style=None, width=200)
    for suffix in ('', '_lidar'):
        with open(os.path.join(PKG, 'maps', f'{name}{suffix}.yaml'), 'w') as f:
            f.write(open(os.path.join(PKG, 'maps', f'{K.NAME}{suffix}.yaml')).read())


def main():
    names = sys.argv[1:] or sorted(os.path.basename(p)[:-5] for p in glob.glob(os.path.join(SC_DIR, 'k*.yaml')))
    print('캠퍼스 배치 계산 (약 20초)')
    L, gbounds, T, g, M, clear, zones_all = setup()
    base_cfg = os.path.join(PKG, 'config', 'maps', f'{K.NAME}_flat.yaml')
    for sid in names:
        sc = yaml.safe_load(open(os.path.join(SC_DIR, sid + '.yaml')))
        name = f'knu_{sid}'
        zones = pick_zones(sc, zones_all)
        pts = scenario_points(sc)
        dock = local_dock(L, g, M, clear, pts, zones_all)
        allp = np.array(pts + [tuple(z['rack']['center']) for z in zones] + [tuple(dock['dock'])])
        bbox = (max(gbounds[0], allp[:, 0].min() - MARGIN), max(gbounds[1], allp[:, 1].min() - MARGIN),
                min(gbounds[2], allp[:, 0].max() + MARGIN), min(gbounds[3], allp[:, 1].max() + MARGIN))
        bbox = tuple(round(v / 4) * 4 for v in bbox)
        nb = write_meshes(L, T, sid, zones, bbox, gbounds)
        write_world(sid, name, zones, dock, bbox)
        write_config(name, sid, zones, dock, base_cfg)
        print(f'{name:<9} {bbox[2] - bbox[0]:>5.0f} x {bbox[3] - bbox[1]:<5.0f} m  구역 {len(zones):>2}  건물 {nb:>2}  '
              f'도크 ({dock["dock"][0]:.0f}, {dock["dock"][1]:.0f})')


if __name__ == '__main__':
    main()
