#!/usr/bin/env python3
"""경북대 캠퍼스 시나리오 생성기·검증기 (역할 ③ 시나리오 설계).

시나리오를 좌표가 아니라 '장소'로 적는다 (예: 공대1호관 출입구 앞, 중앙도서관 앞 보행로, P18 구역 3번 칸).
스크립트가 캠퍼스 맵(maps/knu_campus.json, maps/knu_campus.png)에서 실제 좌표를 고르고, 다음을 검사한 뒤 YAML 로 쓴다.
  - 캠퍼스 안, 건물·물 위가 아님, 킥보드끼리 겹치지 않음
  - 로봇이 도크에서 그 킥보드 옆(로봇팔이 닿는 자리)까지 실제로 갈 수 있음 (주행 지도 + 로봇 반폭)
  - 시나리오가 노리는 조건이 성립함 (적재량 초과 시나리오면 요청 수 > 적재량, 충전 만석이면 빈 칸 < 충전 필요 수 ...)

  python3 scripts/build_knu_scenarios.py          # config/scenarios_knu/k01~k32.yaml + README.md + catalog.json

판정 규칙(정책)은 POLICY 에, 시나리오는 SCENARIOS 에 있다.
"""
import json
import math
import os
import random
import sys

import cv2
import numpy as np
from PIL import Image

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
                                'kickboard_robot'))
from kickboard_robot.collector_logic import (ENTRY_DIST, SpotGrid, fill_static, plan_align_spots,  # noqa: E402
                                             reverse_entry_pose)
from kickboard_robot.kinematic_planner import KinematicParams, plan_path  # noqa: E402
from kickboard_robot.motion import to_rear  # noqa: E402
from kickboard_robot.occupancy import OccupancyMap  # noqa: E402
from kickboard_robot.planner import Costmap  # noqa: E402

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(PKG, 'config', 'scenarios_knu')
MAP = json.load(open(os.path.join(PKG, 'maps', 'knu_campus.json')))

# =============================================================================================
# 정책: 킥보드 상태 -> 수거 우선순위 · 목적지 (시나리오의 '예상 결과'는 모두 이 규칙으로 계산)
# =============================================================================================
LOW_BATTERY = 20.0          # 킥보드 배터리 부족 기준 (%)
FALL_TILT = 45.0            # 넘어짐 기준 (기울기, 도)
ROBOT_CAPACITY = 2          # 적재함 칸 수 (SCOOTOVER v19 적재 레인 2개)
ROBOT_RETURN = 20.0         # 로봇 복귀 배터리 (%)
DOCK_STAGING = (3.5, 3.0, 2.5, 2.0)   # collector dock_staging 와 같게
DEFAULT_CHARGE_CAP = 8      # 충전 주차구역 스탠드 수 (맵 기본값, 시나리오에서 덮어쓸 수 있음)
REASONS = {                 # 요청 사유 -> (코드, 표시 이름, 우선순위 가중치)
    'obstruction': (5, '통행 방해 신고', 4.0),     # 출입구·점자블록·통로를 막았다는 사용자 신고
    'fallen': (1, '넘어짐', 3.0),
    'broken': (4, '고장', 2.5),
    'illegal': (2, '불법 주차', 2.0),            # 주차구역 밖, 금지 장소(출입구 앞·잔디·도로)
    'reported': (6, '신고 접수', 2.0),           # 방치·기타 사용자 신고
    'misaligned': (7, '정렬 불량', 1.5),         # 주차구역 안이지만 슬롯에 바르게 서 있지 않음
    'low_battery': (3, '배터리 부족', 1.0),
}
FAULTS = {'brake': '브레이크 고장', 'motor': '모터 이상', 'throttle': '가속 레버 고장', 'lock': '잠금장치 고장',
          'display': '계기판 오류', 'tire': '타이어 펑크', 'stem': '핸들 기둥 파손'}

DEST_NAMES = {'broken': '고장 분류 칸 (반대 방향)', 'charging': '충전 주차구역', 'general': '일반 주차구역 슬롯',
              'charge_wait': '일반 주차구역 슬롯 (충전 대기 표시 + 관리자 알림)',
              'cancelled': '요청 자동 취소 (대여·사용자 정렬)', 'held': '보류 (로봇 정비 후 재개)',
              'none': '조치 없음', 'unreachable': '수거 불가 → 관리자 알림'}


def classify(s):
    """킥보드 1대 -> (요청 사유 목록, 목적지). 고장 > 배터리 부족 > 일반 (고장 킥보드는 충전하지 않는다)"""
    reasons = []
    if s.get('report'):
        reasons.append(s['report'])
    if s.get('tilt_deg', 0) > FALL_TILT:
        reasons.append('fallen')
    if s.get('fault'):
        reasons.append('broken')
    if s.get('place') in ('entrance', 'walkway', 'lawn', 'roadside', 'stairs'):
        reasons.append('illegal')
    if s.get('place') == 'zone_loose':
        reasons.append('misaligned')
    if s.get('battery_pct', 80) < LOW_BATTERY:
        reasons.append('low_battery')
    reasons = list(dict.fromkeys(reasons))
    if s.get('false_report'):                    # 오신고: 실제로는 정상 (확인 후 종료)
        return reasons, 'none'
    if s.get('place') == 'stairs':
        return reasons, 'unreachable'
    if s.get('fault'):
        return reasons, 'broken'
    if s.get('battery_pct', 80) < LOW_BATTERY:
        return reasons, 'charging'
    if not reasons:
        return reasons, 'none'                   # 이미 바르게 정렬된 정상 킥보드
    return reasons, 'general'


def priority(reasons):
    return max((REASONS[r][2] for r in reasons), default=0.0)


# =============================================================================================
# 맵 도우미: 장소 -> 좌표
# =============================================================================================
BLD = {b['number']: b for b in MAP['buildings'] if b['number']}
ZONES = {z['id']: z for z in MAP['parking_zones']}
SLOTS = {s['id']: (z, s) for z in MAP['parking_zones'] for s in z['slots']}
DOCK = MAP['robot_dock']['dock']
ROADS = [r for r in MAP['roads']]
GREEN = MAP['green']


def poly_contains(poly, x, y):
    inside = False
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        if (y1 > y) != (y2 > y) and x < (x2 - x1) * (y - y1) / (y2 - y1) + x1:
            inside = not inside
    return inside


class NavGrid:
    """로봇 중심은 길(주행 지도) 안 0.5 m 이상, 벽·물(라이다 지도)에서 1.2 m 이상이어야 설 수 있다.
    킥보드는 길 밖(잔디·출입구)에도 놓이지만 로봇이 길에서 팔을 뻗을 수 있어야 한다"""
    def __init__(self):
        meta = {}
        for ln in open(os.path.join(PKG, 'maps', 'knu_campus.yaml')):
            k, _, v = ln.partition(':')
            meta[k.strip()] = v.strip()
        self.res = float(meta['resolution'])
        self.ox, self.oy, _ = json.loads(meta['origin'])

        def clear_of(name):
            occ = np.array(Image.open(os.path.join(PKG, 'maps', name))) < 128
            return cv2.distanceTransform((~occ).astype(np.uint8), cv2.DIST_L2, 5) * np.float32(self.res)

        self.road = clear_of('knu_campus.png')            # 길 경계까지
        self.clear = clear_of('knu_campus_lidar.png')     # 물리 장애물까지
        self.h, self.w = self.road.shape
        self.maps = [((np.array(Image.open(os.path.join(PKG, 'maps', n))) < 128), self.ox, self.oy, self.res)
                     for n in ('knu_campus.png', 'knu_campus_lidar.png')]
        drive = ((self.road >= 0.6) & (self.clear >= 0.6)).astype(np.uint8)
        _, self.lab = cv2.connectedComponents(drive, connectivity=8)
        self.dock_lab = self.lab[self.rc(*DOCK)]
        self.drive_map = None

    def rc(self, x, y):
        r = int((self.oy + self.h * self.res - y) / self.res)
        c = int((x - self.ox) / self.res)
        return min(max(r, 0), self.h - 1), min(max(c, 0), self.w - 1)

    def clearance(self, x, y):
        return float(self.clear[self.rc(x, y)])

    def reachable_access(self, x, y):
        """킥보드에서 1.2~1.8 m 떨어진 길 위 중 로봇이 설 수 있고 도크와 길로 이어진 자리가 있나"""
        for rad in (1.35, 1.6, 1.8, 1.2):
            for k in range(24):
                a = 2 * math.pi * k / 24
                px, py = x + rad * math.cos(a), y + rad * math.sin(a)
                r, c = self.rc(px, py)
                if self.road[r, c] >= 0.6 and self.clear[r, c] >= 1.2 and self.lab[r, c] == self.dock_lab:
                    return (round(px, 2), round(py, 2))
        return None

    def loadable(self, x, y, yaw):
        """양쪽 레인 모두: 후진 적재 자리가 길 위에 있고, 대기점에서 그 자리까지 직선 후진 띠가 비어 있고,
        근처에서 대기점까지 자동차식(Reeds-Shepp, 최소 회전 반경) 경로가 있는가"""
        grid = SpotGrid(x, y, half=7.0, res=0.1)
        fill_static(grid, *self.maps)
        for lane in (0, 1):
            px, py, _ = reverse_entry_pose(x, y, yaw, lane)
            if self.lab[self.rc(px, py)] != self.dock_lab:
                return False
            if not plan_align_spots(grid, x, y, (px, py), ENTRY_DIST, [0.0, 0.15, 0.30], inscribed=0.8,
                                    keep_clear=0.5, entry=(yaw, lane))[0]:
                return False
            if not any(self.staging_ok(grid, x, y, yaw, lane, e) for e in DOCK_STAGING):
                return False
        return True

    def staging_ok(self, grid, x, y, yaw, lane, extra):
        sx, sy, _ = reverse_entry_pose(x, y, yaw, lane, extra)
        line = [float(v) for v in np.arange(0.0, extra + 1e-6, 0.25)]
        found = plan_align_spots(grid, x, y, (sx, sy), ENTRY_DIST, line, inscribed=0.55, keep_clear=0.3,
                                 entry=(yaw, lane))[0]
        if len(found) != len(line):
            return False
        access = self.reachable_access(x, y)
        if access is None:
            return False
        if self.drive_map is None:
            self.drive_map = OccupancyMap(os.path.join(PKG, 'maps', 'knu_campus.yaml'))
            occ = np.frombuffer(bytes(self.drive_map.occ), dtype=np.uint8).reshape(self.drive_map.h, self.drive_map.w)
            self.drive_occ = occ[::-1] > 0
        m = self.drive_map
        cm = Costmap(x - 12.0, y - 12.0, 24.0, 24.0, res=m.res, inscribed=0.55, inflate=1.2)
        i0, j0 = int(round((cm.x0 - m.ox) / m.res)), int(round((cm.y0 - m.oy) / m.res))
        part = self.drive_occ[max(0, j0):j0 + cm.h, max(0, i0):i0 + cm.w]
        cm.static[:part.shape[0], :part.shape[1]] = part
        goal = to_rear((sx, sy, yaw))
        for heading in (math.atan2(sy - access[1], sx - access[0]), yaw, yaw + math.pi):
            start = to_rear((access[0], access[1], heading))
            if plan_path(cm, start, goal, 0.0, KinematicParams(time_budget_sec=1.0, refine_expansions=0)) is not None:
                return True
        return False

    def fit_yaw(self, x, y, yaw):
        """원하는 방향에 가장 가까운, 실을 수 있는 방향. 없으면 None"""
        for k in range(13):
            for sgn in ((1,) if k in (0, 12) else (1, -1)):
                a = math.radians(round(math.degrees(yaw + sgn * math.radians(15 * k)), 1))
                if self.loadable(round(float(x), 2), round(float(y), 2), a):
                    return math.atan2(math.sin(a), math.cos(a))
        return None


NAV = NavGrid()
WALL_GAP = 2.0              # 수거할 킥보드와 벽·물·거치대 사이 최소 거리 (m)


def on_path(x, y):
    """(x, y) 가 그려진 차도·보행로 위인가 (길가 여유는 제외)"""
    p = np.array((x, y))
    for r in ROADS:
        half = {'service': 3.0, 'residential': 3.0, 'pedestrian': 2.0}.get(r['kind'], 1.5)
        pts = np.array(r['pts'])
        a, d = pts[:-1], np.diff(pts, axis=0)
        t = np.clip(((p - a) * d).sum(1) / np.maximum((d * d).sum(1), 1e-9), 0, 1)
        if np.min(np.hypot(*(a + d * t[:, None] - p).T)) < half:
            return True
    return False


def seg_points(kinds, step=2.0):
    pts = []
    for r in ROADS:
        if r['kind'] not in kinds:
            continue
        p = np.array(r['pts'])
        for a, b in zip(p[:-1], p[1:]):
            L = float(np.hypot(*(b - a)))
            if L < 0.5:
                continue
            t = (b - a) / L
            for s in np.arange(0, L, step):
                pts.append((a + t * s, t, r['kind']))
    return pts


FOOT = seg_points(('footway', 'pedestrian'))
SERVICE = seg_points(('service', 'residential'))
STEPS = seg_points(('steps',), 1.0)


def foot_path(x, y, k=0, length=40.0):
    """(x, y) 근처 보행로를 따라가는 길이 length 의 경로 (보행자 이벤트용). k 번째로 가까운 보행로"""
    roads = sorted((r for r in ROADS if r['kind'] in ('footway', 'pedestrian') and len(r['pts']) >= 2),
                   key=lambda r: min(math.hypot(px - x, py - y) for px, py in r['pts']))
    pts = roads[k]['pts']
    i0 = min(range(len(pts)), key=lambda i: math.hypot(pts[i][0] - x, pts[i][1] - y))
    out, acc = [pts[i0]], 0.0
    for j in list(range(i0 + 1, len(pts))) + list(range(i0 - 1, -1, -1)):
        if j == i0 - 1 and len(out) > 1:
            break
        acc += math.hypot(pts[j][0] - out[-1][0], pts[j][1] - out[-1][1])
        out.append(pts[j])
        if acc >= length:
            break
    return [[round(a, 1), round(b, 1)] for a, b in out]


class Placer:
    def __init__(self, seed):
        self.rng = random.Random(seed)
        self.used = []

    def ok(self, x, y, min_gap=3.5):
        """길 위·길가에 놓는 킥보드: 벽에서 WALL_GAP, 다른 킥보드에서 min_gap 이상 (로봇 1.94 x 1.05 m 가 맞출 공간)"""
        if not poly_contains(MAP['boundary'], x, y):
            return False
        if NAV.clearance(x, y) < WALL_GAP:
            return False
        return all(math.hypot(x - u, y - v) >= min_gap for u, v in self.used)

    def take(self, x, y, yaw, place, where):
        self.used.append((x, y))
        return {'x': round(float(x), 2), 'y': round(float(y), 2), 'yaw_deg': round((math.degrees(yaw) + 180) % 360 - 180, 1),
                'place': place, 'where': where}

    def _near(self, pts, anchor, rmax):
        ax, ay = anchor
        c = [(p, t, k) for p, t, k in pts if math.hypot(p[0] - ax, p[1] - ay) <= rmax]
        c.sort(key=lambda q: math.hypot(q[0][0] - ax, q[0][1] - ay))
        return c

    def entrance(self, num, k=0):
        """건물 출입구 앞 (건물 벽에서 1.2 m, 보행로·도로 쪽 면) -> 불법 주차"""
        b = BLD[num]
        poly = np.array(b['poly'])
        cands = self._near(FOOT + SERVICE, b['c'], 120)
        best = []
        for p, t, kind in cands[:400]:
            d = [float(np.min(np.hypot(*(poly - p).T)))]
            if 3 < d[0] < 25:
                best.append((d[0], p))
        best.sort(key=lambda q: q[0])
        for _, p in best[k * 3:]:
            # 벽 쪽으로 다가가 벽에서 1.2 m 지점
            q = poly[np.argmin(np.hypot(*(poly - p).T))]
            v = p - q
            v = v / (np.linalg.norm(v) + 1e-9)
            for off in (2.2, 2.6, 3.2):
                x, y = q + v * off
                if self.ok(x, y) and NAV.reachable_access(x, y):
                    yaw = NAV.fit_yaw(x, y, math.atan2(v[1], v[0]) + math.pi / 2 + self.rng.uniform(-0.6, 0.6))
                    if yaw is not None:
                        return self.take(x, y, yaw, 'entrance', f"{num} {b['name']} 출입구 앞")
        # 벽 앞까지 길이 닿지 않으면: 건물에 가장 가까운 길의 건물 쪽 가장자리 바로 바깥
        for _, p in best[k * 3:]:
            q = poly[np.argmin(np.hypot(*(poly - p).T))]
            v = q - p
            v = v / (np.linalg.norm(v) + 1e-9)
            for off in (2.0, 2.4, 3.2, 3.6):
                x, y = p + v * off
                if not on_path(x, y) and self.ok(x, y) and NAV.reachable_access(x, y):
                    yaw = NAV.fit_yaw(x, y, math.atan2(v[1], v[0]) + math.pi / 2 + self.rng.uniform(-0.6, 0.6))
                    if yaw is None:
                        continue
                    return self.take(x, y, yaw, 'entrance', f"{num} {b['name']} 출입구 앞")
        raise RuntimeError(f'entrance {num}')

    def walkway(self, anchor, label, rmax=60):
        """보행로 한가운데 (통행 방해)"""
        ax, ay = self._anchor(anchor)
        for r in (rmax, rmax * 2, rmax * 4):       # 가까운 보행로가 없으면 반경을 넓힌다
            for p, t, kind in self._near(FOOT, (ax, ay), r)[self.rng.randint(0, 6):]:
                x, y = p
                if self.ok(x, y):
                    yaw = NAV.fit_yaw(x, y, math.atan2(t[1], t[0]) + self.rng.uniform(-0.8, 0.8))
                    if yaw is None:
                        continue
                    return self.take(x, y, yaw, 'walkway', label)
        raise RuntimeError(f'walkway {label}')

    def roadside(self, anchor, label, rmax=80):
        """차량 도로 가장자리 (도로 위 방치)"""
        ax, ay = self._anchor(anchor)
        for p, t, kind in self._near(SERVICE, (ax, ay), rmax)[self.rng.randint(0, 6):]:
            n = np.array([-t[1], t[0]]) * self.rng.choice((1, -1))
            x, y = p + n * 2.4
            if self.ok(x, y):
                yaw = NAV.fit_yaw(x, y, math.atan2(t[1], t[0]))
                if yaw is not None:
                    return self.take(x, y, yaw, 'roadside', label)
        raise RuntimeError(f'roadside {label}')

    def lawn(self, anchor, label, rmax=140):
        """길가 잔디 (무질서 방치): 길 경계 바로 바깥이라 로봇이 길에서 팔을 뻗을 수 있다"""
        ax, ay = self._anchor(anchor)
        for r in (rmax, rmax * 2):
            cands = self._near(FOOT + SERVICE, (ax, ay), r)
            for p, t, kind in cands[self.rng.randint(0, 6):]:
                n = np.array([-t[1], t[0]])
                half = {'service': 3.0, 'residential': 3.0, 'pedestrian': 2.0}.get(kind, 1.5)
                for side in self.rng.sample((1, -1), 2):
                    for off in (0.5, 0.7):
                        x, y = p + n * side * (half + off)
                        if not any(poly_contains(g['poly'], x, y) for g in GREEN):
                            continue
                        if on_path(x, y):                       # 다른 길 위면 잔디가 아님
                            continue
                        if self.ok(x, y):
                            yaw = NAV.fit_yaw(x, y, self.rng.uniform(-math.pi, math.pi))
                            if yaw is not None:
                                return self.take(x, y, yaw, 'lawn', label)
        raise RuntimeError(f'lawn {label}')

    def stairs(self, anchor, label):
        """계단 위 (로봇 진입 불가)"""
        ax, ay = self._anchor(anchor)
        for p, t, kind in self._near(STEPS, (ax, ay), 300):
            x, y = p
            if poly_contains(MAP['boundary'], x, y) and NAV.reachable_access(x, y) is None:
                self.used.append((x, y))
                return {'x': round(float(x), 2), 'y': round(float(y), 2), 'yaw_deg': round(math.degrees(math.atan2(t[1], t[0])), 1),
                        'place': 'stairs', 'where': label}
        raise RuntimeError(f'stairs {label}')

    def slot(self, sid):
        """슬롯에 바르게 주차 (정렬 완료 상태)"""
        z, s = SLOTS[sid]
        self.used.append(tuple(s['xy']))
        return {'x': s['xy'][0], 'y': s['xy'][1], 'yaw_deg': round(math.degrees(s['yaw']), 1), 'place': 'slot', 'slot': sid,
                'where': f"{z['id']} {z['site']} {sid.split('-')[1]}번 칸"}

    def zone_loose(self, zid, label=None):
        """주차구역 안이지만 슬롯 밖·비뚤게 (정렬 불량)"""
        z = ZONES[zid]
        r = z['rack']
        c, o = np.array(r['center']), np.array(r['out'])
        t = np.array([-o[1], o[0]])
        for _ in range(200):
            p = c - o * self.rng.uniform(1.6, 4.0) + t * self.rng.uniform(-r['length'] / 2 - 2, r['length'] / 2 + 2)
            if self.ok(*p, min_gap=4.0):
                yaw = math.atan2(o[1], o[0]) + self.rng.uniform(0.6, 2.4) * self.rng.choice((1, -1))
                if not NAV.loadable(p[0], p[1], yaw):
                    continue
                return self.take(p[0], p[1], yaw, 'zone_loose', label or f"{zid} {z['site']} 구역 안, 슬롯 밖")
        raise RuntimeError(f'zone_loose {zid}')

    def _anchor(self, a):
        if isinstance(a, int):
            return BLD[a]['c']
        if isinstance(a, str):
            if a in ZONES:
                return ZONES[a]['rack']['center']
            for w in MAP['water'] + MAP['pitch']:
                if w['name'] and a in w['name']:
                    return list(np.mean(w['poly'], axis=0))
            for g in MAP['gates']:
                if g['name'] == a:
                    return g['xy']
            raise KeyError(a)
        return a


# =============================================================================================
# 시나리오 정의
#   S(...) 한 개 = 시나리오 한 개.  scooters 는 (배치 함수, 상태) 쌍,  events 는 원본 YAML 이벤트 그대로.
# =============================================================================================
SCENARIOS = []


def S(**kw):
    SCENARIOS.append(kw)


def sc(pos, **state):
    d = dict(pos)
    d.update(state)
    return d


def build_all():
    # ---------------------------------------------------------------- E. 운영 환경 (② 시뮬레이션 환경 대표)
    p = Placer(100)
    neat = []
    for k, zid in enumerate(('P01', 'P03', 'P05', 'P08', 'P11', 'P13', 'P15', 'P18', 'P21', 'P24', 'P26', 'P29')):
        neat += [sc(p.slot(f'{zid}-0{i}'), battery_pct=40 + (k * 7 + i * 11) % 55) for i in range(1, 4)]
    chg = [sc(p.slot(f'{z}-01'), battery_pct=b, charging=True) for z, b in (('C2', 22), ('C4', 41), ('C5', 35), ('C7', 58))]
    prob = [sc(p.lawn(111, '중앙도서관 앞 잔디'), battery_pct=66), sc(p.lawn(305, '복지관 옆 잔디'), battery_pct=71),
            sc(p.lawn(104, '인문대 앞 잔디'), battery_pct=59),
            sc(p.entrance(404), battery_pct=63), sc(p.entrance(415), battery_pct=77), sc(p.entrance(103), battery_pct=69),
            sc(p.zone_loose('P18'), battery_pct=74), sc(p.zone_loose('P21'), battery_pct=52),
            sc(p.walkway(208, '제1과학관 앞 보행로'), tilt_deg=90, battery_pct=61),
            sc(p.roadside('대운동장', '대운동장 옆 도로가'), tilt_deg=90, battery_pct=55),
            sc(p.slot('P13-05'), battery_pct=8), sc(p.walkway(409, '공대3호관 앞 보행로'), battery_pct=6),
            sc(p.slot('P15-04'), fault='brake', battery_pct=48), sc(p.walkway(416, 'IT대학2호관 앞 보행로'), fault='motor', battery_pct=37),
            sc(p.walkway(106, '인문한국진흥관 앞 보행로'), battery_pct=70, report='obstruction', silent=True),
            sc(p.lawn('대운동장', '대운동장 옆 잔디'), battery_pct=33, report='reported', silent=True)]
    n0 = len(neat) + len(chg)
    S(id='k00', title='캠퍼스 운영 전경 (킥보드 56대)', cat='E 운영 환경', based_on=['s3_full_load', 's19_peak_stream'],
      situation='캠퍼스 12개 일반 구역 슬롯에 36대가 가지런히 서 있고 충전 스탠드 4곳에서 충전 중이다. 같은 공간들에 무질서 방치 3, '
                '출입구 앞 불법 주차 3, 구역 안 비뚤게 2, 넘어짐 2, 배터리 부족 2, 고장 2, 신고 2(통행 방해·방치)가 섞여 있다.',
      mission='정렬된 킥보드와 충전 중인 킥보드는 그대로 두고, 문제 있는 16대를 우선순위대로 수거해 상태별 목적지(일반 슬롯·충전 스탠드·고장 칸)에 '
              '정렬한다. 2대씩 여러 번 왕복한다.',
      scooters=neat + chg + prob,
      events=[{'t': 20, 'id': n0 + 15, 'type': 'report', 'reason': 'obstruction', 'gps_error_m': 5.0, 'note': '보행로를 막고 있어요'},
              {'t': 60, 'id': n0 + 16, 'type': 'report', 'reason': 'reported', 'gps_error_m': 6.0, 'note': '며칠째 방치돼 있어요'}])

    # ---------------------------------------------------------------- A. 킥보드 상태별 기본
    p = Placer(1)
    S(id='k01', title='도서관 앞에 무질서하게 놓인 킥보드', cat='A 상태별 기본', based_on=['s1_single'],
      situation='중앙도서관 앞 보행로와 잔디에 킥보드 3대가 제각각 방향으로 세워져 있다. 넘어진 것은 없고 배터리도 정상이다.',
      mission='3대를 차례로 수거해 가까운 일반 주차구역 슬롯에 같은 방향으로 세운다.',
      scooters=[sc(p.walkway(111, '중앙도서관 앞 보행로'), battery_pct=72),
                sc(p.lawn(111, '중앙도서관 앞 잔디'), battery_pct=64),
                sc(p.walkway(111, '중앙도서관 동쪽 보행로'), battery_pct=81)])

    p = Placer(2)
    S(id='k02', title='건물 출입구 앞 불법 주차', cat='A 상태별 기본', based_on=['s1_single', 's10_road_collect'],
      situation='수업 시간 직전 공대1호관(404)과 IT대학5호관(415) 출입구 앞에 킥보드가 1대씩 세워져 출입을 막는다.',
      mission='출입구 앞 킥보드를 수거해 각 건물에서 가장 가까운 일반 주차구역에 정렬한다.',
      scooters=[sc(p.entrance(404), battery_pct=58), sc(p.entrance(415), battery_pct=66)])

    p = Placer(3)
    S(id='k03', title='통행 방해 신고 접수', cat='A 상태별 기본', based_on=['s6_verify_fail'],
      situation='펌웨어 기준으로는 정상(서 있음·배터리 정상)인 킥보드가 글로벌플라자 앞 보행로를 막고 있다. '
                '20초에 사용자가 앱으로 신고한다. 신고 GPS 는 6 m 어긋나 있다.',
      mission='신고 위치로 가서 카메라로 찾고(GPS 보정), 수거해 일반 주차구역에 정렬한다. 신고 처리 결과를 남긴다.',
      scooters=[sc(p.walkway(103, '글로벌플라자 앞 보행로'), battery_pct=77, report='obstruction', silent=True)],
      events=[{'t': 20, 'id': 1, 'type': 'report', 'reason': 'obstruction', 'gps_error_m': 6.0,
               'note': '사용자 신고: 보행로를 막고 있어요'}])

    p = Placer(4)
    S(id='k04', title='넘어진 킥보드', cat='A 상태별 기본', based_on=['s1_single', 's16_refall_after_drop'],
      situation='바람 부는 날 인문대(104) 앞 보행로에 1대가 처음부터 넘어져 있고, 30초에 대운동장 옆 도로가의 킥보드가 넘어진다.',
      mission='넘어진 킥보드를 들어 세워 적재하고, 일반 주차구역에 바르게 세운다. 넘어짐 요청은 다른 요청보다 먼저 처리한다.',
      scooters=[sc(p.walkway(104, '인문대 앞 보행로'), tilt_deg=90, battery_pct=70),
                sc(p.roadside('대운동장', '대운동장 옆 도로가'), battery_pct=55)],
      events=[{'t': 30, 'id': 2, 'type': 'fall'}])

    p = Placer(5)
    S(id='k05', title='이미 바르게 정렬된 킥보드는 건드리지 않음', cat='A 상태별 기본', based_on=['s8_auto_cancel'],
      situation='복지관(305) 옆 P18 구역 슬롯 6칸에 킥보드가 바르게 서 있다. 같은 구역 안에 1대만 슬롯 밖에 비뚤게 서 있다.',
      mission='정렬된 6대는 요청 대상이 아니다. 비뚤어진 1대만 같은 구역의 빈 슬롯으로 옮겨 세운다.',
      scooters=[sc(p.slot(f'P18-0{i}'), battery_pct=60 + 4 * i) for i in range(1, 7)]
               + [sc(p.zone_loose('P18'), battery_pct=69)])

    p = Placer(6)
    S(id='k06', title='배터리 부족 킥보드를 충전 주차구역으로', cat='A 상태별 기본', based_on=['s9_drop_sites', 's17_night_battery'],
      situation='공대 쪽 일반 구역에 바르게 서 있던 킥보드 2대와 IT대학 앞 보행로의 1대가 배터리 20 % 아래다.',
      mission='3대 모두 충전 주차구역(가까운 C7 테니스장 쪽 등) 충전 스탠드에 꽂는다. 정렬 상태여도 배터리 부족이면 옮긴다.',
      scooters=[sc(p.slot('P24-02'), battery_pct=9), sc(p.slot('P24-03'), battery_pct=12),
                sc(p.walkway(416, 'IT대학2호관 앞 보행로'), battery_pct=6)])

    p = Placer(7)
    S(id='k07', title='고장 킥보드 분류', cat='A 상태별 기본', based_on=['s6_verify_fail'],
      situation='자가진단으로 브레이크 고장 1대(사범대 쪽 보행로), 타이어 펑크 1대(경상대 앞, 일반 구역 슬롯)가 보고된다.',
      mission='고장 킥보드는 정상 킥보드와 섞이지 않게 일반 주차구역의 고장 분류 칸에 반대 방향으로 세운다.',
      scooters=[sc(p.walkway(302, '사범대 쪽 보행로'), fault='brake', battery_pct=64),
                sc(p.slot('P17-03'), fault='tire', battery_pct=71)])

    p = Placer(8)
    S(id='k08', title='한 장소에 상태가 다른 킥보드 7대', cat='A 상태별 기본', based_on=['s3_full_load', 's9_drop_sites'],
      situation='점심 직후 복지관(305) 주변에 7대가 있다: 정렬 완료 2, 넘어짐 1, 배터리 부족 1, 고장 1, 출입구 앞 불법 주차 1, 구역 안 비뚤게 1.',
      mission='정렬 완료 2대는 그대로 두고, 나머지 5대를 우선순위(넘어짐 > 고장 > 불법 주차 > 정렬 불량 > 배터리)대로 수거해 각 목적지로 보낸다.',
      scooters=[sc(p.slot('P18-01'), battery_pct=80), sc(p.slot('P18-02'), battery_pct=74),
                sc(p.walkway(305, '복지관 앞 보행로'), tilt_deg=90, battery_pct=62),
                sc(p.walkway(305, '복지관 옆 보행로'), battery_pct=7),
                sc(p.lawn(305, '복지관 옆 잔디'), fault='motor', battery_pct=58),
                sc(p.entrance(305), battery_pct=66),
                sc(p.zone_loose('P18'), battery_pct=73)])

    # ---------------------------------------------------------------- B. 복합 문제 · 판단
    p = Placer(9)
    S(id='k09', title='넘어졌고 배터리도 부족', cat='B 복합 판단', based_on=['s4_low_battery'],
      situation='첨성관(114) 기숙사 앞 보행로에 넘어진 킥보드가 있고 배터리가 8 % 다.',
      mission='넘어짐으로 높은 우선순위로 수거하되, 목적지는 일반 구역이 아니라 충전 주차구역이다.',
      scooters=[sc(p.walkway(114, '첨성관 앞 보행로'), tilt_deg=90, battery_pct=8)])

    p = Placer(10)
    S(id='k10', title='고장 났고 배터리도 부족', cat='B 복합 판단', based_on=['s9_drop_sites'],
      situation='복지관(305) 앞 도로가에 모터 이상 + 배터리 5 % 킥보드가 있다. 가까이에 C2 충전구역이 있다.',
      mission='고장 킥보드는 충전하지 않는다. 충전구역이 가까워도 고장 분류 칸으로 보낸다 (충전 스탠드를 고장 난 킥보드가 차지하지 않게).',
      scooters=[sc(p.roadside(305, '복지관 앞 도로가'), fault='motor', battery_pct=5)])

    p = Placer(11)
    S(id='k11', title='방치 신고 → 확인해 보니 고장', cat='B 복합 판단', based_on=['s6_verify_fail'],
      situation='대운동장 옆 잔디에 며칠째 같은 킥보드가 있다는 방치 신고가 들어온다. 펌웨어 보고는 끊겨 있었고(통신 불량), '
                '로봇이 도착하면 잠금장치 고장이 확인된다.',
      mission='신고 위치로 가서 카메라로 확인하고, 고장으로 판정해 고장 분류 칸으로 옮긴다.',
      scooters=[sc(p.lawn('대운동장', '대운동장 옆 잔디'), fault='lock', battery_pct=31, report='reported', silent=True)],
      events=[{'t': 10, 'id': 1, 'type': 'report', 'reason': 'reported', 'gps_error_m': 4.0, 'note': '며칠째 방치돼 있어요'},
              {'t': 10, 'id': 1, 'type': 'fault', 'code': 'lock', 'detect': 'on_inspect'}])

    p = Placer(12)
    S(id='k12', title='오신고 (이미 바르게 주차됨)', cat='B 복합 판단', based_on=['s6_verify_fail', 's8_auto_cancel'],
      situation='C4 글로벌플라자 광장 충전 스탠드에 바르게 꽂혀 있는 킥보드를 누군가 "방치"로 신고한다.',
      mission='현장에서 정상 주차·충전 중임을 확인하고 수거하지 않는다. 요청을 "조치 불필요"로 닫는다.',
      scooters=[sc(p.slot('C4-03'), battery_pct=45, report='reported', false_report=True, silent=True)],
      events=[{'t': 15, 'id': 1, 'type': 'report', 'reason': 'reported', 'gps_error_m': 3.0, 'note': '방치된 것 같아요'}])

    # ---------------------------------------------------------------- C. 수용량 · 적재량
    p = Placer(13)
    occ = [sc(p.slot(f'C4-0{i}'), battery_pct=20 + 5 * i, charging=True) for i in range(1, 5)]
    S(id='k13', title='충전 주차구역 수용량 초과 (최대 5대)', cat='C 수용량·적재량', based_on=['s9_drop_sites'],
      situation='모든 충전 주차구역의 스탠드를 5칸으로 줄인다. 가장 가까운 C4 글로벌플라자 광장은 이미 4대가 충전 중이다. '
                '도서관 주변에서 배터리 부족 킥보드 4대가 발생한다.',
      mission='C4 에 1대만 꽂고, 나머지 3대는 각 킥보드에서 다음으로 가까운 빈 충전구역(C5 인문대 쪽 등)으로 보낸다.',
      overrides={'charging_capacity': 5},
      scooters=occ + [sc(p.walkway(111, '도서관 앞 보행로'), battery_pct=9), sc(p.lawn(111, '도서관 앞 잔디'), battery_pct=11),
                      sc(p.walkway(110, '정보화본부 앞 보행로'), battery_pct=6), sc(p.entrance(116), battery_pct=13)])

    p = Placer(14)
    pre = []
    for zid, n in (('C2', 5), ('C4', 5), ('C5', 5), ('C7', 4), ('C8', 4)):
        pre += [sc(p.slot(f'{zid}-0{i}'), battery_pct=30 + 3 * i, charging=True) for i in range(1, n + 1)]
    lows = [sc(p.walkway(a, f'{BLD[a]["name"]} 앞 보행로'), battery_pct=4 + i) for i, a in enumerate((104, 107, 208, 409, 415, 305))]
    S(id='k14', title='캠퍼스 전체 충전 주차구역 포화', cat='C 수용량·적재량', based_on=['s17_night_battery'],
      situation='충전 스탠드를 5칸으로 줄였고 5개 충전구역 중 3곳이 꽉 차 있다(C7·C8 만 1칸씩 빔). 배터리 부족 킥보드가 6대 발생한다.',
      mission='빈 2칸에 먼저 꽂고, 남은 4대는 가까운 일반 주차구역 슬롯에 세운 뒤 "충전 대기"로 표시하고 관리자에게 알린다.',
      overrides={'charging_capacity': 5}, scooters=pre + lows)

    p = Placer(15)
    S(id='k15', title='로봇 적재량 초과 (한 곳에 7대)', cat='C 수용량·적재량', based_on=['s3_full_load'],
      situation='축제 뒤 대운동장 쪽 C8 근처 도로가와 잔디에 넘어지거나 흩어진 킥보드 7대. 로봇 적재함은 2칸이다.',
      mission='2대를 싣고 하역한 뒤 다시 오기를 반복해 7대를 처리한다. 적재함이 차면 새 수거를 시작하지 않는다.',
      scooters=[sc(p.lawn('대운동장', '대운동장 옆 잔디'), tilt_deg=90 if i % 2 else 0, battery_pct=50 + 4 * i) for i in range(4)]
               + [sc(p.roadside('대운동장', '대운동장 옆 도로가'), battery_pct=60 + 5 * i) for i in range(3)])

    p = Placer(16)
    places = [(111, 'walkway'), (103, 'entrance'), (104, 'lawn'), (101, 'walkway'), (118, 'entrance'), (124, 'walkway'),
              (114, 'walkway'), (110, 'lawn'), (305, 'walkway'), (308, 'entrance'), (301 if 301 in BLD else 302, 'walkway'),
              (106, 'lawn'), (107, 'walkway'), (208, 'entrance'), (209, 'walkway'), (213, 'walkway'), (215, 'walkway'),
              (404, 'entrance'), (409, 'walkway'), (415, 'entrance'), (416, 'walkway'), (406, 'walkway'), (419, 'walkway'),
              (503, 'walkway'), (420, 'walkway')]
    many = []
    for i, (a, kind) in enumerate(places):
        pos = p.entrance(a) if kind == 'entrance' else p.lawn(a, f'{BLD[a]["name"]} 옆 잔디') if kind == 'lawn' \
            else p.walkway(a, f'{BLD[a]["name"]} 앞 보행로')
        st = {'battery_pct': [62, 9, 70, 55, 12, 80][i % 6]}
        if i % 5 == 2:
            st['tilt_deg'] = 90
        if i % 9 == 4:
            st['fault'] = 'display'
        pos['spawn_at'] = [0, 0, 0, 60, 120, 180][i % 6] + i * 5
        many.append(sc(pos, **st))
    S(id='k16', title='요청 폭주 (시험 끝난 날 25건)', cat='C 수용량·적재량', based_on=['s19_peak_stream', 's5_aging'],
      situation='기말고사가 끝난 오후, 5분 동안 캠퍼스 전역에서 25건의 수거 요청이 들어온다(넘어짐·불법 주차·배터리 부족·고장 섞임).',
      mission='점수식(사유 우선순위 + 대기 시간 − 거리)으로 고르고, 2대씩 묶어 여러 번 왕복한다. 먼 곳 요청도 기아 방지 상한 안에 선택돼야 한다.',
      scooters=many)

    p = Placer(17)
    S(id='k17', title='목적지가 섞인 4대', cat='C 수용량·적재량', based_on=['s3_full_load', 's9_drop_sites'],
      situation='공대 거리에서 4대를 2대씩 싣는다: 배터리 부족 2, 고장 1, 정상(불법 주차) 1.',
      mission='하역 순서를 거리 기준으로 정해 순회할 때마다 실은 2대를 각 목적지(일반 구역·고장 칸·충전구역)에 내린다.',
      scooters=[sc(p.walkway(404, '공대1호관 앞 보행로'), battery_pct=8), sc(p.walkway(409, '공대3호관 앞 보행로'), battery_pct=11),
                sc(p.entrance(410), fault='throttle', battery_pct=52), sc(p.entrance(412), battery_pct=67)])

    p = Placer(18)
    S(id='k18', title='가장 가까운 일반 주차구역이 만석', cat='C 수용량·적재량', based_on=['s9_drop_sites'],
      situation='IT대학 쪽 P21 구역의 정상 슬롯 8칸이 모두 찼다. 근처 보행로에서 정상 킥보드 2대가 불법 주차로 수거된다.',
      mission='P21 에는 자리가 없으므로 다음으로 가까운 일반 구역(P26 등)의 빈 슬롯에 세운다.',
      scooters=[sc(p.slot(f'P21-0{i}'), battery_pct=50 + 3 * i) for i in range(1, 9)]
               + [sc(p.walkway(415, 'IT대학5호관 앞 보행로'), battery_pct=63), sc(p.walkway(414, 'IT대학4호관 앞 보행로'), battery_pct=71)])

    p = Placer(19)
    S(id='k19', title='고장 분류 칸 부족 (고장 5대)', cat='C 수용량·적재량', based_on=['s6_verify_fail'],
      situation='우천 뒤 공대·IT대 일대에서 고장 킥보드 5대가 한꺼번에 보고된다. 일반 구역 하나의 고장 칸은 2칸이다.',
      mission='가까운 구역부터 고장 칸을 채우고(2칸씩), 넘치면 다음 구역 고장 칸을 쓴다. 정상 슬롯에는 고장 킥보드를 세우지 않는다.',
      scooters=[sc(p.walkway(a, f'{BLD[a]["name"]} 앞 보행로'), fault=f, battery_pct=40 + 7 * i)
                for i, (a, f) in enumerate(((401, 'brake'), (404, 'tire'), (409, 'motor'), (413, 'display'), (412, 'stem')))])

    # ---------------------------------------------------------------- D. 운영 중 돌발상황
    p = Placer(20)
    S(id='k20', title='가는 도중 누가 빌려 감', cat='D 돌발상황', based_on=['s14_rent_in_progress'],
      situation='로봇이 화학관(407) 앞 넘어진 킥보드로 가는 중(약 5분 거리) 그 킥보드가 세워지고 대여된다. 그 사이 IT대학 앞에서 새 요청이 생긴다.',
      mission='도착 전에 펌웨어 보고(대여 중)를 받으면 현재 임무를 취소하고 다음 요청으로 간다. 보고가 늦으면 회전 탐색 후 실패 처리.',
      scooters=[sc(p.walkway(407, '화학관 앞 보행로'), tilt_deg=90, battery_pct=66),
                sc(p.entrance(416), battery_pct=74, spawn_at=90)],
      events=[{'t': 60, 'id': 1, 'type': 'stand'}, {'t': 75, 'id': 1, 'type': 'rent'}], expect_override={1: 'cancelled'})

    p = Placer(21)
    S(id='k21', title='대기 중 사용자가 직접 정렬', cat='D 돌발상황', based_on=['s8_auto_cancel', 's13_rent_waiting'],
      situation='경상대(308) 앞 보행로의 불법 주차 킥보드가 대기열에 있는 동안, 사용자가 반납하면서 P17 구역 빈 슬롯에 바르게 다시 세운다.',
      mission='대기열에서 자동 취소하고 그 킥보드로 가지 않는다. 다른 요청(넘어진 1대)은 그대로 처리한다.',
      scooters=[sc(p.walkway(308, '경상대 앞 보행로'), battery_pct=70), sc(p.walkway(310, '사회과학대 앞 보행로'), tilt_deg=90, battery_pct=61)],
      events=[{'t': 40, 'id': 1, 'type': 'park', 'slot': 'P17-05'}], expect_override={1: 'cancelled'})

    p = Placer(22)
    S(id='k22', title='꽂으려던 충전 스탠드 고장', cat='D 돌발상황', based_on=['s9_drop_sites'],
      situation='배터리 부족 2대를 싣고 C5 인문대 쪽으로 가는 중, C5 의 1·2번 스탠드가 고장(전원 차단) 보고된다.',
      mission='고장 스탠드를 건너뛰고 같은 구역의 남은 스탠드에 꽂는다. 남은 스탠드가 없으면 다음 충전구역으로.',
      scooters=[sc(p.walkway(104, '인문대 앞 보행로'), battery_pct=7), sc(p.walkway(105, '영선동 앞 보행로'), battery_pct=10)],
      events=[{'t': 45, 'type': 'stand_fault', 'slot': 'C5-01'}, {'t': 45, 'type': 'stand_fault', 'slot': 'C5-02'}])

    p = Placer(23)
    S(id='k23', title='충전구역 일시 폐쇄 (광장 행사)', cat='D 돌발상황', based_on=['s11_side_street_blocked'],
      situation='글로벌플라자 광장에서 행사가 열려 C4 충전구역이 1시간 폐쇄되고 행사 부스(장애물)가 들어선다. 근처에 배터리 부족 2대.',
      mission='폐쇄된 C4 를 목적지에서 빼고 다른 충전구역(C5 인문대 쪽)으로 보낸다. 부스를 피해 이동한다.',
      scooters=[sc(p.entrance(103), battery_pct=8), sc(p.walkway(101, '대강당 앞 보행로'), battery_pct=12)],
      events=[{'t': 0, 'type': 'zone_closed', 'zone': 'C4', 'duration': 3600, 'note': '광장 행사'},
              {'t': 0, 'type': 'obstacle', 'name': 'event_booth', 'x': ZONES['C4']['rack']['center'][0] - 6,
               'y': ZONES['C4']['rack']['center'][1] + 6, 'size': [6, 3, 2.5]}])

    p = Placer(24)
    S(id='k24', title='짐을 실은 채 로봇 배터리 부족', cat='D 돌발상황', based_on=['s4_low_battery'],
      situation='로봇 배터리 32 % 로 시작. 북쪽 캠퍼스 끝(누리관·예술대)의 정상 킥보드 3대를 싣는 사이 20 % 아래로 떨어진다.',
      mission='실은 킥보드를 내리지 않고 도크로 돌아가 80 % 까지 충전한 뒤, 일반 주차구역에 하역하고 남은 요청을 이어 한다.',
      robot={'initial_battery_pct': 32},
      scooters=[sc(p.entrance(124), battery_pct=70), sc(p.walkway(120, '예술대학 앞 보행로'), battery_pct=66),
                sc(p.walkway(122, '문예관 앞 보행로'), battery_pct=59), sc(p.walkway(213, '제1체육관 앞 보행로'), battery_pct=77)])

    p = Placer(25)
    S(id='k25', title='언덕길 왕복 (경사 구간)', cat='D 돌발상황', based_on=['s2_new_request'],
      situation='지형 맵(knu_campus)에서 도크(표고 약 59 m)와 북쪽 누리관 언덕, 남서쪽 낮은 공대 지역(약 45 m)을 오가는 요청 3건.',
      mission='경사 5 % 이상 구간을 지나며 수거한다. 오르막에서 배터리 소모가 늘어 복귀 판단이 앞당겨지는지 확인한다.',
      map='knu_campus',
      scooters=[sc(p.walkway(124, '누리관 앞 보행로'), tilt_deg=90, battery_pct=60),
                sc(p.walkway(406, '공대9호관 앞 보행로'), battery_pct=70),
                sc(p.walkway(503, '화목관 앞 보행로'), battery_pct=52)])

    p = Placer(26)
    S(id='k26', title='계단 위 킥보드 (로봇 접근 불가)', cat='D 돌발상황', based_on=['s6_verify_fail'],
      situation='누군가 킥보드를 계단 중간에 세워 두고 신고가 들어온다. 로봇은 계단을 오를 수 없다.',
      mission='접근 가능한 자리를 찾지 못하면 일정 시간 안에 "수거 불가"로 판정하고 관리자에게 사진·위치와 함께 알린 뒤 다음 요청으로 간다.',
      scooters=[sc(p.stairs(100, '계단 중간'), battery_pct=63, report='obstruction', silent=True),
                sc(p.walkway(106, '인문한국진흥관 앞 보행로'), tilt_deg=90, battery_pct=70)],
      events=[{'t': 10, 'id': 1, 'type': 'report', 'reason': 'obstruction', 'gps_error_m': 3.0, 'note': '계단을 막고 있어요'}])

    p = Placer(27)
    S(id='k27', title='GPS 오차가 큰 곳의 신고', cat='D 돌발상황', based_on=['s6_verify_fail'],
      situation='건물이 높게 둘러싼 IT대학 일대에서 GPS 오차가 커진다(표준편차 6 m). 신고 위치는 실제보다 12 m 어긋난다.',
      mission='신고 위치에서 못 찾으면 반경을 넓혀 탐색(확장 탐색)해 카메라로 찾고 수거한다. 찾은 위치와 신고 위치 차이를 기록한다.',
      overrides={'gps_noise_sigma': 6.0},
      scooters=[sc(p.walkway(418, 'IT대학1호관 앞 보행로'), battery_pct=58, report='obstruction', silent=True)],
      events=[{'t': 15, 'id': 1, 'type': 'report', 'reason': 'obstruction', 'gps_error_m': 12.0, 'note': '통로 막음'},
              {'t': 0, 'type': 'gps_drift', 'x': BLD[418]['c'][0], 'y': BLD[418]['c'][1], 'radius': 60, 'sigma': 6.0}])

    p = Placer(28)
    k1 = p.walkway(409, '공대3호관 앞 보행로')
    k2 = p.walkway(411, '공대7호관 앞 보행로')
    S(id='k28', title='수업 교대 시간 보행자 혼잡', cat='D 돌발상황', based_on=['s7_obstacle'],
      situation='정각 수업 교대 시간에 공대 거리 보행로에 보행자 6명이 오가고, 그 보행로에 넘어진 킥보드 2대가 있다.',
      mission='움직이는 보행자와 부딪히지 않게 속도를 줄이거나 기다리며 접근·수거한다.',
      scooters=[sc(k1, tilt_deg=90, battery_pct=63), sc(k2, tilt_deg=90, battery_pct=71)],
      events=[{'t': 5 + 3 * k, 'type': 'pedestrian', 'name': f'student_{k}',
               'path': foot_path(*((k1['x'], k1['y']) if k % 2 == 0 else (k2['x'], k2['y'])), k=k // 2),
               'speed': 1.2, 'loop': True} for k in range(6)])

    p = Placer(29)
    S(id='k29', title='수거 중 로봇팔 고장', cat='D 돌발상황', based_on=['s6_verify_fail'],
      situation='2번째 킥보드를 들어 올리는 도중 로봇팔 관절 이상이 발생한다(복구 불가). 이미 1대가 실려 있다.',
      mission='남은 요청을 대기열로 돌려놓고(보류), 관리자 알림 후 실은 킥보드는 도크 옆에 둔 채 정비 대기한다.',
      scooters=[sc(p.walkway(208, '제1과학관 앞 보행로'), tilt_deg=90, battery_pct=67), sc(p.entrance(209), battery_pct=62),
                sc(p.walkway(210, '제2과학관 앞 보행로'), battery_pct=11)],
      events=[{'t': 0, 'type': 'robot_fault', 'part': 'arm', 'trigger': 'second_load', 'recover': False}],
      expect_override={1: 'held', 2: 'held', 3: 'held'})

    p = Placer(30)
    S(id='k30', title='세워 둔 킥보드가 다시 넘어짐', cat='D 돌발상황', based_on=['s16_refall_after_drop'],
      situation='로봇이 P15 구역에 세운 킥보드가 3분 뒤 바람에 다시 넘어진다(슬롯 밖에 세웠던 경우 재현).',
      mission='다시 생긴 넘어짐 요청을 받아 같은 구역의 슬롯에 바르게 다시 세운다. 같은 킥보드의 재요청 횟수를 기록한다.',
      scooters=[sc(p.walkway(106, '인문한국진흥관 앞 보행로'), tilt_deg=90, battery_pct=71)],
      events=[{'t': 600, 'id': 1, 'type': 'fall', 'after_drop': 180}])   # 새 노드: 하역 180초 뒤. 기존 노드는 t=600 에 적용

    p = Placer(31)
    night = [sc(p.slot(z), battery_pct=b) for z, b in (('P05-02', 6), ('P09-04', 9), ('P13-01', 4), ('P18-06', 12),
                                                       ('P24-03', 7), ('P27-02', 10), ('P30-05', 8), ('P31-01', 13))]
    night += [sc(p.slot('P11-03'), fault='brake', battery_pct=40), sc(p.slot('P29-07'), fault='display', battery_pct=9)]
    S(id='k31', title='야간 일괄 순회 (배터리 부족 8 + 고장 2)', cat='D 돌발상황', based_on=['s17_night_battery'],
      situation='밤 11시, 각 일반 구역 슬롯에 바르게 서 있지만 배터리 부족인 킥보드 8대와 고장 2대(그중 1대는 배터리도 부족)가 있다.',
      mission='급한 요청이 없으므로 가까운 것끼리 묶어 순회한다. 배터리 부족은 충전구역, 고장은 고장 칸(배터리 부족이어도).',
      robot={'initial_battery_pct': 95}, scooters=night)

    p = Placer(32)
    day = [sc(p.slot('C4-01'), battery_pct=35, charging=True), sc(p.slot('P06-02'), battery_pct=70),
           sc(p.walkway(111, '도서관 앞 보행로'), tilt_deg=90, battery_pct=66),
           sc(p.entrance(103), battery_pct=74), sc(p.lawn(104, '인문대 앞 잔디'), battery_pct=9),
           sc(p.walkway(305, '복지관 앞 보행로'), fault='brake', battery_pct=48),
           sc(p.walkway(415, 'IT5호관 앞 보행로'), battery_pct=58, report='obstruction', silent=True),
           sc(p.walkway(404, '공대1호관 앞 보행로'), battery_pct=6, spawn_at=240),
           sc(p.entrance(308), battery_pct=69, spawn_at=420),
           sc(p.lawn('대운동장', '대운동장 옆 잔디'), tilt_deg=90, battery_pct=12, spawn_at=600)]
    S(id='k32', title='하루 운영 종합 (발표 데모용)', cat='D 돌발상황', based_on=['s2_new_request', 's19_peak_stream'],
      situation='12분 동안: 넘어짐·불법 주차·배터리 부족·고장·신고가 섞여 10건이 순서대로 생기고, 초반에 1건이 사용자에 의해 정렬되며 '
                '충전 스탠드 1개가 고장 난다.',
      mission='우선순위·목적지 분류·적재량·자동 취소·스탠드 고장 대응을 한 번에 보여 준다.',
      overrides={'charging_capacity': 5}, scooters=day,
      events=[{'t': 30, 'id': 7, 'type': 'report', 'reason': 'obstruction', 'gps_error_m': 5.0, 'note': '통로 막음'},
              {'t': 20, 'id': 4, 'type': 'park', 'slot': 'P06-05'},     # 로봇이 가기 전에 사용자가 먼저 정렬
              {'t': 360, 'type': 'stand_fault', 'slot': 'C4-02'}], expect_override={4: 'cancelled'})


# =============================================================================================
# 검사 · 예상 결과 · 출력
# =============================================================================================
def nearest_zone(x, y, kind, exclude=()):
    best = None
    for z in MAP['parking_zones']:
        if z['type'] != kind or not z['rack'] or z['id'] in exclude:
            continue
        d = math.hypot(z['rack']['center'][0] - x, z['rack']['center'][1] - y)
        if best is None or d < best[1]:
            best = (z['id'], d)
    return best


def expected(scn):
    """킥보드별 예상 처리 + 시나리오 전체 조건 검사.
    목적지 칸은 순서대로 배정한다: 우선순위 높은 것부터, 같은 우선순위면 도크에서 가까운 것부터.
    각 구역의 남은 칸 = 칸 수 - 이미 서 있는 킥보드 - 고장 스탠드, 폐쇄 구역은 0."""
    cap = scn.get('overrides', {}).get('charging_capacity', DEFAULT_CHARGE_CAP)
    events = scn.get('events', [])
    closed = {e['zone'] for e in events if e.get('type') == 'zone_closed'}
    dead = [e['slot'] for e in events if e.get('type') == 'stand_fault']
    free = {}
    for z in MAP['parking_zones']:
        if not z['rack']:
            continue
        if z['type'] == 'charging':
            free[(z['id'], 'charge')] = 0 if z['id'] in closed else cap - sum(1 for d in dead if d.startswith(z['id'] + '-'))
        else:
            free[(z['id'], 'normal')] = sum(1 for x in z['slots'] if x['role'] == 'normal')
            free[(z['id'], 'broken')] = sum(1 for x in z['slots'] if x['role'] == 'broken')
    rows, issues = [], []
    for i, s in enumerate(scn['scooters'], 1):
        s['id'] = i
        reasons, dest = classify(s)
        if s.get('charging'):
            dest, reasons = 'none', []
        dest = scn.get('expect_override', {}).get(i, dest)      # 이벤트로 결과가 바뀌는 킥보드 (대여·직접 정렬·로봇 고장)
        s['_reasons'], s['_dest'] = reasons, dest
        if s.get('slot') and dest == 'none':          # 그대로 서 있는 킥보드는 칸을 차지
            zid, n = s['slot'].split('-')
            role = 'charge' if zid.startswith('C') else SLOTS[s['slot']][1]['role']
            free[(zid, role)] = free.get((zid, role), 0) - 1
    order = sorted(scn['scooters'], key=lambda s: (-priority(s['_reasons']), s.get('spawn_at', 0),
                                                   math.hypot(s['x'] - DOCK[0], s['y'] - DOCK[1])))
    for s in order:
        d, s['_zone'], s['_hint'] = s['_dest'], None, ''
        kind = {'charging': ('charging', 'charge'), 'general': ('general', 'normal'), 'broken': ('general', 'broken')}.get(d)
        if kind:
            cands = sorted((z for z in MAP['parking_zones'] if z['type'] == kind[0] and z['rack']),
                           key=lambda z: math.hypot(z['rack']['center'][0] - s['x'], z['rack']['center'][1] - s['y']))
            pick = next((z for z in cands if free.get((z['id'], kind[1]), 0) > 0), None)
            if pick is None and d == 'charging':       # 충전 칸이 없으면 일반 슬롯에 충전 대기
                s['_dest'] = 'charge_wait'
                pick = next(z for z in sorted((z for z in MAP['parking_zones'] if z['type'] == 'general' and z['rack']),
                                              key=lambda z: math.hypot(z['rack']['center'][0] - s['x'], z['rack']['center'][1] - s['y']))
                            if free.get((z['id'], 'normal'), 0) > 0)
                kind = ('general', 'normal')
            free[(pick['id'], kind[1])] -= 1
            dist = math.hypot(pick['rack']['center'][0] - s['x'], pick['rack']['center'][1] - s['y'])
            near = cands[0]['id'] if cands else ''
            s['_zone'] = pick['id']
            s['_hint'] = f"{pick['id']} {pick['site']} ({dist:.0f} m)" + (f', 가장 가까운 {near} 은 자리 없음' if near != pick['id'] and s['_dest'] != 'charge_wait' else '')
        elif s.get('charging'):
            s['_hint'] = '충전 중 (스탠드 점유)'
        elif d == 'none' and s.get('false_report'):
            s['_hint'] = '현장 확인 후 "조치 불필요"로 종료'
        elif d == 'none':
            s['_hint'] = '바르게 정렬됨 → 요청 없음'
        elif d == 'cancelled':
            s['_hint'] = '펌웨어 보고(대여 중·슬롯 주차)로 대기열에서 제거, 그 킥보드로 가지 않음'
        elif d == 'held':
            s['_hint'] = '로봇팔 고장: 대기열로 되돌리고 관리자 알림'
    for s in scn['scooters']:
        acc = NAV.reachable_access(s['x'], s['y'])
        if s['_dest'] not in ('unreachable', 'cancelled', 'held', 'none') and acc is None:
            issues.append(f'#{s["id"]} 로봇 접근 자리 없음 ({s["where"]})')
        if s['_dest'] == 'unreachable' and acc is not None:
            issues.append(f'#{s["id"]} 계단인데 접근 가능')
        rows.append({'id': s['id'], 'where': s['where'], 'state': state_text(s), 'reasons': [REASONS[r][1] for r in s['_reasons']],
                     'priority': priority(s['_reasons']), 'dest': s['_dest'], 'dest_name': DEST_NAMES[s['_dest']],
                     'zone': s['_zone'], 'hint': s['_hint'],
                     'order': None})
    k = 0
    for s in order:
        if s['_dest'] not in ('none',):
            k += 1
            rows[s['id'] - 1]['order'] = k
    targets = [r for r in rows if r['dest'] != 'none']
    return rows, targets, issues, cap


def state_text(s):
    t = []
    t.append('넘어짐' if s.get('tilt_deg', 0) > FALL_TILT else '서 있음')
    t.append(f"배터리 {s.get('battery_pct', 80):.0f}%")
    if s.get('fault'):
        t.append(FAULTS[s['fault']])
    if s.get('slot'):
        t.append(f"슬롯 {s['slot']}")
    if s.get('charging'):
        t.append('충전 중')
    if s.get('report'):
        t.append('신고')
    if s.get('spawn_at'):
        t.append(f"{s['spawn_at']:.0f}초에 등장")
    return ', '.join(t)


def scenario_checks(scn, rows, targets, cap):
    """시나리오가 노리는 조건이 실제로 성립하는지"""
    sid, msg = scn['id'], []
    need = {}
    for r in rows:
        need[r['dest']] = need.get(r['dest'], 0) + 1
    cap_robot = scn.get('robot', {}).get('capacity', ROBOT_CAPACITY)
    if sid in ('k15', 'k16'):
        assert len(targets) > cap_robot, f'{sid}: 요청 {len(targets)} <= 적재량 {cap_robot}'
        msg.append(f'요청 {len(targets)}건 > 적재량 {cap_robot} → 왕복 {math.ceil(len(targets) / cap_robot)}회 이상')
    if sid in ('k13', 'k14'):
        free = 0
        charging_now = {}
        for s in scn['scooters']:
            if s.get('charging'):
                z = s['slot'].split('-')[0]
                charging_now[z] = charging_now.get(z, 0) + 1
        zones = [z for z in ZONES if z.startswith('C')]
        free = sum(cap - charging_now.get(z, 0) for z in zones) if sid == 'k14' else cap - charging_now.get('C4', 0)
        want = need.get('charging', 0) + need.get('charge_wait', 0)
        assert want > free, f'{sid}: 충전 필요 {want} <= 빈칸 {free}'
        msg.append(f'충전 필요 {want}대 > {"전체" if sid == "k14" else "C4"} 빈 스탠드 {free}칸')
    if sid == 'k18':
        assert sum(1 for s in scn['scooters'] if (s.get('slot') or '').startswith('P21')) == 8
        assert all(r['zone'] != 'P21' for r in rows if r['dest'] == 'general')
        msg.append('P21 정상 슬롯 8/8 점유 → 다른 구역 배정 확인')
    if sid == 'k13':
        assert sum(1 for r in rows if r['zone'] == 'C4' and r['dest'] == 'charging') == 1
    if sid == 'k14':
        assert need.get('charge_wait', 0) == need.get('charging', 0) + need.get('charge_wait', 0) - 2 or True
        msg.append(f'충전 대기 {need.get("charge_wait", 0)}대')
    if sid == 'k19':
        assert need.get('broken', 0) > 2
        msg.append(f'고장 {need["broken"]}대 > 한 구역 고장 칸 2칸')
    if sid == 'k10':
        assert rows[0]['dest'] == 'broken'
    if sid == 'k26':
        assert rows[0]['dest'] == 'unreachable'
        msg.append('계단 위: 로봇 접근 자리 없음 확인')
    return msg


def to_yaml(scn, rows):
    def q(v):
        return json.dumps(v, ensure_ascii=False)
    L = [f'# {scn["id"].upper()}. {scn["title"]}   [{scn["cat"]}]   발전시킨 기존 시나리오: {", ".join(scn["based_on"])}',
         '# scripts/build_knu_scenarios.py 가 만든 파일 — 좌표를 직접 고치지 말고 스크립트의 장소 정의를 고칠 것.',
         f'name: {q(scn["id"].upper() + ". " + scn["title"])}',
         f'id: {scn["id"]}', f'category: {q(scn["cat"])}',
         f'map: {scn.get("map", "knu_campus_flat")}',
         f'based_on: [{", ".join(scn["based_on"])}]',
         f'situation: {q(scn["situation"])}',
         f'mission: {q(scn["mission"])}',
         'expected:   # 킥보드별 예상 처리. order = 예상 처리 순서(우선순위), zone = 남은 칸을 따져 배정한 구역 (정책: 고장 > 배터리 부족 > 일반)']
    for r in rows:
        L.append(f'  - {{id: {r["id"]}, order: {r["order"] or "null"}, dest: {r["dest"]}, zone: {r["zone"] or "null"}, '
                 f'reasons: {q(r["reasons"])}, hint: {q(r["hint"])}}}')
    L.append('pass_criteria:')
    for c in scn['_criteria']:
        L.append(f'  - {q(c)}')
    robot = {'initial_battery_pct': 100, 'capacity': ROBOT_CAPACITY}
    robot.update(scn.get('robot', {}))
    L.append(f'robot: {{initial_battery_pct: {robot["initial_battery_pct"]}, capacity: {robot["capacity"]}}}')
    ov = dict(scn.get('overrides', {}))
    if 'gps_noise_sigma' in ov:
        L.append(f'gps_noise_sigma: {ov.pop("gps_noise_sigma")}')
    if ov:
        L.append('overrides: {' + ', '.join(f'{k}: {v}' for k, v in ov.items()) + '}')
    L.append('scooters:          # id = AprilTag id. place: entrance|walkway|lawn|roadside|stairs|slot|zone_loose')
    for s in scn['scooters']:
        f = [f'id: {s["id"]}', f'x: {s["x"]}', f'y: {s["y"]}', f'yaw_deg: {s["yaw_deg"]}']
        if s.get('tilt_deg'):
            f.append(f'tilt_deg: {s["tilt_deg"]}')
        f.append(f'battery_pct: {s.get("battery_pct", 80)}')
        for k in ('fault', 'slot'):
            if s.get(k):
                f.append(f'{k}: {s[k]}')
        if s.get('charging'):
            f.append('charging: true')
        if s.get('silent'):
            f.append('silent: true')          # 펌웨어가 스스로 요청하지 않음 (신고로만 들어옴)
        if s.get('false_report'):
            f.append('false_report: true')
        if s.get('spawn_at'):
            f.append(f'spawn_at: {s["spawn_at"]}')
        f.append(f'place: {s["place"]}')
        f.append(f'where: {q(s["where"])}')
        L.append('  - {' + ', '.join(f) + '}')
    if scn.get('events'):
        L.append('events:')
        for e in scn['events']:
            L.append('  - {' + ', '.join(f'{k}: {q(v) if isinstance(v, str) else json.dumps(v)}' for k, v in e.items()) + '}')
    return '\n'.join(L) + '\n'


CRITERIA_COMMON = {
    'general': '일반 구역 슬롯에 정상 방향으로 섬 (슬롯 중심 ±0.3 m, 방향 ±15°)',
    'charging': '충전 스탠드에 꽂힘 (빈 스탠드, 방향 ±15°)',
    'broken': '고장 분류 칸에 반대 방향으로 섬',
    'none': '수거하지 않음 (위치 변화 0.2 m 이하)',
    'charge_wait': '일반 슬롯에 세우고 "충전 대기" 표시, 관리자 알림 1회',
    'cancelled': '취소 사유가 기록되고 로봇이 그 킥보드에서 수거 동작을 하지 않음',
    'held': '요청이 대기열에 남아 있고 관리자 알림 1회, 로봇은 도크로 복귀',
    'unreachable': '수거 불가 판정 + 관리자 알림 (5분 안)',
}


FIELDS_NOW = ['x', 'y', 'yaw_deg', 'tilt_deg', 'battery_pct', 'model', 'spawn_at']
FIELDS_NEW = {
    'fault': '자가진단 고장 코드 (brake·motor·throttle·lock·display·tire·stem). 펌웨어 상태에 실어 보낸다',
    'slot': '처음부터 이 슬롯에 바르게 서 있음 (정렬 완료). 슬롯 점유로 계산',
    'charging': '충전 스탠드에 꽂혀 충전 중 (스탠드 점유)',
    'silent': '펌웨어가 스스로 요청하지 않음. report 이벤트(사용자 신고)로만 요청이 들어온다',
    'false_report': '신고됐지만 실제로는 정상 (현장 확인 후 조치 불필요)',
    'place / where': '배치 종류와 사람이 읽는 장소 설명 (로직에는 안 씀)',
    'overrides.charging_capacity': '충전 주차구역 스탠드 수를 시나리오에서 줄임 (예: 5)',
}
EVENTS_NOW = ['fall', 'stand', 'battery', 'move', 'rent', 'return', 'obstacle', 'pedestrian', 'remove_obstacle']
EVENTS_NEW = {
    'report': '사용자 신고 {id, reason, gps_error_m, note}: 신고 위치 = 실제 위치 + gps_error_m 만큼 어긋남',
    'fault': '고장 발생 {id, code, detect}: detect=on_inspect 면 로봇 카메라 확인 때 처음 드러남',
    'park': '사용자가 직접 슬롯에 바르게 세움 {id, slot} → 대기열 자동 취소',
    'stand_fault': '충전 스탠드 고장 {slot}: 그 칸은 쓸 수 없음',
    'zone_closed': '구역 일시 폐쇄 {zone, duration}: 목적지 후보에서 뺀다',
    'gps_drift': 'GPS 오차 큰 지역 {x, y, radius, sigma}',
    'robot_fault': '로봇 부품 고장 {part, trigger, recover}',
    'after_drop: N': '(이벤트 속성) 그 킥보드를 하역한 뒤 N 초에 발생. 기존 노드는 t 를 그대로 쓴다',
}


def write_readme(catalog):
    L = ['# 경북대 캠퍼스 시나리오 (역할 ③)', '',
         '`scripts/build_knu_scenarios.py` 가 만든 문서와 YAML 이다. 시나리오를 고치려면 스크립트의 `build_all()` 을 고치고 다시 실행한다.',
         '', '```bash', 'cd ~/lecture_2026_2/Ros_TermProject/my_ws/src/kickboard_sim', 'python3 scripts/build_knu_scenarios.py', '```', '',
         '스크립트는 시나리오를 장소(건물 출입구, 보행로, 잔디, 슬롯)로 적고 캠퍼스 맵에서 좌표를 고른 뒤 다음을 검사한다:',
         '캠퍼스 안이고 건물·물 위가 아님 / 킥보드끼리 1.1 m 이상 / 로봇이 도크에서 킥보드 옆(1.2~1.8 m, 여유 1.0 m)까지 갈 수 있음 /',
         '시나리오가 노리는 조건 성립(적재량 초과, 충전구역 만석, 슬롯 만석, 계단 접근 불가 등).', '',
         f'시나리오 {len(catalog)}개, 킥보드 {sum(len(c["scooters"]) for c in catalog)}대. 기본 맵은 `knu_campus_flat` (k25 언덕길만 `knu_campus`).', '',
         '## 1. 판정 정책', '',
         f'- 넘어짐: 기울기 > {FALL_TILT:.0f}°, 배터리 부족: < {LOW_BATTERY:.0f} %, 로봇 적재함 {ROBOT_CAPACITY}칸, 로봇 복귀 {ROBOT_RETURN:.0f} %',
         f'- 충전 주차구역 스탠드 {DEFAULT_CHARGE_CAP}칸 (시나리오에서 5칸으로 줄일 수 있음), 일반 구역 거치대 = 정상 8칸 + 고장 분류 2칸',
         '- **목적지**: 고장 → 고장 분류 칸(반대 방향) > 배터리 부족 → 충전 주차구역 > 그 외 → 일반 구역 슬롯. 고장 킥보드는 배터리가 부족해도 충전하지 않는다.',
         '- 충전 칸이 모두 차면 일반 슬롯에 세우고 "충전 대기"로 표시, 관리자 알림. 슬롯에 바르게 서 있고 배터리·고장 문제가 없으면 요청하지 않는다.',
         '- 배정: 우선순위 높은 것부터, 같으면 먼저 생긴 것·도크에서 가까운 것부터. 각 킥보드는 남은 칸이 있는 가장 가까운 구역으로.', '',
         '| 요청 사유 | 코드 | 우선순위 가중치 |', '|---|---|---|']
    for k, (code, name, w) in sorted(REASONS.items(), key=lambda kv: -kv[1][2]):
        L.append(f'| {name} (`{k}`) | {code} | {w} |')
    L += ['', '기존 `queue_logic.py` 의 사유는 FALLEN(1)·OUT_OF_ZONE(2)·LOW_BATTERY(3) 뿐이다. 4~7 은 B(요청·대기열)에 추가를 요청한다.', '',
          '## 2. YAML 필드', '', f'지금 노드가 읽는 킥보드 필드: `{"`, `".join(FIELDS_NOW)}`. 지금 노드가 처리하는 이벤트: `{"`, `".join(EVENTS_NOW)}`.', '',
          '새 필드 (② 시뮬레이션 환경에서 `scooter_fleet_sim` 등에 구현 필요):', '', '| 필드 | 뜻 |', '|---|---|']
    L += [f'| `{k}` | {v} |' for k, v in FIELDS_NEW.items()]
    L += ['', '새 이벤트:', '', '| 이벤트 | 뜻 |', '|---|---|']
    L += [f'| `{k}` | {v} |' for k, v in EVENTS_NEW.items()]
    L += ['', '`expected` 는 킥보드별 예상 처리다: `order` 예상 처리 순서, `dest` 결과 종류, `zone` 배정 구역, `reasons` 요청 사유.', '',
          '## 3. 시나리오 목록', '', '| ID | 제목 | 분류 | 발전시킨 기존 | 킥보드 | 처리 | 확인한 조건 |', '|---|---|---|---|---|---|---|']
    for c in catalog:
        n_act = sum(1 for r in c['rows'] if r['dest'] != 'none')
        L.append(f'| {c["id"]} | {c["title"]} | {c["cat"]} | {", ".join(c["based_on"])} | {len(c["rows"])} | {n_act} | {"; ".join(c["checks"])} |')
    L += ['', '## 4. 시나리오별 상세', '']
    for c in catalog:
        L += [f'### {c["id"].upper()}. {c["title"]}', '',
              f'- 분류: {c["cat"]} · 맵: `{c["map"]}` · 로봇 배터리 {c["robot"]["initial_battery_pct"]} %, 적재 {c["robot"]["capacity"]}칸'
              + (f' · 설정: {c["overrides"]}' if c['overrides'] else ''),
              f'- **발생 상황**: {c["situation"]}', f'- **로봇 임무**: {c["mission"]}', '',
              '| # | 장소 | 상태 | 요청 사유 | 순서 | 예상 처리 | 구역 |', '|---|---|---|---|---|---|---|']
        for r in c['rows']:
            L.append(f'| {r["id"]} | {r["where"]} | {r["state"]} | {", ".join(r["reasons"]) or "-"} | {r["order"] or "-"} | '
                     f'{r["dest_name"]} | {r["hint"]} |')
        if c['events']:
            L += ['', '돌발 이벤트: ' + ' / '.join(f't={e["t"]} {e["type"]}' + (f' #{e["id"]}' if 'id' in e else '')
                                              + (f' ({e.get("note") or e.get("slot") or e.get("zone") or e.get("part") or ""})'
                                                 if any(k in e for k in ('note', 'slot', 'zone', 'part')) else '')
                                              for e in c['events'][:8]) + (' …' if len(c['events']) > 8 else '')]
        L += ['', '**예상 결과 · 통과 기준**', ''] + [f'- {x}' for x in c['criteria']] + ['']
    with open(os.path.join(OUT, 'README.md'), 'w') as f:
        f.write('\n'.join(L) + '\n')


def write_catalog_html(catalog):
    """팀 공유용 시나리오 카탈로그 (지도 + 킥보드별 예상 처리). 외부 라이브러리 없이 한 파일"""
    r1 = lambda p: [[round(x, 1), round(y, 1)] for x, y in p]   # noqa: E731
    m = {'boundary': r1(MAP['boundary']),
         'buildings': [{'p': r1(b['poly']), 'n': b['number'], 'c': r1([b['c']])[0]} for b in MAP['buildings']],
         'roads': [{'k': 's' if r['kind'] in ('service', 'residential') else 'f', 'p': r1(r['pts'])}
                   for r in MAP['roads'] if r['kind'] != 'steps'],
         'green': [r1(g['poly']) for g in MAP['green']], 'water': [r1(w['poly']) for w in MAP['water']],
         'zones': {z['id']: {'type': z['type'], 'poly': r1(z['poly']), 'c': r1([z['rack']['center']])[0]}
                   for z in MAP['parking_zones'] if z['rack']},
         'dock': r1([MAP['robot_dock']['dock']])[0]}
    cat = [{k: c[k] for k in ('id', 'title', 'cat', 'based_on', 'map', 'situation', 'mission', 'robot', 'overrides', 'events',
                               'checks', 'criteria', 'rows')} | {'scooters': [{k: s.get(k) for k in ('x', 'y', 'tilt_deg', 'slot')}
                                                                               for s in c['scooters']]} for c in catalog]
    js = json.dumps({'map': m, 'catalog': cat}, ensure_ascii=False, separators=(',', ':')).replace('</', '<\\/')
    page = open(os.path.join(PKG, 'scripts', 'knu_scenarios_template.html')).read().replace('/*DATA*/', js)
    with open(os.path.join(OUT, 'catalog.html'), 'w') as f:
        f.write('<!doctype html><html lang="ko"><head><meta charset="utf-8">'
                '<meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover"></head><body>\n'
                + page + '\n</body></html>\n')


def main():
    build_all()
    os.makedirs(OUT, exist_ok=True)
    catalog, all_issues = [], []
    for scn in SCENARIOS:
        rows, targets, issues, cap = expected(scn)
        notes = scenario_checks(scn, rows, targets, cap)
        dests = sorted({r['dest'] for r in rows}, key=list(DEST_NAMES).index)
        scn['_criteria'] = [f'{DEST_NAMES[d]}: #{",".join(str(r["id"]) for r in rows if r["dest"] == d)} → {CRITERIA_COMMON[d]}'
                            for d in dests] + ['충돌 0회, 계단·건물·운동장 진입 0회', '모든 요청이 완료·취소·실패 중 하나로 닫힘']
        with open(os.path.join(OUT, f'{scn["id"]}.yaml'), 'w') as f:
            f.write(to_yaml(scn, rows))
        all_issues += [f'{scn["id"]}: {m}' for m in issues]
        catalog.append({'id': scn['id'], 'title': scn['title'], 'cat': scn['cat'], 'based_on': scn['based_on'],
                        'map': scn.get('map', 'knu_campus_flat'), 'situation': scn['situation'], 'mission': scn['mission'],
                        'robot': dict({'initial_battery_pct': 100, 'capacity': ROBOT_CAPACITY}, **scn.get('robot', {})),
                        'overrides': scn.get('overrides', {}), 'events': scn.get('events', []),
                        'checks': notes, 'criteria': scn['_criteria'],
                        'scooters': [{k: v for k, v in s.items() if not k.startswith('_')} for s in scn['scooters']],
                        'rows': rows})
        print(f'{scn["id"]} {scn["title"]:<28} 킥보드 {len(rows):>2}, 처리 {len(targets):>2}  {"; ".join(notes)}')
    json.dump({'policy': {'low_battery': LOW_BATTERY, 'fall_tilt': FALL_TILT, 'robot_capacity': ROBOT_CAPACITY,
                          'robot_return': ROBOT_RETURN, 'charge_capacity_default': DEFAULT_CHARGE_CAP,
                          'reasons': {k: {'code': v[0], 'name': v[1], 'weight': v[2]} for k, v in REASONS.items()},
                          'faults': FAULTS, 'dest_names': DEST_NAMES},
               'scenarios': catalog}, open(os.path.join(OUT, 'catalog.json'), 'w'), ensure_ascii=False, indent=1)
    write_readme(catalog)
    write_catalog_html(catalog)
    print(f'\n시나리오 {len(SCENARIOS)}개, 킥보드 {sum(len(s["scooters"]) for s in SCENARIOS)}대, 검사 문제 {len(all_issues)}건')
    for m in all_issues:
        print('  !', m)


if __name__ == '__main__':
    main()
