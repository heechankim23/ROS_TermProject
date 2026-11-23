#!/usr/bin/env python3
"""시뮬레이션 환경 생성기: 배치(layout) 하나에서 world SDF · 지도 · 맵 설정 · 배치도를 한꺼번에 만든다.

  python3 scripts/build_env.py            # street, campus 둘 다
  python3 scripts/build_env.py campus     # 하나만

배치를 고치면 이 스크립트만 다시 돌리면 된다 (world 와 지도가 어긋나지 않게 한 곳에서 생성).
만드는 파일 (패키지 루트 기준):
  worlds/<맵>.sdf            Gazebo world (구조물 = 충돌체, 바닥 표시 = 시각만)
  maps/<맵>.pgm|yaml         주행 지도 = 구조물 + 진입 금지 구역(keepout: 차도·잔디·계단)  -> nav_bridge, mission_manager
  maps/<맵>_lidar.pgm|yaml   물리 지도 = 구조물만 (라이다는 차도·잔디를 벽으로 보지 않는다) -> sim_lidar
  config/maps/<맵>.yaml      GPS 기준점 · 충전소 · 로봇 스폰 · 주차구역 · 맵별 노드 파라미터(params)
  maps/<맵>_layout.svg       배치도 (문서용)

좌표: map 프레임 = world 프레임. +x = 동, +y = 북, 단위 m. 구조물은 축에 나란한 상자·원기둥만 쓴다.

배치 규칙 (로봇 1.7 x 0.9 m, 제자리 회전 반경 0.98 m 기준 — 새 맵을 만들 때 지킬 것)
  1. 통로 폭 >= 2.5 m 권장 (최소 1.2 m: 장애물에서 0.47 m 안은 통과 불가, 1.1 m 안은 비용이 붙는다)
  2. 주차구역은 한 변이 열린 곳을 향하고, 그 변 밖 0.95 m 지점에서 반경 1.05 m 가 비어 있어야 한다 (로봇이 서는 자리)
  3. 충전소 태그 앞 2 m(pre-dock) 까지 직선으로 비우고, 좌우 1.5 m 는 충전 자리(4곳)로 비운다
  4. 킥보드가 놓일 곳은 1.35 m 떨어진 지점에 반경 1.1 m 빈 원이 하나는 있어야 로봇팔이 닿는다
  5. 진입 금지 구역(keepout)을 건너는 곳(횡단보도)은 폭 4 m 이상
"""
import math
import os
import sys

RES = 0.1
WALL_T = 0.2

# 색 (Gazebo ambient/diffuse, SVG 겸용)
C = {
    'ground': (0.62, 0.62, 0.60), 'road': (0.22, 0.22, 0.24), 'sidewalk': (0.74, 0.72, 0.68),
    'cross': (0.95, 0.95, 0.95), 'zone': (0.20, 0.70, 0.30), 'lawn': (0.35, 0.60, 0.30),
    'stairs': (0.55, 0.55, 0.58), 'lot': (0.36, 0.36, 0.38), 'line': (0.95, 0.85, 0.2),
    'bldg_a': (0.72, 0.52, 0.40), 'bldg_b': (0.55, 0.62, 0.72), 'bldg_c': (0.80, 0.74, 0.60),
    'bldg_d': (0.60, 0.50, 0.62), 'wall': (0.50, 0.50, 0.50), 'car': (0.20, 0.30, 0.55),
    'car2': (0.60, 0.15, 0.15), 'car3': (0.85, 0.85, 0.88), 'tree': (0.20, 0.45, 0.20),
    'pole': (0.35, 0.35, 0.35), 'bench': (0.50, 0.35, 0.20), 'shelter': (0.30, 0.50, 0.60),
    'planter': (0.45, 0.40, 0.30), 'fountain': (0.45, 0.65, 0.80), 'bollard': (0.90, 0.75, 0.10),
    'rack': (0.40, 0.40, 0.45),
}


class Env:
    def __init__(self, name, title, bounds, gps_origin, charger, params, outline=None):
        self.name, self.title = name, title
        self.outline = outline          # 부지 외곽 다각형 [(x, y), ...]. 주면 그 밖은 전부 막힌 곳, 변마다 담장을 세운다
        self.gates = []                 # (x, y, 이름) 배치도·3D 표시용
        self.drivable = []              # [(x0, y0, x1, y1)] 주면 '이 사각형들 밖은 전부 진입 금지' (차도만 다니는 맵)
        self.raised = {}                # 바닥 표시 이름 -> 높이 (인도를 차도보다 높게 그린다)
        self.station = None             # (이름, [x0, y0, x1, y1]) 킥보드 충전소: 로봇 도크와 다른 곳에 있는 하역 장소
        self.mission_variant = ''       # 'station' 이면 launch 가 mission_manager_station 을 띄운다
        self.x0, self.y0, self.x1, self.y1 = bounds
        self.gps_origin, self.charger, self.params = gps_origin, charger, params
        self.solids = []      # (kind, name, cx, cy, a, b, h, color, label)  box: a=sx b=sy / cyl: a=r
        self.marks = []       # (name, cx, cy, sx, sy, color, layer)        바닥 표시 (시각만)
        self.keepout = []     # (name, cx, cy, sx, sy)                       주행 금지 (지도에만)
        self.zones = []       # (name, [x0, y0, x1, y1])
        self.labels = []      # (x, y, text)                                 배치도 글자
        if outline:
            return
        # 경계 벽
        w, h = self.x1 - self.x0, self.y1 - self.y0
        cx, cy = (self.x0 + self.x1) / 2, (self.y0 + self.y1) / 2
        t = WALL_T
        self.box('wall_n', cx, self.y1 - t / 2, w, t, 1.0, 'wall')
        self.box('wall_s', cx, self.y0 + t / 2, w, t, 1.0, 'wall')
        self.box('wall_e', self.x1 - t / 2, cy, t, h, 1.0, 'wall')
        self.box('wall_w', self.x0 + t / 2, cy, t, h, 1.0, 'wall')

    # ----- 배치 요소 (rect 는 x0, y0, x1, y1 로 준다) -----
    def box(self, name, cx, cy, sx, sy, h, color, label=None):
        self.solids.append(('box', name, cx, cy, sx, sy, h, color, label))

    def rect(self, name, x0, y0, x1, y1, h, color, label=None):
        self.box(name, (x0 + x1) / 2, (y0 + y1) / 2, x1 - x0, y1 - y0, h, color, label)

    def cyl(self, name, cx, cy, r, h, color):
        self.solids.append(('cyl', name, cx, cy, r, 0.0, h, color, None))

    def mark(self, name, x0, y0, x1, y1, color, layer=1):
        self.marks.append((name, (x0 + x1) / 2, (y0 + y1) / 2, x1 - x0, y1 - y0, color, layer))

    def keep(self, name, x0, y0, x1, y1, color=None, label=None):
        self.keepout.append((name, (x0 + x1) / 2, (y0 + y1) / 2, x1 - x0, y1 - y0))
        if color:
            self.mark(name, x0, y0, x1, y1, color, 1)
        if label:
            self.labels.append(((x0 + x1) / 2, (y0 + y1) / 2, label))

    def road(self, name, x0, y0, x1, y1, color='road'):
        """로봇이 다닐 수 있는 곳 (차도·주차장). 하나라도 쓰면 나머지는 전부 진입 금지가 된다"""
        self.drivable.append((x0, y0, x1, y1))
        self.mark(name, x0, y0, x1, y1, color, 0)

    def sidewalk(self, name, x0, y0, x1, y1, h=0.06):
        """인도: 차도보다 h 만큼 높은 판 (시각만 — 충돌체는 없고, 진입 금지는 주행 지도가 막는다)"""
        self.mark(name, x0, y0, x1, y1, 'sidewalk', 1)
        self.raised[name] = h

    def scooter_station(self, name, x0, y0, x1, y1):
        """킥보드 충전소: 배터리 부족 킥보드를 가져다 놓는 별도 장소 (주차구역처럼 한 변이 차도에 닿아야 한다)"""
        self.station = (name, [x0, y0, x1, y1])
        self.mission_variant = 'station'
        self.mark('station_' + name, x0, y0, x1, y1, 'fountain', 3)

    def zone(self, name, x0, y0, x1, y1):
        self.zones.append((name, [x0, y0, x1, y1]))
        self.mark('zone_' + name, x0, y0, x1, y1, 'zone', 3)

    # ----- 판정 -----
    def solid_at(self, x, y):
        if not (self.x0 <= x <= self.x1 and self.y0 <= y <= self.y1):
            return True
        if self.outline and not point_in_poly(x, y, self.outline):
            return True
        for kind, _, cx, cy, a, b, _, _, _ in self.solids:
            if kind == 'box' and abs(x - cx) <= a / 2 and abs(y - cy) <= b / 2:
                return True
            if kind == 'cyl' and math.hypot(x - cx, y - cy) <= a:
                return True
        return False

    def keepout_at(self, x, y):
        if self.drivable and not any(a <= x <= c and b <= y <= d for a, b, c, d in self.drivable):
            return True
        return any(abs(x - cx) <= sx / 2 and abs(y - cy) <= sy / 2 for _, cx, cy, sx, sy in self.keepout)

    # ----- 충전소 -----
    def charger_poses(self):
        """-> 충전소 모델 pose, 태그 pose, 도킹(=스폰) pose"""
        c = self.charger
        yaw = math.radians(c['yaw_deg'])
        nx, ny = math.cos(yaw), math.sin(yaw)
        tag = (c['x'] + 0.152 * nx, c['y'] + 0.152 * ny)
        dock = (tag[0] + c['dock_distance'] * nx, tag[1] + c['dock_distance'] * ny)
        return (c['x'], c['y'], yaw), tag, dock


def point_in_poly(x, y, poly):
    inside, n = False, len(poly)
    for i in range(n):
        (x1, y1), (x2, y2) = poly[i], poly[(i + 1) % n]
        if (y1 > y) != (y2 > y) and x < (x2 - x1) * (y - y1) / (y2 - y1) + x1:
            inside = not inside
    return inside


# =====================================================================================
# 1) 일반 도로 + 공영 주차장  (40 x 30 m)
# =====================================================================================
def street():
    """로봇은 차도(와 주차장)로만 다닌다. 인도는 진입 금지이고, 인도 위 킥보드는 차도 가장자리에 서서 로봇팔로 집는다.
    차도망: 간선도로(동서) + 샛길(남북, 가운데) + 뒷길(동서) + 연결로(남북, 동쪽) -> 북동 블록을 한 바퀴 도는 순환로."""
    e = Env('street', '일반 도로 (차도 주행) + 공영 주차장', (-20, -15, 20, 15),
            gps_origin={'lat': 37.5665, 'lon': 126.9780},
            charger={'x': -19.6, 'y': -8.5, 'yaw_deg': 0.0, 'dock_distance': 1.0, 'pre_dock_distance': 2.0},
            params=street_params())
    # --- 인도 (차도보다 6 cm 높은 판, 진입 금지) ---
    e.sidewalk('block_nw', -19.8, 5, -3, 11)          # 북서 블록 (상가)
    e.sidewalk('block_ne', 2, 5, 13, 11)              # 북동 블록 (카페·편의점) — 순환로 안쪽
    e.sidewalk('block_e', 17, 5, 19.8, 11)            # 동쪽 끝 블록
    e.sidewalk('side_sw', -19.8, -3, -9, -1)          # 남측 보도 (주차장 진입로 서쪽)
    e.sidewalk('side_se', -4, -3, 19.8, -1)           # 남측 보도 (진입로 동쪽)
    e.sidewalk('block_se', 5.2, -14.8, 19.8, -3)      # 남동 블록 (오피스)
    # --- 차도·주차장 (주행 가능) ---
    e.road('main_road', -19.8, -1, 19.8, 5)           # 간선도로 6 m
    e.road('side_street', -3, 5, 2, 11)               # 샛길 5 m (가운데)
    e.road('connector', 13, 5, 17, 11)                # 연결로 4 m (동쪽)
    e.road('back_road', -19.8, 11, 19.8, 14.8)        # 뒷길 3.8 m
    e.road('driveway', -9, -3, -4, -1)                # 주차장 진입로 5 m
    e.road('lot', -19.8, -14.8, 5.2, -3, 'lot')       # 공영 주차장 (충전소)
    e.mark('center_line', -19.8, 1.95, 19.8, 2.05, 'line', 2)
    for k in range(9):                                # 주차선
        x = -15.25 + k * 2.5
        e.mark(f'stall_{k}', x - 0.05, -14.8, x + 0.05, -10.0, 'cross', 2)
    e.labels += [(-11, 2.0, '간선도로'), (-0.5, 8.0, '샛길'), (15, 8.0, '연결로'), (8, 12.9, '뒷길'),
                 (-6.5, -2.0, '진입로'), (-8, -6.0, '공영 주차장'), (-11.5, 6.0, '인도 (진입 금지)')]
    # --- 건물 ---
    e.rect('shop', -18.5, 7.2, -6, 10, 4.0, 'bldg_a', '상가')
    e.rect('cafe', 4, 6.8, 11, 9.2, 3.5, 'bldg_c', '카페·편의점')
    e.rect('office', 8, -13.5, 19, -5.5, 6.0, 'bldg_b', '오피스')
    e.rect('bus_shelter', 14, -2.8, 17, -2.2, 2.4, 'shelter', '버스정류장')
    e.cyl('tree_0', -16.5, 6.3, 0.2, 3.0, 'tree')
    e.cyl('tree_1', 18.8, 6.0, 0.2, 3.0, 'tree')
    e.cyl('tree_2', 3.0, -2.6, 0.2, 3.0, 'tree')
    # --- 주차장: 울타리 + 주차 차량 ---
    e.rect('lot_fence_w', -19.8, -3.1, -9, -2.9, 0.8, 'wall')
    e.rect('lot_fence_e', -4, -3.1, 5.2, -2.9, 0.8, 'wall')
    e.rect('lot_fence_side', 5.1, -14.8, 5.3, -3.1, 0.8, 'wall')
    for k, (x, col) in enumerate(((-14.0, 'car'), (-11.5, 'car3'), (-6.5, 'car2'), (-1.5, 'car'),
                                  (1.0, 'car3'), (3.5, 'car2'))):
        e.box(f'car_{k}', x, -12.3, 1.9, 4.5, 1.5, col)
    # --- 킥보드 주차구역: 인도 위, 한 변이 연석(차도 경계)에 닿음 -> 로봇은 차도에 서서 내려놓는다 ---
    e.zone('N1', -14, 5, -11, 6.4)            # 간선도로 북쪽 인도 (상가 앞)
    e.zone('E1', 11.6, 6.5, 13, 9.5)          # 연결로 서쪽 인도 (카페 옆)
    e.zone('S1', 9, -2.4, 12, -1)             # 간선도로 남쪽 인도 (오피스 앞)
    return e


def street_cs():
    """street 와 같은 도로에 '킥보드 충전소'를 따로 둔 버전.
    배터리 부족 킥보드는 로봇 도크 옆이 아니라 뒷길 쪽 충전소까지 싣고 가서 내려놓는다. 로봇은 그 뒤 자기 도크로 돌아가 충전한다."""
    e = street()
    e.name, e.title = 'street_cs', '일반 도로 + 별도 킥보드 충전소'
    e.scooter_station('CS', -16.0, 10.0, -13.0, 11.0)          # 북서 블록 북쪽 인도, 뒷길(y = 11)에 닿음
    e.rect('charge_rack', -16.0, 10.0, -13.0, 10.2, 1.0, 'shelter')   # 충전 거치대 (킥보드 뒤 벽)
    e.labels += [(-18.45, -6.6, '로봇 도크'), (-14.5, 12.9, '뒷길')]
    return e


def street_params():
    p = outdoor_params(d_max=50.0)
    # 차도만 다니는 맵: 설 자리는 벽·차와는 1.1 m(제자리 회전), 인도 경계와는 0.8 m(앞 범퍼 0.75 m)만 떨어지면 된다
    p['nav_bridge'].update({'costmap_res': 0.1, 'goal_clearance': 1.1, 'inflation_radius': 1.1,
                            'keepout_goal_clearance': 0.8})
    p['mission_manager'] = {'drop_clearance': 0.85}    # 주차구역 변 밖 0.95 m(차도 위)에 서서 하역
    return p


# =====================================================================================
# 2) 대학교 캠퍼스 — 경북대학교(KNU) 대구캠퍼스 배치를 본뜬 축소·단순화 맵  (90 x 90 m)
# =====================================================================================
# 총학생회 'KNU ON MAP' 안내도(1080 px)에서 위치를 읽어 1 px = 0.1 m 로 옮겼다:  x = (px - 540) / 10,  y = (540 - py) / 10
# 실제 캠퍼스는 가로 약 1.1 km 이므로 거리는 약 1/12 이다. 건물을 그 비율로 줄이면 건물 사이가 1 m 도 안 되어
# 로봇(1.7 x 0.9 m)이 다닐 수 없다 -> 건물은 대표 28동만 골라 5~6 m 크기로 두고, 사이를 4 m 이상 띄웠다.
# 즉 '위치 관계'는 KNU 를 따르지만 축척·건물 수·도로 폭은 실제와 다르다. 차도와 인도는 구분하지 않았다 (전부 주행 가능).
KNU_OUTLINE = [(16, 44), (26, 30.5), (31, 19), (36.5, 5), (39.5, -6), (42.5, -16), (32, -43.5), (25, -44.5),
               (16, -36.5), (2, -35), (-6, -42), (-13, -42), (-13, -36), (-21, -34.5), (-32.5, -34), (-39, -31),
               (-43, -17.5), (-42.5, -10), (-34, -0.5), (-21, 8), (-7, 15.5), (2, 24), (11, 38)]
KNU_GATES = [(13.5, 41, '농장문'), (26, 30.5, '누리문'), (31, 19, '테크노문'), (36.5, 5, '향토문'), (39.5, -6, '나리문'),
             (42.5, -16, '동문'), (32, -43.5, '정문'), (16, -36.5, '수의대문'), (2, -35, '쪽문'), (-21, -34.5, '조은문'),
             (-32.5, -34, '솔로문'), (-43, -17.5, '서문'), (-34, -0.5, '수영장문'), (-21, 8, '어린이집문'), (-7, 15.5, '북문')]
# (이름, 중심 x, y, 가로, 세로, 높이, 색)
KNU_BUILDINGS = [
    ('농생대4호관', 19, 31.5, 6, 3, 4, 'bldg_b'), ('누리관', 21.5, 25, 5, 3.5, 6, 'bldg_d'),
    ('예술대학', 9, 25.5, 6, 3.5, 4, 'bldg_b'), ('간호대학', 27, 19, 4, 3, 4, 'bldg_b'),
    ('대강당', 5, 18.5, 5, 4, 5, 'bldg_c'), ('약학대학', 13.5, 18.5, 5, 3, 4, 'bldg_b'),
    ('글로벌플라자', 7.2, 12.5, 6, 4, 8, 'bldg_c'), ('중앙도서관', 13.2, 12, 6, 6, 6, 'bldg_c'),
    ('첨성관', 29, 9.5, 5, 4, 6, 'bldg_d'), ('인문대학', 1.5, 6, 6, 3, 4, 'bldg_b'),
    ('농생대1호관', -6, 5, 5, 3.5, 4, 'bldg_b'), ('농생대2호관', -14, 3.5, 5, 3, 4, 'bldg_b'),
    ('사범대학', 24, 1, 6, 3.5, 4, 'bldg_b'), ('본관', 9.5, -2, 5, 4, 5, 'bldg_c'),
    ('자연과학대학', -25.5, -1.5, 6, 3, 4, 'bldg_b'), ('제1과학관', -18, -5.5, 6, 3, 4, 'bldg_b'),
    ('대학원동', 0.5, -7.5, 6, 3, 4, 'bldg_b'), ('제2체육관', -33, -8.5, 5, 4, 5, 'bldg_c'),
    ('생활과학대학', 33, -6, 5, 3, 4, 'bldg_b'), ('복지관', 25, -11, 5, 4, 4, 'bldg_a'),
    ('경상대학', 36, -12.5, 4, 3, 4, 'bldg_b'), ('법학전문대학원', 33, -21, 6, 3, 4, 'bldg_b'),
    ('IT대학3호관', 2, -20.5, 5, 3, 4, 'bldg_b'), ('공대1호관', -13, -23, 6, 3, 4, 'bldg_b'),
    ('IT대학2호관', 9.5, -26.5, 6, 4, 5, 'bldg_b'), ('진리관', -6.5, -28.5, 5, 3, 5, 'bldg_a'),
    ('화학관', -17, -30.5, 6, 3, 4, 'bldg_b'), ('생물학관', -29, -30, 5, 3, 4, 'bldg_b'),
    ('수의과대학', 20.5, -31, 6, 3, 4, 'bldg_b'),
]


def campus():
    e = Env('campus', '대학교 캠퍼스 (KNU 배치 축소·단순화)', (-45, -45, 45, 45),
            gps_origin={'lat': 35.8900, 'lon': 128.6115},     # 경북대 대구캠퍼스 중앙 부근의 대략값 (필요하면 측량값으로 교체)
            charger={'x': 9.5, 'y': -4.15, 'yaw_deg': -90.0, 'dock_distance': 1.0, 'pre_dock_distance': 2.0},
            params=outdoor_params(d_max=120.0), outline=KNU_OUTLINE)
    e.gates = KNU_GATES
    for name, cx, cy, sx, sy, h, col in KNU_BUILDINGS:
        e.box('b_%d' % len(e.solids), cx, cy, sx, sy, h, col, name)
    # --- 진입 금지: 운동장·연못·잔디 (지도에만 막힌 곳. 라이다에는 안 보인다) ---
    e.keep('baseball', -39, -24.5, -31, -16.5, 'lawn', '야구장')
    e.keep('soccer', -28.5, -25, -21.5, -16, 'lawn', '축구장')
    e.keep('courts', -14, -15.5, -8, -12.5, 'lawn', '족구장·테니스장')
    e.keep('pond', 10.5, -17.5, 14.5, -13.5, 'fountain', '일청담')
    e.keep('wolpa', 14, -12, 19, -8, 'lawn', '월파원')
    e.keep('central_park', 27, -30.5, 32, -25.5, 'lawn', '센트럴파크')
    # --- 시설물 ---
    e.cyl('rotary_c', -4.0, -2.0, 1.0, 0.3, 'planter')        # 중앙 로터리
    e.cyl('rotary_e', 19.0, 6.0, 1.0, 0.3, 'planter')         # 동쪽 로터리 (사범대 앞)
    e.cyl('rotary_w', -22.0, -12.0, 1.0, 0.3, 'planter')      # 서쪽 로터리 (운동장 앞)
    for k, (x, col) in enumerate(((19.0, 'car'), (21.5, 'car3'), (24.0, 'car2'))):   # 학생주차장
        e.box(f'car_{k}', x, -22.0, 1.9, 4.5, 1.5, col)
    e.labels += [(21.5, -18.6, '학생주차장'), (9.5, -6.6, '충전소 (본관 남쪽)')]
    for k, (x, y) in enumerate(((-4, 11), (17, 3), (6, -12), (-10, -8), (-20, -27), (15, -22), (30, -16), (15, 27))):
        e.cyl(f'tree_{k}', x, y, 0.25, 4.0, 'tree')
    # --- 킥보드 주차구역: 건물 출입구 쪽 벽에 붙이고 열린 쪽에서 로봇이 접근 ---
    e.zone('LIB', 11.5, 7.6, 14.5, 9.0)          # 중앙도서관 앞
    e.zone('NURI', 20.0, 21.85, 23.0, 23.25)     # 누리관(기숙사) 앞
    e.zone('SCI', -27.0, -4.4, -24.0, -3.0)      # 자연과학대학 앞
    e.zone('ENG', -14.5, -21.5, -11.5, -20.1)    # 공대1호관 앞
    e.zone('IT', 8.0, -24.5, 11.0, -23.1)        # IT대학2호관 앞
    e.zone('WEL', 23.5, -14.4, 26.5, -13.0)      # 복지관 앞
    return e


def outdoor_params(d_max):
    """실외 맵에서 노드 파라미터 덮어쓰기 (config/robot_params.yaml 은 실내 10 x 8 m 기준이다)"""
    return {
        # costmap_res 0.2: 40~50 m 지도에서 A* 한 번이 0.4 s -> 0.09 s (격자가 커진 만큼 여유 거리도 0.1 m 키움)
        'nav_bridge': {'v_max': 0.8, 'w_max': 1.0, 'lookahead': 0.8, 'goal_timeout_sec': 300.0,
                       'replan_sec': 2.0, 'costmap_res': 0.2, 'goal_clearance': 1.2, 'inflation_radius': 1.2},
        'battery_manager': {'k1_pct_per_m': 0.12, 'k2_pct_per_s': 0.02},   # 왕복 100 m ~ 12 %
        'queue_manager': {'d_max': d_max, 'w_d': 1.0},                     # t_max = (10*2 + 1*d_max) / 1
        'sim_lidar': {'range_max': 8.0},                                   # 실외용 라이다 (실내는 3.5 m)
        'collector': {'align_timeout_sec': 120.0},
    }


# =====================================================================================
# 출력
# =====================================================================================
def rgb(c):
    return ' '.join(f'{v:.2f}' for v in C[c])


def write_world(e, path):
    mat = lambda c: f'<material><ambient>{rgb(c)} 1</ambient><diffuse>{rgb(c)} 1</diffuse></material>'  # noqa: E731
    solids, marks = [], []
    for kind, name, cx, cy, a, b, h, color, _ in e.solids:
        geo = (f'<box><size>{a:.3f} {b:.3f} {h:.3f}</size></box>' if kind == 'box'
               else f'<cylinder><radius>{a:.3f}</radius><length>{h:.3f}</length></cylinder>')
        pose = f'<pose>{cx:.3f} {cy:.3f} {h / 2:.3f} 0 0 0</pose>'
        solids.append(f'        <collision name="{name}_c">{pose}<geometry>{geo}</geometry></collision>\n'
                      f'        <visual name="{name}">{pose}<geometry>{geo}</geometry>{mat(color)}</visual>')
    for name, cx, cy, sx, sy, color, layer in e.marks:
        hh = e.raised.get(name, 0.002)
        z = hh / 2 if name in e.raised else 0.002 + 0.002 * layer + (0.06 if e.raised and layer >= 2 else 0.0)
        marks.append(f'        <visual name="{name}"><pose>{cx:.3f} {cy:.3f} {z:.3f} 0 0 0</pose><geometry><box>'
                     f'<size>{sx:.3f} {sy:.3f} {hh:.3f}</size></box></geometry>{mat(color)}</visual>')
    if e.outline:                      # 외곽 다각형의 변마다 담장
        n = len(e.outline)
        for i in range(n):
            (x1, y1), (x2, y2) = e.outline[i], e.outline[(i + 1) % n]
            ln, yaw = math.hypot(x2 - x1, y2 - y1), math.atan2(y2 - y1, x2 - x1)
            pose = f'<pose>{(x1 + x2) / 2:.3f} {(y1 + y2) / 2:.3f} 0.5 0 0 {yaw:.4f}</pose>'
            geo = f'<box><size>{ln + WALL_T:.3f} {WALL_T} 1.0</size></box>'
            solids.append(f'        <collision name="fence_{i}_c">{pose}<geometry>{geo}</geometry></collision>\n'
                          f'        <visual name="fence_{i}">{pose}<geometry>{geo}</geometry>{mat("wall")}</visual>')
    (sx, sy, syaw), _, _ = e.charger_poses()
    nl = '\n'
    sdf = f'''<?xml version="1.0"?>
<!-- {e.title} ({e.x1 - e.x0:.0f} x {e.y1 - e.y0:.0f} m). scripts/build_env.py 가 만든 파일 — 직접 고치지 말고 스크립트를 고칠 것.
     world 프레임 = map 프레임. 맵 설정: config/maps/{e.name}.yaml. 킥보드는 scooter_fleet_sim 이 시나리오대로 만든다. -->
<sdf version="1.9">
  <world name="{e.name}">
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
      <ambient>0.6 0.6 0.6 1</ambient>
      <background>0.75 0.85 0.95 1</background>
      <grid>false</grid>
    </scene>
    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 30 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.4 0.2 -0.9</direction>
    </light>
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="c"><geometry><plane><normal>0 0 1</normal><size>120 120</size></plane></geometry></collision>
        <visual name="v"><geometry><plane><normal>0 0 1</normal><size>120 120</size></plane></geometry>
          <material><ambient>{rgb('ground')} 1</ambient><diffuse>{rgb('ground')} 1</diffuse></material></visual>
      </link>
    </model>
    <!-- 구조물: 건물·울타리·차량·가로수·시설물 (충돌체 있음, 지도에 점유로 들어감) -->
    <model name="structures">
      <static>true</static>
      <link name="link">
{nl.join(solids)}
      </link>
    </model>
    <!-- 바닥 표시: 보도·차도·횡단보도·잔디·주차구역 (시각만) -->
    <model name="ground_marks">
      <static>true</static>
      <link name="link">
{nl.join(marks)}
      </link>
    </model>
    <include>
      <uri>model://charging_station</uri>
      <name>charging_station</name>
      <pose>{sx:.3f} {sy:.3f} 0 0 0 {syaw:.4f}</pose>
    </include>
  </world>
</sdf>
'''
    with open(path, 'w') as f:
        f.write(sdf)


def write_map(e, prefix, with_keepout):
    import numpy as np
    (sx, sy, syaw), _, _ = e.charger_poses()
    w, h = round((e.x1 - e.x0) / RES), round((e.y1 - e.y0) / RES)
    X, Y = np.meshgrid(e.x0 + (np.arange(w) + 0.5) * RES, e.y1 - (np.arange(h) + 0.5) * RES)   # 행 0 = y 최대
    occ = np.zeros((h, w), dtype=bool)
    if e.outline:                               # 외곽 밖 + 담장 두께
        inside = np.zeros((h, w), dtype=bool)
        n = len(e.outline)
        for i in range(n):
            (x1, y1), (x2, y2) = e.outline[i], e.outline[(i + 1) % n]
            cross = ((y1 > Y) != (y2 > Y)) & (X < (x2 - x1) * (Y - y1) / (y2 - y1 + 1e-12) + x1)
            inside ^= cross
            dx, dy = x2 - x1, y2 - y1
            t = np.clip(((X - x1) * dx + (Y - y1) * dy) / (dx * dx + dy * dy), 0, 1)
            occ |= np.hypot(X - (x1 + t * dx), Y - (y1 + t * dy)) <= WALL_T / 2
        occ |= ~inside
    for kind, _, cx, cy, a, b, _, _, _ in e.solids:
        occ |= ((np.abs(X - cx) <= a / 2) & (np.abs(Y - cy) <= b / 2)) if kind == 'box' \
            else (np.hypot(X - cx, Y - cy) <= a)
    c, s_ = math.cos(syaw), math.sin(syaw)      # 충전소 몸체 0.3 x 0.6
    dx, dy = X - sx, Y - sy
    occ |= (np.abs(c * dx + s_ * dy) <= 0.15) & (np.abs(-s_ * dx + c * dy) <= 0.3)
    if with_keepout and e.drivable:
        ok = np.zeros((h, w), dtype=bool)
        for a, b, c2, d in e.drivable:
            ok |= (X >= a) & (X <= c2) & (Y >= b) & (Y <= d)
        occ |= ~ok
    if with_keepout:
        for _, cx, cy, kx, ky in e.keepout:
            occ |= (np.abs(X - cx) <= kx / 2) & (np.abs(Y - cy) <= ky / 2)
    with open(prefix + '.pgm', 'wb') as f:
        f.write(f'P5\n{w} {h}\n255\n'.encode())
        f.write(np.where(occ, 0, 254).astype(np.uint8).tobytes())
    with open(prefix + '.yaml', 'w') as f:
        f.write(f'image: {os.path.basename(prefix)}.pgm\nmode: trinary\nresolution: {RES}\n'
                f'origin: [{e.x0:.1f}, {e.y0:.1f}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')


def write_config(e, path):
    _, tag, dock = e.charger_poses()
    c = e.charger
    L = [f'# {e.title} 맵 설정. scripts/build_env.py 가 만든 파일 — 배치를 바꾸려면 스크립트를 고쳐 다시 생성.',
         'gps_origin:            # map (0,0) 의 위경도. +x = 동, +y = 북',
         f"  lat: {e.gps_origin['lat']}", f"  lon: {e.gps_origin['lon']}", '',
         'robot_spawn:           # = 도킹 pose',
         f'  x: {dock[0]:.3f}', f'  y: {dock[1]:.3f}', f"  yaw_deg: {(c['yaw_deg'] + 180.0) % 360.0:.1f}", '',
         'charger:', '  station_id: station_1', '  tag_id: 100',
         f"  tag_pose: {{x: {tag[0]:.3f}, y: {tag[1]:.3f}, yaw_deg: {c['yaw_deg']:.1f}}}   # 태그 면 중심, yaw = 바깥쪽 법선",
         f"  dock_distance: {c['dock_distance']}", f"  pre_dock_distance: {c['pre_dock_distance']}", '',
         'parking_zones:         # 폴리곤 (map 좌표, 반시계)']
    for name, (x0, y0, x1, y1) in e.zones:
        L += [f'  - name: {name}', f'    polygon: [[{x0}, {y0}], [{x1}, {y0}], [{x1}, {y1}], [{x0}, {y1}]]']
    if e.station:
        n, (x0, y0, x1, y1) = e.station
        L += ['', '# 킥보드 충전소: 배터리 부족 킥보드를 내려놓는 곳. 로봇 충전 도크(charger)와 다른 장소다.',
              f'mission_variant: {e.mission_variant}     # launch 가 mission_manager_station 노드를 띄운다',
              'scooter_charging_station:', f'  name: {n}',
              f'  polygon: [[{x0}, {y0}], [{x1}, {y0}], [{x1}, {y1}], [{x0}, {y1}]]']
    L += ['', 'params:                # 이 맵에서 덮어쓸 노드 파라미터 (launch/system.launch.py 가 적용)']
    for node, kv in e.params.items():
        L.append(f'  {node}:')
        L += [f'    {k}: {v}' for k, v in kv.items()]
    with open(path, 'w') as f:
        f.write('\n'.join(L) + '\n')


def write_svg(e, path):
    S = 16 if (e.x1 - e.x0) <= 50 else 11        # px / m
    if (e.x1 - e.x0) <= 12:
        S = 60
    W, H = (e.x1 - e.x0) * S, (e.y1 - e.y0) * S
    px = lambda x: (x - e.x0) * S                # noqa: E731
    py = lambda y: (e.y1 - y) * S                # noqa: E731
    col = lambda c: 'rgb(%d,%d,%d)' % tuple(int(v * 255) for v in C[c])   # noqa: E731
    out = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W:.0f} {H:.0f}" font-family="sans-serif">',
           f'<rect width="{W:.0f}" height="{H:.0f}" fill="{col("lot" if e.outline else "ground")}"/>']
    if e.outline:
        pts = ' '.join(f'{px(x):.1f},{py(y):.1f}' for x, y in e.outline)
        out.append(f'<polygon points="{pts}" fill="{col("sidewalk")}" stroke="#222" stroke-width="3"/>')
    for name, cx, cy, sx, sy, color, layer in sorted(e.marks, key=lambda m: m[6]):
        curb = ' stroke="#f2f2f2" stroke-width="2.5"' if name in e.raised else ''
        out.append(f'<rect x="{px(cx - sx / 2):.1f}" y="{py(cy + sy / 2):.1f}" width="{sx * S:.1f}" '
                   f'height="{sy * S:.1f}" fill="{col(color)}"{curb}/>')
    for _, cx, cy, sx, sy in e.keepout:
        out.append(f'<rect x="{px(cx - sx / 2):.1f}" y="{py(cy + sy / 2):.1f}" width="{sx * S:.1f}" '
                   f'height="{sy * S:.1f}" fill="none" stroke="#d33" stroke-width="1.5" stroke-dasharray="6 4"/>')
    for kind, name, cx, cy, a, b, h, color, label in e.solids:
        if kind == 'box':
            out.append(f'<rect x="{px(cx - a / 2):.1f}" y="{py(cy + b / 2):.1f}" width="{a * S:.1f}" '
                       f'height="{b * S:.1f}" fill="{col(color)}" stroke="#222" stroke-width="0.8"/>')
            if label:
                out.append(f'<text x="{px(cx):.1f}" y="{py(cy) + 5:.1f}" font-size="{14 if a * S > 75 else 10}" '
                           f'text-anchor="middle" fill="#111">{label}</text>')
        else:
            out.append(f'<circle cx="{px(cx):.1f}" cy="{py(cy):.1f}" r="{max(a * S, 2.5):.1f}" fill="{col(color)}" '
                       f'stroke="#222" stroke-width="0.8"/>')
    for name, (x0, y0, x1, y1) in e.zones:
        out.append(f'<rect x="{px(x0):.1f}" y="{py(y1):.1f}" width="{(x1 - x0) * S:.1f}" height="{(y1 - y0) * S:.1f}" '
                   f'fill="none" stroke="#0a5" stroke-width="2"/>')
        out.append(f'<text x="{px((x0 + x1) / 2):.1f}" y="{py((y0 + y1) / 2) + 4:.1f}" font-size="12" '
                   f'text-anchor="middle" font-weight="bold" fill="#031">{name}</text>')
    if e.station:
        n, (x0, y0, x1, y1) = e.station
        out.append(f'<rect x="{px(x0):.1f}" y="{py(y1):.1f}" width="{(x1 - x0) * S:.1f}" height="{(y1 - y0) * S:.1f}" '
                   f'fill="none" stroke="#1a4fd0" stroke-width="3"/>')
        out.append(f'<text x="{px((x0 + x1) / 2):.1f}" y="{py(y1) - 6:.1f}" font-size="13" text-anchor="middle" '
                   f'font-weight="bold" fill="#fff" stroke="#0b2a7a" stroke-width="3" paint-order="stroke">킥보드 충전소 {n}</text>')
    for x, y, text in e.labels:
        out.append(f'<text x="{px(x):.1f}" y="{py(y) + 4:.1f}" font-size="12" text-anchor="middle" '
                   f'fill="#fff" stroke="#000" stroke-width="2.5" paint-order="stroke">{text}</text>')
    for x, y, text in e.gates:
        out.append(f'<circle cx="{px(x):.1f}" cy="{py(y):.1f}" r="5" fill="#fff" stroke="#222" stroke-width="1.5"/>')
        out.append(f'<text x="{px(x):.1f}" y="{py(y) - 9:.1f}" font-size="12" text-anchor="middle" '
                   f'fill="#fff" stroke="#000" stroke-width="2.5" paint-order="stroke">{text}</text>')
    (sx, sy, _), tag, dock = e.charger_poses()
    out.append(f'<circle cx="{px(sx):.1f}" cy="{py(sy):.1f}" r="7" fill="#1a4fd0" stroke="#fff" stroke-width="1.5"/>')
    out.append(f'<rect x="{px(dock[0]) - 13.6:.1f}" y="{py(dock[1]) - 7.2:.1f}" width="27.2" height="14.4" '
               f'fill="#f5c400" stroke="#222" stroke-width="1"/>')
    out.append(f'<text x="{px(dock[0]):.1f}" y="{py(dock[1]) - 12:.1f}" font-size="12" text-anchor="middle" '
               f'font-weight="bold" fill="#fff" stroke="#000" stroke-width="2.5" paint-order="stroke">충전소·로봇</text>')
    for k in range(0, int(e.x1 - e.x0) + 1, 10):   # 10 m 눈금
        out.append(f'<text x="{k * S + 2:.0f}" y="{H - 4:.0f}" font-size="10" fill="#fff">{e.x0 + k:.0f}</text>')
    for k in range(0, int(e.y1 - e.y0) + 1, 10):
        out.append(f'<text x="2" y="{H - k * S - 2:.0f}" font-size="10" fill="#fff">{e.y0 + k:.0f}</text>')
    out.append('</svg>')
    with open(path, 'w') as f:
        f.write('\n'.join(out))


def write_json(e, path):
    import json
    (sx, sy, syaw), _, dock = e.charger_poses()
    data = {'name': e.name, 'title': e.title, 'bounds': [e.x0, e.y0, e.x1, e.y1], 'outline': e.outline,
            'solids': [{'k': k, 'x': cx, 'y': cy, 'a': a, 'b': b, 'h': h, 'c': [round(v, 2) for v in C[col]],
                        'label': label} for k, _, cx, cy, a, b, h, col, label in e.solids],
            'marks': [{'x': cx, 'y': cy, 'sx': mx, 'sy': my, 'c': [round(v, 2) for v in C[col]], 'layer': layer,
                       'h': e.raised.get(n, 0)} for n, cx, cy, mx, my, col, layer in e.marks],
            'drivable': e.drivable,
            'keepout': [{'x': cx, 'y': cy, 'sx': kx, 'sy': ky} for _, cx, cy, kx, ky in e.keepout],
            'zones': [{'name': n, 'r': r} for n, r in e.zones],
            'station': ({'name': e.station[0], 'r': e.station[1]} if e.station else None), 'gates': e.gates, 'labels': e.labels,
            'charger': {'x': sx, 'y': sy, 'yaw': syaw, 'dock': dock}}
    with open(path, 'w') as f:
        json.dump(data, f, ensure_ascii=False)


def room():
    """기본 실내 맵(worlds/kickboard_world.sdf, 손으로 쓴 world)의 배치도용 사본. world·지도는 만들지 않고 svg·json 만 낸다"""
    e = Env('kickboard_world', '실내 시험장 (기본 맵)', (-5.1, -4.1, 5.1, 4.1),
            gps_origin={'lat': 37.5665, 'lon': 126.9780},
            charger={'x': -4.7, 'y': 0.0, 'yaw_deg': 0.0, 'dock_distance': 1.0, 'pre_dock_distance': 2.0}, params={})
    e.rect('shelf', -1, 3.3, 1, 3.9, 0.8, 'bench', '선반')
    e.cyl('pillar', 4.0, 3.0, 0.3, 1.0, 'pole')
    e.zone('A', -3.0, 2.3, -1.0, 3.7)
    e.zone('B', 2.0, -3.7, 4.0, -2.3)
    return e


ENVS = {'street': street, 'street_cs': street_cs}       # campus() 는 남겨 두었지만 생성 대상에서 뺐다 (결과물은 archive/campus_map)


def main():
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    os.makedirs(os.path.join(root, 'config', 'maps'), exist_ok=True)
    for name in (sys.argv[1:] or list(ENVS)):
        e = ENVS[name]()
        write_world(e, os.path.join(root, 'worlds', f'{name}.sdf'))
        write_map(e, os.path.join(root, 'maps', name), with_keepout=True)
        write_map(e, os.path.join(root, 'maps', f'{name}_lidar'), with_keepout=False)
        write_config(e, os.path.join(root, 'config', 'maps', f'{name}.yaml'))
        write_svg(e, os.path.join(root, 'maps', f'{name}_layout.svg'))
        write_json(e, os.path.join(root, 'maps', f'{name}_layout.json'))
        print(f'{name}: {e.title} — 구조물 {len(e.solids)}개, 진입 금지 {len(e.keepout)}곳, 주차구역 {len(e.zones)}곳 생성')


if __name__ == '__main__':
    main()
