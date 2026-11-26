#!/usr/bin/env python3
"""경북대학교 대구캠퍼스 맵 생성기 (역할 ① 맵 제작).

원본 데이터(maps/knu_campus_src/) 하나에서 Gazebo world · 주행 지도 · 맵 설정 · 2D 지도 · 웹 뷰어 데이터를 한꺼번에 만든다.
배치(충전구역 후보, 슬롯 수 등)를 바꾸면 이 스크립트만 다시 돌린다.

  python3 scripts/build_knu_campus.py

원본 (maps/knu_campus_src/)
  campus_osm.json            OSM 캠퍼스 경계 · 건물 94동(안내도 번호) · 도로/보행로/계단 447개 · 출입문 (KNU 안심동행 3D 지도와 같은 데이터)
  osm_extra.json             OSM 물(일청담 등) · 녹지 · 운동장/테니스장 · 차량 주차장
  parking_zones_image.json   킥보드 앱 '주차 가능 구역' 캡처(흰/하늘색 38곳)를 OSM 경계에 정합한 결과
  dem_srtm30.json            SRTM 30 m 고도 격자 (평활화해서 지형으로 쓴다)

만드는 파일 (패키지 루트 기준)
  worlds/knu_campus.sdf              Gazebo world (지형 높이 포함)
  worlds/knu_campus_flat.sdf         같은 배치, 평지 (기존 노드처럼 z = 0 을 가정하는 코드 검증용)
  models/knu_campus/                 지형·건물·거치대 메시 (OBJ) + 지형 텍스처
  maps/knu_campus.png|yaml           주행 지도 = 건물·물·캠퍼스 밖 + 진입 금지(운동장·계단)
  maps/knu_campus_lidar.png|yaml     물리 지도 = 건물·물·캠퍼스 밖만
  maps/knu_campus.json               모든 레이어 + 고도 격자 + 주차/충전 구역·슬롯 (시각화·시나리오용)
  maps/knu_campus_2d.png             명칭·등고선·구역이 들어간 2D 지도 (문서용)
  config/maps/knu_campus.yaml        GPS 기준점 · 로봇 도크 · 주차구역/충전구역/슬롯 · 노드 파라미터

좌표: map = world 프레임, +x = 동, +y = 북, 단위 m. 원점 = 위도 35.8904, 경도 128.6103 (캠퍼스 중앙 부근).
지형 높이 z 는 로봇 도크 지점이 0 이 되도록 맞췄다 (로봇 스폰 z = 0.01 을 그대로 쓰기 위해).
"""
import json
import math
import os

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.join(PKG, 'maps', 'knu_campus_src')
NAME = 'knu_campus'

LON0, LAT0 = 128.6103, 35.8904
KX = math.cos(math.radians(LAT0)) * 111320.0
KY = 110574.0

# ---------------------------------------------------------------------------------------------
# 공식 안내판(2026, 사용자 제공 사진) 기준 건물 번호·이름과 구역
# ---------------------------------------------------------------------------------------------
GUIDE = {
    100: '본관', 101: '대강당', 102: 'IM뱅크문화센터', 103: '글로벌플라자', 104: '인문대학', 105: '영선동',
    106: '인문한국진흥관', 107: '대학원동', 108: '첨성인미래관', 109: '도서관휴게실', 110: '정보화본부',
    111: '중앙도서관', 112: '글로벌교육센터', 113: '향토관', 114: '첨성관', 115: 'IT융합산업빌딩',
    116: '종합정보센터', 117: '첨단기술융합대학', 118: '약학대학', 119: '조형관', 120: '예술대학', 121: '조소동',
    122: '문예관', 123: '차고', 124: '누리관', 125: '농대4호관', 126: '간호대학',
    201: '농대1호관', 202: '농대3호관', 203: '농대2호관', 204: '출판부', 205: '복현회관', 206: '어린이집',
    208: '제1과학관', 209: '자연과학대학', 210: '제2과학관', 211: '국민체육센터(수영장)', 212: '제2체육관',
    213: '제1체육관', 214: '청룡관', 215: '제1학생회관(백호관)', 216: '학군단', 217: '생물학관',
    218: '공동실험실습관', 219: '미래융합과학관',
    301: '사범대학', 302: '교육대학원', 303: '우당교육관', 304: '제4합동강의동', 305: '복지관', 306: '박물관',
    307: '생활과학대학', 308: '경상대학', 309: '국제경상관', 310: '사회과학대학', 311: '법학전문대학원',
    401: '공대2호관', 402: '공대2A호관', 403: 'P/P구조실습실', 404: '공대1호관', 405: '연구실안전관리센터',
    406: '공대9호관', 407: '화학관', 408: '공대12호관', 409: '공대3호관', 410: '공대6호관', 411: '공대7호관',
    412: '미래창직관', 413: 'IT대학3호관', 414: 'IT대학4호관', 415: 'IT대학5호관', 416: 'IT대학2호관',
    417: '반도체연구동', 418: 'IT대학1호관', 419: '공대8호관', 420: '수의과대학', 421: '수의과대학부속동물병원',
    422: '실험동물자원관리원',
    501: '진리관', 502: '봉사관', 503: '화목관', 504: '보람관', 506: '면학관', 507: '나래관', 509: '교수아파트',
    510: '변전소', 511: '창업보육센터',
}
# 번호 없는 OSM 건물 중 이름으로 안내도 번호를 붙일 수 있는 것
NAME_TO_NUMBER = {'간호대학': 126, '실험동물': 422}
ZONES = {1: ('본부·인문·도서관', '#cb679e', '#b85a85'), 2: ('농생대·자연대', '#eaa06b', '#c78b48'),
         3: ('사범·사회·경상', '#89bc81', '#6c9b6e'), 4: ('공대·IT대', '#72a7df', '#5688b5'),
         5: ('생활관·창업', '#a48bd9', '#8c76ae')}           # (이름, 안내 색, 3D 건물 색) — KNU 안심동행 지도와 같은 색

ROAD_W = {'service': 6.0, 'residential': 6.0, 'pedestrian': 4.0, 'footway': 2.5, 'steps': 2.5}   # 거치대 자리 계산
PATH_W = dict(ROAD_W, footway=3.0)     # 바닥 그림의 길 폭. 보행로는 눈에 띄게 3 m

# ---------------------------------------------------------------------------------------------
# 충전 주차구역 후보 (처음 8곳. 아래 DELETED_ZONES·GENERAL_FROM_CHARGING 을 뺀 5곳이 남는다). anchor 근처(45 m 안)의 기존 주차 가능 구역을 충전구역으로 바꾸고,
# 없으면 그 자리에 새 충전구역을 만든다.  ('bldg', 번호) | ('water', 이름) | ('pitch', sport/kind) | ('mid', a, b)
# ---------------------------------------------------------------------------------------------
CHARGING_SITES = [
    ('C1', '중앙도서관 앞', ('bldg', 111)),
    ('C2', '편의점 인근 (복지관)', ('bldg', 305)),
    ('C3', '일청담–사범대 사이', ('mid', ('water', '일청담'), ('bldg', 302))),   # 301 은 OSM 에 번호가 없어 바로 남쪽 302 기준
    ('C4', '글로벌플라자 광장', ('bldg', 103)),
    ('C5', '인문대 쪽', ('bldg', 104)),
    ('C6', '공식당 쪽 (공대9호관)', ('bldg', 406)),
    ('C7', '테니스장 쪽', ('pitch', 'tennis')),
    ('C8', '대운동장 쪽', ('pitch', 'track')),
]
SITE_MATCH_M = 45.0
# 11-26 배치 변경. 나머지 구역의 위치·번호가 그대로이도록, 전부 배치한 뒤 빼거나 바꾼다
DELETED_ZONES = ('C1', 'C3', 'P19', 'P23')
GENERAL_FROM_CHARGING = {'C6': ('P33', '공대9호관')}      # 충전 후보 -> 일반 주차구역 (새 번호, 이름)

# 거치대 규격
PARK_SLOTS, BROKEN_SLOTS, PARK_PITCH = 8, 2, 0.7        # 일반: 정상 8칸 + 고장 분류 2칸 (반대 방향), 칸 간격 0.7 m
CHARGE_SLOTS, CHARGE_PITCH = 8, 0.85                    # 충전: Kickspot 2대 = 8칸. 자리 찾기 길이는 예전 그대로 8 x 0.85 (일반 구역 배치가 바뀌지 않게)
RACK_DEPTH = 1.4                                        # 킥보드 길이 1.2 m + 거치대
ROBOT_DOCK_ANCHOR = ('bldg', 100)                       # 로봇 도크 = 캠퍼스 중앙 본관(100) 옆 (차고 123 은 OSM 에 없음)

# Blender 에셋 치수 (scripts/build_blender_models.py -> config/blender_assets.json)
with open(os.path.join(PKG, 'config', 'blender_assets.json')) as _f:
    ASSETS = json.load(_f)
KS = ASSETS['kickspot_4']                                # Kickspot 4구: 슬롯 y, 원점 = 슬롯 줄의 앞바퀴 축
KS_UNIT_W = 2.48                                         # 한 대 받침대 폭 (슬롯 4개, 간격 0.62)
KS_UNITS = (-1.55, 1.55)                                 # 충전구역 거치대 1곳 = Kickspot 2대 (8칸), 거치대 축 방향 위치
KS_AXLE = ASSETS['g30']['front_axle'][0]                 # 킥보드 중심(모델 원점) -> 앞바퀴 축 0.42
GS = ASSETS['robot_station']                             # 로봇 충전소 차고: 원점 = 문 앞 지면, +x 안쪽
GARAGE_LEN, GARAGE_W = GS['footprint'][1][0] - GS['footprint'][0][0], GS['footprint'][1][1] - GS['footprint'][0][1]

RES_NAV = 0.25        # 주행 지도 해상도 (m/px)
RES_GRID = 0.5        # 배치 계산용 격자
TERRAIN_STEP = 8.0    # 지형 메시 격자 (m)
TEX = 8192            # 지형 텍스처 (px, 긴 변)
TEX_SS = 2            # 그릴 때 배율 (줄여서 저장 -> 계단 현상 제거)
BLD_VIS_H = 1.2       # 건물 보이는 높이 배율
ROAD_BLD_GAP = 1.0    # 건물 외곽선이 길을 덮는 곳은 길 우선: 길 가장자리에서 이만큼까지 건물을 잘라 냄



def P(lon, lat):
    return ((lon - LON0) * KX, (lat - LAT0) * KY)


def poly_m(coords):
    pts = [P(*c) for c in coords]
    if len(pts) > 2 and np.allclose(pts[0], pts[-1]):
        pts = pts[:-1]
    return np.array(pts)


def poly_area(p):
    x, y = p[:, 0], p[:, 1]
    return 0.5 * (np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)))


def poly_centroid(p):
    a = poly_area(p)
    if abs(a) < 1e-6:
        return p.mean(0)
    x, y = p[:, 0], p[:, 1]
    cr = x * np.roll(y, -1) - np.roll(x, -1) * y
    return np.array([((x + np.roll(x, -1)) * cr).sum(), ((y + np.roll(y, -1)) * cr).sum()]) / (6 * a)


def point_in_poly(x, y, poly):
    inside = False
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        if (y1 > y) != (y2 > y) and x < (x2 - x1) * (y - y1) / (y2 - y1) + x1:
            inside = not inside
    return inside


def dist_to_poly(pt, poly):
    if point_in_poly(pt[0], pt[1], poly):
        return 0.0
    best = 1e9
    for i in range(len(poly)):
        a, b = poly[i], poly[(i + 1) % len(poly)]
        d = b - a
        t = np.clip(np.dot(pt - a, d) / max(np.dot(d, d), 1e-9), 0, 1)
        best = min(best, float(np.hypot(*(a + t * d - pt))))
    return best


def simplify(pts, tol):
    pts = [tuple(p) for p in pts]
    if len(pts) < 3:
        return pts
    (x1, y1), (x2, y2) = pts[0], pts[-1]
    L = math.hypot(x2 - x1, y2 - y1)
    dmax, idx = 0, 0
    for i, (x, y) in enumerate(pts[1:-1], 1):
        d = abs((y2 - y1) * x - (x2 - x1) * y + x2 * y1 - y2 * x1) / L if L > 1e-6 else math.hypot(x - x1, y - y1)
        if d > dmax:
            dmax, idx = d, i
    if dmax > tol:
        return simplify(pts[:idx + 1], tol)[:-1] + simplify(pts[idx:], tol)
    return [pts[0], pts[-1]]


def ear_clip(poly):
    """단순 다각형 삼각분할 -> 꼭짓점 인덱스 삼각형 목록 (반시계로 맞춘 뒤)"""
    pts = [tuple(p) for p in poly]
    idx = list(range(len(pts)))
    if poly_area(np.array(pts)) < 0:
        idx.reverse()
    tris = []

    def is_ear(i0, i1, i2):
        (ax, ay), (bx, by), (cx, cy) = pts[i0], pts[i1], pts[i2]
        if (bx - ax) * (cy - ay) - (by - ay) * (cx - ax) <= 1e-9:
            return False
        for j in idx:
            if j in (i0, i1, i2):
                continue
            px, py = pts[j]
            d1 = (bx - ax) * (py - ay) - (by - ay) * (px - ax)
            d2 = (cx - bx) * (py - by) - (cy - by) * (px - bx)
            d3 = (ax - cx) * (py - cy) - (ay - cy) * (px - cx)
            if d1 >= 0 and d2 >= 0 and d3 >= 0:
                return False
        return True

    guard = 0
    while len(idx) > 3 and guard < 10000:
        guard += 1
        n = len(idx)
        for k in range(n):
            i0, i1, i2 = idx[(k - 1) % n], idx[k], idx[(k + 1) % n]
            if is_ear(i0, i1, i2):
                tris.append((i0, i1, i2))
                idx.pop(k)
                break
        else:                               # 자기교차 등으로 귀가 없으면 부채꼴로 마무리
            for k in range(1, len(idx) - 1):
                tris.append((idx[0], idx[k], idx[k + 1]))
            return tris
    if len(idx) == 3:
        tris.append(tuple(idx))
    return tris


# =============================================================================================
# 1) 원본 읽기 -> 로컬 좌표 레이어
# =============================================================================================
def load_layers():
    site = json.load(open(os.path.join(SRC, 'campus_osm.json')))
    extra = json.load(open(os.path.join(SRC, 'osm_extra.json')))
    pz = json.load(open(os.path.join(SRC, 'parking_zones_image.json')))
    L = {'boundary': poly_m(site['boundary'])}

    def zone_of(c):          # 번호 없는 건물: KNU 안심동행 지도와 같은 규칙
        x, y = c
        return 1 if y > 180 else 2 if x < -170 else 3 if x > 300 else 5 if y < -250 else 4

    blds = []
    for b in site['buildings']:
        p = np.array(simplify(poly_m(b['coords']), 0.4))
        if len(p) < 3:
            continue
        num = int(b['number']) if b.get('number') else None
        osm_name = (b.get('name') or '').replace('경북대학교 ', '').strip()
        if num is None:
            for key, n in NAME_TO_NUMBER.items():
                if key in osm_name:
                    num = n
        c = poly_centroid(p)
        zone = num // 100 if num else zone_of(c)
        name = GUIDE.get(num) or osm_name or ''
        blds.append({'id': b['id'], 'number': num, 'name': name, 'osm_name': osm_name, 'zone': zone,
                     'height': float(min(max(b.get('height') or 8.5, 4.0), 68.0)), 'poly': p, 'c': c})
    numbered = [b for b in blds if b['number']]
    for b in blds:                    # 번호 없는 건물: 가장 가까운 번호 건물의 구역
        if not b['number'] and numbered:
            b['zone'] = min(numbered, key=lambda o: float(np.hypot(*(o['c'] - b['c']))))['zone']
    L['buildings'] = blds
    L['roads'] = [{'kind': r['kind'], 'name': r.get('name', ''), 'pts': np.array([P(*c) for c in r['coords']])}
                  for r in site['roads'] if len(r['coords']) >= 2]
    L['gates'] = [{'name': g['name'] if g['name'] != 'OSM 출입구' else '출입 게이트', 'named': g['name'] != 'OSM 출입구',
                   'xy': np.array(P(g['lon'], g['lat']))} for g in site['gates']]
    L['water'] = [{'name': w['name'], 'poly': poly_m(w['coords'])} for w in extra['water']]
    L['green'] = [{'name': w['name'], 'kind': w.get('kind', ''), 'poly': poly_m(w['coords'])} for w in extra['green']]
    L['pitch'] = [{'name': w['name'], 'kind': w['kind'], 'sport': w.get('sport', ''), 'poly': poly_m(w['coords'])}
                  for w in extra['pitch']]
    L['car_parking'] = [{'poly': poly_m(w['coords'])} for w in extra['car_parking']]
    L['image_zones'] = [{'color': z['image_color'], 'poly': poly_m(z['lonlat'])} for z in pz['zones']]
    return L


# =============================================================================================
# 2) 지형 (SRTM 평활화)
# =============================================================================================
class Terrain:
    def __init__(self, sigma_cells=1.5):
        d = json.load(open(os.path.join(SRC, 'dem_srtm30.json')))
        self.xs = np.array(d['xs'], float)
        self.ys = np.array(d['ys'], float)
        Z = np.array(d['z'], np.float32).reshape(len(self.ys), len(self.xs))      # 행 = y (남 -> 북)
        Zs = cv2.GaussianBlur(Z, (0, 0), sigma_cells, borderType=cv2.BORDER_REFLECT)
        # 32 m 격자 -> 4 m 격자 bicubic (쌍선형만 쓰면 경사·음영이 격자 모양으로 꺾인다)
        k = 8
        nx, ny = (len(self.xs) - 1) * k + 1, (len(self.ys) - 1) * k + 1
        self.Z = cv2.resize(Zs, (nx, ny), interpolation=cv2.INTER_CUBIC).astype(float)
        self.xs = np.linspace(self.xs[0], self.xs[-1], nx)
        self.ys = np.linspace(self.ys[0], self.ys[-1], ny)
        self.datum = 0.0

    def elev(self, x, y):
        """해발 고도 (m), 쌍선형 보간. x, y 는 스칼라 또는 배열"""
        x = np.asarray(x, float)
        y = np.asarray(y, float)
        dx, dy = self.xs[1] - self.xs[0], self.ys[1] - self.ys[0]
        fx = np.clip((x - self.xs[0]) / dx, 0, len(self.xs) - 1.001)
        fy = np.clip((y - self.ys[0]) / dy, 0, len(self.ys) - 1.001)
        i, j = np.floor(fx).astype(int), np.floor(fy).astype(int)
        u, v = fx - i, fy - j
        Z = self.Z
        return (Z[j, i] * (1 - u) * (1 - v) + Z[j, i + 1] * u * (1 - v) + Z[j + 1, i] * (1 - u) * v
                + Z[j + 1, i + 1] * u * v)

    def z(self, x, y):
        """world z (도크 = 0)"""
        return self.elev(x, y) - self.datum


# =============================================================================================
# 3) 격자 (장애물·도로·여유 거리)
# =============================================================================================
class Grid:
    def __init__(self, bounds, res):
        self.x0, self.y0, self.x1, self.y1 = bounds
        self.res = res
        self.w = int(math.ceil((self.x1 - self.x0) / res))
        self.h = int(math.ceil((self.y1 - self.y0) / res))

    def px(self, pts):
        pts = np.asarray(pts, float)
        return np.stack([(pts[..., 0] - self.x0) / self.res, (self.y1 - pts[..., 1]) / self.res], -1)

    def blank(self):
        return np.zeros((self.h, self.w), np.uint8)

    def fill(self, img, poly, val=255):
        cv2.fillPoly(img, [np.round(self.px(poly) * 8).astype(np.int32)], val, lineType=cv2.LINE_8, shift=3)

    def line(self, img, pts, width_m, val=255):
        q = np.round(self.px(pts) * 8).astype(np.int32)
        cv2.polylines(img, [q], False, val, max(1, int(round(width_m / self.res))), lineType=cv2.LINE_8, shift=3)

    def sample(self, img, x, y):
        c = np.clip(((np.asarray(x) - self.x0) / self.res).astype(int), 0, self.w - 1)
        r = np.clip(((self.y1 - np.asarray(y)) / self.res).astype(int), 0, self.h - 1)
        return img[r, c]


def build_masks(L, g):
    inside = g.blank()
    g.fill(inside, L['boundary'])
    bld = g.blank()
    for b in L['buildings']:
        g.fill(bld, b['poly'])
    water = g.blank()
    for w in L['water']:
        g.fill(water, w['poly'])
    pitch = g.blank()
    for p in L['pitch']:
        g.fill(pitch, p['poly'])
    cars = g.blank()
    for p in L['car_parking']:
        g.fill(cars, p['poly'])
    green = g.blank()
    for p in L['green']:
        g.fill(green, p['poly'])
    roads = g.blank()
    steps = g.blank()
    for r in L['roads']:
        if r['kind'] == 'steps':
            g.line(steps, r['pts'], 3.0)
        else:
            g.line(roads, r['pts'], ROAD_W.get(r['kind'], 2.5))
    solid = (bld > 0) | (water > 0) | (inside == 0)
    return {'inside': inside > 0, 'bld': bld > 0, 'water': water > 0, 'pitch': pitch > 0, 'cars': cars > 0,
            'green': green > 0, 'roads': roads > 0, 'steps': steps > 0, 'solid': solid}


def trim_buildings(L, res=0.25):
    """OSM 건물 외곽선이 차도·보행로를 덮는 부분을 잘라 낸다 (길 위에 벽이 생기지 않게).
    메시·주행 지도·라이다 지도가 모두 이 외곽선을 쓰도록 배치 계산이 끝난 뒤 L['buildings'] 를 바꾼다.
    나뉘면 가장 큰 조각만 번호·이름을 갖는다"""
    out = []
    for b in L['buildings']:
        p = b['poly']
        x0, y0 = p.min(0) - 2
        x1, y1 = p.max(0) + 2
        g = Grid((x0, y0, x1, y1), res)
        bld, road = g.blank(), g.blank()
        g.fill(bld, p)
        for r in L['roads']:
            q = r['pts']
            half = PATH_W.get(r['kind'], 3.0) / 2 + ROAD_BLD_GAP
            if r['kind'] == 'steps' or q[:, 0].max() < x0 - half or q[:, 0].min() > x1 + half \
                    or q[:, 1].max() < y0 - half or q[:, 1].min() > y1 + half:
                continue
            g.line(road, q, 2 * half)
        if not (bld & road).any():
            out.append(b)
            continue
        cs, _ = cv2.findContours((bld & ~road).astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        parts = []
        for c in cs:
            c = cv2.approxPolyDP(c, 0.4 / res, True)[:, 0, :].astype(float)
            if len(c) >= 3 and cv2.contourArea(c.astype(np.float32)) * res * res >= 10.0:
                parts.append(np.stack([g.x0 + (c[:, 0] + 0.5) * res, g.y1 - (c[:, 1] + 0.5) * res], -1))
        parts.sort(key=lambda q: -abs(poly_area(q)))
        for k, q in enumerate(parts):
            out.append(dict(b, poly=q) if k == 0 else
                       dict(b, id=f"{b['id']}#{k}", number=None, name='', osm_name='', poly=q, c=poly_centroid(q)))
    L['buildings'] = out


# =============================================================================================
# 4) 주차구역 · 충전구역 · 거치대 · 슬롯
# =============================================================================================
def anchor_point(spec, L):
    kind = spec[0]
    if kind == 'bldg':
        for b in L['buildings']:
            if b['number'] == spec[1]:
                return b['c']
        raise KeyError(f'건물 {spec[1]} 없음')
    if kind == 'water':
        return poly_centroid(next(w['poly'] for w in L['water'] if w['name'] == spec[1]))
    if kind == 'pitch':
        ps = [p['poly'] for p in L['pitch'] if p['sport'] == spec[1] or p['kind'] == spec[1]]
        return np.mean([poly_centroid(p) for p in ps], axis=0)
    if kind == 'mid':
        return (anchor_point(spec[1], L) + anchor_point(spec[2], L)) / 2
    raise ValueError(spec)


def place_rack(L, g, M, clear, target, length, depth=RACK_DEPTH, zone_poly=None, radius=60.0, taken=(), zone_tol=4.0):
    """도로/보행로 가장자리에 붙는 거치대 자리 찾기.
    -> dict(center, yaw(거치대 축 = 도로 방향), out(도로 -> 거치대 방향 단위벡터), road_kind, access(로봇이 서는 자리))"""
    best, best_s = None, -1e9
    tx, ty = target
    for r in L['roads']:
        if r['kind'] not in ('service', 'residential', 'pedestrian', 'footway'):
            continue
        pts = r['pts']
        if np.min(np.hypot(pts[:, 0] - tx, pts[:, 1] - ty)) > radius + 80:
            continue
        hw = ROAD_W[r['kind']] / 2
        for a, b in zip(pts[:-1], pts[1:]):
            seg = b - a
            sl = float(np.hypot(*seg))
            if sl < length * 0.6:
                continue
            t = seg / sl
            n = np.array([-t[1], t[0]])
            for s_along in np.arange(length / 2 + 0.5, sl - length / 2 - 0.5 + 1e-6, 2.0):
                p = a + t * s_along
                if math.hypot(p[0] - tx, p[1] - ty) > radius:
                    continue
                for side in (1.0, -1.0):
                    out = n * side
                    c = p + out * (hw + 0.5 + depth / 2)
                    if zone_poly is not None and dist_to_poly(c, zone_poly) > zone_tol:
                        continue
                    if any(math.hypot(*(c - q)) < length + 4 for q in taken):
                        continue
                    # 거치대 + 앞뒤 여유 0.3 m 안 칸 검사
                    uu, vv = np.meshgrid(np.linspace(-length / 2 - 0.3, length / 2 + 0.3, 9),
                                         np.linspace(-depth / 2 - 0.3, depth / 2 + 0.3, 5))
                    X = c[0] + uu * t[0] + vv * out[0]
                    Y = c[1] + uu * t[1] + vv * out[1]
                    cl = g.sample(clear, X, Y)
                    if cl.min() < 0.5:
                        continue
                    # 다른 길·운동장·계단·차량 주차장과 겹치면 안 된다 (자기 길 경계 0.25 m 띠는 래스터 오차라 제외)
                    ui, vi = np.meshgrid(np.linspace(-length / 2, length / 2, 9), np.linspace(-depth / 2 + 0.25, depth / 2, 4))
                    Xi = c[0] + ui * t[0] + vi * out[0]
                    Yi = c[1] + ui * t[1] + vi * out[1]
                    if g.sample(M['roads_wide'], Xi, Yi).any() or g.sample(M['pitch_u8'], X, Y).any() \
                            or g.sample(M['steps_u8'], X, Y).any() or g.sample(M['cars_u8'], X, Y).any():
                        continue
                    acc = p + out * max(0.0, hw - 0.55)          # 로봇 중심: 도로 안, 거치대 쪽 가장자리
                    acl = g.sample(clear, [acc[0]], [acc[1]])[0]
                    if acl < 1.1:
                        continue
                    score = (3.0 if r['kind'] in ('service', 'residential') else 1.5 if r['kind'] == 'pedestrian' else 0.0) \
                        + min(float(cl.min()), 3.0) - 0.04 * math.hypot(c[0] - tx, c[1] - ty)
                    if score > best_s:
                        best_s = score
                        best = {'center': c, 'yaw': math.atan2(t[1], t[0]), 'axis': t, 'out': out,
                                'road_kind': r['kind'], 'access': acc, 'length': length, 'depth': depth}
    return best


def make_slots(rack, kind):
    c, t, out = rack['center'], rack['axis'], rack['out']
    slots = []
    head = math.atan2(out[1], out[0])                   # 킥보드 앞(+x)이 거치대 안쪽(도로 반대)을 향한다
    if kind == 'charging':                              # Kickspot 2대: 칸 = 슬롯 줄 (킥보드 중심 = 앞바퀴 축 - 0.42)
        return [{'xy': c + t * (u0 + sy), 'yaw': head, 'role': 'charge'}
                for u0 in KS_UNITS for sy in KS['slots_y']]
    n, pitch = PARK_SLOTS + BROKEN_SLOTS, PARK_PITCH
    roles = ['normal'] * PARK_SLOTS + ['broken'] * BROKEN_SLOTS
    for i in range(n):
        u = (i - (n - 1) / 2) * pitch
        p = c + t * u
        yaw = head + (math.pi if roles[i] == 'broken' else 0.0)   # 고장 칸: 반대 방향
        slots.append({'xy': p, 'yaw': math.atan2(math.sin(yaw), math.cos(yaw)), 'role': roles[i]})
    return slots


def kickspot_poses(rack):
    """충전 거치대 -> Kickspot 2대의 원점 (x, y, yaw). 원점 = 슬롯 줄 앞바퀴 축 (킥보드 중심 + out * 0.42), +x = out"""
    c, t, out = rack['center'], rack['axis'], rack['out']
    yaw = math.atan2(out[1], out[0])
    return [(*(c + t * u0 + out * KS_AXLE), yaw) for u0 in KS_UNITS]


def garage_from_rack(rack, near=''):
    """길가 자리(rack: center, out) -> 로봇 충전소 차고 배치.
    문은 길 쪽(-out), 로봇은 out 방향으로 들어가 차고 안 (1.30, 0) 에서 충전한다. 태그(tag 100) = 뒷벽 접점판 (2.44, 0)"""
    out = rack['out']
    t = np.array([-out[1], out[0]])
    x0, x1 = GS['footprint'][0][0], GS['footprint'][1][0]
    origin = rack['center'] - out * ((x0 + x1) / 2)            # 경사 앞끝 ~ 뒷벽 가운데 = 자리 가운데
    yaw_in = math.atan2(out[1], out[0])
    dx, dy, _ = GS['dock_robot_pose']
    dock = origin + out * dx + t * dy
    tag = origin + out * GS['tag_x']
    normal = -out                                               # 태그 바깥쪽 법선 = 문 쪽
    hw = GS['footprint'][1][1]
    # 지도에 넣을 벽 (x0, x1, y0, y1) 차고 좌표. 지도 격자(0.25 m)는 닿은 칸을 다 칠하므로 옆벽·문기둥은 바깥 4 cm 띠만 그린다
    # (문 반폭 0.70, 로봇 반폭 0.525 -> 그대로 그리면 칸이 문 안쪽 0.45 까지 번져 도크 후진·경로가 막힘)
    walls = []
    wy = max(GS['door_half_width'], hw - 0.04)
    for (a0, a1, b0, b1) in ((0.0, 2.62, wy, hw), (0.0, 2.62, -hw, -wy), (2.50, 2.62, -hw, hw)):
        walls.append(np.array([origin + out * a + t * b for a, b in ((a0, b0), (a1, b0), (a1, b1), (a0, b1))]))
    return {'body': origin, 'yaw': math.atan2(normal[1], normal[0]), 'yaw_in': yaw_in, 'tag': tag, 'dock': dock,
            'dock_distance': float(GS['tag_x'] - dx), 'pre_dock_distance': float(GS['tag_x'] + 2.0),
            'floor_z': GS['floor_z'], 'walls': walls, 'road_kind': rack['road_kind'], 'near': near}


def nearest_building_name(L, pt, numbered_only=True):
    best, bd = None, 1e9
    for b in L['buildings']:
        if numbered_only and not b['number']:
            continue
        d = dist_to_poly(pt, b['poly'])
        if d < bd:
            best, bd = b, d
    return best


def build_zones(L, g, M, clear):
    imgz = [dict(z, centroid=poly_centroid(z['poly'])) for z in L['image_zones']]
    # 1) 충전 후보 -> 가까운 이미지 구역 배정 (가까운 것부터)
    pairs = []
    for key, label, spec in CHARGING_SITES:
        a = anchor_point(spec, L)
        for k, z in enumerate(imgz):
            pairs.append((dist_to_poly(a, z['poly']), key, k))
    pairs.sort()
    site_zone, used = {}, set()
    for d, key, k in pairs:
        if key in site_zone or k in used or d > SITE_MATCH_M:
            continue
        site_zone[key] = k
        used.add(k)
    zones = []
    taken = []
    # 2) 충전구역
    for key, label, spec in CHARGING_SITES:
        a = anchor_point(spec, L)
        length = CHARGE_SLOTS * CHARGE_PITCH
        if key in site_zone:
            z = imgz[site_zone[key]]
            rack = place_rack(L, g, M, clear, a, length, zone_poly=z['poly'], radius=90, taken=taken) \
                or place_rack(L, g, M, clear, a, length, radius=70, taken=taken)
            poly, origin = z['poly'], 'image'
        else:
            rack = place_rack(L, g, M, clear, a, length, radius=70, taken=taken)
            poly, origin = None, 'new'
        if rack is None:
            print(f'  ! {key} {label}: 거치대 자리를 찾지 못함')
            continue
        if poly is None:      # 새 충전구역: 거치대 둘레 3 m
            c, t, o = rack['center'], rack['axis'], rack['out']
            hl, hd = length / 2 + 3, RACK_DEPTH / 2 + 3
            poly = np.array([c - t * hl - o * hd, c + t * hl - o * hd, c + t * hl + o * hd, c - t * hl + o * hd])
        taken.append(rack['center'])
        zones.append({'id': key, 'type': 'charging', 'site': label, 'origin': origin, 'poly': poly, 'rack': rack,
                      'slots': make_slots(rack, 'charging'), 'anchor': a,
                      'image_index': site_zone.get(key)})
    # 3) 나머지 이미지 구역 = 일반 주차구역
    rest = [k for k in range(len(imgz)) if k not in used]
    rest.sort(key=lambda k: (-imgz[k]['centroid'][1], imgz[k]['centroid'][0]))   # 북 -> 남
    n = 0
    for k in rest:
        z = imgz[k]
        length = (PARK_SLOTS + BROKEN_SLOTS) * PARK_PITCH
        rack = place_rack(L, g, M, clear, z['centroid'], length, zone_poly=z['poly'], radius=120, taken=taken) \
            or place_rack(L, g, M, clear, z['centroid'], length, zone_poly=z['poly'], radius=160, taken=taken,
                          zone_tol=20.0)       # 구역 안에 길이 없으면 구역 가장자리 20 m 안의 길가
        if rack is None:
            print(f'  ! 이미지 구역 {k}: 거치대 자리를 찾지 못함 (구역만 표시)')
        n += 1
        b = nearest_building_name(L, rack['center'] if rack else z['centroid'])
        label = f"{b['number']} {b['name']}" if b else ''
        if rack is not None:
            taken.append(rack['center'])
        zones.append({'id': f'P{n:02d}', 'type': 'general', 'site': f'{label} 인근', 'origin': 'image',
                      'poly': z['poly'], 'rack': rack, 'slots': make_slots(rack, 'general') if rack else [],
                      'anchor': z['centroid'], 'image_index': k})
    return edit_zones(zones)


def edit_zones(zones):
    out, moved = [], []
    for z in zones:
        if z['id'] in DELETED_ZONES:
            continue
        if z['id'] in GENERAL_FROM_CHARGING:
            new_id, site = GENERAL_FROM_CHARGING[z['id']]
            rack = dict(z['rack'], length=(PARK_SLOTS + BROKEN_SLOTS) * PARK_PITCH)
            moved.append(dict(z, id=new_id, type='general', site=site, rack=rack, slots=make_slots(rack, 'general')))
            continue
        out.append(z)
    return out + moved


def rack_apron(center, out, depth, road_kind, length, reach):
    """길 가운데에서 거치대 쪽으로 reach(거치대 중심 기준, out 방향)까지의 포장 면. 길과 거치대 사이 틈을 잇는다"""
    t = np.array([-out[1], out[0]])
    p = center - out * (ROAD_W[road_kind] / 2 + 0.5 + depth / 2)
    q = center + out * reach
    return np.array([p - t * length / 2, p + t * length / 2, q + t * length / 2, q - t * length / 2])


def zone_apron(z):
    """일반 구역: 슬롯 뒤(길 반대쪽)에도 로봇이 서야 한다 (후진 적재 자리 = 슬롯 중심 앞 2.07 m) -> 3 m 뒤까지, 양끝 3.5 m 돌아갈 폭.
    충전 구역: 기둥 앞까지만"""
    r = z['rack']
    if z['type'] == 'general':
        return rack_apron(r['center'], r['out'], r['depth'], r['road_kind'], r['length'] + 7.0, 3.2)
    return rack_apron(r['center'], r['out'], r['depth'], r['road_kind'], r['length'] + 3.0, -r['depth'] / 2 + 0.3)


def place_dock(L, g, M, clear):
    """로봇 충전소(차고)를 본관 옆 길가에 둔다. 문이 길 쪽"""
    a = anchor_point(ROBOT_DOCK_ANCHOR, L)
    rack = place_rack(L, g, M, clear, a, GARAGE_W + 0.6, depth=GARAGE_LEN, radius=80)
    return garage_from_rack(rack, f"{ROBOT_DOCK_ANCHOR[1]} {GUIDE[ROBOT_DOCK_ANCHOR[1]]}")


# =============================================================================================
# 5) 메시 (OBJ)
# =============================================================================================
class Obj:
    def __init__(self):
        self.v, self.vt, self.groups = [], [], {}

    def add_v(self, p):
        self.v.append(p)
        return len(self.v)

    def tri(self, mat, a, b, c, uv=None):
        self.groups.setdefault(mat, []).append((a, b, c, uv))

    def quad(self, mat, a, b, c, d):
        self.tri(mat, a, b, c)
        self.tri(mat, a, c, d)

    def box(self, mat, cx, cy, cz, sx, sy, sz, yaw=0.0):
        cs, sn = math.cos(yaw), math.sin(yaw)
        ids = []
        for dz in (-sz / 2, sz / 2):
            for dx, dy in ((-sx / 2, -sy / 2), (sx / 2, -sy / 2), (sx / 2, sy / 2), (-sx / 2, sy / 2)):
                ids.append(self.add_v((cx + dx * cs - dy * sn, cy + dx * sn + dy * cs, cz + dz)))
        b0, b1, b2, b3, t0, t1, t2, t3 = ids
        self.quad(mat, b0, b3, b2, b1)
        self.quad(mat, t0, t1, t2, t3)
        for p, q, r, s in ((b0, b1, t1, t0), (b1, b2, t2, t1), (b2, b3, t3, t2), (b3, b0, t0, t3)):
            self.quad(mat, p, q, r, s)

    def prism(self, mat, poly, z0, z1, roof_mat=None):
        poly = np.array(poly)
        if poly_area(poly) < 0:
            poly = poly[::-1]
        n = len(poly)
        bot = [self.add_v((x, y, z0)) for x, y in poly]
        top = [self.add_v((x, y, z1)) for x, y in poly]
        for i in range(n):
            j = (i + 1) % n
            self.quad(mat, bot[i], bot[j], top[j], top[i])
        for a, b, c in ear_clip(poly):
            self.tri(roof_mat or mat, top[a], top[b], top[c])

    def write(self, path, mtl_name, materials, smooth=False):
        """smooth=True: 꼭짓점 법선(지형), False: 면 법선(건물·거치대). 법선이 없으면 gz 물리 엔진이 메시를 버린다."""
        V = np.array(self.v, float)
        faces = [(mat, a, b, c, uv) for mat, tris in self.groups.items() for a, b, c, uv in tris]
        F = np.array([(a - 1, b - 1, c - 1) for _, a, b, c, _ in faces], int)
        FN = np.cross(V[F[:, 1]] - V[F[:, 0]], V[F[:, 2]] - V[F[:, 0]])
        if smooth:
            VN = np.zeros_like(V)
            for k in range(3):
                np.add.at(VN, F[:, k], FN)
            VN /= np.maximum(np.linalg.norm(VN, axis=1, keepdims=True), 1e-9)
        else:
            FN = FN / np.maximum(np.linalg.norm(FN, axis=1, keepdims=True), 1e-9)
        with open(path, 'w') as f:
            f.write(f'# scripts/build_knu_campus.py 가 만든 파일\nmtllib {mtl_name}\n')
            for x, y, z in self.v:
                f.write(f'v {x:.3f} {y:.3f} {z:.3f}\n')
            for u, w in self.vt:
                f.write(f'vt {u:.5f} {w:.5f}\n')
            for n in (VN if smooth else FN):
                f.write(f'vn {n[0]:.4f} {n[1]:.4f} {n[2]:.4f}\n')
            k = 0
            for mat, tris in self.groups.items():
                f.write(f'g {mat}\nusemtl {mat}\n')
                for a, b, c, uv in tris:
                    k += 1
                    na, nb, nc = (a, b, c) if smooth else (k, k, k)
                    if uv:
                        f.write(f'f {a}/{uv[0]}/{na} {b}/{uv[1]}/{nb} {c}/{uv[2]}/{nc}\n')
                    else:
                        f.write(f'f {a}//{na} {b}//{nb} {c}//{nc}\n')
        with open(os.path.join(os.path.dirname(path), mtl_name), 'w') as f:
            for mat, (rgb, tex) in materials.items():
                r, g_, b = rgb
                f.write(f'newmtl {mat}\nKa {r:.3f} {g_:.3f} {b:.3f}\nKd {r:.3f} {g_:.3f} {b:.3f}\n'
                        f'Ks 0.05 0.05 0.05\nNs 10\nd 1.0\nillum 2\n')
                if tex:
                    f.write(f'map_Kd {tex}\n')
                f.write('\n')


def hex_rgb(h):
    h = h.lstrip('#')
    return tuple(int(h[i:i + 2], 16) / 255 for i in (0, 2, 4))


RACK_COLORS = {
    'rack_base': (0.16, 0.16, 0.18), 'rack_purple': (0.55, 0.42, 0.85), 'rack_kiosk': (0.25, 0.26, 0.29),
    'charge_teal': (0.18, 0.66, 0.60), 'charge_dark': (0.10, 0.12, 0.13), 'charge_sign': (0.15, 0.35, 0.80),
    'broken_red': (0.85, 0.20, 0.22), 'gate_red': (0.80, 0.18, 0.22), 'gate_white': (0.92, 0.92, 0.90),
    'dock_blue': (0.20, 0.40, 0.80),
}


def rack_geometry(obj, zones, gates, dock, T, flat):
    zf = (lambda x, y: 0.0) if flat else (lambda x, y: float(T.z(x, y)))
    for gte in gates:
        x, y = gte['xy']
        z0 = zf(x, y)
        obj.box('gate_red', x, y, z0 + 1.6, 0.6, 0.6, 3.2)
        obj.box('gate_white', x, y, z0 + 3.35, 0.8, 0.8, 0.3)
    if not obj.groups:                      # 빈 메시는 gz 가 싫어함
        obj.box('gate_white', 0.0, 0.0, -50.0, 0.1, 0.1, 0.1)


# =============================================================================================
# 6) 텍스처 / 2D 지도
# =============================================================================================
TEX_COL = {
    'outside': (205, 207, 203), 'campus': (214, 210, 196), 'green': (188, 212, 156), 'water': (110, 165, 215),
    'pitch': (120, 175, 105), 'track': (196, 112, 92), 'cars': (112, 114, 120), 'service': (78, 80, 88),
    'pedestrian': (226, 190, 150), 'footway': (236, 208, 164), 'steps': (150, 128, 100),
    'curb': (150, 112, 80), 'road_edge': (235, 235, 235), 'center_line': (245, 196, 40),
    'zone_general': (30, 105, 255), 'zone_charge': (0, 205, 160), 'contour': (150, 120, 90),
    'bld_floor': (120, 120, 125), 'boundary': (180, 60, 60),
}


def hillshade(T, g, exaggeration=4.0, strength=0.22):
    """북서쪽 빛 음영 (0~1 배율 배열, g 격자 크기). 지형이 완만해서 높이를 과장해 보이게 한다"""
    step = 4.0
    xs = np.arange(g.x0, g.x1 + step, step)
    ys = np.arange(g.y1, g.y0 - step, -step)
    X, Y = np.meshgrid(xs, ys)
    E = T.elev(X, Y) * exaggeration
    gy, gx = np.gradient(E, -step, step)
    nx, ny, nz = -gx, -gy, np.ones_like(E)
    n = np.sqrt(nx ** 2 + ny ** 2 + nz ** 2)
    lx, ly, lz = -0.5, 0.5, 0.7
    shade = (nx * lx + ny * ly + nz * lz) / n / math.sqrt(lx * lx + ly * ly + lz * lz)
    shade = 1.0 - strength * (1.0 - np.clip(shade, 0, 1)) / max(1e-6, 1.0 - float(np.clip(shade, 0, 1).min()))
    return cv2.resize(shade.astype(np.float32), (g.w, g.h), interpolation=cv2.INTER_LINEAR)


def draw_roads(d, L, g):
    """보행로(밝은 포장 + 갈색 연석)와 차도(아스팔트 + 흰 가장자리선 + 노란 중앙선)"""
    def px_w(m):
        return max(1, int(round(m / g.res)))

    def line(pts, col, width_m):
        q = [tuple(v) for v in g.px(np.asarray(pts))]
        w = px_w(width_m)
        d.line(q, fill=col, width=w, joint='curve')
        for x, y in (q[0], q[-1]):                   # 끝을 둥글게 (갈림길 틈 메움)
            d.ellipse([x - w / 2, y - w / 2, x + w / 2, y + w / 2], fill=col)

    def of(*kinds):
        return [r for r in L['roads'] if r['kind'] in kinds]

    walk, drive = of('footway', 'pedestrian'), of('service', 'residential')
    for r in walk:
        line(r['pts'], TEX_COL['curb'], PATH_W[r['kind']])
    for r in walk:
        line(r['pts'], TEX_COL[r['kind']], PATH_W[r['kind']] - 0.4)
    for r in of('steps'):                            # 계단: 가로 줄무늬
        line(r['pts'], TEX_COL['steps'], PATH_W['steps'])
        for a, b in zip(r['pts'][:-1], r['pts'][1:]):
            ln = float(np.hypot(*(b - a)))
            if ln < 0.1:
                continue
            t = (b - a) / ln
            n = np.array([-t[1], t[0]]) * (PATH_W['steps'] / 2)
            for u in np.arange(0.3, ln, 0.7):
                d.line([tuple(v) for v in g.px(np.array([a + t * u - n, a + t * u + n]))], fill=(95, 78, 60),
                       width=px_w(0.18))
    for r in drive:
        line(r['pts'], TEX_COL['road_edge'], PATH_W[r['kind']])
    for r in drive:
        line(r['pts'], TEX_COL['service'], PATH_W[r['kind']] - 0.5)
    for r in drive:                                  # 중앙선 (점선 3 m 칠, 2 m 비움)
        done = 0.0
        for a, b in zip(r['pts'][:-1], r['pts'][1:]):
            ln = float(np.hypot(*(b - a)))
            if ln < 0.1:
                continue
            t = (b - a) / ln
            u = 0.0
            while u < ln - 1e-6:
                phase = (done + u) % 5.0
                step = min((3.0 if phase < 3.0 else 5.0) - phase, ln - u)
                step = max(step, 1e-3)
                if phase < 3.0:
                    d.line([tuple(v) for v in g.px(np.array([a + t * u, a + t * (u + step)]))],
                           fill=TEX_COL['center_line'], width=px_w(0.16))
                u += step
            done += ln


def zone_shapes(z):
    """칠할 면: 구역 다각형. 슬롯이 다각형 밖으로 나가면 슬롯 줄도 함께 칠한다"""
    shapes = [np.asarray(z['poly'])]
    r = z['rack']
    if r and any(dist_to_poly(s['xy'], z['poly']) > 0.1 for s in z['slots']):
        shapes.append(zone_apron(z))
    return shapes


def paint_zone(img, g, z, alpha=105):
    """주차구역: 안쪽을 반투명하게 칠하고(일반 파랑, 충전 민트) 같은 색 테두리. 밑의 길이 비쳐 보인다"""
    col = TEX_COL['zone_charge'] if z['type'] == 'charging' else TEX_COL['zone_general']
    shapes = [g.px(s) for s in zone_shapes(z)]
    lw = max(2, int(round(0.7 / g.res)))
    allp = np.vstack(shapes)
    x0, y0 = np.floor(allp.min(0)).astype(int) - lw
    x1, y1 = np.ceil(allp.max(0)).astype(int) + lw
    fill = Image.new('L', (int(x1 - x0), int(y1 - y0)), 0)
    edge = Image.new('L', fill.size, 0)
    df, de = ImageDraw.Draw(fill), ImageDraw.Draw(edge)
    qs = [[(float(x - x0), float(y - y0)) for x, y in s] for s in shapes]
    for q in qs:
        df.polygon(q, fill=alpha)
        de.line(q + q[:2], fill=255, width=lw, joint='curve')
    img.paste(col, (int(x0), int(y0)), fill)
    img.paste(col, (int(x0), int(y0)), edge)


def draw_base(L, T, g, scale_px, M=None, contours=True, zones=None, shade=False):
    """g 격자 기준 바닥 그림 (PIL Image, RGB)"""
    W, H = g.w, g.h
    img = Image.new('RGB', (W, H), TEX_COL['outside'])
    d = ImageDraw.Draw(img)

    def pp(poly):
        q = g.px(np.asarray(poly))
        return [tuple(v) for v in q]

    d.polygon(pp(L['boundary']), fill=TEX_COL['campus'])
    for gr in L['green']:
        d.polygon(pp(gr['poly']), fill=TEX_COL['green'])
    for c in L['car_parking']:
        d.polygon(pp(c['poly']), fill=TEX_COL['cars'])
    for p in L['pitch']:
        col = TEX_COL['track'] if p['kind'] in ('track', 'stadium') else TEX_COL['pitch']
        d.polygon(pp(p['poly']), fill=col)
    for p in L['pitch']:
        if p['kind'] == 'pitch' and p['sport'] in ('soccer', 'baseball'):
            d.polygon(pp(p['poly']), fill=TEX_COL['pitch'])
    for w in L['water']:
        d.polygon(pp(w['poly']), fill=TEX_COL['water'])
    if shade:
        a = np.asarray(img).astype(np.float32) * hillshade(T, g)[..., None]
        img = Image.fromarray(np.clip(a, 0, 255).astype(np.uint8))
        d = ImageDraw.Draw(img)
    if contours:
        for lev, lines in contour_lines(T, L):
            for ln in lines:
                width = 3 if lev % 10 == 0 else 1
                d.line(pp(ln), fill=TEX_COL['contour'], width=max(1, int(width * scale_px / 3.3)))
    draw_roads(d, L, g)
    if zones:
        for z in zones:
            paint_zone(img, g, z)
        d = ImageDraw.Draw(img)
    for b in L['buildings']:
        d.polygon(pp(b['poly']), fill=TEX_COL['bld_floor'])
    return img


_CONTOURS = None


def contour_lines(T, L, step=2.0):
    global _CONTOURS
    if _CONTOURS is not None:
        return _CONTOURS
    xs = np.arange(-640, 641, 4.0)
    ys = np.arange(-640, 641, 4.0)
    X, Y = np.meshgrid(xs, ys)
    E = T.elev(X, Y)
    out = []
    for lev in np.arange(math.ceil(E.min() / step) * step, E.max(), step):
        m = (E >= lev).astype(np.uint8)
        cs, _ = cv2.findContours(m, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        lines = []
        for c in cs:
            c = c[:, 0, :].astype(float)
            # 격자 가장자리 점은 버리고 이어진 구간만
            pts = np.stack([xs[0] + c[:, 0] * 4.0, ys[0] + c[:, 1] * 4.0], -1)
            edge = (c[:, 0] <= 0) | (c[:, 1] <= 0) | (c[:, 0] >= len(xs) - 1) | (c[:, 1] >= len(ys) - 1)
            cur = []
            for p, e in zip(pts, edge):
                if e:
                    if len(cur) > 3:
                        lines.append(np.array(cur))
                    cur = []
                else:
                    cur.append(p)
            if len(cur) > 3:
                lines.append(np.array(cur + ([cur[0]] if not edge.any() else [])))
        lines = [np.array(simplify(ln, 1.0)) for ln in lines if len(ln) > 3]
        out.append((float(lev), lines))
    _CONTOURS = out
    return out


# =============================================================================================
# 7) 출력
# =============================================================================================
def write_terrain_model(L, T, M, zones, bounds, flat):
    mdir = os.path.join(PKG, 'models', NAME)
    os.makedirs(os.path.join(mdir, 'meshes'), exist_ok=True)
    x0, y0, x1, y1 = bounds
    sfx = '_flat' if flat else ''
    # --- 지형: 격자 메시 + 텍스처 ---
    obj = Obj()
    xs = np.arange(x0, x1 + 1e-6, TERRAIN_STEP)
    ys = np.arange(y0, y1 + 1e-6, TERRAIN_STEP)
    ids = {}
    for j, y in enumerate(ys):
        for i, x in enumerate(xs):
            z = 0.0 if flat else float(T.z(x, y))
            ids[i, j] = obj.add_v((x, y, z))
            obj.vt.append(((x - x0) / (x1 - x0), (y - y0) / (y1 - y0)))
    for j in range(len(ys) - 1):
        for i in range(len(xs) - 1):
            a, b, c, d = ids[i, j], ids[i + 1, j], ids[i + 1, j + 1], ids[i, j + 1]
            obj.tri('ground', a, b, c, (a, b, c))
            obj.tri('ground', a, c, d, (a, c, d))
    obj.write(os.path.join(mdir, 'meshes', f'terrain{sfx}.obj'), f'terrain{sfx}.mtl',
              {'ground': ((1, 1, 1), 'campus_ground.png')}, smooth=True)
    # --- 건물: 구역 색 프리즘 (바닥은 지형 최저점 아래 2 m 까지 묻는다) ---
    obj = Obj()
    for b in L['buildings']:
        zb = 0.0 if flat else float(T.z(b['poly'][:, 0], b['poly'][:, 1]).min())
        obj.prism(f'zone{b["zone"]}', b['poly'], zb - 2.0, zb + b['height'] * BLD_VIS_H, roof_mat=f'roof{b["zone"]}')
    mats = {}
    for k, (_, _, col) in ZONES.items():
        r, g_, bb = hex_rgb(col)
        mats[f'zone{k}'] = ((r, g_, bb), None)
        mats[f'roof{k}'] = ((r * 0.75, g_ * 0.75, bb * 0.75), None)
    obj.write(os.path.join(mdir, 'meshes', f'buildings{sfx}.obj'), f'buildings{sfx}.mtl', mats)
    # --- 거치대 · 충전 스탠드 · 출입문 ---
    obj = Obj()
    rack_geometry(obj, zones, L['gates'], None, T, flat)
    obj.write(os.path.join(mdir, 'meshes', f'racks{sfx}.obj'), f'racks{sfx}.mtl',
              {k: (v, None) for k, v in RACK_COLORS.items()})
    return mdir


def write_texture(L, T, zones, bounds):
    x0, y0, x1, y1 = bounds
    long_side = max(x1 - x0, y1 - y0)
    g = Grid(bounds, long_side / (TEX * TEX_SS))
    img = draw_base(L, T, g, scale_px=1.0 / g.res, zones=zones)
    img = img.resize((TEX, TEX), Image.LANCZOS)
    path = os.path.join(PKG, 'models', NAME, 'meshes', 'campus_ground.png')
    img.save(path, optimize=True)
    return path


def write_world(L, T, zones, dock, flat):
    sfx = '_flat' if flat else ''
    zf = (lambda x, y: 0.0) if flat else (lambda x, y: float(T.z(x, y)))
    bx, by = dock['body']
    dz = zf(bx, by)
    kicks = kickspot_includes(zones, zf)
    sdf = f'''<?xml version="1.0"?>
<!-- 경북대학교 대구캠퍼스 ({'평지' if flat else '지형 포함'}). scripts/build_knu_campus.py 가 만든 파일 — 직접 고치지 말고 스크립트를 고칠 것.
     world 프레임 = map 프레임 (+x 동, +y 북, 원점 위도 {LAT0} 경도 {LON0}). 맵 설정: config/maps/{NAME}.yaml
     지형 z 는 로봇 도크(차고 옆) = 0. 주차구역 {sum(z['type'] == 'general' for z in zones)}곳, 충전 주차구역 {sum(z['type'] == 'charging' for z in zones)}곳.
     킥보드는 scooter_fleet_sim 이 시나리오대로 만든다. -->
<sdf version="1.9">
  <world name="{NAME}{sfx}">
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
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>{LAT0}</latitude_deg>
      <longitude_deg>{LON0}</longitude_deg>
      <elevation>{T.datum:.1f}</elevation>
    </spherical_coordinates>
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
    <!-- 지형: SRTM 고도 평활화 + 바닥 그림(도로·보행로·잔디·운동장·일청담·등고선·주차/충전 칸) -->
    <model name="terrain">
      <static>true</static>
      <link name="link">
        <collision name="c"><geometry><mesh><uri>model://{NAME}/meshes/terrain{sfx}.obj</uri></mesh></geometry></collision>
        <visual name="v"><geometry><mesh><uri>model://{NAME}/meshes/terrain{sfx}.obj</uri></mesh></geometry></visual>
      </link>
    </model>
    <!-- 건물 94동 (OSM 외곽선 + 높이, 안내도 5개 구역 색) -->
    <model name="buildings">
      <static>true</static>
      <link name="link">
        <collision name="c"><geometry><mesh><uri>model://{NAME}/meshes/buildings{sfx}.obj</uri></mesh></geometry></collision>
        <visual name="v"><geometry><mesh><uri>model://{NAME}/meshes/buildings{sfx}.obj</uri></mesh></geometry></visual>
      </link>
    </model>
    <!-- 출입문 기둥 (일반 주차구역에는 거치대 없음 — 바닥 그림만) -->
    <model name="kickboard_racks">
      <static>true</static>
      <link name="link">
        <visual name="v"><geometry><mesh><uri>model://{NAME}/meshes/racks{sfx}.obj</uri></mesh></geometry></visual>
      </link>
    </model>
    <!-- 킥보드 충전구역: Kickspot 4구 x 2 (Blender) -->
{kicks}
    <!-- 로봇 충전소 차고 ({dock['near']} 옆, Blender). 문이 길 쪽, 로봇은 안에서 충전 -->
    <include>
      <uri>model://scootover_robot_station</uri>
      <name>charging_station</name>
      <pose>{bx:.3f} {by:.3f} {dz:.3f} 0 0 {dock['yaw_in']:.4f}</pose>
    </include>
  </world>
</sdf>
'''
    path = os.path.join(PKG, 'worlds', f'{NAME}{sfx}.sdf')
    with open(path, 'w') as f:
        f.write(sdf)
    return path


def kickspot_includes(zones, zf=lambda x, y: 0.0):
    out = []
    for z in zones:
        if z['type'] != 'charging' or not z['rack']:
            continue
        for k, (x, y, yaw) in enumerate(kickspot_poses(z['rack'])):
            out.append(f'    <include><uri>model://kickspot_4</uri><name>{z["id"]}_kickspot_{k + 1}</name>'
                       f'<pose>{x:.3f} {y:.3f} {zf(x, y):.3f} 0 0 {yaw:.4f}</pose></include>')
    return '\n'.join(out)


def rack_boxes(zones):
    """충전 거치대(Kickspot)의 기둥 줄 -> [(cx, cy, 길이, 폭, yaw)]. 슬롯과 그 앞은 비워 둔다.
    주행·라이다 지도에 장애물로 넣는다 (Gazebo 에도 충돌체가 있다). 일반 주차구역에는 거치대가 없다"""
    out = []
    for z in zones:
        r = z['rack']
        if not r or z['type'] != 'charging':
            continue
        for x, y, _ in kickspot_poses(r):          # 기둥 줄 (앞바퀴 축 앞 0 ~ 0.6 m)
            p = np.array([x, y]) + r['out'] * 0.30
            out.append((p[0], p[1], KS_UNIT_W, 0.6, r['yaw']))
    return out


def write_nav_maps(M, g, zones=(), dock=None, name=None):
    """dock: 차고 벽을 그려 넣음. name: 다른 이름으로 저장 (시나리오 전용 맵)"""
    racks = g.blank()
    for w in (dock or {}).get('walls', []):
        g.fill(racks, w)
    for cx, cy, L, W, yaw in rack_boxes(zones):
        t, n = np.array([math.cos(yaw), math.sin(yaw)]), np.array([-math.sin(yaw), math.cos(yaw)])
        c = np.array([cx, cy])
        g.fill(racks, np.array([c - t * L / 2 - n * W / 2, c + t * L / 2 - n * W / 2,
                                c + t * L / 2 + n * W / 2, c - t * L / 2 + n * W / 2]))
    occ_l = M['solid'] | (racks > 0)
    occ_n = occ_l | M['pitch'] | M['steps']
    base = name or NAME
    for suffix, occ in (('', occ_n), ('_lidar', occ_l)):
        img = np.where(occ, 0, 254).astype(np.uint8)
        Image.fromarray(img).save(os.path.join(PKG, 'maps', f'{base}{suffix}.png'), optimize=True)
        for nm in ((base,) if name else (NAME, NAME + '_flat')):          # 평지 world 도 같은 지도를 쓴다
            with open(os.path.join(PKG, 'maps', f'{nm}{suffix}.yaml'), 'w') as f:
                f.write(f'image: {base}{suffix}.png\nmode: trinary\nresolution: {g.res}\n'
                        f'origin: [{g.x0:.1f}, {g.y0:.1f}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')


def r3(v):
    return [round(float(a), 3) for a in v]


def write_config(zones, dock, T, bounds):
    gen = [z for z in zones if z['type'] == 'general']
    chg = [z for z in zones if z['type'] == 'charging']

    def zone_yaml(z):
        r = z['rack']
        L_ = [f'  - name: {z["id"]}', f'    site: "{z["site"]}"', f'    origin: {z["origin"]}',
              '    polygon: [' + ', '.join(f'[{p[0]:.1f}, {p[1]:.1f}]' for p in z['poly']) + ']']
        if r:
            a = r['access']
            L_ += [f'    rack: {{x: {r["center"][0]:.2f}, y: {r["center"][1]:.2f}, yaw_deg: {math.degrees(r["yaw"]):.1f}, '
                   f'length: {r["length"]:.2f}, road: {r["road_kind"]}}}',
                   f'    robot_access: {{x: {a[0]:.2f}, y: {a[1]:.2f}, yaw_deg: {math.degrees(r["yaw"]):.1f}}}',
                   '    slots:          # role: normal | broken(고장 분류, 반대 방향) | charge']
            for i, s in enumerate(z['slots']):
                L_.append(f'      - {{id: {z["id"]}-{i + 1:02d}, x: {s["xy"][0]:.2f}, y: {s["xy"][1]:.2f}, '
                          f'z: {float(T.z(*s["xy"])):.2f}, yaw_deg: {math.degrees(s["yaw"]):.1f}, role: {s["role"]}}}')
        return L_

    tx, ty = dock['tag']
    sx, sy = dock['dock']
    yaw_out = math.degrees(dock['yaw'])
    L_ = ['# 경북대학교 대구캠퍼스 맵 설정. scripts/build_knu_campus.py 가 만든 파일 — 배치를 바꾸려면 스크립트를 고쳐 다시 생성.',
          'mission_variant: campus     # launch 가 mission_manager_campus 를 띄운다 (거치대 슬롯 하역)',
          f'map_json: {NAME}.json      # maps/ 아래. nav_bridge 큰 경로, 지형 높이, 시각화',
          'gps_origin:            # map (0,0) 의 위경도. +x = 동, +y = 북',
          f'  lat: {LAT0}', f'  lon: {LON0}', '',
          'terrain:               # 지형 높이: maps/knu_campus.json 의 elevation 격자 (world z = 해발 - datum)',
          f'  datum_elev_m: {T.datum:.2f}', '  source: SRTM 30 m (OpenTopoData), 가우시안 평활 48 m', '',
          f'robot_spawn:           # = 도킹 pose: 로봇 충전소 차고 안 ({dock["near"]} 옆), 바닥 높이 z',
          f'  x: {sx:.3f}', f'  y: {sy:.3f}', f'  z: {dock["floor_z"]:.2f}', f'  yaw_deg: {(yaw_out + 180) % 360:.1f}', '',
          'charger:', '  station_id: station_1', '  tag_id: 100',
          f'  tag_pose: {{x: {tx:.3f}, y: {ty:.3f}, yaw_deg: {yaw_out:.1f}}}   # 차고 뒷벽 접점판, yaw = 바깥쪽 법선 (문 쪽)',
          f'  dock_distance: {dock["dock_distance"]:.2f}      # 충전 위치 (차고 안 1.30) ~ 뒷벽 접점판 (2.44)',
          f'  pre_dock_distance: {dock["pre_dock_distance"]:.2f}  # 문 앞 2 m',
          '  station: {model: scootover_robot_station, '
          f'x: {dock["body"][0]:.3f}, y: {dock["body"][1]:.3f}, yaw_deg: {math.degrees(dock["yaw_in"]):.1f}}}   # 차고 원점 = 문 앞 지면', '',
          f'# 일반 주차구역 {len(gen)}곳: 킥보드 앱의 주차 가능 구역(사용자 제공 이미지) 그대로. 거치대 없이 바닥 칸 정상 {PARK_SLOTS} + 고장 분류 {BROKEN_SLOTS} ({", ".join(DELETED_ZONES)} 삭제)',
          'parking_zones:']
    for z in gen:
        L_ += zone_yaml(z)
    L_ += ['', f'# 충전 주차구역 {len(chg)}곳: 사람이 많이 오가는 곳. Kickspot 4구 x 2 = {CHARGE_SLOTS}칸.',
           '# 칸 x, y = 꽂힌 킥보드 중심 (앞바퀴 축 - 0.42), yaw = 기둥 쪽',
           'charging_zones:']
    for z in chg:
        L_ += zone_yaml(z)
    L_ += ['', 'params:                # 이 맵에서 덮어쓸 노드 파라미터 (launch/system.launch.py 가 적용)',
           '  nav_bridge:', '    v_max: 1.2', '    w_max: 1.0', '    lookahead: 1.2', '    goal_timeout_sec: 1200.0',
           '    replan_sec: 3.0', f'    costmap_res: {RES_NAV}', '    goal_clearance: 1.2', '    inflation_radius: 1.2',
           '    window_m: 70.0', '    max_retries: 2',
           '  battery_manager:', '    k1_pct_per_m: 0.02', '    k2_pct_per_s: 0.01',
           '  queue_manager:          # 경북대 맵 가중치 (2026-11-23, B): 거리·넘어짐 중심, 1점 = 1 m',
           '    w_r: 100.0', '    w_t: 0.5', '    w_d: 1.0', '    d_max: 1200.0',
           '  request_gateway:', '    max_range_m: 1200.0      # 캠퍼스 전체 (원점에서 서쪽 끝까지 약 600 m)',
           '  sim_lidar:', '    range_max: 12.0',
           '  collector:', '    align_timeout_sec: 120.0', '    search_radius_m: 10.5      # 확장 탐색 (GPS 오차 큰 신고)',
           '  mission_manager:', '    return_battery_pct: 20.0',
           '    undock_distance: 3.0          # 차고 안 충전 위치에서 문 밖까지 후진 (문 1.30 + 경사 0.45 + 여유)',
           '  scooter_fleet_sim:', '    charge_rate_pct_per_s: 0.2']
    path = os.path.join(PKG, 'config', 'maps', f'{NAME}.yaml')
    with open(path, 'w') as f:
        f.write('\n'.join(L_) + '\n')
    for flat in ('_flat',):
        with open(os.path.join(PKG, 'config', 'maps', f'{NAME}{flat}.yaml'), 'w') as f:
            f.write('\n'.join(L_).replace('terrain:', 'terrain:               # 평지 world: 모든 z = 0\n  flat: true\n ', 1) + '\n')
    return path


def road_grade(T, pts):
    """길의 최대 경사 (%), 3 m 이상 구간 기준"""
    best = 0.0
    for a, b in zip(pts[:-1], pts[1:]):
        ln = float(np.hypot(*(b - a)))
        if ln > 3:
            best = max(best, abs(float(T.elev(*b) - T.elev(*a))) / ln * 100)
    return round(best, 1)


def write_json(L, T, zones, dock, bounds):
    def P2(a):
        return [[round(float(x), 1), round(float(y), 1)] for x, y in a]
    xs = np.arange(-640, 641, 16.0)
    ys = np.arange(-640, 641, 16.0)
    X, Y = np.meshgrid(xs, ys)
    E = T.elev(X, Y)
    out = {
        'name': NAME, 'title': '경북대학교 대구캠퍼스',
        'frame': {'origin_lat': LAT0, 'origin_lon': LON0, 'x': 'east_m', 'y': 'north_m', 'kx': KX, 'ky': KY},
        'bounds': [round(b, 1) for b in bounds],
        'elevation': {'xs0': -640, 'ys0': -640, 'step': 16, 'nx': len(xs), 'ny': len(ys),
                      'datum': round(T.datum, 2), 'elev': [[round(float(v), 2) for v in row] for row in E],
                      'source': 'SRTM 30 m (OpenTopoData) 가우시안 평활'},
        'contours': [{'level': lev, 'lines': [P2(ln) for ln in lines]} for lev, lines in contour_lines(T, L)],
        'zones_info': {str(k): {'name': v[0], 'color': v[1], 'building_color': v[2]} for k, v in ZONES.items()},
        'boundary': P2(L['boundary']),
        'buildings': [{'id': b['id'], 'number': b['number'], 'name': b['name'], 'zone': b['zone'],
                       'height': b['height'], 'c': r3(b['c']), 'poly': P2(b['poly']),
                       'base_z': round(float(T.z(b['poly'][:, 0], b['poly'][:, 1]).min()), 2)} for b in L['buildings']],
        'roads': [{'kind': r['kind'], 'pts': P2(simplify(r['pts'], 0.5)), 'grade_pct': road_grade(T, r['pts'])}
                  for r in L['roads']],
        'gates': [{'name': g['name'], 'named': g['named'], 'xy': r3(g['xy'])} for g in L['gates']],
        'water': [{'name': w['name'], 'poly': P2(w['poly'])} for w in L['water']],
        'green': [{'name': w['name'], 'poly': P2(w['poly'])} for w in L['green']],
        'pitch': [{'name': w['name'], 'kind': w['kind'], 'sport': w['sport'], 'poly': P2(w['poly'])} for w in L['pitch']],
        'car_parking': [{'poly': P2(w['poly'])} for w in L['car_parking']],
        'parking_zones': [{
            'id': z['id'], 'type': z['type'], 'site': z['site'], 'origin': z['origin'], 'poly': P2(z['poly']),
            'rack': ({'center': r3(z['rack']['center']), 'yaw': round(z['rack']['yaw'], 4),
                      'out': r3(z['rack']['out']), 'length': z['rack']['length'], 'depth': z['rack']['depth'],
                      'road_kind': z['rack']['road_kind'], 'access': r3(z['rack']['access'])} if z['rack'] else None),
            'slots': [{'id': f'{z["id"]}-{i + 1:02d}', 'xy': r3(s['xy']), 'yaw': round(s['yaw'], 4), 'role': s['role'],
                       'z': round(float(T.z(*s['xy'])), 2)} for i, s in enumerate(z['slots'])],
        } for z in zones],
        'robot_dock': {'body': r3(dock['body']), 'yaw': round(dock['yaw'], 4), 'tag': r3(dock['tag']),
                       'dock': r3(dock['dock']), 'near': dock['near']},
        'attribution': '지도 데이터 © OpenStreetMap contributors (ODbL), 고도 SRTM (NASA/USGS), 건물 번호·이름 경북대 캠퍼스 안내도',
    }
    path = os.path.join(PKG, 'maps', f'{NAME}.json')
    with open(path, 'w') as f:
        json.dump(out, f, ensure_ascii=False, separators=(',', ':'))
    return path, out


def korean_font(size, weight=500):
    """한글 폰트 찾기. 없으면: sudo apt install fonts-noto-cjk  (또는 ~/.local/share/fonts/NotoSansKR.ttf)"""
    import glob
    import subprocess
    cands = [os.path.expanduser('~/.local/share/fonts/NotoSansKR.ttf')]
    cands += sorted(glob.glob('/usr/share/fonts/**/NotoSansCJK*-Regular.ttc', recursive=True))
    cands += sorted(glob.glob('/usr/share/fonts/**/NanumGothic*.ttf', recursive=True))
    try:
        cands.append(subprocess.run(['fc-match', '-f', '%{file}', ':lang=ko'], capture_output=True, text=True).stdout)
    except OSError:
        pass
    for c in cands:
        if c and os.path.exists(c):
            f = ImageFont.truetype(c, size)
            try:
                f.set_variation_by_axes([weight])
            except Exception:  # noqa: BLE001 (가변 폰트가 아니면 그대로)
                pass
            return f
    print('  ! 한글 폰트가 없어 2D 지도 글자가 깨집니다: sudo apt install fonts-noto-cjk')
    return ImageFont.load_default()


def write_viewer(data):
    """웹 2D/3D 뷰어 (KNU 안심동행 3D 지도와 같은 야간 톤). 데이터는 HTML 안에 넣는다"""
    tpl = open(os.path.join(PKG, 'scripts', 'knu_campus_viewer_template.html')).read()
    js = json.dumps(data, ensure_ascii=False, separators=(',', ':')).replace('</', '<\\/')
    page = tpl.replace('/*DATA*/', js)
    path = os.path.join(PKG, 'maps', f'{NAME}_viewer.html')
    with open(path, 'w') as f:
        f.write('<!doctype html><html lang="ko"><head><meta charset="utf-8">'
                '<meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover"></head><body>\n')
        f.write(page)
        f.write('\n</body></html>\n')
    return path


def write_2d_map(L, T, zones, dock, bounds):
    """문서용 2D 지도 PNG: 바닥 + 건물(구역 색, 번호·이름) + 등고선 표고 + 주차/충전구역 + 범례"""
    x0, y0, x1, y1 = bounds
    W = 3600
    g = Grid(bounds, (x1 - x0) / W)
    img = draw_base(L, T, g, scale_px=1.0 / g.res, zones=None, shade=True)
    d = ImageDraw.Draw(img, 'RGBA')
    f_s, f_m, f_l, f_t = (korean_font(15, 500), korean_font(20, 600), korean_font(30, 700), korean_font(46, 800))

    def pp(poly):
        return [tuple(v) for v in g.px(np.asarray(poly))]

    # 경사 5 % 이상인 길 (계단 제외) -> 주황 표시
    for r in L['roads']:
        if r['kind'] == 'steps':
            continue
        for a, b in zip(r['pts'][:-1], r['pts'][1:]):
            ln = float(np.hypot(*(b - a)))
            if ln > 3 and abs(float(T.elev(*b) - T.elev(*a))) / ln > 0.05:
                d.line(pp([a, b]), fill=(235, 120, 20, 255), width=5)

    # 경사 (평활 지형 기울기 > 6 %) 빗금 대신 반투명 음영
    for b in L['buildings']:
        col = hex_rgb(ZONES[b['zone']][1])
        d.polygon(pp(b['poly']), fill=tuple(int(c * 255) for c in col) + (255,), outline=(70, 60, 70, 255))
    d.line(pp(np.vstack([L['boundary'], L['boundary'][:1]])), fill=TEX_COL['boundary'], width=5)
    for z in zones:
        charging = z['type'] == 'charging'
        col = (20, 165, 145) if charging else (70, 120, 220)
        d.polygon(pp(z['poly']), fill=col + (55,), outline=col + (255,))
        d.line(pp(np.vstack([z['poly'], z['poly'][:1]])), fill=col + (255,), width=4 if charging else 3)
        if z['rack']:
            c = g.px(z['rack']['center'])
            r = 13 if charging else 9
            d.ellipse([c[0] - r, c[1] - r, c[0] + r, c[1] + r], fill=(255, 255, 255, 255), outline=col + (255,), width=4)
            if charging:      # 번개 모양
                bolt = [(2, -9), (-5, 1), (0, 1), (-2, 9), (5, -1), (0, -1)]
                d.polygon([(c[0] + bx * 1.1, c[1] + by * 1.1) for bx, by in bolt], fill=col + (255,))
    # 등고선 표고 라벨
    for lev, lines in contour_lines(T, L):
        if lev % 10 != 0:
            continue
        for ln in lines:
            if len(ln) < 20:
                continue
            p = ln[len(ln) // 2]
            if not point_in_poly(p[0], p[1], L['boundary']):
                continue
            q = g.px(p)
            d.text(tuple(q), f'{lev:.0f} m', font=f_s, fill=(120, 85, 50, 255), anchor='mm',
                   stroke_width=3, stroke_fill=(240, 235, 220, 255))
    # 건물 번호 + 이름
    for b in L['buildings']:
        if not b['number'] and not b['name']:
            continue
        q = g.px(b['c'])
        if b['number']:
            d.text((q[0], q[1] - 10), str(b['number']), font=f_m, fill=(25, 25, 35, 255), anchor='mm',
                   stroke_width=3, stroke_fill=(255, 255, 255, 230))
            d.text((q[0], q[1] + 11), b['name'], font=f_s, fill=(25, 25, 35, 255), anchor='mm',
                   stroke_width=3, stroke_fill=(255, 255, 255, 230))
        else:
            d.text(tuple(q), b['name'], font=f_s, fill=(60, 60, 70, 255), anchor='mm',
                   stroke_width=2, stroke_fill=(255, 255, 255, 200))
    for w in L['water']:
        if w['name']:
            d.text(tuple(g.px(poly_centroid(w['poly']))), w['name'], font=f_m, fill=(20, 70, 140, 255), anchor='mm',
                   stroke_width=3, stroke_fill=(255, 255, 255, 220))
    for p in L['pitch']:
        if p['name']:
            d.text(tuple(g.px(poly_centroid(p['poly']))), p['name'].split(' (')[0], font=f_m, fill=(255, 255, 255, 255),
                   anchor='mm', stroke_width=3, stroke_fill=(60, 90, 50, 255))
    for p in L['pitch']:
        if p['sport'] == 'tennis':
            d.text(tuple(g.px(poly_centroid(p['poly']))), '테니스장', font=f_s, fill=(255, 255, 255, 255),
                   anchor='mm', stroke_width=2, stroke_fill=(60, 90, 50, 255))
    for gr in L['green']:
        if gr['name'] in ('월파원', 'KNU 센트럴파크', '장미원', '마블링파크'):
            d.text(tuple(g.px(poly_centroid(gr['poly']))), gr['name'], font=f_s, fill=(40, 90, 30, 255), anchor='mm',
                   stroke_width=2, stroke_fill=(255, 255, 255, 200))
    for gt in L['gates']:
        q = g.px(gt['xy'])
        r = 11 if gt['named'] else 6
        d.ellipse([q[0] - r, q[1] - r, q[0] + r, q[1] + r], fill=(200, 40, 50, 255), outline=(255, 255, 255, 255), width=2)
        if gt['named']:
            d.text((q[0], q[1] - 26), gt['name'], font=f_l, fill=(170, 30, 40, 255), anchor='mm',
                   stroke_width=4, stroke_fill=(255, 255, 255, 240))
    # 충전구역 이름
    for z in zones:
        if z['type'] == 'charging' and z['rack']:
            q = g.px(z['rack']['center'])
            d.text((q[0], q[1] + 26), f"{z['id']} {z['site']}", font=f_m, fill=(10, 120, 105, 255), anchor='mm',
                   stroke_width=4, stroke_fill=(255, 255, 255, 245))
    q = g.px(dock['body'])
    d.rectangle([q[0] - 11, q[1] - 11, q[0] + 11, q[1] + 11], fill=(30, 70, 200, 255), outline=(255, 255, 255, 255), width=3)
    d.text((q[0], q[1] + 26), '로봇 도크', font=f_m, fill=(30, 60, 170, 255), anchor='mm', stroke_width=3,
           stroke_fill=(255, 255, 255, 240))
    # 제목·범례·축척·방위
    d.rectangle([30, 30, 1010, 365], fill=(255, 255, 255, 235), outline=(60, 60, 60, 255), width=2)
    d.text((55, 50), '경북대학교 대구캠퍼스 — 킥보드 주차·충전 구역', font=f_t, fill=(30, 30, 40, 255))
    yy = 120
    items = [((70, 120, 220), f'일반 주차구역 {sum(z["type"] == "general" for z in zones)}곳 (앱 주차 가능 구역, 정상 {PARK_SLOTS}칸 + 고장 {BROKEN_SLOTS}칸)'),
             ((20, 165, 145), f'충전 주차구역 {sum(z["type"] == "charging" for z in zones)}곳 (충전 스탠드 {CHARGE_SLOTS}칸)'),
             (TEX_COL['contour'], f'등고선 2 m 간격 (굵은 선 10 m), 표고 {T.Z.min():.0f}~{T.Z.max():.0f} m'),
             (TEX_COL['service'], '차량 도로 (6 m)'), (TEX_COL['footway'], '보행로 (3 m)'), (TEX_COL['steps'], '계단 (진입 금지)'),
             ((235, 120, 20), '경사 5 % 이상인 길 (언덕·경사로)')]
    for col, txt in items:
        d.rectangle([60, yy, 100, yy + 24], fill=tuple(col) + (255,), outline=(60, 60, 60, 255))
        d.text((115, yy - 2), txt, font=f_m, fill=(30, 30, 40, 255))
        yy += 33
    xx = 1060
    for k, (nm, col, _) in ZONES.items():
        d.rectangle([xx, 40, xx + 34, 70], fill=tuple(int(c * 255) for c in hex_rgb(col)) + (255,), outline=(60, 60, 60, 255))
        d.text((xx + 44, 38), f'{k}구역 {nm}', font=f_m, fill=(30, 30, 40, 255))
        xx += 300
    sb = 200 / g.res
    d.rectangle([60, img.height - 90, 60 + sb, img.height - 78], fill=(40, 40, 40, 255))
    d.text((60, img.height - 120), '200 m', font=f_m, fill=(30, 30, 40, 255))
    nx, ny = img.width - 110, 140
    d.polygon([(nx, ny - 60), (nx - 22, ny + 10), (nx, ny), (nx + 22, ny + 10)], fill=(200, 40, 50, 255))
    d.text((nx, ny + 30), 'N', font=f_l, fill=(30, 30, 40, 255), anchor='mm')
    d.text((img.width - 30, img.height - 30), '지도 © OpenStreetMap contributors · 고도 SRTM · 건물 번호 경북대 안내도',
           font=f_s, fill=(80, 80, 80, 255), anchor='rd')
    path = os.path.join(PKG, 'maps', f'{NAME}_2d.png')
    img.save(path, optimize=True)
    return path


def main():
    print('[1] 원본 읽기')
    L = load_layers()
    b = L['boundary']
    bounds = (math.floor(b[:, 0].min() / 10) * 10 - 40, math.floor(b[:, 1].min() / 10) * 10 - 40,
              math.ceil(b[:, 0].max() / 10) * 10 + 40, math.ceil(b[:, 1].max() / 10) * 10 + 40)
    print(f'    건물 {len(L["buildings"])} (안내도 번호 {sum(1 for x in L["buildings"] if x["number"])}), 길 {len(L["roads"])}, '
          f'출입문 {len(L["gates"])}, 주차 가능 구역 {len(L["image_zones"])}, 범위 {bounds}')
    T = Terrain()
    print('[2] 격자·여유 거리')
    g = Grid(bounds, RES_GRID)
    M = build_masks(L, g)
    M['roads_wide'] = M['roads'].astype(np.uint8)
    M['pitch_u8'] = M['pitch'].astype(np.uint8)
    M['steps_u8'] = M['steps'].astype(np.uint8)
    M['cars_u8'] = M['cars'].astype(np.uint8)
    free = (~M['solid']).astype(np.uint8)
    clear = cv2.distanceTransform(free, cv2.DIST_L2, 5) * RES_GRID
    print('[3] 로봇 도크 · 주차/충전 구역 · 거치대')
    dock = place_dock(L, g, M, clear)
    T.datum = float(T.elev(*dock['dock']))
    zones = build_zones(L, g, M, clear)
    trim_buildings(L)
    for z in zones:
        if z['type'] == 'charging':
            print(f"    {z['id']} {z['site']:<16} {'기존 구역' if z['origin'] == 'image' else '새 구역'}  "
                  f"거치대 ({z['rack']['center'][0]:.0f}, {z['rack']['center'][1]:.0f}) {z['rack']['road_kind']}")
    ng = sum(1 for z in zones if z['type'] == 'general')
    print(f'    일반 {ng}곳 (거치대 {sum(1 for z in zones if z["type"] == "general" and z["rack"])}), '
          f'도크 {dock["near"]} 옆 ({dock["dock"][0]:.1f}, {dock["dock"][1]:.1f}), datum {T.datum:.1f} m')
    print('[4] Gazebo 모델·world')
    for flat in (False, True):
        write_terrain_model(L, T, M, zones, bounds, flat)
        write_world(L, T, zones, dock, flat)
    write_texture(L, T, zones, bounds)
    print('[5] 주행 지도 · 맵 설정 · JSON · 2D 지도')
    gn = Grid(bounds, RES_NAV)
    Mn = build_masks(L, gn)
    write_nav_maps(Mn, gn, zones, dock)
    write_config(zones, dock, T, bounds)
    _, data = write_json(L, T, zones, dock, bounds)
    write_viewer(data)
    p = write_2d_map(L, T, zones, dock, bounds)
    zs = T.z(np.array([z['rack']['center'][0] for z in zones if z['rack']]),
             np.array([z['rack']['center'][1] for z in zones if z['rack']]))
    print(f'    거치대 높이 범위 z {zs.min():.1f} ~ {zs.max():.1f} m (도크 = 0)')
    print('완료:', p)


if __name__ == '__main__':
    main()
