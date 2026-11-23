"""F1: 킥보드 펌웨어 흉내.

시나리오 YAML 의 킥보드를 Gazebo 에 배치하고, 이벤트(넘어짐/배터리/이동...)를 시간에 맞춰 재현한다.
GPS(노이즈 σ) · 기울기 · 배터리를 /scooter_states 로 내보내고, 요청 조건이 생기면 RequestCollection 호출.

이벤트 종류 (시나리오 YAML events):
  fall | stand | battery(pct) | move(x, y)          킥보드 상태 변화
  rent                                              누가 빌려 감: 보고가 끊기고 실물이 사라짐
  return(x, y, [yaw_deg, tilt_deg, battery_pct])    반납: 새 위치에 다시 나타남
  obstacle(name, x, y, size, [yaw_deg])             고정 장애물 투입 (불법 주차 차량, 공사 펜스, 적치물)
  remove_obstacle(name)                             장애물 치움
  pedestrian(name, path, [speed, size, loop])       움직이는 장애물 (보행자·자전거): path 를 따라 왕복
장애물·보행자는 /sim/obstacles (std_msgs/String, JSON) 로 내보내 sim_lidar 가 /scan 에 반영한다.

캠퍼스 맵 (맵 설정에 슬롯이 있을 때, config/scenarios_knu/ 시나리오):
  '주차구역 안' = 거치대 슬롯에 바르게 서 있음 (슬롯 중심 0.4 m, 방향 30° 안). 슬롯 밖이지만 거치대 6 m 안 = 정렬 불량.
  요청 사유: 넘어짐 > 고장(고장 분류 칸에 있으면 없음) > 구역 밖 / 정렬 불량 > 배터리 부족(충전 스탠드에 꽂혀 있으면 없음)
  킥보드 필드: fault(고장 코드) · silent(스스로 요청 안 함, 신고로만) · charging · false_report · slot
  충전 스탠드에 꽂힌 킥보드는 charge_rate_pct_per_s 로 충전된다 (고장 스탠드 제외).
  report(id, reason, gps_error_m)   사용자 신고 -> 신고 위치(실제 + 오차)로 요청 (OBSTRUCTION / REPORTED)
  fault(id, code, detect)           고장 발생. detect=on_inspect 면 로봇이 reveal_dist_m 안에 와야 보고에 드러남
  park(id, slot)                    사용자가 슬롯에 바르게 세움 -> 대기열 자동 취소
  gps_drift(x, y, radius, sigma)    그 반경 안 킥보드의 GPS 노이즈를 키움
  stand_fault · zone_closed · robot_fault · (시작 시 overrides)  -> /sim/campus_events (JSON, 미션 관리자가 받음)
  after_drop: N (이벤트 속성)        그 킥보드를 하역한 뒤 N 초에 적용
  거치대 센서: 슬롯마다 서 있는 킥보드 -> /zone_occupancy (JSON {슬롯: 킥보드}, 1 Hz)
"""
import json
import math
import random

import rclpy
import yaml
from geometry_msgs.msg import Pose
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

from kickboard_interfaces.msg import ScooterState, ScooterTruthArray
from kickboard_interfaces.srv import RequestCollection

from .common import EventLog, PoseTracker, now_sec
from .geo import GeoConverter
from .gz_utils import DECK_COLORS, STEM_NORMAL, GzWorld, scooter_colors, scooter_state
from .motion import quat_from_yaw
from .queue_logic import BROKEN, FALLEN, LOW_BATTERY, MISALIGNED, OBSTRUCTION, OUT_OF_ZONE, REPORTED

REPORT_REASONS = {'obstruction': OBSTRUCTION, 'reported': REPORTED, 'fallen': FALLEN, 'broken': BROKEN,
                  'illegal': OUT_OF_ZONE}
CAMPUS_EVENTS = ('stand_fault', 'zone_closed', 'robot_fault')


def point_in_poly(x, y, poly):
    inside = False
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        if (y1 > y) != (y2 > y) and x < (x2 - x1) * (y - y1) / (y2 - y1 + 1e-12) + x1:
            inside = not inside
    return inside


class Scooter:
    def __init__(self, d):
        self.id = str(d['id'])
        self.x, self.y = float(d['x']), float(d['y'])
        self.yaw = math.radians(float(d.get('yaw_deg', 0.0)))
        self.tilt = float(d.get('tilt_deg', 0.0))
        self.battery = float(d.get('battery_pct', 80.0))
        self.has_model = bool(d.get('model', True))        # False: 요청만 오고 실물 없음 (시나리오 6)
        self.spawn_at = float(d.get('spawn_at', 0.0))
        self.spawned = False
        self.placed = False        # Gazebo 모델을 미리 만들어 둠 (spawn_at 0)
        self.collected = False
        self.gone = False          # 누가 빌려 가서 사라짐 (rent 이벤트) -> 보고 없음
        self.last_reason = 0
        self.req_xy = None         # 마지막으로 요청을 보낸 실제 위치
        self.color = None          # Gazebo 에 적용된 (발판, 기둥) 색
        # 캠퍼스 맵
        self.fault = str(d.get('fault') or '')
        self.fault_hidden = False  # fault 이벤트 detect=on_inspect: 로봇이 가까이 와야 드러남
        self.silent = bool(d.get('silent', False))
        self.slot = None           # 지금 서 있는 슬롯 (dict) 또는 None

    @property
    def name(self):
        return f'scooter_{self.id}'


class ScooterFleetSim(Node):
    def __init__(self):
        super().__init__('scooter_fleet_sim')
        self.declare_parameter('scenario_file', '')
        self.declare_parameter('map_config_file', '')
        self.declare_parameter('world_name', 'kickboard_world')
        self.declare_parameter('gz_enabled', True)
        self.declare_parameter('gps_noise_sigma', 1.0)
        self.declare_parameter('tilt_threshold_deg', 45.0)
        self.declare_parameter('battery_threshold_pct', 20.0)
        self.declare_parameter('remove_on_load', True)
        # Gazebo 모델 생성은 대당 몇 초 걸린다. 처음부터 있는 킥보드(spawn_at 0)는 노드 시작 즉시 전부 만들어 두고,
        # 이 시간이 지난 뒤에야 보고·요청을 시작한다 -> 로봇이 출발하기 전에 킥보드가 이미 바닥에 놓여 있다
        self.declare_parameter('initial_hold_sec', -1.0)   # 음수 = 자동 (Gazebo 10초, kinematic 0초)
        self.declare_parameter('move_rerequest_m', 2.0)   # 요청 뒤 이만큼 옮겨지면 새 위치로 다시 요청 (병합)
        self.declare_parameter('seed', 0)
        self.declare_parameter('charge_rate_pct_per_s', 0.2)   # 캠퍼스: 충전 스탠드 충전 속도
        self.declare_parameter('reveal_dist_m', 4.0)           # 숨은 고장이 로봇 카메라에 드러나는 거리
        self.declare_parameter('terrain_json', '')             # 지형 world: 킥보드 높이 (maps/<맵>.json elevation)

        random.seed(self.get_parameter('seed').value)
        with open(self.get_parameter('map_config_file').value) as f:
            mc = yaml.safe_load(f)
        with open(self.get_parameter('scenario_file').value) as f:
            sc = yaml.safe_load(f)

        g = mc['gps_origin']
        self.geo = GeoConverter(g['lat'], g['lon'])
        self.zones = [z['polygon'] for z in mc.get('parking_zones', [])]
        # 캠퍼스 맵: 거치대 슬롯 (일반 normal·broken, 충전 charge)
        self.slots = []
        self.racks = []
        for z in mc.get('parking_zones', []) + mc.get('charging_zones', []):
            if z.get('rack'):
                self.racks.append((z['rack']['x'], z['rack']['y']))
            for sl in z.get('slots', []):
                self.slots.append(dict(sl, zone=z['name'], yaw=math.radians(sl['yaw_deg'])))
        self.campus = bool(self.slots)
        self.dead_slots = set()
        self.drift = []            # (x, y, radius, sigma)
        self.after_drop = {}       # 킥보드 id -> [이벤트]
        self.terrain = None
        if self.get_parameter('terrain_json').value:
            from .terrain import Terrain
            self.terrain = Terrain(self.get_parameter('terrain_json').value)
        self.sigma = float(sc.get('gps_noise_sigma', self.get_parameter('gps_noise_sigma').value))
        self.tilt_th = self.get_parameter('tilt_threshold_deg').value
        self.batt_th = self.get_parameter('battery_threshold_pct').value
        self.scooters = {s.id: s for s in (Scooter(d) for d in sc.get('scooters', []))}
        evs = []
        for e in sc.get('events', []):
            if 'after_drop' in e and 'id' in e:
                self.after_drop.setdefault(str(e['id']), []).append(e)
            else:
                evs.append(e)
        self.events = sorted(evs, key=lambda e: float(e['t']))
        self.overrides = dict(sc.get('overrides') or {})
        self.gz = GzWorld(self.get_parameter('world_name').value, self.get_logger(),
                          self.get_parameter('gz_enabled').value)
        self.gz.beacons = self.campus          # 처리가 필요한 킥보드 위 상태 색 신호등 (정렬되면 사라짐)

        self.ev = EventLog(self)
        self.state_pub = self.create_publisher(ScooterState, '/scooter_states', 20)
        self.truth_pub = self.create_publisher(ScooterTruthArray, '/sim/scooter_truth', 10)
        self.obst_pub = self.create_publisher(String, '/sim/obstacles', 10)
        latched = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.campus_pub = self.create_publisher(String, '/sim/campus_events', latched)
        self.occ_pub = self.create_publisher(String, '/zone_occupancy', 10)
        self.robot = PoseTracker(self) if self.campus else None
        self.obstacles = {}        # name -> {x, y, sx, sy, yaw, [path, speed, seg, dir]}
        self.obst_t = None
        self.obst_gz_tick = 0
        self.cli = self.create_client(RequestCollection, '/request_collection')
        self.create_subscription(String, '/scooter_loaded', self.on_loaded, 10)
        self.create_subscription(ScooterTruthArray, '/scooter_dropped', self.on_dropped, 10)

        self.t0 = None
        self.create_timer(1.0, self.tick)          # 펌웨어 보고 주기 1 Hz
        self.create_timer(0.2, self.publish_truth)
        self.get_logger().info(f'시나리오 "{sc.get("name", "?")}": 킥보드 {len(self.scooters)}대, '
                               f'이벤트 {len(self.events)}개')

    # ---------- 시나리오 진행 ----------
    def tick(self):
        now = now_sec(self)
        if now <= 0.0:          # use_sim_time 인데 /clock 이 아직 안 옴
            return
        if self.t0 is None:
            hold = self.get_parameter('initial_hold_sec').value
            if hold < 0:
                hold = 10.0 if self.gz.enabled else 0.0
            self.t0 = now + hold                 # 시나리오 시각 0 = 지금부터 hold 초 뒤
            n = 0
            for s in self.scooters.values():     # 처음부터 있는 킥보드는 지금 전부 배치
                if s.spawn_at <= 0.0 and s.has_model:
                    s.slot = self.slot_of(s)
                    s.color = self.colors_for(s)
                    self.gz.spawn(s.name, s.x, s.y, s.yaw, self._roll(s), s.color, z0=self.z(s))
                    s.placed = True
                    n += 1
            if self.campus:
                self.campus_event({'type': 'config', 'overrides': self.overrides})
            if hold > 0:
                self.get_logger().info(f'킥보드 {n}대를 먼저 배치했습니다. {hold:.0f}초 뒤 보고·요청 시작')
        t = now - self.t0
        if t < 0.0:
            return

        for s in self.scooters.values():
            if not s.spawned and t >= s.spawn_at:
                s.spawned = True
                if s.has_model and not s.placed:
                    s.slot = self.slot_of(s)
                    s.color = self.colors_for(s)
                    self.gz.spawn(s.name, s.x, s.y, s.yaw, self._roll(s), s.color, z0=self.z(s))
                self.ev.emit('scooter_spawned', id=s.id, x=s.x, y=s.y, has_model=s.has_model)

        while self.events and float(self.events[0]['t']) <= t:
            self.apply_event(self.events.pop(0))
        self.t_now = t

        if self.campus:
            self.campus_tick()
        for s in self.scooters.values():
            if s.spawned and not s.collected and not s.gone:
                self.report(s)
                self.update_color(s)

    def apply_event(self, e):
        kind = e['type']
        if kind in ('obstacle', 'pedestrian'):  # 고정 장애물 / 움직이는 장애물(보행자)
            name = e.get('name', kind)
            moving = kind == 'pedestrian'
            size = tuple(float(v) for v in e.get('size', (0.5, 0.5, 1.7) if moving else (0.5, 0.5, 0.5)))
            o = {'sx': size[0], 'sy': size[1], 'yaw': math.radians(float(e.get('yaw_deg', 0.0)))}
            if moving:
                o['path'] = [(float(p[0]), float(p[1])) for p in e['path']]
                o['speed'], o['loop'] = float(e.get('speed', 0.8)), bool(e.get('loop', True))
                o['seg'], o['dir'] = 0, 1
                o['x'], o['y'] = o['path'][0]
            else:
                o['x'], o['y'] = float(e['x']), float(e['y'])
            self.obstacles[name] = o
            self.gz.spawn_box(name, o['x'], o['y'], size, o['yaw'],
                              rgb='0.2 0.4 0.9' if moving else '0.8 0.1 0.1')
            self.get_logger().info(f'[t={float(e["t"]):.0f}s] {"보행자" if moving else "장애물"} {name} 투입 '
                                   f'({o["x"]}, {o["y"]})')
            self.ev.emit('obstacle', name=name, x=o['x'], y=o['y'], moving=moving)
            return
        if kind in CAMPUS_EVENTS:               # 미션 관리자가 처리 (스탠드 고장 · 구역 폐쇄 · 로봇 고장)
            if kind == 'stand_fault':
                self.dead_slots.add(e['slot'])
            self.campus_event(dict(e))
            self.get_logger().info(f'[t={float(e["t"]):.0f}s] {kind} {e.get("slot") or e.get("zone") or e.get("part", "")}')
            return
        if kind == 'gps_drift':
            self.drift.append((float(e['x']), float(e['y']), float(e['radius']), float(e['sigma'])))
            self.get_logger().info(f'[t={float(e["t"]):.0f}s] GPS 오차 지역 반경 {e["radius"]} m, σ {e["sigma"]} m')
            return
        if kind == 'remove_obstacle':
            name = e.get('name', 'obstacle')
            if self.obstacles.pop(name, None) is not None:
                self.gz.remove(name)
            self.get_logger().info(f'[t={float(e["t"]):.0f}s] 장애물 {name} 치움')
            self.ev.emit('obstacle_removed', name=name)
            return
        s = self.scooters.get(str(e['id']))
        if s is None or s.collected:
            return
        if kind == 'rent':                      # 누가 빌려 감: 보고가 끊기고 실물이 사라진다
            s.gone = True
            if s.has_model and s.spawned:
                self.gz.remove(s.name)
            self.get_logger().info(f'[t={float(e["t"]):.0f}s] 킥보드 {s.id}: 대여됨 (사라짐)')
            self.ev.emit('scooter_event', id=s.id, type=kind)
            return
        if kind == 'return':                    # 반납: 새 위치에 다시 나타남
            s.x, s.y = float(e.get('x', s.x)), float(e.get('y', s.y))
            s.yaw = math.radians(float(e.get('yaw_deg', math.degrees(s.yaw))))
            s.tilt = float(e.get('tilt_deg', 0.0))
            s.battery = float(e.get('battery_pct', s.battery))
            s.gone, s.last_reason = False, 0
            if s.has_model and s.spawned:
                s.color = self.colors_for(s)
                self.gz.spawn(s.name, s.x, s.y, s.yaw, self._roll(s), s.color)
            self.get_logger().info(f'[t={float(e["t"]):.0f}s] 킥보드 {s.id}: 반납됨 ({s.x}, {s.y})')
            self.ev.emit('scooter_event', id=s.id, type=kind)
            return
        if s.gone:
            return
        if kind == 'report':                    # 사용자 신고: 신고 위치 = 실제 + gps_error_m (방향 무작위)
            err = float(e.get('gps_error_m', 5.0))
            a = random.uniform(-math.pi, math.pi)
            lat, lon = self.geo.to_latlon(s.x + err * math.cos(a), s.y + err * math.sin(a))
            reason = REPORT_REASONS.get(e.get('reason', 'reported'), REPORTED)
            self.send_request(s, lat, lon, reason)
            self.get_logger().info(f'[t={float(e["t"]):.0f}s] 킥보드 {s.id}: 사용자 신고 {e.get("reason")} '
                                   f'"{e.get("note", "")}" (GPS 오차 {err:.0f} m)')
            self.ev.emit('report', id=s.id, reason=e.get('reason'), note=e.get('note', ''), gps_error_m=err)
            return
        if kind == 'fault':
            s.fault = str(e.get('code', 'unknown'))
            s.fault_hidden = e.get('detect') == 'on_inspect'
        elif kind == 'park':                    # 사용자가 슬롯에 바르게 세움
            sl = next((q for q in self.slots if q['id'] == e['slot']), None)
            if sl is None:
                self.get_logger().warn(f'park: 슬롯 {e["slot"]} 없음')
                return
            s.x, s.y, s.yaw, s.tilt = sl['x'], sl['y'], sl['yaw'], 0.0
        elif kind == 'fall':
            s.tilt = float(e.get('tilt_deg', 90.0))
        elif kind == 'stand':
            s.tilt = 0.0
        elif kind == 'battery':
            s.battery = float(e['pct'])
        elif kind == 'move':
            s.x, s.y = float(e['x']), float(e['y'])
            s.yaw = math.radians(float(e.get('yaw_deg', math.degrees(s.yaw))))
        else:
            self.get_logger().warn(f'알 수 없는 이벤트 {kind}')
            return
        # 상태(색)가 바뀌면 새 pose 로 다시 만들고, 아니면 pose 만 옮긴다
        s.slot = self.slot_of(s)
        if s.has_model and s.spawned and not self.update_color(s):
            self.gz.set_pose(s.name, s.x, s.y, s.yaw, self._roll(s))
        self.get_logger().info(f'[t={float(e["t"]):.0f}s] 킥보드 {s.id}: {kind}')
        self.ev.emit('scooter_event', id=s.id, type=kind)

    def colors_for(self, s):
        if not self.campus:
            state = scooter_state(s.tilt, self.in_zone(s), s.battery, self.tilt_th, self.batt_th)
            return scooter_colors(state, s.battery < self.batt_th)
        charging = self.charging(s)
        fault = s.fault if not s.fault_hidden else ''
        state = scooter_state(s.tilt, s.slot is not None, s.battery, self.tilt_th, self.batt_th, fault=fault,
                              misaligned=self.near_rack(s), charging=charging)
        return scooter_colors(state, s.battery < self.batt_th, broken=bool(fault), charging=charging)

    def update_color(self, s):
        """상태가 바뀌면 Gazebo 모델을 새 색으로 다시 만든다 (발판 = 상태, 기둥·핸들 = 배터리). 바꿨으면 True"""
        if not s.has_model or not s.spawned or s.collected or s.gone:
            return False
        c = self.colors_for(s)
        if c == s.color:
            return False
        s.color = c
        self.gz.respawn(s.name, s.x, s.y, s.yaw, self._roll(s), c, z0=self.z(s))
        return True

    @staticmethod
    def _roll(s):
        return math.radians(s.tilt) if s.tilt > 1.0 else 0.0

    # ---------- 펌웨어 보고 ----------
    def in_zone(self, s):
        if self.campus:
            return s.slot is not None
        return any(point_in_poly(s.x, s.y, z) for z in self.zones)

    # ---------- 캠퍼스 맵 ----------
    def z(self, s):
        return 0.0 if self.terrain is None else self.terrain.z(s.x, s.y)

    def slot_of(self, s):
        """바르게 서 있는 슬롯 (중심 0.4 m, 방향 30° 안) 또는 None"""
        if not self.campus or s.tilt > self.tilt_th:
            return None
        for sl in self.slots:
            if math.hypot(s.x - sl['x'], s.y - sl['y']) < 0.4:
                d = math.atan2(math.sin(s.yaw - sl['yaw']), math.cos(s.yaw - sl['yaw']))
                if abs(d) < math.radians(30):
                    return sl
        return None

    def near_rack(self, s):
        return s.slot is None and any(math.hypot(s.x - x, s.y - y) < 6.0 for x, y in self.racks)

    def charging(self, s):
        return s.slot is not None and s.slot['role'] == 'charge' and s.slot['id'] not in self.dead_slots \
            and not s.fault

    def campus_event(self, e):
        self.campus_pub.publish(String(data=json.dumps(e, ensure_ascii=False)))

    def campus_tick(self):
        """충전 · 숨은 고장 드러남 · 거치대 센서 (1 Hz)"""
        rate = self.get_parameter('charge_rate_pct_per_s').value
        robot = self.robot.get() if self.robot else None
        occ = {}
        for s in self.scooters.values():
            if not s.spawned or s.collected or s.gone:
                continue
            s.slot = self.slot_of(s)
            if s.slot is not None:
                occ[s.slot['id']] = s.id
            if self.charging(s) and s.battery < 100.0:
                s.battery = min(100.0, s.battery + rate)
            if s.fault_hidden and robot is not None and \
                    math.hypot(robot[0] - s.x, robot[1] - s.y) < self.get_parameter('reveal_dist_m').value:
                s.fault_hidden = False
                self.get_logger().info(f'킥보드 {s.id}: 로봇 카메라 확인 -> 고장 ({s.fault})')
                self.ev.emit('fault_revealed', id=s.id, fault=s.fault)
        self.occ_pub.publish(String(data=json.dumps({'t': round(getattr(self, 't_now', 0.0), 1), 'occupied': occ,
                                                     'dead': sorted(self.dead_slots)})))

    def sigma_at(self, s):
        sig = self.sigma
        for x, y, r, sg in self.drift:
            if math.hypot(s.x - x, s.y - y) <= r:
                sig = max(sig, sg)
        return sig

    def report(self, s):
        sig = self.sigma_at(s)
        lat, lon = self.geo.to_latlon(s.x + random.gauss(0, sig), s.y + random.gauss(0, sig))
        in_zone = self.in_zone(s)
        fault = s.fault if not s.fault_hidden else ''
        area = self.campus and self.near_rack(s)
        msg = ScooterState(id=s.id, lat=lat, lon=lon, tilt_deg=float(s.tilt),
                           battery_pct=float(s.battery), in_parking_zone=in_zone, fault=fault, in_slot_area=area)
        msg.stamp = self.get_clock().now().to_msg()
        self.state_pub.publish(msg)

        # 요청 조건 (가장 심각한 사유 하나): 넘어짐 > [고장] > 주차구역 밖 / [정렬 불량] > 배터리 부족
        reason = 0
        role = s.slot['role'] if s.slot is not None else None
        if s.tilt > self.tilt_th:
            reason = FALLEN
        elif fault and role != 'broken':
            reason = BROKEN
        elif fault:
            reason = 0                          # 고장 분류 칸에 반대 방향으로 세워짐 -> 분류 끝
        elif not in_zone:
            reason = MISALIGNED if area else OUT_OF_ZONE
        elif s.battery < self.batt_th and role != 'charge':
            reason = LOW_BATTERY
        if s.silent:
            reason = 0                          # 통신·판정이 안 되는 킥보드: 사용자 신고로만 요청이 들어온다
        # 같은 사유라도 요청 뒤에 멀리 옮겨졌으면(누가 끌고 감) 새 위치로 다시 요청 -> 대기열에서 병합·위치 갱신
        moved = (s.req_xy is not None and math.hypot(s.x - s.req_xy[0], s.y - s.req_xy[1])
                 > self.get_parameter('move_rerequest_m').value)
        if reason and (reason != s.last_reason or moved):
            if not self.send_request(s, lat, lon, reason):
                return                      # 다음 주기에 다시 시도
            s.req_xy = (s.x, s.y)
        s.last_reason = reason

    def send_request(self, s, lat, lon, reason):
        if not self.cli.service_is_ready():
            self.get_logger().warn('request_gateway 가 아직 없습니다. 다음 주기에 재시도')
            return False
        req = RequestCollection.Request(scooter_id=s.id, lat=lat, lon=lon, reason=reason)
        gx, gy = self.geo.to_map(lat, lon)
        self.ev.emit('request_sent', id=s.id, reason=reason, gps_x=gx, gps_y=gy,
                     true_x=s.x, true_y=s.y, has_model=s.has_model)
        fut = self.cli.call_async(req)
        fut.add_done_callback(lambda f, sid=s.id: self._on_resp(sid, f))
        return True

    def _on_resp(self, sid, fut):
        r = fut.result()
        if r is not None:
            self.get_logger().info(f'요청 {sid}: accepted={r.accepted} 순번={r.queue_position} {r.message}')
        if r is None or not r.accepted:
            s = self.scooters.get(sid)
            if s is not None and not s.collected:
                s.last_reason = 0            # 거절·무응답이면 다음 보고 주기에 다시 요청 (같은 ID 는 병합되므로 안전)

    # ---------- 적재 / 진실값 ----------
    def on_loaded(self, msg):
        s = self.scooters.get(msg.data)
        if s is None:
            return
        s.collected = True
        if self.get_parameter('remove_on_load').value and s.has_model:
            self.gz.remove(s.name)

    def on_dropped(self, msg):
        """주차구역에 세워 둔 킥보드: 새 위치·똑바로 선 상태로 다시 정상 킥보드가 된다 (배터리는 그대로)"""
        for sid, p in zip(msg.ids, msg.poses):
            s = self.scooters.get(sid)
            if s is None:
                continue
            s.x, s.y = p.position.x, p.position.y
            o = p.orientation
            s.yaw = math.atan2(2 * (o.w * o.z + o.x * o.y), 1 - 2 * (o.y * o.y + o.z * o.z))
            s.tilt, s.collected, s.last_reason = 0.0, False, 0
            s.slot = self.slot_of(s)
            for e in self.after_drop.pop(sid, []):      # 하역한 뒤 N 초에 일어나는 이벤트
                e = dict(e, t=getattr(self, 't_now', 0.0) + float(e['after_drop']))
                self.events.append(e)
                self.events.sort(key=lambda q: float(q['t']))
            s.color = (DECK_COLORS['OK'], STEM_NORMAL)   # arm_loader 가 기본(정상) 색으로 다시 만듦
            # 로봇팔(arm_loader)이 없으면 모델도 여기서 다시 만든다 (있으면 arm_loader 가 이미 만듦)
            if self.get_parameter('remove_on_load').value and s.has_model:
                s.color = self.colors_for(s)
                self.gz.spawn(s.name, s.x, s.y, s.yaw, colors=s.color, z0=self.z(s))
            slot = s.slot['id'] if s.slot is not None else ''
            self.get_logger().info(f'킥보드 {sid}: 주차구역에 세워짐 ({s.x:.2f}, {s.y:.2f}), '
                                   f'구역 안={self.in_zone(s)}' + (f', 슬롯 {slot}' if slot else ''))
            self.ev.emit('scooter_dropped', id=sid, x=s.x, y=s.y, in_zone=self.in_zone(s), slot=slot,
                         yaw_deg=round(math.degrees(s.yaw), 1))

    def publish_truth(self):
        m = ScooterTruthArray()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        for s in self.scooters.values():
            if s.spawned and s.has_model and not s.collected and not s.gone:
                p = Pose()
                p.position.x, p.position.y = s.x, s.y
                q = quat_from_yaw(s.yaw)
                p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = q
                m.ids.append(s.id)
                m.poses.append(p)
        self.truth_pub.publish(m)
        self.step_obstacles()

    def step_obstacles(self):
        """보행자를 path 따라 움직이고, 장애물 목록을 /sim/obstacles 로 내보낸다 (5 Hz)"""
        now = now_sec(self)
        dt = 0.0 if self.obst_t is None else min(1.0, max(0.0, now - self.obst_t))
        self.obst_t = now
        self.obst_gz_tick += 1
        for name, o in self.obstacles.items():
            path = o.get('path')
            if not path or len(path) < 2 or o['dir'] == 0:
                continue
            left = o['speed'] * dt
            while left > 1e-9 and o['dir'] != 0:
                nxt = o['seg'] + o['dir']
                if not 0 <= nxt < len(path):
                    if o['loop']:
                        o['dir'] = -o['dir']         # 끝에 닿으면 되돌아감
                        continue
                    o['dir'] = 0                     # 끝에서 멈춤
                    break
                tx, ty = path[nxt]
                d = math.hypot(tx - o['x'], ty - o['y'])
                if d <= left:
                    o['x'], o['y'], o['seg'] = tx, ty, nxt
                    left -= d
                else:
                    o['x'] += (tx - o['x']) / d * left
                    o['y'] += (ty - o['y']) / d * left
                    left = 0.0
            if self.obst_gz_tick % 3 == 0:           # Gazebo 모델은 gz service 호출이 무거워 ~1.7 Hz 로만 옮김
                self.gz.set_pose(name, o['x'], o['y'], o['yaw'])
        self.obst_pub.publish(String(data=json.dumps(
            [{'name': n, 'x': o['x'], 'y': o['y'], 'sx': o['sx'], 'sy': o['sy'], 'yaw': o['yaw']}
             for n, o in self.obstacles.items()])))


def main():
    rclpy.init()
    node = ScooterFleetSim()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
