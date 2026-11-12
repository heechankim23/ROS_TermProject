"""F1: 킥보드 펌웨어 흉내.

시나리오 YAML 의 킥보드를 Gazebo 에 배치하고, 이벤트(넘어짐/배터리/이동...)를 시간에 맞춰 재현한다.
GPS(노이즈 σ) · 기울기 · 배터리를 /scooter_states 로 내보내고, 요청 조건이 생기면 RequestCollection 호출.
"""
import math
import random

import rclpy
import yaml
from geometry_msgs.msg import Pose
from rclpy.node import Node
from std_msgs.msg import String

from kickboard_interfaces.msg import ScooterState, ScooterTruthArray
from kickboard_interfaces.srv import RequestCollection

from .common import EventLog, now_sec
from .geo import GeoConverter
from .gz_utils import DECK_COLORS, STEM_NORMAL, GzWorld, scooter_colors, scooter_state
from .motion import quat_from_yaw
from .queue_logic import FALLEN, LOW_BATTERY, OUT_OF_ZONE


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
        self.collected = False
        self.last_reason = 0
        self.color = None          # Gazebo 에 적용된 (발판, 기둥) 색

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
        self.declare_parameter('battery_threshold_pct', 15.0)
        self.declare_parameter('remove_on_load', True)
        self.declare_parameter('seed', 0)

        random.seed(self.get_parameter('seed').value)
        with open(self.get_parameter('map_config_file').value) as f:
            mc = yaml.safe_load(f)
        with open(self.get_parameter('scenario_file').value) as f:
            sc = yaml.safe_load(f)

        g = mc['gps_origin']
        self.geo = GeoConverter(g['lat'], g['lon'])
        self.zones = [z['polygon'] for z in mc.get('parking_zones', [])]
        self.sigma = float(sc.get('gps_noise_sigma', self.get_parameter('gps_noise_sigma').value))
        self.tilt_th = self.get_parameter('tilt_threshold_deg').value
        self.batt_th = self.get_parameter('battery_threshold_pct').value
        self.scooters = {s.id: s for s in (Scooter(d) for d in sc.get('scooters', []))}
        self.events = sorted(sc.get('events', []), key=lambda e: float(e['t']))
        self.gz = GzWorld(self.get_parameter('world_name').value, self.get_logger(),
                          self.get_parameter('gz_enabled').value)

        self.ev = EventLog(self)
        self.state_pub = self.create_publisher(ScooterState, '/scooter_states', 20)
        self.truth_pub = self.create_publisher(ScooterTruthArray, '/sim/scooter_truth', 10)
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
            self.t0 = now
        t = now - self.t0

        for s in self.scooters.values():
            if not s.spawned and t >= s.spawn_at:
                s.spawned = True
                if s.has_model:
                    s.color = self.colors_for(s)
                    self.gz.spawn(s.name, s.x, s.y, s.yaw, self._roll(s), s.color)
                self.ev.emit('scooter_spawned', id=s.id, x=s.x, y=s.y, has_model=s.has_model)

        while self.events and float(self.events[0]['t']) <= t:
            self.apply_event(self.events.pop(0))

        for s in self.scooters.values():
            if s.spawned and not s.collected:
                self.report(s)
                self.update_color(s)

    def apply_event(self, e):
        if e['type'] == 'obstacle':             # 시나리오 7: 경로에 장애물 투입
            self.gz.spawn_box(e.get('name', 'obstacle'), float(e['x']), float(e['y']),
                              tuple(e.get('size', (0.5, 0.5, 0.5))))
            self.get_logger().info(f'[t={float(e["t"]):.0f}s] 장애물 투입 ({e["x"]}, {e["y"]})')
            self.ev.emit('obstacle', x=e['x'], y=e['y'])
            return
        s = self.scooters.get(str(e['id']))
        if s is None or s.collected:
            return
        kind = e['type']
        if kind == 'fall':
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
        if s.has_model and s.spawned and not self.update_color(s):
            self.gz.set_pose(s.name, s.x, s.y, s.yaw, self._roll(s))
        self.get_logger().info(f'[t={float(e["t"]):.0f}s] 킥보드 {s.id}: {kind}')
        self.ev.emit('scooter_event', id=s.id, type=kind)

    def colors_for(self, s):
        state = scooter_state(s.tilt, self.in_zone(s), s.battery, self.tilt_th, self.batt_th)
        return scooter_colors(state, s.battery < self.batt_th)

    def update_color(self, s):
        """상태가 바뀌면 Gazebo 모델을 새 색으로 다시 만든다 (발판 = 상태, 기둥·핸들 = 배터리). 바꿨으면 True"""
        if not s.has_model or not s.spawned or s.collected:
            return False
        c = self.colors_for(s)
        if c == s.color:
            return False
        s.color = c
        self.gz.respawn(s.name, s.x, s.y, s.yaw, self._roll(s), c)
        return True

    @staticmethod
    def _roll(s):
        return math.radians(s.tilt) if s.tilt > 1.0 else 0.0

    # ---------- 펌웨어 보고 ----------
    def in_zone(self, s):
        return any(point_in_poly(s.x, s.y, z) for z in self.zones)

    def report(self, s):
        lat, lon = self.geo.to_latlon(s.x + random.gauss(0, self.sigma),
                                      s.y + random.gauss(0, self.sigma))
        in_zone = self.in_zone(s)
        msg = ScooterState(id=s.id, lat=lat, lon=lon, tilt_deg=float(s.tilt),
                           battery_pct=float(s.battery), in_parking_zone=in_zone)
        msg.stamp = self.get_clock().now().to_msg()
        self.state_pub.publish(msg)

        # 요청 조건: 넘어짐 > 주차구역 밖 > 배터리 부족 (가장 심각한 사유 하나)
        reason = 0
        if s.tilt > self.tilt_th:
            reason = FALLEN
        elif not in_zone:
            reason = OUT_OF_ZONE
        elif s.battery < self.batt_th:
            reason = LOW_BATTERY
        if reason and reason != s.last_reason:
            if not self.send_request(s, lat, lon, reason):
                return                      # 다음 주기에 다시 시도
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
            s.color = (DECK_COLORS['OK'], STEM_NORMAL)   # arm_loader 가 기본(정상) 색으로 다시 만듦
            # 로봇팔(arm_loader)이 없으면 모델도 여기서 다시 만든다 (있으면 arm_loader 가 이미 만듦)
            if self.get_parameter('remove_on_load').value and s.has_model:
                s.color = self.colors_for(s)
                self.gz.spawn(s.name, s.x, s.y, s.yaw, colors=s.color)
            self.get_logger().info(f'킥보드 {sid}: 주차구역에 세워짐 ({s.x:.2f}, {s.y:.2f}), '
                                   f'구역 안={self.in_zone(s)}')
            self.ev.emit('scooter_dropped', id=sid, x=s.x, y=s.y, in_zone=self.in_zone(s))

    def publish_truth(self):
        m = ScooterTruthArray()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        for s in self.scooters.values():
            if s.spawned and s.has_model and not s.collected:
                p = Pose()
                p.position.x, p.position.y = s.x, s.y
                q = quat_from_yaw(s.yaw)
                p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = q
                m.ids.append(s.id)
                m.poses.append(p)
        self.truth_pub.publish(m)


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
