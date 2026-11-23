"""[역할 ④] 운영 현황 모니터: 로봇이 지금 어디로 · 왜 · 어떤 킥보드를 위해 가는지, 수거한 킥보드를 어느 구역(일반/충전/고장 칸)으로
옮기는지, 정렬이 끝났는지를 한곳에 모아 보여 주고 기록한다.

입력  /mission_events (모든 노드의 이벤트 JSON), /robot_status, TF(map->base_footprint), /queue_status,
      /scooter_states (펌웨어 상태), /sim/scooter_truth (바닥에 있는 킥보드 실제 위치), /zone_occupancy (거치대 센서)
출력  /ops_status        std_msgs/String JSON (2 Hz) — 현재 임무 카드 + 집계
      /ops_markers       visualization_msgs/MarkerArray (1 Hz) — 킥보드 상태 색, 로봇 경로, 목적지 화살표·이유, 구역 점유
      /ops_map_markers   캠퍼스 외곽·건물·구역 (한 번, latched)
      log_dir/<시나리오>_<시각>.jsonl — 리플레이용 기록 (scripts/build_ops_replay.py 로 웹 리플레이 생성)
"""
import json
import math
import os
import time

import rclpy
import yaml
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray

from kickboard_interfaces.msg import AdminAlert, QueueStatus, RobotStatus, ScooterState, ScooterTruthArray

from .common import PoseTracker, now_sec, spin_single
from .motion import yaw_from_quat

REASON_KO = {1: '넘어짐', 2: '불법 주차·방치', 3: '배터리 부족', 4: '고장', 5: '통행 방해 신고', 6: '방치 신고', 7: '정렬 불량'}
KIND_KO = {'normal': '일반 주차구역 슬롯', 'broken': '고장 분류 칸 (반대 방향)', 'charge': '충전 주차구역 스탠드',
           'charge_wait': '일반 슬롯 (충전 대기)'}
PHASE_KO = {'IDLE': '대기', 'SELECTING': '다음 할 일 고르는 중', 'NAVIGATING': '킥보드로 이동', 'VERIFYING': '카메라로 킥보드 확인',
            'LOADING': '로봇팔로 적재', 'FAILED': '실패 처리', 'RETURNING': '도크로 복귀', 'DOCKING': '도킹',
            'CHARGING': '충전', 'DROPPING': '주차구역으로 옮겨 정렬'}
STATE_RGB = {'OK': (0.15, 0.65, 0.3), 'FALLEN': (0.9, 0.15, 0.15), 'OUT_OF_ZONE': (1.0, 0.5, 0.0),
             'MISALIGNED': (0.95, 0.8, 0.1), 'LOW_BATTERY': (0.6, 0.25, 0.9), 'BROKEN': (0.15, 0.15, 0.18),
             'CHARGING': (0.1, 0.75, 0.65), 'IN_BAY': (0.3, 0.45, 0.95)}


def scooter_code(st, in_slot_charge=False):
    """펌웨어 상태 -> 표시 상태"""
    if st is None:
        return 'OK'
    if st.tilt_deg > 45:
        return 'FALLEN'
    if st.fault:
        return 'BROKEN'
    if not st.in_parking_zone:
        return 'MISALIGNED' if st.in_slot_area else 'OUT_OF_ZONE'
    if in_slot_charge:
        return 'CHARGING'
    if st.battery_pct < 20:
        return 'LOW_BATTERY'
    return 'OK'


class OpsMonitor(Node):
    def __init__(self):
        super().__init__('ops_monitor')
        self.declare_parameter('map_config_file', '')
        self.declare_parameter('scenario_file', '')
        self.declare_parameter('log_dir', os.path.expanduser('~/.ros/kickboard_ops'))
        self.declare_parameter('snapshot_sec', 2.0)
        self.pose = PoseTracker(self)
        self.robot = None
        self.queue = {}             # id -> QueueItem
        self.states = {}            # id -> ScooterState
        self.truth = {}             # id -> (x, y, yaw) 바닥에 있는 킥보드
        self.occ = {}               # 슬롯 -> 킥보드
        self.cargo = {}             # 실은 킥보드 -> 종류 (normal/broken/charge)
        self.plan = {}              # 하역 배정 id -> (슬롯, 종류)
        self.done = []              # 정렬 완료 [{id, slot, zone, kind, t}]
        self.failed = []
        self.why = ''
        self.stop = None            # 지금 하역하러 가는 구역
        self.trail = []
        self.initial = {}           # 처음 본 상태 (결과 비교용)
        self.zones, self.slot_xy, self.dock = {}, {}, None
        mc_path = self.get_parameter('map_config_file').value
        if mc_path:
            with open(mc_path) as f:
                mc = yaml.safe_load(f)
            for typ, key in (('general', 'parking_zones'), ('charging', 'charging_zones')):
                for z in mc.get(key, []) or []:
                    if z.get('rack'):
                        self.zones[z['name']] = {'type': typ, 'site': z.get('site', ''), 'x': z['rack']['x'],
                                                 'y': z['rack']['y'], 'n': len(z.get('slots', [])), 'poly': z['polygon']}
                    for s in z.get('slots', []):
                        self.slot_xy[s['id']] = (s['x'], s['y'], s['role'])
            sp = mc.get('robot_spawn', {})
            self.dock = (sp.get('x', 0.0), sp.get('y', 0.0))
        self.scenario = ''
        sc_path = self.get_parameter('scenario_file').value
        if sc_path:
            with open(sc_path) as f:
                sc = yaml.safe_load(f) or {}
            self.scenario = sc.get('name', os.path.basename(sc_path))
            self.sc_id = sc.get('id', os.path.splitext(os.path.basename(sc_path))[0])
        else:
            self.sc_id = 'run'
        os.makedirs(self.get_parameter('log_dir').value, exist_ok=True)
        self.log_path = os.path.join(self.get_parameter('log_dir').value,
                                     f'{self.sc_id}_{time.strftime("%m%d_%H%M%S")}.jsonl')
        self.log = open(self.log_path, 'w')
        self.write({'type': 'meta', 'scenario': self.scenario, 'id': self.sc_id,
                    'map_config': mc_path, 'zones': self.zones, 'slots': self.slot_xy, 'dock': self.dock})

        self.create_subscription(String, '/mission_events', self.on_event, 100)
        self.create_subscription(RobotStatus, '/robot_status', lambda m: setattr(self, 'robot', m), 10)
        self.create_subscription(QueueStatus, '/queue_status', lambda m: setattr(self, 'queue', {i.id: i for i in m.items}), 10)
        self.create_subscription(ScooterState, '/scooter_states', self.on_state, 100)
        self.create_subscription(ScooterTruthArray, '/sim/scooter_truth', self.on_truth, 10)
        self.create_subscription(String, '/zone_occupancy', self.on_occ, 10)
        self.create_subscription(AdminAlert, '/admin_alerts', lambda m: self.write(
            {'type': 'event', 't': round(now_sec(self), 2),
             'e': {'ev': 'alert', 'id': m.id, 'reason': m.reason, 'message': m.message}}), 20)
        self.status_pub = self.create_publisher(String, '/ops_status', 10)
        self.mk_pub = self.create_publisher(MarkerArray, '/ops_markers', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub = self.create_publisher(MarkerArray, '/ops_map_markers', latched)
        self.publish_map()
        self.last_snap = -1e9
        self.create_timer(0.5, self.tick)
        self.create_timer(1.0, self.publish_markers)
        self.get_logger().info(f'운영 기록: {self.log_path}')

    # ---------- 입력 ----------
    def write(self, d):
        self.log.write(json.dumps(d, ensure_ascii=False) + '\n')

    def on_state(self, m):
        self.states[m.id] = m
        if m.id not in self.initial:
            self.initial[m.id] = scooter_code(m)

    def on_truth(self, m):
        self.truth = {sid: (p.position.x, p.position.y, yaw_from_quat(p.orientation)) for sid, p in zip(m.ids, m.poses)}

    def on_occ(self, m):
        try:
            self.occ = json.loads(m.data).get('occupied', {})
        except ValueError:
            pass

    def on_event(self, m):
        try:
            e = json.loads(m.data)
        except ValueError:
            return
        ev, sid = e.get('ev'), str(e.get('id', ''))
        if ev == 'state':
            self.why = e.get('why', '')
            if e.get('to') != 'DROPPING' and e.get('frm') == 'DROPPING':
                self.stop = None
        elif ev == 'collected':
            self.cargo[sid] = e.get('dest', 'normal')
        elif ev == 'drop_plan':
            self.plan = {k: tuple(v) for k, v in (e.get('assignments') or {}).items()}
        elif ev == 'slot_assigned':
            self.stop = e.get('zone')
        elif ev == 'slot_filled':
            kind = self.cargo.pop(sid, e.get('kind', ''))
            self.plan.pop(sid, None)
            self.done.append({'id': sid, 'slot': e.get('slot'), 'zone': e.get('zone'), 'kind': kind or e.get('kind'),
                              't': round(e.get('t', now_sec(self)), 1), 'from': self.initial.get(sid, '')})
        elif ev == 'dropped' and not self.slot_xy:          # 슬롯이 없는 맵 (기존 맵)
            self.cargo.pop(sid, None)
            self.done.append({'id': sid, 'slot': '', 'zone': '', 'kind': 'normal', 't': round(e.get('t', 0), 1),
                              'from': self.initial.get(sid, '')})
        elif ev in ('collect_failed',) or (ev == 'item_status' and e.get('status') in ('FAILED', 'NEEDS_HUMAN', 'CANCELLED')):
            self.failed.append({'id': sid, 'why': e.get('reason') or e.get('note') or e.get('status'), 't': round(e.get('t', 0), 1),
                                'status': e.get('status', 'FAILED')})
        self.write({'type': 'event', 't': round(e.get('t', now_sec(self)), 2), 'e': e})

    # ---------- 현재 임무 ----------
    def mission(self):
        r = self.robot
        st = r.state if r else 'IDLE'
        card = {'phase': st, 'phase_ko': PHASE_KO.get(st, st), 'why': self.why, 'target': None, 'dest': None,
                'dest_xy': None, 'load': r.load_count if r else 0, 'capacity': r.capacity if r else 0,
                'battery': round(r.battery_pct, 1) if r else None,
                'cargo': [{'id': k, 'kind': v, 'kind_ko': KIND_KO.get(v, v), 'slot': (self.plan.get(k) or ('',))[0]}
                          for k, v in self.cargo.items()]}
        if r and r.current_target_id and st in ('NAVIGATING', 'VERIFYING', 'LOADING'):
            sid = r.current_target_id
            q = self.queue.get(sid)
            xy = self.truth.get(sid)
            card['target'] = {'id': sid, 'reason': REASON_KO.get(q.reason, '?') if q else '?',
                              'xy': [round(xy[0], 1), round(xy[1], 1)] if xy else ([round(q.x, 1), round(q.y, 1)] if q else None),
                              'state': scooter_code(self.states.get(sid))}
            card['dest_xy'] = card['target']['xy']
            card['dest'] = f'킥보드 #{sid} ({card["target"]["reason"]}) 수거'
        elif st == 'DROPPING' and self.stop in self.zones:
            z = self.zones[self.stop]
            kinds = sorted({self.plan.get(k, ('', ''))[1] or v for k, v in self.cargo.items()
                            if (self.plan.get(k) or ('',))[0].startswith(self.stop + '-')})
            card['dest'] = (f'{self.stop} {z["site"]} — ' + ('충전 주차구역' if z['type'] == 'charging' else '일반 주차구역')
                            + (f' ({", ".join(KIND_KO.get(k, k) for k in kinds)})' if kinds else ''))
            card['dest_zone'] = self.stop
            card['dest_type'] = z['type']
            card['dest_xy'] = [z['x'], z['y']]
        elif st in ('RETURNING', 'DOCKING', 'CHARGING') and self.dock:
            card['dest'] = '로봇 도크 (본관 옆) — 충전'
            card['dest_xy'] = list(self.dock)
        return card

    def summary(self):
        counts = {}
        for sid, st in self.states.items():
            slot = next((k for k, v in self.occ.items() if v == sid), None)
            charge = slot is not None and self.slot_xy.get(slot, (0, 0, ''))[2] == 'charge'
            c = 'IN_BAY' if sid in self.cargo else scooter_code(st, charge)
            if sid not in self.truth and sid not in self.cargo:
                continue
            counts[c] = counts.get(c, 0) + 1
        waiting = sum(1 for q in self.queue.values() if q.status == 'WAITING')
        active = sum(1 for q in self.queue.values() if q.status in ('WAITING', 'IN_PROGRESS'))
        return {'counts': counts, 'waiting': waiting, 'active': active, 'aligned': len(self.done),
                'failed': len(self.failed), 'done': self.done[-12:], 'failed_list': self.failed[-6:]}

    def tick(self):
        now = now_sec(self)
        p = self.pose.get()
        if p is not None and (not self.trail or math.dist(self.trail[-1], p[:2]) > 1.0):
            self.trail.append((round(p[0], 1), round(p[1], 1)))
            self.trail = self.trail[-600:]
        card, summ = self.mission(), self.summary()
        status = {'t': round(now, 1), 'scenario': self.scenario, 'robot': [round(v, 2) for v in p] if p else None,
                  'mission': card, 'summary': summ}
        self.status_pub.publish(String(data=json.dumps(status, ensure_ascii=False)))
        if now - self.last_snap >= self.get_parameter('snapshot_sec').value:
            self.last_snap = now
            sc = []
            for sid, st in self.states.items():
                if sid in self.cargo:
                    sc.append([sid, None, None, 'IN_BAY', round(st.battery_pct)])
                    continue
                xy = self.truth.get(sid)
                if xy is None:
                    continue
                slot = next((k for k, v in self.occ.items() if v == sid), None)
                charge = slot is not None and self.slot_xy.get(slot, (0, 0, ''))[2] == 'charge'
                sc.append([sid, round(xy[0], 2), round(xy[1], 2), scooter_code(st, charge), round(st.battery_pct),
                           slot or '', round(math.degrees(xy[2]))])
            self.write({'type': 'snap', 't': round(now, 1), 'robot': status['robot'], 'mission': card,
                        'summary': {k: summ[k] for k in ('counts', 'waiting', 'active', 'aligned', 'failed')},
                        'scooters': sc, 'occ': self.occ})
            self.log.flush()

    # ---------- RViz ----------
    def mk(self, ns, mid, typ, rgb, scale, a=1.0):
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns, m.id, m.type, m.action = ns, mid, typ, Marker.ADD
        m.scale.x, m.scale.y, m.scale.z = (float(v) for v in scale)
        m.color = ColorRGBA(r=float(rgb[0]), g=float(rgb[1]), b=float(rgb[2]), a=float(a))
        m.pose.orientation.w = 1.0
        return m

    def publish_map(self):
        arr = MarkerArray()
        for k, (zid, z) in enumerate(self.zones.items()):
            rgb = (0.1, 0.75, 0.65) if z['type'] == 'charging' else (0.12, 0.3, 0.85)
            m = self.mk('zones', k, Marker.LINE_STRIP, rgb, (0.6, 0, 0), 0.9)
            m.points = [Point(x=float(x), y=float(y), z=0.05) for x, y in z['poly'] + z['poly'][:1]]
            arr.markers.append(m)
            t = self.mk('zone_names', k, Marker.TEXT_VIEW_FACING, rgb, (0, 0, 5.0))
            t.pose.position.x, t.pose.position.y, t.pose.position.z = float(z['x']), float(z['y']), 6.0
            t.text = zid
            arr.markers.append(t)
        self.map_pub.publish(arr)

    def publish_markers(self):
        arr = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)
        pts = self.mk('scooters', 0, Marker.SPHERE_LIST, (1, 1, 1), (1.6, 1.6, 1.6))
        for sid, (x, y, _) in self.truth.items():
            slot = next((k for k, v in self.occ.items() if v == sid), None)
            charge = slot is not None and self.slot_xy.get(slot, (0, 0, ''))[2] == 'charge'
            rgb = STATE_RGB[scooter_code(self.states.get(sid), charge)]
            pts.points.append(Point(x=float(x), y=float(y), z=0.8))
            pts.colors.append(ColorRGBA(r=float(rgb[0]), g=float(rgb[1]), b=float(rgb[2]), a=1.0))
        arr.markers.append(pts)
        if len(self.trail) > 1:
            tr = self.mk('trail', 0, Marker.LINE_STRIP, (0.2, 0.35, 0.95), (0.5, 0, 0), 0.7)
            tr.points = [Point(x=float(x), y=float(y), z=0.3) for x, y in self.trail]
            arr.markers.append(tr)
        p = self.pose.get()
        card = self.mission()
        if p is not None:
            rb = self.mk('robot', 0, Marker.CUBE, (0.2, 0.35, 0.95), (3.0, 2.0, 2.0))
            rb.pose.position.x, rb.pose.position.y, rb.pose.position.z = float(p[0]), float(p[1]), 1.0
            arr.markers.append(rb)
            if card['dest_xy']:
                ar = self.mk('dest', 0, Marker.ARROW, (1.0, 0.35, 0.1), (0.6, 1.4, 1.6))
                ar.points = [Point(x=float(p[0]), y=float(p[1]), z=2.0), Point(x=float(card['dest_xy'][0]), y=float(card['dest_xy'][1]), z=2.0)]
                arr.markers.append(ar)
            txt = self.mk('mission', 0, Marker.TEXT_VIEW_FACING, (0.05, 0.05, 0.1), (0, 0, 4.0))
            txt.pose.position.x, txt.pose.position.y, txt.pose.position.z = float(p[0]), float(p[1]), 9.0
            txt.text = f'{card["phase_ko"]}' + (f' → {card["dest"]}' if card['dest'] else '') + f' [{card["load"]}/{card["capacity"]}]'
            arr.markers.append(txt)
        for k, d in enumerate(self.done[-40:]):                   # 정렬 완료 표시
            xy = self.slot_xy.get(d['slot'])
            if xy:
                ok = self.mk('aligned', k, Marker.CYLINDER, (0.1, 0.8, 0.3), (1.2, 1.2, 0.2), 0.8)
                ok.pose.position.x, ok.pose.position.y, ok.pose.position.z = float(xy[0]), float(xy[1]), 0.1
                arr.markers.append(ok)
        self.mk_pub.publish(arr)

    def destroy_node(self):
        try:
            self.write({'type': 'end', 't': round(now_sec(self), 1), 'done': self.done, 'failed': self.failed})
            self.log.close()
        except Exception:  # noqa: BLE001
            pass
        super().destroy_node()


def main():
    rclpy.init()
    spin_single(OpsMonitor())
