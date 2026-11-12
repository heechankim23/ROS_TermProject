"""[시뮬레이션 전용, sim:=gz] 킥보드 상태 목록을 Gazebo GUI 패널에 보여 준다.

gz 토픽 /echo 로 1초마다 gz.msgs.StringMsg_V 를 보낸다. Gazebo GUI 의 "Kickboard status" 패널
(TopicEcho, config/gz_gui.config)에서 Echo 를 누르면 보인다 (토픽 기본값이 /echo).

  robot DROPPING | bay 3/4 | battery 87%
  #1 FALLEN | 80% | COLLECTING -> PARK        바닥: 상태 | 배터리 | 대기열 상태 -> 하역 예정지
  #2 IN BAY | 70% -> PARK                     적재함에 실림
  #3 AT CHARGER | 10%                         충전소에 맡김
  #4 OK | 75% (parked)                        주차구역에 세워 둠
상태 이름·색은 gz_utils.scooter_state / DECK_COLORS 와 같다.
"""
import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from gz.msgs10.stringmsg_v_pb2 import StringMsg_V
from gz.transport13 import Node as GzNode

from kickboard_interfaces.msg import QueueStatus, RobotStatus, ScooterState, ScooterTruthArray

from .common import spin_single
from .gz_utils import scooter_state

DEST = {'parking': 'PARK', 'charger': 'CHARGER'}


def sort_key(sid):
    return (0, int(sid), '') if sid.isdigit() else (1, 0, sid)


class StatusPanel(Node):
    def __init__(self):
        super().__init__('status_panel')
        self.declare_parameter('topic', '/echo')
        self.declare_parameter('tilt_threshold_deg', 45.0)
        self.declare_parameter('battery_threshold_pct', 15.0)
        self.states = {}         # id -> ScooterState
        self.on_ground = set()   # 바닥에 있는(실리지 않은) 킥보드
        self.queue = {}          # id -> 대기열 상태
        self.robot = None        # RobotStatus
        self.cargo = {}          # id -> 하역 예정지 (실은 순서)
        self.charger = []        # 충전소에 맡긴 id
        self.parked = set()      # 주차구역에 세워 둔 id
        self.create_subscription(ScooterState, '/scooter_states', lambda m: self.states.__setitem__(m.id, m), 20)
        self.create_subscription(ScooterTruthArray, '/sim/scooter_truth',
                                 lambda m: setattr(self, 'on_ground', set(m.ids)), 10)
        self.create_subscription(QueueStatus, '/queue_status',
                                 lambda m: setattr(self, 'queue', {i.id: i.status for i in m.items}), 10)
        self.create_subscription(RobotStatus, '/robot_status', lambda m: setattr(self, 'robot', m), 10)
        self.create_subscription(String, '/mission_events', self.on_event, 50)
        # gz 는 보내기만 한다 (구독을 걸지 않음 — arm_loader.py 설명 참고)
        self.gz = GzNode()
        self.pub = self.gz.advertise(self.get_parameter('topic').value, StringMsg_V)
        self.create_timer(1.0, self.publish)

    def on_event(self, m):
        try:
            e = json.loads(m.data)
        except ValueError:
            return
        ev, sid = e.get('ev'), str(e.get('id', ''))
        if ev == 'collected':
            self.cargo[sid] = e.get('dest', 'charger')
            self.parked.discard(sid)
        elif ev == 'dropped':
            self.cargo.pop(sid, None)
            self.parked.add(sid)
        elif ev == 'unloaded_at_charger':
            self.cargo.pop(sid, None)
            if sid not in self.charger:
                self.charger.append(sid)
        elif ev == 'charged_out':                 # 충전 끝 -> 충전 자리에서 치움
            if sid in self.charger:
                self.charger.remove(sid)

    def line(self, sid):
        s = self.states.get(sid)
        batt = '?' if s is None else f'{s.battery_pct:.0f}%'
        if sid in self.cargo:
            return f'#{sid} IN BAY | {batt} -> {DEST.get(self.cargo[sid], self.cargo[sid])}'
        if sid in self.charger:
            return f'#{sid} AT CHARGER | {batt}'
        if s is None or sid not in self.on_ground:
            return None
        bt = self.get_parameter('battery_threshold_pct').value
        st = scooter_state(s.tilt_deg, s.in_parking_zone, s.battery_pct,
                           self.get_parameter('tilt_threshold_deg').value, bt)
        text = f'#{sid} {st} | {batt}'
        qs = self.queue.get(sid, '')
        if self.robot is not None and self.robot.current_target_id == sid:
            qs = 'COLLECTING'
        if qs in ('WAITING', 'COLLECTING', 'FAILED', 'NEEDS_HUMAN', 'IN_PROGRESS'):
            text += f' | {qs}'
        if st != 'OK':
            text += ' -> ' + ('CHARGER' if s.battery_pct < bt else 'PARK')
        elif sid in self.parked:
            text += ' (parked)'
        return text

    def publish(self):
        msg = StringMsg_V()
        if self.robot is not None:
            r = self.robot
            msg.data.append(f'robot {r.state} | bay {r.load_count}/{r.capacity} | battery {r.battery_pct:.0f}%')
        ids = set(self.states) | set(self.cargo) | set(self.charger)
        for sid in sorted(ids, key=sort_key):
            ln = self.line(sid)
            if ln:
                msg.data.append(ln)
        self.pub.publish(msg)


def main():
    rclpy.init()
    spin_single(StatusPanel())
