"""Opt-in mission, queue, fleet and dock nodes; legacy classes are unchanged."""
import json
import math
import random

import rclpy
import yaml
from std_msgs.msg import String
from kickboard_interfaces.msg import RobotStatus
from kickboard_interfaces.srv import CancelRequest
from .common import now_sec, spin_single
from .mission_manager import MissionManager, SELECTING, FAILED, CHARGING
from .queue_manager import QueueManager
from .scooter_fleet_sim import ScooterFleetSim
from .dock_controller import DockController
from .extended_policy import admission
from . import queue_logic as ql


def config(node):
    node.declare_parameter('extended_file', '')
    with open(node.get_parameter('extended_file').value, encoding='utf-8') as f:
        return yaml.safe_load(f)


class ExtendedMission(MissionManager):
    def __init__(self):
        super().__init__()
        self.ext = config(self)
        self.specs = {str(s['id']): s for s in self.ext['scooters']}
        self.weights = {sid: float(s.get('weight_kg', 18)) for sid, s in self.specs.items()}
        self.weight_limit = float(self.ext.get('extended', {}).get('weight_limit_kg', 75))
        self.cancel_target = None
        self.create_subscription(String, '/extended/cancel', self.on_cancel, 10, callback_group=self.cb)

    def start_collect(self, sid, target_pose):
        decision = admission(self.specs.get(sid, {}), [c[0] for c in self.cargo], self.weights, self.weight_limit)
        if decision == 'NEEDS_HUMAN':
            self.report(sid, 'NEEDS_HUMAN', 'damaged_or_individually_overweight')
            self.alert(sid, 'manual_collection', '파손 또는 개별 중량 초과: 자동 적재 금지')
            self.ev.emit('admission_rejected', id=sid)
            self.set_state(SELECTING)
            return
        if decision == 'UNLOAD_FIRST':
            self.report(sid, 'REQUEUE', 'weight_limit: unload before collection')
            self.ev.emit('weight_limit', id=sid, load_kg=sum(self.weights.get(c[0], 18) for c in self.cargo))
            if self.cargo.for_dest('parking') and self.start_drop('(중량 제한)'):
                return
            self.start_dock('(중량 제한: 우선 하역)')
            return
        super().start_collect(sid, target_pose)

    def on_cancel(self, m):
        if m.data == self.target and self.state in ('NAVIGATING', 'VERIFYING'):
            self.cancel_target = m.data
            self._cancel_active()

    def _cancel_active(self):
        if self.cancel_target and self.collect_handle:
            self.collect_handle.cancel_goal_async()

    def on_collect_accepted(self, f):
        super().on_collect_accepted(f)
        self._cancel_active()

    def on_collect_done(self, f):
        wrapped = f.result()
        if self.cancel_target and wrapped is not None and not wrapped.result.success:
            sid = self.cancel_target
            self.cancel_target = None
            self.target, self.collect_handle = None, None
            self.report(sid, 'CANCELLED', 'transit cancellation acknowledged')
            self.ev.emit('transit_cancelled', id=sid)
            self.set_state(SELECTING)
            return
        self.cancel_target = None
        super().on_collect_done(f)

    def on_dock_done(self, f):
        wrapped = f.result()
        if wrapped is None or not wrapped.result.success:
            self.pending = False
            self.dock_fail += 1
            self.ev.emit('dock_retry', attempt=self.dock_fail)
            if self.dock_fail >= self.p('max_dock_retries'):
                self.at_dock = False
                self.alert('station', 'dock_failed', '재시도 소진: 정지, 충전·하역 금지')
                self.set_state('NEEDS_HUMAN', '(도킹 실패)')
            else:
                self.start_dock(f'(도킹 재시도 {self.dock_fail})')
            return
        super().on_dock_done(f)


class ExtendedQueue(QueueManager):
    def __init__(self):
        super().__init__()
        self.robot_phase, self.robot_target = '', ''
        self.cancel_pub = self.create_publisher(String, '/extended/cancel', 10)
        self.cancel_pending = set()
        self.create_subscription(RobotStatus, '/robot_status', self.robot_state, 10)

    def robot_state(self, m):
        self.robot_phase, self.robot_target = m.state, m.current_target_id

    def on_cancel(self, req, res):
        it = self.items.get(req.scooter_id)
        if it and it.status == ql.IN_PROGRESS:
            if self.robot_target != req.scooter_id or self.robot_phase not in ('NAVIGATING', 'VERIFYING'):
                res.success, res.message = False, '정렬·적재 중 취소 금지 또는 상태 동기화 대기'
            else:
                self.cancel_pending.add(req.scooter_id)
                self.cancel_pub.publish(String(data=req.scooter_id))
                res.success, res.message = True, '취소 접수: 액션 종료 후 CANCELLED 확정'
            return res
        return super().on_cancel(req, res)

    def publish_status(self):
        for sid in list(getattr(self, 'cancel_pending', ())):
            it = self.items.get(sid)
            if it and it.status == ql.IN_PROGRESS:
                self.cancel_pub.publish(String(data=sid))
            else:
                self.cancel_pending.discard(sid)
        super().publish_status()


class ExtendedFleet(ScooterFleetSim):
    def __init__(self):
        super().__init__()
        self.ext = config(self)
        self.specs = {str(s['id']): s for s in self.ext['scooters']}
        self.cancel_cli = self.create_client(CancelRequest, '/cancel_request')
        self.triggers = list(self.ext.get('triggers', []))
        self.cancelled = set()
        self.cleared = set()
        self.create_subscription(RobotStatus, '/robot_status', self.status_trigger, 10)
        self.sigma_base = self.sigma

    def status_trigger(self, msg):
        if self.t0 is None or now_sec(self) < self.t0:
            return
        for e in self.triggers[:]:
            if msg.state != e['state'] or (e.get('id') is not None and str(e['id']) != msg.current_target_id):
                continue
            self.triggers.remove(e)
            event = dict(e['event'], t=now_sec(self) - self.t0)
            self.apply_event(event)

    def apply_event(self, e):
        if e['type'] == 'cancel':
            sid = str(e['id'])
            if not self.cancel_cli.service_is_ready():
                self.events.append(dict(e, t=now_sec(self) - self.t0 + 1))
                self.events.sort(key=lambda v: v['t'])
                return
            f = self.cancel_cli.call_async(CancelRequest.Request(scooter_id=sid))
            def done(f):
                r = f.result()
                self.ev.emit('cancel_response', id=sid, accepted=bool(r and r.success))
                if r and r.success:
                    self.cancelled.add(sid)
            f.add_done_callback(done)
            return
        if e['type'] == 'gps_noise':
            self.sigma = float(e['sigma'])
            self.ev.emit('gps_noise', sigma=self.sigma)
            return
        super().apply_event(e)

    def send_request(self, s, lat, lon, reason):
        if s.id in self.cancelled:
            return True
        order = self.ext.get('extended', {}).get('cluster_order', [])
        if s.id in order:
            # Keep all models visible to lidar, but admit only the outermost uncleared request.
            before = order[:order.index(s.id)]
            if any(sid not in self.cleared for sid in before):
                return False
        return super().send_request(s, lat, lon, reason)

    def on_loaded(self, msg):
        self.cleared.add(msg.data)
        super().on_loaded(msg)

    def colors_for(self, s):
        if getattr(self, 'specs', {}).get(s.id, {}).get('damaged'):
            return ('0.12 0.12 0.12', '0.8 0.05 0.05')
        return super().colors_for(s)


class ExtendedDock(DockController):
    """Inject a bounded ALIGN result failure; real alignment still runs each attempt."""
    def __init__(self):
        super().__init__()
        self.ext = config(self)
        self.failures = int(self.ext.get('extended', {}).get('dock_failures', 0))

    def done(self, gh, ok):
        if ok and self.failures > 0:
            self.failures -= 1
            self.get_logger().warning('FAULT INJECTION: alignment confirmation rejected')
            ok = False
        return super().done(gh, ok)


def run(cls):
    rclpy.init()
    spin_single(cls())


def mission_main(): run(ExtendedMission)
def queue_main(): run(ExtendedQueue)
def fleet_main(): run(ExtendedFleet)
def dock_main(): run(ExtendedDock)
