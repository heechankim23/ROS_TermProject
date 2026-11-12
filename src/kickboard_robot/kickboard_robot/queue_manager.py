"""F4 / F12 / F13 / F15: 대기열과 우선순위.

- /queue/enqueue (내부)        : request_gateway 가 전달. 같은 ID 면 병합(사유만 갱신)
- /queue/select_next (내부)    : mission_manager 가 다음 대상을 요청 -> 점수 최고 항목 IN_PROGRESS
- /queue/update_status (내부)  : DONE / FAILED / NEEDS_HUMAN / REQUEUE
- /get_queue, /cancel_request, /set_config (P1)
- /queue_status (1 Hz), /queue_markers (RViz)
"""
import rclpy
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import Parameter as ParamMsg
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

from kickboard_interfaces.msg import QueueItem, QueueStatus, ScooterState
from kickboard_interfaces.srv import (CancelRequest, EnqueueRequest, GetQueue, SelectNextTarget,
                                      SetConfig, UpdateRequestStatus)

from . import queue_logic as ql
from .common import EventLog, PoseTracker, now_sec, spin_single


class QueueManager(Node):
    def __init__(self):
        super().__init__('queue_manager')
        self.declare_parameter('w_r', 10.0)
        self.declare_parameter('w_t', 1.0)
        self.declare_parameter('w_d', 5.0)
        self.declare_parameter('d_max', 15.0)            # 맵 대각선 길이 (기아 방지 상한 계산용)
        self.declare_parameter('auto_cancel', True)      # F13
        self.declare_parameter('tilt_threshold_deg', 45.0)
        self.declare_parameter('battery_threshold_pct', 15.0)

        self.items = {}            # id -> ql.Item
        self.robot_xy = (0.0, 0.0)
        self.pose = PoseTracker(self)
        self.ev = EventLog(self)
        cb = ReentrantCallbackGroup()

        self.create_service(EnqueueRequest, '/queue/enqueue', self.on_enqueue, callback_group=cb)
        self.create_service(SelectNextTarget, '/queue/select_next', self.on_select, callback_group=cb)
        self.create_service(UpdateRequestStatus, '/queue/update_status', self.on_update, callback_group=cb)
        self.create_service(GetQueue, '/get_queue', self.on_get, callback_group=cb)
        self.create_service(CancelRequest, '/cancel_request', self.on_cancel, callback_group=cb)
        self.create_service(SetConfig, '/set_config', self.on_set_config, callback_group=cb)
        self.mission_param_cli = self.create_client(SetParameters, '/mission_manager/set_parameters',
                                                    callback_group=cb)
        self.create_subscription(ScooterState, '/scooter_states', self.on_scooter_state, 50)
        self.status_pub = self.create_publisher(QueueStatus, '/queue_status', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/queue_markers', 10)
        self.create_timer(1.0, self.publish_status)

        w = self.weights()
        self.get_logger().info(
            f'가중치 w_r={w.w_r} w_t={w.w_t} w_d={w.w_d} -> 기아 방지 상한 '
            f't_max={ql.starvation_bound(w, self.get_parameter("d_max").value):.1f}s')

    def weights(self):
        g = self.get_parameter
        return ql.Weights(g('w_r').value, g('w_t').value, g('w_d').value)

    def _robot_xy(self):
        p = self.pose.get()
        if p:
            self.robot_xy = (p[0], p[1])
        return self.robot_xy

    # ---------- services ----------
    def on_enqueue(self, req, res):
        now = now_sec(self)
        it = self.items.get(req.scooter_id)
        if it is not None and it.status in ql.ACTIVE:
            it.reason = req.reason                       # 병합: 사유만 갱신
            res.merged = True
            if it.status == ql.WAITING:                  # 처리 중인 항목은 위치를 바꾸지 않음
                it.x, it.y, it.lat, it.lon = req.x, req.y, req.lat, req.lon
        else:
            self.items[req.scooter_id] = ql.Item(req.scooter_id, req.reason, req.x, req.y,
                                                 req.lat, req.lon, now)
            res.merged = False
            self.ev.emit('enqueued', id=req.scooter_id, reason=req.reason, x=req.x, y=req.y)
        res.accepted = True
        res.queue_position = ql.queue_position(self.items.values(), req.scooter_id, now,
                                               self._robot_xy(), self.weights())
        res.message = ql.REASON_NAMES.get(req.reason, '?')
        return res

    def on_select(self, req, res):
        now = now_sec(self)
        self.robot_xy = (req.robot_x, req.robot_y)
        order = ql.ranked(self.items.values(), now, self.robot_xy, self.weights())
        res.found = bool(order)
        if not order:
            return res
        it = order[0]
        it.status = ql.IN_PROGRESS
        res.scooter_id, res.reason = it.id, it.reason
        res.score = float(ql.score(it, now, self.robot_xy, self.weights()))
        res.target_pose = PoseStamped()
        res.target_pose.header.frame_id = 'map'
        res.target_pose.header.stamp = self.get_clock().now().to_msg()
        res.target_pose.pose.position.x, res.target_pose.pose.position.y = it.x, it.y
        res.target_pose.pose.orientation.w = 1.0
        self.get_logger().info(f'다음 대상 {it.id} (점수 {res.score:.1f}, 대기 {now - it.t_enqueued:.0f}s, '
                               f'후보 {len(order)}건)')
        return res

    def on_update(self, req, res):
        it = self.items.get(req.scooter_id)
        if it is None:
            res.success = False
            return res
        now = now_sec(self)
        if req.has_corrected_pose:
            it.x, it.y = req.corrected_pose.position.x, req.corrected_pose.position.y
        st = req.status
        if 'nav' in req.note:
            it.nav_failures += 1
        if st == 'REQUEUE':
            it.status = ql.WAITING                       # t_enqueued 유지 -> aging 계속
        elif st in (ql.DONE, ql.FAILED, ql.NEEDS_HUMAN, ql.CANCELLED):
            it.status = st
        else:
            res.success = False
            return res
        wait = now - it.t_enqueued
        self.ev.emit('item_status', id=it.id, status=it.status, wait_sec=wait, note=req.note)
        self.get_logger().info(f'{it.id}: {st} -> {it.status} (대기 {wait:.0f}s) {req.note}')
        res.success = True
        return res

    def on_get(self, req, res):
        res.items = self.snapshot()
        return res

    def on_cancel(self, req, res):
        it = self.items.get(req.scooter_id)
        if it is None or it.status not in ql.ACTIVE:
            res.success, res.message = False, '대기 중인 항목 없음'
        elif it.status == ql.IN_PROGRESS:
            res.success, res.message = False, '처리 중인 항목은 취소할 수 없음(비선점)'
        else:
            it.status = ql.CANCELLED
            res.success, res.message = True, '취소됨'
            self.ev.emit('item_status', id=it.id, status=it.status,
                         wait_sec=now_sec(self) - it.t_enqueued, note='manual cancel')
        return res

    def on_set_config(self, req, res):
        from rclpy.parameter import Parameter
        upd = [Parameter(n, value=float(getattr(req, n))) for n in ('w_r', 'w_t', 'w_d')
               if getattr(req, n) >= 0]
        if upd:
            self.set_parameters(upd)
        mission = []
        if req.capacity >= 0:
            mission.append(ParamMsg(name='capacity', value=ParameterValue(
                type=ParameterType.PARAMETER_INTEGER, integer_value=int(req.capacity))))
        if req.return_battery_pct >= 0:
            mission.append(ParamMsg(name='return_battery_pct', value=ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE, double_value=float(req.return_battery_pct))))
        if mission:
            if not self.mission_param_cli.service_is_ready():
                res.success, res.message = False, 'mission_manager 파라미터 서비스 없음'
                return res
            self.mission_param_cli.call_async(SetParameters.Request(parameters=mission))
        w = self.weights()
        res.success = True
        res.message = (f'w_r={w.w_r} w_t={w.w_t} w_d={w.w_d}, '
                       f't_max={ql.starvation_bound(w, self.get_parameter("d_max").value):.1f}s')
        return res

    # ---------- F13 자동 취소 ----------
    def on_scooter_state(self, msg):
        if not self.get_parameter('auto_cancel').value:
            return
        it = self.items.get(msg.id)
        if it is None or it.status != ql.WAITING:
            return
        ok = (msg.tilt_deg <= self.get_parameter('tilt_threshold_deg').value and msg.in_parking_zone
              and msg.battery_pct >= self.get_parameter('battery_threshold_pct').value)
        if ok:
            it.status = ql.CANCELLED
            self.get_logger().info(f'{it.id}: 정상 상태로 돌아와 자동 취소')
            self.ev.emit('item_status', id=it.id, status=it.status,
                         wait_sec=now_sec(self) - it.t_enqueued, note='auto cancel')

    # ---------- 발행 ----------
    def snapshot(self):
        now, rxy, w = now_sec(self), self._robot_xy(), self.weights()
        out = []
        for it in self.items.values():
            out.append(QueueItem(id=it.id, reason=it.reason, wait_sec=float(now - it.t_enqueued),
                                 score=float(ql.score(it, now, rxy, w)), status=it.status,
                                 x=it.x, y=it.y, nav_failures=it.nav_failures))
        out.sort(key=lambda q: (q.status not in ql.ACTIVE, -q.score))
        return out

    def publish_status(self):
        items = self.snapshot()
        m = QueueStatus(items=items)
        m.stamp = self.get_clock().now().to_msg()
        self.status_pub.publish(m)

        ma = MarkerArray()
        ma.markers.append(Marker(action=Marker.DELETEALL))
        active = [q for q in items if q.status in ql.ACTIVE]
        smax = max([q.score for q in active], default=1.0)
        smin = min([q.score for q in active], default=0.0)
        for k, q in enumerate(active):
            r = 1.0 if smax == smin else (q.score - smin) / (smax - smin)
            for typ, mid in ((Marker.CYLINDER, 2 * k), (Marker.TEXT_VIEW_FACING, 2 * k + 1)):
                mk = Marker(type=typ, id=mid, ns='queue', action=Marker.ADD)
                mk.header.frame_id = 'map'
                mk.pose.position.x, mk.pose.position.y = q.x, q.y
                mk.pose.orientation.w = 1.0
                if typ == Marker.CYLINDER:
                    mk.scale.x = mk.scale.y = 0.35
                    mk.scale.z = 0.05
                    mk.color.r, mk.color.g, mk.color.b, mk.color.a = r, 0.2, 1.0 - r, 0.8
                    if q.status == ql.IN_PROGRESS:
                        mk.color.r, mk.color.g, mk.color.b = 1.0, 1.0, 0.0
                else:
                    mk.pose.position.z = 0.6
                    mk.scale.z = 0.2
                    mk.color.r = mk.color.g = mk.color.b = mk.color.a = 1.0
                    mk.text = f'{q.id} {ql.REASON_NAMES.get(q.reason, "?")} S={q.score:.0f}'
                ma.markers.append(mk)
        self.marker_pub.publish(ma)


def main():
    rclpy.init()
    spin_single(QueueManager())


if __name__ == '__main__':
    main()
