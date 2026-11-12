"""F7 / F8: CollectScooter Action 서버.

NAVIGATING : nav_bridge 로 킥보드에서 approach_standoff(1.6 m) 떨어진 곳까지 이동
VERIFYING  : 태그(ID = 킥보드 ID) 검출 대기, 없으면 제자리 회전. 10초 안에 못 찾으면 실패
ALIGNING   : 검출한 태그로 킥보드 중심을 추정하고, nav_bridge(GoToPose, standoff=align_distance 1.3 m)로
             중심 둘레의 '빈 자리'까지 경로 주행해 중심을 바라본다. 자리 선택·경로 모두 코스트맵
             (정적 지도 + 라이다)을 써서 벽·다른 킥보드·이 킥보드의 넘어진 기둥에 부딪히지 않는다.
             로봇팔은 손목을 돌려 킥보드 방향에 맞추므로 어느 쪽에서 접근해도 된다.
LOADING    : 로봇팔(arm_loader, sim:=gz) 이 발판을 집어 적재함 칸에 싣는다 -> 적재 +1, /scooter_loaded 발행
             arm_loader 가 없으면(kinematic 모드 등) load_time_sec 정지로 대신한다
"""
import math

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from std_msgs.msg import String

from kickboard_interfaces.action import CollectScooter, GoToPose, LoadScooter
from kickboard_interfaces.msg import RobotStatus, TagDetectionArray

from .common import PoseTracker, future_wait, now_sec, sleep_sim, run_blocking, spin_single
from .motion import quat_from_yaw, yaw_from_quat


def tag_id_of(scooter_id):
    try:
        return int(scooter_id)
    except ValueError:
        return abs(hash(scooter_id)) % 90 + 1


class Collector(Node):
    def __init__(self):
        super().__init__('collector')
        self.declare_parameter('approach_standoff', 1.6)     # 로봇 중심 -> 킥보드 (로봇 길이 1.7 m)
        self.declare_parameter('verify_timeout_sec', 10.0)
        self.declare_parameter('search_angular_speed', 0.65)
        self.declare_parameter('align_distance', 1.35)        # 로봇 중심 -> 킥보드 중심 (팔이 발판에 닿는 거리)
        self.declare_parameter('align_pos_tol', 0.05)
        self.declare_parameter('align_yaw_tol_deg', 10.0)
        self.declare_parameter('align_timeout_sec', 90.0)     # 정렬 자리까지 경로 주행 포함
        self.declare_parameter('load_time_sec', 3.0)
        self.declare_parameter('scooter_half_depth', 0.2)    # 태그 면 -> 킥보드 중심 (근사)
        self.declare_parameter('detection_max_age_sec', 0.6)
        self.declare_parameter('arm_timeout_sec', 120.0)
        self.declare_parameter('early_detect_margin', 0.2)   # 이동 중 태그가 standoff+margin 안에서 보이면 도착 처리

        self.cb = ReentrantCallbackGroup()
        self.pose = PoseTracker(self)
        self.dets = {}            # tag id -> (t, TagDetection)
        self.load_count = 0
        self.create_subscription(TagDetectionArray, '/tag_detections', self.on_dets, 10, callback_group=self.cb)
        self.create_subscription(RobotStatus, '/robot_status',
                                 lambda m: setattr(self, 'load_count', m.load_count), 10, callback_group=self.cb)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.loaded_pub = self.create_publisher(String, '/scooter_loaded', 10)
        self.nav = ActionClient(self, GoToPose, '/nav_bridge/go_to_pose', callback_group=self.cb)
        self.loader = ActionClient(self, LoadScooter, '/arm_loader/load', callback_group=self.cb)
        ActionServer(self, CollectScooter, '/collect_scooter', self.execute,
                     goal_callback=lambda g: GoalResponse.ACCEPT,
                     cancel_callback=lambda g: CancelResponse.ACCEPT, callback_group=self.cb)

    def on_dets(self, msg):
        t = now_sec(self)
        for d in msg.detections:
            self.dets[d.id] = (t, d)

    def fresh(self, tag):
        e = self.dets.get(tag)
        if e and now_sec(self) - e[0] <= self.get_parameter('detection_max_age_sec').value:
            return e[1]
        return None

    def twist(self, v=0.0, w=0.0):
        t = Twist()
        t.linear.x, t.angular.z = v, w
        self.cmd_pub.publish(t)

    def fb(self, gh, phase, dist=0.0, load=None):
        gh.publish_feedback(CollectScooter.Feedback(
            phase=phase, distance_remaining=float(dist),
            load_count=self.load_count if load is None else load))

    def finish(self, gh, ok, reason='', corrected=None, ignore_cancel=False):
        self.twist()
        r = CollectScooter.Result(success=ok, failure_reason=reason)
        if corrected is not None:
            r.corrected_pose = corrected
        if gh.is_cancel_requested and not ignore_cancel:
            r.failure_reason = 'canceled'
            gh.canceled()
        elif ok:
            gh.succeed()
        else:
            gh.abort()
        return r

    # ------------------------------------------------------------------
    async def execute(self, gh):
        # 블로킹 루프는 작업 스레드에서, executor 는 단일 스레드로 다른 콜백 처리
        return await run_blocking(self._execute, gh)

    def _execute(self, gh):
        sid = gh.request.scooter_id
        tag = tag_id_of(sid)
        self.get_logger().info(f'수거 시작: 킥보드 {sid} (tag {tag})')

        # 1) NAVIGATING
        self.fb(gh, 'NAVIGATING')
        if not self.nav.wait_for_server(timeout_sec=5.0):
            return self.finish(gh, False, 'nav_bridge 없음')
        ng = GoToPose.Goal(target_pose=gh.request.target_pose,
                           standoff=float(self.get_parameter('approach_standoff').value))
        nh = future_wait(self, self.nav.send_goal_async(
            ng, feedback_callback=lambda m: self.fb(gh, 'NAVIGATING', m.feedback.distance_remaining)), 5.0)
        if nh is None or not nh.accepted:
            return self.finish(gh, False, 'nav goal rejected')
        rf = nh.get_result_async()
        early = self.get_parameter('approach_standoff').value + self.get_parameter('early_detect_margin').value
        arrived_early = False
        while rclpy.ok() and not rf.done():
            if gh.is_cancel_requested:
                nh.cancel_goal_async()
                return self.finish(gh, False)
            d = self.fresh(tag)
            if d is not None and d.distance <= early:
                # PRD: '도착 후 카메라가 태그를 검출하면 접근 완료' — GPS 오차로 목표가 킥보드에 너무 붙어 있어도 여기서 멈춤
                self.get_logger().info(f'이동 중 태그 {tag} 검출 ({d.distance:.2f} m) -> 접근 완료')
                nh.cancel_goal_async()
                arrived_early = True
                break
            sleep_sim(self, 0.1)
        if not arrived_early:
            if rf.result() is None:
                return self.finish(gh, False, 'nav_failed: no result')
            nres = rf.result().result
            if not nres.success:
                return self.finish(gh, False, f'nav_failed: {nres.message}')
        self.twist()

        # 2) VERIFYING — 회전 탐색
        self.fb(gh, 'VERIFYING')
        t0 = now_sec(self)
        w = self.get_parameter('search_angular_speed').value
        while self.fresh(tag) is None:
            if gh.is_cancel_requested:
                return self.finish(gh, False)
            if now_sec(self) - t0 > self.get_parameter('verify_timeout_sec').value:
                return self.finish(gh, False, 'verify_failed: tag not found')
            if now_sec(self) - t0 > 0.5:        # 도착 직후 0.5초는 정지 상태에서 확인
                self.twist(0.0, w)
            sleep_sim(self, 0.05)
        self.twist()

        # 3) ALIGNING — 검출값으로 킥보드 중심을 추정 -> nav_bridge 로 '팔이 닿는 빈 자리'까지 경로 주행.
        #    자리 선택과 경로 모두 코스트맵(정적 지도 + 라이다)을 써서 다른 킥보드·벽·이 킥보드의
        #    넘어진 기둥까지 피한다. 로봇팔은 손목을 돌려 맞추므로 어느 방향에서 다가가도 된다.
        self.fb(gh, 'ALIGNING')
        dist = self.get_parameter('align_distance').value
        half = self.get_parameter('scooter_half_depth').value
        centers = []

        def collect():
            d = self.fresh(tag)
            if d is not None:
                ty = yaw_from_quat(d.pose.orientation)
                centers.append((d.pose.position.x - half * math.cos(ty), d.pose.position.y - half * math.sin(ty), ty))

        t0 = now_sec(self)
        while now_sec(self) - t0 < 1.0:              # 1초 모아서 평균 (검출 노이즈)
            collect()
            sleep_sim(self, 0.05)
        if not centers:
            return self.finish(gh, False, 'align_failed: tag lost')
        target = PoseStamped()
        target.header.frame_id = 'map'
        target.pose.position.x = sum(c[0] for c in centers) / len(centers)
        target.pose.position.y = sum(c[1] for c in centers) / len(centers)
        target.pose.orientation.w = 1.0
        nh = future_wait(self, self.nav.send_goal_async(GoToPose.Goal(target_pose=target, standoff=float(dist))), 5.0)
        if nh is None or not nh.accepted:
            return self.finish(gh, False, 'align_failed: nav goal rejected')
        rf = nh.get_result_async()
        t0 = now_sec(self)
        while rclpy.ok() and not rf.done():
            if gh.is_cancel_requested:
                nh.cancel_goal_async()
                return self.finish(gh, False)
            if now_sec(self) - t0 > self.get_parameter('align_timeout_sec').value:
                nh.cancel_goal_async()
                return self.finish(gh, False, 'align_timeout')
            collect()
            pose = self.pose.get()
            if pose is not None:
                self.fb(gh, 'ALIGNING', math.hypot(target.pose.position.x - pose[0],
                                                   target.pose.position.y - pose[1]) - dist)
            sleep_sim(self, 0.1)
        if rf.result() is None or not rf.result().result.success:
            msg = rf.result().result.message if rf.result() is not None else 'no result'
            return self.finish(gh, False, f'align_failed: {msg}')
        self.twist()

        corrected = PoseStamped()
        corrected.header.frame_id = 'map'
        corrected.header.stamp = self.get_clock().now().to_msg()
        recent = centers[-20:]
        corrected.pose.position.x = sum(c[0] for c in recent) / len(recent)
        corrected.pose.position.y = sum(c[1] for c in recent) / len(recent)
        q = quat_from_yaw(recent[-1][2])
        o = corrected.pose.orientation
        o.x, o.y, o.z, o.w = q

        # 4) LOADING
        self.fb(gh, 'LOADING')
        if self.loader.wait_for_server(timeout_sec=0.5):
            err = self.arm_load(gh, sid)
            if err:
                return self.finish(gh, False, f'load_failed: {err}', corrected)
            # 팔이 이미 실었으므로 취소 요청이 와도 적재 성공으로 끝낸다
            self.loaded_pub.publish(String(data=sid))
            self.fb(gh, 'LOADING', 0.0, self.load_count + 1)
            self.get_logger().info(f'킥보드 {sid} 적재 완료 (로봇팔)')
            return self.finish(gh, True, '', corrected, ignore_cancel=True)
        # 로봇팔 없음 — load_time_sec 정지 = 1대 적재
        t0 = now_sec(self)
        while now_sec(self) - t0 < self.get_parameter('load_time_sec').value:
            if gh.is_cancel_requested:
                return self.finish(gh, False)
            self.twist()
            sleep_sim(self, 0.1)
        self.loaded_pub.publish(String(data=sid))
        self.fb(gh, 'LOADING', 0.0, self.load_count + 1)
        self.get_logger().info(f'킥보드 {sid} 적재 완료')
        return self.finish(gh, True, '', corrected)

    def arm_load(self, gh, sid):
        """arm_loader 로 적재. 성공하면 '', 실패하면 이유. 시작한 뒤에는 취소하지 않는다."""
        h = future_wait(self, self.loader.send_goal_async(
            LoadScooter.Goal(scooter_id=sid),
            feedback_callback=lambda m: self.fb(gh, 'LOADING', 1.0 - m.feedback.progress)), 5.0)
        if h is None or not h.accepted:
            return 'arm_loader rejected'
        rf = h.get_result_async()
        t0 = now_sec(self)
        while rclpy.ok() and not rf.done():
            if now_sec(self) - t0 > self.get_parameter('arm_timeout_sec').value:
                return 'arm timeout'
            self.twist()
            sleep_sim(self, 0.1)
        res = rf.result().result if rf.result() is not None else None
        if res is None or not res.success:
            return res.message if res is not None else 'no result'
        return ''


def main():
    rclpy.init()
    spin_single(Collector())
