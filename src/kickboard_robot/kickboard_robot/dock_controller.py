"""F10: DockToCharger Action 서버.

APPROACH : nav_bridge 로 pre-dock 지점(충전소 태그 앞 pre_dock_distance)까지 이동
ALIGN    : 충전소 태그를 보며 P제어로 dock_distance 지점까지 진입
DOCKED   : 설정된 도킹 pose 대비 위치·방향 오차 보고
"""
import math

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from kickboard_interfaces.action import DockToCharger, GoToPose
from kickboard_interfaces.msg import TagDetectionArray

from .common import PoseTracker, future_wait, now_sec, sleep_sim, run_blocking, spin_single
from .motion import GoToController, quat_from_yaw, wrap, yaw_from_quat


def charger_poses(c):
    """map_config 의 charger 항목 -> (tag, dock, pre_dock) 각각 (x, y, yaw). yaw 는 로봇이 바라볼 방향."""
    tx, ty, tyaw = c['tag_pose']['x'], c['tag_pose']['y'], math.radians(c['tag_pose']['yaw_deg'])
    face = tyaw + math.pi
    dock = (tx + c['dock_distance'] * math.cos(tyaw), ty + c['dock_distance'] * math.sin(tyaw), face)
    pre = (tx + c['pre_dock_distance'] * math.cos(tyaw), ty + c['pre_dock_distance'] * math.sin(tyaw), face)
    return (tx, ty, tyaw), dock, pre


class DockController(Node):
    def __init__(self):
        super().__init__('dock_controller')
        self.declare_parameter('map_config_file', '')
        self.declare_parameter('tag_wait_sec', 5.0)
        self.declare_parameter('align_timeout_sec', 40.0)
        with open(self.get_parameter('map_config_file').value) as f:
            self.charger = yaml.safe_load(f)['charger']
        self.tag_id = int(self.charger['tag_id'])
        self.tag, self.dock, self.pre = charger_poses(self.charger)

        self.cb = ReentrantCallbackGroup()
        self.pose = PoseTracker(self)
        self.det = None
        self.create_subscription(TagDetectionArray, '/tag_detections', self.on_dets, 10, callback_group=self.cb)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.nav = ActionClient(self, GoToPose, '/nav_bridge/go_to_pose', callback_group=self.cb)
        ActionServer(self, DockToCharger, '/dock_to_charger', self.execute,
                     goal_callback=lambda g: GoalResponse.ACCEPT,
                     cancel_callback=lambda g: CancelResponse.ACCEPT, callback_group=self.cb)

    def on_dets(self, msg):
        for d in msg.detections:
            if d.id == self.tag_id:
                self.det = (now_sec(self), d)

    def fresh(self):
        return self.det[1] if self.det and now_sec(self) - self.det[0] < 0.6 else None

    def twist(self, v=0.0, w=0.0):
        t = Twist()
        t.linear.x, t.angular.z = v, w
        self.cmd_pub.publish(t)

    def errors(self, pose):
        dx, dy = pose[0] - self.dock[0], pose[1] - self.dock[1]
        lateral = -dx * math.sin(self.dock[2]) + dy * math.cos(self.dock[2])
        return math.hypot(dx, dy), lateral, wrap(pose[2] - self.dock[2])

    def done(self, gh, ok):
        self.twist()
        r = DockToCharger.Result(success=ok)
        pose = self.pose.get()
        if pose:
            pe, _, ye = self.errors(pose)
            r.position_error_m, r.yaw_error_deg = float(pe), float(math.degrees(ye))
        if gh.is_cancel_requested:
            gh.canceled()
        elif ok:
            gh.succeed()
        else:
            gh.abort()
        return r

    async def execute(self, gh):
        # 블로킹 루프는 작업 스레드에서, executor 는 단일 스레드로 다른 콜백 처리
        return await run_blocking(self._execute, gh)

    def _execute(self, gh):
        fb = DockToCharger.Feedback(phase='APPROACH')
        gh.publish_feedback(fb)
        if not self.nav.wait_for_server(timeout_sec=5.0):
            return self.done(gh, False)
        target = PoseStamped()
        target.header.frame_id = 'map'
        target.pose.position.x, target.pose.position.y = self.pre[0], self.pre[1]
        o = target.pose.orientation
        o.x, o.y, o.z, o.w = quat_from_yaw(self.pre[2])
        nh = future_wait(self, self.nav.send_goal_async(GoToPose.Goal(target_pose=target, standoff=0.0)), 5.0)
        if nh is None or not nh.accepted:
            return self.done(gh, False)
        rf = nh.get_result_async()
        while rclpy.ok() and not rf.done():
            if gh.is_cancel_requested:
                nh.cancel_goal_async()
                return self.done(gh, False)
            sleep_sim(self, 0.1)
        if rf.result() is None or not rf.result().result.success:
            self.get_logger().warn('pre-dock 이동 실패')
            return self.done(gh, False)

        # ALIGN — 태그 기반. 태그가 안 보이면 설정된 dock pose 로 블라인드 진입
        fb.phase = 'ALIGN'
        gh.publish_feedback(fb)
        t0 = now_sec(self)
        while self.fresh() is None and now_sec(self) - t0 < self.get_parameter('tag_wait_sec').value:
            sleep_sim(self, 0.05)
        if self.fresh() is None:
            self.get_logger().warn('충전소 태그 미검출 -> 설정 pose 로 진입')
        ctrl = GoToController(v_max=0.08, w_max=0.6, k_lin=0.6, pos_tol=0.02,
                              yaw_tol=math.radians(2.0), allow_reverse=True)
        dist = self.charger['dock_distance']
        goal = self.dock
        smooth = None              # 검출 노이즈(σ 2 cm) 저역 통과 -> 목표가 흔들리지 않게
        t0 = now_sec(self)
        while True:
            if gh.is_cancel_requested:
                return self.done(gh, False)
            if now_sec(self) - t0 > self.get_parameter('align_timeout_sec').value:
                pose = self.pose.get()
                if pose is not None:
                    pe = math.hypot(goal[0] - pose[0], goal[1] - pose[1])
                    ye = math.degrees(wrap(goal[2] - pose[2]))
                    # 아주 정밀한 목표(2 cm, 2°)에 못 들어가도 PRD 기준(5 cm, 5°) 안이면 도킹 성공
                    if pe <= 0.05 and abs(ye) <= 5.0:
                        self.get_logger().info(f'도킹 정렬 시간 초과지만 기준 안 ({pe * 100:.1f} cm, {ye:.1f}°) -> 완료')
                        break
                    self.get_logger().warn(f'도킹 정렬 타임아웃 (목표까지 {pe * 100:.1f} cm, {ye:.1f}°, '
                                           f'로봇 ({pose[0]:.2f}, {pose[1]:.2f}))')
                else:
                    self.get_logger().warn('도킹 정렬 타임아웃')
                return self.done(gh, False)
            d = self.fresh()
            if d is not None:
                ty = yaw_from_quat(d.pose.orientation)
                raw = (d.pose.position.x + dist * math.cos(ty), d.pose.position.y + dist * math.sin(ty), ty)
                if smooth is None:
                    smooth = raw
                else:
                    a = 0.15
                    smooth = (smooth[0] + a * (raw[0] - smooth[0]), smooth[1] + a * (raw[1] - smooth[1]),
                              smooth[2] + a * wrap(raw[2] - smooth[2]))
                goal = (smooth[0], smooth[1], smooth[2] + math.pi)
            pose = self.pose.get()
            if pose is None:
                sleep_sim(self, 0.05)
                continue
            _, lat, ye = self.errors(pose)
            fb.lateral_error, fb.yaw_error = float(lat), float(math.degrees(ye))
            gh.publish_feedback(fb)
            v, w, ok = ctrl.step(pose, goal)
            if ok:
                break
            self.twist(v, w)
            sleep_sim(self, 0.05)

        fb.phase = 'DOCKED'
        gh.publish_feedback(fb)
        r = self.done(gh, True)
        self.get_logger().info(f'도킹 완료: 위치 오차 {r.position_error_m * 100:.1f} cm, '
                               f'방향 오차 {r.yaw_error_deg:.1f}°')
        return r


def main():
    rclpy.init()
    spin_single(DockController())
