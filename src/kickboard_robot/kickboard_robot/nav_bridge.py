"""F6: 우리 인터페이스(GoToPose) <-> 주행 백엔드 연결. 직접 구현과 오픈소스(Nav2)의 경계.

backend:
  nav2   : Nav2 NavigateToPose 호출 (nav2_bringup 이 떠 있어야 함)
  simple : Nav2 없이 직접 주행. 코스트맵(정적 지도 + 라이다로 본 킥보드 등) 위에서 A* 로 경로를 찾고
           경로점을 따라간다 (planner.py). 1초마다, 또는 새 장애물이 경로를 막으면 다시 계획.
           장애물에서 로봇 반폭(0.47 m) 안으로는 안 가고, 1.1 m(로봇 전체를 덮는 원 0.98 m + 격자 오차)까지는 되도록 피한다.
           그래도 전방 obstacle_stop_dist 안에 뭔가 있으면 정지 (안전장치).
standoff > 0 이면 목표 주변 standoff 원 위에서 장애물과 goal_clearance 이상 떨어진 빈 자리를 목표로 한다
(로봇 쪽 방향부터 좌우로 넓혀 찾음). 실패 시 max_retries 번 재시도.
실패 시 max_retries 번 재시도.
"""
import math
import threading

import numpy as np

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from kickboard_interfaces.action import GoToPose

from .common import PoseTracker, future_wait, now_sec, sleep_sim, run_blocking, spin_single, wait_for
from .motion import GoToController, quat_from_yaw, standoff_point, yaw_from_quat
from .occupancy import OccupancyMap, free_standoff
from .planner import Costmap, free_spot


class NavBridge(Node):
    def __init__(self):
        super().__init__('nav_bridge')
        self.declare_parameter('backend', 'simple')
        self.declare_parameter('max_retries', 2)
        self.declare_parameter('goal_timeout_sec', 120.0)
        self.declare_parameter('v_max', 0.22)
        self.declare_parameter('w_max', 1.2)
        self.declare_parameter('pos_tol', 0.08)
        self.declare_parameter('yaw_tol_deg', 8.0)
        self.declare_parameter('obstacle_stop_dist', 0.15)    # simple: 전방 이 거리 안이면 정지 (안전장치)
        self.declare_parameter('blocked_timeout_sec', 8.0)
        self.declare_parameter('map_yaml', '')                # 빈 공간 판정용 지도 (없으면 판정 생략)
        self.declare_parameter('goal_clearance', 1.1)        # 목표 지점에서 장애물(정적 + 라이다)까지: 회전 반경 0.98 + 격자 오차
        self.declare_parameter('inscribed_radius', 0.47)      # 로봇 반폭: 장애물이 이보다 가까운 칸은 통과 불가
        self.declare_parameter('inflation_radius', 1.1)       # 로봇을 덮는 원(반 대각선 0.98) + 격자 오차: 되도록 피함
        self.declare_parameter('replan_sec', 1.0)
        self.declare_parameter('lookahead', 0.5)              # 이만큼 가까워지면 다음 경로점으로
        self.declare_parameter('scan_offset_x', 0.80)         # base_footprint -> base_scan (model.sdf)
        self.declare_parameter('footprint', [-0.92, 0.89, 0.50])   # 차체 x 최소·최대, |y| (+여유): 여기 찍힌 점 무시
        self.declare_parameter('obstacle_memory_sec', 60.0)   # 라이다로 본 장애물을 다시 안 봐도 기억하는 시간

        self.backend = self.get_parameter('backend').value
        self.omap = None
        self.cm = None
        self.cm_lock = threading.Lock()
        if self.get_parameter('map_yaml').value:
            self.omap = OccupancyMap(self.get_parameter('map_yaml').value)
            gp = self.get_parameter
            self.cm = Costmap.from_occupancy(self.omap, inscribed=gp('inscribed_radius').value,
                                             inflate=gp('inflation_radius').value,
                                             decay_sec=gp('obstacle_memory_sec').value)
            self.get_logger().info(f'경로 계획 지도: {gp("map_yaml").value} (격자 {self.cm.w}x{self.cm.h}, '
                                   f'{self.cm.res} m)')
        self.cb = ReentrantCallbackGroup()
        self.pose = PoseTracker(self)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.front_min = float('inf')
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data,
                                 callback_group=self.cb)
        self.nav2 = None
        if self.backend == 'nav2':
            from nav2_msgs.action import NavigateToPose      # Nav2 설치 시에만 필요
            self.NavigateToPose = NavigateToPose
            self.nav2 = ActionClient(self, NavigateToPose, 'navigate_to_pose', callback_group=self.cb)
        self.server = ActionServer(self, GoToPose, '/nav_bridge/go_to_pose', self.execute,
                                   goal_callback=lambda g: GoalResponse.ACCEPT,
                                   cancel_callback=lambda g: CancelResponse.ACCEPT,
                                   callback_group=self.cb)
        self.get_logger().info(f'nav_bridge backend = {self.backend}')

    def on_scan(self, msg):
        # 라이다는 앞 범퍼 위(x 0.80)에 있어서 뒤쪽 빔은 자기 차체에 맞는다 -> 차체 안에 찍힌 점은 버림
        off = self.get_parameter('scan_offset_x').value
        fx0, fx1, fy = self.get_parameter('footprint').value
        r = np.asarray(msg.ranges, dtype=float)
        a = msg.angle_min + msg.angle_increment * np.arange(len(r))
        with np.errstate(invalid='ignore'):
            px, py = off + r * np.cos(a), r * np.sin(a)
            own = np.isfinite(r) & (px > fx0) & (px < fx1) & (np.abs(py) < fy)
            front = (np.abs(np.arctan2(np.sin(a), np.cos(a))) < math.radians(20)) & ~own \
                & np.isfinite(r) & (r > msg.range_min)
        self.front_min = float(r[front].min()) if front.any() else float('inf')
        pose = self.pose.get()
        if self.cm is None or pose is None:
            return
        sx, sy = pose[0] + off * math.cos(pose[2]), pose[1] + off * math.sin(pose[2])
        with self.cm_lock:
            self.cm.update_scan(sx, sy, pose[2], r, msg.angle_min, msg.angle_increment,
                                msg.range_min, msg.range_max, now_sec(self), ignore=own)

    # ------------------------------------------------------------------
    async def execute(self, gh):
        # 블로킹 루프는 작업 스레드에서, executor 는 단일 스레드로 다른 콜백 처리
        return await run_blocking(self._execute, gh)

    def _execute(self, gh):
        g = gh.request
        res = GoToPose.Result()
        if not wait_for(self, lambda: self.pose.get() is not None, 5.0):
            res.success, res.message = False, 'TF map->base_footprint 없음'
            gh.abort()
            return res
        tries = 1 + self.get_parameter('max_retries').value
        for attempt in range(1, tries + 1):
            res.attempts = attempt
            goal = self.make_goal(g)
            self.get_logger().info(f'이동 시도 {attempt}/{tries} -> ({goal[0]:.2f}, {goal[1]:.2f}, '
                                   f'{math.degrees(goal[2]):.0f}°)')
            ok, msg = (self.run_nav2 if self.nav2 else self.run_simple)(gh, goal, attempt)
            if gh.is_cancel_requested:
                self.stop()
                res.success, res.message = False, 'canceled'
                gh.canceled()
                return res
            if ok:
                res.success, res.message = True, 'arrived'
                gh.succeed()
                return res
            self.get_logger().warn(f'이동 실패({msg}), 재시도')
            sleep_sim(self, 1.0)
        res.success, res.message = False, f'nav failed after {tries} attempts'
        gh.abort()
        return res

    def make_goal(self, g):
        p = g.target_pose.pose
        tx, ty = p.position.x, p.position.y
        if g.standoff > 0.0:
            rx, ry, _ = self.pose.get()
            # PRD: 킥보드에서 standoff 만큼 떨어진 '빈 공간'. 로봇 쪽 방향이 막혀 있으면 각도를 돌려 재탐색.
            # 코스트맵(정적 지도 + 라이다로 본 킥보드)으로 판정 -> 다른 킥보드·벽 옆 자리는 고르지 않음
            clearance = self.get_parameter('goal_clearance').value
            if self.cm is not None:
                with self.cm_lock:
                    goal = free_spot(self.cm, (tx, ty), (rx, ry), g.standoff, clearance, now_sec(self))
            else:
                goal = free_standoff(self.omap, (tx, ty), (rx, ry), g.standoff, clearance)
            if goal is None:
                self.get_logger().warn('주변에 빈 접근 지점이 없음 -> 직선 offset 사용')
                goal = standoff_point((tx, ty), (rx, ry), g.standoff)
            return goal
        return tx, ty, yaw_from_quat(p.orientation)

    def stop(self):
        self.cmd_pub.publish(Twist())

    # ---------------- simple backend ----------------
    def run_simple(self, gh, goal, attempt):
        gp = self.get_parameter
        ctrl = GoToController(v_max=gp('v_max').value, w_max=gp('w_max').value, pos_tol=gp('pos_tol').value,
                              yaw_tol=math.radians(gp('yaw_tol_deg').value))
        t0 = now_sec(self)
        blocked_since = None
        fb = GoToPose.Feedback(attempt=attempt)
        stop_d = gp('obstacle_stop_dist').value
        path, planned_at = None, -1e9
        while rclpy.ok():
            if gh.is_cancel_requested:
                return False, 'canceled'
            now = now_sec(self)
            if now - t0 > gp('goal_timeout_sec').value:
                self.stop()
                return False, 'timeout'
            pose = self.pose.get()
            if pose is None:
                sleep_sim(self, 0.05)
                continue
            dist = math.hypot(goal[0] - pose[0], goal[1] - pose[1])

            # ---- 경로 계획 (코스트맵이 있을 때) ----
            if self.cm is not None and dist > gp('pos_tol').value:
                # 목표가 '빈 자리'로 고른 것(standoff)인데 그사이 새 장애물(방금 내려놓은 킥보드 등)이 붙었으면 다시 고른다
                if gh.request.standoff > 0.0 and now - planned_at > gp('replan_sec').value:
                    with self.cm_lock:
                        tight = self.cm.dist_at(goal[0], goal[1], now) < gp('goal_clearance').value - 0.05
                    if tight:
                        new = self.make_goal(gh.request)
                        self.get_logger().info(f'목표 주변에 새 장애물 -> 목표 변경 ({goal[0]:.2f}, {goal[1]:.2f}) -> '
                                               f'({new[0]:.2f}, {new[1]:.2f})')
                        goal, path = new, None
                        dist = math.hypot(goal[0] - pose[0], goal[1] - pose[1])
                with self.cm_lock:
                    need = (path is None or now - planned_at > gp('replan_sec').value
                            or self.cm.path_blocked(path, now, pose[:2]))
                    if need:
                        path = self.cm.plan(pose[:2], goal[:2], now)
                        planned_at = now
                if path is None:
                    self.stop()
                    blocked_since = blocked_since or now
                    if now - blocked_since > gp('blocked_timeout_sec').value:
                        return False, 'no path'
                    sleep_sim(self, 0.2)
                    continue
                look = gp('lookahead').value
                if not path:
                    path = [goal[:2]]
                while len(path) > 1 and math.hypot(path[0][0] - pose[0], path[0][1] - pose[1]) < look:
                    path.pop(0)
                wp = path[0]
            else:
                wp = goal
            last = self.cm is None or len(path or []) <= 1
            v, w, done = ctrl.step(pose, (wp[0], wp[1], goal[2]), final_yaw=last)
            if done and last:
                self.stop()
                return True, ''
            if done:                                       # 중간 경로점 도착 -> 다음 점
                path.pop(0)
                continue
            if v > 0 and self.front_min < stop_d:          # 안전장치: 계획에 없던 가까운 장애물
                v = 0.0
                blocked_since = blocked_since or now
                if now - blocked_since > gp('blocked_timeout_sec').value:
                    self.stop()
                    return False, 'blocked'
            else:
                blocked_since = None
            tw = Twist()
            tw.linear.x, tw.angular.z = v, w
            self.cmd_pub.publish(tw)
            fb.distance_remaining = dist
            gh.publish_feedback(fb)
            sleep_sim(self, 0.05)
        return False, 'shutdown'

    # ---------------- nav2 backend ----------------
    def run_nav2(self, gh, goal, attempt):
        if not self.nav2.wait_for_server(timeout_sec=5.0):
            return False, 'navigate_to_pose 서버 없음'
        ng = self.NavigateToPose.Goal()
        ng.pose = PoseStamped()
        ng.pose.header.frame_id = 'map'
        ng.pose.header.stamp = self.get_clock().now().to_msg()
        ng.pose.pose.position.x, ng.pose.pose.position.y = goal[0], goal[1]
        q = quat_from_yaw(goal[2])
        o = ng.pose.pose.orientation
        o.x, o.y, o.z, o.w = q
        fb = GoToPose.Feedback(attempt=attempt)

        def on_fb(m):
            fb.distance_remaining = float(m.feedback.distance_remaining)
            gh.publish_feedback(fb)

        handle = future_wait(self, self.nav2.send_goal_async(ng, feedback_callback=on_fb), 10.0)
        if handle is None or not handle.accepted:
            return False, 'goal rejected'
        rf = handle.get_result_async()
        t0 = now_sec(self)
        while rclpy.ok() and not rf.done():
            if gh.is_cancel_requested or now_sec(self) - t0 > self.get_parameter('goal_timeout_sec').value:
                handle.cancel_goal_async()
                return False, 'canceled/timeout'
            sleep_sim(self, 0.1)
        from action_msgs.msg import GoalStatus
        r = rf.result()
        return (r is not None and r.status == GoalStatus.STATUS_SUCCEEDED), f'status={getattr(r, "status", "?")}'


def main():
    rclpy.init()
    spin_single(NavBridge())
