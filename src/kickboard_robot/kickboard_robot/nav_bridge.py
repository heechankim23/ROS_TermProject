"""F6: 우리 인터페이스(GoToPose) <-> 주행 백엔드 연결. 직접 구현과 오픈소스(Nav2)의 경계.

backend:
  nav2   : Nav2 NavigateToPose 호출 (nav2_bringup 이 떠 있어야 함)
  simple : Nav2 없이 직접 주행. 코스트맵(정적 지도 + 라이다로 본 킥보드 등) 위에서 A* 로 경로를 찾고
           경로점을 따라간다 (planner.py). 1초마다, 또는 새 장애물이 경로를 막으면 다시 계획.
           장애물에서 로봇 반폭(0.47 m) 안으로는 안 가고, 0.95 m(로봇 전체를 덮는 원)까지는 되도록 피한다.
           그래도 전방 obstacle_stop_dist 안에 뭔가 있으면 정지 (안전장치).
standoff > 0 이면 목표 주변 standoff 원 위에서 장애물과 goal_clearance 이상 떨어진 빈 자리를 목표로 한다
(로봇 쪽 방향부터 좌우로 넓혀 찾음). 탐색·이동 실패 모두 max_retries 번 재시도한다.
목표 제한·재계획은 ROS 시간, 센서 수신 감시·취소 확인 대기는 실제 경과 시간을 쓴다.
"""
import math
import threading
import time

import numpy as np

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from rclpy.time import Time

from kickboard_interfaces.action import GoToPose

from .common import PoseTracker, future_wait, now_sec, run_blocking, spin_single
from .motion import GoToController, quat_from_yaw, yaw_from_quat
from .occupancy import OccupancyMap, free_standoff
from .planner import Costmap, free_spot, nearest_free
from .kinematic_planner import KinematicParams, PathTracker, path_collides, plan_path


class NavBridge(Node):
    def __init__(self):
        super().__init__('nav_bridge')
        self.declare_parameter('backend', 'simple')
        self.declare_parameter('max_retries', 2)
        self.declare_parameter('goal_timeout_sec', 120.0)
        self.declare_parameter('v_max', 0.22)
        self.declare_parameter('w_max', 1.0)          # Gazebo DiffDrive max_angular_velocity 와 같게 (넘으면 잘림)
        self.declare_parameter('pos_tol', 0.08)
        self.declare_parameter('yaw_tol_deg', 8.0)
        self.declare_parameter('obstacle_stop_dist', 0.15)    # simple: 전방 이 거리 안이면 정지 (안전장치)
        self.declare_parameter('blocked_timeout_sec', 8.0)
        self.declare_parameter('backoff_m', 0.6)               # 앞이 막혔을 때 뒤로 물러나는 최대 거리 (0 = 안 함)
        self.declare_parameter('map_yaml', '')                # 빈 공간 판정용 지도 (없으면 판정 생략)
        self.declare_parameter('goal_clearance', 1.1)        # 목표 지점에서 장애물(정적 + 라이다)까지: 회전 반경 0.98 + 격자 오차
        self.declare_parameter('inscribed_radius', 0.58)      # 로봇 반폭 0.525 + 여유: 장애물이 이보다 가까운 칸은 통과 불가
        self.declare_parameter('inflation_radius', 1.2)       # 로봇을 덮는 원(반 대각선 1.11) + 격자 오차: 되도록 피함
        self.declare_parameter('replan_sec', 1.0)
        self.declare_parameter('lookahead', 0.5)              # 이만큼 가까워지면 다음 경로점으로
        self.declare_parameter('scan_offset_x', 0.86)         # base_footprint -> base_scan (model.sdf)
        self.declare_parameter('footprint', [-0.98, 0.96, 0.58])   # 차체 x 최소·최대, |y| (+여유): 여기 찍힌 점 무시
        self.declare_parameter('obstacle_memory_sec', 60.0)   # 라이다로 본 장애물을 다시 안 봐도 기억하는 시간
        self.declare_parameter('physical_map_yaml', '')       # 물리 장애물만 든 지도. 주행 지도와 다른 칸 = 진입 금지 구역
        self.declare_parameter('keepout_goal_clearance', -1.0)   # 설 자리와 진입 금지 경계 사이 최소 거리 (음수 = 벽과 똑같이 취급)
        self.declare_parameter('costmap_res', 0.1)            # 경로 계획 격자 (큰 지도는 0.2 로 키우면 A* 가 빨라짐)
        # 캠퍼스처럼 큰 지도: 맵 JSON(maps/<맵>.json)을 주면 큰 경로(1 m 격자 그래프) + 로봇 주변 창 A* 로 계획 (campus_nav.py)
        self.declare_parameter('route_map_json', '')
        self.declare_parameter('window_m', 70.0)

        # 운동학 제약 경로 (kinematic_planner.py): 최소 회전 반경 곡선 + 차체 충돌 검사 + 곡률 제한 추종.
        # 계획이 안 되면 이번 목표는 예전 방식(격자 A* + 제자리 회전 후 직진)으로 간다
        self.declare_parameter('kinematic_planning', True)
        self.declare_parameter('min_turn_radius', 0.8)        # 로봇 최소 회전 반경 (m). 추종 곡률 한계, 계획은 1.25배
        self.declare_parameter('k_rate_max', 2.5)             # 조향 속도 한계: 명령 곡률 변화 1/(m·s). 경로 곡률도 연속(완화 곡선)
        self.declare_parameter('allow_reverse', True)         # 좁은 곳에서 후진 허용 (비용 2.5배, 전·후진 전환 벌점)
        self.declare_parameter('pivot_at_ends', True)         # 출발·도착점에서만 제자리 회전 (회전할 공간이 있을 때)
        self.declare_parameter('plan_time_budget_sec', 2.5)
        self.declare_parameter('kin_replan_dev_m', 0.6)       # 경로에서 이만큼 벗어나면 다시 계획

        self.declare_parameter('scan_timeout_sec', 1.0)
        self.declare_parameter('pose_timeout_sec', 1.0)
        self.declare_parameter('sensor_wait_timeout_sec', 5.0)
        self.goal_lock = threading.Lock()
        self.last_scan_at = None
        self.nav2_pending = None

        kc = self.get_parameter('keepout_goal_clearance').value
        self.keep_clear = kc if kc >= 0 else None
        self.backend = self.get_parameter('backend').value
        self.omap = None
        self.cm = None
        self.cm_lock = threading.Lock()
        if self.get_parameter('map_yaml').value:
            self.omap = OccupancyMap(self.get_parameter('map_yaml').value)
            gp = self.get_parameter
            if gp('route_map_json').value:
                from .campus_nav import RollingCostmap
                self.cm = RollingCostmap(self.omap, gp('route_map_json').value, window_m=gp('window_m').value,
                                         res=gp('costmap_res').value, inscribed=gp('inscribed_radius').value,
                                         inflate=gp('inflation_radius').value,
                                         decay_sec=gp('obstacle_memory_sec').value, logger=self.get_logger().warn)
                self.get_logger().info(f'큰 지도 모드: 큰 경로 격자 {self.cm.graph.n} 칸, 계획 창 {gp("window_m").value:.0f} m')
            else:
                self.cm = Costmap.from_occupancy(self.omap, res=gp('costmap_res').value,
                                                 inscribed=gp('inscribed_radius').value,
                                                 inflate=gp('inflation_radius').value,
                                                 decay_sec=gp('obstacle_memory_sec').value)
                if gp('physical_map_yaml').value and gp('keepout_goal_clearance').value >= 0:
                    self.cm.set_physical(OccupancyMap(gp('physical_map_yaml').value))
                self.get_logger().info(f'경로 계획 지도: {gp("map_yaml").value} (격자 {self.cm.w}x{self.cm.h}, '
                                       f'{self.cm.res} m)')
        self.cb = ReentrantCallbackGroup()
        self.pose = PoseTracker(self)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.front_min = float('inf')
        self.scan_wait = []        # 찍힌 시각의 TF 를 기다리는 스캔 (최대 3개)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data,
                                 callback_group=self.cb)
        self.nav2 = None
        if self.backend == 'nav2':
            from nav2_msgs.action import NavigateToPose      # Nav2 설치 시에만 필요
            self.NavigateToPose = NavigateToPose
            self.nav2 = ActionClient(self, NavigateToPose, 'navigate_to_pose', callback_group=self.cb)
        self.server = ActionServer(self, GoToPose, '/nav_bridge/go_to_pose', self.execute,
                                   goal_callback=self.accept_goal,
                                   cancel_callback=lambda g: CancelResponse.ACCEPT,
                                   callback_group=self.cb)
        self.get_logger().info(f'nav_bridge backend = {self.backend}')

    def accept_goal(self, request):
        # Reserve before execution so two accepted goals cannot drive together.
        if self.nav2_pending is not None and not self.nav2_pending.done():
            return GoalResponse.REJECT
        p = request.target_pose.pose
        values = (p.position.x, p.position.y, p.orientation.x, p.orientation.y,
                  p.orientation.z, p.orientation.w, request.standoff)
        if (request.target_pose.header.frame_id not in ('', 'map')
                or not all(math.isfinite(v) for v in values) or request.standoff < 0):
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT if self.goal_lock.acquire(blocking=False) else GoalResponse.REJECT

    def current_pose(self):
        # Check TF age here without changing the shared PoseTracker.
        try:
            transform = self.pose.buf.lookup_transform('map', 'base_footprint', Time())
            stamp = transform.header.stamp
            age = now_sec(self) - (stamp.sec + stamp.nanosec * 1e-9)
            if not 0.0 <= age <= self.get_parameter('pose_timeout_sec').value:
                return None
            tr = transform.transform
            pose = (tr.translation.x, tr.translation.y, yaw_from_quat(tr.rotation))
            return pose if all(math.isfinite(v) for v in pose) else None
        except Exception:
            return None

    def scan_ready(self):
        return (self.last_scan_at is not None
                and time.monotonic() - self.last_scan_at <= self.get_parameter('scan_timeout_sec').value)

    def on_scan(self, msg):
        # 라이다는 앞 범퍼 위(x 0.80)에 있어서 뒤쪽 빔은 자기 차체에 맞는다 -> 차체 안에 찍힌 점은 버림
        stamp = msg.header.stamp
        age = now_sec(self) - (stamp.sec + stamp.nanosec * 1e-9)
        if (not msg.ranges or not math.isfinite(msg.range_max) or msg.range_max <= msg.range_min
                or not math.isfinite(msg.angle_min) or not math.isfinite(msg.angle_increment)
                or not 0.0 <= age <= self.get_parameter('scan_timeout_sec').value):
            self.last_scan_at = None
            return
        off = self.get_parameter('scan_offset_x').value
        fx0, fx1, fy = self.get_parameter('footprint').value
        r = np.asarray(msg.ranges, dtype=float)
        a = msg.angle_min + msg.angle_increment * np.arange(len(r))
        with np.errstate(invalid='ignore'):
            px, py = off + r * np.cos(a), r * np.sin(a)
            own = np.isfinite(r) & (px > fx0) & (px < fx1) & (np.abs(py) < fy)
            front = (np.abs(np.arctan2(np.sin(a), np.cos(a))) < math.radians(20)) & ~own \
                & np.isfinite(r) & (r >= msg.range_min - 1e-6)
        sector = (np.abs(np.arctan2(np.sin(a), np.cos(a))) < math.radians(20)) & ~own
        usable = (np.isfinite(r) & (r >= msg.range_min - 1e-6) & (r <= msg.range_max + 1e-6)) | np.isposinf(r)
        # Unknown/too-close forward beams must not be interpreted as clear space.
        if not sector.any() or not np.all(usable[sector]):
            self.last_scan_at = None
            return
        self.front_min = float(r[front].min()) if front.any() else float('inf')
        pose = self.current_pose()
        if pose is None:
            self.last_scan_at = None
            return
        if self.cm is None:
            self.last_scan_at = time.monotonic()
            return
        # 코스트맵에는 '스캔이 찍힌 시각'의 로봇 pose 로 넣는다. 최신 pose 를 쓰면 회전 중(최대 1.2 rad/s)에
        # 처리 지연 0.2 s 만으로 3.5 m 앞 벽이 0.8 m 옆에 찍혀, 빈 곳에 유령 장애물이 생기고 60초 동안 길을 막았다.
        # 그 시각의 TF 는 스캔보다 조금 늦게 오므로 다음 스캔 때 처리한다.
        self.scan_wait.append((msg, r, own))
        self.scan_wait = self.scan_wait[-3:]
        while self.scan_wait:
            m, rr, ow = self.scan_wait[0]
            pose = self.pose.get_at(m.header.stamp)
            if pose is None:
                if len(self.scan_wait) < 3:
                    break                       # TF 가 아직 안 옴 -> 다음 스캔 때 다시
                self.scan_wait.pop(0)           # 끝내 못 구한 스캔은 버림
                continue
            self.scan_wait.pop(0)
            sx, sy = pose[0] + off * math.cos(pose[2]), pose[1] + off * math.sin(pose[2])
            with self.cm_lock:
                self.cm.update_scan(sx, sy, pose[2], rr, m.angle_min, m.angle_increment,
                                    m.range_min, m.range_max, now_sec(self), ignore=ow)
        self.last_scan_at = time.monotonic()

    # ------------------------------------------------------------------
    async def execute(self, gh):
        # 블로킹 루프는 작업 스레드에서, executor 는 단일 스레드로 다른 콜백 처리
        return await run_blocking(self._execute, gh)

    def _execute(self, gh):
        try:
            return self._execute_goal(gh)
        except Exception as exc:
            self.stop()
            self.get_logger().error(f'Navigation exception: {exc}')
            res = GoToPose.Result()
            res.success, res.message = False, str(exc)
            gh.abort()
            return res
        finally:
            self.goal_lock.release()

    def sensor_problem(self):
        pose_missing = self.current_pose() is None
        scan_missing = not self.nav2 and not self.scan_ready()
        if pose_missing and scan_missing:
            return 'pose/scan unavailable'
        if pose_missing:
            return 'pose unavailable'
        return 'scan unavailable' if scan_missing else ''

    def wait_for_sensors(self, gh):
        """Wait stopped, bounded by wall time even if the simulation clock stops."""
        deadline = time.monotonic() + self.get_parameter('sensor_wait_timeout_sec').value
        while rclpy.ok():
            if gh.is_cancel_requested:
                return 'canceled'
            problem = self.sensor_problem()
            if not problem:
                return ''
            self.stop()
            if time.monotonic() >= deadline:
                return problem
            time.sleep(0.05)
        return 'shutdown'

    def wait_stopped(self, gh, duration):
        """Keep publishing zero velocity and allow cancellation between retries."""
        deadline = time.monotonic() + duration
        while rclpy.ok():
            self.stop()
            if gh.is_cancel_requested:
                return 'canceled'
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return ''
            time.sleep(min(0.05, remaining))
        return 'shutdown'

    def failed_result(self, gh, res, reason):
        self.stop()
        res.success, res.message = False, reason
        if gh.is_cancel_requested or reason == 'canceled':
            res.message = 'canceled'
            gh.canceled()
        else:
            gh.abort()
        return res

    def _execute_goal(self, gh):
        g = gh.request
        res = GoToPose.Result()
        problem = self.wait_for_sensors(gh)
        if problem:
            return self.failed_result(gh, res, problem)

        # attempts is uint8 in the existing action; do not change its interface.
        tries = min(255, max(1, 1 + self.get_parameter('max_retries').value))
        for attempt in range(1, tries + 1):
            res.attempts = attempt
            problem = self.wait_for_sensors(gh)
            if problem:
                return self.failed_result(gh, res, problem)
            goal, msg = self.resolve_goal(g)
            if gh.is_cancel_requested:
                return self.failed_result(gh, res, 'canceled')
            if not rclpy.ok():
                return self.failed_result(gh, res, 'shutdown')
            if goal is None:
                ok = False
            else:
                self.get_logger().info(f'이동 시도 {attempt}/{tries} -> ({goal[0]:.2f}, {goal[1]:.2f}, '
                                       f'{math.degrees(goal[2]):.0f}°)')
                ok, msg = (self.run_nav2 if self.nav2 else self.run_simple)(gh, goal, attempt)
            if gh.is_cancel_requested:
                return self.failed_result(gh, res, 'canceled')
            if not rclpy.ok():
                return self.failed_result(gh, res, 'shutdown')
            self.stop()
            if ok:
                res.success, res.message = True, 'arrived'
                gh.succeed()
                return res
            p = g.target_pose.pose.position
            self.get_logger().warn(f'이동 실패 {attempt}/{tries}: {msg}; '
                                   f'target=({p.x:.2f}, {p.y:.2f}), standoff={g.standoff:.2f}')
            if msg in ('sensor timeout', 'shutdown', 'canceled', 'cancel pending'):
                break
            if attempt < tries:
                problem = self.wait_stopped(gh, 0.1)
                if problem:
                    return self.failed_result(gh, res, problem)
        return self.failed_result(gh, res, f'nav failed after {attempt} attempts: {msg}')

    def make_goal(self, g):
        """Keep the existing goal-only helper API for navigation callers/tests."""
        return self.resolve_goal(g)[0]

    def resolve_goal(self, g):
        """Return (goal, reason) without changing GoToPose or other nodes."""
        p = g.target_pose.pose
        tx, ty = p.position.x, p.position.y
        if g.standoff > 0.0:
            pose = self.current_pose()
            if pose is None:
                return None, 'pose unavailable'
            rx, ry, _ = pose
            clearance = self.get_parameter('goal_clearance').value
            if self.cm is not None:
                with self.cm_lock:
                    cmv = self.cm.view_for((tx, ty)) if hasattr(self.cm, 'view_for') else self.cm
                    goal = free_spot(cmv, (tx, ty), (rx, ry), g.standoff, clearance, now_sec(self),
                                     keep_clearance=self.keep_clear)
            else:
                goal = free_standoff(self.omap, (tx, ty), (rx, ry), g.standoff, clearance)
            if goal is None and self.cm is not None:
                # GPS 오차로 목표가 벽 근처·벽 밖·건물 안에 찍힌 경우: standoff 원을 포기하고
                # 목표에서 가장 가까운 빈 자리로 간다 (거기서 VERIFYING 회전 탐색으로 태그를 찾음)
                with self.cm_lock:
                    cmv = self.cm.view_for((tx, ty)) if hasattr(self.cm, 'view_for') else self.cm
                    goal = nearest_free(cmv, (tx, ty), clearance, now_sec(self), min_dist=0.8,
                                        keep_clearance=self.keep_clear)
                if goal is not None:
                    self.get_logger().warn(f'standoff 원 위에 빈 자리 없음 -> 가장 가까운 빈 자리 '
                                           f'({goal[0]:.2f}, {goal[1]:.2f}) 사용')
            return (goal, '') if goal is not None else (None, 'no safe approach point')
        return (tx, ty, yaw_from_quat(p.orientation)), ''

    def stop(self):
        self.cmd_pub.publish(Twist())

    # ---------------- 운동학 경로 ----------------
    def kin_params(self, greedy=False):
        """greedy: 첫 계획이 시간 안에 안 끝났을 때 — 휴리스틱을 더 믿어 빨리 찾음 (조금 덜 짧은 경로)"""
        gp = self.get_parameter
        return KinematicParams(min_turn_radius=gp('min_turn_radius').value, allow_reverse=gp('allow_reverse').value,
                               pivot_at_ends=gp('pivot_at_ends').value,
                               time_budget_sec=gp('plan_time_budget_sec').value,
                               heuristic_weight=3.0 if greedy else 1.4)

    def kin_plan(self, pose, goal, now, greedy=False):
        """-> (촘촘한 경로, 최종 목표인가). 큰 지도(RollingCostmap)에서 목표가 창 밖이면 큰 경로의 중간 목표까지.
        계획은 수 초 걸릴 수 있어 지도 복사본으로 잠금 밖에서 한다 (잠금을 쥐면 on_scan 이 막혀 단일 executor 의
        TF·스캔 수신까지 멈춤)"""
        cm, snap = self.plan_snapshot(pose)
        target, final = tuple(goal), True
        if hasattr(snap, 'win'):
            if not snap.in_window(goal, 2.0):
                found = snap.plan(pose[:2], goal[:2], now) is not None and snap.sub is not None
                self.keep_route(cm, snap)
                if not found:
                    return None, True
                sx, sy = snap.sub
                target, final = (sx, sy, math.atan2(sy - pose[1], sx - pose[0])), False
            else:
                snap.sub = None
                self.keep_route(cm, snap)
            snap = snap.win
        path = plan_path(snap, tuple(pose), target, now, self.kin_params(greedy), goal_yaw_free=not final,
                         log=self.get_logger().info)
        return path, final

    def plan_snapshot(self, pose):
        """-> (원본 코스트맵, 계획용 복사본). 잠금은 복사하는 동안만"""
        with self.cm_lock:
            if hasattr(self.cm, 'win'):
                self.cm.ensure(pose[:2])
            return self.cm, self.cm.snapshot()

    def keep_route(self, cm, snap):
        """복사본에서 정한 중간 목표를 원본에 남긴다 (needs_replan 이 씀)"""
        with self.cm_lock:
            cm.sub = snap.sub
            if snap.sub is not None:
                cm.sub_t = snap.sub_t

    # ---------------- simple backend ----------------
    def run_simple(self, gh, goal, attempt):
        gp = self.get_parameter
        ctrl = GoToController(v_max=gp('v_max').value, w_max=gp('w_max').value, pos_tol=gp('pos_tol').value,
                              yaw_tol=math.radians(gp('yaw_tol_deg').value))
        t0 = now_sec(self)
        blocked_since = None
        sensor_missing_since = None
        fb = GoToPose.Feedback(attempt=attempt)
        stop_d = gp('obstacle_stop_dist').value
        path, planned_at = None, -1e9
        backed = 0.0
        kin = {'tracker': None, 'checked_at': -1e9, 'fails': 0, 'failed': False, 'final': True}
        while rclpy.ok():
            if gh.is_cancel_requested:
                self.stop()
                return False, 'canceled'
            now = now_sec(self)
            if now - t0 > gp('goal_timeout_sec').value:
                self.stop()
                return False, 'timeout'
            pose = self.current_pose()
            if pose is None or not self.scan_ready():
                self.stop()
                path = None
                if sensor_missing_since is None:
                    sensor_missing_since = time.monotonic()
                    self.get_logger().warn(f'센서 복구 대기: {self.sensor_problem()}')
                if time.monotonic() - sensor_missing_since > gp('sensor_wait_timeout_sec').value:
                    self.get_logger().warn(f'센서 복구 시간 초과: {self.sensor_problem()}')
                    return False, 'sensor timeout'
                time.sleep(0.05)
                continue
            sensor_missing_since = None
            dist = math.hypot(goal[0] - pose[0], goal[1] - pose[1])

            # 목표가 '빈 자리'로 고른 것(standoff)인데 그사이 새 장애물이 붙었으면 다시 고른다 (두 방식 공통)
            if self.cm is not None and dist > gp('pos_tol').value:
                if gh.request.standoff > 0.0 and now - planned_at > gp('replan_sec').value:
                    with self.cm_lock:
                        tight = not self.cm.spot_ok(goal[0], goal[1], now, gp('goal_clearance').value - 0.05,
                                                    None if self.keep_clear is None else self.keep_clear - 0.05)
                    if tight:
                        new, reason = self.resolve_goal(gh.request)
                        if new is None:
                            self.stop()
                            return False, reason
                        self.get_logger().info(f'목표 주변에 새 장애물 -> 목표 변경 ({goal[0]:.2f}, {goal[1]:.2f}) -> '
                                               f'({new[0]:.2f}, {new[1]:.2f})')
                        goal, path = new, None
                        kin['tracker'] = None
                        dist = math.hypot(goal[0] - pose[0], goal[1] - pose[1])
            use_kin = self.cm is not None and gp('kinematic_planning').value and not kin['failed']
            if use_kin:
                # ---- 운동학 제약 경로 (최소 회전 반경 곡선 + 차체 충돌 검사) ----
                tr = kin['tracker']
                need = tr is None
                if not need and now - kin['checked_at'] > gp('replan_sec').value:
                    kin['checked_at'] = planned_at = now
                    with self.cm_lock:
                        grid = self.cm.win if hasattr(self.cm, 'win') else self.cm
                        blocked = path_collides(grid, tr.path, now, self.kin_params(), from_index=tr.i,
                                                start=pose[:2])
                        need = (blocked or tr.cross_track(pose) > gp('kin_replan_dev_m').value
                                or (hasattr(self.cm, 'needs_replan') and self.cm.needs_replan(pose, now)))
                    if need:
                        self.get_logger().info('경로 다시 계획 (' + ('새 장애물' if blocked else '경로 이탈/구간 끝') + ')')
                if need:
                    self.stop()                  # 계획하는 동안 직전 속도 명령으로 계속 가지 않게
                    kpath, final = self.kin_plan(pose, goal, now, greedy=kin['fails'] > 0)
                    if kpath is None:
                        self.stop()
                        kin['fails'] += 1
                        if kin['fails'] >= 2:
                            kin['failed'] = True
                            self.get_logger().warn('운동학 경로를 못 찾음 -> 이번 목표는 격자 A* 경로(예전 방식)로')
                        time.sleep(0.05)
                        continue
                    kin['fails'] = 0
                    kin['final'] = final
                    kin['checked_at'] = planned_at = now
                    tr = kin['tracker'] = PathTracker(
                        kpath, v_max=gp('v_max').value, w_max=gp('w_max').value,
                        min_turn_radius=gp('min_turn_radius').value, pos_tol=gp('pos_tol').value,
                        yaw_tol=math.radians(gp('yaw_tol_deg').value), final_yaw=final,
                        k_rate_max=gp('k_rate_max').value)
                    blocked_since = None
                v, w, done = tr.step(pose)
                if done:
                    if kin['final']:
                        self.stop()
                        return True, ''
                    kin['tracker'] = None                    # 큰 지도의 중간 목표 도착 -> 다음 구간 계획
                    continue
                nxt = tr.path[min(tr.i, len(tr.path) - 1)]
                wp = (nxt[0], nxt[1])
            else:
                # ---- 경로 계획 (코스트맵이 있을 때) ----
                if self.cm is not None and dist > gp('pos_tol').value:
                    with self.cm_lock:
                        need = (path is None or now - planned_at > gp('replan_sec').value
                                or self.cm.path_blocked(path, now, pose[:2])
                                or (hasattr(self.cm, 'needs_replan') and self.cm.needs_replan(pose, now)))
                    if need:
                        self.stop()
                        cm, snap = self.plan_snapshot(pose)
                        path = snap.plan(pose[:2], goal[:2], now)
                        if hasattr(snap, 'win'):
                            self.keep_route(cm, snap)
                        planned_at = now
                    if path is None:
                        self.stop()
                        if blocked_since is None:      # 왜 길이 없는지 한 번 남긴다 (출발 칸이 막힌 곳인지, 목표가 막힌 곳인지)
                            with self.cm_lock:
                                sj, si = self.cm.cell(pose[0], pose[1])
                                gj, gi = self.cm.cell(goal[0], goal[1])
                                ins = self.cm.inside(sj, si) and self.cm.inside(gj, gi)
                                info = (f'출발 ({pose[0]:.2f}, {pose[1]:.2f}) 정적 장애물/진입 금지 칸={bool(self.cm.static[sj, si])} '
                                        f'장애물까지 {self.cm.dist_at(pose[0], pose[1], now):.2f} m, '
                                        f'목표 ({goal[0]:.2f}, {goal[1]:.2f}) 장애물까지 {self.cm.dist_at(goal[0], goal[1], now):.2f} m'
                                        ) if ins else '출발 또는 목표가 지도 밖'
                            self.get_logger().warn(f'경로 없음: {info}')
                        blocked_since = now if blocked_since is None else blocked_since
                        if now - blocked_since > gp('blocked_timeout_sec').value:
                            return False, 'no path'
                        time.sleep(0.05)
                        continue
                    look = gp('lookahead').value
                    if not path:
                        path = [goal[:2]]
                    while len(path) > 1 and math.hypot(path[0][0] - pose[0], path[0][1] - pose[1]) < look:
                        with self.cm_lock:
                            if not self.cm.segment_ok(pose[:2], path[1], self.cm.cost_grid(now), 1e18):
                                break
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
                v, w = 0.0, 0.0
                blocked_since = now if blocked_since is None else blocked_since
                # 앞이 거의 닿아 있으면(거치대·표지판 옆에서 적재하며 돌다가 붙은 경우) 뒤가 비었을 때 조금 물러난다
                back = gp('backoff_m').value
                if back > 0 and now - blocked_since > 1.0 and backed < back and self.cm is not None:
                    bx, by = pose[0] - 1.3 * math.cos(pose[2]), pose[1] - 1.3 * math.sin(pose[2])
                    with self.cm_lock:
                        rear_ok = self.cm.dist_at(bx, by, now) > gp('inscribed_radius').value
                    if rear_ok:
                        v, w = -0.15, 0.0
                        backed += 0.15 * 0.05
                        if backed >= back:
                            blocked_since, path = None, None     # 다 물러났으면 다시 계획
                            kin['tracker'] = None
                if blocked_since is not None and now - blocked_since > gp('blocked_timeout_sec').value:
                    self.stop()
                    self.get_logger().warn(f'전방 {self.front_min:.2f} m 에 장애물 (계획에 없음): 로봇 ({pose[0]:.1f}, {pose[1]:.1f}, '
                                           f'{math.degrees(pose[2]):.0f}°) 다음 점 ({wp[0]:.1f}, {wp[1]:.1f}) 목표 ({goal[0]:.1f}, {goal[1]:.1f})')
                    return False, 'blocked'
            else:
                blocked_since = None
            tw = Twist()
            tw.linear.x, tw.angular.z = v, w
            self.cmd_pub.publish(tw)
            fb.distance_remaining = dist
            gh.publish_feedback(fb)
            time.sleep(0.05)
        self.stop()
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
        self.nav2_pending = rf
        t0 = now_sec(self)
        while rclpy.ok() and not rf.done():
            if gh.is_cancel_requested or now_sec(self) - t0 > self.get_parameter('goal_timeout_sec').value:
                handle.cancel_goal_async()
                self.stop()
                deadline = time.monotonic() + 2.0
                while rclpy.ok() and not rf.done() and time.monotonic() < deadline:
                    time.sleep(0.05)
                return False, 'canceled/timeout' if rf.done() else 'cancel pending'
            time.sleep(0.05)
        from action_msgs.msg import GoalStatus
        r = rf.result()
        return (r is not None and r.status == GoalStatus.STATUS_SUCCEEDED), f'status={getattr(r, "status", "?")}'


def main():
    rclpy.init()
    spin_single(NavBridge())
