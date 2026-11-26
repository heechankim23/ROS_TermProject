"""F7 / F8: CollectScooter Action 서버.

NAVIGATING : nav_bridge 로 킥보드에서 approach_standoff(1.6 m) 떨어진 곳까지 이동
VERIFYING  : 태그(ID = 킥보드 ID) 검출 대기, 없으면 제자리 회전. 10초 안에 못 찾으면 실패
ALIGNING   : 검출한 태그로 킥보드 중심을 추정하고, nav_bridge(GoToPose, standoff=align_distance 1.3 m)로
             중심 둘레의 '빈 자리'까지 경로 주행해 중심을 바라본다. 자리 선택·경로 모두 코스트맵
             (정적 지도 + 라이다)을 써서 벽·다른 킥보드·이 킥보드의 넘어진 기둥에 부딪히지 않는다.
             로봇팔은 손목을 돌려 킥보드 방향에 맞추므로 어느 쪽에서 접근해도 된다.
             이미 팔이 닿는 거리면 정렬 주행을 생략한다. 이동이 실패하면 먼저 align_wait_sec 동안 멈춰 기다렸다
             같은 자리로 다시 간다 (사람·수레처럼 지나가는 장애물). 그래도 실패(또는 도착했지만 팔이 안 닿는 먼 자리)하면
             고정 장애물(나무·구조물)로 보고 반경(align_distance)은 그대로 두고 각도만 바꾼 자리로 다시 간다 (align_retries 번까지,
             처음 실패한 자리 기준 align_retry_offsets_deg 순서. 0 이면 바로 실패). 실패·시간 초과여도
             팔이 닿는 거리(align_distance + arm_reach_margin 이내)면 그대로 적재한다.
             align_spot_mode: dock (robot_params.yaml) 이면 적재 자리 앞 대기점까지 간 뒤 직선 후진으로 도킹한다
             (align_by_dock). 대기점이 없거나 load_mode arm 이면 아래 scan 방식.
             align_spot_mode: scan (기본) 이면 자리를 collector 가 직접 고른다 — 정적 지도(nav_bridge 의 map_yaml)
             + 진입 금지 구역 + 최근 라이다 점으로 팔이 닿는 띠(align_spot_radii) 위 후보를 검사(몸체·팔 경로·갈 수 있음)·
             채점해 가장 좋은 자리로 보내고, 실패하면 기다렸다 다시 채점 -> 실패한 자리에서 떨어진 다음 후보. 후보가 없으면
             align_blocked_wait_sec 동안 1초마다 다시 채점하고, 그래도 없으면 실패 (collector_logic.plan_align_spots).
             load_mode: reverse (기본, SCOOTOVER v19) 이면 자리는 하나다 — 킥보드와 같은 방향으로 킥보드 앞에 서서
             뒤 램프를 킥보드 앞바퀴에 댄다 (collector_logic.reverse_entry_pose, 다음 레인 = /arm_loader/next_lane).
             킥보드 방향은 태그 면 ID(면마다 다름)로 안다. 후보 = 그 자리 + 진행 방향으로 조금 더 먼 자리(entry_extra).
LOADING    : 로봇팔(arm_loader, sim:=gz) 이 발판을 집어 적재함 칸에 싣는다 -> 적재 +1, /scooter_loaded 발행
             arm_loader 가 없으면(kinematic 모드 등) load_time_sec 정지로 대신한다
"""
import math
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rcl_interfaces.srv import GetParameters
from rclpy.parameter import parameter_value_to_python
from sensor_msgs.msg import LaserScan
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Int8, String

from kickboard_interfaces.action import CollectScooter, GoToPose, LoadScooter
from kickboard_interfaces.msg import RobotStatus, TagDetectionArray

from .collector_logic import (ARM_REACH_NEAR, ARM_REACH_SKIP_FAR, BODY_HALF_W, BODY_X, ENTRY_DIST, SpotGrid,
                              align_next_step, align_pose, dock_errors, dock_ok, dock_step, entry_error, fill_static, fine_align_plan, next_spot,
                              plan_align_spots, retry_angles, reverse_entry_pose, scooter_from_face, split_tag_id,
                              within_reach, wrap_angle)
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
        self.declare_parameter('align_distance', 1.35)        # 로봇 중심 -> 킥보드 중심 (팔이 발판에 닿는 거리, load_mode arm)
        self.declare_parameter('load_mode', 'reverse')        # reverse = v19 뒤 램프 후진 적재, arm = 예전 로봇팔(어느 방향이든)
        self.declare_parameter('entry_extra', [0.0, 0.15, 0.30])    # reverse: 적재 자리 후보 (진행 방향 추가 거리 m)
        self.declare_parameter('entry_tol', [0.35, 0.15, 10.0])     # reverse: 적재 가능 오차 (앞뒤 m, 옆 m, yaw 도)
        # align_spot_mode dock: 대기점 = 적재 자리에서 진행 방향으로 이만큼 더 앞 (앞의 것부터). 거기서 직선 후진
        self.declare_parameter('dock_staging', [1.5, 1.0, 2.0])
        self.declare_parameter('dock_tries', 3)
        # reverse: 정렬 이동이 끝난 뒤 적재 자리에 이 오차 안으로 직접 맞춘다 (회전 + 직진, collector_logic.fine_align_plan).
        # nav_bridge 도착 판정(경로 끝 0.24 m, 끝 회전 뒤 위치 재확인 없음)이 적재 옆 오차(0.15 m)보다 느슨해서 필요
        self.declare_parameter('align_pos_tol', 0.05)
        self.declare_parameter('align_yaw_tol_deg', 3.0)
        self.declare_parameter('align_lat_tol', 0.08)         # 옆 오차 허용 (맞추려면 제자리 회전 2번, 적재 한계 0.15 m)
        self.declare_parameter('fine_align_max_m', 0.6)       # 이보다 멀면 직접 맞추지 않음 (정렬 이동 실패로 처리)
        self.declare_parameter('align_timeout_sec', 90.0)     # 정렬 자리까지 경로 주행 포함 (시도 1번당)
        # 정렬 이동 실패 시 1단계: 이만큼(초) 멈춰 기다렸다 같은 자리로 다시 (지나가는 사람·수레). 0 = 바로 2단계
        self.declare_parameter('align_wait_sec', 5.0)
        # 2단계: 같은 반경(align_distance), 다른 각도의 자리로 다시 시도 (나무·구조물 같은 고정 장애물). 0 = 안 함
        self.declare_parameter('align_retries', 2)
        self.declare_parameter('align_retry_offsets_deg', [90.0, -90.0, 180.0])   # 처음 실패한 자리 각도 기준 후보 순서
        self.declare_parameter('align_retry_min_sep_deg', 45.0)   # 이미 시도한 각도와 이보다 가까운 후보는 건너뜀
        # 정렬 자리 고르기: 'scan' = collector 가 정적 지도 + 진입 금지 구역 + 라이다로 팔이 닿는 띠 위 후보를 검사·채점해
        # 정확한 자리로 보냄 (재시도도 다음 후보, 후보가 없으면 바로 실패). 'nav_bridge' = 예전 방식 (standoff 원, 고정 각도 재시도)
        self.declare_parameter('align_spot_mode', 'scan')
        # 후보 반경: 팔이 닿는 1.10 ~ 1.80 안, 바깥은 도착 오차·중심 추정 오차 몫으로 0.2 m 남김 (1.75 는 도착 뒤 'out of arm reach' 가 났음)
        self.declare_parameter('align_spot_radii', [1.15, 1.30, 1.45, 1.60])
        self.declare_parameter('align_spot_step_deg', 10.0)   # 후보 각도 간격
        self.declare_parameter('align_spot_scan_sec', 1.5)    # 이 시간 동안의 라이다 점을 모아 쓴다
        self.declare_parameter('align_spot_retry_min_dist', 0.6)   # 다음 후보는 실패한 자리에서 이만큼(m) 이상 떨어진 곳
        self.declare_parameter('align_blocked_wait_sec', 20.0)   # 후보가 없을 때 1초마다 다시 채점하며 기다리는 최대 시간
        self.declare_parameter('load_time_sec', 3.0)
        # 팔 없는 적재(load_time_sec)에서 취소를 받는 앞부분 비율. 실제 팔은 REACH 4 s + DESCEND 2.5 s 뒤 GRIP (전체 21.8 s 중 0.3).
        # 1.0 = 예전처럼 끝까지 취소 가능
        self.declare_parameter('load_cancel_frac', 0.3)
        self.declare_parameter('scooter_half_depth', 0.2)    # 태그 면 -> 킥보드 중심 (근사)
        self.declare_parameter('detection_max_age_sec', 0.6)
        self.declare_parameter('arm_timeout_sec', 120.0)
        self.declare_parameter('arm_reach_margin', 0.45)     # 정렬 뒤 킥보드까지 align_distance + 이 값보다 멀면 팔이 안 닿음
        self.declare_parameter('early_detect_margin', 0.2)   # 이동 중 태그가 standoff+margin 안에서 보이면 도착 처리
        # 확장 탐색: 제자리 회전으로 못 찾으면 목표 둘레(3.5 m 간격 원) 지점으로 옮겨 가며 다시 회전 (0 = 안 함).
        # GPS 오차가 카메라 거리(3 m)보다 큰 신고 위치용 — 캠퍼스 맵에서 켠다
        self.declare_parameter('search_radius_m', 0.0)

        self.cb = ReentrantCallbackGroup()
        self.pose = PoseTracker(self)
        self.dets = {}            # tag id -> (t, TagDetection)
        self.load_count = 0
        self.create_subscription(TagDetectionArray, '/tag_detections', self.on_dets, 10, callback_group=self.cb)
        self.scans = deque(maxlen=30)      # (수신 시각, LaserScan)
        self.create_subscription(LaserScan, '/scan', lambda m: self.scans.append((now_sec(self), m)), 10,
                                 callback_group=self.cb)
        self.nav_params_cli = self.create_client(GetParameters, '/nav_bridge/get_parameters', callback_group=self.cb)
        self.static_maps = None            # 자리 채점용 지도 (처음 쓸 때 nav_bridge 파라미터로 불러옴)
        self.create_subscription(RobotStatus, '/robot_status',
                                 lambda m: setattr(self, 'load_count', m.load_count), 10, callback_group=self.cb)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.loaded_pub = self.create_publisher(String, '/scooter_loaded', 10)
        self.nav = ActionClient(self, GoToPose, '/nav_bridge/go_to_pose', callback_group=self.cb)
        self.loader = ActionClient(self, LoadScooter, '/arm_loader/load', callback_group=self.cb)
        self.next_lane, self.entry = None, None  # 다음 적재 레인 (arm_loader), 지금 정렬 중인 (킥보드 방향, 레인)
        self.create_subscription(Int8, '/arm_loader/next_lane', lambda m: setattr(self, 'next_lane', m.data),
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
                                 callback_group=self.cb)
        ActionServer(self, CollectScooter, '/collect_scooter', self.execute,
                     goal_callback=lambda g: GoalResponse.ACCEPT,
                     cancel_callback=lambda g: CancelResponse.ACCEPT, callback_group=self.cb)

    def on_dets(self, msg):
        t = now_sec(self)
        for d in msg.detections:
            base, face = split_tag_id(d.id)      # 면마다 ID 가 다름 -> 킥보드 ID 로 모으고 면 번호를 같이 둠
            self.dets[base] = (t, d, face)

    def fresh(self, tag):
        e = self.fresh_face(tag)
        return e[0] if e else None

    def fresh_face(self, tag):
        e = self.dets.get(tag)
        if e and now_sec(self) - e[0] <= self.get_parameter('detection_max_age_sec').value:
            return e[1], e[2]
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
        self.goal_xy = (gh.request.target_pose.pose.position.x, gh.request.target_pose.pose.position.y)
        ng = GoToPose.Goal(target_pose=gh.request.target_pose,
                           standoff=float(self.get_parameter('approach_standoff').value))
        navigating = [True]      # 취소한 뒤 늦게 도착한 nav 피드백이 상태를 NAVIGATING 으로 되돌리지 않게

        def nav_fb(m):
            if navigating[0]:
                self.fb(gh, 'NAVIGATING', m.feedback.distance_remaining)
        nh = future_wait(self, self.nav.send_goal_async(ng, feedback_callback=nav_fb), 5.0)
        if nh is None or not nh.accepted:
            return self.finish(gh, False, 'nav goal rejected')
        rf = nh.get_result_async()
        early = self.get_parameter('approach_standoff').value + self.get_parameter('early_detect_margin').value
        arrived_early = False
        while rclpy.ok() and not rf.done():
            if gh.is_cancel_requested:
                self.cancel_nav(nh)
                return self.finish(gh, False)
            d = self.fresh(tag)
            if d is not None and d.distance <= early:
                # PRD: '도착 후 카메라가 태그를 검출하면 접근 완료' — GPS 오차로 목표가 킥보드에 너무 붙어 있어도 여기서 멈춤
                self.get_logger().info(f'이동 중 태그 {tag} 검출 ({d.distance:.2f} m) -> 접근 완료')
                self.cancel_nav(nh)      # 취소가 끝나기 전에 다음 목표를 보내면 nav_bridge 가 거절함
                arrived_early = True
                break
            sleep_sim(self, 0.1)
        if not arrived_early:
            if rf.result() is None:
                return self.finish(gh, False, 'nav_failed: no result')
            nres = rf.result().result
            if not nres.success:
                return self.finish(gh, False, f'nav_failed: {nres.message}')
        navigating[0] = False
        self.twist()

        # 2) VERIFYING — 회전 탐색
        self.fb(gh, 'VERIFYING')
        t0 = now_sec(self)
        w = self.get_parameter('search_angular_speed').value
        found = self.spin_search(gh, tag, w)
        if found is None:
            return self.finish(gh, False)
        if not found:
            found = self.wide_search(gh, tag, w)
            if found is None:
                return self.finish(gh, False)
            if not found:
                return self.finish(gh, False, 'verify_failed: tag not found')
        self.twist()

        # 3) ALIGNING — 검출값으로 킥보드 중심을 추정 -> nav_bridge 로 '팔이 닿는 빈 자리'까지 경로 주행.
        #    자리 선택과 경로 모두 코스트맵(정적 지도 + 라이다)을 써서 다른 킥보드·벽·이 킥보드의
        #    넘어진 기둥까지 피한다. 로봇팔은 손목을 돌려 맞추므로 어느 방향에서 다가가도 된다.
        self.fb(gh, 'ALIGNING')
        reverse = self.get_parameter('load_mode').value == 'reverse'
        # arm_loader 가 없으면(kinematic) 실은 수로: 0 대 -> 왼쪽, 1 대 -> 오른쪽 (mission_manager_campus.sync_lanes 와 같음)
        lane = max(self.next_lane, 0) if self.next_lane is not None else min(self.load_count, 1)
        dist = ENTRY_DIST if reverse else self.get_parameter('align_distance').value
        half = self.get_parameter('scooter_half_depth').value
        centers = []

        def collect():
            e = self.fresh_face(tag)
            if e is not None:
                d, face = e
                ty = yaw_from_quat(d.pose.orientation)
                if reverse:     # 면 번호로 킥보드 진행 방향까지
                    centers.append(scooter_from_face(d.pose.position.x, d.pose.position.y, ty, face))
                else:
                    centers.append((d.pose.position.x - half * math.cos(ty), d.pose.position.y - half * math.sin(ty),
                                    ty))

        t0 = now_sec(self)
        while now_sec(self) - t0 < 1.0:              # 1초 모아서 평균 (검출 노이즈)
            collect()
            sleep_sim(self, 0.05)
        if not centers:
            return self.finish(gh, False, 'align_failed: tag lost')
        cx = sum(c[0] for c in centers) / len(centers)
        cy = sum(c[1] for c in centers) / len(centers)
        hd = math.atan2(sum(math.sin(c[2]) for c in centers), sum(math.cos(c[2]) for c in centers))
        self.entry = (hd, lane) if reverse else None
        if reverse:
            self.get_logger().info(f'킥보드 {sid}: 방향 {math.degrees(hd):.0f}° -> '
                                   f'{"왼" if lane == 0 else "오른"}쪽 레인 후진 적재 자리로')
        target = PoseStamped()
        target.header.frame_id = 'map'
        target.pose.position.x, target.pose.position.y = cx, cy
        target.pose.orientation.w = 1.0
        margin = self.get_parameter('arm_reach_margin').value

        def reach_dist():
            pose = self.pose.get()
            return None if pose is None else math.hypot(cx - pose[0], cy - pose[1])

        def entry_ok(far=None):
            # v19: 뒤 램프가 킥보드 앞바퀴에 닿는 자리인가 (앞뒤·옆·방향 오차). far 가 있으면(정렬 생략 판단) 더 좁게
            pose = self.pose.get()
            if pose is None:
                return False, None
            ea, el, ey = entry_error(pose, cx, cy, hd, lane)
            ta, tl, ty = self.get_parameter('entry_tol').value
            if far is not None:
                ta, tl, ty = ta / 2, tl / 2, ty / 2
            return abs(ea) <= ta and abs(el) <= tl and abs(math.degrees(ey)) <= ty, (ea, el, math.degrees(ey))

        def in_reach(far=None):
            if reverse:
                return entry_ok(far)[0]
            # 로봇팔은 손목을 돌려 맞추므로 방향과 무관하게 거리만 맞으면 실을 수 있다
            r = reach_dist()
            return r is not None and within_reach(r, dist, ARM_REACH_NEAR, margin if far is None else far)

        def too_far(status, msg):
            if reverse and status == 'ok':
                # 도착 판정이 느슨하므로 '팔이 안 닿음' 판단 전에 적재 자리로 직접 맞춘다
                if self.fine_align(gh, sid, cx, cy, hd, lane) is None:
                    return 'canceled', 'canceled'
            # 이동은 성공했는데 nav_bridge 가 먼 자리를 골랐으면(좁은 곳: 주차 차량 사이, 진입 금지 구역 안)
            # 팔이 닿지 않는다 -> 이동 실패로 보고 다른 각도 자리로 다시 시도한다
            if reverse:
                ok, e = entry_ok()
                if status == 'ok' and not ok and e is not None:
                    return 'failed', f'out of arm reach (적재 자리 오차 앞뒤 {e[0]:.2f} m, 옆 {e[1]:.2f} m, {e[2]:.0f}°)'
                return status, msg
            r = reach_dist()
            if status == 'ok' and r is not None and r > dist + margin:
                return 'failed', f'out of arm reach ({r:.2f} m)'
            return status, msg

        if in_reach(ARM_REACH_SKIP_FAR):
            # 이미 팔이 닿는 거리 -> 정렬 주행 생략. (킥보드를 정면에 둔 채 반대편 자리로 돌아가려다
            # 앞 장애물 정지에 걸려 맴도는 경우가 있었음: k16 #21 align_timeout)
            self.get_logger().info(f'킥보드 {sid}: 이미 팔이 닿는 거리 -> 정렬 주행 생략')
        else:
            mode = self.get_parameter('align_spot_mode').value
            if mode == 'dock' and reverse:
                status, msg, more = self.align_by_dock(gh, sid, tag, cx, cy, hd, lane, dist, collect, in_reach, too_far)
            elif mode in ('scan', 'dock'):
                status, msg, more = self.align_by_scan(gh, sid, tag, cx, cy, dist, collect, in_reach, too_far)
            else:
                status, msg, more = self.align_by_nav_bridge(gh, sid, tag, target, cx, cy, dist, collect, in_reach,
                                                             too_far)
            if status == 'canceled':
                return self.finish(gh, False)
            if status == 'rejected':
                return self.finish(gh, False, 'align_failed: nav goal rejected')
            if status != 'ok':
                if in_reach():
                    self.get_logger().warn(f'킥보드 {sid}: 정렬 자리까지는 못 갔지만 팔이 닿는 거리 -> 그대로 적재')
                elif status == 'timeout':
                    self.twist()
                    return self.finish(gh, False, 'align_timeout')
                else:
                    return self.finish(gh, False, f'align_failed: {msg}{more}')
        self.twist()
        if reverse:
            # 정렬 생략·'그대로 적재' 경우도 포함해 적재 직전에 자리를 한 번 더 맞추고 다시 확인한다
            if self.fine_align(gh, sid, cx, cy, hd, lane) is None:
                return self.finish(gh, False)
            ok, e = entry_ok()
            if not ok:
                detail = f' (앞뒤 {e[0]:.2f} m, 옆 {e[1]:.2f} m, {e[2]:.0f}°)' if e is not None else ''
                return self.finish(gh, False, f'align_failed: out of arm reach after fine align{detail}')

        corrected = PoseStamped()
        corrected.header.frame_id = 'map'
        corrected.header.stamp = self.get_clock().now().to_msg()
        recent = centers[-20:]
        corrected.pose.position.x = sum(c[0] for c in recent) / len(recent)
        corrected.pose.position.y = sum(c[1] for c in recent) / len(recent)
        q = quat_from_yaw(recent[-1][2])
        o = corrected.pose.orientation
        o.x, o.y, o.z, o.w = q

        # 4) LOADING — 팔이 잡기 시작하면(그리퍼가 닫히면) 취소를 받지 않는다
        if gh.is_cancel_requested:
            return self.finish(gh, False)
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
        # 로봇팔 없음 — load_time_sec 정지 = 1대 적재. 앞쪽 load_cancel_frac 만큼(팔이 다가가 내려가는 동안)만 취소를 받는다
        t0, T = now_sec(self), self.get_parameter('load_time_sec').value
        grip = T * self.get_parameter('load_cancel_frac').value
        while now_sec(self) - t0 < T:
            if gh.is_cancel_requested and now_sec(self) - t0 < grip:
                return self.finish(gh, False)
            self.twist()
            sleep_sim(self, 0.1)
        self.loaded_pub.publish(String(data=sid))
        self.fb(gh, 'LOADING', 0.0, self.load_count + 1)
        self.get_logger().info(f'킥보드 {sid} 적재 완료')
        return self.finish(gh, True, '', corrected, ignore_cancel=True)

    # ---------------- 정렬 자리 채점 (align_spot_mode: scan) ----------------
    def load_static_maps(self):
        """nav_bridge 가 쓰는 지도 경로·로봇 치수를 받아 지도를 한 번 불러 둔다. 실패하면 라이다만 쓴다."""
        if self.static_maps is not None:
            return self.static_maps
        names = ['map_yaml', 'physical_map_yaml', 'keepout_goal_clearance', 'inscribed_radius', 'scan_offset_x',
                 'footprint']
        info = {'nav': None, 'phys': None, 'keep_clear': None, 'inscribed': 0.47, 'scan_off': 0.80,
                'footprint': [-0.98, 0.96, 0.58]}
        vals = {}
        if self.nav_params_cli.wait_for_service(timeout_sec=2.0):
            res = future_wait(self, self.nav_params_cli.call_async(GetParameters.Request(names=names)), 3.0)
            if res is not None:
                vals = {n: parameter_value_to_python(v) for n, v in zip(names, res.values)}
        from .occupancy import OccupancyMap

        def load(path):
            if not path:
                return None
            m = OccupancyMap(path)
            return m.grid, m.ox, m.oy, m.res
        try:
            info['nav'] = load(vals.get('map_yaml'))
            info['phys'] = load(vals.get('physical_map_yaml'))
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'자리 채점용 지도 불러오기 실패 ({exc}) -> 라이다만 사용')
        kc = vals.get('keepout_goal_clearance')
        info['keep_clear'] = kc if isinstance(kc, float) and kc >= 0 else None
        for k, n in (('inscribed', 'inscribed_radius'), ('scan_off', 'scan_offset_x'), ('footprint', 'footprint')):
            if vals.get(n) is not None:
                info[k] = vals[n]
        self.static_maps = info
        self.get_logger().info(f'자리 채점 지도: 주행 {"있음" if info["nav"] else "없음"}, '
                               f'물리 {"있음" if info["phys"] else "없음"}, 진입 금지 여유 {info["keep_clear"]}')
        return info

    def recent_scan_points(self, info):
        """최근 align_spot_scan_sec 동안의 라이다 점 (지도 좌표). 차체에 찍힌 점·최대 거리(맞은 것 없음)는 뺀다."""
        horizon = self.get_parameter('align_spot_scan_sec').value
        now = now_sec(self)
        fx0, fx1, fy = info['footprint']
        xs, ys = [], []
        for t, m in list(self.scans):
            if now - t > horizon or not m.ranges:
                continue
            pose = self.pose.get_at(m.header.stamp) or self.pose.get()
            if pose is None:
                continue
            r = np.asarray(m.ranges, dtype=float)
            a = m.angle_min + m.angle_increment * np.arange(len(r))
            hit = np.isfinite(r) & (r >= m.range_min) & (r < m.range_max - 0.05)
            px, py = info['scan_off'] + r[hit] * np.cos(a[hit]), r[hit] * np.sin(a[hit])
            own = (px > fx0) & (px < fx1) & (np.abs(py) < fy)
            px, py = px[~own], py[~own]
            c, s_ = math.cos(pose[2]), math.sin(pose[2])
            xs.append(pose[0] + px * c - py * s_)
            ys.append(pose[1] + px * s_ + py * c)
        if not xs:
            return np.zeros(0), np.zeros(0)
        return np.concatenate(xs), np.concatenate(ys)

    def plan_spots(self, cx, cy, dist, radii=None):
        """-> (후보 목록, 탈락 사유별 개수). 후보가 없으면 빈 목록."""
        info = self.load_static_maps()
        pose = self.pose.get()
        if pose is None:
            return [], {'no_pose': 1}
        grid = SpotGrid(cx, cy, half=6.0, res=0.1)
        fill_static(grid, info['nav'], info['phys'])
        grid.add_points(*self.recent_scan_points(info))
        if radii is None:
            radii = list(self.get_parameter('entry_extra' if self.entry else 'align_spot_radii').value)
        return plan_align_spots(grid, cx, cy, pose[:2], dist, radii,
                                step_deg=self.get_parameter('align_spot_step_deg').value,
                                inscribed=float(info['inscribed']), keep_clear=info['keep_clear'], entry=self.entry)

    # ---------------- 후진 도킹 (align_spot_mode: dock) ----------------
    def align_by_dock(self, gh, sid, tag, cx, cy, hd, lane, dist, collect, in_reach, too_far):
        """대기점(적재 자리 앞)까지 경로 주행 -> 직선 후진으로 들어가며 옆·방향 오차를 계속 보정.
        어긋난 채 끝나면 대기점까지 직진해 빠졌다가 다시. 대기점이 하나도 안 비면 예전 방식(scan)."""
        target = reverse_entry_pose(cx, cy, hd, lane)
        staging = [float(e) for e in self.get_parameter('dock_staging').value]
        cands, why = self.plan_spots(cx, cy, dist, radii=staging)
        extra = None
        for e in staging:                                   # 설정한 순서대로 첫 번째 빈 대기점
            x, y, _ = reverse_entry_pose(cx, cy, hd, lane, e)
            if any(math.hypot(c['x'] - x, c['y'] - y) < 0.05 for c in cands):
                extra = e
                break
        if extra is None:
            self.get_logger().warn(f'킥보드 {sid}: 도킹 대기점 없음 (탈락 {why}) -> 자리 채점 방식으로')
            return self.align_by_scan(gh, sid, tag, cx, cy, dist, collect, in_reach, too_far)
        x, y, yaw = reverse_entry_pose(cx, cy, hd, lane, extra)
        self.get_logger().info(f'킥보드 {sid}: 도킹 대기점 ({x:.2f}, {y:.2f}) = 적재 자리 앞 {extra:.1f} m')
        status, msg = self.align_drive(gh, self.spot_goal({'x': x, 'y': y, 'yaw': yaw}), cx, cy, dist, collect)
        if status != 'ok':
            return status, msg, ' (도킹 대기점까지 못 감)'
        pose = self.pose.get()
        if pose is None or abs(dock_errors(pose, target)[1]) > 0.6:
            return 'failed', 'staging pose off line', ''
        tries = int(self.get_parameter('dock_tries').value)
        for k in range(tries):
            if not self.turn_to(gh, yaw):                   # 대기점에서 방향부터 (후진 전 한 번)
                return ('canceled', 'canceled', '') if gh.is_cancel_requested else ('failed', 'turn timeout', '')
            r = self.dock_drive(gh, target, 0.0, collect)
            if r is None:
                return 'canceled', 'canceled', ''
            pose = self.pose.get()
            if pose is None:
                return 'failed', 'pose unavailable', ''
            ea, el, ey = dock_errors(pose, target)
            self.get_logger().info(f'킥보드 {sid}: 후진 도킹 {k + 1}/{tries} -> 앞뒤 {ea:.2f} m, 옆 {el:.2f} m, '
                                   f'{math.degrees(ey):.0f}°')
            if r and dock_ok(pose, target):
                self.turn_to(gh, yaw)
                return 'ok', '', ''
            if k + 1 < tries and not self.dock_drive(gh, target, extra, collect):
                break
        return 'failed', 'dock failed', f' ({tries}회 시도)'

    def dock_drive(self, gh, target, goal_along, collect, timeout=60.0):
        """적재 자리 직선 위에서 goal_along 까지. True 도착, False 막힘·시간 초과, None 취소"""
        t0, checked = now_sec(self), -1e9
        while rclpy.ok():
            if gh.is_cancel_requested:
                self.twist()
                return None
            pose = self.pose.get()
            if pose is None or now_sec(self) - t0 > timeout:
                break
            v, w, done = dock_step(pose, target, goal_along)
            if done:
                self.twist()
                return True
            if now_sec(self) - checked > 0.3:
                checked = now_sec(self)
                rem = goal_along - dock_errors(pose, target)[0]        # 남은 직진 거리 (음수 = 후진)
                if not self.path_clear(math.copysign(min(abs(rem), 0.6), rem)):
                    self.get_logger().warn(f'도킹: {"앞" if rem > 0 else "뒤"}쪽에 장애물 -> 멈춤')
                    break
            collect()
            self.twist(v, w)
            sleep_sim(self, 0.05)
        self.twist()
        return False

    def spot_goal(self, spot):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x, goal.pose.position.y = spot['x'], spot['y']
        q = quat_from_yaw(spot['yaw'])
        goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w = q
        return GoToPose.Goal(target_pose=goal, standoff=0.0)

    def align_by_scan(self, gh, sid, tag, cx, cy, dist, collect, in_reach, too_far):
        """채점한 자리로 정렬. -> (상태, 메시지, 최종 실패 시 덧붙일 설명).
        1) 가장 좋은 자리로 간다.
        2) 실패하면 기다렸다(align_wait_sec) 다시 채점해 가장 좋은 자리로 (지나간 장애물이면 보통 같은 자리가 다시 1등).
        3) 그래도 실패하면 실패한 자리에서 align_spot_retry_min_dist 이상 떨어진 다음 후보 (align_retries 번).
        후보가 없으면 align_blocked_wait_sec 동안 1초마다 다시 채점하며 기다린다 (자리에 선 사람이 비켜나길)."""
        retries = int(self.get_parameter('align_retries').value)
        wait_sec = self.get_parameter('align_wait_sec').value
        min_dist = self.get_parameter('align_spot_retry_min_dist').value
        blocked_wait = self.get_parameter('align_blocked_wait_sec').value
        failed = []                      # 이동이 실패한 자리 (x, y)
        driven = []                      # 가 본 자리 (x, y), 겹치지 않게
        notes = []

        def log_plan(cands, why, head):
            top = ', '.join(f'{math.degrees(c["a"]):.0f}°/{c["r"]:.2f}m' for c in cands[:3])
            self.get_logger().info(f'킥보드 {sid}: {head} 자리 후보 {len(cands)}곳 [{top}] 탈락 {why}')

        def pick(head):
            """후보를 채점해 고른다. 없으면 1초마다 다시 채점하며 기다린다. -> 자리 | None | 'canceled'"""
            cands, why = self.plan_spots(cx, cy, dist)
            log_plan(cands, why, head)
            spot = next_spot(cands, failed, min_dist)
            if spot is None and blocked_wait > 0:
                self.get_logger().warn(f'킥보드 {sid}: 팔이 닿는 자리 후보 없음 -> 멈춰 기다렸다 다시 채점 '
                                       f'(최대 {blocked_wait:.0f}초, 1초마다. 지나가는 장애물이면 비켜남)')
                t0 = now_sec(self)
                while spot is None and now_sec(self) - t0 < blocked_wait:
                    if self.wait_still(gh, tag, 1.0, collect) is None:
                        return 'canceled'
                    cands, why = self.plan_spots(cx, cy, dist)
                    spot = next_spot(cands, failed, min_dist)
                notes.append(f'{now_sec(self) - t0:.0f}초 기다리며 다시 채점')
                log_plan(cands, why, f'기다린 뒤({now_sec(self) - t0:.0f}초)')
            return spot

        def drive(sp):
            if all(math.hypot(sp['x'] - x, sp['y'] - y) >= min_dist for x, y in driven):
                driven.append((sp['x'], sp['y']))
            return too_far(*self.align_drive(gh, self.spot_goal(sp), cx, cy, dist, collect))

        def mark_failed(sp, why):
            # 도착은 했는데 팔이 조금 안 닿음 -> 그 자리 근처(0.25 m)만 빼서 같은 방향의 더 가까운 자리를 남긴다
            # (reverse 후보는 0.15 m 간격이라 0.1 m 만 빼야 옆 후보가 같이 빠지지 않는다)
            near = 0.1 if self.entry else 0.25
            failed.append((sp['x'], sp['y'], near if why.startswith('out of arm reach') else min_dist))

        def note(head=''):
            parts = ([head] if head else []) + ([f'자리 {len(driven)}곳 시도'] if len(driven) > 1 else []) + notes
            return f' ({", ".join(parts)})' if parts else ''

        spot = pick('정렬')
        if spot == 'canceled':
            return 'canceled', 'canceled', ''
        if spot is None:
            return 'failed', 'no reachable arm spot', note('주변 장애물로 팔이 닿는 자리 없음')
        status, msg = drive(spot)
        if align_next_step(False, status, msg, wait_sec) == 'wait_same' and not in_reach():
            self.get_logger().warn(f'정렬 이동 실패 ({msg}) -> {wait_sec:.0f}초 멈춰 기다렸다 다시 채점 '
                                   f'(지나가는 장애물이면 같은 자리가 다시 1등)')
            seen = self.wait_still(gh, tag, wait_sec, collect)
            if seen is None:
                return 'canceled', 'canceled', ''
            prev, prev_msg = spot, msg
            spot = pick('기다린 뒤')
            if spot == 'canceled':
                return 'canceled', 'canceled', ''
            if spot is None:
                return status, msg, note()
            same = math.hypot(spot['x'] - prev['x'], spot['y'] - prev['y']) < min_dist
            status, msg = drive(spot)
            if status == 'ok':
                how = '같은 자리 성공 -> 지나가는 장애물이었음' if same else '다른 자리 성공'
                self.get_logger().info(f'킥보드 {sid}: 기다린 뒤 {how}')
            elif status == 'failed':
                notes.append('기다린 뒤에도 실패')
                if same:
                    self.get_logger().warn(f'킥보드 {sid}: 기다린 뒤에도 같은 자리 실패 ({msg}) -> 고정 장애물로 보고 '
                                           f'다른 자리 (기다리는 동안 태그 보임 {seen:.0%})')
                else:
                    mark_failed(prev, prev_msg)
        while status == 'failed' and not in_reach() and len(failed) <= retries:
            mark_failed(spot, msg)
            if len(failed) > retries:
                break
            spot = pick(f'다시 채점 ({len(failed)}곳 실패 뒤)')
            if spot == 'canceled':
                return 'canceled', 'canceled', ''
            if spot is None:
                notes.append('남은 후보 없음')
                break
            self.get_logger().warn(f'정렬 이동 실패 ({msg}) -> 다음 후보 {math.degrees(spot["a"]):.0f}° / '
                                   f'{spot["r"]:.2f} m 자리로 다시 시도 {len(failed)}/{retries}')
            status, msg = drive(spot)
        return status, msg, note()

    def align_by_nav_bridge(self, gh, sid, tag, target, cx, cy, dist, collect, in_reach, too_far):
        """예전 방식 (align_spot_mode: nav_bridge). -> (상태, 메시지, 최종 실패 시 덧붙일 설명)."""
        # 첫 시도: 자리는 nav_bridge 가 코스트맵(정적 지도 + 라이다)으로 고른다 (standoff)
        first = GoToPose.Goal(target_pose=target, standoff=float(dist))
        status, msg = too_far(*self.align_drive(gh, first, cx, cy, dist, collect))
        # 1단계 — 실패했고 팔도 안 닿으면: 멈춰 기다렸다 같은 자리로 다시. 되면 지나가는 장애물(사람·수레),
        # 또 막히면 고정 장애물(나무·구조물)로 보고 2단계(각도 변경)로 간다.
        wait_sec = self.get_parameter('align_wait_sec').value
        waited = ''
        if align_next_step(False, status, msg, wait_sec) == 'wait_same' and not in_reach():
            self.get_logger().warn(f'정렬 이동 실패 ({msg}) -> {wait_sec:.0f}초 멈춰 기다렸다 같은 자리로 다시 '
                                   f'(지나가는 장애물인지 확인)')
            seen = self.wait_still(gh, tag, wait_sec, collect)
            if seen is None:
                return 'canceled', 'canceled', ''
            status, msg = too_far(*self.align_drive(gh, first, cx, cy, dist, collect))
            if status == 'ok':
                self.get_logger().info(f'킥보드 {sid}: 기다린 뒤 같은 자리 성공 -> 지나가는 장애물이었음')
            elif status == 'failed':
                waited = ', 기다린 뒤 같은 자리도 실패'
                self.get_logger().warn(f'킥보드 {sid}: 기다린 뒤에도 같은 자리 실패 ({msg}) -> 고정 장애물로 보고 '
                                       f'각도 변경 (기다리는 동안 태그 보임 {seen:.0%})')
        # 2단계 — 반경 dist 는 그대로, 킥보드 중심에서 본 각도만 바꾼 자리로 직접 지정해 다시 간다.
        # 바라보는 방향이 바뀌므로 앞을 막던 물체(라이다 앞 구간)나 그 자리의 장애물을 벗어난다.
        # 시간 초과는 다시 시도하지 않는다 (시도마다 align_timeout_sec 이 더 걸림).
        retries = int(self.get_parameter('align_retries').value)
        offsets = list(self.get_parameter('align_retry_offsets_deg').value)
        min_sep = self.get_parameter('align_retry_min_sep_deg').value
        tried, base = [], None
        while status == 'failed' and not in_reach() and len(tried) - (base is not None) < retries:
            if base is None:                              # 처음 실패한 자리의 각도 (로봇이 멈춘 곳 기준)
                pose = self.pose.get()
                if pose is None:
                    break
                base = math.atan2(pose[1] - cy, pose[0] - cx)
                tried.append(base)
            cands = retry_angles(base, offsets, tried, min_sep)
            if not cands:
                break
            a = cands[0]
            tried.append(a)
            n = len(tried) - 1
            self.get_logger().warn(f'정렬 이동 실패 ({msg}) -> 반경 {dist:.2f} m 유지, 각도 {math.degrees(a):.0f}° '
                                   f'자리로 다시 시도 {n}/{retries}')
            x, y, yaw = align_pose(cx, cy, dist, a)
            goal = PoseStamped()
            goal.header.frame_id = 'map'
            goal.pose.position.x, goal.pose.position.y = x, y
            q = quat_from_yaw(yaw)
            goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w = q
            status, msg = too_far(*self.align_drive(
                gh, GoToPose.Goal(target_pose=goal, standoff=0.0), cx, cy, dist, collect))
        more = f'같은 반경 {len(tried)}곳 시도' if len(tried) > 1 else ''
        more = f' ({more}{waited})' if more else (f' ({waited[2:]})' if waited else '')
        return status, msg, more

    def align_drive(self, gh, goal, cx, cy, dist, collect):
        """정렬 자리로 한 번 이동. -> (상태, 메시지). 상태: ok / failed(nav_bridge 이동 실패) / rejected / timeout / canceled"""
        nh = future_wait(self, self.nav.send_goal_async(goal), 5.0)
        if nh is None or not nh.accepted:
            return 'rejected', 'nav goal rejected'
        rf = nh.get_result_async()
        t0 = now_sec(self)
        while rclpy.ok() and not rf.done():
            if gh.is_cancel_requested:
                self.cancel_nav(nh)
                return 'canceled', 'canceled'
            if now_sec(self) - t0 > self.get_parameter('align_timeout_sec').value:
                self.cancel_nav(nh)
                return 'timeout', 'align_timeout'
            collect()
            pose = self.pose.get()
            if pose is not None:
                self.fb(gh, 'ALIGNING', math.hypot(cx - pose[0], cy - pose[1]) - dist)
            sleep_sim(self, 0.1)
        if rf.result() is None:
            return 'failed', 'no result'
        res = rf.result().result
        return ('ok', '') if res.success else ('failed', res.message)

    def cancel_nav(self, handle, timeout=10.0):
        """nav_bridge 목표를 취소하고 끝날 때까지 기다린다 (경로 계획 중이면 늦게 끝남).
        안 기다리고 다음 목표를 보내면 nav_bridge 가 '아직 이전 목표 실행 중'으로 거절한다."""
        future_wait(self, handle.cancel_goal_async(), 5.0)
        if future_wait(self, handle.get_result_async(), timeout) is None:
            self.get_logger().warn(f'nav_bridge 취소 완료를 {timeout:.0f}초 안에 못 받음')
        self.twist()

    def fine_align(self, gh, sid, cx, cy, hd, lane):
        """reverse: 후진 적재 자리에 align_pos_tol / align_yaw_tol_deg 안으로 직접 선다 (회전 + 짧은 직진).
        -> True 맞춤, False 못 맞춤(멀거나 막힘, 지금 자리 그대로), None 취소"""
        target = reverse_entry_pose(cx, cy, hd, lane)
        tol = self.get_parameter('align_pos_tol').value
        ytol = math.radians(self.get_parameter('align_yaw_tol_deg').value)
        max_move = self.get_parameter('fine_align_max_m').value
        lat = self.get_parameter('align_lat_tol').value
        for _ in range(3):
            pose = self.pose.get()
            if pose is None:
                return False
            plan = fine_align_plan(pose, target, tol, ytol, max_move, lat)
            if plan is None:
                self.get_logger().warn(f'킥보드 {sid}: 적재 자리에서 {math.hypot(pose[0] - target[0], pose[1] - target[1]):.2f} m '
                                       f'-> 직접 맞추기엔 멂')
                return False
            if not plan:
                return True
            ea, el, ey = entry_error(pose, cx, cy, hd, lane)
            self.get_logger().info(f'킥보드 {sid}: 적재 자리 맞춤 (오차 앞뒤 {ea:.2f} m, 옆 {el:.2f} m, '
                                   f'{math.degrees(ey):.0f}°) -> ' + ', '.join(
                                       f'회전 {math.degrees(v):.0f}°' if k == 'turn' else f'직진 {v:+.2f} m'
                                       for k, v in plan))
            for kind, val in plan:
                r = self.turn_to(gh, val) if kind == 'turn' else self.drive_straight(gh, val)
                if r is None:
                    return None
                if not r:
                    return False
        pose = self.pose.get()
        return pose is not None and fine_align_plan(pose, target, tol, ytol, max_move, lat) == []

    def turn_to(self, gh, yaw, timeout=15.0):
        """제자리 회전. True 도착, False 시간 초과, None 취소"""
        t0 = now_sec(self)
        while rclpy.ok():
            if gh.is_cancel_requested:
                self.twist()
                return None
            pose = self.pose.get()
            if pose is None or now_sec(self) - t0 > timeout:
                self.twist()
                return False
            err = wrap_angle(yaw - pose[2])
            if abs(err) <= math.radians(1.0):
                self.twist()
                return True
            w = max(0.12, min(0.5, 1.5 * abs(err)))
            self.twist(0.0, math.copysign(w, err))
            sleep_sim(self, 0.05)
        self.twist()
        return False

    def path_clear(self, dist):
        """직진 dist(m, 음수 = 후진) 동안 차체가 지나갈 띠에 라이다 점이 없는가"""
        pose = self.pose.get()
        if pose is None:
            return False
        xs, ys = self.recent_scan_points(self.load_static_maps())
        if len(xs) == 0:
            return True
        c, s_ = math.cos(pose[2]), math.sin(pose[2])
        px = (xs - pose[0]) * c + (ys - pose[1]) * s_
        py = -(xs - pose[0]) * s_ + (ys - pose[1]) * c
        if dist >= 0:
            band = (px > BODY_X[1]) & (px < BODY_X[1] + dist + 0.1)
        else:
            band = (px < BODY_X[0]) & (px > BODY_X[0] + dist - 0.1)
        return not bool((band & (np.abs(py) < BODY_HALF_W)).any())

    def drive_straight(self, gh, dist, timeout=20.0):
        """지금 방향으로 dist(m, 음수 = 후진) 직진. 차체가 지나갈 띠에 장애물이 보이면 멈춤.
        True 도착, False 막힘·시간 초과, None 취소"""
        start = self.pose.get()
        if start is None:
            return False
        if not self.path_clear(dist):
            self.get_logger().warn(f'적재 자리 맞춤: {"앞" if dist >= 0 else "뒤"}쪽에 장애물 -> 직진 안 함')
            return False
        h = start[2]
        hx, hy = math.cos(h), math.sin(h)
        t0, checked = now_sec(self), now_sec(self)
        while rclpy.ok():
            if gh.is_cancel_requested:
                self.twist()
                return None
            pose = self.pose.get()
            if pose is None or now_sec(self) - t0 > timeout:
                self.twist()
                return False
            done = (pose[0] - start[0]) * hx + (pose[1] - start[1]) * hy
            rem = dist - done
            if abs(rem) <= 0.01:
                self.twist()
                return True
            if now_sec(self) - checked > 0.5:
                checked = now_sec(self)
                if not self.path_clear(rem):
                    self.twist()
                    self.get_logger().warn('적재 자리 맞춤: 직진 중 장애물 -> 멈춤')
                    return False
            v = max(0.04, min(0.12, 0.8 * abs(rem)))
            w = max(-0.3, min(0.3, 1.5 * wrap_angle(h - pose[2])))
            self.twist(math.copysign(v, rem), w)
            sleep_sim(self, 0.05)
        self.twist()
        return False

    def wait_still(self, gh, tag, sec, collect):
        """멈춰서 sec 초 기다린다. 그동안 태그가 보인 비율(0~1)을 돌려준다 (가림 판단 참고용). 취소되면 None"""
        t0 = now_sec(self)
        n = seen = 0
        while now_sec(self) - t0 < sec:
            if gh.is_cancel_requested:
                return None
            self.twist()
            collect()
            n += 1
            seen += self.fresh(tag) is not None
            sleep_sim(self, 0.1)
        return seen / n if n else 0.0

    def spin_search(self, gh, tag, w):
        """제자리 회전 탐색. 찾으면 True, 시간 초과 False, 취소 None"""
        t0 = now_sec(self)
        while self.fresh(tag) is None:
            if gh.is_cancel_requested:
                return None
            if now_sec(self) - t0 > self.get_parameter('verify_timeout_sec').value:
                self.twist()
                return False
            if now_sec(self) - t0 > 0.5:        # 도착 직후 0.5초는 정지 상태에서 확인
                self.twist(0.0, w)
            sleep_sim(self, 0.05)
        self.twist()
        return True

    def wide_search(self, gh, tag, w):
        """확장 탐색: 목표(신고 위치) 둘레 3.5 m 간격 원 위의 지점으로 옮겨 가며 회전 탐색. 찾으면 True"""
        R = self.get_parameter('search_radius_m').value
        if R <= 0:
            return False
        tp = self.goal_xy
        pose = self.pose.get()
        pts = []
        r = 3.5
        while r <= R + 0.1:
            n = max(4, int(round(2 * math.pi * r / 4.0)))
            pts += [(tp[0] + r * math.cos(2 * math.pi * k / n), tp[1] + r * math.sin(2 * math.pi * k / n)) for k in range(n)]
            r += 3.5
        self.get_logger().info(f'태그 {tag} 못 찾음 -> 확장 탐색 (반경 {R:.0f} m, {len(pts)}곳)')
        cur = pose[:2] if pose else tp
        while pts:                                         # 가까운 곳부터 차례로
            nxt = min(pts, key=lambda q: math.dist(q, cur))
            pts.remove(nxt)
            target = PoseStamped()
            target.header.frame_id = 'map'
            target.pose.position.x, target.pose.position.y = nxt
            target.pose.orientation.w = 1.0
            nh = future_wait(self, self.nav.send_goal_async(GoToPose.Goal(target_pose=target, standoff=0.6)), 5.0)
            if nh is None or not nh.accepted:
                continue
            rf = nh.get_result_async()
            while rclpy.ok() and not rf.done():
                if gh.is_cancel_requested:
                    self.cancel_nav(nh)
                    return None
                if self.fresh(tag) is not None:
                    self.cancel_nav(nh)
                    self.twist()
                    self.get_logger().info(f'확장 탐색 중 태그 {tag} 검출')
                    return True
                sleep_sim(self, 0.1)
            p = self.pose.get()
            cur = p[:2] if p else nxt
            ok = self.spin_search(gh, tag, w)
            if ok is None or ok:
                if ok:
                    self.get_logger().info(f'확장 탐색으로 태그 {tag} 검출 (신고 위치에서 {math.dist(cur, tp):.1f} m)')
                return ok
        return False

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
