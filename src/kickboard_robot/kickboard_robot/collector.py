"""F7 / F8: CollectScooter Action 서버.

NAVIGATING : nav_bridge 로 킥보드에서 approach_standoff(1.6 m) 떨어진 곳까지 이동
VERIFYING  : 태그(ID = 킥보드 ID) 검출 대기, 없으면 제자리 회전. 10초 안에 못 찾으면 실패
ALIGNING   : 검출한 태그로 킥보드 중심을 추정하고, nav_bridge(GoToPose, standoff=align_distance 1.3 m)로
             중심 둘레의 '빈 자리'까지 경로 주행해 중심을 바라본다. 자리 선택·경로 모두 코스트맵
             (정적 지도 + 라이다)을 써서 벽·다른 킥보드·이 킥보드의 넘어진 기둥에 부딪히지 않는다.
             로봇팔은 손목을 돌려 킥보드 방향에 맞추므로 어느 쪽에서 접근해도 된다.
             이미 팔이 닿는 거리면 정렬 주행을 생략한다. 이동이 실패(또는 도착했지만 팔이 안 닿는 먼 자리)하면
             반경(align_distance)은 그대로 두고 각도만 바꾼 자리로 다시 간다 (align_retries 번까지,
             처음 실패한 자리 기준 align_retry_offsets_deg 순서. 0 이면 바로 실패). 실패·시간 초과여도
             팔이 닿는 거리(align_distance + arm_reach_margin 이내)면 그대로 적재한다.
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

from .collector_logic import ARM_REACH_NEAR, ARM_REACH_SKIP_FAR, align_pose, retry_angles, within_reach
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
        self.declare_parameter('align_timeout_sec', 90.0)     # 정렬 자리까지 경로 주행 포함 (시도 1번당)
        # 정렬 이동 실패 시: 같은 반경(align_distance), 다른 각도의 자리로 다시 시도. 0 = 바로 실패
        self.declare_parameter('align_retries', 2)
        self.declare_parameter('align_retry_offsets_deg', [90.0, -90.0, 180.0])   # 처음 실패한 자리 각도 기준 후보 순서
        self.declare_parameter('align_retry_min_sep_deg', 45.0)   # 이미 시도한 각도와 이보다 가까운 후보는 건너뜀
        self.declare_parameter('load_time_sec', 3.0)
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
        cx = sum(c[0] for c in centers) / len(centers)
        cy = sum(c[1] for c in centers) / len(centers)
        target = PoseStamped()
        target.header.frame_id = 'map'
        target.pose.position.x, target.pose.position.y = cx, cy
        target.pose.orientation.w = 1.0
        margin = self.get_parameter('arm_reach_margin').value

        def reach_dist():
            pose = self.pose.get()
            return None if pose is None else math.hypot(cx - pose[0], cy - pose[1])

        def in_reach(far=None):
            # 로봇팔은 손목을 돌려 맞추므로 방향과 무관하게 거리만 맞으면 실을 수 있다
            r = reach_dist()
            return r is not None and within_reach(r, dist, ARM_REACH_NEAR, margin if far is None else far)

        def too_far(status, msg):
            # 이동은 성공했는데 nav_bridge 가 먼 자리를 골랐으면(좁은 곳: 주차 차량 사이, 진입 금지 구역 안)
            # 팔이 닿지 않는다 -> 이동 실패로 보고 다른 각도 자리로 다시 시도한다
            r = reach_dist()
            if status == 'ok' and r is not None and r > dist + margin:
                return 'failed', f'out of arm reach ({r:.2f} m)'
            return status, msg

        if in_reach(ARM_REACH_SKIP_FAR):
            # 이미 팔이 닿는 거리 -> 정렬 주행 생략. (킥보드를 정면에 둔 채 반대편 자리로 돌아가려다
            # 앞 장애물 정지에 걸려 맴도는 경우가 있었음: k16 #21 align_timeout)
            self.get_logger().info(f'킥보드 {sid}: 이미 팔이 닿는 거리 -> 정렬 주행 생략')
        else:
            # 첫 시도: 자리는 nav_bridge 가 코스트맵(정적 지도 + 라이다)으로 고른다 (standoff)
            status, msg = too_far(*self.align_drive(
                gh, GoToPose.Goal(target_pose=target, standoff=float(dist)), cx, cy, dist, collect))
            # 실패했고 팔도 안 닿으면: 반경 dist 는 그대로, 킥보드 중심에서 본 각도만 바꾼 자리로 직접 지정해 다시 간다.
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
                    more = f' (같은 반경 {len(tried)}곳 시도)' if len(tried) > 1 else ''
                    return self.finish(gh, False, f'align_failed: {msg}{more}')
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

    def align_drive(self, gh, goal, cx, cy, dist, collect):
        """정렬 자리로 한 번 이동. -> (상태, 메시지). 상태: ok / failed(nav_bridge 이동 실패) / rejected / timeout / canceled"""
        nh = future_wait(self, self.nav.send_goal_async(goal), 5.0)
        if nh is None or not nh.accepted:
            return 'rejected', 'nav goal rejected'
        rf = nh.get_result_async()
        t0 = now_sec(self)
        while rclpy.ok() and not rf.done():
            if gh.is_cancel_requested:
                nh.cancel_goal_async()
                return 'canceled', 'canceled'
            if now_sec(self) - t0 > self.get_parameter('align_timeout_sec').value:
                nh.cancel_goal_async()
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
                    nh.cancel_goal_async()
                    return None
                if self.fresh(tag) is not None:
                    nh.cancel_goal_async()
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
