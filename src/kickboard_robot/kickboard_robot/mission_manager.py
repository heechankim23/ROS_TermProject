"""F5 / F11 / F14: 미션 상태 머신 (PRD 3장 + 주차구역 하역).

           ┌──────────── 만재 · 배터리 ≤ 20% · 대기열 빈 ───────────┐
 IDLE ─요청─> SELECTING ─그 외─> NAVIGATING ─> VERIFYING ─> LOADING ┘(+1 -> SELECTING)
                ^                     │  3회 실패     │ 10초 미검출
                └──── FAILED <────────┴───────────────┘
 DROPPING(주차구역으로 이동 -> 로봇팔로 내려 세움) ─> SELECTING
 RETURNING ─> DOCKING ─> (충전소행 킥보드 하역) CHARGING(≥80%) ─> SELECTING
 (주행 중 배터리 ≤ 20% 이면 현재 대상을 대기열에 되돌리고 RETURNING)

하역 장소는 실을 때 킥보드마다 정한다 (/scooter_states 의 배터리):
  배터리 < scooter_battery_threshold_pct  -> 'charger' (충전소에 맡김. 넘어짐·구역 밖 사유여도)
  그 외 (넘어짐 · 구역 밖)                -> 'parking' (가까운 주차구역에 세워 둠 -> 다시 정상 킥보드)
만재이거나 대기열이 비면: parking 킥보드가 있으면 DROPPING, 아니면 충전소로.
로봇 배터리가 부족하면 parking 킥보드를 실은 채 먼저 충전하고, 충전 후 DROPPING.
주차구역에 빈 자리가 없으면 충전소에서 전부 하역한다.

판단은 SELECTING 한 곳에서만 한다 -> 이동 중 새 요청이 와도 현재 경로를 끊지 않음(비선점).
"""
import math

import rclpy
import yaml
from geometry_msgs.msg import Pose, PoseStamped
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from std_msgs.msg import Float32

from kickboard_interfaces.action import CollectScooter, DockToCharger, GoToPose, UnloadScooter
from kickboard_interfaces.msg import AdminAlert, QueueStatus, RobotStatus, ScooterState, ScooterTruthArray
from kickboard_interfaces.srv import SelectNextTarget, UpdateRequestStatus

from .common import EventLog, PoseTracker, now_sec, spin_single
from .drop_zones import charger_spots, choose_plan, edge_plans
from .motion import quat_from_yaw
from .occupancy import OccupancyMap

IDLE, SELECTING, NAVIGATING, VERIFYING, LOADING = 'IDLE', 'SELECTING', 'NAVIGATING', 'VERIFYING', 'LOADING'
FAILED, RETURNING, DOCKING, CHARGING = 'FAILED', 'RETURNING', 'DOCKING', 'CHARGING'
DROPPING = 'DROPPING'
STATES = (IDLE, SELECTING, NAVIGATING, VERIFYING, LOADING, FAILED, RETURNING, DOCKING, CHARGING, DROPPING)
PARKING, CHARGER = 'parking', 'charger'      # 킥보드별 하역 장소
COLLECT_PHASE_TO_STATE = {'NAVIGATING': NAVIGATING, 'VERIFYING': VERIFYING,
                          'ALIGNING': LOADING, 'LOADING': LOADING}


def should_go_charge(load, capacity, battery, return_pct):
    return load >= capacity or battery <= return_pct


def drop_destination(battery_pct, threshold_pct):
    """실은 킥보드의 하역 장소. 배터리를 모르면(보고 없음) 주차구역."""
    return CHARGER if battery_pct is not None and battery_pct < threshold_pct else PARKING


class MissionManager(Node):
    def __init__(self):
        super().__init__('mission_manager')
        self.declare_parameter('capacity', 3)
        self.declare_parameter('return_battery_pct', 20.0)
        self.declare_parameter('charge_target_pct', 80.0)
        self.declare_parameter('start_docked', True)
        self.declare_parameter('station_id', 'station_1')
        self.declare_parameter('admin_alerts', True)        # F14 (P1)
        self.declare_parameter('max_dock_retries', 3)
        self.declare_parameter('scooter_battery_threshold_pct', 15.0)   # 이보다 낮으면 충전소에 하역
        self.declare_parameter('map_config_file', '')      # 주차구역 (없으면 주차구역 하역 안 함)
        self.declare_parameter('map_yaml', '')             # 로봇이 설 자리 빈 공간 판정
        self.declare_parameter('drop_clearance', 1.05)
        self.declare_parameter('unload_sim_sec', 1.5)      # 로봇팔이 없을 때(kinematic) 1대 하역 시간

        self.cb = MutuallyExclusiveCallbackGroup()   # 모든 콜백 직렬화 -> 상태 경합 없음
        self.pose = PoseTracker(self)
        self.ev = EventLog(self)
        self.state = SELECTING
        self.at_dock = self.get_parameter('start_docked').value
        self.cargo = []                # [(킥보드 id, PARKING|CHARGER)] 실은 순서
        self.scooter_batt = {}         # id -> 최근 보고 배터리
        self.used_spots = []           # 주차구역에 이미 세운 자리 (x, y)
        self.dock_unload_all = False   # 주차구역 자리가 없을 때 충전소에서 전부 하역
        self.drop_plan = None
        self.charger_spots = []        # 충전소 충전 자리 [(x, y, yaw)]
        self.charger_occ = []          # 자리별로 맡긴 킥보드 id (None = 빈 자리)
        self.charger_fifo = []         # 맡긴 순서 (가득 차면 가장 오래된 것을 '충전 끝'으로 치움)
        self.battery = 100.0
        self.target = None             # scooter id
        self.waiting_items = 0
        self.pending = False           # 비동기 호출 진행 중
        self.collect_handle = None
        self.abort_for_battery = False
        self.dock_fail = 0
        self.ready = False
        self.state_since = 0.0

        self.create_subscription(Float32, '/battery_pct', lambda m: setattr(self, 'battery', m.data), 10,
                                 callback_group=self.cb)
        self.create_subscription(QueueStatus, '/queue_status', self.on_queue, 10, callback_group=self.cb)
        self.create_subscription(ScooterState, '/scooter_states',
                                 lambda m: self.scooter_batt.__setitem__(m.id, m.battery_pct), 20,
                                 callback_group=self.cb)
        self.dropped_pub = self.create_publisher(ScooterTruthArray, '/scooter_dropped', 10)
        self.status_pub = self.create_publisher(RobotStatus, '/robot_status', 10)
        self.alert_pub = self.create_publisher(AdminAlert, '/admin_alerts', 10)
        self.select_cli = self.create_client(SelectNextTarget, '/queue/select_next', callback_group=self.cb)
        self.update_cli = self.create_client(UpdateRequestStatus, '/queue/update_status', callback_group=self.cb)
        self.collect_ac = ActionClient(self, CollectScooter, '/collect_scooter', callback_group=self.cb)
        self.dock_ac = ActionClient(self, DockToCharger, '/dock_to_charger', callback_group=self.cb)
        self.nav_ac = ActionClient(self, GoToPose, '/nav_bridge/go_to_pose', callback_group=self.cb)
        self.unload_ac = ActionClient(self, UnloadScooter, '/arm_loader/unload', callback_group=self.cb)
        self.drop_plans = self.load_drop_plans()
        self.create_timer(0.2, self.tick, callback_group=self.cb)
        self.create_timer(0.5, self.publish_status, callback_group=self.cb)

    # ---------------- helpers ----------------
    def p(self, n):
        return self.get_parameter(n).value

    @property
    def load(self):
        return len(self.cargo)

    def cargo_for(self, dest):
        return [sid for sid, d in self.cargo if d == dest]

    def load_drop_plans(self):
        if not self.p('map_config_file'):
            self.get_logger().warn('map_config_file 없음 -> 주차구역 하역 안 함 (전부 충전소 하역)')
            return []
        with open(self.p('map_config_file')) as f:
            mc = yaml.safe_load(f)
        zones = mc.get('parking_zones', [])
        tag = mc['charger']['tag_pose']
        self.charger_spots = charger_spots(tag['x'], tag['y'], math.radians(tag['yaw_deg']))
        self.charger_occ = [None] * len(self.charger_spots)
        omap = OccupancyMap(self.p('map_yaml')) if self.p('map_yaml') else None
        plans = []
        for z in zones:
            plans += edge_plans(z['name'], z['polygon'], omap, self.p('drop_clearance'))
        self.get_logger().info(f'주차구역 하역 자리: 구역 {len(zones)}개, 로봇 위치 {len(plans)}곳, '
                               f'킥보드 자리 {sum(len(p["spots"]) for p in plans)}개')
        return plans

    def set_state(self, s, why=''):
        if s != self.state:
            self.get_logger().info(f'[상태] {self.state} -> {s} {why}')
            self.ev.emit('state', frm=self.state, to=s, why=why, load=self.load, battery=round(self.battery, 1))
            self.state = s
            self.state_since = now_sec(self)
            self.publish_status()

    def publish_status(self):
        self.status_pub.publish(RobotStatus(
            battery_pct=float(self.battery), load_count=int(self.load), capacity=int(self.p('capacity')),
            state=self.state, current_target_id=self.target or ''))

    def on_queue(self, m):
        self.waiting_items = sum(1 for i in m.items if i.status == 'WAITING')

    def report(self, sid, status, note='', corrected=None):
        req = UpdateRequestStatus.Request(scooter_id=sid, status=status, note=note)
        if corrected is not None:
            req.has_corrected_pose, req.corrected_pose = True, corrected
        self.update_cli.call_async(req)

    def alert(self, sid, reason, message):
        if not self.p('admin_alerts'):
            return
        a = AdminAlert(id=sid, reason=reason, message=message)
        a.stamp = self.get_clock().now().to_msg()
        self.alert_pub.publish(a)
        self.get_logger().warn(f'[관리자 알림] {sid}: {reason} {message}')

    # ---------------- main loop ----------------
    def tick(self):
        if not self.ready:
            servers = (self.select_cli.service_is_ready() and self.update_cli.service_is_ready()
                       and self.collect_ac.server_is_ready() and self.dock_ac.server_is_ready())
            if not servers or now_sec(self) <= 0.0:
                return
            self.ready = True
            self.get_logger().info('모든 서버 준비 완료, 미션 시작')
            self.state_since = now_sec(self)
        if self.pending:
            return

        s = self.state
        if s == SELECTING:
            self.do_select()
        elif s == IDLE:
            if self.battery <= self.p('return_battery_pct'):
                self.set_state(CHARGING, f'(대기 중 배터리 {self.battery:.0f}%)')
            elif self.waiting_items > 0:
                self.set_state(SELECTING, '(새 요청)')
        elif s in (NAVIGATING, VERIFYING, LOADING):
            if (self.battery <= self.p('return_battery_pct') and self.collect_handle is not None
                    and not self.abort_for_battery and s != LOADING):
                self.abort_for_battery = True
                self.get_logger().warn('주행 중 배터리 부족 -> 현재 대상 반납 후 복귀')
                self.collect_handle.cancel_goal_async()
        elif s == FAILED:
            if now_sec(self) - self.state_since > 1.0:
                self.set_state(SELECTING)
        elif s == CHARGING:
            if self.battery >= self.p('charge_target_pct'):
                self.set_state(SELECTING, f'(충전 {self.battery:.0f}%)')

    def do_select(self):
        if self.at_dock and (self.cargo_for(CHARGER) or (self.dock_unload_all and self.cargo)):
            self.unload_at_dock(lambda n: self.set_state(SELECTING, f'(충전소 하역 {n}대)'))
            return
        low = self.battery <= self.p('return_battery_pct')
        if low or self.load >= self.p('capacity'):
            why = f'(배터리 {self.battery:.0f}%)' if low else '(만재)'
            if not low and self.cargo_for(PARKING) and self.start_drop(why):
                return
            if self.at_dock:
                self.set_state(CHARGING, why)
            else:
                self.start_dock(why)
            return
        pose = self.pose.get()
        if pose is None:
            return
        self.pending = True
        fut = self.select_cli.call_async(SelectNextTarget.Request(robot_x=pose[0], robot_y=pose[1]))
        fut.add_done_callback(self.on_selected)

    def on_selected(self, fut):
        self.pending = False
        r = fut.result()
        if r is None:
            return
        if not r.found:
            if self.cargo_for(PARKING) and self.start_drop('(대기열 빈)'):
                return
            if self.at_dock:
                self.set_state(IDLE, '(대기열 빈)')
            else:
                self.start_dock('(대기열 빈)')
            return
        self.start_collect(r.scooter_id, r.target_pose)

    # ---------------- collect ----------------
    def start_collect(self, sid, target_pose):
        self.target = sid
        self.at_dock = False
        self.abort_for_battery = False
        self.set_state(NAVIGATING, f'-> 킥보드 {sid}')
        self.pending = True
        goal = CollectScooter.Goal(scooter_id=sid, target_pose=target_pose)
        f = self.collect_ac.send_goal_async(goal, feedback_callback=self.on_collect_fb)
        f.add_done_callback(self.on_collect_accepted)

    def on_collect_fb(self, m):
        st = COLLECT_PHASE_TO_STATE.get(m.feedback.phase)
        if st and st != self.state and self.state in (NAVIGATING, VERIFYING, LOADING):
            self.set_state(st)

    def on_collect_accepted(self, f):
        h = f.result()
        if h is None or not h.accepted:
            self.pending = False
            self.report(self.target, 'REQUEUE', 'collect goal rejected')
            self.set_state(FAILED, '(goal rejected)')
            return
        self.collect_handle = h
        self.pending = False
        h.get_result_async().add_done_callback(self.on_collect_done)

    def on_collect_done(self, f):
        res = f.result().result
        sid, self.collect_handle = self.target, None
        if res.success:
            batt = self.scooter_batt.get(sid)
            dest = drop_destination(batt, self.p('scooter_battery_threshold_pct'))
            self.cargo.append((sid, dest))
            self.report(sid, 'DONE', 'loaded', res.corrected_pose.pose)
            self.ev.emit('collected', id=sid, corrected_x=res.corrected_pose.pose.position.x,
                         corrected_y=res.corrected_pose.pose.position.y, load=self.load, dest=dest)
            self.target = None
            where = '충전소' if dest == CHARGER else '주차구역'
            batt_s = '?' if batt is None else f'{batt:.0f}%'
            self.set_state(SELECTING, f'(적재 {self.load}/{self.p("capacity")}, 킥보드 {sid} 배터리 {batt_s} '
                                      f'-> {where} 하역 예정)')
        elif self.abort_for_battery:
            self.report(sid, 'REQUEUE', 'battery low')
            self.target = None
            self.start_dock('(배터리 부족)')
        else:
            reason = res.failure_reason
            is_nav = reason.startswith('nav')
            status = 'NEEDS_HUMAN' if (is_nav and self.p('admin_alerts')) else 'FAILED'
            self.report(sid, status, reason)
            self.alert(sid, 'nav_failed' if is_nav else 'verify_failed', reason)
            self.ev.emit('collect_failed', id=sid, reason=reason)
            self.target = None
            self.set_state(FAILED, f'({reason})')

    # ---------------- dock ----------------
    def start_dock(self, why=''):
        self.set_state(RETURNING, why)
        self.pending = True
        f = self.dock_ac.send_goal_async(DockToCharger.Goal(station_id=self.p('station_id')),
                                         feedback_callback=self.on_dock_fb)
        f.add_done_callback(self.on_dock_accepted)

    def on_dock_fb(self, m):
        if m.feedback.phase in ('ALIGN', 'DOCKED') and self.state == RETURNING:
            self.set_state(DOCKING)

    def on_dock_accepted(self, f):
        h = f.result()
        if h is None or not h.accepted:
            self.pending = False
            self.set_state(FAILED, '(dock rejected)')
            return
        h.get_result_async().add_done_callback(self.on_dock_done)

    def on_dock_done(self, f):
        self.pending = False
        res = f.result().result
        if res.success:
            self.dock_fail = 0
            self.at_dock = True
            self.ev.emit('docked', position_error_m=res.position_error_m, yaw_error_deg=res.yaw_error_deg,
                         unloaded=len(self.cargo_for(CHARGER)))
            self.unload_at_dock(lambda n: self.set_state(CHARGING, f'(충전소 하역 {n}대)'))
        else:
            self.dock_fail += 1
            if self.dock_fail >= self.p('max_dock_retries'):
                self.alert('station', 'dock_failed', f'{self.dock_fail}회 도킹 실패')
                self.dock_fail = 0
                self.at_dock = True                            # 사람이 옮겨준다고 가정
                self.unload_at_dock(lambda n: self.set_state(CHARGING, '(도킹 실패 — 수동 처리 가정)'))
            else:
                self.start_dock(f'(도킹 재시도 {self.dock_fail})')

    # ---------------- 하역 ----------------
    def unload_at_dock(self, then):
        """충전소: 충전소행(또는 dock_unload_all 이면 전부) 킥보드를 로봇팔로 충전 자리에 세움.
        자리가 다 차 있으면 가장 오래 맡긴 킥보드를 '충전 끝 -> 치움' 처리하고 그 자리를 쓴다."""
        ids = [sid for sid, d in self.cargo if d == CHARGER or self.dock_unload_all]
        self.dock_unload_all = False
        jobs = []
        for sid in reversed(ids):                      # 나중에 실은 것(앞 칸)부터
            if not self.charger_spots:
                jobs.append((sid, None, 'remove'))
                continue
            if None in self.charger_occ:
                idx = self.charger_occ.index(None)
            else:
                idx = self.charger_fifo.pop(0)
                jobs.append((self.charger_occ[idx], None, 'charged'))
            self.charger_occ[idx] = sid
            self.charger_fifo.append(idx)
            jobs.append((sid, self.charger_spots[idx], 'charger'))
        self.run_unloads(jobs, lambda n: then(n))

    def start_drop(self, why):
        """가장 가까운 주차구역 변으로 이동 -> 로봇팔로 내려 세움. 갈 곳이 없으면 False (충전소에서 전부 하역)"""
        pose = self.pose.get()
        if pose is None:
            return False
        parking = self.cargo_for(PARKING)
        ch = choose_plan(self.drop_plans, self.used_spots, pose[:2], need=len(parking))
        if ch is None:
            self.get_logger().warn('주차구역에 빈 자리 없음 -> 충전소에서 전부 하역')
            self.dock_unload_all = True
            return False
        self.drop_plan = ch
        plan, spots = ch
        self.at_dock = False
        self.set_state(DROPPING, f'{why} -> 주차구역 {plan["zone"]} ({len(parking)}대)')
        rx, ry, ryaw = plan['robot']
        target = PoseStamped()
        target.header.frame_id = 'map'
        target.pose.position.x, target.pose.position.y = rx, ry
        o = target.pose.orientation
        o.x, o.y, o.z, o.w = quat_from_yaw(ryaw)
        self.pending = True
        f = self.nav_ac.send_goal_async(GoToPose.Goal(target_pose=target, standoff=0.0))
        f.add_done_callback(self.on_drop_nav_accepted)
        return True

    def on_drop_nav_accepted(self, f):
        h = f.result()
        if h is None or not h.accepted:
            self.on_drop_nav_failed('nav goal rejected')
            return
        h.get_result_async().add_done_callback(self.on_drop_nav_done)

    def on_drop_nav_done(self, f):
        r = f.result()
        if r is None or not r.result.success:
            self.on_drop_nav_failed(r.result.message if r is not None else 'no result')
            return
        plan, spots = self.drop_plan
        # 나중에 실은 것(앞 칸)부터 꺼낸다 -> 팔이 다른 킥보드 위로 지나가지 않음
        parking = list(reversed(self.cargo_for(PARKING)))
        jobs = [(sid, spot, 'park') for sid, spot in zip(parking, spots)]
        self.run_unloads(jobs, lambda n: self.set_state(SELECTING, f'(주차구역 {plan["zone"]} 하역 {n}대)'))

    def on_drop_nav_failed(self, why):
        self.pending = False
        self.alert('drop', 'nav_failed', f'주차구역 이동 실패: {why} -> 충전소에서 전부 하역')
        self.dock_unload_all = True
        self.start_dock('(주차구역 이동 실패)')

    def run_unloads(self, jobs, then):
        """jobs = [(id, (x, y, yaw) 또는 None, 종류)] 를 하나씩. 끝나면 then(적재함에서 내린 수)
          'park'    주차구역에 세움 -> 다시 정상 킥보드 (/scooter_dropped)
          'charger' 충전소 충전 자리에 세움 (충전하러 맡김)
          'charged' 충전 자리의 오래된 킥보드를 치움 (충전 끝) — 적재 수와 무관
          'remove'  적재함에서 바로 지움 (충전 자리 정보가 없을 때)"""
        self.pending = True
        jobs, done = list(jobs), [0]

        def finish():
            self.pending = False
            self.publish_status()
            then(done[0])

        def next_job():
            if not jobs:
                finish()
                return
            sid, spot, kind = jobs.pop(0)
            if not self.unload_ac.server_is_ready():
                # 로봇팔 없음 (kinematic / arm:=false): 잠깐 멈췄다가 하역한 것으로 처리
                tm = []

                def fire():
                    tm[0].cancel()
                    job_done(sid, spot, kind, spot)
                tm.append(self.create_timer(self.p('unload_sim_sec') if spot else 0.1, fire,
                                            callback_group=self.cb))
                return
            goal = UnloadScooter.Goal(scooter_id=sid, remove_only=spot is None, keep_as_cargo=kind == 'charger')
            if spot is not None:
                goal.target.position.x, goal.target.position.y = spot[0], spot[1]
                q = quat_from_yaw(spot[2])
                goal.target.orientation.x, goal.target.orientation.y = q[0], q[1]
                goal.target.orientation.z, goal.target.orientation.w = q[2], q[3]
            self.unload_ac.send_goal_async(goal).add_done_callback(lambda f: on_accepted(f, sid, spot, kind))

        def on_accepted(f, sid, spot, kind):
            h = f.result()
            if h is None or not h.accepted:
                job_failed(sid, spot, kind, 'unload rejected')
                return
            h.get_result_async().add_done_callback(lambda rf: on_result(rf, sid, spot, kind))

        def on_result(rf, sid, spot, kind):
            r = rf.result()
            if r is None or not r.result.success:
                job_failed(sid, spot, kind, r.result.message if r is not None else 'no result')
                return
            pp = r.result.placed_pose
            job_done(sid, spot, kind, (pp.position.x, pp.position.y) if spot is not None else None)

        def job_done(sid, spot, kind, placed):
            if kind == 'charged':
                self.ev.emit('charged_out', id=sid)
                self.get_logger().info(f'킥보드 {sid} 충전 끝 -> 충전 자리에서 치움')
                next_job()
                return
            self.cargo = [c for c in self.cargo if c[0] != sid]
            done[0] += 1
            if kind == 'park':
                self.used_spots.append((spot[0], spot[1]))
                m = ScooterTruthArray()
                m.header.frame_id = 'map'
                m.header.stamp = self.get_clock().now().to_msg()
                p = Pose()
                p.position.x, p.position.y = float(placed[0]), float(placed[1])
                q = quat_from_yaw(spot[2])
                p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = q
                m.ids.append(sid)
                m.poses.append(p)
                self.dropped_pub.publish(m)
                self.ev.emit('dropped', id=sid, x=placed[0], y=placed[1])
                self.get_logger().info(f'킥보드 {sid} 주차구역에 세움 ({placed[0]:.2f}, {placed[1]:.2f})')
            else:
                self.ev.emit('unloaded_at_charger', id=sid)
                if spot is not None:
                    self.get_logger().info(f'킥보드 {sid} 충전소 충전 자리에 세움 ({placed[0]:.2f}, {placed[1]:.2f})')
            self.publish_status()
            next_job()

        def job_failed(sid, spot, kind, why):
            self.get_logger().warn(f'킥보드 {sid} 하역 실패 ({kind}): {why}')
            if kind == 'park':                   # 주차구역에 못 내리면 충전소에서 내림
                self.cargo = [(s, CHARGER if s == sid else d) for s, d in self.cargo]
                self.used_spots.append((spot[0], spot[1]))     # 그 자리는 다시 쓰지 않음
            elif kind == 'charger':              # 충전 자리에 못 세우면 적재함에서 바로 지움
                jobs.insert(0, (sid, None, 'remove'))
                if sid in self.charger_occ:
                    idx = self.charger_occ.index(sid)
                    self.charger_occ[idx] = None
                    self.charger_fifo.remove(idx)
            elif kind == 'remove':
                self.cargo = [c for c in self.cargo if c[0] != sid]
                self.ev.emit('unloaded_at_charger', id=sid)
            next_job()

        next_job()


def main():
    rclpy.init()
    spin_single(MissionManager())
