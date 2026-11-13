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

이 파일은 통신부다: 메시지 변환, 서비스·액션 호출 순서, 로그만 맡는다.
판단 규칙(decide_*), 적재함(Cargo), 충전 자리 배정(ChargerSlots)은 mission_logic.py (가공부, ROS 없음) 에 있다.
KICKBOARD_PROFILE=1 이면 판단 1회 시간을 로그에 남긴다 ('tick', 'select').
"""
import dataclasses
import math

import rclpy
import yaml
from geometry_msgs.msg import Pose, PoseStamped
from rcl_interfaces.msg import SetParametersResult
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from std_msgs.msg import Float32

from kickboard_interfaces.action import CollectScooter, DockToCharger, GoToPose, UnloadScooter
from kickboard_interfaces.msg import AdminAlert, QueueStatus, RobotStatus, ScooterState, ScooterTruthArray
from kickboard_interfaces.srv import SelectNextTarget, UpdateRequestStatus

from .common import EventLog, PoseTracker, now_sec, spin_single
from .drop_zones import charger_spots, choose_plan, edge_plans
from .mission_logic import (CHARGER, CHARGING, COLLECT_PHASE_TO_STATE, DOCKING, DROPPING, FAILED, LOADING,
                            NAVIGATING, NEEDS_HUMAN, PARKING, RETURNING, SELECTING, VERIFYING, Cargo, ChargerSlots,
                            MissionParams, WaitGuard, classify_failure, decide_queue_empty, decide_select, decide_tick,
                            dock_should_stop, drop_destination, drop_next_points, parking_jobs)
from .motion import quat_from_yaw
from .occupancy import OccupancyMap
from .profiling import Profiler

MISSION_PARAM_NAMES = tuple(f.name for f in dataclasses.fields(MissionParams))


class MissionManager(Node):
    def __init__(self):
        super().__init__('mission_manager')
        # 판단에 쓰는 값 (MissionParams 와 이름이 같아야 한다. /set_config 가 capacity·return_battery_pct 를 바꾼다)
        self.declare_parameter('capacity', 3)
        self.declare_parameter('return_battery_pct', 20.0)
        self.declare_parameter('charge_target_pct', 80.0)
        self.declare_parameter('admin_alerts', True)        # F14 (P1)
        self.declare_parameter('max_dock_retries', 3)
        self.declare_parameter('scooter_battery_threshold_pct', 15.0)   # 이보다 낮으면 충전소에 하역
        self.declare_parameter('drop_next_weight', 1.0)    # 하역 자리: 하역 후 갈 곳 거리 가중치 (0 = 예전 방식)
        # 통신부 설정
        self.declare_parameter('start_docked', True)
        self.declare_parameter('station_id', 'station_1')
        self.declare_parameter('map_config_file', '')      # 주차구역 (없으면 주차구역 하역 안 함)
        self.declare_parameter('map_yaml', '')             # 로봇이 설 자리 빈 공간 판정
        self.declare_parameter('drop_clearance', 1.05)
        self.declare_parameter('unload_sim_sec', 1.5)      # 로봇팔이 없을 때(kinematic) 1대 하역 시간
        # 멈춤 방지: 응답·결과가 이 시간(sim s) 안에 안 오면 취소하고 복구한다. 아래 노드들의 자체 제한보다 넉넉하게
        self.declare_parameter('response_timeout_sec', 10.0)    # 서비스 응답, 액션 goal 수락
        self.declare_parameter('collect_timeout_sec', 900.0)    # collector: 이동 3회 x 120 s + 정렬 90 s + 팔 120 s ...
        self.declare_parameter('dock_timeout_sec', 600.0)       # dock_controller: 이동 3회 x 120 s + 정렬 40 s
        self.declare_parameter('drop_nav_timeout_sec', 480.0)   # nav_bridge: 3회 x 120 s
        self.declare_parameter('unload_timeout_sec', 120.0)     # 로봇팔 1대 하역 (Gazebo 약 21 s)
        self._mp = None                                    # MissionParams 캐시 (파라미터가 바뀌면 None)
        self.add_on_set_parameters_callback(self.on_set_params)

        self.cb = MutuallyExclusiveCallbackGroup()   # 모든 콜백 직렬화 -> 상태 경합 없음
        self.pose = PoseTracker(self)
        self.ev = EventLog(self)
        self.prof = Profiler(self.get_name(), self.get_logger().info)
        self.state = SELECTING
        self.at_dock = self.get_parameter('start_docked').value
        self.cargo = Cargo()           # 실은 순서대로 (킥보드 id, PARKING | CHARGER)
        self.scooter_batt = {}         # id -> 최근 보고 배터리
        self.used_spots = []           # 주차구역에 이미 세운 자리 (x, y)
        self.dock_unload_all = False   # 주차구역 자리가 없을 때 충전소에서 전부 하역
        self.drop_plan = None
        self.charger = ChargerSlots([])   # 충전소 충전 자리 배정 (map_config 를 읽으면 채움)
        self.battery = 100.0
        self.target = None             # scooter id
        self.waiting_items = 0
        self.waiting_xy = []           # 대기 중인 킥보드 위치 (하역 후 갈 곳 후보)
        self.dock_xy = None            # 도킹 위치 (map_config robot_spawn)
        self.pending = False           # 비동기 호출 진행 중 (tick 의 판단을 쉼)
        self.wait = WaitGuard()        # 지금 기다리는 응답 (멈춤 방지)
        self.collect_handle = None
        self.dock_handle = None
        self.drop_handle = None
        self.unload_handle = None
        self.unload_on_timeout = None  # 하역 1대가 시간 초과됐을 때 부를 실패 처리
        self.missing_servers = set()   # 사라진 서버 (알림을 한 번만 보내려고)
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
    def mp(self):
        """판단에 쓰는 파라미터. 바뀐 뒤 처음 읽을 때만 다시 만든다."""
        if self._mp is None:
            self._mp = MissionParams(**{n: self.p(n) for n in MISSION_PARAM_NAMES})
        return self._mp

    def on_set_params(self, params):
        # 값이 실제로 적용되기 전에 불린다 -> 여기서는 캐시만 비우고, 다음에 읽을 때 적용된 값을 가져온다
        if any(p.name in MISSION_PARAM_NAMES for p in params):
            self._mp = None
        return SetParametersResult(successful=True)

    def load_drop_plans(self):
        if not self.p('map_config_file'):
            self.get_logger().warn('map_config_file 없음 -> 주차구역 하역 안 함 (전부 충전소 하역)')
            return []
        with open(self.p('map_config_file')) as f:
            mc = yaml.safe_load(f)
        zones = mc.get('parking_zones', [])
        tag = mc['charger']['tag_pose']
        if 'robot_spawn' in mc:                       # = 도킹 pose
            self.dock_xy = (mc['robot_spawn']['x'], mc['robot_spawn']['y'])
        self.charger = ChargerSlots(charger_spots(tag['x'], tag['y'], math.radians(tag['yaw_deg'])))
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
            self.ev.emit('state', frm=self.state, to=s, why=why, load=self.cargo.load,
                         battery=round(self.battery, 1))
            self.state = s
            self.state_since = now_sec(self)
            self.publish_status()

    def publish_status(self):
        self.status_pub.publish(RobotStatus(
            battery_pct=float(self.battery), load_count=int(self.cargo.load), capacity=int(self.mp.capacity),
            state=self.state, current_target_id=self.target or ''))

    def on_queue(self, m):
        self.waiting_xy = [(i.x, i.y) for i in m.items if i.status == 'WAITING']
        self.waiting_items = len(self.waiting_xy)

    def report(self, sid, status, note='', corrected=None):
        req = UpdateRequestStatus.Request(scooter_id=sid, status=status, note=note)
        if corrected is not None:
            req.has_corrected_pose, req.corrected_pose = True, corrected
        self.update_cli.call_async(req)

    def alert(self, sid, reason, message):
        if not self.mp.admin_alerts:
            return
        a = AdminAlert(id=sid, reason=reason, message=message)
        a.stamp = self.get_clock().now().to_msg()
        self.alert_pub.publish(a)
        self.get_logger().warn(f'[관리자 알림] {sid}: {reason} {message}')

    # ---------------- 멈춤 방지 ----------------
    WAIT_TIMEOUT_PARAM = {'collect': 'collect_timeout_sec', 'dock': 'dock_timeout_sec',
                          'drop_nav': 'drop_nav_timeout_sec', 'unload': 'unload_timeout_sec'}

    def begin_wait(self, kind):
        """응답 대기 시작. 결과 대기(collect·dock·drop_nav·unload)는 각자 제한, 나머지는 response_timeout_sec."""
        return self.wait.begin(kind, now_sec(self), self.p(self.WAIT_TIMEOUT_PARAM.get(kind, 'response_timeout_sec')))

    def guarded(self, token, fn):
        """token 의 대기가 아직 살아 있을 때만 fn 을 부르는 콜백 (시간 초과 뒤 늦게 온 응답은 버린다)."""
        def cb(arg):
            if not self.wait.end(token):
                self.get_logger().warn(f'시간 초과 뒤 늦게 온 응답 무시: {fn.__name__}')
                return
            fn(arg)
        return cb

    def server_up(self, name, ready):
        """서버가 사라지거나 돌아올 때 한 번씩만 알린다. ready 를 그대로 돌려준다."""
        if ready and name in self.missing_servers:
            self.missing_servers.discard(name)
            self.get_logger().info(f'{name} 다시 연결됨 -> 재개')
            self.ev.emit('server_back', name=name)
        elif not ready and name not in self.missing_servers:
            self.missing_servers.add(name)
            self.alert(name, 'server_missing', f'{name} 응답 없음 -> 대기')
            self.ev.emit('server_missing', name=name, state=self.state)
        return ready

    @staticmethod
    def cancel(handle):
        if handle is not None:
            handle.cancel_goal_async()

    def on_wait_timeout(self, kind):
        self.get_logger().warn(f'[응답 없음] {kind} 제한 시간 초과 -> 취소하고 복구 (상태 {self.state})')
        self.ev.emit('wait_timeout', kind=kind, state=self.state, target=self.target or '')
        if kind == 'select':                       # SELECTING 그대로 -> 다음 tick 에 다시 묻는다
            self.pending = False
        elif kind in ('collect_accept', 'collect'):
            self.cancel(self.collect_handle)
            self.collect_handle, self.pending = None, False
            sid, self.target = self.target, None
            if sid:
                self.report(sid, 'REQUEUE', 'collect timeout')
                self.alert(sid, 'timeout', f'수거 응답 없음 ({kind})')
            self.set_state(FAILED, '(수거 응답 없음)')
        elif kind in ('dock_accept', 'dock'):
            self.cancel(self.dock_handle)
            self.dock_handle = None
            self.dock_failed(None, f'응답 없음 ({kind})')
        elif kind in ('drop_accept', 'drop_nav'):
            self.cancel(self.drop_handle)
            self.drop_handle = None
            self.on_drop_nav_failed(f'응답 없음 ({kind})')
        elif kind == 'unload':                     # 하역 묶음은 계속 진행 (pending 유지)
            self.cancel(self.unload_handle)
            self.unload_handle = None
            cb, self.unload_on_timeout = self.unload_on_timeout, None
            if cb is not None:
                cb()

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
        kind = self.wait.expired(now_sec(self))
        if kind is not None:
            self.on_wait_timeout(kind)
        if self.pending:
            return
        with self.prof.section('tick'):
            d = decide_tick(self.state, self.battery, self.mp, self.waiting_items, now_sec(self) - self.state_since,
                            self.collect_handle is not None, self.abort_for_battery)
        if d.action == 'SELECT':
            self.do_select()
        elif d.action == 'SET':
            self.set_state(d.state, d.why)
        elif d.action == 'ABORT_COLLECT':
            self.abort_for_battery = True
            self.get_logger().warn('주행 중 배터리 부족 -> 현재 대상 반납 후 복귀')
            self.collect_handle.cancel_goal_async()

    def do_select(self):
        with self.prof.section('select'):
            d = decide_select(self.at_dock, self.cargo, self.battery, self.mp, self.dock_unload_all)
        if d.action == 'UNLOAD_AT_DOCK':
            self.unload_at_dock(lambda n: self.set_state(SELECTING, f'(충전소 하역 {n}대)'))
        elif d.action == 'RETURN':
            self.go_return(d)
        elif d.action == 'QUERY':
            # collector 가 없으면 대상을 받지 않는다 (받으면 IN_PROGRESS -> 시간 초과 -> REQUEUE 를 끝없이 반복)
            if not self.server_up('collector', self.collect_ac.server_is_ready()):
                return
            pose = self.pose.get()
            if pose is None:
                return
            self.pending = True
            fut = self.select_cli.call_async(SelectNextTarget.Request(robot_x=pose[0], robot_y=pose[1]))
            fut.add_done_callback(self.guarded(self.begin_wait('select'), self.on_selected))

    def go_return(self, d, queue_empty=False):
        """RETURN 판단 실행: try_drop 이면 주차구역 하역부터 시도, 안 되면 fallback (CHARGING | IDLE | 'DOCK')."""
        if d.try_drop and self.start_drop(d.why, queue_empty):
            return
        if d.fallback == 'DOCK':
            self.start_dock(d.why)
        else:
            self.set_state(d.fallback, d.why)

    def on_selected(self, fut):
        self.pending = False
        r = fut.result()
        if r is None:
            return
        if not r.found:
            self.go_return(decide_queue_empty(self.at_dock, self.cargo), queue_empty=True)
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
        f.add_done_callback(self.guarded(self.begin_wait('collect_accept'), self.on_collect_accepted))

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
        h.get_result_async().add_done_callback(self.guarded(self.begin_wait('collect'), self.on_collect_done))

    def on_collect_done(self, f):
        res = f.result().result
        sid, self.collect_handle = self.target, None
        if res.success:
            batt = self.scooter_batt.get(sid)
            dest = drop_destination(batt, self.mp.scooter_battery_threshold_pct)
            self.cargo.add(sid, dest)
            self.report(sid, 'DONE', 'loaded', res.corrected_pose.pose)
            self.ev.emit('collected', id=sid, corrected_x=res.corrected_pose.pose.position.x,
                         corrected_y=res.corrected_pose.pose.position.y, load=self.cargo.load, dest=dest)
            self.target = None
            where = '충전소' if dest == CHARGER else '주차구역'
            batt_s = '?' if batt is None else f'{batt:.0f}%'
            self.set_state(SELECTING, f'(적재 {self.cargo.load}/{self.mp.capacity}, 킥보드 {sid} 배터리 {batt_s} '
                                      f'-> {where} 하역 예정)')
        elif self.abort_for_battery:
            self.report(sid, 'REQUEUE', 'battery low')
            self.target = None
            self.start_dock('(배터리 부족)')
        else:
            reason = res.failure_reason
            status, kind = classify_failure(reason, self.mp.admin_alerts)
            self.report(sid, status, reason)
            self.alert(sid, kind, reason)
            self.ev.emit('collect_failed', id=sid, reason=reason)
            self.target = None
            self.set_state(FAILED, f'({reason})')

    # ---------------- dock ----------------
    def start_dock(self, why=''):
        self.set_state(RETURNING, why)
        self.pending = True
        f = self.dock_ac.send_goal_async(DockToCharger.Goal(station_id=self.p('station_id')),
                                         feedback_callback=self.on_dock_fb)
        f.add_done_callback(self.guarded(self.begin_wait('dock_accept'), self.on_dock_accepted))

    def on_dock_fb(self, m):
        if m.feedback.phase in ('ALIGN', 'DOCKED') and self.state == RETURNING:
            self.set_state(DOCKING)

    def on_dock_accepted(self, f):
        h = f.result()
        if h is None or not h.accepted:              # 거절도 도킹 실패로 센다 (예전: 횟수 없이 무한 재시도)
            self.dock_failed(None, 'dock rejected')
            return
        self.dock_handle = h
        h.get_result_async().add_done_callback(self.guarded(self.begin_wait('dock'), self.on_dock_done))

    def on_dock_done(self, f):
        self.dock_handle = None
        self.pending = False
        res = f.result().result
        if res.success:
            self.dock_fail = 0
            self.at_dock = True
            self.ev.emit('docked', position_error_m=res.position_error_m, yaw_error_deg=res.yaw_error_deg,
                         unloaded=len(self.cargo.for_dest(CHARGER)))
            self.unload_at_dock(lambda n: self.set_state(CHARGING, f'(충전소 하역 {n}대)'))
        else:
            self.dock_failed(res, 'dock failed')

    def dock_failed(self, res, why):
        """도킹 실패 1회 (결과 실패·goal 거절·응답 없음). 한계에 닿으면 정지, 아니면 재시도."""
        self.dock_fail += 1
        if dock_should_stop(self.dock_fail, self.mp.max_dock_retries):
            # 충전·하역으로 넘어가지 않고 정지 (지표 왜곡 방지). 관리자가 처리할 때까지 대기
            self.alert('station', 'dock_failed', f'{self.dock_fail}회 도킹 실패 ({why}) -> 정지, 관리자 처리 필요')
            self.ev.emit('dock_failed', attempts=self.dock_fail, why=why,
                         position_error_m=None if res is None else res.position_error_m,
                         yaw_error_deg=None if res is None else res.yaw_error_deg, cargo=self.cargo.load)
            self.dock_fail = 0
            self.pending = False
            self.set_state(NEEDS_HUMAN, '(도킹 실패)')
        else:
            self.start_dock(f'(도킹 재시도 {self.dock_fail}: {why})')

    # ---------------- 하역 ----------------
    def unload_at_dock(self, then):
        """충전소: 충전소행(또는 dock_unload_all 이면 전부) 킥보드를 로봇팔로 충전 자리에 세움.
        자리 배정(다 차 있으면 가장 오래 맡긴 킥보드를 '충전 끝 -> 치움')은 ChargerSlots 가 한다."""
        ids = self.cargo.ids_for_dock(self.dock_unload_all)
        self.dock_unload_all = False
        self.run_unloads(self.charger.plan_unload(ids), then)

    def start_drop(self, why, queue_empty=False):
        """주차구역 변으로 이동 -> 로봇팔로 내려 세움. 갈 곳이 없으면 False (충전소에서 전부 하역)
        자리는 '로봇 -> 하역 위치 + 하역 위치 -> 다음 갈 곳'이 가장 짧은 것 (다음 갈 곳: drop_next_points)"""
        pose = self.pose.get()
        if pose is None:
            return False
        parking = self.cargo.for_dest(PARKING)
        next_xys = drop_next_points(queue_empty, self.waiting_xy, self.dock_xy)
        ch = choose_plan(self.drop_plans, self.used_spots, pose[:2], need=len(parking), next_xys=next_xys,
                         next_weight=self.mp.drop_next_weight)
        if ch is None:
            self.get_logger().warn('주차구역에 빈 자리 없음 -> 충전소에서 전부 하역')
            self.dock_unload_all = True
            return False
        self.drop_plan = ch
        plan, spots = ch
        self.ev.emit('drop_plan', zone=plan['zone'], robot_x=round(pose[0], 2), robot_y=round(pose[1], 2),
                     next=('waiting' if not queue_empty and self.waiting_xy else 'dock' if next_xys else 'none'),
                     n_next=len(next_xys))
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
        f.add_done_callback(self.guarded(self.begin_wait('drop_accept'), self.on_drop_nav_accepted))
        return True

    def on_drop_nav_accepted(self, f):
        h = f.result()
        if h is None or not h.accepted:
            self.on_drop_nav_failed('nav goal rejected')
            return
        self.drop_handle = h
        h.get_result_async().add_done_callback(self.guarded(self.begin_wait('drop_nav'), self.on_drop_nav_done))

    def on_drop_nav_done(self, f):
        self.drop_handle = None
        r = f.result()
        if r is None or not r.result.success:
            self.on_drop_nav_failed(r.result.message if r is not None else 'no result')
            return
        plan, spots = self.drop_plan
        # 나중에 실은 것(앞 칸)부터 꺼낸다 -> 팔이 다른 킥보드 위로 지나가지 않음
        jobs = parking_jobs(self.cargo, spots)
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
            tok = self.begin_wait('unload')          # 수락 + 결과를 한 제한 시간으로
            self.unload_on_timeout = lambda: job_failed(sid, spot, kind, '응답 없음')
            self.unload_ac.send_goal_async(goal).add_done_callback(lambda f: on_accepted(f, sid, spot, kind, tok))

        def on_accepted(f, sid, spot, kind, tok):
            if not self.wait.is_current(tok):        # 이미 시간 초과로 넘어감
                return
            h = f.result()
            if h is None or not h.accepted:
                self.wait.end(tok)
                self.unload_on_timeout = None
                job_failed(sid, spot, kind, 'unload rejected')
                return
            self.unload_handle = h
            h.get_result_async().add_done_callback(lambda rf: on_result(rf, sid, spot, kind, tok))

        def on_result(rf, sid, spot, kind, tok):
            if not self.wait.end(tok):
                return
            self.unload_handle = self.unload_on_timeout = None
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
            self.cargo.remove(sid)
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
                self.cargo.redirect_to_charger(sid)
                self.used_spots.append((spot[0], spot[1]))     # 그 자리는 다시 쓰지 않음
            elif kind == 'charger':              # 충전 자리에 못 세우면 적재함에서 바로 지움
                jobs.insert(0, (sid, None, 'remove'))
                self.charger.release(sid)
            elif kind == 'remove':
                self.cargo.remove(sid)
                self.ev.emit('unloaded_at_charger', id=sid)
            next_job()

        next_job()


def main():
    rclpy.init()
    spin_single(MissionManager())
