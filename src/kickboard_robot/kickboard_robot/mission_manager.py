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
근처 주차구역: 배터리 정상 + parking 킥보드가 있고, 고를 하역 위치(start_drop 과 같은 고르기)가 opportunistic_drop_dist(m) 안이면
  다음 대상을 고르기 전에 먼저 DROPPING (0 이면 끔).
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
from geometry_msgs.msg import Pose, PoseStamped, Twist
from rcl_interfaces.msg import SetParametersResult
from rcl_interfaces.srv import GetParameters
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

from kickboard_interfaces.action import CollectScooter, DockToCharger, GoToPose, UnloadScooter
from kickboard_interfaces.msg import AdminAlert, QueueStatus, RobotStatus, ScooterState, ScooterTruthArray
from kickboard_interfaces.srv import SelectNextTarget, UpdateRequestStatus

from .common import EventLog, PoseTracker, now_sec, spin_single
from .drop_zones import charger_spots, choose_plan, clear_plans, edge_plans, nearby_drop
# IDLE 은 여기서 쓰지 않지만 mission_manager_campus / _station 이 이 모듈에서 가져가므로 남겨 둔다
from .mission_logic import (CHARGER, CHARGING, COLLECT_PHASE_TO_STATE, DOCKING, DROPPING, FAILED, IDLE,  # noqa: F401
                            LOADING, LYING_STATUSES,
                            NAVIGATING, NEEDS_HUMAN, PARKING, RETURNING, SELECTING, VERIFYING, Cargo, ChargerSlots,
                            EnergyModel, MissionParams, ResolveWatch, WaitGuard, affordable, charge_needed,
                            classify_failure,
                            collect_cost, decide_queue_empty, decide_select, decide_tick, dock_should_stop,
                            drop_cost, drop_destination, drop_next_points, must_head_home, parking_jobs, rear_clear,
                            scan_to_body_points)
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
        self.declare_parameter('scooter_battery_threshold_pct', 20.0)   # 이보다 낮으면 충전소에 하역
        self.declare_parameter('drop_next_weight', 1.0)    # 하역 자리: 하역 후 갈 곳 거리 가중치 (0 = 예전 방식)
        self.declare_parameter('drop_keepout_m', 1.5)      # 대기·실패 킥보드에서 이보다 가까운 하역 위치는 안 씀 (0 = 끔)
        self.declare_parameter('max_drop_attempts', 2)     # 하역 이동 실패 시 다른 하역 위치까지 몇 곳 시도
        self.declare_parameter('drop_block_sec', 60.0)     # 이동에 실패한 하역 위치를 쓰지 않는 시간 (코스트맵 장애물 기억과 같게)
        self.declare_parameter('drop_stall_sec', 6.0)      # 하역 이동 피드백이 이만큼 없으면(경로 없음) 취소하고 다른 위치로 (0 = 끔)
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
        # 배터리 판단: '갔다가 가장 가까운 충전소까지 돌아올 수 있나'. false 면 예전처럼 20% 기준만 쓴다
        # 소모 계수 k1·k2 는 battery_manager 파라미터를 읽어 온다 (못 읽으면 EnergyModel 기본값 = battery_manager 기본값)
        self.declare_parameter('energy_check', True)
        # 시작 때 소모 계수를 받을 때까지 판단을 미룬다 (기본값 k1 1.0 %/m 로 첫 대상을 '못 감' 오판하던 경합). 0 = 안 기다림
        self.declare_parameter('battery_param_wait_sec', 5.0)
        self.declare_parameter('energy_reserve_pct', 5.0)       # 충전소 도착 때 남길 배터리
        self.declare_parameter('energy_detour', 1.2)            # 직선거리 -> 경로 길이 배율
        self.declare_parameter('energy_speed_mps', 0.2)
        self.declare_parameter('energy_collect_sec', 40.0)
        self.declare_parameter('energy_unload_sec', 25.0)
        # 출발한 요청도 다시 확인: 수거하러 가는 동안 대상 킥보드의 펌웨어 보고가 연속 recheck_confirm_reports 번
        # "정상(바르게 서 있고 구역 안, 배터리 기준 이상 또는 충전 중, 고장 없음)"이면 수거를 멈추고 CANCELLED(현장 해소).
        # 팔이 잡기 시작한 뒤에는 collector 가 취소를 받지 않고 그대로 싣는다. false = 예전처럼 끝까지 수거
        self.declare_parameter('recheck_resolved', True)
        self.declare_parameter('recheck_confirm_reports', 2)
        self.declare_parameter('recheck_tilt_deg', 45.0)                # 넘어짐 기준 (scooter_fleet_sim 과 같게)
        # 도크에서 출발할 때: 충전소를 앞에 두고 선 도킹 자세이므로 먼저 뒤로 빠져나온다. 라이다로 뒤쪽 통로
        # (차체 뒤끝에서 남은 거리 + 여유, 차체 폭)를 계속 확인하며 막히면 멈춰 기다리고, undock_wait_sec 넘게 막히면 후진을 접고 출발.
        # 0 = 예전처럼 바로 출발
        self.declare_parameter('undock_distance', 1.0)
        self.declare_parameter('undock_speed', 0.2)
        self.declare_parameter('undock_margin', 0.3)
        self.declare_parameter('undock_wait_sec', 10.0)
        self.declare_parameter('undock_near_dock_m', 0.6)               # 도크 자리에서 이만큼 안이면 '도크에서 출발'
        self.declare_parameter('scan_offset_x', 0.80)                   # 라이다(base_scan) 위치, nav_bridge 와 같게
        self._mp = None                                    # MissionParams 캐시 (파라미터가 바뀌면 None)
        self.add_on_set_parameters_callback(self.on_set_params)
        # 근처 주차구역: 대상을 고르기 전에 이 거리(m) 안에 하역 위치가 있으면 들러서 내림. 0 = 끔 (main 5c92abe)
        self.declare_parameter('opportunistic_drop_dist', 2.0)

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
        self.drop_nearby = False       # 지금 하역이 '근처 주차구역' 하역인가
        self.nearby_ok = True          # 근처 하역 이동이 실패하면 다음 적재 전까지 다시 시도하지 않음
        self.battery = 100.0
        self.target = None             # scooter id
        self.waiting_items = 0
        self.waiting_xy = []           # 대기 중인 킥보드 위치 (하역 후 갈 곳 후보)
        self.lying_xy = []             # 아직 그 자리에 놓여 있을 킥보드 위치 (하역 위치에서 피함, GPS 값일 수 있음)
        self.blocked_stands = []       # 이동에 실패한 하역 위치 [(x, y, 다시 쓸 시각)]
        self.drop_why, self.drop_queue_empty, self.drop_attempt = '', False, 1
        self.drop_progress_t = 0.0     # 하역 이동 피드백을 마지막으로 받은 시각
        self.drop_stall_sent = False
        self.dock_xy = None            # 도킹 위치 (map_config robot_spawn)
        self.pending = False           # 비동기 호출 진행 중 (tick 의 판단을 쉼)
        self.wait = WaitGuard()        # 지금 기다리는 응답 (멈춤 방지)
        self.collect_handle = None
        self.dock_handle = None
        self.drop_handle = None
        self.unload_handle = None
        self.unload_on_timeout = None  # 하역 1대가 시간 초과됐을 때 부를 실패 처리
        self.missing_servers = set()   # 사라진 서버 (알림을 한 번만 보내려고)
        self.battery_k = None          # battery_manager 의 (k1, k2)
        self.battery_k_asked = False
        self.battery_k_failed = False   # 읽기 실패 -> 기본값으로 진행
        self.ready_at = None
        self.charge_goal = None        # 다음 일에 80% 보다 더 필요하면 그만큼 충전 (CHARGING 을 나가면 지움)
        self.abort_for_battery = False
        self.watch = None              # 수거 중인 대상의 사유 해소 감시 (ResolveWatch)
        self.target_xy = None
        self.last_scan = None          # (받은 시각, LaserScan) — 도크 후진 때 뒤쪽 확인
        self.undock = None             # 진행 중인 도크 후진 (dict)
        self.abort_resolved = None     # 사유가 사라져 수거를 멈춘 이유 (취소를 보냈으면 문자열)
        self.collect_phase = ''
        self.dock_fail = 0
        self.ready = False
        self.state_since = 0.0

        self.create_subscription(Float32, '/battery_pct', lambda m: setattr(self, 'battery', m.data), 10,
                                 callback_group=self.cb)
        self.create_subscription(QueueStatus, '/queue_status', self.on_queue, 10, callback_group=self.cb)
        self.create_subscription(ScooterState, '/scooter_states', self.on_scooter_state, 20,
                                 callback_group=self.cb)
        self.create_subscription(LaserScan, '/scan', lambda m: setattr(self, 'last_scan', (now_sec(self), m)), 5,
                                 callback_group=self.cb)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.dropped_pub = self.create_publisher(ScooterTruthArray, '/scooter_dropped', 10)
        self.status_pub = self.create_publisher(RobotStatus, '/robot_status', 10)
        self.alert_pub = self.create_publisher(AdminAlert, '/admin_alerts', 10)
        self.select_cli = self.create_client(SelectNextTarget, '/queue/select_next', callback_group=self.cb)
        self.update_cli = self.create_client(UpdateRequestStatus, '/queue/update_status', callback_group=self.cb)
        self.collect_ac = ActionClient(self, CollectScooter, '/collect_scooter', callback_group=self.cb)
        self.dock_ac = ActionClient(self, DockToCharger, '/dock_to_charger', callback_group=self.cb)
        self.nav_ac = ActionClient(self, GoToPose, '/nav_bridge/go_to_pose', callback_group=self.cb)
        self.unload_ac = ActionClient(self, UnloadScooter, '/arm_loader/unload', callback_group=self.cb)
        self.batt_param_cli = self.create_client(GetParameters, '/battery_manager/get_parameters',
                                                 callback_group=self.cb)
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

    # 변형 노드(mission_manager_campus / _station, extended_runtime) 가 예전 이름으로 쓰는 적재함 접근.
    # campus 는 self.cargo = [...] 로 통째로 바꾸기도 하므로 들어온 리스트를 Cargo 로 감싼다.
    @property
    def cargo(self):
        return self._cargo

    @cargo.setter
    def cargo(self, items):
        self._cargo = items if isinstance(items, Cargo) else Cargo(items)

    @property
    def load(self):
        return self._cargo.load

    def cargo_for(self, dest):
        return self._cargo.for_dest(dest)

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
            if self.state == CHARGING:
                self.charge_goal = None
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
        self.lying_xy = [(i.x, i.y) for i in m.items if i.status in LYING_STATUSES]

    def on_scooter_state(self, m):
        self.scooter_batt[m.id] = m.battery_pct
        w = self.watch
        if w is None or m.id != self.target or not self.p('recheck_resolved'):
            return
        if not w.update(now_sec(self), m.tilt_deg, m.in_parking_zone, m.battery_pct, m.fault):
            return
        if self.collect_handle is None or self.abort_resolved is not None or self.abort_for_battery:
            return                      # 아직 수거 명령이 안 받아졌거나 이미 멈추는 중
        pose = self.pose.get()
        dist = None if pose is None or self.target_xy is None else math.dist(pose[:2], self.target_xy)
        self.abort_resolved = w.why
        dist_s = '?' if dist is None else f'{dist:.1f} m'
        self.get_logger().warn(f'킥보드 {m.id}: 요청 사유가 사라짐 ({w.why}, 보고 {w.streak}회 연속) '
                               f'-> 수거 멈춤 (단계 {self.collect_phase}, 남은 거리 {dist_s})')
        self.ev.emit('resolved_abort', id=m.id, why=w.why, phase=self.collect_phase,
                     dist=None if dist is None else round(dist, 2))
        self.collect_handle.cancel_goal_async()

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

    # ---------------- 배터리 판단 ----------------
    @property
    def energy(self):
        k1, k2 = self.battery_k or (EnergyModel.k1_pct_per_m, EnergyModel.k2_pct_per_s)
        return EnergyModel(k1_pct_per_m=k1, k2_pct_per_s=k2, detour=self.p('energy_detour'),
                           speed_mps=self.p('energy_speed_mps'), collect_sec=self.p('energy_collect_sec'),
                           unload_sec=self.p('energy_unload_sec'), reserve_pct=self.p('energy_reserve_pct'))

    def chargers(self):
        """충전할 수 있는 곳 [(x, y)]. 지금은 충전소 1곳 (map_config robot_spawn = 도킹 위치)."""
        return [self.dock_xy] if self.dock_xy is not None else []

    def ask_battery_params(self):
        """battery_manager 의 소모 계수를 한 번 읽어 온다 (두 노드의 예측식이 어긋나지 않게)."""
        if self.battery_k_asked or not self.batt_param_cli.service_is_ready():
            return
        self.battery_k_asked = True

        def done(f):
            r = f.result()
            if r is None or len(r.values) != 2:
                self.battery_k_failed = True
                self.get_logger().warn('battery_manager 소모 계수를 못 읽음 -> 기본값 사용')
                return
            self.battery_k = (r.values[0].double_value, r.values[1].double_value)
            self.get_logger().info(f'배터리 소모 계수 k1={self.battery_k[0]} %/m, k2={self.battery_k[1]} %/s')
        self.batt_param_cli.call_async(GetParameters.Request(names=['k1_pct_per_m', 'k2_pct_per_s'])) \
            .add_done_callback(done)

    def energy_ok_for_collect(self, sid, target_pose):
        """대상까지 가서 싣고 충전소까지 돌아올 배터리가 있나. 없으면 대상을 돌려놓고 충전하러 간다 (False)."""
        pose = self.pose.get()
        if not self.p('energy_check') or pose is None:
            return True
        m, ch = self.energy, self.chargers()
        txy = (target_pose.pose.position.x, target_pose.pose.position.y)
        cost = collect_cost(m, pose[:2], txy, ch)
        if affordable(self.battery, cost, m):
            return True
        # 충전소(대상에 가장 유리한 곳)에서 출발할 때 필요한 양
        need = charge_needed(min(collect_cost(m, c, txy, ch) for c in ch) if ch else cost, m)
        self.ev.emit('energy_check', what='collect', id=sid, cost=round(cost, 1), battery=round(self.battery, 1),
                     need=round(need, 1))
        if need > 100.0:                           # 가득 충전해도 못 감 -> 사람에게
            self.report(sid, NEEDS_HUMAN, f'out of range: need {need:.0f}%')
            self.alert(sid, 'out_of_range', f'가득 충전해도 다녀올 수 없음 (필요 {need:.0f}%)')
            return False
        self.report(sid, 'REQUEUE', 'battery low (예측)')
        self.charge_goal = need
        why = f'(배터리 부족 예측: 킥보드 {sid} 다녀오기 {cost:.0f}%, 남음 {self.battery:.0f}%)'
        if self.at_dock:
            self.set_state(CHARGING, why)
        else:
            self.start_dock(why)
        return False

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
            self.state_since = self.ready_at = now_sec(self)
        if self.battery_k is None:
            self.ask_battery_params()
            if (self.p('energy_check') and not self.battery_k_failed
                    and now_sec(self) - self.ready_at < self.p('battery_param_wait_sec')):
                return                  # 소모 계수를 받기 전에는 대상 고르기·배터리 판단을 하지 않음
        self.check_drop_stall()
        kind = self.wait.expired(now_sec(self))
        if kind is not None:
            self.on_wait_timeout(kind)
        if self.pending:
            return
        must = False
        if (self.p('energy_check') and self.state in (NAVIGATING, VERIFYING) and self.collect_handle is not None
                and not self.abort_for_battery):
            pose = self.pose.get()
            must = pose is not None and must_head_home(self.battery, self.energy, pose[:2], self.chargers())
        p = self.mp
        if self.charge_goal is not None and self.charge_goal > p.charge_target_pct:
            p = dataclasses.replace(p, charge_target_pct=min(100.0, self.charge_goal))
        with self.prof.section('tick'):
            d = decide_tick(self.state, self.battery, p, self.waiting_items, now_sec(self) - self.state_since,
                            self.collect_handle is not None, self.abort_for_battery, must_return=must)
        if d.action == 'SELECT':
            self.do_select()
        elif d.action == 'SET':
            self.set_state(d.state, d.why)
        elif d.action == 'ABORT_COLLECT':
            self.abort_for_battery = True
            why = ('충전소까지 갈 배터리만 남음' if self.battery > self.mp.return_battery_pct
                   else f'{self.mp.return_battery_pct:.0f}% 이하')
            self.get_logger().warn(f'주행 중 배터리 부족 ({why}) -> 현재 대상 반납 후 복귀')
            self.ev.emit('energy_abort', battery=round(self.battery, 1), why=why, target=self.target or '')
            self.collect_handle.cancel_goal_async()

    def do_select(self):
        with self.prof.section('select'):
            d = decide_select(self.at_dock, self.cargo, self.battery, self.mp, self.dock_unload_all,
                              drop_when_low=self.p('energy_check'))
        if d.action == 'UNLOAD_AT_DOCK':
            self.unload_at_dock(lambda n: self.set_state(SELECTING, f'(충전소 하역 {n}대)'))
        elif d.action == 'RETURN':
            self.go_return(d)
        elif d.action == 'QUERY':
            # 근처 주차구역: 대기열에 묻기 전에 주차구역행을 먼저 내림 (SELECTING 에서만 판단 -> 비선점 유지)
            if self.nearby_ok and self.start_drop('', nearby=True):
                return
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
        if not self.energy_ok_for_collect(r.scooter_id, r.target_pose):
            return
        self.start_collect(r.scooter_id, r.target_pose)

    # ---------------- 도크에서 출발 ----------------
    def depart(self, send):
        """도크 자리에서 출발하면 먼저 뒤쪽을 확인하며 undock_distance 만큼 후진한 뒤 send() (목표 전송). 아니면 바로 send()."""
        pose = self.pose.get()
        d = self.p('undock_distance')
        dock = self.dock_position()
        if (d <= 0.0 or pose is None or dock is None
                or math.dist(pose[:2], dock) > self.p('undock_near_dock_m')):
            send()
            return
        self.pending = True
        self.undock = {'send': send, 'start': pose[:2], 't0': now_sec(self), 'blocked': None, 'waited': 0.0}
        self.get_logger().info(f'도크에서 출발: 뒤쪽을 확인하며 {d:.1f} m 후진')
        self.undock['timer'] = self.create_timer(0.1, self.undock_step, callback_group=self.cb)

    def dock_position(self):
        """도킹 위치 (x, y) — 도크 후진 판단용. 상속 노드(campus 등)가 load_drop_plans 를 바꿔 dock_xy 를 안 채우면
        map_config 의 robot_spawn 을 직접 읽는다 (dock_xy 는 건드리지 않음: 배터리 예측의 충전소 목록이 바뀌지 않게)."""
        if self.dock_xy is not None:
            return self.dock_xy
        if not hasattr(self, '_undock_dock'):
            self._undock_dock = None
            try:
                with open(self.p('map_config_file')) as f:
                    sp = (yaml.safe_load(f) or {}).get('robot_spawn')
                if sp:
                    self._undock_dock = (float(sp['x']), float(sp['y']))
            except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError):
                pass
        return self._undock_dock

    def undock_step(self):
        u = self.undock
        if u is None:
            return
        now, pose = now_sec(self), self.pose.get()
        d = self.p('undock_distance')
        moved = 0.0 if pose is None else math.dist(pose[:2], u['start'])
        if moved >= d:
            return self.undock_end('done', moved)
        if now - u['t0'] > d / max(0.05, self.p('undock_speed')) * 3.0 + self.p('undock_wait_sec'):
            return self.undock_end('timeout', moved)
        clear = False
        if self.last_scan is not None and now - self.last_scan[0] < 0.5:
            m = self.last_scan[1]
            pts = scan_to_body_points(m.ranges, m.angle_min, m.angle_increment, m.range_min, m.range_max,
                                      self.p('scan_offset_x'))
            clear = rear_clear(pts, d - moved, self.p('undock_margin'))
        cmd = Twist()
        if clear:
            cmd.linear.x = -abs(self.p('undock_speed'))
            u['blocked'] = None
        elif u['blocked'] is None:
            u['blocked'] = now
            why = '라이다 없음' if self.last_scan is None or now - self.last_scan[0] >= 0.5 else '뒤에 장애물'
            self.get_logger().warn(f'도크 후진 멈춤 ({why}) -> 비켜나길 기다림 (최대 {self.p("undock_wait_sec"):.0f}초)')
            self.ev.emit('undock_blocked', why=why, moved=round(moved, 2))
        elif now - u['blocked'] > self.p('undock_wait_sec'):
            return self.undock_end('blocked', moved)
        self.cmd_pub.publish(cmd)

    def undock_end(self, how, moved):
        u, self.undock = self.undock, None
        u['timer'].cancel()
        self.destroy_timer(u['timer'])
        self.cmd_pub.publish(Twist())
        msg = {'done': '후진 끝', 'timeout': '후진 시간 초과', 'blocked': '뒤가 계속 막혀 후진을 접음'}[how]
        (self.get_logger().info if how == 'done' else self.get_logger().warn)(f'도크 출발: {msg} ({moved:.2f} m) -> 목표로 출발')
        self.ev.emit('undock', how=how, moved=round(moved, 2), sec=round(now_sec(self) - u['t0'], 1))
        u['send']()

    # ---------------- collect ----------------
    def start_collect(self, sid, target_pose):
        self.target = sid
        self.target_xy = (target_pose.pose.position.x, target_pose.pose.position.y)
        self.at_dock = False
        self.abort_for_battery = False
        self.abort_resolved = None
        self.collect_phase = 'NAVIGATING'
        self.watch = ResolveWatch(tilt_th=self.p('recheck_tilt_deg'), batt_th=self.mp.scooter_battery_threshold_pct,
                                  confirm=self.p('recheck_confirm_reports'))
        self.set_state(NAVIGATING, f'-> 킥보드 {sid}')
        self.pending = True
        goal = CollectScooter.Goal(scooter_id=sid, target_pose=target_pose)

        def send():
            f = self.collect_ac.send_goal_async(goal, feedback_callback=self.on_collect_fb)
            f.add_done_callback(self.guarded(self.begin_wait('collect_accept'), self.on_collect_accepted))
        self.depart(send)

    def on_collect_fb(self, m):
        self.collect_phase = m.feedback.phase
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
        self.watch, resolved, self.abort_resolved = None, self.abort_resolved, None
        if res.success and resolved is not None:
            self.get_logger().info(f'킥보드 {sid}: 팔이 이미 잡기 시작해 그대로 적재 (사유 해소: {resolved})')
            self.ev.emit('resolved_too_late', id=sid, why=resolved)
        if res.success:
            batt = self.scooter_batt.get(sid)
            dest = drop_destination(batt, self.mp.scooter_battery_threshold_pct)
            self.cargo.add(sid, dest)
            self.nearby_ok = True
            self.report(sid, 'DONE', 'loaded', res.corrected_pose.pose)
            self.ev.emit('collected', id=sid, corrected_x=res.corrected_pose.pose.position.x,
                         corrected_y=res.corrected_pose.pose.position.y, load=self.cargo.load, dest=dest)
            self.target = None
            where = '충전소' if dest == CHARGER else '주차구역'
            batt_s = '?' if batt is None else f'{batt:.0f}%'
            self.set_state(SELECTING, f'(적재 {self.cargo.load}/{self.mp.capacity}, 킥보드 {sid} 배터리 {batt_s} '
                                      f'-> {where} 하역 예정)')
        elif resolved is not None and not self.abort_for_battery:
            self.report(sid, 'CANCELLED', f'resolved on site: {resolved}')
            self.ev.emit('resolved_cancel', id=sid, why=resolved)
            self.target = None
            self.set_state(SELECTING, f'(킥보드 {sid} 현장 해소: {resolved} -> 수거 취소)')
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

    def start_drop(self, why, queue_empty=False, attempt=1, nearby=False):
        """주차구역 변으로 이동 -> 로봇팔로 내려 세움. 갈 곳이 없으면 False (충전소에서 전부 하역)
        자리는 '로봇 -> 하역 위치 + 하역 위치 -> 다음 갈 곳'이 가장 짧은 것 (다음 갈 곳: drop_next_points)
        attempt: 몇 번째 하역 위치 시도인지 (이동 실패 시 on_drop_nav_failed 가 다음 위치로 다시 부른다)
        nearby=True: 근처 주차구역 하역 (do_select). 고른 위치가 opportunistic_drop_dist 밖이면 아무것도 안 하고 False.
          이동에 실패해도 충전소로 돌아가지 않고 실은 채로 수거를 계속한다."""
        self.drop_why, self.drop_queue_empty, self.drop_attempt = why, queue_empty, attempt
        pose = self.pose.get()
        if pose is None:
            return False
        parking = self.cargo.for_dest(PARKING)
        next_xys = drop_next_points(queue_empty, self.waiting_xy, self.dock_xy)
        plans = clear_plans(self.drop_plans, self.lying_xy, self.p('drop_keepout_m'))
        plans = clear_plans(plans, self.blocked_xy(), 0.3)      # 방금 이동에 실패한 위치는 잠시 안 씀
        if nearby:                                  # 고를 위치가 가까울 때만 (start_drop 과 같은 고르기)
            d = nearby_drop(plans, self.used_spots, pose[:2], len(parking), self.p('opportunistic_drop_dist'),
                            next_xys=next_xys, next_weight=self.mp.drop_next_weight)
            if d is None:
                return False
            why = f'(근처 주차구역 {d:.1f} m)'
        if len(plans) < len(self.drop_plans):
            self.ev.emit('drop_keepout', excluded=len(self.drop_plans) - len(plans), left=len(plans))
        ch = choose_plan(plans, self.used_spots, pose[:2], need=len(parking), next_xys=next_xys,
                         next_weight=self.mp.drop_next_weight)
        if ch is None:
            self.get_logger().warn(f'주차구역에 빈 자리 없음 (근처에 킥보드가 있어 뺀 후보 {len(self.drop_plans) - len(plans)}곳)'
                                   f' -> 충전소에서 전부 하역')
            self.dock_unload_all = True
            return False
        plan, spots = ch
        if self.p('energy_check'):                 # 하역하고 충전소까지 갈 배터리가 되나 (안 되면 그냥 복귀)
            cost = drop_cost(self.energy, pose[:2], plan['robot'][:2], len(parking), self.chargers())
            if not affordable(self.battery, cost, self.energy):
                self.get_logger().warn(f'주차구역 {plan["zone"]} 하역 후 복귀에 {cost:.0f}% 필요, 남음 '
                                       f'{self.battery:.0f}% -> 하역 미루고 충전 먼저')
                self.ev.emit('energy_check', what='drop', zone=plan['zone'], cost=round(cost, 1),
                             battery=round(self.battery, 1))
                return False
        self.drop_plan = ch
        self.drop_nearby = nearby
        self.ev.emit('drop_plan', zone=plan['zone'], robot_x=round(pose[0], 2), robot_y=round(pose[1], 2),
                     next=('waiting' if not queue_empty and self.waiting_xy else 'dock' if next_xys else 'none'),
                     n_next=len(next_xys), nearby=nearby)
        self.at_dock = False
        self.set_state(DROPPING, f'{why} -> 주차구역 {plan["zone"]} ({len(parking)}대)')
        rx, ry, ryaw = plan['robot']
        target = PoseStamped()
        target.header.frame_id = 'map'
        target.pose.position.x, target.pose.position.y = rx, ry
        o = target.pose.orientation
        o.x, o.y, o.z, o.w = quat_from_yaw(ryaw)
        self.pending = True
        self.drop_progress_t, self.drop_stall_sent = now_sec(self), False

        def send():
            self.drop_progress_t = now_sec(self)
            f = self.nav_ac.send_goal_async(GoToPose.Goal(target_pose=target, standoff=0.0),
                                            feedback_callback=self.on_drop_fb)
            f.add_done_callback(self.guarded(self.begin_wait('drop_accept'), self.on_drop_nav_accepted))
        self.depart(send)
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

    def blocked_xy(self):
        now = now_sec(self)
        self.blocked_stands = [b for b in self.blocked_stands if b[2] > now]
        return [(x, y) for x, y, _ in self.blocked_stands]

    def on_drop_fb(self, m):
        self.drop_progress_t = now_sec(self)

    def check_drop_stall(self):
        """nav_bridge 는 경로를 못 찾는 동안 피드백을 안 보낸다 -> drop_stall_sec 동안 없으면 취소하고 다른 위치로.
        (그대로 두면 8 s x 3회 = 약 27 s 를 기다린 뒤 실패한다)"""
        lim = self.p('drop_stall_sec')
        if (self.state != DROPPING or self.drop_handle is None or self.drop_stall_sent or lim <= 0
                or now_sec(self) - self.drop_progress_t <= lim):
            return
        self.drop_stall_sent = True
        self.get_logger().warn(f'하역 위치로 {lim:.0f} s 동안 진행 없음 (경로 없음) -> 취소하고 다른 위치로')
        self.ev.emit('drop_stall', zone=self.drop_plan[0]['zone'] if self.drop_plan else '')
        self.drop_handle.cancel_goal_async()       # 결과(실패)는 on_drop_nav_done -> on_drop_nav_failed 로 온다

    def on_drop_nav_failed(self, why):
        self.pending = False
        self.drop_handle = None
        plan = self.drop_plan[0] if self.drop_plan else None
        if plan is not None:                       # 이 위치는 한동안 안 씀
            self.blocked_stands.append((plan['robot'][0], plan['robot'][1], now_sec(self) + self.p('drop_block_sec')))
        self.ev.emit('drop_failed', zone=plan['zone'] if plan else '', why=why, attempt=self.drop_attempt)
        if self.drop_nearby:
            # 들르는 하역이었으므로 실은 채로 수거를 계속한다. 같은 실패를 반복하지 않게 다음 적재 전까지 근처 하역 끔
            self.drop_nearby = False
            self.nearby_ok = False
            self.get_logger().warn(f'근처 주차구역 이동 실패 ({why}) -> 실은 채로 수거 계속')
            self.set_state(SELECTING, '(근처 주차구역 이동 실패)')
            return
        if self.drop_attempt < self.p('max_drop_attempts'):
            self.get_logger().warn(f'주차구역 이동 실패 ({why}) -> 다른 하역 위치 시도 {self.drop_attempt + 1}')
            if self.start_drop(self.drop_why, self.drop_queue_empty, attempt=self.drop_attempt + 1):
                return
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
