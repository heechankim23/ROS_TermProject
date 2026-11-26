"""[시뮬레이션 전용, sim:=gz] SCOOTOVER v19 적재·하차 동작 재생 + 실린 킥보드 표시.

Blender 애니메이션(config/scootover_rig.json, build_blender_models.py 가 만듦)을 관절 명령으로 그대로 재생한다.
  적재 (LoadScooter, /arm_loader/load)  애니메이션 프레임 1~729: 로봇이 뒤를 킥보드에 대고 선 상태에서
      지지대·뒤 램프 -> 셔틀 마중 -> 팔이 스템을 잡음 -> 램프 타고 셔틀로 -> 셔틀이 거치 위치(x 0.65)까지 -> 옆 집게 닫힘 -> 접음
      collector 가 로봇을 '킥보드 앞바퀴 축이 로봇 기준 (-1.65, 레인 y)' 인 자리에 세운다 (collector_logic.reverse_entry_pose).
  하차 (UnloadScooter, /arm_loader/unload)  프레임 730~1690: 앞 램프 -> 팔이 스템을 밀어 Kickspot 거치대까지 -> 걸쇠 -> 0.65 m 후진
      remove_only  충전소 하역(예전 방식) — 적재함에서 바로 지움
      keep_as_cargo  Kickspot 충전 자리: 복사본을 거치 자세(앞바퀴 축 높이·기울기 = blender_assets.json kickspot_4)로 둔다
      그 외        주차구역: 같은 동작으로 바닥(target)에 세우고 원래 킥보드 모델(충돌체 있음)을 되살린다
레인 2개 (적재 2대): 0 = 왼쪽(y +0.27), 1 = 오른쪽(y -0.27). 애니메이션은 왼쪽 레인만 있어서 오른쪽은
  옆 부품 = 거울 짝 관절에 왼쪽 트랙, 팔 = '<관절>@r' 트랙(빌드 때 손목 거울상으로 푼 IK), 킥보드 = y 거울.
킥보드 실제 pose 와 애니메이션 pose 의 차이(정렬 오차·넘어짐)는 팔이 잡는 동안 섞어서 없앤다.
하차 목표와 애니메이션 끝 pose 의 차이도 팔이 미는 동안 섞어서 목표에 정확히 놓는다.

Gazebo 와의 통신
  gz-transport 파이썬 바인딩: 관절 명령 /kickboard_robot/joint/<관절> (model.sdf 의 JointPositionController),
                              set_pose_vector / create / remove 서비스, 킥보드 pose (시작 때 한 번만)
  ROS: /sim/robot_pose (로봇 world pose 정답값), 하차 끝 후진은 /cmd_vel (drive_reverse)
  ※ gz 파이썬 subscribe 를 계속 걸어 두면 안 된다 (GIL 때문에 request() 타임아웃, 실측 거의 100% 실패).
"""
import json
import math
import os
import threading

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Int8

from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.double_pb2 import Double
from gz.msgs10.entity_factory_pb2 import EntityFactory
from gz.msgs10.entity_pb2 import Entity
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzNode

from kickboard_interfaces.action import LoadScooter, UnloadScooter
from kickboard_interfaces.msg import RobotStatus, ScooterState

from .common import run_blocking, spin_single
from .gz_utils import scooter_colors, scooter_sdf, scooter_state

LANES = 2
LOAD_RANGE = (0, 729)            # 트랙 번호 [시작, 끝) — Blender 프레임 1~729
UNLOAD_RANGE = (729, 1690)       # 프레임 730~1690
GRIP_K, UPRIGHT_K = 124, 254     # 적재: 팔이 스템을 잡음(프레임 125) ~ 세움(255). 이 사이에 실제 pose -> 애니메이션 pose
PUSH_K, LATCH_K = 839, 1304      # 하차: 팔이 잡음(840) ~ 걸쇠 잠김(1305). 이 사이에 목표 pose 쪽으로 보정
PREP_SEC = 1.0                   # 애니메이션 첫 프레임 자세로 옮겨 가는 시간
WHEELS = {'j_wheel_fl', 'j_wheel_fr', 'j_wheel_rl', 'j_wheel_rr'}
MAX_START_ERR = (1.0, math.radians(45))   # 적재 시작: 킥보드가 애니메이션 시작 pose 에서 이 이상 벗어나면 거절


# ---------------- 수학 ----------------
def rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1.0, 0], [-s, 0, c]])


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def quat_to_mat(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def mat_to_quat(m):
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        q = (0.25 * s, (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s)
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        q = ((m[2, 1] - m[1, 2]) / s, 0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s)
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        q = ((m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s)
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        q = ((m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s)
    q = np.array(q)
    return q / np.linalg.norm(q)


def slerp(m0, m1, u):
    q0, q1 = mat_to_quat(m0), mat_to_quat(m1)
    d = float(np.dot(q0, q1))
    if d < 0:
        q1, d = -q1, -d
    if d > 0.9995:
        q = q0 + u * (q1 - q0)
    else:
        th = math.acos(d)
        q = (math.sin((1 - u) * th) * q0 + math.sin(u * th) * q1) / math.sin(th)
    return quat_to_mat(q / np.linalg.norm(q))


def ease(u):
    u = min(max(u, 0.0), 1.0)
    return u * u * (3 - 2 * u)


def blend(a, b, u):
    """pose (p, R) 두 개를 u 로 섞음"""
    return a[0] + u * (b[0] - a[0]), slerp(a[1], b[1], u)


def blend_about(corr, pivot, u):
    """world 에서 왼쪽에 곱하는 보정 corr 를 u(0~1) 만큼만, pivot 점을 중심으로 적용하는 pose.
    blend(단위, corr, u)는 world 원점 둘레로 돌아 원점에서 먼 곳(캠퍼스 수백 m)에서는 중간 프레임의
    킥보드가 수십 m 휘어 나갔다. pivot(애니메이션 끝 자리) 기준으로 바꾸면 평행이동이 실제 차이만큼만 남는다.
    u = 1 이면 corr 와 같다."""
    local = (corr[0] + corr[1] @ pivot - pivot, corr[1])          # pivot 기준 보정
    p, rm = blend((np.zeros(3), np.eye(3)), local, u)
    return pivot + p - rm @ pivot, rm


def compose(a, b):
    return a[0] + a[1] @ b[0], a[1] @ b[1]


def inverse(a):
    return -a[1].T @ a[0], a[1].T


def heading(rm):
    return math.atan2(rm[1, 0], rm[0, 0])


# ---------------- 애니메이션 데이터 ----------------
class Rig:
    """scootover_rig.json -> 레인별 관절값 표 Q[lane] (프레임 x 관절), 킥보드·로봇 pose 트랙"""

    def __init__(self, path, axle):
        d = json.load(open(path))
        self.fps = float(d['fps'])
        self.markers = [(m['frame'] - 1, m['name'].strip()) for m in d['markers']]
        tr = d['tracks']
        js = [j for j in d['joints'] if j['name'] not in WHEELS]
        self.names = [j['name'] for j in js]
        mirror = {j['name']: j['mirror_of'] for j in js if j['mirror_of']}
        left = set(mirror.values())
        F = d['frames']
        zero = [0.0] * F

        def col(n, lane):
            if lane == 0:
                return tr.get(n, zero)
            if n + '@r' in tr:
                return tr[n + '@r']
            if n in mirror:
                return tr[mirror[n]]
            return zero if n in left else tr.get(n, zero)
        self.Q = [np.array([col(n, lane) for n in self.names]).T for lane in range(LANES)]
        off = np.array(axle)                     # 킥보드 모델 원점 -> 앞바퀴 축 (Blender 원점은 앞바퀴 축)
        self.scooter = []                        # 레인별 [(p, R)] 로봇 기준 킥보드 모델 pose
        for lane in range(LANES):
            out = []
            for v in tr['ref_scooter_l@robot']:
                p, q = np.array(v[:3]), v[3:]
                if lane == 1:                    # y 거울
                    p = p * np.array([1, -1, 1])
                    q = (q[0], -q[1], q[2], -q[3])
                rm = quat_to_mat(q)
                out.append((p - rm @ off, rm))
            self.scooter.append(out)
        self.root = [(np.array(v[:3]), quat_to_mat(v[3:])) for v in tr['robot_root@start']]

    def phase(self, k):
        name = self.markers[0][1]
        for f, n in self.markers:
            if f <= k:
                name = n
        return name


class ArmLoader(Node):
    def __init__(self):
        super().__init__('arm_loader')
        self.declare_parameter('world_name', 'kickboard_world')
        self.declare_parameter('robot_name', 'kickboard_robot')
        self.declare_parameter('time_scale', 1.0)          # >1 이면 동작이 느려짐 (발표 녹화용)
        self.declare_parameter('rate_hz', 25.0)
        self.declare_parameter('drive_reverse', True)      # 하차 끝 0.65 m 후진을 실제로 (/cmd_vel)
        self.declare_parameter('rig_path', '')             # 비우면 kickboard_sim/config/scootover_rig.json
        self.declare_parameter('tilt_threshold_deg', 45.0)       # 킥보드 색 (scooter_fleet_sim 과 같은 기준)
        self.declare_parameter('battery_threshold_pct', 20.0)
        self.world = self.get_parameter('world_name').value
        self.robot = self.get_parameter('robot_name').value
        share = get_package_share_directory('kickboard_sim')
        assets = json.load(open(os.path.join(share, 'config', 'blender_assets.json')))
        self.axle = assets['g30']['front_axle']
        self.kickspot = assets['kickspot_4']
        self.rig = Rig(self.get_parameter('rig_path').value or os.path.join(share, 'config', 'scootover_rig.json'),
                       self.axle)

        self.lock = threading.Lock()
        self.robot_pose = None   # (위치 np(3), 회전 np(3x3)) world
        self.gz = GzNode()
        self.cmd = {j: self.gz.advertise(f'/{self.robot}/joint/{j}', Double) for j in self.rig.names}
        self.cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.q_sent = np.zeros(len(self.rig.names))      # 마지막으로 보낸 관절값 (0 = 수송 자세)
        self.q_force = True

        self.slots = [None] * LANES          # 레인 -> 실린 모델 이름
        self.cargo_rel = {}                  # 실린 모델 -> (p, R) 로봇 좌표
        self.station_cargo = set()           # 충전 자리에 세워 둔 모델 (로봇을 따라가지 않음)
        self.anim = None
        self.last_sent = None
        self.load_count = 0
        # 다음에 실을 레인 (collector 가 로봇을 그 레인에 맞춰 세움). 가득 차면 -1
        self.lane_pub = self.create_publisher(Int8, '/arm_loader/next_lane',
                                              QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.lane_sent = None

        self.cb = ReentrantCallbackGroup()
        self.create_subscription(RobotStatus, '/robot_status', self.on_status, 10, callback_group=self.cb)
        self.create_subscription(PoseStamped, '/sim/robot_pose', self.on_robot_pose, 10, callback_group=self.cb)
        self.scooter_info = {}   # id -> ScooterState (옮기는 복사본도 같은 상태 색으로)
        self.create_subscription(ScooterState, '/scooter_states',
                                 lambda m: self.scooter_info.__setitem__(m.id, m), 20, callback_group=self.cb)
        # 적재·하역은 중간에 못 멈춤 (킥보드를 든 채로 끝남) -> 취소 거절
        ActionServer(self, LoadScooter, '/arm_loader/load', self.execute_load, goal_callback=self.on_goal,
                     cancel_callback=lambda g: CancelResponse.REJECT, callback_group=self.cb)
        ActionServer(self, UnloadScooter, '/arm_loader/unload', self.execute_unload, goal_callback=self.on_goal,
                     cancel_callback=lambda g: CancelResponse.REJECT, callback_group=self.cb)
        self.create_timer(1.0 / self.get_parameter('rate_hz').value, self.tick, callback_group=self.cb)
        self.get_logger().info(f'SCOOTOVER 적재기 준비 (/arm_loader/load, 관절 {len(self.rig.names)}개, 레인 {LANES})')

    # ---------------- 입력 ----------------
    def on_robot_pose(self, msg):
        p, o = msg.pose.position, msg.pose.orientation
        with self.lock:
            self.robot_pose = (np.array([p.x, p.y, p.z]), quat_to_mat((o.w, o.x, o.y, o.z)))

    def snapshot_poses(self, timeout=3.0):
        """Gazebo 모든 모델의 world pose 를 한 번 받아 온다 (받으면 바로 구독 해제 — 모듈 설명의 GIL 문제)"""
        topic = f'/world/{self.world}/pose/info'
        got, out = threading.Event(), {}

        def cb(msg):
            if got.is_set():
                return
            for p in msg.pose:
                if p.name in out:         # 같은 이름의 링크(상대 pose)보다 최상위 모델이 먼저 온다
                    continue
                o = p.orientation
                out[p.name] = (np.array([p.position.x, p.position.y, p.position.z]),
                               quat_to_mat((o.w, o.x, o.y, o.z)))
            got.set()
        self.gz.subscribe(Pose_V, topic, cb)
        got.wait(timeout)
        self.gz.unsubscribe(topic)
        return out

    def on_status(self, msg):
        prev, self.load_count = self.load_count, msg.load_count
        if prev > 0 and msg.load_count == 0:
            self.unload_all()

    # ---------------- 유틸 ----------------
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def get_robot_pose(self):
        with self.lock:
            return self.robot_pose

    def send_joints(self, q):
        """바뀐 관절만 보낸다 (관절 65개를 매 주기 다 보내지 않게). 처음·끝에는 전부"""
        for i, j in enumerate(self.rig.names):
            if self.q_force or abs(q[i] - self.q_sent[i]) > 1e-4:
                self.cmd[j].publish(Double(data=float(q[i])))
        self.q_sent = np.array(q, dtype=float)
        self.q_force = False

    def gz_request(self, srv, req, req_t, rep_t=Boolean, timeout=1000):
        try:
            ok, rep = self.gz.request(f'/world/{self.world}/{srv}', req, req_t, rep_t, timeout)
            return ok and getattr(rep, 'data', True)
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f'gz {srv} 실패: {e}')
            return False

    def colors_of(self, sid):
        s = self.scooter_info.get(sid)
        if s is None:
            return {}
        bt = self.get_parameter('battery_threshold_pct').value
        st = scooter_state(s.tilt_deg, s.in_parking_zone, s.battery_pct,
                           self.get_parameter('tilt_threshold_deg').value, bt, fault=s.fault, misaligned=s.in_slot_area)
        deck, stem = scooter_colors(st, s.battery_pct < bt, broken=bool(s.fault))
        return {'deck': deck, 'stem': stem}

    def create_visual_copy(self, name, p, rm, collide=False, colors=None):
        q = mat_to_quat(rm)
        req = EntityFactory(sdf=scooter_sdf(name, collide=collide, **(colors or {})), name=name,
                            allow_renaming=False)
        req.pose.position.x, req.pose.position.y, req.pose.position.z = p
        req.pose.orientation.w, req.pose.orientation.x, req.pose.orientation.y, req.pose.orientation.z = q
        return self.gz_request('create', req, EntityFactory)

    def remove(self, name):
        return self.gz_request('remove', Entity(name=name, type=Entity.MODEL), Entity)

    def unload_all(self):
        if self.anim is not None:
            return
        names = [n for n in self.slots if n]
        for n in names:
            self.remove(n)
            self.cargo_rel.pop(n, None)
        self.slots = [None] * LANES
        self.last_sent = None
        if names:
            self.get_logger().info(f'하역: 적재함 비움 ({len(names)}대)')

    # ---------------- 액션 ----------------
    def on_goal(self, goal):
        if self.anim is not None:
            self.get_logger().warn('적재·하차 동작 중 -> 거절')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    # 기다리는 동안 단일 스레드 executor 가 tick 을 계속 돌리도록 작업 스레드에서 대기
    async def execute_load(self, gh):
        return await run_blocking(self._run, gh, 'load')

    async def execute_unload(self, gh):
        return await run_blocking(self._run, gh, 'unload')

    def _run(self, gh, mode):
        sid = gh.request.scooter_id
        res = LoadScooter.Result() if mode == 'load' else UnloadScooter.Result()
        if mode == 'unload' and gh.request.remove_only:
            ok, res.message = self.remove_cargo(sid)
            res.success = ok
            (gh.succeed if ok else gh.abort)()
            return res
        err, anim = (self.plan_load(sid) if mode == 'load'
                     else self.plan_unload(sid, gh.request.target, gh.request.keep_as_cargo))
        if err:
            self.get_logger().warn(f'킥보드 {sid} {"적재" if mode == "load" else "하차"} 불가: {err}')
            res.success, res.message = False, err
            gh.abort()
            return res
        ts = self.get_parameter('time_scale').value
        k0, k1 = anim['range']
        anim.update(gh=gh, mode=mode, done=threading.Event(), t0=self.now(), ts=ts, phase=None,
                    total=PREP_SEC * ts + (k1 - k0) / self.rig.fps * ts, q_start=self.q_sent.copy(),
                    fb=LoadScooter.Feedback if mode == 'load' else UnloadScooter.Feedback, reversing=False)
        if mode == 'load':
            # 원래 모델(충돌체 있음)을 충돌체 없는 복사본으로 바꿔치기 -> 램프·셔틀과 부딪히지 않음
            sp = anim['src_pose']
            self.create_visual_copy(anim['cargo'], sp[0], sp[1], colors=self.colors_of(sid))
            self.remove(anim['src'])
            self.get_logger().info(f'적재 시작: 킥보드 {sid} -> {"왼" if anim["lane"] == 0 else "오른"}쪽 레인'
                                   + (' (넘어진 킥보드 세우기)' if anim['fallen'] else '')
                                   + f', 시작 오차 {anim["start_err"][0]:.2f} m / '
                                     f'{math.degrees(anim["start_err"][1]):.0f}°')
        else:
            self.slots[anim['lane']] = None
            self.cargo_rel.pop(anim['cargo'], None)
            where = 'Kickspot 충전 자리' if anim['keep'] else '주차구역'
            self.get_logger().info(f'하차 시작: {"왼" if anim["lane"] == 0 else "오른"}쪽 레인 킥보드 {sid} -> {where}')
        self.anim = anim
        anim['done'].wait()
        res.success = True
        if mode == 'load':
            res.message, res.slot = 'loaded', anim['lane']
        else:
            res.message = 'dropped'
            pw, rw = anim['placed']
            res.placed_pose.position.x, res.placed_pose.position.y, res.placed_pose.position.z = (
                float(v) for v in pw)
            q = mat_to_quat(rw)
            o = res.placed_pose.orientation
            o.w, o.x, o.y, o.z = (float(v) for v in q)
        gh.succeed()
        return res

    def remove_cargo(self, sid):
        """애니메이션 없이 지움: 충전 자리의 킥보드(충전 끝) 또는 적재함의 킥보드"""
        name = f'cargo_{sid}'
        if name in self.station_cargo:
            self.remove(name)
            self.station_cargo.discard(name)
            self.get_logger().info(f'충전 끝: 킥보드 {sid} 를 충전 자리에서 치움')
            return True, 'removed from charger'
        if name not in self.slots:
            return False, f'적재함에 {name} 없음'
        self.remove(name)
        self.slots[self.slots.index(name)] = None
        self.cargo_rel.pop(name, None)
        self.last_sent = None
        self.get_logger().info(f'충전소 하역: 킥보드 {sid}')
        return True, 'removed'

    def plan_load(self, sid):
        """바닥의 킥보드(로봇 뒤) -> 빈 레인. 실패하면 (이유, None)"""
        src = f'scooter_{sid}'
        sp = self.snapshot_poses().get(src)
        rp = self.get_robot_pose()
        if rp is None:
            return 'robot pose 없음 (/sim/robot_pose 미수신)', None
        if sp is None:
            return f'Gazebo 에 {src} 모델 없음', None
        try:
            lane = self.slots.index(None)
        except ValueError:
            return '적재함 가득 참', None
        rel = compose(inverse(rp), sp)
        exp = self.rig.scooter[lane][LOAD_RANGE[0]]
        err = (float(np.linalg.norm((rel[0] - exp[0])[:2])), abs(wrap(heading(rel[1]) - heading(exp[1]))))
        if err[0] > MAX_START_ERR[0] or err[1] > MAX_START_ERR[1]:
            return (f'킥보드가 뒤 램프 자리에 없음 (로봇 기준 ({rel[0][0]:.2f}, {rel[0][1]:.2f}), 기대 '
                    f'({exp[0][0]:.2f}, {exp[0][1]:.2f}), 방향 차 {math.degrees(err[1]):.0f}°)'), None
        return None, {'sid': sid, 'src': src, 'src_pose': sp, 'cargo': f'cargo_{sid}', 'lane': lane,
                      'range': LOAD_RANGE, 'rel0': rel, 'start_err': err, 'fallen': abs(rel[1][2, 2]) < 0.7}

    def target_pose(self, target, keep):
        """하차 목표(world, 킥보드 중심 바닥 + yaw) -> 킥보드 모델 pose. Kickspot 이면 거치 자세(앞바퀴 축 높이·기울기)"""
        o = target.orientation
        yaw = math.atan2(2 * (o.w * o.z + o.x * o.y), 1 - 2 * (o.y * o.y + o.z * o.z))
        p = np.array([target.position.x, target.position.y, 0.0])
        if not keep:
            return p, rz(yaw)
        rm = rz(yaw) @ ry(self.kickspot['pitch'])
        ax = p + rz(yaw) @ np.array([self.axle[0], 0.0, 0.0])
        ax[2] = self.kickspot['front_axle_z']
        return ax - rm @ np.array(self.axle), rm

    def plan_unload(self, sid, target, keep=False):
        name = f'cargo_{sid}'
        if name not in self.slots or name not in self.cargo_rel:
            return f'적재함에 {name} 없음', None
        rp = self.get_robot_pose()
        if rp is None:
            return 'robot pose 없음', None
        lane = self.slots.index(name)
        k0, k1 = UNLOAD_RANGE
        base = compose(rp, inverse(self.rig.root[k0]))           # 애니메이션 기준 world (하차 시작 = 지금 로봇)
        end = compose(compose(base, self.rig.root[k1 - 1]), self.rig.scooter[lane][k1 - 1])
        goal = self.target_pose(target, keep)
        corr = compose(goal, inverse(end))                        # world 에서 왼쪽에 곱하는 보정
        # 거리·방향 차이는 끝 자리와 목표를 직접 비교한다. corr 의 평행이동 성분은 world 원점 둘레 회전이 섞여
        # 원점에서 먼 곳(캠퍼스 280 m)에서는 방향이 8° 만 달라도 37 m 로 나와 멀쩡한 하차를 거절했다 (k04 P24)
        dist = float(np.linalg.norm((goal[0] - end[0])[:2]))
        dyaw = abs(wrap(heading(goal[1]) - heading(end[1])))
        if dist > 1.5 or dyaw > MAX_START_ERR[1]:
            return (f'하차 목표가 너무 멂 (애니메이션 끝 자리에서 {dist:.2f} m, 방향 차 {math.degrees(dyaw):.0f}°. '
                    f'로봇 앞이 목표 앞 1.12 m 에 서야 함)'), None
        return None, {'sid': sid, 'cargo': name, 'lane': lane, 'range': UNLOAD_RANGE, 'keep': keep,
                      'base': base, 'corr': corr, 'pivot': end[0], 'goal': goal}

    # ---------------- 주기 처리 ----------------
    def tick(self):
        if tuple(self.slots) != self.lane_sent:
            self.lane_sent = tuple(self.slots)
            self.lane_pub.publish(Int8(data=self.slots.index(None) if None in self.slots else -1))
        rp = self.get_robot_pose()
        if rp is None:
            return
        updates = {}
        a = self.anim
        if a is not None:
            self.step_anim(a, rp, updates)
        for name, rel in self.cargo_rel.items():
            updates[name] = compose(rp, rel)
        if updates:
            self.send_poses(updates, force=a is not None)

    def frame_of(self, a):
        """-> (트랙 번호, 준비 단계 진행률 0~1 또는 None, 끝났나)"""
        t = (self.now() - a['t0']) / a['ts']
        if t < PREP_SEC:
            return a['range'][0], t / PREP_SEC, False
        k = a['range'][0] + int((t - PREP_SEC) * self.rig.fps)
        return min(k, a['range'][1] - 1), None, k >= a['range'][1]

    def step_anim(self, a, rp, updates):
        k, prep, finished = self.frame_of(a)
        lane = a['lane']
        if finished:
            return self.finish_anim(a, rp)
        q = self.rig.Q[lane][k]
        if prep is not None:
            q = a['q_start'] + ease(prep) * (q - a['q_start'])
        self.send_joints(q)
        name = 'PREP' if prep is not None else self.rig.phase(k)
        if a['phase'] != name:
            a['phase'] = name
            el = (self.now() - a['t0']) / a['ts']
            a['gh'].publish_feedback(a['fb'](phase=name, progress=float(min(1.0, el * a['ts'] / a['total']))))
        anim_rel = self.rig.scooter[lane][k]
        if a['mode'] == 'load':
            if k < GRIP_K:
                rel = a['rel0']
            else:
                rel = blend(a['rel0'], anim_rel, ease((k - GRIP_K) / (UPRIGHT_K - GRIP_K)))
            updates[a['cargo']] = compose(rp, rel)
            return
        # 하차: world = 시작 로봇 · 로봇 이동(애니메이션) · 킥보드(로봇 기준), 목표 쪽 보정을 미는 동안 섞음
        w = compose(compose(a['base'], self.rig.root[k]), anim_rel)
        u = ease((k - PUSH_K) / (LATCH_K - PUSH_K))
        corr = blend_about(a['corr'], a['pivot'], u)
        updates[a['cargo']] = compose(corr, w)
        self.drive_reverse(a, k)

    def drive_reverse(self, a, k):
        """하차 끝 후진: 애니메이션의 로봇 이동(robot_root)을 /cmd_vel 로"""
        if not self.get_parameter('drive_reverse').value:
            return
        r0, r1 = self.rig.root[k], self.rig.root[min(k + 1, len(self.rig.root) - 1)]
        v = float((r1[0] - r0[0])[0]) * self.rig.fps / a['ts']
        if abs(v) > 1e-4:
            tw = Twist()
            tw.linear.x = v
            self.cmd_vel.publish(tw)
            a['reversing'] = True
        elif a['reversing']:
            self.cmd_vel.publish(Twist())
            a['reversing'] = False

    def finish_anim(self, a, rp):
        self.q_force = True
        self.send_joints(np.zeros(len(self.rig.names)))           # 수송 자세
        if a.get('reversing'):
            self.cmd_vel.publish(Twist())
        lane = a['lane']
        if a['mode'] == 'load':
            self.cargo_rel[a['cargo']] = self.rig.scooter[lane][LOAD_RANGE[1] - 1]
            self.slots[lane] = a['cargo']
            self.get_logger().info(f'적재 완료: 킥보드 {a["sid"]} ({"왼" if lane == 0 else "오른"}쪽 레인)')
        else:
            pw, rw = a['goal']
            if a['keep']:
                self.station_cargo.add(a['cargo'])
                self.send_poses({a['cargo']: (pw, rw)}, force=True)
                where = 'Kickspot 충전 자리'
            else:
                self.create_visual_copy(f'scooter_{a["sid"]}', pw, rw, collide=True)
                self.remove(a['cargo'])
                where = '주차구역'
            p_floor = pw.copy()
            p_floor[2] = 0.0
            a['placed'] = (p_floor, rz(heading(rw)))
            self.get_logger().info(f'하차 완료: 킥보드 {a["sid"]} {where}')
        self.last_sent = None
        self.anim = None
        a['done'].set()

    def send_poses(self, updates, force=False):
        key = {n: (np.round(p, 3).tobytes(), np.round(r, 3).tobytes()) for n, (p, r) in updates.items()}
        if not force and key == self.last_sent:
            return                                      # 로봇이 서 있으면 보내지 않음
        self.last_sent = key
        req = Pose_V()
        for n, (p, r) in updates.items():
            m = req.pose.add()
            m.name = n
            m.position.x, m.position.y, m.position.z = (float(v) for v in p)
            q = mat_to_quat(r)
            m.orientation.w, m.orientation.x, m.orientation.y, m.orientation.z = (float(v) for v in q)
        self.gz_request('set_pose_vector', req, Pose_V, timeout=300)


def main():
    rclpy.init()
    spin_single(ArmLoader())
