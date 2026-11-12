"""[시뮬레이션 전용, sim:=gz] 로봇팔 적재·하역 애니메이션 + 적재함 화물 표시.

LoadScooter 액션(/arm_loader/load)을 받으면:
  REACH   팔을 펴서 킥보드 발판 위로
  DESCEND 발판 높이까지 내려감
  GRIP    집게를 닫음 -> 원래 킥보드 모델을 '충돌체 없는 복사본(cargo_<id>)'으로 바꿔치기
  LIFT    들어 올리면서 넘어진 킥보드를 똑바로 세움
  SWING   팔을 180° 돌려 적재함 빈 칸 위로 (킥보드는 칸 방향으로 돌림)
  PLACE   칸에 내려놓음
  RELEASE 집게를 벌림
  RETRACT 접은 자세로 복귀
UnloadScooter 액션(/arm_loader/unload)을 받으면:
  remove_only  충전소 하역 — 적재함에서 바로 지움
  그 외        칸에서 집어 꺼내 target 위치(주차구역)에 세우고, 그 자리에 원래 킥보드 모델(충돌체 있음)을 되살린다
적재함에 실린 킥보드는 로봇이 움직이는 동안 매 주기 로봇 pose 를 따라가게 set_pose 한다.
안전장치: /robot_status.load_count 가 0 이 되었는데 적재함에 남은 게 있으면 지운다.

Gazebo 와의 통신
  gz-transport 파이썬 바인딩으로 직접: 관절 명령 /kickboard_robot/arm/<관절> (model.sdf 의 JointPositionController),
                                    set_pose_vector / create / remove 서비스, 킥보드 pose (적재 시작 때 한 번만)
  ROS 브리지로: /joint_states (관절 실측), /sim/robot_pose (로봇 world pose 정답값)
  ※ gz 파이썬 subscribe 를 계속 걸어 두면 안 된다. gz 수신 스레드가 파이썬 콜백(GIL)을 기다리는 동안
    서비스 응답이 처리되지 않아 request() 가 타임아웃 나고 set_pose 가 적용되지 않는다 (실측: 거의 100% 실패).
팔 치수·적재함 칸 위치는 models/kickboard_robot/model.sdf 와 같아야 한다.
"""
import math
import threading

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

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

# ---------------- 로봇 치수 (model.sdf 와 일치) ----------------
ARM_BASE = np.array([0.50, 0.0, 0.95])      # 회전축 x,y + 어깨 높이
L1, L2 = 0.75, 0.75                          # 윗팔, 아랫팔
LG = 0.20                                    # 손목 -> 집게 중심(잡는 점)
FINGER_OPEN_GAP = 0.11                       # 완전히 벌렸을 때 손가락 안쪽 면의 y (집게 중심 기준)
# 접은 자세: 적재함 쪽(yaw 180°)을 보고, 집게(잡는 점)를 적재함 앞쪽 위 (r 0.45, 높이 LIFT_Z) 에 둔다.
# 실린 킥보드 핸들(1.26 m)보다 높아서 닿지 않는다. 값 = ik(pi, 0.45, 1.40). model.sdf initial_position 과 같게.
HOME = {'arm_yaw': math.pi, 'arm_shoulder': -1.9809, 'arm_elbow': 2.0314,
        'arm_wrist_pitch': -0.0504, 'arm_wrist_rot': 0.0, 'arm_finger_l': 0.0, 'arm_finger_r': 0.0}
HOME_R, HOME_Z = 0.45, 1.40
JOINTS = tuple(HOME)
# 적재함 칸 (뒤 칸부터 채운다 — 앞쪽 칸이 비어 있어야 팔과 들고 가는 킥보드가 실린 킥보드를 지나가지 않음)
# (x, 방향) : 방향 +1 이면 킥보드 앞(기둥)이 로봇 +y 쪽. 이웃 칸끼리 반대로 놓아 핸들이 겹치지 않게 한다.
SLOTS = ((-0.58, +1), (-0.36, -1), (-0.14, +1), (0.08, -1))
SLOT_Z = 0.31                                # 적재함 바닥 윗면
LIFT_Z = 1.40                                # 옮길 때 잡는 점 높이 (캡·실린 킥보드 핸들 1.26 m 위로)
# 킥보드 (gz_utils.scooter_sdf): 발판 중심과 반 치수
DECK_C = np.array([0.0, 0.0, 0.105])
DECK_HALF = np.array([0.28, 0.08, 0.025])


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


def fk(q):
    """관절값 -> (잡는 점 위치, 집게 회전행렬), 로봇(base_footprint) 좌표"""
    yaw, sh, el, wp, wr = (q[j] for j in JOINTS[:5])
    e1, e12 = -sh, -(sh + el)
    wrist = ARM_BASE + rz(yaw) @ np.array([L1 * math.cos(e1) + L2 * math.cos(e12), 0.0,
                                           L1 * math.sin(e1) + L2 * math.sin(e12)])
    rg = rz(yaw) @ ry(sh + el + wp) @ rz(wr)
    return wrist + rg @ np.array([0, 0, -LG]), rg


def ik(yaw, r, z, rot, finger):
    """잡는 점을 원통 좌표 (팔 회전각 yaw, 회전축에서 거리 r, 높이 z) 에, 집게는 수직 아래로. 팔꿈치는 위로."""
    dz = z + LG - ARM_BASE[2]
    d = (r * r + dz * dz - L1 * L1 - L2 * L2) / (2 * L1 * L2)
    if abs(d) > 1.0:
        return None
    t2 = -math.acos(d)
    t1 = math.atan2(dz, r) - math.atan2(L2 * math.sin(t2), L1 + L2 * math.cos(t2))
    sh, el = -t1, -t2
    return {'arm_yaw': yaw, 'arm_shoulder': sh, 'arm_elbow': el, 'arm_wrist_pitch': -(sh + el),
            'arm_wrist_rot': rot, 'arm_finger_l': finger, 'arm_finger_r': finger}


def cyl(p):
    """로봇 좌표의 점 -> (yaw, r, z) (팔 회전축 기준)"""
    dx, dy = p[0] - ARM_BASE[0], p[1] - ARM_BASE[1]
    return math.atan2(dy, dx), math.hypot(dx, dy), p[2]


class ArmLoader(Node):
    def __init__(self):
        super().__init__('arm_loader')
        self.declare_parameter('world_name', 'kickboard_world')
        self.declare_parameter('robot_name', 'kickboard_robot')
        self.declare_parameter('time_scale', 1.0)          # >1 이면 동작이 느려짐 (발표 녹화용)
        self.declare_parameter('rate_hz', 25.0)
        self.declare_parameter('tilt_threshold_deg', 45.0)       # 킥보드 색 (scooter_fleet_sim 과 같은 기준)
        self.declare_parameter('battery_threshold_pct', 15.0)
        self.world = self.get_parameter('world_name').value
        self.robot = self.get_parameter('robot_name').value

        self.lock = threading.Lock()
        self.robot_pose = None   # (위치 np(3), 회전 np(3x3)) world
        self.joints = {}         # 관절 이름 -> 위치 (실측)
        self.gz = GzNode()
        self.cmd = {j: self.gz.advertise(f'/{self.robot}/arm/{j}', Double) for j in JOINTS}

        self.slots = [None] * len(SLOTS)     # 칸 -> 실린 모델 이름
        self.cargo_rel = {}                  # 실린 모델 -> (p, R) 로봇 좌표
        self.station_cargo = set()           # 충전소 충전 자리에 세워 둔 모델 (로봇을 따라가지 않음)
        self.anim = None                     # 진행 중인 적재 동작 (dict)
        self.home_yaw = math.pi              # 접은 자세의 회전각 (+pi 또는 -pi: 돌아온 방향에 따라)
        self.last_sent = None
        self.load_count = 0

        self.cb = ReentrantCallbackGroup()
        self.create_subscription(RobotStatus, '/robot_status', self.on_status, 10, callback_group=self.cb)
        self.create_subscription(PoseStamped, '/sim/robot_pose', self.on_robot_pose, 10, callback_group=self.cb)
        self.create_subscription(JointState, '/joint_states', self.on_joints, 10, callback_group=self.cb)
        self.scooter_info = {}   # id -> ScooterState (들고 가는 복사본도 같은 상태 색으로)
        self.create_subscription(ScooterState, '/scooter_states',
                                 lambda m: self.scooter_info.__setitem__(m.id, m), 20, callback_group=self.cb)
        # 적재·하역은 중간에 못 멈춤 (킥보드를 든 채로 끝남) -> 취소 거절
        ActionServer(self, LoadScooter, '/arm_loader/load', self.execute_load, goal_callback=self.on_goal,
                     cancel_callback=lambda g: CancelResponse.REJECT, callback_group=self.cb)
        ActionServer(self, UnloadScooter, '/arm_loader/unload', self.execute_unload, goal_callback=self.on_goal,
                     cancel_callback=lambda g: CancelResponse.REJECT, callback_group=self.cb)
        self.create_timer(1.0 / self.get_parameter('rate_hz').value, self.tick, callback_group=self.cb)
        self.get_logger().info('로봇팔 적재기 준비 (/arm_loader/load)')

    # ---------------- 입력 ----------------
    def on_robot_pose(self, msg):
        p, o = msg.pose.position, msg.pose.orientation
        with self.lock:
            self.robot_pose = (np.array([p.x, p.y, p.z]), quat_to_mat((o.w, o.x, o.y, o.z)))

    def on_joints(self, msg):
        js = {n: q for n, q in zip(msg.name, msg.position) if n.startswith('arm_')}
        with self.lock:
            self.joints.update(js)

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
        for j in JOINTS:
            self.cmd[j].publish(Double(data=float(q[j])))

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
                           self.get_parameter('tilt_threshold_deg').value, bt)
        deck, stem = scooter_colors(st, s.battery_pct < bt)
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
        self.slots = [None] * len(SLOTS)
        self.last_sent = None
        if names:
            self.get_logger().info(f'하역: 적재함 비움 ({len(names)}대)')

    # ---------------- 액션 ----------------
    def on_goal(self, goal):
        if self.anim is not None:
            self.get_logger().warn('로봇팔 동작 중 -> 거절')
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
            self.get_logger().warn(f'킥보드 {sid} {"적재" if mode == "load" else "하역"} 불가: {err}')
            res.success, res.message = False, err
            gh.abort()
            return res
        anim.update(gh=gh, mode=mode, done=threading.Event(), t0=self.now(),
                    fb=LoadScooter.Feedback if mode == 'load' else UnloadScooter.Feedback)
        self.anim = anim
        if mode == 'load':
            self.get_logger().info(f'로봇팔: 킥보드 {sid} -> {anim["slot"] + 1}번 칸'
                                   + (' (넘어진 킥보드 세우기)' if anim['fallen'] else ''))
        else:
            where = '충전소 충전 자리' if anim['keep'] else '주차구역'
            self.get_logger().info(f'로봇팔: {anim["slot"] + 1}번 칸 킥보드 {sid} -> {where}')
        anim['done'].wait()
        res.success = True
        if mode == 'load':
            res.message, res.slot = 'loaded', anim['slot']
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

    @staticmethod
    def grip_params(p_obj, r_obj):
        """물체(로봇 좌표 pose)의 발판을 잡는 자세: 위치(원통 좌표), 집게 회전, 손가락, 잡은 뒤 상대 자세"""
        g = p_obj + r_obj @ DECK_C                     # 잡는 점 = 발판 중심
        heading = math.atan2(r_obj[1, 0], r_obj[0, 0])  # 킥보드 길이 방향 (넘어져도 x 축은 수평)
        yaw, r, z = cyl(g)
        rot = wrap(heading - yaw)                      # 집게 x 를 킥보드 길이 방향에 맞춤 -> 손가락이 발판 양옆을 잡음
        if rot > math.pi / 2:
            rot -= math.pi
        elif rot < -math.pi / 2:
            rot += math.pi
        r_grip = rz(yaw + rot)
        a = r_grip[:, 1]                               # 손가락이 닫히는 방향
        half = float(np.sum(np.abs(a @ r_obj) * DECK_HALF))
        finger = float(np.clip(-(FINGER_OPEN_GAP - half) - 0.005, -0.11, 0.0))
        rel0 = r_grip.T @ r_obj                        # 잡은 순간 집게 기준 킥보드 자세
        rel_heading = math.atan2(rel0[1, 0], rel0[0, 0])
        return {'yaw': yaw, 'r': r, 'z': z, 'rot': rot, 'finger': finger,
                'rel0': rel0, 'rel_heading': rel_heading, 'rel_up': rz(rel_heading)}

    @staticmethod
    def place_params(p_dest, yaw_dest, gp, yaw_from):
        """세운 킥보드를 p_dest(바닥 중심), yaw_dest 로 놓을 때의 (yaw, r, z, 집게 회전)"""
        yaw, r, z = cyl(p_dest + DECK_C)
        yaw = yaw_from + wrap(yaw - yaw_from)          # 가까운 쪽으로 돌기
        rot = wrap(yaw_dest - yaw - gp['rel_heading'])
        return yaw, r, z, gp['rot'] + wrap(rot - gp['rot'])

    def build(self, wp, seq):
        """seq = [(단계 이름, 기본 시간, 시작 경유점, 끝 경유점)] -> (오류, 단계 목록, 총 시간)"""
        for k, v in wp.items():
            if ik(*v) is None:
                rp = self.get_robot_pose()
                where = '' if rp is None else f', robot=({rp[0][0]:.2f}, {rp[0][1]:.2f})'
                return f'팔이 닿지 않음 ({k}: r={v[1]:.2f} z={v[2]:.2f}{where})', None, 0.0
        ts = self.get_parameter('time_scale').value
        phases, t = [], 0.0
        for name, dur, a, b in seq:
            phases.append((name, t, t + dur * ts, wp[a], wp[b]))
            t += dur * ts
        return None, phases, t

    def plan_load(self, sid):
        """바닥의 킥보드 -> 적재함 빈 칸 (로봇 좌표). 실패하면 (이유, None)"""
        src = f'scooter_{sid}'
        # 킥보드(static)는 pose/info 스냅샷에서, 로봇은 계속 들어오는 /sim/robot_pose 에서.
        # (스냅샷의 로봇 pose 가 예전 값으로 와서 '팔이 닿지 않음 r=3 m' 로 실패한 적이 있음)
        sp = self.snapshot_poses().get(src)
        rp = self.get_robot_pose()
        if rp is None:
            return 'robot pose 없음 (/sim/robot_pose 미수신)', None
        if sp is None:
            return f'Gazebo 에 {src} 모델 없음', None
        try:
            slot = self.slots.index(None)
        except ValueError:
            return '적재함 가득 참', None
        r_inv = rp[1].T
        r_s0 = r_inv @ sp[1]
        gp = self.grip_params(r_inv @ (sp[0] - rp[0]), r_s0)
        h = self.home_yaw
        y = h + wrap(gp['yaw'] - h)                    # 접은 자세(뒤)에서 앞으로 도는 각
        sx, sgn = SLOTS[slot]
        yaw_s, r_s, z_s, rot_s = self.place_params(np.array([sx, 0.0, SLOT_Z]), sgn * math.pi / 2, gp, y)
        r, z, rot, f = gp['r'], gp['z'], gp['rot'], gp['finger']
        wp = {   # 경유점: (yaw, r, z, 집게 회전, 손가락).  돌 때는 항상 LIFT_Z 높이 (캡·적재함 위)
            'home': (h, HOME_R, HOME_Z, 0.0, 0.0),
            'high': (y, r, LIFT_Z, rot, 0.0), 'grasp': (y, r, z, rot, 0.0), 'grip': (y, r, z, rot, f),
            'lift': (y, min(r, 0.85), LIFT_Z, rot, f),
            'over': (yaw_s, r_s, LIFT_Z, rot_s, f), 'place': (yaw_s, r_s, z_s, rot_s, f),
            'open': (yaw_s, r_s, z_s, rot_s, 0.0), 'rise': (yaw_s, r_s, LIFT_Z, rot_s, 0.0),
            'end': (yaw_s, HOME_R, HOME_Z, 0.0, 0.0),
        }
        err, phases, total = self.build(wp, (
            ('REACH', 4.0, 'home', 'high'), ('DESCEND', 2.5, 'high', 'grasp'), ('GRIP', 1.0, 'grasp', 'grip'),
            ('LIFT', 3.0, 'grip', 'lift'), ('SWING', 5.0, 'lift', 'over'), ('PLACE', 2.5, 'over', 'place'),
            ('RELEASE', 0.8, 'place', 'open'), ('RISE', 1.5, 'open', 'rise'), ('RETRACT', 1.5, 'rise', 'end')))
        if err:
            return err, None
        return None, {'sid': sid, 'src': src, 'src_pose': sp, 'cargo': f'cargo_{sid}', 'slot': slot,
                      'phases': phases, 'total': total, 'rel0': gp['rel0'], 'rel_up': gp['rel_up'],
                      'attached': False, 'fallen': abs(r_s0[2, 2]) < 0.7, 'phase': None, 'end_yaw': yaw_s}

    def plan_unload(self, sid, target, keep=False):
        """적재함 칸의 킥보드 -> target(world 바닥 중심 + yaw) 에 세움.
        keep=True 면 충전소 충전 자리: 복사본(cargo_<id>)을 그대로 두고 정상 킥보드로 되돌리지 않는다"""
        name = f'cargo_{sid}'
        if name not in self.slots or name not in self.cargo_rel:
            return f'적재함에 {name} 없음', None
        rp = self.get_robot_pose()
        if rp is None:
            return 'robot pose 없음', None
        slot = self.slots.index(name)
        p_obj, r_obj = self.cargo_rel[name]
        gp = self.grip_params(p_obj, r_obj)
        o = target.orientation
        yaw_w = math.atan2(2 * (o.w * o.z + o.x * o.y), 1 - 2 * (o.y * o.y + o.z * o.z))
        robot_yaw = math.atan2(rp[1][1, 0], rp[1][0, 0])
        p_dest = rp[1].T @ (np.array([target.position.x, target.position.y, 0.0]) - rp[0])
        p_dest[2] = 0.0
        h = self.home_yaw
        y = h + wrap(gp['yaw'] - h)                    # 칸은 뒤쪽 (±180°) = 접은 자세 방향
        yaw_d, r_d, z_d, rot_d = self.place_params(p_dest, wrap(yaw_w - robot_yaw), gp, y)
        end_yaw = yaw_d + wrap(math.pi - yaw_d)        # 내려놓은 뒤 가까운 쪽으로 돌아 다시 적재함 쪽 (±pi)
        r, z, rot, f = gp['r'], gp['z'], gp['rot'], gp['finger']
        wp = {
            'home': (h, HOME_R, HOME_Z, 0.0, 0.0),
            'above': (y, r, LIFT_Z, rot, 0.0), 'grasp': (y, r, z, rot, 0.0), 'grip': (y, r, z, rot, f),
            'lift': (y, r, LIFT_Z, rot, f),
            'over': (yaw_d, r_d, LIFT_Z, rot_d, f), 'place': (yaw_d, r_d, z_d, rot_d, f),
            'open': (yaw_d, r_d, z_d, rot_d, 0.0), 'rise': (yaw_d, r_d, LIFT_Z, rot_d, 0.0),
            'end': (end_yaw, HOME_R, HOME_Z, 0.0, 0.0),
        }
        err, phases, total = self.build(wp, (
            ('MOVE', 2.0, 'home', 'above'), ('DESCEND', 2.0, 'above', 'grasp'),
            ('GRIP', 1.0, 'grasp', 'grip'), ('LIFT', 2.0, 'grip', 'lift'), ('SWING', 5.0, 'lift', 'over'),
            ('PLACE', 2.5, 'over', 'place'), ('RELEASE', 0.8, 'place', 'open'), ('RISE', 2.0, 'open', 'rise'),
            ('RETRACT', 4.0, 'rise', 'end')))
        if err:
            return err, None
        return None, {'sid': sid, 'cargo': name, 'slot': slot, 'phases': phases, 'total': total,
                      'rel0': gp['rel0'], 'rel_up': gp['rel_up'], 'attached': False, 'phase': None,
                      'end_yaw': end_yaw, 'keep': keep}

    # ---------------- 주기 처리 ----------------
    def tick(self):
        rp = self.get_robot_pose()
        if rp is None:
            return
        updates = {}
        a = self.anim
        if a is not None:
            self.step_anim(a, updates)
        for name, (p, rm) in self.cargo_rel.items():
            updates[name] = (rp[1] @ p + rp[0], rp[1] @ rm)
        if updates:
            self.send_poses(updates, force=a is not None and a['attached'])

    def step_anim(self, a, updates):
        t = self.now() - a['t0']
        cur = None
        for ph in a['phases']:
            if t < ph[2]:
                cur = ph
                break
        if cur is None:                                 # 끝 -> 접은 자세 유지
            self.home_yaw = a['end_yaw']
            self.send_joints(ik(self.home_yaw, HOME_R, HOME_Z, 0.0, 0.0))
            self.anim = None
            a['done'].set()
            if a['mode'] == 'load':
                self.get_logger().info(f'로봇팔: 킥보드 {a["sid"]} 적재 완료 ({a["slot"] + 1}번 칸)')
            else:
                where = '충전소 충전 자리' if a['keep'] else '주차구역'
                self.get_logger().info(f'로봇팔: 킥보드 {a["sid"]} {where}에 세움')
            return
        name, t0, t1, w0, w1 = cur
        if a['phase'] != name:
            self.on_phase_end(a, a['phase'])
            a['phase'] = name
            a['gh'].publish_feedback(a['fb'](phase=name, progress=float(t / a['total'])))
        u = ease((t - t0) / max(t1 - t0, 1e-6))
        q = ik(*(w0[i] + u * (w1[i] - w0[i]) for i in range(5)))
        if q is not None:
            self.send_joints(q)
        if a['attached']:
            with self.lock:
                js = dict(self.joints)
            if len(js) < len(JOINTS):
                return
            pg, rg = fk(js)
            righting = 1.0 if name != 'LIFT' else u
            rs = rg @ slerp(a['rel0'], a['rel_up'], righting)
            ps = pg - rs @ DECK_C
            if name in ('RELEASE', 'RISE', 'RETRACT'):      # 놓은 뒤에는 칸에 고정
                return
            a['carried_rel'] = (ps, rs)
            rp = self.get_robot_pose()
            updates[a['cargo']] = (rp[1] @ ps + rp[0], rp[1] @ rs)

    def on_phase_end(self, a, name):
        rp = self.get_robot_pose()
        if name == 'GRIP':
            if a['mode'] == 'load':
                # 원래 모델(충돌체 있음)을 충돌체 없는 복사본으로 바꿔치기 -> 들고 다녀도 로봇과 부딪히지 않음
                sp = a['src_pose']                      # 킥보드는 static 이라 계획 때 pose 그대로
                self.create_visual_copy(a['cargo'], sp[0], sp[1], colors=self.colors_of(a['sid']))
                self.remove(a['src'])
            else:                                       # 칸에서 꺼냄 -> 이제 집게를 따라감
                self.cargo_rel.pop(a['cargo'], None)
                self.slots[a['slot']] = None
            a['attached'] = True
        elif name == 'PLACE':
            ps, rs = a.get('carried_rel', (None, None))
            if a['mode'] == 'load':
                if ps is None:
                    sx, sgn = SLOTS[a['slot']]
                    ps, rs = np.array([sx, 0.0, SLOT_Z]), rz(sgn * math.pi / 2)
                self.cargo_rel[a['cargo']] = (ps, rs)
                self.slots[a['slot']] = a['cargo']
            else:
                pw, rw = rp[1] @ ps + rp[0], rp[1] @ rs
                pw[2] = 0.0
                a['placed'] = (pw, rw)
                if a['keep']:
                    # 충전소 충전 자리: 복사본을 바닥에 내려 그대로 둔다 (로봇을 따라가지 않음)
                    self.station_cargo.add(a['cargo'])
                    self.send_poses({a['cargo']: (pw, rw)}, force=True)
                else:
                    # 주차구역: 원래 킥보드(충돌체 있음)를 되살리고 복사본은 지움 -> 다시 정상 킥보드
                    self.create_visual_copy(f'scooter_{a["sid"]}', pw, rw, collide=True)
                    self.remove(a['cargo'])
                self.last_sent = None

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
