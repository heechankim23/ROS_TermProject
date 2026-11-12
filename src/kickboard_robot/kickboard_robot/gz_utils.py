"""Gazebo(Harmonic, gz sim 8) 엔티티 조작. `gz service` CLI 를 백그라운드 스레드로 호출한다."""
import math
import shutil
import subprocess
import threading
import time

# 킥보드 외형: 옆에서 보면 'ㄴ' 자 — 앞쪽 세로 기둥(+핸들) + 낮은 발판 + 바퀴 2개.  +x = 앞(기둥 쪽)
SCOOTER_L, SCOOTER_W = 0.6, 0.17          # 태그 면 기준 외곽: 앞뒤 ±0.3, 좌우 ±0.085 (sim_tag_detector 와 공유)
FALLEN_Z = 0.21                           # 넘어졌을 때(roll 90°) 핸들이 땅에 묻히지 않게 띄우는 높이

# 상태별 색 (Gazebo 에서 한눈에 보이게).  발판 = 상태,  기둥·핸들 = 배터리
DECK_COLORS = {
    'OK': '0.1 0.65 0.25',           # 초록: 정상 (구역 안, 서 있음, 배터리 정상)
    'FALLEN': '0.9 0.1 0.1',         # 빨강: 넘어짐
    'OUT_OF_ZONE': '1.0 0.5 0.0',    # 주황: 주차구역 밖
    'LOW_BATTERY': '0.6 0.2 0.9',    # 보라: 배터리 부족 (서 있고 구역 안)
}
STEM_NORMAL, STEM_LOW_BATTERY = '0.75 0.75 0.78', '0.6 0.2 0.9'   # 회색 / 보라 (배터리 부족이면 사유와 무관하게)


def scooter_state(tilt_deg, in_zone, battery_pct, tilt_th=45.0, batt_th=15.0):
    """요청 사유와 같은 우선순위: 넘어짐 > 구역 밖 > 배터리 부족 > 정상"""
    if tilt_deg > tilt_th:
        return 'FALLEN'
    if not in_zone:
        return 'OUT_OF_ZONE'
    if battery_pct < batt_th:
        return 'LOW_BATTERY'
    return 'OK'


def scooter_colors(state, low_battery):
    """-> (발판 색, 기둥·핸들 색)"""
    return DECK_COLORS[state], (STEM_LOW_BATTERY if low_battery else STEM_NORMAL)


def _box(name, x, y, z, sx, sy, sz, rgb, collide=False, roll=0.0, yaw=0.0):
    pose = f"<pose>{x:.3f} {y:.3f} {z:.3f} {roll:.4f} 0 {yaw:.4f}</pose>"
    geo = f"<geometry><box><size>{sx} {sy} {sz}</size></box></geometry>"
    mat = f"<material><ambient>{rgb} 1</ambient><diffuse>{rgb} 1</diffuse></material>"
    out = f"<visual name='{name}'>{pose}{geo}{mat}</visual>"
    if collide:
        out += f"<collision name='{name}_c'>{pose}{geo}</collision>"
    return out


def _wheel(name, x):
    return (f"<visual name='{name}'><pose>{x} 0 0.06 1.5708 0 0</pose>"
            f"<geometry><cylinder><radius>0.06</radius><length>0.04</length></cylinder></geometry>"
            f"<material><ambient>0.05 0.05 0.05 1</ambient><diffuse>0.05 0.05 0.05 1</diffuse></material></visual>")


def _tag(i, x, y, z, yaw):
    """태그 자리: 흰 판 + 검은 사각형 (AprilTag 처럼 보이게).
    TODO(A): 실제 tag36h11 텍스처로 바꾸면 apriltag_ros 백엔드에서 검출 가능."""
    c, s = math.cos(yaw), math.sin(yaw)
    return (_box(f'tag{i}', x + 0.004 * c, y + 0.004 * s, z, 0.004, 0.2, 0.2, '1 1 1', yaw=yaw)
            + _box(f'tag{i}_in', x + 0.007 * c, y + 0.007 * s, z, 0.004, 0.11, 0.11, '0 0 0', yaw=yaw))


def body_link(name):
    """킥보드 링크 이름 (모델마다 다르게 — Gazebo Entity Tree 에서 구분하기 쉽게)"""
    return f'{name}_body'


def scooter_sdf(name, collide=True, deck=DECK_COLORS['OK'], stem=STEM_NORMAL):
    """collide=False: 충돌체 없는 복사본 (로봇팔이 들고 다니거나 적재함에 실린 킥보드)"""
    c = collide
    L, W = SCOOTER_L, SCOOTER_W
    dark = '0.12 0.12 0.12'
    body = (
        _box('deck', 0.0, 0, 0.105, L - 0.04, 0.16, 0.05, deck, collide=c)                # 발판
        + _box('stem', L / 2 - 0.03, 0, 0.52, 0.05, 0.05, 0.8, stem, collide=c)          # 세로 기둥 (ㄴ 의 세로)
        + _box('bar', L / 2 - 0.03, 0, 0.93, 0.04, 0.42, 0.04, stem)                      # 핸들
        + _box('fender', -L / 2 + 0.005, 0, 0.2, 0.01, 0.16, 0.2, dark, collide=c)       # 뒤 흙받이 (뒤 태그용)
        + _box('side_l', 0.0, W / 2 - 0.005, 0.22, 0.24, 0.01, 0.2, dark)                # 옆 태그 판
        + _box('side_r', 0.0, -W / 2 + 0.005, 0.22, 0.24, 0.01, 0.2, dark)
        + _wheel('wheel_f', L / 2 - 0.06) + _wheel('wheel_r', -L / 2 + 0.06)
    )
    tags = (_tag(0, L / 2 - 0.005, 0, 0.42, 0.0)                 # 앞: 기둥 앞면
            + _tag(1, -L / 2, 0, 0.2, math.pi)                   # 뒤: 흙받이
            + _tag(2, 0, W / 2, 0.22, math.pi / 2)               # 왼쪽
            + _tag(3, 0, -W / 2, 0.22, -math.pi / 2))            # 오른쪽
    return (f"<?xml version='1.0'?><sdf version='1.9'><model name='{name}'><static>true</static>"
            f"<link name='{body_link(name)}'>{body}{tags}</link></model></sdf>")


class GzWorld:
    def __init__(self, world, logger, enabled=True):
        self.world = world
        self.log = logger
        self.enabled = enabled and shutil.which('gz') is not None
        if enabled and not self.enabled:
            logger.warn('gz CLI 를 찾지 못했습니다. Gazebo 모델 조작을 건너뜁니다.')

    def _run(self, args, what):
        def job():
            try:
                r = subprocess.run(args, capture_output=True, text=True, timeout=15)
                if r.returncode != 0 or 'data: false' in r.stdout:
                    self.log.warn(f'gz {what} 실패: {r.stdout.strip()} {r.stderr.strip()}')
            except Exception as e:  # noqa: BLE001
                self.log.warn(f'gz {what} 예외: {e}')
        if self.enabled:
            threading.Thread(target=job, daemon=True).start()

    def spawn(self, name, x, y, yaw, roll=0.0, colors=None):
        z = FALLEN_Z if abs(roll) > 0.1 else 0.0
        deck, stem = colors or (DECK_COLORS['OK'], STEM_NORMAL)
        self._run(['ros2', 'run', 'ros_gz_sim', 'create', '-world', self.world,
                   '-name', name, '-string', scooter_sdf(name, deck=deck, stem=stem),
                   '-x', str(x), '-y', str(y), '-z', str(z), '-R', str(roll), '-Y', str(yaw)],
                  f'spawn {name}')

    def spawn_box(self, name, x, y, size=(0.5, 0.5, 0.5)):
        sx, sy, sz = size
        sdf = (f"<?xml version='1.0'?><sdf version='1.9'><model name='{name}'><static>true</static>"
               f"<link name='l'><collision name='c'><pose>0 0 {sz / 2} 0 0 0</pose><geometry><box>"
               f"<size>{sx} {sy} {sz}</size></box></geometry></collision><visual name='v'>"
               f"<pose>0 0 {sz / 2} 0 0 0</pose><geometry><box><size>{sx} {sy} {sz}</size></box>"
               f"</geometry><material><ambient>0.8 0.1 0.1 1</ambient><diffuse>0.8 0.1 0.1 1</diffuse>"
               f"</material></visual></link></model></sdf>")
        self._run(['ros2', 'run', 'ros_gz_sim', 'create', '-world', self.world, '-name', name,
                   '-string', sdf, '-x', str(x), '-y', str(y), '-z', '0'], f'spawn {name}')

    def set_pose(self, name, x, y, yaw, roll=0.0):
        cr, sr = math.cos(roll / 2), math.sin(roll / 2)
        cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
        qw, qx, qy, qz = cr * cy, sr * cy, sr * sy, cr * sy        # yaw * roll
        z = FALLEN_Z if abs(roll) > 0.1 else 0.0
        req = (f'name: "{name}" position {{x: {x} y: {y} z: {z}}} '
               f'orientation {{x: {qx} y: {qy} z: {qz} w: {qw}}}')
        self._run(['gz', 'service', '-s', f'/world/{self.world}/set_pose',
                   '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
                   '--timeout', '3000', '--req', req], f'set_pose {name}')

    def respawn(self, name, x, y, yaw, roll, colors):
        """색을 바꾸려고 모델을 지웠다가 같은 자리에 새 색으로 다시 만든다.
        (gz sim 8 의 visual_config / material_color 는 이 VM(ogre1)에서 화면에 반영되지 않았다)"""
        if not self.enabled:
            return
        z = FALLEN_Z if abs(roll) > 0.1 else 0.0
        cr, sr = math.cos(roll / 2), math.sin(roll / 2)
        cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
        qw, qx, qy, qz = cr * cy, sr * cy, sr * sy, cr * sy        # yaw * roll
        deck, stem = colors
        sdf = scooter_sdf(name, deck=deck, stem=stem)       # 작은따옴표만 씀 -> 요청 문자열에 그대로 넣을 수 있음
        assert '"' not in sdf and '\\' not in sdf
        srv = f'/world/{self.world}'

        def call(service, reqtype, req):
            r = subprocess.run(['gz', 'service', '-s', f'{srv}/{service}', '--reqtype', reqtype,
                                '--reptype', 'gz.msgs.Boolean', '--timeout', '3000', '--req', req],
                               capture_output=True, text=True, timeout=15)
            return r.returncode == 0 and 'data: false' not in r.stdout

        def job():
            try:
                call('remove', 'gz.msgs.Entity', f'name: "{name}" type: MODEL')
                time.sleep(0.3)                 # 지운 뒤에 만들어야 이름이 겹치지 않음
                ok = call('create', 'gz.msgs.EntityFactory',
                          f'sdf: "{sdf}" name: "{name}" '
                          f'pose {{position {{x: {x} y: {y} z: {z}}} '
                          f'orientation {{x: {qx} y: {qy} z: {qz} w: {qw}}}}}')
                if not ok:
                    self.log.warn(f'gz 색 바꾸기(재생성) 실패: {name}')
            except Exception as e:  # noqa: BLE001
                self.log.warn(f'gz 색 바꾸기 예외 {name}: {e}')
        threading.Thread(target=job, daemon=True).start()

    def remove(self, name):
        self._run(['gz', 'service', '-s', f'/world/{self.world}/remove',
                   '--reqtype', 'gz.msgs.Entity', '--reptype', 'gz.msgs.Boolean',
                   '--timeout', '3000', '--req', f'name: "{name}" type: MODEL'], f'remove {name}')
