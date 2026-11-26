"""Gazebo(Harmonic, gz sim 8) 엔티티 조작. `gz service` CLI 를 백그라운드 스레드로 호출한다."""
import math
import shutil
import subprocess
import threading
import time

# 킥보드 외형: 옆에서 보면 'ㄴ' 자 — 앞쪽 세로 기둥(+핸들) + 낮은 발판 + 바퀴 2개.  +x = 앞(기둥 쪽)
SCOOTER_L, SCOOTER_W = 0.6, 0.17          # 태그 면 기준 외곽: 앞뒤 ±0.3, 좌우 ±0.085 (sim_tag_detector 와 공유, 인식 로직용)
FALLEN_Z = 0.236                          # 넘어졌을 때(roll 90°) 핸들(반폭 0.236)이 땅에 묻히지 않게 띄우는 높이
# 외형: Blender G30 메시 (models/g30_scooter, scripts/build_blender_models.py 가 만듦).
#   모델 원점 = 앞·뒤 바퀴 축 가운데 지면, +x = 앞. 휠베이스 0.84, 앞바퀴 축 (0.42, 0, 0.127), 길이 1.17, 핸들 폭 0.47
G30_MESH = 'model://g30_scooter/meshes/'
G30_WB, G30_R = 0.84, 0.127

# 상태별 색 (Gazebo 에서 한눈에 보이게).  발판 = 상태,  기둥·핸들 = 배터리
DECK_COLORS = {
    'OK': '0.1 0.65 0.25',           # 초록: 정상 (구역 안, 서 있음, 배터리 정상)
    'FALLEN': '0.9 0.1 0.1',         # 빨강: 넘어짐
    'OUT_OF_ZONE': '1.0 0.5 0.0',    # 주황: 주차구역 밖
    'LOW_BATTERY': '0.6 0.2 0.9',    # 보라: 배터리 부족 (서 있고 구역 안)
    'BROKEN': '0.18 0.18 0.2',       # 검정: 고장 (캠퍼스 맵)
    'MISALIGNED': '0.95 0.8 0.1',    # 노랑: 주차구역 안이지만 슬롯 밖·비뚤게 (캠퍼스 맵)
}
STEM_NORMAL, STEM_LOW_BATTERY = '0.75 0.75 0.78', '0.6 0.2 0.9'   # 회색 / 보라 (배터리 부족이면 사유와 무관하게)
STEM_BROKEN, STEM_CHARGING = '0.85 0.15 0.15', '0.1 0.75 0.65'    # 빨강: 고장 / 청록: 충전 스탠드에 꽂혀 충전 중


def scooter_state(tilt_deg, in_zone, battery_pct, tilt_th=45.0, batt_th=20.0, fault='', misaligned=False,
                  charging=False):
    """요청 사유와 같은 우선순위: 넘어짐 > 고장 > 구역 밖 > 정렬 불량 > 배터리 부족 > 정상.
    fault·misaligned·charging 은 캠퍼스 맵에서만 쓴다 (기본값이면 기존 맵과 같다)"""
    if tilt_deg > tilt_th:
        return 'FALLEN'
    if fault:
        return 'BROKEN'
    if not in_zone:
        return 'MISALIGNED' if misaligned else 'OUT_OF_ZONE'
    if battery_pct < batt_th and not charging:
        return 'LOW_BATTERY'
    return 'OK'


def scooter_colors(state, low_battery, broken=False, charging=False):
    """-> (발판 색, 기둥·핸들 색). 기둥: 고장 빨강 > 충전 중 청록 > 배터리 부족 보라 > 회색"""
    stem = STEM_BROKEN if broken else STEM_CHARGING if charging else STEM_LOW_BATTERY if low_battery else STEM_NORMAL
    return DECK_COLORS[state], stem


def body_link(name):
    """킥보드 링크 이름 (모델마다 다르게 — Gazebo Entity Tree 에서 구분하기 쉽게)"""
    return f'{name}_body'


def _beacon(rgb, roll=0.0, h=2.3):
    """신호등: 킥보드 위 h m 에 상태 색 공 + 가는 기둥 (충돌체 없음). 넘어진 킥보드(roll)여도 세상 기준 위쪽에 선다"""
    sr, cr = math.sin(roll), math.cos(roll)
    pole_h = h - 1.1
    pz = 1.1 + pole_h / 2
    mat = f"<material><ambient>{rgb} 1</ambient><diffuse>{rgb} 1</diffuse><emissive>{rgb} 1</emissive></material>"
    return (f"<visual name='beacon_pole'><pose>0 {pz * sr:.3f} {pz * cr:.3f} {-roll:.4f} 0 0</pose>"
            f"<geometry><cylinder><radius>0.025</radius><length>{pole_h:.2f}</length></cylinder></geometry>{mat}</visual>"
            f"<visual name='beacon'><pose>0 {h * sr:.3f} {h * cr:.3f} 0 0 0</pose>"
            f"<geometry><sphere><radius>0.32</radius></sphere></geometry>{mat}</visual>")


def _mesh(name, uri, rgb=None, pose='0 0 0 0 0 0'):
    mat = (f"<material><ambient>{rgb} 1</ambient><diffuse>{rgb} 1</diffuse><specular>0.2 0.2 0.2 1</specular></material>"
           if rgb else '')
    return (f"<visual name='{name}'><pose>{pose}</pose><geometry><mesh><uri>{uri}</uri></mesh></geometry>"
            f"{mat}</visual>")


def _cbox(name, x, y, z, sx, sy, sz, pitch=0.0):
    return (f"<collision name='{name}'><pose>{x:.3f} {y:.3f} {z:.3f} 0 {pitch:.4f} 0</pose>"
            f"<geometry><box><size>{sx} {sy} {sz}</size></box></geometry></collision>")


def scooter_sdf(name, collide=True, deck=DECK_COLORS['OK'], stem=STEM_NORMAL, beacon=False, roll=0.0):
    """G30 킥보드. 발판 = 상태 색, 스템·핸들 = 배터리 색 (나머지는 Blender 재질 그대로).
    collide=False: 충돌체 없는 복사본 (로봇이 싣고 다니거나 옮기는 중인 킥보드)
    beacon=True: 처리가 필요한 킥보드(발판 색이 정상 초록이 아님)면 위에 상태 색 신호등을 단다 (캠퍼스 맵)"""
    body = (_mesh('g30_body', G30_MESH + 'g30_body.obj') + _mesh('g30_stand', G30_MESH + 'g30_stand.obj')
            + _mesh('deck', G30_MESH + 'g30_deck.obj', deck) + _mesh('stem', G30_MESH + 'g30_stem.obj', stem))
    if collide:
        ax = G30_WB / 2
        t = math.radians(14)                                   # 스템 뒤 기울기
        body += (_cbox('deck_c', -0.075, 0, 0.125, 0.85, 0.17, 0.09)
                 + _cbox('wheel_f_c', ax, 0, G30_R, 0.25, 0.06, 0.25)
                 + _cbox('wheel_r_c', -ax, 0, G30_R, 0.25, 0.06, 0.25)
                 + _cbox('stem_c', ax - 0.53 * math.sin(t), 0, G30_R + 0.53 * math.cos(t), 0.05, 0.05, 1.06, -t)
                 + _cbox('bar_c', ax - 1.06 * math.sin(t), 0, G30_R + 1.06 * math.cos(t), 0.05, 0.47, 0.04))
    if beacon and deck != DECK_COLORS['OK']:
        body += _beacon(deck, roll)
    return (f"<?xml version='1.0'?><sdf version='1.9'><model name='{name}'><static>true</static>"
            f"<link name='{body_link(name)}'>{body}</link></model></sdf>")


class GzWorld:
    def __init__(self, world, logger, enabled=True):
        self.beacons = False       # 캠퍼스 맵: 처리가 필요한 킥보드 위에 신호등
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

    def spawn(self, name, x, y, yaw, roll=0.0, colors=None, z0=0.0):
        """gz 서비스로 바로 만든다 (킥보드마다 ros2 run 프로세스를 띄우면 수십 대일 때 수십 초 걸림). z0 = 지형 높이"""
        if not self.enabled:
            return
        z = (FALLEN_Z if abs(roll) > 0.1 else 0.0) + z0
        deck, stem = colors or (DECK_COLORS['OK'], STEM_NORMAL)
        sdf = scooter_sdf(name, deck=deck, stem=stem, beacon=self.beacons, roll=roll)
        cr, sr = math.cos(roll / 2), math.sin(roll / 2)
        cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
        qw, qx, qy, qz = cr * cy, sr * cy, sr * sy, cr * sy        # yaw * roll
        self._run(['gz', 'service', '-s', f'/world/{self.world}/create', '--reqtype', 'gz.msgs.EntityFactory',
                   '--reptype', 'gz.msgs.Boolean', '--timeout', '5000', '--req',
                   f'sdf: "{sdf}" name: "{name}" pose {{position {{x: {x} y: {y} z: {z}}} '
                   f'orientation {{x: {qx} y: {qy} z: {qz} w: {qw}}}}}'], f'spawn {name}')

    def spawn_box(self, name, x, y, size=(0.5, 0.5, 0.5), yaw=0.0, rgb='0.8 0.1 0.1'):
        sx, sy, sz = size
        sdf = (f"<?xml version='1.0'?><sdf version='1.9'><model name='{name}'><static>true</static>"
               f"<link name='l'><collision name='c'><pose>0 0 {sz / 2} 0 0 0</pose><geometry><box>"
               f"<size>{sx} {sy} {sz}</size></box></geometry></collision><visual name='v'>"
               f"<pose>0 0 {sz / 2} 0 0 0</pose><geometry><box><size>{sx} {sy} {sz}</size></box>"
               f"</geometry><material><ambient>{rgb} 1</ambient><diffuse>{rgb} 1</diffuse>"
               f"</material></visual></link></model></sdf>")
        self._run(['ros2', 'run', 'ros_gz_sim', 'create', '-world', self.world, '-name', name,
                   '-string', sdf, '-x', str(x), '-y', str(y), '-z', '0', '-Y', str(yaw)], f'spawn {name}')

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

    def respawn(self, name, x, y, yaw, roll, colors, z0=0.0):
        """색을 바꾸려고 모델을 지웠다가 같은 자리에 새 색으로 다시 만든다.
        (gz sim 8 의 visual_config / material_color 는 이 VM(ogre1)에서 화면에 반영되지 않았다)"""
        if not self.enabled:
            return
        z = (FALLEN_Z if abs(roll) > 0.1 else 0.0) + z0
        cr, sr = math.cos(roll / 2), math.sin(roll / 2)
        cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
        qw, qx, qy, qz = cr * cy, sr * cy, sr * sy, cr * sy        # yaw * roll
        deck, stem = colors
        sdf = scooter_sdf(name, deck=deck, stem=stem, beacon=self.beacons, roll=roll)       # 작은따옴표만 씀 -> 요청 문자열에 그대로 넣을 수 있음
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
