#!/usr/bin/env python3
"""Blender 내보내기 결과(robot_blender/export) -> Gazebo 모델.

  python3 src/kickboard_sim/scripts/build_blender_models.py [robot_blender/export 경로]

만드는 것 (src/kickboard_sim 아래):
  models/kickboard_robot/      SCOOTOVER v19 (모델 이름은 그대로: 브리지·launch·노드가 이 이름을 씀)
      model.sdf, meshes/*.obj+.mtl
  models/g30_scooter/meshes/   G30 킥보드 메시 (gz_utils.scooter_sdf 가 상태 색을 입혀 씀)
  models/kickspot_4/           킥보드 충전소 4구 (static)
  models/scootover_robot_station/  로봇 충전소 차고 (static, 셔터 열림)
  config/scootover_rig.json    관절 정의 + 수송 자세 + Blender 애니메이션을 관절값으로 바꾼 동작 (arm_loader 가 재생)
  config/blender_assets.json   G30·Kickspot·차고 치수 (gz_utils, 맵 생성 스크립트가 씀)

관절은 손으로 적지 않는다. 수송 자세(정지 모델) 대비 애니메이션 각 프레임의 부모 기준 변환 D = M0^-1 M(t) 를 보고
  이동이 한 방향이면 prismatic, 회전축이 일정하면 revolute, 아니면 Z-Y-X revolute 3개(가짜 링크)로 나눈다.
  애니메이션에 없는 오른쪽 부품(_r)은 왼쪽(_l) 관절을 XZ 평면에 거울 대칭해 만든다 (같은 값 = 거울 동작).
모든 관절값 0 = 수송 자세 (= Blender 정지 모델).
"""
import json
import math
import os
import shutil
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
REPO = os.path.dirname(os.path.dirname(PKG))
EXPORT = os.path.abspath(sys.argv[1]) if len(sys.argv) > 1 else os.path.join(REPO, 'robot_blender', 'export')
MODELS = os.path.join(PKG, 'models')
ROBOT = 'kickboard_robot'
M_MIRROR = np.diag([1.0, -1.0, 1.0])


# ---------------- 수학 ----------------
def quat_to_mat(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def pose7_to_T(p):
    T = np.eye(4)
    T[:3, :3] = quat_to_mat(p[3:7])
    T[:3, 3] = p[:3]
    return T


def mat_to_rpy(R):
    """SDF rpy (고정축 X->Y->Z, R = Rz(y) Ry(p) Rx(r))"""
    p = math.asin(max(-1.0, min(1.0, -R[2, 0])))
    if abs(math.cos(p)) > 1e-6:
        r = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(R[1, 0], R[0, 0])
    else:
        r = math.atan2(-R[1, 2], R[1, 1])
        y = 0.0
    return r, p, y


def pose_str(T):
    r, p, y = mat_to_rpy(T[:3, :3])
    x = T[:3, 3]
    return f'{x[0]:.6f} {x[1]:.6f} {x[2]:.6f} {r:.6f} {p:.6f} {y:.6f}'


def rot(axis, a):
    axis = np.asarray(axis, float)
    axis = axis / np.linalg.norm(axis)
    K = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    return np.eye(3) + math.sin(a) * K + (1 - math.cos(a)) * K @ K


def axis_angle(R):
    c = max(-1.0, min(1.0, (np.trace(R) - 1) / 2))
    a = math.acos(c)
    if a < 1e-9:
        return np.array([0, 0, 1.0]), 0.0
    if a > math.pi - 1e-4:
        w, v = np.linalg.eigh((R + np.eye(3)) / 2)
        return v[:, np.argmax(w)], a
    ax = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    return ax / np.linalg.norm(ax), a


def zyx(R):
    """R = Rz(a) Ry(b) Rx(c) (가짜 링크 체인: z 다음 y 다음 x). b = ±90° 면 a = 0 으로 둠"""
    b = math.asin(max(-1.0, min(1.0, -R[2, 0])))
    if abs(math.cos(b)) > 1e-6:
        return math.atan2(R[1, 0], R[0, 0]), b, math.atan2(R[2, 1], R[2, 2])
    if b > 0:
        return 0.0, b, math.atan2(R[0, 1], R[1, 1])
    return 0.0, b, math.atan2(-R[0, 1], R[1, 1])


def euler_chain(Rs):
    """세 축 체인 R = R_i(a) R_j(b) R_k(c). 가운데 각이 ±90° 에서 가장 먼 축 순서를 고른다 -> (축 3개, 값 [F,3])"""
    import itertools
    E = np.eye(3)
    best = None
    for i, j, k in itertools.permutations(range(3)):
        P = np.array([E[k], E[j], E[i]])            # P e_i = z, P e_j = y, P e_k = x
        s = 1.0
        if np.linalg.det(P) < 0:
            P[1] *= -1                              # P e_j = -y  -> j 회전각 = -(y 회전각)
            s = -1.0
        vals = np.array([zyx(P @ R @ P.T) for R in Rs])
        vals[:, 1] *= s
        worst = np.abs(np.sin(vals[:, 1])).max()
        if best is None or worst < best[0] - 1e-9:
            best = (worst, [E[i].copy(), E[j].copy(), E[k].copy()], vals, 'xyz'[i] + 'xyz'[j] + 'xyz'[k])
    return best[1], best[2], best[3]


def r6(v):
    return round(float(v), 6)


# ---------------- 입력 ----------------
def load(rel):
    with open(os.path.join(EXPORT, rel)) as f:
        return json.load(f)


def read_obj_verts(path):
    vs = []
    with open(path) as f:
        for ln in f:
            if ln.startswith('v '):
                vs.append([float(t) for t in ln.split()[1:4]])
    return np.array(vs)


def copy_mesh(src_dir, name, dst_dir, rename=None):
    os.makedirs(dst_dir, exist_ok=True)
    base = rename or name
    for ext in ('.obj', '.mtl'):
        s = os.path.join(src_dir, name + ext)
        if not os.path.exists(s):
            continue
        d = os.path.join(dst_dir, base + ext)
        with open(s) as f:
            txt = f.read()
        if ext == '.obj' and rename:
            txt = txt.replace(f'mtllib {name}.mtl', f'mtllib {base}.mtl')
        with open(d, 'w') as f:
            f.write(txt)
    return f'model://{os.path.basename(os.path.dirname(dst_dir))}/meshes/{base}.obj'


def box_inertia(m, size):
    sx, sy, sz = (max(s, 0.01) for s in size)
    return (m / 12 * (sy * sy + sz * sz), m / 12 * (sx * sx + sz * sz), m / 12 * (sx * sx + sy * sy))


def inertial_xml(m, center=(0, 0, 0), size=(0.1, 0.1, 0.1)):
    ixx, iyy, izz = box_inertia(m, size)
    c = center
    return (f'<inertial><pose>{c[0]:.4f} {c[1]:.4f} {c[2]:.4f} 0 0 0</pose><mass>{m:.4f}</mass>'
            f'<inertia><ixx>{ixx:.6f}</ixx><iyy>{iyy:.6f}</iyy><izz>{izz:.6f}</izz>'
            f'<ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>')


def mesh_visual(name, uri, T=None):
    pose = f'<pose>{pose_str(T)}</pose>' if T is not None else ''
    return f'<visual name="{name}">{pose}<geometry><mesh><uri>{uri}</uri></mesh></geometry></visual>'


def box_collision(name, center, size, rpy=(0, 0, 0), mu=None):
    fr = (f'<surface><friction><ode><mu>{mu}</mu><mu2>{mu}</mu2></ode></friction></surface>'
          if mu is not None else '')
    return (f'<collision name="{name}"><pose>{center[0]:.4f} {center[1]:.4f} {center[2]:.4f} '
            f'{rpy[0]:.5f} {rpy[1]:.5f} {rpy[2]:.5f}</pose>'
            f'<geometry><box><size>{size[0]:.4f} {size[1]:.4f} {size[2]:.4f}</size></box></geometry>{fr}</collision>')


def write(path, txt):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w') as f:
        f.write(txt)


def model_config(name, desc):
    return (f'<?xml version="1.0"?>\n<model><name>{name}</name><version>1.0</version>'
            f'<sdf version="1.9">model.sdf</sdf>\n<description>{desc}</description></model>\n')


# ================= 로봇 =================
# 로봇 링크 질량 (kg, 추정 — 인계 문서 7절). 없는 이름은 0.3
MASS = {'base_link': 100.0, 'wheel': 5.0, 'arm_base': 8.0, 'upper_arm': 4.0, 'forearm': 3.0, 'wrist': 1.5,
        'finger': 0.2, 'ramp': 3.0, 'ramp_tip': 2.5, 'front_ramp': 1.5, 'front_ramp_tip': 1.0, 'outrigger': 1.5,
        'outrigger_foot': 0.5, 'wheel_shuttle': 2.0, 'gate': 0.1, 'stem_clamp': 0.3, 'dock_holder': 0.5,
        'dock_clamp_mount': 0.6}
SKIP_ROBOT = {'robot_root'}
SENSOR_LINKS = {'lidar': 'base_scan', 'cam_front': 'camera_link'}


def mass_of(name):
    for k in sorted(MASS, key=len, reverse=True):
        if name.startswith(k) or k in name:
            return MASS[k]
    return 0.3


def counterpart(n):
    """왼쪽 부품 이름 -> 오른쪽 짝 (shuttle_gate_l_a -> shuttle_gate_r_a, front_ramp_l -> front_ramp_r)"""
    if '_l_' in n:
        return n.replace('_l_', '_r_', 1)
    if n.endswith('_l'):
        return n[:-2] + '_r'
    return None


def analyse_joint(name, M0, Ms):
    """M0: 부모 기준 수송 자세, Ms: 프레임별 부모 기준 변환 -> 관절 체인 정의 + 프레임별 값"""
    Ds = [np.linalg.inv(M0) @ M for M in Ms]
    ts = np.array([D[:3, 3] for D in Ds])
    chain, values = [], []
    # 이동
    tn = np.linalg.norm(ts, axis=1)
    if tn.max() > 2e-4:
        u = ts[np.argmax(tn)] / tn.max()
        resid = np.linalg.norm(ts - np.outer(ts @ u, u), axis=1).max()
        if resid > 2e-3:
            raise RuntimeError(f'{name}: 이동이 한 방향이 아님 (잔차 {resid:.4f})')
        chain.append({'type': 'prismatic', 'axis': u})
        values.append(ts @ u)
    # 회전
    aas = [axis_angle(D[:3, :3]) for D in Ds]
    angs = np.array([a for _, a in aas])
    if angs.max() > 1e-4:
        ref = aas[int(np.argmax(angs))][0]
        # 축 일치 검사는 각이 충분히 큰 프레임만 (작은 각의 축은 반올림 잡음)
        single = all(abs(abs(float(np.dot(ax, ref))) - 1) < 2e-3 for ax, a in aas if a > 0.02)
        sv = [float(np.dot(ax * a, ref)) for ax, a in aas]
        if single:
            chain.append({'type': 'revolute', 'axis': ref})
            values.append(np.unwrap(np.array(sv)))
        else:
            axes, e, order = euler_chain([D[:3, :3] for D in Ds])
            for k, ax in enumerate(axes):
                chain.append({'type': 'revolute', 'axis': np.array(ax), 'sub': order[k]})
                values.append(np.unwrap(e[:, k]))
    return chain, values


def chain_fk(chain, vals):
    T = np.eye(4)
    for j, v in zip(chain, vals):
        S = np.eye(4)
        if j['type'] == 'prismatic':
            S[:3, 3] = j['axis'] * v
        else:
            S[:3, :3] = rot(j['axis'], v)
        T = T @ S
    return T


ARM_LINKS = ('arm_base', 'upper_arm', 'forearm', 'wrist')


def arm_right_lane(chains, Mrel, parent, rig, F):
    """Blender 애니메이션은 왼쪽 레인(y +0.27)만 있다. 오른쪽 레인용 팔 관절 트랙 '<관절>@r' 을 만든다.
    손목 목표 = 왼쪽 손목 pose 의 거울상 (팔꿈치가 옆으로 0.11 비켜 있어 관절값을 그냥 뒤집으면 안 됨) -> 수치 IK.
    팔이 접힌 자세 근처에서는 목표를 수송 자세 손목으로 섞어, 시작·끝이 관절값 0 과 이어지게 한다."""
    names = []                                   # (링크, 체인 안 번호, 관절 이름)
    for n in ARM_LINKS:
        jn = [j['name'] for j in rig['joints'] if j['link'] == n]
        names += [(n, k, jn[k]) for k in range(len(chains[n][0]))]
    Mm = np.diag([1.0, -1.0, 1.0, 1.0])

    def wrist(q):
        T = np.eye(4)
        for n in ARM_LINKS:
            vals = [q[i] for i, (ln, _, _) in enumerate(names) if ln == n]
            T = T @ Mrel[n] @ chain_fk(chains[n][0], vals)
        return T

    def err(T, Tt):
        e = np.zeros(6)
        e[:3] = Tt[:3, 3] - T[:3, 3]
        Rd = Tt[:3, :3] @ T[:3, :3].T
        e[3:] = 0.3 * 0.5 * np.array([Rd[2, 1] - Rd[1, 2], Rd[0, 2] - Rd[2, 0], Rd[1, 0] - Rd[0, 1]])
        return e

    def slerp_R(R0, R1, u):
        ax, a = axis_angle(R0.T @ R1)
        return R0 @ rot(ax, a * u) if a > 1e-9 else R0

    W0 = wrist(np.zeros(len(names)))
    # 거울 짝 자세 (여유 자유도 기준): 축이 xz 평면(거울면과 나란)이면 각도 부호 반대, y 축이면 그대로
    sign = []
    for n, k, _ in names:
        T = np.eye(4)
        for m in ARM_LINKS[:ARM_LINKS.index(n) + 1]:
            T = T @ Mrel[m]
        j = chains[n][0][k]
        w = T[:3, :3] @ j['axis']
        sign.append(1.0 if j['type'] == 'prismatic' or abs(w[1]) > 0.5 else -1.0)
    sign = np.array(sign)
    qr = np.zeros(len(names))
    out = {jn: [] for _, _, jn in names}
    worst = 0.0
    for k in range(F):
        ql = np.array([rig['tracks'][jn][k] for _, _, jn in names])
        Wm = Mm @ wrist(ql) @ Mm
        s = float(np.clip(np.abs(ql).max() / 0.3, 0.0, 1.0))
        Tt = np.eye(4)
        Tt[:3, 3] = (1 - s) * W0[:3, 3] + s * Wm[:3, 3]
        Tt[:3, :3] = slerp_R(W0[:3, :3], Wm[:3, :3], s)
        qref = sign * ql
        if s == 0.0:
            qr = np.zeros(len(names))
        for it in range(80):
            e = err(wrist(qr), Tt)
            if np.linalg.norm(e) < 2e-4 and (it > 0 or s == 0.0):
                break
            J = np.zeros((6, len(names)))
            for i in range(len(names)):
                dq = qr.copy()
                dq[i] += 1e-5
                J[:, i] = (err(wrist(qr), Tt) - err(wrist(dq), Tt)) / 1e-5
            Jp = J.T @ np.linalg.inv(J @ J.T + 1e-4 * np.eye(6))
            # 손목 목표는 그대로, 남는 자유도로 거울 짝 자세에 가깝게 (안 그러면 2π 돌거나 팔꿈치가 뒤집힌 해로 흘러감)
            qr = qr + Jp @ e + 0.3 * (np.eye(len(names)) - Jp @ J) @ (qref - qr)
        worst = max(worst, float(np.linalg.norm(err(wrist(qr), Tt)[:3])))
        for i, (_, _, jn) in enumerate(names):
            out[jn].append(round(float(qr[i]), 5))
    print(f'오른쪽 레인 팔 IK: 손목 위치 최대 오차 {worst * 1000:.1f} mm')
    return {f'{jn}@r': v for jn, v in out.items()}


def build_robot():
    inv = load('robot/inventory.json')
    trk = load('anim/tracks.json')
    objs = inv['objects']
    stats = inv['mesh_stats']
    mesh_src = os.path.join(EXPORT, 'robot', 'meshes')
    mdir = os.path.join(MODELS, ROBOT)
    if os.path.isdir(os.path.join(mdir, 'meshes')):
        shutil.rmtree(os.path.join(mdir, 'meshes'))
    meshes_dir = os.path.join(mdir, 'meshes')

    robot_objs = {n: o for n, o in objs.items()
                  if not ({'Stage', 'Reference'} & set(o['collections'])) and n not in SKIP_ROBOT}
    Mw = {n: np.array(o['matrix_world']) for n, o in robot_objs.items()}
    Mrel = {n: np.array(o['matrix_parent_rel']) for n, o in robot_objs.items()}
    parent = {n: (o['parent'] if o['parent'] in robot_objs else None) for n, o in robot_objs.items()}

    # 움직이는 링크: 애니메이션에서 부모 기준 변환이 바뀌는 것 + 그 오른쪽 짝 + 바퀴
    moving = {n for n, e in trk['moving'].items() if n in robot_objs and e.get('rel_moves')}
    wheels = {n for n in robot_objs if n.startswith('wheel_') and n[6:] in ('fl', 'fr', 'rl', 'rr')}
    moving -= wheels
    mirrored = {}
    for n in list(moving):
        c = counterpart(n)
        if c and c in robot_objs and c not in moving:
            mirrored[c] = n
    links = sorted(moving | set(mirrored) | wheels)

    def link_of(n):
        """오브젝트 -> 붙는 링크 (움직이지 않는 조상까지 올라감)"""
        while n is not None and n not in links:
            n = parent[n]
        return n or 'base_link'

    # ---- 관절 분석 (애니메이션 프레임별) ----
    F = trk['frame_end'] - trk['frame_start'] + 1
    rig = {'joints': [], 'frames': F, 'fps': trk['fps'], 'markers': trk['markers'], 'tracks': {}}
    chains = {}
    report = []
    for n in sorted(moving):
        e = trk['moving'][n]
        rels = e['rel'] if e['rel_moves'] else [e['rel']] * F
        Ms = [pose7_to_T(p) for p in rels]
        M0 = Mrel[n]
        chain, vals = analyse_joint(n, M0, Ms)
        err = max(np.abs(M0 @ chain_fk(chain, [v[k] for v in vals]) - Ms[k]).max() for k in range(0, F, 5))
        report.append(f'{n:22s} {"+".join(j["type"][0] + ("" if "sub" not in j else j["sub"]) for j in chain):8s} '
                      f'재구성 오차 {err:.2e}')
        if err > 5e-3:
            raise RuntimeError(f'{n}: 관절 재구성 오차 {err}')
        chains[n] = (chain, vals)
    for c, src in mirrored.items():
        chain, _ = chains[src]
        R0l, R0r = Mw[src][:3, :3], Mw[c][:3, :3]
        mch = []
        for j in chain:
            w = R0l @ j['axis']
            w = M_MIRROR @ w if j['type'] == 'prismatic' else -(M_MIRROR @ w)
            mj = dict(j)
            mj['axis'] = R0r.T @ w
            mch.append(mj)
        chains[c] = (mch, None)
        report.append(f'{c:22s} = 거울({src})')
    for w in wheels:
        chains[w] = ([{'type': 'continuous', 'axis': np.array([0, 1.0, 0])}], None)

    # ---- SDF ----
    L = ['<?xml version="1.0"?>',
         '<!-- 자동 생성: src/kickboard_sim/scripts/build_blender_models.py (원본 robot_blender/export, Blender SCOOTOVER).',
         '     손으로 고치지 말 것. 관절값 0 = 수송 자세.  모델 원점 = 차체 중심 아래 지면 = base_footprint, +x 앞.',
         '     충돌체: 차체(상자)와 바퀴만. 팔·램프·셔틀·집게는 보이기만 한다 (실린 킥보드는 arm_loader 가 옮김). -->',
         '<sdf version="1.9">', f'  <model name="{ROBOT}">']
    # base_link
    base_vis = []
    for n, o in sorted(robot_objs.items()):
        if link_of(n) != 'base_link' or n in SENSOR_LINKS or n not in stats or 'file' not in stats[n]:
            continue
        uri = copy_mesh(mesh_src, n, meshes_dir)
        base_vis.append('      ' + mesh_visual(n, uri, Mw[n]))
    coll = [
        box_collision('chassis', (0.0, 0.0, 0.17), (1.80, 1.05, 0.18)),
        box_collision('front_l', (0.65, 0.335, 0.37), (0.50, 0.38, 0.22)),
        box_collision('front_r', (0.65, -0.335, 0.37), (0.50, 0.38, 0.22)),
        box_collision('side_l', (-0.25, 0.475, 0.30), (1.30, 0.10, 0.10)),
        box_collision('side_r', (-0.25, -0.475, 0.30), (1.30, 0.10, 0.10)),
        box_collision('panel_l', (0.555, 0.45, 0.82), (0.50, 0.10, 0.74)),
        box_collision('panel_r', (0.555, -0.45, 0.82), (0.50, 0.10, 0.74)),
    ]
    L += ['    <link name="base_link">', '      ' + inertial_xml(MASS['base_link'], (0, 0, 0.3), (1.8, 1.05, 0.5))]
    L += ['      ' + c for c in coll] + base_vis + ['    </link>']
    # 센서 링크
    sensors = {
        'lidar': ('<sensor name="hls_lfcd_lds" type="gpu_lidar"><gz_frame_id>base_scan</gz_frame_id><topic>scan</topic>'
                  '<update_rate>5</update_rate><always_on>true</always_on><visualize>false</visualize><lidar><scan>'
                  '<horizontal><samples>360</samples><resolution>1</resolution><min_angle>-3.14159</min_angle>'
                  '<max_angle>3.14159</max_angle></horizontal></scan><range><min>0.12</min><max>3.5</max>'
                  '<resolution>0.015</resolution></range><noise><type>gaussian</type><mean>0</mean>'
                  '<stddev>0.01</stddev></noise></lidar></sensor>'),
        'cam_front': ('<sensor name="camera" type="camera"><pose>0.02 0 0 0 0 0</pose>'
                      '<gz_frame_id>camera_optical_frame</gz_frame_id><topic>camera/image_raw</topic>'
                      '<update_rate>5</update_rate><always_on>true</always_on><camera>'
                      '<camera_info_topic>camera/camera_info</camera_info_topic><horizontal_fov>1.085595</horizontal_fov>'
                      '<image><width>320</width><height>240</height><format>R8G8B8</format></image>'
                      '<clip><near>0.03</near><far>10</far></clip></camera></sensor>'),
    }
    for n, lname in SENSOR_LINKS.items():
        uri = copy_mesh(mesh_src, n, meshes_dir)
        T = Mw[n].copy()
        L += [f'    <link name="{lname}">', f'      <pose>{pose_str(T)}</pose>',
              '      ' + inertial_xml(0.2, size=(0.1, 0.1, 0.08)),
              '      ' + mesh_visual(n, uri), '      ' + sensors[n], '    </link>',
              f'    <joint name="{lname}_joint" type="fixed"><parent>base_link</parent><child>{lname}</child></joint>']
    # 손목 카메라 (cam_wrist 메시 위치, 앞 = 집게 축 +x)
    # 움직이는 링크
    joint_plugins = []
    for n in links:
        chain, vals = chains[n]
        T = Mw[n]
        st = stats.get(n, {})
        bb = np.array(st.get('bbox_local', [[-0.05] * 3, [0.05] * 3]))
        m = mass_of(n)
        vis = []
        if 'file' in st:
            vis.append(mesh_visual(n, copy_mesh(mesh_src, n, meshes_dir)))
        for cn, co in robot_objs.items():          # 이 링크에 고정된 자식 (예: cam_wrist)
            if cn != n and link_of(cn) == n and cn in stats and 'file' in stats[cn]:
                vis.append(mesh_visual(cn, copy_mesh(mesh_src, cn, meshes_dir), np.linalg.inv(T) @ Mw[cn]))
        extra = ''
        if n.startswith('wheel_') and n in wheels:
            extra = ('<collision name="tire"><pose>0 0 0 1.5708 0 0</pose><geometry><cylinder><radius>0.125</radius>'
                     '<length>0.086</length></cylinder></geometry><surface><friction><ode><mu>1.0</mu><mu2>0.6</mu2>'
                     '</ode></friction></surface></collision>')
        if n == 'wrist':
            extra = ('<sensor name="camera_wrist" type="camera"><pose>0.05 0 0.05 0 0 0</pose>'
                     '<gz_frame_id>camera_wrist_link</gz_frame_id><topic>camera_wrist/image_raw</topic>'
                     '<update_rate>5</update_rate><always_on>true</always_on><camera>'
                     '<camera_info_topic>camera_wrist/camera_info</camera_info_topic><horizontal_fov>1.2</horizontal_fov>'
                     '<image><width>320</width><height>240</height><format>R8G8B8</format></image>'
                     '<clip><near>0.02</near><far>5</far></clip></camera></sensor>')
        names = [n] if len(chain) == 1 else [f'{n}__{k}' for k in range(len(chain) - 1)] + [n]
        par = link_of(parent[n]) if parent[n] else 'base_link'
        for k, (j, ln) in enumerate(zip(chain, names)):
            last = ln == n
            body = ''.join(vis) + extra if last else ''
            inert = (inertial_xml(m, (bb[0] + bb[1]) / 2, bb[1] - bb[0]) if last
                     else inertial_xml(0.02, size=(0.02, 0.02, 0.02)))
            grav = '' if n in wheels else '<gravity>false</gravity>'
            L += [f'    <link name="{ln}">', f'      <pose>{pose_str(T)}</pose>', f'      {grav}{inert}',
                  f'      {body}', '    </link>']
            if len(chain) == 1:
                jname = f'j_{n}'
            elif j['type'] == 'prismatic':
                jname = f'j_{n}_t'
            else:
                jname = f'j_{n}_r' + (j.get('sub') or '')
            ax = j['axis']
            if j['type'] == 'continuous':
                lim = ''
            else:
                if vals is not None:
                    v = vals[k]
                    lo, hi = float(min(v.min(), 0)), float(max(v.max(), 0))
                else:                               # 거울 짝: 원본 범위
                    src = mirrored.get(n)
                    v = chains[src][1][k] if src else np.array([0.0])
                    lo, hi = float(min(v.min(), 0)), float(max(v.max(), 0))
                pad = 0.02 if j['type'] == 'prismatic' else 0.1
                if j.get('sub'):
                    lo, hi = -2 * math.pi, 2 * math.pi
                lim = (f'<limit><lower>{lo - pad:.4f}</lower><upper>{hi + pad:.4f}</upper>'
                       f'<effort>1000</effort><velocity>5</velocity></limit>')
            jt = 'revolute' if j['type'] == 'continuous' else j['type']
            L.append(f'    <joint name="{jname}" type="{jt}"><parent>{par}</parent><child>{ln}</child>'
                     f'<axis><xyz>{ax[0]:.6f} {ax[1]:.6f} {ax[2]:.6f}</xyz>{lim}</axis></joint>')
            rig['joints'].append({'name': jname, 'link': n, 'type': jt, 'axis': [r6(a) for a in ax],
                                  'parent': par, 'mirror_of': None})
            if j['type'] != 'continuous':
                cmax = 3.0 if j['type'] == 'prismatic' else 20.0   # 애니메이션 원본이 빠름 (램프 끝 14 rad/s)
                joint_plugins.append(
                    f'    <plugin filename="gz-sim-joint-position-controller-system" '
                    f'name="gz::sim::systems::JointPositionController"><joint_name>{jname}</joint_name>'
                    f'<topic>/{ROBOT}/joint/{jname}</topic><use_velocity_commands>true</use_velocity_commands>'
                    f'<p_gain>25</p_gain><cmd_max>{cmax}</cmd_max><cmd_min>-{cmax}</cmd_min>'
                    f'<initial_position>0</initial_position></plugin>')
            if vals is not None:
                rig['tracks'][jname] = [round(float(x), 5) for x in vals[k]]
            par = ln
        if n in mirrored:
            src = mirrored[n]
            for jj, sj in zip([j for j in rig['joints'] if j['link'] == n],
                              [j for j in rig['joints'] if j['link'] == src]):
                jj['mirror_of'] = sj['name']
    L += [
        '    <plugin filename="gz-sim-diff-drive-system" name="gz::sim::systems::DiffDrive">',
        '      <left_joint>j_wheel_fl</left_joint><left_joint>j_wheel_rl</left_joint>',
        '      <right_joint>j_wheel_fr</right_joint><right_joint>j_wheel_rr</right_joint>',
        '      <wheel_separation>0.95</wheel_separation><wheel_radius>0.125</wheel_radius>',
        '      <max_linear_velocity>1.2</max_linear_velocity><max_angular_velocity>1.0</max_angular_velocity>',
        '      <topic>cmd_vel</topic><odom_topic>odom</odom_topic><tf_topic>tf</tf_topic>',
        '      <frame_id>odom</frame_id><child_frame_id>base_footprint</child_frame_id>',
        '      <odom_publish_frequency>30</odom_publish_frequency>',
        '    </plugin>',
        '    <plugin filename="gz-sim-joint-state-publisher-system" name="gz::sim::systems::JointStatePublisher">',
        '      <topic>joint_states</topic></plugin>',
        '    <plugin filename="gz-sim-pose-publisher-system" name="gz::sim::systems::PosePublisher">',
        '      <publish_model_pose>true</publish_model_pose><publish_link_pose>false</publish_link_pose>',
        '      <publish_nested_model_pose>false</publish_nested_model_pose><use_pose_vector_msg>false</use_pose_vector_msg>',
        '      <update_frequency>30</update_frequency></plugin>',
        '    <!-- 기구 관절: 속도 명령 위치 제어. 명령 토픽 /kickboard_robot/joint/<관절> (gz.msgs.Double), 0 = 수송 자세 -->',
    ] + joint_plugins + ['  </model>', '</sdf>', '']
    write(os.path.join(mdir, 'model.sdf'), '\n'.join(L))
    write(os.path.join(mdir, 'model.config'),
          model_config(ROBOT, 'SCOOTOVER v19 킥보드 수거 로봇 (Blender 외형, 자동 생성 build_blender_models.py)'))
    # 킥보드 pose (로봇 기준) 트랙: 적재·하차 때 실린 킥보드를 옮기는 데 씀
    root = trk['moving'].get('robot_root')
    roots = [pose7_to_T(p) for p in root['world']] if root else [np.eye(4)] * F
    for sc in ('ref_scooter_l', 'ref_scooter_r'):
        e = trk['moving'].get(sc) or trk['static'].get(sc)
        if e is None:
            continue
        ws = e['world'] if isinstance(e['world'][0], list) else [e['world']] * F
        rel = []
        for k in range(F):
            T = np.linalg.inv(roots[k]) @ pose7_to_T(ws[k])
            q = mat_to_quat(T[:3, :3])
            rel.append([round(float(v), 5) for v in list(T[:3, 3]) + list(q)])
        rig['tracks'][f'{sc}@robot'] = rel
    rig['tracks']['robot_root@start'] = [[round(float(v), 5) for v in list(T[:3, 3]) + list(mat_to_quat(T[:3, :3]))]
                                         for T in [np.linalg.inv(roots[0]) @ R for R in roots]]
    for n in ('latch_4', 'kickspot_station'):
        e = trk['moving'].get(n) or trk['static'].get(n)
        if e is not None:
            ws = e['world'] if isinstance(e['world'][0], list) else [e['world']] * F
            rig['tracks'][f'{n}@world'] = ws
    rig['transport_scooter_axle'] = {'l': [0.65, 0.27, 0.277], 'r': [0.65, -0.27, 0.277]}
    rig['tracks'].update(arm_right_lane(chains, Mrel, parent, rig, F))
    write(os.path.join(PKG, 'config', 'scootover_rig.json'), json.dumps(rig, ensure_ascii=False))
    print('로봇:', len(links), '움직이는 링크,', len(rig['joints']), '관절')
    for ln in report:
        print('  ', ln)


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


# ================= G30 킥보드 =================
def parse_obj(path):
    vs, vns, faces, cur = [], [], [], None
    with open(path) as f:
        for ln in f:
            t = ln.split()
            if not t:
                continue
            if t[0] == 'v':
                vs.append([float(x) for x in t[1:4]])
            elif t[0] == 'vn':
                vns.append([float(x) for x in t[1:4]])
            elif t[0] == 'usemtl':
                cur = t[1]
            elif t[0] == 'f':
                faces.append((cur, [tuple(int(x) if x else 0 for x in p.split('/')) for p in t[1:]]))
    return np.array(vs), vns, faces


def write_obj(path, vs, vns, faces, mtl=None, T=None):
    """faces: [(재질, [(v, vt, vn) 1-based])]. 쓰인 정점만 다시 번호. T: 정점 변환(4x4)"""
    used_v = sorted({p[0] for _, f in faces for p in f})
    used_n = sorted({p[-1] for _, f in faces for p in f if len(p) > 2 and p[-1]})
    vm = {v: i + 1 for i, v in enumerate(used_v)}
    nm = {n: i + 1 for i, n in enumerate(used_n)}
    out = ['# generated by build_blender_models.py']
    if mtl:
        out.append(f'mtllib {mtl}')
    R = np.eye(3) if T is None else T[:3, :3]
    for v in used_v:
        p = np.array(vs[v - 1])
        if T is not None:
            p = R @ p + T[:3, 3]
        out.append(f'v {p[0]:.6f} {p[1]:.6f} {p[2]:.6f}')
    for n in used_n:
        q = R @ np.array(vns[n - 1])
        out.append(f'vn {q[0]:.4f} {q[1]:.4f} {q[2]:.4f}')
    cur = None
    for m, f in faces:
        if mtl and m != cur:
            out.append(f'usemtl {m}')
            cur = m
        out.append('f ' + ' '.join(f'{vm[p[0]]}//{nm[p[-1]]}' if len(p) > 2 and p[-1] else f'{vm[p[0]]}'
                                   for p in f))
    write(path, '\n'.join(out) + '\n')


def components(faces):
    """정점을 공유하는 면끼리 묶음 -> 면 인덱스 목록들"""
    par = {}

    def find(a):
        while par.setdefault(a, a) != a:
            par[a] = par[par[a]]
            a = par[a]
        return a
    for _, f in faces:
        r0 = find(f[0][0])
        for p in f[1:]:
            r = find(p[0])
            if r != r0:
                par[r] = r0
    groups = {}
    for i, (_, f) in enumerate(faces):
        groups.setdefault(find(f[0][0]), []).append(i)
    return list(groups.values())


def build_g30():
    """G30 메시를 '킥보드 모델 원점(바닥, 앞·뒤 바퀴 축 가운데)' 기준으로 옮겨 3조각으로 나눈다:
    g30_body (원래 재질), g30_deck / g30_stem (재질 없음 — SDF 에서 상태·배터리 색을 입힘)"""
    inv = load('robot/inventory.json')
    src = os.path.join(EXPORT, 'robot', 'meshes')
    dst = os.path.join(MODELS, 'g30_scooter', 'meshes')
    if os.path.isdir(dst):
        shutil.rmtree(dst)
    os.makedirs(dst)
    WB = 0.84
    # 앞바퀴 축 원점 -> 모델 원점 (축 가운데 아래 지면): x + WB/2, z + 바퀴 반지름
    T = np.eye(4)
    T[:3, 3] = [WB / 2, 0, 0.127]
    vs, vns, faces = parse_obj(os.path.join(src, 'ref_scooter_l_geo.obj'))
    paint = [i for i, (m, _) in enumerate(faces) if m == 'mat_scooter']
    sub = [faces[i] for i in paint]
    deck, stem = [], []
    for comp in components(sub):
        zs = [vs[p[0] - 1][2] for i in comp for p in sub[i][1]]
        xs = [vs[p[0] - 1][0] for i in comp for p in sub[i][1]]
        # 스템·핸들: 앞바퀴 축 근처에서 위로 솟은 부분 (축 기준 z 평균 > 0.25)
        (stem if (np.mean(zs) > 0.25 and np.mean(xs) > -0.45) else deck).extend(sub[i] for i in comp)
    body = [f for i, f in enumerate(faces) if i not in set(paint)]
    shutil.copy(os.path.join(src, 'ref_scooter_l_geo.mtl'), os.path.join(dst, 'g30_body.mtl'))
    write_obj(os.path.join(dst, 'g30_body.obj'), vs, vns, body, mtl='g30_body.mtl', T=T)
    write_obj(os.path.join(dst, 'g30_deck.obj'), vs, vns, deck, T=T)
    write_obj(os.path.join(dst, 'g30_stem.obj'), vs, vns, stem, T=T)
    # 킥스탠드 (펴짐, 앞바퀴 축 기준 pose)
    so = inv['objects']['ref_scooter_l_stand']
    Ts = T @ np.array(so['matrix_parent_rel'])
    svs, svns, sf = parse_obj(os.path.join(src, 'ref_scooter_l_stand.obj'))
    shutil.copy(os.path.join(src, 'ref_scooter_l_stand.mtl'), os.path.join(dst, 'g30_stand.mtl'))
    write_obj(os.path.join(dst, 'g30_stand.obj'), svs, svns, sf, mtl='g30_stand.mtl', T=Ts)
    allv = np.array([T[:3, :3] @ v + T[:3, 3] for v in vs])
    # 넘어졌을 때(roll ±90°) 지면 위로 띄울 높이
    fz = {}
    for s in (1, -1):
        R = rot([1, 0, 0], s * math.pi / 2)
        fz[s] = float(-(allv @ R.T)[:, 2].min())
    info = {'wheelbase': WB, 'wheel_radius': 0.127,
            'bbox': [[r6(v) for v in allv.min(0)], [r6(v) for v in allv.max(0)]],
            'front_axle': [WB / 2, 0.0, 0.127], 'fallen_z': {'+': r6(fz[1]), '-': r6(fz[-1])},
            'deck_faces': len(deck), 'stem_faces': len(stem), 'body_faces': len(body)}
    write(os.path.join(MODELS, 'g30_scooter', 'model.config'),
          model_config('g30_scooter', 'Ninebot Max G30 메시 (원점 = 두 바퀴 축 가운데 지면). 모델은 gz_utils.scooter_sdf 가 만든다'))
    print('G30:', info)
    return info


# ================= Kickspot =================
def build_kickspot():
    inv = load('kickspot/inventory.json')
    src = os.path.join(EXPORT, 'kickspot', 'meshes')
    mdir = os.path.join(MODELS, 'kickspot_4')
    if os.path.isdir(os.path.join(mdir, 'meshes')):
        shutil.rmtree(os.path.join(mdir, 'meshes'))
    vis, col = [], []
    for n, o in sorted(inv['objects'].items()):
        if 'Station' not in o['collections'] or n not in inv['mesh_stats']:
            continue
        T = np.array(o['matrix_world'])
        vis.append(mesh_visual(n, copy_mesh(src, n, os.path.join(mdir, 'meshes')), T))
        bb = np.array(inv['mesh_stats'][n]['bbox_local'])
        c = T[:3, :3] @ ((bb[0] + bb[1]) / 2) + T[:3, 3]
        if n == 'kickspot_base':
            col.append(box_collision('base', (0.15, 0, 0.03), (0.90, 2.48, 0.06)))
        elif n.startswith('post_'):
            # 기둥: 18° 뒤로 기울어진 판 -> 기울인 상자 (밑은 받침대 앞쪽)
            col.append(box_collision(n, (c[0] + 0.05, c[1], 0.55), (0.30, 0.26, 1.0), rpy=(0, -0.314, 0)))
        elif n.startswith('holder_'):
            col.append(box_collision(n, c, bb[1] - bb[0]))
    # 앞 경사 (−0.52 ~ −0.30, 높이 0 -> 0.06)
    ang = math.atan2(0.06, 0.22)
    col.append(box_collision('ramp_front', (-0.41, 0, 0.03 - 0.02 * math.cos(ang)), (0.229, 2.48, 0.04),
                             rpy=(0, -ang, 0)))
    txt = ['<?xml version="1.0"?>',
           '<!-- 자동 생성 (build_blender_models.py). Kickspot 4구 킥보드 충전소.',
           '     원점 = 1·4번 슬롯 가운데 지면, +x = 기둥 쪽 (킥보드는 +x 를 보고 앞바퀴부터 꽂힘).',
           '     슬롯 y = -0.93, -0.31, +0.31, +0.93. 꽂힌 킥보드 앞바퀴 축 (0, 슬롯 y, 0.187), 4.1° 들림 -->',
           '<sdf version="1.9">', '  <model name="kickspot_4">', '    <static>true</static>', '    <link name="body">']
    txt += ['      ' + c for c in col] + ['      ' + v for v in vis] + ['    </link>', '  </model>', '</sdf>', '']
    write(os.path.join(mdir, 'model.sdf'), '\n'.join(txt))
    write(os.path.join(mdir, 'model.config'), model_config('kickspot_4', 'Kickspot 4구 킥보드 충전소 (Blender)'))
    # 꽂힌 킥보드 pose (모델 원점 = 바퀴 축 가운데 지면 기준): 앞바퀴 축 (0, y, 0.187), pitch -4.1°
    s1 = inv['objects']['scooter_1']
    R = np.array(s1['matrix_world'])[:3, :3]
    pitch = math.atan2(-R[2, 0], R[0, 0])
    slots = [-0.93, -0.31, 0.31, 0.93]
    info = {'slots_y': slots, 'front_axle_z': 0.187, 'pitch': r6(pitch),
            'footprint': [[-0.52, -1.46], [0.60, 1.46]], 'post_x': [0.0, 0.55]}
    print('Kickspot:', info)
    return info


# ================= 로봇 충전소 (차고) =================
def build_station():
    inv = load('robot_station/inventory.json')
    src = os.path.join(EXPORT, 'robot_station', 'meshes')
    mdir = os.path.join(MODELS, 'scootover_robot_station')
    if os.path.isdir(os.path.join(mdir, 'meshes')):
        shutil.rmtree(os.path.join(mdir, 'meshes'))
    vis = []
    for n, o in sorted(inv['objects'].items()):
        if not ({'Station', 'Shutter'} & set(o['collections'])) or n not in inv['mesh_stats']:
            continue
        mesh = n + '__open' if n + '__open' in inv['mesh_stats'] else n     # 셔터: 열린 모양
        T = np.array(o['matrix_world'])
        vis.append(mesh_visual(n, copy_mesh(src, mesh, os.path.join(mdir, 'meshes'), rename=n), T))
    # 충돌체: 벽·지붕·문틀·안쪽 바닥(0.10)·앞 경사. 문 폭 1.40 (y ±0.70), 통과 높이 1.42
    ang = math.atan2(0.10, 0.45)
    col = [
        box_collision('wall_l', (1.32, 0.82, 1.0), (2.60, 0.12, 2.0)),
        box_collision('wall_r', (1.32, -0.82, 1.0), (2.60, 0.12, 2.0)),
        box_collision('wall_back', (2.57, 0, 1.0), (0.10, 1.76, 2.0)),
        box_collision('roof', (1.32, 0, 1.97), (2.60, 1.76, 0.10)),
        box_collision('door_l', (0.05, 0.78, 1.0), (0.12, 0.16, 2.0)),
        box_collision('door_r', (0.05, -0.78, 1.0), (0.12, 0.16, 2.0)),
        box_collision('door_top', (0.05, 0, 1.75), (0.12, 1.40, 0.50)),
        box_collision('floor', (1.30, 0, 0.05), (2.56, 1.52, 0.10), mu=1.0),
        box_collision('ramp', (-0.225, 0, 0.05 - 0.05 / math.cos(ang) + 0.0), (0.461, 1.40, 0.10), rpy=(0, -ang, 0),
                      mu=1.0),
    ]
    txt = ['<?xml version="1.0"?>',
           '<!-- 자동 생성 (build_blender_models.py). SCOOTOVER 로봇 충전소 (컨테이너 차고), 셔터 열림.',
           '     원점 = 문 앞 지면 가운데. 문은 -x, 로봇은 +x 로 들어감. 충전 위치: 로봇 원점 (1.30, 0, 0.10), +x 방향.',
           '     뒷벽 충전 접점판 x 2.44. 도킹 태그(tag 100) = 뒷벽 (2.44, 0) 을 기준으로 맵 설정에 넣는다 -->',
           '<sdf version="1.9">', '  <model name="scootover_robot_station">', '    <static>true</static>',
           '    <link name="body">']
    txt += ['      ' + c for c in col] + ['      ' + v for v in vis] + ['    </link>', '  </model>', '</sdf>', '']
    write(os.path.join(mdir, 'model.sdf'), '\n'.join(txt))
    write(os.path.join(mdir, 'model.config'),
          model_config('scootover_robot_station', 'SCOOTOVER 로봇 충전소 차고 (Blender, 셔터 열림)'))
    info = {'dock_robot_pose': [1.30, 0.0, 0.10], 'tag_x': 2.44, 'footprint': [[-0.45, -0.88], [2.62, 0.88]],
            'door_half_width': 0.70, 'floor_z': 0.10}
    print('차고:', info)
    return info


def main():
    if not os.path.isdir(EXPORT):
        sys.exit(f'내보내기 폴더 없음: {EXPORT}')
    build_robot()
    assets = {'g30': build_g30(), 'kickspot_4': build_kickspot(), 'robot_station': build_station()}
    write(os.path.join(PKG, 'config', 'blender_assets.json'), json.dumps(assets, ensure_ascii=False, indent=1))
    print('끝')


if __name__ == '__main__':
    main()
