# SCOOTOVER v19 pose switcher (bi-fold ramp, 3-axis wrist, G30 reference): set_pose('transport') / set_pose('loading')
import bpy, math
from mathutils import Matrix, Vector

RAMP_MID_DEG = 55.0
def ramp_state(k, ra):
    """two-section bi-fold ramp: 0..0.3 first section 90->55 deg (second folded behind it),
    0.3..0.6 second section unfolds in the air, 0.6..1 both go down to the ground"""
    def s3(t):
        t = max(0.0, min(1.0, t)); return t * t * (3 - 2 * t)
    mid = math.radians(RAMP_MID_DEG)
    if k <= 0.3: return math.pi / 2 + (mid - math.pi / 2) * s3(k / 0.3), -math.pi
    if k <= 0.6: return mid, -math.pi + math.pi * s3((k - 0.3) / 0.3)
    return mid + (-ra - mid) * s3((k - 0.6) / 0.4), 0.0

ARM_OFF = -0.11          # forearm/wrist plane lateral offset from upper-arm plane
L_UP, L_FORE = 0.60, 0.55
GRIP = 0.11              # wrist joint -> grip centre along gripper axis
FINGER_C = 0.0345        # finger centre offset from grip axis (inner gap 0.049, stem d 0.045)
STEM_TILT = math.radians(14)
STEM_Z_LOCAL = 0.39      # grip height on the stem above the front axle (below the charging pad)


X_START = -1.65                       # front axle of the scooter waiting behind the robot
RAMP_L = 0.60
RAMP_ANG = math.asin(0.1495 / RAMP_L)  # 14.4 deg, tip 0.5 mm above ground
LEVER_PIV = (-RAMP_L, 0.10, 0.0035)    # kick lever pivot in ramp-local frame
STAND_PIV = Vector((-0.30, 0.085, -0.04))    # G30 kickstand: front-left of the deck, scooter-local (front axle origin)
FOOT_DEP = Vector((-0.24, 0.12, -0.095))
FOOT_FOLD = Vector((-0.40, 0.09, -0.035))
STAND_LEN = (FOOT_DEP - STAND_PIV).length


def stand_quats():
    d0 = (FOOT_DEP - STAND_PIV).normalized(); d1 = (FOOT_FOLD - STAND_PIV).normalized()
    qd = Vector((1, 0, 0)).rotation_difference(d0)
    return qd, d0.rotation_difference(d1) @ qd


def set_stand(side, t):   # t: 0 deployed .. 1 folded (single hinge axis)
    qd, qf = stand_quats(); ob = bpy.data.objects['ref_scooter_%s_stand' % side]
    ob.rotation_mode = 'QUATERNION'; ob.location = STAND_PIV; ob.rotation_quaternion = qd.slerp(qf, t)


def set_lever(side, beta):    # relative to the ramp: pi = folded onto the ramp top, RAMP_ANG = lying on the ground
    bpy.data.objects['kick_lever_' + side].rotation_euler = (0, beta, 0)


def _frame(p, a, L):
    a = Vector(a).normalized(); L = Vector(L).normalized(); z = a.cross(L)
    return Matrix(((a.x, L.x, z.x, p[0]), (a.y, L.y, z.y, p[1]),
                   (a.z, L.z, z.z, p[2]), (0, 0, 0, 1)))


def arm_worlds(psi, S, E, W, g):
    S, E, W, g = Vector(S), Vector(E), Vector(W), Vector(g)
    L = Vector((-math.sin(psi), math.cos(psi), 0))
    base = Matrix.Translation((-0.80, 0, 0.15)) @ Matrix.Rotation(psi, 4, 'Z')
    up = _frame(S, E - S, L)
    Eo = E + L * ARM_OFF
    fo = _frame(Eo, W - Eo, L)
    wr = _frame(W, g, L)
    root = W + g * 0.06
    return {'arm_base': base, 'upper_arm': up, 'forearm': fo, 'wrist': wr,
            'finger_l': _frame(root + L * FINGER_C, g, L),
            'finger_r': _frame(root - L * FINGER_C, g, L)}


def stem_point(axle):
    ax = Vector(axle)
    return Vector((ax.x - STEM_Z_LOCAL * math.tan(STEM_TILT), ax.y, ax.z + STEM_Z_LOCAL))


def loading_arm():
    T = stem_point((X_START, 0.27, 0.127))
    P = Vector((-0.80, 0, 0))
    d = Vector((T.x - P.x, T.y - P.y, 0))
    f = lambda p: d.x * -math.sin(p) + d.y * math.cos(p) - ARM_OFF
    lo, hi = math.radians(150), math.radians(179.9)
    for _ in range(80):
        mid = (lo + hi) / 2
        if f(lo) * f(mid) <= 0: hi = mid
        else: lo = mid
    psi = (lo + hi) / 2
    F = Vector((math.cos(psi), math.sin(psi), 0)); L = Vector((-math.sin(psi), math.cos(psi), 0))
    uw = d.dot(F) - GRIP
    W = Vector((P.x, P.y, 0)) + F * uw + L * ARM_OFF; W.z = T.z
    du, dh = uw, T.z - 0.60
    D = math.hypot(du, dh)
    phi = math.atan2(dh, du) + math.acos((L_UP**2 + D**2 - L_FORE**2) / (2 * L_UP * D))
    E = Vector((P.x, P.y, 0)) + F * (L_UP * math.cos(phi)); E.z = 0.60 + L_UP * math.sin(phi)
    return psi, Vector((-0.80, 0, 0.60)), E, W, F, T


def _set_chain(worlds):
    order = ['arm_base', 'upper_arm', 'forearm', 'wrist', 'finger_l', 'finger_r']
    for n in order:
        ob = bpy.data.objects[n]
        Wm = worlds[n]
        if ob.parent:
            ob.matrix_parent_inverse = Matrix.Identity(4)
            ob.matrix_basis = worlds[ob.parent.name].inverted() @ Wm
        else:
            ob.matrix_basis = Wm


def _outrigger(side, deployed):
    s = 1 if side == 'l' else -1
    arm = bpy.data.objects['outrigger_' + side]
    foot = bpy.data.objects['outrigger_%s_foot' % side]
    piv = Vector((-0.35, s * 0.525, 0.20))
    if deployed:
        alpha = math.asin((0.20 - 0.0105) / 0.40)
        R = Matrix.Rotation(-s * alpha, 4, 'X') @ Matrix.Rotation(s * math.pi / 2, 4, 'Z')
        footR = Matrix.Identity(4)
    else:
        R = Matrix.Identity(4)
        footR = Matrix.Rotation(-s * math.pi / 2, 4, 'X')   # mint top face points outward on both sides
    Wa = Matrix.Translation(piv) @ R
    arm.matrix_basis = Wa
    tip = Wa @ Vector((0.40, 0, 0))
    foot.matrix_parent_inverse = Matrix.Identity(4)
    foot.matrix_basis = Wa.inverted() @ (Matrix.Translation(tip) @ footR)


SHUTTLE_HOME, SHUTTLE_WAIT = 0.29, -0.70


def set_shuttle(side, x, gates_open):
    s = 1 if side == 'l' else -1
    bpy.data.objects['wheel_shuttle_' + side].location = (x, s * 0.27, 0.147)
    for k, w in (('a', 1), ('b', -1)):
        bpy.data.objects['shuttle_gate_%s_%s' % (side, k)].rotation_euler = (0, 0, -w * math.pi / 2 if gates_open else 0)


def set_stem_gates(side, ang):
    for k, w in (('a', 1), ('b', -1)):
        bpy.data.objects['stem_gate_%s_%s' % (side, k)].rotation_euler = (0, 0, w * ang)


FACE0, FACE_TH, CLAMP_OPEN = 0.34411, math.radians(14), math.radians(55.0)
JAWS = {'up': (-0.005, 0.068, 0.99, 1.05, 0.0235, 0.06), 'lo': (-0.005, 0.088, 0.75, 0.95, 0.046, 0.08)}


def clamp_matrix(side, lvl, k, frac):   # frac 0 = closed (clamping), 1 = open (V funnel)
    c = 0.27 if side == 'l' else -0.27
    w = 1 if k == 'a' else -1
    hx, hy, z0, z1 = JAWS[lvl][:4]
    sm = ((z0 - 0.277) + (z1 - 0.277)) / 2 / math.cos(FACE_TH)
    F = Matrix.Translation((FACE0, c, 0.277)) @ Matrix.Rotation(-FACE_TH, 4, 'Y')
    return F @ Matrix.Translation((hx, w * hy, sm)) @ Matrix.Rotation(-w * CLAMP_OPEN * frac, 4, 'Z')


def set_clamps(side, frac):
    for lvl in JAWS:
        for k in 'ab':
            ob = bpy.data.objects['stem_clamp_%s_%s_%s' % (lvl, side, k)]
            ob.rotation_mode = 'XYZ'; ob.matrix_basis = clamp_matrix(side, lvl, k, frac)


def set_pose(mode):
    O = bpy.data.objects
    if mode == 'transport':
        O['ref_scooter_l'].location = (0.29, 0.27, 0.277); O['ref_scooter_l'].rotation_euler = (0, 0, 0)
        O['ref_scooter_r'].location = (0.29, -0.27, 0.277)
        O['ramp'].rotation_euler = (0, math.pi / 2, 0); O['ramp_tip'].rotation_euler = (0, -math.pi, 0)   # folded outward
        set_shuttle('l', SHUTTLE_HOME, False); set_shuttle('r', SHUTTLE_HOME, False)
        set_clamps('l', 0.0 if mode == 'transport' else 1.0); set_clamps('r', 0.0)
        set_stand('l', 0.0); set_stand('r', 0.0)          # stand stays deployed; it runs in its own groove
        _outrigger('l', False); _outrigger('r', False)
        _set_chain(arm_worlds(0.0, (-0.80, 0, 0.60), (-0.2028, 0, 0.6580),
                              (-0.74, ARM_OFF, 0.54), (0, 0, -1)))
    elif mode == 'loading':
        O['ref_scooter_l'].location = (X_START, 0.27, 0.127); O['ref_scooter_l'].rotation_euler = (0, 0, 0)
        O['ref_scooter_r'].location = (0.29, -0.27, 0.277)
        O['ramp'].rotation_euler = (0, -RAMP_ANG, 0); O['ramp_tip'].rotation_euler = (0, 0, 0)
        set_shuttle('l', SHUTTLE_WAIT, True); set_shuttle('r', SHUTTLE_HOME, False)   # left shuttle waits at the ramp top
        set_clamps('l', 0.0 if mode == 'transport' else 1.0); set_clamps('r', 0.0)
        set_stand('l', 0.0); set_stand('r', 0.0)          # stand stays deployed; it runs in its own groove
        _outrigger('l', True); _outrigger('r', True)
        psi, S, E, W, F, T = loading_arm()
        _set_chain(arm_worlds(psi, S, E, W, F))
    bpy.context.view_layer.update()


# ===================== v19 additions (drive-through, side gripper) =====================
GRIP_S, GRIP_Y, GRIP_OUT = 0.73, 0.20, 0.10      # grip height along the stem, beam offset, head retract (outward)
DOCK_X = 0.65                                    # docked front axle x (v14: 0.29), shuttle home follows
SHUTTLE_HOME = DOCK_X
GRIP_X0, GRIP_Z0 = DOCK_X - 0.73 * math.sin(math.radians(14)), 0.277 + 0.73 * math.cos(math.radians(14))


def _lane(side): return (0.27, 1) if side == 'l' else (-0.27, -1)


def _set_chain(worlds):
    for n in ['arm_base', 'upper_arm', 'forearm', 'wrist', 'finger_l', 'finger_r']:
        ob = bpy.data.objects[n]; Wm = worlds[n]
        if ob.parent and ob.parent.name in worlds:
            ob.matrix_parent_inverse = Matrix.Identity(4); ob.matrix_basis = worlds[ob.parent.name].inverted() @ Wm
        else:
            ob.matrix_basis = Wm


UP_S, LO_S = ((0.99 - 0.277) + (1.05 - 0.277)) / 2 / math.cos(math.radians(14)), 0.86


CLAMP_SLIDE = 0.085
FX = Vector((math.cos(math.radians(14)), 0, math.sin(math.radians(14))))   # v12 dock-face normal (old frame)


def set_clamps(side, frac):              # v12 jaws turned 90 deg about the stem (side mounted); 0 closed .. 0.5 V-open (55 deg) .. 1 slid 8.5 cm into the panel
    c, s = _lane(side); u = CLAMP_SLIDE * max(0.0, 2 * frac - 1)
    SFo = Matrix.Translation((0.29, c, 0.277)) @ Matrix.Rotation(-math.radians(14), 4, 'Y')
    SFn = Matrix.Translation((DOCK_X, c, 0.277)) @ Matrix.Rotation(-math.radians(14), 4, 'Y')
    for lvl, ds in (('up', 0.0), ('lo', LO_S - UP_S)):
        Qm = SFn @ Matrix.Rotation(s * math.pi / 2, 4, 'Z') @ Matrix.Translation((0, 0, ds)) @ SFo.inverted()
        for k in 'ab':
            ob = bpy.data.objects['stem_clamp_%s_%s_%s' % (lvl, side, k)]
            ob.rotation_mode = 'XYZ'; ob.matrix_basis = Qm @ Matrix.Translation(FX * u) @ clamp_matrix(side, 'up', k, min(1.0, 2 * frac))
    vn = (SFn @ Matrix.Rotation(s * math.pi / 2, 4, 'Z') @ SFo.inverted()).to_3x3() @ FX     # slide direction (outward, into the panel)
    for n in ('dock_holder_', 'dock_holder_lo_', 'dock_clamp_mount_'):
        ob = bpy.data.objects[n + side]
        if 'closed_mw' in ob:
            m = list(ob['closed_mw']); ob.rotation_mode = 'XYZ'
            ob.matrix_basis = Matrix.Translation(vn * u) @ Matrix([m[0:4], m[4:8], m[8:12], m[12:16]])


def set_front_ramp(side, k):
    a1, a2 = ramp_state(k, RAMP_ANG)
    bpy.data.objects['front_ramp_' + side].rotation_euler = (0, -a1, 0)
    bpy.data.objects['front_ramp_%s_tip' % side].rotation_euler = (0, -a2, 0)


def set_fgates(side, frac):
    for k, w in (('a', 1), ('b', -1)):
        bpy.data.objects['shuttle_fgate_%s_%s' % (side, k)].rotation_euler = (0, 0, w * math.pi / 2 * frac)


_set_pose_v12 = set_pose


def set_pose(mode):
    # 'transport' | 'loading' (left head open and pulled out) | 'front_unload' (left front ramp down, gates open)
    for sd in 'lr':
        set_fgates(sd, 0.0); set_front_ramp(sd, 0.0)
    _set_pose_v12('loading' if mode == 'loading' else 'transport')
    bpy.data.objects['ref_scooter_r'].location.x = DOCK_X
    if mode != 'loading': bpy.data.objects['ref_scooter_l'].location.x = DOCK_X
    if mode == 'loading':
        set_clamps('l', 1.0)
    if mode == 'front_unload':
        set_front_ramp('l', 1.0); set_fgates('l', 1.0); _outrigger('l', True); _outrigger('r', True)
    bpy.context.view_layer.update()


# ---- v19: main arm on a centre rail (7th axis) ----
RAIL_MIN, RAIL_MAX = -0.80, 0.79
def arm_ik(bx, W, g, Lw, fc):
    # robot-frame worlds of the arm chain: base at (bx, 0, 0.15), wrist centre W, gripper axis g, finger axis Lw, finger offset fc
    W, g, Lw = Vector(W), Vector(g).normalized(), Vector(Lw)
    Lw = (Lw - g * Lw.dot(g)).normalized()
    dx, dy = W.x - bx, W.y; r = math.hypot(dx, dy)
    psi = math.atan2(dy, dx) + math.asin(max(-1.0, min(1.0, 0.11 / max(r, 0.111))))
    F = Vector((math.cos(psi), math.sin(psi), 0)); L = Vector((-math.sin(psi), math.cos(psi), 0))
    uw = Vector((dx, dy, 0)).dot(F); dh = W.z - 0.60
    D = max(0.06, min(L_UP + L_FORE - 0.002, math.hypot(uw, dh)))
    phi = math.atan2(dh, uw) + math.acos(max(-1.0, min(1.0, (L_UP ** 2 + D ** 2 - L_FORE ** 2) / (2 * L_UP * D))))
    S = Vector((bx, 0, 0.60)); E = Vector((bx, 0, 0)) + F * (L_UP * math.cos(phi)); E.z = 0.60 + L_UP * math.sin(phi)
    wd = arm_worlds(psi, S, E, W, g)
    wd['arm_base'] = Matrix.Translation((bx, 0, 0.15)) @ Matrix.Rotation(psi, 4, 'Z')
    wd['wrist'] = _frame(W, g, Lw); root = W + g * 0.06
    wd['finger_l'] = _frame(root + Lw * fc, g, Lw); wd['finger_r'] = _frame(root - Lw * fc, g, Lw)
    return wd, math.hypot(uw, dh)

