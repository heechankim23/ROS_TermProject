# SCOOTOVER v19 animation: v12 rear loading (1..729) + front unloading with the robot parked once in front of the slot.
# The rail-mounted arm holds the stem from the inner side the whole way, pushes the scooter down the front ramp, across the
# station's own ramp and into the holder; when the station latch locks (rear wheel still on our ramp) the arm lets go,
# the robot backs off, the ramp slides out from under the rear wheel and the scooter settles about the latched stem.
# Run: Blender -b v19/scootover_v19.blend --python v19/scripts/anim_v19.py   -> v19/scootover_v19_anim.blend
# Needs v13/scripts/v12_loading.json (v13/scripts/dump_v12_loading.py on v12/scootover_v12_anim.blend).
import bpy, json, math, os
from mathutils import Matrix, Vector

PROJ = '/Users/khc/Desktop/2026-2/RobotOS/TermProject_robot_form'
O = bpy.data.objects; sc = bpy.context.scene
ns = {}; exec(bpy.data.texts['scootover_pose.py'].as_string(), ns)
ns['set_pose']('transport')
D = json.load(open(os.path.join(PROJ, 'v13', 'scripts', 'v12_loading.json')))

# ---------------- Kickspot (target = empty slot 4 on the left lane; slot 3 occupied) ----------------
XS, YL = 2.02, 0.27          # robot front 1.12 m before the slot: folded right-lane front ramp clears the neighbour's tail by 2 cm
with bpy.data.libraries.load(os.path.join(PROJ, 'kickspot', 'kickspot.blend'), link=False) as (src, dst):
    dst.objects = [n for n in src.objects if n.startswith(('kickspot_', 'post_', 'holder_', 'latch_', 'scooter_'))]
kc = bpy.data.collections.new('Kickspot'); sc.collection.children.link(kc)
st = bpy.data.objects.new('kickspot_station', None); kc.objects.link(st); st.location = (XS, YL - 0.93, 0)
for o in dst.objects:
    kc.objects.link(o)
    if o.type == 'MESH':
        for slot in o.material_slots:
            if slot.material and '.' in slot.material.name and slot.material.name.split('.')[0] in bpy.data.materials:
                slot.material = bpy.data.materials[slot.material.name.split('.')[0]]
for o in dst.objects:
    if o.parent is None: o.parent = st; o.matrix_parent_inverse = Matrix.Identity(4)
PHI_S, TH = math.asin(0.06 / 0.84), math.radians(14)
def MS(y): return Matrix.Translation((0, y, 0.187)) @ Matrix.Rotation(-PHI_S, 4, 'Y') @ Matrix.Rotation(-TH, 4, 'Y')
PV = Vector((-0.044, -0.038, 0.84))
def latch4(a): O['latch_4'].matrix_basis = MS(0.93) @ Matrix.Translation(PV) @ Matrix.Rotation(math.radians(a), 4, 'Z')
O['ref_scooter_r'].parent = O['robot_root']; O['ref_scooter_r'].matrix_parent_inverse = Matrix.Identity(4)

# ---------------- timeline ----------------
F = dict(od0=730, od1=770, fr0=775, fr1=835, gt0=840, gt1=855, sb0=855, sb1=895, ab0=860, ab1=920, au0=920, au1=960,
         ai0=960, ai1=975, fc0=975, fc1=985, so0=985, so1=1015, tr0=1020, tr1=1300, kl0=1305, kl1=1325, fo0=1330, fo1=1340,
         aw0=1340, aw1=1380, at0=1380, at1=1470, of0=1475, of1=1515, bk0=1520, bk1=1600, rf0=1605, rf1=1665, sr0=1605, sr1=1645,
         gc0=1645, gc1=1660, cc0=1660, cc1=1680, END=1690,
         li0=642, li1=657, lf0=657, lf1=670, dk0=565, dk1=640)
def ss(t): t = max(0.0, min(1.0, t)); return t * t * (3 - 2 * t)
def seg(f, a, b): return ss((f - F[a]) / (F[b] - F[a]))
def lerp(a, b, t): return a + (b - a) * t
DX = ns['DOCK_X']
RA = ns['RAMP_ANG']; R, WB = 0.127, 0.84; TOE = 0.90 + 0.60 * math.cos(RA)
RAIL_MAX = ns['RAIL_MAX']; B0 = -0.10
BACK = -0.65                                  # robot backs off this far (folding ramp clears the settled rear wheel)

def robot_x(f): return BACK * seg(f, 'bk0', 'bk1')

# ---------------- ground under the left lane (world), wheel contact, scooter pose ----------------
def h(x, rx, ramp_on):
    xr = x - rx; best = 0.0
    if -0.90 <= xr <= 0.90: best = 0.15
    if ramp_on and 0.90 < xr <= TOE: best = max(best, 0.15 - (xr - 0.90) * math.tan(RA))
    if XS - 0.52 < x <= XS - 0.30: best = max(best, 0.06 * (x - (XS - 0.52)) / 0.22)
    if x > XS - 0.30: best = max(best, 0.06)
    return best
def zc(x, rx, ramp_on):
    return max(h(x + u, rx, ramp_on) + math.sqrt(max(0.0, R * R - u * u)) for u in [R * (i / 30.0 - 1) for i in range(61)])
def scooter_M(xf, rx, ramp_on):
    zf = zc(xf, rx, ramp_on); lo, hi = -0.5, 0.5
    for _ in range(50):
        th = (lo + hi) / 2
        if zf - WB * math.sin(th) - zc(xf - WB * math.cos(th), rx, ramp_on) > 0: lo = th
        else: hi = th
    return Matrix.Translation((xf, YL, zf)) @ Matrix.Rotation(-(lo + hi) / 2, 4, 'Y')
ST = Matrix.Rotation(-TH, 4, 'Y')
def stem_pt(M, s): return M @ ST @ Vector((0, 0, s))
def stem_x_at(M, z):
    p0 = M @ Vector((0, 0, 0)); p1 = stem_pt(M, 1.0); t = (z - p0.z) / (p1.z - p0.z); return p0.x + t * (p1.x - p0.x)

# the stem is held at the top of the holder's rubber stop (s 0.875 of the docked design pose)
M_fin = scooter_M(XS, 0.0, False)
P_HOLD = stem_pt(M_fin, 0.875)
def xf_held(rx, ramp_on):                    # front axle that puts the stem through P_HOLD (wheels on their surfaces)
    lo, hi = XS - 0.40, XS + 0.02
    for _ in range(40):
        m = (lo + hi) / 2
        if stem_x_at(scooter_M(m, rx, ramp_on), P_HOLD.z) < P_HOLD.x: lo = m
        else: hi = m
    return (lo + hi) / 2
XL = xf_held(0.0, True)                      # latch pose: rear wheel still on our ramp

def grip_frame(M, s=0.36):                   # grip from the inner side of the lane: fingers in front of / behind the stem
    T = stem_pt(M, s); a = (M.to_3x3() @ ST.to_3x3() @ Vector((0, 0, 1))).normalized()
    g = Vector((0, 1, 0)); g = (g - a * g.dot(a)).normalized(); Lw = a.cross(g).normalized()
    return T, g, Lw

# ---------------- arm helpers (robot frame) ----------------
FOLD_W, FOLD_G, FOLD_L = Vector((0.06, -0.11, 0.54)), Vector((0, 0, -1)), Vector((0, 1, 0))
READY_W, READY_G, READY_L = Vector((1.30, 0.15, 0.80)), Vector((1, 0, 0)), Vector((0, 1, 0))
FC_C, FC_O, GRIP = ns['FINGER_C'], 0.07, ns['GRIP']
def nl(a, b, t): return (Vector(a) * (1 - t) + Vector(b) * t).normalized()
maxD = [0.0, 0]; CUR = [0]
def put_arm(bx, W, g, Lw, fc):
    wd, Dv = ns['arm_ik'](bx, W, g, Lw, fc)
    if Dv > maxD[0]: maxD[0], maxD[1] = Dv, CUR[0]
    ns['_set_chain'](wd)

def outrigger(side, t):
    s = 1 if side == 'l' else -1
    arm, foot = O['outrigger_' + side], O['outrigger_%s_foot' % side]
    alpha = math.asin((0.20 - 0.0105) / 0.40); t1 = min(1.0, 2 * t); t2 = max(0.0, 2 * t - 1)
    Wa = Matrix.Translation((-0.35, s * 0.525, 0.20)) @ Matrix.Rotation(-s * alpha * t2, 4, 'X') @ Matrix.Rotation(s * math.pi / 2 * t1, 4, 'Z')
    arm.matrix_basis = Wa
    foot.matrix_parent_inverse = Matrix.Identity(4)
    foot.matrix_basis = Wa.inverted() @ Matrix.Translation(Wa @ Vector((0.40, 0, 0))) @ Matrix.Rotation(-s * math.pi / 2 * (1 - t2), 4, 'X')

WHEELS = ['wheel_fl', 'wheel_fr', 'wheel_rl', 'wheel_rr']; W0 = {n: O[n].matrix_basis.copy() for n in WHEELS}
GRIPS = ['stem_clamp_%s_l_%s' % (l, k) for l in ('up', 'lo') for k in 'ab'] + ['dock_holder_l', 'dock_holder_lo_l', 'dock_clamp_mount_l']
def loading_grip(f): ns['set_clamps']('l', 1 - seg(f, 'li0', 'lf1'))

M_dock = scooter_M(DX, 0.0, False)
T0, g0, L0 = grip_frame(M_dock); W_DOCK = T0 - g0 * GRIP
def scooter_state(f):
    if f < F['tr1']: return scooter_M(lerp(DX, XL, seg(f, 'tr0', 'tr1')), 0.0, True)
    rx = robot_x(f); on = f < F['rf0']
    return scooter_M(xf_held(rx, on), rx, on)                  # settles about the latched stem as the ramp slides out

def unload(f):
    for n in KEYED: O[n].rotation_mode = 'XYZ'
    t_out = seg(f, 'od0', 'od1') - seg(f, 'of0', 'of1'); outrigger('l', t_out); outrigger('r', t_out)
    ns['set_front_ramp']('l', seg(f, 'fr0', 'fr1') - seg(f, 'rf0', 'rf1'))
    g = seg(f, 'gt0', 'gt1') - seg(f, 'gc0', 'gc1'); ns['set_fgates']('l', g)
    for k, w in (('a', 1), ('b', -1)): O['shuttle_gate_l_%s' % k].rotation_euler = (0, 0, -w * math.pi / 2 * g)
    O['wheel_shuttle_l'].location.x = DX - 1.10 * (seg(f, 'sb0', 'sb1') - seg(f, 'sr0', 'sr1'))
    ns['set_clamps']('l', seg(f, 'so0', 'so1') - seg(f, 'cc0', 'cc1'))
    M = scooter_state(f); O['ref_scooter_l'].matrix_basis = M
    rx = robot_x(f); O['robot_root'].location = (rx, 0, 0)
    for n in WHEELS: O[n].matrix_basis = W0[n] @ Matrix.Rotation(rx / 0.125, 4, 'Y')
    latch4(95 * (1 - seg(f, 'kl0', 'kl1')))
    # ---- arm (robot frame) ----
    T, gg, Lw = grip_frame(M); Tr = T - Vector((rx, 0, 0)); Wg = Tr - gg * GRIP
    if f < F['au0']:
        bx = lerp(-0.80, B0, seg(f, 'ab0', 'ab1')); put_arm(bx, Vector((bx, 0, 0)) + FOLD_W, FOLD_G, FOLD_L, FC_C)
    elif f < F['ai0']:
        t = seg(f, 'au0', 'au1'); Wp = Wg - gg * 0.10
        put_arm(B0, (Vector((B0, 0, 0)) + FOLD_W).lerp(Wp, t), nl(FOLD_G, gg, t), nl(FOLD_L, Lw, t), lerp(FC_C, FC_O, t))
    elif f < F['fo0']:
        bx = min(RAIL_MAX, max(B0, Wg.x - (W_DOCK.x - B0)))
        put_arm(bx, Wg - gg * 0.10 * (1 - seg(f, 'ai0', 'ai1')), gg, Lw, lerp(FC_O, FC_C, seg(f, 'fc0', 'fc1')))
    elif f < F['at0']:                       # open, back out sideways along the gripper axis, then to a ready pose
        bx = min(RAIL_MAX, max(B0, Wg.x - (W_DOCK.x - B0)))
        t = ss((f - F['aw0'] - 12) / (F['aw1'] - F['aw0'] - 12)); Wb = Wg - gg * 0.12 * ss((f - F['aw0']) / 12)
        put_arm(bx, Wb.lerp(READY_W, t), nl(gg, READY_G, t), nl(Lw, READY_L, t), lerp(FC_C, FC_O, seg(f, 'fo0', 'fo1')))
    else:
        t = seg(f, 'at0', 'at1'); bx = lerp(BX_REL, -0.80, t)
        put_arm(bx, READY_W.lerp(Vector((bx, 0, 0)) + FOLD_W, t), nl(READY_G, FOLD_G, t), nl(READY_L, FOLD_L, t), lerp(FC_O, FC_C, t))

_T, _g, _L = grip_frame(scooter_M(XL, 0.0, True)); BX_REL = min(RAIL_MAX, max(B0, (_T - _g * GRIP).x - (W_DOCK.x - B0)))
ARM = ['arm_base', 'upper_arm', 'forearm', 'wrist', 'finger_l', 'finger_r']
NEW = ['robot_root', 'front_ramp_l', 'front_ramp_l_tip', 'shuttle_fgate_l_a', 'shuttle_fgate_l_b', 'latch_4'] + WHEELS + GRIPS + ARM
OLD = [n for n in D['frames']['1'] if n in O]
KEYED = sorted(set(NEW) | set(OLD))
prev = {}
def key(f):
    bpy.context.view_layer.update()
    for n in KEYED:
        ob = O[n]; ob.rotation_mode = 'QUATERNION'
        loc, q, scl = ob.matrix_basis.decompose()
        if n in prev: q.make_compatible(prev[n])
        prev[n] = q; ob.location = loc; ob.rotation_quaternion = q
        ob.keyframe_insert('location', frame=f); ob.keyframe_insert('rotation_quaternion', frame=f)

for o in KEYED:
    if O[o].animation_data: O[o].animation_data_clear()
latch4(95)
for f in range(1, 730):
    for n in KEYED: O[n].rotation_mode = 'XYZ'
    for n, m in D['frames'][str(f)].items():
        if n in O: O[n].matrix_basis = Matrix(m)
    if f >= F['dk0']:
        x = lerp(-0.10, DX, seg(f, 'dk0', 'dk1'))
        for n in ('ref_scooter_l', 'wheel_shuttle_l'): O[n].location.x = x
    loading_grip(f); key(f)
for f in range(730, F['END'] + 1):
    CUR[0] = f; unload(f); key(f)

sc.timeline_markers.clear()
for fr, name in D['markers']:
    sc.timeline_markers.new(name.replace('6 집게 닫힘·잠금', '6 옆 집게(v12 집게)가 닫힘').replace('5 도킹 (열린 집게 V로 진입)', '5 셔틀이 앞쪽 거치 위치(x 0.65)까지'), frame=640 if fr == 612 else fr)
for fr, name in [(730, '[앞 하차] 1 충전소 앞 정차, 지지대·앞 램프'), (840, '2 셔틀 물러남, 팔이 레일 타고 와서 스템을 안쪽 옆에서 잡음'),
                 (985, '3 옆 집게 열고 패널 안으로'), (1020, '4 팔이 밀어 램프·충전소 경사 지나 거치대까지'), (1305, '5 충전기 걸쇠 잠김 (뒷바퀴는 아직 램프 위)'),
                 (1330, '6 팔 놓고 레일 뒤로 복귀'), (1475, '7 지지대 접고 후진: 램프가 빠지며 킥보드가 제자리에 앉음'), (1605, '8 앞 램프 접음, 셔틀 복귀')]:
    sc.timeline_markers.new(name, frame=fr)
sc.frame_start, sc.frame_end = 1, F['END']; sc.frame_set(1)
bpy.ops.wm.save_as_mainfile(filepath=os.path.join(PROJ, 'v19', 'scootover_v19_anim.blend'))
Ml = scooter_M(XL, 0.0, True)
print('ANIM_OK', F['END'], 'max arm reach D %.3f / %.2f at frame %d' % (maxD[0], ns['L_UP'] + ns['L_FORE'], maxD[1]),
      'latch xf %.3f (design %.3f), rear wheel z at latch %.3f' % (XL, XS, (Ml @ Vector((-0.84, 0, 0))).z - 0.127))
