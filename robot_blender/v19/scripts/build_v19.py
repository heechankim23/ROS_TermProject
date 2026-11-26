# SCOOTOVER v19 (= v16 with the side posts/beam replaced by a slim mint dock panel on the outer side of each lane, Kickspot style): run with  Blender -b v12/scootover_v12.blend --python v19/scripts/build_v19.py
# Front unloading into a Kickspot slot. The mint dock panels, clamps and overhead parts are gone: each lane has a
# mint side beam on its outer side carrying a 3-stage telescope (+ vertical slide) whose C-head grips the stem from
# the side, holds the scooter upright and pushes it out through a lane tunnel and a bi-fold front ramp.
# No on-board charging (charging happens at the Kickspot). Rear loading (ramp, arm, shuttle) is unchanged from v12.
import bpy, bmesh, math, os
from mathutils import Matrix, Vector

PROJ = '/Users/khc/Desktop/2026-2/RobotOS/TermProject_robot_form'
O, C = bpy.data.objects, bpy.data.collections
SIDES = [('l', 0.27, 1), ('r', -0.27, -1)]       # side, lane centre y, outward sign


class MB:
    def __init__(s): s.bm = bmesh.new(); s.mats = []
    def _tag(s, verts, m):
        if m not in s.mats: s.mats.append(m)
        i = s.mats.index(m)
        for f in {f for v in verts for f in v.link_faces}: f.material_index = i
    def box(s, mn, mx, m, T=None):
        vs = bmesh.ops.create_cube(s.bm, size=1.0)['verts']
        M = Matrix.Translation([(a + b) / 2 for a, b in zip(mn, mx)]) @ Matrix.Diagonal((*[b - a for a, b in zip(mn, mx)], 1))
        bmesh.ops.transform(s.bm, matrix=(T @ M) if T else M, verts=vs); s._tag(vs, m)
    def cyl(s, c, r, L, axis, m, seg=20, T=None):
        vs = bmesh.ops.create_cone(s.bm, cap_ends=True, segments=seg, radius1=r, radius2=r, depth=L)['verts']
        M = Matrix.Translation(c) @ Vector((0, 0, 1)).rotation_difference(Vector(axis)).to_matrix().to_4x4()
        bmesh.ops.transform(s.bm, matrix=(T @ M) if T else M, verts=vs); s._tag(vs, m)
    def prism_xz(s, pts, y0, y1, m):
        v0 = [s.bm.verts.new((x, y0, z)) for x, z in pts]; v1 = [s.bm.verts.new((x, y1, z)) for x, z in pts]
        fs = [s.bm.faces.new(v0), s.bm.faces.new(v1[::-1])]
        for i in range(len(pts)):
            j = (i + 1) % len(pts); fs.append(s.bm.faces.new((v0[i], v1[i], v1[j], v0[j])))
        bmesh.ops.recalc_face_normals(s.bm, faces=fs); s._tag(v0 + v1, m)
    def build(s, name, coll, origin=(0, 0, 0), bevel=None):
        bmesh.ops.translate(s.bm, vec=-Vector(origin), verts=s.bm.verts)
        me = bpy.data.meshes.new(name); s.bm.to_mesh(me); s.bm.free()
        for m in s.mats: me.materials.append(bpy.data.materials[m])
        ob = bpy.data.objects.new(name, me); C[coll].objects.link(ob); ob.location = origin
        if bevel:
            bv = ob.modifiers.new('Bevel', 'BEVEL'); bv.width = bevel; bv.segments = 2; bv.limit_method = 'ANGLE'
        return ob


def keep_parent(ch, par):
    mw = ch.matrix_world.copy(); ch.parent = par; ch.matrix_parent_inverse = par.matrix_world.inverted(); ch.matrix_world = mw
def move_coll(ob, coll):
    for c in list(ob.users_collection): c.objects.unlink(ob)
    C[coll].objects.link(ob)
def bake(ob, mods):
    for m in ob.modifiers: m.show_viewport = m in mods
    dg = bpy.context.evaluated_depsgraph_get(); me = bpy.data.meshes.new_from_object(ob.evaluated_get(dg))
    for m in mods: ob.modifiers.remove(m)
    for m in ob.modifiers: m.show_viewport = True
    ob.data = me


# ---------- 1. remove the dock (panel, guides, holder, clamps, floor plates, front-wheel holder) ----------
for n in [o.name for o in O if o.name.startswith(('dock_panel_', 'dock_guide_', 'dock_frame_', 'wheel_holder_', 'stem_clamp_lo_'))]:
    bpy.data.objects.remove(O[n], do_unlink=True)

# ---------- 2. front body: lane tunnels (deck +-8.5 cm, deployed stand +13.5 cm) + stand notch at the front edge ----------
bf = O['body_front']; cut = []
for sd, c, s in SIDES:
    for mn, mx in (((0.38, c - 0.14, 0.15), (0.95, c + 0.14, 0.47)),      # lane tunnel
                   ((0.38, c + 0.08, 0.11), (0.95, c + 0.145, 0.16)),     # stand groove
                   ((0.38, c - 0.055, 0.135), (0.80, c + 0.055, 0.16))):  # shuttle channel continuation (dock now at x 0.65)
        b = MB(); b.box(mn, mx, 'mat_dark'); cut.append(b.build('_cut_' + sd, 'Stage'))
b = MB(); b.box((0.38, -0.09, 0.15), (0.88, 0.09, 0.47), 'mat_dark'); cut.append(b.build('_cut_arm', 'Stage'))   # centre slot: arm column travels to x 0.79
old = bf.data
for solver in ('EXACT', 'MANIFOLD', 'FLOAT'):
    mods = []
    for k in cut:
        m = bf.modifiers.new('cut', 'BOOLEAN'); m.operation = 'DIFFERENCE'; m.solver = solver; m.object = k; mods.append(m)
    bake(bf, mods)
    if len(bf.data.vertices) > len(old.vertices): print('BOOL_SOLVER', solver, len(bf.data.vertices)); break
    bf.data = old
for k in cut: bpy.data.objects.remove(k, do_unlink=True)
for sd, c, s in SIDES:                                          # headlights move outboard of the tunnels
    hl = O['headlight_' + sd]
    for v in hl.data.vertices: v.co.y *= 0.4
    hl.location.y = s * 0.47

# ---------- 5. bi-fold front ramps (one per lane, open on the stand side) ----------
for sd, c, s in SIDES:
    b = MB()
    b.box((0.0, c - 0.13, -0.03), (0.30, c + 0.09, 0.0), 'mat_metal')
    b.box((0.005, c - 0.05, -0.002), (0.295, c + 0.05, 0.0005), 'mat_rubber')   # rubber tread on the wheel track
    b.box((0.0, c - 0.13, 0.0), (0.30, c - 0.12, 0.02), 'mat_dark')           # guide lip (non-stand side)
    for y in (-0.10, 0.06): b.cyl((0.0, c + y, -0.015), 0.015, 0.05, (0, 1, 0), 'mat_dark', 16)
    fr = b.build('front_ramp_' + sd, 'Mechanisms', (0, c, 0), bevel=0.002); fr.location = (0.90, c, 0.15)
    b = MB(); b.prism_xz([(0, 0), (0.15, 0), (0.30, 0.03), (0, 0.03)], c - 0.13, c + 0.09, 'mat_metal')
    b.box((0.005, c - 0.05, 0.028), (0.28, c + 0.05, 0.0305), 'mat_rubber')
    b.box((0.0, c - 0.13, 0.03), (0.30, c - 0.12, 0.05), 'mat_dark')
    b.cyl((0.0, c - 0.02, 0.015), 0.013, 0.05, (0, 1, 0), 'mat_dark', 16)
    tp = b.build('front_ramp_%s_tip' % sd, 'Mechanisms', (0, c, 0), bevel=0.002)
    tp.parent = fr; tp.matrix_parent_inverse = Matrix.Identity(4); tp.location = (0.30, 0, -0.03)

# ---------- 6. shuttle: drop the fixed front stop, add front gates ----------
for sd, c, s in SIDES:
    sh = O['wheel_shuttle_' + sd]
    bm = bmesh.new(); bm.from_mesh(sh.data)
    for co, no in (((0.112, 0, 0), (1, 0, 0)), ((0, 0, 0.226 - 0.147), (0, 0, 1))):   # drop the front stop; lower the top under the deck
        r = bmesh.ops.bisect_plane(bm, geom=bm.verts[:] + bm.edges[:] + bm.faces[:], plane_co=co, plane_no=no, clear_outer=True)
        bmesh.ops.holes_fill(bm, edges=[e for e in r['geom_cut'] if isinstance(e, bmesh.types.BMEdge)], sides=0)
    bm.to_mesh(sh.data); bm.free(); sh.data.update()
    for k, w in (('a', 1), ('b', -1)):
        src = O['shuttle_gate_%s_%s' % (sd, k)]
        g = bpy.data.objects.new('shuttle_fgate_%s_%s' % (sd, k), src.data.copy()); C['Mechanisms'].objects.link(g)
        for m in src.modifiers:
            bv = g.modifiers.new(m.name, m.type); bv.width = m.width; bv.segments = m.segments
        g.parent = sh; g.matrix_parent_inverse = Matrix.Identity(4); g.location = (0.12, w * 0.043, 0.035)


# ---------- 4. side gripper per lane: posts, housing (vertical slide), 3 telescoping stages, C-head + closing finger ----------
XD = 0.65                                                          # docked front axle x (v14: 0.29)
UP_S, LO_S = ((0.99 - 0.277) + (1.05 - 0.277)) / 2 / math.cos(math.radians(14)), 0.86   # jaw centres along the stem (second pair above the v12 pair)
GX0, GZ0 = XD - 0.73 * math.sin(math.radians(14)), 0.277 + 0.73 * math.cos(math.radians(14))   # docked grip point (stem s = 0.73)
for sd, c, s in SIDES:
    yb = c + s * 0.20                                                  # beam axis, outer side of the lane
    # slim mint dock panel beside the lane, leaning with the stem (v12 dock style turned sideways); the C-head slides out of it
    sx = lambda z: XD - (z - 0.277) * math.tan(math.radians(14))          # docked stem axis x at height z
    prof = [(sx(0.45) - 0.12, 0.45), (sx(1.18) - 0.12, 1.18), (sx(1.18) + 0.15, 1.18), (sx(0.72) + 0.17, 0.72), (sx(0.45) + 0.20, 0.45)]
    y0, y1 = sorted((c + s * 0.13, c + s * 0.23))
    d = MB(); d.prism_xz(prof, y0, y1, 'mat_mint')
    d.box((sx(1.18) - 0.12, y0 - 0.002, 1.18), (sx(1.18) + 0.15, y1 + 0.002, 1.195), 'mat_white')        # top cap
    d.box((sx(1.18) - 0.10, y0 + 0.01, 1.195), (sx(1.18) + 0.13, y1 - 0.01, 1.198), 'mat_light')         # status light
    yi = c + s * 0.13; yi0, yi1 = sorted((yi, yi - s * 0.002))
    for z0, z1 in ((1.16, 1.175),):                                                                       # thin dark trim under the cap
        d.box((sx((z0 + z1) / 2) - 0.07, yi0, z0), (sx((z0 + z1) / 2) + 0.09, yi1, z1), 'mat_dark')
    yo = c + s * 0.23; yo0, yo1 = sorted((yo, yo + s * 0.002))                                             # white K on the outer face
    d.box((sx(0.8) - 0.02, yo0, 0.66), (sx(0.8), yo1, 0.86), 'mat_white')
    for ang, cz in ((-40, 0.81), (40, 0.71)):
        Mk = Matrix.Translation((sx(0.8) + 0.035, 0, cz)) @ Matrix.Rotation(math.radians(ang), 4, 'Y')
        d.box((-0.05, yo0, -0.01), (0.035, yo1, 0.01), 'mat_white', Mk)
    d.build('dock_side_' + sd, 'Body', (sx(0.75), c + s * 0.18, 0.75), bevel=0.005)
    # v12 stem clamps turned 90 deg about the stem axis so they close from the side panel: upper pair as in v12,
    # lower pair (same jaws) moved below the receiver pad; holder housings follow; short connectors to the panel
    SFo = Matrix.Translation((0.29, c, 0.277)) @ Matrix.Rotation(-math.radians(14), 4, 'Y')
    SFn = Matrix.Translation((XD, c, 0.277)) @ Matrix.Rotation(-math.radians(14), 4, 'Y')
    Q = lambda ds: SFn @ Matrix.Rotation(s * math.pi / 2, 4, 'Z') @ Matrix.Translation((0, 0, ds)) @ SFo.inverted()
    hu = O['dock_holder_' + sd]; H0 = hu.matrix_world.copy(); hu.matrix_world = Q(0.0) @ H0; hu['closed_mw'] = [x for r in Q(0.0) @ H0 for x in r]
    hl = bpy.data.objects.new('dock_holder_lo_' + sd, hu.data.copy()); C['Body'].objects.link(hl)
    for m in hu.modifiers:
        bv = hl.modifiers.new(m.name, m.type); bv.width = m.width; bv.segments = m.segments
    hl.matrix_world = Q(LO_S - UP_S) @ H0; hl['closed_mw'] = [x for r in Q(LO_S - UP_S) @ H0 for x in r]
    for k in 'ab':
        src = O['stem_clamp_up_%s_%s' % (sd, k)]
        g = bpy.data.objects.new('stem_clamp_lo_%s_%s' % (sd, k), src.data.copy()); C['Mechanisms'].objects.link(g)
        for m in src.modifiers:
            bv = g.modifiers.new(m.name, m.type); bv.width = m.width; bv.segments = m.segments
    b = MB()
    for sc_ in (UP_S, LO_S):
        Tm = SFn @ Matrix.Rotation(s * math.pi / 2, 4, 'Z') @ Matrix.Translation((0, 0, sc_))
        b.box((0.112, -0.09, -0.01), (0.134, 0.09, 0.05), 'mat_dark', Tm)                                # connector: housing -> panel
    mo = b.build('dock_clamp_mount_' + sd, 'Body', (XD - 0.15, c + s * 0.12, 0.8), bevel=0.002); mo['closed_mw'] = [x for r in Matrix.Translation(mo.location) for x in r]

# ---------- 4b. arm travel rail (centre, x -0.95..0.84) and lidar mast moved to the right-front corner ----------
b = MB()
for w in (1, -1):
    y0, y1 = sorted((w * 0.05, w * 0.065)); b.box((-0.88, y0, 0.149), (0.87, y1, 0.153), 'mat_metal')
b.box((-0.88, -0.03, 0.149), (0.87, 0.03, 0.151), 'mat_dark')                                      # rack
b.build('arm_rail', 'Body', (0, 0, 0.15))
for n, z in (('mast', 0.45), ('lidar', 1.04)): O[n].location = (0.86, -0.48, z)

# ---------- 7. robot root (lets the robot back away after docking) ----------
root = bpy.data.objects.new('robot_root', None); root.empty_display_type = 'PLAIN_AXES'; root.empty_display_size = 0.3
C['Mechanisms'].objects.link(root)
for cn in ('Body', 'Mechanisms', 'Sensors'):
    for o in list(C[cn].objects):
        if o.parent is None and o is not root:
            o.parent = root; o.matrix_parent_inverse = Matrix.Identity(4)


# ---------- 8. pose text: v12 functions + v19 parts ----------
POSE_ADD = """

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

"""
t = bpy.data.texts['scootover_pose.py']; src = t.as_string()
src = src.replace('# SCOOTOVER v12 pose switcher', '# SCOOTOVER v19 pose switcher') + POSE_ADD
t.clear(); t.write(src)
open(os.path.join(PROJ, 'v19', 'scripts', 'pose_v19.py'), 'w').write(src)
ns = {}; exec(src, ns); ns['set_pose']('transport')
bpy.ops.wm.save_as_mainfile(filepath=os.path.join(PROJ, 'v19', 'scootover_v19.blend'))
print('BUILD_OK', len(bpy.data.objects))
