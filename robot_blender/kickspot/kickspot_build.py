# Kickspot-style 4-slot charging station for the Ninebot MAX G30 (standalone: run in Blender, rebuilds kickspot.blend)
# ===== shared helpers (materials, mesh builder, G30 model) copied from the SCOOTOVER v12 build =====
import bpy, bmesh, math, os
from mathutils import Vector, Matrix

OUT = '/Users/khc/Desktop/2026-2/RobotOS/TermProject_robot_form'
sc = bpy.context.scene
T5 = math.tan(math.radians(5))

# ---------------- prep ----------------
for o in list(bpy.data.objects): bpy.data.objects.remove(o, do_unlink=True)
for c in list(bpy.data.collections): bpy.data.collections.remove(c)
for _ in range(2):
    for blk in (bpy.data.meshes, bpy.data.materials, bpy.data.cameras, bpy.data.lights, bpy.data.curves):
        for b in list(blk):
            if b.users == 0: blk.remove(b)
for t in list(bpy.data.texts): bpy.data.texts.remove(t)
us = sc.unit_settings; us.system = 'METRIC'; us.scale_length = 1.0; us.length_unit = 'METERS'
C = {}
for n in ['Body', 'Mechanisms', 'Sensors', 'Reference', 'Stage']:
    c = bpy.data.collections.new(n); sc.collection.children.link(c); C[n] = c

def lin(h):
    h = h.lstrip('#'); v = [int(h[i:i+2], 16) / 255 for i in (0, 2, 4)]
    return [x / 12.92 if x <= 0.04045 else ((x + 0.055) / 1.055) ** 2.4 for x in v]
MAT = {}
def mat(name, hexc, rough, metal=0.0, emit=0.0):
    m = bpy.data.materials.new(name)
    try: m.use_nodes = True
    except Exception: pass
    nt = m.node_tree
    b = next((n for n in nt.nodes if n.type == 'BSDF_PRINCIPLED'), None)
    if b is None:
        out = next((n for n in nt.nodes if n.type == 'OUTPUT_MATERIAL'), None) or nt.nodes.new('ShaderNodeOutputMaterial')
        b = nt.nodes.new('ShaderNodeBsdfPrincipled'); nt.links.new(b.outputs[0], out.inputs['Surface'])
    col = lin(hexc)
    b.inputs['Base Color'].default_value = (*col, 1)
    b.inputs['Roughness'].default_value = rough
    b.inputs['Metallic'].default_value = metal
    if emit:
        k = 'Emission Color' if 'Emission Color' in b.inputs else 'Emission'
        b.inputs[k].default_value = (*col, 1); b.inputs['Emission Strength'].default_value = emit
    m.diffuse_color = (*col, 1); m.roughness = rough; m.metallic = metal
    MAT[name] = m
mat('mat_white', '#F2F2F0', 0.35)
mat('mat_dark', '#2B2E33', 0.5)
mat('mat_mint', '#3CC8A8', 0.4)
mat('mat_orange', '#F28C28', 0.45)
mat('mat_metal', '#8C9096', 0.35, 0.8)
mat('mat_rubber', '#151515', 0.8)
mat('mat_light', '#FFFFFF', 0.3, 0.0, 4.0)
mat('mat_mint_glow', '#3CC8A8', 0.3, 0.0, 3.0)
mat('mat_glass', '#111418', 0.12)
mat('mat_scooter', '#4A4E54', 0.5)
mat('mat_ground', '#D6D7D9', 0.9)

class MB:
    def __init__(s): s.bm = bmesh.new(); s.mats = []
    def mi(s, m):
        if m not in s.mats: s.mats.append(m)
        return s.mats.index(m)
    def _tag(s, verts, m):
        i = s.mi(m)
        for f in {f for v in verts for f in v.link_faces}: f.material_index = i
    def box(s, mn, mx, m, M=None):
        mn, mx = Vector(mn), Vector(mx); c = (mn + mx) / 2; d = mx - mn
        T = Matrix.Translation(c) @ Matrix.Diagonal((d.x, d.y, d.z, 1))
        if M is not None: T = M @ T
        r = bmesh.ops.create_cube(s.bm, size=1.0, matrix=T); s._tag(r['verts'], m); return r['verts']
    def cyl(s, center, r, length, axis, m, seg=24, M=None):
        axis = Vector(axis).normalized()
        T = Matrix.Translation(center) @ Vector((0, 0, 1)).rotation_difference(axis).to_matrix().to_4x4()
        if M is not None: T = M @ T
        res = bmesh.ops.create_cone(s.bm, cap_ends=True, cap_tris=False, segments=seg,
                                    radius1=r, radius2=r, depth=length, matrix=T)
        s._tag(res['verts'], m); return res['verts']
    def sphere(s, center, r, m, M=None):
        T = Matrix.Translation(center)
        if M is not None: T = M @ T
        res = bmesh.ops.create_uvsphere(s.bm, u_segments=20, v_segments=12, radius=r, matrix=T)
        s._tag(res['verts'], m); return res['verts']
    def beam(s, p1, p2, wy, t, m, M=None):
        p1, p2 = Vector(p1), Vector(p2); a = (p2 - p1).normalized(); L = Vector((0, 1, 0)); z = a.cross(L)
        R = Matrix(((a.x, L.x, z.x, 0), (a.y, L.y, z.y, 0), (a.z, L.z, z.z, 0), (0, 0, 0, 1)))
        T = Matrix.Translation((p1 + p2) / 2) @ R
        if M is not None: T = M @ T
        l = (p2 - p1).length
        return s.box((-l/2, -wy/2, -t/2), (l/2, wy/2, t/2), m, T)
    def prism_xz(s, pts, y0, y1, m):
        v0 = [s.bm.verts.new((x, y0, z)) for x, z in pts]
        v1 = [s.bm.verts.new((x, y1, z)) for x, z in pts]
        fs = [s.bm.faces.new(v0), s.bm.faces.new(v1)]
        n = len(pts)
        for i in range(n):
            fs.append(s.bm.faces.new((v0[i], v0[(i+1) % n], v1[(i+1) % n], v1[i])))
        bmesh.ops.recalc_face_normals(s.bm, faces=fs)
        s._tag(v0 + v1, m); return v0 + v1
    def arc_ring(s, cx, cz, r0, r1, y0, y1, m, a0=0, a1=math.pi, seg=20):
        pts = [(cx + r1*math.cos(a0 + (a1-a0)*i/seg), cz + r1*math.sin(a0 + (a1-a0)*i/seg)) for i in range(seg+1)]
        pts += [(cx + r0*math.cos(a1 - (a1-a0)*i/seg), cz + r0*math.sin(a1 - (a1-a0)*i/seg)) for i in range(seg+1)]
        bm = s.bm; vs = []
        for i in range(seg):
            q = [pts[i], pts[i+1], pts[2*seg+1-(i+1)], pts[2*seg+1-i]]
            v0 = [bm.verts.new((x, y0, z)) for x, z in q]; v1 = [bm.verts.new((x, y1, z)) for x, z in q]
            fs = [bm.faces.new(v0), bm.faces.new(v1)] + [bm.faces.new((v0[k], v0[(k+1) % 4], v1[(k+1) % 4], v1[k])) for k in range(4)]
            bmesh.ops.recalc_face_normals(bm, faces=fs); vs += v0 + v1
        bmesh.ops.remove_doubles(bm, verts=vs, dist=1e-6)
        vs = [v for v in vs if v.is_valid]; s._tag(vs, m); return vs
    def build(s, name, coll, origin=(0, 0, 0), bevel=None):
        o = Vector(origin)
        bmesh.ops.translate(s.bm, verts=s.bm.verts[:], vec=-o)
        for f in s.bm.faces: f.smooth = True
        for e in s.bm.edges:
            e.smooth = not (len(e.link_faces) != 2 or e.calc_face_angle(0) > 0.6)
        me = bpy.data.meshes.new(name); s.bm.to_mesh(me); s.bm.free()
        for m in s.mats: me.materials.append(MAT[m])
        ob = bpy.data.objects.new(name, me); C[coll].objects.link(ob); ob.location = o
        if bevel: add_bevel(ob, bevel)
        return ob

def add_bevel(ob, w=0.012, seg=2):
    md = ob.modifiers.new('Bevel', 'BEVEL'); md.width = w; md.segments = seg
    md.limit_method = 'ANGLE'; md.angle_limit = math.radians(40)
def bool_cut(ob, cutters):
    for c in cutters:
        sc.collection.objects.link(c)
        md = ob.modifiers.new('cut', 'BOOLEAN'); md.operation = 'DIFFERENCE'; md.object = c
        try: md.solver = 'EXACT'
        except Exception: pass
        try: md.material_mode = 'TRANSFER'
        except Exception: pass
    bpy.context.view_layer.update()
    dg = bpy.context.evaluated_depsgraph_get()
    new = bpy.data.meshes.new_from_object(ob.evaluated_get(dg))
    old = ob.data; ob.modifiers.clear(); ob.data = new
    bpy.data.meshes.remove(old); new.name = ob.name
    for c in cutters:
        me = c.data; bpy.data.objects.remove(c, do_unlink=True); bpy.data.meshes.remove(me)
def bool_union(ob, parts):
    for c in parts:
        sc.collection.objects.link(c)
        md = ob.modifiers.new('join', 'BOOLEAN'); md.operation = 'UNION'; md.object = c
        try: md.solver = 'EXACT'
        except Exception: pass
    bpy.context.view_layer.update()
    dg = bpy.context.evaluated_depsgraph_get()
    new = bpy.data.meshes.new_from_object(ob.evaluated_get(dg))
    old = ob.data; ob.modifiers.clear(); ob.data = new
    bpy.data.meshes.remove(old); new.name = ob.name
    for c in parts:
        me = c.data; bpy.data.objects.remove(c, do_unlink=True); bpy.data.meshes.remove(me)
def merge_into(dst, src):
    bpy.context.view_layer.update()
    names = [m.name for m in dst.data.materials]
    for m in src.data.materials:
        if m.name not in names: dst.data.materials.append(m); names.append(m.name)
    remap = [names.index(m.name) for m in src.data.materials]
    sm = src.data.copy()
    for p in sm.polygons: p.material_index = remap[p.material_index]
    sm.transform(dst.matrix_world.inverted() @ src.matrix_world)
    bm = bmesh.new(); bm.from_mesh(dst.data); bm.from_mesh(sm); bm.to_mesh(dst.data); bm.free()
    bpy.data.meshes.remove(sm); me = src.data; bpy.data.objects.remove(src, do_unlink=True); bpy.data.meshes.remove(me)
def cutter_cyl(center, r, length, axis, m='mat_dark'):
    b = MB(); b.cyl(center, r, length, axis, m, 40); ob = b.build('_cut', 'Stage'); C['Stage'].objects.unlink(ob); return ob
def cutter_box(mn, mx, m='mat_dark'):
    b = MB(); b.box(mn, mx, m); ob = b.build('_cut', 'Stage'); C['Stage'].objects.unlink(ob); return ob
def parent(child, par):
    child.parent = par; child.matrix_parent_inverse = Matrix.Identity(4)
def paint(b, rule):
    """rule(face) -> material name or None"""
    for f in b.bm.faces:
        m = rule(f)
        if m: f.material_index = b.mi(m)
def bisect(b, z):
    g = b.bm.verts[:] + b.bm.edges[:] + b.bm.faces[:]
    bmesh.ops.bisect_plane(b.bm, geom=g, plane_co=(0, 0, z), plane_no=(0, 0, 1))
SIDES = [('l', 1), ('r', -1)]

TYRE_PROFILE = [(0.0, 0.004, 0.127), (0.004, 0.010, 0.1255), (0.010, 0.016, 0.1224),
                (0.016, 0.022, 0.1174), (0.022, 0.027, 0.110), (0.027, 0.030, 0.100)]
# ---------------- reference scooters (generic, with wireless receiver pad) ----------------
STEM_DEG, WBASE = 14.0, 0.84      # Ninebot MAX G30: stem rake and wheelbase (estimated from photos)
def build_scooter(name):
    th = math.radians(STEM_DEG); St = Matrix.Rotation(-th, 4, 'Y')
    Ls = 1.06 / math.cos(th)                          # handlebar centre 1.06 above the axle -> 1.203 overall
    b = MB()
    for x in (0.0, -WBASE):                            # 10" pneumatic tyres, rounded section
        for y0, y1, r in TYRE_PROFILE:
            for sgn in (1, -1):
                b.cyl((x, sgn * (y0 + y1) / 2, 0), r, y1 - y0, (0, 1, 0), 'mat_rubber', 32)
        b.cyl((x, 0, 0), 0.078, 0.062, (0, 1, 0), 'mat_dark', 24)
        b.cyl((x, 0, 0), 0.03, 0.064, (0, 1, 0), 'mat_metal', 16)
    # deck 0.51 x 0.17, 82 mm ground clearance, grip tape, raised tail
    b.box((-0.68, -0.085, -0.045), (-0.17, 0.085, 0.035), 'mat_scooter')
    b.box((-0.66, -0.075, 0.035), (-0.20, 0.075, 0.038), 'mat_rubber')
    b.box((-0.73, -0.06, -0.03), (-0.68, 0.06, 0.06), 'mat_scooter')
    for s in (1, -1):                                  # rear wheel mounts + fender struts
        b.box((-0.90, s*0.040, -0.025), (-0.70, s*0.050, 0.03), 'mat_scooter')
        b.box((-0.80, s*0.040, 0.03), (-0.72, s*0.050, 0.10), 'mat_scooter')
    joint = St @ Vector((0, 0, 0.26))
    b.beam((-0.20, 0, -0.005), (joint.x, 0, joint.z), 0.09, 0.06, 'mat_scooter')     # deck nose -> folding joint
    for s in (1, -1):                                  # front fork
        b.box((-0.018, s*0.040, 0.0), (0.018, s*0.050, 0.24), 'mat_scooter', St)
    b.box((-0.025, -0.05, 0.22), (0.025, 0.05, 0.26), 'mat_scooter', St)               # fork crown
    b.box((-0.035, -0.035, 0.24), (0.035, 0.035, 0.32), 'mat_dark', St)                # folding joint
    b.box((-0.05, -0.012, 0.27), (-0.035, 0.012, 0.31), 'mat_dark', St)                # folding latch lever
    b.cyl((0, 0, (0.30 + Ls) / 2), 0.0225, Ls - 0.30, (0, 0, 1), 'mat_scooter', 24, St)  # stem d 45 mm
    b.box((0.0225, -0.045, 0.50), (0.0475, 0.045, 0.68), 'mat_dark', St)               # wireless receiver pad
    b.box((0.0475, -0.03, 0.655), (0.0482, 0.03, 0.665), 'mat_mint_glow', St)          # pad indicator
    b.box((0.0225, -0.025, Ls - 0.11), (0.05, 0.025, Ls - 0.07), 'mat_light', St)      # headlight
    top = St @ Vector((0, 0, Ls))
    b.cyl(top, 0.016, 0.272, (0, 1, 0), 'mat_dark', 16)                                # bar, 0.472 with grips
    for s in (1, -1):
        b.cyl((top.x, s*0.186, top.z), 0.016, 0.10, (0, 1, 0), 'mat_rubber', 16)
        b.box((top.x + 0.01, s*0.11, top.z - 0.012), (top.x + 0.07, s*0.15, top.z - 0.004), 'mat_dark')  # brake levers
    b.box((top.x - 0.035, -0.05, top.z + 0.004), (top.x + 0.035, 0.05, top.z + 0.016), 'mat_dark')      # display head
    b.box((top.x - 0.02, -0.03, top.z + 0.0155), (top.x + 0.02, 0.03, top.z + 0.0162), 'mat_mint_glow')
    b.arc_ring(0.0, 0.0, 0.137, 0.147, -0.035, 0.035, 'mat_scooter', math.radians(30), math.radians(150), 16)       # front fender
    b.arc_ring(-WBASE, 0.0, 0.137, 0.147, -0.04, 0.04, 'mat_scooter', math.radians(25), math.radians(160), 18)    # rear fender
    b.box((-1.038, -0.035, 0.03), (-0.975, 0.035, 0.06), 'mat_scooter')                # fender tail
    b.box((-1.040, -0.025, 0.035), (-1.036, 0.025, 0.055), 'mat_orange')               # tail light
    geo = b.build(name + '_geo', 'Reference')
    emp = bpy.data.objects.new(name, None); emp.empty_display_type = 'ARROWS'; emp.empty_display_size = 0.2
    C['Reference'].objects.link(emp); parent(geo, emp)
    PIV = Vector((-0.30, 0.085, -0.04)); LEN = (Vector((-0.24, 0.12, -0.095)) - PIV).length   # kickstand, front-left of the deck
    b = MB()
    b.cyl((LEN / 2, 0, 0), 0.008, LEN, (1, 0, 0), 'mat_dark', 12)
    b.cyl((LEN, 0, 0), 0.014, 0.012, (1, 0, 0), 'mat_rubber', 16)
    b.cyl((0, 0, 0), 0.011, 0.024, (0, 1, 0), 'mat_metal', 12)
    st = b.build(name + '_stand', 'Reference'); parent(st, emp); st.location = PIV

# ===== station =====
import os
from mathutils import Matrix, Vector, Quaternion
from mathutils.bvhtree import BVHTree
OUTD = os.path.join(OUT, 'kickspot'); os.makedirs(os.path.join(OUTD, 'screenshots'), exist_ok=True)
for n in ['Body', 'Mechanisms', 'Sensors', 'Stage']:
    pass
C['Station'] = bpy.data.collections.new('Station'); sc.collection.children.link(C['Station'])
mat('mat_checker', '#B9BCC0', 0.3, 0.9)

PITCH, NSLOT = 0.62, 4
YS = [(i - (NSLOT - 1) / 2) * PITCH for i in range(NSLOT)]
HB = PITCH * NSLOT / 2                      # half length of the base (1.24)
PLAT = 0.06                                 # platform height; front wheel stands on it, rear wheel on the ground
X0, X1 = -0.30, 0.60                        # platform x range (front axle of a docked scooter at x 0)
PHI = math.asin(PLAT / WBASE)               # scooter pitch (front up)
TH = math.radians(STEM_DEG)
AXLE_Z = PLAT + 0.127

def MS(y):      # stem frame of the scooter docked at slot y: z along the stem, x toward the post
    return Matrix.Translation((0, y, AXLE_Z)) @ Matrix.Rotation(-PHI, 4, 'Y') @ Matrix.Rotation(-TH, 4, 'Y')
def W(x, z):    # stem-frame (x, z) -> world (x, z)
    v = MS(0) @ Vector((x, 0, z)); return (v.x, v.z)

# ---------------- base: steel frame, rubber tiles per slot, checker-plate ramps ----------------
b = MB()
b.box((X0, -HB, 0), (X1, HB, PLAT - 0.01), 'mat_dark')
for y in YS:
    b.box((X0 + 0.01, y - PITCH / 2 + 0.01, PLAT - 0.01), (X1 - 0.01, y + PITCH / 2 - 0.01, PLAT), 'mat_rubber')
    for s in (1, -1):                                       # front-wheel guide rails
        y0, y1 = sorted((y + s * 0.042, y + s * 0.055))
        b.box((X0 + 0.02, y0, PLAT), (0.17, y1, PLAT + 0.03), 'mat_dark')
    b.box((0.155, y - 0.055, PLAT), (0.17, y + 0.055, PLAT + 0.045), 'mat_dark')   # wheel stop
b.build('kickspot_base', 'Station', (0, 0, 0), bevel=0.004)
b = MB()
b.prism_xz([(X0 - 0.22, 0.0), (X0, 0.0), (X0, PLAT)], -HB, HB, 'mat_checker')      # front ramp
a = math.atan2(PLAT, 0.22); Lr = math.hypot(PLAT, 0.22)
for s in (1, -1):                                                                   # end ramps
    M = Matrix.Translation((0, s * HB, PLAT)) @ Matrix.Rotation(-s * a, 4, 'X')
    b.box((X0 - 0.22 if False else X0, 0 if s > 0 else -Lr, -0.004), (X1, Lr if s > 0 else 0, 0.0), 'mat_checker', M)
for i in range(int((2 * HB) / 0.05)):                                               # diamond tread bumps on the front ramp
    for j in range(4):
        x = X0 - 0.20 + j * 0.05 + (0.025 if i % 2 else 0); yb = -HB + 0.025 + i * 0.05
        if x > X0 - 0.01: continue
        z = PLAT * (x - (X0 - 0.22)) / 0.22
        b.box((x - 0.012, yb - 0.004, z - 0.002), (x + 0.012, yb + 0.004, z + 0.002), 'mat_checker',
              Matrix.Rotation(math.radians(45 if (i + j) % 2 else -45), 4, 'Z').to_4x4() if False else None)
b.build('kickspot_ramp', 'Station', (0, 0, 0))

# ---------------- posts: mint panel leaning with the stem, charging face, stem holder, latch ----------------
A = W(0.05, 0.93); B = W(0.05, 0.18); Fp = W(0.20, 0.93)
POST = [A, B, (0.20, B[1]), (0.20, PLAT), (0.55, PLAT), Fp]
for i, y in enumerate(YS, 1):
    pm = MB(); pm.prism_xz(POST, y - 0.13, y + 0.13, 'mat_mint')
    post = pm.build('post_%d' % i, 'Station', (0.30, y, 0.5), bevel=0.008)
    d = MB(); M = MS(y)
    d.box((0.049, -0.085, 0.40), (0.056, 0.085, 0.79), 'mat_dark', M)          # black charging channel
    d.box((0.0485, -0.045, 0.50), (0.056, 0.045, 0.68), 'mat_glass', M)       # wireless transmitter pad (2 mm to the receiver)
    d.box((0.0485, -0.03, 0.72), (0.056, 0.03, 0.73), 'mat_mint_glow', M)     # charge indicator
    d.box((0.05, -0.131, 0.93), (0.20, 0.131, 0.942), 'mat_white', M)         # top cap
    d.box((0.06, -0.10, 0.942), (0.19, 0.10, 0.945), 'mat_light', M)          # status light
    for s in (1, -1):                                                         # white "K" on both sides
        yy = y + s * 0.1305; y0, y1 = sorted((yy, yy + s * 0.002))
        d.box((0.25, y0, 0.24), (0.275, y1, 0.44), 'mat_white')
        for ang, cz in ((-40, 0.385), (40, 0.295)):
            Mk = Matrix.Translation((0.30, 0, cz)) @ Matrix.Rotation(math.radians(ang), 4, 'Y')
            d.box((-0.06, y0, -0.0125), (0.04, y1, 0.0125), 'mat_white', Mk)
    merge_into(post, d.build('_d', 'Station', (0.30, y, 0.5)))
    # stem holder: black U head on the post face, prongs around the stem, rubber stop + mint pads
    h = MB()
    h.box((0.05, -0.065, 0.79), (0.11, 0.065, 0.89), 'mat_dark', M)
    for s in (1, -1):
        y0, y1 = sorted((s * 0.026, s * 0.05)); h.box((-0.05, y0, 0.80), (0.05, y1, 0.88), 'mat_dark', M)
        y0, y1 = sorted((s * 0.0245, s * 0.026)); h.box((-0.04, y0, 0.805), (0.045, y1, 0.875), 'mat_mint', M)
    h.box((0.025, -0.026, 0.805), (0.05, 0.026, 0.875), 'mat_rubber', M)
    h.build('holder_%d' % i, 'Station', tuple(M @ Vector((0.05, 0, 0.84))), bevel=0.003)
    # latch: closes behind the stem (hinged on the right prong tip, axis = stem axis)
    PV = Vector((-0.044, -0.038, 0.84))
    l = MB(); l.box((-0.006, 0.0, -0.03), (0.006, 0.076, 0.03), 'mat_dark')
    lt = l.build('latch_%d' % i, 'Station', (0, 0, 0), bevel=0.002)
    lt.matrix_world = M @ Matrix.Translation(PV)

# ---------------- scooters: 3 docked, slot 4 empty (latch open) ----------------
d0 = (Vector((-0.24, 0.12, -0.095)) - Vector((-0.30, 0.085, -0.04))).normalized()
d1 = (Vector((-0.40, 0.09, -0.035)) - Vector((-0.30, 0.085, -0.04))).normalized()
QFOLD = d0.rotation_difference(d1) @ Vector((1, 0, 0)).rotation_difference(d0)
for i, y in enumerate(YS[:3], 1):
    n = 'scooter_%d' % i; build_scooter(n)
    e = bpy.data.objects[n]; e.location = (0, y, AXLE_Z); e.rotation_euler = (0, -PHI, 0)
    st = bpy.data.objects[n + '_stand']; st.rotation_mode = 'QUATERNION'; st.rotation_quaternion = QFOLD
lt = bpy.data.objects['latch_4']; lt.matrix_world = MS(YS[3]) @ Matrix.Translation(PV) @ Matrix.Rotation(math.radians(95), 4, 'Z')

# ---------------- stage ----------------
b = MB(); b.box((-8, -8, -0.02), (8, 8, 0), 'mat_ground'); b.build('floor', 'Stage', (0, 0, -0.01))
def cam(name, loc, tgt, lens):
    cd = bpy.data.cameras.new(name); cd.lens = lens; o = bpy.data.objects.new(name, cd); C['Stage'].objects.link(o)
    o.location = loc; o.rotation_euler = (Vector(tgt) - Vector(loc)).to_track_quat('-Z', 'Y').to_euler(); return o
cams = [cam('cam_front', (-3.6, -2.4, 1.6), (0.0, 0.0, 0.5), 40), cam('cam_street', (-2.6, 2.2, 1.2), (0.0, -0.3, 0.55), 35),
        cam('cam_holder', (-0.65, YS[3] + 0.45, 1.35), (0.0, YS[3], 0.95), 40)]
ld = bpy.data.lights.new('key_light', 'SUN'); ld.energy = 3.5; ld.angle = math.radians(8)
kl = bpy.data.objects.new('key_light', ld); C['Stage'].objects.link(kl); kl.rotation_euler = (math.radians(50), math.radians(-15), math.radians(-150))
lf = bpy.data.lights.new('fill_light', 'AREA'); lf.energy = 400; lf.size = 4
fl = bpy.data.objects.new('fill_light', lf); C['Stage'].objects.link(fl); fl.location = (-3.0, -3.0, 3.0)
fl.rotation_euler = (Vector((0, 0, 0.4)) - fl.location).to_track_quat('-Z', 'Y').to_euler()
w = sc.world or bpy.data.worlds.new('World'); sc.world = w
bg = next((n for n in w.node_tree.nodes if n.type == 'BACKGROUND'), None) if w.node_tree else None
if bg: bg.inputs[0].default_value = (0.62, 0.65, 0.70, 1); bg.inputs[1].default_value = 0.6

# ---------------- quick check: station parts vs docked scooters, gaps at the holder ----------------
bpy.context.view_layer.update(); dg = bpy.context.evaluated_depsgraph_get()
def bvh(o):
    e = o.evaluated_get(dg); m = e.to_mesh(); m.transform(o.matrix_world)
    t = BVHTree.FromPolygons([v.co.copy() for v in m.vertices], [p.vertices[:] for p in m.polygons]); e.to_mesh_clear(); return t
ST = [o for o in C['Station'].objects if o.type == 'MESH']
hits = []
for i in range(1, 4):
    for p in ('_geo', '_stand'):
        ts = bvh(bpy.data.objects['scooter_%d%s' % (i, p)])
        for o in ST + [bpy.data.objects['floor']]:
            if ts.overlap(bvh(o)): hits.append(('scooter_%d%s' % (i, p), o.name))
# stem clearances in slot 1 (stem frame)
Mi = MS(YS[0]).inverted()
geo = bpy.data.objects['scooter_1_geo']
def low(o):
    e = o.evaluated_get(dg); m = e.to_mesh(); z = min((o.matrix_world @ v.co).z for v in m.vertices); e.to_mesh_clear(); return z
res = {'hits': hits, 'rear_tyre_lowest_z_mm': round(low(geo) * 1000, 2),
       'handlebar_world_z': round((MS(0) @ Vector((0, 0, 1.06 / math.cos(TH)))).z, 3),
       'post_top_world_z': round(max(A[1], Fp[1]) + 0.015, 3), 'pitch_deg': round(math.degrees(PHI), 2)}
sc.camera = cams[0]
sc.render.resolution_x, sc.render.resolution_y = 1600, 1000
for c in cams:
    sc.camera = c; sc.render.filepath = os.path.join(OUTD, 'screenshots', 'kickspot_%s.png' % c.name[4:])
    bpy.ops.render.render(write_still=True)
sc.camera = cams[0]
bpy.ops.wm.save_as_mainfile(filepath=os.path.join(OUTD, 'kickspot.blend'))
print(res)
