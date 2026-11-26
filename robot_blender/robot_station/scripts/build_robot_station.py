# SCOOTOVER robot station (container garage with roll-up shutter), sized for the SCOOTOVER robot (v19).
# Run: Blender -b --factory-startup --python robot_station/scripts/build_robot_station.py
# -> robot_station/scootover_robot_station.blend + screenshots. Station frame: door faces -X, robot drives in along +X.
import bpy, bmesh, math, os
from mathutils import Matrix, Vector

PROJ = '/Users/khc/Desktop/2026-2/RobotOS/TermProject_robot_form'
OUT = os.path.join(PROJ, 'robot_station'); os.makedirs(os.path.join(OUT, 'screenshots'), exist_ok=True)
ROBOT = os.path.join(PROJ, 'v19', 'scootover_v19.blend')

for o in list(bpy.data.objects): bpy.data.objects.remove(o, do_unlink=True)
for c in list(bpy.data.collections): bpy.data.collections.remove(c)
sc = bpy.context.scene; sc.unit_settings.system = 'METRIC'
C = {}
for n in ('Station', 'Shutter', 'Stage', 'Robot_preview'):
    C[n] = bpy.data.collections.new(n); sc.collection.children.link(C[n])

def lin(h):
    h = h.lstrip('#'); c = [int(h[i:i + 2], 16) / 255 for i in (0, 2, 4)]
    return tuple(x / 12.92 if x <= 0.04045 else ((x + 0.055) / 1.055) ** 2.4 for x in c)
def mat(name, hexc, rough, metal=0.0, emit=0.0):
    m = bpy.data.materials.get(name) or bpy.data.materials.new(name)
    try: m.use_nodes = True
    except Exception: pass
    b = next((n for n in m.node_tree.nodes if n.type == 'BSDF_PRINCIPLED'), None) or m.node_tree.nodes.new('ShaderNodeBsdfPrincipled')
    b.inputs['Base Color'].default_value = (*lin(hexc), 1); b.inputs['Roughness'].default_value = rough; b.inputs['Metallic'].default_value = metal
    if emit:
        b.inputs['Emission Color'].default_value = (*lin(hexc), 1); b.inputs['Emission Strength'].default_value = emit
    out = next((n for n in m.node_tree.nodes if n.type == 'OUTPUT_MATERIAL'), None) or m.node_tree.nodes.new('ShaderNodeOutputMaterial')
    if not out.inputs['Surface'].links: m.node_tree.links.new(b.outputs['BSDF'], out.inputs['Surface'])
    m.diffuse_color = (*lin(hexc), 1)
for args in (('mat_white', '#F2F2F0', 0.35), ('mat_dark', '#2B2E33', 0.5), ('mat_mint', '#3CC8A8', 0.4), ('mat_metal', '#8C9096', 0.35, 0.8),
             ('mat_rubber', '#151515', 0.8), ('mat_light', '#FFFFFF', 0.3, 0.0, 4.0), ('mat_mint_glow', '#3CC8A8', 0.3, 0.0, 3.0),
             ('mat_glass', '#111418', 0.12), ('mat_gray', '#A9AEB3', 0.55, 0.2), ('mat_shutter', '#D9DCDF', 0.4, 0.3),
             ('mat_ground', '#C9CCC8', 0.9), ('mat_floor', '#3A3D42', 0.8), ('mat_checker', '#B9BCC0', 0.3, 0.9)):
    mat(*args)

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
    def cyl(s, c, r, L, axis, m, seg=24, r2=None):
        vs = bmesh.ops.create_cone(s.bm, cap_ends=True, segments=seg, radius1=r, radius2=r if r2 is None else r2, depth=L)['verts']
        M = Matrix.Translation(c) @ Vector((0, 0, 1)).rotation_difference(Vector(axis)).to_matrix().to_4x4()
        bmesh.ops.transform(s.bm, matrix=M, verts=vs); s._tag(vs, m)
    def sphere(s, c, r, m):
        vs = bmesh.ops.create_uvsphere(s.bm, u_segments=20, v_segments=10, radius=r, matrix=Matrix.Translation(c))['verts']; s._tag(vs, m)
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

# ---------------- dimensions (robot: 1.94 x 1.10 x 1.20 m) ----------------
X0, X1 = 0.0, 2.60            # front face (door) .. back face
HY, H = 0.86, 2.00            # half width, height
FL = 0.10                     # inner floor height
OY, OZ = 0.70, 1.70           # door opening half width, top
HB = 1.52                     # shutter housing bottom -> clear height 1.42
T = 0.08                      # frame tube

# ---------------- mint frame ----------------
b = MB()
for x0 in (X0, X1 - T):
    for w in (1, -1):
        y0, y1 = sorted((w * HY, w * (HY - T))); b.box((x0, y0, 0), (x0 + T, y1, H), 'mat_mint')                 # corner posts
    b.box((x0, -HY, H - T), (x0 + T, HY, H), 'mat_mint'); b.box((x0, -HY, 0), (x0 + T, HY, FL), 'mat_mint')       # top / bottom cross
for w in (1, -1):
    y0, y1 = sorted((w * HY, w * (HY - T)))
    b.box((X0, y0, H - T), (X1, y1, H), 'mat_mint'); b.box((X0, y0, 0), (X1, y1, FL), 'mat_mint')                # side rails
b.box((X0 - 0.02, -HY, OZ), (X0 + T, HY, OZ + 0.06), 'mat_mint')                                                  # lintel
for w in (1, -1):
    y0, y1 = sorted((w * OY, w * HY)); b.box((X0 - 0.02, y0, 0), (X0 + T, y1, H), 'mat_mint')                     # door jambs (wide front frame)
b.build('station_frame', 'Station', (X1 / 2, 0, H / 2), bevel=0.006)

# ---------------- grey corrugated walls, white roof, inner floor ----------------
b = MB()
for w in (1, -1):
    yw = w * (HY - 0.03); y0, y1 = sorted((yw, yw - w * 0.02))
    b.box((X0 + T, y0, FL), (X1 - T, y1, H - T), 'mat_gray')
    yr0, yr1 = sorted((w * (HY - 0.01), w * (HY - 0.03)))
    for i in range(int((X1 - 2 * T) / 0.16)):
        x = X0 + T + 0.06 + i * 0.16; b.box((x, yr0, FL + 0.03), (x + 0.04, yr1, H - T - 0.03), 'mat_gray')      # vertical ribs
b.box((X1 - T - 0.02, -HY + T, FL), (X1 - T, HY - T, H - T), 'mat_gray')                                          # back wall
for i in range(int((2 * HY - 2 * T) / 0.16)):
    y = -HY + T + 0.06 + i * 0.16; b.box((X1 - T, y, FL + 0.03), (X1 - 0.01, y + 0.04, H - T - 0.03), 'mat_gray')
b.box((X0 + 0.02, -HY - 0.02, H), (X1 + 0.02, HY + 0.02, H + 0.04), 'mat_white')                                  # roof (white, small overhang)
b.box((X0 + T, -HY + T, FL - 0.02), (X1 - T, HY - T, FL), 'mat_floor')                                            # inner floor
for w in (1, -1):                                                                                                  # mint guide lines for the wheels
    y0, y1 = sorted((w * 0.475 - 0.02, w * 0.475 + 0.02)); b.box((X0 + T + 0.05, y0, FL), (X1 - T - 0.05, y1, FL + 0.002), 'mat_mint')
b.box((X0 + T, -HY + T, H - T - 0.02), (X1 - T, HY - T, H - T), 'mat_white')                                      # ceiling
b.box((X0 + 0.4, -0.05, H - T - 0.035), (X1 - 0.4, 0.05, H - T - 0.02), 'mat_light')                              # ceiling light strip
b.build('station_shell', 'Station', (X1 / 2, 0, H / 2), bevel=0.003)

# ---------------- front: sign band, sign, beacon, control box + conduit, entry ramp ----------------
b = MB()
b.box((X0 - 0.03, -0.25, OZ + 0.08), (X0 - 0.005, 0.62, H - T - 0.02), 'mat_white')                               # sign plate
b.box((X0 - 0.032, -0.25, OZ + 0.08), (X0 - 0.03, 0.62, OZ + 0.095), 'mat_mint_glow')                             # light line under the sign
b.box((X0 - 0.12, -0.70, H - T - 0.26), (X0 - 0.02, -0.45, H - T - 0.02), 'mat_white')                              # control box (top right)
b.box((X0 - 0.122, -0.67, H - T - 0.20), (X0 - 0.118, -0.48, H - T - 0.06), 'mat_gray')                             # box door
b.cyl((X0 - 0.06, -0.34, OZ + 0.24), 0.012, 0.22, (0, 1, 0), 'mat_white', 12)                                       # conduit to the sign
b.cyl((X0 - 0.06, -0.78, OZ + 0.16), 0.07, 0.04, (1, 0, 0), 'mat_dark', 24)                                         # beacon base (front, right)
b.sphere((X0 - 0.10, -0.78, OZ + 0.16), 0.06, 'mat_mint_glow')                                                      # beacon dome
b.prism_xz([(X0 - 0.45, 0.0), (X0, 0.0), (X0, FL), (X0 - 0.02, FL)], -OY, OY, 'mat_checker')                      # entry ramp
b.build('station_front', 'Station', (X0, 0, 1.0), bevel=0.003)
# "SCOOTOVER" lettering
cu = bpy.data.curves.new('sign_text', 'FONT'); cu.body = 'SCOOTOVER'; cu.size = 0.15; cu.extrude = 0.004
cu.align_x = 'CENTER'; cu.align_y = 'CENTER'
to = bpy.data.objects.new('sign_text', cu); C['Station'].objects.link(to)
to.rotation_euler = (math.pi / 2, 0, -math.pi / 2); to.location = (X0 - 0.036, 0.185, (OZ + 0.08 + H - T - 0.02) / 2)
mat('mat_mint_deep', '#14876C', 0.4); to.data.materials.append(bpy.data.materials['mat_mint_deep'])
bpy.context.view_layer.update(); dg = bpy.context.evaluated_depsgraph_get()
me = bpy.data.meshes.new_from_object(to.evaluated_get(dg)); tm = bpy.data.objects.new('station_sign_text', me); C['Station'].objects.link(tm)
tm.matrix_world = to.matrix_world.copy(); bpy.data.objects.remove(to, do_unlink=True)
if not me.materials: me.materials.append(bpy.data.materials['mat_mint_deep'])

# ---------------- side door + electrical cabinet (right side, -Y), back-wall charger and AR markers ----------------
b = MB()
yd = -HY - 0.004
b.box((1.20, yd - 0.02, FL + 0.06), (2.30, yd, H - T - 0.08), 'mat_gray')                                         # side door leaf
b.box((1.20, yd - 0.025, H - T - 0.30), (2.30, yd - 0.02, H - T - 0.26), 'mat_dark')                              # vent slot
b.box((2.12, yd - 0.06, 0.95), (2.16, yd - 0.02, 1.05), 'mat_metal')                                               # handle
b.box((2.25, -HY - 0.30, 0.10), (2.55, -HY - 0.02, 0.62), 'mat_metal')                                             # electrical cabinet
b.box((2.27, -HY - 0.302, 0.14), (2.53, -HY - 0.298, 0.58), 'mat_gray')
b.box((X1 - T - 0.06, -0.32, FL), (X1 - T - 0.02, 0.32, 0.62), 'mat_white')                                       # wall charger body
b.box((X1 - T - 0.065, -0.25, 0.30), (X1 - T - 0.06, 0.25, 0.36), 'mat_metal')                                    # contact plates (robot bumper height)
b.box((X1 - T - 0.065, -0.20, 0.50), (X1 - T - 0.06, 0.20, 0.52), 'mat_mint_glow')                                # charge indicator
b.box((X1 - T - 0.03, -0.70, 0.12), (X1 - T - 0.025, -0.38, 0.30), 'mat_dark')                                    # vent grille
for i in range(5):
    b.box((X1 - T - 0.032, -0.70, 0.135 + i * 0.033), (X1 - T - 0.028, -0.38, 0.145 + i * 0.033), 'mat_gray')
b.box((X1 - T - 0.025, 0.35, 1.05), (X1 - T - 0.02, 0.75, 1.45), 'mat_white')                                     # AR marker board
for i, j, sz in ((0, 0, 0.16), (1, 0, 0.16), (0, 1, 0.16), (1, 1, 0.16)):
    yc, zc = 0.45 + i * 0.20, 1.15 + j * 0.20
    b.box((X1 - T - 0.027, yc - sz / 2, zc - sz / 2), (X1 - T - 0.025, yc + sz / 2, zc + sz / 2), 'mat_dark')
    for (a, c2) in ((-0.04, -0.04), (0.03, 0.02), (-0.02, 0.04), (0.04, -0.03)):
        b.box((X1 - T - 0.0275, yc + a - 0.02, zc + c2 - 0.02), (X1 - T - 0.0265, yc + a + 0.02, zc + c2 + 0.02), 'mat_white')
b.build('station_fittings', 'Station', (1.8, 0, 0.6), bevel=0.002)

# ---------------- roll-up shutter: housing + slat curtain with an 'open' shape key ----------------
b = MB()
b.box((X0 + 0.02, -OY, HB), (X0 + 0.26, OY, OZ), 'mat_white')                                                     # housing (behind the lintel)
for w in (1, -1):
    y0, y1 = sorted((w * (OY - 0.04), w * OY)); b.box((X0 + 0.10, y0, FL), (X0 + 0.14, y1, HB), 'mat_metal')       # side guides
b.build('shutter_housing', 'Shutter', (X0 + 0.14, 0, HB), bevel=0.003)
bm = bmesh.new(); N = 28; hs = (HB - FL) / N
for i in range(N):
    z0 = FL + i * hs
    vs = bmesh.ops.create_cube(bm, size=1.0)['verts']
    bmesh.ops.transform(bm, matrix=Matrix.Translation((X0 + 0.12, 0, z0 + hs / 2)) @ Matrix.Diagonal((0.018, 2 * (OY - 0.035), hs * 0.92, 1)), verts=vs)
me = bpy.data.meshes.new('shutter_curtain'); bm.to_mesh(me); bm.free(); me.materials.append(bpy.data.materials['mat_shutter'])
cur = bpy.data.objects.new('shutter_curtain', me); C['Shutter'].objects.link(cur)
cur.shape_key_add(name='Basis'); kb = cur.shape_key_add(name='open')
for i, v in enumerate(me.vertices):                       # rolled up: slats packed into the housing
    z = v.co.z; kb.data[i].co = Vector((X0 + 0.14 + (v.co.x - (X0 + 0.12)) * 4, v.co.y, HB + 0.02 + (z - FL) * (0.14 / (HB - FL))))
cur.data.shape_keys.key_blocks['open'].value = 0.0

# ---------------- stage ----------------
b = MB(); b.box((-6, -6, -0.02), (8, 6, 0), 'mat_ground'); b.build('floor', 'Stage', (0, 0, -0.01))
def cam(name, loc, tgt, lens):
    cd = bpy.data.cameras.new(name); cd.lens = lens; o = bpy.data.objects.new(name, cd); C['Stage'].objects.link(o)
    o.location = loc; o.rotation_euler = (Vector(tgt) - Vector(loc)).to_track_quat('-Z', 'Y').to_euler(); return o
cams = {'front': cam('cam_front', (-3.4, -2.3, 1.7), (1.0, 0.0, 0.9), 35), 'open': cam('cam_open', (-3.0, 1.8, 1.4), (1.2, 0.0, 0.8), 35),
        'side': cam('cam_side', (1.6, -4.2, 1.5), (1.3, 0.0, 0.9), 35)}
ld = bpy.data.lights.new('sun', 'SUN'); ld.energy = 3.5; ld.angle = math.radians(8)
sl = bpy.data.objects.new('sun', ld); C['Stage'].objects.link(sl); sl.rotation_euler = (math.radians(50), math.radians(-15), math.radians(-120))
lf = bpy.data.lights.new('fill', 'AREA'); lf.energy = 500; lf.size = 4
fl = bpy.data.objects.new('fill', lf); C['Stage'].objects.link(fl); fl.location = (-3.5, -2.5, 3.0)
fl.rotation_euler = (Vector((1, 0, 0.8)) - fl.location).to_track_quat('-Z', 'Y').to_euler()
w = sc.world or bpy.data.worlds.new('World'); sc.world = w
try: w.use_nodes = True
except Exception: pass
bg = next((n for n in w.node_tree.nodes if n.type == 'BACKGROUND'), None) if w.node_tree else None
if bg: bg.inputs[0].default_value = (0.62, 0.65, 0.70, 1); bg.inputs[1].default_value = 0.6

# ---------------- robot preview (v19 robot, transport pose) parked inside ----------------
with bpy.data.libraries.load(ROBOT, link=False) as (src, dst):
    dst.objects = [n for n in src.objects if not n.startswith(('cam_', 'floor', 'key_light', 'fill_light'))]
for o in dst.objects:
    if o is None: continue
    C['Robot_preview'].objects.link(o)
    if o.type == 'MESH':
        for s in o.material_slots:
            if s.material and '.' in s.material.name and s.material.name.split('.')[0] in bpy.data.materials:
                s.material = bpy.data.materials[s.material.name.split('.')[0]]
pv = bpy.data.objects.new('robot_preview', None); C['Robot_preview'].objects.link(pv); pv.location = (1.30, 0, FL)
for o in dst.objects:
    if o is not None and o.parent is None: o.parent = pv; o.matrix_parent_inverse = Matrix.Identity(4)

# ---------------- clearance check + renders ----------------
bpy.context.view_layer.update()
robot = [o for o in C['Robot_preview'].objects if o.type == 'MESH']
mn = [min((o.matrix_world @ v.co)[i] for o in robot for v in o.data.vertices) for i in range(3)]
mx = [max((o.matrix_world @ v.co)[i] for o in robot for v in o.data.vertices) for i in range(3)]
print('ROBOT_BOX', [round(v, 3) for v in mn], [round(v, 3) for v in mx], 'door clear: y %.3f..%.3f  z top %.3f' % (-OY + 0.035, OY - 0.035, HB))
sc.render.resolution_x, sc.render.resolution_y, sc.render.resolution_percentage = 1600, 1000, 60
shots = [('front', 0.0, False), ('open', 1.0, True), ('side', 1.0, True)]
for name, sk, show_robot in shots:
    cur.data.shape_keys.key_blocks['open'].value = sk
    for o in C['Robot_preview'].objects: o.hide_render = not show_robot
    sc.camera = cams[name]; sc.render.filepath = os.path.join(OUT, 'screenshots', 'robot_station_%s.png' % name)
    bpy.ops.render.render(write_still=True)
cur.data.shape_keys.key_blocks['open'].value = 1.0
for o in C['Robot_preview'].objects: o.hide_render = False
sc.camera = cams['open']
bpy.ops.wm.save_as_mainfile(filepath=os.path.join(OUT, 'scootover_robot_station.blend'))
print('BUILD_OK')
