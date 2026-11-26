# Per-frame overlap check of scootover_v19_anim.blend.
# Run: Blender -b v19/scootover_v19_anim.blend --python v19/scripts/anim_check_v19.py
import bpy, itertools
from mathutils import Matrix
from mathutils.bvhtree import BVHTree

O = bpy.data.objects; sc = bpy.context.scene
ROBOT = [o.name for cn in ('Body', 'Mechanisms', 'Sensors') for o in bpy.data.collections[cn].objects if o.type == 'MESH']
STATION = [o.name for o in bpy.data.collections['Kickspot'].objects if o.type == 'MESH']   # posts, holders, latches, base, ramp, neighbours
SCOOT = ['ref_scooter_l_geo', 'ref_scooter_l_stand']
SUPPORT = {'floor', 'bay_floor', 'rail_l', 'body_front', 'front_ramp_l', 'front_ramp_l_tip', 'kickspot_base', 'kickspot_ramp',
           'wheel_shuttle_l', 'ramp', 'ramp_tip'}
MOVING = ['front_ramp_l', 'front_ramp_l_tip', 'shuttle_fgate_l_a', 'shuttle_fgate_l_b', 'shuttle_gate_l_a', 'shuttle_gate_l_b',
          'stem_clamp_up_l_a', 'stem_clamp_up_l_b', 'stem_clamp_lo_l_a', 'stem_clamp_lo_l_b', 'dock_holder_l', 'dock_holder_lo_l', 'dock_clamp_mount_l', 'outrigger_l', 'outrigger_r', 'outrigger_l_foot', 'outrigger_r_foot',
          'ramp', 'ramp_tip', 'wheel_shuttle_l', 'arm_base', 'upper_arm', 'forearm', 'wrist', 'finger_l', 'finger_r', 'cam_wrist']
_src = open('/Users/khc/Desktop/2026-2/RobotOS/TermProject_robot_form/v19/scripts/check_v19.py').read()
_g = {'bpy': bpy, 'O': O}
exec(_src[_src.index('OK = {'):_src.index('def trees')], _g); OKP = _g['OK']
for n in MOVING:                                   # parent/child and designed hinge contacts
    p = O[n].parent
    if p is not None and p.type == 'MESH': OKP.add(frozenset((n, p.name)))
OKP |= {frozenset(p) for p in [('wheel_shuttle_l', 'bay_floor'), ('outrigger_l_foot', 'floor'), ('outrigger_r_foot', 'floor'),
        ('ramp_tip', 'floor'), ('arm_base', 'upper_arm'), ('upper_arm', 'forearm'), ('forearm', 'wrist'), ('wrist', 'finger_l'),
        ('wrist', 'finger_r'), ('forearm', 'cam_wrist'), ('wrist', 'cam_wrist')]}
dg = bpy.context.evaluated_depsgraph_get()
LOCAL = {}
for n in set(ROBOT + STATION + SCOOT + ['floor', 'ref_scooter_r_geo', 'ref_scooter_r_stand']):
    e = O[n].evaluated_get(dg); m = e.to_mesh(); LOCAL[n] = ([v.co.copy() for v in m.vertices], [p.vertices[:] for p in m.polygons]); e.to_mesh_clear()
CACHE = {}
LIFT = Matrix.Translation((0, 0, 0.003))   # wheels rest on surfaces (and the 2 mm tread bumps)
def tree(n, lift=False):
    M = O[n].matrix_world.copy()
    if lift: M = (Matrix.Translation((0, 0, 0.006)) if lift == 'k' else LIFT) @ M
    k = (n, lift, tuple(round(x, 6) for r in M for x in r))
    if CACHE.get((n, lift), (None,))[0] != k:
        vs = [M @ v for v in LOCAL[n][0]]
        CACHE[(n, lift)] = (k, BVHTree.FromPolygons(vs, LOCAL[n][1]), [min(v[i] for v in vs) for i in range(3)], [max(v[i] for v in vs) for i in range(3)])
    return CACHE[(n, lift)][1:]
def ov(a, b, la=False):
    A, B = tree(a, la), tree(b)
    if la and b == 'kickspot_ramp':        # tread bumps stand up to 5 mm on the slope
        A = tree(a, 'k')
    if any(A[1][i] > B[2][i] or B[1][i] > A[2][i] for i in range(3)): return False
    return A[0].overlap(B[0]) != []

OTHERS = [n for n in ROBOT + STATION + ['floor', 'ref_scooter_r_geo', 'ref_scooter_r_stand']]
hits = {}
for f in range(sc.frame_start, sc.frame_end + 1):
    sc.frame_set(f)
    for s in SCOOT:
        for n in OTHERS:
            if ov(s, n, la=(n in SUPPORT)): hits.setdefault((s, n), []).append(f)
    for a in MOVING:
        for n in OTHERS + ['ref_scooter_l_geo', 'ref_scooter_l_stand']:
            if a == n or frozenset((a, n)) in OKP or (n in MOVING and n < a): continue
            if n in SCOOT and a in SUPPORT: continue       # wheels rolling on it: covered by the lifted scooter test
            if n == 'floor' and a not in ('front_ramp_l', 'front_ramp_l_tip', 'ramp', 'ramp_tip'): pass
            if ov(a, n): hits.setdefault((a, n), []).append(f)
    if f >= 730:                                                # parked next to the station: whole robot vs the station and neighbours
        for a in ROBOT:
            for n in STATION:
                if ov(a, n): hits.setdefault((a, n), []).append(f)
for (a, b), fs in sorted(hits.items(), key=lambda x: x[1][0]):
    print('HIT %s x %s frames %d..%d (%d)' % (a, b, fs[0], fs[-1], len(fs)))
print('CHECK_DONE', 'NO COLLISIONS' if not hits else '%d pairs' % len(hits))
