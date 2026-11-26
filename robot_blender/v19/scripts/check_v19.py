# Static overlap check for v19 poses + standard renders. Run: Blender -b v19/scootover_v19.blend --python v19/scripts/check_v19.py
import bpy, os, itertools
from mathutils.bvhtree import BVHTree

PROJ = '/Users/khc/Desktop/2026-2/RobotOS/TermProject_robot_form'
O = bpy.data.objects
ns = {}; exec(bpy.data.texts['scootover_pose.py'].as_string(), ns)
ROBOT = [o for cn in ('Body', 'Mechanisms', 'Sensors') for o in bpy.data.collections[cn].objects if o.type == 'MESH']
SC = [O[n] for n in ('ref_scooter_l_geo', 'ref_scooter_l_stand', 'ref_scooter_r_geo', 'ref_scooter_r_stand')]
ALL = ROBOT + SC + [O['floor']]
# pairs that touch by design (mounting faces, hinges, nested telescopes)
OK = {frozenset(p) for p in [('body_front', 'bay_floor'), ('body_front', 'body_side_l'), ('body_front', 'body_side_r'), ('body_front', 'mast'),
      ('body_front', 'cam_front'), ('body_front', 'headlight_l'), ('body_front', 'headlight_r'), ('body_front', 'gantry'), ('mast', 'lidar'),
      ('bay_floor', 'rail_l'), ('bay_floor', 'rail_r'), ('bay_floor', 'body_side_l'), ('bay_floor', 'body_side_r'), ('body_front', 'arch_trim_l'),
      ('body_front', 'arch_trim_r'), ('body_side_l', 'arch_trim_l'), ('body_side_r', 'arch_trim_r'), ('arm_base', 'bay_floor'), ('body_front', 'rail_l'), ('body_front', 'rail_r'),
      ('bay_floor', 'ramp'), ('rail_l', 'ramp'), ('rail_r', 'ramp'), ('ramp', 'ramp_tip'), ('outrigger_l', 'outrigger_l_foot'), ('outrigger_r', 'outrigger_r_foot'),
      ('arm_base', 'upper_arm'), ('upper_arm', 'forearm'), ('forearm', 'wrist'), ('forearm', 'cam_wrist'), ('wrist', 'finger_l'), ('wrist', 'finger_r'),
      ('arm_base', 'wrist'), ('arm_base', 'finger_l'), ('arm_base', 'arm_rail'), ('arm_base', 'body_front'), ('arm_rail', 'bay_floor'), ('arm_rail', 'body_front'), ('mast', 'body_front')]}   # v12 transport fold: wrist rests in the base cradle (unchanged from v12)
for sd in 'lr':
    OK |= {frozenset(p) for p in [('gantry', 'boom_%s_s1' % sd), ('gantry', 'boom_%s_s2' % sd), ('boom_%s_s1' % sd, 'boom_%s_s2' % sd),
           ('boom_%s_trolley' % sd, 'boom_%s_s2' % sd), ('boom_%s_trolley' % sd, 'boom_%s_fork' % sd), ('boom_%s_fork' % sd, 'boom_%s_latch' % sd),
           ('front_ramp_' + sd, 'front_ramp_%s_tip' % sd), ('front_ramp_' + sd, 'body_front'), ('dock_panel_' + sd, 'dock_guide_' + sd),
           ('dock_panel_' + sd, 'dock_holder_' + sd), ('dock_panel_' + sd, 'dock_hanger_' + sd), ('dock_guide_' + sd, 'dock_hanger_' + sd),
           ('dock_holder_' + sd, 'dock_hanger_' + sd), ('wheel_shuttle_' + sd, 'rail_' + sd), ('wheel_shuttle_' + sd, 'shuttle_fgate_%s_a' % sd),
           ('wheel_shuttle_' + sd, 'shuttle_fgate_%s_b' % sd), ('wheel_shuttle_' + sd, 'shuttle_gate_%s_a' % sd), ('wheel_shuttle_' + sd, 'shuttle_gate_%s_b' % sd),
           ('ref_scooter_%s_geo' % sd, 'ref_scooter_%s_stand' % sd), ('wheel_' + sd.replace('l', 'fl').replace('r', 'fr'), 'body_front')]}
    OK |= {frozenset(p) for p in [('grip_housing_' + sd, 'grip_s1_' + sd), ('grip_s1_' + sd, 'grip_s2_' + sd), ('grip_s2_' + sd, 'grip_s3_' + sd),
           ('grip_s3_' + sd, 'grip_head_' + sd), ('dock_side_' + sd, 'body_front'), ('dock_clamp_mount_' + sd, 'dock_side_' + sd), ('dock_holder_' + sd, 'dock_side_' + sd), ('dock_holder_lo_' + sd, 'dock_side_' + sd), ('dock_clamp_mount_' + sd, 'dock_holder_' + sd), ('dock_clamp_mount_' + sd, 'dock_holder_lo_' + sd), ('grip_housing_' + sd, 'grip_head_' + sd), ('grip_housing_' + sd, 'grip_s2_' + sd), ('grip_housing_' + sd, 'grip_s3_' + sd), ('grip_s1_' + sd, 'grip_s3_' + sd), ('grip_head_' + sd, 'grip_finger_' + sd), ('grip_post_' + sd, 'body_side_' + sd),
           ('grip_post_' + sd, 'arch_trim_' + sd)]}
    for k in 'ab':
        for lvl in ('up', 'lo'):
            OK |= {frozenset(('stem_clamp_%s_%s_%s' % (lvl, sd, k), n)) for n in ('dock_holder_' + sd, 'dock_holder_lo_' + sd, 'dock_clamp_mount_' + sd, 'dock_side_' + sd)}


def trees():
    dg = bpy.context.evaluated_depsgraph_get(); T = {}
    for o in ALL:
        e = o.evaluated_get(dg); m = e.to_mesh(); vs = [o.matrix_world @ v.co for v in m.vertices]
        T[o.name] = (BVHTree.FromPolygons(vs, [p.vertices[:] for p in m.polygons]),
                     [min(v[i] for v in vs) for i in range(3)], [max(v[i] for v in vs) for i in range(3)])
        e.to_mesh_clear()
    return T


def hits(skip_floor_for=()):
    T = trees(); out = []
    for a, b in itertools.combinations(T, 2):
        if frozenset((a, b)) in OK: continue
        if 'floor' in (a, b) and not any(n in (a, b) for n in ('ref_scooter_l_geo', 'ref_scooter_l_stand', 'front_ramp_l', 'front_ramp_l_tip', 'front_ramp_r_tip')):
            continue
        A, B = T[a], T[b]
        if any(A[1][i] > B[2][i] or B[1][i] > A[2][i] for i in range(3)): continue
        if A[0].overlap(B[0]): out.append((a, b))
    return out


res = {}
for mode in ('transport', 'loading', 'front_unload'):
    ns['set_pose'](mode); res[mode] = hits()
    print('POSE', mode, res[mode] or 'no overlaps')
ns['set_pose']('front_unload')
fr_tip = max(O['front_ramp_l_tip'].matrix_world @ v.co for v in O['front_ramp_l_tip'].data.vertices)
lowest = min((O['front_ramp_l_tip'].matrix_world @ v.co).z for v in O['front_ramp_l_tip'].data.vertices)
print('FRONT_RAMP tip x %.3f lowest z %.4f' % (max((O['front_ramp_l_tip'].matrix_world @ v.co).x for v in O['front_ramp_l_tip'].data.vertices), lowest))
top = max((o.matrix_world @ v.co).z for o in ROBOT for v in o.data.vertices)
ns['set_pose']('transport')
top_t = max((o.matrix_world @ v.co).z for o in ROBOT for v in o.data.vertices)
print('ROBOT_TOP %.3f' % top_t)
# renders (cameras from v12)
sc = bpy.context.scene; OUT = os.path.join(PROJ, 'v19', 'screenshots')
for cam, mode in (('cam_persp', 'transport'), ('cam_front34', 'transport'), ('cam_dock', 'transport'), ('cam_front34', 'front_unload'), ('cam_side', 'transport')):
    ns['set_pose'](mode); sc.camera = O[cam]; sc.render.resolution_percentage = 50
    sc.render.filepath = os.path.join(OUT, 'render_%s_%s_v19.png' % (cam[4:], mode)); bpy.ops.render.render(write_still=True)
sc.render.resolution_percentage = 100; ns['set_pose']('transport'); sc.camera = O['cam_persp']
bpy.ops.wm.save_mainfile()
