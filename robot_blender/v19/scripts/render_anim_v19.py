# Render the front-unloading sequence of scootover_v19_anim.blend -> v15/screenshots/frontunload_*.png + sheet
import bpy, os
from mathutils import Vector
PROJ = '/Users/khc/Desktop/2026-2/RobotOS/TermProject_robot_form'; OUT = os.path.join(PROJ, 'v19', 'screenshots')
sc = bpy.context.scene
cd = bpy.data.cameras.new('_cam_fu'); cd.lens = 28; cam = bpy.data.objects.new('_cam_fu', cd); sc.collection.objects.link(cam)
cam.location = (0.5, -3.6, 2.2); cam.rotation_euler = (Vector((0.9, 0.2, 0.5)) - cam.location).to_track_quat('-Z', 'Y').to_euler()
sc.camera = cam; sc.render.resolution_percentage = 50
FR = [700, 835, 985, 1080, 1160, 1240, 1300, 1325, 1400, 1520, 1600, 1690]
for f in FR:
    sc.frame_set(f); sc.render.filepath = os.path.join(OUT, 'frontunload_%04d.png' % f); bpy.ops.render.render(write_still=True)
cam.location = (1.2, -1.0, 1.3); cam.rotation_euler = (Vector((1.75, 0.2, 0.6)) - cam.location).to_track_quat('-Z', 'Y').to_euler(); cd.lens = 45
for f in (1100, 1325):
    sc.frame_set(f); sc.render.filepath = os.path.join(OUT, 'frontunload_grip_%04d.png' % f); bpy.ops.render.render(write_still=True)
print('RENDER_OK')
