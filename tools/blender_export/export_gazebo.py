"""SCOOTOVER v12 / Kickspot -> Gazebo 이관용 내보내기 (Blender 5.x, 화면 없이 실행).

맥에서 실행 (이 파일이 robot_blender/ 바로 아래에 있어야 함):
    /Applications/Blender.app/Contents/MacOS/Blender -b --python <경로>/robot_blender/export_gazebo.py

다른 위치에 두었으면 robot_blender 폴더를 인자로 준다:
    ... -b --python export_gazebo.py -- /path/to/robot_blender

결과: robot_blender/export/
  robot/      가장 높은 vNN 폴더의 scootover_vNN.blend: inventory.json (오브젝트·부모·행렬·재질·모디파이어),
              poses.json (자세 스크립트가 있으면), meshes/*.obj+.mtl
  <동작>/     같은 폴더의 scootover_vNN_<동작>.blend (예: anim, righting): inventory.json, tracks.json (프레임별 pose),
              meshes/ (정지 모델에 없는 오브젝트만)
  kickspot/   kickspot/kickspot.blend: inventory.json, meshes/*.obj+.mtl  (kickspot_<이름>.blend 는 동작으로)
  파일을 추가·이름 변경해도 위 규칙만 지키면 스크립트를 고칠 필요 없음
  materials.json, export_log.txt

메시는 모디파이어를 적용한 결과(evaluated)를 오브젝트 로컬 좌표 그대로 쓴다 (+X 앞, +Y 왼쪽, +Z 위, m).
축 변환 없음. 원점 = 오브젝트 원점(= 관절 피벗). 월드 배치는 inventory.json 의 행렬로 한다.
"""
import json
import math
import os
import sys
import time
import traceback

import bpy

# ---------------- 경로 ----------------
argv = sys.argv[sys.argv.index('--') + 1:] if '--' in sys.argv else []
if argv:
    ROOT = os.path.abspath(argv[0])
else:
    ROOT = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(ROOT, 'export')
if os.path.isdir(OUT):                # 예전 결과(다른 버전 폴더)가 섞이지 않게 매번 새로 만든다
    import shutil
    shutil.rmtree(OUT)
os.makedirs(OUT, exist_ok=True)

def discover():
    """가장 높은 vNN 폴더의 scootover_vNN.blend = 정지 모델(full), 같은 폴더의 scootover_vNN_<이름>.blend = 동작(track).
    kickspot/ 의 kickspot.blend = full, kickspot_<이름>.blend = track.  키 = 'robot', 동작 이름, 'kickspot', 'kickspot_<이름>'"""
    files = {}
    vers = sorted((d for d in os.listdir(ROOT) if d.startswith('v') and d[1:].isdigit()
                   and os.path.isdir(os.path.join(ROOT, d))), key=lambda d: int(d[1:]))
    if vers:
        v = vers[-1]
        base = f'scootover_{v}'
        for fn in sorted(os.listdir(os.path.join(ROOT, v))):
            if not fn.endswith('.blend') or not fn.startswith(base):
                continue
            stem = fn[:-6]
            if stem == base:
                files['robot'] = (f'{v}/{fn}', 'full')
            else:
                files[stem[len(base) + 1:]] = (f'{v}/{fn}', 'track')
    kd = os.path.join(ROOT, 'kickspot')
    if os.path.isdir(kd):
        for fn in sorted(os.listdir(kd)):
            if fn.endswith('.blend') and fn.startswith('kickspot'):
                stem = fn[:-6]
                files[stem] = (f'kickspot/{fn}', 'full' if stem == 'kickspot' else 'track')
    sd = os.path.join(ROOT, 'robot_station')
    if os.path.isdir(sd):
        for fn in sorted(os.listdir(sd)):
            if fn.endswith('.blend') and fn.startswith('scootover_robot_station'):
                stem = fn[:-6]
                key = 'robot_station' if stem == 'scootover_robot_station' else 'robot_station_' + stem[24:]
                files[key] = (f'robot_station/{fn}', 'full' if key == 'robot_station' else 'track')
    # 정지 모델을 먼저 (동작 파일은 정지 모델에 없는 오브젝트만 메시로 내보냄)
    return dict(sorted(files.items(), key=lambda kv: (kv[1][1] != 'full', kv[0])))


FILES = discover()
SKIP_COLLECTIONS = {'Stage', 'Robot_preview'}   # 카메라·조명·바닥, 충전소 안 로봇 미리보기
SKIP_NAMES = {'floor'}
ND = 6                                # 소수 자리

LOG = []
MATERIALS = {}


def log(*a):
    s = ' '.join(str(x) for x in a)
    print('[export]', s)
    LOG.append(s)


def r(v, n=ND):
    return round(float(v), n)


def mat4(m):
    return [[r(m[i][j]) for j in range(4)] for i in range(4)]


def lin2srgb(c):
    c = max(0.0, min(1.0, float(c)))
    return c * 12.92 if c <= 0.0031308 else 1.055 * c ** (1 / 2.4) - 0.055


def pose_of(m):
    t, q, s = m.decompose()
    e = q.to_euler('XYZ')            # URDF/SDF rpy = 고정축 X->Y->Z = Blender 'XYZ' 오일러
    return {'xyz': [r(v) for v in t], 'quat_wxyz': [r(v) for v in q], 'rpy': [r(v) for v in e],
            'scale': [r(v) for v in s]}


# ---------------- 재질 ----------------
def material_info(mat):
    if mat is None:
        return None
    if mat.name in MATERIALS:
        return MATERIALS[mat.name]
    info = {'name': mat.name, 'diffuse_color_viewport': [r(v, 4) for v in mat.diffuse_color]}
    base, src = None, 'viewport'
    try:
        if mat.use_nodes and mat.node_tree:
            for n in mat.node_tree.nodes:
                if n.type == 'BSDF_PRINCIPLED':
                    inp = n.inputs['Base Color']
                    if inp.is_linked:
                        ln = inp.links[0].from_node
                        info['base_color_linked_from'] = ln.type
                        if ln.type == 'RGB':
                            base, src = list(ln.outputs[0].default_value), 'rgb_node'
                        elif ln.type == 'TEX_CHECKER':
                            c1 = list(ln.inputs['Color1'].default_value)
                            c2 = list(ln.inputs['Color2'].default_value)
                            base = [(a + b) / 2 for a, b in zip(c1, c2)]
                            src = 'checker_avg'
                            info['checker'] = {'color1': [r(v, 4) for v in c1], 'color2': [r(v, 4) for v in c2],
                                               'scale': r(ln.inputs['Scale'].default_value, 4)}
                    else:
                        base, src = list(inp.default_value), 'principled'
                    for k in ('Metallic', 'Roughness', 'Alpha'):
                        if k in n.inputs:
                            info[k.lower()] = r(n.inputs[k].default_value, 4)
                    for k in ('Emission Color', 'Emission'):
                        if k in n.inputs and not n.inputs[k].is_linked:
                            info['emission_linear'] = [r(v, 4) for v in n.inputs[k].default_value][:3]
                            break
                    if 'Emission Strength' in n.inputs:
                        info['emission_strength'] = r(n.inputs['Emission Strength'].default_value, 4)
                    break
    except Exception as e:  # noqa: BLE001
        info['node_error'] = str(e)
    if base is None:
        base = list(mat.diffuse_color)
    info['base_color_source'] = src
    info['base_linear'] = [r(v, 4) for v in base[:3]]
    info['base_srgb'] = [r(lin2srgb(v), 4) for v in base[:3]]
    info['hex_srgb'] = '#' + ''.join(f'{round(lin2srgb(v) * 255):02X}' for v in base[:3])
    MATERIALS[mat.name] = info
    return info


# ---------------- 메시 ----------------
def safe(name):
    return ''.join(c if c.isalnum() or c in '_-.' else '_' for c in name)


def export_obj(ob, mesh_dir):
    """평가된(모디파이어 적용) 메시를 오브젝트 로컬 좌표로 OBJ+MTL 로 쓴다. -> 통계 dict"""
    dg = bpy.context.evaluated_depsgraph_get()
    ob_eval = ob.evaluated_get(dg)
    me = ob_eval.to_mesh()
    try:
        me.calc_loop_triangles()
        verts = [v.co for v in me.vertices]
        if hasattr(me, 'corner_normals'):
            cn = [c.vector for c in me.corner_normals]
        else:                                    # 4.0 이하
            me.calc_normals_split()
            cn = [lp.normal for lp in me.loops]
        slots = [s.material for s in ob.material_slots]
        base = safe(ob.name)
        obj_path = os.path.join(mesh_dir, base + '.obj')
        mtl_path = os.path.join(mesh_dir, base + '.mtl')
        nidx, nlist = {}, []
        by_mat = {}
        for tri in me.loop_triangles:
            by_mat.setdefault(tri.material_index, []).append(tri)
        lines = [f'# {ob.name} (Blender {bpy.app.version_string}) local coords, +X fwd +Y left +Z up, meters',
                 f'mtllib {base}.mtl', f'o {base}']
        for v in verts:
            lines.append(f'v {v.x:.6f} {v.y:.6f} {v.z:.6f}')
        face_lines = []
        used_mats = []
        for mi, tris in sorted(by_mat.items()):
            mat = slots[mi] if mi < len(slots) else None
            mname = safe(mat.name) if mat else 'default'
            used_mats.append((mname, mat))
            face_lines.append(f'usemtl {mname}')
            for tri in tris:
                idx = []
                for k in range(3):
                    li = tri.loops[k]
                    n = cn[li]
                    key = (round(n.x, 4), round(n.y, 4), round(n.z, 4))
                    if key not in nidx:
                        nidx[key] = len(nlist) + 1
                        nlist.append(key)
                    idx.append(f'{tri.vertices[k] + 1}//{nidx[key]}')
                face_lines.append('f ' + ' '.join(idx))
        for n in nlist:
            lines.append(f'vn {n[0]:.4f} {n[1]:.4f} {n[2]:.4f}')
        lines += face_lines
        with open(obj_path, 'w') as f:
            f.write('\n'.join(lines) + '\n')
        ml = []
        seen = set()
        for mname, mat in used_mats:
            if mname in seen:
                continue
            seen.add(mname)
            info = material_info(mat)
            c = info['base_srgb'] if info else [0.7, 0.7, 0.7]
            ml += [f'newmtl {mname}', f'Ka {c[0]:.4f} {c[1]:.4f} {c[2]:.4f}',
                   f'Kd {c[0]:.4f} {c[1]:.4f} {c[2]:.4f}', 'Ks 0.1000 0.1000 0.1000', 'Ns 20', 'd 1.0', 'illum 2', '']
            if info and info.get('emission_strength', 0) > 0 and info.get('emission_linear'):
                e = [lin2srgb(v) for v in info['emission_linear']]
                ml.insert(-1, f'Ke {e[0]:.4f} {e[1]:.4f} {e[2]:.4f}')
        with open(mtl_path, 'w') as f:
            f.write('\n'.join(ml) + '\n')
        xs = [v.x for v in verts] or [0]
        ys = [v.y for v in verts] or [0]
        zs = [v.z for v in verts] or [0]
        return {'file': os.path.relpath(obj_path, OUT), 'verts': len(verts), 'tris': len(me.loop_triangles),
                'materials': [m for m, _ in used_mats],
                'bbox_local': [[r(min(xs)), r(min(ys)), r(min(zs))], [r(max(xs)), r(max(ys)), r(max(zs))]]}
    finally:
        ob_eval.to_mesh_clear()


def exportable(ob):
    if ob.name in SKIP_NAMES:
        return False
    if any(c.name in SKIP_COLLECTIONS for c in ob.users_collection):
        return False
    return ob.type in {'MESH', 'FONT', 'CURVE', 'SURFACE', 'META'}


# ---------------- 인벤토리 ----------------
def inventory():
    objs = {}
    for ob in bpy.data.objects:
        mw = ob.matrix_world
        rel = (ob.parent.matrix_world.inverted() @ mw) if ob.parent else mw
        d = {
            'type': ob.type,
            'parent': ob.parent.name if ob.parent else None,
            'parent_type': ob.parent_type if ob.parent else None,
            'parent_bone': ob.parent_bone or None,
            'collections': [c.name for c in ob.users_collection],
            'matrix_world': mat4(mw),
            'matrix_parent_rel': mat4(rel),
            'world': pose_of(mw),
            'parent_rel': pose_of(rel),
            'dimensions': [r(v) for v in ob.dimensions],
            'hide_render': bool(ob.hide_render),
            'hide_viewport': bool(ob.hide_viewport),
            'materials': [s.material.name if s.material else None for s in ob.material_slots],
            'modifiers': [{'name': m.name, 'type': m.type,
                           'show_render': bool(m.show_render)} for m in ob.modifiers],
            'constraints': [{'name': c.name, 'type': c.type,
                             'target': getattr(getattr(c, 'target', None), 'name', None)} for c in ob.constraints],
            'rotation_mode': ob.rotation_mode,
            'has_animation': bool(ob.animation_data and ob.animation_data.action),
            'custom_props': {k: (v if isinstance(v, (int, float, str)) else str(v))
                             for k, v in ob.items() if not k.startswith('_')},
        }
        sk = getattr(getattr(ob, 'data', None), 'shape_keys', None)
        if sk:
            d['shape_keys'] = {kb.name: r(kb.value, 4) for kb in sk.key_blocks}
        if ob.type == 'EMPTY':
            d['empty_display'] = ob.empty_display_type
            if ob.instance_type == 'COLLECTION' and ob.instance_collection:
                d['instance_collection'] = ob.instance_collection.name
        if ob.type == 'CAMERA':
            d['lens_mm'] = r(ob.data.lens, 3)
            d['sensor_width_mm'] = r(ob.data.sensor_width, 3)
            d['angle_rad'] = r(ob.data.angle, 5)
        for s in ob.material_slots:
            material_info(s.material)
        objs[ob.name] = d
    sc = bpy.context.scene
    colls = {c.name: {'objects': [o.name for o in c.objects], 'children': [ch.name for ch in c.children]}
             for c in bpy.data.collections}
    texts = {t.name: len(t.as_string()) for t in bpy.data.texts}
    return {
        'blender': bpy.app.version_string,
        'file': bpy.data.filepath,
        'scene': {'name': sc.name, 'frame_start': sc.frame_start, 'frame_end': sc.frame_end,
                  'frame_current': sc.frame_current, 'fps': sc.render.fps / sc.render.fps_base,
                  'unit_scale': r(sc.unit_settings.scale_length),
                  'markers': sorted([{'frame': m.frame, 'name': m.name} for m in sc.timeline_markers],
                                    key=lambda x: x['frame'])},
        'collections': colls,
        'texts': texts,
        'actions': [{'name': a.name, 'frame_range': [r(v, 2) for v in a.frame_range]} for a in bpy.data.actions],
        'objects': objs,
    }


def dump(path, data):
    with open(path, 'w') as f:
        json.dump(data, f, ensure_ascii=False, indent=1)


def export_meshes(out_dir, only=None):
    mesh_dir = os.path.join(out_dir, 'meshes')
    os.makedirs(mesh_dir, exist_ok=True)
    stats, n_fail = {}, 0
    for ob in bpy.data.objects:
        if not exportable(ob) or (only is not None and ob.name not in only):
            continue
        try:
            stats[ob.name] = export_obj(ob, mesh_dir)
            sk = getattr(ob.data, 'shape_keys', None) if ob.type == 'MESH' else None
            if sk:                                   # 셰이프 키마다 값 1 (나머지 0) 메시도: <이름>__<키>.obj
                kbs = list(sk.key_blocks)[1:]
                saved = {kb.name: kb.value for kb in kbs}
                orig_name = ob.name
                for kb in kbs:
                    for o in kbs:
                        o.value = 1.0 if o is kb else 0.0
                    bpy.context.view_layer.update()
                    ob.name = f'{orig_name}__{kb.name}'
                    try:
                        stats[ob.name] = export_obj(ob, mesh_dir)
                    finally:
                        ob.name = orig_name
                for o in kbs:
                    o.value = saved[o.name]
                bpy.context.view_layer.update()
        except Exception as e:  # noqa: BLE001
            n_fail += 1
            stats[ob.name] = {'error': str(e)}
            log(f'  메시 실패 {ob.name}: {e}')
    log(f'  메시 {len(stats) - n_fail}개 ({n_fail}개 실패) -> {os.path.relpath(mesh_dir, OUT)}')
    return stats


def save_texts(out_dir):
    tdir = os.path.join(out_dir, 'texts')
    for t in bpy.data.texts:
        os.makedirs(tdir, exist_ok=True)
        with open(os.path.join(tdir, safe(t.name)), 'w') as f:
            f.write(t.as_string())


def world_poses():
    return {ob.name: {'matrix_world': mat4(ob.matrix_world), 'world': pose_of(ob.matrix_world),
                      'parent_rel': pose_of((ob.parent.matrix_world.inverted() @ ob.matrix_world)
                                            if ob.parent else ob.matrix_world)}
            for ob in bpy.data.objects if not any(c.name in SKIP_COLLECTIONS for c in ob.users_collection)}


def try_pose_script(out_dir, blend_rel):
    """자세 스크립트(.blend 안 텍스트 또는 vNN/scripts/pose_vNN.py)의 set_pose 자세를 기록 (있으면)"""
    txt = next((t for t in bpy.data.texts if 'pose' in t.name.lower()), None)
    src, name = (txt.as_string(), txt.name) if txt else (None, None)
    if src is None:
        sd = os.path.join(ROOT, os.path.dirname(blend_rel), 'scripts')
        cands = sorted(f for f in os.listdir(sd) if f.startswith('pose') and f.endswith('.py')) if os.path.isdir(sd) else []
        if cands:
            name = cands[-1]
            with open(os.path.join(sd, name)) as f:
                src = f.read()
    if src is None:
        log('  자세 스크립트 없음')
        return
    ns = {'__name__': 'scootover_pose'}
    try:
        exec(compile(src, name, 'exec'), ns)
        if 'set_pose' not in ns:
            log(f'  {txt.name}: set_pose 없음')
            return
        out = {}
        for name in ('loading', 'transport'):
            ns['set_pose'](name)
            bpy.context.view_layer.update()
            out[name] = world_poses()
        dump(os.path.join(out_dir, 'poses.json'), out)
        log(f'  자세 기록: {list(out)} (스크립트 {name})')
    except Exception as e:  # noqa: BLE001
        log(f'  자세 스크립트 실행 실패: {e}')
        LOG.append(traceback.format_exc())


def export_tracks(out_dir):
    """프레임마다 모든 오브젝트의 world / 부모 기준 pose. 한 번도 안 움직인 오브젝트는 static 으로만 남김"""
    sc = bpy.context.scene
    names = [ob.name for ob in bpy.data.objects
             if not any(c.name in SKIP_COLLECTIONS for c in ob.users_collection)]
    frames = list(range(sc.frame_start, sc.frame_end + 1))
    raw = {n: {'world': [], 'rel': []} for n in names}
    t0 = time.time()
    for f in frames:
        sc.frame_set(f)
        for n in names:
            ob = bpy.data.objects[n]
            mw = ob.matrix_world
            rel = (ob.parent.matrix_world.inverted() @ mw) if ob.parent else mw
            for key, m in (('world', mw), ('rel', rel)):
                t, q, _ = m.decompose()
                raw[n][key].append([r(t.x, 5), r(t.y, 5), r(t.z, 5), r(q.w, 5), r(q.x, 5), r(q.y, 5), r(q.z, 5)])
    moving, static = {}, {}
    for n, d in raw.items():
        changed_rel = any(v != d['rel'][0] for v in d['rel'])
        changed_world = any(v != d['world'][0] for v in d['world'])
        parent = bpy.data.objects[n].parent
        entry = {'parent': parent.name if parent else None}
        if changed_rel or changed_world:
            entry['rel_moves'] = changed_rel
            entry['rel'] = d['rel'] if changed_rel else d['rel'][0]
            entry['world'] = d['world']
            moving[n] = entry
        else:
            entry['rel'] = d['rel'][0]
            entry['world'] = d['world'][0]
            static[n] = entry
    dump(os.path.join(out_dir, 'tracks.json'), {
        'format': 'pose = [x, y, z, qw, qx, qy, qz]; world = 월드, rel = 부모 기준 (부모 없으면 월드)',
        'frame_start': sc.frame_start, 'frame_end': sc.frame_end, 'fps': sc.render.fps / sc.render.fps_base,
        'markers': sorted([{'frame': m.frame, 'name': m.name} for m in sc.timeline_markers], key=lambda x: x['frame']),
        'moving': moving, 'static': static})
    sc.frame_set(sc.frame_start)
    log(f'  트랙: {len(frames)}프레임, 움직이는 오브젝트 {len(moving)}개, 고정 {len(static)}개 '
        f'({time.time() - t0:.0f} s)')


# ---------------- 실행 ----------------
def main():
    log(f'Blender {bpy.app.version_string}, ROOT={ROOT}')
    main_names = set()
    log('파일: ' + ', '.join(f'{k}={v[0]}({v[1]})' for k, v in FILES.items()))
    for key, (rel, mode) in FILES.items():
        path = os.path.join(ROOT, rel)
        if not os.path.exists(path):
            log(f'[{key}] 파일 없음: {path}')
            continue
        log(f'[{key}] {rel}')
        try:
            bpy.ops.wm.open_mainfile(filepath=path, load_ui=False)
        except Exception as e:  # noqa: BLE001
            log(f'  열기 실패: {e}')
            continue
        out_dir = os.path.join(OUT, key)
        os.makedirs(out_dir, exist_ok=True)
        try:
            inv = inventory()
            if mode == 'full':
                inv['mesh_stats'] = export_meshes(out_dir)
                if key == 'robot':
                    main_names |= set(inv['objects'])
                    inv['source_version'] = rel.split('/')[0]
                    try_pose_script(out_dir, rel)
            else:
                extra = {n for n in inv['objects'] if n not in main_names}
                inv['mesh_stats'] = export_meshes(out_dir, only=extra) if extra else {}
                export_tracks(out_dir)
            save_texts(out_dir)
            dump(os.path.join(out_dir, 'inventory.json'), inv)
            log(f'  오브젝트 {len(inv["objects"])}개, 프레임 {inv["scene"]["frame_start"]}~{inv["scene"]["frame_end"]}')
        except Exception as e:  # noqa: BLE001
            log(f'  실패: {e}')
            LOG.append(traceback.format_exc())
    dump(os.path.join(OUT, 'materials.json'), MATERIALS)
    with open(os.path.join(OUT, 'export_log.txt'), 'w') as f:
        f.write('\n'.join(LOG) + '\n')
    log(f'끝 -> {OUT}')


main()
