"""19-scenario opt-in suite: unchanged s1-s9 plus scenarios_extended/s10-s19."""
import os
import sys
import math
import tempfile
import xml.etree.ElementTree as ET
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration as LC


def temporary(text, suffix):
    with tempfile.NamedTemporaryFile('w', encoding='utf-8', suffix=suffix, delete=False) as f:
        f.write(text)
        return f.name


def _setup(context):
    share = get_package_share_directory('kickboard_sim')
    directory = os.path.join(share, 'launch')
    sys.path.insert(0, directory)
    from map_utils import map_files
    scenario = LC('scenario').perform(context)
    sim = LC('sim').perform(context)
    sim = 'gz' if sim == 'gazebo' else sim
    nav = LC('nav').perform(context)
    if sim not in ('gz','kinematic') or nav not in ('nav2','simple'):
        raise ValueError('sim: gazebo|gz|kinematic; nav: nav2|simple')
    def include(file, args):
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(directory,file)),
                                         launch_arguments=args.items())
    common = {n: LC(n).perform(context) for n in ('gui','rviz','sensors','arm')}
    # Legacy dispatch has no extension nodes or parameter changes.
    if scenario.split('_')[0] in [f's{i}' for i in range(1,10)]:
        return [include('bringup.launch.py', dict(common, scenario=scenario,sim=sim,nav=nav))]
    if os.path.basename(scenario) != scenario:
        raise ValueError('Use a scenario name from config/scenarios_extended/manifest.json')
    path = os.path.join(share,'config','scenarios_extended',scenario+'.yaml')
    with open(path, encoding='utf-8') as f:
        sc = yaml.safe_load(f)
    map_name = LC('map').perform(context) or sc['map']
    if map_name not in ('street_extended','campus_extended'):
        raise ValueError('Extended scenarios require street_extended or campus_extended')
    mf = map_files(map_name)
    with open(mf['config'], encoding='utf-8') as f:
        mc = yaml.safe_load(f)
    with open(os.path.join(share,'config','nav2_params.yaml'),encoding='utf-8') as f:
        np = yaml.safe_load(f)
    spawn = mc['robot_spawn']
    np['amcl']['ros__parameters']['initial_pose'] = dict(x=spawn['x'],y=spawn['y'],z=0.0,yaw=math.radians(spawn['yaw_deg']))
    np['amcl']['ros__parameters']['laser_max_range'] = 8.0
    # AMCL uses the physical map; both costmaps use the non-drivable policy map.
    for key in ('local_costmap','global_costmap'):
        p = np[key][key]['ros__parameters']
        p['plugins'] = ['static_layer','obstacle_layer','inflation_layer']
        p['static_layer'] = dict(plugin='nav2_costmap_2d::StaticLayer',map_topic='/policy_map',
                                map_subscribe_transient_local=True, subscribe_to_updates=False)
        p['footprint'] = '[[-0.85,-0.45],[0.85,-0.45],[0.85,0.45],[-0.85,0.45]]'
        p['inflation_layer']['inflation_radius'] = 1.05
    local = np['local_costmap']['local_costmap']['ros__parameters']
    local.update(width=6,height=6)
    np['controller_server']['ros__parameters']['progress_checker']['movement_time_allowance'] = 30.0
    nav_params = temporary(yaml.safe_dump(np), '_nav2.yaml')
    world = ET.parse(mf['world'])
    robot = ET.parse(os.path.join(share,'models','kickboard_robot','model.sdf'))
    ext = sc.get('extended',{})
    if ext.get('night'):
        w = world.find('world')
        scene = w.find('scene')
        if scene is None: scene = ET.SubElement(w,'scene')
        for tag,value in [('ambient','0.03 0.03 0.05 1'),('background','0.01 0.01 0.03 1')]:
            e = scene.find(tag)
            if e is None: e = ET.SubElement(scene,tag)
            e.text=value
        for light in w.findall('light'):
            for tag in ('diffuse','specular'):
                e=light.find(tag)
                if e is not None: e.text='0.02 0.02 0.03 1'
        link = robot.find('.//model/link')
        light = ET.fromstring('''<light name="collection_headlight" type="spot">
          <pose>0.80 0 0.7 0 0 0</pose><diffuse>1 1 0.9 1</diffuse><specular>0.3 0.3 0.3 1</specular>
          <direction>1 0 -0.12</direction><cast_shadows>true</cast_shadows>
          <attenuation><range>10</range><constant>0.5</constant><linear>0.1</linear><quadratic>0.02</quadratic></attenuation>
          <spot><inner_angle>0.4</inner_angle><outer_angle>0.8</outer_angle><falloff>1</falloff></spot></light>''')
        link.append(light)
    if 'friction' in ext:
        for tree in (world,robot):
            for collision in tree.findall('.//collision'):
                surface = collision.find('surface')
                if surface is None: surface=ET.SubElement(collision,'surface')
                friction=surface.find('friction')
                if friction is None: friction=ET.SubElement(surface,'friction')
                ode=friction.find('ode')
                if ode is None: ode=ET.SubElement(friction,'ode')
                for key in ('mu','mu2'):
                    e=ode.find(key)
                    if e is None: e=ET.SubElement(ode,key)
                    e.text=str(ext['friction'])
    wp = temporary(ET.tostring(world.getroot(),encoding='unicode'),'_world.sdf')
    rp = temporary(ET.tostring(robot.getroot(),encoding='unicode'),'_robot.sdf')
    return [include('sim_extended.launch.py',dict(sim=sim,nav=nav,map_name=map_name,
                gui=common['gui'],rviz=common['rviz'],sensors=common['sensors'],
                world_file=wp,robot_file=rp,nav_params=nav_params)),
            TimerAction(period=6.0,actions=[include('system_extended.launch.py',dict(
                scenario=path,sim=sim,nav=nav,map=map_name,arm=common['arm'],perception='sim'))])]


def generate_launch_description():
    args = {'scenario':'s10_damaged_kickboard','sim':'gazebo','nav':'nav2','map':'',
            'gui':'true','rviz':'false','sensors':'true','arm':'true'}
    return LaunchDescription([DeclareLaunchArgument(k,default_value=v) for k,v in args.items()]
                             + [OpaqueFunction(function=_setup)])
