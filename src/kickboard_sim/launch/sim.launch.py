"""시뮬레이션 환경만 띄운다: Gazebo(Harmonic) 또는 kinematic 베이스 + 브리지 + TF (+ Nav2).

  sim:=gz         Gazebo world + 로봇 + ros_gz_bridge (기본)
  sim:=kinematic  Gazebo 없이 fake_base 가 /cmd_vel 을 적분 (로직 검증용, 가벼움)
  nav:=simple     map->odom 을 정적 TF 로 고정 (odom = 정답 위치로 가정)
  nav:=nav2       nav2_bringup(AMCL + map_server + planner/controller) 실행
  camera:=true    로봇 카메라 렌더링 (bringup 은 perception:=apriltag 일 때 자동으로 켬)
  sensors:=false  라이다·카메라 렌더링을 끈다 (X 디스플레이 없이 실행할 때, simple+sim 조합에서만)
"""
import math
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction, TimerAction,
                            SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import Node


def _setup(context):
    import sys
    share = get_package_share_directory('kickboard_sim')
    sys.path.insert(0, os.path.join(share, 'launch'))
    from map_utils import map_files
    mf = map_files(LC('map_name').perform(context))
    nav_map = LC('map').perform(context) or mf['nav_map']
    sim, nav = LC('sim').perform(context), LC('nav').perform(context)
    gui = LC('gui').perform(context) == 'true'
    use_sim_time = sim == 'gz'
    with open(mf['config']) as f:
        spawn = yaml.safe_load(f)['robot_spawn']
    sx, sy, syaw = spawn['x'], spawn['y'], math.radians(spawn['yaw_deg'])

    acts = []
    st = {'use_sim_time': use_sim_time}

    def static_tf(name, parent, child, xyz, rpy=(0, 0, 0)):
        return Node(package='tf2_ros', executable='static_transform_publisher', name=name,
                    arguments=['--x', str(xyz[0]), '--y', str(xyz[1]), '--z', str(xyz[2]),
                               '--roll', str(rpy[0]), '--pitch', str(rpy[1]), '--yaw', str(rpy[2]),
                               '--frame-id', parent, '--child-frame-id', child],
                    parameters=[st], output='log')

    acts += [static_tf('tf_scan', 'base_footprint', 'base_scan', (0.80, 0, 0.22)),
             static_tf('tf_cam', 'base_footprint', 'camera_link', (0.76, 0, 0.45)),
             static_tf('tf_cam_opt', 'camera_link', 'camera_optical_frame', (0, 0, 0),
                       (-math.pi / 2, 0, -math.pi / 2))]

    if sim == 'gz':
        models = os.path.join(share, 'models')
        acts.append(SetEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH', os.pathsep.join(filter(None, [os.environ.get('GZ_SIM_RESOURCE_PATH'), models]))))
        world = mf['world']
        rtf = float(LC('rtf').perform(context) or 1.0)
        if LC('sensors').perform(context) != 'true' or abs(rtf - 1.0) > 1e-6:
            import re
            import tempfile
            with open(world) as f:
                txt = f.read()
            if LC('sensors').perform(context) != 'true':
                # 렌더링(라이다·카메라) 없이 물리만: X 디스플레이/GPU 가 없어도 동작
                txt = re.sub(r'<plugin filename="gz-sim-sensors-system".*?</plugin>', '', txt, flags=re.S)
            # 배속: 시뮬레이션 시간을 실제보다 rtf 배 빠르게 (노드는 모두 sim time 을 따른다). 0 = 가능한 한 빠르게
            txt = re.sub(r'<real_time_factor>[^<]*</real_time_factor>', f'<real_time_factor>{rtf}</real_time_factor>', txt)
            tmp = tempfile.NamedTemporaryFile('w', suffix=f'_{mf["name"]}.sdf', delete=False)
            tmp.write(txt)
            tmp.close()
            world = tmp.name
        # GUI: 기본 설정 + 'Kickboard status' 패널 (config/gz_gui.config)
        gui_path = os.path.join(share, 'config', 'gz_gui.config')
        if gui:
            # 처음 화면: 로봇 뒤 위쪽에서 로봇을 바라봄 (캠퍼스 전체가 아니라 시나리오 현장부터 보이게)
            import re
            import tempfile
            cx, cy = sx - 7 * math.cos(syaw), sy - 7 * math.sin(syaw)
            with open(gui_path) as f:
                gtxt = re.sub(r'<camera_pose>[^<]*</camera_pose>',
                              f'<camera_pose>{cx:.1f} {cy:.1f} 5 0 0.55 {syaw:.3f}</camera_pose>', f.read())
            gtmp = tempfile.NamedTemporaryFile('w', suffix='_gui.config', delete=False)
            gtmp.write(gtxt)
            gtmp.close()
            gui_path = gtmp.name
        gui_cfg = f"--gui-config {gui_path} " if gui else ''
        gz_args = f"-r {'' if gui else '-s '}{gui_cfg}--render-engine ogre {world}"
        acts.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'true'}.items()))
        robot_sdf = os.path.join(models, 'kickboard_robot', 'model.sdf')
        if LC('camera').perform(context) != 'true':
            # perception:=sim 에서는 카메라 영상을 아무도 안 쓰므로 렌더링 부하를 없앤다
            import re
            import tempfile
            with open(robot_sdf) as f:
                txt = re.sub(r'<sensor name="camera".*?</sensor>', '', f.read(), flags=re.S)
            tmp = tempfile.NamedTemporaryFile('w', suffix='_robot.sdf', delete=False)
            tmp.write(txt)
            tmp.close()
            robot_sdf = tmp.name
        # 로봇 생성: Gazebo 가 world(메시·텍스처)를 다 불러오기 전에 요청하면 응답 시간 초과로 로봇이 안 생긴다.
        # 4초 뒤에 만들고, 12초 뒤에 한 번 더 요청한다 (이미 있으면 '이름 중복'으로 무시됨)
        for k, delay in enumerate((4.0, 12.0)):
            acts.append(TimerAction(period=delay, actions=[Node(
                package='ros_gz_sim', executable='create', output='log', name=f'spawn_robot_{k}',
                arguments=['-world', mf['name'], '-name', 'kickboard_robot', '-file', robot_sdf,
                           '-x', str(sx), '-y', str(sy), '-z', '0.01', '-Y', str(syaw)])]))
        if gui and LC('follow').perform(context) == 'true':
            # 카메라가 로봇을 따라가게 (Gazebo 의 CameraTracking: /gui/follow).
            # world·GUI 를 불러오는 시간이 PC 마다 달라서 정해진 시각에 한 번 부르면 놓친다 ->
            # 로봇이 생기고 GUI 의 follow 서비스가 뜰 때까지 기다렸다가 부르고, 확실히 하려고 몇 번 더 부른다 (최대 3분)
            follow = (
                "for i in $(seq 90); do "
                "  gz model --list 2>/dev/null | grep -q kickboard_robot && gz service -l 2>/dev/null | grep -q '^/gui/follow$' && break; "
                "  sleep 2; "
                "done; "
                "for k in 1 2 3 4; do "
                "  gz service -s /gui/follow --reqtype gz.msgs.StringMsg --reptype gz.msgs.Boolean --timeout 3000 "
                "--req 'data: \"kickboard_robot\"'; "
                "  gz service -s /gui/follow/offset --reqtype gz.msgs.Vector3d --reptype gz.msgs.Boolean --timeout 3000 "
                "--req 'x: -6 y: 0 z: 4.5'; "
                "  sleep 3; "
                "done")
            acts.append(TimerAction(period=3.0, actions=[ExecuteProcess(cmd=['bash', '-c', follow], output='log')]))
        acts.append(Node(package='ros_gz_bridge', executable='parameter_bridge', name='gz_bridge',
                         parameters=[{'config_file': os.path.join(share, 'config', 'gz_bridge.yaml'),
                                      'use_sim_time': True}], output='log'))
    else:
        acts.append(Node(package='kickboard_robot', executable='fake_base', output='log'))

    # 라이다: Gazebo gpu_lidar 는 이 VM 에서 쓸 수 없어 sim_lidar 가 /scan 을 만든다 (gz·kinematic 공통)
    acts.append(Node(package='kickboard_robot', executable='sim_lidar', output='log',
                     parameters=[{'use_sim_time': use_sim_time, 'map_yaml': mf['lidar_map']},
                                 mf['params'].get('sim_lidar', {})]))

    if nav == 'nav2':
        acts.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')),
            launch_arguments={'map': nav_map,
                              'params_file': os.path.join(share, 'config', 'nav2_params.yaml'),
                              'use_sim_time': str(use_sim_time).lower(),
                              'autostart': 'true'}.items()))
    elif sim == 'gz':
        # simple + Gazebo: 정답 위치로 map->odom 을 계속 보정 (완벽한 AMCL 가정). 1.7 m 로봇은 부딪히거나
        # 미끄러지면 오도메트리가 수 m 어긋나서, 정적 TF 로는 도킹·정렬 위치가 틀어졌다.
        acts.append(Node(package='kickboard_robot', executable='sim_localizer', output='log',
                         parameters=[{'use_sim_time': True}]))
    else:
        # kinematic: fake_base 는 미끄러짐이 없으므로 odom 원점 = 로봇 스폰 위치로 고정
        acts.append(static_tf('tf_map_odom', 'map', 'odom', (sx, sy, 0), (0, 0, syaw)))

    if LC('rviz').perform(context) == 'true':
        acts.append(Node(package='rviz2', executable='rviz2', output='log',
                         arguments=['-d', os.path.join(share, 'rviz', 'kickboard.rviz')], parameters=[st]))
    return acts


def generate_launch_description():
    share = get_package_share_directory('kickboard_sim')
    return LaunchDescription([
        DeclareLaunchArgument('sim', default_value='gz', choices=['gz', 'kinematic']),
        DeclareLaunchArgument('nav', default_value='simple', choices=['simple', 'nav2']),
        DeclareLaunchArgument('gui', default_value='true', description='Gazebo GUI 표시'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('camera', default_value='false',
                              description='true: 로봇 카메라 렌더링 (perception:=apriltag 에서 필요)'),
        DeclareLaunchArgument('follow', default_value='true', description='Gazebo 카메라가 로봇을 따라감 (gui 일 때)'),
        DeclareLaunchArgument('rtf', default_value='1.0',
                              description='배속: Gazebo 시뮬레이션 시간 배율 (예: 4 = 4배속, 0 = 최대한 빠르게)'),
        DeclareLaunchArgument('sensors', default_value='true',
                              description='false: 라이다·카메라 렌더링 끔 (디스플레이 없는 환경, nav2 불가)'),
        DeclareLaunchArgument('map_name', default_value='kickboard_world',
                              description='맵 이름: kickboard_world | street | campus (launch/map_utils.py)'),
        DeclareLaunchArgument('map', default_value='', description='Nav2 지도 yaml 경로 (비우면 맵 이름에서 정함)'),
        OpaqueFunction(function=_setup),
    ])
