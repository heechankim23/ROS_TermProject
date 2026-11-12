"""직접 구현한 9개 노드 + 시뮬레이션 보조 노드를 띄운다.

  scenario:=s1_single     config/scenarios/<이름>.yaml (또는 절대 경로)
  sim:=gz|kinematic       use_sim_time / Gazebo 모델 조작 여부 결정
  nav:=simple|nav2        nav_bridge 백엔드
  perception:=sim|apriltag  sim: 시뮬 검출기, apriltag: apriltag_ros + 어댑터
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import Node


def _setup(context):
    share = get_package_share_directory('kickboard_sim')
    sim, nav, perception = (LC(n).perform(context) for n in ('sim', 'nav', 'perception'))
    scenario = LC('scenario').perform(context)
    if not os.path.isabs(scenario):
        scenario = os.path.join(share, 'config', 'scenarios',
                                scenario if scenario.endswith('.yaml') else scenario + '.yaml')
    with open(scenario) as f:
        sc = yaml.safe_load(f)
    robot = sc.get('robot', {})
    map_cfg = os.path.join(share, 'config', 'map_config.yaml')
    params = os.path.join(share, 'config', 'robot_params.yaml')
    use_sim_time = sim == 'gz'
    use_arm = sim == 'gz' and LC('arm').perform(context) == 'true'
    common = {'use_sim_time': use_sim_time}
    out = 'screen'

    def node(exe, extra=None):
        return Node(package='kickboard_robot', executable=exe, name=exe, output=out, emulate_tty=True,
                    parameters=[params, common, extra or {}])

    acts = [
        LogInfo(msg=f'시나리오: {sc.get("name")} — {sc.get("description", "").strip()}'),
        node('queue_manager'),
        node('request_gateway', {'map_config_file': map_cfg}),
        node('mission_manager', dict({'map_config_file': map_cfg,
                                      'map_yaml': os.path.join(share, 'maps', 'kickboard_world.yaml')},
                                     **{k: v for k, v in (('capacity', robot.get('capacity')),) if v is not None})),
        node('battery_manager', {'initial_pct': float(robot.get('initial_battery_pct', 100.0))}),
        node('nav_bridge', {'backend': nav,
                            'map_yaml': os.path.join(share, 'maps', 'kickboard_world.yaml')}),
        node('collector'),
        node('dock_controller', {'map_config_file': map_cfg}),
        node('scooter_fleet_sim', {'scenario_file': scenario, 'map_config_file': map_cfg,
                                   'gz_enabled': sim == 'gz',
                                   # 로봇팔이 킥보드 모델을 직접 치우므로 fleet_sim 은 지우지 않음
                                   'remove_on_load': not use_arm}),
    ]
    if use_arm:
        acts.append(node('arm_loader'))
    if sim == 'gz':
        acts.append(node('status_panel'))      # Gazebo GUI 'Kickboard status' 패널 (Echo 버튼)
    if perception == 'sim':
        acts.append(node('sim_tag_detector', {'map_config_file': map_cfg}))
    else:
        acts.append(Node(package='apriltag_ros', executable='apriltag_node', output=out,
                         remappings=[('image_rect', '/camera/image_raw'), ('camera_info', '/camera/camera_info')],
                         parameters=[os.path.join(share, 'config', 'apriltag.yaml'), common]))
        acts.append(node('apriltag_adapter'))
    return acts


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('scenario', default_value='s1_single'),
        DeclareLaunchArgument('sim', default_value='gz', choices=['gz', 'kinematic']),
        DeclareLaunchArgument('nav', default_value='simple', choices=['simple', 'nav2']),
        DeclareLaunchArgument('perception', default_value='sim', choices=['sim', 'apriltag']),
        DeclareLaunchArgument('arm', default_value='true',
                              description='sim:=gz 에서 로봇팔 적재 애니메이션 (false: 3초 정지로 적재)'),
        OpaqueFunction(function=_setup),
    ])
