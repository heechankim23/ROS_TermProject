"""직접 구현한 9개 노드 + 시뮬레이션 보조 노드를 띄운다.

  scenario:=k08           config/scenarios_knu/<이름>.yaml (경북대 시나리오, 또는 절대 경로)
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
    import sys
    share = get_package_share_directory('kickboard_sim')
    sys.path.insert(0, os.path.join(share, 'launch'))
    from map_utils import map_files, resolve_map, scenario_path
    sim, nav, perception = (LC(n).perform(context) for n in ('sim', 'nav', 'perception'))
    speed = LC('speed').perform(context)
    scenario = scenario_path(LC('scenario').perform(context))
    with open(scenario) as f:
        sc = yaml.safe_load(f)
    robot = sc.get('robot', {})
    mf = map_files(resolve_map(LC('map').perform(context), scenario))
    map_cfg, nav_map = mf['config'], mf['nav_map']
    params = os.path.join(share, 'config', 'robot_params.yaml')
    use_sim_time = sim == 'gz'
    use_arm = sim == 'gz' and LC('arm').perform(context) == 'true'
    common = {'use_sim_time': use_sim_time}
    out = 'screen'

    def node(exe, extra=None, executable=None):
        # 우선순위: robot_params.yaml < 맵 설정의 params.<노드> < 시나리오 YAML 의 params.<노드> < launch 가 정한 값
        return Node(package='kickboard_robot', executable=executable or exe, name=exe, output=out, emulate_tty=True,
                    parameters=[params, common, mf['params'].get(exe, {}), (sc.get('params') or {}).get(exe, {}),
                                extra or {}])

    acts = [
        LogInfo(msg=f'맵: {mf["name"]} / 시나리오: {sc.get("name")} — {sc.get("description", "").strip()}'),
        node('queue_manager', {'map_yaml': nav_map, 'route_map_json': mf['map_json']} if mf['map_json'] else None),
        node('request_gateway', {'map_config_file': map_cfg}),
        node('mission_manager', dict({'map_config_file': map_cfg, 'map_yaml': nav_map},
                                     **{k: v for k, v in (('capacity', robot.get('capacity')),) if v is not None}),
             # 킥보드 충전소가 따로 있는 맵 / 캠퍼스 맵: 같은 노드 이름(mission_manager)으로 station·campus 판을 띄운다
             executable={'station': 'mission_manager_station', 'campus': 'mission_manager_campus'}.get(mf['mission_variant'])),
        node('battery_manager', {'initial_pct': float(robot.get('initial_battery_pct', 100.0))}),
        node('nav_bridge', dict({'backend': nav, 'map_yaml': nav_map, 'route_map_json': mf['map_json'],
                            # 물리 지도가 따로 있으면(진입 금지 구역이 있는 맵) 넘겨서 벽과 구분하게 한다
                            'physical_map_yaml': mf['lidar_map'] if mf['lidar_map'] != nav_map else ''},
                           # speed:=2.5 처럼 주면 로봇 최고 속도를 덮어쓴다 (빨리 보기)
                           **({'v_max': float(speed), 'w_max': min(2.0, 0.6 + 0.3 * float(speed))} if speed else {}))),
        node('collector'),
        node('dock_controller', {'map_config_file': map_cfg, 'gz_enabled': sim == 'gz', 'world_name': mf['name']}),
        node('scooter_fleet_sim', {'scenario_file': scenario, 'map_config_file': map_cfg, 'terrain_json': mf['terrain_json'],
                                   'gz_enabled': sim == 'gz', 'world_name': mf['name'],
                                   # 로봇팔이 킥보드 모델을 직접 치우므로 fleet_sim 은 지우지 않음
                                   'remove_on_load': not use_arm}),
    ]
    # 운영 현황 (역할 ④): 현재 임무·이유·목적지·정렬 결과 -> /ops_status, RViz /ops_markers, ~/.ros/kickboard_ops/*.jsonl
    acts.append(node('ops_monitor', {'map_config_file': map_cfg, 'scenario_file': scenario}))
    if use_arm:
        acts.append(node('arm_loader', {'world_name': mf['name']}))
    if sim == 'gz':
        acts.append(node('status_panel', {'world_name': mf['name'] if mf['map_json'] else ''}))   # 패널 + 목적지 기둥(캠퍼스)
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
        DeclareLaunchArgument('scenario', default_value='k08'),
        DeclareLaunchArgument('map', default_value='', description='맵 이름. 비우면 시나리오 YAML 의 map 키'),
        DeclareLaunchArgument('sim', default_value='gz', choices=['gz', 'kinematic']),
        DeclareLaunchArgument('nav', default_value='simple', choices=['simple', 'nav2']),
        DeclareLaunchArgument('perception', default_value='sim', choices=['sim', 'apriltag']),
        DeclareLaunchArgument('speed', default_value='', description='로봇 최고 속도 m/s 덮어쓰기 (비우면 맵 기본값)'),
        DeclareLaunchArgument('arm', default_value='true',
                              description='sim:=gz 에서 로봇팔 적재 애니메이션 (false: 3초 정지로 적재)'),
        OpaqueFunction(function=_setup),
    ])
