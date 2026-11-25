"""한 번에 실행: 시뮬레이션 환경 + 전체 노드.

  ros2 launch kickboard_sim bringup.launch.py scenario:=s3_full_load
  ros2 launch kickboard_sim bringup.launch.py sim:=kinematic scenario:=s4_low_battery rviz:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration as LC
from launch.substitutions import PythonExpression

ARGS = {
    'scenario': 'k08', 'sim': 'gz', 'nav': 'simple', 'perception': 'sim',
    'gui': 'true', 'rviz': 'false', 'sensors': 'true', 'start_delay': '6.0', 'arm': 'true', 'rtf': '1.0',
    'speed': '', 'follow': 'true',
    'map': '',      # 맵 이름 (kickboard_world | street | campus). 비우면 시나리오 YAML 의 map 키 -> 기본 맵
}


def _setup(context):
    import sys
    launch_dir = os.path.join(get_package_share_directory('kickboard_sim'), 'launch')
    sys.path.insert(0, launch_dir)
    from map_utils import resolve_map
    map_name = resolve_map(LC('map').perform(context), LC('scenario').perform(context))
    pass_args = lambda names: {n: LC(n) for n in names}.items()  # noqa: E731
    return [
            IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(launch_dir, 'sim.launch.py')),
                                     launch_arguments=list(pass_args(['sim', 'nav', 'gui', 'rviz', 'sensors', 'rtf', 'follow'])) + [
                                         ('map_name', map_name),
                                         # 카메라는 실제 AprilTag 검출을 쓸 때만 렌더링
                                         ('camera', PythonExpression(
                                             ["'true' if '", LC('perception'), "' == 'apriltag' else 'false'"]))]),
            # Gazebo 가 뜨고 로봇이 스폰될 시간을 준 뒤 노드 시작
            TimerAction(period=LC('start_delay'), actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(os.path.join(launch_dir, 'system.launch.py')),
                    launch_arguments=list(pass_args(['scenario', 'sim', 'nav', 'perception', 'arm', 'speed']))
                    + [('map', map_name)])]),
    ]


def generate_launch_description():
    return LaunchDescription(
        [DeclareLaunchArgument(k, default_value=v) for k, v in ARGS.items()] + [OpaqueFunction(function=_setup)])
