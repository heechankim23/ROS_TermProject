"""한 번에 실행: 시뮬레이션 환경 + 전체 노드.

  ros2 launch kickboard_sim bringup.launch.py scenario:=s3_full_load
  ros2 launch kickboard_sim bringup.launch.py sim:=kinematic scenario:=s4_low_battery rviz:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration as LC
from launch.substitutions import PythonExpression

ARGS = {
    'scenario': 's1_single', 'sim': 'gz', 'nav': 'simple', 'perception': 'sim',
    'gui': 'true', 'rviz': 'false', 'sensors': 'true', 'start_delay': '6.0', 'arm': 'true',
}


def generate_launch_description():
    launch_dir = os.path.join(get_package_share_directory('kickboard_sim'), 'launch')
    pass_args = lambda names: {n: LC(n) for n in names}.items()  # noqa: E731
    return LaunchDescription(
        [DeclareLaunchArgument(k, default_value=v) for k, v in ARGS.items()] + [
            IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(launch_dir, 'sim.launch.py')),
                                     launch_arguments=list(pass_args(['sim', 'nav', 'gui', 'rviz', 'sensors'])) + [
                                         # 카메라는 실제 AprilTag 검출을 쓸 때만 렌더링
                                         ('camera', PythonExpression(
                                             ["'true' if '", LC('perception'), "' == 'apriltag' else 'false'"]))]),
            # Gazebo 가 뜨고 로봇이 스폰될 시간을 준 뒤 노드 시작
            TimerAction(period=LC('start_delay'), actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(os.path.join(launch_dir, 'system.launch.py')),
                    launch_arguments=pass_args(['scenario', 'sim', 'nav', 'perception', 'arm']))]),
        ])
