"""Humble Nav2 with explicit velocity routing through the extended guard."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import Node


def setup(context):
    params = LC('params_file').perform(context)
    sim_time = LC('use_sim_time').perform(context) == 'true'
    acts = [IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
        get_package_share_directory('nav2_bringup'),'launch','localization_launch.py')),
        launch_arguments={'map': LC('map').perform(context), 'params_file': params,
                          'use_sim_time': str(sim_time).lower(), 'autostart':'true',
                          'use_composition':'False'}.items())]
    names = []
    for package, exe in [('nav2_controller','controller_server'), ('nav2_smoother','smoother_server'),
                         ('nav2_planner','planner_server'), ('nav2_behaviors','behavior_server'),
                         ('nav2_bt_navigator','bt_navigator'), ('nav2_waypoint_follower','waypoint_follower'),
                         ('nav2_velocity_smoother','velocity_smoother')]:
        names.append(exe)
        remaps = [('/tf','tf'),('/tf_static','tf_static')]
        if exe in ('controller_server','velocity_smoother'):
            remaps += [('cmd_vel','/cmd_vel_nav')]
        if exe == 'velocity_smoother':
            remaps += [('cmd_vel_smoothed','/extended/cmd_vel_raw')]
        if exe == 'behavior_server':
            remaps += [('cmd_vel','/extended/cmd_vel_raw')]
        acts.append(Node(package=package,executable=exe,name=exe,output='screen',
                         parameters=[params,{'use_sim_time':sim_time}],remappings=remaps))
    acts.append(Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',
                     name='lifecycle_manager_navigation',parameters=[{'use_sim_time':sim_time,
                     'autostart':True,'node_names':names}]))
    return acts


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('map'),DeclareLaunchArgument('params_file'),
                              DeclareLaunchArgument('use_sim_time',default_value='true'),
                              OpaqueFunction(function=setup)])
