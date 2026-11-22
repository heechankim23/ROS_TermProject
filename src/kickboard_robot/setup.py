from setuptools import setup

package_name = 'kickboard_robot'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ROS TermProject Team',
    maintainer_email='team@example.com',
    description='공유 킥보드 자율 수거·충전 로봇 노드',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            f'{n} = {package_name}.{n}:main' for n in (
                'scooter_fleet_sim', 'request_gateway', 'queue_manager', 'mission_manager',
                'collector', 'dock_controller', 'battery_manager', 'nav_bridge',
                'sim_tag_detector', 'apriltag_adapter', 'fake_base', 'analyze', 'arm_loader', 'status_panel', 'sim_localizer', 'sim_lidar')
        ] + [
            'extended_mission = kickboard_robot.extended_runtime:mission_main',
            'extended_queue = kickboard_robot.extended_runtime:queue_main',
            'extended_fleet = kickboard_robot.extended_runtime:fleet_main',
            'extended_dock = kickboard_robot.extended_runtime:dock_main',
            'extended_collector = kickboard_robot.extended_collector:main',
            'extended_guard = kickboard_robot.extended_guard:main',
            'extended_lidar = kickboard_robot.extended_lidar:main',
            'extended_nav = kickboard_robot.extended_nav:main',
            'mission_manager_station = kickboard_robot.mission_manager_station:main',
            'mission_manager_campus = kickboard_robot.mission_manager_campus:main',
            'ops_monitor = kickboard_robot.ops_monitor:main',
        ],
    },
)
