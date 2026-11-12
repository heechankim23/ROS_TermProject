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
        ],
    },
)
