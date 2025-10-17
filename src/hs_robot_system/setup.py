from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'hs_robot_system'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test'], include=['hs_robot_system', 'hs_robot_system.*']),
    #packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'map'), glob('map/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Daniel',
    maintainer_email='u3192696@uni.canberra.edu.au',
    description='Final integration project for IRS',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'nav_controller = hs_robot_system.nodes.nav_controller:main',
            'arm_manager = hs_robot_system.nodes.arm_manager:main',
            'plc_location_service = hs_robot_system.nodes.plc_location_service:main',
            'robot_state_service = hs_robot_system.nodes.robot_state_service:main',
        ],
    },
)
