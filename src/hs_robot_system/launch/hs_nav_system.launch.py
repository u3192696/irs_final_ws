#!/usr/bin/env python3
"""
hs_nav_system.launch.py
-----------------------
Launch file to start the navigation controller and mock PLC location service
together for integrated system testing.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # PLC location service node
    plc_service_node = Node(
        package='hs_robot_system',
        executable='plc_location_service',
        name='plc_location_service',
        output='screen'
    )

    # Navigation controller node
    nav_controller_node = Node(
        package='hs_robot_system',
        executable='nav_controller',
        name='nav_controller',
        output='screen'
    )

    # (Optional) You can later add RViz and Nav2 bringup launch includes here

    return LaunchDescription([
        plc_service_node,
        nav_controller_node,
    ])
