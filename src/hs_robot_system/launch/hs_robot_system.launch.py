#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node, LifecycleNode
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    pkg_share = FindPackageShare('hs_robot_system')

    # --- Package share and file paths
    pkg_share = FindPackageShare('hs_robot_system')
    rviz_cfg = PathJoinSubstitution([pkg_share, 'rviz', 'pa_rviz_nav2.rviz'])
    map_yaml = PathJoinSubstitution([pkg_share, 'map', 'pa_warehouse_map_02.yaml'])
    nav2_params = PathJoinSubstitution([pkg_share, 'config', 'pa_nav2_params.yaml'])

    # --- Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    # --- Static transform publisher (if needed)
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_base_to_lidar',
        arguments=[
            '0', '0', '0',              # x y z
            '1', '0', '0', '0',         # qx qy qz qw
            'virtual_hand_solo/base_link',
            'virtual_hand_solo/lidar_link'
        ]
    )

    # --- NAV2 Core Lifecycle Nodes
    map_server = LifecycleNode(
        package='nav2_map_server',
        executable='map_server',
        namespace='',
        name='map_server',
        output='screen',
        parameters=[
            nav2_params,
            {'yaml_filename': map_yaml,
            'use_sim_time': use_sim_time}
        ]
    )

    amcl = LifecycleNode(
        package='nav2_amcl',
        executable='amcl',
        namespace='',
        name='amcl',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': use_sim_time}]
    )

    controller_server = LifecycleNode(
        package='nav2_controller',
        executable='controller_server',
        namespace='',
        name='controller_server',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': use_sim_time}]
    )

    planner_server = LifecycleNode(
        package='nav2_planner',
        executable='planner_server',
        namespace='',
        name='planner_server',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': use_sim_time}]
    )

    behavior_server = LifecycleNode(
        package='nav2_behaviors',
        executable='behavior_server',
        namespace='',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': use_sim_time}]
    )

    bt_navigator = LifecycleNode(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        namespace='',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': use_sim_time}]
    )

    waypoint_follower = LifecycleNode(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        namespace='',
        name='waypoint_follower',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': use_sim_time}]
    )

    velocity_smoother = LifecycleNode(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        namespace='',
        name='velocity_smoother',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': use_sim_time}],
        remappings=[('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]
    )

    # --- Lifecycle Managers
    lifecycle_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['map_server', 'amcl']
        }]
    )

    lifecycle_navigation = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': [
                'controller_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower',
                'velocity_smoother'
            ]
        }]
    )

    # --- RViz2 Visualization
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_cfg],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # --- System Integration Nodes
    plc_service = Node(
        package='hs_robot_system',
        executable='plc_location_service',
        name='plc_location_service',
        output='screen'
    )

    robot_state_service = Node(
        package='hs_robot_system',
        executable='robot_state_service',
        name='robot_state_service',
        output='screen'
    )

    nav_controller = Node(
        package='hs_robot_system',
        executable='nav_controller',
        name='nav_controller',
        output='screen'
    )

    arm_manager = Node(
        package='hs_robot_system',
        executable='arm_manager',
        name='arm_manager',
        output='screen'
    )

    # --- Final Launch Description
    return LaunchDescription([
        declare_use_sim_time,

        # TF tree
        static_tf,

        # Localization
        map_server,
        amcl,
        lifecycle_localization,

        # Navigation
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        velocity_smoother,
        lifecycle_navigation,

        # Visualization
        rviz,

        # Integration nodes
        plc_service,
        robot_state_service,
        nav_controller,
        arm_manager
    ])
