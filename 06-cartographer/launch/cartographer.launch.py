#!/usr/bin/env python3
"""
ROS 2 Launch file for Google Cartographer 2D on NVIDIA JetBot CS532
===================================================================
Brings up:
1. jetbot_ros_bridge_node: Ingests JB-Binary telemetry & publishes /scan + TF
2. cartographer_node: Google Cartographer 2D SLAM
3. cartographer_occupancy_grid_node: Publishes standard /map
4. map_bridge_to_web: Bridges /map and TF to web dashboard
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
    config_dir_default = os.path.join(pkg_dir, 'config')

    # Declare arguments
    config_dir_arg = DeclareLaunchArgument(
        'cartographer_config_dir',
        default_value=config_dir_default,
        description='Directory containing Cartographer Lua configuration file'
    )

    lua_filename_arg = DeclareLaunchArgument(
        'configuration_basename',
        default_value='jetbot_d500_2d.lua',
        description='Basename of Lua configuration file'
    )

    resolution_arg = DeclareLaunchArgument(
        'resolution',
        default_value='0.05',
        description='Resolution of occupancy grid map in meters'
    )

    ws_url_arg = DeclareLaunchArgument(
        'ws_url',
        default_value='ws://127.0.0.1:8000/ws/robot',
        description='WebSocket URL to connect to JetBot hardware bridge'
    )

    # 1. JetBot ROS Bridge Node
    bridge_node = Node(
        package='jetbot_cartographer',
        executable='jetbot_ros_bridge_node.py',
        name='jetbot_ros_bridge_node',
        output='screen',
        parameters=[{
            'ws_url': LaunchConfiguration('ws_url'),
            'connection_mode': 'ws_client',
            'laser_frame_id': 'laser_frame',
            'base_frame_id': 'base_link',
            'laser_offset_x': 0.08,
            'laser_offset_y': 0.00,
            'laser_offset_z': 0.07,
            'range_min': 0.05,
            'range_max': 6.0
        }]
    )

    # 2. Cartographer Node (Google Cartographer 2D)
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': False}],
        arguments=[
            '-configuration_directory', LaunchConfiguration('cartographer_config_dir'),
            '-configuration_basename', LaunchConfiguration('configuration_basename')
        ],
        remappings=[
            ('scan', '/scan')
        ]
    )

    # 3. Cartographer Occupancy Grid Node
    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'resolution': LaunchConfiguration('resolution'),
            'publish_period_sec': 1.0
        }]
    )

    # 4. Map Bridge to Web Dashboard
    map_bridge_node = Node(
        package='jetbot_cartographer',
        executable='map_bridge_to_web.py',
        name='map_bridge_to_web',
        output='screen',
        parameters=[{
            'dashboard_ws_url': 'ws://127.0.0.1:8000/ws',
            'map_topic': '/map',
            'publish_throttle_hz': 2.0
        }]
    )

    return LaunchDescription([
        config_dir_arg,
        lua_filename_arg,
        resolution_arg,
        ws_url_arg,
        bridge_node,
        cartographer_node,
        occupancy_grid_node,
        map_bridge_node
    ])
