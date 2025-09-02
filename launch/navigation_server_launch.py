#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
import os

# LIDAR_MODEL=os.environ['LIDAR_MODEL']
# pkg_dir = get_package_share_directory('robot_bringup')

this_pkg_name = 'navigation_server'
home_dir = os.path.expanduser('~')
params_path = os.path.join( home_dir, '.robot_config', this_pkg_name, 'server_node.yaml')


def generate_launch_description():
    return LaunchDescription([
        Node(
            package=this_pkg_name,
            executable='server_node',
            output='screen',
            parameters=[params_path]
        )
    ])
