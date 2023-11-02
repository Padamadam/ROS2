#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pick',
            executable='pick_control',
            name='pick_node',
            parameters=[
                {'height': 4},
            ],
            output='screen',
        ),
    ])
