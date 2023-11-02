#!/usr/bin/env python3


from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim'
        ),
        Node(
            package='lab2',
            executable='turtle_control',
            name='turtle_control_node',
            parameters=[
                {'second_turtle_name': 'turtle2'},
                {'third_turtle_name': 'turtle3'},
            ],
            output='screen',
        ),
    ])
