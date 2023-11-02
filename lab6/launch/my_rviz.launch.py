import os

from ament_index_python import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import ExecuteProcess


def generate_launch_description():
    res = []


    model_launch_arg = DeclareLaunchArgument(
        "model",
        default_value=os.path.join(
            get_package_share_directory("lab6"),
            "urdf/fake_bombel.urdf.xacro"
        )
    )
    res.append(model_launch_arg)

    rvizconfig_launch_arg = DeclareLaunchArgument(
        "rvizconfig",
        default_value=os.path.join(
            get_package_share_directory("lab6"),
            "config/my_rviz.rviz"
        )
    )
    res.append(rvizconfig_launch_arg)
    

    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]),
                                       value_type=str)
    

    # robot_state_publisher_node = Node(
    #     name="robot_state_publisher",
    #     package="robot_state_publisher",
    #     executable="robot_state_publisher",
    #     parameters=[{'robot_description': robot_description}]
    # )
    # res.append(robot_state_publisher_node)

    # rviz_node = Node(
    #     name="rviz2",
    #     package="rviz2",
    #     executable="rviz2",
    #     output="screen",
    #     arguments=['-d', LaunchConfiguration("rvizconfig")],
    # )
    # res.append(rviz_node)

    # change_first_name = ExecuteProcess(
    #     cmd=[[
    #         'ros2 param set ',
    #         '/joint_state_publisher source_list',
    #         "/joint_states/position[0]"
    #     ]],
    #     shell=True
    # )
    # res.append(change_first_name)
            
    # res.append(Node(
    #         package='lab6',
    #         executable='camera_link_publisher',
    #         name='camera_link_publisher',
    #         output='screen',
    #         emulate_tty=True))

    # res.append(Node(
    #         package='lab6',
    #         executable='listener',
    #         name='custom_minimal_param_node',
    #         output='screen',
    #         emulate_tty=True))
            
    # res.append(Node(
    #         package='lab6',
    #         executable='marker_publisher',
    #         name='marker_publisher',
    #         output='screen',
    #         emulate_tty=True))
            
    # res.append(Node(
    #         package='lab6',
    #         executable='joint_state_publisher',
    #         name='joint_state_publisher',
    #         output='screen',
    #         emulate_tty=True))
    
    # res.append(Node(
    #         package='lab6',
    #         executable='pick_node',
    #         name='pick_node',
    #         output='screen',
    #         emulate_tty=True))
    
    res.append(Node(
            package='lab6',
            executable='lab6node',
            name='lab6node',
            output='screen',
            emulate_tty=True))
    

    return LaunchDescription(res)

