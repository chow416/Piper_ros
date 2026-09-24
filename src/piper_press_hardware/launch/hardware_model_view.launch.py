"""Visualize real joint feedback without launching hardware control."""

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    description_share = get_package_share_path('piper_description')
    hardware_share = get_package_share_path('piper_press_hardware')
    xacro_path = description_share / 'urdf' / 'piper_description.xacro'
    rviz_config = hardware_share / 'rviz' / 'hardware_model.rviz'

    robot_description = ParameterValue(
        Command(['xacro ', str(xacro_path), ' use_press_tool:=true']),
        value_type=str,
    )
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='piper_press_hardware_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': False}],
        remappings=[('/joint_states', '/hardware_joint_states')],
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='piper_press_hardware_rviz',
        output='screen',
        arguments=['-d', str(rviz_config)],
        parameters=[{'use_sim_time': False}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        robot_state_publisher,
        rviz,
    ])
