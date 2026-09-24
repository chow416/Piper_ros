"""Read-only hardware feedback; does not launch a controller or MoveIt."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('can_port', default_value='can0'),
        Node(
            package='piper_press_hardware',
            executable='joint_state_reader',
            name='piper_press_joint_state_reader',
            output='screen',
            parameters=[{
                'can_port': LaunchConfiguration('can_port'),
                'use_sim_time': False,
            }],
        ),
    ])
