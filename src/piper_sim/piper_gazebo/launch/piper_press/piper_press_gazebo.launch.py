import os
import re

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def remove_comments(text):
    return re.sub(r"<!--(.*?)-->", "", text, flags=re.DOTALL)


def generate_launch_description():
    description_share = get_package_share_directory("piper_description")
    gazebo_share = get_package_share_directory("piper_gazebo")
    xacro_path = os.path.join(
        description_share, "urdf", "piper_description_gazebo.xacro"
    )
    default_world = os.path.join(gazebo_share, "worlds", "elevator_table.world")

    gui = LaunchConfiguration("gui")
    world = LaunchConfiguration("world")

    document = xacro.process_file(xacro_path, mappings={"use_press_tool": "true"})
    robot_description = {"robot_description": remove_comments(document.toxml())}

    gazebo = ExecuteProcess(
        cmd=[
            "gazebo", "--verbose", "-s", "libgazebo_ros_init.so",
            "-s", "libgazebo_ros_factory.so", world,
        ],
        output="screen",
        condition=IfCondition(gui),
    )
    gazebo_server = ExecuteProcess(
        cmd=[
            "gzserver", "--verbose", "-s", "libgazebo_ros_init.so",
            "-s", "libgazebo_ros_factory.so", world,
        ],
        output="screen",
        condition=UnlessCondition(gui),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"use_sim_time": True}, robot_description, {"publish_frequency": 15.0}],
        output="screen",
    )
    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity", "piper", "-topic", "robot_description"],
        output="screen",
    )

    load_joint_states = ExecuteProcess(
        cmd=[
            "ros2", "control", "load_controller", "--set-state", "active",
            "joint_state_broadcaster",
        ],
        output="screen",
    )
    load_arm = ExecuteProcess(
        cmd=[
            "ros2", "control", "load_controller", "--set-state", "active",
            "arm_controller",
        ],
        output="screen",
    )

    load_joint_states_after_spawn = RegisterEventHandler(
        OnProcessExit(target_action=spawn_robot, on_exit=[load_joint_states])
    )
    load_arm_after_joint_states = RegisterEventHandler(
        OnProcessExit(target_action=load_joint_states, on_exit=[load_arm])
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "gui", default_value="false",
            description="Start the Gazebo client in addition to the physics server.",
        ),
        DeclareLaunchArgument(
            "world", default_value=default_world, description="Gazebo world file.",
        ),
        load_joint_states_after_spawn,
        load_arm_after_joint_states,
        gazebo,
        gazebo_server,
        robot_state_publisher,
        spawn_robot,
    ])
