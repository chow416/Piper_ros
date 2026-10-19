"""Run one hardware press from a fixed semantic perception target.

This wrapper deliberately owns only the interface between perception and the
existing press task.  It trusts the incoming ``base_link`` position but ignores
the incoming orientation: a configured fixed panel normal supplies the press
axis until camera calibration supports a measured normal. Start the real
arm/controller separately, and start the camera/perception pipeline separately;
combining them here would hide their independent failure modes and make stale
camera data look like a fresh task.

Example (after hardware bringup and perception are healthy)::

  ros2 launch piper_elevator_manipulation press_button_semantic_hardware.launch.py \
    target_semantic:=floor_4 execute:=true hardware_enable:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    target_semantic = LaunchConfiguration("target_semantic")
    input_topic = LaunchConfiguration("input_topic")
    expected_frame = LaunchConfiguration("expected_frame")
    max_age_s = LaunchConfiguration("max_age_s")
    future_tolerance_s = LaunchConfiguration("future_tolerance_s")
    reacquire_distance_m = LaunchConfiguration("reacquire_distance_m")
    press_direction_source = LaunchConfiguration("press_direction_source")
    fixed_press_direction_x = LaunchConfiguration("fixed_press_direction_x")
    fixed_press_direction_y = LaunchConfiguration("fixed_press_direction_y")
    fixed_press_direction_z = LaunchConfiguration("fixed_press_direction_z")
    execute = LaunchConfiguration("execute")
    hardware_enable = LaunchConfiguration("hardware_enable")
    start_moveit = LaunchConfiguration("start_moveit")
    scene_wait_s = LaunchConfiguration("scene_wait_s")
    rviz = LaunchConfiguration("rviz")
    press_params_file = LaunchConfiguration("press_params_file")

    share = get_package_share_directory("piper_elevator_manipulation")
    hardware_launch = os.path.join(share, "launch", "press_button_hardware.launch.py")

    selector = Node(
        package="piper_elevator_manipulation",
        executable="button_target_selector.py",
        output="screen",
        parameters=[
            {
                "input_topic": input_topic,
                "coarse_target_topic": "/press/target_pose",
                "refine_target_topic": "/press/refine_pose",
                "target_semantic": target_semantic,
                "expected_frame": expected_frame,
                "max_age_s": ParameterValue(max_age_s, value_type=float),
                "future_tolerance_s": ParameterValue(
                    future_tolerance_s, value_type=float
                ),
                "reacquire_distance_m": ParameterValue(
                    reacquire_distance_m, value_type=float
                ),
                "press_direction_source": press_direction_source,
                "fixed_press_direction_x": ParameterValue(
                    fixed_press_direction_x, value_type=float
                ),
                "fixed_press_direction_y": ParameterValue(
                    fixed_press_direction_y, value_type=float
                ),
                "fixed_press_direction_z": ParameterValue(
                    fixed_press_direction_z, value_type=float
                ),
            }
        ],
    )

    press_task = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(hardware_launch),
        launch_arguments={
            "target_source": "topic",
            "target_topic": "/press/target_pose",
            "max_presses": "1",
            "execute": execute,
            "hardware_enable": hardware_enable,
            "start_moveit": start_moveit,
            "scene_wait_s": scene_wait_s,
            "rviz": rviz,
            "press_params_file": press_params_file,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("target_semantic", default_value="floor_4"),
            DeclareLaunchArgument(
                "input_topic", default_value="/perception/button_targets_base"
            ),
            DeclareLaunchArgument("expected_frame", default_value="base_link"),
            DeclareLaunchArgument("max_age_s", default_value="0.5"),
            DeclareLaunchArgument("future_tolerance_s", default_value="0.05"),
            DeclareLaunchArgument("reacquire_distance_m", default_value="0.04"),
            DeclareLaunchArgument(
                "press_direction_source",
                default_value="perception",
                description="'perception' keeps the depth normal from the "
                            "perception chain; 'fixed' uses the configured "
                            "fixed_press_direction for debugging.",
            ),
            DeclareLaunchArgument(
                "fixed_press_direction_x",
                default_value="1.0",
                description="base_link X component of the fixed inward panel normal.",
            ),
            DeclareLaunchArgument(
                "fixed_press_direction_y",
                default_value="0.0",
                description="base_link Y component of the fixed inward panel normal.",
            ),
            DeclareLaunchArgument(
                "fixed_press_direction_z",
                default_value="0.0",
                description="base_link Z component of the fixed inward panel normal.",
            ),
            DeclareLaunchArgument("execute", default_value="false"),
            DeclareLaunchArgument("hardware_enable", default_value="false"),
            DeclareLaunchArgument("start_moveit", default_value="true"),
            DeclareLaunchArgument("scene_wait_s", default_value="0.0"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument(
                "press_params_file",
                default_value=os.path.join(share, "config", "press_params.yaml"),
            ),
            selector,
            press_task,
        ]
    )
