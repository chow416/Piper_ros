import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    perception_share = get_package_share_directory("piper_button_perception")
    gazebo_share = get_package_share_directory("piper_gazebo")

    output_dir = LaunchConfiguration("output_dir")
    num_samples = LaunchConfiguration("num_samples")
    random_seed = LaunchConfiguration("random_seed")
    overwrite = LaunchConfiguration("overwrite")
    gazebo_gui = LaunchConfiguration("gazebo_gui")

    gazebo_launch = os.path.join(
        gazebo_share,
        "launch",
        "piper_with_gripper",
        "piper_gazebo.launch.py",
    )
    config_file = os.path.join(
        perception_share,
        "config",
        "dataset_generation.yaml",
    )

    generator = Node(
        package="piper_button_perception",
        executable="generate_button_dataset",
        name="button_dataset_generator",
        output="screen",
        parameters=[
            config_file,
            {
                "output_dir": output_dir,
                "num_samples": ParameterValue(num_samples, value_type=int),
                "random_seed": ParameterValue(random_seed, value_type=int),
                "overwrite": ParameterValue(overwrite, value_type=bool),
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "output_dir",
            default_value="/ws/data/button_dataset_v1",
            description="Dataset output directory inside the container.",
        ),
        DeclareLaunchArgument("num_samples", default_value="500"),
        DeclareLaunchArgument("random_seed", default_value="17"),
        DeclareLaunchArgument("overwrite", default_value="false"),
        DeclareLaunchArgument("gazebo_gui", default_value="false"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments={"gui": gazebo_gui}.items(),
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=generator,
                on_exit=[EmitEvent(event=Shutdown(reason="dataset generator finished"))],
            )
        ),
        TimerAction(period=7.0, actions=[generator]),
    ])
