from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    gazebo_gui = LaunchConfiguration("gazebo_gui")

    gazebo_launch = os.path.join(
        get_package_share_directory("piper_gazebo"),
        "launch", "piper_with_gripper", "piper_gazebo.launch.py",
    )
    moveit_launch = os.path.join(
        get_package_share_directory("piper_with_gripper_moveit"),
        "launch", "piper_moveit.launch.py",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "gazebo_gui",
            default_value="false",
            description="Show Gazebo client. RViz is started in both modes.",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments={"gui": gazebo_gui}.items(),
        ),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(moveit_launch)),
    ])
