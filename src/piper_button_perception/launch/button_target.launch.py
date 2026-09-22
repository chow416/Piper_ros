from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    config = os.path.join(get_package_share_directory("piper_button_perception"),
                          "config", "button_target.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("checkpoint", default_value="/ws/models/button_ssdlite_v2/best.pt"),
        DeclareLaunchArgument("target_color", default_value="red"),
        Node(package="piper_button_perception", executable="button_target_node",
             name="button_target_node", output="screen",
             parameters=[config, {
                 "checkpoint": LaunchConfiguration("checkpoint"),
                 "default_target_color": LaunchConfiguration("target_color"),
             }]),
    ])
