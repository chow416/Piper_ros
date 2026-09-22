from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("piper", package_name="piper_press_moveit")
        .robot_description(mappings={"use_press_tool": "true"})
        .robot_description_semantic(file_path="config/piper.srdf")
        .to_moveit_configs()
    )

    capabilities = LaunchConfiguration("capabilities")
    start_rviz = LaunchConfiguration("rviz")

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "use_sim_time": True,
                "allow_trajectory_execution": True,
                "publish_robot_description_semantic": True,
                "publish_planning_scene": True,
                "publish_geometry_updates": True,
                "publish_state_updates": True,
                "publish_transforms_updates": True,
                "capabilities": ParameterValue(capabilities, value_type=str),
            },
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", str(moveit_config.package_path / "config" / "moveit.rviz")],
        parameters=[moveit_config.to_dict(), {"use_sim_time": True}],
        condition=IfCondition(start_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "capabilities",
            default_value="",
            description="Additional MoveGroup capabilities separated by spaces.",
        ),
        DeclareLaunchArgument(
            "rviz", default_value="true", description="Start MoveIt RViz.",
        ),
        move_group,
        rviz,
    ])
