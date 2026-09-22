from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
import os


def generate_launch_description():
    execute = LaunchConfiguration("execute")
    button_x = LaunchConfiguration("button_x")
    button_y = LaunchConfiguration("button_y")
    button_z = LaunchConfiguration("button_z")
    press_distance = LaunchConfiguration("press_distance")
    tool_length = LaunchConfiguration("tool_length")
    start_scene = LaunchConfiguration("start_scene")

    gazebo_launch = os.path.join(
        get_package_share_directory("piper_gazebo"),
        "launch", "piper_with_gripper", "piper_gazebo.launch.py",
    )
    moveit_launch = os.path.join(
        get_package_share_directory("piper_with_gripper_moveit"),
        "launch", "piper_moveit.launch.py",
    )
    moveit_config = MoveItConfigsBuilder(
        "piper", package_name="piper_with_gripper_moveit"
    ).to_moveit_configs()

    demo_node = Node(
        package="piper_elevator_manipulation",
        executable="press_button_demo",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True,
             "execute": execute,
             "button_x": button_x,
             "button_y": button_y,
             "button_z": button_z,
             "press_distance": press_distance,
             "tool_length": tool_length},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("execute", default_value="false",
                              description="Execute the planned press in Gazebo. Defaults to planning only."),
        DeclareLaunchArgument("button_x", default_value="0.300"),
        DeclareLaunchArgument("button_y", default_value="0.000"),
        DeclareLaunchArgument("button_z", default_value="0.240"),
        DeclareLaunchArgument("press_distance", default_value="0.040",
                              description="Pre-press standoff and Cartesian travel to the button surface."),
        DeclareLaunchArgument("tool_length", default_value="0.1358",
                              description="link6 to gripper contact point along local +Z."),
        DeclareLaunchArgument("start_scene", default_value="true",
                              description="Start Gazebo and MoveIt, or use an already running scene."),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(gazebo_launch),
                                 launch_arguments={"gui": "false"}.items(),
                                 condition=IfCondition(start_scene)),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(moveit_launch),
            launch_arguments={
                "capabilities": "move_group/ExecuteTaskSolutionCapability"
            }.items(),
            condition=IfCondition(start_scene),
        ),
        # Gazebo must spawn the robot and activate its controllers before an MTC
        # task can query current state and execute a FollowJointTrajectory goal.
        TimerAction(period=10.0, actions=[demo_node], condition=IfCondition(start_scene)),
        TimerAction(period=0.5, actions=[demo_node], condition=UnlessCondition(start_scene)),
    ])
