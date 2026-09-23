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
    press_direction_x = LaunchConfiguration("press_direction_x")
    press_direction_y = LaunchConfiguration("press_direction_y")
    press_direction_z = LaunchConfiguration("press_direction_z")
    approach_distance = LaunchConfiguration("approach_distance")
    press_depth = LaunchConfiguration("press_depth")
    start_scene = LaunchConfiguration("start_scene")
    rviz = LaunchConfiguration("rviz")

    gazebo_launch = os.path.join(
        get_package_share_directory("piper_gazebo"),
        "launch", "piper_press", "piper_press_gazebo.launch.py",
    )
    moveit_launch = os.path.join(
        get_package_share_directory("piper_press_moveit"),
        "launch", "piper_press_moveit.launch.py",
    )
    moveit_config = MoveItConfigsBuilder(
        "piper", package_name="piper_press_moveit"
    ).robot_description(mappings={"use_press_tool": "true"}).to_moveit_configs()

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
             "press_direction_x": press_direction_x,
             "press_direction_y": press_direction_y,
             "press_direction_z": press_direction_z,
             "approach_distance": approach_distance,
             "press_depth": press_depth},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("execute", default_value="false",
                              description="Execute the planned press in Gazebo. Defaults to planning only."),
        DeclareLaunchArgument("button_x", default_value="0.300"),
        DeclareLaunchArgument("button_y", default_value="0.000"),
        DeclareLaunchArgument("button_z", default_value="0.240"),
        DeclareLaunchArgument("press_direction_x", default_value="1.0"),
        DeclareLaunchArgument("press_direction_y", default_value="0.0"),
        DeclareLaunchArgument("press_direction_z", default_value="0.0"),
        DeclareLaunchArgument("approach_distance", default_value="0.030",
                              description="Clearance from the surface to the aligned pre-press waypoint."),
        DeclareLaunchArgument("press_depth", default_value="0.004",
                              description="Travel beyond the contact point during the press."),
        DeclareLaunchArgument("start_scene", default_value="true",
                              description="Start Gazebo and MoveIt, or use an already running scene."),
        DeclareLaunchArgument("rviz", default_value="true",
                              description="Start RViz with the five-axis MoveIt scene."),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(gazebo_launch),
                                 launch_arguments={"gui": "false"}.items(),
                                 condition=IfCondition(start_scene)),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(moveit_launch),
            launch_arguments={
                "capabilities": "move_group/ExecuteTaskSolutionCapability",
                "rviz": rviz,
            }.items(),
            condition=IfCondition(start_scene),
        ),
        # Gazebo must spawn the robot and activate its controllers before an MTC
        # task can query current state and execute a FollowJointTrajectory goal.
        TimerAction(period=10.0, actions=[demo_node], condition=IfCondition(start_scene)),
        TimerAction(period=0.5, actions=[demo_node], condition=UnlessCondition(start_scene)),
    ])
