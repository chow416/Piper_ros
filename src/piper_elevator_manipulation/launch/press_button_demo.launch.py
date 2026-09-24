from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
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
    target_source = LaunchConfiguration("target_source")
    target_topic = LaunchConfiguration("target_topic")
    target_wait_timeout = LaunchConfiguration("target_wait_timeout_s")
    max_presses = LaunchConfiguration("max_presses")
    approach_distance = LaunchConfiguration("approach_distance")
    press_depth = LaunchConfiguration("press_depth")
    start_scene = LaunchConfiguration("start_scene")
    rviz = LaunchConfiguration("rviz")
    free_route_file = LaunchConfiguration("free_route_file")
    simulated_contact = LaunchConfiguration("simulated_contact")
    virtual_contact_offset_m = LaunchConfiguration("virtual_contact_offset_m")

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
            ParameterFile(free_route_file),
            {"use_sim_time": True,
             "execute": execute,
             "simulated_contact": simulated_contact,
             "button_x": button_x,
             "button_y": button_y,
             "button_z": button_z,
             "press_direction_x": press_direction_x,
             "press_direction_y": press_direction_y,
             "press_direction_z": press_direction_z,
             "target_source": target_source,
             "target_topic": target_topic,
             "target_wait_timeout_s": ParameterValue(
                 target_wait_timeout, value_type=float
             ),
             "max_presses": ParameterValue(max_presses, value_type=int),
             "approach_distance": approach_distance,
             "press_depth": press_depth},
        ],
    )

    virtual_contact_node = Node(
        package="piper_elevator_manipulation",
        executable="virtual_contact_sim",
        output="screen",
        condition=IfCondition(simulated_contact),
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True,
             "button_x": button_x,
             "button_y": button_y,
             "button_z": button_z,
             "press_direction_x": press_direction_x,
             "press_direction_y": press_direction_y,
             "press_direction_z": press_direction_z,
             "virtual_contact_offset_m": virtual_contact_offset_m},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("execute", default_value="false",
                              description="Execute the planned press in Gazebo. Defaults to planning only."),
        DeclareLaunchArgument(
            "simulated_contact", default_value="false",
            description="Explicitly enable the virtual contact source for Gazebo execution only."),
        DeclareLaunchArgument(
            "target_source", default_value="parameters",
            choices=["parameters", "topic"],
            description=(
                "'parameters' presses the fixed button_x/y/z point from these launch "
                "arguments. 'topic' waits for geometry_msgs/PoseStamped on target_topic "
                "and presses once per accepted message; see docs/perception_interface.md."
            ),
        ),
        DeclareLaunchArgument(
            "target_topic", default_value="/press/target_pose",
            description="Topic used when target_source:=topic.",
        ),
        DeclareLaunchArgument(
            "target_wait_timeout_s", default_value="0.0",
            description=(
                "Give up after this many seconds without a valid target when "
                "target_source:=topic. 0 waits forever, reporting rejections."
            ),
        ),
        DeclareLaunchArgument(
            "max_presses", default_value="1",
            description=(
                "Stop after this many presses. 0 keeps the node running and "
                "pressing every accepted target."
            ),
        ),
        DeclareLaunchArgument(
            "virtual_contact_offset_m", default_value="0.0",
            description="Simulation-only contact plane offset from the target point."),
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
        DeclareLaunchArgument(
            "free_route_file",
            default_value=os.path.join(
                get_package_share_directory("piper_elevator_manipulation"),
                "config", "free_motion_route.yaml",
            ),
            description="YAML file of prerecorded free-motion joint waypoints.",
        ),
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
        virtual_contact_node,
        # Gazebo must spawn the robot and activate its controllers before an MTC
        # task can query current state and execute a FollowJointTrajectory goal.
        TimerAction(period=10.0, actions=[demo_node], condition=IfCondition(start_scene)),
        TimerAction(period=0.5, actions=[demo_node], condition=UnlessCondition(start_scene)),
    ])
