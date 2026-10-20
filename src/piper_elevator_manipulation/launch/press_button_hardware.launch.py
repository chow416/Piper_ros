"""Run the press task against the REAL arm, with a fixed-travel press.

This is the hardware counterpart of press_button_demo.launch.py. It deliberately
follows docs/hardware_readiness.md item 7: a separate launch profile with an
explicit enable gate, rather than retargeting the simulation demo.

Differences from the simulation profile:

  * Gazebo is never started. The arm is expected to already be under
    ros2_control, started separately with:
        ros2 launch piper_hardware piper_real_bringup.launch.py enable_motors:=true
  * use_sim_time is false everywhere.
  * execution_mode is hardware_fixed_travel: there is no contact sensor, so the
    planned trajectory defines the press travel. The independently measured TCP
    travel still acts as a safety cap a few millimetres above the plan.
  * Commanding the arm needs `execute:=true` AND `hardware_enable:=true`. Two
    flags, because reading a target from launch arguments and moving real
    hardware should never be one careless flag apart.

The numbers being tuned -- `contact_standoff_m`, `press_depth`,
`approach_distance` and both gates -- live in `config/press_params.yaml`, which
the node **re-reads before every press**. Edit that file rather than passing the
launch arguments below: it wins when present, and an edit takes effect on the
next press with no restart and no rebuild. The launch arguments remain as the
fallback for when the file is turned off with `press_params_file:=''`.

Note that the planning scene is empty on hardware: the real table and panel are
not in it unless a sensor or a collision object puts them there. Verify the
approach path is clear before enabling.

Start with the defaults, which plan and print without moving anything:

    ros2 launch piper_elevator_manipulation press_button_hardware.launch.py

Then, with the arm enabled by piper_hardware and the workspace clear:

    ros2 launch piper_elevator_manipulation press_button_hardware.launch.py \\
      target_source:=topic max_presses:=0 \\
      button_x:=0.30 button_y:=0.00 button_z:=0.24 \\
      press_direction_x:=1.0 press_direction_y:=0.0 press_direction_z:=0.0

with the standoff, the depth and the two enable gates set in
`config/press_params.yaml` instead of on this command line.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    execute = LaunchConfiguration("execute")
    hardware_enable = LaunchConfiguration("hardware_enable")
    button_x = LaunchConfiguration("button_x")
    button_y = LaunchConfiguration("button_y")
    button_z = LaunchConfiguration("button_z")
    press_direction_x = LaunchConfiguration("press_direction_x")
    press_direction_y = LaunchConfiguration("press_direction_y")
    press_direction_z = LaunchConfiguration("press_direction_z")
    target_source = LaunchConfiguration("target_source")
    target_topic = LaunchConfiguration("target_topic")
    target_wait_timeout = LaunchConfiguration("target_wait_timeout_s")
    target_max_age = LaunchConfiguration("target_max_age_s")
    press_direction_sign = LaunchConfiguration("press_direction_sign")
    max_presses = LaunchConfiguration("max_presses")
    approach_distance = LaunchConfiguration("approach_distance")
    contact_standoff = LaunchConfiguration("contact_standoff_m")
    press_depth = LaunchConfiguration("press_depth")
    travel_margin = LaunchConfiguration("hardware_travel_margin_m")
    start_moveit = LaunchConfiguration("start_moveit")
    scene_wait_s = LaunchConfiguration("scene_wait_s")
    rviz = LaunchConfiguration("rviz")
    free_route_file = LaunchConfiguration("free_route_file")
    press_params_file = LaunchConfiguration("press_params_file")

    moveit_launch = os.path.join(
        get_package_share_directory("piper_press_moveit"),
        "launch",
        "piper_press_moveit.launch.py",
    )
    moveit_config = (
        MoveItConfigsBuilder("piper", package_name="piper_press_moveit")
        .robot_description(mappings={"use_press_tool": "true"})
        .to_moveit_configs()
    )

    demo_node = Node(
        package="piper_elevator_manipulation",
        executable="press_button_demo",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            ParameterFile(free_route_file),
            {
                "use_sim_time": False,
                "execution_mode": "hardware_fixed_travel",
                "execute": ParameterValue(execute, value_type=bool),
                "hardware_enable": ParameterValue(hardware_enable, value_type=bool),
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
                "target_max_age_s": ParameterValue(target_max_age, value_type=float),
                "press_direction_sign": ParameterValue(
                    press_direction_sign, value_type=float
                ),
                "max_presses": ParameterValue(max_presses, value_type=int),
                "approach_distance": approach_distance,
                "contact_standoff_m": contact_standoff,
                "press_depth": press_depth,
                "hardware_travel_margin_m": travel_margin,
                "press_params_file": press_params_file,
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "execute", default_value="false",
            description="Run the validated trajectory. Still needs hardware_enable:=true.",
        ),
        DeclareLaunchArgument(
            "target_source", default_value="parameters",
            choices=["parameters", "topic"],
            description=(
                "'parameters' presses the fixed button_x/y/z point from these launch "
                "arguments. 'topic' waits for geometry_msgs/PoseStamped on "
                "target_topic and presses once per accepted message; see "
                "docs/perception_interface.md."
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
            "target_max_age_s", default_value="2.0",
            description=("Reject a target whose header.stamp is older than this."),
        ),
        DeclareLaunchArgument(
            "press_direction_sign", default_value="1.0",
            description=(
                "+1 uses the target orientation's +Z axis as the press direction. "
                "-1 flips it, for publishers that send an outward surface normal."
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
            "hardware_enable", default_value="false",
            description=(
                "Second gate for commanding the real arm. Requires execute:=true. "
                "Keep the emergency stop within reach."
            ),
        ),
        DeclareLaunchArgument("button_x", default_value="0.300"),
        DeclareLaunchArgument("button_y", default_value="0.000"),
        DeclareLaunchArgument("button_z", default_value="0.240"),
        DeclareLaunchArgument("press_direction_x", default_value="1.0"),
        DeclareLaunchArgument("press_direction_y", default_value="0.0"),
        DeclareLaunchArgument("press_direction_z", default_value="0.0"),
        DeclareLaunchArgument(
            "approach_distance", default_value="0.030",
            description="Clearance from the surface to the aligned pre-press waypoint.",
        ),
        DeclareLaunchArgument(
            "contact_standoff_m", default_value="0.005",
            description=(
                "How far short of the target the approach stops, in metres. "
                "The press phase then covers this plus press_depth, so it "
                "moves where the approach stops, NOT where the press ends -- "
                "the rod still reaches press_depth past the aim point. What "
                "it buys is failure tolerance: any error that puts the aim "
                "point inside the real surface is spent on this gap instead "
                "of on the target, and the gap is something you can see and "
                "put a shim into. Set it to 0 to restore the old behaviour."
            ),
        ),
        DeclareLaunchArgument(
            "press_depth", default_value="0.003",
            description=(
                "Fixed forward travel of the press, measured past the aim "
                "point. There is no contact sensor, so this value IS the stop "
                "condition. On a real button it should be the button's own "
                "travel -- 1 to 2 mm for an elevator call button -- because "
                "anything past that bottoms out and loads the panel."
            ),
        ),
        DeclareLaunchArgument(
            "hardware_travel_margin_m", default_value="0.003",
            description=(
                "Extra travel tolerated above press_depth before the measured TCP "
                "travel aborts the press."
            ),
        ),
        DeclareLaunchArgument(
            "start_moveit", default_value="true",
            description=(
                "Start move_group and RViz. Set false when they are already running, "
                "for example when driving the same controller manager from RViz."
            ),
        ),
        # How long to leave the press node waiting for move_group to come up.
        #
        # The node needs move_group's planning scene, but not at start-up: it
        # only asks for it when a press is planned. So the number that matters
        # is "how long after start-up does the first press arrive", not "how
        # long does move_group take".
        #
        # 10 s suits the one-shot deployment (`max_presses=1` with the target
        # coming from parameters), where the node presses the moment it starts.
        # It is pure dead time when the target arrives on a topic: the operator
        # has to aim first (several seconds of settling) and the waiter in
        # tools/press_board.sh refuses to aim until move_group is alive anyway.
        DeclareLaunchArgument(
            "scene_wait_s", default_value="10.0",
            description=(
                "Seconds to delay the press node so move_group is up first. "
                "Lower it when the target arrives on a topic (the node then "
                "waits for one), not when the node must press immediately."
            ),
        ),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument(
            "free_route_file",
            default_value=os.path.join(
                get_package_share_directory("piper_elevator_manipulation"),
                "config",
                "free_motion_route.yaml",
            ),
            description="YAML file of prerecorded free-motion joint waypoints.",
        ),
        # The one file an operator edits. Its values are re-read before every
        # press, so an edit lands on the next press with no restart and no
        # rebuild -- which is what makes repeated standoff tuning bearable.
        # Pass an empty string to go back to driving these from the launch
        # arguments above.
        DeclareLaunchArgument(
            "press_params_file",
            default_value=os.path.join(
                get_package_share_directory("piper_elevator_manipulation"),
                "config",
                "press_params.yaml",
            ),
            description=(
                "YAML file of press parameters (contact_standoff_m, press_depth, "
                "approach_distance, execute, hardware_enable), re-read before "
                "every press. Empty string disables it and falls back to the "
                "launch arguments."
            ),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(moveit_launch),
            launch_arguments={
                "use_sim_time": "false",
                "rviz": rviz,
            }.items(),
            condition=IfCondition(start_moveit),
        ),
        # Planning itself runs in-process, but wait for move_group anyway so the
        # published solution can be inspected in RViz before anything moves.
        # See `scene_wait_s`: this is a delay in *start-up order*, not a
        # measurement of how long move_group takes.
        TimerAction(period=scene_wait_s, actions=[demo_node],
                    condition=IfCondition(start_moveit)),
        TimerAction(period=1.0, actions=[demo_node],
                    condition=UnlessCondition(start_moveit)),
    ])
