"""Bring up ros2_control for a physical Piper arm on SocketCAN.

This launch starts the controller manager with piper_hardware/PiperSystem. It
intentionally does not start MoveIt: bring this up first, confirm the arm is
readable and that a small command moves it, and only then start the planning
stack.

Motors stay disabled unless `enable_motors:=true` is passed, so the default run
exercises the whole software chain without moving the arm.
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from pathlib import Path


def _process_state(pid):
    """Single-letter process state from /proc/<pid>/stat, or None.

    The comm field is wrapped in parentheses and may itself contain spaces and
    parentheses, so the reliable split is on the *last* ')'.
    """
    try:
        stat = Path(f"/proc/{pid}/stat").read_text()
        return stat.rsplit(")", 1)[1].split()[0]
    except (OSError, IndexError):
        return None


def _running_controller_managers():
    """PIDs of live ros2_control_node processes.

    Reads /proc/<pid>/comm rather than the full command line: comm is just the
    executable name, so a shell whose arguments happen to contain the string
    cannot be mistaken for the real process. (That mistake is easy to make with
    `pkill -f`, which happily matches the shell running it.)

    Zombies are skipped. A process that has exited but not been reaped still has
    a /proc entry and still matches on comm, but it owns no CAN socket and
    cannot answer a service call, so treating it as "in use" would block a
    perfectly good launch.
    """
    found = []
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        try:
            comm = (entry / "comm").read_text().strip()
        except (OSError, PermissionError):
            continue
        # comm is truncated to 15 characters.
        if not comm.startswith("ros2_control_no"):
            continue
        if _process_state(entry.name) in ("Z", "X", "x"):
            continue
        found.append(entry.name)
    return found


def _refuse_if_controller_manager_already_runs(context):
    """Abort the launch when another controller manager owns the CAN bus.

    Two controller managers both write joint commands to can0 every 5 ms, so
    the arm receives two contradictory setpoints per cycle. The visible symptom
    is a control problem -- sluggish motion, tens of degrees of tracking lag --
    and the spawners below additionally fail with confusing messages
    ("Failed loading controller", "Controller already loaded, skipping
    load_controller") because they query one manager and configure the other.

    Refusing here is much better than debugging that afterwards.
    """
    del context  # not needed: the check is about the machine, not the arguments
    pids = _running_controller_managers()
    if not pids:
        return []
    raise RuntimeError(
        "refusing to start: a ros2_control_node is already running "
        "(pid " + ", ".join(pids) + ").\n"
        "Only one controller manager may own the CAN bus. Kill the existing "
        "one first:\n"
        "    kill -9 " + " ".join(pids) + "\n"
        "Use kill -9 rather than a plain kill: a graceful exit runs the "
        "hardware interface's deactivate hook and that disables the motors.\n"
        "Afterwards run go_home.py, because nothing is driving the arm until "
        "this launch comes back up."
    )


def generate_launch_description():
    can_interface = LaunchConfiguration("can_interface")
    enable_motors = LaunchConfiguration("enable_motors")
    use_press_tool = LaunchConfiguration("use_press_tool")
    command_speed_ratio = LaunchConfiguration("command_speed_ratio")
    feedback_timeout = LaunchConfiguration("feedback_timeout")

    robot_description = ParameterValue(
        Command(
            [
                FindExecutable(name="xacro"),
                " ",
                PathJoinSubstitution(
                    [
                        get_package_share_directory("piper_hardware"),
                        "urdf",
                        "piper_real.urdf.xacro",
                    ]
                ),
                " use_press_tool:=", use_press_tool,
                " can_interface:=", can_interface,
                " enable_motors_on_activate:=", enable_motors,
                " command_speed_ratio:=", command_speed_ratio,
                " feedback_timeout:=", feedback_timeout,
            ]
        ),
        value_type=str,
    )

    controllers_file = PathJoinSubstitution(
        [
            get_package_share_directory("piper_press_moveit"),
            "config",
            "ros2_controllers.yaml",
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[{"robot_description": robot_description}, controllers_file],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # Activating the arm controller before the broadcaster is running would make
    # the controller fail on a missing /joint_states subscription.
    delay_arm_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner],
        )
    )

    return LaunchDescription(
        [
            # Must come first: nothing may start when another controller
            # manager is already driving the arm.
            OpaqueFunction(function=_refuse_if_controller_manager_already_runs),
            DeclareLaunchArgument(
                "can_interface", default_value="can0",
                description="SocketCAN interface the arm is connected to.",
            ),
            DeclareLaunchArgument(
                "enable_motors", default_value="false",
                description=(
                    "Enable the joint motors during activation. Defaults to false so the "
                    "software chain can be exercised without moving the arm."
                ),
            ),
            DeclareLaunchArgument(
                "use_press_tool", default_value="true",
                description="Use the five-axis press tool instead of the gripper.",
            ),
            DeclareLaunchArgument(
                "command_speed_ratio", default_value="100",
                description="Arm-side speed percentage (1-100). Lower is safer.",
            ),
            DeclareLaunchArgument(
                "feedback_timeout", default_value="0.5",
                description="Seconds without arm feedback before reporting a hardware failure.",
            ),
            robot_state_publisher,
            control_node,
            joint_state_broadcaster_spawner,
            delay_arm_controller,
        ]
    )
