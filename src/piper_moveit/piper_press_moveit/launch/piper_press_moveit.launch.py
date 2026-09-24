from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder

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


def _running_move_groups():
    """PIDs of live move_group processes.

    Matches on /proc/<pid>/comm rather than the command line, because comm is
    just the executable name and cannot be confused with a shell that merely
    mentions it. Zombies are skipped: they still have a /proc entry but cannot
    answer an action call, so counting them would block a good launch.
    """
    found = []
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        try:
            comm = (entry / "comm").read_text().strip()
        except (OSError, PermissionError):
            continue
        if comm != "move_group":
            continue
        if _process_state(entry.name) in ("Z", "X", "x"):
            continue
        found.append(entry.name)
    return found


def _refuse_if_move_group_already_runs(context):
    """Abort when another move_group is already up.

    Two move_groups publish separate monitored_planning_scene and
    planning_scene streams, and the task node then talks to one while having
    inspected the other. That is not a crash, it is worse: everything looks
    normal and the plan is silently based on a scene nobody is maintaining.

    This is not hypothetical. On 2026-09-24 a leftover move_group from a dry
    run sat alongside the real one, and the next run behaved as if the target
    had never been sent.
    """
    del context  # the check is about the machine, not the launch arguments
    pids = _running_move_groups()
    if not pids:
        return []
    raise RuntimeError(
        "refusing to start: a move_group is already running (pid "
        + ", ".join(pids) + ").\n"
        "Two of them publish competing planning scenes and the task node will "
        "silently plan against the wrong one. Kill the existing one first:\n"
        "    kill -9 " + " ".join(pids) + "\n"
        "Then start this launch again."
    )


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("piper", package_name="piper_press_moveit")
        .robot_description(mappings={"use_press_tool": "true"})
        .robot_description_semantic(file_path="config/piper.srdf")
        .to_moveit_configs()
    )

    # Checked before anything else starts; see _refuse_if_move_group_already_runs.
    guard = OpaqueFunction(function=_refuse_if_move_group_already_runs)

    capabilities = LaunchConfiguration("capabilities")
    start_rviz = LaunchConfiguration("rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
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
        parameters=[moveit_config.to_dict(),
                    {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}],
        condition=IfCondition(start_rviz),
    )

    return LaunchDescription([
        guard,
        DeclareLaunchArgument(
            "capabilities",
            default_value="",
            description="Additional MoveGroup capabilities separated by spaces.",
        ),
        DeclareLaunchArgument(
            "rviz", default_value="true", description="Start MoveIt RViz.",
        ),
        DeclareLaunchArgument(
            "use_sim_time", default_value="true",
            description="Use the /clock topic. Set false for the real arm.",
        ),
        move_group,
        rviz,
    ])
