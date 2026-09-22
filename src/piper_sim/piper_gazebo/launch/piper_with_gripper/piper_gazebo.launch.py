import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration

from launch.event_handlers import OnProcessExit

import xacro

import re
def remove_comments(text):
    pattern = r'<!--(.*?)-->'
    return re.sub(pattern, '', text, flags=re.DOTALL)

def generate_launch_description():
    robot_name_in_model = 'piper'
    package_name = 'piper_description'
    urdf_name = "piper_description_gazebo.xacro"

    pkg_share = FindPackageShare(package=package_name).find(package_name) 
    urdf_model_path = os.path.join(pkg_share, f'urdf/{urdf_name}')
    gazebo_pkg_share = FindPackageShare(package='piper_gazebo').find('piper_gazebo')
    default_world = os.path.join(gazebo_pkg_share, 'worlds', 'elevator_table.world')

    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Start the Gazebo client in addition to the physics server.'
    )
    gui = LaunchConfiguration('gui')
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='Gazebo world file.'
    )
    world = LaunchConfiguration('world')

    # With GUI, `gazebo` starts both gzserver and gzclient.  For Docker and
    # batch tests, run gzserver alone to keep simulation physics and ROS
    # plugins while avoiding an X11/OpenGL client.
    start_gazebo_cmd =  ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so', world],
        output='screen',
        condition=IfCondition(gui))

    start_gazebo_server_cmd = ExecuteProcess(
        cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so', world],
        output='screen',
        condition=UnlessCondition(gui))


    # 因为 urdf文件中有一句 $(find mybot) 需要用xacro进行编译一下才行
    xacro_file = urdf_model_path
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description': remove_comments(doc.toxml())}

    # 启动了robot_state_publisher节点后，该节点会发布 robot_description 话题，话题内容是模型文件urdf的内容？
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': True}, params, {"publish_frequency":15.0}],
        output='screen'
    )

    # Launch the robot, 通过robot_description话题进行模型内容获取从而在gazebo中生成模型
    spawn_entity_cmd = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-entity', robot_name_in_model,  '-topic', 'robot_description'], output='screen')

    # 关节状态发布器
    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 
             'arm_controller'],
        output='screen'
        )

    load_gripper_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 
             'gripper_controller'],
        output='screen'
        )
    
    load_gripper8_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 
             'gripper8_controller'],
        output='screen'
        )

    close_evt1 =  RegisterEventHandler( 
            event_handler=OnProcessExit(
                target_action=spawn_entity_cmd,
                on_exit=[load_joint_state_controller],
            )
    )

    # Controller manager services can time out when several load requests are
    # sent concurrently on slower software-rendered simulations.  Load them in
    # a deterministic sequence instead.
    close_evt2 = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_joint_trajectory_controller],
            )
    )

    close_evt3 = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_trajectory_controller,
                on_exit=[load_gripper_trajectory_controller],
            )
    )

    close_evt4 = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_gripper_trajectory_controller,
                on_exit=[load_gripper8_trajectory_controller],
            )
    )

    node_gripper_mirror_controller = Node(
        package='piper_gazebo',
        executable='joint8_ctrl.py',
        output='screen'
    )

    ld = LaunchDescription()

    ld.add_action(gui_arg)
    ld.add_action(world_arg)
    ld.add_action(close_evt1)
    ld.add_action(close_evt2)
    ld.add_action(close_evt3)
    ld.add_action(close_evt4)
    ld.add_action(node_gripper_mirror_controller)
    ld.add_action(start_gazebo_cmd)
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(node_robot_state_publisher)
    ld.add_action(spawn_entity_cmd)

    return ld
