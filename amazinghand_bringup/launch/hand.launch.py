"""
amazinghand_bringup/launch/hand.launch.py

Brings up a single AmazingHand (real hardware or Gazebo sim).

Launch arguments:
  hand_name    (default: right_hand)
  serial_port  (default: /dev/ttyUSB0)
  use_sim      (default: false)
  use_rviz     (default: true)

Example — real right hand:
  ros2 launch amazinghand_bringup hand.launch.py

Example — real left hand:
  ros2 launch amazinghand_bringup hand.launch.py \
      hand_name:=left_hand serial_port:=/dev/ttyUSB1

Example — Gazebo simulation:
  ros2 launch amazinghand_bringup hand.launch.py use_sim:=true
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ------------------------------------------------------------------
    # Declared arguments
    # ------------------------------------------------------------------
    declared_args = [
        DeclareLaunchArgument("hand_name",   default_value="right_hand"),
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("use_sim",     default_value="false",
                              choices=["true", "false"]),
        DeclareLaunchArgument("use_rviz",    default_value="true",
                              choices=["true", "false"]),
    ]

    hand_name   = LaunchConfiguration("hand_name")
    serial_port = LaunchConfiguration("serial_port")
    use_sim     = LaunchConfiguration("use_sim")
    use_rviz    = LaunchConfiguration("use_rviz")

    # ------------------------------------------------------------------
    # URDF / robot description
    # ------------------------------------------------------------------
    robot_description_content = Command([
        FindExecutable(name="xacro"), " ",
        PathJoinSubstitution([
            FindPackageShare("amazinghand_description"),
            "urdf", "hand.urdf.xacro",
        ]),
        " hand_name:=",   hand_name,
        " serial_port:=", serial_port,
        " use_sim:=",     use_sim,
    ])

    robot_description = {"robot_description": robot_description_content}

    # ------------------------------------------------------------------
    # robot_state_publisher
    # ------------------------------------------------------------------
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # ------------------------------------------------------------------
    # ros2_control_node  (real hardware only)
    # ------------------------------------------------------------------
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            PathJoinSubstitution([
                FindPackageShare("amazinghand_bringup"),
                "config",
                [hand_name, ".yaml"],
            ]),
        ],
        output="screen",
        condition=UnlessCondition(use_sim),
    )

    # ------------------------------------------------------------------
    # Spawners — run after controller_manager is ready
    # ------------------------------------------------------------------
    spawn_jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    spawn_ah_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["amazinghand_controller", "--controller-manager", "/controller_manager"],
    )

    # Ensure joint_state_broadcaster is up before spawning the hand controller
    delay_hand_ctrl = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_jsb,
            on_exit=[spawn_ah_controller],
        )
    )

    # ------------------------------------------------------------------
    # RViz
    # ------------------------------------------------------------------
    rviz_config = PathJoinSubstitution([
        FindPackageShare("amazinghand_description"),
        "rviz", "hand.rviz",
    ])

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        condition=IfCondition(use_rviz),
        output="log",
    )

    # ------------------------------------------------------------------
    # Assemble
    # ------------------------------------------------------------------
    return LaunchDescription([
        *declared_args,
        rsp_node,
        ros2_control_node,
        spawn_jsb,
        delay_hand_ctrl,
        rviz_node,
    ])
