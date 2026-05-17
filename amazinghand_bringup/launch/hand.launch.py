"""
amazinghand_bringup/launch/hand.launch.py

Launch arguments:
  hand_name   (default: right_hand)
  serial_port (default: /dev/ttyUSB0)

Example:
  ros2 launch amazinghand_bringup hand.launch.py serial_port:=/dev/ttyACM0
  ros2 launch amazinghand_bringup hand.launch.py hand_name:=left_hand serial_port:=/dev/ttyUSB1

Not included yet (future work):
  robot_state_publisher  - requires mesh files
  rviz2                  - requires mesh files and rviz config
  Gazebo sim             - requires gz_ros2_control wiring
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    declared_args = [
        DeclareLaunchArgument("hand_name",   default_value="right_hand"),
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyUSB0"),
    ]

    hand_name   = LaunchConfiguration("hand_name")
    serial_port = LaunchConfiguration("serial_port")

    robot_description_content = Command([
        FindExecutable(name="xacro"), " ",
        PathJoinSubstitution([
            FindPackageShare("amazinghand_description"),
            "urdf", "hand.urdf.xacro",
        ]),
        " hand_name:=",   hand_name,
        " serial_port:=", serial_port,
        " use_sim:=false",
    ])

#    robot_description = {"robot_description": robot_description_content}

    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }
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
    )

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

    delay_hand_ctrl = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_jsb,
            on_exit=[spawn_ah_controller],
        )
    )

    return LaunchDescription([
        *declared_args,
        ros2_control_node,
        spawn_jsb,
        delay_hand_ctrl,
    ])
