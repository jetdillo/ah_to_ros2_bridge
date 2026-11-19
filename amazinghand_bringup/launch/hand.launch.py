"""
amazinghand_bringup/launch/hand.launch.py

Launch arguments:
  hand_name   (default: right_hand)
  serial_port (default: /dev/ttyUSB0)
  use_sim     (default: false)
  gui         (default: true, Gazebo only)
  rqt         (default: false, Gazebo only)

Example:
  ros2 launch amazinghand_bringup hand.launch.py serial_port:=/dev/ttyACM0
  ros2 launch amazinghand_bringup hand.launch.py hand_name:=left_hand serial_port:=/dev/ttyUSB1
  ros2 launch amazinghand_bringup hand.launch.py use_sim:=true

Not included yet (future work):
  rviz2                  - requires mesh files and rviz config
"""

from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
    Shutdown,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch_ros.substitutions import FindPackageShare
import xacro


def launch_gazebo(context, hand_config, xacro_file, sliders_config):
    hand_name = LaunchConfiguration("hand_name").perform(context)
    use_sliders = IfCondition(LaunchConfiguration("rqt")).evaluate(context)
    if use_sliders:
        controller_config = str(sliders_config.evaluate(context))
        controller = "hand_trajectory_controller"
    else:
        controller_config = hand_config.perform(context)
        controller = "amazinghand_controller"

    robot_description = xacro.process_file(xacro_file.perform(context), mappings={
        "hand_name": hand_name,
        "use_sim": "true",
        "gazebo_controllers": controller_config,
    }).toxml()

    gazebo_args = ["sim", "-r", "empty.sdf"]
    if UnlessCondition(LaunchConfiguration("gui")).evaluate(context):
        gazebo_args.append("-s")

    return [
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        ),
        AppendEnvironmentVariable(
            "GZ_SIM_RESOURCE_PATH", PathJoinSubstitution([
                FindPackageShare("amazinghand_description"), "..",
            ]),
        ),
        ExecuteProcess(
            cmd=[FindExecutable(name="gz"), *gazebo_args],
            output="screen",
            on_exit=Shutdown(),
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description, "use_sim_time": True}],
        ),
        Node(
            package="ros_gz_sim",
            executable="create",
            arguments=["-world", "empty", "-topic", "/robot_description", "-name", hand_name],
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", controller,
                       "--controller-manager", "/controller_manager",
                       "--controller-manager-timeout", "30",
                       "--param-file", controller_config],
        ),
        Node(
            package="rqt_joint_trajectory_controller",
            executable="rqt_joint_trajectory_controller",
            condition=IfCondition(LaunchConfiguration("rqt")),
        ),
    ]


def generate_launch_description():

    declared_args = [
        DeclareLaunchArgument("hand_name", default_value="right_hand",
                              choices=["right_hand", "left_hand"]),
        DeclareLaunchArgument("use_sim", default_value="false"),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("rqt", default_value="false"),
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyUSB0"),
    ]

    use_sim = LaunchConfiguration("use_sim")
    hand_name   = LaunchConfiguration("hand_name")
    serial_port = LaunchConfiguration("serial_port")
    xacro_file = PathJoinSubstitution([
        FindPackageShare("amazinghand_description"), "urdf", "hand.urdf.xacro",
    ])

    robot_description_content = Command([
        FindExecutable(name="xacro"), " ", xacro_file,
        " hand_name:=",   hand_name,
        " serial_port:=", serial_port,
        " use_sim:=false",
    ])

#    robot_description = {"robot_description": robot_description_content}

    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }
    controller_config = PathJoinSubstitution([
        FindPackageShare("amazinghand_bringup"), "config", [hand_name, ".yaml"],
    ])
    state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        condition=UnlessCondition(use_sim),
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_config],
        output="screen",
        condition=UnlessCondition(use_sim),
    )

    spawn_jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        condition=UnlessCondition(use_sim),
    )

    spawn_ah_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["amazinghand_controller", "--controller-manager", "/controller_manager",
                   "--param-file", controller_config],
    )

    delay_hand_ctrl = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_jsb,
            on_exit=[spawn_ah_controller],
        ),
        condition=UnlessCondition(use_sim),
    )

    # Keep the generated parameter file alive while Gazebo and the spawner use it.
    sliders_config = ParameterFile(PathJoinSubstitution([
        FindPackageShare("amazinghand_bringup"), "config", "gazebo_controllers.yaml",
    ]), allow_substs=True)
    gazebo = OpaqueFunction(
        function=launch_gazebo,
        args=[controller_config, xacro_file, sliders_config],
        condition=IfCondition(use_sim),
    )

    return LaunchDescription([
        *declared_args,
        gazebo,
        state_publisher,
        ros2_control_node,
        spawn_jsb,
        delay_hand_ctrl,
    ])
