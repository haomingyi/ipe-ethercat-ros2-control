from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    description_share = Path(get_package_share_directory("ipe_description"))
    bringup_share = Path(get_package_share_directory("ipe_bringup"))
    xacro_file = description_share / "urdf" / "ipe_single_joint.urdf.xacro"
    controller_config = bringup_share / "config" / "controllers.yaml"
    reference_config = bringup_share / "config" / "reference_manager.yaml"

    interface = LaunchConfiguration("interface")
    zero_count = LaunchConfiguration("zero_count")
    direction = LaunchConfiguration("direction")
    velocity_scale = LaunchConfiguration("velocity_raw_to_rad_s")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    robot_description = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            str(xacro_file),
            " interface:=",
            interface,
            " zero_count:=",
            zero_count,
            " direction:=",
            direction,
            " velocity_raw_to_rad_s:=",
            velocity_scale,
            " use_mock_hardware:=",
            use_mock_hardware,
        ]
    )

    actions = [
        DeclareLaunchArgument("interface", default_value="enp130s0"),
        DeclareLaunchArgument("zero_count", default_value="130336"),
        DeclareLaunchArgument("direction", default_value="-1"),
        DeclareLaunchArgument(
            "velocity_raw_to_rad_s", default_value="2.3731138426448658e-7"
        ),
        DeclareLaunchArgument("use_mock_hardware", default_value="false"),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="ethercat_master",
            executable="ipe_ros2_control_node",
            output="screen",
            parameters=[str(controller_config), {"robot_description": robot_description}],
        ),
        Node(
            package="ipe_control",
            executable="reference_manager",
            output="screen",
            parameters=[str(reference_config)],
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
            output="screen",
        ),
    ]

    for controller in (
        "ipe_cst_impedance_controller",
        "ipe_position_controller",
        "ipe_velocity_raw_controller",
        "ipe_torque_raw_controller",
    ):
        actions.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller, "--inactive", "--controller-manager", "/controller_manager"],
                output="screen",
            )
        )
    return LaunchDescription(actions)
