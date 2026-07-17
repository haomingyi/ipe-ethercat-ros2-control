from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_name = "ethercat_master"
    package_share = Path(get_package_share_directory(package_name))
    robot_description = (
        package_share / "urdf" / "ipe_single_joint_three_mode.urdf"
    ).read_text(encoding="utf-8")
    controller_config = package_share / "config" / "ipe_three_mode_controllers.yaml"

    nodes = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package=package_name,
            executable="ipe_ros2_control_node",
            output="screen",
            parameters=[str(controller_config), {"robot_description": robot_description}],
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
            output="screen",
        ),
    ]
    for controller in (
        "ipe_position_controller",
        "ipe_velocity_raw_controller",
        "ipe_torque_raw_controller",
    ):
        nodes.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller, "--inactive", "--controller-manager", "/controller_manager"],
                output="screen",
            )
        )
    return LaunchDescription(nodes)
