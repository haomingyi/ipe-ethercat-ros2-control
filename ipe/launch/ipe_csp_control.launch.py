from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_name = "ethercat_master"
    package_share = Path(get_package_share_directory(package_name))
    robot_description = (
        package_share / "urdf" / "ipe_single_joint_csp.urdf"
    ).read_text(encoding="utf-8")
    controller_config = package_share / "config" / "ipe_csp_controllers.yaml"

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )
    controller_manager = Node(
        package=package_name,
        executable="ipe_ros2_control_node",
        output="screen",
        parameters=[str(controller_config), {"robot_description": robot_description}],
    )
    joint_state_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )
    position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "ipe_position_controller",
            "--inactive",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            robot_state_publisher,
            controller_manager,
            joint_state_spawner,
            position_controller_spawner,
        ]
    )
