from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("ethercat_master"))
    robot_description = (
        package_share / "urdf" / "ipe_single_joint.urdf"
    ).read_text(encoding="utf-8")
    parameter_file = package_share / "config" / "ipe_joint_state.yaml"

    return LaunchDescription(
        [
            Node(
                package="ethercat_master",
                executable="ipe_joint_state_publisher",
                name="ipe_joint_state_publisher",
                output="screen",
                parameters=[str(parameter_file)],
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description}],
            ),
        ]
    )
