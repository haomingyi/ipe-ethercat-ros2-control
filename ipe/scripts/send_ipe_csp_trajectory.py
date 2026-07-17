#!/usr/bin/env python3

import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


JOINT_NAME = "ipe_joint"
COMMAND_TOPIC = "/ipe_position_controller/commands"
MAX_RELATIVE_DEGREES = 36000.0
MAX_OUTPUT_RPM = 27.0
COMMAND_RATE_HZ = 100.0


class CspTrajectoryNode(Node):
    def __init__(self) -> None:
        super().__init__("ipe_csp_trajectory_command")
        self.position = None
        self.subscription = self.create_subscription(
            JointState, "/joint_states", self._joint_state, 10
        )
        self.publisher = self.create_publisher(
            Float64MultiArray, COMMAND_TOPIC, 10
        )

    def _joint_state(self, message: JointState) -> None:
        try:
            index = message.name.index(JOINT_NAME)
        except ValueError:
            return
        if index < len(message.position) and math.isfinite(message.position[index]):
            self.position = message.position[index]


def positive_finite(value: float, name: str) -> float:
    if not math.isfinite(value) or value <= 0.0:
        raise argparse.ArgumentTypeError(f"{name} must be greater than zero")
    return value


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Move ipe_joint by a relative angle using the project ROS 2 "
            "trajectory sender"
        )
    )
    parser.add_argument(
        "--degrees",
        type=float,
        required=True,
        help="relative ROS angle, nonzero and within +/-36000 degrees",
    )
    timing = parser.add_mutually_exclusive_group()
    timing.add_argument(
        "--rpm",
        type=lambda value: positive_finite(float(value), "--rpm"),
        default=5.0,
        help="output-flange speed magnitude in rpm (default: 5, maximum: 27)",
    )
    timing.add_argument(
        "--seconds",
        type=lambda value: positive_finite(float(value), "--seconds"),
        help="trajectory duration; use this instead of --rpm",
    )
    arguments, ros_arguments = parser.parse_known_args()

    if (
        not math.isfinite(arguments.degrees)
        or not 0.0 < abs(arguments.degrees) <= MAX_RELATIVE_DEGREES
    ):
        parser.error(
            f"--degrees must be nonzero and within +/-{MAX_RELATIVE_DEGREES:g}"
        )

    if arguments.seconds is None:
        if arguments.rpm > MAX_OUTPUT_RPM:
            parser.error(f"--rpm must not exceed {MAX_OUTPUT_RPM:g}")
        duration = abs(arguments.degrees) / (arguments.rpm * 6.0)
    else:
        duration = arguments.seconds
        requested_rpm = abs(arguments.degrees) / (duration * 6.0)
        if requested_rpm > MAX_OUTPUT_RPM:
            parser.error(
                f"angle/time requests {requested_rpm:.3f} rpm; "
                f"maximum is {MAX_OUTPUT_RPM:g} rpm"
            )

    if duration < 0.1:
        parser.error("trajectory duration must be at least 0.1 seconds")

    rclpy.init(args=[sys.argv[0], *ros_arguments])
    node = CspTrajectoryNode()

    deadline = time.monotonic() + 5.0
    while rclpy.ok() and node.position is None and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.position is None:
        node.get_logger().error(
            "No valid ipe_joint position received within 5 seconds"
        )
        node.destroy_node()
        rclpy.shutdown()
        return 2

    target = node.position + math.radians(arguments.degrees)
    effective_rpm = abs(arguments.degrees) / (duration * 6.0)
    node.get_logger().info(
        f"current={node.position:.9f} rad, delta={arguments.degrees:.3f} deg, "
        f"target={target:.9f} rad, duration={duration:.3f} s, "
        f"flange_speed={effective_rpm:.3f} rpm"
    )

    start = node.position
    steps = max(1, math.ceil(duration * COMMAND_RATE_HZ))
    start_time = time.monotonic()
    completed = True
    for index in range(1, steps + 1):
        if not rclpy.ok():
            completed = False
            break
        fraction = index / steps
        command = start + (target - start) * fraction
        node.publisher.publish(Float64MultiArray(data=[command]))
        rclpy.spin_once(node, timeout_sec=0.0)
        deadline = start_time + index / COMMAND_RATE_HZ
        remaining = deadline - time.monotonic()
        if remaining > 0.0:
            time.sleep(remaining)

    if completed:
        for _ in range(10):
            node.publisher.publish(Float64MultiArray(data=[target]))
            rclpy.spin_once(node, timeout_sec=0.01)
        node.get_logger().info("Trajectory command completed")
    else:
        node.get_logger().warning(
            "Trajectory command interrupted; the controller holds the last "
            "published position"
        )

    node.destroy_node()
    rclpy.try_shutdown()
    return 0 if completed else 130


if __name__ == "__main__":
    raise SystemExit(main())
