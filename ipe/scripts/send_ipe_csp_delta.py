#!/usr/bin/env python3

import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class DeltaCommandNode(Node):
    def __init__(self) -> None:
        super().__init__("ipe_csp_delta_command")
        self.position = None
        self.subscription = self.create_subscription(
            JointState, "/joint_states", self._joint_state, 10
        )
        self.publisher = self.create_publisher(
            Float64MultiArray, "/ipe_position_controller/commands", 10
        )

    def _joint_state(self, message: JointState) -> None:
        try:
            index = message.name.index("ipe_joint")
        except ValueError:
            return
        if index < len(message.position) and math.isfinite(message.position[index]):
            self.position = message.position[index]


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Send one safety-limited relative CSP command to ipe_joint"
    )
    parser.add_argument(
        "--degrees",
        type=float,
        required=True,
        help="relative ROS angle, range +/-1 degree (standalone CSP controller)",
    )
    arguments, ros_arguments = parser.parse_known_args()
    if (
        not math.isfinite(arguments.degrees)
        or not 0.0 < abs(arguments.degrees) <= 1.0
    ):
        parser.error("--degrees must be nonzero and within +/-1")

    rclpy.init(args=[sys.argv[0], *ros_arguments])
    node = DeltaCommandNode()
    deadline = time.monotonic() + 5.0
    while rclpy.ok() and node.position is None and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.position is None:
        node.get_logger().error("No valid ipe_joint position received within 5 seconds")
        node.destroy_node()
        rclpy.shutdown()
        return 2

    target = node.position + math.radians(arguments.degrees)
    message = Float64MultiArray(data=[target])
    node.get_logger().info(
        f"current={node.position:.9f} rad delta={arguments.degrees:.4f} deg "
        f"target={target:.9f} rad"
    )
    for _ in range(10):
        node.publisher.publish(message)
        rclpy.spin_once(node, timeout_sec=0.05)

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
