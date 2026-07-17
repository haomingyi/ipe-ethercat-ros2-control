#!/usr/bin/env python3

import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


LIMITS = {"csv": 13107200.0, "cst": 300.0}
TOPICS = {
    "csv": "/ipe_velocity_raw_controller/commands",
    "cst": "/ipe_torque_raw_controller/commands",
}


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Send a short, auto-zeroing raw CSV or CST command"
    )
    parser.add_argument("--mode", choices=("csv", "cst"), required=True)
    parser.add_argument("--value", type=float, required=True)
    parser.add_argument("--seconds", type=float, default=0.25)
    arguments, ros_arguments = parser.parse_known_args()
    limit = LIMITS[arguments.mode]
    if not math.isfinite(arguments.value) or not 0.0 < abs(arguments.value) <= limit:
        parser.error(f"--value must be nonzero and within +/-{limit:g}")
    if not math.isfinite(arguments.seconds) or not 0.05 <= arguments.seconds <= 1.0:
        parser.error("--seconds must be within 0.05..1.0")

    rclpy.init(args=[sys.argv[0], *ros_arguments])
    node = Node(f"ipe_{arguments.mode}_raw_pulse")
    publisher = node.create_publisher(Float64MultiArray, TOPICS[arguments.mode], 10)
    discovery_deadline = time.monotonic() + 5.0
    while (
        rclpy.ok()
        and publisher.get_subscription_count() == 0
        and time.monotonic() < discovery_deadline
    ):
        rclpy.spin_once(node, timeout_sec=0.05)
    if publisher.get_subscription_count() == 0:
        node.get_logger().error(
            f"No subscriber on {TOPICS[arguments.mode]}; is its controller active?"
        )
        node.destroy_node()
        rclpy.shutdown()
        return 2
    command = Float64MultiArray(data=[arguments.value])
    zero = Float64MultiArray(data=[0.0])
    node.get_logger().info(
        f"{arguments.mode} raw={arguments.value:g} for {arguments.seconds:.3f}s, then zero"
    )
    deadline = time.monotonic() + arguments.seconds
    while rclpy.ok() and time.monotonic() < deadline:
        publisher.publish(command)
        rclpy.spin_once(node, timeout_sec=0.02)
    for _ in range(20):
        publisher.publish(zero)
        rclpy.spin_once(node, timeout_sec=0.02)
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
