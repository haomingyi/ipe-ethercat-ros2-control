#!/usr/bin/env python3

import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def main() -> int:
    parser = argparse.ArgumentParser(description="Send one IPE joint trajectory goal")
    target = parser.add_mutually_exclusive_group(required=True)
    target.add_argument("--position-rad", type=float, help="absolute ROS joint position")
    target.add_argument("--relative-degrees", type=float, help="relative ROS joint angle")
    parser.add_argument("--seconds", type=float, required=True)
    arguments, ros_arguments = parser.parse_known_args()
    if not math.isfinite(arguments.seconds) or arguments.seconds <= 0.0:
        parser.error("--seconds must be positive")

    rclpy.init(args=[sys.argv[0], *ros_arguments])
    node = Node("ipe_send_joint_goal")
    current_position = None

    def state_callback(message: JointState) -> None:
        nonlocal current_position
        try:
            index = message.name.index("ipe_joint")
        except ValueError:
            return
        if index < len(message.position) and math.isfinite(message.position[index]):
            current_position = message.position[index]

    subscription = node.create_subscription(JointState, "/joint_states", state_callback, 10)
    publisher = node.create_publisher(JointTrajectory, "/ipe/command", 10)
    if arguments.relative_degrees is not None:
        deadline = time.monotonic() + 5.0
        while rclpy.ok() and current_position is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if current_position is None:
            node.get_logger().error("No ipe_joint feedback received within 5 seconds")
            node.destroy_node()
            rclpy.try_shutdown()
            return 2
        goal_position = current_position + math.radians(arguments.relative_degrees)
    else:
        goal_position = arguments.position_rad
    if not math.isfinite(goal_position):
        parser.error("target position must be finite")

    message = JointTrajectory()
    message.joint_names = ["ipe_joint"]
    point = JointTrajectoryPoint()
    point.positions = [goal_position]
    seconds = int(arguments.seconds)
    point.time_from_start.sec = seconds
    point.time_from_start.nanosec = int((arguments.seconds - seconds) * 1e9)
    message.points = [point]

    discovery_deadline = time.monotonic() + 5.0
    while publisher.get_subscription_count() == 0 and time.monotonic() < discovery_deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if publisher.get_subscription_count() == 0:
        node.get_logger().error("No reference manager is subscribed to /ipe/command")
        node.destroy_node()
        rclpy.try_shutdown()
        return 3
    publisher.publish(message)
    rclpy.spin_once(node, timeout_sec=0.1)
    node.get_logger().info(
        f"goal={goal_position:.6f} rad duration={arguments.seconds:.3f}s"
    )
    node.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
