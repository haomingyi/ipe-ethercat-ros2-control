#!/usr/bin/env python3

import bisect
import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory


class ReferenceManager(Node):
    """Turns a standard JointTrajectory into a continuous CST reference stream."""

    def __init__(self) -> None:
        super().__init__("ipe_reference_manager")
        self.declare_parameter("joint_name", "ipe_joint")
        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("command_topic", "/ipe/command")
        self.declare_parameter(
            "reference_topic", "/ipe_cst_impedance_controller/reference"
        )
        self.joint_name = self.get_parameter("joint_name").value
        rate = float(self.get_parameter("publish_rate").value)
        if rate <= 0.0:
            raise ValueError("publish_rate must be positive")

        self.actual_position = None
        self.actual_velocity = 0.0
        self.hold_position = None
        self.trajectory = []
        self.trajectory_start = 0.0

        self.reference_publisher = self.create_publisher(
            JointState, self.get_parameter("reference_topic").value, 10
        )
        self.state_subscription = self.create_subscription(
            JointState, "/joint_states", self._state_callback, 10
        )
        self.command_subscription = self.create_subscription(
            JointTrajectory,
            self.get_parameter("command_topic").value,
            self._command_callback,
            10,
        )
        self.timer = self.create_timer(1.0 / rate, self._publish_reference)

    def _state_callback(self, message: JointState) -> None:
        try:
            index = message.name.index(self.joint_name)
        except ValueError:
            return
        if index < len(message.position) and math.isfinite(message.position[index]):
            self.actual_position = message.position[index]
            if self.hold_position is None:
                self.hold_position = self.actual_position
        if index < len(message.velocity) and math.isfinite(message.velocity[index]):
            self.actual_velocity = message.velocity[index]

    @staticmethod
    def _duration_seconds(duration) -> float:
        return float(duration.sec) + float(duration.nanosec) * 1e-9

    def _command_callback(self, message: JointTrajectory) -> None:
        if self.joint_name not in message.joint_names or not message.points:
            self.get_logger().error("Trajectory must contain ipe_joint and at least one point")
            return
        joint_index = message.joint_names.index(self.joint_name)
        start_position = (
            self.actual_position if self.actual_position is not None else self.hold_position
        )
        if start_position is None:
            self.get_logger().error("No joint feedback is available; trajectory rejected")
            return

        points = [(0.0, start_position, self.actual_velocity, 0.0)]
        previous_time = 0.0
        for point in message.points:
            if joint_index >= len(point.positions):
                self.get_logger().error("Every trajectory point needs a joint position")
                return
            stamp = self._duration_seconds(point.time_from_start)
            position = point.positions[joint_index]
            if (
                not math.isfinite(position)
                or stamp <= previous_time
                or stamp <= 0.0
            ):
                self.get_logger().error("Trajectory times must increase and values must be finite")
                return
            velocity = math.nan
            if joint_index < len(point.velocities) and math.isfinite(
                point.velocities[joint_index]
            ):
                velocity = point.velocities[joint_index]
            points.append((stamp, position, velocity, 0.0))
            previous_time = stamp

        self.trajectory = points
        self.trajectory_start = time.monotonic()
        self.get_logger().info(
            f"Accepted {len(points) - 1} point trajectory, duration={previous_time:.3f}s"
        )

    def _sample(self) -> tuple[float, float, float]:
        if not self.trajectory:
            return self.hold_position, 0.0, 0.0
        elapsed = time.monotonic() - self.trajectory_start
        times = [point[0] for point in self.trajectory]
        if elapsed >= times[-1]:
            final = self.trajectory[-1]
            self.hold_position = final[1]
            self.trajectory = []
            return self.hold_position, 0.0, final[3]

        upper_index = bisect.bisect_right(times, elapsed)
        lower = self.trajectory[upper_index - 1]
        upper = self.trajectory[upper_index]
        fraction = (elapsed - lower[0]) / (upper[0] - lower[0])
        position = lower[1] + fraction * (upper[1] - lower[1])
        segment_velocity = (upper[1] - lower[1]) / (upper[0] - lower[0])
        if math.isfinite(upper[2]):
            velocity = upper[2]
        else:
            velocity = segment_velocity
        feedforward = lower[3] + fraction * (upper[3] - lower[3])
        return position, velocity, feedforward

    def _publish_reference(self) -> None:
        if self.hold_position is None:
            return
        position, velocity, _ = self._sample()
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = [self.joint_name]
        message.position = [position]
        message.velocity = [velocity]
        self.reference_publisher.publish(message)


def main() -> None:
    rclpy.init()
    node = ReferenceManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
