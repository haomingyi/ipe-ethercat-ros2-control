#!/usr/bin/env python3

import math
import select
import sys
import time

import rclpy
from control_msgs.msg import DynamicJointState
from controller_manager_msgs.srv import SwitchController
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


CONTROLLER = "ipe_torque_raw_controller"
OTHER_MOTION_CONTROLLERS = [
    "ipe_position_controller",
    "ipe_velocity_raw_controller",
]
COMMAND_TOPIC = f"/{CONTROLLER}/commands"
SWITCH_SERVICE = "/controller_manager/switch_controller"
COMMAND_LIMIT = 300
TESTED_BOUNDARY = 100
PUBLISH_PERIOD = 0.01


class CstSessionNode(Node):
    def __init__(self) -> None:
        super().__init__("ipe_cst_session")
        self.position = math.nan
        self.velocity_raw = math.nan
        self.torque_raw = math.nan
        self.publisher = self.create_publisher(
            Float64MultiArray, COMMAND_TOPIC, 10
        )
        self.subscription = self.create_subscription(
            DynamicJointState,
            "/dynamic_joint_states",
            self._dynamic_state,
            10,
        )
        self.switch_client = self.create_client(
            SwitchController, SWITCH_SERVICE
        )

    def _dynamic_state(self, message: DynamicJointState) -> None:
        try:
            joint_index = message.joint_names.index("ipe_joint")
        except ValueError:
            return
        if joint_index >= len(message.interface_values):
            return
        interfaces = message.interface_values[joint_index]
        values = dict(zip(interfaces.interface_names, interfaces.values))
        self.position = values.get("position", self.position)
        self.velocity_raw = values.get("velocity_raw", self.velocity_raw)
        self.torque_raw = values.get("torque_raw", self.torque_raw)

    def switch(
        self, activate: list[str], deactivate: list[str], strictness: int
    ) -> bool:
        if not self.switch_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(
                f"{SWITCH_SERVICE} is unavailable; start three-mode control first"
            )
            return False
        request = SwitchController.Request()
        request.activate_controllers = activate
        request.deactivate_controllers = deactivate
        request.strictness = strictness
        request.activate_asap = True
        request.timeout.sec = 5
        future = self.switch_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=7.0)
        response = future.result()
        if response is None or not response.ok:
            message = response.message if response is not None else "service timeout"
            self.get_logger().error(f"Controller switch failed: {message}")
            return False
        return True

    def publish(self, value: int) -> None:
        self.publisher.publish(Float64MultiArray(data=[float(value)]))

    def print_status(self, desired: int) -> None:
        print(
            f"desired={desired} raw "
            f"position={self.position:.6f} rad "
            f"velocity_raw={self.velocity_raw:.0f} "
            f"torque_raw={self.torque_raw:.0f}"
        )


def parse_value(words: list[str], command: str, desired: int) -> int | None:
    if len(words) != 2:
        print(f"Usage: {command} <integer>")
        return None
    try:
        value = int(words[1], 10)
    except ValueError:
        print("Torque command must be an integer raw value.")
        return None
    target = value if command == "set" else desired + value
    if abs(target) > COMMAND_LIMIT:
        print(f"Rejected: target must be within +/-{COMMAND_LIMIT} raw.")
        return None
    return target


def main() -> int:
    rclpy.init(args=sys.argv)
    node = CstSessionNode()
    desired = 0
    active = False
    interrupted = False

    try:
        active = node.switch(
            [CONTROLLER],
            OTHER_MOTION_CONTROLLERS,
            SwitchController.Request.BEST_EFFORT,
        )
        if not active:
            return 2

        discovery_deadline = time.monotonic() + 3.0
        while (
            rclpy.ok()
            and node.publisher.get_subscription_count() == 0
            and time.monotonic() < discovery_deadline
        ):
            rclpy.spin_once(node, timeout_sec=0.05)
        if node.publisher.get_subscription_count() == 0:
            node.get_logger().error(
                f"No subscriber on {COMMAND_TOPIC}; controller is not ready"
            )
            return 3

        print(
            "CST ROS session started with a zero raw target.\n"
            "Commands: set <raw>, add <raw>, zero, status, help, stop.\n"
            f"Hard limit +/-{COMMAND_LIMIT} raw; values above the tested "
            f"+/-{TESTED_BOUNDARY} raw range produce a warning."
        )
        print("cst> ", end="", flush=True)
        next_publish = time.monotonic()

        while rclpy.ok():
            now = time.monotonic()
            if now >= next_publish:
                node.publish(desired)
                rclpy.spin_once(node, timeout_sec=0.0)
                next_publish += PUBLISH_PERIOD
                if next_publish < now:
                    next_publish = now + PUBLISH_PERIOD

            wait = max(0.0, min(PUBLISH_PERIOD, next_publish - time.monotonic()))
            readable, _, _ = select.select([sys.stdin], [], [], wait)
            if not readable:
                continue
            line = sys.stdin.readline()
            if not line:
                break
            words = line.strip().lower().split()
            if not words:
                print("cst> ", end="", flush=True)
                continue

            command = words[0]
            if command in ("set", "add"):
                target = parse_value(words, command, desired)
                if target is not None:
                    desired = target
                    print(f"target={desired} raw")
                    if abs(desired) > TESTED_BOUNDARY:
                        print(
                            f"Warning: command exceeds the tested +/-{TESTED_BOUNDARY} raw range; "
                            "continuously monitor speed, current, and temperature."
                        )
            elif command == "zero":
                desired = 0
                print("Target set to zero raw; the hardware is ramping down.")
            elif command == "status":
                node.print_status(desired)
            elif command == "help":
                print(
                    "set <raw>: set target; add <raw>: change current target;\n"
                    "zero: ramp to zero while CST stays enabled; status: show feedback;\n"
                    "stop: ramp to zero, disable, and exit."
                )
            elif command in ("stop", "quit", "exit"):
                break
            else:
                print("Unknown command; type help.")
            print("cst> ", end="", flush=True)
    except KeyboardInterrupt:
        interrupted = True
        print("\nCtrl+C received; zeroing and disabling.")
    finally:
        if active:
            zero_wait = min(abs(desired) * 0.02 + 0.3, 6.5)
            deadline = time.monotonic() + zero_wait
            while rclpy.ok() and time.monotonic() < deadline:
                node.publish(0)
                rclpy.spin_once(node, timeout_sec=PUBLISH_PERIOD)
            node.switch(
                [],
                [CONTROLLER],
                SwitchController.Request.BEST_EFFORT,
            )
        node.destroy_node()
        rclpy.try_shutdown()

    print("CST target is zero and the controller is disabled.")
    return 130 if interrupted else 0


if __name__ == "__main__":
    raise SystemExit(main())
