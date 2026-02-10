#!/usr/bin/env python3
"""
Manual Control Node (Final)
Publishes user-defined velocity commands to /user_cmd_vel for exactly 1 second,
then publishes a STOP command.

This node only publishes user commands and does not perform any safety checks.
Safety is handled by a dedicated Safety Node.
"""

import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class ManualControl(Node):
    def __init__(self):
        super().__init__('manual_control_node')

        # Publisher for user commands (raw, unsafe by definition)
        self.publisher = self.create_publisher(Twist, '/user_cmd_vel', 10)

        self.get_logger().info("ManualControl ready. Publishing on /user_cmd_vel")

    def send_for_one_second(self, msg: Twist):
        """
        Publish the given Twist repeatedly for 1 second (at ~20 Hz),
        then publish a STOP (zero Twist).
        """
        start_time = time.time()

        # Publish the command continuously for 1 second
        while (time.time() - start_time) < 1.0:
            self.publisher.publish(msg)
            time.sleep(0.05)  # 20 Hz

        # After 1 second, send a STOP command to avoid drifting
        stop = Twist()
        self.publisher.publish(stop)

    def run(self):
        """
        Simple terminal-based interface:
        - Press Enter to send a command
        - Type 'q' to quit
        """
        while rclpy.ok():
            try:
                cmd = input("Press Enter to send command, or 'q' to quit: ").strip().lower()
                if cmd == 'q':
                    break

                # Read desired velocities from user input
                linear = float(input("Linear velocity (e.g., 0.3): "))
                angular = float(input("Angular velocity (e.g., 0.8): "))

                # Build the Twist message
                msg = Twist()
                msg.linear.x = linear
                msg.angular.z = angular

                self.get_logger().info("Sending command for 1 second...")
                self.send_for_one_second(msg)

            except KeyboardInterrupt:
                break
            except ValueError:
                self.get_logger().warn("Invalid input. Please enter numeric values.")


def main():
    rclpy.init()
    node = ManualControl()

    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
