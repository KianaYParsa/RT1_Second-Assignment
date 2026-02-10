#!/usr/bin/env python3
"""
Manual Control Node
Publishes user-defined velocity commands to /user_cmd_vel
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class ManualControl(Node):
    def __init__(self):
        super().__init__('manual_control_node')

        self.publisher = self.create_publisher(Twist, '/user_cmd_vel', 10)
        self.get_logger().info("ManualControl ready. Publishing on /user_cmd_vel")

    def run(self):
        while rclpy.ok():
            try:
                input("Press Enter to send a command for 1s, or 'q' to quit: ")
                linear = float(input("Linear velocity (e.g., 0.3): "))
                angular = float(input("Angular velocity (e.g., 0.8): "))

                msg = Twist()
                msg.linear.x = linear
                msg.angular.z = angular

                self.publisher.publish(msg)
                self.get_logger().info("Command sent for 1s, then STOP.")

            except KeyboardInterrupt:
                break


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
