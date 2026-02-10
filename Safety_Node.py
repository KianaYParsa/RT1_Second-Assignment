#!/usr/bin/env python3
"""
Safety Node
Filters user velocity commands based on obstacle distance
Publishes safe commands to /safe_cmd_vel
"""

import math
from collections import deque

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

from rt2_interfaces.srv import SetThreshold, GetAvgVel
from rt2_interfaces.msg import ObstacleInfo


class SafetyNode(Node):

    def __init__(self):
        super().__init__('safety_node')

        self.threshold = 0.7
        self.latest_min_dist = float('inf')
        self.last_user_cmd = Twist()

        self.vel_history = deque(maxlen=50)

        # Subscribers
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Twist, '/user_cmd_vel', self.user_cmd_callback, 10)

        # Publishers
        self.pub_safe = self.create_publisher(Twist, '/safe_cmd_vel', 10)
        self.pub_info = self.create_publisher(ObstacleInfo, '/obstacle_info', 10)

        # Services
        self.create_service(SetThreshold, '/set_threshold', self.set_threshold_cb)
        self.create_service(GetAvgVel, '/get_avg_vel', self.get_avg_vel_cb)

        self.timer = self.create_timer(0.05, self.control_loop)
        self.log_timer = self.create_timer(0.5, self.publish_info)

        self.get_logger().info("SafetyNode started.")

    def scan_callback(self, msg: LaserScan):
        valid = [
            r for r in msg.ranges
            if math.isfinite(r) and msg.range_min <= r <= msg.range_max
        ]
        if valid:
            self.latest_min_dist = min(valid)

    def user_cmd_callback(self, msg: Twist):
        self.last_user_cmd = msg

    def control_loop(self):
        out = Twist()

        if self.latest_min_dist < self.threshold:
            out.linear.x = 0.0
            out.angular.z = 0.5
        else:
            out = self.last_user_cmd

        self.pub_safe.publish(out)
        self.vel_history.append(out)

    def publish_info(self):
        msg = ObstacleInfo()
        msg.closest_distance = self.latest_min_dist
        msg.threshold = self.threshold
        msg.direction = "front"
        self.pub_info.publish(msg)

    def set_threshold_cb(self, request, response):
        self.threshold = request.threshold
        response.ok = True
        response.message = f"Threshold set to {self.threshold:.3f} m"
        return response

    def get_avg_vel_cb(self, request, response):
        if not self.vel_history:
            response.avg_linear = 0.0
            response.avg_angular = 0.0
            response.window_size = 0
            return response

        response.avg_linear = sum(v.linear.x for v in self.vel_history) / len(self.vel_history)
        response.avg_angular = sum(v.angular.z for v in self.vel_history) / len(self.vel_history)
        response.window_size = len(self.vel_history)
        return response


def main():
    rclpy.init()
    node = SafetyNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
