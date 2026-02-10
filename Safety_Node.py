#!/usr/bin/env python3
"""
Safety Node (Final)
Filters user velocity commands based on obstacle distance (LaserScan).
Publishes safe commands to /cmd_vel so Gazebo/bridge can consume them.

Topics:
- Subscribes:
  - /scan         (sensor_msgs/msg/LaserScan)
  - /user_cmd_vel (geometry_msgs/msg/Twist)
- Publishes:
  - /cmd_vel       (geometry_msgs/msg/Twist)   [SAFE output]
  - /obstacle_info (rt2_interfaces/msg/ObstacleInfo)

Services:
- /set_threshold (rt2_interfaces/srv/SetThreshold)
  Change the safety distance threshold at runtime.
- /get_avg_vel   (rt2_interfaces/srv/GetAvgVel)
  Returns average linear/angular velocity over the most recent 5 SAFE outputs.
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

        # Safety distance threshold (meters)
        self.threshold = 0.7

        # Latest scan and minimum obstacle distance (meters)
        self.latest_scan: LaserScan | None = None
        self.latest_min_dist = float('inf')

        # Latest user command (raw input)
        self.last_user_cmd = Twist()

        # Keep last 5 SAFE outputs for the average velocity service
        self.vel_history = deque(maxlen=5)

        # Subscribers
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Twist, '/user_cmd_vel', self.user_cmd_callback, 10)

        # Publishers
        # IMPORTANT: Gazebo bridge listens on /cmd_vel (not /safe_cmd_vel)
        self.pub_safe = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_info = self.create_publisher(ObstacleInfo, '/obstacle_info', 10)

        # Services
        self.create_service(SetThreshold, '/set_threshold', self.set_threshold_cb)
        self.create_service(GetAvgVel, '/get_avg_vel', self.get_avg_vel_cb)

        # Timers:
        # - control loop at 20 Hz (0.05s)
        # - info publishing at ~3.3 Hz (0.3s)
        self.timer = self.create_timer(0.05, self.control_loop)
        self.info_timer = self.create_timer(0.3, self.publish_info)

        self.get_logger().info("SafetyNode started. Publishing SAFE commands on /cmd_vel")

    # Callbacks
    def scan_callback(self, msg: LaserScan):
        """
        Store latest scan and compute the minimum valid distance.
        We ignore NaNs/Infs and values outside [range_min, range_max].
        """
        self.latest_scan = msg

        valid_ranges = [
            r for r in msg.ranges
            if math.isfinite(r) and msg.range_min <= r <= msg.range_max
        ]

        if valid_ranges:
            self.latest_min_dist = min(valid_ranges)
        else:
            # If nothing valid, treat as "no obstacle detected"
            self.latest_min_dist = float('inf')

    def user_cmd_callback(self, msg: Twist):
        """Store the latest user command (raw input)."""
        self.last_user_cmd = msg

    # Safety logic
    def control_loop(self):
        """
        Main safety filtering loop:
        - If too close to obstacle: override with backward + small turn.
        - Else: forward user's command unchanged.
        The output is published to /cmd_vel for Gazebo/bridge.
        """
        out = Twist()

        if self.latest_min_dist < self.threshold:
            # Unsafe: override user input with recovery behavior
            out.linear.x = -0.2   # move backward
            out.angular.z = 0.3   # small rotation (helps escape)
        else:
            # Safe: forward the user's command as-is
            out = self.last_user_cmd

        # Publish safe output and store it for average velocity computation
        self.pub_safe.publish(out)
        self.vel_history.append(out)

    def compute_direction(self) -> str:
        """
        Compute obstacle direction (left/front/right) from the latest LaserScan.
        Strategy:
        - Split scan into 3 sectors (right / front / left).
        - Compute min valid range in each sector.
        - Direction = sector with smallest distance.
        """
        if self.latest_scan is None or not self.latest_scan.ranges:
            return "front"

        ranges = self.latest_scan.ranges
        n = len(ranges)

        # Split into 3 roughly equal sectors
        right = ranges[: int(n * 0.33)]
        front = ranges[int(n * 0.33) : int(n * 0.66)]
        left = ranges[int(n * 0.66) :]

        def min_valid(arr):
            vals = [r for r in arr if math.isfinite(r)]
            return min(vals) if vals else float('inf')

        m_right = min_valid(right)
        m_front = min_valid(front)
        m_left = min_valid(left)

        m = min(m_left, m_front, m_right)
        if m == m_left:
            return "left"
        if m == m_right:
            return "right"
        return "front"

    def publish_info(self):
        """
        Publish the custom ObstacleInfo message for monitoring/debugging.
        Includes:
        - closest_distance
        - threshold
        - direction
        """
        info = ObstacleInfo()
        info.closest_distance = float(self.latest_min_dist)
        info.threshold = float(self.threshold)
        info.direction = self.compute_direction()
        self.pub_info.publish(info)

    # Services
    def set_threshold_cb(self, request, response):
        """Service callback to update the safety threshold."""
        self.threshold = float(request.threshold)
        response.ok = True
        response.message = f"Threshold set to {self.threshold:.3f} m"
        return response

    def get_avg_vel_cb(self, request, response):
        """
        Service callback that returns the average linear and angular velocity
        over the last 5 SAFE outputs (window_size <= 5).
        """
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
