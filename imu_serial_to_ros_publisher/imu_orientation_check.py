#!/usr/bin/env python3
"""
imu_orientation_check.py
Interactive sign check for sensor_msgs/Imu.

Guided steps:
  1) Rotate LEFT on the spot: expect gyro.z > +threshold (ENU).
  2) Move FORWARD then stop: expect accel.x > +threshold during start.
  3) Push LEFT then stop: expect accel.y > +threshold during push.

Parameters:
- topic (string): IMU topic to check (default: /imu/data_raw or /imu/data_raw_aligned)
- gyro_thresh (float): rad/s threshold for spin detection (default: 0.5)
- accel_thresh (float): m/s^2 threshold for nudge detection (default: 1.0)
- window_ms (int): rolling window for detection (default: 500)

Run:
  ros2 run edubot imu_orientation_check --ros-args -p topic:=/imu/data_raw_aligned
"""

import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu


class ImuOrientationCheck(Node):
    def __init__(self):
        super().__init__("imu_orientation_check")
        self.declare_parameter("topic", "/imu/data_raw")
        self.declare_parameter("gyro_thresh", 0.5)
        self.declare_parameter("accel_thresh", 1.0)
        self.declare_parameter("window_ms", 500)

        self.topic = self.get_parameter("topic").get_parameter_value().string_value
        self.gyro_thresh = float(self.get_parameter("gyro_thresh").get_parameter_value().double_value)
        self.accel_thresh = float(self.get_parameter("accel_thresh").get_parameter_value().double_value)
        self.window_ms = int(self.get_parameter("window_ms").get_parameter_value().integer_value)

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=50)
        self.sub = self.create_subscription(Imu, self.topic, self.on_imu, qos)

        self.buf_gz = deque()
        self.buf_ax = deque()
        self.buf_ay = deque()
        self.buf_t = deque()

        self.step = 1
        self.last_prompt = 0.0
        self.print_prompt()

    def print_prompt(self):
        if self.step == 1:
            self.get_logger().info("Step 1: Rotate the robot LEFT (counter-clockwise) on the spot for ~1s, then stop.")
        elif self.step == 2:
            self.get_logger().info("Step 2: Nudge the robot FORWARD (x+) briefly, then stop.")
        elif self.step == 3:
            self.get_logger().info("Step 3: Nudge the robot LEFT (y+) briefly, then stop.")
        else:
            self.get_logger().info("All checks completed.")

    def on_imu(self, msg: Imu):
        now = self.get_clock().now().nanoseconds / 1e9
        self.buf_t.append(now)
        self.buf_gz.append(msg.angular_velocity.z)
        self.buf_ax.append(msg.linear_acceleration.x)
        self.buf_ay.append(msg.linear_acceleration.y)

        # Drop old samples outside window
        cutoff = now - (self.window_ms / 1000.0)
        while self.buf_t and self.buf_t[0] < cutoff:
            self.buf_t.popleft()
            self.buf_gz.popleft()
            self.buf_ax.popleft()
            self.buf_ay.popleft()

        if self.step == 1:
            if any(gz > self.gyro_thresh for gz in self.buf_gz):
                self.get_logger().info("PASS: Left spin detected with positive gyro.z.")
                self.step = 2
                self.print_prompt()
            elif any(gz < -self.gyro_thresh for gz in self.buf_gz):
                self.get_logger().warn("FAIL: Detected right spin (gyro.z negative) during left rotation.")
        elif self.step == 2:
            if any(ax > self.accel_thresh for ax in self.buf_ax):
                self.get_logger().info("PASS: Forward acceleration detected with positive accel.x.")
                self.step = 3
                self.print_prompt()
            elif any(ax < -self.accel_thresh for ax in self.buf_ax):
                self.get_logger().warn("FAIL: accel.x negative during forward nudge (check X sign).")
        elif self.step == 3:
            if any(ay > self.accel_thresh for ay in self.buf_ay):
                self.get_logger().info("PASS: Leftward acceleration detected with positive accel.y.")
                self.step = 4
                self.get_logger().info("All sign checks passed.")
            elif any(ay < -self.accel_thresh for ay in self.buf_ay):
                self.get_logger().warn("FAIL: accel.y negative during left nudge (check Y sign).")


def main():
    rclpy.init()
    node = ImuOrientationCheck()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
