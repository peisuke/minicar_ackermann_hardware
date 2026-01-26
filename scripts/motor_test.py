#!/usr/bin/env python3
"""
Simple motor/servo test script for Ackermann hardware.

Usage:
  ros2 run minicar_ackermann_hardware motor_test.py

This script sends test commands to verify ESC and servo operation.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time


class MotorTestNode(Node):
    def __init__(self):
        super().__init__('motor_test_node')

        self.publisher = self.create_publisher(
            Twist,
            '/real_robot/ackermann_steering_controller/reference_unstamped',
            10
        )

        self.get_logger().info('Motor test node started')
        self.get_logger().info('Press Ctrl+C to stop')

    def send_command(self, linear_x: float, angular_z: float, duration: float):
        """Send a velocity command for specified duration."""
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z

        self.get_logger().info(
            f'Sending: linear={linear_x:.2f} m/s, angular={angular_z:.2f} rad/s '
            f'for {duration:.1f}s'
        )

        start_time = time.time()
        while time.time() - start_time < duration:
            self.publisher.publish(msg)
            time.sleep(0.02)  # 50Hz

    def stop(self):
        """Send stop command."""
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0

        self.get_logger().info('Stopping')
        for _ in range(10):
            self.publisher.publish(msg)
            time.sleep(0.02)

    def run_test_sequence(self):
        """Run a test sequence."""
        try:
            self.get_logger().info('=== Starting test sequence ===')

            # Test 1: Steering left
            self.get_logger().info('Test 1: Steering left')
            self.send_command(0.0, 0.5, 2.0)

            # Test 2: Steering right
            self.get_logger().info('Test 2: Steering right')
            self.send_command(0.0, -0.5, 2.0)

            # Test 3: Center steering
            self.get_logger().info('Test 3: Center steering')
            self.send_command(0.0, 0.0, 1.0)

            # Test 4: Forward slow
            self.get_logger().info('Test 4: Forward slow')
            self.send_command(0.3, 0.0, 2.0)

            # Test 5: Forward with turn
            self.get_logger().info('Test 5: Forward with turn')
            self.send_command(0.3, 0.3, 2.0)

            # Stop
            self.stop()

            self.get_logger().info('=== Test sequence complete ===')

        except KeyboardInterrupt:
            self.stop()
            self.get_logger().info('Test interrupted')


def main(args=None):
    rclpy.init(args=args)

    node = MotorTestNode()

    try:
        node.run_test_sequence()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
