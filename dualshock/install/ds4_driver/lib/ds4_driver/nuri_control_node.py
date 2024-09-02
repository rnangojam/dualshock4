#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class TeleopJoy(Node):
    def __init__(self):
        super().__init__('teleop_joy')
        self.publisher_ = self.create_publisher(Twist, 'nuri_vel', 10)
        self.get_logger().info('Joystick teleop initialized')
        self.subscription = self.create_subscription(Joy, 'joy', self.joy_callback, 10)

    def joy_callback(self, joy_msg):
        twist = Twist()
        
        # 왼쪽 조이스틱 Y축 - 왼쪽 모터 제어
        twist.linear.x = joy_msg.axes[1] * 0.5  # Scale value as needed

        # 오른쪽 조이스틱 X축 - 오른쪽 모터 제어
        twist.angular.z = joy_msg.axes[3] * 0.5  # Scale value as needed

        self.publisher_.publish(twist)
        self.get_logger().info(f"Publishing Twist: linear.x = {twist.linear.x}, angular.z = {twist.angular.z}")

def main(args=None):
    rclpy.init(args=args)
    node = TeleopJoy()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()