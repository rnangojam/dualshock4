#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

class DS4TestNode(Node):
    def __init__(self):
        super().__init__('ds4_test_node')
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )
        self.prev_buttons = []

    def joy_callback(self, msg):
        # 이전 상태와 현재 상태를 비교하여 변화가 있으면 로그 출력
        if self.prev_buttons != msg.buttons:
            self.log_button_states(msg.buttons)
            self.prev_buttons = msg.buttons

    def log_button_states(self, buttons):
        button_names = [
            "Square", "Triangle", "Circle", "Cross",
            "L1", "R1", "L2", "R2",
            "Share", "Options", "PS", "Trackpad",
            "L3", "R3", "D-Pad Left", "D-Pad Up", 
            "D-Pad Right", "D-Pad Down"
        ]
        for i, state in enumerate(buttons):
            if state:
                self.get_logger().info(f"{button_names[i]} pressed")
            else:
                self.get_logger().info(f"{button_names[i]} released")

def main(args=None):
    rclpy.init(args=args)
    node = DS4TestNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()