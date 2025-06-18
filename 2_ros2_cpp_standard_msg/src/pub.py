#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')
        self.publisher_ = self.create_publisher(String, 'example_topic', 10)
        self.counter = 1
        timer_period = 1.0  # Setiap 1 detik
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        message = f"{self.counter} - Aku Sayang Kamu"
        msg = String()
        msg.data = message
        self.publisher_.publish(msg)
        print(message)
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = SimplePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
