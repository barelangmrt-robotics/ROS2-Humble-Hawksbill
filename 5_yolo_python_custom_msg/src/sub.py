#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from yolo_python_custom_msg.msg import Vision

class VisionSubscriber(Node):
    def __init__(self):
        super().__init__('vision_sub_node')

        # Membuat subscriber ke topic 'vision_topic' dengan pesan Vision
        self.subscription = self.create_subscription(
            Vision,
            'vision_topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f"Bounding Box : {msg.bounding_box}")
        self.get_logger().info(f"Titik Tengah : {msg.titik_tengah}")
        self.get_logger().info(f"FPS         : {msg.fps}")
        self.get_logger().info('-----------------------------')

def main(args=None):
    rclpy.init(args=args)
    node = VisionSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
