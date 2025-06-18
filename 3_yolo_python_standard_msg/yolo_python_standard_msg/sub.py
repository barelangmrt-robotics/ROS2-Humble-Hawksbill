import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class VisionSubscriber(Node):
    def __init__(self):
        super().__init__('vision_sub_node')

        self.bounding_box = 0
        self.titik_tengah = 0
        self.fps = 0

        self.sub_bbox = self.create_subscription(
            Int32,
            'bounding_box_topic',
            self.bbox_callback,
            10)

        self.sub_center = self.create_subscription(
            Int32,
            'center_topic',
            self.center_callback,
            10)

        self.sub_fps = self.create_subscription(
            Int32,
            'fps_topic',
            self.fps_callback,
            10)

    def bbox_callback(self, msg):
        self.bounding_box = msg.data
        self.print_data()

    def center_callback(self, msg):
        self.titik_tengah = msg.data
        self.print_data()

    def fps_callback(self, msg):
        self.fps = msg.data
        self.print_data()

    def print_data(self):
        # Print all data whenever any callback received new data
        self.get_logger().info(f'Bounding Box : {self.bounding_box}')
        self.get_logger().info(f'Titik Tengah : {self.titik_tengah}')
        self.get_logger().info(f'FPS         : {self.fps}')
        self.get_logger().info('-----------------------------')

def main(args=None):
    rclpy.init(args=args)
    node = VisionSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
