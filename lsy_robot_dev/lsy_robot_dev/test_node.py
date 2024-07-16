import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image

class Test(Node):
    def __init__(self):
        super().__init__('test')
        self.rgb = self.create_subscription(Image, 'camera/color/image_raw', self.rgb_cb, 1)
        self.depth = self.create_subscription(Image, 'camera/aligned_depth_to_color/image_raw', self.depth_cb, 1)
        self.rgb
        self.depth

    def rgb_cb(self, msg):
        print('RGB')

    def depth_cb(self, msg):
        print('DEPTH')

if __name__ == '__main__':
    rclpy.init()
    node = Test()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
