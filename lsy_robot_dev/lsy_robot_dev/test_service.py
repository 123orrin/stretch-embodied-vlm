from lsy_interfaces.srv import VLService

import rclpy
from rclpy.node import Node

class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(VLService, 'vision_language_client', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        return 'I see skies of blue, and clouds of white'


def main():
    rclpy.init()
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    rclpy.shutdown()


if __name__ == '__main__':
    main()