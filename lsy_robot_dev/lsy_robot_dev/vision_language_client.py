from lsy_interfaces.srv import VLService
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class VLClient(Node):

    def __init__(self):
        super().__init__('vision_language_client')
        self.cli = self.create_client(VLService, 'vision_language_client')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = VLService.Request()

        self.image_sub = self.create_subscription(Image, 'camera/color/image_raw', self.image_callback)
        self.image = None
        self.prompt = None
        self.result = None

    def image_callback(self, msg):
        self.image = msg.data

    def send_request(self):
        self.req.image = self.image
        return self.cli.call_async(self.req)
    
    def read_message(self):
        self.get_logger().info(self.result)


def main():
    rclpy.init()

    client = VLClient()
    future = client.send_request()
    rclpy.spin_until_future_complete(client, future)
    client.result = future.result()
    client.read_message()

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()