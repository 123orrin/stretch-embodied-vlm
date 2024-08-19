# launch the stretch driver launch file beforehand

from std_srvs.srv import TriggerRequest
import rclpy
import time
import hello_helpers.hello_misc as hm

class DummyModeNode(hm.HelloNode):

    def __init__(self):
        hm.HelloNode.__init__(self)
        hm.HelloNode.main(self, 'vlm_teleop', 'vlm_teleop', wait_for_first_pointcloud=False)

     ## Node main
    def main(self):
        while rclpy.ok():
            rclpy.spin_once(self)
            print('switching to position mode')
            self.switch_to_position_mode()
            print('done switching to position mode ')
            print('switching to navigation mode')
            self.switch_to_navigation_mode()
            print('done switching to navigation mode ')

    
    


def main(args=None):
    try:
        #rclpy.init()
        node = DummyModeNode()
        time.sleep(5)
        node.main()
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt. Shutting Down Node...')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


# temp = hm.HelloNode.quick_create('temp')
# temp.switch_to_position_mode_service(TriggerRequest())