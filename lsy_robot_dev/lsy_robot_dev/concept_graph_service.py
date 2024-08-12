from lsy_interfaces.srv import ConceptGraphService
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import rclpy
from rclpy.node import Node

import numpy as np

def quaternion_to_rotation_matrix(q):
    """
    Convert a quaternion into a rotation matrix.
    
    Parameters:
    - q: A quaternion in the format [x, y, z, w].
    
    Returns:
    - A 3x3 rotation matrix.
    """
    w, x, y, z = q[3], q[0], q[1], q[2]
    return np.array([
        [1 - 2*y**2 - 2*z**2, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
        [2*x*y + 2*z*w, 1 - 2*x**2 - 2*z**2, 2*y*z - 2*x*w],
        [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x**2 - 2*y**2]
    ])

def make_matrix(t: TransformStamped):
        q0 = t.transform.rotation.x
        q1 = t.transform.rotation.y
        q2 = t.transform.rotation.z
        q3 = t.transform.rotation.w
        rotation_matrix = quaternion_to_rotation_matrix([q0, q1, q2, q3])

        transformation_matrix = np.eye(4)  # Create a 4x4 identity matrix
        transformation_matrix[:3, :3] = rotation_matrix  # Set the top-left 3x3 to the rotation matrix
        transformation_matrix[:3, 3] = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]

        return transformation_matrix


class ConceptGraphsHost(Node):

    def __init__(self):
        super().__init__('concept_graph_service')
        self.get_logger().info('STARTING SERVICE...')
        self.rgb_sub = self.create_subscription(Image, 'camera/color/image_raw', self.rgb_callback, 1)
        self.depth_sub = self.create_subscription(Image, 'camera/aligned_depth_to_color/image_raw', self.depth_callback, 1)
        self.intrinsics_sub = self.create_subscription(CameraInfo, '/camera/color/camera_info', self.camera_info_callback, 1)
        self.rgb_sub
        self.depth_sub
        self.intrinsics_sub

        self.rgb = None
        self.depth = None
        self.intrinsics = None

        self.srv = self.create_service(ConceptGraphService, 'concept_graph_service', self.receive_imgs)
        self.srv
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.get_logger().info('SERVICE HAS BEEN STARTED...')

    def receive_imgs(self, request, response):
        # pose code inside timer callback
        from_frame_rel = 'camera_link'
        to_frame_rel = 'map'

        try:
            t = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                rclpy.time.Time())
            response.rgb_image = self.rgb
            response.depth_image = self.depth
            response.camera_info = self.intrinsics
            response.pose = t
        
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
        
            response.rgb_image = None
            response.depth_image = None
            response.intrinsics = None
            response.pose = None
        print('end of callback')
        return response
        
        #self.pose = make_matrix(t)
        #print('GOT POSE, HERE IS THE POSE MATRIX:')
        #print(self.pose)
        #self.have_rgb = False
        #self.have_depth = False
        

    def rgb_callback(self, msg):
        self.rgb = msg

    def depth_callback(self, msg):
        #flatten_depth_img = np.frombuffer(msg.data, dtype=np.uint16)  # shape =(width*height,)
        #self.depth = flatten_depth_img.reshape(msg.height, msg.width) # shape =(width, height)
        self.depth = msg

    def camera_info_callback(self, msg):
        self.intrinsics = msg


def main():
    rclpy.init()
    minimal_service = ConceptGraphsHost()
    rclpy.spin(minimal_service)
    rclpy.shutdown()


if __name__ == '__main__':
    main()