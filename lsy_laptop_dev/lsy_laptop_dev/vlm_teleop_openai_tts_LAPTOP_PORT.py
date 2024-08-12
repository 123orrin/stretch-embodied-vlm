from lsy_interfaces.srv import VLService, StretchSpeechService
import hello_helpers.hello_misc as hm

import os
from sound_play.libsoundplay import SoundClient
from sound_play_msgs.msg import SoundRequest
from openai import OpenAI #### ZK added, OpenAI is installed globally

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import sys
import math
from enum import Enum
import time

from sensor_msgs.msg import Image
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from speech_recognition_msgs.msg import SpeechRecognitionCandidates
from std_msgs.msg import Int32, String

from geometry_msgs.msg import PointStamped, PoseStamped, Twist
# from navigation.NavigateConceptGraph import *
# from stretch_nav2.robot_navigator import BasicNavigator, TaskResult

import builtins # noqa: E402, I100
import rosidl_parser.definition # noqa: E402, I100

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from copy import deepcopy

# os.system('pactl set-default-sink alsa_output.pci-0000_00_1f.3.analog-stereo') # ZK added
# os.system('amixer set Master 200%') # ZK added


# class Metaclass_String(type):
#     """Metaclass of message 'String'."""

#     _CREATE_ROS_MESSAGE = None
#     _CONVERT_FROM_PY = None
#     _CONVERT_TO_PY = None
#     _DESTROY_ROS_MESSAGE = None
#     _TYPE_SUPPORT = None

#     __constants = {
#     }

#     @classmethod
#     def __import_type_support__(cls):
#         try:
#             from rosidl_generator_py import import_type_support
#             module = import_type_support('std_msgs')
#         except ImportError:
#             import logging
#             import traceback
#             logger = logging.getLogger(
#                 'std_msgs.msg.String')
#             logger.debug(
#                 'Failed to import needed modules for type support:\n' +
#                 traceback.format_exc())
#         else:
#             cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__string
#             cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__string
#             cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__string
#             cls._TYPE_SUPPORT = module.type_support_msg__msg__string
#             cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__string

#     @classmethod
#     def __prepare__(cls, name, bases, **kwargs):
#         # list constant names here so that they appear in the help text of
#         # the message class under "Data and other attributes defined here:"
#         # as well as populate each message instance
#         return {
#         }


# class String(metaclass=Metaclass_String):
#     """Message class 'String'."""

#     __slots__ = [
#         '_data',
#     ]

#     _fields_and_field_types = {
#         'data': 'string',
#     }

#     SLOT_TYPES = (
#         rosidl_parser.definition.UnboundedString(),  # noqa: E501
#     )

#     def __init__(self, **kwargs):
#         assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
#             'Invalid arguments passed to constructor: %s' % \
#             ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
#         self.data = kwargs.get('data', str())

#     def __repr__(self):
#         typename = self.__class__.__module__.split('.')
#         typename.pop()
#         typename.append(self.__class__.__name__)
#         args = []
#         for s, t in zip(self.__slots__, self.SLOT_TYPES):
#             field = getattr(self, s)
#             fieldstr = repr(field)
#             # We use Python array type for fields that can be directly stored
#             # in them, and "normal" sequences for everything else.  If it is
#             # a type that we store in an array, strip off the 'array' portion.
#             if (
#                 isinstance(t, rosidl_parser.definition.AbstractSequence) and
#                 isinstance(t.value_type, rosidl_parser.definition.BasicType) and
#                 t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
#             ):
#                 if len(field) == 0:
#                     fieldstr = '[]'
#                 else:
#                     assert fieldstr.startswith('array(')
#                     prefix = "array('X', "
#                     suffix = ')'
#                     fieldstr = fieldstr[len(prefix):-len(suffix)]
#             args.append(s[1:] + '=' + fieldstr)
#         return '%s(%s)' % ('.'.join(typename), ', '.join(args))

#     def __eq__(self, other):
#         if not isinstance(other, self.__class__):
#             return False
#         if self.data != other.data:
#             return False
#         return True

#     @classmethod
#     def get_fields_and_field_types(cls):
#         from copy import copy
#         return copy(cls._fields_and_field_types)

#     @builtins.property
#     def data(self):
#         """Message field 'data'."""
#         return self._data

#     @data.setter
#     def data(self, value):
#         if __debug__:
#             assert \
#                 isinstance(value, str), \
#                 "The 'data' field must be of type 'str'"
#         self._data = value



class VLMTeleop(hm.HelloNode):
    def __init__(self):
        hm.HelloNode.__init__(self)
        hm.HelloNode.main(self, 'vlm_teleop', 'vlm_teleop', wait_for_first_pointcloud=False)

        self.rate = 10.0
        self.joint_state = None

        self.rad_per_deg = math.pi/180.0

        self.soundhandle = SoundClient(self, blocking=True)
        self.openai_key = os.getenv('OPENAI_API_KEY')
        self.openai_client = OpenAI(api_key=self.openai_key)
        self.tts_model = 'tts-1'
        self.tts_voice = 'alloy'
        self.tts_volume = 1.0

        # Initialize the voice command, image, and result
        self.voice_command = ' '  # used in callback_speech
        self.image = None  # used in image_callback
        self.result = None  # used in read_message --> not needed? remove if not

        # Initialize subscribers and publishers
        # self.speech_to_text_sub = self.create_subscription(String, "/yayayay", self.callback_speech, 1) # look into SpeechRecognitionCandidates to see how we can make robot only detect when a human is talking to it
        self.speech_to_text_sub = self.create_subscription(SpeechRecognitionCandidates, "/speech_to_text", self.callback_speech, 1) ## look into SpeechRecognitionCandidates to see how we can make robot only detect when a human is talking to it
        # self.speech_to_text_sub = self.create_subscription(String, "/yayayay", self.callback_speech, 1)

        self.create_subscription(JointState, '/stretch/joint_states', self.joint_states_callback, 1)
        # should we use the following instead of line above?
        # self.joint_state_sub = self.create_subscription(JointState, '/stretch/joint_states', self.joint_states_callback, 1)
        self.image_sub = self.create_subscription(Image, 'camera/color/image_raw', self.image_callback, 1)
        self.target_point_publisher = self.create_publisher(PointStamped, "/clicked_point", 1)

        # Initialize variables related to VLService and StretchSpeechService
        self.vl_cli = self.create_client(VLService, 'vision_language_client')
        while not self.vl_cli.wait_for_service(timeout_sec=1.0):
            print('VLService not available, waiting again')
            #self.get_logger().info('service not available, waiting again...')
        self.vl_req = VLService.Request()
        #self.vl_future = None # change to self.vl_cli_future?

        self.llm_response_pub = self.create_publisher(String, "llm_response", 1)
        # self.stretch_speech_cli = self.create_client(StretchSpeechService, 'stretch_speech_client')
        # while not self.stretch_speech_cli.wait_for_service(timeout_sec=1.0):
        #     print('StretchSpeechService not available, waiting again')
        #     #self.get_logger().info('service not available, waiting again...')
        # self.stretch_speech_req = StretchSpeechService.Request()
        # #self.stretch_speech_future = None # use or don't use? change to self.stretch_speech_cli_future?


    ## Callback functions
    def callback_speech(self,msg):
        self.voice_command = ' '.join(map(str,msg.transcript))
        if self.voice_command != None:
            self.get_logger().info(f'Voice Command: {self.voice_command}')
        # self.voice_command = msg.data
        # if self.voice_command != None:
        #     self.get_logger().info(f'Voice Command: {self.voice_command}')
            
    def image_callback(self, msg):
        # print('Made it into image_callback')
        self.image = msg.data
        # print("W, H: ", msg.width, msg.height)
        
    def joint_states_callback(self, msg):
        self.joint_state = msg


    ## Miscellaneous helper functions 
    def get_inc(self):
        inc = {'rad': self.rotate, 'translate': self.translate}
        return inc
    
    def read_message(self):  # not used? remove if not
        self.get_logger().info(self.result)
    
    
    ## Retrieving and processing prompts
    def get_preprompt(self):
        if self.voice_command == ' ':
            return (None, None)
        if 'amy' not in self.voice_command.lower():
            return (None, None)

        prompt_type = None
        user_prompt = self.voice_command ## need to confirm that this is correct

        print('Processing initial query')
        # query = 'You are given three categories: "move", "describe", "chat". From these three categories, output the one that best represents the prompt below. Make sure to use the exact same formatting as above, including keeping the words in lower case. In case of uncertainty, output "chat". \nPrompt: ' + user_prompt
        # query = 'You are given three categories: "describe", "move", "chat". From these three categories, output the one that best represents the prompt below. If you output "chat", please also continue the conversation on a new line. In case of uncertainty, output "chat". /nPrompt: ' + self.user_prompt
        query = 'You are given three categories: "move", "describe", "chat". From these three categories, output the one that best represents the prompt below. Make sure to use the exact same formatting as above, including keeping the words in lower case. In case of uncertainty, output "chat". In addition, output "True" or "False" if you think the prompt below requires you to receive an image to be able to best answer the prompt. If you don\'t have the information required to answer the prompt or if you do not have access to real-time data, you should output "True". Your final output should be in the following format, substituting prompt_type for "move", "describe", "chat", and substituting use_image for "True", "False": prompt_type,use_image \nPrompt: ' + user_prompt
        #If you don\'t have the information required to answer the prompt without an image because your training data was cutoff before that date or because you do not have access to real-time data, you should always output "True".
        print('Initial user_prompt: ', user_prompt)
        self.vl_req.prompt = query
        self.vl_req.image = self.image
        self.vl_req.use_image = False
        self.vl_req.is_preprompt = True
        
        self.vl_cli_future = self.vl_cli.call_async(self.vl_req)
        rclpy.spin_until_future_complete(self, self.vl_cli_future)
        print('Finished getting prompt type from VLM')
        result = self.vl_cli_future.result()
        preprompt_result = result.result.split(',')
        prompt_type = preprompt_result[0]
        self.vl_req.use_image = True if "True" in preprompt_result[1] else False
        self.get_logger().info(f'VLM result.result, aka prompt_type,use_image: {prompt_type, self.vl_req.use_image}')
        
        # Reset voice command to None so that it's ready for next iteration of getting preprompt; OK to do this since we've already saved self.voice_command to user_prompt
        self.voice_command = ' ' # not sure about the best location for this line
        
        return (prompt_type, user_prompt)


    def process_prompt(self, prompt_type, prompt):
        if prompt is None:
            return

        joint_state = self.joint_state

        # Add in later:
        # if 'stretch shut down' in prompt:
        #     self.get_logger().info('Received Shutdown Voice Command. Shutting Down Node...')
        #     self.destroy_node()
        #     rclpy.shutdown()
        self.vl_req.is_preprompt = False
        if prompt_type == 'describe' or prompt_type == 'chat':
            self.vl_req.image = self.image
            self.vl_req.prompt = prompt

            if self.vl_req.use_image:
                print('Mode: send image. Sending user prompt along with image to VLM')
            else:
                print('Mode: don\'t send image. Sending only user prompt to VLM')

            self.vl_cli_future = self.vl_cli.call_async(self.vl_req)
            rclpy.spin_until_future_complete(self, self.vl_cli_future)
            print('Finished getting VLM answer')
            vl_result = self.vl_cli_future.result()
            self.get_logger().info(f'VLM Result: {vl_result}')
            
            msg = String()
            # msg.data = 'Test string'
            msg.data = vl_result.result
            self.llm_response_pub.publish(msg)
            # #test_str = 'Hello, I am OpenAI'
            # # self.stretch_speech_req = test_str
            # self.stretch_speech_req.llm_response_text = vl_result.result
            # print('self.stretch_speech_req: ', self.stretch_speech_req, ' | type(self.stretch_speech_req): ', type(self.stretch_speech_req))
            # print('self.stretch_speech_req.llm_response_text: ', self.stretch_speech_req.llm_response_text, ' | type(self.stretch_speech_req.llm_response_text): ', type(self.stretch_speech_req.llm_response_text))
            # #print('test_str: ', test_str, ' | type(self.stretch_speech_req): ', type(self.stretch_speech_req))
            # self.stretch_speech_cli_future = self.stretch_speech_cli.call_async(self.stretch_speech_req)
            # rclpy.spin_until_future_complete(self, self.stretch_speech_cli_future)
            # print('Finished getting prompt type from VLM')
            # stretch_speech_result = self.stretch_speech_cli_future.result()
            # self.get_logger().info(f'StretchSpeechService Result, tts_processed: {stretch_speech_result}')

            
        elif prompt_type == 'move' and joint_state is not None:
            print('Made it into "move" elif statement.')
            self.vl_req.image = self.image
            self.vl_req.use_image = False # only hard-coded for move mode
            # Get prompt in easy-to-use format from GPT-4o-mini:
            self.vl_req.prompt = 'Reformat the following prompt into two lists with the following format:\nlist_1 = [direction_1, direction_2, ..., direction_i, ..., direction_n] | list_2 = [distance_1, distance_2, ..., distance_i, ..., distance_n]\ndirection_i must be one of four words: forward, backward, left, right. distance_i must be a float value. The unit of distance should be meters. \nPrompt: ' + prompt
            print('VLM Prompt: ', self.vl_req.prompt)
            # list_1 = [direction_1, distance_1), ..., (direction_i, distance_i), ..., (direction_n, distance_n)]
            self.vl_cli_future = self.vl_cli.call_async(self.vl_req)
            rclpy.spin_until_future_complete(self, self.vl_cli_future)
            print('Finished getting VLM answer')
            vl_result = self.vl_cli_future.result()
            self.get_logger().info(f'VLM Result: {vl_result}')
            moves = vl_result.result
            print('Original moves: ', moves)
            moves = moves.replace('list_1 = [', '')
            moves = moves.replace(' list_2 = [', '')
            moves = moves.replace(']', '')
            moves = moves.replace('"', '').replace(' ', '')
            moves = moves.split('|')
            print('Processed moves: ', moves)
            move_directions = moves[0]
            move_distances = moves[1]
            move_directions_list = move_directions.split(', ')
            move_distances.replace(']', '')
            move_distances_list = move_distances.split(', ')

            assert len(move_directions_list) == len(move_distances_list)
 
            for i in range(len(move_directions_list)):
                print('direction_i, distance_i: ', move_directions_list[i], move_distances_list[i])
                if move_distances_list[i] == '':
                    move_distances_list[i] = '0.0'
                # Assign point as a JointTrajectoryPoint message prompt_type
                point = JointTrajectoryPoint()
                point.time_from_start = Duration(seconds=0).to_msg()

                # Assign trajectory_goal as a FollowJointTrajectoryGoal message prompt_type
                trajectory_goal = FollowJointTrajectory.Goal()
                trajectory_goal.goal_time_tolerance = Duration(seconds=0).to_msg()

                print(move_directions_list, move_distances_list)
                if move_directions_list[i] == 'forward' or move_directions_list[i] == 'backward':
                    joint_name = 'translate_mobile_base'
                    inc = float(move_distances_list[i])
                    if move_directions_list[i] == 'backward':
                        inc = -inc

                    trajectory_goal.trajectory.joint_names = [joint_name]
                    new_value = inc
                    # Assign the new_value position to the trajectory goal message prompt_type
                    point.positions = [new_value]
                    trajectory_goal.trajectory.points = [point]
                    trajectory_goal.trajectory.header.stamp = self.get_clock().now().to_msg()
                    #self.get_logger().info('joint_name = {0}, trajectory_goal = {1}'.format(joint_name, trajectory_goal))
                    # Make the action call and send goal of the new joint position
                    self.trajectory_client.send_goal_async(trajectory_goal)
                    time.sleep(5)
                    self.get_logger().info('Done sending linear translation command.')
                    

                elif move_directions_list[i] == 'right' or move_directions_list[i] == 'left':
                    joint_name = 'rotate_mobile_base'
                    inc = -math.pi/2
                    if move_directions_list[i] == 'left':
                        inc = -inc
                    trajectory_goal.trajectory.joint_names = [joint_name]
                    new_value = inc

                    # Assign the new_value position to the trajectory goal message prompt_type
                    point.positions = [new_value]
                    trajectory_goal.trajectory.points = [point]
                    trajectory_goal.trajectory.header.stamp = self.get_clock().now().to_msg()
                    #self.get_logger().info('joint_name = {0}, trajectory_goal = {1}'.format(joint_name, trajectory_goal))
                    # Make the action call and send goal of the new joint position
                    self.trajectory_client.send_goal_async(trajectory_goal)
                    time.sleep(5)
                    self.get_logger().info('Done sending rotation command.')

                    trajectory_goal.trajectory.joint_names = ['translate_mobile_base']
                    inc = float(move_distances_list[i])
                    new_value = inc
                    # Assign the new_value position to the trajectory goal message prompt_type
                    point.positions = [new_value]
                    trajectory_goal.trajectory.points = [point]
                    trajectory_goal.trajectory.header.stamp = self.get_clock().now().to_msg()
                    #self.get_logger().info('joint_name = {0}, trajectory_goal = {1}'.format(joint_name, trajectory_goal))
                    # Make the action call and send goal of the new joint position
                    self.trajectory_client.send_goal_async(trajectory_goal)
                    time.sleep(5)
                    self.get_logger().info('Done sending linear translation command.')

            self.get_logger().info('Finished Target Movement')


            """
            nav = NavigateConceptGraph(system_prompt_path='/home/hello-robot/ament_ws/src/lsy_robot_dev/lsy_robot_dev/navigation/prompts/concept_graphs_planner.txt', scene_json_path='/home/hello-robot/ament_ws/src/lsy_robot_dev/lsy_robot_dev/navigation/obj_json_r_mapping_stride14.json')
            nav_query = prompt.lower().replace("amy", "")
            target_object, target_coords = nav.query_goal(query=nav_query, visual=False, excluded_ids=[])

            print(target_object)
            print(target_coords)
            

            if target_object['object_id'] != -1:

                from_frame_rel = 'odom'
                to_frame_rel = 'map'

                try:
                    t = self.tf_buffer.lookup_transform(
                        to_frame_rel,
                        from_frame_rel,
                        rclpy.time.Time())

                    theta = math.atan2(target_coords[1] - t.transform.translation.y, target_coords[0] - t.transform.translation.x)
                    lin = math.sqrt((target_coords[1] - t.transform.translation.y)**2 + (target_coords[0] - t.transform.translation.x)**2)
                    
                    # twist = Twist()
                    # vel = 0.5
                    # t = theta/vel
                    # twist.angular.z = vel
                    # self.twist_pub.publish(twist)
                    # start = time.time()
                    # while time.time() - start < t:
                    #     pass
                    # twist.angular.z = 0.
                    # self.twist_pub.publish(twist)

                    # t = lin/vel
                    # twist.linear.y = vel
                    # self.twist_pub.publish(twist)
                    # start = time.time()
                    # while time.time() - start < t:
                    #     pass
                    # twist.linear.y = 0
                    # self.twist_pub.publish(twist)   

                    # Assign point as a JointTrajectoryPoint message prompt_type
                    point = JointTrajectoryPoint()
                    point.time_from_start = Duration(seconds=0).to_msg()

                    # Assign trajectory_goal as a FollowJointTrajectoryGoal message prompt_type
                    trajectory_goal = FollowJointTrajectory.Goal()
                    trajectory_goal.goal_time_tolerance = Duration(seconds=0).to_msg()

                    joint_name = 'rotate_mobile_base'
                    inc = theta + math.pi/2

                    trajectory_goal.trajectory.joint_names = [joint_name]
                    new_value = inc
                    # Assign the new_value position to the trajectory goal message prompt_type
                    point.positions = [new_value]
                    trajectory_goal.trajectory.points = [point]
                    trajectory_goal.trajectory.header.stamp = self.get_clock().now().to_msg()
                    #self.get_logger().info('joint_name = {0}, trajectory_goal = {1}'.format(joint_name, trajectory_goal))
                    # Make the action call and send goal of the new joint position
                    self.trajectory_client.send_goal_async(trajectory_goal)
                    time.sleep(5)
                    print('I HAVE TURNED')

                    joint_name = 'translate_mobile_base'
                    inc = lin
                    trajectory_goal.trajectory.joint_names = [joint_name]
                    new_value = inc
                    # Assign the new_value position to the trajectory goal message prompt_type
                    point.positions = [new_value]
                    trajectory_goal.trajectory.points = [point]
                    trajectory_goal.trajectory.header.stamp = self.get_clock().now().to_msg()
                    #self.get_logger().info('joint_name = {0}, trajectory_goal = {1}'.format(joint_name, trajectory_goal))
                    # Make the action call and send goal of the new joint position
                    self.trajectory_client.send_goal_async(trajectory_goal)
                    time.sleep(5)
                    print('I HAVE FORWARDED MYSELF')
                    self.get_logger().info('Finished Moving to Desired Object')    
                
                except TransformException as ex:
                    self.get_logger().info(
                        f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')     
                    
                """
            """
                navigator = BasicNavigator()

                initial_pose = PoseStamped()
                initial_pose.header.frame_id = 'map'
                initial_pose.header.stamp = navigator.get_clock().now().to_msg()
                initial_pose.pose.position.x = 0.0
                initial_pose.pose.position.y = 0.0
                initial_pose.pose.orientation.z = 0.0
                initial_pose.pose.orientation.w = 1.0
                navigator.setInitialPose(initial_pose)

                navigator.waitUntilNav2Active()

                route_poses = []
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.header.stamp = navigator.get_clock().now().to_msg()
                pose.pose.orientation.w = 1.0
                
                pose.pose.position.x = target_coords[0]
                pose.pose.position.y = target_coords[1]
                route_poses.append(deepcopy(pose))

                navigator.followWaypoints(route_poses)

                while not navigator.isTaskComplete():
                    i += 1
                    feedback = navigator.getFeedback()
                    if feedback and i % 5 == 0:
                        navigator.get_logger().info('Executing current waypoint: ' +
                            str(feedback.current_waypoint + 1) + '/' + str(len(route_poses)))
                        
                result = navigator.getResult()
                if result == TaskResult.SUCCEEDED:
                    navigator.get_logger().info('Route complete!')
                elif result == TaskResult.CANCELED:
                    navigator.get_logger().info('Security route was canceled, exiting.')
                elif result == TaskResult.FAILED:
                    navigator.get_logger().info('Security route failed!')
            """

            '''
                msg = PointStamped()
                msg.header.frame_id = '/base_link'

                msg.point.x = target_coords[0]
                msg.point.y = target_coords[1]
                msg.point.z = target_coords[2]

                self.target_point_publisher.publish(msg)
            '''



    ## Node main
    def main(self):
        while rclpy.ok():
            rclpy.spin_once(self)
            prompt_type, prompt = self.get_preprompt()
            self.process_prompt(prompt_type, prompt)


def main(args=None):
    try:
        #rclpy.init()
        node = VLMTeleop()
        time.sleep(5)
        node.main()
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt. Shutting Down Node...')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()