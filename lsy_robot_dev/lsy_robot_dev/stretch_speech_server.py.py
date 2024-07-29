from lsy_interfaces.srv import StretchSpeechService

import rclpy
from rclpy.node import Node

import os
from sound_play.libsoundplay import SoundClient
from sound_play_msgs.msg import SoundRequest
from openai import OpenAI

os.system('pactl set-default-sink alsa_output.pci-0000_00_1f.3.analog-stereo') # ZK added
os.system('amixer set Master 200%') # ZK added

class StretchSpeechServer(Node):

    def __init__(self):
        super().__init__('stretch_speech_service')
        self.get_logger().info('StretchSpeechServer Node Initialized')
        self.srv = self.create_service(StretchSpeechService, 'stretch_speech_client', self.stretch_speech_callback)
        
        self.soundhandle = SoundClient(self, blocking=True)
        self.openai_key = os.getenv('OPENAI_API_KEY')
        self.openai_client = OpenAI(api_key=self.openai_key)
        self.tts_model = 'tts-1'
        self.tts_voice = 'alloy'
        self.tts_volume = 1.0
        
        self.audio_result_path = '/home/hello-robot/lsy_software_tests/vlm_teleop_audio_output/vlm_teleop_openai_tts.wav' # must be .WAV or .OGG


    def stretch_speech_callback(self, request, response):
        try:
            print(request.llm_response_text)
            tts_response = self.openai_client.audio.speech.create(model=self.tts_model,
                                                        voice=self.tts_voice,
                                                        input=request.llm_response_text,
                                                        response_format="wav")
            tts_response.write_to_file(self.audio_result_path)
            print("File should be saved now.")
            print("Speaking should start now.")
            self.soundhandle.playWave(self.audio_result_path, self.tts_volume)
            print("Speaking should have ended now.")
            response.tts_processed = True

        except:
            response.tts_processed = False
            print('An exception occured.')

        return response


def main():
    rclpy.init()
    stretch_speech_service = StretchSpeechServer
    rclpy.spin(stretch_speech_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

##########################################################################################

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
from std_msgs.msg import Int32

os.system('pactl set-default-sink alsa_output.pci-0000_00_1f.3.analog-stereo') # ZK added
os.system('amixer set Master 200%') # ZK added

class VLMTeleop(hm.HelloNode):

    def __init__(self):
        hm.HelloNode.__init__(self)
        hm.HelloNode.main(self, 'vlm_teleop', 'vlm_teleop', wait_for_first_pointcloud=False)

        self.rate = 10.0
        self.joint_state = None

        self.rad_per_deg = math.pi/180.0
        # self.rotate = 20 * self.rad_per_deg # radians
        # self.translate = 0.05 # meters

        self.soundhandle = SoundClient(self, blocking=True)
        self.openai_key = os.getenv('OPENAI_API_KEY')
        # print("openai_key: ", self.openai_key)
        self.openai_client = OpenAI(api_key=self.openai_key)
        self.tts_model = 'tts-1'
        self.tts_voice = 'alloy'
        self.tts_volume = 1.0

        # Initialize the voice command
        self.voice_command = ' ' 

        # Initialize the sound direction
        self.sound_direction = 0 # not used?

        # Initialize subscribers
        self.speech_to_text_sub = self.create_subscription(String, "/yayayay", self.callback_speech, 1) ## look into SpeechRecognitionCandidates to see how we can make robot only detect when a human is talking to it
        # self.speech_to_text_sub = self.create_subscription(SpeechRecognitionCandidates, "/speech_to_text", self.callback_speech, 1) ## look into SpeechRecognitionCandidates to see how we can make robot only detect when a human is talking to it

        self.sound_direction_sub = self.create_subscription(Int32, "/sound_direction", self.callback_direction, 1)
        self.create_subscription(JointState, '/stretch/joint_states', self.joint_states_callback, 1)
        # should we use the following instead of line above?
        # self.joint_state_sub = self.create_subscription(JointState, '/stretch/joint_states', self.joint_states_callback, 1)
        self.image_sub = self.create_subscription(Image, 'camera/color/image_raw', self.image_callback, 1)
        self.target_point_publisher = self.create_publisher(PointStamped, "/clicked_point", 1)

        ###### Copy-pasted from is_speaking.py ######
        self.robot_sound_sub = self.create_subscription(GoalStatusArray, '~/robotsound', self.robot_sound_callback, 1)
        self.is_speaking = False
        self.pub_speech_flag = self.create_publisher(
            Bool, '~/output/is_speaking', 1)
        self.is_speaking_timer = self.create_timer(0.01, self.speech_timer_cb)
        #############################################

        # Initialize variables related to VLService
        self.vl_cli = self.create_client(VLService, 'vision_language_client')
        while not self.vl_cli.wait_for_service(timeout_sec=1.0):
            print('VLService not available, waiting again')
            #self.get_logger().info('service not available, waiting again...')
        self.vl_req = VLService.Request()
        self.vl_future = None

        #self.user_prompt = None
        self.image = None
        self.result = None
        #self.is_speaking = False  # initialize variable


    ## Callback functions
    def callback_direction(self, msg):
        #self.sound_direction = msg.data * -self.rad_per_deg
        self.sound_direction = None

    # def is_speaking_callback(self, msg):
    #     self.is_speaking = msg.data
    #     print('is_speeching msg.data (equiv. self.is_speaking): ', msg.data)
    #     #if self.is_speaking:
    #         #time.sleep(20)


    def callback_speech(self,msg):
        if not self.is_speaking: # might be redundant with same logic that's now in get_preprompt
            # self.voice_command = ' '.join(map(str,msg.transcript))
            self.voice_command = msg.data

            if self.voice_command != None:
                self.get_logger().info(f'Voice Command: {self.voice_command}')
            
    def image_callback(self, msg):
        self.image = msg.data
        # print("W, H: ", msg.width, msg.height)
        
    def joint_states_callback(self, msg):
        self.joint_state = msg

    ###### Copy-pasted from is_speaking.py ######
    def check_speak_status(self, status_msg):
        """Returns True when speaking.

        """
        # If it is not included in the terminal state,
        # it is determined as a speaking state.
        if status_msg.status in [GoalStatus.STATUS_ACCEPTED,
                                 GoalStatus.STATUS_EXECUTING]:
            return True
        return False

    def robot_sound_callback(self, msg):
        for status in msg.status_list:
            if self.check_speak_status(status):
                self.is_speaking = True
                return
        self.is_speaking = False

    def speech_timer_cb(self):
        self.pub_speech_flag.publish(
            Bool(data=self.is_speaking))
    #############################################

    ## Miscellaneous helper functions 
    def get_inc(self):
        inc = {'rad': self.rotate, 'translate': self.translate}
        return inc
    
    
    def read_message(self):
        self.get_logger().info(self.result)
    
    
    ## Retrieving and processing prompts
    def get_preprompt(self):
        #print('#######\nENTERED get_preprompt()\nself.is_speaking: ', self.is_speaking, '\nself.voice_command: ', self.voice_command, '\n#######')
        if self.is_speaking:
            return (None, None)
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

            """
            if prompt_type == 'describe':
                print('Mode: describe. Sending user prompt along with image to VLM')
                self.vl_req.use_image = True
            else:
                print('Mode: chat. Sending only user prompt to VLM')
                self.vl_req.use_image = False
            """
            if self.vl_req.use_image:
                print('Mode: send image. Sending user prompt along with image to VLM')
            else:
                print('Mode: don\'t send image. Sending only user prompt to VLM')

            self.vl_cli_future = self.vl_cli.call_async(self.vl_req)
            rclpy.spin_until_future_complete(self, self.vl_cli_future)
            print('Finished getting VLM answer')
            vl_result = self.vl_cli_future.result()
            self.get_logger().info(f'VLM Result: {vl_result}')
            
            tts_response = self.openai_client.audio.speech.create(model=self.tts_model,
                                                      voice=self.tts_voice,
                                                      input=vl_result.result,
                                                      response_format="wav")
            audio_result_path = "/home/hello-robot/lsy_software_tests/vlm_teleop_audio_output/vlm_teleop_openai_tts.wav" # must be .WAV or .OGG
            tts_response.write_to_file(audio_result_path)
            print("File should be saved now.")
            print("Speaking should start now.")
            self.soundhandle.playWave(audio_result_path, self.tts_volume)
            print("Speaking should have ended now.")
            
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