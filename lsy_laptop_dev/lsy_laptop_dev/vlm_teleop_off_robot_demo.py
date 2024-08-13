from lsy_interfaces.srv import VLService
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
# from speech_recognition_msgs.msg import SpeechRecognitionCandidates
from std_msgs.msg import Int32, String

from geometry_msgs.msg import PointStamped, PoseStamped, Twist
# from navigation.NavigateConceptGraph import *
# from stretch_nav2.robot_navigator import BasicNavigator, TaskResult

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from copy import deepcopy

# os.system('pactl set-default-sink alsa_output.pci-0000_00_1f.3.analog-stereo') # ZK added
# os.system('amixer set Master 200%') # ZK added


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
        self.speech_to_text_sub = self.create_subscription(String, "/yay_rode_mic", self.callback_speech, 1) # look into SpeechRecognitionCandidates to see how we can make robot only detect when a human is talking to it
        # self.speech_to_text_sub = self.create_subscription(SpeechRecognitionCandidates, "/speech_to_text", self.callback_speech, 1) ## look into SpeechRecognitionCandidates to see how we can make robot only detect when a human is talking to it

        self.create_subscription(JointState, '/stretch/joint_states', self.joint_states_callback, 1)
        # should we use the following instead of line above?
        # self.joint_state_sub = self.create_subscription(JointState, '/stretch/joint_states', self.joint_states_callback, 1)
        self.image_sub = self.create_subscription(Image, 'camera/color/image_raw', self.image_callback, 1)
        self.target_point_publisher = self.create_publisher(PointStamped, "/clicked_point", 1)

        # Initialize variables related to VLService
        self.vl_cli = self.create_client(VLService, 'vision_language_client')
        while not self.vl_cli.wait_for_service(timeout_sec=1.0):
            print('VLService not available, waiting again')
            #self.get_logger().info('service not available, waiting again...')
        self.vl_req = VLService.Request()
        self.vl_future = None


    ## Callback functions
    def callback_speech(self,msg):
        self.voice_command = msg.data

        if self.voice_command != None:
            self.get_logger().info(f'Voice Command: {self.voice_command}')
            
    def image_callback(self, msg):
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

            ############# OPENAI TTS, WHERE INPUT TO TTS IS vl_result.result #############
            tts_response = self.openai_client.audio.speech.create(model=self.tts_model,
                                                      voice=self.tts_voice,
                                                      input=vl_result.result,
                                                      response_format="wav",
                                                      timeout=60)
            audio_result_path = "./vlm_teleop_openai_tts.wav" # must be .WAV or .OGG
            tts_response.write_to_file(audio_result_path)
            print("File should be saved now.")
            print("Speaking should start now.")
           
            # insert call to python script playing wav file on laptop (instead of using builtin ROS package for robot)
            import pyaudio  
            import wave  
            
            #define stream chunk   
            chunk = 1024  
            
            #open a wav format music  
            f = wave.open(r"./vlm_teleop_openai_tts.wav","rb")  
            #instantiate PyAudio  
            p = pyaudio.PyAudio()  
            #open stream  
            stream = p.open(format = p.get_format_from_width(f.getsampwidth()),  
                            channels = f.getnchannels(),  
                            rate = f.getframerate(),  
                            output = True)  
            #read data  
            data = f.readframes(chunk)  
            
            #play stream  
            while data:  
                stream.write(data)  
                data = f.readframes(chunk)  
            
            #stop stream  
            stream.stop_stream()  
            stream.close()  
            
            #close PyAudio  
            p.terminate() 
            print("Speaking should have ended now.")
            

            ############# REALTIME TTS, WHERE INPUT TO TTS IS vl_result.result #############
            # import pyttsx3 
            # engine = pyttsx3.init()
            # engine.say(vl_result.result)
            # engine.runAndWait()
                        
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