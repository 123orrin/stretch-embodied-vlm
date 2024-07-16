# MOST UP TO DATE CODE, 10-07-2024, 16:15

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
from speech_recognition_msgs.msg import SpeechRecognitionCandidates
from std_msgs.msg import Int32

# Set speaker and volume to activate the speakers
os.system('pactl set-default-sink alsa_output.pci-0000_00_1f.3.analog-stereo')
os.system('amixer set Master 200%')

class Prompt(Enum):
    DESCRIBE = 0
    MOVE = 1

class GetVoiceCommands:
    pass  

class VLClient(Node):
    pass

class VLMTeleop(hm.HelloNode):

    def __init__(self):
        hm.HelloNode.__init__(self)
        hm.HelloNode.main(self, 'vlm_teleop', 'vlm_teleop', wait_for_first_pointcloud=False)

        self.rate = 10.0
        self.joint_state = None

        self.rad_per_deg = math.pi/180.0
        self.rotate = 20 * self.rad_per_deg # radians
        self.translate = 0.05 # meters

        self.soundhandle = SoundClient(self, blocking=True)
        self.openai_key = os.getenv('OPENAI_API_KEY')
        # print("openai_key: ", self.openai_key)
        self.openai_client = OpenAI(api_key=self.openai_key)
        self.tts_model = 'tts-1'
        self.tts_voice = 'alloy'
        self.tts_volume = 1.0

        ## Previously in GetVoiceCommands init
        # Initialize the voice command
        self.voice_command = ' '

        # Initialize the sound direction
        self.sound_direction = 0

        # Initialize subscribers
        self.speech_to_text_sub = self.create_subscription(SpeechRecognitionCandidates, "/speech_to_text", self.callback_speech, 1)
        self.sound_direction_sub = self.create_subscription(Int32, "/sound_direction", self.callback_direction, 1)
        self.create_subscription(JointState, '/stretch/joint_states', self.joint_states_callback, 1)

        ## Previously in VLClient init
        self.cli = self.create_client(VLService, 'vision_language_client')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            print('service not available, waiting again')
            #self.get_logger().info('service not available, waiting again...')
        self.req = VLService.Request()
        self.client_futures = []
        self.future = None

        self.image_sub = self.create_subscription(Image, 'camera/color/image_raw', self.image_callback, 1)
        self.image = None
        self.prompt = None
        self.result = None
    
    # Previously in GetVoiceCommands
    def callback_direction(self, msg):
        #self.sound_direction = msg.data * -self.rad_per_deg
        self.sound_direction = None

    def callback_speech(self,msg):
        self.voice_command = ' '.join(map(str,msg.transcript))
        if self.voice_command != None:
            self.get_logger().info(f'Voice Command: {self.voice_command}')


    def get_inc(self):
        inc = {'rad': self.rotate, 'translate': self.translate}
        return inc

    def print_commands(self):
        """
        A function that prints the voice teleoperation menu.
        :param self: The self reference.
        """
        print('                                           ')
        print('------------ VLM TELEOP MENU ------------')
        print('                                           ')
        print('               VOICE COMMANDS              ')
        print(' "stretch describe": DESCRIBE SCENE        ')
        print(' "stretch move [object]": MOVE TO OBJECT   ')
        print('                                           ')
        print('                                           ')
        print(' "quit"   : QUIT AND CLOSE NODE            ')
        print('                                           ')
        print('-------------------------------------------')

    def get_prompt(self):
        prompt_type = None
        prompt = None
        # Move base forward command
        if 'describe' in self.voice_command:
            prompt_type = Prompt.DESCRIBE
            prompt = 'Describe what you see in the image in a short sentence.'

        # Move base back command
        if 'move' in self.voice_command:
            prompt_type = Prompt.MOVE
            desired_obj = self.voice_command.split(' ')[-1]
            prompt = f'Describe how to move from your current location to the {desired_obj}. Please answer by providing a comma separated array using only a combination of the words in the following list [forward, backward, left, right].'

        # Rotate base right command
        if self.voice_command == 'qwerty':
            prompt = 'Please say potato'
            #command = {'joint': 'rotate_mobile_base', 'inc': -self.get_inc()['rad']}

        # Move base to sound direction command
        if self.voice_command == 'werty':
            prompt = 'Please say potato'
            #command = {'joint': 'translate_mobile_base', 'inc': self.get_inc['translate']}

        if self.voice_command == 'ertyu':
            # Sends a signal to ros to shutdown the ROS interfaces
            self.get_logger().info("done")

            # Exit the Python interpreter
            sys.exit(0)

        # Reset voice command to None
        self.voice_command = ' '

        # return the updated command
        return (prompt_type, prompt)
    

    # Previously in VLClient
    def image_callback(self, msg):
        self.image = msg.data
        # print("W, H: ", msg.width, msg.height)

    # def send_request(self, prompt):
    #     self.req.image = self.image
    #     self.req.prompt = prompt
    #     self.future = self.cli.call_async(self.req)
    #     return self.cli.call_async(self.req)
    
    def read_message(self):
        self.get_logger().info(self.result)


    # Original functions in VLMTeleop
    def _dummy(self):
        pass

    def joint_states_callback(self, msg):
        self.joint_state = msg

    def timer_get_prompt(self):
        # Get voice command
        prompt_type, prompt = self.get_prompt()

        if prompt != None:
            self.get_logger().info(f'Prompt: {prompt}')
        # self.speaker.say('I am Hello Robot.')
        # Send voice command for joint trajectory goals
        self.process_prompt(prompt_type, prompt)

    def process_prompt(self, prompt_type, prompt):
        if prompt is None:
            return

        print('Processing Prompt')
        self.req.image = self.image
        self.req.prompt = prompt
        
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future) ########## HELLO, HI, I'M THE PROBLEM ITS ME
        print('Finished getting VLM answer')
        result = self.future.result()
        self.get_logger().info(f'VLM Result: {result}')
        
        joint_state = self.joint_state


        if prompt_type == Prompt.DESCRIBE:
            print('THIS IS WHAT I SEE')
            tts_response = self.openai_client.audio.speech.create(model=self.tts_model,
                                                      voice=self.tts_voice,
                                                      input=result.result,
                                                      response_format="wav")
            # audio_result_path = "~/lsy_software_tests/vlm_teleop_openai_tts.wav" # must be .WAV or .OGG
            audio_result_path = "/home/hello-robot/lsy_software_tests/vlm_teleop_audio_output/vlm_teleop_openai_tts.wav" # must be .WAV or .OGG
            tts_response.write_to_file(audio_result_path)
            print("File should be saved now.")
            print("Speaking should start now.")
            self.soundhandle.playWave(audio_result_path, self.tts_volume)
            print("Speaking should have ended now.")

        elif prompt_type == Prompt.MOVE and joint_state is not None:
            result = result.result
            words = result.split(', ')
            print(f'Commands: {words}')
            for command in words:
                # Assign point as a JointTrajectoryPoint message prompt_type
                point = JointTrajectoryPoint()
                point.time_from_start = Duration(seconds=0).to_msg()

                # Assign trajectory_goal as a FollowJointTrajectoryGoal message prompt_type
                trajectory_goal = FollowJointTrajectory.Goal()
                trajectory_goal.goal_time_tolerance = Duration(seconds=0).to_msg()

                # Extract the joint name from the command dictionary
                print(f'Command: {command}')
                if command == 'forward':
                    joint_name = 'translate_mobile_base'
                    inc = self.translate
                elif command == 'backward':
                    joint_name = 'translate_mobile_base'
                    inc = -self.translate
                elif command == 'right':
                    joint_name = 'rotate_mobile_base'
                    inc = self.rotate
                elif command == 'left':
                    joint_name = 'rotate_mobile_base'
                    inc = -self.rotate

                trajectory_goal.trajectory.joint_names = [joint_name]
                new_value = inc

                # Assign the new_value position to the trajectory goal message prompt_type
                point.positions = [new_value]
                trajectory_goal.trajectory.points = [point]
                trajectory_goal.trajectory.header.stamp = self.get_clock().now().to_msg()
                #self.get_logger().info('joint_name = {0}, trajectory_goal = {1}'.format(joint_name, trajectory_goal))
                # Make the action call and send goal of the new joint position
                self.trajectory_client.send_goal_async(trajectory_goal)
                print('test')
                time.sleep(5)
                self.get_logger().info('Done sending command.')
            self.get_logger().info('Finished Moving to Desired Object')

    def main(self):
        while rclpy.ok():
            rclpy.spin_once(self)
            prompt_type, prompt = self.get_prompt()
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