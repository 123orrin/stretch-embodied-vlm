# MOST UP TO DATE CODE, 10-07-2024, 16:15

from lsy_interfaces.srv import VLService
from lsy_interfaces.srv import PrePrompt
import hello_helpers.hello_misc as hm

import os
from sound_play.libsoundplay import SoundClient
from sound_play_msgs.msg import SoundRequest

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

class Prompt(Enum):
    DESCRIBE = 0
    MOVE = 1

class VLMTeleop(hm.HelloNode):

    def __init__(self):
        hm.HelloNode.__init__(self)
        hm.HelloNode.main(self, 'vlm_teleop', 'vlm_teleop', wait_for_first_pointcloud=False)

        self.rate = 10.0
        self.joint_state = None
        
        self.image = None
        self.prompt = None
        self.result = None

        self.rad_per_deg = math.pi/180.0
        self.rotate = 20 * self.rad_per_deg # radians
        self.translate = 0.05 # meters

        self.soundhandle = SoundClient(self, blocking=True)
        self.voice = 'voice_cmu_us_ahw_cg' # 'voice_kal_diphone' # cmu_us_slt_cg
        self.volume = 1.0

        # Initialize the voice command
        self.voice_command = ' ' 

        # Initialize the sound direction
        self.sound_direction = 0 # not used?

        # Initialize subscribers
        self.speech_to_text_sub = self.create_subscription(SpeechRecognitionCandidates, "/speech_to_text", self.callback_speech, 1)
        self.sound_direction_sub = self.create_subscription(Int32, "/sound_direction", self.callback_direction, 1)
        self.create_subscription(JointState, '/stretch/joint_states', self.joint_states_callback, 1)
        # should we use the following instead of line above?
        # self.joint_state_sub = self.create_subscription(JointState, '/stretch/joint_states', self.joint_states_callback, 1)
        self.image_sub = self.create_subscription(Image, 'camera/color/image_raw', self.image_callback, 1)

        # Initialize variables related to VLService
        self.vl_cli = self.create_client(VLService, 'vision_language_client')
        while not self.vl_cli.wait_for_service(timeout_sec=1.0):
            print('VLService not available, waiting again')
            #self.get_logger().info('service not available, waiting again...')
        self.vl_req = VLService.Request()
        self.vl_cli_futures = []
        self.vl_future = None
        
        # Initialize variables related to PrePrompt service
        self.pp_cli = self.create_client(PrePrompt, 'preprompt_client')
        while not self.pp_cli.wait_for_service(timeout_sec=1.0):
            print('PrePrompt service not available, waiting again')
            #self.get_logger().info('service not available, waiting again...')
        self.pp_req = PrePrompt.Request()
        self.pp_cli_futures = []
        self.pp_future = None

    
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
    

    def image_callback(self, msg):
        self.image = msg.data
        # print("W, H: ", msg.width, msg.height)
    
    def read_message(self):
        self.get_logger().info(self.result)


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
        self.vl_req.image = self.image
        self.vl_req.prompt = prompt
        
        self.vl_cli_future = self.vl_cli.call_async(self.vl_req)
        rclpy.spin_until_future_complete(self, self.vl_cli_future) ########## HELLO, HI, I'M THE PROBLEM ITS ME
        print('Finished getting VLM answer')
        result = self.vl_cli_future.result()
        self.get_logger().info(f'VLM Result: {result}')
        
        joint_state = self.joint_state

        if prompt_type == Prompt.DESCRIBE:
            print('THIS IS WHAT I SEE')
            print("Speaking should start now.")
            self.soundhandle.say(result.result, self.voice, self.volume)
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
                # Make the acsoundhandle.say(result.result, self.voice, self.volume)

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