# MOST UP TO DATE CODE, 17-07-2024, 

from lsy_interfaces.srv import VLService
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


class VLMTeleop(hm.HelloNode):

    def __init__(self):
        hm.HelloNode.__init__(self)
        hm.HelloNode.main(self, 'vlm_teleop', 'vlm_teleop', wait_for_first_pointcloud=False)

        self.rate = 10.0
        self.joint_state = None
        
        # self.preprompt = None
        self.user_prompt = None
        self.image = None
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


    ## Callback functions
    def callback_direction(self, msg):
        #self.sound_direction = msg.data * -self.rad_per_deg
        self.sound_direction = None

    def callback_speech(self,msg):
        self.voice_command = ' '.join(map(str,msg.transcript))
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
    
    def read_message(self):
        self.get_logger().info(self.result)
    
    
    ## Retrieving and processing prompts
    def get_preprompt(self):
        self.user_prompt = self.voice_command ## need to confirm that this is correct
        if self.user_prompt == ' ':
            return None

        print('Processing initial query')
        query = 'You are given three categories: "describe", "move", "chat". From these three categories, output the one that best represents the prompt below. In case of uncertainty, output "chat". \nPrompt: ' + self.user_prompt
        # query = 'You are given three categories: "describe", "move", "chat". From these three categories, output the one that best represents the prompt below. If you output "chat", please also continue the conversation on a new line. In case of uncertainty, output "chat". /nPrompt: ' + self.user_prompt
        print('Initial query: ', query)
        self.vl_req.prompt = query
        self.vl_req.image = self.image
        self.vl_req.use_image = False
        
        self.vl_cli_future = self.vl_cli.call_async(self.vl_req)
        rclpy.spin_until_future_complete(self, self.vl_cli_future) ########## HELLO, HI, I'M THE PROBLEM ITS ME (OLD PROBLEM)
        print('Finished getting initial result from VLM')
        result = self.vl_cli_future.result()
        preprompt = result.result ## need to confirm that this is correct
        self.get_logger().info(f'VLM Result, aka preprompt: {result}')
        print('preprompt: ', preprompt) 
        ## need to decide whether to keep preprompt as local var or revert to global (self.preprompt)
        
        # Reset voice command to None so that it's ready for next iteration of getting preprompt; OK to do this since we've already saved self.voice_command to self.user_prompt
        self.voice_command = ' ' # not sure about the best location for this line
        
        return preprompt


    def process_prompt(self, preprompt):
        if preprompt is None:
            return

        joint_state = self.joint_state
        
        if preprompt == 'describe' or preprompt == 'chat':
            print('Sending user prompt along with image to VLM')
            self.vl_req.image = self.image
            self.vl_req.prompt = self.user_prompt
            self.vl_req.use_image = True
            self.vl_cli_future = self.vl_cli.call_async(self.vl_req)
            rclpy.spin_until_future_complete(self, self.vl_cli_future) ########## HELLO, HI, I'M THE PROBLEM ITS ME (OLD PROBLEM)
            print('Finished getting VLM answer')
            vl_result = self.vl_cli_future.result()
            self.get_logger().info(f'VLM Result: {vl_result}')
            print("Robot speaking should start now.")
            self.soundhandle.say(vl_result.result, self.voice, self.volume)
            print("Robot speaking should have ended now.")
            
        elif preprompt == 'move' and joint_state is not None:
            # call Ken's code here
            pass ## delete when connection to concept graphs and navigation pipeline is ready
            ## automatically start concept graphs mapping and navigation modules, don't use Phi3 for movement anymore
            # once movement is done, have the robot say that it's reached its target
            print("Robot speaking should start now.")
            self.soundhandle.say("I have reached the target.", self.voice, self.volume)
            print("Robot speaking should have ended now.")

        # else: # preprompt == 'chat'
        #    pass ## send image, similar to 'describe' mode?
            # maybe remove this else statement if team agrees with using same framework for describe and chat


    ## Node main
    def main(self):
        while rclpy.ok():
            rclpy.spin_once(self)
            preprompt = self.get_preprompt()
            self.process_prompt(preprompt)  
                      


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