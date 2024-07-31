import hello_helpers.hello_misc as hm

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import os
import time
from sound_play.libsoundplay import SoundClient
from sound_play_msgs.msg import SoundRequest
from openai import OpenAI

os.system('pactl set-default-sink alsa_output.pci-0000_00_1f.3.analog-stereo') # ZK added
os.system('amixer set Master 200%') # ZK added

class StretchLlmResponseSub(Node):
    def __init__(self):
        super().__init__('stretch_llm_response_sub')
        self.get_logger().info('StretchLlmResponseSub Node Initialized')
        self.llm_response_sub = self.create_subscription(String, "llm_response", self.stretch_tts_callback, 1)
        self.llm_response_sub # prevent unused variable warning

        self.soundhandle = SoundClient(self, blocking=True)
        self.openai_key = os.getenv('OPENAI_API_KEY')
        self.openai_client = OpenAI(api_key=self.openai_key)
        self.tts_model = 'tts-1'
        self.tts_voice = 'alloy'
        self.tts_volume = 1.0
        #self.tts_processed = False
        
        self.audio_result_folder = '/home/hello-robot/lsy_software_tests/vlm_teleop_audio_output' # must be .WAV or .OGG
        self.audio_result_num = 0


    def stretch_tts_callback(self, msg):
        #if msg.data != None and msg.data != '':
        print('>>>> Currently in stretch_tts_callback.')
        print('>>>> msg.data: ', msg.data)

        self.audio_result_num += 1
        audio_filename = ''.join(('vlm_teleop_openai_tts_', str(self.audio_result_num), '.wav'))
        audio_result_path = os.path.join(self.audio_result_folder, audio_filename)

        llm_response_text = msg.data

        ######### OpenAI
        tts_response = self.openai_client.audio.speech.create(model=self.tts_model,
                                                    voice=self.tts_voice,
                                                    input=llm_response_text,
                                                    response_format="wav",
                                                    timeout=60)
        tts_response.write_to_file(audio_result_path)


        ###### Realtime TTS Backup -- To be added

        print("File should be saved now.")
        print("Speaking should start now.")
        self.soundhandle.playWave(audio_result_path, self.tts_volume)
        #time.sleep(5)
        #self.soundhandle.stopWave(audio_result_path)
        print("Speaking should have ended now.")
        #self.tts_processed = True
        #print('self.tts_processed: ', self.tts_processed)


def main():
    rclpy.init()
    stretch_llm_response_sub = StretchLlmResponseSub()
    rclpy.spin(stretch_llm_response_sub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically when the garbage collector destroys the node object)
    stretch_llm_response_sub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()