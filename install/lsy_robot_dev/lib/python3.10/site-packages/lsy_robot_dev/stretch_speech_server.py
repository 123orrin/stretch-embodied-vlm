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
    #try:
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

    #except:
        response.tts_processed = False
        print('An exception occured.')

        return response


def main():
    rclpy.init()
    stretch_speech_service = StretchSpeechServer()
    rclpy.spin(stretch_speech_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()