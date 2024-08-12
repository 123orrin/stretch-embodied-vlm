import pyaudio  
import wave  
import os

#define stream chunk   
chunk = 1024  
  
#open a wav format music  
audio_result_folder = '/home/hello-robot/lsy_software_tests/vlm_teleop_audio_output' # must be .WAV or .OGG
audio_filename = ''.join(('vlm_teleop_openai_tts_', str(3), '.wav'))
audio_result_path = os.path.join(audio_result_folder, audio_filename)

f = wave.open(audio_result_path,"rb")  
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
