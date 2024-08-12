import sounddevice as sd
from piper.voice import PiperVoice
import numpy as np

def tts_and_audio_output(message):
    model = '/home/hornywombat/repos/mic/en_US-lessac-medium.onnx'
    voice = PiperVoice.load(model)
    text = message
    stream = sd.OutputStream(samplerate=voice.config.sample_rate, channels=1, dtype='int16')
    stream.start()
    for audio_bytes in voice.synthesize_stream_raw(text):
        int_data = np.frombuffer(audio_bytes, dtype=np.int16)
        stream.write(int_data)
        stream.stop()
        stream.close()

if __name__ == '__main__':
    tts_and_audio_output('hello my name is thomas and I love durums, I love videogames too.')