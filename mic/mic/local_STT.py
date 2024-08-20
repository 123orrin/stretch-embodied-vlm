import vosk
import pyaudio
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


from std_msgs.msg import String


# Here I have downloaded this model to my PC, extracted the files 
# and saved it in local directory
# Set the model pathros2 topic li
model_path = "vosk-model-en-us-0.42-gigaspeech"
# model_path = "vosk-model-small-en-us-0.15"

# Initialize the model with model-path
model = vosk.Model(model_path)


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, '/mic_stt', 10)
        timer_period = 0.5  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)
        # self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    msg = String()

    minimal_publisher = MinimalPublisher()
    while True:      
        rec = vosk.KaldiRecognizer(model, 16000)

        p = pyaudio.PyAudio()
        stream = p.open(format=pyaudio.paInt16,
                        channels=1,
                        rate=16000,               
                        input=True,
                        frames_per_buffer=8192)


        print("Listening for speech. Say 'Terminate' to stop.")
        # Start streaming and recognize speech
        while True:
            data = stream.read(4096)#read in chunks of 4096 bytes
            if rec.AcceptWaveform(data):#accept waveform of input voice
                # Parse the JSON result and get the recognized text
                result = json.loads(rec.Result())
                recognized_text = result['text']
                msg.data = recognized_text
                minimal_publisher.publisher_.publish(msg)
                print(recognized_text)
                
                if "terminate" in recognized_text.lower():
                    print("Termination keyword detected. Stopping...")
                    break

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

