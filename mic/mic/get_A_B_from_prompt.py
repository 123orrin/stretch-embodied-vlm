import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from langchain_ollama import OllamaLLM

def get_llm_response(message):
    '''Self explanatory'''
    llm = OllamaLLM(model="llama3.1")
    response = llm.invoke(message)
    return response

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            '/mic_stt',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        text = msg.data
        if 'amy' in text or "Amy" in text:
            prompt = f'Reformat the following prompt into the following format: "A|B". Only output this format and do not output anything else \
                    A will be the object we are trying to move to and B will be the object or location where A is located\
                    if you are unsure of either A or B, return None for each of them respectively\ An example for the prompt "Amy, could you move to the water bottle on the chair?" should output "water bottle|chair"\
                    The given prompt is: {text}'
            
            response = get_llm_response(prompt)

            AB = response.split('|')
            A = AB[0]
            B = AB[1]
            print(f'A: {A}  B: {B}')
            with open("AB.txt", "w") as f:
                f.write(f'new\n{A}\n{B}')
            

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()