from lsy_interfaces.srv import VLService

import rclpy
from rclpy.node import Node

import os
import base64
import requests
import json

from openai import OpenAI
 
import numpy as np
from PIL import Image as PIL_Image
# from transformers import AutoModelForCausalLM 
# from transformers import AutoProcessor 

from geometry_msgs.msg import PointStamped


class VLServer(Node):

    def __init__(self):
        super().__init__('vision_language_server')
        self.srv = self.create_service(VLService, 'vision_language_client', self.vision_language_callback)

        self.openai_key = os.getenv('OPENAI_API_KEY')
        self.openai_client = OpenAI(api_key=self.openai_key)
        self.model_id = "gpt-4o-mini"  # using gpt-4o-mini because according to OpenAI, it is cheaper and faster than gpt-3.5-turbo as of July 2024
        # self.model_id_vision = "gpt-4o-mini-2024-07-18"
        # self.model_id_chat = "gpt-3.5-turbo-0125"
        self.openai_vision_detail = "low"  # can also be set to: "high" or "auto", but "high" is more expensive

        self.image_path = '/home/hornywombat/stretch-images-vlmteleop/vlm_teleop_image.jpeg'

        self.sub = self.create_subscription(PointStamped, '/clicked_point', self.a, 1)

    def a(self, msg):
        print('GOT POINTSTAMPED')
        print(msg.point.x, msg.point.y, msg.point.z)

    def encode_image(self):
        with open(self.image_path, "rb") as image_file:
            return base64.b64encode(image_file.read()).decode('utf-8')
        # return base64.b64encode(image).decode('utf-8')

    def vision_language_callback(self, request, response):
        if request.use_image:
            image = np.array(request.image).reshape(720, 1280, 3)
            image = np.rot90(image, 3)
            image = PIL_Image.fromarray(image)
            image = image.save(self.image_path)
            # image = np.ascontiguousarray(image)
            base64_image = self.encode_image()


            headers = {
            "Content-Type": "application/json",
            "Authorization": f"Bearer {self.openai_key}"
            }

            payload = {
            "model": self.model_id,
            "messages": [
                {
                "role": "user",
                "content": [
                    {
                    "type": "text",
                    "text": f"{request.prompt}. Respond in less than 40 words."
                    },
                    {
                    "type": "image_url",
                    "image_url": {
                        "url": f"data:image/jpeg;base64,{base64_image}",
                        "detail": self.openai_vision_detail
                    }
                    }
                ]
                }
            ],
            "max_tokens": 300
            }

            completion = requests.post("https://api.openai.com/v1/chat/completions", headers=headers, json=payload)
            # completion_data = json.load(completion.json())

            completion_json = completion.json()
            completion_choices_dict = completion_json.get('choices')[0]
            completion_message_content = completion_choices_dict.get('message').get('content')
            #completion_data = json.load(completion_json)

            print('type(completion.json()): ', type(completion.json()), '\ncompletion.json(): ', completion.json())
            # print(completion_data)
            #print('completion_json.keys()', completion_json.keys())
            #print('completion_json.get("choices")', completion_json.get('choices'))
            print('##### completion_message_content', completion_message_content)
            #print('completion_json.get("choices")[0].message.content: ', completion_json.get('choices')[0].message.content)

        
        else:
            completion = self.openai_client.chat.completions.create(
                            model=self.model_id,
                            messages=[
                                #{"role": "system", "content": "You are a helpful assistant."},
                                {"role": "user", "content": f"{request.prompt}. Respond in less than 20 words."}
                            ]
                            )
            
            print('completion.choices[0].message', completion.choices[0].message)
            print('completion.choices[0].message.content', completion.choices[0].message.content)
            completion_message_content = completion.choices[0].message.content
        
        response.result = completion_message_content
            
        return response


def main():
    rclpy.init()

    vl_service = VLServer()

    rclpy.spin(vl_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
