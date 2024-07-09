from lsy_interfaces.srv import VLService

import rclpy
from rclpy.node import Node

from PIL import Image as PIL_Image 
import numpy as np
from transformers import AutoModelForCausalLM 
from transformers import AutoProcessor 


class VLServer(Node):

    def __init__(self):
        super().__init__('vision_language_server')
        self.srv = self.create_service(VLService, 'vision_language_client', self.vision_language_callback)

        self.model_id = "microsoft/Phi-3-vision-128k-instruct" 
        self.model = AutoModelForCausalLM.from_pretrained(self.model_id, device_map="cuda", trust_remote_code=True, torch_dtype="auto", _attn_implementation='flash_attention_2') # use _attn_implementation='eager' to disable flash attention
        self.processor = AutoProcessor.from_pretrained(self.model_id, trust_remote_code=True) 

    def vision_language_callback(self, request, response):
        image = np.array(request.image)
        image = np.rot90(image, 1)
        image = PIL_Image.fromarray(image)
        messages = [ 
            {"role": "user", "content": f"<|image|>\n {request.prompt}"} 
            #{"role": "assistant", "content": "The chart displays the percentage of respondents who agree with various statements about their preparedness for meetings. It shows five categories: 'Having clear and pre-defined goals for meetings', 'Knowing where to find the information I need for a meeting', 'Understanding my exact role and responsibilities when I'm invited', 'Having tools to manage admin tasks like note-taking or summarization', and 'Having more focus time to sufficiently prepare for meetings'. Each category has an associated bar indicating the level of agreement, measured on a scale from 0% to 100%."}, 
            #{"role": "user", "content": "Provide insightful questions to spark discussion."} 
        ] 

        prompt = self.processor.tokenizer.apply_chat_template(messages, tokenize=False, add_generation_prompt=True)

        inputs = self.processor(prompt, [image], return_tensors="pt").to("cuda:0") 
                
        generation_args = { 
            "max_new_tokens": 500, 
            "temperature": 0.0, 
            "do_sample": False, 
        } 

        generate_ids = self.model.generate(**inputs, eos_token_id=self.processor.tokenizer.eos_token_id, **generation_args) 

        generate_ids = generate_ids[:, inputs['input_ids'].shape[1]:]
        response.result = self.processor.batch_decode(generate_ids, skip_special_tokens=True, clean_up_tokenization_spaces=False)[0] 

        print(response.result)
        
        return response


def main():
    rclpy.init()

    vl_service = VLServer()

    rclpy.spin(vl_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
