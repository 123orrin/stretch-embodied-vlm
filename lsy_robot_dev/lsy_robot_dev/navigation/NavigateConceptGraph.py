from .utils import read_json_file, query_llm
from openai import OpenAI

class NavigateConceptGraph:
    def __init__(self, system_prompt_path, scene_json_path):
        self.system_prompt = open(system_prompt_path, "r").read()
        scene_desc = read_json_file(scene_json_path)
        self.scene_desc = [val for key, val in scene_desc.items()]
        self.client = OpenAI() 
            
    def query_goal(self, query, visual=False, excluded_ids=[]):
        """Process query and publish goal pose of relevant object to /cf_object_location."""
     
        scene = [obj for obj in self.scene_desc if obj["id"] not in excluded_ids]

        # if visual:
        #     llava_query = "Describe the current image. If you see a text or a sign, read it and include the content in your answer."
        #     image_desc = self.llava(query=llava_query, image_features=self.get_image_features())
        #     query_gpt = f"I see an image described as '{image_desc}' and and would like you to find an object matching the query '{query}'."
        #     self.llava.reset()
        # else:
        query_gpt = query


        self.object_desc_base = query
        # query_gpt = f"'The object described as '{query}' is not in the scene. Find a likely container or storage space where someone is likely to have moved the object described as '{query}'?"
        """query_gpt = (f"The object described as '{self.object_desc_base}' is not in the scene. "
                 f"Perhaps someone has moved it, or put it away. "
                 f"Let's try to find the object by visiting the likely places, storage or containers that are "
                 f"appropriate for the missing object (eg: a a cabinet for a wineglass, or closet for a broom). "
                 f"So the new query is find a likely container or storage space where someone typically would"
                 f"have moved the object described as '{self.object_desc_base}'?")"""
        query_gpt = (f"The objects in the scene were found in a house. Which object best accomplishes the desired user goal: {query}?")
        # query_gpt = (f"The objects in the scene were found in a house. Find an a storage space or container "
        #              f"where would you expect to find '{self.object_desc_base}' in a typical house.")

        response = query_llm(query_gpt, self.system_prompt, scene, client=self.client)

        query_achievable = response["query_achievable"]
        print('response:', response)
        if query_achievable:
            object_id = response["final_relevant_objects"][0]
            object_desc = response["most_relevant_desc"]

            # Find object data
            for object_data in self.scene_desc:
                if object_data["id"] == object_id:
                    break


            # Publish the message
        else:
            object_id, object_desc = -1, "NOT AN OBJECT"

        return dict(object_id=object_id, object_desc=object_desc, query_achievable=query_achievable)
        

if __name__ == "__main__":
    nav = NavigateConceptGraph(system_prompt_path='./conceptgraph/navigation/prompts/concept_graphs_planner.txt', scene_json_path='./conceptgraph/navigation/obj_json_r_mapping_stride13.json')
    target_object = nav.query_goal(query='something I can wear to get warm', visual=False, excluded_ids=[])

    print(target_object)