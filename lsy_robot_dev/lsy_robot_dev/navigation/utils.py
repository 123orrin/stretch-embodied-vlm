import openai
import json

def read_json_file(filepath):
    # Read the uploaded JSON file
    content = None
    with open(filepath, "r") as f:
        content = f.read()
    data = json.loads(content)
    return data

def find_objects_by_ids(object_list, target_ids):
    return [obj for obj in object_list if obj['id'] in target_ids]


def query_llm(query, system_prompt, scene_desc, client):
    CHUNK_SIZE = 80  # Adjust this size as needed

    scene_desc_chunks = [scene_desc[i:i + CHUNK_SIZE] for i in range(0, len(scene_desc), CHUNK_SIZE)]
    aggregate_relevant_objects = []

    # for idx, chunk in enumerate(scene_desc_chunks):

    for idx in range(len(scene_desc_chunks) + 1):

        if idx < len(scene_desc_chunks):
            chunk = scene_desc_chunks[idx]
        else:  # On last iteration pass the aggregate_relevant_objects
            print(f"final query")
            chunk = aggregate_relevant_objects
            print(f"chunk : {chunk}")

        scene_desc_str = json.dumps(chunk)
        num_tokens = len(scene_desc_str.split())
        
        completion = client.chat.completions.create(
            model="gpt-3.5-turbo",
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": json.dumps(chunk)},
                {"role": "assistant", "content": "I'm ready."},
                {"role": "user", "content": query},
            ],
            temperature=1,
            top_p=1,
            frequency_penalty=0,
            presence_penalty=0,
            )
        
        response = json.loads(completion.choices[0].message.content)
        
        curr_relevant_objects = find_objects_by_ids(
            chunk, response['final_relevant_objects'])

        aggregate_relevant_objects.extend(curr_relevant_objects)
    try:
        # Try parsing the response as JSON
        response = json.loads(completion.choices[0].message.content)
        # response = json.dumps(response, indent=4)
    except:
        # Otherwise, just print the response
        response = completion.choices[0].message.content
        print("NOTE: The language model did not produce a valid JSON")

    return response
