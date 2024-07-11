# usage: python llm.py /media/rainer/_data/20-data/M3-robocar_training/donkey_datasets_private/2023-icra-london/tub_202_18-01-28

# llm_analysis_part.py v22
# GPT-4 model, DonkeyCar LLM Image Analysis Part

import argparse
import json
import threading
from langchain_community.chat_models import ChatOllama
from langchain_core.prompts import ChatPromptTemplate
from PIL import Image, ImageChops
import os
import donkeycar as dk
from donkeycar.parts.datastore import Tub  # Correct import from donkeycar

ENHANCED_PROMPT_TEMPLATE = """
Analyze the following image context and provide detailed information about the scene. Include descriptions of the following aspects:

1. **Environment**: Describe the setting, weather, time of day, and overall atmosphere.
2. **Track**: Provide details about the track, including its type, condition, and any relevant markings or features.
3. **Persons**: Identify and describe any persons in the scene, including their activities, attire, and positions relative to other elements.
4. **Obstacles**: Identify and describe any obstacles, including their types, positions, and potential impact on the track or persons.
5. **Objects**: Identify and describe any notable objects in the scene, including vehicles, tools, or other significant items.
6. **Activities**: Describe any ongoing activities or interactions between persons, objects, or obstacles.
7. **Relationships**: Highlight any significant relationships or interactions between elements in the scene, such as a person navigating around an obstacle or a vehicle moving on the track.

Image context: {context}

Provide a detailed analysis based on the above criteria.
"""

class LLMAnalysisPart(threading.Thread):
    def __init__(self, llm_model: str = "llava", tub_path: str = None):
        threading.Thread.__init__(self)
        self.llm_model = llm_model
        self.previous_image_path = None
        self.tub = None
        self.daemon = True
        if tub_path:
            self.tub = Tub(tub_path)

    def analyze_image(self, image_path: str) -> dict:
        context_text = f"Image located at {image_path}"
        
        if self.previous_image_path:
            prev_image = Image.open(self.previous_image_path)
            curr_image = Image.open(image_path)
            diff_image = ImageChops.difference(prev_image, curr_image)

            context_text += f"\n\nMovement detected between frames: {diff_image.getbbox() is not None}"

        prompt_template = ChatPromptTemplate.from_template(ENHANCED_PROMPT_TEMPLATE)
        prompt = prompt_template.format(context=context_text)

        model = ChatOllama(model=self.llm_model, temperature=0.0)
        response_text = model.invoke(prompt)

        analysis_result = json.loads(response_text)

        self.previous_image_path = image_path

        return analysis_result

    def run_analysis(self, image_path: str) -> str:
        analysis_result = self.analyze_image(image_path)
        
        scene = analysis_result.get('scene', '')
        track = analysis_result.get('track', '')
        persons = analysis_result.get('persons', [])
        obstacles = analysis_result.get('obstacles', [])
        objects = analysis_result.get('objects', [])
        activities = analysis_result.get('activities', [])
        relationships = analysis_result.get('relationships', [])
        
        data = {
            'scene': scene,
            'track': track,
            'persons': persons,
            'obstacles': obstacles,
            'objects': objects,
            'activities': activities,
            'relationships': relationships
        }

        # Write data to tub file if available
        if self.tub:
            self.tub.put_record({
                'image_path': image_path,
                'scene': scene,
                'track': track,
                'persons': json.dumps(persons),
                'obstacles': json.dumps(obstacles),
                'objects': json.dumps(objects),
                'activities': json.dumps(activities),
                'relationships': json.dumps(relationships)
            })

        return json.dumps(data, indent=4)

    def run(self):
        pass  # Thread run method, not used directly for analysis

    def analyze(self, image_path: str):
        analysis_thread = threading.Thread(target=self.run_analysis, args=(image_path,))
        analysis_thread.start()

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("tub_path", type=str, help="The path to the tub file directory.")
    parser.add_argument("llm_model", type=str, nargs='?', default="llava", help="The name of the LLM model to use.")
    args = parser.parse_args()

    tub = Tub(args.tub_path)

    image_analysis = LLMAnalysisPart(llm_model=args.llm_model, tub_path=args.tub_path)
    
    for ix in range(tub.get_num_records()):
        try:
            record = tub.get_record(ix)
            image_path = record['image_path']
            print(f"Processing {image_path}")
            image_analysis.analyze(image_path)
        except FileNotFoundError as e:
            print(f"File not found: {e}")

if __name__ == "__main__":
    main()
