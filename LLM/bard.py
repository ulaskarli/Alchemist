
import re
import math, copy
import numpy as np
import os
import json
import time, shutil
import rospy
from std_msgs.msg import String
from bardapi import Bard

class GPT():
    def __init__(self):

        rospy.init_node("gpt_controller")
        robot_name=rospy.wait_for_message("/start_llm",String)
        rospy.loginfo("got robot name")
        abstraction=rospy.wait_for_message("/abstraction",String)
        rospy.loginfo("got abstraction level")
        self.responsePublisher=rospy.Publisher("/llm_response",String,queue_size=10)
        self.execPublisher=rospy.Publisher("/run_gpt_code", String, queue_size=1)

        self.robot_name=robot_name.data.lower()
        self.abstraction=abstraction.data.lower()

        open('./LLM/chat_history.txt', 'w').close()

        with open("./LLM/Lib/"+self.robot_name+"/config.json", "r") as f:
            self.config = json.load(f)

        rospy.loginfo("Initializing ChatGPT...")

        os.environ['_BARD_API_KEY']=self.config["BARD_API_KEY"]
        self.bard = Bard(token=self.config["BARD_API_KEY"], timeout=10)

        self.last_response=''

        self.sysprompt_path = "./LLM/Lib/"+self.robot_name+"/system_prompts/basic.txt"
        self.prompt_path = "./LLM/Lib/"+self.robot_name+"/prompts/"+self.abstraction+".txt"
        self.env_prompt_path="./LLM/Lib/"+self.robot_name+"/env_prompts/basic.txt"


        self.inits={
        "from Lib."+self.robot_name+".FunctionLibrary import FunctionLib": 0,
        "import rospy":0,
        "rospy.init_node(\'gpt\')":0,
        "lib = FunctionLib()":0
        }

        with open(self.sysprompt_path, "r") as f:
            self.sysprompt = f.read()

        with open(self.prompt_path, "r") as f:
            self.prompt = f.read()

        with open(self.env_prompt_path, "r") as f:
            self.env_prompt = f.read()

        self.code_block_regex = re.compile(r"```(.*?)```", re.DOTALL)

        self.chat_history = [
            {
                "role": "system",
                "content": self.sysprompt
            },
            {
                "role": "user",
                "content": "there is a jar on marker 6 with height 0.14 meters and 0.04 radius. put jar in the workspace then pick it."
            },
            {
                "role": "assistant",
                "content": """```python
from Lib."""+self.robot_name+""".FunctionLibrary import FunctionLib
import rospy

# Initialize rospy node called gpt
rospy.init_node('gpt')

# Initialize function library
lib = FunctionLib()

lib.move_to_home_position()
rospy.sleep(2)

# Define the jar dimensions
cylinder_height = 0.2
cylinder_radius = 0.05

# Get the location of marker 6 and marker 9
marker_6_location = lib.get_marker_location(6)
marker_9_location = lib.get_marker_location(9)

if marker_6_location is None:
    print("Marker 7 not found. Please check the environment.")
        exit()
        
if marker_9_location is None:
    print("Marker 7 not found. Please check the environment.")
        exit()
        
# Add jar to the workspace in front of marker 6
lib.add_cylinder_to_workspace("jar", marker_6_location[0], marker_6_location[1],
                              marker_6_location[2] + cylinder_height / 2.0, cylinder_height, cylinder_radius)

# Move above 0.1 meters the jar's location
success = lib.go(marker_6_location[0], marker_6_location[1], marker_6_location[2]+ cylinder_height / 2.0 + 0.1, marker_6_location[3], marker_6_location[4], marker_6_location[5])

if not success:
    print("Could not reach the cylinder")
    exit()

rospy.sleep(2)

# Move down to grasp the jar
success = lib.go(marker_6_location[0], marker_6_location[1], marker_6_location[2] + cylinder_height / 2.0, marker_6_location[3], marker_6_location[4], marker_6_location[5])

# Close the gripper to grasp the cylinder
lib.close_gripper("jar")
rospy.sleep(2)

# Move up 0.1 meters
success = lib.go(marker_6_location[0], marker_6_location[1], marker_6_location[2] + cylinder_height / 2.0 + 0.1, marker_6_location[3], marker_6_location[4], marker_6_location[5])
```
This code puts a cylinder in the workspace as jar on marker 6 and then picks it and lifts it up 0.1 meters"""
            }
        ]

        self.ask(self.prompt + self.env_prompt)
        self.responsePublisher.publish("Welcome to the "+ self.robot_name +" chatbot! I am ready to help you with your "+ self.robot_name +" questions and commands.")
        rospy.loginfo("Done")


    def ask(self, prompt):
        self.chat_history.append(
            {
                "role": "user",
                "content": prompt,
            }
        )

        completion = self.bard.get_answer(self.chat_history)['content']

        self.chat_history.append(
            {
                "role": "assistant",
                "content": completion,
            }
        )
        return self.chat_history[-1]["content"]

    def extract_python_code(self, content):
        code_blocks = self.code_block_regex.findall(content)
        if code_blocks:
            full_code = "\n".join(code_blocks)

            if full_code.startswith("python"):
                full_code = full_code[7:]

            return full_code
        else:
            return None
        
    def extract_text(self, content):
        full_text = self.code_block_regex.sub('',content)
        return full_text
    
    def reduce_history(self):
        
        #chat_history_last = copy.deepcopy(self.chat_history[-3:-1])
        #chat_history_top = copy.deepcopy(self.chat_history[0])
        #chat_history_lib = copy.deepcopy(self.chat_history[3:5])

        chat = copy.deepcopy(self.chat_history)
        
        chat[1:3]=copy.deepcopy(self.chat_history[-3:-1])

        #chat.append(chat_history_lib)
        
        rospy.loginfo("History reduced")
        self.chat_history = copy.deepcopy(chat[0:5])


    ## GPT common mistake: forgot to add object to workspace before closing the gripper (attach object not found) 
    def verify_add_cylinder(self, file_path):
        add_object_called = False
        with open(file_path, "r") as file:
            data = file.readlines()

            updated_data = []

            for line in data:
                # Check if the add_cylinder_to_workspace function is called
                if "lib.add_cylinder_to_workspace" or "lib.add_box_to_workspace" in line:
                    add_object_called = True

                # Add the line to the updated_data list
                updated_data.append(line)

                # Check if close_gripper function is called
                if "lib.close_gripper" in line:
                    # If add_cylinder_to_workspace was not called before close_gripper,
                    # add the function call before close_gripper
                    if not add_object_called:
                        updated_data.insert(-1, "lib.add_cylinder_to_workspace(...)\n") # or ask GPT to generate the response again ...?

        # Write the updated_data back to the file
        with open(file_path, "w") as file:
            file.writelines(updated_data)


    def verify_code(self,file_path): 
        with open(file_path, "r") as file:   
            line_number = 1
            data = file.readlines()
            for line in data:
                for key in self.inits.keys():
                    if key == line[:-1]:
                        self.inits[key]=line_number
                
                line_number += 1
        
            key_number = 1
            for key in self.inits.keys():
                val = self.inits[key]
                if val == 0:
                    data.insert(key_number,key+"\n")
                    key_number += 1
                else: 
                    key_number = val+1

        with open(file_path, "w") as file:
            file.writelines(data)

    def get_gpt_response(self,question):

        try:
            response = self.ask(question)
        except:
            rospy.loginfo("Exceeded number of tokens. Code not generated")
            self.reduce_history()
            return False

        f = open("./LLM/gpt_code.py", "w")
        h = open('./LLM/chat_history.txt', 'a')

        h.write(response)
        print(response)
            
        code = self.extract_python_code(response)

        text = self.extract_text(response)
        self.responsePublisher.publish(text)

        if code is not None:
            self.responsePublisher.publish(text)
            self.responsePublisher.publish("Please run the code by using the terminal...")
            codeByLLM=self.extract_python_code(response)
            f.write(codeByLLM)
            f.close()
            self.verify_code("./LLM/gpt_code.py")
            self.verify_add_cylinder("./LLM/gpt_code.py")
            try:
                rospy.loginfo("running code...")    
                #exec(codeByLLM)
                self.execPublisher.publish("True")
            except Exception as e:
                self.responsePublisher.publish("An exception occured while running the code!\n")
                rospy.loginfo("exception occured while running code")
                print(e)
            else:
                self.responsePublisher.publish("Ready!\n")
                rospy.loginfo("finsihed running code")

            h.close()

            return True
        else:

            h.close()

            return False

if __name__ == '__main__':
    gpt=GPT()
    files=1
    while not rospy.is_shutdown():
        flag=False

        question = rospy.wait_for_message("/llm_propmt",String)
        question = question.data

        if question == "!quit" or question == "!exit":
            break
        
        if question == "save":
            shutil.copyfile('./LLM/gpt_code.py', "./LLM/gpt_code_"+str(files)+".py")
            files+=1
            flag = True
        
        # while not flag:
        flag=gpt.get_gpt_response(question)
            # rospy.loginfo("in the loop.")

    rospy.loginfo("LLM Node killed.")
