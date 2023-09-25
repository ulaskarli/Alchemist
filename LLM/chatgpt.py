import openai
import re
import math
import numpy as np
import os
import json
import time
import rospy
from std_msgs.msg import String

class GPT():
    def __init__(self):

        rospy.init_node("gpt_controller")
        robot_name=rospy.wait_for_message("/start_llm",String)
        self.responsePublisher=rospy.Publisher("/llm_response",String,queue_size=10)
        self.execPublisher=rospy.Publisher("/run_gpt_code", String, queue_size=1)

        self.robot_name=robot_name.data.lower()

        open('./LLM/chat_history.txt', 'w').close()

        with open("./LLM/Lib/"+self.robot_name+"/config.json", "r") as f:
            self.config = json.load(f)

        rospy.loginfo("Initializing ChatGPT...")

        openai.api_key = self.config["OPENAI_API_KEY"]
        self.last_response=''

        self.sysprompt_path = "./LLM/Lib/"+self.robot_name+"/system_prompts/basic.txt"
        self.prompt_path = "./LLM/Lib/"+self.robot_name+"/prompts/basic.txt"

        with open(self.sysprompt_path, "r") as f:
            self.sysprompt = f.read()

        with open(self.prompt_path, "r") as f:
            self.prompt = f.read()

        self.code_block_regex = re.compile(r"```(.*?)```", re.DOTALL)

        self.chat_history = [
            {
                "role": "system",
                "content": self.sysprompt
            },
            {
                "role": "user",
                "content": "pick tube1 and place on tube2"
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

# Get the current base pose
current_pose = lib.get_current_end_effector_pose()

# Get the location of tube1
target_location = lib.get_object_location("tube1")

# Get the location of tube2
goal_location = lib.get_object_location("tube2")

if target_location[1] > 0.25:
    # Get the grasp orientation for tube1 if it is more than 0.2 meters to the left top grasp first
    target_orientation = lib.get_grasp_orientation(top=True)
    # increase z of the tube1 grasping location by fourth of its height so we can get a top grasp
    target_location[2] = target_location[2]+0.025
    # increase z of the tube2 location by three fourth of its height so we can put tube1 on top of tube2
    goal_location[2] = goal_location[2]+0.075
else:
    # Else Get the grasp orientation for tube1 side grasp
    target_orientation = lib.get_grasp_orientation(top=False)
    # increase z of the tube2 location by its height so we can put tube1 on top of tube2
    goal_location[2] = goal_location[2]+0.1

# Plan to move the robot arm to tube1 location
flag=lib.go(target_location[0], target_location[1], target_location[2], target_orientation[0], target_orientation[1], target_orientation[2])

count=0
abort=False
# If not desired pose abort try different grasp orientation at most 5 times
while not flag:
    # try different grasp orientation
    target_orientation = lib.get_grasp_orientation(top=False)
    # increase z of the tube2 location by its height so we can put tube1 on top of tube2
    goal_location[2] = goal_location[2]+0.1
    # Plan to move the robot arm to tube1 location
    flag=lib.go(target_location[0], target_location[1], target_location[2], target_orientation[0], target_orientation[1], target_orientation[2])

    # if can't find a plan in 5 tries abort
    count+=1
    if count == 5:
        abort=True
        break

#if not abort do the rest
if not abort:
    # If able to grasp
    lib.close_gripper("tube1",0.04)

    # Plan to move the robot arm on top of tube2
    lib.go(goal_location[0], goal_location[1], goal_location[2], target_orientation[0], target_orientation[1], target_orientation[2])

    lib.open_gripper()

    lib.move_to_home_position()
```
This code picks tube1 and places it on tube2."""
            }
        ]


        self.ask(self.prompt)
        self.responsePublisher.publish("Welcome to the "+ self.robot_name +" chatbot! I am ready to help you with your "+ self.robot_name +" questions and commands.")
        rospy.loginfo("Done")


    def ask(self, prompt):
        self.chat_history.append(
            {
                "role": "user",
                "content": prompt,
            }
        )
        completion = openai.ChatCompletion.create(
            model="gpt-3.5-turbo",
            messages=self.chat_history
            # temperature=0
        )
        self.chat_history.append(
            {
                "role": "assistant",
                "content": completion.choices[0].message.content,
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
        self.chat_history = [
            {
                "role": "system",
                "content": self.sysprompt
            },
            {
                "role": "user",
                "content": self.prompt,
            },
            {
                "role": "assistant",
                "content":self.last_response
            }
        ]

    def get_gpt_response(self,question):

        try:
            response = self.ask(question)
            self.last_response=response
        except:
            self.reduce_history()

        f = open("./LLM/gpt_code.py", "w")
        h = open('./LLM/chat_history.txt', 'a')

        h.write(response)
            
        code = self.extract_python_code(response)

        text = self.extract_text(response)

        self.responsePublisher.publish(text)

        if code is not None:
            self.responsePublisher.publish("Please wait while I run the code in Simulation...")
            codeByLLM=self.extract_python_code(response)
            f.write(codeByLLM)
            try:
                rospy.loginfo("running code...")    
                #exec(codeByLLM)
                self.execPublisher.publish("True")
            except Exception as e:
                self.responsePublisher.publish("An exception occured while running the code!\n")
                rospy.loginfo("exception occured while running code")
                print(e)
            else:
                self.responsePublisher.publish("Done!\n")
                rospy.loginfo("finsihed running code")
            
        f.close()
        h.close()


if __name__ == '__main__':
    gpt=GPT()
    while not rospy.is_shutdown():

        question = rospy.wait_for_message("/llm_propmt",String)
        question = question.data

        if question == "!quit" or question == "!exit":
            break
        gpt.get_gpt_response(question)
    rospy.loginfo("LLM Node killed.")