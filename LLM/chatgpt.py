import openai
import re
import math, copy
import numpy as np
import os
import json
import time, shutil
import rospy
from std_msgs.msg import String
from collections import OrderedDict

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

        self.reset_count=0

        rospy.sleep(3)

        open('./LLM/chat_history.txt', 'w').close()

        with open("./LLM/Lib/"+self.robot_name+"/config.json", "r") as f:
            self.config = json.load(f)

        rospy.loginfo("Initializing ChatGPT...")

        openai.api_key = self.config["OPENAI_API_KEY"]
        self.last_response=''

        self.sysprompt_path = "./LLM/Lib/"+self.robot_name+"/system_prompts/basic.txt"
        self.prompt_path = "./LLM/Lib/"+self.robot_name+"/prompts/"+self.abstraction+".txt"
        self.env_prompt_path="./LLM/Lib/"+self.robot_name+"/env_prompts/basic.txt"


        self.inits=OrderedDict()
        self.inits["from Lib."+self.robot_name+".FunctionLibrary import FunctionLib"] = -1
        self.inits["import rospy"] = -1
        self.inits["rospy.init_node(\'gpt\')"] = -1
        self.inits["lib = FunctionLib()"] = -1

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
                "content": self.prompt + " \n Pick up the graduated cylinder with water inside and pour its content into 500mL beaker. After pouring, place the graduated cylinder at marker 6."
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

# get the graduated cylinder's name by calling get_object_name_by_contents function
object_name = lib.get_object_name_by_contents("water")

# Get the graduated cylinder dimensions by calling get_object_dimensions function
dims=lib.get_object_dimensions(object_name)
if dims is not None:
    cylinder_radius = dims[0]
    cylinder_height = dims[1]

# Get the locations of marker 6, cylinder and 500mL beaker
marker_6_location = lib.get_marker_location(6)
cylinder_location = lib.get_object_location(object_name)

if marker_6_location is None:
    print("Marker 6 not found. Please check the environment.")
    exit()

# Move above 0.05 meters of the top of the cylinder
success = lib.go(cylinder_location[0], cylinder_location[1], cylinder_location[2] + cylinder_height / 2.0 + 0.05, 
                 cylinder_location[3], cylinder_location[4], cylinder_location[5]) 

# Move down to grasp the cylinder
success = lib.go(cylinder_location[0], cylinder_location[1], cylinder_location[2], 
                 cylinder_location[3], cylinder_location[4], cylinder_location[5])

# Close the gripper to grasp the cylinder
lib.close_gripper(object_name)

# Pour to 500mL beaker
lib.pour("beaker 500mL")

# Move to marker 6's location
success = lib.go(marker_6_location[0], marker_6_location[1], marker_6_location[2] + cylinder_height / 2.0,
                 marker_6_location[3], marker_6_location[4], marker_6_location[5])

# Open the gripper to release the cylinder
lib.open_gripper()

lib.move_to_home_position()
rospy.sleep(0.5)

print("Task finished")
```
This code picks the graduated cylinder with water inside and pours it into 500mL beaker then places the cylinder in front of marker 6."""
            }
        ]

        #self.ask(self.env_prompt + self.prompt)
        self.init_history = copy.deepcopy(self.chat_history)
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
            model="gpt-4", #"gpt-3.5-turbo-0613",#"gpt-3.5-turbo",
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
        return full_text.strip()
    
    def reduce_history(self):
        chat = copy.deepcopy(self.chat_history)
        chat[1:3]=copy.deepcopy(self.chat_history[-3:-1])
        
        rospy.loginfo("History reduced")
        self.chat_history = copy.deepcopy(chat[0:5])

    def reset_history(self):
        rospy.loginfo("GPT History Resetted")
        self.chat_history = copy.deepcopy(self.init_history)
        self.reset_count+=1

    def add_grounding_to_prompt(self,question):
        if "add" in question.lower() and "workspace" in question.lower():
            question+=" Make sure to use marker location."

        if "pour" in question.lower():
             question+=" Don't move above the beaker before pouring, just call the pour function. Also, after pouring, make sure you place the object back to where it was on the table and then open the gripper to release it."

        if "function" in question.lower() or "generic" in question.lower() or "code" in question.lower():
            question+= " If you wrote a function, remember to add a example function call at the end."

        question="By using the function library you are provided. "+question
        question+=" make sure to move back to home after the task is finished."

        return question

    def verify_code(self,file_path): 
        with open(file_path, "r") as file:   
            line_number = 0
            data = file.readlines()
            for line in data:
                for key in self.inits.keys():
                    if key in line and not "#" in line:#key == line[:-1]:
                        self.inits[key]=line_number
                
                line_number += 1
        
            key_number = 1
            for key in self.inits.keys():
                val = self.inits[key]
                if val == -1:
                    data.insert(key_number,key+"\n")
                    key_number += 1
                    self.inits[key]=key_number
                else: 
                    key_number = val+1

        rospy_line_num = self.inits["rospy.init_node(\'gpt\')"]
        rospy_line = data[rospy_line_num]
        lib_line_num = self.inits["lib = FunctionLib()"]
        lib_line = data[lib_line_num]

        if lib_line_num < rospy_line_num:
            data[lib_line_num] = rospy_line
            data[rospy_line_num] = lib_line

        with open(file_path, "w") as file:
            file.writelines(data)

    def convert_fstrings_to_format(self, input_string):
        # Define a regular expression pattern to match f-string statements
        fstring_pattern = r'f"([^\"]*)"'
        
        # Use regex to find all f-string statements in the input string
        fstring_matches = re.findall(fstring_pattern, input_string)
        res=""
        # Replace f-string statements with .format() method calls
        for match in fstring_matches:
            format_args = re.findall(r'\{([^\}]*)\}', match)
            format_args = ', '.join(format_args)
            format_method_call = '\"{}\"'.format(match)
            # format_method_call = re.sub(format_args,'{}',match)+'".format({})"'.format(format_args)
            input_string = input_string.replace('f"{}"'.format(match), format_method_call)
            # re.sub("\{.*?\}","\{\}", input_string)
            x=input_string.replace("{","*{")
            x=x.replace("}","}*")
            y=x.split("*")
            res=""

            for i in y:
                if len(i)!=0:
                    if i[0]=="{" and i[-1]=="}":
                        res+="{}"
                    else:
                        res+=i

            res="".join(res)
            if res[-1] == "\n":
                res = res[:-2]+(".format("+format_args+")")+res[-2:]
            else:
                res = res[:-1]+(".format("+format_args+")")+res[-1:]

        if res == "":
            return input_string
        else:
            return res

    def code_python_version_correction(self, file_path):
            with open(file_path, "r") as file:   
                data = file.readlines()
                res=""
                for i, line in enumerate(data):
                    corrected_line = self.convert_fstrings_to_format(line)
                    print(corrected_line)
                    res +=(corrected_line)
            with open(file_path, "w") as file:
                file.writelines(res)

    def get_gpt_response(self,question):

        question = self.add_grounding_to_prompt(question)
        # rospy.loginfo("Grounded prompt: "+question)

        try:
            response = self.ask(question)
        except:
            rospy.loginfo("Exceeded number of tokens. Code not generated")
            self.reset_history()
            return False
        
        self.reset_count = 0

        f = open("./LLM/gpt_code.py", "w")
        h = open('./LLM/chat_history.txt', 'a')
        h.write(question)
        h.write(response)
            
        code = self.extract_python_code(response)

        text = self.extract_text(response)

        if code is not None:
            self.responsePublisher.publish(text)
            self.responsePublisher.publish("Please run the code by using the terminal...")
            codeByLLM=self.extract_python_code(response)
            f.write(codeByLLM)
            f.close()
            self.verify_code("./LLM/gpt_code.py")
            self.code_python_version_correction("./LLM/gpt_code.py")
            try:
                rospy.loginfo("running code...")
                self.execPublisher.publish("True")
            except Exception as e:
                self.responsePublisher.publish("An exception occured while running the code!\n")
                rospy.loginfo("exception occured while running code")
                print(e)
            else:
                self.responsePublisher.publish("Ready!")
                rospy.loginfo("Ready for running the code")

            h.close()

            return True
        else:

            h.close()

            return False

if __name__ == '__main__':
    files=1
    reset=True
    gpt=GPT()
    
    while not rospy.is_shutdown():
        flag=False

        question = rospy.wait_for_message("/llm_propmt",String)
        question = question.data

        if question == "!quit" or question == "!exit":
            break
        
        if "save" in question:
            shutil.copyfile('./LLM/gpt_code.py', "./LLM/gpt_code_"+str(files)+".py")
            gpt.responsePublisher.publish("Code saved with number "+str(files))
            files+=1
            flag = True

        if question == "reset":
            gpt.responsePublisher.publish("Resetting GPT! Last saved file number: "+str(files-1))
            gpt.reset_history()
            flag = True
            
        while not flag:
            flag=gpt.get_gpt_response(question)
            rospy.loginfo("in the loop.")
            if gpt.reset_count > 1:
                break

    rospy.loginfo("LLM Node killed.")