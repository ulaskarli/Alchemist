# Alchemist
A coding interface for users to write robot platform specific code in natural language with the help of Large Language Models.
This repo is based on the paper "Alchemist: LLM-Aided End-User Development of Robot Applications" published at HRI 24 Boulder.

## Usage
Place OpenAI API key in config.json file under every robot platform intended to be used inside Lib folder. Then startup ROS nodes of your selected robot platform (real or simulated), after that run the following bash script to launch the app.

```
$ ./main.sh
```

first a pop up screen with two drop-downs will appear select your robot platform and abstraction level, then the main application screen will come up. Refer to user manual for UI usage.

https://drive.google.com/file/d/1EKO_2-yTFqT2fMO5ACmnhjlJg2VYo2o3/view?usp=sharing

## Robot Platforms
We provide this platform with 3 different robot platforms (UR5, Franka Emika Panda, TIAGo). We ran our experiments with UR5 for the paper of this work and for that pllatform we have high and low level abstractions in our function library. For the other two platforms we provide templates for high and low abstraction but one should edit the high level abstraction based on specific use case. High level abstraction is meant to have complex higher level functions that are task based. To add your own function open the folder of the platform you wish to work on inside the Lib folder and add the function to the FunctionLibrary.py file. After that add the documentation of that new function in the high.txt file under prompts folder.

You can add your own robot platform to this system by simply adding a new folder under lib named with your robot platform and must include a prompts, system_prompts folder, a config.json for API key and a FunctionLibrary.py file that contains the implementations of the functions to control your robot platform. After that make sure to add the name of the robot in the dropdown menu of the starter pane in app.py

## Vision System
Current version of this system uses the following vision system but it can be swapped for another vision system by changing the function library implementation.

## References

RVIZ Python Interface
http://docs.ros.org/en/kinetic/api/rviz_python_tutorial/html/

PyQt Text Editor
https://pythonprogramming.net/text-editor-pyqt-tutorial/

PyQt Chat Client
https://build-system.fman.io/python-qt-tutorial

Microsoft Research PromptCraft-Robotics
https://github.com/microsoft/PromptCraft-Robotics
