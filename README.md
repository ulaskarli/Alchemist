# Alchemist
A coding interface for users to write robot platform specific code in natural language with the help of Large Language Models

## Usage
Place OpenAI API key in config.json file under every robot platform intended to be used inside Lib folder. Then startup ROS nodes of your selected robot platform (real or simulated), after that run the following bash script to launch the app.

```
$ ./main.sh
```

first a pop up screen with two drop-downs will appear select your robot platform and abstraction level, then the main application screen will come up. Refer to user manual for UI usage.

https://drive.google.com/file/d/1EKO_2-yTFqT2fMO5ACmnhjlJg2VYo2o3/view?usp=sharing

## References

RVIZ Python Interface
http://docs.ros.org/en/kinetic/api/rviz_python_tutorial/html/

PyQt Text Editor
https://pythonprogramming.net/text-editor-pyqt-tutorial/

PyQt Chat Client
https://build-system.fman.io/python-qt-tutorial

Microsoft Research PromptCraft-Robotics
https://github.com/microsoft/PromptCraft-Robotics
