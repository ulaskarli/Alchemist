from Lib.ur5.FunctionLibrary import FunctionLib
import rospy
from gpt_code_3 import pick_and_pour

# Initialize rospy node called gpt
rospy.init_node('gpt')

# Initialize function library
lib = FunctionLib()

pick_and_pour(lib,"graduated cylinder 100mL", "beaker 500mL")