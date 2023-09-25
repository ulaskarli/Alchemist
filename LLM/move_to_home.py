from Lib.ur5.FunctionLibrary import FunctionLib
import rospy

# Initialize rospy node called gpt
rospy.init_node('gpt')

# Initialize function library
lib = FunctionLib()

lib.move_to_home_position()
rospy.sleep(1)
