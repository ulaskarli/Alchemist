from Lib.ur5.FunctionLibrary import FunctionLib
import time
import rospy

# Initialize rospy node called gpt
rospy.init_node('gpt')

# Initialize function library
lib = FunctionLib()

lib.move_to_home_position()
print("moved to home position")
offset = 0.1
next_marker= int(raw_input("next_marker\n"))

while True:
    if next_marker==-1:
        break
    marker = lib.get_marker_location(next_marker)
    print(marker)
    success = lib.go(marker[0],marker[1],marker[2]+offset,marker[3],marker[4],marker[5])
    next_marker = int(raw_input("next_marker\n"))
    
lib.move_to_home_position()
print("moved to home position and exit\n")
