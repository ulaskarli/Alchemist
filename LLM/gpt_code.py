from Lib.ur5.FunctionLibrary import FunctionLib
import rospy

# Initialize rospy node called gpt
rospy.init_node('gpt')

# Initializing function library
lib = FunctionLib()

# Moving the robot to neutral home position
lib.move_to_home_position()
rospy.sleep(2)

beaker_name = "beaker 500mL"

# Get the beaker's location and dimensions
beaker_location = lib.get_object_location(beaker_name)
dims = lib.get_object_dimensions(beaker_name)
beaker_height = dims[1]

# Reach above the top of the beaker with an offset of 0.05
beaker_grab_height = beaker_location[2] + beaker_height / 2.0 + 0.05
success = lib.go(beaker_location[0], beaker_location[1], beaker_grab_height, 
                 beaker_location[3], beaker_location[4], beaker_location[5])

# Move down to grab the beaker
success = lib.go(beaker_location[0], beaker_location[1], beaker_location[2] + beaker_height / 2.0, 
                 beaker_location[3], beaker_location[4], beaker_location[5])

# Close the gripper to grasp the beaker
lib.close_gripper(beaker_name) 

# Get the marker 10 location
marker_10_location = lib.get_marker_location(10)

# Move to marker 10's location
lib.go(marker_10_location[0], marker_10_location[1], marker_10_location[2] + beaker_height / 2, 
       marker_10_location[3], marker_10_location[4], marker_10_location[5])

# Release the beaker
lib.open_gripper()

# Move robot back to home position after completion
lib.move_to_home_position()
rospy.sleep(0.5)

print("Task finished")
