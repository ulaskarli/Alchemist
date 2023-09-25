from Lib.ur5.FunctionLibrary import FunctionLib

import rospy

# Initialize rospy node called gpt
rospy.init_node('gpt')

# Initialize function library
lib = FunctionLib()

lib.move_to_home_position()
rospy.sleep(1)

# Define the jar dimensions
cylinder_height = lib.object_dimensions["100mL graduated cylinder"]["height"]
cylinder_radius = lib.object_dimensions["100mL graduated cylinder"]["radius"]

container_height = lib.object_dimensions["500mL beaker"]["height"]
container_radius = lib.object_dimensions["500mL beaker"]["radius"]

# print(cylinder_height, cylinder_radius, container_height, container_radius)

# Get the location of marker 6 and marker 15
marker_6_location = lib.get_marker_location(6)
marker_15_location = lib.get_marker_location(9)

if marker_6_location is None:
    print("Marker 6 not found. Please check the environment.")
    exit()
        
if marker_15_location is None:
    print("Marker 15 not found. Please check the environment.")
    exit()
        
# Add jar to the workspace in front of marker 6
lib.add_cylinder_to_workspace("grad_cylinder_100mL", marker_6_location[0], marker_6_location[1],
                              marker_6_location[2] + cylinder_height / 2.0, marker_6_location[3], 
                              marker_6_location[4], marker_6_location[5],cylinder_height, cylinder_radius)

lib.add_cylinder_to_workspace("beaker_1L", marker_15_location[0], marker_15_location[1],
                              marker_15_location[2] + container_height / 2.0, marker_15_location[3], 
                              marker_15_location[4], marker_15_location[5], container_height, container_radius)

rospy.sleep(1)

# Move above 0.1 meters the jar's location
cylinder = lib.get_object_location("grad_cylinder_100mL")
success = lib.go(cylinder[0], cylinder[1], cylinder[2] +  0.20, 
                 cylinder[3], cylinder[4], cylinder[5]) 

# # Move down to grasp the jar
success = lib.go(cylinder[0], cylinder[1], cylinder[2], 
                 cylinder[3], cylinder[4], cylinder[5])


# Close the gripper to grasp the cylinder
lib.close_gripper("grad_cylinder_100mL")
rospy.sleep(2)

# Move up 0.1 meters
success = lib.go(cylinder[0], cylinder[1], cylinder[2] +  0.1, 
                 cylinder[3], cylinder[4], cylinder[5])

# pour the contents
lib.pour("beaker_1L")

# place it back to its original location
success = lib.go(cylinder[0], cylinder[1], cylinder[2], 
                 cylinder[3], cylinder[4], cylinder[5])

# Open the gripper to release the jar
lib.open_gripper()

lib.move_to_home_position()
rospy.sleep(1)

print("Task finished!")