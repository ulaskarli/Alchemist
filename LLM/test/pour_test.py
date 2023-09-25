from Lib.ur5.FunctionLibrary import FunctionLib

import rospy

# Initialize rospy node called gpt
rospy.init_node('gpt')

# Initialize function library
lib = FunctionLib()

lib.move_to_home_position()
rospy.sleep(1)

print("Options:\n(1). beaker 1L (2). beaker 500mL (3). beaker 250mL (4). beaker 100mL (5). beaker 50mL\n(6). graduated cylinder 250mL (7). graduated cylinder 100mL (8). graduated cylinder 50mL (9). graduated cylinder 25mL (10). graduated cylinder 10mL")

obj_dict = {"1": "beaker 1L", "2": "beaker 500mL", "3": "beaker 250mL", "4": "beaker 100mL", "5": "beaker 50mL", "6": "graduated cylinder 250mL",
            "7": "graduated cylinder 100mL", "8": "graduated cylinder 50mL", "9": "graduated cylinder 25mL", "10": "graduated cylinder 10mL"}

obj_name = raw_input("\nwhich object do you wanna pick up:\n")

if obj_name == "-1":
    exit()

obj_num = raw_input("\nwhich marker is the object in front of:\n")

if obj_num == "-1":
    exit()

print("[pick up " + obj_dict[obj_name] + " in front of marker " + str(obj_num)+ "]")

container_name = raw_input("\nwhich container to pour into:\n")

if container_name == "-1":
    exit()

container_num = raw_input("\nwhich marker is the container in front of:\n")

if container_num == "-1":
    exit()

print("[pour into " + obj_dict[container_name] + " in front of marker " + str(container_num)+ "]")

# Define the jar dimensions
cylinder_height = lib.object_dimensions[obj_dict[obj_name]]["height"]
cylinder_radius = lib.object_dimensions[obj_dict[obj_name]]["radius"]

container_height = lib.object_dimensions[obj_dict[container_name]]["height"]
container_radius = lib.object_dimensions[obj_dict[container_name]]["radius"]


# Get the location of marker 6 and marker 15
marker_obj_location = lib.get_marker_location(obj_num)
marker_container_location = lib.get_marker_location(container_num)

if marker_obj_location is None: 
    print("Marker" +str(obj_num)+ "not found. Please check the environment.")
    exit()
        
if marker_container_location is None:
    print("Marker" +str(container_num)+ "not found. Please check the environment.")
    exit()
        
rospy.sleep(1)
# Add jar to the workspace in front of marker 6
lib.add_cylinder_to_workspace(obj_dict[obj_name], obj_num)

lib.add_cylinder_to_workspace(obj_dict[container_name], container_num)

rospy.sleep(1)

# Move above 0.1 meters the jar's location
cylinder = lib.get_object_location(obj_dict[obj_name])

success = lib.go(cylinder[0], cylinder[1], cylinder[2] +  0.20, 
                 cylinder[3], cylinder[4], cylinder[5]) 

# # Move down to grasp the jar
success = lib.go(cylinder[0], cylinder[1], cylinder[2], 
                 cylinder[3], cylinder[4], cylinder[5])


# Close the gripper to grasp the cylinder
lib.close_gripper(obj_dict[obj_name])
rospy.sleep(2)

# pour the contents
lib.pour(obj_dict[container_name])

# place it back to its original location
success = lib.go(cylinder[0], cylinder[1], cylinder[2], 
                 cylinder[3], cylinder[4], cylinder[5])

# Open the gripper to release the jar
lib.open_gripper()

lib.move_to_home_position()
rospy.sleep(1)

print("Task finished!")
# lib.scene.remove_world_object()