from Lib.ur5.FunctionLibrary import FunctionLib

import rospy

# Initialize rospy node called gpt
rospy.init_node('gpt')

# Initialize function library
lib = FunctionLib()

lib.move_to_home_position()
rospy.sleep(1)

# Add 500mL beaker to the workspace
beaker_500 = lib.get_marker_location(9)
if beaker_500 is not None:
    lib.add_cylinder_to_workspace("beaker 500mL", beaker_500[0], beaker_500[1], beaker_500[2],
                                  beaker_500[3], beaker_500[4], beaker_500[5])

# Add 250mL graduated cylinder to the workspace
graduated_cylinder_250 = lib.get_marker_location(8)
if graduated_cylinder_250 is not None:
    lib.add_cylinder_to_workspace("graduated cylinder 250mL", graduated_cylinder_250[0], graduated_cylinder_250[1],
                                  graduated_cylinder_250[2], graduated_cylinder_250[3],
                                  graduated_cylinder_250[4], graduated_cylinder_250[5])

# Add 100mL graduated cylinder to the workspace
graduated_cylinder_100 = lib.get_marker_location(7)
if graduated_cylinder_100 is not None:
    lib.add_cylinder_to_workspace("graduated cylinder 100mL", graduated_cylinder_100[0], graduated_cylinder_100[1],
                                  graduated_cylinder_100[2], graduated_cylinder_100[3],
                                  graduated_cylinder_100[4], graduated_cylinder_100[5])

# Add 50mL graduated cylinder to the workspace
graduated_cylinder_50 = lib.get_marker_location(6)
if graduated_cylinder_50 is not None:
    lib.add_cylinder_to_workspace("graduated cylinder 50mL", graduated_cylinder_50[0], graduated_cylinder_50[1],
                                  graduated_cylinder_50[2], graduated_cylinder_50[3],
                                  graduated_cylinder_50[4], graduated_cylinder_50[5])

# Add 25mL graduated cylinder to the workspace
graduated_cylinder_25 = lib.get_marker_location(5)
if graduated_cylinder_25 is not None:
    lib.add_cylinder_to_workspace("graduated cylinder 25mL", graduated_cylinder_25[0], graduated_cylinder_25[1],
                                  graduated_cylinder_25[2], graduated_cylinder_25[3],
                                  graduated_cylinder_25[4], graduated_cylinder_25[5])

# Add 10mL graduated cylinder to the workspace
graduated_cylinder_10 = lib.get_marker_location(4)
if graduated_cylinder_10 is not None:
    lib.add_cylinder_to_workspace("graduated cylinder 10mL", graduated_cylinder_10[0], graduated_cylinder_10[1],
                                  graduated_cylinder_10[2], graduated_cylinder_10[3],
                                  graduated_cylinder_10[4], graduated_cylinder_10[5])

print("Objects added to the workspace")

print("Options:\n(1). beaker 1L (2). beaker 500mL (3). beaker 250mL (4). beaker 100mL (5). beaker 50mL\n(6). graduated cylinder 250mL (7). graduated cylinder 100mL (8). graduated cylinder 50mL (9). graduated cylinder 25mL (10). graduated cylinder 10mL")

obj_dict = {"1": "beaker 1L", "2": "beaker 500mL", "3": "beaker 250mL", "4": "beaker 100mL", "5": "beaker 50mL", "6": "graduated cylinder 250mL",
            "7": "graduated cylinder 100mL", "8": "graduated cylinder 50mL", "9": "graduated cylinder 25mL", "10": "graduated cylinder 10mL"}

obj_name = raw_input("\nwhich object do you wanna pick up:\n")

if obj_name == "-1":
    exit()


# Define the jar dimensions
cylinder_height = lib.object_dimensions[obj_dict[obj_name]]["height"]
cylinder_radius = lib.object_dimensions[obj_dict[obj_name]]["radius"]

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

success = lib.go(cylinder[0], cylinder[1], cylinder[2], 
                 cylinder[3], cylinder[4], cylinder[5])

# Open the gripper to release the jar
lib.open_gripper()

lib.move_to_home_position()
rospy.sleep(1)

print("Task finished!")
# lib.scene.remove_world_object()