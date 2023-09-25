from Lib.ur5.FunctionLibrary import FunctionLib
import rospy

# Initialize rospy node called gpt
rospy.init_node('gpt')

# Initialize function library
lib = FunctionLib()

# Move the robot to home position
lib.move_to_home_position()

def add_objects_to_workspace():
    # Define the dimensions of the objects
    beaker_1L_height = 0.15
    beaker_1L_radius = 0.08
    beaker_500mL_height = 0.115
    beaker_500mL_radius = 0.065
    beaker_100mL_height = 0.07
    beaker_100mL_radius = 0.03
    beaker_50mL_height = 0.055
    beaker_50mL_radius = 0.025
    graduated_cylinder_250mL_height = 0.30
    graduated_cylinder_250mL_radius = 0.025
    graduated_cylinder_25mL_height = 0.145
    graduated_cylinder_25mL_radius = 0.0125
    
    # Get the location of markers
    marker_15_location = lib.get_marker_location(15)
    marker_7_location = lib.get_marker_location(7)
    marker_8_location = lib.get_marker_location(8)
    marker_6_location = lib.get_marker_location(6)
    marker_4_location = lib.get_marker_location(4)
    marker_9_location = lib.get_marker_location(9)
    
    # Add the 1L beaker to the workspace in front of marker 15
    lib.add_cylinder_to_workspace("beaker_1L", marker_15_location[0], marker_15_location[1],
                                  marker_15_location[2] + beaker_1L_height / 2.0, beaker_1L_height, beaker_1L_radius)
    
    # Add the 500mL beaker to the workspace in front of marker 7
    lib.add_cylinder_to_workspace("beaker_500mL", marker_7_location[0], marker_7_location[1],
                                  marker_7_location[2] + beaker_500mL_height / 2.0, beaker_500mL_height, beaker_500mL_radius)
    
    # Add the 100mL beaker to the workspace in front of marker 8
    lib.add_cylinder_to_workspace("beaker_100mL", marker_8_location[0], marker_8_location[1],
                                  marker_8_location[2] + beaker_100mL_height / 2.0, beaker_100mL_height, beaker_100mL_radius)
    
    # Add the 50mL beaker to the workspace in front of marker 6
    lib.add_cylinder_to_workspace("beaker_50mL", marker_6_location[0], marker_6_location[1],
                                  marker_6_location[2] + beaker_50mL_height / 2.0, beaker_50mL_height, beaker_50mL_radius)
    
    # Add the 250mL graduated cylinder to the workspace in front of marker 4
    lib.add_cylinder_to_workspace("grad_cylinder_250mL", marker_4_location[0], marker_4_location[1],
                                  marker_4_location[2] + graduated_cylinder_250mL_height / 2.0, graduated_cylinder_250mL_height, graduated_cylinder_250mL_radius)
    
    # Add the 25mL graduated cylinder to the workspace in front of marker 9
    lib.add_cylinder_to_workspace("grad_cylinder_25mL", marker_9_location[0], marker_9_location[1],
                                  marker_9_location[2] + graduated_cylinder_25mL_height / 2.0, graduated_cylinder_25mL_height, graduated_cylinder_25mL_radius)

add_objects_to_workspace()
