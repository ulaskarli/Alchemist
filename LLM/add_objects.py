from Lib.ur5.FunctionLibrary import FunctionLib
import rospy

# Initialize rospy node called gpt
rospy.init_node('gpt')

# Initialize function library
lib = FunctionLib()

def add_items_to_workspace():
    # Get the locations of the markers
    marker_15_location = lib.get_marker_location(15)
    marker_4_location = lib.get_marker_location(4)
    marker_6_location = lib.get_marker_location(6)
    marker_8_location = lib.get_marker_location(8)
    marker_9_location = lib.get_marker_location(9)

    # Check if any markers are missing
    if (
        marker_15_location is None or
        marker_4_location is None or
        marker_6_location is None or 
        marker_8_location is None or
        marker_9_location is None
    ):
        print("One or more markers not found. Please check the environment.")
        exit()

    # Add the 500 mL beaker in front of marker 15
    beaker_500ml_height = 0.115
    beaker_500ml_radius = 0.065
    lib.add_cylinder_to_workspace("beaker_500ml", marker_15_location[0], marker_15_location[1], 
                                  marker_15_location[2] + beaker_500ml_height / 2.0, 
                                  beaker_500ml_height, beaker_500ml_radius)

    # Add the 250 mL graduated cylinder in front of marker 4
    graduated_cylinder_250ml_height = 0.3
    graduated_cylinder_250ml_radius = 0.025
    lib.add_cylinder_to_workspace("grad_cylinder_250ml", marker_4_location[0], marker_4_location[1], 
                                  marker_4_location[2] + graduated_cylinder_250ml_height / 2.0, 
                                  graduated_cylinder_250ml_height, graduated_cylinder_250ml_radius)

    # Add the 100 mL graduated cylinder in front of marker 6
    graduated_cylinder_100ml_height = 0.245
    graduated_cylinder_100ml_radius = 0.02
    lib.add_cylinder_to_workspace("grad_cylinder_100ml", marker_6_location[0], marker_6_location[1], 
                                  marker_6_location[2] + graduated_cylinder_100ml_height / 2.0, 
                                  graduated_cylinder_100ml_height, graduated_cylinder_100ml_radius)

    # Add the 50 mL graduated cylinder in front of marker 8
    graduated_cylinder_50ml_height = 0.18
    graduated_cylinder_50ml_radius = 0.015
    lib.add_cylinder_to_workspace("grad_cylinder_50ml", marker_8_location[0], marker_8_location[1], 
                                  marker_8_location[2] + graduated_cylinder_50ml_height / 2.0, 
                                  graduated_cylinder_50ml_height, graduated_cylinder_50ml_radius)

    # Add the 25 mL graduated cylinder in front of marker 9
    graduated_cylinder_25ml_height = 0.145
    graduated_cylinder_25ml_radius = 0.0125
    lib.add_cylinder_to_workspace("grad_cylinder_25ml", marker_9_location[0], marker_9_location[1], 
                                  marker_9_location[2] + graduated_cylinder_25ml_height / 2.0, 
                                  graduated_cylinder_25ml_height, graduated_cylinder_25ml_radius)

# Call the function to add the items to the workspace
add_items_to_workspace()
