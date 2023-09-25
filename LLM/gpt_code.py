from Lib.panda.FunctionLibrary import FunctionLib
import rospy

# Initialize rospy node called gpt
rospy.init_node('gpt')

# Initialize function library
lib = FunctionLib()

# Define the names of the tubes
tubes = ["tube0", "tube1", "tube2"]

# Get the height of the tubes
tube_height = 0.1

# Iterate through each tube
for i in range(len(tubes)):
    tube_name = tubes[i]

    # Get the location of the current tube
    tube_location = lib.get_object_location(tube_name)

    # Determine the grasp orientation for picking up the tube
    grasp_orientation = lib.get_grasp_orientation(top=False)

    # Plan a trajectory to move the robot arm to the location of the tube
    # If the initial attempt fails, try different grasp orientations up to 5 times
    success = False
    count = 0
    while not success and count < 5:
        success = lib.go(tube_location[0], tube_location[1], tube_location[2], grasp_orientation[0], grasp_orientation[1], grasp_orientation[2])
        count += 1
        if not success:
            grasp_orientation = lib.get_grasp_orientation(top=False)

    if not success:
        print("Could not reach the tube: " + tube_name)
        continue

    # Close the gripper around the tube
    lib.close_gripper(tube_name, 0.04)

    # Lift the tube by moving the robot arm up
    target_location = [tube_location[0], tube_location[1], tube_location[2] + tube_height]
    lib.go(target_location[0], target_location[1], target_location[2], grasp_orientation[0], grasp_orientation[1], grasp_orientation[2])

    # If it's not the last tube, move to the next tube's location
    if i < len(tubes) - 1:
        next_tube_location = lib.get_object_location(tubes[i+1])

        # Plan a trajectory to move the robot arm to the next tube's location
        lib.go(next_tube_location[0], next_tube_location[1], next_tube_location[2], grasp_orientation[0], grasp_orientation[1], grasp_orientation[2])

        # Lower the tube by moving the robot arm down
        target_location = [next_tube_location[0], next_tube_location[1], next_tube_location[2] + tube_height]
        lib.go(target_location[0], target_location[1], target_location[2], grasp_orientation[0], grasp_orientation[1], grasp_orientation[2])

    # Open the gripper to release the tube
    lib.open_gripper()

