import rospy, moveit_commander,roslaunch, math
import moveit_msgs.msg, geometry_msgs.msg
import time, actionlib,sys,tf, tf2_ros, copy
import numpy as np
from tf import TransformListener
from std_msgs.msg import String, Bool
import geometry_msgs.msg
from actionlib import SimpleActionClient, GoalStatus
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from std_srvs.srv import Empty
from moveit_commander.conversions import pose_to_list
from franka_gripper.msg import GraspAction, GraspGoal, MoveAction, MoveGoal
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject


class FunctionLib:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        #self.node = rospy.init_node('function_library',log_level=rospy.INFO, anonymous=False)

        self.graspClient=SimpleActionClient("/franka_gripper/grasp",GraspAction)
        self.gripperMotion=SimpleActionClient('/franka_gripper/move', MoveAction)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        group_name = "panda_arm"
        self.move_group = moveit_commander.MoveGroupCommander(group_name)
        self.display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )
        self.plan=None
        self.object_dict={}
        self.tf_listener = tf.TransformListener()
    
    def get_current_end_effector_pose(self):
        pose=self.move_group.get_current_pose().pose
        roll,pitch,yaw=self.quaternion_to_euler(pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w)
        return [pose.position.x,pose.position.y,pose.position.z,roll,pitch,yaw]
    
    def matrix_to_quaternion(self, rotation_matrix):
        trace = np.trace(rotation_matrix)
        
        if trace > 0:
            S = np.sqrt(trace + 1.0) * 2
            qw = 0.25 * S
            qx = (rotation_matrix[2, 1] - rotation_matrix[1, 2]) / S
            qy = (rotation_matrix[0, 2] - rotation_matrix[2, 0]) / S
            qz = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) / S
        elif rotation_matrix[0, 0] > rotation_matrix[1, 1] and rotation_matrix[0, 0] > rotation_matrix[2, 2]:
            S = np.sqrt(1.0 + rotation_matrix[0, 0] - rotation_matrix[1, 1] - rotation_matrix[2, 2]) * 2
            qw = (rotation_matrix[2, 1] - rotation_matrix[1, 2]) / S
            qx = 0.25 * S
            qy = (rotation_matrix[0, 1] + rotation_matrix[1, 0]) / S
            qz = (rotation_matrix[0, 2] + rotation_matrix[2, 0]) / S
        elif rotation_matrix[1, 1] > rotation_matrix[2, 2]:
            S = np.sqrt(1.0 + rotation_matrix[1, 1] - rotation_matrix[0, 0] - rotation_matrix[2, 2]) * 2
            qw = (rotation_matrix[0, 2] - rotation_matrix[2, 0]) / S
            qx = (rotation_matrix[0, 1] + rotation_matrix[1, 0]) / S
            qy = 0.25 * S
            qz = (rotation_matrix[1, 2] + rotation_matrix[2, 1]) / S
        else:
            S = np.sqrt(1.0 + rotation_matrix[2, 2] - rotation_matrix[0, 0] - rotation_matrix[1, 1]) * 2
            qw = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) / S
            qx = (rotation_matrix[0, 2] + rotation_matrix[2, 0]) / S
            qy = (rotation_matrix[1, 2] + rotation_matrix[2, 1]) / S
            qz = 0.25 * S
            
        return qx, qy, qz, qw
    
    def euler_to_quaternion(self,roll, pitch, yaw):
        # Convert Euler angles (roll, pitch, yaw) to quaternions
        roll=math.radians(roll)
        pitch=math.radians(pitch)
        yaw=math.radians(yaw)

        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        w = cy * cp * cr + sy * sp * sr
        x = cy * cp * sr - sy * sp * cr
        y = sy * cp * sr + cy * sp * cr
        z = sy * cp * cr - cy * sp * sr

        return [w, x, y, z]
    
    def quaternion_to_euler(self, x, y, z, w):
        # Convert quaternion to Euler angles (roll, pitch, yaw)
        t0 = 2.0 * (w * x + y * z)
        t1 = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)

        t2 = 2.0 * (w * y - z * x)
        t2 = 1.0 if t2 > 1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)

        t3 = 2.0 * (w * z + x * y)
        t4 = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)

        return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)
    
    def close_gripper(self,name,width):
        self.graspClient.wait_for_server()
        self.attach_object_to_gripper(name)

        goal=GraspGoal()
        goal.width=width
        goal.epsilon.inner=0.1
        goal.epsilon.outer=0.1
        goal.speed=0.1
        goal.force=20

        self.graspClient.send_goal(goal)
        self.graspClient.wait_for_result()

        return self.graspClient.get_result()

    def open_gripper(self):
        self.gripperMotion.wait_for_server()
        goal = MoveGoal(width=0.07, speed=1.0)
        self.gripperMotion.send_goal(goal)
        self.gripperMotion.wait_for_result()
        self.detach_object_from_gripper()

        return self.gripperMotion.get_result() 

    def get_marker_location(self, marker_number):
        while True:
            try:
                self.tf_listener.waitForTransform('/world', '/ar_marker_' + str(marker_number), rospy.Time(0), rospy.Duration(10.0))
                (trans, rot) = self.tf_listener.lookupTransform('/world', '/ar_marker_' + str(marker_number), rospy.Time(0))
                return trans
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logwarn("Marker %d not found. Waiting for marker to appear...", marker_number)
                rospy.sleep(1.0)

    def move_arm(self, x, y, z, roll, pitch ,yaw):

        waypoints=[]
        ori=self.euler_to_quaternion(roll,pitch,yaw)

        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = z

        pose_goal.orientation.w = ori[0]
        pose_goal.orientation.x = ori[1]
        pose_goal.orientation.y = ori[2]
        pose_goal.orientation.z = ori[3]

        current_position=self.move_group.get_current_pose().pose.position

        mid_pose=geometry_msgs.msg.Pose()
        mid_pose.position.x = (current_position.x+x)/2
        mid_pose.position.y = (current_position.y+y)/2
        mid_pose.position.z = (current_position.z+z)/2
        mid_pose.orientation.w = ori[0]
        mid_pose.orientation.x = ori[1]
        mid_pose.orientation.y = ori[2]
        mid_pose.orientation.z = ori[3]

        pose_goal_top=copy.deepcopy(pose_goal)
        pose_goal_top.position.z = z+0.1

        waypoints.append(mid_pose)
        # waypoints.append(pose_goal_top)
        waypoints.append(pose_goal)


        (plan, fraction) = self.move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
        )

        self.plan=plan

    def display_trajectory(self):
        if self.plan==None:
            pass
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(self.plan)
        self.display_trajectory_publisher.publish(display_trajectory)

    def execute_plan(self):
        if self.plan==None:
            pass
        self.move_group.execute(self.plan, wait=True)

    def add_cylinder_to_workspace(self,name,x,y,z,height,radius):
        self.object_dict[name]=[x,y,z, height, radius]
        pose = geometry_msgs.msg.PoseStamped()
        pose.header.frame_id = "world"
        pose.pose.orientation.w = 1.0
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z+height/2

        self.scene.add_cylinder(name, pose, height, radius)

    def add_box_to_workspace(self,name,x,y,z,h,l,w):
        self.object_dict[name]=[x,y,z,h,l,w]
        pose = geometry_msgs.msg.PoseStamped()
        pose.header.frame_id = "world"
        pose.pose.orientation.w = 1.0
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z+h/2

        self.scene.add_box(name, pose, size=(h,l,w))

    def attach_object_to_gripper(self, name):
        #self.scene.removeCollisionObject(name,wait=True)
        eef_link = self.move_group.get_end_effector_link()
        grasping_group = "panda_hand"
        touch_links = self.robot.get_link_names(group=grasping_group)
        obj=self.scene.get_objects([name])
        aco = AttachedCollisionObject()
        aco.object = obj[name]
        aco.link_name = eef_link
        aco.touch_links = touch_links
        self.scene.attach_object(aco)
        #p=self.object_dict[name]
        #self.scene.attach_box(eef_link,p[-1],p[-1],p[3],p[0],p[1],p[2],name, touch_links=touch_links)


    def detach_object_from_gripper(self):
        self.scene.remove_attached_object()






