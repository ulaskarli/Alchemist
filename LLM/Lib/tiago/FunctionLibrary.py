import rospy, moveit_commander,roslaunch, math
import moveit_msgs.msg, geometry_msgs.msg
import speech_recognition as sr
import time, actionlib,sys,tf, tf2_ros
import numpy as np
from tf import TransformListener
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist,PoseStamped
from sensor_msgs.msg import Image
from actionlib import SimpleActionClient, GoalStatus
from pal_interaction_msgs.msg import TtsAction, TtsGoal
from control_msgs.msg import PointHeadActionGoal
from geometry_msgs.msg import PointStamped
from math import pi, tau, dist, fabs, cos
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from move_base_msgs.msg import MoveBaseAction, MoveBaseActionGoal, MoveBaseActionFeedback
from pal_common_msgs.msg import DisableActionGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from std_srvs.srv import Empty
from aruco_msgs.msg import MarkerArray
from moveit_commander.conversions import pose_to_list
from cv_bridge import CvBridge, CvBridgeError


class FunctionLib:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        self.node = rospy.init_node('function_library',log_level=rospy.INFO, anonymous=False)
        rospy.loginfo("Node %s initialized", 'function_library')
        self.arm_right = actionlib.SimpleActionClient("/safe_arm_right_controller/follow_joint_trajectory",FollowJointTrajectoryAction)
        rospy.loginfo("%s action client initialized", '/safe_arm_right_controller/follow_joint_trajectory')
        self.arm_left = actionlib.SimpleActionClient("/safe_arm_left_controller/follow_joint_trajectory",FollowJointTrajectoryAction)
        rospy.loginfo("%s action client initialized", '/safe_arm_left_controller/follow_joint_trajectory')
        self.torso = actionlib.SimpleActionClient("/safe_torso_controller/follow_joint_trajectory",FollowJointTrajectoryAction)
        rospy.loginfo("%s action client initialized", '/safe_torso_controller/follow_joint_trajectory')
        self.head = actionlib.SimpleActionClient("/head_controller/follow_joint_trajectory",FollowJointTrajectoryAction)
        rospy.loginfo("%s action client initialized", '/head_controller/follow_joint_trajectory')
        self.alive_disable = rospy.Publisher('/pal_head_manager/disable/goal', DisableActionGoal,queue_size=1)
        rospy.loginfo("%s publisher initialized", '/pal_head_manager/disable/goal')
        self.talker = SimpleActionClient('/tts', TtsAction)
        rospy.loginfo("%s action client initialized", '/tts')
        self.base = SimpleActionClient('/move_base',MoveBaseAction)
        rospy.loginfo("%s action client initialized", '/move_base')
        self.tf=tf.TransformListener()
        rospy.loginfo("%s initialized", 'tf')
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        left_group_name = "arm_left_torso"
        right_group_name = "arm_right_torso"
        self.left_move_group = moveit_commander.MoveGroupCommander(left_group_name)
        self.right_move_group = moveit_commander.MoveGroupCommander(right_group_name)
        self.display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        p=self.get_current_pose('map','base_footprint')
        self.base_pose=[p[0],p[1],p[3],p[4],p[5],p[6]]
        self.playMotion=SimpleActionClient('/play_motion', PlayMotionAction)

    def talk(self,prompt):
        #input takes in a prompt as a string and makes tiago talk using text-to-speech
        #output returns nothing 
        #This function can be used to make tiago robot say any desired prompt to the user
        self.talker.wait_for_server()
        goal = TtsGoal()
        goal.rawtext.text = prompt
        goal.rawtext.lang_id = "en_GB"
        self.talker.send_goal_and_wait(goal)

    def send_joint_goal(self,joint_names,joint_trajectory,time):
        stopGoal=DisableActionGoal()
        stopGoal.goal.duration=sum(time)+60.0
        self.alive_disable.publish(stopGoal)
        if 'torso_lift_joint' in joint_names:
            joint=self.torso
        elif 'head_1_joint' or 'head_2_joint' in joint_names:
            joint=self.head
        elif 'arm_left' in joint_names[0]:
            joint=self.arm_left
        elif 'arm_right' in joint_names[0]:
            joint=self.arm_right
        else:
            pass
        rospy.loginfo("Waiting for follow_joint_trajectory server")
        joint.wait_for_server()
        rospy.loginfo("Connected to follow_joint_trajectory server")
        rospy.sleep(1)
        joint_trajectory = JointTrajectory()
        joint_trajectory.joint_names = joint_names
        i=0
        for point in joint_trajectory:
            joint_trajectory.points.append(JointTrajectoryPoint())
            joint_trajectory.points[i].positions = point
            joint_trajectory.points[i].time_from_start = rospy.Duration(time[i])
            i+=1
        rospy.loginfo("Preparing for moving the joint to goal position!")
        rospy.sleep(1)
        joint_goal_pos = FollowJointTrajectoryGoal()
        joint_goal_pos.trajectory = joint_trajectory

        joint_goal_pos.goal_time_tolerance = rospy.Duration(0)
        joint.send_goal(joint_goal_pos)
        rospy.loginfo("Send goal to the trajectory server successfully!")
        joint.wait_for_result()

    def move_to(self,x,y,qx,qy,qz,qw):
        self.base.wait_for_server()
        nav=MoveBaseActionGoal()
        p=PoseStamped()
        p.header.frame_id='map'
        p.pose.position.x=x
        p.pose.position.y=y
        p.pose.orientation.x=qx
        p.pose.orientation.y=qy
        p.pose.orientation.z=qz
        p.pose.orientation.w=qw
        nav.goal.target_pose=p
        self.base.send_goal(nav.goal,feedback_cb=self._save_current_base_pose)
        result=self.base.wait_for_result()
    
    def get_current_pose(self,frame1,frame2):
        trans = self.tfBuffer.lookup_transform(frame1, frame2, rospy.Time(),rospy.Duration(5.0))
        transform,rot=self.tf.lookupTransform(frame1, frame2, rospy.Time(0))
        return [transform[0],transform[1],transform[2],rot[0],rot[1],rot[2],rot[3]]
    
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

        return [x, y, z, w]
    
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
    
    def _save_current_base_pose(self,feedback):
        pose=feedback.base_position.pose
        self.base_pose=[pose.position.x,pose.position.y,pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w]
    
    def get_base_pose(self):
        return self.base_pose
    
    def close_gripper(self,side='left'):
        if side=='left':
            rospy.wait_for_service('/parallel_gripper_left_controller/grasp')
            grasp=rospy.ServiceProxy('/parallel_gripper_left_controller/grasp',Empty)
            grasp()
        elif side=='right':
            rospy.wait_for_service('/parallel_gripper_right_controller/grasp')
            grasp=rospy.ServiceProxy('/parallel_gripper_right_controller/grasp',Empty)
            grasp()
        else:
            rospy.logwarn('Incorrect side selection. Please select either left or right.')

    def move_arm(self,side, x, y, z, roll, pitch ,yaw):

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "base_footprint"
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = z
        q = self.euler_to_quaternion(roll, pitch, yaw)
        goal_pose.pose.orientation.x = q[0]
        goal_pose.pose.orientation.y = q[1]
        goal_pose.pose.orientation.z = q[2]
        goal_pose.pose.orientation.w = q[3]

        if side=='left':
            self.left_move_group.set_pose_target(goal_pose)
            success = self.left_move_group.go(wait=True)
            self.left_move_group.stop()
            self.left_move_group.clear_pose_targets()
            return success
        elif side=='right':
            self.right_move_group.set_pose_target(goal_pose)
            success = self.right_move_group.go(wait=True)
            self.right_move_group.stop()
            self.right_move_group.clear_pose_targets()
            return success
        else:
            rospy.logwarn("Please choose correct side to move.")
            return False
        
    def get_current_arm_pose(self,side):

        if side=='left':
            pose = self.left_move_group.get_current_pose().pose
            euler= self.quaternion_to_euler(pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w)
            return [pose.position.x,pose.position.y,pose.position.z,euler[0],euler[1],euler[2]]
        elif side=='right':
            pose = self.right_move_group.get_current_pose().pose
            euler= self.quaternion_to_euler(pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w)
            return [pose.position.x,pose.position.y,pose.position.z,euler[0],euler[1],euler[2]]
        else:
            rospy.logwarn("Please choose correct side to move.")
            pose=PoseStamped().pose
            euler= self.quaternion_to_euler(pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w)
            return [pose.position.x,pose.position.y,pose.position.z,euler[0],euler[1],euler[2]]

    def play(self, goalString):

        rospy.loginfo("Waiting for Action Server...")
        self.playMotion.wait_for_server()

        goal = PlayMotionGoal()
        goal.motion_name = goalString
        goal.skip_planning = False
        goal.priority = 0  # Optional

        rospy.loginfo("Sending goal with motion: " + goalString)
        self.playMotion.send_goal(goal)

        rospy.loginfo("Waiting for result...")
        action_ok = self.playMotion.wait_for_result(rospy.Duration(30.0))

        state = self.playMotion.get_state()

        #if action_ok:
        #    rospy.loginfo("Action finished succesfully with state: " + str(get_status_string(state)))
        #else:
        #    rospy.logwarn("Action failed with state: " + str(get_status_string(state)))



