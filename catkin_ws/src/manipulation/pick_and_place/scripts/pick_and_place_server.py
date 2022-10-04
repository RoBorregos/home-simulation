#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import rospy
from actionlib import SimpleActionClient, SimpleActionServer
import tf2_ros
import tf.transformations as transformations
import moveit_commander
from moveit_commander import PlanningSceneInterface
from moveit_msgs.msg import Grasp, GripperTranslation, PickupAction, PickupGoal, PickupResult, MoveItErrorCodes
from moveit_msgs.msg import PlaceAction, PlaceGoal, PlaceResult, PlaceLocation
from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Vector3Stamped, Vector3, Quaternion
from nav_msgs.msg import Odometry
from pick_and_place.msg import PickAndPlaceAction, PickAndPlaceGoal, PickAndPlaceResult, PickAndPlaceFeedback
from moveit_msgs.srv import GetPlanningScene, GetPlanningSceneRequest, GetPlanningSceneResponse
from std_srvs.srv import Empty, EmptyRequest
from std_msgs.msg import Header
from gpd_ros.msg import GraspConfig, GraspConfigList
import numpy

moveit_error_dict = {}
for name in MoveItErrorCodes.__dict__.keys():
    if not name[:1] == '_':
        code = MoveItErrorCodes.__dict__[name]
        moveit_error_dict[code] = name

ARM_GROUP="arm"
GRIPPER_GROUP="hand"

# Function to create a PickupGoal with the provided data.

def createPickupGoal(group=ARM_GROUP, target="part",
                     possible_grasps=[],
                     links_to_allow_contact=[],
                     support_surface_name = "<octomap>"):
    pug = PickupGoal()
    pug.target_name = target
    pug.group_name = group
    pug.possible_grasps.extend(possible_grasps)
    pug.support_surface_name = support_surface_name
    pug.allow_gripper_support_collision = True
    pug.allowed_planning_time = 30.0
    pug.planning_options.planning_scene_diff.is_diff = True
    pug.planning_options.planning_scene_diff.robot_state.is_diff = True
    pug.planning_options.plan_only = False
    pug.planning_options.replan = True
    pug.planning_options.replan_attempts = 3
    pug.allowed_touch_objects = ['<octomap>']
    pug.attached_object_touch_links = ['<octomap>']
    pug.attached_object_touch_links.extend(links_to_allow_contact)
    return pug

# Function to create a PlaceGoal with the provided data..
def createPlaceGoal(place_locations,
                    group=ARM_GROUP,
                    target="part",
                    links_to_allow_contact=[],
                    support_surface_name = "<octomap>"):
    placeg = PlaceGoal()
    placeg.group_name = group
    placeg.support_surface_name = support_surface_name
    placeg.allow_gripper_support_collision = True
    placeg.attached_object_name = target
    placeg.place_locations = place_locations
    placeg.allowed_planning_time = 30.0
    placeg.planning_options.planning_scene_diff.is_diff = True
    placeg.planning_options.planning_scene_diff.robot_state.is_diff = True
    placeg.planning_options.plan_only = False
    placeg.planning_options.replan = True
    placeg.planning_options.replan_attempts = 3
    placeg.allowed_touch_objects = ['<octomap>']
    placeg.allowed_touch_objects.extend(links_to_allow_contact)
    return placeg

# Function to convert from GPD to MoveIt
def gpd_to_moveit(grasp_config_list):
    header = Header()
    msg.grasp_pose.header.frame_id = "base_footprint"
    approach_distance = 0.1
    eef_yaw_offset = 0.78
    eef_offset = 0.154 # M_PI / 4;
    table_height = 0.1
    object_height_min = 0.028
    kThresholdScore = 1.0
    res = []
    for grasp_config in grasp_config_list:
        msg = Grasp()
        msg.grasp_pose.header = header
        # TODO: Use kThresholdScore according to this.
        msg.grasp_quality = grasp_config.score.data

        offset = eef_offset
        # Make sure a distance of 'object_height_min/2' from tabletop to fingertip.
        offset += (table_height + object_height_min / 2) - grasp_config.top.z
        rospy.loginfo("offset is %f", offset)

        # Set grasp position, translation from hand-base to the parent-link of EEF
        msg.grasp_pose.pose.position.x = grasp_config.bottom.x - grasp_config.approach.x * offset
        msg.grasp_pose.pose.position.y = grasp_config.bottom.y - grasp_config.approach.y * offset
        msg.grasp_pose.pose.position.z = grasp_config.bottom.z - grasp_config.approach.z * offset

        # Rotation Matrix
        rot = numpy.array([[grasp_config.binormal.x, grasp_config.axis.x, grasp_config.approach.x],
                        [grasp_config.binormal.y, grasp_config.axis.y, grasp_config.approach.y],
                        [grasp_config.binormal.z, grasp_config.axis.z, grasp_config.approach.z]])
        R = numpy.eye(4)
        R[:3, :3] = rot
        quat = transformations.quaternion_from_matrix(R)

        # EEF yaw-offset to its parent-link (last link of arm)
        offquat = transformations.quaternion_about_axis(eef_yaw_offset, (0, 0, 1))
        quat = transformations.quaternion_multiply(quat, offquat)
        quat = transformations.unit_vector(quat)
        # Set grasp orientation
        msg.grasp_pose.pose.orientation.x = quat[0]
        msg.grasp_pose.pose.orientation.y = quat[1]
        msg.grasp_pose.pose.orientation.z = quat[2]
        msg.grasp_pose.pose.orientation.w = quat[3]
        rospy.loginfo("*** MoveIt pick pose/tool0 " + msg.grasp_pose)

        # Set pre-grasp approach
        msg.pre_grasp_approach.direction.header = header
        msg.pre_grasp_approach.direction.vector = grasp_config.approach
        msg.pre_grasp_approach.min_distance = approach_distance / 2
        msg.pre_grasp_approach.desired_distance = approach_distance

        # Set post-grasp retreat
        msg.post_grasp_retreat.direction.header = header
        msg.post_grasp_retreat.direction.vector.x = -grasp_config.approach.x
        msg.post_grasp_retreat.direction.vector.y = -grasp_config.approach.y
        msg.post_grasp_retreat.direction.vector.z = -grasp_config.approach.z
        msg.post_grasp_retreat.min_distance = approach_distance / 2
        msg.post_grasp_retreat.desired_distance = approach_distance

        res.append(msg)

    return res

class PickAndPlaceServer(object):
    def __init__(self):
        self.tfBuffer = tf2_ros.Buffer()
        self.tf_l = tf2_ros.TransformListener(self.tfBuffer)

        rospy.loginfo("Initalizing PickAndPlaceServer...")
        rospy.loginfo("Connecting to pickup AS")
        self.pickup_ac = SimpleActionClient('/pickup', PickupAction)
        self.pickup_ac.wait_for_server()
        rospy.loginfo("Succesfully connected.")
        rospy.loginfo("Connecting to place AS")
        self.place_ac = SimpleActionClient('/place', PlaceAction)
        self.place_ac.wait_for_server()
        rospy.loginfo("Succesfully connected.")
        self.scene = PlanningSceneInterface()
        rospy.loginfo("Connecting to /get_planning_scene service")
        self.scene_srv = rospy.ServiceProxy(
            '/get_planning_scene', GetPlanningScene)
        self.scene_srv.wait_for_service(timeout=100)
        rospy.loginfo("Connected.")

        rospy.loginfo("Connecting to clear octomap service...")
        self.clear_octomap_srv = rospy.ServiceProxy(
            '/clear_octomap', Empty)
        self.clear_octomap_srv.wait_for_service(timeout=100)
        rospy.loginfo("Connected!")
        
        self.gripper_group = moveit_commander.MoveGroupCommander(GRIPPER_GROUP, wait_for_servers = 0)

        rospy.loginfo("Gripper Group Up!")
        self.pick_as = SimpleActionServer(
            '/pickup_pose', PickAndPlaceAction,
            execute_cb=self.pick_cb, auto_start=False)
        self.pick_as.start()

        rospy.loginfo("Action Server1 Up!")
        self.place_as = SimpleActionServer(
            '/place_pose', PickAndPlaceAction,
            execute_cb=self.place_cb, auto_start=False)
        self.place_as.start()

        rospy.loginfo("Action Server2 Up!")
        
    
    
    # Function to handle pickup callback.
    def pick_cb(self, goal):
        """
        :type goal: PickAndPlaceGoal
        """
        grasps = gpd_to_moveit(goal.grasp_config_list)
        error_code = self.grasp_object(grasps, goal.object_name, goal.allow_contact_with)
        p_res = PickAndPlaceResult()
        p_res.error_code = error_code
        if error_code != 1:
            self.pick_as.set_aborted(p_res)
        else:
            self.pick_as.set_succeeded(p_res)

    # Function to handle place callback.
    def place_cb(self, goal):
        """
        :type goal: PickAndPlaceGoal
        """
        error_code = self.place_object(goal.object_pose, goal.object_name, goal.allow_contact_with)
        p_res = PickAndPlaceResult()
        p_res.error_code = error_code
        if error_code != 1:
            self.place_as.set_aborted(p_res)
        else:
            self.place_as.set_succeeded(p_res)

    # Function to confirm that an object is or not attached.
    def confirm_status_with_attached_objects(self, code_error, object_name, expected):
        rospy.sleep(0.5) # Give some time to appear in attached_objects
        scene_attached_objects = self.scene.get_attached_objects([object_name])
        if expected:
            if code_error == 1 and not (object_name in scene_attached_objects):
                code_error = -1
            
            if code_error != 1 and (object_name in scene_attached_objects):
                code_error = 1
        else:
            if code_error == 1 and (object_name in scene_attached_objects):
                code_error = -1
            
            if code_error != 1 and not (object_name in scene_attached_objects):
                code_error = 1

        return code_error

    # Function to grasp an object.
    def grasp_object(self, grasps, object_name, allow_contact_with = []):
        # Confirm if not attached already
        scene_attached_objects = self.scene.get_attached_objects([object_name])
        rospy.loginfo("Attached objects:" + str(scene_attached_objects))
        if object_name in scene_attached_objects:
            return

        possible_grasps = grasps

        links_to_allow_contact = []

        links_to_allow_contact.extend(allow_contact_with)

        goal = createPickupGoal(
            ARM_GROUP, object_name, possible_grasps, links_to_allow_contact)
        
        error_code = self.handle_pick_as(goal)

        # Confirm STATUS looking into attached objects
        error_code = self.confirm_status_with_attached_objects(error_code, object_name, True)

        rospy.logwarn(
            "Pick result: " +
        str(moveit_error_dict[error_code]))

        return error_code

    # Function to handle pick action server call.
    def handle_pick_as(self, goal):
        class PickScope:
            state = ""
            error_code = 0
            result_received = False

        def pick_feedback(feedback_msg):
            PickScope.state = feedback_msg.state
            pass
        
        def pick_callback(state, result):
            rospy.logwarn("Result Received")
            PickScope.error_code = result.error_code.val
            PickScope.result_received = True

        rospy.loginfo("Sending goal")
        self.pickup_ac.send_goal(goal,
                    feedback_cb=pick_feedback,
                    done_cb=pick_callback)
        
        while not PickScope.result_received:
            pass

        return PickScope.error_code

    # Function to handle place action server call.
    def handle_place_as(self, goal):
        class PlaceScope:
            state = ""
            error_code = 0
            result_received = False

        def place_feedback(feedback_msg):
            PlaceScope.state = feedback_msg.state
            rospy.logwarn("Feedback Received: " + str(PlaceScope.state))
            pass
        
        def place_callback(state, result):
            rospy.logwarn("Result Received")
            PlaceScope.error_code = result.error_code.val
            PlaceScope.result_received = True

        rospy.loginfo("Sending goal")
        self.place_ac.send_goal(goal,
                    feedback_cb=place_feedback,
                    done_cb=place_callback)
        
        while not PlaceScope.result_received:
            pass

        return PlaceScope.error_code
    
    # Function to place an object.
    def place_object(self, object_pose, object_name, allow_contact_with = [], try_only_with_arm_first = False):

        possible_placings = [] # TODO: Get Placing Positions
        
        links_to_allow_contact = []

        links_to_allow_contact.extend(allow_contact_with)

        error_code = -1

        
        rospy.loginfo("Trying to place with arm and torso")
        
        goal = createPlaceGoal(
            possible_placings, ARM_GROUP, object_name, links_to_allow_contact)

        error_code = self.handle_place_as(goal)
    
        # Confirm STATUS looking into attached objects
        error_code = self.confirm_status_with_attached_objects(error_code, object_name, False)

        rospy.logwarn(
            "Place Result: " +
        str(moveit_error_dict[error_code]))

        return error_code

if __name__ == '__main__':
    rospy.init_node('pick_and_place_server')
    paps = PickAndPlaceServer()
    rospy.spin()
