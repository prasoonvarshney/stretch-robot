#! /usr/bin/env python

import hello_helpers.hello_misc as hm

import rospy
import logging
import tf
import tf2_ros
import time
from math import pi, sqrt, atan2
import json 
import datetime
import sys
import math

from std_srvs.srv import Trigger

import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped, PoseStamped, Point
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from visualization_msgs.msg import MarkerArray
from actionlib_msgs.msg import *

import actionlib
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction


from constants import *

COORDS = json.load(open('coordinates.json', 'r'))
GOAL = COORDS['couch_team2']

def get_angles_from_quaternion(quaternion):
    return tf.transformations.euler_from_quaternion(
        (quaternion[0], quaternion[1], quaternion[2], quaternion[3])
    )


class ArucoNavigation(hm.HelloNode):
    def __init__(self):
        hm.HelloNode.__init__(self)
        self.init_translation, self.init_orientation = None, None
        self.base_translation, self.base_orientation = None, None
        self.tag_translation,  self.tag_orientation  = None, None
        self.start_time = None
        self.found_basket_time = None
        self.nav_basket_time = None
        self.pickup_basket_time = None
        self.nav_couch_time = None
        self.place_basket_time = None
        self.nav_home_time = None
        self.output_fname = datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S") + ".txt"
        self.mode = 'nav_to_aruco'
        self.main()

    def find_tag(self, requested_tag):
        '''
        Pans the head at three tilt angles to search for the requested frame (usually either the name of an aruco tag or "map"). Cycles up for up to two full searches (a total of 6 rotations) before timing 
        out. If the frame is found, a tf_listener finds the pose of the base_link in the requested frame and saves it in the translation and rotation variables for use in the next functions.
        '''

        # self.switch_to_position_mode()

        min_rotation = -4.05
        max_rotation = 1.78
        num_steps = 10
        step = abs(min_rotation - max_rotation) / num_steps

        pose = {'joint_head_tilt': -1*pi/4, 'joint_head_pan': min_rotation}
        self.move_to_pose(pose)

        found_tag = False

        count = 0
        while not found_tag:
            
            command = {'joint': 'joint_head_pan', 'delta': step}
            self.send_command(command)
            time.sleep(.5)

            try:
                rospy.loginfo('LOOKING FOR THIS TAG: ')
                rospy.loginfo(requested_tag)
                self.tf_listener.waitForTransform('map', requested_tag, rospy.Time(), rospy.Duration(4.0))
                self.tag_translation, self.tag_rotation = self.tf_listener.lookupTransform('map', requested_tag, rospy.Time(0))
                rospy.loginfo("Found Requested Tag")
                
                found_tag = True
            
            except:
                
                # Check if the head has completed a full rotation
                if self.joint_state.position[self.joint_state.name.index('joint_head_pan')] > (max_rotation - step):
                    
                    pose = {'joint_head_pan': min_rotation}
                    self.move_to_pose(pose)

                    # After a full head rotation, change the head tilt 
                    if self.joint_state.position[self.joint_state.name.index('joint_head_tilt')] >= -0.1:
                        pose = {'joint_head_tilt': -1*pi/4}
                        self.move_to_pose(pose)
                        count += 1
                    else:
                        command = {'joint': 'joint_head_tilt', 'delta': pi/8}
                        self.send_command(command)

                    time.sleep(.5)
    
            if count >= 4:
                rospy.loginfo("Timed Out Looking for Tag")
                # self.switch_to_navigation_mode()
                return False

        self.switch_to_navigation_mode()
        return True
    

    def move_joints(self, pose):
        print(f"Moving to pose: {pose}")
        self.move_to_pose(pose, return_before_done=False)
        time.sleep(0.5)

    def record_times(self):
        times = {
            "start": self.start_time,
            "found_basket": self.found_basket_time,
            "nav_basket": self.nav_basket_time,
            "pickup_basket": self.pickup_basket_time,
            "nav_couch": self.nav_couch_time,
            "place_basket": self.place_basket_time,
            "nav_home": self.nav_home_time,
        }
        with open(self.output_fname, 'w') as f:
            f.write(json.dumps(times, indent=2))

    def record_distance_from_goal(self, file_suffix, reached_translation, reached_orientation, goal_translation, goal_orientation):
        reached_x = reached_translation[0]
        reached_y = reached_translation[1]
        goal_x = round(goal_translation[0], 2)
        goal_y = round(goal_translation[1], 2)

        diff_x = abs(reached_x - goal_x)
        diff_y = abs(reached_y - goal_y)

        reached_z_angle = math.degrees(get_angles_from_quaternion(reached_orientation)[2])
        goal_z_angle = math.degrees(get_angles_from_quaternion(goal_orientation)[2])
        diff_d = abs(reached_z_angle - goal_z_angle)

        diffs = {
            "target_x": goal_x,
            "error_x": diff_x,
            "target_y": goal_y,
            "error_y": diff_y,
            "target_d": goal_z_angle,
            "error_d": diff_d,
        }
        with open(self.output_fname + file_suffix, 'w') as f:
            f.write(json.dumps(diffs, indent=2))


    def callback(self, goal):
        if self.mode == 'nav_to_aruco':

            if self.start_time is None: 
                self.start_time = time.time()

            requested_tag = "basket"
            rospy.loginfo(f"LOOKING FOR THIS TAG: {requested_tag}")

            # NAVIGATION
            self.switch_to_navigation_mode()
            map_goal = MoveBaseGoal()
            map_goal.target_pose.header.frame_id = 'map'
            map_goal.target_pose.header.stamp = rospy.Time.now()
            try:
                self.base_translation, self.base_orientation = self.tf_listener.lookupTransform('map', 'base_link', rospy.Time(0))
                if self.init_translation is None:
                    self.init_translation, self.init_orientation = self.base_translation, self.base_orientation
                try:
                    translation, rotation = self.tf_listener.lookupTransform('map', 'basket', rospy.Time(0))
                    print("Basket is within view!")
                    if self.found_basket_time is None and self.start_time is not None: 
                        self.found_basket_time = time.time() - self.start_time
                        self.record_times()
                except Exception as e:
                    # Couldn't find basket in view, go with last remembered location
                    print("Error looking up map-basket transform... using historically saved coordinates")
                    
                    if self.tag_translation is not None and self.tag_orientation is not None:
                        translation, rotation = self.tag_translation, self.tag_orientation
                    else:
                        # Use hardcoded memory
                        if self.find_tag("basket"):
                            translation, rotation = self.tf_listener.lookupTransform('map', 'basket', rospy.Time(0))
                        else: 
                            translation, rotation = ([3.04, -6.53, 0], [0, 0, -0.24, 0.97])

                    
                self.tag_translation, self.tag_orientation = translation, rotation
        
                joint_lift = self.tag_translation[2] + OFFSET_LIFT
                pose = {'joint_lift': joint_lift}
                print(f"Lifting arm {pose}")
                self.move_to_pose(pose)

                # rospy.loginfo(f"Base wrt Map: {self.base_translation}")
                # rospy.loginfo(f"Basket wrt Map: {self.tag_translation}")
                # rospy.loginfo(f"Base Orientation: {self.base_orientation}, as angles: {get_angles_from_quaternion(self.base_orientation)}")
                # rospy.loginfo(f"Basket Orientation: {self.tag_orientation}, as angles: {get_angles_from_quaternion(self.tag_orientation)}")
                
                eul = get_angles_from_quaternion(self.tag_orientation)
                angle_z = eul[2]
                quat = tf.transformations.quaternion_from_euler(0.0, 0.0, angle_z)

                angle_z_tag_normal = (angle_z + 3*pi/2 + pi) % (2*pi) - pi  # keep in the range [-pi,pi]
                angle_z_base_direction = (angle_z + pi + pi) % (2*pi) - pi  # keep in the range [-pi,pi]
                rospy.loginfo(f"angle_z_tag_normal: {angle_z_tag_normal} ({math.degrees(angle_z_tag_normal)})")
                rospy.loginfo(f"angle_z_base_direction: {angle_z_base_direction} ({math.degrees(angle_z_base_direction)})")

                quat = tf.transformations.quaternion_from_euler(0.0, 0.0, angle_z_base_direction)
                
                goal_x = self.tag_translation[0] + OFFSET_RADIUS * math.cos(angle_z_tag_normal) - 0.08 * math.cos(angle_z_base_direction)
                goal_y = self.tag_translation[1] + OFFSET_RADIUS * math.sin(angle_z_tag_normal) - 0.08 * math.sin(angle_z_base_direction)

                map_goal.target_pose.pose.position.x = round(goal_x, 2)
                map_goal.target_pose.pose.position.y = round(goal_y, 2)
                map_goal.target_pose.pose.position.z = 0.0
                map_goal.target_pose.pose.orientation.x = round(quat[0], 2)
                map_goal.target_pose.pose.orientation.y = round(quat[1], 2)
                map_goal.target_pose.pose.orientation.z = round(quat[2], 2)
                map_goal.target_pose.pose.orientation.w = round(quat[3], 2)
                rospy.loginfo(f"Nav goal, approach 2: {[map_goal.target_pose.pose.position.x, map_goal.target_pose.pose.position.y, math.degrees(get_angles_from_quaternion(quat)[2])]}")

                rospy.loginfo(f"Sending navigation map_goal {map_goal.target_pose.pose}!")
                goal_status = self.client.send_goal_and_wait(map_goal, execute_timeout=rospy.Duration(60))

                if goal_status == GoalStatus.SUCCEEDED:
                    reached_translation, reached_orientation = self.tf_listener.lookupTransform('map', 'base_link', rospy.Time(0))   
                    self.record_distance_from_goal("_nav_errors_basket", reached_translation, reached_orientation, [goal_x, goal_y, 0], quat)
                    rospy.loginfo("DONE navigating to basket!")
                    if self.nav_basket_time is None: 
                        self.nav_basket_time = time.time() - self.start_time
                    self.mode = 'pick_up'
                else:
                    time.sleep(3)

            except Exception as e:
                rospy.logerr(f"Error sending navigation map_goal {e}")
                time.sleep(3)
            self.record_times()

        elif self.mode == 'pick_up':
            # wrist up
            self.move_joints({'joint_wrist_pitch': WRIST_PITCH_UP})

            # open gripper
            self.move_joints({"gripper_aperture": GRIPPER_APERTURE_OPEN})

            # extend arm
            self.move_joints({"wrist_extension": WRIST_EXTENSTION_REACH})

            # wrist down
            self.move_joints({'joint_wrist_pitch': WRIST_PITCH_DOWN})

            # close gripper 
            self.move_joints({"gripper_aperture": GRIPPER_APERTURE_CLOSED})

            # wrist straight 
            self.move_joints({'joint_wrist_pitch': WRIST_PITCH_STRAIGHT})

            # joint lift to 1.09
            self.move_joints({'joint_lift': MAX_LIFT})

            # retract arm
            self.move_joints({'wrist_extension': WRIST_EXTENSTION_RETRACT})

            if self.pickup_basket_time is None: 
                self.pickup_basket_time = time.time() - self.start_time

            self.mode = 'deliver'
            self.record_times()

        elif self.mode == 'deliver':

            delivery_goal = MoveBaseGoal()
            delivery_goal.target_pose.header.frame_id = 'map'
            delivery_goal.target_pose.header.stamp = rospy.Time.now()
            
            delivery_goal.target_pose.pose.position = Point(GOAL['position']['x'], GOAL['position']['y'], 0)
            delivery_goal.target_pose.pose.orientation.x = GOAL['orientation']['x']
            delivery_goal.target_pose.pose.orientation.y = GOAL['orientation']['y']
            delivery_goal.target_pose.pose.orientation.z = GOAL['orientation']['z']
            delivery_goal.target_pose.pose.orientation.w = GOAL['orientation']['w']

            rospy.loginfo(f"Sending navigation delivery_goal {delivery_goal.target_pose.pose}!")
            goal_status = self.client.send_goal_and_wait(delivery_goal, execute_timeout=rospy.Duration(60))

            if goal_status == GoalStatus.SUCCEEDED:
                rospy.loginfo("DONE navigating to COUCH!")
                reached_translation, reached_orientation = self.tf_listener.lookupTransform('map', 'base_link', rospy.Time(0))   
                self.record_distance_from_goal("_nav_errors_couch", reached_translation, reached_orientation, [GOAL['position']['x'], GOAL['position']['y'], 0], 
                                               [GOAL['orientation']['x'], GOAL['orientation']['y'], GOAL['orientation']['z'], GOAL['orientation']['w']])
                self.mode = 'place'
                if self.nav_couch_time is None: 
                    self.nav_couch_time = time.time() - self.start_time
            self.record_times()

        elif self.mode == 'place':

            # extend arm
            self.move_joints({"wrist_extension": WRIST_EXTENSTION_REACH})

            # lower joint lift to couch
            self.move_joints({'joint_lift': COUCH_LIFT})

            # wrist down
            self.move_joints({'joint_wrist_pitch': WRIST_PITCH_DOWN})

            # open gripper
            self.move_joints({"gripper_aperture": GRIPPER_APERTURE_OPEN})

            # joint lift to 1.09
            self.move_joints({'joint_lift': MAX_LIFT})

            # retract arm
            self.move_joints({'wrist_extension': WRIST_EXTENSTION_RETRACT})

            self.mode = "go_home"
            if self.place_basket_time is None: 
                self.place_basket_time = time.time() - self.start_time
            self.record_times()
            
        elif self.mode == 'go_home':

            delivery_goal = MoveBaseGoal()
            delivery_goal.target_pose.header.frame_id = 'map'
            delivery_goal.target_pose.header.stamp = rospy.Time.now()
            
            delivery_goal.target_pose.pose.position = Point(self.init_translation[0], self.init_translation[1], 0)
            delivery_goal.target_pose.pose.orientation.x = self.init_orientation[0]
            delivery_goal.target_pose.pose.orientation.y = self.init_orientation[1]
            delivery_goal.target_pose.pose.orientation.z = self.init_orientation[2]
            delivery_goal.target_pose.pose.orientation.w = self.init_orientation[3]

            rospy.loginfo(f"Sending navigation go_home {delivery_goal.target_pose.pose}!")
            goal_status = self.client.send_goal_and_wait(delivery_goal, execute_timeout=rospy.Duration(60))
            if goal_status == GoalStatus.SUCCEEDED:
                rospy.loginfo("DONE navigating to HOME!")
                if self.nav_home_time is None: 
                    self.nav_home_time = time.time() - self.start_time   
                self.record_times()
                self.mode = "done"

        self.record_times()
        exit(0)


    def main(self):
        rospy.init_node('ArucoNavigation', anonymous=True)
        self.node_name = rospy.get_name()

        self.switch_to_position_mode = rospy.ServiceProxy('/switch_to_position_mode', Trigger)
        self.switch_to_navigation_mode = rospy.ServiceProxy('/switch_to_navigation_mode', Trigger)

        self.r = rospy.Rate(rospy.get_param('~rate', 15.0))

        rospy.Subscriber('/aruco/marker_array', MarkerArray, self.callback)

        # tf_buffer = tf2_ros.Buffer()
        # listener = tf2_ros.TransformListener(tf_buffer)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf.TransformListener(self.tf_buffer)
        rospy.loginfo("Trying to connect to trajectory client...")
        self.trajectory_client = actionlib.SimpleActionClient('/stretch_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        server_reached = self.trajectory_client.wait_for_server(timeout=rospy.Duration(60.0))
        if not server_reached:
            rospy.signal_shutdown('Unable to connect to arm action server. Timeout exceeded.')
            sys.exit()

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

        

if __name__ == '__main__':
    
    node = ArucoNavigation()
    node.main()

    try:
        rospy.spin()

    except KeyboardInterrupt:
        print('interrupt received, so shutting down')