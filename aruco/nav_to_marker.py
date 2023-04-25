#! /usr/bin/env python

import hello_helpers.hello_misc as hm

import rospy
import logging
import tf
import tf2_ros
import time
from math import pi, sqrt, atan2
import json 
import os
import sys
import math

from std_srvs.srv import Trigger

import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped, PoseStamped
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from visualization_msgs.msg import MarkerArray

import actionlib
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction


ARM_LENGTH = 0.5  # meters?
OFFSET_LIFT = 0.08
OFFSET_RADIUS = 0.5


def get_angles_from_quaternion(quaternion):
    return tf.transformations.euler_from_quaternion(
        (quaternion[0], quaternion[1], quaternion[2], quaternion[3])
    )


class ArucoNavigation(hm.HelloNode):
    def __init__(self):
        hm.HelloNode.__init__(self)
        self.base_translation, self.base_orientation = None, None
        self.tag_translation,  self.tag_orientation  = None, None
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

    def callback(self, goal):
        self.markers = goal.markers
        print(f"Goal markers: {goal.markers}")

        requested_tag = "basket"

        rospy.loginfo(f"LOOKING FOR THIS TAG: {requested_tag}")

        # MANIPULATION
        # self.tag_translation, self.tag_rotation = self.tf_listener.lookupTransform(requested_tag, 'base_link', rospy.Time(0))
        # rospy.loginfo("Found Requested Tag")
        
        # found_tag = True

        # pose = {'wrist_extension': 0.01}
        # print(f"Trying to move to pose {pose}")
        # self.move_to_pose(pose)

        # pose = {'joint_wrist_yaw': 3.3}
        # print(f"Trying to move to pose {pose}")
        # self.move_to_pose(pose)
        
        # pose = {'joint_lift': 0.82}
        # print(f"Trying to move to pose {pose}")
        # self.move_to_pose(pose)
        
        # print(f"Trying to define target pose")
        # target_pose = PoseStamped()
        # target_pose.header.frame_id = 'basket'
        
        # target_pose.pose.position.x = self.tag_translation[0]
        # target_pose.pose.position.y = self.tag_translation[1]
        # target_pose.pose.position.z = self.tag_translation[2]

        # target_pose.pose.orientation.x = self.tag_rotation[0]
        # target_pose.pose.orientation.y = self.tag_rotation[1]
        # target_pose.pose.orientation.z = self.tag_rotation[2]
        # target_pose.pose.orientation.w = self.tag_rotation[3]


        # NAVIGATION
        self.switch_to_navigation_mode()

        map_goal = MoveBaseGoal()
        map_goal.target_pose.header.frame_id = 'map'
        map_goal.target_pose.header.stamp = rospy.Time()
        try:
            self.base_translation, self.base_orientation = self.tf_listener.lookupTransform('map', 'base_link', rospy.Time(0))
            try:
                translation, rotation = self.tf_listener.lookupTransform('map', 'basket', rospy.Time(0))
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
                        translation, rotation = ([6.7, -3.28, 0], [0, 0, 0.96, 0.28])

                
            self.tag_translation, self.tag_orientation = translation, rotation
    
            joint_lift = self.tag_translation[2] + OFFSET_LIFT
            pose = {'joint_lift': joint_lift}
            print(f"Lifting arm {pose}")
            self.move_to_pose(pose)

            print(f"Trying to define map_goal")

            rospy.loginfo(f"Base wrt Map: {self.base_translation}")
            rospy.loginfo(f"Basket wrt Map: {self.tag_translation}")
            rospy.loginfo(f"Base Orientation: {self.base_orientation}, as angles: {get_angles_from_quaternion(self.base_orientation)}")
            rospy.loginfo(f"Basket Orientation: {self.tag_orientation}, as angles: {get_angles_from_quaternion(self.tag_orientation)}")

            length = math.sqrt((self.tag_translation[0] - self.base_translation[0])**2 + (self.tag_translation[1] - self.base_translation[1] - self.base_translation[1])**2)
            if length > ARM_LENGTH:
                multiplicative_ratio = (length - ARM_LENGTH) / length
                x = self.base_translation[0] + multiplicative_ratio * (self.tag_translation[0] - self.base_translation[0])
                y = self.base_translation[1] + multiplicative_ratio * (self.tag_translation[1] - self.base_translation[1])
            else: 
                x = self.base_translation[0]
                y = self.base_translation[1]
            
            eul = get_angles_from_quaternion(self.tag_orientation)
            angle_z = eul[2]
            quat = tf.transformations.quaternion_from_euler(0.0, 0.0, angle_z)
    
            map_goal.target_pose.pose.position.x = x
            map_goal.target_pose.pose.position.y = y
            map_goal.target_pose.pose.position.z = 0.0
            map_goal.target_pose.pose.orientation.x = quat[0]
            map_goal.target_pose.pose.orientation.y = quat[1]
            map_goal.target_pose.pose.orientation.z = quat[2]
            map_goal.target_pose.pose.orientation.w = quat[3]
    
            rospy.loginfo(f"Nav goal, approach 1: {[x, y, math.degrees(angle_z)]}")


            angle_z_tag_normal = (angle_z + 3*pi/2 + pi) % (2*pi) - pi  # keep in the range [-pi,pi]
            angle_z_base_direction = (angle_z + pi + pi) % (2*pi) - pi  # keep in the range [-pi,pi]
            rospy.loginfo(f"angle_z_tag_normal: {angle_z_tag_normal} ({math.degrees(angle_z_tag_normal)})")
            rospy.loginfo(f"angle_z_base_direction: {angle_z_base_direction} ({math.degrees(angle_z_base_direction)})")


            quat = tf.transformations.quaternion_from_euler(0.0, 0.0, angle_z_base_direction)
            
            goal_x = self.tag_translation[0] + OFFSET_RADIUS * math.cos(angle_z_tag_normal)
            goal_y = self.tag_translation[1] + OFFSET_RADIUS * math.sin(angle_z_tag_normal)
            map_goal.target_pose.pose.position.x = goal_x
            map_goal.target_pose.pose.position.y = goal_y
            map_goal.target_pose.pose.position.z = 0.0
            map_goal.target_pose.pose.orientation.x = quat[0]
            map_goal.target_pose.pose.orientation.y = quat[1]
            map_goal.target_pose.pose.orientation.z = quat[2]
            map_goal.target_pose.pose.orientation.w = quat[3]
            rospy.loginfo(f"Nav goal, approach 2: {[self.tag_translation[0] + OFFSET_RADIUS * math.cos(angle_z_tag_normal), self.tag_translation[1] + OFFSET_RADIUS * math.sin(angle_z_tag_normal), math.degrees(angle_z_base_direction)]}")

    
            rospy.loginfo(f"Sending navigation map_goal {map_goal.target_pose.pose}!")
            self.client.send_goal_and_wait(map_goal)
            rospy.loginfo("DONE!")
            
            time.sleep(10)

        except Exception as e:
            rospy.logerr(f"Error sending navigation map_goal {e}")
            time.sleep(10)
        

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