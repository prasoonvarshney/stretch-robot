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


ARM_LENGTH = 1  # meters?


class ArucoNavigation(hm.HelloNode):
    def __init__(self):
        hm.HelloNode.__init__(self)
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
                self.tf_listener.waitForTransform(requested_tag, 'base_link', rospy.Time(), rospy.Duration(4.0))
                self.translation, self.rotation = self.tf_listener.lookupTransform(requested_tag, 'base_link', rospy.Time(0))
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
    
            if count >= 2:
                rospy.loginfo("Timed Out Looking for Tag")
                # self.switch_to_navigation_mode()
                return False

        self.switch_to_navigation_mode()
        return True

    def callback(self, goal):
        self.markers = goal.markers
        print(f"Goal markers: {goal.markers}")

        requested_tag = "basket"
        min_rotation = -4.05
        max_rotation = 1.78
        num_steps = 10
        step = abs(min_rotation - max_rotation) / num_steps

        rospy.loginfo(f"LOOKING FOR THIS TAG: {requested_tag}")
        rospy.loginfo(requested_tag)

        # MANIPULATION
        self.translation, self.rotation = self.tf_listener.lookupTransform(requested_tag, 'base_link', rospy.Time(0))
        rospy.loginfo("Found Requested Tag")
        
        found_tag = True

        self.switch_to_position_mode()

        # pose = {'wrist_extension': 0.01}
        # print(f"Trying to move to pose {pose}")
        # self.move_to_pose(pose)

        # pose = {'joint_wrist_yaw': 3.3}
        # print(f"Trying to move to pose {pose}")
        # self.move_to_pose(pose)
        
        # pose = {'joint_lift': 0.82}
        # print(f"Trying to move to pose {pose}")
        # self.move_to_pose(pose)
        
        print(f"Trying to define target pose")
        target_pose = PoseStamped()
        target_pose.header.frame_id = 'basket'
        
        target_pose.pose.position.x = self.translation[0]
        target_pose.pose.position.y = self.translation[1]
        target_pose.pose.position.z = self.translation[2]

        target_pose.pose.orientation.x = self.rotation[0]
        target_pose.pose.orientation.y = self.rotation[1]
        target_pose.pose.orientation.z = self.rotation[2]
        target_pose.pose.orientation.w = self.rotation[3]


        # NAVIGATION
        self.switch_to_navigation_mode()

        map_goal = MoveBaseGoal()
        map_goal.target_pose.header.frame_id = 'map'
        map_goal.target_pose.header.stamp = rospy.Time()
        try:
            base_translation, base_orientation = self.tf_listener.lookupTransform('base_link', 'map', rospy.Time(0))
            self.translation, self.orientation = self.tf_listener.lookupTransform('basket', 'map', rospy.Time(0))
    
            print(f"Trying to define map_goal")

            length = math.sqrt((self.translation[0] - base_translation[0])**2 + (self.translation[1] - base_translation[1] - base_translation[1])**2)
            if length > ARM_LENGTH:
                multiplicative_ratio = (length - ARM_LENGTH) / length
                x = base_translation[0] + multiplicative_ratio * (self.translation[0] - base_translation[0])
                y = base_translation[1] + multiplicative_ratio * (self.translation[1] - base_translation[1])
            else: 
                x = base_translation[0]
                y = base_translation[1]
    
            map_goal.target_pose.pose.position.x = x
            map_goal.target_pose.pose.position.y = y
            map_goal.target_pose.pose.position.z = 0.0
            
            eul = tf.transformations.euler_from_quaternion((map_goal.target_pose.pose.orientation.x, map_goal.target_pose.pose.orientation.y, map_goal.target_pose.pose.orientation.z, map_goal.target_pose.pose.orientation.w))
            quat = tf.transformations.quaternion_from_euler(0.0, 0.0, eul[2])
            map_goal.target_pose.pose.orientation.x = quat[0]
            map_goal.target_pose.pose.orientation.y = quat[1]
            map_goal.target_pose.pose.orientation.z = quat[2]
            map_goal.target_pose.pose.orientation.w = quat[3]
    
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