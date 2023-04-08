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

from std_srvs.srv import Trigger

import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped, PoseStamped
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from visualization_msgs.msg import MarkerArray

import actionlib
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction


class ArucoNavigation():
    def __init__(self):
        hm.HelloNode.__init__(self)

    def callback(self, goal):
        self.markers = goal.markers
        print(f"Goal markers: {goal.markers}")
        

    def main(self):
        rospy.init_node('ArucoNavigation', anonymous=True)

        self.r = rospy.Rate(rospy.get_param('~rate', 15.0))

        rospy.Subscriber('/aruco/marker_array', MarkerArray, self.callback)

        tf_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tf_buffer)

        # self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        # self.client.wait_for_server()

        self.tf_listener = tf.TransformListener()
        self.switch_to_position_mode = rospy.ServiceProxy('/switch_to_position_mode', Trigger)
        self.switch_to_navigation_mode = rospy.ServiceProxy('/switch_to_navigation_mode', Trigger)

if __name__ == '__main__':
    
    node = ArucoNavigation()
    node.main()

    try:
        rospy.spin()

    except KeyboardInterrupt:
        print('interrupt received, so shutting down')