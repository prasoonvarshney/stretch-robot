# Base code from: http://edu.gaitech.hk/turtlebot/map-navigation.html

import json
import time
import argparse

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import radians, degrees
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point


COORDS = json.load(open('coordinates.json', 'r'))
GOAL = COORDS['couch']
START_1 = COORDS['counter_sink']
START_2 = COORDS['counter_couch']

class map_navigation():

    def choose(self):
        choice='q'
        
        rospy.loginfo("|-------------------------------|")
        rospy.loginfo("|PRESS A KEY:")
        rospy.loginfo("|'0': Goal (Couch) ")
        rospy.loginfo("|'1': Countertop 1 ")
        rospy.loginfo("|'2': Countertop 2 ")
        rospy.loginfo("|'q': Quit ")
        rospy.loginfo("|-------------------------------|")
        rospy.loginfo("|WHERE TO GO?")
        choice = input()
        return choice
    
    def map_choice_to_coords(self, choice):
        if choice == '0':
            return GOAL
        elif choice == '1':
            return START_1
        elif choice == '2':
            return START_2

    def __init__(self, goal):
        # initialize
        rospy.init_node('map_navigation', anonymous=False)
        
        # declare the coordinates of interest
        if goal is None:
            choice = self.choose()
        else:
            choice = str(goal)
            
        goal_coords = self.map_choice_to_coords(choice)
        self.goal_position = goal_coords['position']
        self.goal_orientation = goal_coords['orientation']
        
        self.xGoal, self.yGoal = self.goal_position['x'], self.goal_position['y']

        self.goalReached = self.moveToGoal(self.xGoal, self.yGoal)
        
        if (choice != 'q'):
            if (self.goalReached):
                rospy.loginfo("Congratulations!")
            else:
                rospy.loginfo("Hard Luck!")
                
        if goal is None:
            while choice != 'q':
                choice = self.choose()
                goal_coords = self.map_choice_to_coords(choice)
                self.goal_position = goal_coords['position']
                self.goal_orientation = goal_coords['orientation']
                
                self.xGoal, self.yGoal = self.goal_position['x'], self.goal_position['y']

                self.goalReached = self.moveToGoal(self.xGoal, self.yGoal)
                
                if (choice != 'q'):
                    if (self.goalReached):
                        rospy.loginfo("Congratulations!")
                    else:
                        rospy.loginfo("Hard Luck!")
        else:
            self.shutdown()

    def shutdown(self):
        # stop turtlebot
        rospy.loginfo("Quit program")
        rospy.sleep()

    def moveToGoal(self, xGoal, yGoal):

        #define a client for to send goal requests to the move_base server through a SimpleActionClient
        ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        #wait for the action server to come up
        while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
            rospy.loginfo("Waiting for the move_base action server to come up")


        goal = MoveBaseGoal()

        #set up the frame parameters
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        # moving towards the goal*/

        goal.target_pose.pose.position =  Point(xGoal,yGoal,0)
        goal.target_pose.pose.orientation.x = self.goal_orientation['x']
        goal.target_pose.pose.orientation.y = self.goal_orientation['y']
        goal.target_pose.pose.orientation.z = self.goal_orientation['z']
        goal.target_pose.pose.orientation.w = self.goal_orientation['w']

        rospy.loginfo("Sending goal location ...")
        ac.send_goal(goal)
        start_time = time.perf_counter()
        
        ac.wait_for_result(rospy.Duration(60))

        if(ac.get_state() ==  GoalStatus.SUCCEEDED):
            total_time = time.perf_counter() - start_time
            rospy.loginfo(f"You have reached the destination in {total_time} seconds")
            return True
        else:
            rospy.loginfo("The robot failed to reach the destination")
            return False

if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--goal', '-g', type=int, default=None)
    args = p.parse_args()
    
    try:
        rospy.loginfo("You have reached the destination")
        map_navigation(args.goal)
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("map_navigation node terminated.")