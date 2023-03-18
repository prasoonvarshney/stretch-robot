# Base code from: http://edu.gaitech.hk/turtlebot/map-navigation.html

import time
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import radians, degrees
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point


class map_navigation():

    def choose(self):
        choice='q'
        
        rospy.loginfo("|-------------------------------|")
        rospy.loginfo("|PRESS A KEY:")
        rospy.loginfo("|'0': Goal ")
        rospy.loginfo("|'1': Countertop 1 ")
        rospy.loginfo("|'2': Countertop 2 ")
        rospy.loginfo("|'q': Quit ")
        rospy.loginfo("|-------------------------------|")
        rospy.loginfo("|WHERE TO GO?")
        choice = input()
        return choice

    def __init__(self):
        # declare the coordinates of interest
        self.xGoal = 4.97
        self.yGoal = -5.07
        self.xCounter1 = 6.86
        self.yCounter1 = -3.08
        self.xCounter2 = 3.64
        self.yCounter2 = -4.10
        self.goalReached = False

        # initialize
        rospy.init_node('map_navigation', anonymous=False)
        choice = self.choose()

        if (choice == '0'):
            self.goalReached = self.moveToGoal(self.xGoal, self.yGoal)
        elif (choice == '1'):
            self.goalReached = self.moveToGoal(self.xCounter1, self.yCounter1)
        elif (choice == '2'):
            self.goalReached = self.moveToGoal(self.xCounter2, self.yCounter2)

        if (choice!='q'):
            if (self.goalReached):
                rospy.loginfo("Congratulations!")
                #rospy.spin()
            else:
                rospy.loginfo("Hard Luck!")

        while choice != 'q':
            choice = self.choose()
            if (choice == '0'):
                self.goalReached = self.moveToGoal(self.xGoal, self.yGoal)
            elif (choice == '1'):
                self.goalReached = self.moveToGoal(self.xCounter1, self.yCounter1)
            elif (choice == '2'):
                self.goalReached = self.moveToGoal(self.xCounter2, self.yCounter2)

            if (choice!='q'):
                if (self.goalReached):
                    rospy.loginfo("Congratulations!")
                    #rospy.spin()
                else:
                    rospy.loginfo("Hard Luck!")


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
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = 0.96
        goal.target_pose.pose.orientation.w = 0.25

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
    try:
        rospy.loginfo("You have reached the destination")
        map_navigation()
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("map_navigation node terminated.")