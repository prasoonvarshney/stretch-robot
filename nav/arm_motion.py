#!/usr/bin/env python3

import rospy
import time
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
import hello_helpers.hello_misc as hm
import argparse

class MultiPointCommand(hm.HelloNode):
  """
  A class that sends multiple joint trajectory goals to the stretch robot.
  """
  def __init__(self):
    hm.HelloNode.__init__(self)

  def issue_multipoint_command(self, location, interaction):
    """
    Function that makes an action call and sends multiple joint trajectory goals
    to the joint_lift, wrist_extension, and joint_wrist_yaw.
    :param self: The self reference.
    """
    point0 = JointTrajectoryPoint()
    # default: retracted and open
    joint_lift, wrist_extension, joint_wrist_pitch, gripper_aperture = 1.0, 0.0, -0.8, 0.07

    if location == "retracted":
      joint_lift, wrist_extension = 1.0, 0.0
    elif location == "couch":
      joint_lift, wrist_extension = 0.7, 0.4
    elif location == "counter1":
      joint_lift, wrist_extension = 1.0, 0.5
    elif location == "counter2":
      joint_lift, wrist_extension = 1.0, 0.3

    if interaction == "open":
      joint_wrist_pitch, gripper_aperture = -0.8, 0.07
    elif interaction == "close":
      joint_wrist_pitch, gripper_aperture = 0.0, -0.12
    elif interaction == "rest":
      joint_wrist_pitch, gripper_aperture = -1.5, 0
    point0.positions = [joint_lift, wrist_extension, joint_wrist_pitch, gripper_aperture]

    trajectory_goal = FollowJointTrajectoryGoal()
    trajectory_goal.trajectory.joint_names = ['joint_lift', 'wrist_extension', 'joint_wrist_pitch', 'gripper_aperture']
    trajectory_goal.trajectory.points = [point0]
    trajectory_goal.trajectory.header.stamp = rospy.Time(0.0)
    trajectory_goal.trajectory.header.frame_id = 'base_link'

    self.trajectory_client.send_goal_and_wait(trajectory_goal, execute_timeout=rospy.Duration(20.0))
    rospy.loginfo('Sent list of goals = {0}'.format(trajectory_goal))
    self.trajectory_client.wait_for_result()

  def main(self, args):
    """
    Function that initiates the multipoint_command function.
    :param self: The self reference.
    """
    hm.HelloNode.main(self, 'multipoint_command', 'multipoint_command', wait_for_first_pointcloud=False)
    rospy.loginfo(f'issuing multipoint command for location {args.location} and interaction {args.interaction}...')
    self.issue_multipoint_command(args.location, args.interaction)
    time.sleep(2)


if __name__ == '__main__':

  p = argparse.ArgumentParser()
  p.add_argument('--location', '-loc', type=str, default="retracted")
  p.add_argument('--interaction', '-int', type=str, default="open")
  args = p.parse_args()

  try:
    node = MultiPointCommand()
    node.main(args)
  except KeyboardInterrupt:
    rospy.loginfo('interrupt received, so shutting down')
