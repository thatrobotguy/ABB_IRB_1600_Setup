#!/usr/bin/env python

# This script is just supposed to generate poses for the robot to go to.
import sys, time
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Pose, PoseStamped
from moveit_commander.conversions import pose_to_list

class TestABBPoseGeneration:
  def __init__(self):
    # All this file does is create poses for the robot to move to. We are simply publishing poses.
    # This is the subscriber that receives poses from the Pixy node
    self.sendPixyPose=rospy.Publisher('/pixy_traj', Pose, queue_size=1)
    # This is the pose we will publish.
    self.pixypose=Pose()
    self.pixypose.position.x=0
    self.pixypose.position.y=0
    self.pixypose.position.z=0
    self.pixypose.orientation.w=0
    self.squareposes=[]
    """
    UNITS ARE IN METERS!!!!

    Positive X means that it moves away from the TA office/lounge
    Positive y means that it moves towards WHRL Labs
    Positive z means that it moves towards heaven
    """
    # Bottom left point in square
    pose_goal = geometry_msgs.msg.Pose()
    #pose_goal.orientation.w = 1.0
    pose_goal.position.x = -0.35 # - float(100.0 / 1000.0)
    pose_goal.position.y = -1.45 # THis is tha actual maximum
    pose_goal.position.z = 0.7

    """
    We are going to say that the z must always be set to 0.5. The y must never be greater than -0.4 or so. 
    The X is the component must be between -.7 and .7 meters.
    """
    # Now we append the pose goal into the array
    # This position, from the robot persp, points towards the other robot arm, with the eoat pointing to the exit of the workshop.
    self.squareposes.append(copy.deepcopy(pose_goal))

  # This is the calback function for when we reeive a posestamped from the pixy camera node.
  def movetostart(self):
    # First, send the pose we want.
    self.sendPixyPose.publish(self.squareposes[0])
    # Now we delay a bit.
    time.sleep(5) # Sleep for the input amount of seconds.
    #
    # Point 1
    #
    # top  - (306, 128) - the first number is left to right, the second number is top down
    # side - (302, 204)
    # 0.292 along the -y extended from the eoat
    # 0.0889 in the positive x from the eoat
    # 0.0 in the z from the eoat
    # This was the position of the robot arm when we tok these measurements
    """pose_goal.position.x = 0.15 # - float(100.0 / 1000.0)
    pose_goal.position.y = -1.45 # THis is tha actual maximum
    pose_goal.position.z = 0.4865 # - float(100.0 / 1000.0)"""
    # 
    # Point 2
    #
    # top  - (280, 117) - the first number is left to right, the second number is top down
    # side - (258, 183)
    # 0.292+0.3556 along the -y extended from the eoat
    # -0.015875 meters the positive x from the eoat
    # +5/8-3 = -0.060325 meters in the z from the eoat
    """pose_goal.position.x = 0.35 # - float(100.0 / 1000.0)
    pose_goal.position.y = -1.45 # THis is tha actual maximum
    pose_goal.position.z = 0.7"""
    #
    # Point 3
    #
    # top  - (278, 58) - the first number is left to right, the second number is top down
    # side - (240, 173)
    # 0.292 along the -y extended from the eoat
    # 0.0889 in the positive x from the eoat
    # 0.0 in the z from the eoat
    """pose_goal.position.x = -0.35 # - float(100.0 / 1000.0)
    pose_goal.position.y = -1.45 # THis is tha actual maximum
    pose_goal.position.z = 0.7"""


# This was a bug and also fixed:
# https://github.com/ros-planning/moveit/issues/86

def main():
  # Actually first, create a ros node
  rospy.init_node('dummy_pose_maker')
  # First, we will try and create the movegroup commander
  try:
    tutorial = TestABBPoseGeneration()
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return
  # Now that the object has been created, we need to loop, waiting for trajectories to follow.
  while not rospy.is_shutdown():
    # Increment the counter
    #counter=(counter+1) % 1000000
    try:
      # We are simply looping the trajectory of moving in the square shape.
      print "Starting a trajectory"
      tutorial.movetostart()
      print "Completed the trajectory"

    except rospy.ROSInterruptException:
      return
    except KeyboardInterrupt:
      return

if __name__ == '__main__':
  main()
  # These are the things I am running
  # roscore
  # roslaunch abb_irb1600_6_12_moveit_config moveit_planning_execution.launch robot_ip:=192.168.100.100 sim:=true
  # rosrun moveit_tutorials andrew_abb_move.py 
  # rosrun moveit_tutorials pose_generator.py 