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
    # Here I am going to hard code a home position because I am not sure how else to home the robot.
    # We only use this when we want to home the robot.
    self.homepose=Pose()
    self.homepose.position.x=float(910.15 / 1000.0)
    self.homepose.position.y=float(0.0)
    self.homepose.position.z=float(1194.48 / 1000.0)
    # We are going to create an array containting the poses we will navigate to.
    self.squareposes=[]

    # This is the hard coded z value for our final project
    self.finalz=0.3
    """
    UNITS ARE IN METERS!!!!

    Positive X means that it moves away from the TA office/lounge
    Positive y means that it moves towards WHRL Labs
    Positive z means that it moves towards heaven
    """
    # Bottom left point in square
    pose_goal = geometry_msgs.msg.Pose()
    #pose_goal.orientation.w = 1.0
    pose_goal.position.x = -0.75 # - float(100.0 / 1000.0)
    pose_goal.position.y = -0.5 # - float(100.0 / 1000.0)
    pose_goal.position.z = self.finalz # - float(100.0 / 1000.0)

    """
    We are going to say that the z must always be set to 0.5. The y must never be greater than -0.4 or so. 
    The X is the component must be between -.7 and .7 meters.
    """

    # first, we append the home position.
    # self.squareposes.append(copy.deepcopy(self.homepose))

    # Now we append the pose goal into the array
    # This position, from the robot persp, points towards the other robot arm, with the eoat pointing to the exit of the workshop.
    self.squareposes.append(copy.deepcopy(pose_goal))

    # Now we create the upper left trajectory.
    # This pos points to the camera stand we have set up.
    #pose_goal.position.x = -0.75
    pose_goal.position.y = -0.75

    # Now we append the pose goal into the array
    self.squareposes.append(copy.deepcopy(pose_goal))
    
    # Now we create the upper right trajectory.
    pose_goal.position.x = 0.75
    #pose_goal.position.y = -0.75

    # Now we append the pose goal into the array
    self.squareposes.append(copy.deepcopy(pose_goal))
    
    # Now we create the lower right trajectory.
    # pose_goal.position.x = 0.75
    pose_goal.position.y = -0.5

    # Now we append the pose goal into the array
    self.squareposes.append(copy.deepcopy(pose_goal))

    # finally, we append the home position.
    #self.squareposes.append(copy.deepcopy(self.homepose))

    # THis is the variable that controls what position we want to go to.
    self.whichpose=0
    # THese will be indeces from 0 to the "len(self.squareposes)-1"




  # This is the calback function for when we reeive a posestamped from the pixy camera node.
  def doSquareTraj(self):
    # First, send the pose we want.
    self.sendPixyPose.publish(self.squareposes[self.whichpose])
    # Now we delay a bit.
    time.sleep(5) # Sleep for the input amount of seconds.
    # Now we increment the which position counter.
    self.whichpose=(self.whichpose+1)%len(self.squareposes)













  def go_to_joint_state(self):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    group = self.group

    ## BEGIN_SUB_TUTORIAL plan_to_joint_state
    ##
    ## Planning to a Joint Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^^
    ## The Panda's zero configuration is at a `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_ so the first
    ## thing we want to do is move it to a slightly better configuration.
    # We can get the joint values from the group and adjust some of the values:

    # We are using an ABB irb 1600
    joint_goal = group.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = pi/12
    joint_goal[2] = 0
    joint_goal[3] = -pi/2
    joint_goal[4] = 0
    joint_goal[5] = pi/3
    joint_goal[6] = 0
    
    """joint_goal = group.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = -pi/4
    joint_goal[2] = 0
    joint_goal[3] = -pi/2
    joint_goal[4] = 0
    joint_goal[5] = pi/3
    joint_goal[6] = 0"""

    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    group.stop()

    ## END_SUB_TUTORIAL

    # For testing:
    # Note that since this section of code will not be included in the tutorials
    # we use the class variable rather than the copied state variable
    current_joints = self.group.get_current_joint_values()
    # return all_close(joint_goal, current_joints, 0.01)
    result = all_close(joint_goal, current_joints, 0.01)

  # This is what I want to do. - Andrew S
  def go_to_pose_goal(self):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    group = self.group

    ## BEGIN_SUB_TUTORIAL plan_to_pose
    ##
    ## Planning to a Pose Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## We can plan a motion for this group to a desired pose for the
    ## end-effector:
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = 1.0
    pose_goal.position.x = 0.4
    pose_goal.position.y = 0.1
    pose_goal.position.z = 0.4
    group.set_pose_target(pose_goal)

    ## Now, we call the planner to compute the plan and execute it.
    plan = group.go(wait=True)

    # Calling `stop()` ensures that there is no residual movement
    group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    group.clear_pose_targets()

    ## END_SUB_TUTORIAL

    # For testing:
    # Note that since this section of code will not be included in the tutorials
    # we use the class variable rather than the copied state variable
    current_pose = self.group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)

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
      tutorial.doSquareTraj()
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