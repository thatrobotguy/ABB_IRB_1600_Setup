/*

  This is an example ROS package that controls the ABB robot arm. Simply send a Pose message
  to this ROS node and it will send it to the ABB robot controller node that then send
  a message to the arm to drive it.

  This package was written by:
  
  thatrobotguy

*/

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

// Make sure we have access to the necessary messages.
#include <geometry_msgs/Pose.h>

#include "std_msgs/String.h"
#include "std_msgs/Bool.h"

// These were ripped off of the MQP repo
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <math.h>
#include <tf/transform_datatypes.h>

// For cout
#include <iostream>


// "manipulator" is for the abb arm.
static const std::string PLANNING_GROUP = "manipulator";

// This is the boolean value that says if we successfully planned a path to that location.
bool isgoodplan = false;

// This is a printing counter
int printcounter = 0;

// This is where we store the pose we want to go to. THIS IS THE FINAL POSE TO SEND TO THE ROBOT.
geometry_msgs::Pose poseEOAT;

// This is the home pose
geometry_msgs::Pose homepose;

// This is the boolean flag that says if we have a new pose or not.
bool dowehavenewpose = false;

// Is the arm done moving?
std_msgs::Bool armdoingmove;

// I ripped these off of th eMQP repository
std::string base_frame = "/base_link";
//Location variables set at launch
float roll, pitch, yaw;
float x, y, z;
tf::Quaternion q_rot;
tf::StampedTransform j6_trans;
tf::Transform tool_trans;
tf::Transform j6_to_base;

// This is the callback function for receiving the pose we want to go to
void poseCallback(const geometry_msgs::Pose& msg)
{
  // We only want to save the pose if we do not already have a pose to go to.
  if (dowehavenewpose == false)
  {
    ROS_INFO("Running callback function.");
    // Now we get the position info
    poseEOAT.position.x=msg.position.x;
    poseEOAT.position.y=msg.position.y;
    poseEOAT.position.z=msg.position.z;
    // Now we get the orientation info. For now, we ignore the wanted orientation from the pose publisher.
    // quaternionTFToMsg(q_rot,poseEOAT.orientation);

    // The line above only happens once in main()

    // poseEOAT.orientation.x=msg.orientation.x;
    // poseEOAT.orientation.y=msg.orientation.y;
    // poseEOAT.orientation.z=msg.orientation.z;
    // poseEOAT.orientation.w=msg.orientation.w;
    // Now I need to set the boolean flag to say that we have received a new pose to go to.
    dowehavenewpose = true;
  }
}

// roscore
// roslaunch abb_irb1600_6_12_moveit_config moveit_planning_execution.launch robot_ip:=192.168.100.100 sim:=true
// roslaunch abb_1600_driver abb_interface.launch
// rosrun moveit_tutorials pose_generator.py 

// This is the old python script
// rosrun moveit_tutorials andrew_abb_move.py   

/*
  I have to give credit to the MQP guys because I am ripping some of the logic straight from there repository
  (specifically, the tool orientation code and yaml file stuff.)
*/


// The main running code.
int main(int argc, char** argv)
{
  // This MUST BE CALLED FIRST!!
  ros::init(argc, argv, "abb_mover");

  ros::NodeHandle node_handle;
  // I forgot to register the subscriber
  ros::Subscriber sub = node_handle.subscribe("/pixy_traj", 2, poseCallback); // the second argument is the queue size
  /*
    This node pulblishes to a topic so that other nodes can know that 
    the robot is busy moving to a pose provided by this node. 
  */
  ros::Publisher pub = node_handle.advertise<std_msgs::Bool>("/is_arm_moving", 1); // 1 is the queue size.
  // Here we start a background thread to handle spinning.
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // In order to get the end effector transform, we need to listen to transform frames.
  tf::TransformListener listener;

  // The :move_group_interface:`MoveGroup` class can be easily
  // setup using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  // We will use the :planning_scene_interface:`PlanningSceneInterface`
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const robot_state::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // Visualization
  // ^^^^^^^^^^^^^
  //
  // The package MoveItVisualTools provides many capabilties for visualizing objects, robots,
  // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("abb_link0");
  visual_tools.deleteAllMarkers();

  // Remote control is an introspection tool that allows users to step through a high level script
  // via buttons and keyboard shortcuts in RViz
  visual_tools.loadRemoteControl();

  // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
  Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
  text_pose.translation().z() = 1.75;
  visual_tools.publishText(text_pose, "MoveGrop ABB Demo", rvt::WHITE, rvt::XLARGE);

  // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
  visual_tools.trigger();

  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // We can print the name of the reference frame for this robot.
  ROS_INFO_NAMED("tutorial", "Reference frame: %s", move_group.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());

  /*
    At this point, we have loaded all of the information we need to start the node.
    Now we just need to receive poses from the callback function before we call the planner.
    This means that we can simply have a while true loop that does nothing but make the
    program wait.
  */



  // These next few lines are ripped from the mqp team
  // Get the ROS params and store them in this file.
  node_handle.getParam("home_pose/roll",roll);
	node_handle.getParam("home_pose/pitch",pitch);
  node_handle.getParam("home_pose/yaw",yaw);
  node_handle.getParam("home_pose/x_pos",x);
	node_handle.getParam("home_pose/y_pos",y);
  node_handle.getParam("home_pose/z_pos",z);
  // Convert to radians
  roll=roll*(M_PI/180);
  pitch=pitch*(M_PI/180);
  yaw=yaw*(M_PI/180);
  // create quarternion
  q_rot = tf::createQuaternionFromRPY(roll, pitch, yaw);
  // Now we set the end of arm tooling orientation.
  // quaternionTFToMsg(q_rot,poseEOAT.orientation);
  quaternionTFToMsg(q_rot,poseEOAT.orientation);
  // Now we set the orientation of the home pose to the same thing because we know it is valid.
  quaternionTFToMsg(q_rot,homepose.orientation);
  std::cout << "This is the quarternion for the HOME POSITION end effector: \n" << homepose.orientation << std::endl;
  // Now we fill in the position
  poseEOAT.position.x=x;
  poseEOAT.position.y=y;
  poseEOAT.position.z=z;
  // Now we place the wanted pose into a transform.
  tf::poseMsgToTF(poseEOAT, tool_trans);
  // Get the transform between the base link and the end effector-mounted link
  listener.waitForTransform("/base_link","/link_6",ros::Time(0), ros::Duration(4.0));
  listener.lookupTransform("/base_link", "/link_6", ros::Time(0), j6_trans);
  // Bring the transform with respect to the base
  j6_to_base = j6_trans * tool_trans;
  // Now we set he transform message to the pose
  tf::poseTFToMsg(j6_to_base, poseEOAT);
  // Now I want to see what the orientation is
  std::cout << "This is the quarternion for the end effector: \n" << poseEOAT.orientation << std::endl;
  // The part I ripped above was solely to orient the eoat in some hardcoded x,y,z


  // Back to my own code
  // Initialize the is moving boolean
  armdoingmove.data=false;
  pub.publish(armdoingmove);

  // Just in case the base link is NOT the home position, it SHOULD BE
  move_group.setPoseReferenceFrame(base_frame);

  // This is the main program while loop.
  while (ros::ok())
  {
    // Increment the printing counter.
    printcounter=(printcounter+1) % 40000000; // the number there is arbitrary - i just need it to slow the printing down   
    // First we check to see if we have a new pose to move to.
    if (dowehavenewpose)
    {
      // This means we need to do the visual updating to get the arm to move.
      // First we set the target
      move_group.setPoseTarget(poseEOAT);
      // Now, we call the planner to compute the plan and visualize it.
      // Note that we are just planning, not asking move_group
      // to actually move the robot.
      moveit::planning_interface::MoveGroupInterface::Plan my_plan;
      // Now we check to see if the path is valid.
      isgoodplan = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      // Get status messages
      ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", isgoodplan ? "" : "FAILED");
      ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
      visual_tools.publishAxisLabeled(poseEOAT, "pose1");
      visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
      visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
      visual_tools.trigger();
      // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
      // Publish that we are busy doing a trajectory
      armdoingmove.data=true;
      pub.publish(armdoingmove);
      // Now we do the move - Note that this is a blocking function. We will have to fix that later.
      move_group.move();
      // This is where we reset the current state.
      move_group.setStartStateToCurrentState();
      // Say that we do not have a new pose
      dowehavenewpose = false;
      // Now we say the arm is not moving
      armdoingmove.data=false;
      pub.publish(armdoingmove);
      // I am not 100% sure, but I think that the order of this "I'm done publisher" and actually moving DOES MATTER
    } else {
      // We do not have a new pose, so we do not want to move the robot.
      if (printcounter==0)
      {
        // Tell us that the robot is waiting for a trajectory.
        ROS_INFO("Waiting for Pose Trajectory.");
        // We also want to tell the other node that the robot is not moving, so we will publish that the robot is not moving here as well
        pub.publish(armdoingmove);
      } else { }
    }
  }
  // We use this because of background threads.
  // https://wiki.ros.org/roscpp/Overview/Callbacks%20and%20Spinning#Multi-threaded_Spinning
  ros::waitForShutdown();
  return 0;
}