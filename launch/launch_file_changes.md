# Launch file modifications

This file describes the launch file modifications required to make working with the ABB IRB1600 1.45 model and ABB IRB120 model work properly in `MoveIt!`.


## ABB Package Modifications

Once you have gotten the `ROS-Industrial` and `MoveIt!` installations squared away, we will have to do some manual modifications of launch files before you connect the robot to ROS.

In the ros package `abb_experimental` there is a subpackage called `abb_irb1600_6_12_moveit_config` with this launch file:
```
moveit_planning_execution.launch
```
You need to edit this launch file to decouple joints 2 and 3. For me, this file is located here:

`/home/$USER/catkin_ws/src/abb_experimental/abb_irb1600_6_12_moveit_config/launch`

I do not remember what the old version of the file looks like, but I only commented out lines. I did not delete anything.
Basically, in this file, we turned off `J23-coupling` by setting the `arg` to false.
```

<launch>
<!-- The planning and execution components of MoveIt! configured to run -->
<!-- using the ROS-Industrial interface. -->
<!-- Non-standard joint names:
- Create a file [robot_moveit_config]/config/joint_names.yaml
controller_joint_names: [joint_1, joint_2, ... joint_N] 
- Update with joint names for your robot (in order expected by rbt controller)
- and uncomment the following line: -->

<rosparam command="load" file="$(find abb_irb1600_support)/config/joint_names_irb1600_6_12.yaml" />

<!-- the "sim" argument controls whether we connect to a Simulated or Real robot -->
<!-- - if sim=false, a robot_ip argument is required -->

<arg name="sim" default="true" />
<arg name="robot_ip" unless="$(arg sim)" />

<!-- By default, we do not start a database (it can be large) -->

<arg name="db" default="false" />

<!-- Allow user to specify database location -->

<arg name="db_path" default="$(find abb_irb1600_6_12_moveit_config)/default_warehouse_mongo_db" />



<!-- Here we set the default linkage coupling. This needs to be false. -->
<arg name="link_2_3_coupled" default="false" doc="If true, compensate for J2-J3 parallel linkage" />



<!-- load the robot_description parameter before launching ROS-I nodes -->

<include file="$(find abb_irb1600_6_12_moveit_config)/launch/planning_context.launch" >
<arg name="load_robot_description" value="true" />
</include>

<!-- run the robot simulator and action interface nodes -->

<group if="$(arg sim)">
<include file="$(find industrial_robot_simulator)/launch/robot_interface_simulator.launch" />
</group>

<!-- run the "real robot" interface nodes -->
<!-- - this typically includes: robot_state, motion_interface, and joint_trajectory_action nodes -->
<!-- - replace these calls with appropriate robot-specific calls or launch files -->

<group unless="$(arg sim)">
   
<!-- I needed to change this. -->
   <!-- 
  <include file="$(find abb_irb1600_support)/launch/robot_interface_download_irb1600_6_12.launch" >
    <arg name="robot_ip" value="$(arg robot_ip)"/>
  </include>
   -->

<include file="$(find abb_irb1600_support)/launch/robot_interface_download_irb1600_6_12_modified.launch" >
<arg name="robot_ip" value="$(arg robot_ip)"/>
<arg name="J23_coupled" value="$(arg link_2_3_coupled)"/> <!-- I am adding a param and setting it to false. -->
</include>


</group>

<!-- publish the robot state (tf transforms) -->
<node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" />

<include file="$(find abb_irb1600_6_12_moveit_config)/launch/move_group.launch">
<arg name="publish_monitored_planning_scene" value="true" />
</include>

<include file="$(find abb_irb1600_6_12_moveit_config)/launch/moveit_rviz.launch">
<arg name="config" value="true"/>
</include>

<!-- If database loading was enabled, start mongodb as well -->

<include file="$(find abb_irb1600_6_12_moveit_config)/launch/default_warehouse_db.launch" if="$(arg db)">
<arg name="moveit_warehouse_database_path" value="$(arg db_path)"/>
</include>

</launch>
```



The launch file described above also calls another launch file that also needs editing. The file you need to change is
```
robot_interface_download_irb1600_6_12.launch
```
I created my own version called:
```
robot_interface_download_irb1600_6_12_modified.launch
```
So that I do not have to call the original launch file, which is located here:
```
/home/$USER/catkin_ws/src/abb_experimental/abb_irb1600_support/launch
```
This modified launch file looks like this:
```
<!--
Manipulator specific version of abb_driver's 'robot_interface.launch'.

Defaults provided for IRB 1600:
- J23_coupled = true

Usage:
robot_interface_download_irb1600.launch robot_ip:=<value>
-->

<launch>
<arg name="robot_ip" doc="IP of the controller" />

<!-- tharobotguy moved this line into a comment.
<arg name="J23_coupled" default="true" doc="If true, compensate for J2-J3 parallel linkage" />
-->

<arg name="J23_coupled" default="false" doc="If true, compensate for J2-J3 parallel linkage" />

<rosparam command="load" file="$(find abb_irb1600_support)/config/joint_names_irb1600_6_12.yaml" />

<include file="$(find abb_driver)/launch/robot_interface.launch">
<arg name="robot_ip" value="$(arg robot_ip)" />
<arg name="J23_coupled" value="$(arg J23_coupled)" />
</include>

</launch>
```
In the upper launch file, I turned the couple_J_2_3 off as well as set the default IP address. Other than that, everything else is left alone.
I did make one other change to the launch files. This one should not be _necessary_, but I have included it here so that I am not missing anything as I happened to make these changes in the process. This launch file only matters when you are running your code on the real robot as opposed to the simulated robot. I talk about it more in the final notes section. This is the .xml file I edited: `trajectory_execution.launch.xml`. (It is in the same directory as the upper level launch file, If I am not mistaken.) I got the idea to edit it [from this](https://answers.ros.org/question/196586/how-do-i-disable-execution_duration_monitoring/) post. Basically, if you want to do any mildly long trajectory in `MoveIt!`, you will want to call set this variable to be larger.

Here is what the file looks like for me:
```
<launch>

<!-- This file makes it easy to include the settings for trajectory execution --> 

<!-- Flag indicating whether MoveIt! is allowed to load/unload or switch controllers -->
<arg name="moveit_manage_controllers" default="true"/>
<param name="moveit_manage_controllers" value="$(arg moveit_manage_controllers)"/>

<!-- thatrobotguy added this line from here:
https://answers.ros.org/question/196586/how-do-i-disable-execution_duration_monitoring/

<param name="trajectory_execution/execution_duration_monitoring" value="true" />
-->

<!-- When determining the expected duration of a trajectory, this multiplicative factor is applied to get the allowed duration of execution -->
<param name="trajectory_execution/allowed_execution_duration_scaling" value="1.2"/> <!-- default 1.2 -->
<!-- Allow more than the expected execution time before triggering a trajectory cancel (applied after scaling) -->
<!-- thatrobotguy played with this too.-->
<param name="trajectory_execution/allowed_goal_duration_margin" value="3.5"/> <!-- default 0.5 -->

<!-- Allowed joint-value tolerance for validation that trajectory's first point matches current robot state -->
<!-- thatrobotguy set the default from 0.01 to 0.0-->
<param name="trajectory_execution/allowed_start_tolerance" value="0.1"/> <!-- default 0.01 -->
<!-- Load the robot specific controller manager; this sets the moveit_controller_manager ROS parameter -->
<arg name="moveit_controller_manager" default="abb_irb1600_6_12" />
<include file="$(find abb_irb1600_6_12_moveit_config)/launch/$(arg moveit_controller_manager)_moveit_controller_manager.launch.xml" />
</launch>
```

Those are the major changes required. Hopefully you understood somewhat how to set the robot up. Once you start playing with the parameters, it should be fairly straightforward getting things running.