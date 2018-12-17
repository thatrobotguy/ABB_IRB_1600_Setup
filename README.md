# WPI_ABB_IRB_1600_Setup
This is the repository that explains how to set up the ABB IRB 1600 robot arm at WPI's Washburn Labs with ROS.

## Prerequisites
This guide assumes many things about the people reading it. This guide is not for those kinds of people without any experience with ROS. This guide refers to other guides so this tutorial is not complete in the traditional sense. I have to point you to the other links because they may get updated sooner then this document. Also, I do not maintain those packages, so I am not the one with all the answers. This document is simply how I got my machine working, but with much more clarity than scratch work and other [MQP git repositories](https://www.wpi.edu/academics/undergraduate/major-qualifying-project).

#### Things to have set up before starting on this guide
1. A working installation of Ubuntu 1604 on any computer
2. This computer must have an ethernet port. Theoretically, you could run this installation with a USB to Ethernet adapter, but we have not tried that.
3. ROS Kinetic
4. Python 2.7
5. Skills on the Linux terminal
6. Stable internet connection
7. git
8. Your favorite text editor

### In the beginning...
The first thing you will need to do is set up a static ip address on your ethernet nic. In order to network with the robot, you will be connecting your device to an ethernet port on a network switch. At the time of writing of this document, this is an 8 port switch sitting on top of the robot controller. This switch is expected to be upgraded soon, so ask the prof if you cannot find it. I should thank the MQP team for pointing me in the right direction for some of these links. If anybody feels there are changes that need to be made, make a pull request or create a github bug thing or whatever it is called. Either I or somebody else may have seen the bug before. I have been making frequent updates so that this document looks as clean as possible. I also should mention that I have tried to list the steps in order, but some steps _can_ be done out of order, and some steps may be in the wrong order, so you should read this document in its entirety before embarking on the installation of ROS Industrial and MoveIt!. I should also mention that, before blindly doing the steps in all of these tutorials, you need to read the steps I ask you to do before you do them. For example, multiple tutorials show different commands that have to do with modifying your `~/.bashrc` flie. I already provide most of the environment variables that are necessary to make ROS work, therefore you should only add environment variables when they have been __explicitly__ mentioned __outside__ of this tutorial  and __not__ explicitly mentioned __here__. I should also mention that, for the networking portion of this tutorial, I am assuming that you are on WPI internet. If not, setting the static IP may require a different process. At home, I have to set the static IP on the router, not my desktop. Even then, I can only use simulation in this case. You will have to think about what you are doing before you go and do what these other links say to do. Also, if I say to git clone something, assume that it needs to be `git clone xxx` in your catkin workspace.

## Updates

I will be updating this document periodically. I will pose those updates __TO THE BOTTOM__ of the document. Be sure to check those out before continuing on...


## Static IP configuring
In order to set a static ip address, you will first have to know some stuff.
The ABB robot arm resides on a closed off local network. This means that it cannot access the internet (as one would hope).
The a device attempts to connect to the arm, the arm expects ab IP address of `192.168.100.100`. This means that your machine _cannot_ be assigned this address. I gave my laptop a static IP of `192.168.100.123`. If you want to know what IP addresses are currently in use on the network, plug in to the switch and type this:
```
sudo arp-scan --interface=myethernetcardname --localnet
```
This will list all of the IP addresses on the network that you have attached to your ethernet port on your laptop.
In order to find out what your ethernet IP is, just run `ifconfig`. You should see something like this on the output:
```
$USER@$USER-MyComputerModel:~$ ifconfig
enp7s0f1  Link encap:Ethernet  HWaddr jokes:on:you:bro
          UP BROADCAST MULTICAST  MTU:1500  Metric:1
          RX packets:0 errors:0 dropped:0 overruns:0 frame:0
          TX packets:0 errors:0 dropped:0 overruns:0 carrier:0
          collisions:0 txqueuelen:1000 
          RX bytes:0 (0.0 B)  TX bytes:0 (0.0 B)

lo        Link encap:Local Loopback  
          inet addr:127.0.0.1  Mask:255.0.0.0
          inet6 addr: ::1/128 Scope:Host
          UP LOOPBACK RUNNING  MTU:65536  Metric:1
          RX packets:277219 errors:0 dropped:0 overruns:0 frame:0
          TX packets:277219 errors:0 dropped:0 overruns:0 carrier:0
          collisions:0 txqueuelen:1000 
          RX bytes:45637381 (45.6 MB)  TX bytes:45637381 (45.6 MB)

wlp0s20f3 Link encap:Ethernet  HWaddr not:for:you:sucka
          inet addr:192.168.0.23  Bcast:192.168.0.255  Mask:255.255.255.0
          inet6 addr: fe80::a11f:ab53:e881:ad/64 Scope:Link
          UP BROADCAST RUNNING MULTICAST  MTU:1500  Metric:1
          RX packets:20356 errors:0 dropped:0 overruns:0 frame:0
          TX packets:12788 errors:0 dropped:0 overruns:0 carrier:0
          collisions:0 txqueuelen:1000 
          RX bytes:17042440 (17.0 MB)  TX bytes:2453717 (2.4 MB)

```
Given this output, I would type this:
```
sudo arp-scan --interface=enp7s0f1 --localnet
```
If this errors out, you probably need to either install `arp-scan` or check your spelling.
I should also mention that the address you choose _must_ be in the range of `192.168.1xx.2` to `92.168.1xx.255`.
Now that you have figured out what IP addresses are currently is use, decide what IP address you would like to use. 
You first need to click on the internet symbol in the upper right hand corner (furthest left icon):

![Image of Ubuntuwifi](https://github.com/thatrobotguy/WPI_ABB_IRB_1600_Setup/blob/master/upperrightcorner.png)

Now you need to select the ethernet option in the next menu like so:

![Image of network options](https://github.com/thatrobotguy/WPI_ABB_IRB_1600_Setup/blob/master/networkoptions.png)

Then you click `edit`. Now, go to the IPv4 settings tab and set the `Method` to `Manual`. now you need to enter the address and netmask and then click `add`. Set your machine IP like this:

![Image of ipv4 settings](https://github.com/thatrobotguy/WPI_ABB_IRB_1600_Setup/blob/master/ipv4_settings.png)

Now once these are set you will want to click `save` and then restart your machine to make sure all your changes are written properly.

## ROS Setup

First, you will need to make sure you have `ros-kinetic-desktop-full` installed on your ubuntu 1604 linux machine. My ROS catkin workspace is located here:
```
/home/$USER/catkin_ws/
```
This may be different on your system. If you have not yet done this installation you can go to [this link](https://wiki.ros.org/kinetic/Installation/Ubuntu).

I have some environment variables set up in my ~/.bashrc file.
This is the line that connects my path to ROS 1 setup.bash:
`source /opt/ros/kinetic/setup.bash`
I have to source this in my catkin workspace:
`source ~/catkin_ws/devel/setup.bash`
We also have to specifiy where `roscore` is running. We will be specifying that it runs on the Ubuntu laptop you are installing `ROS-I` and `MoveIt!` onto:
```
# Grab the machine IP addresses
machine_ip=(`hostname -I`)
rosport=':11311'
rosmasterbegin='http://'
# Now we set the ip location for roscore
export ROS_MASTER_URI=$rosmasterbegin${machine_ip[0]}$rosport
# This is the hostname of the current machine.
export ROS_HOSTNAME=${machine_ip[0]}
# This is the ROS distribution that we are running
export ROS_DISTRO=kinetic
# Sometimes you need to set ROS_IP for transforms and the parameter server
# https://answers.ros.org/question/163556/how-to-solve-couldnt-find-an-af_inet-address-for-problem/
export ROS_IP=${machine_ip[0]}
```
Supposedly you do not need to set ROS_MASTER_URI or ROS_HOSTNAME, but I seem to get more stable results setting these environment variables, I am telling you to do the same.
If you do not want to use `hostname -I`, you can run `ifconfig` and look for `inet addr: xxx.xxx.xxx.xxx` to set the address.
Make sure to run `source ~/.bashrc` in all open terminals once that is all done. If you want an official explanation to what these variables are doing, [check out this link.](http://wiki.ros.org/ROS/EnvironmentVariables)

This is the [main link](http://wiki.ros.org/Industrial/Install) to install `ROS-Industrial`. Because we will be building from source, however, you can skip this link. I only show you this because some people do not like not seeing other options. I would not rely on this link to give you what you need.

You now need to git clone [this repository](https://github.com/ros-industrial/industrial_core) for the core functionalities of ros-industrial.

## MoveIt! Installation

Next, you will want to go to the [link provided here](http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/trac_ik/trac_ik_tutorial.html) and run the `sudo apt-get install xxxxxxx` commands.

You will also have to apt install this thing:

`sudo apt-get install ros-kinetic-tf2-geometry-msgs`

Now you need to install MoveIt! You can go [here to install](http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/getting_started/getting_started.html) the regular way or you can go [here to install](http://moveit.ros.org/install/source/) from source. I got everything to work by first starting with the `apt-get` installations from the first link until nothing worked: then I went to the second link to install from source.I expect that, when you get to the second page from the first link, you cannot go further. This is because you need to do the advanced setup for MoveIt (build from source) since you still do not have all of the necessary packages set up. We will be modifying the launch files for the `abb` robot so it is best to just build `MoveIt!` from source. The MQP team said they had to build from source, so you probably should. When you build from source, make sure that you go to the bottom of the page and use the `build from source` instructions that uses the `ROS_DISTRO` environment variable; but do not forget the `apt-get` commands in those instructions. Also, where the MoveIt advanced install instructions say to do `catkin build`, you really want to do `catkin_make`. __If you do not do this, your catkin workspace will be BROKEN.__ As a reminder, this is the [link to the main webpage](https://moveit.ros.org/
) for the MoveIt libraries. This should not be necessary, but it is nice to know where other documentation is located.

## ABB Specific Installation

Once you have built the moveit libraries from source, we need to get the ABB specific packages.

You need to `cd` into your catkin workspace source folder, which is for me

`~/catkin_ws/src/`

You will need to git clone some repositories into your workspace.

Git clone this one on the `kinetic-devel` branch.
https://github.com/ros-industrial/abb_experimental

Run `git branch -a` to figure out what branch you are currenly on. If you are on the wrong branch, run

`git checkout kinetic-devel`

that will put you on the correct branch.

# WAIT! 

These are the old instructions:
```
You now need the abb robot driver. This is in the non-experimental abb repository for ros industrial.
This is the [link you need](https://github.com/ros-industrial/abb.git) to git clone.
```

You need to git clone my fork instead [which is linked](https://github.com/thatrobotguy/abb_experimental) here.

Now that you have downloaded all of those repositories into your workspace, you can finally run `catkin_make` in
`/home/$USER/catkin_ws/`.

I should note that you should NOT `catkin_make` until everything is git cloned, except for after using `wstool` to get the `MoveIt!` repositories when building from source.

### Now that you have git cloned everything, you now can skip the "ABB Package Modifications" steps and go to the "How to start up the ABB IRB 1600 with ROS" steps

## ABB Package Modifications

Once you have gotten the `ROS-Industrial` and `MoveIt!` installations squared away, we will have to do some manual modifications of launch files before you connect the robot to ROS.

In the ros package `abb_experimental` there is a subpackage called `abb_irb1600_6_12_moveit_config` with this launch file:
```
moveit_planning_execution.launch
```
You need to edit this launch file to decouple joints 2 and 3. For me, this file is located here:

`/home/$USER/catkin_ws/src/abb_experimental/abb_irb1600_6_12_moveit_config/launch`

I do not remember what the old version of the file looks like, but I only commented out lines. I did not delete anything.
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
I did make one other change to the launch files. This one should not be _necessary_, but I have included it here so that I am not missing anything as I happened to make these changes in the process. This launch file only matters when you are running your code on the real robot as opposed to the simulated robot. I talk about it more in the final notes section. This is the .xml file I edited: `trajectory_execution.launch.xml`. (It is in the same directory as the upper level launch file, If I am not mistaken.) I got the idea to edit it [from this post.](https://answers.ros.org/question/196586/how-do-i-disable-execution_duration_monitoring/)

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
## How to start up the ABB IRB 1600 with ROS

Now that you have ROS installed with the ABB packages, we have to now look at the robot arm itself and the teach pendant. The first order of business is to boot the robot arm into `System2_ROS`. Assuming you are currently booted into `System2`, you will need to switch OSs. Click on the ABB logo in the upper left hand corner. Then click `Control Panel`. Then look down low at the last option in the list above the boot options. The option you want to click is `installed systems`. You then need to select the `System2_ROS` option and then look down and to the right and select `Activate`. Select `Yes` to start the reboot. The pendant will then reboot to the ROS arm configuration. Once that is done, you need to click on the “ABB” in the upper left and then select the `Program editor` option. Then you select `Debug` on the bottom and then select `PP to main` and then you should be set to go to…

STOP! NEVER FORGET THIS LAST STEP!

Press `play` on the teach pendant. ONLY THEN can you execute trajectories.

BUT WAIT! THE TEACH PENDANT ERRORS OUT!

You must be pressing the motors enable grip down just enough to enable the motors before your program will run, even if you are controlling everthing with ROS. If you forget to do so, the teach pendant will throw errors. This is true only if you are in manual mode.

Now that you have installed everything, you can now run this command to start the simulated robot up with ROS:
```
roslaunch abb_irb1600_6_12_moveit_config moveit_planning_execution.launch robot_ip:=192.168.100.100 sim:=true
```
Or start the real robot with ROS:
```
roslaunch abb_irb1600_6_12_moveit_config moveit_planning_execution.launch robot_ip:=192.168.100.100 sim:=false
```
The only difference being the `sim:=true/false` at the end.

## Final notes

I find that the ROS stuff is more stable if you start the robot `pp to main` first, then run the launch file. You also do not need to start roscore separately as the launch file will do that for you. If you run into issues with roscore not working, you probably need to make sure you set the environment variables properly.

I believe that this is everything required to get the arm to work. You should call the uppermost launch file described earlier in their own custom launch file in their own custom ROS package so that they do not have any of the ROS-Industrial packages in their personal git repositories. The MQP team made that mistake already.

## Possible Errors

Sometimes you may get this error while running in manual mode:
```
[ERROR] [1544201172.958564157]: Controller is taking too long to execute trajectory (the expected upper bound for the trajectory execution was 4.625994 seconds). Stopping trajectory.
```
You need to look at that 3rd launch flie I provided and give the robot more time to execute trajectories.
You can do that by changing this line:
```
<param name="trajectory_execution/execution_duration_monitoring" value="true" />
```
To this line:
```
<param name="trajectory_execution/execution_duration_monitoring" value="false" />
```
It will no longer care about how long the trajectories take. When you start using Automatic mode, you should probably turn this back on.
You could also make this value larger:
```
<param name="trajectory_execution/allowed_goal_duration_margin" value="3.5"/> <!-- default 0.5 -->
```
But if you are in manual mode it is easier to just turn of this safety. [This link](https://answers.ros.org/question/196586/how-do-i-disable-execution_duration_monitoring/) has more discussions about this problem.

You may also encounter [this issue](https://github.com/qboticslabs/mastering_ros/issues/24) while trying to drive the robot. Also, if you are wanting to listen to transforms, you need to create a listener AFTER you call `initnode(argc, argv, "mynodename")`. This means that you cannot create a global variable listener outside of `main()` or `rosrun mypackage mynodewithtflisteners` will not work.

If you have been poking around this repository, you will notice that there is a launch file and a ros node provided. This si an example action server that receives a pose message and then handles the communication with the robot launch file. When you start the action server, _right after you start the robot node_, the action server will automatically set the orientation of any received poses to the rpy angles set in the referenced yaml file. All you need to do is send "x,y,z" values that are non-zero and the robot will handle the joint orientation. Currently, you need to have the robot set to the zero position before you start the action server. I highly recommend changing this so that you can arbitrarily move the robot (thorugh RViz) before you start the action server. This ros package is simply an example on how to control the arm with ROS 1. You should be able to get the arm up and running in a couple hours with a simple pose publisher _that publishes valid possible poses in the task space_. Do not worry: the MoveIt! IK planner is smart, but do remember, the collision for link 2, at the moment, is currently too forgiving.

## Updates

I have decided that, given the amount of changes to the `abb_experimental` ros package, that it is best to simply fork it into my own github account and then modify everything there. Not only are the 3 launch files changed, but the .urdf and .xacro are also incorrect. Link 2 is actually 0.7 meters long, not 0.485 meters long (1.45 meter robot, not the 1.2 as stated before: both are the 6kg version). Simply clone [this repository](https://github.com/thatrobotguy/abb_experimental) instead of the stock `abb_experimental` repository and the edits will be taken care of for you. __Do note, however, that THE COLLISION PROTECTION IS NOW BROKEN__. This means that the robot could destroy itself until this is fixed. I will have to ask around about this. Ideally, this would be in a package called `abb_irb1600_6_145_moveit_config`, but I have not gotten that far yet. I have simply been reusing the `abb_irb1600_6_12_moveit_config` ros package. Once this is done, I can merge with the original repository hosted by [these guys](https://github.com/ros-industrial).

#### More good links:
```
https://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/move_group_interface/move_group_interface_tutorial.html
https://docs.ros.org/kinetic/api/moveit_tutorials/html/index.html
https://wiki.ros.org/ROS/EnvironmentVariables
https://industrial-training-master.readthedocs.io/en/latest/_source/session4/Motion-Planning-CPP.html#further-information-and-resources
https://industrial-training-master.readthedocs.io/en/latest/_source/session2/Launch-Files.html
https://industrial-training-master.readthedocs.io/en/latest/_source/session2/Actions.html
https://industrial-training-master.readthedocs.io/en/latest/_source/session3/Build-a-Moveit!-Package.html
http://wiki.ros.org/abb/Tutorials
https://erlerobotics.gitbooks.io/erlerobot/en/ros/tutorials/rosnavigating.html
```
## Documentation written by thatrobotguy
