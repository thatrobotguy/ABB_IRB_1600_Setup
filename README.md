# WPI_ABB_IRB_1600_Setup
This is the repository that explains how to set up the ABB IRB 1600 robot arm at WPI's Washburn Labs with ROS.

## Prerequisites
This guide assumes many things about the people reading it. This guide is not for those kinds of people without any experience with ROS.

### Things to have setup before starting on this guide
1. A working installation of Ubuntu 1604 on any computer
2. This computer must have an ethernet port. Theoretically, you could run this installation with a USB to Ethernet adapter, but we have not tried that.
2. ROS Kinetic
3. Python 2.7
4. Skills on the Linux terminal
7. Counting

## In the beginning...
The first thing you will need to do is set up a static ip address on your ethernet nic. In order to network with the robot, you will be connecting you device to an ethernet port on a network switch. At the time of writing, this is an 8 port switch sitting on top of the robot controller, but this is expected to be upgraded soon.
