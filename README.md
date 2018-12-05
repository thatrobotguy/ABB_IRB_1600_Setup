# WPI_ABB_IRB_1600_Setup
This is the repository that explains how to set up the ABB IRB 1600 robot arm at WPI's Washburn Labs with ROS.

## Prerequisites
This guide assumes many things about the people reading it. This guide is not for those kinds of people without any experience with ROS.

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
The first thing you will need to do is set up a static ip address on your ethernet nic. In order to network with the robot, you will be connecting your device to an ethernet port on a network switch. At the time of writing of this document, this is an 8 port switch sitting on top of the robot controller. This switch is expected to be upgraded soon, so ask the prof if you cannot find it.

## Static IP configuring
In order to set a static ip address, you will first have to know some stuff.
The ABB robot arm resides on a closed off local network. This means that it cannot access the internet (as one would hope).
The a device attempts to connect to the arm, the arm expects ab IP address of `192.168.100.100`. This means that your machine _cannot_ be assigned this address. I gave my laptop a static IP of `192.168.100.123`. If you want to know what IP addresses are currently in use on the network, plug in to the switch and type this:
```
sudo arp-scan --interface=myethernetcardname –localnet
```
This will list all of the IP addresses on the network that you have attached to your ethernet port on your laptop.
In order to find out what your ethernet IP is, just run `ifconfig`. You should see something like this:
If this errors out, you probably need to either install the software of check your spelling.
`sudo arp-scan --interface=enp7s0f1 –localnet`
