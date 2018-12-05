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
sudo arp-scan --interface=enp7s0f1 –localnet
```
If this errors out, you probably need to either install `arp-scan` or check your spelling.
I should also mention that the address you choose _must_ be in the range of `192.168.1xx.2` to `92.168.1xx.255`.
Now that you have figured out what IP addresses are currently is use, decide what IP address you would like to use. 
You first need to

### Documentation written by thatrobotguy
