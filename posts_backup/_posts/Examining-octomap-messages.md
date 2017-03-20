---
title: Extending rosserial size
date: 2017-03-17 12:05:47
tags:
---
The `rosbag` I have access to contains octomap messages.
Using `rosbag info`, it has 55 messages. Echoing the first one shows it contains 83860 bytes.
The maximum I've been able to get using `rosserial` is 1177, although I'd expect it to be 65536.

Managed to get it up to 2221 points by increasing `INPUT_SIZE` and `OUTPUT_SIZE` from 512 to 1024
(Repeatable):
1776 using 512
	3.46875
2221 using 1024
	2.1689453125
4065 using 2048
	1.98486328125
		31
65468 using 65536
	0.99896240234375
		68

Need 9830400 every 0.1 seconds...
1500 Hz at this throughput!

An interesting development.
When publishing only just over the maximum buffer, `rosserial_server` exits without hanging:
```bash Command line
terminate called after throwing an instance of 'ros::serialization::StreamOverrunException'
  what():  Buffer Overrun
Aborted (core dumped)
```
This time, the limit is ROS in linux.
It's not a compatibility issue between different versions of `rosserial_server` and `rosserial_windows` (tested it).


```bash Command line
$ mkdir -p ~/catkin_ws/src
$ cd !$
$ catkin_init_workspace
$ cd src/
$ git clone -b indigo-devel https://github.com/ros-drivers/rosserial.git
$ cd ../
$ catkin_make
```
Sending 1000x largest clouds at 100Hz.
`rostopic echo` is as expected, takes 10 seconds.
Windows subscriber stops at 751, takes
	7.499 to 37.415 = 29.916 seconds
	3.666 to 32.948 = 29.282

Decreasing wait in spin() loop to 1ms from 5ms
Did 852 this time!
	3.733 to 30.648 = 26.915 seconds

Switching to Release in Visual Stuio causes `rosserial_server` to complain:
```bash Command line
[ERROR] [1489763341.521603888]: Buffer overrun when attempting to parse setup message.
[ERROR] [1489763341.521666993]: Is this firmware from a pre-Groovy rosserial?
[ERROR] [1489763341.521753826]: Buffer overrun when attempting to parse setup message.
```
95.63% of the CPU used was in `WindowsSocket::read`.


Multi-threaded spinning?! Yes please.
http://wiki.ros.org/roscpp/Overview/Callbacks%20and%20Spinning#Multi-threaded_Spinning

It's not a part of `rosserial_windows`...
There's a `roscpp` folder but there's not much in there.

Mashed!
- https://github.com/ros/ros_comm
- https://github.com/ros/roscpp_core

WallTime and WallDuration replace with Time and Duration respectively.
Build succeeded! WOW.
No. No.

16th of optimised point cloud = 19200
25.5Hz at max data size (65468), limited by CPU though (single core).
87Hz max like this (my CPU). Ok.

https://www.phoronix.com/scan.php?page=article&item=steamvr-linux-beta&num=4
