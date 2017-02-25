---
title: ROS serial
date: 2016-11-21 13:03:18
tags:
---
Following the instructions [here](http://wiki.ros.org/rosserial_windows/Tutorials/Hello%20World) with a few edits.
`apt` can mostly replace `apt-get` now, and I'm using version indigo, not hydro.

Install rosserial_windows:
```zsh Command line
$ sudo apt install ros-indigo-rosserial-windows ros-indigo-rosserial-server
```
Make sure I've set up the environment:
```zsh Command line
$ cd ~/catkin_ws
$ . devel/setup.zsh
```
Generate ros_lib:
```zsh Command line
$ rosrun rosserial_windows make_libraries.py my_library
```
I couldn't run this, rosserial wasn't coming up in the tab completion for rosrun.
Refreshing the tree fixed it:
```zsh Command line
$ rospack profile
```
After running the installer, it failed giving the following message:
```none rosrun output
*** Warning, failed to generate libraries for the following packages: ***
    baxter_sim_kinematics (missing dependency)
    pr2_description (missing dependency)
```
Installing them:
```zsh Command line
$ sudo apt install ros-indigo-baxter-sim-kinematics ros-indigo-pr2-description 
```
ros-indigo-pr2-description was already installed...
So it wasn't surprising when this installation didn't help
Checking dependencies didn't help:
```zsh Command line
$ rosdep check --from-paths ./src -ignore-src
All system dependencies have been satisfied
```
It turns out that there were more dependencies to install:
```zsh Command line
$ sudo apt install ros-indigo-moveit-resources ros-indigo-convex-decomposition
```
This solved the `baxter-sim-kinematics` error.
After installing `ros-indigo-ivcon` it exited successfully.

In Windows, I created a new Console Application in Visual Studio.
Copied the generated folder to the project folder.
Added the files not in their own folders to the project.
Then I added the folder to the additional includes list.
After copying the 'hello world' code, it build and began running, listening on `1.2.3.4`.

As at this point I was running Xubuntu and ROS in a virtual machine, I tried to see if I could contact it.
I had to change the virtual machine's network settings from NAT ro Bridged adaptor.
After a reboot, I could ping it at the new address.
I changed the `ros_master` to the new address.
In the guest system, I ran three terminals as the instructions said to start the server.

I got a response in the built program!
![hello_world running on Windows](/Robotic-Telepresence/2016/11/21/ROS-serial/Visual Studio.png)
I also got the correct output in the virtual machine:
![rostopic echo /cmd_vel](/Robotic-Telepresence/2016/11/21/ROS-serial/Terminal.png)

I'd like to try and get this to work within the Unity project.
This might be a little difficult, as I'm using C# in Unity, but I haven't done the research yet.
