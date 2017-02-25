---
title: Robot models
date: 2016-10-23 14:32:34
tags:
---
### URDF
I've found there's a [Unified Robot Description Format](http://wiki.ros.org/urdf), an XML format which ROS uses.
I don't know the full extent for what it's for or what it contains yet.

### Baxter models
There's [an official page](http://sdk.rethinkrobotics.com/wiki/URDF) for Baxter's URDF.
I will try to see how easy it is to input its information into Unity.
There's also a [GitHub page](https://github.com/RethinkRobotics) for ReThink Robotics Inc. (Baxter).
Some of the meshes can be found there:
```zsh
$ git clone https://github.com/RethinkRobotics/baxter_common
```
Each body part is provided as a Collada `.dae` format.
Even the collision bounds models are included.
They can be dragged straight into Unity.
![Showing the torso and some random arm parts](/Robotic-Telepresence/2016/10/23/Robot-models/Models.png)
The lighting and shading is immediately higher quality than that in the simulator, Gazebo:

![This was taken from the web as an example.](/Robotic-Telepresence/2016/10/23/Robot-models/Gazebo.png)
A Google search yields a URDF to Unity GameObject converter, [neptune](https://github.com/MangoMangoDevelopment/neptune/wiki/Importing-and-Displaying-URDF-file).
> Our system will be able to detect when you're importing a URDF and create a corresponding Unity GameObject to represent the robot/sensor.

This will save time and maintain compatibility, rather than assembling the complete model myself.
Part of the `neptune` git clone seems to be a Unity project, so I've imported it and upgraded it to version 5.5.
It seems like I will have to use Visual Studio on Windows to compile the conversion utility, as Unity is expected to be running on Windows.

I managed to compile them, but I'm not sure how to use them. I am left with a `.dll` file in the 'Debug' directory.
It doesn't help that the GitHub project is mostly private.

I looked up if you could get Gazebo running in Windows.
The [site](http://gazebosim.org/tutorials?tut=install_on_windows&cat=install) says you can and gives instructions, but it's easy to miss this statement:
> It does not actually work yet.
