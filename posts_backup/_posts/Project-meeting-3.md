---
title: Project meeting 3
date: 2016-11-08 20:07:40
tags:
---
I still need to get swipe access to the robot lab.
After that I need to do health and safety briefing for Baxter.

I confirmed my suspicions that ROS versions don't generally mix, while the Kinetic upgrade is an especially significant change.
Baxter is running on Indigo and I would need a very good reason to use Kinetic instead.
Remember, Indigo doesn't work on Ubuntu 16. Use 14 for the LTS.

I got Oscar to send me some point cloud data
- Raw data directly from kinect (428MB)
- Coloured data (571MB)
- Coloured subsampled data (6.39MB)
- Monocular reconstruction cloud (17MB)
- Octree (53.1MB)
The point cloud data is in `.ply` binary format.

He also sent me a list of libraries I'll need to generate/view it.
```
meshlab
ros-indigo-octomap
ros-indigo-octomap-mapping
ros-indigo-octomap-msgs
ros-indigo-octomap-ros
ros-indigo-octomap-rviz-plugins 
ros-indigo-octomap-server
ros-indigo-octovis
```
Also, a link to the wiki page for [sending serial messages to ROS from Windows](http://wiki.ros.org/rosserial_windows/Tutorials/Hello%20World).

Oscar also said that a Raspberry Pi may not have a fast enough internet connection to handle the point cloud data, or have very good latency.
This isn't too much of a problem. In the robot lab, there's a computer dedicated to Baxter which has university log-on (first run will apparently still be a pain)
The others (including the beefy one with 4x12GB VRAM) don't, but I'll get some help when the time comes.
