---
title: Project meeting 1
date: 2016-10-21 20:32:55
tags:
---
Following meeting with Rich (21/10/16), these things were discussed:

- I need a plan, a Gantt chart.
	It should also take into account other things like exams, coursework, and holidays.
- I don't need to do it all on Linux if I'm going to be communcating with ROS anyway.
	ROS is specifically designed with strong communication support.
	Baxter and others run their own instances of ROS anyway, It's ROS to ROS.
	If I had done this research first, I may have reached this conclusion.
- I think, ideally we'd want a 'ROS node' which communicates with Unity.
	Alternatively a Unity plugin to communicate with ROS could work but that might be horrible.
- Oscar mentioned octrees, how a 3D point cloud is represented in 3D space to give things surfaces.
	This might need to be done by me, but Unity as a game engine seems likely to have this built in, or have plugins for it.
- Remember `rvis` is for visualising the robot.
	`Gazebo` isn't the same, that is for simulation.
