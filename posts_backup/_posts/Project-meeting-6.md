---
title: Project meeting 6
date: 2016-12-01 16:57:35
tags:
---
#### Discussion about the previous week's work
I tried to explain the problems I was having with `.dlls`.
I had some potentially useful tips about how to get them to work with arrays. I should look into [marshal functions](https://msdn.microsoft.com/en-us/library/system.runtime.interopservices.marshal.aspx) and [interop](https://msdn.microsoft.com/en-us/library/04fy9ya1.aspx).
To analyse the extent of 'name mangling', I can put the `.dll` through [Dependency Walker](http://www.dependencywalker.com/).
I came across these in the past week, I think I should have more experience and knowledge with C# besides Unity scripting for this to be successful.

We have decided it would be much better to move away from compiling `.dlls` as Unity plugins, and instead use a compiled console application in Windows to receive the messages into RAM, and get Unity to poll the most recent values.
I have been talking to James, who is trying to implement UDP communication in Unity with ROS.

In terms of the networking layout, the VR PC will be subscribing directly from Baxter (who has an internal ROS publisher) and publishing to the linux computer connected to Baxter, which will be calculating the inverse kinematics from the hand target positions.
We thing a position and a pose is required for this, so I will need the quaternions of the rotations of each hand.
Unity works with quaternions so this doesn't sound too much of a challenge, but to start with I might well fix the pose to have the hands facing forwards.

I wasn't able to filter the joints that were in the public `urdf` and those in the `rosbag`. Maybe Baxter's urdf is a previous verion?
I was also strange that the gripper messages were arriving separately.

#### Interim report
The same examiner(s) who marks this will mark the final dissertation.
Therefore, don't commit to work which is unlikely to come to fruition. Light exaggeration could back me into a corner.
Also, there will be some overlap between the interim report and the final submission. They have to exist as separate entities, but maybe not a total copy.

I have two weeks left before Christmas. If I want any useful feedback (I do) I should have a report structure by [next meeting](/Robotic-Telepresence/2016/12/13/Project-meeting-7/).

