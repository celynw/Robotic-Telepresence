---
title: Pose control of Baxter
date: 2017-03-16 00:33:43
tags:
---
After learning about how the IK solver works, I realised I'd have to create another ROS node, this time on the Linux PC connected to Baxter.

#### Control of Baxter
Following some of the documentation on the Baxter site, I found an example which could be useful
There's an example which can be run using:
```bash Command line
$ rosrun baxter_interface joint_trajectory_action_server
$ rosrun baxter_examples joint_trajectory_client -l left
```
This moves Baxter's left arm to three preset positions.

#### IK service
Given a pose, this simply returns the planned path for the arm if a solution was found, it does not offer any control.
I can already subscribe to a pose topic from Unity.
The related example can be run like this:
```bash Command line
$ rosrun baxter_interface joint_trajectory_action_server
$ rosrun baxter_examples ik_service_client.py -l left
```
This runs for a hard-coded preset point.

Initially I thought about mashing these two together, but there were a few problems.
- Collision would be ignored. This is risky
- I have no easy way to debug, e.g. coordinate frame conversion errors

#### MoveIt
With some help from Rebecca, this solves some of those problems.

I wrote a python script which subscribes to two topics, `/vr/pose_l` and `/vr/pose_r`, both of which published by the VR PC.
On receiving a message, the IK service is called. It has a suitable planning time allowance, and final position error.
If a solution is found, the arm will move to that location.

The planned motion (and the subsequent motion) is shown in RViz.
Using RViz, I can also insert a primitive collision bound so that anywhere below table-height is off-limits.

```None baxter_table.scene
table
* table
1
box
3 3 0.05
0 0 -0.2
0 0 0 1
0 0 0 0
.

# Scene name
# Object name
# Number of shapes
# Shape type
# Size
# Position
# Orientation
# RGBA colour
# Necessary
```

Remember to run:
```bash Command line
$ source ../moveit_ws/devel/setup.sh
$ rosrun baxter_interface joint_trajectory_action_server
$ roslaunch baxter_moveit_config baxter_grippers.launch
$ python ~/path-to-script.py
```


#### Final touches
- The script can be optimised, it contains a lot of extra information found for tutorial purposes.
- I need feedback to Unity for when no solution could be found for a target position.
- I need feedback to Unity for when a target position has been reached.
- There are a few extra messages been published, such as messages which are all zero in initialisation. Iron out these.

#### Still to do!
'Telepresence' is nearly there. But it's useless unless we can get data representing the real world in.
I'll need to set up operating the grippers.
Once these are done I can begin the user tests.
