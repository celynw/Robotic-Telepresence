---
title: Project meeting 4
date: 2016-11-15 19:46:17
tags:
renderer_options:
  gfm: false
---
#### Next steps
The next step to do is to try and animate Baxter.
We'll try and get some recorded motion from Baxter to replay as if it was being transmitted live.
Apparently the joint positions etc. are just smooshed together in a text file (not like the `urdf`, but the joint and link names will be consistent).
To get rosserial_windows working:
- Install packages in Linux (Ubuntu 14)
- Generate C# code
- Compile in Windows (or use in Unity..?)
- Run executable

#### Inverse kinematics (IK)
Baxter has an IK solver for working out joint positions based on an end effector position.
Unity probably has some similar functionality, it's common to want to animate boned meshes.
If it's feasible, I think it would be prefereble to use Baxter's solutions for this.
There can be multiple solutions for solving inverse kinematics.
If it makes a difference, Baxter's internal results could be more predictable/reliable.
It might also only require the data for the end effector position to be sent to Baxter rather than the whole arm.
The user would to move Baxter's hand with a Vive wand and request an attempt to move to that position.
I wonder what happens if you attempt to move to an impossible position outside of the movement limits.

Also, fill out the Health and Safety form for the Robot Lab.
