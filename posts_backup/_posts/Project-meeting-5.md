---
title: Project meeting 5
date: 2016-11-22 23:02:04
tags:
---
I'm definitely working on the right track, but whenever I get asked specifics about my plans I come up blank.
It's becoming increasingly clear that I need to update my Gantt chart! This will be done by next week I swear.

I learned about the octree.
While the kinect only has a line-of-sight view of the world, the octree retains data of geometry which is obstructed.
This geometry can only be removed if it is confirmed to not be there anymore, at which point its probability of being there is lowered gradually.

I filled out the health and safety forms for Baxter and the robot lab.
I should be able to go in and get some real experience with `ROS`.
(My swipe access might work now as well)

The most useful thing for me to do now would be to see if I could get specific communication using `rosserial`, not just general Boolean confirmation that it's happening.
We recorded Baxter waving his arms around for about 60 seconds.
This data was compiled into a `rosbag`, which supposedly can be swapped out for real data being streamed from Baxter with very little changes experienced.
In this `rosbag` we included:
- The joint angles.
  This should plug in to the Unity project because I already have the URDF and virtual representation
- The point cloud data from the Kinect
  I don't know how to do this yet, but there has got to be ways to do it, I'm not too worried.
- The octrees
  I'll have to think about this, but I'm sure it will be possible somehow.
To use the `rosbags`, you can do
```zsh Command line
$ rosbag (record | play) baxter.bag
```
It plays back in RViz fine. Lasts about 60 seconds.
We confirmed its integrity by transferring it to another computer and directly playing it back to Baxter.
Oscar will send me a dropbox link with the rosbag, it is quite large (~8.5GB) and we had trouble putting it on a USB stick (possibly because of the 4GB Fat32 filesize limit?)
Apparently a RAMdisk might be helpful for the rosbag.
If I need to translate coordinate frames relative to Baxter, the frame is 'base'"?
I knew I wouldn't remember everything but I know these things exist to ask about.
