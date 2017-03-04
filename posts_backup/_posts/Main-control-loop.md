---
title: Main control loop
date: 2017-03-02 21:37:29
tags:
---
#### Current objective
I really need to get the main control loop sorted. Each part has been demonstrated to work separately, but it needs to work all together.
I need to get joint angles from the robot (using a `rosbag` for now), which are published on a linux machine running `roscore`.
These need to be subscribed to from a Windows machine, and the data should be put into shared memory.
This data needs to be read by Unity in and each corresponding joint should be rotated in real time to mirror the robot.

I then also need to be able to set target positions and poses for the hands using the tracked Vive controllers in VR.
These targets need to be put back into some shared memory, which is read by the console application and published back to ROS.

#### After that
I still need to know how to tell Baxter to move, I aim to do that this week.
I also need to implement bounds, for target positions. I can do that at home.
Now that the grippers are in the model, I can implement messages for those too (but they don't move in the `rosbag`)
I suppose I can call the executable directly from Unity, making it more seamless. Not a priority but might be useful.

#### On the back burner
Point clouds, grr.
It will (should...) be much easier to transmit a depth image over the network, so if I can work out how to convert that into a surfacein Unity, I might be able to implement functionality for the kinect, whether it's plugged into the Windows or ROS Linux machine.
This isn't taking priority right now though.

#### Development work
I had to reinstall my virtual machine as the virtual hard disk got corrupted. Probably because I previously ran it on a RAMdisk and had to copy it over each time.
This was not particularly useful or sensible, and should be fine on a SATA SSD.
I had to redownload the `rosbag` we recorded before. This time it would sit in a shared folder on the host machine, not inside the virtual hard disk.
To save time I used the compressed version which I didn't notice a problem with before (1.38GB as oppposed to 7.95GB).
This time, the compressed bag playback halts as soon as something tries to subscribe to a topic, even when locally running `rostopic echo /robot/joint_states --noarr --nostr`.
The uncompressed one plays fine.

I have a lot of code files and scripts floating around in various projects and folders, so it was not straight forward to remember which ones had the working code in.
It took around 4 hours, but I was eventually able to get everything working for data getting from ROS to Unity
![Baxter is waving as in the video in <a href=/Robotic-Telepresence/2016/11/28/Rosbag-communication>this post</a>, but this time using lovely OOP using shared memory, rather than janky code with pipes](/Robotic-Telepresence/2017/03/02/Main-control-loop/Waving.png)
