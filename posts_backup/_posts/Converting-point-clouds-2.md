---
title: Converting point clouds 2
date: 2017-01-02 13:47:58
tags:
---
To further my investigation of why I was unable to convert the PointCloud2 messages into usable values, I tried using `Rviz`, which I know can view the data properly.
It's a hassle as I have to restart my PC, and I lose access to most of my development tools (rviz won't run in a virtual machine).
If I can export some of the data, I'll have something to compare the results of my program to.
`rviz` showed how many messages were coming through, and you could select an area of points to see their data. However, you couldn't select _all_ points, or export the data.

Later, I came across a tool called `bag_to_pcd` [here](http://re.je/notes/2013/capturing-point-clouds-with-pcl-tools/).
I converted each message in the bag to a `.pcd` file, and then converted the first one into other formats from there.
```zsh Command line
$ sudo add-apt-repository ppa:v-launchpad-jochen-sprickerhof-de/pcl
$ sudo apt update
$ sudo apt install libpcl-all
$ sudo apt install libpcl-all-dev
$ sudo apt install ros-indigo-perception

$ rosrun pcl_ros bag_to_pcd ~/Downloads/baxter.bag /camera/depth_registered/points ~/Downloads
$ pcl_pcd2ply 1479833365.438671766.pcd output.ply
$ pcl_convert_pcd_ascii_binary 1479833365.438671766.pcd output.pcd 0
```
I was left with a binary `.pcd` file, an ascii `.pcd` file, and a binary `.ply` file (which didn't work, unexpected eof even with trailing newline)

I could view them to make sure they were still OK using `pcd_viewer`
```zsh Command line
$ rosrun perception_pcl pcd_viewer output.pcd
```
![Incredibly buggy in my virtual machine!](/Robotic-Telepresence/2017/01/02/Converting-point-clouds-2/View PCL.png)

### New information
When viewing the ascii `.pcd` file, it became apparent that huge sections at the start and end of the file was meaningless data.
This could help to explain why I wasn't getting anywhere.
I also learned that a devices like a kinect will often output `nan` values if the incident ray wasn't returned.

I'll have to re-export the `rosbag` message, so that it's the first message in the bag. Then I can compare my results to the actual points.
Hopefully then I can come up with a way to filter out the bad data efficiently.
