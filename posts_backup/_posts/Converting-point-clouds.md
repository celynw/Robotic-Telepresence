---
title: Converting point clouds
date: 2016-12-07 21:33:29
tags:
---
I spent a long time trying to write something which formats a message into data useable by Unity for generating a vertex mesh.
It should have been easier, but I ran into a lot of issues.

The general idea is to convert 4 bytes of the 32-byte point data to a float for each of X, Y, and Z.
The message even states the endian-ness of this layout ([documentation](http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html)).

I learned of the **union**:
```cpp Conversion code
    union {
      float f;
      int b[4];
    } data;
```
I couldn't use the whole data from a single message each time, I couldn't even open it in a text editor as it was too large.
Fortunately I could split the file without even opening it.
```zsh Command line
$ split -b 200000 PointCloud.txt
```
This generated a list of 200kB files, with `xaa` being the start with the header messages.

Whatever I did to the conversion program, I was always getting `nan` values, or crazily small ones.
I read about using the pcl library to use their methods for this rather than me implementing something myself.
Such methods exist in `pcl_ros/point_cloud.h`. However, this resource and the resources it depends on are part of the `ROS` environment.
Ideally, The Windows side would do the work, so that the Linux side only publishes the related data. It shouldn't have to know what's on the other end.
