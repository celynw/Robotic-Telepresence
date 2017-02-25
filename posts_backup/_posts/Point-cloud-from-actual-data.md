---
title: Point cloud from actual data
date: 2016-12-07 13:03:08
tags:
---
There were some errors in the [previous post](/Robotic-Telepresence/2016/12/05/Point-Clouds-in-Unity/).
There were 9,830,400 individual numbers in a single frame of the point cloud in the `rosbag`.
Looking at the [documentation](http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html) for the message type, and the header of the message, the number of points is far fewer.
```zsh Command line
$ rostopic echo --noarr --nostr /camera/depth_registered/points
header: 
  seq: 16832
  stamp: 
    secs: 1479833365
    nsecs: 438671766
height: 480
width: 640
is_bigendian: False
point_step: 32
row_step: 20480
is_dense: False
---
```
```none Message header
fields: 
  - 
    name: x
    offset: 0
    datatype: 7
    count: 1
  - 
    name: y
    offset: 4
    datatype: 7
    count: 1
  - 
    name: z
    offset: 8
    datatype: 7
    count: 1
  - 
    name: rgb
    offset: 16
    datatype: 7
    count: 1
```
The width and height of the cloud from the Kinect 1 is 640x480, with 32 bytes for each point.
`9830400 / (640 * 480) = 32`
According to the documentation for the [PointField](http://docs.ros.org/api/sensor_msgs/html/msg/PointField.html) message, each datum uses `datatype: 7`, which is a `FLOAT32`. This means 32 bits, or 4 bytes.
The data values in the PointCloud2 message are of type `uint8`, which uses 8 bits, or 1 byte.
This means that for each point in the message, there's space for 8x `float32`.

Examining the first 32 bytes of the first message, the fields described in the message header are visible:
```none First point in first message
0 0 192 127 //x
0 0 192 127 //y
0 0 192 127 //z
0 0 0 0
131 115 115 255 //rgb
0 0 0 0
0 0 0 0
0 0 0 0
```

For some reason, playing the rosbag with a windows listener subscribed to the point cloud, causes the `socket_node` to hang, even on one message, with network activity dropping to zero.
This happens even when minimally changed from subscribing to `JointStates`, (which works perfectly), and not storing or printing the output.
The same happens with octomap, which has significatnly smaller messages.
