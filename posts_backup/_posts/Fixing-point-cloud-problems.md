---
title: Fixing point cloud problems
date: 2017-02-28 22:50:35
tags:
---
Now that the lab computer has access to the network, it was possible to attempt to communicate with the ROS machine.
After sorting out an issue caused by Windows Firewall, it was possible to receive joint messages over `rosserial_server socket_node`.
However, a single `PointCloud2` message caused CPU to skip to (over) 100%, and the serial server to hang, just as for me.
Oscar found a python script which would publish a custom point cloud and regularly send the message, containing just 3 white points.
This was able to be received!
It's time to verify whether the problem was caused by the point clouds being too big, or if it was something like the colour interfering with the expected message format.

New test field format
> 
| 0-4   | 5-8   | 7-11  |
| ----- | ----- | ----- |
| X     | Y     | Z     |

Old real field format
> 
| 0-4   | 5-8   | 7-11  | 12-15 | 16-19 | 20-23 | 24-27 | 28-31 | 
| ----- | ----- | ----- | ----- | ----- | ----- | ----- | ----- |
| X     | Y     | Z     | --    | C     | --    | --    | --    |


The python script is only able to publish point clouds in the first format.
I had to reverse engineer the python file using functionality in:
`/opt/ros/indigo/lib/python2.7/dist-packages/sensor_msgs/point_cloud2.py`
It should match the format of the existing clouds from the Kinect wth Baxter.

I was able to receive three points in the new format.

Since the real point cloud data is in the byte format, I'll use random Matlab data to test.
```Matlab Matlab workspace
data = randn(length,8);
csvwrite('out.csv', data)
```
I then pasted the (formatted) values directly into the python file.

I found some weird problems. Looking into it:
`msg.data_length` is:
- 32 for 1 point (expected)
- 64 for 2 points (expected)
- 128 for 4 points (expected)
- ACCESS VIOLATION at 8 points when attempting to read `msg.data[0]`
- 224 for 7 points (expected)
- 32 for 9 points
- Nothing received for many (>100 points)
  - These can still be listened to locally with `rostopic echo <path>`
  - Everything is still responsive

On closer inspection, it seems like there is some kind of quantisation going on.
16 points gives the same error as for 8 points, and 9 points looks like 1 point.
Not quite sure why yet.
The python script makes `height = 1`, and `width = data length`.

I tried changing the buffer sizes in the generated C++ files on the Windows PC:
```cpp ros_lib/ros/node_handle.h excerpt
...
namespace ros {
  using rosserial_msgs::TopicInfo;
  /* Node Handle */
  template<class Hardware,
           int MAX_SUBSCRIBERS=25, // Reduced from 25
           int MAX_PUBLISHERS=25, // Reduced from 25
           int INPUT_SIZE=200000, // Increased from 512
           int OUTPUT_SIZE=200000> // Increased from 512
  class NodeHandle_ : public NodeHandleBase_
  {
...
```
This resulted in me being able to see large (1x640) messages!
However, the length is still zero with no data being transmitted.
By reducing the actual length by 1, so making it 1x639, the message was reported to have length of 224.
Not sure where this is being limited, but I'll try the real point cloud from the `rosbag`.
The same thing happens as before.

Reverting back to the example point cloud, made up of just [X, Y, Z], the limit seems to be `256` bytes.
as `22` points gives a `data_length` of `252` (4*3*22), but `23` points gives a length of `8` bytes.

Can read exactly 367 times more memory in `data[]` than I thought I could? (32 bytes, proper point cloud)
3056 times with simple point cloud

Windows application seems to make a buffer of arbitrary size, we should read until `data_length` even though the array doesn't end there.
Adapted python program to only print one point cloud and then quit.
