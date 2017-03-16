---
title: Point cloud problem identified
date: 2017-03-16 19:19:24
tags:
---
As suspected, it wasn't the python script which was limiting the data to 8 points.
Something I couldn't check with the rosbag: running `rostopic echo /camera/depth_registered/points` gives an accurate response.

When using normal clouds:
- Described as {X, Y, Z}
- 12 bytes long (4 bytes for each axis)
- A 21 point message reports a length of 252 (as expected)
- A 22 point message reports a length of 8 (264 expected)

When using the point cloud type I need:
- Described as {X, Y, Z, \_, C, \_, \_, \_}
- 32 bytes long
- A 1 point message reports a length of 32 (as expected)
- A 7 point message reports a length of 224 (as expected)
- An 8 point message reports a length of 0 (256 expected)

It's not the python script. It's the serial communication.

With some further investigation:
`msg.data_length` is a `uint8_t`, which is the same length as a `char` and has a maximum of 255.
This is because `rosserial` was written with Arduino-like embedded devices in mind, which have much tighter hardware limitations than traditional PCs.
The modifications to ROS to allow for greater sizes for things such as point cloud, images etc. has been implemented for `Kinetic Kame` and the development branch of `Jade Turtle`.
The `msg.data_length` is now a `uint32_t`.
Somebody had success patching Indigo and building the source, I don't think I have this choice.

My options:
- Try to split the point cloud into chunks
  This "[sounds like a huge pain in the butt](https://github.com/ros-drivers/rosserial/issues/130#issuecomment-52449568)".
- Use `roscpp/winros` rather than `rosserial`.
  It could be too late in the process to make the move.
  Also I've heard it's not very stable or reliable.
  However [this post](https://github.com/ros-drivers/rosserial/issues/130#issuecomment-52449568) mentions that Hydro (before Indigo, the release we're using) it was very usable back in 2014.
- Generate `rosserial_windows` libraries from Jade
- Try to transmit much less data.
  Just sending locations of objects and rendering them in Unity3D. Discussed in the [previous project meeting](here)

Here are some more people with this problem
- [Recent](https://stackoverflow.com/questions/40813625/rosserial-publish-sensor-msgs-image-from-windows)
- [Same date as Github issues linked above, no answers](http://answers.ros.org/question/189571/array-size-limited-to-255-in-rosserial/)

#### Kinetic `rosserial_windows`
I set up a new virtual machine to install a fresh copy of `Kinetic Kame` to generate new `rosserial_windows` source.
This seems to run with no issue, even when the host `rosserial_server` is running `Indigo` (2 versions behind).
I modified the point cloud python script to use a single-byte point field to see exactly how much could be received.
Now I can send much more than 256 bytes... but there seems to be a hard limit of 1177 bytes, this would allow 36 full points...
Not nearly enough. Shouldn't I be getting up to 65536 bytes (2048 points)??

I even tried using the `Kinetic` environment as a host. I get the same problem, and the stability seems much worse!
