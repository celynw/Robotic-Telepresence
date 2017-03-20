---
title: Extending rosserial size
date: 2017-03-17 12:05:47
tags:
---
This didn't work when using a library derived from `Indigo`, but I was able to increasing `INPUT_SIZE` and `OUTPUT_SIZE` in `node_handle.h` from 512.
Since an `I32` is used instead now, this can be very large.

I set up a python script to publish a point cloud with a field of a single byte to see what the maximum throughput could be.
The highest I could get was `65468`. This now seems to be a limitation of `rosserial_server`.
When publishing only just over the maximum, `rosserial_server` exits without hanging:
```bash Command line
terminate called after throwing an instance of 'ros::serialization::StreamOverrunException'
  what():  Buffer Overrun
Aborted (core dumped)
```
However, sending more causes the hanging issue and 100% CPU usage.
Restricted to this limit, if I was to split full-size point clouds up, to publish 9,830,400 bytes every 0.1 seconds I would need to publish/subscribe at 1500 Hz!

To test the rate I can send them, I've set up the python script to publish 1000x of the largest clouds at 100Hz.
`rostopic echo` running locally is as expected, there is no bottleneck.
The Windows subscriber stops at 751, takes between 29 and 30 seconds.
I think this is because a backlog of messages is filled and some messages are dropped.
By decreasing the wait in the `spin()` loop to 1ms from 5ms gives 852, taking 27 seconds.
By removing the wait entirely, the performance is a little better but it isn't good enough.
Using Visual Studio's profiler, I found that I reached 100% CPU on one core (~12% total). 95.63% of the CPU used was in `WindowsSocket::read`.

ROS can support [multi-threaded spinning](http://wiki.ros.org/roscpp/Overview/Callbacks%20and%20Spinning#Multi-threaded_Spinning
). However, it's not a part of `rosserial_windows`.
There's a `roscpp` folder, which is where I'd expect to find it natively, but there's not much in there.

If I can optimise the point cloud (I think half of the data being sent is all zeros), I may be able to make some compromises.

=======
In other news, VR [may be coming to Linux](https://www.phoronix.com/scan.php?page=article&item=steamvr-linux-beta&num=4), finally.
This could have made my project much easier, had it been out 6 months ago.
