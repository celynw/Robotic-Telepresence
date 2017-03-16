---
title: Pose message from Unity
date: 2017-03-15 00:56:56
tags:
---
I open a new shared memory file for the poses to be sent out of Unity in a `geometry_msgs::Pose` message and subscribed to by Baxter.

The layout is as follows:
> 
| Offset     | Length | Description             | Notes                                    |
| ---------- | ------ | ----------------------- | ---------------------------------------- |
| 0          | 4      | Left Status bit (000X)  | X set to 1 if something has been updated |
| 4          | 4      | Left Position X         | -X from Unity                            |
| 8          | 4      | Left Position Y         | Y from Unity                             |
| 12         | 4      | Left Position Z         | Z from Unity                             |
| 16         | 4      | Left Orientation W      | W from Unity                             |
| 20         | 4      | Left Orientation X      | X from Unity                             |
| 24         | 4      | Left Orientation Y      | -Y from Unity                            |
| 28         | 4      | Left Orientation Z      | -Z from Unity                            |
| 32         | 4      | Right Status bit (000X) | Repeats, similar to the left side        |
|            |        |                         |                                          |
| 64         |        |                         | [ended]                                  |

In order to get the rotation correct (Baxter is rotated -90_x from ROS coordinates to be upright in Unity), I created a dummy parent `GameObject` with the relevant transform to hold the tragets, and then just used `transform.localPosition` and `transform.localRotation`.
This accounts for the _user_ being at the origin (which is how VR seems to operate) rather than Baxter being at the origin of his coordinate frame.

I'll have to do the same for the target models as well.
![You can see it's 90 degrees out. Also the endpoint for the IK service is between the ends of the grippers, while Unity thinks it's the centroid(?)](/Robotic-Telepresence/2017/03/15/Pose-message-from-Unity/Endpoint.png)

I have set it up so that any change to the number of targets in the queue will publish a message containing the pose of the frontmost (topmost) element.

