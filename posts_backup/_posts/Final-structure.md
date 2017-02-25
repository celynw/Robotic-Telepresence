---
title: Final structure
date: 2017-01-31 23:17:07
tags:
---
I've started to write what could be the final few files, in terms of the main data communication functionality.

#### Console application
This will be the ROS node, which connects to a serial server (socket).
It will subscribe to whichever topics are required and set up shared memory for each. As I can have many parallel shared spaces I think this is easier than trying to navigate a single one.
I'm trying a class-based approach to keep things tidy and encapsulated.
I had a lot of problems trying to encapsulate the callbacks (required), so in the end I've left some of it in `main()`.

As the shared memory can only be written in integers, I'll have to convert the joint angles (floats) back in to 4xbytes. Here's a reason to convert bytes into floats in the Unity script (point clouds).

#### Unity application
Once the data's in the shared space, it will probably be just a block of integers. I'll have to know how to read things in the correct order.
I've not worked out splitting work into individual threads yet, but I know it's supported and doable.
I plan to have the VR in one (I think this might be automatic), point clouds in another (they will be intensive) and anything else in another? Perhaps keep a limit of 4 cores?

#### Problems
Following [the last time I tried this](/2016/12/07/Point-Cloud-from-Actual-Data/) I ran into the same problem.
Even trying to send a single point cloud over the socket, the virtual machine's CPU jumps to around 20%, never completes, no errors are displayed but the `rosserial_server` node has silently crashed.
I'm hoping this is due to complications with it being the same physical machine and it being virtualised, but there's no problems with the joint angles at all.
I'll have to run this in the robot lab.

Using `std::vec` made sense for making a more robust program, but I had to convert it back into an array for the `CopyMemory()` function for shared memory and I ran into all sorts of endian-ness problems. They were difficult to spot as printing the contents to `stdout` didn't show any problems.
