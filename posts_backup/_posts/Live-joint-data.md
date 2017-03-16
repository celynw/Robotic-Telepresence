---
title: Live joint data
date: 2017-03-08 10:51:47
tags:
---
I ran the new code and project in the robot lab.
I appears that the jerky motion exhibited by the `rosbag` was related to the recording.
The computer running `roscore` connected to baxter is not connected directly to the Unity PC, but they are in the same network.
As there are no problems detected so far, the URDF data contained in Baxter matches the URDF file in the Unity project.

{%youtube IKGjTFgAbt4%}

I got the ROSnode application to open with Unity, and close when the editor is stopped.
There isn't any kind of message to say when it's ready. If it's not ready, the shared memory is mapped after a couple of failed attempts, but a 500ms delay has been added to make sure it connects first time.

Initiallly I made it so a window doesn't open at all, now it seems like internal process.
Any output is printed to the Unity Editor log. It's redirected so if the window was visible there wouldn't be anything printed anyway.
It's also colour coded so it's easy to see where messages are coming from.
However, when doing this, the serial client loses synchronisation after around 20 seconds for some reason.
I suspect it's to do with an issue redirecting standard output and error, I didn't get a problem when running it as before.
Disabling this functionality for now.
