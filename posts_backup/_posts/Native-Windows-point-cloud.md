---
title: Native Windows point cloud
date: 2017-02-22 00:21:18
tags:
---
Most existing Unity plugins or assets seem to be focused on calculating bone positions of humans.
In the robot lab, I was showed how the Kinect (v1) was interfaced with using WPF C#.
This is the model which recorded the point cloud for the `ROSbag`, although there are v2s around which are better.

I can either adapt a sample from within the [Kinect Developer Toolkit 1.8.0](https://www.microsoft.com/en-us/download/details.aspx?id=40276), or adapt existing code already compatible with UnityScript, [Microsoft Unity Pro packages](https://developer.microsoft.com/en-us/windows/kinect/tools)
I will still try to stream them over `rosserial_windows` as this is not turning out to be an easy sidestep of the problem.
Perhaps there is a way to decimate the point cloud which could solve the problem.
