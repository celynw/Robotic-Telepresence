---
title: Live depth data in Unity
date: 2017-02-22 16:27:47
tags:
---
Due to the way Unity scripts work, many features in Visual Studio which are usually present are not available.
I wasn't able to use `using Microsoft.Kinect`, change the references, or change project properties.
My only option was to keep it as a separate application or modify some existing code.
Since I already have external applications, it was preferable to find the functionality in an existing asset.

The [Kinect with MS-SDK](https://www.assetstore.unity3d.com/en/#!/content/7747) plugin I found before, along with any others, seemed to focus more on calculating skeletal data.
This plugin was the most complete, and it had a demo which showed live depth data.

I hacked it apart, brutally removing anything which wasn't necessary.
I still don't have direct access to the data, and I'm not sure how difficult or intensive getting point cloud data will be.
The supporting code seems to have some direct communication to the Kinect, and I think I will be able to do things like decimate the data if it's too many points.
It should be relatively similar to the Kinect v2 as well.
![Me at the desk](/Robotic-Telepresence/2017/02/22/Live-depth-data-in-Unity/Depth Kinect.png)
TBC this week
