---
title: Kinect with Unity
date: 2017-02-14 19:21:31
tags:
---
To side-step the communication issues with `PointCloud2` for now, I'll try to connect the Kinect directly to Unity and its host PC.
[This video](https://www.youtube.com/watch?v=YLzMT_Epvpw) shows that moving point clouds are possible, but I'm not sure how out-of-the-box it will be.

The official Kinect SDK apparently uses .NET 4 rather than Unity's .NET 2.
I installed version 1.7.
I also added the "[Kinect with MS-SDK](https://www.assetstore.unity3d.com/en/#!/content/7747)" asset from the asset store.
It apparently includes a simulator for the Kinect, but it didn't seem to do anything.
As with most other things I've found, it's geared towards calculating skeletal data, but will hopefully allow access to the point clouds.
This applies to what I think was being referred to for Unreal, the "[Kinect 4 Unreal](http://www.opaque.media/kinect-4-unreal/)" plugin by Opaque Media Group.
I'm pretty sure this isn't compatible in any way.
