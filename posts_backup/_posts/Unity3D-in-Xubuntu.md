---
title: Unity3D in Xubuntu
date: 2016-10-20 15:41:58
tags:
---
The Unity website download page only lists **Windows** and **MacOSX** availability.
I finally managed to find the public beta [Forum thread](https://forum.unity3d.com/threads/unity-on-linux-release-notes-and-known-issues.350256/) with the Linux downloads.
I have the proprietary graphics drivers installed already, for my **NVidia GTX 970** GPU.
I'm going to try the most recent one, `5.5.0b5`.

I was able to get it to install and so far it appears to work flawlessly.
![Unity 5.5.0b5 in Xubuntu](/Robotic-Telepresence/2016/10/20/Unity3D-in-Xubuntu/Unity Linux.png)
The title bar says `OpenGL 4.5`.
In Windows, this is what you do to enable virtual reality:
Install the [SteamVR Plugin](https://www.assetstore.unity3d.com/en/#!/content/32647) asset.
Go `Edit > Project Settings > Player > Other Settings > Virtual Reality Suported`
But the [Unity page](https://docs.unity3d.com/Manual/VRDevices-OpenVR.html) for OpenVR states:
> Steam is required to run OpenVR applications, so install Steam and SteamVR. Once SteamVR is working properly with your headset, add OpenVR to the list of supported SDKs

If I can get SteamVR (OpenVR) to work, this should be able to work as well, but this depends on that for now.
