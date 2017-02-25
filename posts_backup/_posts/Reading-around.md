---
title: Reading around
date: 2016-10-13 17:28:51
tags:
---
### ROS compatibility
The main objective is to interface with ROS.
This is only officialy supported from a Linux distribution for now ([Homepage](http://wiki.ros.org/ROS/Installation)).
There is shaky support for [Windows](http://wiki.ros.org/win_ros).

### Vive compatibility
The Vive is interfaced with SteamVR which is only supported in Windows (through the Steam client).
Most personal VR projects I have seen don't integrate into the Steam client in any way, but it still needs to be open to run the SteamVR daemon.
However, the hardware itself is supposed to be quite open soit's likely that unofficial drivers exist for other operating systems.

### Possible courses of action:
##### Run ROS under Windows
Advantages:
The entire project could be run from a single operating system, so only one separate device from the robotics would be required.
Most modern game engines are only supported, or have the most compatibility with Windows, so I would be able to take advantage of the environments, tools, code snippets and superior rendering quality.
This also applies to graphics card drivers.

Worries:
The support for ROS is shaky and as I am inexperienced with it I won't know to what extent it isn't working or how to fix things, if even possible.

##### Run game engine in Linux
Advantages:
Again, the entire project could be run from a single device.
I wouldn't have to worry about extra issues with running ROS.
Most of my text-based programming experience is in Linux so it's a familiar environment.

Worries:
With a first look, most modern game engine development suites don't list support for Linux development
There may be some alpha builds, but Unreal and Unity don't list Linux distributions on their downloads pages.

An alternative would be to use raw OpenGL, however this might end up being a lot of extra work on 'primitive' functions which are second nature to popular game engines. I would presumably use some custom libraries which I would need to choose and learn to use.
This could make the executable slimmer and less dependent on the industry, but I'm not sure how much this matters.

##### Use a combination
Advantages:
This would take the best of both worlds, having compatibility with VR, game engines and ROS.
I have not done much reading on ROS yet, but it may be possible to have a low-powered networked device like a _Raspberry Pi_ to route communications from the VR headset in a game engine in a ROS-like way to the robotics.

Worries:
Having communications between devices in two places is a greater potential for errors.
I'm not sure how easy or hard this will be, I think Rich mentioned ROS/Python interoperability.

This seems like the way to go.

### Next steps:
Before I can do anything like a Gantt chart I should probably choose which of these paths to follow.
In order to make an informed decision I should do some reading and testing around this area.
1. Do some research on ROS. I hear there are some beginner tutorials so I can work my way through those.
I shouldn't need access to a physical robot to do them.
1. Look at VR and Unity(/Unreal) support under Linux.
These are the two Rich mentioned, and the only (or best) two I know of which support virtual reality.
I have used Unity before, and it has been free for personal use for much longer, so I will try this first.
1. Set up development environments.
This involves physical Linux partitions on my PC, and probably Linux virtual machines.
I will have to choose a distribution, it will likely be a lightweight flavour of Ubuntu (LTS) (Long-term support).
I'll also need to host my notes in a convenient, secure and easy manner. I'm writing this in a text document, I'll probably copy them over to the new format once I've decided.
As I'll be on multiple machines and operating systems I'm thinking of using Markdown, and hopefully something better than Dropbox/equivalent services.

### At some point
Work out to a greater degree of specificity the direction this project will take.
I'll be using the <INSERT BRAND> Baxter robot to start off with, but the project is to interface with ROS generally, so could be applied to many types of robot.

Do I use a purely virtual model of Baxter?
Do I use a point cloud from the Kinect, mounted above Baxter?
Do I do some combination of the two?
...or something else?
