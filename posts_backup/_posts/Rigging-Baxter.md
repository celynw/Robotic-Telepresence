---
title: Rigging Baxter
date: 2016-10-27 17:05:31
tags:
---
I've managed to get the Baxter meshes into Unity, but they are completely separate parts.
ROS uses a `.urdf` file format to relate parts to each other, which isn't directly compatible or convertible to a Unity supported format.
In order to progress, I need to manually rig Baxter.
I can view the official files as they are text-based (an `xml` format).
Within the `baxter.urdf` file, there is a header:
```
baxter.urdf is DEPRECATED. It is included here for backward compatibility.
Please use baxter.urdf.xacro by running:
$ rosrun xacro xacro.py - -inorder `rospack find baxter_description`/urdf/baxter.urdf.xacro
```
The `xacro` format also is `xml` based.
It seems to be the same, but also include metadata for the materials.
I'll be using this `.urdf.xacro` file for reference and ignoring the `.urdf`.

I started with the torso, as that was described in the `baxter_base.urdf.xacro` as being at the origin.
![](/Robotic-Telepresence/2016/10/27/Rigging-Baxter/Torso.png)
I am thinking of adding the collision models later when(/if) I need to.
There are two files in the head folder, `H0` and `H1`. The `urdf` describes `H1` being called the 'screen'.
By looking at photos of a fully assembled Baxter, `H1` is a round part which appears to attach below the head part of the torso model, and the screen should attach to that.
Unfortunately, the coordinates in the `urdf` do not match the coordinate system in Unity.
- An decrease in `Z` in the `urdf` is an increase in `Y` in Unity.
- The units in the `urdf` appear to be almost 100x smaller than those in Unity.
Even manually moving the `H0` part to the correct Y location, it is still offset, even though the `urdf` describes the other axes as being zero.
![Showing the strange offset](/Robotic-Telepresence/2016/10/27/Rigging-Baxter/Offset.png)

I also noticed that the screen (`H1`) was rotated 90 degrees. The `urdf` describes the rotation in radians, while Unity describes it in degrees.

Looking at a `urdf` [tutorial](http://wiki.ros.org/urdf/Tutorials/Building%20a%20Visual%20Robot%20Model%20with%20URDF%20from%20Scratch), ROS uses metres as the units of distance.
Unity uses generic units, but in relation to the scale of the model parts this looks about correct.

After writing half an email to Oscar, I worked it out.
I had a better look at the `urdf` and noticed that the `"head"` link is mentioned in some `joint` objects.
Specifically, the `"head_pan"` joint with the parent of `"torso"` and the child of `"head"` had the correct coordinates (some inverted and not the correct xyz).
The `urdf` is heirarchical, with inherited transformations.
Unity also works like this, so I can lay out Unity GameObjects in the same manner as in the `urdf`. Duh.
Perhaps the origin values in the links themselves could be the internal origins in the `.dae` files?

I started off with the parent-most object: the `base`. Everything inherits from here.
As I went along, GameObjects were made for joints, links, parents, children... I started assigning Unity tags to them to remember what was what, keeping their original names.
The `head_camera` joint has a child link called `head_camera`, so GameObjects like these need to be separate.
As I was going through, it became obvious that going down the hierarchy, it alternates between joint and link.
Imay have to specify between `"fixed"` and `"revolute"` joint types.
To save myself some work, I copied the right arm to the left once I'd created all of the GameObjects.
There will still be a good amount of work to input all of the coordinates.
I think this is necessary, even if I do end up making a converter, so that I understand it and I know it works.
When dragging the models in, these should replace the relevant "URDF link" tagged GameObjects.

I dragged in the torso, pedestal, head mount and screen.
The screen has some rotation which I need to work out.

I can't work out the rotation for the screen (`head_nod`), so I'll continue adding parts for now.
Is the origin of each model as described in the `urdf` the origin in the DAE file? Does Unity see this?
If so, undoing that in the parent GameObject will be good?
By comparing the `left_torso_arm_mount` and the `right_torso_arm_mount`, we can see which numbers are mirrored, and so which correspond to what?

I talked with a housemate who worked with Unity on placement.
When you drag a `.DAE` into Unity, it creates a new GameObject, which contains 3 children:
- Camera
- Lamp
- Mesh (with the same name as the GameObject)

He pointed out that the Camera and Lamp are not created by Unity, but exported from whatever was used to design the models (Blender/3DS Max).

Changing a GameObject's parent does not move the object in the scene. As a result, a transformation is applied to counteract the change the parent's transformation has on this GameObject.
I didn't realise this until I messed up the values everywhere. Correcting...

I got pretty confused with the combined rotations for the screen, so I'm going to try doing the arms.
Some of the transformations don't seem to make sense. The part can be perfectly in place but have one transformation still to do.
I think the rotations are being done in the wrong order.

I found somebody trying to do exactly what I am, [here](https://forum.unity3d.com/threads/problem-with-setting-game-object-rotation.426902/), but their problem is related to gimbal lock with eurler angles.

### Looking at an example
If I could get `Gazebo` to work in a virtual machine, I can see what baxter looks like at time=0 on one monitor, and have Unity on the other.
I had some problems getting it to run.
The problems were caused by VirtualBox Additions, for shared folders/clipboard, resolution fixes etc.
However I still can't get it to run. I think there's some incompatibility between ROS Kinetic and Indigo.
(The [website](http://sdk.rethinkrobotics.com/wiki/Baxter_Simulator) only gives support for up to Indigo).
I've also found that Indigo isn't supported on Ubuntu 16! Only [13 and 14](http://answers.ros.org/question/240718/installing-indigo-after-kinetic).

I created a new virtual machine, and I seemed to get close to it working, but it started quitting with no error feedback,
