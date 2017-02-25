---
title: Sorting out rotation
date: 2016-10-30 12:55:44
tags:
---
I naïvely thought that if I rotate the object so that regardless of _which_ axis is which, that all three from the `urdf` line up with those in Unity, I could then just swap them out with each other and everything would work.  
However, this doesn't seem to work. Maybe it's something to do with Unity doing rotation in Z, then X, then Y according to the [docs](https://docs.unity3d.com/ScriptReference/Transform-eulerAngles.html), whereas ROS does X, Y, Z.

This was a headache until I realised that instead of using the `transform.localEulerAngles`, I can use `transform.Rotate`.
Rather than setting the rotation it can apply one in a single axis.
Changing the order of rotations in a script _does_ change the result, as one might expect.
However, there's no combination of rotations with 100 degrees and 90 degrees which would place the screen in the correct location!

Much later - I've worked it out!
I've been presuming that every link definition has an origin rotation of zero.
This is true for all cases _except_ the screen (which is a child of head_nod).
I think this is unique because this has display as a child which will need to be the correct way up in the simulator. D'oh!
I still haven't worked out the correct rotation combinations for the script, another time.
