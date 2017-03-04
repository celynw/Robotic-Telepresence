---
title: 'Named shared memory: Episode 2'
date: 2016-12-03 01:07:37
tags:
---
I did a lot of looking around, and I could find very occasional signs that some people had been able to get named shared memory working with Unity.
Eventually I found [this forum question](http://answers.unity3d.com/questions/1249732/how-to-use-shared-memory-in-unity.html) which showed some example code for Unity C# and other C++.
The C++ code I could tell had been adapted from the [MSDN page](https://msdn.microsoft.com/en-us/library/aa366551.aspx) I got mine from, which had been adapted.
I adapted it some more to fit the other end and made it into a Unity Script.

After learning I needed to use unsafe operations, I found out that to enable them you had to add a file in yout assets called `smcs.rsp`.
This turned out to be different if you enabled the full .NET 2.0 API compatibility level, being `gmcs.rsp`.
This also turned out to be depreceated, so I used `mcs.rsp` instead.
It should contain the line:
```none mcs.rsp
-unsafe
```

I couldn't get the ends to connect. I tried removing the `Global\\` from the file mapping name. Now administrator access is no longer required.
In the console, It showed signs of life at the other end:
![Although I haven't even checked what data transfer is supposed to be happening](/Robotic-Telepresence/2016/12/03/Named-shared-memory-Episode-2/Life.png)

I've somehow broken my Visual Studio projects which use the ROS library.
Including `<windows.h>` somehow conflicts with `"ros.h"`.
I _need_ `windows.h` for `CreateFileMapping()`.
Fixed!
The solution was: `windows.h` was #defining `ERROR`, which was being used in `ros/Log.h`.
Adding a `#undef ERROR` at the start made it build.

#### Putting ROS messages into shared memory
The example uses `CopyMemory()` to put a string into shared memory.
I'll need to know the size of the message.
Looking at `JointState.h` in the ROS library in Windows, I was able to see how the joint messages were built and how to determine their sizes
