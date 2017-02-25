---
title: Rosbag communication
date: 2016-11-28 15:11:18
tags:
---
I'll be trying to get full rosbag communication into Unity, and communication in the other direction too.
Ideally this will be configurable inside Unity, I'm not sure how dynamic I can/need to make this.
The first step is to get virtual Baxter to move as the bag plays.

From the [documentation](http://wiki.ros.org/Names), ROS topic names can be composed of the following characters: `0-9`, `a-z`, `A-Z`, `_`, `/`.
So long as I use a different delimiter in the message formatting for my Unity interface this won't cause problems.
I'd like to keep this concise to minimize latency and parsing effort.
For now, I'm transmitting the names in the same order as they are in the bag (alphabetical) next to their values, separated with `=`.

I set up a post-build task to copy the resulting `.dll` to the `plugins/` folder in the Unity assets.
I'm still getting lots of 'sync lost' messages, one every second.
It may be something to do with `nh_spinOnce()`, but I should be calling it every Unity loop.
```zsh Command line
[ WARN] [1480349686.965718293]: Sync with device lost.
```

There doesn't seem to be a whole lot of error checking.
For instance, trying to read an `int` return value from a `.dll` function which has a void return type happily works.
If the socket connection cannot be made, it's easy to create a block which will not allow the Unity application to start, and it becomes unresponsive and has to be forced to quit.

The hard part is that the data from the `rosbag` is being received in a callback.
The way to get data out from the `.dll` and into Unity seems to be using return values from functions which are called in a Unity script.
I could make a set of variables which are updated with the most recent values.
This would be OK in the way that the model only has to be up to date and doesn't need to show every position transmitted, except I would need to know the variables I am expecting before compiling the `.dll`.
Alternatively, I could preallocate a large number of variables to fill in, and the Unity client would know what is coming and in what order because of the imported robot `URDF`.
For now I will hard-code it and maybe ask in the project meeting.

I can't pass strings out of the `.dll`. This seems to be misunderstanding `.dlls` or Unity.
I'm trying to think of a way to get the data out.
I could provide pointers to data from Unity into the `.dll` which match up with the subscription, but I still need a better callback.
At the end of [this tutorial](http://wiki.ros.org/rosserial_windows/Tutorials/Receiving%20Messages) it mentions:
> It's possible to subclass the Subscriber class for a more object-oriented approach. In that case, instead of registering with the call back above simply instantiate the new class and pass that in to the subscribe call.

This sounds more like what I want?
I'll try with `rosserial_hello_world` first.
I've replaced all the `printf` functions with a better C++ approach (`cout` streams).

I couldn't work out what the page meant.
I tried creating a dictionary of joints and angles in Unity and feeding that into the `.dll`, but I needed to 'use' `System.Collections.Generic` and I couldn't work out how to do that either.
I managed to pass a string as an argument into a `.dll` function, by casting `char*`:
```cs Unity script
...
[DllImport ("rosserial_unity")]
private static extern double getValue(string key);
...
```
```cpp dll code
...
DLLEXPORT double getValue(char* key) {
	if ((string)key == "head_pan")
...
```

Now I will go through the robot hierarchy in Unity, filter out the `GameObjects` which are tagged "URDF joints", sort them alphabetically and then request their values from the socket stream.
Unfortunately, the list of `GameObjects` is much longer (48) than those contained within the stream (18), and I can't differentiate.
I will either have to check one by one somehow or find out what makes the ones in the `rosbag` special.
I'll just hard code it for now...
This is difficult too.
There is no pattern to the messages with 18 joints, or the ones which just contain a right or left gripper.

It's pretty annoying having to restart Unity every time I've updated the `.dll`.

#### Some success
I resorted to the most god-awful code, but I've finally got joint angles into Unity.
This will DEFINITELY need changing, but I can progress with making sure that I have the update rate etc.

I've managed to get _some_ form of the ros bag onto virtual Baxter.
The 'sync lost' messages have stopped, I'm not sure why, because the update loop isn't going much faster or slower than before.
I'll have to set additional attributes to each link during the import to record which axis rotates, and the effort limits.

I've made a script with private variables for just axis to start with, and modified the import script to attach it to every joint.
I'll also have to do the same to the origin values to keep a reference to them.
I also attach the `rosserial` script to the root by default.

After some playing around with rotations, I got it to work:
{% youtube 0TW5gbRr-oQ %}

The jerkiness can be down to many things:
- Only the most recent message is stored
- If that message was a gripper, the rest of the values are empty
- The messages are only read every Unity frame at the moment, probably 60FPS
