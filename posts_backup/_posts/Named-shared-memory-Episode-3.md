---
title: 'Named shared memory: Episode 3'
date: 2017-01-18 14:40:07
tags:
---
#### Inter-process communication
Following the success of [Named Shared Memory: Episode 2](/Robotic-Telepresence/2016/12/03/Named-Shared-Memory-Episode-2/), it wasn't too far to be able to see the message generated in one process in the Unity debug log.
As long as it's known what data is expected at which offsets it can be reconstructed.
The only function data types available are:
> 
- Byte
- Int16
- Int32
- Int64
- IntPtr

Luckily these can be cast to whatever you like.

![Process writing the message can be started before or after Unity starts](/Robotic-Telepresence/2017/01/18/Named-Shared-Memory-Episode-3/Message.png)

I've had trouble casting to floats from Int32s.
I've come across 'BitConverter.ToSingle' which does the same task as the byte/float union in my console application.
Perhaps I could use a `.dll` after all.

#### Project structure
I've decided to make the console application the smallest middle-man possible, so it will only transfer the data to the shared memory.
Any multithreading will be solely handled by Unity.
I've managed to get this working, but it has not reduced the time yet (still at ~19 seconds for a single frame).
I think this is because earlier it was reading from a stream, writing to another stream and having that stream being read which could pipeline.
Now it's only doint the first read from a stream.
I will see what it's like by combining the `NamedSharedMemory` project and the `rosserial` projects, cutting out any streams.
