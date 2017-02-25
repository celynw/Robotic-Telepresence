---
title: Named pipe dll
date: 2016-12-01 20:56:37
tags:
---
I had to enable CLR support for the `System` namespaces.
> Project > rosserial_unity Properties > Configuration Properties > General > Project Defaults > Common Language Runtime Support > enable /clr

I didn't know that Unity keeps a track of all global classes in the assets (seems obvious now). I moved the StreamString class definition to a separate script.
I moved the pipe server code into a DLL. It compiled and could run, but caused Unity to hang.
I moved the while loop out of the `Main()` function and put it into a separate function which is to be called by Unity's `Update()`.
This meant I had to make `PipeServer::server` global, which caused `server == null` to happen in some places.

It took a while! I ended up with this structure:
- A `.dll` to open a named pipe called "rospipe". When the pipe is connected to, it would send "Hello Unity", and then close it.
- A `GameObject` to call the `.dll` and open the pipe. Once it's confirmed to be open, it enables the next `GameObject`:
- A `GameObject` which connects to the pipe "rospipe" if it exists, and listens for messages.

![Proof. The string exists only in the .dll, and no other programs were running](/Robotic-Telepresence/2016/12/01/Named-pipe-dll/Debug Log.png)






Maybe I will have to use `WaitNamedPipe` ([MSDN](https://msdn.microsoft.com/en-us/library/windows/desktop/aa365800.aspx))? When trying to debug, I got an error in Visual Studio:
> Exception thrown: 'System.IO.IOException' in System.Core.dll
An unhandled exception of type 'System.IO.IOException' occurred in System.Core.dll
Additional information: All pipe instances are busy.

I also added a `close_pipe` function, now that we don't want to necessarily close straight after initilasing. It should trigger when I stop the game using the `OnApplicationQuit()` callback.
It doesn't seem to work, though.
Every second time I try to run the game, Unity crashes.
I'm using `PipeServer.Close()` which runs, but `PipeServer.Disconnect()` and `PipeDispose.()` crashed things immediately.
After a long time, I realised it was caused by the `.dll` having `close_pipe()` as a `void` return, but Unity had it as a `bool` return. D'Oh!
Now I can run the code a few times before crashing. Wow!

I've set the PipeDirection in the `.dll` to 'Out' rather than 2-way.
It took me too long to realise I had to change the other end to 'In'...

# I've made a terrible mistake
As educational this was, I'm trying to write values which overwrite the old ones, regardless of whether they were read or not.
The data in named pipes MUST be read on the receiving end in the same order, FIFO.
I have, however, gained a much better understanding of how all of this works, and some new keywords for Googling.
[Here](https://msdn.microsoft.com/en-us/library/aa365574%28v=vs.85%29.aspx) is a list of IPC (inter-process communication) methods provided by Windows.
I'll choose one of these.
> 
| Axis             | Info          |
| ---------------- | ------------- |
| ~~Clipboard~~    | The user might want to use this/overwrite the data |
| ~~COM~~          | This looks like it is built for other purposes, it uses ActiveX Object Services |
| Data Copy        |               |
| ~~DDE~~          | "Not as efficient as newer technologies". We need this to be as fast as possible |
| **File Mapping** | This sounds good. Programs treat the memory as their own. You can even do _named_ file mapping. Many other IPC methods are based on this so generally have poorer performance |
| ~~Mailslots~~    | Messages are short (400 bytes) and are FIFO |
| ~~Pipes~~        | As mentioned, is strictly FIFO |
| RPC              |               |
| Windows Sockets  | Intended for network communications, can be TCP or UDP. Not so high performance |
