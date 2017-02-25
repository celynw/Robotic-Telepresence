---
title: Data to Unity from RAM
date: 2016-12-01 17:18:29
tags:
---
I started by copying code from the "rosserial_hello_world" and "rosserial_unity" (`.dll` code) solutions into a new solution.

I'm looking up how to create shared memory in Windows for sharing the data.
You can't feasibly do this in a `.dll` because the space would have to be initialised statically.
The way to do this seems to be using [Named Shared Memory](https://msdn.microsoft.com/en-us/library/windows/desktop/aa366551.aspx).
Unfortunately, it seems that I might still have to worry about marshal functions or 'unsafe' code because of the C# in Unity.
An alternative could be [Named Pipes](https://msdn.microsoft.com/en-us/library/windows/desktop/aa365590.aspx).

###### Named pipes
I made _another_ visual studio solution to test this, and created an empty `GameObject` in Unity with a new script for the other end.
In order for some classes to be valid such as `System.IO.Pipes`, I changed the project settings to handle the full .Net 2.0:
> Edit > Project Settings > Player > Other Settings > Optimization > Api Compitbility Level > ".NET 2.0"
(rather than ".NET 2.0 Subset")

For the Unity side, I found some recent example code on the [Unity3D forums](http://answers.unity3d.com/questions/483123/how-do-i-get-named-pipes-to-work-in-unity.html#answer-1272634).
It pointed me to the ["How to use names pipes for network interprocess communication" MSDN page](https://msdn.microsoft.com/en-us/library/bb546085%28v=vs.110%29.aspx).
I copied the C++ code from the example just to see if it would compile, or work in any way.
I made sure that the `NamedPipeClientStream` for each end had the same name ("testpipe" for now).
At first, I was successful, except Unity got stuck in the starting state, and became unresponsive and needed to be forced quit.
![The 'PipeDataSend' Windows console application, after running Unity](/Robotic-Telepresence/2016/12/01/Standalone-rosserial-application/Connection.png)
With a bit of tweaking in the Unity script, I was able to get the expected message into Unity.
I had to cut out a `getPipedData()` function, which sounds like it's defeating the point.
However, the functionality is repeated exactly in the `Setup()` of the Unity script, which works.
![The "I am the one true server" string was defined in the 'PipeDataSend' Windows console application](/Robotic-Telepresence/2016/12/01/Standalone-rosserial-application/Success.png)
```cs The part of PipeDataReceive.cs which was commented out
private void getPipedData() {
	//UnityEngine.Debug.Log("Thread Called - Start");
	StreamString ss = new StreamString(pipeClient);
	UnityEngine.Debug.Log(ss.ReadString());
	//UnityEngine.Debug.Log("Thread Called - End");
}
```
I noticed afterwards that after Unity gets stuck, closing the console applications un-freezes Unity which returns with:
> OverflowException: Number overflow.
StreamString.ReadString() (at Assets/_Scripts/PipeDataReceive.cs:51)
PipeDataReceive.getPipedData() (at Assets/_Scripts/PipeDataReceive.cs:33)
PipeDataReceive.Update() (at Assets/_Scripts/PipeDataReceive.cs:27)

I lowered the threads from 4 to 1.
I had missed the part about reading from and writing to a file for example purposes, so I stripped it out.
The `PipeDataSend` and `PipeDataReceive` files are about as short and compact as I can make them now.
They work well. The `PipeDataSend` waits for a connection. As soon as `PipeDataReceive` is started, `PipeDataSend` send over "Hello Unity!" and then finishes.
However, you must not stop the Unity game before the pipe server or it will crash...

Next step: See how this performs after integrating with `rosserial_windows` in Unity.
```cpp PipeDataSend.cpp
#using <System.Core.dll>

using namespace System;
using namespace System::IO;
using namespace System::IO::Pipes;
using namespace System::Text;
using namespace System::Threading;

public ref class StreamString {
private:
	Stream^ ioStream;
	UnicodeEncoding^ streamEncoding;
public:
	StreamString(Stream^ ioStream) {
		this->ioStream = ioStream;
		streamEncoding = gcnew UnicodeEncoding();
	}

	String^ ReadString() {
		int len;

		len = ioStream->ReadByte() * 256;
		len += ioStream->ReadByte();
		array<Byte>^ inBuffer = gcnew array<Byte>(len);
		ioStream->Read(inBuffer, 0, len);

		return streamEncoding->GetString(inBuffer);
	}

	int WriteString(String^ outString) {
		array<Byte>^ outBuffer = streamEncoding->GetBytes(outString);
		int len = outBuffer->Length;
		if (len > UInt16::MaxValue)
			len = (int)UInt16::MaxValue;
		ioStream->WriteByte((Byte)(len / 256));
		ioStream->WriteByte((Byte)(len & 255));
		ioStream->Write(outBuffer, 0, len);
		ioStream->Flush();

		return outBuffer->Length + 2;
	}
};

public ref class PipeServer {
public:
	static void Main() {
		Console::WriteLine("Pipe server started");
		Console::WriteLine("Waiting for client connection...");
		Thread^ server = gcnew Thread(gcnew ThreadStart(&ServerThread));
		server->Start();
		Thread::Sleep(250);
		while (true) {
			if (server != nullptr) {
				// Check if it has gone. If so, remove it.
				if (server->Join(250)) {
					Console::WriteLine("Server thread finished.", server->ManagedThreadId);
					server = nullptr;
					break;
				}
			}
		}
		Console::WriteLine("Finished.");
	}
private:
	static void ServerThread() {
		NamedPipeServerStream^ pipeServer = gcnew NamedPipeServerStream("rospipe", PipeDirection::InOut);

		// Wait for a client to connect
		pipeServer->WaitForConnection();

		Console::WriteLine("Client connected");
		// Do my data transfer here!
		try {
			StreamString^ ss = gcnew StreamString(pipeServer);

			ss->WriteString("Hello Unity!");
		}
		catch (IOException^ e) {
			Console::WriteLine("ERROR: {0}", e->Message);
		}
		pipeServer->Close();
	}
};

int main() {
	PipeServer::Main();
	Console::WriteLine("\nPlease close this window.");
	while (1) {}
}
```
```cs PipeDataReceive.cs
using UnityEngine;
using System.Collections;
using System.IO;
using System.IO.Pipes;
using System.Threading;
using System.Text;
using System;
using System.Security.Principal;
using System.Diagnostics;

public class PipeDataReceive : MonoBehaviour {
	public NamedPipeClientStream pipeClient = new NamedPipeClientStream(".", "rospipe", PipeDirection.InOut, PipeOptions.Asynchronous);
	public StreamString ss;
	Thread readThread;

	void Start() {
		UnityEngine.Debug.Log("Connecting to pipe server...");
		pipeClient.Connect();
		Thread.Sleep(250);
	}

	void Update() {
		getPipedData();
	}

	private void getPipedData() {
		StreamString ss = new StreamString(pipeClient);
		UnityEngine.Debug.Log("piped: "+ss.ReadString());
	}
}

public class StreamString {
	private Stream ioStream;
	private UnicodeEncoding streamEncoding;

	public StreamString(Stream ioStream) {
		this.ioStream = ioStream;
		streamEncoding = new UnicodeEncoding();
	}

	public string ReadString() {
		int len;
		len = ioStream.ReadByte() * 256;
		len += ioStream.ReadByte();
		byte[] inBuffer = new byte[len];
		ioStream.Read(inBuffer, 0, len);
		string outString = streamEncoding.GetString(inBuffer);

		return outString;
	}

	public int WriteString(string outString) {
		byte[] outBuffer = streamEncoding.GetBytes(outString);
		int len = outBuffer.Length;
		if (len > UInt16.MaxValue)
			len = (int)UInt16.MaxValue;
		ioStream.WriteByte((byte)(len / 256));
		ioStream.WriteByte((byte)(len & 255));
		ioStream.Write(outBuffer, 0, len);
		ioStream.Flush();

		return outBuffer.Length + 2;
	}
}
```
