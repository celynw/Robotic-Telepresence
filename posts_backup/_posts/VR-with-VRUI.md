---
title: VR with VRUI
date: 2016-11-02 13:33:20
tags:
---
A few things have surfaced this week, primarily, Vive support for `vrui` in Linux.
I downloaded the source:
```zsh
$ wget http://idav.ucdavis.edu/~okreylos/ResDev/Vrui/Build-Ubuntu.sh
$ sha1sum Build-Ubuntu.sh
$ ./Build-Ubuntu.sh
$ cd ~/src/Vrui-4.2-004
```
There's a part which enables device permissions for "non-standard devices".
```zsh
$ sudo make installudevrules
```
I wondered if this would fix the issues I was having with SteamVR, [here](/Robotic-Telepresence/2016/10/19/Attempting-VR-in-Ubuntu/).
No luck.

Time to try `Vrui` with the Vive.
I had to go to the Vrui release [blog post](http://doc-ok.org/?p=1508) in order to find full specific install instructions:
```zsh
$ cd
$ wget http://idav.ucdavis.edu/~okreylos/ResDev/Vrui/Build-Ubuntu.sh
$ bash Build-Ubuntu.sh
$ cd ~/src/Vrui-4.2-004
$ make STEAMVRDIR=/home/celynwalters/.steam/steam/steamapps/common/SteamVR/
$ sudo make STEAMVRDIR=/home/celynwalters/.steam/steam/steamapps/common/SteamVR/ install
```
However, the tracking driver still fails:
```zsh
$ RunViveTracker.sh
VRDeviceDaemon: Reading configuration file
/usr/local/bin/VRDeviceDaemon: relocation error: /usr/local/lib/x86_64-linux-gnu/Vrui-4.2/libMisc.g++-3.so.4: symbol _ZNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEE10_M_replaceEmmPKcm, version GLIBCXX_3.4.21 not defined in file libstdc++.so.6 with link time reference
```
I found a [reddit post](https://www.reddit.com/r/Vive/comments/580ep4/vive_is_working_on_linux_steamvr_still_isnt_though/d8y9ent/) communicating with the tool author, `Doc_Ok`, who had my issue.
`Doc_Ok` replied with a fix:

```zsh
$ ldd /usr/local/bin/VRDeviceDaemon
	linux-vdso.so.1 =>  (0x00007fff0fae4000)
	libdl.so.2 => /lib/x86_64-linux-gnu/libdl.so.2 (0x00007fa3799ce000)
	libComm.g++-3.so.4 => /usr/local/lib/x86_64-linux-gnu/Vrui-4.2/libComm.g++-3.so.4 (0x00007fa3797b9000)
	libGeometry.g++-3.so.4 => /usr/local/lib/x86_64-linux-gnu/Vrui-4.2/libGeometry.g++-3.so.4 (0x00007fa379477000)
	libIO.g++-3.so.4 => /usr/local/lib/x86_64-linux-gnu/Vrui-4.2/libIO.g++-3.so.4 (0x00007fa37924b000)
	libMisc.g++-3.so.4 => /usr/local/lib/x86_64-linux-gnu/Vrui-4.2/libMisc.g++-3.so.4 (0x00007fa37902c000)
	libThreads.g++-3.so.4 => /usr/local/lib/x86_64-linux-gnu/Vrui-4.2/libThreads.g++-3.so.4 (0x00007fa378e26000)
	libpthread.so.0 => /lib/x86_64-linux-gnu/libpthread.so.0 (0x00007fa378c09000)
	libstdc++.so.6 => /usr/lib/x86_64-linux-gnu/libstdc++.so.6 (0x00007fa378887000)
	libm.so.6 => /lib/x86_64-linux-gnu/libm.so.6 (0x00007fa37857d000)
	libgcc_s.so.1 => /lib/x86_64-linux-gnu/libgcc_s.so.1 (0x00007fa378367000)
	libc.so.6 => /lib/x86_64-linux-gnu/libc.so.6 (0x00007fa377f9e000)
	/lib64/ld-linux-x86-64.so.2 (0x000055761dc87000)
	libMath.g++-3.so.4 => /usr/local/lib/x86_64-linux-gnu/Vrui-4.2/libMath.g++-3.so.4 (0x00007fa377d86000)
	libz.so.1 => /lib/x86_64-linux-gnu/libz.so.1 (0x00007fa377b6c000)
```
The script was using Steam's libstdc++, which I've redirected to use the built in ones.
```zsh
$ sudo nano $(which RunViveTracker.sh)
```
Replace the line:
```
RUNTIMEDIR2=$STEAMDIR/ubuntu12_32/steam-runtime/amd64/usr/lib/x86_64-linux-gnu
```
with
```zsh
RUNTIMEDIR2=/usr/lib/
```
### Sucess!
```zsh
VRDeviceDaemon: Reading configuration file
VRDeviceDaemon: Initializing device manager
VRDeviceManager: Loading device Vive of type OpenVRHost
GetBool for driver_lighthouse/dbhistory
GetString for driver_lighthouse/usedisambiguation
GetInt32 for driver_lighthouse/disambiguationdebug
GetBool for driver_lighthouse/disableimu
GetInt32 for driver_lighthouse/primarybasestation
GetBool for driver_lighthouse/trackedCamera
OpenVRHost: Activating 1 tracked devices
OpenVRHost: Activating tracked device 0 with ID 0
GetBool for driver_lighthouse/fakeHtcHmdMainboard
OpenVRHost: Updating HMD configuration... done
OpenVRHost: Battery level of device 0 is 100%
VRDeviceManager: Managing 3 trackers, 10 buttons, 6 valuators
VRDeviceManager: Managing 3 virtual devices
VRDeviceDaemon: Initializing device server
VRDeviceServer: Listening for incoming connections on TCP port 8555
ioctl (GFEATURE): Broken pipe
...
ioctl (SFEATURE): Broken pipe
OpenVRHost: Updating HMD configuration... done
OpenVRHost: Battery level of device 0 is 100%
VRDeviceServer: Sending updated HMD configuration 0 to clients... done
OpenVRHost: Updating HMD configuration... done
OpenVRHost: Physical IPD on device 0 set to 63.7mm
OpenVRHost: Updating HMD configuration... done
OpenVRHost: Physical IPD on device 0 set to 63.7mm
VRDeviceServer: Sending updated HMD configuration 0 to clients... done
OpenVRHost: Adding device with serial number LHB-3CF79497
OpenVRHost: Error retrieving connected device 1
OpenVRHost: Adding device with serial number LHB-5CEC2D20
OpenVRHost: Error retrieving connected device 1
```
While this is running, we can run a position test in another shell:
```zsh
$ DeviceTest localhost:8555 -t 0 -p
```
Giving us a real-time position:
```zsh
     Pos X     Pos Y     Pos Z
(   -0.853    -0.863    -1.730)
```
And more output in the original shell:
```zsh
Creating new client state... done
VRDeviceServer: Connecting new client localhost:53204
Adding new client state to list
Adding listener for client's socket
Client connected
Reading message... done, 0
Reading protocol version... done, 4
Sending connect reply... done
Reading message... done, 3
VRDeviceManager: Starting devices
Reading message... done, 7
Sending packet reply... done
VRDeviceServer: Disconnecting client localhost:53204 due to exception "Client terminated connection"
VRDeviceManager: Stopping devices
Creating new client state... done
VRDeviceServer: Connecting new client localhost:53206
Adding new client state to list
Adding listener for client's socket
Client connected
Reading message... done, 0
Reading protocol version... done, 4
Sending connect reply... done
Reading message... done, 3
VRDeviceManager: Starting devices
Reading message... done, 7
Sending packet reply... done
OpenVRHost: Updating HMD configuration... done
OpenVRHost: Battery level of device 0 is 100%
OpenVRHost: Device 0 has been connected
OpenVRHost: World transform for device 0: {(0, 0, 0), {(-0.995354, 0.0181601, 0.0945509), 0.381241}}
OpenVRHost: Local transform for device 0: {(-0.00849516, -0.00496823, -0.0715013), {(-1, -0, -0), 3.14159}}
OpenVRHost: Updating HMD configuration... done
OpenVRHost: Battery level of device 0 is 100%
OpenVRHost: Proximity sensor on device 0 triggered
OpenVRHost: Proximity sensor on device 0 untriggered
OpenVRHost: Proximity sensor on device 0 triggered
VRDeviceServer: Disconnecting client localhost:53206 due to exception "Client terminated connection"
VRDeviceManager: Stopping devices
```
We have tracking!
We can also try it out on a controller:
```zsh
$ DeviceTest localhost:8555 -t 1 -p -b
```
Giving the position of one, and the button states for both:
```
     Pos X     Pos Y     Pos Z
(   -0.765    -1.238    -1.572) . . . . . X . . . X
```
There's even a room setup utility. Somehow it already knows my room layout so I skipped this step.
I went into `nvidia-settings` to turn on the Vive display. It acts as a desktop again.
My Vive had the display connector name `DPF-1`.
Set the environment variable, build and run the program:
```zsh
$ export __GL_SYNC_DISPLAY_DEVICE=DFP-1
$ cd ~/src/Vrui-4.2-004/ExamplePrograms
$ make INSTALLDIR=/usr/local
$ sudo make INSTALLDIR=/usr/local install
$ ClusterJello -rootSection Vive
```
It worked! I was put into a virtual environment
The VR lags a bit, which will cause motion sickness.
I'm not sure what the issue is which is causing it.
It could be a graphics driver incompatibility.

![I couldn't take a screenshot, but this the demo application I used, 'ClusterJello'. Taken from Oliver Kreylos' <a href=https://www.youtube.com/watch?v=OJlzmKxDd40>video</a>](/Robotic-Telepresence/2016/11/02/VR-with-VRUI/Cluster Jello.png)
