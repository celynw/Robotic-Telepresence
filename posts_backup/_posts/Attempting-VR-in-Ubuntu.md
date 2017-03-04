---
title: Attempting VR in Ubuntu
date: 2016-10-19 11:42:15
tags:
---
To see whether this is plausible, I'll be trying to get it to work.
There seem to be three options.
#### VRUI
Since trying to get OpenVR to work, I have found some software called `Vrui`.
It is developed by Oliver Kreylos, an associate researcher for virtual reality at [UC Davis](http://www.ucdavis.edu/), goes under the name [doc-ok](http://doc-ok.org/?page_id=6).
It can be found [here](http://idav.ucdavis.edu/~okreylos/ResDev/Vrui/), and supposedly the next release will add Vive support
It is being actively updated.

#### OSVR
It's an open source virtual reality project, which has a hardware and a software branch.
At the time of writing, there is no Vive support released, but this may change in the near future.
There's already some Vive-related material on the official Github fork.

#### OpenVR in Xubuntu
Native support may be coming in the near future, particularly as SteamOS is Debian-based.

I will first try OpenVR by itself, without Unity.
It includes an application callled HelloVR.
Get [https://github.com/ChristophHaag/openvr](this fork) of OpenVR:
```zsh
$ git clone https://github.com/ChristophHaag/openvr.git
$ cd openvr
$ mkdir build
$ cd !$
$ cmake -DCMAKE_BUILD_TYPE=Release ../
$ make
```
Turns out I needed a couple of other packages:
```zsh
$ sudo apt-get install libsdl2-dev
$ sudo apt-get install libglew-dev
```
Now **plug in the Vive**.
Set up some permissions for the device:
```zsh
$ sudo chmod a+rw /dev/hidraw*
```
It seems I will have to install Steam and SteamVR after all. Let's see what happens.
At time of writing in May 2016, apparently the beta branch of SteamVR did not work.
Supposedly the lighthouse drivers are included and working though.
```zsh
$ sudo apt-get install steam
```
Once it is installed, clicking on the VR icon as you would to run VR in Windows doesn't start anything.
The following script should enable this to work:
```zsh vrcmd
#!/bin/zsh
export openvr=~/openvr
export steam=~/.steam
export steamvr=$steam/steam/steamapps/common/SteamVR

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:\
/usr/lib/:\
/usr/lib32/:\
$openvr/lib/linux32/:\
$openvr/lib/linux64/:\
$steam/ubuntu12_32/steam-runtime/i386/lib/i386-linux-gnu/:\
$steam/ubuntu12_32/steam-runtime/amd64/lib/x86_64-linux-gnu/:\
$steamvr/bin/linux32/:\
$steamvr/bin/linux64/:\
$steamvr/drivers/lighthouse/bin/linux32/:\
$steamvr/drivers/lighthouse/bin/linux64/
```
Now you run `vrcmd` from the command line.
I got stuck here. The headset wasn't found.

The [OpenHMD readme](https://github.com/OpenHMD/OpenHMD/blob/master/README.md) suggested this method to set permissions permanently:
```zsh
$ echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="0bb4", MODE="0666", GROUP="plugdev"' >> /etc/udev/rules.d/83-hmd.rules
$ echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="28de", MODE="0666", GROUP="plugdev"' >> /etc/udev/rules.d/83-hmd.rules
$ udevadm control --reload-rules
```
I tried doing this to fix the problem.
The file `/etc/udev/rules.d/83-hmd.rules` didn't exist yet, it was created.
I ran `dmesg` and these were related to plugging in the Vive:

```zsh
[23323.256741] hid-generic 0003:0BB4:2C87.0006: hiddev0,hidraw5: USB HID v1.11 Device [HTC HTC Vive] on usb-0000:08:00.0-2.1.5/input0
[23328.501417] hid-generic 0003:28DE:2101.0007: hiddev0,hidraw6: USB HID v1.11 Device [Valve Software Watchman Dongle] on usb-0000:08:00.0-2.1.6/input0
[23328.797700] hid-generic 0003:28DE:2101.0008: hiddev0,hidraw7: USB HID v1.11 Device [Valve Software Watchman Dongle] on usb-0000:08:00.0-2.1.7/input0
[23329.090160] hid-generic 0003:28DE:2000.0009: hiddev0,hidraw8: USB HID v1.01 Device [Valve Software Lighthouse FPGA RX] on usb-0000:08:00.0-2.1.1/input0
[23329.091253] hid-generic 0003:28DE:2000.000A: hiddev0,hidraw9: USB HID v1.01 Device [Valve Software Lighthouse FPGA RX] on usb-0000:08:00.0-2.1.1/input1
[23329.447498] hid-generic 0003:0D8C:0012.000B: input,hidraw10: USB HID v1.00 Device [C-Media Electronics Inc. USB Audio Device] on usb-0000:08:00.0-2.1.4/input3
[23329.647254] usb 3-2.1.2: new high-speed USB device number 11 using xhci_hcd
```

The vendor IDs, `0BB4` and `28DE` matched the rules entered earlier.
There is an extra one for the audio, `0D8C`. I'm not sure if this is related, I may have to add this as a rule somewhere later.

This potential fix didn't help.

Poking around I noticed that the `vrcmd` script wasn't running properly.
It turned out that it was using Windows line endings. I converted them to Unix line endings.
Still no help.
The only discernable message from running `vrcmd` is

```zsh
libdbus-1.so.3: no version information available
libgpg-error.so.0: no version information available
```
Installing these does not make any difference, I suspect there's a PATH issue maybe it's not too important.

I *did* notice that the HMD display has turned up in the Displays settings for Xubuntu. As an additional monitor.
![Xubuntu Display Settings](/Robotic-Telepresence/2016/10/19/Attempting-VR-in-Ubuntu/Displays Preferences.png)
Changing the display projection enabled the Vive to be used as a screen:
![Photo of screen through lenses. It's split over each eye](/Robotic-Telepresence/2016/10/19/Attempting-VR-in-Ubuntu/VR Screen.jpg)
After a reboot, `vrcmd` didn't give a million broken pipe errors, but it had the same effective result. Running it as root doesn't help.
I tried mashing the script into one command:
```zsh
$ export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib/:/usr/lib32/:~/openvr/lib/linux32/:~/openvr/lib/linux64/:~/.steam/ubuntu12_32/steam-runtime/i386/lib/i386-linux-gnu/:~/.steam/ubuntu12_32/steam-runtime/amd64/lib/x86_64-linux-gnu/:~/.steam/steam/steamapps/common/SteamVR/bin/linux32/:~/.steam/steam/steamapps/common/SteamVR/bin/linux64/:~/.steam/steam/steamapps/common/SteamVR/drivers/lighthouse/bin/linux32/:~/.steam/steam/steamapps/common/SteamVR/drivers/lighthouse/bin/linux64/
$ ./.steam/steam/steamapps/common/SteamVR/bin/linux64/vrserver
```
Now `vrserver` will run without error instead of not detecting a headset, but it still doesn't detect one. `vrcmd` still doesn't detect it either.
When the Vive is unplugged though, it only gives this message:
```zsh
VR_IsHmdPresent returned false to indicate that there are no attached HMDs.
```
This means it is detected in some way.

This took a long time, so I will give up for now.
