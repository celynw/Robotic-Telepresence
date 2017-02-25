---
title: PCL point clouds
date: 2017-01-03 21:06:59
tags:
---
Now I have a list of 200kB files (`xaa`, `xab`, `xac` etc.), split from the first PointCloud2 message in the `rosbag`.
Viewing the ascii `.pcd` file, I found what looked like the start of meaningful data, at point 29515.
Doing some arithmetic led me to the 17th file, or `xaq`.
I used this to confirm my program wasn't working.

### BUG
I've just realised that I wasn't discarding data properly!
The offsets of 4 bytes of 0, 1, 2 and 4 are required, but I wasn't discarding 3, 5, 6 or 7!
Along with making sure the commas were removed: Success!!
![Using a custom segment of data which won't return NaNs](/Robotic-Telepresence/2017/01/03/PCL-point-clouds/Conversion Success.png)

I needed to adapt the program to skip the header parts of the message to only read the data.
I ran it on a full message and it took a couple of minutes to finish printing, but it worked.
Removing the printing functionality sped it up to about 20 seconds.
**This needs to be faster!**
I can optimise the code a bit more, and it's only running on a single core.
I can also get Unity to read from RAM rather than wait for stdout redirection.

### Into Unity
I can call the program in Unity, however it does not close automatically.
I realised that this is caused by redirecting standard output into Unity.
By completely reading everything in the buffer, it is fixed.

I got it to display the first 65000 points of a whole message.
![It's white because there's a problem with the rendering I haven't realised yet.](/Robotic-Telepresence/2017/01/03/PCL-point-clouds/First 65000 White.png)
65000 is the maximum number of vertices in a mesh in Unity.
I'll have to split the cloud into submeshes.

### More hindsight
I've made another mistake. After a long time of trial and error, I've found that while you can split a mesh into submeshes, the overall limit is still 65000 points because it's actually still one mesh.
The purpose of submeshes is for different materials.
I'll have to make more GameObjects with their own meshes.

### Full mesh
A partial success, but the colours still don't work and the rotation is wrong.
![The cloud is structured in strips, from Baxter's back to front](/Robotic-Telepresence/2017/01/03/PCL-point-clouds/Full Selection.png)
To fix the colours, I've realised that the colours are supposed to be between 0 and 1, not 0 and 255.
To fix the rotation, I need to find out the base transformation.

### Colours
But the colours seem wrong.
Perhaps this is to do with endian-ness I didn't account for.
![It's not just my eyes? I think?](/Robotic-Telepresence/2017/01/03/PCL-point-clouds/Colours Wrong.png)
Yep! That was it. Now for the transformation.
![Phew](/Robotic-Telepresence/2017/01/03/PCL-point-clouds/Colours Fixed.png)

### Transformation
This was quite difficult to find.
The Googlable phrases were all quite similar, and most people were trying to _set_ the transform, not _get_ it.
I started with `RViz` again. I remember having to set the base transform for viewing the octomap.
This didn't exactly help, although I saw some error messages which said:
```none In RViz
	Failed to transform from frame [camera_rgb_optical_frame] to frame [base]
```
This gave me a bit more to go on. From the `PointCloud2` message itself, I knew it used the `camera_rgb_optical_frame`.
This eventually led me to `tf_echo`:
```zsh Command line
$ rosrun tf tf_echo camera_rgb_optical_frame base
At time 1479833365.881
- Translation: [-0.024, -0.546, 1.873]
- Rotation: in Quaternion [0.708, 0.675, -0.154, 0.145]
            in RPY (radian) [-3.139, 0.425, 1.524]
            in RPY (degree) [-179.836, 24.340, 87.300]

$ rosrun tf tf_echo base camera_rgb_optical_frame
At time 1479833366.181
- Translation: [1.270, 0.003, 1.481]
- Rotation: in Quaternion [0.708, 0.675, -0.154, -0.145]
            in RPY (radian) [-2.717, 0.022, 1.528]
            in RPY (degree) [-155.691, 1.276, 87.539]
```
I 100% wasn't sure which way around it was, so I tried both.
Using my knowledge from the [URDF importer](/Robotic-Telepresence/2016/11/14/URDF-importer/), I made a Unity script to convert these into the native frame.
By negating the X translation, rotating by -90 degrees in X, then rotating by -Z, then -Y, then +X with the values for the translation from `base` to `camera_rgb_optical_frame`, it worked:
![It still seems off, probably not my fault.](/Robotic-Telepresence/2017/01/03/PCL-point-clouds/Complete.png)
Thankfully, the point clouds are dense enough for now to not worry about them being represented by single pixels.
There seem to be no performance issues whatsoever.

### Still lots to do
I still have to work out how to make this not take 20 seconds each frame, while still allowing other processes to run.
Not to mention bringing everything together...
