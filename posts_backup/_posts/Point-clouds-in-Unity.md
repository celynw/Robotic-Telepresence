---
title: Point clouds in Unity
date: 2016-12-05 23:41:03
tags:
---
Giving the `NamedSharedMemory` a bit of a break while I focus on point clouds.
If I can't get `NamedSharedMemory` to work, I've always got the slower less efficient stdout redirection into Unity.

#### Method
The example point clouds which I was provided with were very large.
> 
| File                                          | Size    | Vertices   |
| --------------------------------------------- | ------- | ---------- |
| Kinect_Aligned_ColourFromOscar_Subsampled.ply | 6.7MB   | 419,033    |
| Oscar_Monocular_Reconstruction_Cloud.ply      | 17.9MB  | 396,257    |
| Kinect_Aligned.ply                            | 449.7MB | 37,478,946 |
| Kinect_Aligned_ColourFromOscar.ply            | 599.7MB | 37,478,946 |

![The smallest point cloud, which has been edited by hand](/Robotic-Telepresence/2016/12/05/Point-Clouds-in-Unity/Subsampled.png)
The best way to draw them would be to take advantage of the GPU.
By using a vertex shader, we can make each point in the cloud a vertex.
I grabbed some code from [here](http://www.kamend.com/2014/05/rendering-a-point-cloud-inside-unity/) to adapt and try out.
Unity only allows 65,534 vertices for each mesh, at which it is split into multiple meshes.
My computer, with the recommended GPU for entry-level VR had no trouble at all rendering 60,000 points in a cube.
![](/Robotic-Telepresence/2016/12/05/Point-Clouds-in-Unity/60k Cube.png)
This is ~500 times fewer points than in the largest `.ply` file, but I don't know how large it will _need_ to be.
I couldn't work out how to vary the point size. The functionality should be enabled by a script attached to the camera which specifies:
```cs PointCLoudConfig.cs
const UInt32 GL_VERTEX_PROGRAM_POINT_SIZE = 0x8642;
glEnable(GL_VERTEX_PROGRAM_POINT_SIZE);
```

#### `rosbag` Point Cloud
In order to find how many points I should expect, I ran the `rosbag` step by step until a message containing the output was printed, which I redirected to a file.
The file caused my text editor to not be very responsive. I found the commas were only present in the list of points, so I counted them without opening the file.
```zsh Command line
$ rosbag play ~/Downloads/baxter.bag --pause -l
$ rostopic echo /camera/depth_registered/points | tee ~/PointCloud.txt
$ fgrep -o "," ~/PointCloud.txt | wc -l
9830399
```
This gave a file of 36.5MB, with 9,830,400 points, which would split into 151 meshes.

I wrote a little script which would create `X` number of child `GameObjects`, each drawing 60,000 points in the new shader, offset by 10 units.
```cs CreatePointCloudObject.cs
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CreatePointCloudObjects : MonoBehaviour {
	public int num_clouds = 0;
	void Start() {
		Material mat = Resources.Load("PointCloud", typeof(Material)) as Material;
		for (int i=0; i<num_clouds; i++) {
			GameObject gameObject = new GameObject("PointCloud fragment");
			gameObject.transform.SetParent(this.transform);
			gameObject.AddComponent<PointCloud>(); //MeshRenderer and MeshFilter already added by script
			gameObject.GetComponent<MeshRenderer>().material = mat;
			gameObject.transform.position = new Vector3((float)i*10, 0.0f, 0.0f);
		}
	}
}
```
When increasing `X` to 164, it also seemed to run without issue.
![Drawing 9,840,000 vertices with random vertex colours, at 1px per vertex](/Robotic-Telepresence/2016/12/05/Point-Clouds-in-Unity/All Points.png)

#### Changing point size
I realised that whist the `GL_VERTEX_PROGRAM_POINT_SIZE` was being enabled, it wasn't being enabled because I was running in DirectX11.
By going to `Edit > Project Settings > Player > Other Settings` and unchecking `Auto Graphics API for Windows`, I was given the option to enable OpenGLCore and move it to the top of the list.
Now the property was being enabled (only when viewing through the scene camera, not the editor) but I wasn't seeing any changes.
I still can't get it to work.
[This guy](http://1darray.com/blog/2012/09/01/xyz-point-cloud-data-viewer-dx11/) seems to have a very capable asset package for point clouds, but he won't give any information away because he's selling his work.
