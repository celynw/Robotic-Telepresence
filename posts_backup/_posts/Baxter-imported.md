---
title: Baxter imported
date: 2016-11-14 23:39:32
tags:
---
To import the meshes, I first tried to add new mesh filter and mesh renderer components to the created GameObjects.
Unfortunately, this doesn't add the materials like it does when you drag them in, and I can't seem to find an easy way to do add multiple, let alone choose the correct ones.
If a mesh had too many trianlges and was split into two, you also can't attach both of them to the mesh filter, or attach two mesh filters to a `GameObject`.

I managed to find out how to import them in a nice(ish) way:

```cs
string path = meshNode.Attributes["filename"].Value;
System.Uri uri = new System.Uri(path);
path = uri.Host + uri.PathAndQuery; //Removes "package://"
obj = GameObject.Instantiate(AssetDatabase.LoadAssetAtPath("Assets/"+path, typeof(GameObject))) as GameObject;
```

This keeps the extra GameObjects such as Camera, Light and the Animation component.
The meshes are also children of each 'URDF_link' `GameObject` rather that acting as them.
You cannot easily copy the components (mesh filter, mesh renderer) to another object.
Even copying the only values would be a hassle.
The easiest was swapping the `GameObjects` around
The transformations and modifications would be applied afterwards.

![Baxter at 'position zero'](/Robotic-Telepresence/2016/11/14/Baxter-imported/Baxter.png)

To run the code, you only have to drag the robot package into the assets window, and the robot is automatically added to the scene.
In the end, it might be better if this was saved as a `PreFab` and not added to the scene.
The body looks hollow because the second mesh part was not automatically imported.
There are still problems with multiple mesh parts, where the `.urdf` and the `.urdf.xacro` are both being parsed.

![Showing the generated hierarchy. Some of the transformations have rounding errors.](/Robotic-Telepresence/2016/11/14/Baxter-imported/Hierarchy.png)

I wanted to try it out on some other robots, but most the ones I could get my hands on did not contain `.DAE` files, only `.STL` files which aren't supported by Unity.
Fortuantely, ROS provides a [converter](http://wiki.ros.org/collada_urdf).
>To be continued
