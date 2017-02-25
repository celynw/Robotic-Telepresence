---
title: Importing directly from URDF
date: 2016-11-13 18:28:59
tags:
---
The `urdf` file is an XML-based format.
In order to import values and heirarchical structure from the file into Unity automatically, I'll have to parse it in some way.
I can ignore many parts of the `urdf` specification as I don't need them yet.
There are a few options:

- ### Scene files
The scene files in Unity hold the structure of `GameObjects`, their configuration, and how the world is laid out.
The online documentation for Unity states that the scene files can be saved in a [text-based format](https://docs.unity3d.com/Manual/TextSceneFormat.html).
One option could be to use a program or script to create a new scene file with the `urdf` parameters within.

- ### Prefab
Unity `GameObjects` can be instances of a `Prefab`. Usually, the `GameObjects` are instantiated from the pre-existing `Prefab`.
However, `Prefabs` can be retroactively created from a `GameObject`, and then the `GameObject` becomes an instance of that new `Prefab`.
One idea could have been to create `Prefabs` outside of Unity and then import them, but it looks like `Prefabs` are binary files (not zip files), and there's no documentation of the format.

- ### Unity script
There are methods in Unity for creating, instantiating, modifying and destroying `GameObjects`.
Scripts attached to `GameObjects` usually execute when the scene is set to run, but there are other ways they can be used to enhance the workflow of the developer.
If I can achieve a sufficient level of functionality with `C#` this seems like the best solution.

	Enter `AssetPostProcessor`.

### AssetPostProcessor
According to the [docs](https://docs.unity3d.com/ScriptReference/AssetPostprocessor.html):
> **AssetPostprocessor** lets you hook into the import pipeline and run scripts prior or after importing assets.

Create an `Editor` folder in the root directory of the project, if it doesn't exist already.
(You can re-import assets by right-clicking them and selecting, `Reimport`.)

At first, the script runs for all assets, including itself!
By adding in some functions to check if the extension contains a `.urdf` file, we can filter the results.

```cs urdfPostProcessor.cs initial version
using UnityEngine;
using UnityEditor;
using System.Collections;
using System.IO; //For Path

public class urdfPostProcessor : AssetPostprocessor
{
	private static void OnPostprocessAllAssets(string[] importedAssets, string[] deletedAssets, string[] movedAssets, string[] movedFromPath) {
		foreach (string str in importedAssets) {
			string ext = Path.GetExtension(str);
			if (ext == ".xacro")
				ext = Path.GetExtension(Path.GetFileNameWithoutExtension(str));
			if (ext == ".urdf")
				Debug.Log("YES: "+str);
			else
				Debug.Log("no: "+str);
		}
	}
}
```
Now that this script runs for each `urdf` file, we can try to access some of the data.
It is reccommended to use the system's xml parsing.
```cs urdfPostProcessor.cs continued
...
using System.Xml; //For urdf
	private static void importURDF(string filename) {
		//Load XML file
		var xmlData = new XmlDocument();
		xmlData.Load(filename);
		//Set root
		var root = xmlData.DocumentElement;//.GetEnumerator();
		//Create nodelist
		XmlNodeList nodeList;
		nodeList = root.SelectNodes("link");
		//Print out some results
		for (int i=0; i<nodeList.Count; i++) {
			var node = nodeList[i];
			XmlNode ret = node.SelectSingleNode("visual/origin");
			if (ret != null)
				Debug.Log(node.Attributes["name"].Value+": "+ret.Attributes["rpy"].Value);
		}
	}
```
The code above finds the `urdf` links which have a `visual` child, with an `origin` child of its own.
For each of those, it prints their names and the `rpy` values of the origin node.

![Values from the XML in the debug logger in Unity](/Robotic-Telepresence/2016/11/13/Importing-directly-from-URDF/XML Values.png)

This isn't exactly what we want, because it doesn't follow the hierarchy. But we're getting there.
