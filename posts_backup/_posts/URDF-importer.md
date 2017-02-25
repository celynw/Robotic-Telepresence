---
title: URDF importer
date: 2016-11-14 00:09:35
tags:
---
Most robot repositories should have a repository with a robot description folder.
In Baxter's case, this is `baxter_description`. It countains the `urdf` files and the mesh files.
By dragging this folder into the Unity Assets for a project, the importer script will automatically pick up the urdf files and filter down to the relevant one.
From here it can browse to the path of the meshes as the paths should match the existing directory layout.

The updated script is able to produce the hierarchy of joints and links with their respective transforms.
It uses a pair of functions that alternate and recurse with each other until the ends of the hierarchy are met, creating links and joints.
```cs udrfPostProcessor.cs excerpt, updated
...
	private static void parseURDF(string filename) {
		//Load file
		var xmlData = new XmlDocument();
		xmlData.Load(filename);
		//Set root node
		XmlNode rootNode = xmlData.DocumentElement;
		string name = rootNode.Attributes["name"].Value;
		GameObject rootObj = new GameObject(name);
		rootObj.tag = "URDF link";
		//Start recursive alternating functions to reach bottom of hierarchy
		XmlNodeList joints = rootNode.SelectNodes("joint/parent[@link='base']");
		foreach (XmlNode jointNode in joints)
			generateJoint(jointNode, rootObj, rootNode);
	}
	private static void generateJoint(XmlNode node, GameObject parentObj, XmlNode rootNode) {
		//Create joint
		node = node.ParentNode;
		string name = node.Attributes["name"].Value;
		GameObject obj = new GameObject(name);
		obj.transform.parent = parentObj.transform;
		obj.tag = "URDF joint";
		//Get origin rotation and translation parameters
		getOrigin(node.SelectSingleNode("origin"), obj);
		//Find the child link
		Debug.Log(node.SelectSingleNode("child").Attributes["link"].Value);
		XmlNode link = rootNode.SelectSingleNode("link[@name='"+node.SelectSingleNode("child").Attributes["link"].Value+"']");
		generateLink(link, obj, rootNode);
	}
	private static void generateLink(XmlNode node, GameObject parentObj, XmlNode rootNode) {
		//Create link
		string name = node.Attributes["name"].Value;
		GameObject obj = new GameObject(name);
		obj.transform.parent = parentObj.transform;
		obj.tag = "URDF link";
		//Get origin rotation and translation parameters
		XmlNode originNode = node.SelectSingleNode("visual/origin");
		if (originNode != null) //Not all links have a visual node
			getOrigin(originNode, obj);
		//Find all joints who have this link as a parent
		XmlNodeList joints = rootNode.SelectNodes("joint/parent[@link='"+name+"']");
		foreach (XmlNode jointNode in joints)
			generateJoint(jointNode, obj, rootNode);
	}

	private static void getOrigin(XmlNode node, GameObject obj) {
		//Parse for translation
		string[] xyzStr = node.Attributes["xyz"].Value.Split(' ');
		Vector3 locations = new Vector3();
		locations.x = float.Parse(xyzStr[0]);
		locations.y = float.Parse(xyzStr[1]);
		locations.z = float.Parse(xyzStr[2]);
		//Parse for rotation
		string[] rpyStr = node.Attributes["rpy"].Value.Split(' ');
		Vector3 angles = new Vector3();
		angles.x = float.Parse(rpyStr[0]); //r
		angles.y = float.Parse(rpyStr[1]); //p
		angles.z = float.Parse(rpyStr[2]); //y
		angles = angles * Mathf.Rad2Deg;
		//Apply Transform
		obj.transform.Translate(-locations.x, locations.y, locations.z);
		obj.transform.Rotate(0, 0, -angles.z, Space.Self);
		obj.transform.Rotate(0, -angles.y, 0, Space.Self);
		obj.transform.Rotate(angles.x, 0, 0, Space.Self);
	}
}
```

An annoying run-time error which got me for a while was that a trailing slash can cause the XML parser to reach the end of the file unexpectedly.
```cs
node.SelectSingleNode("visual/geometry/mesh/");
```

Unfortunately, adding meshes directly using scripting loses out in some of the extra functionality that goes on behind the scenes when you drag an asset into the scene.
Creating a mesh renderer and a mesh filter via a script for the GameObjects is possible, but attached models are bright pink (no assigned texture).
When you drag an asset to instantiate it, Unity knows which materials to assign (which have been automativally generated).
Perhaps an easier way would be to emulate this action, and then delete any superfluous components?
It also doesn't handle meshes which have been split for having too many vertices very well (i.e. the torso).

There's still scope for the other parameters, such as movement limits, collision geometry, primitive shapes in place of models, and so on.
