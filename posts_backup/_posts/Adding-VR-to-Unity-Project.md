---
title: Adding VR to Unity Project
date: 2017-02-07 15:47:54
tags:
---
#### Enabling VR
This step was very simple.
A preliminary step is to ensure the Steam client is installed, along with the SteamVR application.
In Unity, under "Player Preferences", tick the "Virtual Reality Supported" checkbox.
Done.
Now when SteamVR is running, pressing "Play" in Unity will focus itself in the VR headset.
This only enables head tracking though.
To get more functionality, download the free SteamVR plugin from the Unity assets store.
You can drag the prefabs out into the base project directory and ther controllers are shown (in the same skin as the user has set in their SteamVR preferences).
There seem to be some great resources for getting started, I'm using [this page](https://docs.unity3d.com/Manual/OpenVRControllers.html) for controller input.

#### VR tests
There seem to be many ways to access data from the headset or controllers.
I wrote a little script to spawn primitives at a controller's position with a trigger press.
The idea is to develop it to draw commands for where to move Baxter's hand to next.
```cs Unity script
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ControllerTest : MonoBehaviour {
	private SteamVR_TrackedController controller;
	private PrimitiveType currentPrimitiveType = PrimitiveType.Sphere;
	private bool arrowStarted = false;
	private Vector3 arrowStartPos;

	private void OnEnable() {
		controller = GetComponent<SteamVR_TrackedController>();
		controller.TriggerClicked += handle_trigger_clicked;
	}

	private void OnDisable() {
		controller.TriggerClicked -= handle_trigger_clicked;
	}

	private void handle_trigger_clicked(object sender, ClickedEventArgs e) {
		spawn_arrow_at_controller();
	}

	private void spawn_sphere_at_controller() {
		var sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
		sphere.transform.position = transform.position;
		sphere.transform.rotation = transform.rotation;
		sphere.transform.localScale = new Vector3(0.1f, 0.1f, 0.1f);
	}

	private void spawn_arrow_at_controller() {
		GameObject cylinder;
		if (!arrowStarted) {
			arrowStartPos = transform.position;
		} else {
			cylinder = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
			cylinder.transform.position = (arrowStartPos + transform.position) / 2;
			cylinder.transform.rotation = transform.rotation;
			cylinder.transform.localScale = new Vector3(0.1f, 0.1f, 0.1f);
		}
		arrowStarted = !arrowStarted;
	}
}
```
![First, just spawning at controller position](/Robotic-Telepresence/2017/02/07/Adding-VR-to-Unity-Project/Cylinders.png)
![Then, pick two points and it instantiates at the midpoint](/Robotic-Telepresence/2017/02/07/Adding-VR-to-Unity-Project/Midpoints.png)
I'll be rotating and scaling the cylinders into arrows to view the expected path of a hand.

#### Drawing arrows
After some tweaking, I managed to get it to be able to drag the controllers to draw 'arrows'.
I'll need to offset them, because the controller origin can't be visually pinpointed, so it's difficult for the user to choose a precise location.
The image below shows the controller origin. I'm thinking of using the very centre of the ring.

![{Red, Green, Blue} are {X, Y, Z} respectively](/Robotic-Telepresence/2017/02/07/Adding-VR-to-Unity-Project/Controller.png)

The script below enables functionality demonstrated in the video underneath.

```cs Final Unity Script
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ControllerTest : MonoBehaviour {
	private SteamVR_TrackedController controller;
	private PrimitiveType currentPrimitiveType = PrimitiveType.Sphere;
	public GameObject prefabCylinder; // Fill with arrow prefab
	private GameObject arrow; // Current arrow being drawn
	private Vector3 arrowStartPos;

	private void OnEnable() {
		// Register controller callbacks
		controller = GetComponent<SteamVR_TrackedController>();
		controller.TriggerClicked += handle_trigger_clicked;
		controller.TriggerUnclicked += handle_trigger_unclicked;
	}

	private void OnDisable() {
		// Unegister controller callbacks
		controller.TriggerClicked -= handle_trigger_clicked;
		controller.TriggerUnclicked += handle_trigger_unclicked;
	}

	// Start drawing the arrow
	private void handle_trigger_clicked(object sender, ClickedEventArgs e) {
		arrowStartPos = transform.position;
		arrow = (GameObject)Instantiate(prefabCylinder);
	}

	// Finished drawing the arrow
	private void handle_trigger_unclicked(object sender, ClickedEventArgs e) {
		arrow = null;
	}

	private void Update() {
		// Handle arrow logic
		if (controller.triggerPressed) {
			var direction = transform.position - arrowStartPos;
			var midpoint = direction/2;
			arrow.transform.rotation = Quaternion.FromToRotation(Vector3.up, direction);
        	arrow.transform.localScale = new Vector3(0.05f, direction.magnitude/2, 0.05f);
			arrow.transform.position = midpoint + arrowStartPos;
		}
	}
}
```

{% youtube 0gkBvNeSKKs %}
