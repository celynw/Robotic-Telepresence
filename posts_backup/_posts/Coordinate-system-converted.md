---
title: Coordinate system converted
date: 2016-11-09 13:04:25
tags:
---
### Differences
ROS uses a right-handed coordinate system.
Unity uses a left-handed coordinate system.
([handedness](https://en.wikipedia.org/wiki/Right-hand_rule))

### Translation
ROS uses [this convention](http://www.ros.org/reps/rep-0103.html):
> 
| Axis  | ROS Direction |
| ----- | ------------- |
| X +ve | forwards      |
| Y +ve | left          |
| Z +ve | up            |

On importing Baxter models as a Unity assets, they come in at +90 degrees rotated around Unity's X axis.
In relation to Baxter, the Unity coordinate system is:
> 
| Axis  | Unity Direction |
| ----- | --------------- |
| X +ve | backwards       |
| Y +ve | left            |
| Z +ve | up              |

When the parent-most GameObject is rotated so that Baxter is upright, the child XYZ frame is also rotated.
So for translation, all I need to do is negate the X translation.

### Rotation
The following table describes what an increase in rotation value for each axis looks like in ROS and Unity (using local rotations).
As the axes for translations can now match up, disregarding negativity they should for rotations.
> 
| Rotation | Label | Axis    | Direction in ROS | Direction in Unity |
| -------- | ----- | ------- | ---------------- | ------------------ |
| Roll     | r     | X (+ve) | right            | right              |
| Pitch    | p     | Y (+ve) | forwards         | backwards          |
| Yaw      | y     | Z (+ve) | left             | right              |

The order in which rotations are performed makes a difference.
The code in the following script was used to successfully rotate the meshes to the correct starting positions.
```cs ConvertCoordinateSystem.cs
using UnityEngine;
using System.Collections;

public class InitialRotation : MonoBehaviour {
	public Vector3 input = new Vector3();
	public Vector3 angles = new Vector3(0.0f, 0.0f, 0.0f);
	private Vector3 saved;

	// Use this for initialization
	void Start () {
		angles.x = input.x * Mathf.Rad2Deg;
		angles.y = input.y * Mathf.Rad2Deg;
		angles.z = input.z * Mathf.Rad2Deg;

		transform.localEulerAngles = Vector3.zero;
		transform.Rotate(0, 0, -angles.z, Space.Self);
		transform.Rotate(0, -angles.y, 0, Space.Self);
		transform.Rotate(angles.x, 0, 0, Space.Self);
		saved = transform.localEulerAngles;

		//Don't switch to the game window
		UnityEditor.SceneView.FocusWindowIfItsOpen(typeof(UnityEditor.SceneView));
	}

	// Update is called once per frame
	void Update () {}
}
```
The `Vector3` variable 'Input' was used to manually enter the values from the `.xacro.urdf` file.
The `Vector3` variable 'angles' I used as an indicator to show what (additional?) rotation was being performed on the part.

![Script inputs](/Robotic-Telepresence/2016/11/09/Coordinate-system-converted/Inputs.png)

To verify that the script was working, I rotated the joints using the `<limit lower/>` and `<limit upper/>` values to see if the arm would clip into another body part.
This showed that it worked correctly, because there was no clipping.
It showed that the shape of Baxter has been designed so that there are hollows for the respective protruding parts to get a larger operating range of movement.

![Verifying the joint limits](/Robotic-Telepresence/2016/11/09/Coordinate-system-converted/Limits.png)

I had trouble importing E1 (lower elbow) and E0 for some reason, maybe it was a corrupt file.
Replacing it with a fresh one from the GitHub repository solved the issue.
