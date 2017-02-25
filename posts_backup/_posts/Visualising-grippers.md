---
title: Visualising grippers
date: 2017-02-15 11:24:14
tags:
---
The meshes for the grippers were not previously present beacuse they are in a separate folder.
I'm going to be using the models to show the target position for the hands.
The way the grippers are included in the `URDF` description is by using variables within the `.urdf` file, such as `${side}`.
The reason for this is because gripper parts are used (reflected) for left and right, and for each arm. The grippers can be interchanged for different types as well.
My importer script does not handle variables, so at first it seemed simple enough to just manually enter their locations into Unity.
I recall that:

For position
> 
| ROS   | Unity |
| ----- | ------|
| X +ve | X -ve |
| Y +ve | Y +ve |
| Z +ve | Z +ve |

For rotation:
> 
| ROS   | Unity |
| ----- | ------|
| X +ve | X +ve |
| Y +ve | Y -ve |
| Z +ve | Z -ve |
Must be performed in order: Z, Y, X

I am using the "standard narrow" grippers for now.

I was running into problems with manually adding the grippers to the scene, there's a lot of numbers in the wrong order and format.
It would be a reasonable task to edit my importer to support the extra files and conditional arguments in the new file.
The easiest thing to do was to merge the `.urdf` files into a single one manually, and then importing that.

Each link creates a joint, and each joint creates a link, recursively until the bottom of the hierarchy.
When creating a link, if it has a `visual/geometry/mesh` node, then the current link `GameObject` is destroyed, and replaced with the related mesh (which generates its own `GameObject`).
The list below shows how meshes in the `baxter_base` section import, and their components:
> 
- NAME
  [Animator]
  - Camera
  - NAME
    [Mesh Filter]
    [Mesh Renderer]
  - Lamp

However, this is what the grippers import like:
> 
- NAME
  [Animator]
  [Mesh Filter]
  [Mesh Renderer]

I've changed the importer to adapt.
I must have made some small mistakes, as I had to manually change the sign or axes of some values for the parts to be in their correct locations.

I had to replace some variables, such as `${-pi/2}` and `${finger_length}`.
`${reflect}` is used as ±1.

![Using the standard narrow grippers for now, in slot 0](/Robotic-Telepresence/2017/02/15/Visualising-grippers/Result.png)

![A photo of Baxter's wrist stub showing the slots](/Robotic-Telepresence/2017/02/15/Visualising-grippers/Photo.jpg)
