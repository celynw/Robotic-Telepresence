---
title: 'VR feature: Angle snapping'
date: 2017-02-12 17:59:33
tags:
---
I thought this would be an easy feature to add.
When drawing targets, holding the grip buttons would snap the new location to a 45-degree-quantised vector from the first position.

At first, I thought about comparing the actual position difference to the 45-degree vectors and using the smallest angle difference.
This had problems with vectors 180 degrees out from each other.

Then I tried normalising the position difference and comparing the position with the normalised 45-degree vectors.
This seems possible.

![This shows the angles to snap to](/Robotic-Telepresence/2017/02/12/VR-feature-Angle-snapping/Angles.png)

---
However, I haven't tried to use Baxter yet, and I'm not sure of how the IK solver will project the path of the hand; it may not necessarily be a straight path, so this visualisation could be misleading.
Maybe a better way would be to replace the controller models with ghostly grippers which will remain at the target positions until the IK solver has finished.

I've also had a quick look into a Unity-based IK solver. They are common in games with avatars.
