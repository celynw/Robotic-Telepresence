---
title: VR improvements
date: 2017-02-20 17:55:29
tags:
---
#### Hands
I've made it so that the correct-side gripper is drawn at the controller's position when the trigger is pressed.
It's translucent to signify a target, as well as having the green line to show the order between targets.
The controller models disappear when selecting a position so that it's easier to see where to line the gripper up with.

I've also added text to display which hand is which, as well as diagrams to show which orientation we are in.

{% youtube Q-NrmlMVt88 %}

#### Command queue
I'm not sure how to mark these steps as 'done' so that they can be removed from the queue, but subsequent moves are queued for Baxter's IK solver to crunch through.

#### Still need
- The ability to cancel the last move
- When swapping orientations, move the physical location of the user (fade through black?)
