---
title: Memory structure
date: 2017-03-02 22:47:09
tags:
---
#### Baxter's Hands
On examining the `URDF`, the different type of movement the hands have is called `prismatic`, as opposed to `revolute` for all other movable joints.
I've addded the functionality to read the messages from a topic. The rosbag only publishes one gripper for each hand, because the `urdf` talles the other to "mimic" the other one but with mirrored motion.
While trying to get this to work, I decided the structure of the shared memory needed to be improved.

#### Proposed structure
> 
| Offset    | Length | Description                    | Notes                                            |
| --------- | ------ | ------------------------------ | ------------------------------------------------ |
| 0         | 1      | How many joints `n`            | Static                                           |
| 1         | 1      | Joint name length `l`          | Static. Goes down hierarchy, then alphabetically |
| 2         | `l`    | Joint name                     | Mirrors joint name order                         |
|           |        | _(other joint names, lengths)_ |                                                  |
| `n*(l+1)` | 4      | Joint position                 | double in msg, truncated to float, as 4 bytes    |
|           |        | _(other joint positions)_      |                                                  |
| `n*(l+5)` | -      | End of data                    |                                                  |

The Unity project SHOULD! be able to dynamically associate the related `GameObjects`, as the names should match perfectly.
It could then notify the developer of any conflicts.

#### NEW proposed structure
I realised that as you won't know the names in advance, each one will be added to a list (vector :wink:) of known joints as they come in.
This means the offsets can't be set on startup.
I'll probably have to create objects in a vector for each joint:
> 
- Vector<Object type>
  - __Joint object__
  - Name
  - Offset of position

Each message will have to be compared to the list of objects, but the data for that comes in every message anyway so the bottleneck should still be in the receiving of messages.

> 
| Joint number `n` | Offset     | Length | Description           | Notes                              |
| ---------------- | ---------- | ------ | --------------------- | ---------------------------------- |
| -                | 0          | 1      | Number of joints      | Will increase as more are added    |
| 1                | 1          | 4      | Joint position        | Joint name to follow               |
|                  | 5          | 1      | Joint name length `l` | Need to know this to read the name |
|                  | 6          | `l`    | Joint name            | We know how many to read           |
|                  |            |        |                       |                                    |
| 2                | 1+`5n`+`l` |        | Joint position        | etc.                               |
|                  |            |        |                       |                                    |

After some head scratching (for a few days), I managed to get it working.
It showed me that my combined URDF was incorrect; I had replaced `${side}` with "right" and "left", and it should have been "r" and "l".
![This list is empty before running](/Robotic-Telepresence/2017/03/02/Memory-Structure/Dynamic Objects.png)
