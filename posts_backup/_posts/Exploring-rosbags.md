---
title: Exploring rosbags
date: 2016-11-23 23:02:48
tags:
---
In the [last project meeting](/Robotic-Telepresence/2016/11/22/Project-meeting-5/), we recorded about 60 seconds worth of Baxter waving his arms around and saved it as a rosbag.
The uncompressed version is 8.5GB, and to view it in RViz for example will probably require it to be loaded into RAM.
Oscar recommended using a RAMdisk to speed things up.
Since I have _too much_ RAM (64GB), I thought it would be easier to just put the whole Ubuntu virtual machine into the RAMdisk.

First, make sure roscore is running.
In each separate terminal, I remember I have to source the `workspace/devel/setup.zsh` file.
```zsh Command line 1
$ cd ~/catkin_ws
$ . ./devel/setup.zsh
$ roscore
```
From here, I can do
```zsh Command line 2
$ rosbag info ~/Downloads/baxter.bag
path:        /home/celynwalters/Downloads/baxter.bag
version:     2.0
duration:    1:03s (63s)
start:       Nov 22 2016 16:49:25.44 (1479833365.44)
end:         Nov 22 2016 16:50:28.49 (1479833428.49)
size:        7.9 GB
messages:    37025
compression: none [866/866 chunks]
types:       octomap_msgs/Octomap    [9a45536b45c5e409cd49f04bb2d9999f]
             sensor_msgs/JointState  [3066dcd76a6cfaef579bd0f34173e9fd]
             sensor_msgs/PointCloud2 [1158d486dd51d683ce2f1be655c3c181]
             tf2_msgs/TFMessage      [94810edda583a504dfda3829e70d7eec]
topics:      /camera/depth_registered/points     865 msgs    : sensor_msgs/PointCloud2
             /octomap_full                        55 msgs    : octomap_msgs/Octomap   
             /robot/joint_states               24149 msgs    : sensor_msgs/JointState  (2 connections)
             /tf                               11954 msgs    : tf2_msgs/TFMessage      (7 connections)
             /tf_static                            2 msgs    : tf2_msgs/TFMessage      (2 connections)
```
Great, I can see all of the ROS topics it contains.
I know from my `rosserial` experiments that I can listen to a topic using `rostopic echo` even if it isn't visible yet.
In a separate shell I did this for the joint angles as this is the 'easiest' to integrate into Unity for now.
```zsh Command line 3
$ rostopic echo /robot/joint_states
```
Back in this shell, I played back the bag:
```zsh Command line 2
$ rosbag play ~/Downloads/baxter.bag
[ INFO] [1479942083.502475169]: Opening /home/celynwalters/Downloads/baxter.bag

Waiting 0.2 seconds after advertising topics... done.

Hit space to toggle paused, or 's' to step.
 [RUNNING]  Bag Time: 1479833369.512015   Duration: 4.075116 / 63.055731     
```
This gives a very fast output of each frame for each topic:
```zsh Command line 3
header: 
  seq: 387581
  stamp: 
    secs: 1479833365
    nsecs: 441285362
  frame_id: ''
name: ['head_nod', 'head_pan', 'left_e0', 'left_e1', 'left_s0', 'left_s1', 'left_w0', 'left_w1', 'left_w2', 'right_e0', 'right_e1', 'right_s0', 'right_s1', 'right_w0', 'right_w1', 'right_w2', 'torso_t0']
position: [0.0, 0.03029612056073692, -1.405126401703039, 1.2931458041874038, -0.04832039481839053, 0.030679615757708275, 1.5991749713705439, 1.3537380453088776, -0.4636456931383663, 1.647111870991963, 1.5945730290068876, -0.18676216092504913, 0.13537380453088776, -1.5872866202644318, 1.491029325824622, -0.9004467224887379, -12.565987119160338]
velocity: [0.0, 0.003926990816250607, -0.009817477040626518, -0.005497787142750851, -0.005890486224375911, -0.00510508806112579, -0.012959069693627005, 0.003926990816250607, -0.01021017612225158, 0.011780972448751823, 0.01649336142825255, 0.0007853981632501215, 0.023561944897503646, -0.009032078877376396, -0.01531526418337737, -0.009817477040626518, 0.0]
effort: [0.0, 0.0, -18.58, -0.312, 0.216, -7.88, 0.896, 0.2, 0.14, 21.096, 1.26, 1.088, -4.268, 0.828, -0.052, -0.036, -20.48]
---
header: 
  seq: 387582
  stamp: 
    secs: 1479833365
    nsecs: 451285944
  frame_id: ''
name: ['head_nod', 'head_pan', 'left_e0', 'left_e1', 'left_s0', 'left_s1', 'left_w0', 'left_w1', 'left_w2', 'right_e0', 'right_e1', 'right_s0', 'right_s1', 'right_w0', 'right_w1', 'right_w2', 'torso_t0']
position: [0.0, 0.03029612056073692, -1.4043594113090963, 1.2931458041874038, -0.04793689962141918, 0.03106311095467963, 1.5991749713705439, 1.3537380453088776, -0.464412683532309, 1.6455778902040776, 1.5934225434159734, -0.18561167533413506, 0.1357572997278591, -1.5869031250674606, 1.491029325824622, -0.9004467224887379, -12.565987119160338]
velocity: [0.0, -0.0047123889795007284, 0.010995574285501701, 0.02120575040775328, -0.003926990816250607, 0.01688606050987761, -0.001570796326500243, -0.00039269908162506076, 0.006675884387626033, 0.014529866020127248, -0.006283185306000972, 0.01649336142825255, 0.02552544030562895, -0.0047123889795007284, 0.023169245815878585, 0.01021017612225158, 0.0]
effort: [0.0, 0.0, -18.42, -0.248, 0.404, -7.912, 0.924, 0.216, 0.128, 20.932, 1.228, 1.184, -4.368, 0.816, -0.064, -0.028, -20.48]
---
...
```

I'm going to adapt the `rosserial_hello_world` code to subscribe instead of publish.
The [documentation](http://wiki.ros.org/rosserial/Overview/Publishers%20and%20Subscribers) says that you need to include the header file for the sort of message you want to use.
Thankfully, the built `lib/` folder on the windows side includes baxter's message descriptions.
You then "create a function that returns void and takes a constant reference of your message type as its argument".
"Then to actually setup the connection we have to create a statically allocated ros::Subscriber which is templated on your message type and call NodeHandle::subscribe within the setup() function"
Sounds easy enough...

After I spent ages trying to make sense of the `baxter_core_msgs/JointCommand.h` header file, I eventually found out which messages to use.
To find out, play the `rosbag`, and in another shell do:
```zsh Command line 3
$ rostopic list -v

Published topics:
 * /robot/joint_states [sensor_msgs/JointState] 1 publisher
 * /camera/depth_registered/points [sensor_msgs/PointCloud2] 1 publisher
 * /octomap_full [octomap_msgs/Octomap] 1 publisher
 * /rosout [rosgraph_msgs/Log] 1 publisher
 * /tf [tf2_msgs/TFMessage] 1 publisher
 * /tf_static [tf2_msgs/TFMessage] 1 publisher
 * /clock [rosgraph_msgs/Clock] 1 publisher
 * /rosout_agg [rosgraph_msgs/Log] 1 publisher

Subscribed topics:
 * /rosout [rosgraph_msgs/Log] 1 subscriber
```
So actually, the joint angles use the `sensor_msgs/JointState` message, 
I had to move some lines around, correct some typos, syntax errors etc. etc.
I wasn't sure about printing actual joint angles yet, but I know I can print `position_length`, which presumably means how many joint positions there are.

It worked! The screenshot (often...) shows there are 17 angles. This almost definitely corresponds to:
- 14 arm joints (2x7)
- 2 head joints
- 1 torso joint

![Left side is Ubuntu running ROS playing the Baxter rosbag, right side is Windows subscribing to it](/Robotic-Telepresence/2016/11/23/Exploring-rosbags/Subscriber.png)
With a little more tweaking, and it's actually pretty easy!
I can pull the names out and format it in any way I choose to suit my Unity needs.

![The '1' position_length messages are revealed to be for the grippers](/Robotic-Telepresence/2016/11/23/Exploring-rosbags/Named.png)

Next stop, compiling this code into the Unity `.dll`, and then getting the joints to mirror the `rosbag`.
NICE.
