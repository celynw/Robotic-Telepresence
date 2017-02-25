---
title: Introduction to ROS
date: 2016-10-21 10:31:34
tags:
---
I'll mostly be following the [instructions](http://wiki.ros.org/kinetic/Installation/Ubuntu) on the official ROS site, with some modifications.
First I had to add the ROS repository to the source list:
`deb http://packages.ros.org/ros/ubuntu xenial main`
```zsh
$ sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 0xB01FA116
$ sudo apt-get update
$ sudo apt-get install ros-kinetic-desktop-full
```
Initialise `rosdep`. Can install dependencies and required for some core components of ROS.
```zsh
$ sudo rosdep init
$ rosdep update
```
Set up the environment.
`catkin` is the build system for ROS.
The installation instructions on the website assume you have `bash` as your shell.
I have `zsh`. The directory also has a set up script for `zsh` users too.
```zsh
$ echo ". /opt/ros/kinetic/setup.zsh" >> ~/.zshrc
$ . ~/.zshrc
```
Get rosinstall. Distributed separately, is for downloading source trees for ROS packages.
```zsh
$ sudo apt-get install python-rosinstall
```

### First ROS tests
Checking the environment variables have been set:
```zsh
$ printenv | grep ROS
```
Creating a ROS workspace:

```zsh
$ mkdir -p ~/catkin_ws/src
$ cd !$
$ catkin_init_workspace
$ cd ../
$ catkin_make
$ . devel/setup.zsh
```
Now the environment has been overlayed onto our own. Check with:
```zsh
$ echo $ROS_PACKAGE_PATH
```

### Tutorials
Install tutorials if I haven't got them already:
```zsh
$ sudo apt-get install ros-kinetic-ros-tutorials
```
There seems to be lots of interesting tools, which are all available with tab-completion. This explains the specific zsh and bash setup scripts.
