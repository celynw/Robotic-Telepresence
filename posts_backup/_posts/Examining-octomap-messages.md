---
title: Examining octomap messages
date: 2017-03-17 12:05:47
tags:
---
The `rosbag` I have access to contains octomap messages.
Using `rosbag info`, it has 55 messages. Echoing the first one shows it contains 83860 bytes.
The maximum I've been able to get using `rosserial` is 1177, although I'd expect it to be 65536.
