# raven_ros_basics

A simple example demonstrating ROS control of the Raven II surgical robot.

# Requirements

Tested on ROS Kinetic and Ubuntu 16.X. Also works from Bash for Windows.

# Installation

1. **Clone this repo into your Raven II workspace.**
It should be a peer directory to
the [raven_2 repo](https://github.com/uw-biorobotics/raven2).

```
> cd ros_workspace/src
> ll
raven_2
> git clone https://github.com/aaronsnoswell/raven_ros_basics.git
```

2. **Build your workspace**

```
> cd ..
> pwd
/Users/aaron/ros_workspace
> catkin_make
```

If the make worked, you should be able to `roscd raven_ros_basics`.

# Usage

TODO...
