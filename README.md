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

2. **Important: Re-source your workspace**

Without this, python won't be able to see the raven_2 package

```
> source devel/setup.bash
```

You should now be able to see the raven_2 package from python;

```
> python
Python 2.7.12 (default, Dec  4 2017, 14:50:18)
[GCC 5.4.0 20160609] on linux2
Type "help", "copyright", "credits" or "license" for more information.
>>> import raven_2
>>>
```

If you can do this, you should be good to go...

# Usage

TODO...
