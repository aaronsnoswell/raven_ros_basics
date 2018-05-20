# draw_cube

A simple example demonstrating ROS contorl of the Raven II end effector from
python

# Installation

1. Clone this repo into your Raven II workspace. It should be a peer directory to
the [raven_2 repo](https://github.com/uw-biorobotics/raven2).

```
> cd ros_workspace/src
> ll
raven_2
> git clone https://github.com/aaronsnoswell/draw_cube.git
```

2. Build your workspace

```
> cd ..
> pwd
/Users/aaron/ros_workspace
> catkin_make
```

If the make worked, you should be able to `roscd draw_cube`.

3. Run the code in simulation mode

To run in simulation only, using a raven stub;

```
> roslaunch draw_cube draw_cube.launch
```

You should see console output indicating the simulated end effector is moving
through the points of a cube.

4. Run on the real hardware

To run on the real hardware, ensure your raven master node is running, the
hardware is homed and clear of any potential collisions, and the Raven II is
in cartesian mode.

```
> roslaunch draw_cube draw_cube.launch sim:=false
```

You should see the Raven II left end effector move through the coordinates of
a 2x2x2cm cube.
The initial location of the end effector is used as the center
of the cube.
