#!/usr/bin/env python
"""Demonstrates basic ROS control of the Raven II from python
"""

import sys
import rospy
import std_msgs.msg
import roslib

import numpy as np

# TODO ajs 21/May/18 Use message definitions from the raven_2 package
from AutoCircle_generater.msg import raven_state, raven_automove
#from raven_2.msg import raven_state, raven_automove


class DrawCube:
    """A simple class that draws a cube with the Raven 2 end effector
    """

    def __init__(self, pub_rate):
        """Constructor

        Args:
            pub_rate (int): Target rate to publish at
        """

        # Target publish tick rate
        self.pub_rate = pub_rate
        
        # Subscribe to raven state
        self.sub = rospy.Subscriber("/ravenstate", raven_state, self.callback)

        # Use publish queue size that matches publish rate
        self.pub = rospy.Publisher("/raven_automove", raven_automove, queue_size=pub_rate)

        self.latest_state = None
        self.current_pos = None

        # Units are in microns
        # cube_scale = 10000 creates a 2cm square cube
        self.cube_scale = 10000

        self.cube_points = self.cube_scale * np.array([
            [-1, 1, 1],
            [-1, -1, 1],
            [1, -1, 1],
            [1, 1, 1],
            [1, 1, -1],
            [1, -1, -1],
            [-1, -1, -1],
            [-1, 1, -1]
        ], dtype=float)

        # Current goal point we're going to
        self.goal_point_index = 0

        # Micron tolerance for each cube coordinate
        self.point_eps = 5

        # Max number of microns to step
        self.max_step_size = 3

        # Log status
        rospy.loginfo("Waiting for raven state message...")


    def tick(self):
        """Publish method - call this at ~1000Hz
        """
        if self.latest_state is None:
            return

        # Get current goal point
        goal_point = self.cube_points[self.goal_point_index]
        goal_vec = goal_point - self.current_pos
        dist = np.linalg.norm(goal_vec)

        # Limit step sizes
        goal_vec /= dist
        goal_vec *= min(dist, self.max_step_size)
        
        if dist < self.point_eps:
            
            # Reached current goal
            tmp = self.goal_point_index

            self.goal_point_index += 1
            self.goal_point_index = self.goal_point_index % len(self.cube_points)
            
            if self.goal_point_index < tmp:
                print("Finished cube!")
                print("")

            print("Moving to goal point {}/{} ({})".format(
                self.goal_point_index+1,
                len(self.cube_points),
                self.cube_points[self.goal_point_index])
            )

            return

        # Prepare a new movement command
        msg = raven_automove()

        # Set header
        msg.hdr = std_msgs.msg.Header()
        msg.hdr.stamp = rospy.Time.now()

        # Set End Effector tip location
        msg.tf_incr[0].translation.x = goal_vec[0]
        msg.tf_incr[0].translation.y = goal_vec[1]
        msg.tf_incr[0].translation.z = goal_vec[2]

        # Make quaternions valid
        msg.tf_incr[0].rotation.w = 1
        msg.tf_incr[1].rotation.w = 1
        
        # Uncomment for debugging info
        #rospy.loginfo("Publishing raven_automove message, tf_incr[0].translation: {}".format(goal_vec))
        self.pub.publish(msg)


    def callback(self, data):
        """Callback to recieve Raven II state
        """

        if self.latest_state is None:
            rospy.loginfo("Got raven state, moving now...")

            # Make cube points relative to initial end effector location
            current_xyz = np.array(data.pos[0:3], dtype=float)
            self.cube_points = self.cube_points + current_xyz

        self.latest_state = data
        self.current_pos = self.latest_state.pos[0:3]

        
def main(args):
    """Main function
    """

    # Tick rate in Hz
    tick_rate = 1000

    rospy.init_node('draw_cube', anonymous=True)
    rospy.loginfo("Let's draw a cube!")
    dc = DrawCube(tick_rate)

    r = rospy.Rate(tick_rate)
    while not rospy.is_shutdown():
        dc.tick()
        r.sleep()


if __name__ == '__main__':
    main(sys.argv)
