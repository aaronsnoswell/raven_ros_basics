#!/usr/bin/env python
"""Demonstrates basic ROS control of the Raven II from python
"""

import sys
import rospy
import roslib

import numpy as np

# TODO ajs 21/May/18 Use message definitions from the raven_2 package
from autocircle_generator.msg import raven_state, raven_automove
#from raven_2.msg import raven_state, raven_automove


class DrawCube:
    """A simple class that draws a cube with the Raven 2 end effector
    """

    def __init__(self):
        """Constructor
        """
        self.sub = rospy.Subscriber("/ravenstate", raven_state, self.callback)
        self.pub = rospy.Publisher("/raven_automove", raven_automove, queue_size=1)

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
        self.max_step_size = 10


    def tick(self):
        """Publish method - call this at ~1000Hz
        """
        if self.latest_state == None:
            rospy.loginfo("Waiting for raven state message...")
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
        msg.hrd = std_msgs.msg.Header()
        msg.hdr.stamp = rospy.Time.now()

        # Set End Effector tip location
        msg.tf_incr[0].translation.x = goal_vec[0]
        msg.tf_incr[0].translation.y = goal_vec[1]
        msg.tf_incr[0].translation.z = goal_vec[2]

        # Make quaternion valud
        msg.rf_incr[0].rotation.w = 1


        
        # Uncomment for debugging info
        #rospy.loginfo("Publishing raven_automove message, tf_incr[0].translation: {}".format(goal_vec))
        self.pub.publish(msg)


    def callback(self, data):
        """Callback to recieve Raven II state
        """
        self.latest_state = data
        self.current_pos = self.latest_state.pos[0:3]

        
def main(args):
    """Main function
    """
    ds = DrawCube()
    rospy.init_node('draw_cube', anonymous=True)

    r = rospy.Rate(1000)
    while not rospy.is_shutdown():
        ds.tick()
        r.sleep()


if __name__ == '__main__':
    main(sys.argv)
