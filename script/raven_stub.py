#!/usr/bin/env python
"""A fake Raven II stub that follows translational raven_automove commands
"""

import sys
import rospy
import roslib

import numpy as np

# TODO ajs 21/May/18 Use message definitions from the raven_2 package
from autocircle_generator.msg import raven_state, raven_automove
#from raven_2.msg import raven_state


class RavenStub:
    """A Raven II stub that follows translational raven_automove commands
    """

    def __init__(self):
        """Constructor
        """
        self.sub = rospy.Subscriber("/raven_automove", raven_automove, self.callback)
        self.pub = rospy.Publisher("/ravenstate", raven_state, queue_size=1)
        self.latest_state = raven_state()

        # Units are microns
        self.noise_scale = 3


    def tick(self):
        """Publishes state
        """
        self.pub.publish(self.latest_state)


    def callback(self, data):
        """Callback to process raven_automove messages
        """

        # Apply translations that track the tf_incr parameter, with some actuator noise
        self.latest_state.pos[0] += data.tf_incr[0].translation.x + np.random.uniform(self.noise_scale)
        self.latest_state.pos[1] += data.tf_incr[0].translation.y + np.random.uniform(self.noise_scale)
        self.latest_state.pos[2] += data.tf_incr[0].translation.z + np.random.uniform(self.noise_scale)
        self.latest_state.pos[3] += data.tf_incr[1].translation.x + np.random.uniform(self.noise_scale)
        self.latest_state.pos[4] += data.tf_incr[1].translation.y + np.random.uniform(self.noise_scale)
        self.latest_state.pos[5] += data.tf_incr[1].translation.z + np.random.uniform(self.noise_scale)

        
def main(args):
    """Main function
    """
    frf = RavenStub()
    rospy.init_node('raven_stub', anonymous=True)

    r = rospy.Rate(1000)
    while not rospy.is_shutdown():
        frf.tick()
        r.sleep()


if __name__ == '__main__':
    main(sys.argv)
