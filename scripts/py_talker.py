#!/usr/bin/env python

import rospy
from raven_2.msg import raven_automove


def main():
    """
    """
    rospy.init_node('py_talker')

    # Frequency to tick at
    # We also use the value for the length of the publisher queue
    tick_hz = 1000;
    pub = rospy.Publisher(
        '/raven_automove',
        raven_automove,
        queue_size=tick_hz
    )

    rate = rospy.Rate(tick_hz)
    while not rospy.is_shutdown():

        # Make a new automove message
        msg = raven_automove()
        msg.hdr.stamp = rospy.Time.now()

        # Make quaternions valid unit quaternions
        msg.tf_incr[0].rotation.w = 1.0
        msg.tf_incr[1].rotation.w = 1.0

        rospy.loginfo("Sending raven_automove")
        pub.publish(msg)

        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
