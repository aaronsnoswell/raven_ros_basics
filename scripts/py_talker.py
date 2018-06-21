#!/usr/bin/env python

import rospy
from raven_2.msg import raven_automove

def talker():
    pub = rospy.Publisher('/raven_automove', raven_automove, queue_size=1000)
    rospy.init_node('py_talker', anonymous=True)

    rate = rospy.Rate(1000)
    while not rospy.is_shutdown():
        msg = raven_automove()
        rospy.loginfo("Sending raven_automove")
        pub.publish()

        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
