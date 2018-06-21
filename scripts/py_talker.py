#!/usr/bin/env python

import rospy
from raven_2.msg import raven_automove

def talker():
    pub = rospy.Publisher('/raven_automove', raven_automove, queue_size=1000)
    rospy.init_node('py_talker', anonymous=True)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():

        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)

        pub.publish(raven_automove())

        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
