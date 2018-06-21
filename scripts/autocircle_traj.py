#!/usr/bin/env python
"""Replays sample trajectory from Autocircle_generater
"""

import sys
import csv
import os
import rospy
import std_msgs.msg
import roslib

import numpy as np

# TODO ajs 21/May/18 Use message definitions from the raven_2 package
#from autocircle_generator.msg import raven_state, raven_automove
#from raven_2.msg import raven_state, raven_automove
from draw_cube.msg import raven_state, raven_automove

# CSV parameters
csv_filename = os.path.join(
    os.path.dirname(os.path.realpath(__file__)),
    "autocircle_traj.csv"
)
csv_cols = {
    "time": 0,
    "field.hdr.seq": 1,
    "field.hdr.stamp": 2,
    "field.hdr.frame_id": 3,
    "field.del_pos0": 4,
    "field.del_pos1": 5,
    "field.del_pos2": 6,
    "field.del_pos3": 7,
    "field.del_pos4": 8,
    "field.del_pos5": 9,
    "field.tf_incr0.translation.x": 10,
    "field.tf_incr0.translation.y": 11,
    "field.tf_incr0.translation.z": 12,
    "field.tf_incr0.rotation.x": 13,
    "field.tf_incr0.rotation.y": 14,
    "field.tf_incr0.rotation.z": 15,
    "field.tf_incr0.rotation.w": 16,
    "field.tf_incr1.translation.x": 17,
    "field.tf_incr1.translation.y": 18,
    "field.tf_incr1.translation.z": 19,
    "field.tf_incr1.rotation.x": 20,
    "field.tf_incr1.rotation.y": 21,
    "field.tf_incr1.rotation.z": 22,
    "field.tf_incr1.rotation.w": 23
}

        
def main(args):
    """Main function
    """

    # Tick rate in Hz
    tick_rate = 1000

    # Load CSV trajectory into array
    traj = np.empty(shape=(0, 14), dtype=float)
    with open(csv_filename, "rb") as csvfile:
        reader = csv.reader(csvfile, delimiter=',')

        # Skip header row
        reader.next()

        for row in reader:
            traj = np.vstack((
                traj,
                np.array(((
                    row[csv_cols["field.tf_incr0.translation.x"]],
                    row[csv_cols["field.tf_incr0.translation.y"]],
                    row[csv_cols["field.tf_incr0.translation.z"]],

                    row[csv_cols["field.tf_incr0.rotation.x"]],
                    row[csv_cols["field.tf_incr0.rotation.y"]],
                    row[csv_cols["field.tf_incr0.rotation.z"]],
                    row[csv_cols["field.tf_incr0.rotation.w"]],

                    row[csv_cols["field.tf_incr1.translation.x"]],
                    row[csv_cols["field.tf_incr1.translation.y"]],
                    row[csv_cols["field.tf_incr1.translation.z"]],

                    row[csv_cols["field.tf_incr1.rotation.x"]],
                    row[csv_cols["field.tf_incr1.rotation.y"]],
                    row[csv_cols["field.tf_incr1.rotation.z"]],
                    row[csv_cols["field.tf_incr1.rotation.w"]]
                )), dtype=float)
            ))

    # Create node
    rospy.init_node('replay_autocircle', anonymous=True)

    # Use publish queue size that matches publish rate
    pub = rospy.Publisher("/raven_automove", raven_automove, queue_size=tick_rate)

    def tick(row):
        """Publish method - call this at ~1000Hz
        """

        # Prepare a new movement command
        msg = raven_automove()

        # Set header
        msg.hdr = std_msgs.msg.Header()
        msg.hdr.stamp = rospy.Time.now()

        # Set fields from CSV array
        msg.tf_incr[0].translation.x = traj[row, 0]
        msg.tf_incr[0].translation.y = traj[row, 1]
        msg.tf_incr[0].translation.z = traj[row, 2]
        msg.tf_incr[0].rotation.x = traj[row, 3]
        msg.tf_incr[0].rotation.y = traj[row, 4]
        msg.tf_incr[0].rotation.z = traj[row, 5]
        msg.tf_incr[0].rotation.w = traj[row, 6]

        msg.tf_incr[1].translation.x = traj[row, 7]
        msg.tf_incr[1].translation.y = traj[row, 8]
        msg.tf_incr[1].translation.z = traj[row, 9]
        msg.tf_incr[1].rotation.x = traj[row, 10]
        msg.tf_incr[1].rotation.y = traj[row, 11]
        msg.tf_incr[1].rotation.z = traj[row, 12]
        msg.tf_incr[1].rotation.w = traj[row, 13]
        
        # Publish message
        pub.publish(msg)

    r = rospy.Rate(tick_rate)
    array_row = 0
    while not rospy.is_shutdown():
        tick(array_row)

        r.sleep()

        # Update row index
        array_row += 1
        array_row = array_row % len(traj)


if __name__ == '__main__':
    main(sys.argv)
