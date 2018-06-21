
#include <cmath>
#include "ros/ros.h"
#include "raven_2/raven_automove.h"
#include "raven_2/raven_state.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "cpp_talker");

    // Frequency to tick at
    // We also use the value for the length of the publisher queue
    int tick_hz = 1000;
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<raven_2::raven_automove>(
        "/raven_automove",
        tick_hz
    );

    // Record start time
    ros::Time start_time = ros::Time::now();

    ros::Rate loop_rate(tick_hz);
    while (ros::ok())
    {

        // Make a new automove message
        raven_2::raven_automove msg;
        msg.hdr.stamp = msg.hdr.stamp.now();

        // Find elapsed time in seconds
        double elapsed_time = (msg.hdr.stamp - start_time).toSec();
        
        // Command z velocity with 2*pi period (in seconds) sine wave
        double pos = sin(elapsed_time);
        msg.tf_incr[0].translation.z = pos;
        msg.tf_incr[1].translation.z = pos;

        // Make quaternions valid unit quaternions
        msg.tf_incr[0].rotation.w = 1.0;
        msg.tf_incr[1].rotation.w = 1.0;

        ROS_INFO("Sending raven_automove");
        pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }


    return 0;
}
