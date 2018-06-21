#include "ros/ros.h"

#include "raven_2/raven_automove.h"
#include "raven_2/raven_state.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "cpp_talker");

  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<raven_2::raven_automove>("/raven_automove", 1000);
  ros::Rate loop_rate(1000);

  int count = 0;
  while (ros::ok())
  {
    raven_2::raven_automove msg;

    ROS_INFO("Sending raven_automove");
    pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
