#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
void chatterCallback(const geometry_msgs::Twist& msg)
{
  ROS_INFO_STREAM("Subscribed velocity Vx:"<<msg.linear.x<<" Vy:"<<msg.linear.y<<" angular:"<<msg.angular.z);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "base_controller");
  ros::NodeHandle n;

  // Subscribe "cmd_vel"
  ros::Subscriber sub = n.subscribe("cmd_vel", 1000, chatterCallback);

  ros::spin();

  return 0;
}
