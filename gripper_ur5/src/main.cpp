#include "ros/ros.h"
#include "std_msgs/Bool.h"
bool status = false;
bool status2 = false;

void gripperOpenCallback(const std_msgs::Bool::ConstPtr& msg)
{
  status = msg->data;
  ros::param::set("bool_gripperOpen_param", status);
  //ROS_INFO("I heard: [%i]", status);
  //if (ros::param::get("/bool_gripperOpen_param", status2))
  //{
  //ROS_INFO("I heard: [%i]", status2);
  //}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("gripperOpen", 1000, gripperOpenCallback);
  ros::spin();
  return 0;
}
