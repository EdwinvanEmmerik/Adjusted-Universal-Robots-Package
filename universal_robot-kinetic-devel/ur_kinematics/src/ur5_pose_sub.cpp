#include <ros/ros.h>
#include <std_msgs/Float64.h>

void chatterCallback(const std_msgs::Float64::ConstPtr& msg)
{
	ROS_INFO("I heard: [%f]", msg->data);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ur_pose_subscriber");
	ros::NodeHandle n;

	while(ros::ok())
	{
		ros::Subscriber sub = n.subscribe("desired_pose", 1000, chatterCallback);
		ros::spin();
	}
	return 0;
}