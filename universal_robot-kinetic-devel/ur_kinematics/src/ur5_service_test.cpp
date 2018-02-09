#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/Pose.h>
#include <ur_kinematics/service_test.h>
#include <cstdlib>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

bool PAPService(ur_kinematics::service_test::Request &req, ur_kinematics::service_test::Response &res)
{
	res.successful = false;
	//std::cout << req.desired_pose << std::endl;
	ROS_INFO("response is: %s", (bool)res.successful? "true" : "false");
	sleep(2);
	return true;
}
int main(int argc, char **argv)
{
	ros::init(argc, argv, "Pick_and_Place_server");
	ros::NodeHandle n;

	ros::ServiceServer service = n.advertiseService("PAP_service", PAPService);
	ROS_INFO("Ready to accept a service");

	ros::spin();
	return 0;
}