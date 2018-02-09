#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/Pose.h>
#include <ur_kinematics/service_test.h>
#include <ur_kinematics/vision_data.h>
#include <cstdlib>

// Boost function that binds the obtained pose message to a member in the callback function
typedef const boost::function<void(const ur_kinematics::vision_data::ConstPtr &)> callback;

class PickandPlaceLocation
{
/*
Class PickandPlaceLocation
Description:
This class is used to construct a client that obtains the pose message over a certain topic and calling
the pick-and-place service.
*/
public:
	void desiredPoseCallback(const ur_kinematics::vision_data::ConstPtr &vision_data);
	geometry_msgs::Pose goal_pose;
	std_msgs::String action;
	ros::NodeHandle n;
	ros::ServiceClient client;
	ur_kinematics::service_test srv;
};
void PickandPlaceLocation::desiredPoseCallback(const ur_kinematics::vision_data::ConstPtr &vision_data)
{
/*
This function assigns the obtained pose to the 'goal_pose' member of its instance. After that it constructs
the client and sets the goal_pose to the service request. When this initialization is finished the client will
call the service.
*/
	goal_pose = vision_data->desired_pose;
	action = vision_data->pick_or_place;
	client = n.serviceClient<ur_kinematics::service_test>("PAP_service");
	srv.request.desired_pose = goal_pose;
	srv.request.pick_or_place = action;
	//std::cout<< goal_pose << std::endl;
	if(client.call(srv) && srv.response.successful)
	{
		ROS_INFO("Service delivered: %s", (bool)srv.response.successful? "true" : "false");
	}
	else if(client.call(srv) && !srv.response.successful)
	{
		ROS_WARN("Service called but execution failed..");
	}
	else
	{
		ROS_ERROR("Failed to call service");
	}
}
int main(int argc, char **argv)
{
	ros::init(argc, argv, "ur5_pose_subscriber_and_service");
	const std::string topic = "desired_pose";
	PickandPlaceLocation task;
	// Initialize callback function
	callback boundDesiredPoseCallback = boost::bind(&PickandPlaceLocation::desiredPoseCallback, &task, _1);
	
	ROS_INFO("Pick and place client has been started.");
	ROS_INFO("Subscribed to the topic %s", (const char*)topic.c_str());
	
	while(ros::ok())
	{
		ros::Subscriber desired_pose_sub = task.n.subscribe(topic, 1, boundDesiredPoseCallback);
		ros::spin();
	}
	return 0;
}
/*
#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/Pose.h>
#include <ur_kinematics/service_test.h>
#include <ur_kinematics/vision_data.h>
#include <cstdlib>

// Boost function that binds the obtained pose message to a member in the callback function
typedef const boost::function<void(const geometry_msgs::PoseConstPtr &)> callback;

class PickandPlaceLocation
{

//Class PickandPlaceLocation
//Description:
//This class is used to construct a client that obtains the pose message over a certain topic and calling
//the pick-and-place service.

public:
	void desiredPoseCallback(const geometry_msgs::PoseConstPtr &pose_msg);
	geometry_msgs::Pose goal_pose;
	ros::NodeHandle n;
	ros::ServiceClient client;
	ur_kinematics::service_test srv;
	ur_kinematics::vision_data msg;
};
void PickandPlaceLocation::desiredPoseCallback(const geometry_msgs::PoseConstPtr &pose_msg)
{

//This function assigns the obtained pose to the 'goal_pose' member of its instance. After that it constructs
//the client and sets the goal_pose to the service request. When this initialization is finished the client will
//call the service.

	this -> goal_pose = *pose_msg;
	client = n.serviceClient<ur_kinematics::service_test>("PAP_service");
	srv.request.desired_pose = goal_pose;
	//std::cout<< goal_pose << std::endl;
	if(client.call(srv) && srv.response.successful)
	{
		ROS_INFO("Service delivered: %s", (bool)srv.response.successful? "true" : "false");
	}
	else if(client.call(srv) && !srv.response.successful)
	{
		ROS_WARN("Service called but execution failed..");
	}
	else
	{
		ROS_ERROR("Failed to call service");
	}
}
int main(int argc, char **argv)
{
	ros::init(argc, argv, "ur5_pose_subscriber_and_service");
	const std::string topic = "desired_pose";
	PickandPlaceLocation task;
	// Initialize callback function
	callback boundDesiredPoseCallback = boost::bind(&PickandPlaceLocation::desiredPoseCallback, &task, _1);
	
	ROS_INFO("Pick and place client has been started.");
	ROS_INFO("Subscribed to the topic %s", (const char*)topic.c_str());
	
	while(ros::ok())
	{
		ros::Subscriber desired_pose_sub = task.n.subscribe(topic, 1, boundDesiredPoseCallback);
		ros::spin();
	}
	return 0;
}
*/