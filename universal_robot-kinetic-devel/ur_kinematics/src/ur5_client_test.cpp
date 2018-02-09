#include <ros/ros.h>
#include <iostream>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>
#include <ur_kinematics/service_test.h>
#include <ur_kinematics/vision_data2.h>

#include <cstdlib>
#include <sstream>

#define waypoint_steps 6	// Without home_pose


// Boost function that binds the obtained custom 'vision_data' message to a member in the callback function.
typedef const boost::function<void(const ur_kinematics::vision_data2::ConstPtr &)> callback;
// Boost function that binds the obtained gripper status message to a member in the callback function.
typedef const boost::function<void(const std_msgs::BoolConstPtr &)> gripperStatusCallback;

class PickandPlaceLocation
{
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Class PickandPlaceLocation
// Description:
// This class is used to construct a client that obtains the pose message over a certain topic and calling
// the pick-and-place service.
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
public:
	void desiredPoseCallback(const ur_kinematics::vision_data2::ConstPtr &vision_msg);
	std::vector<geometry_msgs::Pose> computeGoalPoseVector(geometry_msgs::Pose &pick_pose, geometry_msgs::Pose &place_pose);
	geometry_msgs::Pose pick_pose;
	geometry_msgs::Pose place_pose;

	std_msgs::String action;
	ros::NodeHandle n;
	ros::ServiceClient client;
	ur_kinematics::service_test srv;
private:
	float gripper_offset = 0.1; 
	geometry_msgs::Pose pose_holder;
	std::vector<geometry_msgs::Pose> waypoint_vect;
};
class GripperStatus : public PickandPlaceLocation
{
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Class: GripperStatus
// Inherited class: PickandPlaceLocation
// Description:
// This class is used to send and recieve commands from the gripper.
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
public:
	void gripperStatusCallback(const std_msgs::BoolConstPtr &real_gripper_status_msg);
	std_msgs::Bool real_gripper_open;
	std_msgs::String new_real_gripper_status;
	std_msgs::Float64 rviz_gripper_status;
	ros::Publisher real_gripper_status_pub = n.advertise<std_msgs::String>("changeGripperState",10);
	ros::Publisher rviz_gripper_status_pub = n.advertise<std_msgs::Float64>("gripper_controller/command",1);
	std::stringstream stringstream;
};
void GripperStatus::gripperStatusCallback(const std_msgs::BoolConstPtr &real_gripper_status_msg)
{
	real_gripper_open.data = real_gripper_status_msg->data;
}
std::vector<geometry_msgs::Pose> PickandPlaceLocation::computeGoalPoseVector(geometry_msgs::Pose &pick_pose, geometry_msgs::Pose &place_pose)
{
	// Define the wait pose here.
	geometry_msgs::Pose wait_pose;
	wait_pose.position.x = -0.5;
	wait_pose.position.y = -0.4;
	wait_pose.position.z = 0.1;
	wait_pose.orientation.w = 0.0;
	wait_pose.orientation.x = 0.0;
	wait_pose.orientation.y = 0.707;
	wait_pose.orientation.z = 0.707;
	// Divide the pick and place sequence in waypoints.
	for(int i = 0; i < waypoint_steps+1; i++)
	{
		if(i == 0 || i == 2)
		{
			pose_holder = pick_pose;
			pose_holder.position.z = pick_pose.position.z + gripper_offset;
			waypoint_vect.push_back(pose_holder);
		}
		else if(i == 1)
		{
			waypoint_vect.push_back(pick_pose);
		}
		else if(i == 3 || i == 5)
		{
			pose_holder = place_pose;
			pose_holder.position.z = place_pose.position.z + gripper_offset;
			waypoint_vect.push_back(pose_holder);
		}
		else if(i == 4)
		{
			waypoint_vect.push_back(place_pose);
		}
		else if(i == 6)
		{
			waypoint_vect.push_back(wait_pose);
		}
	}
	return waypoint_vect;
}
void PickandPlaceLocation::desiredPoseCallback(const ur_kinematics::vision_data2::ConstPtr &vision_msg)
{
	pick_pose = vision_msg->pick_pose;
	place_pose = vision_msg->place_pose;

	client = n.serviceClient<ur_kinematics::service_test>("PAP_service");

	waypoint_vect = computeGoalPoseVector(pick_pose, place_pose);
	GripperStatus gripperStatus;
	bool gripper_param;
	// If the pick_pose of the vision_msg contains x=0, y=0 and z=0 the request will be to go to the wait_pose.
	// Using this the vision system can use this as default for situations where no block can be detected.
	if((pick_pose.position.x == 0) && (pick_pose.position.y == 0) && (pick_pose.position.z ==0))
	{
		srv.request.desired_pose = waypoint_vect[6];
		// Here the client calls the service
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
	else
	{
		for(int i = 0; i < waypoint_steps+1;)
		{
			GripperStatus gripperStatus;
			gripperStatusCallback boundGripperStatusCallback = boost::bind(&GripperStatus::gripperStatusCallback, &gripperStatus, _1);
			ros::param::get("bool_gripperOpen_param", gripper_param);
			if(i == 1)// && gripper_param == false)
			{
				gripperStatus.rviz_gripper_status.data = 0.3;
				gripperStatus.rviz_gripper_status_pub.publish(gripperStatus.rviz_gripper_status);
				gripperStatus.new_real_gripper_status.data = "Open";
				gripperStatus.real_gripper_status_pub.publish(gripperStatus.new_real_gripper_status);
			}
			else if(i == 2)// && gripper_param == true)
			{
				gripperStatus.rviz_gripper_status.data = 0.0;
				gripperStatus.rviz_gripper_status_pub.publish(gripperStatus.rviz_gripper_status);
				gripperStatus.new_real_gripper_status.data = "Close";
				gripperStatus.real_gripper_status_pub.publish(gripperStatus.new_real_gripper_status);
			}
			else if(i == 4)// && gripper_param == true)
			{
				gripperStatus.rviz_gripper_status.data = 0.0;
				gripperStatus.rviz_gripper_status_pub.publish(gripperStatus.rviz_gripper_status);
				gripperStatus.new_real_gripper_status.data = "Close";
				gripperStatus.real_gripper_status_pub.publish(gripperStatus.new_real_gripper_status);
			}
			else if(i == 5)// && gripper_param == false)
			{
				gripperStatus.rviz_gripper_status.data = 0.3;
				gripperStatus.rviz_gripper_status_pub.publish(gripperStatus.rviz_gripper_status);	
				gripperStatus.new_real_gripper_status.data = "Open";
				gripperStatus.real_gripper_status_pub.publish(gripperStatus.new_real_gripper_status);
			}
			std::cout << waypoint_vect[i] << std::endl;
			srv.request.desired_pose = waypoint_vect[i];
			// Here the client calls the service
			if(client.call(srv) && srv.response.successful)
			{
				ROS_INFO("Service delivered: %s", (bool)srv.response.successful? "true" : "false");
				i++;
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
	}
}
int main(int argc, char **argv)
{
	ros::init(argc, argv, "ur5_pose_subscriber_and_service");
	const std::string pose_topic = "desired_pose";
	PickandPlaceLocation task;
	callback boundDesiredPoseCallback = boost::bind(&PickandPlaceLocation::desiredPoseCallback, &task, _1);
	ROS_INFO("Pick and place client has been started.");
	ROS_INFO("Subscribed to the topic %s", (const char*)pose_topic.c_str());
	while(ros::ok())
	{
		ros::Subscriber desired_pose_sub = task.n.subscribe(pose_topic, 1, boundDesiredPoseCallback);
		ros::spin();
	}
	return 0;
}
/*
#include <ros/ros.h>
#include <iostream>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>
#include <ur_kinematics/service_test.h>
#include <ur_kinematics/vision_data.h>
#include <cstdlib>
#include <sstream>

#define goal_pose_steps 3	// Without home_pose
//#define goal_pose_steps 4	// With home_pose

// Boost function that binds the obtained custom 'vision_data' message to a member in the callback function.
typedef const boost::function<void(const ur_kinematics::vision_data::ConstPtr &)> callback;
// Boost function that binds the obtained gripper status message to a member in the callback function.
typedef const boost::function<void(const std_msgs::BoolConstPtr &)> gripperStatusCallback;

class PickandPlaceLocation
{
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Class PickandPlaceLocation
// Description:
// This class is used to construct a client that obtains the pose message over a certain topic and calling
// the pick-and-place service.
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
public:
	void desiredPoseCallback(const ur_kinematics::vision_data::ConstPtr &vision_msg);
	std::vector<geometry_msgs::Pose> computeGoalPoseVector(geometry_msgs::Pose &goal_pose);
	geometry_msgs::Pose goal_pose;
	std_msgs::String action;
	ros::NodeHandle n;
	ros::ServiceClient client;
	ur_kinematics::service_test srv;
private:
	float gripper_offset = 0.1; 
	geometry_msgs::Pose pose_holder;
	std::vector<geometry_msgs::Pose> goal_pose_vect;
};
class GripperStatus : public PickandPlaceLocation
{
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Class: GripperStatus
// Inherited class: PickandPlaceLocation
// Description:
// This class is used to send and recieve commands from the gripper.
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
public:
	void gripperStatusCallback(const std_msgs::BoolConstPtr &real_gripper_status_msg);
	std_msgs::Bool real_gripper_open;
	std_msgs::String new_real_gripper_status;
	std_msgs::Float64 rviz_gripper_status;
	ros::Publisher real_gripper_status_pub = n.advertise<std_msgs::String>("changeGripperState",10);
	ros::Publisher rviz_gripper_status_pub = n.advertise<std_msgs::Float64>("gripper_controller/command",1);
	std::stringstream stringstream;
};
void GripperStatus::gripperStatusCallback(const std_msgs::BoolConstPtr &real_gripper_status_msg)
{
	real_gripper_open.data = real_gripper_status_msg->data;
}
std::vector<geometry_msgs::Pose> PickandPlaceLocation::computeGoalPoseVector(geometry_msgs::Pose &goal_pose)
{
	// Define the home pose here.
	geometry_msgs::Pose home_pose;
	home_pose.position.x = -0.5;
	home_pose.position.y = -0.4;
	home_pose.position.z = 0.1;
	home_pose.orientation.w = 0.707;
	home_pose.orientation.x = 0.707;
	home_pose.orientation.y = 0.0;
	home_pose.orientation.z = 0.0;
	for(int i = 0; i < goal_pose_steps; i++)
	{
		if(i == 0 || i == 2)
		{
			pose_holder = goal_pose;
			pose_holder.position.z = goal_pose.position.z + gripper_offset;
			goal_pose_vect.push_back(pose_holder);
			//std::cout << "Pose " << (i+1) << ": " << pose_holder << std::endl;
		}
		else if(i == 1)
		{
			goal_pose_vect.push_back(goal_pose);
			//std::cout << "Pose 2: " << goal_pose << std::endl;
		}
		//else if(i == 3)
		//{
		//	goal_pose_vect.push_back(home_pose);
		//}
	}
	return goal_pose_vect;
}
void PickandPlaceLocation::desiredPoseCallback(const ur_kinematics::vision_data::ConstPtr &vision_msg)
{
	goal_pose = vision_msg->desired_pose;
	action = vision_msg->pick_or_place;

	client = n.serviceClient<ur_kinematics::service_test>("PAP_service");
	goal_pose_vect = computeGoalPoseVector(goal_pose);

	GripperStatus gripperStatus;
	//gripperStatusCallback boundGripperStatusCallback = boost::bind(&GripperStatus::gripperStatusCallback, &gripperStatus, _1);

	bool gripper_param;
	std::cout << "pick or place: " << action.data << std::endl;

	for(int i = 0; i < goal_pose_steps;)
	{
		GripperStatus gripperStatus;
		gripperStatusCallback boundGripperStatusCallback = boost::bind(&GripperStatus::gripperStatusCallback, &gripperStatus, _1);
		//ros::Subscriber gripper_status_sub = n.subscribe("gripperOpen", 1000, boundGripperStatusCallback);
		ros::param::get("bool_gripperOpen_param", gripper_param);
		std::cout << "Gripper param: " << gripper_param << std::endl;
		if(action.data == "pick")
		{
			if(i == 1)// && gripper_param == false)
			{
				gripperStatus.rviz_gripper_status.data = 0.5;
				gripperStatus.rviz_gripper_status_pub.publish(gripperStatus.rviz_gripper_status);
				gripperStatus.new_real_gripper_status.data = "Open";
				gripperStatus.real_gripper_status_pub.publish(gripperStatus.new_real_gripper_status);
			}
			else if(i == 2)// && gripper_param == true)
			{
				gripperStatus.rviz_gripper_status.data = 0.0;
				gripperStatus.rviz_gripper_status_pub.publish(gripperStatus.rviz_gripper_status);
				gripperStatus.new_real_gripper_status.data = "Close";
				gripperStatus.real_gripper_status_pub.publish(gripperStatus.new_real_gripper_status);
			}
		}
		else if(action.data == "place")
		{
			if(i == 1)// && gripper_param == true)
			{
				gripperStatus.rviz_gripper_status.data = 0.0;
				gripperStatus.rviz_gripper_status_pub.publish(gripperStatus.rviz_gripper_status);
				gripperStatus.new_real_gripper_status.data = "Close";
				gripperStatus.real_gripper_status_pub.publish(gripperStatus.new_real_gripper_status);
			}
			else if(i == 2)// && gripper_param == false)
			{
				gripperStatus.rviz_gripper_status.data = 0.5;
				gripperStatus.rviz_gripper_status_pub.publish(gripperStatus.rviz_gripper_status);	
				gripperStatus.new_real_gripper_status.data = "Open";
				gripperStatus.real_gripper_status_pub.publish(gripperStatus.new_real_gripper_status);
			}
		}

		//std::cout << goal_pose_vect[i] << std::endl;
		srv.request.desired_pose = goal_pose_vect[i];
		// Here the client calls the service
		if(client.call(srv) && srv.response.successful)
		{
			ROS_INFO("Service delivered: %s", (bool)srv.response.successful? "true" : "false");
			i++;
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
}
int main(int argc, char **argv)
{
	ros::init(argc, argv, "ur5_pose_subscriber_and_service");
	const std::string pose_topic = "desired_pose";
	PickandPlaceLocation task;
	callback boundDesiredPoseCallback = boost::bind(&PickandPlaceLocation::desiredPoseCallback, &task, _1);
	ROS_INFO("Pick and place client has been started.");
	ROS_INFO("Subscribed to the topic %s", (const char*)pose_topic.c_str());
	while(ros::ok())
	{
		ros::Subscriber desired_pose_sub = task.n.subscribe(pose_topic, 1000, boundDesiredPoseCallback);

		ros::spin();
	}
	return 0;
}
*/