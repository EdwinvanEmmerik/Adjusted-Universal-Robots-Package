#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <iostream>
#include <geometry_msgs/Pose.h>
#include <ur_kinematics/service_test.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <sensor_msgs/JointState.h>

class PAPServer : public moveit::planning_interface::MoveGroupInterface
{
/*
Class: PAPServer
Inherited: moveit::planning_interface::MoveGroupInterface
Description: This class is used to execute the pick-and-place service.
*/
public:
	PAPServer(const std::string group) : moveit::planning_interface::MoveGroupInterface(group){}; //Constructor
	bool PAPService(ur_kinematics::service_test::Request &req, ur_kinematics::service_test::Response &res);
};

bool PAPServer::PAPService(ur_kinematics::service_test::Request &req, ur_kinematics::service_test::Response &res)
{
/*
This function is used to assign the requested message to the pose-target of the movegroup. After this the
movegroup is moved to that pose. If this went successful the response will return 'true', ifnot the response
will return 'false'
*/
	ROS_INFO("You requested to move the UR5.");
	setPoseTarget(req.desired_pose);
	ROS_INFO("Pose target has been set.");
	if(move())
	{
		res.successful = true;
	}
	else
	{
		res.successful = false;
	}
	//std::cout << req.desired_pose << std::endl;
	ROS_INFO("Moving %s", (bool)res.successful? "went successful" : "failed");
	ROS_INFO("##################################");
	return true; // Used the check if the function got called
}
int main(int argc, char **argv)
{
	ros::init(argc, argv, "Pick_and_Place_server");
	ros::NodeHandle n;
	// Create callback queue to avoid different types of multi threading being combined
	ros::CallbackQueue callback_queue;
	n.setCallbackQueue(&callback_queue);
	callback_queue.callOne(ros::WallDuration());
	// Start spinners to allow multi-threading
	ros::AsyncSpinner spinner(0, &callback_queue);
  	spinner.start();

	// Create instance of PAPServer
  	PAPServer papserver("manipulator");
	// Set planner ID
	papserver.setPlannerId("RRTstarkConfigDefault");
	// Allow the move group to replan
	papserver.allowReplanning(true);
	papserver.setNumPlanningAttempts(20);
	// Set tolerances
	papserver.setGoalTolerance(0.001);
	papserver.setGoalOrientationTolerance(0.05);

	// load the robot model from the ROS parameter server
	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
	// Construct the planning scene
	//planning_scene::PlanningScene planning_scene(robot_model);
	// Get the planning frames
	ROS_INFO("Reference frame: %s", papserver.getPlanningFrame().c_str());
	ROS_INFO("Reference frame: %s", papserver.getEndEffectorLink().c_str());
	while(ros::ok())
	{
		ros::ServiceServer service = n.advertiseService("PAP_service", &PAPServer::PAPService, &papserver);
		ROS_INFO("Ready to accept a service");
		ros::spin();
	}
	return 0;
}