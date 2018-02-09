#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <geometric_shapes/shape_operations.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/ExecuteKnownTrajectory.h>
#include <moveit/planning_request_adapter/planning_request_adapter.h>
int main(int argc, char **argv)
{
	ros::init(argc, argv, "trajectory_tester");
 	ros::NodeHandle node_handle;  
  	ros::AsyncSpinner spinner(1);
  	spinner.start();
  	// Construct planning scene for the move_group
	const std::string PLANNING_GROUP = "manipulator";
	moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
	move_group.setWorkspace(-0.2, -0.2, 0.05, 1, 1, 1);	// set workspace size for move_group
	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
	// Make sure that the robot model gets loaded
	if (robot_model != 0)
	{
		ROS_INFO("Robot model loaded");
	}
	else
	{
		ROS_INFO("Could not load robot model");
	}

	// Get the planning frames
	ROS_INFO("Reference frame: %s", move_group.getPlanningFrame().c_str());
	ROS_INFO("Reference frame: %s", move_group.getEndEffectorLink().c_str());
  	move_group.setPlannerId("PRMstarkConfigDefault"); //default planner is set to RTTCONNECT in ompl_planning_.yaml
  	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  	/*
  	int chosenone;
  	std::cout << "You can choose between pick and place(1) and sinusoidal movements(2)" << std::endl;
  	std::cin >> chosenone;
  	std::cout << std::endl;
  	*/
  	// Construct target_pose object of type geometry_msgs
	// Construct waypoints vector of type geometry_msgs::Pose
	std::vector<geometry_msgs::Pose> waypoints;
	geometry_msgs::Pose start_pose = move_group.getCurrentPose().pose;
	sleep(1);
	geometry_msgs::Pose target_pose = start_pose;

  	// Keep initiazlize state and also keep the UR5 facing downwards for default
  	target_pose.position.x = 0.6;
  	target_pose.position.y = 0.05;
  	target_pose.position.z = 0.1;
	target_pose.orientation.w = 0.707;
	target_pose.orientation.x = 0;
	target_pose.orientation.y = 0.707;
	target_pose.orientation.z = 0;
	move_group.setPoseTarget(target_pose);
	move_group.plan(my_plan);
	move_group.move();
			
/* THIS HAS BEEN REPLACING BY ur5_PAP_test.cpp
	int number_of_waypoints = 7;
	double x[number_of_waypoints];
	x[0] = 0.5;
	x[1] = 0.0;
	x[2] = 0.0;
	x[3] = 0.0;
	x[4] = 0.5;
	x[5] = 0.5;
	x[6] = 0.5;
	double y[number_of_waypoints];
	y[0] = 0.0;
	y[1] = 0.5;
	y[2] = 0.5;
	y[3] = 0.5;
	y[4] = 0.0;
	y[5] = 0.0;
	y[6] = 0.0;
	double z[number_of_waypoints];
	z[0] = 0.4;
	z[1] = 0.4;
	z[2] = 0.1;
	z[3] = 0.4;
	z[4] = 0.4;
	z[5] = 0.1;
	z[6] = 0.4;
	for (int i = 0; i < number_of_waypoints; i++)
	{
		target_pose.position.x = x[i];
		target_pose.position.y = y[i];
		target_pose.position.z = z[i];
		waypoints.push_back(target_pose);
	}
*/  
	// Creating the sine data in x and y coordinates
	double amplitude = 0.1;
	double w = 30.0;
	double step = 0.005;
	double upperLimit_x = -0.5;
	double lowerLimit_x = -0.1;
	double offset = 0.20;
	int size = round( (upperLimit_x - lowerLimit_x)/step) + 1;
	double x[size];
	double y[size];
	x[0] = lowerLimit_x;
	for(int i = 1; i < size; i++)
	{
		x[i] = x[i-1] + step;
	}
	for(int i = 0; i < size; i++)
	{
		y[i] = amplitude*sin((x[i]+offset)*w)+0.4;
	}
	// Inserting the sine data to waypoints
	for(int i = 0; i < size; i++)
	{
		target_pose.position.x = x[i];
		target_pose.position.y = y[i];
		waypoints.push_back(target_pose);
	}
	// The same sine data but backwards
	for(int i = size-2; i != -1; i--)
	{
		target_pose.position.x = x[i];
		target_pose.position.y = -y[i]+0.8;	// and inverted
		waypoints.push_back(target_pose);
	}
	// Creating sine data in x and z coordinates TODO

	// After whatever case is chosen return to its starting position
   	waypoints.push_back(start_pose);
  	moveit_msgs::RobotTrajectory trajectory;
  	double fraction = move_group.computeCartesianPath(waypoints, 0.001, 0.0, trajectory);
  	ROS_INFO("Visualizing plan (cartesian path) (%.2f%% acheived)", fraction * 100.0);    

	// Adding time parametrization to the trajectory
  	robot_trajectory::RobotTrajectory rt(move_group.getCurrentState()->getRobotModel(), "manipulator");
  	rt.setRobotTrajectoryMsg(*move_group.getCurrentState(), trajectory);
  	trajectory_processing::IterativeParabolicTimeParameterization iptp;
  	bool success = iptp.computeTimeStamps(rt);
  	ROS_INFO("Computed time stamp %s",success?"SUCCEDED":"FAILED");
  	rt.getRobotTrajectoryMsg(trajectory);
  	//std::vector<double> invalid;
  	//bool valid = my_plan.isPathValid(rt, PLANNING_GROUP, false, invalid);
  	//move_group.setPathConstraints();
  	//std::cout << move_group.getPathConstraints() << std::endl;
  	sleep(5);
  	//planning_request_adapter::PlanningRequestAdapter::FixWorkspaceBound();
  	my_plan.trajectory_ = trajectory;
	if (fraction == 1.0) {
		sleep(2);
		move_group.execute(my_plan);

	}
	else
	{
		ROS_WARN("Could not compute the cartesian path :( ");
		std::cout << fraction << std::endl;
	}
	// Debug waypoints
	/*
	for(int k = 0; k < waypoints.size(); k++)
	{
			std::cout << waypoints[k] << std::endl;
	}
	*/
	waypoints.clear();
  	ros::shutdown();  
  	return 0;
}