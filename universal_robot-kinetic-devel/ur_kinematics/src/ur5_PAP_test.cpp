#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/robot_state/robot_state.h>
//#include <moveit/planning_request_adapter/planning_request_adapter.h>
#include <moveit_msgs/PlanningScene.h>
#include <geometric_shapes/shape_operations.h>
class PickandPlaceLocation{
/*	Class PickandPlaceLocation
	Description:
		Used as object to construct different kind of tasks.
	members: x_pick,
			 y_pick,
			 z_pick,
			 x_place,
			 y_place,
			 z_place,
			 z_offset
*/
public:
		double x_pick;
		double y_pick;
		double z_pick;
		double x_place;
		double y_place;
		double z_place;
		double z_offset;
};
std::vector<geometry_msgs::Pose> calculatePAPWaypoints(geometry_msgs::Pose start_pose, PickandPlaceLocation task)
{
/*	Function calculateWaypoints
	Description:
		Calculates 6 pick and place waypoints depending on the value of the members of the task object.
	input:
		geometry_msgs::Pose start_pose
		PickandPlaceLocation task
	returns	waypoints, vector that shortly describes the pick and place waypoints
*/
	std::vector<geometry_msgs::Pose> waypoints;			// Construct waypoints vector of type geometry_msgs::Pose
	geometry_msgs::Pose target_pose = start_pose;		// Set target_pose to same type as start_pose for some reason this is needed
	// First movement
	target_pose.position.x = task.x_pick;
	target_pose.position.y = task.y_pick;
	target_pose.position.z = task.z_pick + task.z_offset;
	waypoints.push_back(target_pose);
	// Second movement
	target_pose.position.x = task.x_pick;
	target_pose.position.y = task.y_pick;
	target_pose.position.z = task.z_pick;
	waypoints.push_back(target_pose);
	// Third movement (for good measure)
	target_pose.position.x = task.x_pick;
	target_pose.position.y = task.y_pick;
	target_pose.position.z = task.z_pick + task.z_offset;
	waypoints.push_back(target_pose);
	// Fourth movement
	target_pose.position.x = task.x_place;
	target_pose.position.y = task.y_place;
	target_pose.position.z = task.z_place + task.z_offset;
	waypoints.push_back(target_pose);
	// Fifth movement
	target_pose.position.x = task.x_place;
	target_pose.position.y = task.y_place;
	target_pose.position.z = task.z_place;
	waypoints.push_back(target_pose);
	// Sixth movement (for good measure)
	target_pose.position.x = task.x_place;
	target_pose.position.y = task.y_place;
	target_pose.position.z = task.z_place + task.z_offset;
	
	waypoints.push_back(target_pose);
	waypoints.push_back(start_pose);
	return waypoints;
}
geometry_msgs::Pose setPAPOrientation()
{
/*	Function setPAPOrientation
	Description:
		Sets the robot its end-effector to a initialized position and to a pick and place orientation.
	returns PAP_pose
*/
	geometry_msgs::Pose PAP_pose;
	PAP_pose.position.x = 0.6;
  	PAP_pose.position.y = 0.05;
  	PAP_pose.position.z = 0.1;
	PAP_pose.orientation.w = 0.707;
	PAP_pose.orientation.x = 0;
	PAP_pose.orientation.y = 0.707;
	PAP_pose.orientation.z = 0;
	return PAP_pose;
}
moveit_msgs::RobotTrajectory addTimeParametrization(const std::string PLANNING_GROUP, moveit::planning_interface::MoveGroupInterface &move_group, moveit_msgs::RobotTrajectory trajectory)
{
/*	Function addTimeParametrization
	Description:
		Devide the trajectory based on waypoints in time steps. In this way the trajectory can be followed
		according to time steps.
	input:
		const std::string PLANNING_GROUP
		moveit::planning_interface::MoveGroupInterface &move_group
		moveit_msgs::RobotTrajectory trajectory
	returns trajectory
*/
	// Adding time parametrization to the trajectory
  	robot_trajectory::RobotTrajectory rt(move_group.getCurrentState()->getRobotModel(), "manipulator");
  	rt.setRobotTrajectoryMsg(*move_group.getCurrentState(), trajectory);
  	trajectory_processing::IterativeParabolicTimeParameterization iptp;
  	bool success = iptp.computeTimeStamps(rt);
  	ROS_INFO("Computed time stamp %s",success?"SUCCEDED":"FAILED");
  	rt.getRobotTrajectoryMsg(trajectory);
  	return trajectory;
}
int main(int argc, char **argv)
{
	ros::init(argc, argv, "Pick_and_Place_tester");
 	ros::NodeHandle node_handle;  
  	ros::AsyncSpinner spinner(1);
  	spinner.start();
  	// Construct planning scene for the move_group
	const std::string PLANNING_GROUP = "manipulator";
	moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
	moveit_msgs::PlanningScene planning_scene;
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
////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////
	// Get the planning frames
	ROS_INFO("Reference frame: %s", move_group.getPlanningFrame().c_str());
	ROS_INFO("Reference frame: %s", move_group.getEndEffectorLink().c_str());
  	//move_group.setPlannerId("PRMstarkConfigDefault"); //default planner is set to RTTCONNECT in ompl_planning_.yaml
  	move_group.setPlannerId("RTTCONNECT");
  	move_group.allowReplanning(true);
  	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  	
	move_group.setPoseTarget(setPAPOrientation());
	move_group.plan(my_plan);
	move_group.move();
	// First task
	PickandPlaceLocation task1; 
   	task1.x_pick = 0.2;
   	task1.y_pick = 0.5;
   	task1.z_pick = 0.1;
   	task1.x_place = 0.5;
   	task1.y_place = 0.2;
   	task1.z_place = 0.1;
   	task1.z_offset = 0.3;
   	
  	moveit_msgs::RobotTrajectory trajectory;
  	double fraction = move_group.computeCartesianPath(calculatePAPWaypoints(move_group.getCurrentPose().pose, task1), 0.001, 0.0, trajectory);
  	ROS_INFO("Visualizing plan (cartesian path) (%.2f%% acheived)", fraction * 100.0);
  	my_plan.trajectory_ = addTimeParametrization(PLANNING_GROUP, move_group, trajectory);

  	//std::vector<double> invalid;
  	//bool valid = my_plan.isPathValid(rt, PLANNING_GROUP, false, invalid);
  	//std::cout << move_group.getPathConstraints() << std::endl;

	if (fraction == 1.0) {
		sleep(2);
		//move_group.move(); // visualize plan
		ROS_INFO("EXECUTING PLAN");
		move_group.execute(my_plan);
		
		//if (planning_scene.isPathValid(move_group.getCurrentPose(), trajectory) != true)
		//{
		//	move_group.stop();
		//}
		

	}
	else
	{
		ROS_WARN("Could not compute the cartesian path");
	}
	//waypoints.clear();
  	ros::shutdown();  
  	return 0;
}