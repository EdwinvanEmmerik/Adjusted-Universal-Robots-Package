#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
class PickandPlaceLocation
{
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
	void defineTask(double input_x_pick, double input_y_pick, double input_z_pick, double input_x_place, double input_y_place, double input_z_place);
		double x_pick;
		double y_pick;
		double z_pick;
		double x_place;
		double y_place;
		double z_place;
		double z_offset;
};
void PickandPlaceLocation::defineTask(double input_x_pick, double input_y_pick, double input_z_pick, double input_x_place, double input_y_place, double input_z_place)
{
/* 	Function PickandPlaceLocation
	Description:
		Used to define the members of an instance made from the class PickandPlaceLocation

*/
	x_pick = input_x_pick;
	y_pick = input_y_pick;
	z_pick = input_z_pick;
	x_place = input_x_place;
	y_place = input_y_place;
	z_place = input_z_place;
}
bool pickBlock(geometry_msgs::Pose pick_target_pose, moveit::planning_interface::MoveGroupInterface &move_group,
			   moveit::planning_interface::MoveGroupInterface::Plan &my_plan, planning_scene::PlanningScene &planning_scene)
{
/*	Function pickBlock
	Description:
	This function tries to pick up the block for the given members of a task (created as instance of class PickandPlaceLocation).
	TODO:
	added the planning_scene.isStateCollding but this doesn't work.
	I have to add the planning scene monitor somewhere.
*/
	bool successful_pick = false;
	ROS_INFO("YOU ENTERED THE PICK SEQUENCE");
	move_group.setPoseTarget(pick_target_pose);
	if(move_group.plan(my_plan))
	{
		//if(bool ps = planning_scene.isStateColliding("manipulator"))
		//{
		//	std::cout << "collide?: " << ps << std::endl;
		//	move_group.stop();
		//}
		//else if(!planning_scene.isStateColliding("manipulator"))
		//{
			sleep(2);	// Give moveit some time to visualize the trajectory in rviz
			ROS_INFO("Succesfully planned the pick trajectory");
			if(move_group.execute(my_plan))
			{
				ROS_INFO("Sucessfully moved to pick location");
				successful_pick = true;
				sleep(2);
			}
			else
			{
				ROS_INFO("Failed to move to pick location");
			}
		//}
	}
	else
	{
		ROS_INFO("Failed to plan the pick trajectory");
	}
	return successful_pick;
}
bool placeBlock(geometry_msgs::Pose place_target_pose, moveit::planning_interface::MoveGroupInterface &move_group,
			   moveit::planning_interface::MoveGroupInterface::Plan &my_plan, planning_scene::PlanningScene &planning_scene)
{
/*	Function placeBlock
	Description:
	This function tries to place a block for the given mebers of a task (created as instance of class PickandPlaceLocation).
	TODO:
	added the planning_scene.isStateCollding but this doesn't work.
	I have to add the planning scene monitor somewhere.
*/
	bool successful_place = false;
	ROS_INFO("YOU ENTERED THE PLACE SEQUENCE");
	move_group.setPoseTarget(place_target_pose);
	if(move_group.plan(my_plan)) //&& !planning_scene.isStateColliding("manipulator"))
	{
		sleep(2);	// Give moveit some time to visualize the trajectory in rviz
		ROS_INFO("Succesfully planned the place trajectory");
		if(move_group.execute(my_plan))
		{
			ROS_INFO("Sucessfully moved to place location");
			successful_place = true;
			sleep(2);
		}
		else
		{
			ROS_INFO("Failed to move to place location");
		}
	}
	else
	{
		ROS_INFO("Failed to plan the place trajectory");
	}
	return successful_place;
}
geometry_msgs::Pose defineTargetPose(PickandPlaceLocation task, std::string pick_or_place)
{
/*	Function defineTargetPose
	Description:
	This function defines the task instance into the desired target pose. The orientation in quaternions is set
	downward.
*/
	geometry_msgs::Pose target_pose;
	if(pick_or_place == "pick")
	{
		target_pose.orientation.x = 0.707;
		target_pose.orientation.y = 0.0;
		target_pose.orientation.z = 0.0;
		target_pose.orientation.w = 0.707;
		target_pose.position.x = task.x_pick;
		target_pose.position.y = task.y_pick;
		target_pose.position.z = task.z_pick;
	}
	else if(pick_or_place == "place")
	{
		target_pose.orientation.x = 0.5;
		target_pose.orientation.y = -0.5;
		target_pose.orientation.z = -0.5;
		target_pose.orientation.w = 0.5;
		target_pose.position.x = task.x_place;
		target_pose.position.y = task.y_place;
		target_pose.position.z = task.z_place;
	}
	return target_pose;

}
bool pickAction(geometry_msgs::Pose pick_target_pose, moveit::planning_interface::MoveGroupInterface &move_group,
			   moveit::planning_interface::MoveGroupInterface::Plan &my_plan, planning_scene::PlanningScene &planning_scene)
{
	ROS_INFO("YOU ENTERED THE PICK SEQUENCE");
	move_group.setPoseTarget(pick_target_pose);
	bool action = false;
	if (move_group.move())
	{
		action = true;
	}
	else
	{
		//move_group.stop();
	}
	return action;
}
bool placeAction(geometry_msgs::Pose place_target_pose, moveit::planning_interface::MoveGroupInterface &move_group,
			   moveit::planning_interface::MoveGroupInterface::Plan &my_plan, planning_scene::PlanningScene &planning_scene)
{
	ROS_INFO("YOU ENTERED THE PLACE SEQUENCE");
	move_group.setPoseTarget(place_target_pose);
	bool action = false;
	if (move_group.move())
	{
		action = true;
	}
	else
	{
		//move_group.stop();
	}
	return action;
}			   
int main(int argc, char** argv)
{
	ros::init(argc, argv, "Pick_and_Placing");
	ros::NodeHandle nodehandle;

	ros::AsyncSpinner spinner(1);
  	spinner.start();

  	// Construct planning interface for the move_group
	const std::string PLANNING_GROUP = "manipulator";
	moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

	// Set planner id
	//move_group.setPlannerId("PRMstarkConfigDefault"); 	//move_group.setPlannerId("RRTConnectkConfigDefault");
	//move_group.setPlannerId("BKPIECEkConfigDefault");
	move_group.setPlannerId("RRTstarkConfigDefault");
	// Allow the move group to replan
	move_group.allowReplanning(true);
	move_group.setNumPlanningAttempts(10);
	// Set tolerances
	move_group.setGoalTolerance(0.01);
	move_group.setGoalOrientationTolerance(0.05);

	// load the robot model from the ROS parameter server
	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
	// Construct the planning scene
	planning_scene::PlanningScene planning_scene(robot_model);
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
	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	// Create pick and place location object called task1
	PickandPlaceLocation task1;
	// Define pick and place location of object task1
	task1.defineTask(0.6, 0.0, 0.0, 0.0, 0.6, 0.0);

	// Define pick target pose object configured in the object task1
	geometry_msgs::Pose pick_target_pose = defineTargetPose(task1, "pick");
	// Define place target pose object configured in the object task1
	geometry_msgs::Pose place_target_pose = defineTargetPose(task1, "place");
	planning_scene_monitor::PlanningSceneMonitor planning_scene_monitor();
	//planning_scene_monitor.getPlanningScene();
	while(ros::ok())
	{/*
		if(pickBlock(pick_target_pose, move_group, my_plan, planning_scene))
		{	
		// UNFORTUNATEY THIS CANNOT BE REALIZED BECAUSE NO INTERUPTION IS ALLOWED BY THE USER
		//	while(!placeBlock(place_target_pose, move_group, my_plan)){}
		//
			// This the slow solution for this problem because when the placing fails the program
			// will run the picking first, and this will slow the process down by 5 seconds.
			placeBlock(place_target_pose, move_group, my_plan, planning_scene);
		}
		else
		{
			ROS_INFO("Could not place block because the picking failed");
		}*/
		if(pickAction(pick_target_pose, move_group, my_plan, planning_scene))
		{
			placeAction(place_target_pose, move_group, my_plan, planning_scene);
		}

	}
	

	ros::spinOnce();
	return 0;
}