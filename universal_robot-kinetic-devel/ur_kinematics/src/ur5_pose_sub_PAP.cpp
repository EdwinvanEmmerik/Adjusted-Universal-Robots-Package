#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Pose.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

typedef const boost::function<void(const geometry_msgs::Pose2DConstPtr &)> callback2d;
typedef const boost::function<void(const geometry_msgs::PoseConstPtr &)> callback3d;

class PickandPlaceLocation
{
/*	Class PickandPlaceLocation
	Description:
		Used as object to construct different kind of tasks.
	members: goal_pose2d
			 goal_pose3d
*/
public:
	void desiredPose2DCallback(const geometry_msgs::Pose2DConstPtr &pose2d_msg);
	void desiredPose3DCallback(const geometry_msgs::PoseConstPtr &pose3d_msg);
	
	geometry_msgs::Pose2D goal_pose2d;
	geometry_msgs::Pose goal_pose3d;
};
bool pickAction(geometry_msgs::Pose pick_target_pose, moveit::planning_interface::MoveGroupInterface &move_group,
			   moveit::planning_interface::MoveGroupInterface::Plan &my_plan, planning_scene::PlanningScene &planning_scene)
{
/*	Function pickAction
	Description:
		This function is used to move the move_group to the set pick target pose.
	Return:
		bool action
*/
	ROS_INFO("YOU ENTERED THE PICK SEQUENCE");
	move_group.setPoseTarget(pick_target_pose);
	bool action = false;
	if (move_group.move())
	{
		action = true;
	}
	else
	{

	}
	return action;
}
bool placeAction(geometry_msgs::Pose place_target_pose, moveit::planning_interface::MoveGroupInterface &move_group,
			   moveit::planning_interface::MoveGroupInterface::Plan &my_plan, planning_scene::PlanningScene &planning_scene)
{
/*	Function placeAction
	Description:
		This function is used to move the move_group to the set place target pose.
	Return:
		bool action
*/
	ROS_INFO("YOU ENTERED THE PLACE SEQUENCE");
	move_group.setPoseTarget(place_target_pose);
	bool action = false;
	if (move_group.move())
	{
		action = true;
	}
	else
	{

	}
	return action;
}
void PickandPlaceLocation::desiredPose2DCallback(const geometry_msgs::Pose2DConstPtr &pose2d_msg)
{
	this -> goal_pose2d = *pose2d_msg;
}
void PickandPlaceLocation::desiredPose3DCallback(const geometry_msgs::PoseConstPtr &pose3d_msg)
{
	this -> goal_pose3d = *pose3d_msg;
}
void advertiseGripperStatus()
{

}
int main(int argc, char **argv)
{
	// Node initialization
	ros::init(argc, argv, "ur5_pose_subscriber");
	ros::NodeHandle n;
  	// Construct the PickandPlaceLocation instance
	PickandPlaceLocation task;
	// Construct the PickandPlaceLocation vector instance
	std::vector<PickandPlaceLocation> task_vect;
	//callback2d boundDesiredPose2DCallback = boost::bind(&PickandPlaceLocation::desiredPose2DCallback, &task, _1);
	
	// Moveit initialization
	ros::AsyncSpinner spinner(1);
  	spinner.start();
  	// Construct planning interface for the move_group
	const std::string PLANNING_GROUP = "manipulator";
	moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
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

	callback3d boundDesiredPose3DCallback = boost::bind(&PickandPlaceLocation::desiredPose3DCallback, &task, _1);
	ros::Rate looprate(1);
	int count = 1;
	while(ros::ok())
	{
	/*	OLD VERSION
		ros::Subscriber desired_pose2d_sub = n.subscribe("desired_pose", 10, boundDesiredPose2DCallback);
		if(desired_pose_sub)
		{
			task_vect.push_back(task);
		}
	*/
		ros::Subscriber desired_pose3d_sub = n.subscribe("desired_pose", 10, boundDesiredPose3DCallback);
		if(desired_pose3d_sub)
		{
			task_vect.push_back(task);
		}
	 	// Does work but not really well
		if(task_vect.size() >= 3)
		{
						std::cout<< task_vect[count].goal_pose3d << std::endl;
			if(pickAction(task_vect[count].goal_pose3d, move_group, my_plan, planning_scene))
			{
				count++;
				std::cout << task_vect[count].goal_pose3d << std::endl;
				if(placeAction(task_vect[count].goal_pose3d, move_group, my_plan, planning_scene))
				{
					count++;
				}
			}
		}

		looprate.sleep();
		ros::spinOnce();
	}

	for(int i = 1; i < 10; i++)
	{
		std::cout<< task_vect[i].goal_pose3d << std::endl;
	}
	
	task_vect.clear();
	return 0;
}