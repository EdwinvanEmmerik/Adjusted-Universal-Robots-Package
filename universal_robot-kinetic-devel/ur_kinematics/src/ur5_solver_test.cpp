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
#define number_of_inputs 7
int main(int argc, char** argv)
{
	ros::init(argc,argv,"move_group_pick");	// initialze ros node
	ros::NodeHandle n("~");					// handle the node..
	ros::AsyncSpinner spinner(1);			// initialize multi-thread spinning
	spinner.start();						// start the spinner

	const std::string PLANNING_GROUP = "manipulator"; // name of move_group initialized in ur5_moveit_config/ur5.srdf
	moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP); 	// construct a planning scene that maintains the state of the world
	move_group.setPlannerId("RRTConnectkConfigDefault"); //default planner is set to RTTCONNECT in ompl_planning_.yaml
	move_group.allowReplanning(true);

	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
	// Make sure that the robot_model actually gets loaded
	if (robot_model != 0)
	{
		ROS_INFO("Robot model loaded");
	}
	else
	{
		ROS_INFO("Could not load robot model");
	}


	// Setting up the planning pipeline
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	ROS_INFO("Reference frame: %s", move_group.getPlanningFrame().c_str());
	ROS_INFO("Reference frame: %s", move_group.getEndEffectorLink().c_str());

	move_group.setGoalPositionTolerance(0.01); 		// set position tolerance
	move_group.setGoalOrientationTolerance(0.05);	// set orientation tolerance

	/* ////////////////////////////////////////////////////////// STILL WORKING ON IT
	// Define collision object ROS message and name it
	moveit_msgs::CollisionObject collision_object;
	collision_object.header.frame_id = move_group.getPlanningFrame(); //set frame to base frame
	collision_object.id = "box1";
	shapes::Mesh* m  = shapes::createMeshFromResource("package://ur_description/meshes/ur5/collision/box1.stl");
	shape_msgs::Mesh mesh;
	shapes::ShapeMsg mesh_msg;
	shapes::constructMsgFromShape(m, mesh_msg);
	mesh = boost::get<shape_msgs::Mesh>(mesh_msg);
	// Define a box to add to the world 
	collision_object.meshes.resize(0.001);
	//collision_object.mesh_poses.resize(0.001);
	collision_object.meshes[0] = mesh;
	collision_object.mesh_poses[0].position.x = -0.5;
	collision_object.mesh_poses[0].position.y = -0.5;
	collision_object.mesh_poses[0].position.z = -0.1;
	collision_object.mesh_poses[0].orientation.x = 0.707;
	collision_object.mesh_poses[0].orientation.w = 0.707;

	collision_object.meshes.push_back(mesh);
	collision_object.mesh_poses.push_back(collision_object.mesh_poses[0]);
	collision_object.ADD;

	std::vector<moveit_msgs::CollisionObject> collision_vector;
	collision_vector.push_back(collision_object);

	planning_scene_interface.addCollisionObjects(collision_vector);
	*/
	// Control the move_group to a desired position of the user input
	std::cout << "Welcome to the user input state." << std::endl;
	std::cout << "Please give your input for the coordinates of the end-effector to pick and place in the format < input 0 = x, input 1 = y, input 2 = z >" << std::endl;
	geometry_msgs::Pose target_pose;
	std::vector<double> position;
	double pos;
	while(ros::ok())
	{
		for (int j = 0; j < number_of_inputs-4; j++)
		{
			std::cout << "input " << j << ": ";
			std::cin >> pos;
			std::cout << "     ";
			position.push_back(pos);
		}
		// 3 dimensional space; x, y, z
		target_pose.position.x = position[0];
		target_pose.position.y = position[1];
		target_pose.position.z = position[2];
		// quaternions; x, y, z, w
		/*
		target_pose.orientation.w = position[3];
		target_pose.orientation.x = position[4];
		target_pose.orientation.y = position[5];
		target_pose.orientation.z = position[6];
		*/
		// Default for point downwards e.g. to pick and place
		target_pose.orientation.w = 0.707;
		target_pose.orientation.x = 0;
		target_pose.orientation.y = 0.707;
		target_pose.orientation.z = 0;
		
		// set target_pose as posetarget
		move_group.setPoseTarget(target_pose);

		moveit::planning_interface::MoveGroupInterface::Plan my_plan;
		//moveit_msgs::RobotTrajectory moveit::planning_interface::MoveGroupInterface::Plan::trajectory_ my_plan;
		std::vector<double> somevec;
		//move_group.setNumPlanningAttempts(10); 				// set planning attempts to 10
		bool succes_plan = move_group.plan(my_plan);
		if (succes_plan == true)
		{
			ROS_INFO("SUCCESFUL PLANNNG");
		// Only when the planning is succesfull the move_group needs to try and move.
		//bool valid = planning_scene::PlanningScene::isPathValid(PLANNING_GROUP, my_plan, false, somevec);
			bool succes_move = move_group.move();
			if (succes_move == true)
			{
				ROS_INFO("SUCCESFUL MOVING");
			}
			else
			{
					ROS_INFO("FAILED MOVING");
			}
		}
		else
		{
			ROS_INFO("FAILED PLANNING");
		}
		position.clear();
		ros::spinOnce();
		sleep(5);
	}
	return 0;
}
