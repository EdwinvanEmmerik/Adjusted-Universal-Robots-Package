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
#define number_of_inputs 3
int main(int argc, char** argv)
{
	ros::init(argc,argv,"move_group_pick");
	ros::NodeHandle n("~");
	ros::AsyncSpinner spinner(1);
	spinner.start();
	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
	/* Make sure that the robot_model actually gets loaded */
	if (robot_model != 0)
	{
		ROS_INFO("Robot model loaded");
	}
	else
	{
		ROS_INFO("Could not load robot model");
	}
	/* Construct a planning scene that maintains the state of the world */
	const std::string PLANNING_GROUP = "manipulator";
	moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	/* Setting up the planning pipeline */
	ROS_INFO("Reference frame: %s", move_group.getPlanningFrame().c_str());
	ROS_INFO("Reference frame: %s", move_group.getEndEffectorLink().c_str());
	// Define collision object ROS message and name it
	/*
	moveit_msgs::CollisionObject collision_object;
	//collision_object.header.frame_id = move_group.getPlanningFrame();
	collision_object.id = "box1";
	shapes::Mesh* m  = shapes::createMeshFromResource("package://ur_description/meshes/ur5/collision/box1.stl");
	shape_msgs::Mesh mesh;
	shapes::ShapeMsg mesh_msg;
	shapes::constructMsgFromShape(m, mesh_msg);
	mesh = boost::get<shape_msgs::Mesh>(mesh_msg);
	// Define a box to add to the world 
	collision_object.meshes.resize(0.001);
	collision_object.mesh_poses.resize(0.001);
	collision_object.meshes[0] = mesh;
	collision_object.header.frame_id = "table";
	collision_object.mesh_poses[0].position.x = -0.5;
	collision_object.mesh_poses[0].position.y = -0.5;
	collision_object.mesh_poses[0].position.z = -0.1;
	collision_object.mesh_poses[0].orientation.x = 1.57;

	collision_object.meshes.push_back(mesh);
	collision_object.mesh_poses.push_back(collision_object.mesh_poses[0]);
	collision_object.ADD;

	std::vector<moveit_msgs::CollisionObject> collision_vector;
	collision_vector.push_back(collision_object);

	planning_scene_interface.addCollisionObjects(collision_vector);
	*/


	/*  */
	int i;
	std::cout << "Would you like to run the 'test case'(1), the 'input case(2)', the 'constraint case(3)' or the 'waypoint case(4)'?: ";
	std::cin >> i;
	std::cout << std::endl;
	switch (i){
		case 1: //Test case
		{
			std::cout << "You chose the test case." << std::endl;
			/* Define end-effector goal */
			geometry_msgs::Pose target_pose;
			target_pose.orientation.w = 1.0;
			target_pose.position.x = 0.5;
			target_pose.position.y = 0.5;
			target_pose.position.z = 0.5;
			move_group.setPoseTarget(target_pose);

			moveit::planning_interface::MoveGroupInterface::Plan my_plan;

			bool succes_plan = move_group.plan(my_plan);
			if (succes_plan == true)
			{
				ROS_INFO("SUCCESFUL PLANNNG");
			}
			else
			{
				ROS_INFO("FAILED PLANNING");
			}
			
			bool succes_move = move_group.move();
			if (succes_move == true)
			{
				ROS_INFO("SUCCESFUL MOVING");
			}
			else
			{
				ROS_INFO("FAILED MOVING");
			}
			break;
		}
		case 2:	//Play case
		{
			std::cout << "You chose the input case." << std::endl;
			std::cout << "input 0 = x, input 1 = y, input 2 = z, input 3 = w" << std::endl;
			geometry_msgs::Pose target_pose;
			std::vector<double> position;
			double pos;
			while(ros::ok())
			{
				for (int j = 0; j < number_of_inputs; j++)
				{
					std::cout << "input " << j << ": ";
					std::cin >> pos;
					std::cout << "     ";
					position.push_back(pos);
				}
				target_pose.position.x = position[0];
				target_pose.position.y = position[1];
				target_pose.position.z = position[2];
				move_group.setPoseTarget(target_pose);
				moveit::planning_interface::MoveGroupInterface::Plan my_plan;
				bool succes_plan = move_group.plan(my_plan);
				if (succes_plan == true)
				{
					ROS_INFO("SUCCESFUL PLANNNG");
				}
				else
				{
					ROS_INFO("FAILED PLANNING");
				}
				
				bool succes_move = move_group.move();
				if (succes_move == true)
				{
					ROS_INFO("SUCCESFUL MOVING");
				}
				else
				{
					ROS_INFO("FAILED MOVING");
				}
				position.clear();
				ros::spinOnce();
				sleep(5);
			}
			break;
		}
		case 3:	//constraint case DOES NOT WORK
		{
			std::cout << "You chose the constraint case" << std::endl;
			/* DO THIS FIRST
			geometry_msgs::Pose target_pose;
			target_pose.orientation.w = 1.0;
			target_pose.position.x = 0.5;
			target_pose.position.y = 0.5;
			target_pose.position.z = 0.5;
			move_group.setPoseTarget(target_pose);

			moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  			// First define the path constraint
			moveit_msgs::OrientationConstraint ocm;  
			ocm.link_name = "ee_link";  
			ocm.header.frame_id = "base_link";
			ocm.orientation.w = 1.0;
			ocm.absolute_x_axis_tolerance = 0.1;
			ocm.absolute_y_axis_tolerance = 0.1;
			ocm.absolute_z_axis_tolerance = 0.1;
			ocm.weight = 0.1;
			// Now, set it as the path constraint for the group.
  			moveit_msgs::Constraints test_constraints;
  			test_constraints.orientation_constraints.push_back(ocm);  
			move_group.setPathConstraints(test_constraints);
			// First the start state has to be set to the robot it's current state
			robot_state::RobotState start_state(*move_group.getCurrentState());
			geometry_msgs::Pose start_pose2;
			start_pose2.orientation.w = 1.0;
			start_pose2.position.x = 0.6;
			start_pose2.position.y = 0.6;
			start_pose2.position.z = 0.6;
			const robot_state::JointModelGroup *joint_model_group = start_state.getJointModelGroup(move_group.getName());
			start_state.setFromIK(joint_model_group, start_pose2);
			move_group.setStartState(start_state);
			// Now plan from current state to pose2
			move_group.setPoseTarget(target_pose);
			move_group.plan(my_plan);
			move_group.move();
			sleep(10);
			move_group.clearPathConstraints();*/
		}
		case 4: //waypoint case DOES NOT WORK
		{
			std::cout << "You chose the waypoint case" << std::endl;
			// Start with a pose
			std::vector<double> state;
			
				//state.push_back(moveit::core::RobotState::computeAABB());
			

			std::vector<geometry_msgs::Pose> waypoints;
			//waypoints.push_back(move_group.setStartState());
			geometry_msgs::Pose target_pose;
			target_pose.orientation.w = 1.0;
			target_pose.position.x = 0.4;
			target_pose.position.y = 0.4;
			target_pose.position.z = 0.3;
			// To plan a cartesian path you can directly specify a list of waypoints

			target_pose.position.x += 0.2;
			target_pose.position.y += 0.2;
			waypoints.push_back(target_pose); // x 0.6 and y = 0.6

			target_pose.position.z += 0.1;
			waypoints.push_back(target_pose);

			target_pose.position.x -= 0.2;
			target_pose.position.y -= 0.2;
			target_pose.position.z -= 0.1;
			waypoints.push_back(target_pose);
			// The cartesian path is interpolated at a resolution of 1 cm.
			move_group.setPlanningTime(10.0);
			moveit_msgs::RobotTrajectory trajectory_msg;

			 // First to create a RobotTrajectory object
  			robot_trajectory::RobotTrajectory rt(move_group.getCurrentState()->getRobotModel(), PLANNING_GROUP);
  			rt.setRobotTrajectoryMsg(*move_group.getCurrentState(),trajectory_msg);
  			trajectory_processing::IterativeParabolicTimeParameterization iptp;
		    bool success = iptp.computeTimeStamps(rt);
		    ROS_INFO("Computed time stamp %s",success?"SUCCEDED":"FAILED");

  			rt.getRobotTrajectoryMsg(trajectory_msg);
  			//plan.trajectory_ = trajectory_msg;
  			//move_group.execute(rt);
  			move_group.computeCartesianPath(waypoints, 0.01, 0.0, trajectory_msg,false);
  			//std::cout << trajectory_msg << std::endl;
  			sleep(10);
		}
	}
	return 0;
}