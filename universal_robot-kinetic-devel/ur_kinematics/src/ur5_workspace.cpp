#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometric_shapes/shape_operations.h>
#define pi 3.14159265359

int main(int argc, char **argv)
{
	ros::init(argc, argv, "building_workspace");
	ros::NodeHandle node_handle;
	ros::AsyncSpinner spinner(1);
	spinner.start();

	const std::string PLANNING_GROUP = "manipulator";
	moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	Eigen::Vector3d ground_scaling(0.0005, 0.0015, 0.0015);

	moveit_msgs::CollisionObject collision_object;
	collision_object.id = "wall";
	shapes::Mesh* ground = shapes::createMeshFromResource("package://ur_description/meshes/ur5/collision/box1.stl", ground_scaling);
	ROS_INFO("wall mesh loaded");

	shape_msgs::Mesh mesh;
	shapes::ShapeMsg mesh_msg;
	shapes::constructMsgFromShape(ground, mesh_msg);
	mesh = boost::get<shape_msgs::Mesh>(mesh_msg);
	collision_object.meshes.resize(1);
	collision_object.mesh_poses.resize(1);
	std::vector<moveit_msgs::CollisionObject> collision_vector;
// SIMULATION
	// Ground wall
	collision_object.meshes[0] = mesh;
	collision_object.header.frame_id = move_group.getPlanningFrame();
	collision_object.mesh_poses[0].position.x = -0.25;
	collision_object.mesh_poses[0].position.y = -0.25;
	collision_object.mesh_poses[0].position.z = -0.25;
	collision_object.mesh_poses[0].orientation.w = 0.5;
	collision_object.mesh_poses[0].orientation.x = -0.5;
	collision_object.mesh_poses[0].orientation.y = -0.5;
	collision_object.mesh_poses[0].orientation.z = -0.5;

	collision_object.meshes.push_back(mesh);
	collision_object.mesh_poses.push_back(collision_object.mesh_poses[0]);
	collision_vector.push_back(collision_object);
	// Back wall
	Eigen::Vector3d back_wall_scaling(0.0005, 0.0015, 0.00075);
	shapes::Mesh* back_wall = shapes::createMeshFromResource("package://ur_description/meshes/ur5/collision/box1.stl", back_wall_scaling);
	shapes::constructMsgFromShape(back_wall, mesh_msg);
	mesh = boost::get<shape_msgs::Mesh>(mesh_msg);

	collision_object.meshes[1] = mesh;
	collision_object.header.frame_id = move_group.getPlanningFrame();
	collision_object.mesh_poses[1].position.x = -0.35;
	collision_object.mesh_poses[1].position.y = -0.25;
	collision_object.mesh_poses[1].position.z = -0.1;
	collision_object.mesh_poses[1].orientation.w = 0.0;
	collision_object.mesh_poses[1].orientation.x = 0.0;
	collision_object.mesh_poses[1].orientation.y = 0.0;
	collision_object.mesh_poses[1].orientation.z = 0.0;
	collision_object.meshes.push_back(mesh);
	collision_object.mesh_poses.push_back(collision_object.mesh_poses[1]);
	collision_vector.push_back(collision_object);

	// Front wall
	Eigen::Vector3d front_wall_scaling(0.0005, 0.00075, 0.0015);
	shapes::Mesh* front_wall = shapes::createMeshFromResource("package://ur_description/meshes/ur5/collision/box1.stl", front_wall_scaling);
	shapes::constructMsgFromShape(front_wall, mesh_msg);
	mesh = boost::get<shape_msgs::Mesh>(mesh_msg);
	collision_object.meshes[2] = mesh;
	collision_object.header.frame_id = move_group.getPlanningFrame();
	collision_object.mesh_poses[2].position.x = 1.25;
	collision_object.mesh_poses[2].position.y = -0.25;
	collision_object.mesh_poses[2].position.z = -0.1;
	collision_object.mesh_poses[2].orientation.w = 0.5;
	collision_object.mesh_poses[2].orientation.x = 0.5;
	collision_object.mesh_poses[2].orientation.y = -0.5;
	collision_object.mesh_poses[2].orientation.z = -0.5;

	collision_object.meshes.push_back(mesh);
	collision_object.mesh_poses.push_back(collision_object.mesh_poses[2]);
	collision_vector.push_back(collision_object);
/*	// REAL
// Ground wall
	collision_object.meshes[0] = mesh;
	collision_object.header.frame_id = move_group.getPlanningFrame();
	collision_object.mesh_poses[0].position.x = -1.00;
	collision_object.mesh_poses[0].position.y = -0.75;
	collision_object.mesh_poses[0].position.z = -0.11;
	collision_object.mesh_poses[0].orientation.w = 0.5;
	collision_object.mesh_poses[0].orientation.x = -0.5;
	collision_object.mesh_poses[0].orientation.y = -0.5;
	collision_object.mesh_poses[0].orientation.z = -0.5;

	collision_object.meshes.push_back(mesh);
	collision_object.mesh_poses.push_back(collision_object.mesh_poses[0]);
	collision_vector.push_back(collision_object);
	// Back wall
	Eigen::Vector3d back_wall_scaling(0.0005, 0.0015, 0.00075);
	shapes::Mesh* back_wall = shapes::createMeshFromResource("package://ur_description/meshes/ur5/collision/box1.stl", back_wall_scaling);
	shapes::constructMsgFromShape(back_wall, mesh_msg);
	mesh = boost::get<shape_msgs::Mesh>(mesh_msg);

	collision_object.meshes[1] = mesh;
	collision_object.header.frame_id = move_group.getPlanningFrame();
	collision_object.mesh_poses[1].position.x = 0.15;
	collision_object.mesh_poses[1].position.y = -1.0;
	collision_object.mesh_poses[1].position.z = -0.1;
	collision_object.mesh_poses[1].orientation.w = 0.0;
	collision_object.mesh_poses[1].orientation.x = 0.0;
	collision_object.mesh_poses[1].orientation.y = 0.0;
	collision_object.mesh_poses[1].orientation.z = 0.0;
	collision_object.meshes.push_back(mesh);
	collision_object.mesh_poses.push_back(collision_object.mesh_poses[1]);
	collision_vector.push_back(collision_object);

	// Front wall
	Eigen::Vector3d front_wall_scaling(0.0005, 0.00075, 0.0015);
	shapes::Mesh* front_wall = shapes::createMeshFromResource("package://ur_description/meshes/ur5/collision/box1.stl", front_wall_scaling);
	shapes::constructMsgFromShape(front_wall, mesh_msg);
	mesh = boost::get<shape_msgs::Mesh>(mesh_msg);
	collision_object.meshes[2] = mesh;
	collision_object.header.frame_id = move_group.getPlanningFrame();
	collision_object.mesh_poses[2].position.x = 0.25;
	collision_object.mesh_poses[2].position.y = 0.50;
	collision_object.mesh_poses[2].position.z = -0.1;
	collision_object.mesh_poses[2].orientation.w = 0.5;
	collision_object.mesh_poses[2].orientation.x = 0.5;
	collision_object.mesh_poses[2].orientation.y = -0.5;
	collision_object.mesh_poses[2].orientation.z = -0.5;

	collision_object.meshes.push_back(mesh);
	collision_object.mesh_poses.push_back(collision_object.mesh_poses[2]);
	collision_vector.push_back(collision_object);	
//*/

	planning_scene_interface.applyCollisionObjects(collision_vector);
	ROS_INFO("Walls added into the world");

	sleep(5.0);
	ros::shutdown();
	return 0;
}