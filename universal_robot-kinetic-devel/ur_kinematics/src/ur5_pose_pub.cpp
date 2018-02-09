#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Pose.h>
#include <ur_kinematics/vision_data2.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ur_pose_publisher");
	ros::NodeHandle n;

	//ros::Publisher pub = n.advertise<geometry_msgs::Pose2D>("desired_pose", 10);
	ros::Publisher pub = n.advertise<ur_kinematics::vision_data2>("desired_pose", 1000);
	int loop_count = 0;

	//Construct vision message
	ur_kinematics::vision_data2 vision_msg;
	//Pose message, includes: x, y, z , w, rx, ry and rz
	geometry_msgs::Pose pick_msg;
	geometry_msgs::Pose place_msg;
	while(ros::ok())
	{
		/*
			// SIM
			pick_msg.position.x = 0.5;
			pick_msg.position.y = 0.2;
			pick_msg.position.z = 0.005;
			pick_msg.orientation.w = 0.707;
			pick_msg.orientation.x = 0.707;
			pick_msg.orientation.y = 0.0;
			pick_msg.orientation.z = 0.0;
			vision_msg.pick_pose = pick_msg;

			place_msg.position.x = 0.2;
			place_msg.position.y = 0.5;
			place_msg.position.z = 0.005;
			place_msg.orientation.w = 0.707;
			place_msg.orientation.x = 0.707;
			place_msg.orientation.y = 0.0;
			place_msg.orientation.z = 0.0;
			vision_msg.place_pose = place_msg;
			*/
			// REAL
			
			pick_msg.position.x = -0.5;
			pick_msg.position.y = -0.2;
			pick_msg.position.z = 0.1;
			pick_msg.orientation.w = 0.707;
			pick_msg.orientation.x = 0.707;
			pick_msg.orientation.y = 0.0;
			pick_msg.orientation.z = 0.0;
			vision_msg.pick_pose = pick_msg;

			place_msg.position.x = -0.5;
			place_msg.position.y = -0.0;
			place_msg.position.z = 0.1013;
			place_msg.orientation.w = 0.707;
			place_msg.orientation.x = 0.707;
			place_msg.orientation.y = 0.0;
			place_msg.orientation.z = 0.0;
			vision_msg.place_pose = place_msg;
			
			pub.publish(vision_msg);
			//pose_msg.position.x = double(rand()) / double(RAND_MAX);
			//pose_msg.position.y = double(rand()) / double(RAND_MAX);
			//pose_msg.position.z = double(rand()) / double(RAND_MAX);
		
		loop_count++;
		//Let the world know
		ROS_INFO("I published something!");
		//Do this.
		ros::spinOnce();
		//Added a delay so not to spam
		sleep(1);
	}
	return 0;
}
/*
#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Pose.h>
#include <ur_kinematics/vision_data.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ur_pose_publisher");
	ros::NodeHandle n;

	//ros::Publisher pub = n.advertise<geometry_msgs::Pose2D>("desired_pose", 10);
	ros::Publisher pub = n.advertise<ur_kinematics::vision_data>("desired_pose", 1000);
	int loop_count = 0;

	//Construct vision message
	ur_kinematics::vision_data vision_msg;
	//Pose message, includes: x, y, z , w, rx, ry and rz
	std_msgs::String action;
	geometry_msgs::Pose pose_msg;
	while(ros::ok())
	{
		//srand(time(0));
		//Pose2D message, includes: x, y and theta
		//geometry_msgs::Pose2D pose_msg;
		//pose_msg.x = double(rand()) / double(RAND_MAX);
		//pose_msg.y = double(rand()) / double(RAND_MAX);
		//pose_msg.theta = double(rand()) / double(RAND_MAX);
	
		std::cout << (loop_count%2) << std::endl;

		if(loop_count >= 6 && loop_count%2 == 0)
		{
			action.data = "pick";
			pose_msg.position.x = -0.6;
			pose_msg.position.y = 0.0;
			pose_msg.position.z = 0.0;
			pose_msg.orientation.w = 0.707;
			pose_msg.orientation.x = 0.707;
			pose_msg.orientation.y = 0.0;
			pose_msg.orientation.z = 0.0;
			vision_msg.desired_pose = pose_msg;
			vision_msg.pick_or_place = action;
			pub.publish(vision_msg);
		}
		else if(loop_count >= 6 && loop_count%2 == 1)
		{
			action.data = "place";
			pose_msg.position.x = -0.6;
			pose_msg.position.y = 0.0;
			pose_msg.position.z = 0.1013;
			pose_msg.orientation.w = 0.707;
			pose_msg.orientation.x = 0.707;
			pose_msg.orientation.y = 0.0;
			pose_msg.orientation.z = 0.0;
			vision_msg.desired_pose = pose_msg;
			vision_msg.pick_or_place = action;
			pub.publish(vision_msg);
			//pose_msg.position.x = double(rand()) / double(RAND_MAX);
			//pose_msg.position.y = double(rand()) / double(RAND_MAX);
			//pose_msg.position.z = double(rand()) / double(RAND_MAX);

		}
		loop_count++;
		//Let the world know
		ROS_INFO("I published something!");
		//Do this.
		ros::spinOnce();
		//Added a delay so not to spam
		sleep(1);
	}
	return 0;
}
*/