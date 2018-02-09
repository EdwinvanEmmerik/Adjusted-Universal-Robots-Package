#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

#include <moveit/planning_scene_interface/planning_scene_interface.h>


#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>

#include <octomap/OcTree.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "octomap_publisher");

	ros::NodeHandle n;

	//publisher for the planning scene
	ros::Publisher octomap_pub = n.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
	ros::Rate loop_rate(1);

  	int count = 0;
  	while (ros::ok())
  	{
    	static octomap_msgs::Octomap octomap;
    	//octomap.setResolution(1);
        static bool msgGenerated = false;
        //Turn the octomap .bt file into an octree format, which is needed by BinaryMapToMsg

        if ( msgGenerated == false)
        {
            octomap::OcTree myOctomap("/Dir1/Dir2/.../SOLMeasurePolygon.bt");
            myOctomap.setResolution(1);
            octomap_msgs::binaryMapToMsg(myOctomap, octomap);
            msgGenerated = true;
        }

    	moveit_msgs::PlanningScene planning_scene;
      	planning_scene.world.octomap.header.frame_id = "odom_combined";
	    planning_scene.world.octomap.octomap.header.frame_id = "odom_combined";
	    planning_scene.world.octomap.octomap.binary = true;
	    planning_scene.world.octomap.octomap.id = "OcTree";
	    planning_scene.world.octomap.octomap.data = octomap.data;

	    ROS_INFO("Adding the octomap into the world.");
	    octomap_pub.publish(planning_scene);
	    ros::spinOnce();
    	octomap.data.clear();

    	loop_rate.sleep();
    	++count;
  	} 
  	return 0;
}