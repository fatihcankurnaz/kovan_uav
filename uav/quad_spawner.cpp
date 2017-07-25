#include <ros/ros.h>
#include <gazebo_msgs/SpawnModel.h>
#include <iostream>
#include <fstream>
#include <cstdlib>

std::string& convertUrdfToXml()
{
	/*const char* home = "HOME";
	const char* value = getenv(home);*/
	
	std::string out_xml = "";
	std::ifstream file("/home/burak/catkin_ws/src/hector_manipulator/src/asdas.txt");
	std::string line;

	/*while(!file.eof())
	{
		std::getline(file, line);
		file.ignore();
		ROS_INFO("Line is : %s", line);
		out_xml += line;
	}*/

	file.close();
	return out_xml;
}

int main(int argc, char **argv) 
{
	ros::init(argc, argv, "quad_spawner");
	ros::NodeHandle nh;
	ros::ServiceClient spawn = nh.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_urdf_model");

	gazebo_msgs::SpawnModel spawn_quad;
	geometry_msgs::Pose init_pose;

	init_pose.position.x = 0;
	init_pose.position.y = 0;
	init_pose.position.z = 10;
	init_pose.orientation.x = 0;
	init_pose.orientation.y = 0;
	init_pose.orientation.z = 0;
	init_pose.orientation.w = 1;

	spawn_quad.request.model_name = "HoBag";
	spawn_quad.request.model_xml = convertUrdfToXml();
	spawn_quad.request.robot_namespace = "";
	spawn_quad.request.initial_pose = init_pose;
	spawn_quad.request.reference_frame = "world";

	ROS_INFO("Trying to spawn the quadrotor.");
	/*if (spawn.call(spawn_quad))
	{
		if (spawn_quad.response.success) ROS_INFO("Quadrotor with name HoBag is ready to operate.\n");
		else ROS_ERROR_STREAM("Quadrotor spawning has failed.\n");
	}*/

	return 0;
}