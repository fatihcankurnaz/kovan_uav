#include <ros/ros.h>
#include <gazebo_msgs/SpawnModel.h>
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <string>

std::string convertUrdfToXml()
{
	const char *name = "HOME";
  	std::string abs_path;
	
	abs_path = getenv(name);
	std::string path = "/catkin_ws/src/hector_manipulator/robot_model.xml";
  	abs_path.append(path);
  	std::ifstream file(abs_path.c_str());
	std::string out_xml = "";
	std::string line;
	
	if (file.is_open())
	{
	    while ( getline (file,line) )
	    {
	      //ROS_INFO("%s",line.c_str());
	      out_xml+=line;
	    }
	    file.close();
	}
  	else ROS_INFO("Unable to open file");
	return out_xml;
}

int main(int argc, char **argv) 
{
	ros::init(argc, argv, "quad_spawner");
	ros::NodeHandle nh;
	ros::ServiceClient spawn = nh.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_urdf_model");

	gazebo_msgs::SpawnModel spawn_quad;
	geometry_msgs::Pose init_pose;
	
	if (argc != 2) 
		ROS_ERROR_STREAM("Need to know the number of quadrotors in simulation.");

	int number_of_quads = (int)(*argv[1]) - 48;

	for (int i=1; i<=number_of_quads; i++) 
	{
		init_pose.position.x = -1*i;
		init_pose.position.y = 2*i;
		init_pose.position.z = 1;
		init_pose.orientation.x = 0;
		init_pose.orientation.y = 0;
		init_pose.orientation.z = 0;
		init_pose.orientation.w = 1;
		std::string name = "drone";
		std::string n = std::to_string(i+1);
		name.append(n);

		spawn_quad.request.model_name = name;
		spawn_quad.request.model_xml = convertUrdfToXml();
		spawn_quad.request.robot_namespace = name;
		spawn_quad.request.initial_pose = init_pose;
		spawn_quad.request.reference_frame = "world";
	
		ROS_INFO("Trying to spawn the quadrotor.");
		if (spawn.call(spawn_quad))
		{
			if (spawn_quad.response.success) ROS_INFO("Quadrotor with name %s is ready to operate.\n",name.c_str());
			else ROS_ERROR_STREAM("Quadrotor spawning has failed.\n");
		}
	}
	

	return 0;
}
