#include <map>
#include <ctime>
#include <cstdlib>
#include <vector>
#include <utility>
#include <ros/ros.h>
#include <boost/bind.hpp>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/PoseStamped.h>
#include <hector_uav_msgs/Done.h>

std::map<std::string, ros::Subscriber> uav_done;
std::map<std::string, ros::Publisher> uav_goal;
int initial_model_count = 1;

void initial_trigger(std::vector<std::string>& uavs)
{
	geometry_msgs::PoseStamped init_pose1, init_pose2, init_pose3, init_pose4;

	init_pose1.header.stamp = ros::Time(0);
	init_pose2.header.stamp = ros::Time(0);
	init_pose3.header.stamp = ros::Time(0);
	init_pose4.header.stamp = ros::Time(0);

	init_pose1.header.frame_id = "world";
	init_pose2.header.frame_id = "world";
	init_pose3.header.frame_id = "world";
	init_pose4.header.frame_id = "world";

	init_pose1.pose.position.x = 0;
	init_pose2.pose.position.x = -10;
	init_pose3.pose.position.x = -10;
	init_pose4.pose.position.x = 0;

	init_pose1.pose.position.y = 0;
	init_pose2.pose.position.y = 0;
	init_pose3.pose.position.y = 10;
	init_pose4.pose.position.y = 10;

	init_pose1.pose.position.z = 10;
	init_pose2.pose.position.z = 10;
	init_pose3.pose.position.z = 10;
	init_pose4.pose.position.z = 10;

	init_pose1.pose.orientation.x = 0;
	init_pose2.pose.orientation.x = 0;
	init_pose3.pose.orientation.x = 0;
	init_pose4.pose.orientation.x = 0;

	init_pose1.pose.orientation.y = 0;
	init_pose2.pose.orientation.y = 0;
	init_pose3.pose.orientation.y = 0;
	init_pose4.pose.orientation.y = 0;

	init_pose1.pose.orientation.z = 0;
	init_pose2.pose.orientation.z = 0;
	init_pose3.pose.orientation.z = 0;
	init_pose4.pose.orientation.z = 0;

	init_pose1.pose.orientation.w = 1;
	init_pose2.pose.orientation.w = 1;
	init_pose3.pose.orientation.w = 1;
	init_pose4.pose.orientation.w = 1;

	uav_goal[uavs[0]].publish(init_pose1);
	uav_goal[uavs[1]].publish(init_pose2);
	uav_goal[uavs[2]].publish(init_pose3);
	uav_goal[uavs[3]].publish(init_pose4);

	ROS_INFO("SHOTS FIRED!!! SHOTS FIRED!!! TRIGERRED!!!");
}

void doneCallback(std::string& uav, const hector_uav_msgs::Done::ConstPtr& msg)
{
	int rand_x, rand_y, rand_z;
	srand(time(0));
	rand_x = rand() % 20 + 1;
	srand(time(0)*2 + 3);
	rand_y = rand() % 20 + 1;
	srand(time(0)*5 + 7);
	rand_z = rand() % 20 + 11;

	geometry_msgs::PoseStamped new_pose;
	new_pose.header.stamp = ros::Time(0);
	new_pose.header.frame_id = "world";

	new_pose.pose.position.x = rand_x;
	new_pose.pose.position.y = rand_y;
	new_pose.pose.position.z = rand_z;
	new_pose.pose.orientation.x = 0;
	new_pose.pose.orientation.y = 0;
	new_pose.pose.orientation.z = 0;
	new_pose.pose.orientation.w = 1;

	uav_goal[uav].publish(new_pose);
}

void modelStateCallback(ros::NodeHandle& node, const gazebo_msgs::ModelStates::ConstPtr& msg)
{
	int current_model_count = msg->name.size();
	if (current_model_count > initial_model_count)
	{
		std::vector<std::string> uavs;
		for(int i=1; i<current_model_count; i++)
		{
			std::string uav_name = msg->name[i];

			ros::Publisher goal_pub = node.advertise<geometry_msgs::PoseStamped>("/" + uav_name + "/move_base_simple/goal", 10);
			uav_goal.insert(std::make_pair(uav_name, goal_pub));

			ros::Subscriber done_sub = node.subscribe<hector_uav_msgs::Done>("/" + uav_name + "/Done", 10, 
				boost::bind(&doneCallback, uav_name, _1));
			uav_done.insert(std::make_pair(uav_name, done_sub));
			
			ROS_INFO("UAV: %s is ready for action.", uav_name.c_str());
			uavs.push_back(uav_name);
		}

		initial_model_count = current_model_count;
		initial_trigger(uavs);
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "quad_master");
	ros::NodeHandle nh;

	ros::Subscriber models_sub = nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 10, boost::bind(&modelStateCallback, nh, _1));

	ros::spin();
	return 0;
}