#include <ros/ros.h>
#include <hector_uav_msgs/Vector.h>
#include <std_msgs/String.h>

ros::Publisher ultimate_goal_pub;

void UAVArrivedGoalCallback(const std_msgs::String::ConstPtr& msg)
{
	/*if (msg->data == "SUCCESS")
		ROS_INFO("SUCCESSFUL PATH PLANNING!!!");
	else if (msg->data == "FAILURE")
		ROS_INFO("PATH PLANNING FAILED!!!");
	else if (msg->data == "")
		ROS_INFO("QUAD DID NOT TAKE OFF!!!");
	else
		ROS_INFO("Unknown: %s.", msg->data.c_str());*/

	// maybe send another ultimate goal position ...
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "quad_ui");
	ros::NodeHandle nh;

	ultimate_goal_pub = nh.advertise<hector_uav_msgs::Vector>("actual_uav_goal", 10);
	ros::Subscriber uav_arrived_sub = nh.subscribe("ultimate_arrival", 10, &UAVArrivedGoalCallback);

	ros::Duration(2.0).sleep();

	hector_uav_msgs::Vector _goal;
	_goal.x = 5.0;
	_goal.y = 2.0;
	_goal.z = 0.5;
	ultimate_goal_pub.publish(_goal);

	ros::spin();
	return 0;
}