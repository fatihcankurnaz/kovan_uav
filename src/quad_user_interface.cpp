#include <ros/ros.h>
#include <hector_uav_msgs/Vector.h>
#include <std_msgs/String.h>

ros::Publisher ultimate_goal_pub_uav1,ultimate_goal_pub_uav2,ultimate_goal_pub_uav3,ultimate_goal_pub_uav4;

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

	ultimate_goal_pub_uav1 = nh.advertise<hector_uav_msgs::Vector>("/uav1/actual_uav_goal", 10);
	ultimate_goal_pub_uav2 = nh.advertise<hector_uav_msgs::Vector>("/uav2/actual_uav_goal", 10);
	ultimate_goal_pub_uav3 = nh.advertise<hector_uav_msgs::Vector>("/uav3/actual_uav_goal", 10);	
	ultimate_goal_pub_uav4 = nh.advertise<hector_uav_msgs::Vector>("/uav4/actual_uav_goal", 10);
	//ros::Subscriber uav_arrived_sub = nh.subscribe("ultimate_arrival", 10, &UAVArrivedGoalCallback);

	ros::Duration(2.0).sleep();

	hector_uav_msgs::Vector _goal1, _goal2,_goal3,_goal4;
	_goal1.x = 7.0;
	_goal1.y = 5.0;
	_goal1.z = 1.0;
	ROS_INFO("Publishing uav1 goal");
	ultimate_goal_pub_uav1.publish(_goal1);
	_goal2.x = -7.0;
	_goal2.y = -5.0;
	_goal2.z = 5.0;
	ultimate_goal_pub_uav2.publish(_goal2);
	_goal3.x = 7.0;
	_goal3.y = 3.0;
	_goal3.z = 2.3;
	ultimate_goal_pub_uav3.publish(_goal3);
	_goal4.x = 4.0;
	_goal4.y = -6.0;
	_goal4.z = 4.5;
	
	ultimate_goal_pub_uav4.publish(_goal4);
	ros::spin();
	return 0;
}
