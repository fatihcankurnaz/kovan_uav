#include <ros/ros.h>
#include <hector_uav_msgs/EnableMotors.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>

int main(int argc,char** argv){
	ros::init(argc,argv,"goal_setter");
	ros::NodeHandle nh;
	//ros::Publisher pub_vel = nh.advertise<hector_uav_msgs::PoseActionGoal>("/action/pose/goal",1000);
	ros::Publisher goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",1);
	//hector_uav_msgs::PoseActionGoal goal;
	geometry_msgs::PoseStamped goal;
	
	goal.pose.position.z = 15;
	goal.pose.position.y = 10;
	goal.pose.position.x = 0;

	goal.pose.orientation.x = 0;
	goal.pose.orientation.y = 0;
	goal.pose.orientation.z = 0;
	goal.pose.orientation.w = 1;
	/* ---------------------------- Enabling motors handled by pose_action node ---------------------*/
	ros::ServiceClient enable_motors = nh.serviceClient<hector_uav_msgs::EnableMotors>("/enable_motors");
	hector_uav_msgs::EnableMotors srv;
	srv.request.enable = true;
	if(enable_motors.call(srv)){
		if(srv.response.success){
			ROS_INFO("Motors are enabled");		
		}
	}
	
	ros::Rate rate(10.0);
	while(ros::ok()){
		goal.header.stamp = ros::Time::now();
		goal.header.frame_id = "world";
		goal_pub.publish(goal);
				
		rate.sleep();
	}
	return 0;
}

