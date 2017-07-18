#include <ros/ros.h>
#include <hector_uav_msgs/PoseActionGoal.h>



int main(int argc,char** argv){
	ros::init(argc,argv,"goal_setter");
	ros::NodeHandle nh;
	ros::Publisher pub_vel = nh.advertise<hector_uav_msgs::PoseActionGoal>("/action/pose/goal",1);
	
	hector_uav_msgs::PoseActionGoal goal;
	
	
	goal.goal.target_pose.pose.position.z = 8;
	goal.goal.target_pose.pose.position.y = 0;
	goal.goal.target_pose.pose.position.x = 0;
	ros::Rate rate(10.0);
	while(ros::ok()){
		goal.goal.target_pose.header.stamp = ros::Time::now();
		goal.goal.target_pose.header.frame_id = "world";
		pub_vel.publish(goal);
		rate.sleep();
	}
	return 0;
}

