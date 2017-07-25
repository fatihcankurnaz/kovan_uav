#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <hector_uav_msgs/Done.h>
#include <control_toolbox/pid.h>
#include <stdlib.h>
#include <tf/transform_listener.h>
#include <cmath>


#define EQUAL_CONST 0.5

bool desired_updated;
bool desired_achived;


ros::Publisher pub_done; 
ros::Publisher pub_vel; 



bool isEqual(double a,double b){
	return abs(a-b)>EQUAL_CONST ? false:true;
}

bool shouldWait(){
	return (!desired_updated) && (desired_achived);
}

double goalX,goalY,goalZ,yaw_command;
double currentPositionX,currentPositionY,currentPositionZ;

double qX,qY,qZ,qW; //quaternion values that are extracted from transformation

struct
{
   control_toolbox::Pid x, y, z, yaw;
} pid_;


void goalCallback(const geometry_msgs::PoseStampedConstPtr& goal){
	double x =  goal->pose.position.x;
	double y =  goal->pose.position.y;
	double z =  goal->pose.position.z;

	
      	double temp;
      	tf::Quaternion q(goal->pose.orientation.x,goal->pose.orientation.y,goal->pose.orientation.z,goal->pose.orientation.w);
      	tf::Matrix3x3(q).getRPY(temp, temp, yaw_command);

	if((goalX == x) && (goalY = y) && (goalZ = z))
		return;
	goalX = x;
	goalY = y;
	goalZ = z;
	desired_updated = true;
	desired_achived = false;
	//ROS_INFO("Goal is set : %f,%f,%f",goalX,goalY,goalZ);
}



void currentPose_callback(const geometry_msgs::PoseStamped &msg){
	
	currentPositionX = msg.pose.position.x;
	currentPositionY = msg.pose.position.y;
	currentPositionZ = msg.pose.position.z;

	geometry_msgs::Twist pub_msg;
		
	
	ros::Duration period(0.1); 
	if(shouldWait() == false){
		double x = pid_.x.computeCommand(goalX - currentPositionX, period);
		double y = pid_.y.computeCommand(goalY - currentPositionY, period);
		double z = pid_.z.computeCommand(goalZ - currentPositionZ, period);
		
		tf::Quaternion q(msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w);
		double yaw = tf::getYaw(q);
		double yaw_error = yaw_command - yaw;
	    // detect wrap around pi and compensate
		if (yaw_error > 3.14)
		{
		      yaw_error -= 2 * 3.14;
		}
		else if (yaw_error < -3.14)
		{
		      yaw_error += 2 * 3.14;
		}
		
		// if UAV is in the correct position it will return to CommandDone
		if( isEqual(goalX, currentPositionX) && isEqual(goalY, currentPositionY) && isEqual(goalZ, currentPositionZ) ){

			hector_uav_msgs::Done done_send;
			done_send.commandDone = true;
			done_send.position.x = goalX;
			done_send.position.y = goalY;
			done_send.position.z = goalZ;
			done_send.orientation.x = 0;
			done_send.orientation.y = 0;
			done_send.orientation.z = 0;
			pub_done.publish(done_send);
		}
		// corrects the position of the quadro by giving velocity
		
		pub_msg.linear.x = x;
		pub_msg.linear.y = y;
		pub_msg.linear.z = z;
		pub_msg.angular.z = pid_.yaw.computeCommand(yaw_error, period);
	
		//ROS_INFO("Linear vel=(%f,%f,%f) : angular vel = %f",x,y,z,pub_msg.angular.z);
		pub_vel.publish(pub_msg);
		 
	}		
}




int main(int argc,char** argv){
	ros::init(argc,argv,"basic_position_controller");
	ros::NodeHandle controller_nh;
	pub_done = controller_nh.advertise<hector_uav_msgs::Done>("/done",10);
	pub_vel  = controller_nh.advertise<geometry_msgs::Twist>("/cmd_vel",10);

	// Initialize PID controllers
	pid_.x.init(ros::NodeHandle(controller_nh, "x"));
	pid_.y.init(ros::NodeHandle(controller_nh, "y"));
	pid_.z.init(ros::NodeHandle(controller_nh, "z"));
	pid_.yaw.init(ros::NodeHandle(controller_nh, "yaw"));
	
	

	ros::Subscriber goal_sub = controller_nh.subscribe("/move_base_simple/goal",1000, &goalCallback);
	ros::Subscriber sub_current = controller_nh.subscribe("/ground_truth_to_tf/pose",1000,&currentPose_callback);
	
	
	ros::spin();
	return 0;
}
