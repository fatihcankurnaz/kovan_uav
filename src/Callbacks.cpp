#include "../include/hector_manipulator/QuadController.h"


#define EQUAL_CONST 0.4
bool isEqual(double a, double b) { return ( fabs(a - b) > EQUAL_CONST ) ? false : true; }

/*---------------------------------------------- CALLBACK FUNCTIONS FOR QUAD_MANIPULATOR ---------------------------*/

// If a collision risk is detected, this callback reverts the "slowing" boolean to true, hence enforces the UAV to slow.
void Wrapper::QuadController::collisionCallback(const std::string& robot_frame,const std_msgs::Bool::ConstPtr& msg){
	slowing = msg->data; 
}


// This callback updates the UAV's velocity according to the number of waypoints in the path. This method is embraced, because 
// as the waypoints decreases, distance between them increases. Implemented function paces up to higher speeds as distance
// grows, so we have determined to use a variable velocity factor.
void Wrapper::QuadController::UpdateVelocity(const std::string& robot_frame,const std_msgs::Int32::ConstPtr& msg){
	velocity_factor = 0.15 * msg->data;
	if(velocity_factor > 1.0)
	velocity_factor = 1.0;
}
void Wrapper::QuadController::goalPoseCallback(const std::string& robot_frame,const geometry_msgs::PoseStamped::ConstPtr& msg) 
{
	operating = true;
	QuadController* correspondingQuad = activeQuadrotors[robot_frame];

	double x = msg->pose.position.x;
	double y = msg->pose.position.y;
	double z = msg->pose.position.z;
	ROS_INFO("%s : Step goal is (%f,%f,%f)",robot_frame.c_str(),x,y,z);
	double temp;
	tf::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y, 
		msg->pose.orientation.z, msg->pose.orientation.w);
	tf::Matrix3x3(q).getRPY(temp, temp, correspondingQuad->yaw_command);

	if((correspondingQuad->goalPose.pose.position.x == x) 
		&& (correspondingQuad->goalPose.pose.position.y == y) 
		&& (correspondingQuad->goalPose.pose.position.z == z))
		return;
	correspondingQuad->goalPose.pose.position.x = x;
	correspondingQuad->goalPose.pose.position.y = y;
	correspondingQuad->goalPose.pose.position.z = z;
	correspondingQuad->desired_updated = true;
	correspondingQuad->desired_achived = false;
}


/*TODO: Actually, quadPoseCallback should be implemented here as well; however, when moved to here,
 it cannot publish velocities somehow. It should be solved. */



/*-----------------------------------------------------------------------------------------------------------------*/
