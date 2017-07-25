#include <ros/ros.h>
#include <control_toolbox/pid.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/WrenchStamped.h>
#include <control_msgs/PidState.h>
#include <tf/transform_listener.h>
#include <hector_uav_msgs/AttitudeCommand.h>
#include <hector_uav_msgs/YawrateCommand.h>
#include <hector_uav_msgs/ThrustCommand.h>

struct
{
   control_toolbox::Pid x, y, z, yaw;
} pid_;

double current_velocity_x,current_velocity_y,current_velocity_z;
double command_lin_velocity_x,command_lin_velocity_y,command_lin_velocity_z,command_ang_velocity_z;
double qX,qY,qZ,qW; //quaternion values that are extracted from transformation

void x_callBack(const control_msgs::PidStateConstPtr& state){
	current_velocity_x = state->output;
	//ROS_INFO("x_callback : %f",current_velocity_x);
}

void y_callBack(const control_msgs::PidStateConstPtr& state){
	current_velocity_y = state->output;
	//ROS_INFO("y_callback : %f",current_velocity_y);
}

void z_callBack(const control_msgs::PidStateConstPtr& state){
	current_velocity_z = state->output;
	//ROS_INFO("z_callback : %f",current_velocity_z);
}	

void cmd_velCommandCallback(const geometry_msgs::TwistConstPtr &command){
	command_lin_velocity_x = command->linear.x;
	command_lin_velocity_y = command->linear.y;
	command_lin_velocity_z = command->linear.z;
	command_ang_velocity_z = command->angular.z;
	//ROS_INFO("cmd_velCallback : %f,%f,%f",command_velocity_x,command_velocity_y,command_velocity_z);
}


void groundCallback(const geometry_msgs::PoseStampedConstPtr &ground_data){
	qX = ground_data->pose.orientation.x;
	qY = ground_data->pose.orientation.y;
	qZ = ground_data->pose.orientation.z;
	qW = ground_data->pose.orientation.w;

}
int main(int argc,char** argv){
	ros::init(argc,argv,"basic_velocity_controller");
	ros::NodeHandle controller_nh;

	/*---------------- SUBSCRIBERS ----------------------*/        
	ros::Subscriber x_output_sub = controller_nh.subscribe("/controller/velocity/x/state",10,&x_callBack);
	ros::Subscriber y_output_sub = controller_nh.subscribe("/controller/velocity/y/state",10,&y_callBack);
	ros::Subscriber z_output_sub = controller_nh.subscribe("/controller/velocity/z/state",10,&z_callBack);

	ros::Subscriber cmd_vel_sub = controller_nh.subscribe("/cmd_vel",10,&cmd_velCommandCallback);
	ros::Subscriber ground_sub = controller_nh.subscribe("/ground_truth_to_tf/pose",10,&groundCallback);
	/*--------------------------------------------------*/

	/*---------------- PUBLISHERS ----------------------*/
	ros::Publisher attitude_pub = controller_nh.advertise<hector_uav_msgs::AttitudeCommand>("/command/attitude",1);
	ros::Publisher yawrate_pub = controller_nh.advertise<hector_uav_msgs::YawrateCommand>("/command/yawrate",1);
	ros::Publisher thrust_pub = controller_nh.advertise<hector_uav_msgs::ThrustCommand>("/command/thrust",1);

	pid_.x.init(ros::NodeHandle(controller_nh, "x"));
	pid_.y.init(ros::NodeHandle(controller_nh, "y"));
	pid_.z.init(ros::NodeHandle(controller_nh, "z"));
	pid_.yaw.init(ros::NodeHandle(controller_nh, "yaw"));
	
	ros::Duration period(0.1); 
	const double gravity = 9.8065;
	const double mass =  1.477000;//extracted from robot's model and inertia. For further information, refer to handles.cpp in hector_quadroto_interface package.
	const double load_factor_limit = 1.5;
	ros::Rate r(10);
	while(controller_nh.ok()){
		tf::Quaternion q(qX,qY,qZ,qW);
		double yaw = tf::getYaw(q), sin_yaw, cos_yaw;
		sincos(yaw, &sin_yaw, &cos_yaw);
		
		
		//ROS_INFO("qX,qY,qZ,qW = %f,%f,%f,%f ----- yaw = %f, sin_yaw = %f, cos_yaw = %f",qX,qY,qZ,qW,yaw,sin_yaw,cos_yaw);
		double load_factor = 1. / ( qW* qW - qX * qX - qY * qY + qZ * qZ );
		if(load_factor > load_factor_limit) load_factor = load_factor_limit;
		
		double acceleration_commandX = pid_.x.computeCommand(command_lin_velocity_x - current_velocity_x, period);
		double acceleration_commandY = pid_.y.computeCommand(command_lin_velocity_y - current_velocity_y, period);
		double acceleration_commandZ = pid_.z.computeCommand(command_lin_velocity_z - current_velocity_z, period);
	
		double acceleration_commandX_base_stabilized = cos_yaw * acceleration_commandX + sin_yaw * acceleration_commandY;
		double acceleration_commandY_base_stabilized = -sin_yaw * acceleration_commandX  + cos_yaw * acceleration_commandY;
		double acceleration_commandZ_base_stabilized = acceleration_commandZ;

		hector_uav_msgs::AttitudeCommand attitude_control;
		hector_uav_msgs::YawrateCommand yawrate_control;
		hector_uav_msgs::ThrustCommand thrust_control;

		attitude_control.roll    = -asin(std::min(std::max(acceleration_commandY_base_stabilized / gravity, -1.0), 1.0));
		attitude_control.pitch   =  asin(std::min(std::max(acceleration_commandX_base_stabilized / gravity, -1.0), 1.0));
		yawrate_control.turnrate = command_ang_velocity_z;
		//ROS_INFO("turnrate = %f",yawrate_control.turnrate);
		thrust_control.thrust    = mass * ((acceleration_commandZ_base_stabilized - gravity) * load_factor + gravity);
		
		attitude_pub.publish(attitude_control);
		yawrate_pub.publish(yawrate_control);
		thrust_pub.publish(thrust_control);

		ros::spinOnce();	
		r.sleep();
	}
return 0;
}
