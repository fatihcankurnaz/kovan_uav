#include <ros/ros.h>

#include <control_toolbox/pid.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>

#include <hector_uav_msgs/EnableMotors.h>
#include <hector_uav_msgs/Done.h>

#include <boost/bind.hpp>
#include <stdlib.h>
#include <fstream>
#include <map>
#include <cmath>

#include <tf/transform_listener.h>


struct 
{
	control_toolbox::Pid x, y, z, yaw;
} pid_;

bool isEqual(double a, double b);
class Wrapper{
public:
	class QuadController {
		private:
			bool shouldWait() { return (!desired_updated) && (desired_achived); }
		public:
			ros::NodeHandle node;
			ros::ServiceClient engage_motors;
			ros::Subscriber goal_sub, quad_sub,collision_sub,waypoint_number_sub;
			ros::Publisher quad_vel, quad_done;
			geometry_msgs::PoseStamped goalPose, quadPose;

			std::string quad_name;
			float velocity_factor,slowing_factor;
			double yaw_command;
			bool desired_updated;
			bool desired_achived;
			bool operating;
			bool slowing;
			QuadController(ros::NodeHandle _node,std::string uav_name) 
			{
				
				activeQuadrotors.insert(std::pair<std::string, Wrapper::QuadController*>(uav_name,this));

				quad_name = uav_name;
				node = _node;
				desired_updated = false;
				desired_achived = false;
				operating = false;
				slowing = false;
				yaw_command = 0;
				velocity_factor = 0.15; //It will be multiplied by number of waypoints in the path.
				slowing_factor = 0.15;
				engage_motors = node.serviceClient<hector_uav_msgs::EnableMotors>("/"+quad_name+"/enable_motors");
				hector_uav_msgs::EnableMotors srv;
				srv.request.enable = true;
				if(engage_motors.call(srv)){
					if(srv.response.success){
						ROS_INFO("%s motors are enabled",quad_name.c_str());		
					}
				}
				goal_sub = node.subscribe<geometry_msgs::PoseStamped>("/" + quad_name + "/move_base_simple/goal", 10, 
					boost::bind(&QuadController::goalPoseCallback, this,quad_name, _1));
				quad_sub = node.subscribe<geometry_msgs::PoseStamped>("/"+quad_name+"/ground_truth_to_tf/pose", 10, 
					boost::bind(&QuadController::quadPoseCallback, this,quad_name, _1));
				collision_sub = node.subscribe<std_msgs::Bool>("/"+quad_name+"/check_collision",1,
					boost::bind(&QuadController::collisionCallback, this,quad_name, _1));

				waypoint_number_sub = node.subscribe<std_msgs::Int32>("/"+quad_name+"/path_number",1,
					boost::bind(&QuadController::UpdateVelocity,this,quad_name,_1));
				quad_vel = node.advertise<geometry_msgs::Twist>("/"+quad_name+"/cmd_vel", 10);
				quad_done = node.advertise<hector_uav_msgs::Done>("/"+quad_name+"/Done", 10);
			}
			// If a collision risk is detected, this callback reverts the "slowing" boolean to true, hence enforces the UAV to slow.
			void collisionCallback(const std::string& robot_frame,const std_msgs::Bool::ConstPtr& msg);
			/* This callback updates the UAV's velocity according to the number of waypoints in the path. This method is embraced, because as the waypoints decreases, distance between them increases. Implemented function paces up to higher speeds as distance grows, so we have determined to use a variable velocity factor.*/
			void UpdateVelocity(const std::string& robot_frame,const std_msgs::Int32::ConstPtr& msg);
			//Goal callback. Updates the goal point.
			void goalPoseCallback(const std::string& robot_frame,const geometry_msgs::PoseStamped::ConstPtr& msg);

			//Quadrotor's own position callback. It not only updates the position of quadrotor; but also computes the necessary velocity vectors in order to reach to the goal. 
			void quadPoseCallback(const std::string& robot_frame,const geometry_msgs::PoseStamped::ConstPtr& msg) 
			{
				Wrapper::QuadController* correspondingQuad = Wrapper::activeQuadrotors[robot_frame];

				double goalX = correspondingQuad->goalPose.pose.position.x;
				double goalY = correspondingQuad->goalPose.pose.position.y;
				double goalZ = correspondingQuad->goalPose.pose.position.z;
				correspondingQuad->quadPose.pose.position.x = msg->pose.position.x;
				correspondingQuad->quadPose.pose.position.y = msg->pose.position.y;
				correspondingQuad->quadPose.pose.position.z = msg->pose.position.z;
				//if(robot_frame == "uav1")
					//ROS_INFO("PoseCallback for %s",robot_frame.c_str());
				geometry_msgs::Twist vel_msg;
				ros::Duration period(0.1);

				if(correspondingQuad->shouldWait() == false)
				{
					double x = pid_.x.computeCommand(goalX - correspondingQuad->quadPose.pose.position.x, period);
					double y = pid_.y.computeCommand(goalY - correspondingQuad->quadPose.pose.position.y, period);
					double z = pid_.z.computeCommand(goalZ - correspondingQuad->quadPose.pose.position.z, period);

					tf::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y,
						msg->pose.orientation.z, msg->pose.orientation.w);
					double yaw = tf::getYaw(q);
					double yaw_error = correspondingQuad->yaw_command - yaw;
				    // detect wrap around pi and compensate

					if (yaw_error > 3.14) 
						yaw_error -= 2 * 3.14;
					else if (yaw_error < -3.14) 
						yaw_error += 2 * 3.14;
					// if UAV is in the correct position it will return to CommandDone
					if(operating && isEqual(goalX, correspondingQuad->quadPose.pose.position.x) 
						&& isEqual(goalY, correspondingQuad->quadPose.pose.position.y) 
						&& isEqual(goalZ, correspondingQuad->quadPose.pose.position.z) )
					{
						hector_uav_msgs::Done done_send;
						/*ROS_INFO("Goal (%f,%f), Robot (%f,%f), Equality(goal,pose) : %.4f",goalX,goalY,
							correspondingQuad->quadPose.pose.position.x,correspondingQuad->quadPose.pose.position.y,
							fabs(goalX - correspondingQuad->quadPose.pose.position.x));*/

						done_send.commandDone = true;
						done_send.position.x = goalX;
						done_send.position.y = goalY;
						done_send.position.z = goalZ;
						done_send.orientation.x = 0;
						done_send.orientation.y = 0;
						done_send.orientation.z = 0;

						//ROS_INFO("Robot step is done.");
						correspondingQuad->quad_done.publish(done_send);
						operating = false;
					}
					// corrects the position of the quadro by giving velocity
					if(!slowing && operating){
						vel_msg.linear.x = x * velocity_factor;
						vel_msg.linear.y = y * velocity_factor;
						vel_msg.angular.z = 2 * pid_.yaw.computeCommand(yaw_error, period);
						//ROS_INFO("%s : x,y,z = %.2f,%.2f,%.2f",robot_frame.c_str(),vel_msg.linear.x,vel_msg.linear.y,vel_msg.angular.z);
						
					}
					else if (slowing && operating){
						//ROS_INFO("Slowing down %s",robot_frame.c_str());
						vel_msg.linear.x = x * slowing_factor;
						vel_msg.linear.y = y * slowing_factor;
						
						slowing_factor = slowing_factor - 0.05;
						if(slowing_factor < 0)
							slowing_factor = 0;
						vel_msg.angular.z = 2 * pid_.yaw.computeCommand(yaw_error, period);
					}
					vel_msg.linear.z = z;
					correspondingQuad->quad_vel.publish(vel_msg);
				}
			}
			
			~QuadController() {}
		};
	static std::map<std::string, QuadController*> activeQuadrotors;
};
