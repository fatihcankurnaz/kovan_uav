#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <control_toolbox/pid.h>
#include <gazebo_msgs/SpawnModel.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>

#include <hector_uav_msgs/PoseActionGoal.h>
#include <hector_uav_msgs/EnableMotors.h>
#include <hector_uav_msgs/Done.h>

#include <stdlib.h>
#include <fstream>
//#include <thread>
//#include <mutex>
//#include <map> // mapping of threads, common to all threads
#include <cmath>

#define EQUAL_CONST 0.5
static int quadrotor_id = 1;

// PROBLEMATIC FILE READING !!!!
std::string quad_xacro_path = "/home/burak/catkin_ws/src/hector_quadrotor_description/urdf/quadrotor_hokuyo_utm30lx.gazebo.xacro";

void goalPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
void quadPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

//std::map<std::string, QuadController*> activeQuadrotors;
//std::vector<std::thread*> quadrotorThreads;

// THERE IS AMBIGUITY ABOUT HOW TO PUT THIS INTO A THREAD AND START ITS SERVING.
bool isEqual(double a, double b) { return abs(a - b) > EQUAL_CONST ? false : true; }

std::string& convertUrdfToXml(std::string& urdf_path)
{
	std::string out_xml = "";
	std::ifstream file(urdf_path.c_str());
	std::string line;

	while(!file.eof())
	{
		std::getline(file, line);
		out_xml += line;
	}

	file.close();
	return out_xml;
}

class QuadController {
	public:
		struct 
		{
			control_toolbox::Pid x, y, z, yaw;
		} pid_;

		ros::NodeHandle node;
		ros::ServiceClient /*engage_motors,*/ spawn_self;
		ros::Subscriber goal_sub, quad_sub;
		ros::Publisher quad_vel, quad_done;
		geometry_msgs::PoseStamped goalPose, quadPose;

		std::string quad_name, frame_name;
		double yaw_command;
		bool desired_updated;
		bool desired_achived;
		bool operating;

		QuadController(ros::NodeHandle _node) 
		{
			node = _node;
			desired_updated = false;
			desired_achived = false;
			operating = false;
			yaw_command = 0;
			pid_.x.init(ros::NodeHandle(node, "x"));
			pid_.y.init(ros::NodeHandle(node, "y"));
			pid_.z.init(ros::NodeHandle(node, "z"));
			pid_.yaw.init(ros::NodeHandle(node, "yaw"));
			//engage_motors = node.serviceClient<hector_uav_msgs::EnableMotors>("/enable_motors");
			spawn_self = node.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_urdf_model");
			goal_sub = node.subscribe("/move_base_simple/goal", 10, &goalPoseCallback);
			quad_sub = node.subscribe("/ground_truth_to_tf/pose", 10, &quadPoseCallback);
			quad_vel = node.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
			quad_done = node.advertise<hector_uav_msgs::Done>("/Done", 10);
			quad_name = "quadrotor_" + quadrotor_id;
			frame_name = "";
			quadrotor_id++;
		}

		~QuadController() {}

		bool spawn()   // THREADED !!!
		{
			gazebo_msgs::SpawnModel spawn_quad;
			geometry_msgs::Pose init_pose;		// May start at a random place - probably after integrating scan/sense functonality

			init_pose.position.x = 0;
			init_pose.position.y = 0;
			init_pose.position.z = 0;
			init_pose.orientation.x = 0;
			init_pose.orientation.y = 0;
			init_pose.orientation.z = 0;
			init_pose.orientation.w = 1;

			spawn_quad.request.model_name = quad_name;
			spawn_quad.request.model_xml = convertUrdfToXml(quad_xacro_path);
			spawn_quad.request.robot_namespace = quad_name;
			spawn_quad.request.initial_pose = init_pose;
			spawn_quad.request.reference_frame = "world";

			ROS_INFO("Trying t spawn the quadrotor.");
			if (spawn_self.call(spawn_quad))
			{
				if (spawn_quad.response.success)
				{
					ROS_INFO("Quadrotor with name %s is ready to operate.\n", quad_name.c_str());
					//frame_name = "/" + quad_name + "/base_link";
					return true;
				}
				else
					ROS_ERROR_STREAM("Quadrotor spawning has failed.\n");
			}

			return false;
		}

		bool shouldWait() { return (!desired_updated) && (desired_achived); }
};

QuadController* the_quad;

void goalPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) 
{
	/*std::string robot_frame = msg->header.frame_id;
	QuadController* correspondingQuad = activeQuadrotors[robot_frame];*/

	double x = msg->pose.position.x;
	double y = msg->pose.position.y;
	double z = msg->pose.position.z;

	double temp;
	tf::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y, 
		msg->pose.orientation.z, msg->pose.orientation.w);
	tf::Matrix3x3(q).getRPY(temp, temp, the_quad->yaw_command);

	if((the_quad->goalPose.pose.position.x == x) 
		&& (the_quad->goalPose.pose.position.y == y) 
		&& (the_quad->goalPose.pose.position.z == z))
		return;
	the_quad->goalPose.pose.position.x = x;
	the_quad->goalPose.pose.position.y = y;
	the_quad->goalPose.pose.position.z = z;
	the_quad->desired_updated = true;
	the_quad->desired_achived = false;

	/*ROS_INFO("Goal is set : %f, %f, %f", the_quad->goalPose.pose.position.x, 
		the_quad->goalPose.pose.position.y, the_quad->goalPose.pose.position.z);*/
}

void quadPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) 
{
	/*std::string robot_frame = msg->header.frame_id;
	QuadController* correspondingQuad = activeQuadrotors[robot_frame];*/

	double goalX = the_quad->goalPose.pose.position.x;
	double goalY = the_quad->goalPose.pose.position.y;
	double goalZ = the_quad->goalPose.pose.position.z;
	the_quad->quadPose.pose.position.x = msg->pose.position.x;
	the_quad->quadPose.pose.position.y = msg->pose.position.y;
	the_quad->quadPose.pose.position.z = msg->pose.position.z;

	geometry_msgs::Twist vel_msg;
	ros::Duration period(0.1);

	if(the_quad->shouldWait() == false)
	{
		double x = the_quad->pid_.x.computeCommand(goalX - the_quad->quadPose.pose.position.x, period);
		double y = the_quad->pid_.y.computeCommand(goalY - the_quad->quadPose.pose.position.y, period);
		double z = the_quad->pid_.z.computeCommand(goalZ - the_quad->quadPose.pose.position.z, period);

		tf::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y,
			msg->pose.orientation.z, msg->pose.orientation.w);
		double yaw = tf::getYaw(q);
		double yaw_error = the_quad->yaw_command - yaw;
	    // detect wrap around pi and compensate

		if (yaw_error > 3.14) 
			yaw_error -= 2 * 3.14;
		else if (yaw_error < -3.14) 
			yaw_error += 2 * 3.14;

		// if UAV is in the correct position it will return to CommandDone
		if( isEqual(goalX, the_quad->quadPose.pose.position.x) 
			&& isEqual(goalY, the_quad->quadPose.pose.position.y) 
			&& isEqual(goalZ, the_quad->quadPose.pose.position.z) )
		{
			hector_uav_msgs::Done done_send;

			done_send.commandDone = true;
			done_send.position.x = goalX;
			done_send.position.y = goalY;
			done_send.position.z = goalZ;
			done_send.orientation.x = 0;
			done_send.orientation.y = 0;
			done_send.orientation.z = 0;

			the_quad->quad_done.publish(done_send);
		}
		// corrects the position of the quadro by giving velocity
		
		vel_msg.linear.x = x;
		vel_msg.linear.y = y;
		vel_msg.linear.z = z;

		//ROS_INFO("Linear vel-x : %f, vel-y: %f, vel-z: %f", x, y, z);
		the_quad->quad_vel.publish(vel_msg);
	}	
}

int main(int argc, char **argv) 
{
	//ROS_INFO("HELLOOOOO!");
	ros::init(argc, argv, "quad_manipulator");
	ros::NodeHandle root_node;

	/*if (argc != 2) 
		ROS_ERROR_STREAM("Need to know the number of quadrotos in simulation.");

	int number_of_quads = (int)argv[1] - 48;

	for (int i=1; i<=number_of_quads; i++) 
	{
		//...
		std::thread quad_thread;
		// fire thread performs spawnQuadrotor method
		// push thread address (ptr to thread) into the vector quadrotorThreads
	}*/

	//QuadController* quad_instance = new QuadController(root_node);
	//quad_instance->spawn();
	//activeQuadrotors[quad_instance->frame_name] = quad_instance;
	the_quad = new QuadController(root_node);
	//the_quad->spawn();
	//ROS_INFO("Spawn method has returned.");

	ros::spin();
	return 0;
}