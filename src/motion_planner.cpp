#include <ros/ros.h>
#include <hector_uav_msgs/EnableMotors.h>
#include <geometry_msgs/PoseStamped.h>
#include <gazebo_msgs/ModelStates.h>

static int mapGrid[200][200];//Stores the [x:-10,10] [y:-10,10] coordinates.
int total_model_count = 0;


void propagate_obstacle(float x,float y,float model_radius,float inflation_radius){
	int model_propagator =(int) (model_radius*10);
	int inflation_propagator = (int) (inflation_radius*10);
	int counter = model_propagator + inflation_propagator;
	for(int i=(-1*counter);i<counter;i++){
		for(int j=(-1*counter);j<counter;j++){
			mapGrid[((int)x*10+100+i)][((int)y*10+100)+j] = 1;//The obstacle's width and height frame holds also a collision probability.
		}	
	}

}

void print_grid(){

	for(int i=0;i<200;i++){	
		for(int j=0;j<200;j++){
			std::cout<<mapGrid[i][j];
		}
		std::cout<<std::endl;
	}
}

void modelCallback(const gazebo_msgs::ModelStatesConstPtr& msg){
	float model_radius = 0.5;
	float inflation_radius = 0.4;
	int name_size = msg->name.size();
	
	if(name_size>2 && name_size!=total_model_count){
		std::cout<<"model count = "<<name_size<<std::endl;
		total_model_count = name_size;
		for(int i=2;i<name_size;i++){
			float roundedX = floorf(msg->pose[i].position.x * 10) / 10;
			float roundedY = floorf(msg->pose[i].position.y * 10) / 10;
			mapGrid[((int)roundedX*10+100)][((int)roundedY*10+100)] = 1;//1 means there is an obstacle/possible collision.
			std::cout<<"Index["<<(int)roundedX*10+100<<"]["<<(int)roundedY*10+100<<"] has an obstacle"<<std::endl;
			propagate_obstacle(roundedX,roundedY,model_radius,inflation_radius);
		}
		//print_grid();	
	}
		
}




int main(int argc,char** argv){
	ros::init(argc,argv,"goal_setter");
	ros::NodeHandle nh;
	ros::Publisher goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",1);
	ros::Subscriber model_listener_sub = nh.subscribe("/gazebo/model_states",2,&modelCallback);
	
	geometry_msgs::PoseStamped goal;
	

	goal.pose.position.z = 15;
	goal.pose.position.y = 10;
	goal.pose.position.x = 0;

	goal.pose.orientation.x = 0;
	goal.pose.orientation.y = 0;
	goal.pose.orientation.z = 0;
	goal.pose.orientation.w = 1;

	ros::Rate rate(10.0);
	while(ros::ok()){
		/*goal.header.stamp = ros::Time::now();
		goal.header.frame_id = "world";
		goal_pub.publish(goal);
		*/
		rate.sleep();
		ros::spinOnce();
	}
	return 0;
}

