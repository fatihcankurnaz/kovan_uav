#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <gazebo_msgs/ModelStates.h>
size_t total_model_count = 2;
std::vector<std::pair<float,float> > uav_list;
ros::Publisher uav1_pub,uav2_pub;

float euclidean_distance(float x1,float y1,float x2,float y2){
	return sqrt(pow(x2 - x1,2) + pow(y2-y1,2));
}
// UAVs must have a priority. We simply set it as descending order. Naemly uav_itr has a higher priority than uav2_itr that represents UAVs.
void check_collision(){
	std_msgs::Bool uav1,uav2;
	uav1.data = false; // operating default
	uav2.data = false; // operating default
	size_t size = uav_list.size();
	//ROS_INFO("size = %u",size);
	for(size_t i= 0;i < size;i++){ //Bigger loop to select checkers.
        for(size_t j = i+1; j < size;j++){ //Inner loop to select checkees.
        	float dist = euclidean_distance(uav_list[i].first,uav_list[i].second,uav_list[j].first,uav_list[j].second);
        	//ROS_INFO("Distance between uav1 and uav2 = %.2f",dist);
        	if(dist < 1.75){
        		uav2.data = true;
                uav1.data = true;
            }
        }
    } 
    uav1_pub.publish(uav1);
    uav2_pub.publish(uav2);
}

void loadUAVs(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
	size_t model_count = msg->name.size();
	size_t uav_counter = 0;
	uav_list.clear();
	for (unsigned int i = 1; i < model_count; i++){
        size_t found = msg->name[i].find("uav");
        if(found!=std::string::npos)
            uav_list.push_back(std::make_pair(msg->pose[i].position.x,msg->pose[i].position.y));
    }
    //ROS_INFO("UAVs loaded");
    check_collision();
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "collision_checker");
	ros::NodeHandle node;

	ros::Subscriber model_sub = node.subscribe("/gazebo/model_states", 10, &loadUAVs);
    uav1_pub = node.advertise<std_msgs::Bool>("/uav1/check_collision",1);
    uav2_pub = node.advertise<std_msgs::Bool>("/uav2/check_collision",1);
	ros::spin();
	return 0;
}
