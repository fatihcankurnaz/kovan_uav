#include <map>
#include <ros/ros.h>
#include <boost/bind.hpp>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <gazebo_msgs/ModelStates.h>
#include "Eigen/Dense"

#define UAV_COUNT 3
size_t total_model_count = 2;
ros::Publisher uavs_pub[UAV_COUNT];  // uav1_pub, uav2_pub, uav3_pub;
ros::Subscriber uavs_sub[UAV_COUNT];
std::map<std::string, float> remaining;

using Eigen::MatrixXf;
MatrixXf uavPoseMatrix(3, 3);
MatrixXf uavDistanceMatrix(3, 3);

// UAVs must have a priority based on shortest remaining step distance first.
void remainingStepCallback(const std::string& _qname, const std_msgs::Float64::ConstPtr& msg) { remaining[_qname] = msg->data; }

void check_collision()
{
    std_msgs::Bool uavs[UAV_COUNT];
    for(int i = 0; i < UAV_COUNT; i++)
        uavs[i].data = false;
    
    for(int i = 1; i < UAV_COUNT; i++)
    {
        for(int j = 0; j < i; j++)
        { 
            // j represents the higher order UAV
            if(uavDistanceMatrix(i, j) < 1.5)
            { 
                //Check whether they have the possibility of "near miss"
                //ROS_INFO("UAV%d and UAV%d has a risk of collision with distance %f", j, i, uavDistanceMatrix(i,j));
                //uavs[i].data = true;

                std::string uav_i = "uav", uav_j = "uav";
                uav_i.push_back(i + 48);
                uav_j.push_back(j + 48);

                if (remaining[uav_i] < remaining[uav_j])
                	uavs[i].data = true;
                else
                	uavs[j].data = true;
            }
        }
    }
    
    for(int i = 0; i < UAV_COUNT; i++)
        uavs_pub[i].publish(uavs[i]);
}

void loadUAVs(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
    //uavPoseMatrix accumulates the origins of UAVs in real-time.
    size_t model_count = msg->name.size();
    for (unsigned int i = 1; i < model_count; i++)
    {
        size_t found = msg->name[i].find("uav");
        if(found!=std::string::npos)
        {
            if(msg->name[i].compare("uav1") == 0)
            {
                uavPoseMatrix(0,0) = msg->pose[i].position.x;
                uavPoseMatrix(0,1) = msg->pose[i].position.y;
                uavPoseMatrix(0,2) = msg->pose[i].position.z;
            }
            else if(msg->name[i].compare("uav2") == 0)
            {
                uavPoseMatrix(1,0) = msg->pose[i].position.x;
                uavPoseMatrix(1,1) = msg->pose[i].position.y;
                uavPoseMatrix(1,2) = msg->pose[i].position.z;
            }
            else if(msg->name[i].compare("uav3") == 0)
            {
                uavPoseMatrix(2,0) = msg->pose[i].position.x;
                uavPoseMatrix(2,1) = msg->pose[i].position.y;
                uavPoseMatrix(2,2) = msg->pose[i].position.z;                    
            }
        }
    }

    //std::cout<<uavPoseMatrix<<std::endl;
    
    //uavDistanceMatrix calculates the distances between UAVs in pairs.
    for(int i = 1; i < UAV_COUNT; i++)
        for(int j = 0; j < i; j++)
            uavDistanceMatrix(i, j) = sqrt((uavPoseMatrix.row(i) - uavPoseMatrix.row(j)).squaredNorm());
         

    check_collision();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "collision_checker");
    ros::NodeHandle node;
    ros::Subscriber model_sub = node.subscribe("/gazebo/model_states", 10, &loadUAVs);
    //Euclidean distances to themselves always equal to 0.
    uavDistanceMatrix(0,0) = 0; 
    uavDistanceMatrix(1,1) = 0;
    uavDistanceMatrix(2,2) = 0;

    uavs_pub[0] = node.advertise<std_msgs::Bool>("/uav1/check_collision", 1);
    uavs_pub[1] = node.advertise<std_msgs::Bool>("/uav2/check_collision", 1);
    uavs_pub[2] = node.advertise<std_msgs::Bool>("/uav3/check_collision", 1);

    uavs_sub[0] = node.subscribe<std_msgs::Float64>("/uav1/remaining_step", 1, boost::bind(&remainingStepCallback, "uav1", _1));
    uavs_sub[1] = node.subscribe<std_msgs::Float64>("/uav2/remaining_step", 1, boost::bind(&remainingStepCallback, "uav2", _1));
    uavs_sub[2] = node.subscribe<std_msgs::Float64>("/uav3/remaining_step", 1, boost::bind(&remainingStepCallback, "uav3", _1));

    ros::spin();
    return 0;
}
