#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/DiscreteMotionValidator.h>
#include <ompl/base/samplers/ObstacleBasedValidStateSampler.h>


#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <hector_uav_msgs/Done.h>
#include <hector_uav_msgs/Vector.h>
#include <std_msgs/String.h>
#include <gazebo_msgs/ModelStates.h>

#include <fstream>
#include <iostream>
#include <boost/bind.hpp>

#include <ctime>
#include <vector>
#include <utility>
#include <cmath>
#include <map>

namespace ob = ompl::base;
namespace og = ompl::geometric;

std::vector<std::pair<float, float> > obstacle_list;
std::map<std::string, std::vector<std::pair<float, float> > > all_paths;
std::map<std::string, std::pair<float,float> > uav_list;
size_t total_model_count = 2;
float waiting_duration = 2.0;


bool isStateValid(const std::string& quad_name,const ob::State* state)
{
    const ob::SE2StateSpace::StateType *se2state = state->as<ob::SE2StateSpace::StateType>();
    //const ob::RealVectorStateSpace::StateType *pos = se2state->as<ob::RealVectorStateSpace::StateType>(0);
    //const ob::SO3StateSpace::StateType *rot = se3state->as<ob::SO3StateSpace::StateType>(1);

    // Check state if its in any of the occupied obstacle areas or the ones inflated with robot's radius
    float sx = se2state->getX();
    float sy = se2state->getY();
    std::vector<std::pair<float, float> >::iterator obs = obstacle_list.begin();
    for(; obs != obstacle_list.end(); obs++)
    {
        // obstacles are unit spheres (unit circles when projected on 2D) with 0.5 m as their radius
        // inflation radius (the radius that wraps the quadrotor properly) is at least 0.5 m
        
        float result = pow(obs->first - sx, 2) + pow(obs->second - sy, 2) - 2.0;
        if (result < 0) // The state falls into either an inner region of an obstacle or an inflation area
            return false;
    }
    if(!all_paths.empty()){
        std::map<std::string, std::vector<std::pair<float, float> > >::iterator map_itr = all_paths.begin();
        float a_square = 0.49; //Elipse's short radius
        for(;map_itr!=all_paths.end();map_itr++){
            //ROS_INFO("Path of %s is registered!",(map_itr->first).c_str());
            for(unsigned int i=0;i<(map_itr->second).size() - 1;i++){
                std::pair<float, float> next_step, previous_step;
                previous_step = (map_itr->second)[i];
                next_step = (map_itr->second)[i+1];
                float distance_between_step_goals = pow(next_step.first - previous_step.first, 2) + pow(next_step.second - previous_step.second, 2);
                float b_square =pow(sqrt(distance_between_step_goals) + 0.5, 2); //Ellipse's long radius
                float ellipse_center_x =  (next_step.first + previous_step.first) / 2;
                float ellipse_center_y =  (next_step.second + previous_step.second) / 2;
                float result = (b_square * pow(sx - ellipse_center_x,2)) + (a_square * pow(sy - ellipse_center_y,2)) - (a_square * b_square);
                if(result < 0)                    
                    return false;
            }     
        }

    }
    

    return true;
}


class WrapperPlanner{
    public:
      ros::NodeHandle node;
      ros::Publisher samp_pub, step_cmd;
      ros::Subscriber done_sub, mainGoal_sub, rloc_sub;
      std::vector< std::pair<float,float> > paths;
      size_t p_total_model_count;
      
      std::pair<float, float> robot_pos, goal_pos;

      std::string quad_name;
      clock_t timeBeforePlanning;

      WrapperPlanner(ros::NodeHandle _nh, std::string uav_name){
            node = _nh;
            quad_name = uav_name;
            done_sub = node.subscribe<hector_uav_msgs::Done>("/"+quad_name+"/Done", 10, boost::bind(&WrapperPlanner::UAV_StepDone,this,quad_name,_1));
            mainGoal_sub = node.subscribe<hector_uav_msgs::Vector>("/"+quad_name+"/actual_uav_goal", 1, boost::bind(&WrapperPlanner::UAV_MainGoal,this,quad_name,_1));
            rloc_sub = node.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 10,boost::bind(&WrapperPlanner::rloc_callback,this,quad_name,_1));

            step_cmd = node.advertise<geometry_msgs::PoseStamped>("/"+quad_name+"/move_base_simple/goal", 1);
            samp_pub = node.advertise<hector_uav_msgs::Vector>("/"+quad_name+"/sampled_point", 1000);
            //dynamic_obstacle_pub = node.advertise<hector_uav_msgs::Vector>("/"+quad_name+/"/")
            timeBeforePlanning = clock();

            p_total_model_count = 2;
      }
      ~WrapperPlanner() {}
      void UAV_MainGoal(const std::string& robot_frame, const hector_uav_msgs::Vector::ConstPtr& goal)
      {
            goal_pos.first = goal->x;
            goal_pos.second = goal->y;

            manualPlanning(robot_pos.first,robot_pos.second,goal_pos.first,goal_pos.second);
            publishStep();
      }
      void UAV_StepDone(const std::string& robot_frame,const hector_uav_msgs::Done::ConstPtr& msg)
      {
            if (!paths.empty())
                publishStep();
            else
                ROS_INFO("Either quadrotor arrived or no path found!");
      }
      void rloc_callback(const std::string& robot_frame, const gazebo_msgs::ModelStates::ConstPtr& msg){
            size_t model_count = msg->name.size();
            if (model_count > p_total_model_count)
            {
                for (unsigned int i = 1; i < model_count; i++){
                    size_t found = msg->name[i].find(quad_name);
                    if(found!=std::string::npos){
                        robot_pos.first = msg->pose[i].position.x;
                        robot_pos.second = msg->pose[i].position.y;
                        break;
                    }
                }
                p_total_model_count = model_count;
            }
      }
      
      bool terminalCondition()
      {
            clock_t checkTime = clock() - timeBeforePlanning;
            float seconds_passed = ((float)checkTime) / CLOCKS_PER_SEC;
            //ROS_INFO("%s seconds_passed: %f ",quad_name.c_str(),seconds_passed);
            if (seconds_passed >= waiting_duration)
            {
                ROS_INFO("Termination achieved.");
                waiting_duration += waiting_duration + 0.1;
                return true;

            }
            else
                return false;
      }


      void manualPlanning(float robotX,float robotY, float goalX,float goalY)
      {
            ob::StateSpacePtr space(new ob::SE2StateSpace());
            ob::RealVectorBounds bounds(2);
            bounds.setLow(-10);
            bounds.setHigh(10);
            space->as<ob::SE2StateSpace>()->setBounds(bounds);
            ROS_INFO("Manual planning for %s : initial_point(%.2f,%.2f)",quad_name.c_str(),robotX,robotY);
            ob::SpaceInformationPtr si(new ob::SpaceInformation(space));
            si->setStateValidityChecker(boost::bind(&isStateValid,quad_name,_1));

            si->setMotionValidator(std::make_shared<ob::DiscreteMotionValidator>(si));
            si->setStateValidityCheckingResolution(0.05);

            ob::ScopedState<ob::SE2StateSpace> robot(space);
            robot->setX(robotX);
            robot->setY(robotY);
            robot->setYaw(0.0);

            ob::ScopedState<ob::SE2StateSpace> goal(space);
            goal->setX(goalX);
            goal->setY(goalY);
            goal->setYaw(0.0);

            ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));
            pdef->setStartAndGoalStates(robot, goal);

            auto planner(std::make_shared<og::RRTstar>(si));
            planner->setProblemDefinition(pdef);
            planner->setup();

            
            std::function<bool()> f = std::bind(&WrapperPlanner::terminalCondition,this);
            ob::PlannerTerminationCondition ptc(f);
            ob::PlannerStatus solved = planner->solve(ptc);

            if (solved)
            {
                ROS_INFO("Found a solution for %s!",quad_name.c_str());
                ob::PathPtr p = pdef->getSolutionPath();

                og::PathGeometric* path = (*p).as<og::PathGeometric> ();
                std::vector<ob::State*> path_states = path->getStates();
                if(!paths.empty()) //Implies the first iteration of planner.
                    paths.clear();
                for(unsigned int i=0;i<path_states.size();i++){
                        const ob::State* state = path_states[i];
                        const ob::SE2StateSpace::StateType *se2state = state->as<ob::SE2StateSpace::StateType>();
                        float x = se2state->getX();
                        float y = se2state->getY();
                        hector_uav_msgs::Vector p;
                        p.x = x;
                        p.y = y;
                        if(i==0)
                            p.z = 99;
                        samp_pub.publish(p);
                        paths.emplace_back(std::make_pair(x,y));

                }
            
                all_paths[quad_name]= paths;

            }
            else
            {
                ROS_INFO("No solution is found for %s!",quad_name.c_str());
                paths = std::vector< std::pair<float,float> >();
            }
      }


      void publishStep()
      {
            if (!paths.empty())
            {
                std::vector< std::pair<float,float> >::iterator step_itr = paths.begin();
                        
                geometry_msgs::PoseStamped step;
                step.header.stamp = ros::Time(0);
                step.header.frame_id = "world";

                step.pose.position.x = step_itr->first;
                step.pose.position.y = step_itr->second;
                paths.erase(step_itr);

                step.pose.position.z = 0.5;
                step.pose.orientation.x = 0;
                step.pose.orientation.y = 0;
                step.pose.orientation.z = 0;
                step.pose.orientation.w = 1;

                step_cmd.publish(step);
                //After this command is published, CPU goes a low-running state. We can exploit that time duration to further plan the motion to find more optimal paths.
                /*if(abs(step_itr->first - goal_pos.first) > 0.2 && abs(step_itr->second - goal_pos.second) > 0.2 )
                    manualPlanning(step.pose.position.x,step.pose.position.y,goal_pos.first,goal_pos.second);*/
            }
            else
                ROS_INFO("Either quadrotor arrived or no path found!");
      }


};




void loadModels(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
	size_t model_count = msg->name.size();
	if (model_count > total_model_count)
	{
		for (unsigned int i = 1; i < model_count; i++){
            size_t found = msg->name[i].find("uav");
            if(found==std::string::npos)
			    obstacle_list.push_back(std::make_pair(msg->pose[i].position.x, msg->pose[i].position.y));
        }
		total_model_count = model_count;
	}
}



int main(int argc, char** argv)
{
	ros::init(argc, argv, "motion_planner_ompl");
	ros::NodeHandle node;

	ros::Subscriber model_sub = node.subscribe("/gazebo/model_states", 10, &loadModels);
    WrapperPlanner* planner1 = new WrapperPlanner(node,"uav1");
    ros::Duration(0.5).sleep();
    WrapperPlanner* planner3 = new WrapperPlanner(node,"uav3");
    ros::Duration(0.5).sleep();
    WrapperPlanner* planner2 = new WrapperPlanner(node,"uav2");
    
	ros::spin();
	return 0;
}
