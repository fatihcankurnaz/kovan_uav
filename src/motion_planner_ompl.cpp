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
std::map<std::string, std::vector< std::pair<float,float> > > other_uav_list;

size_t total_model_count = 2;
float waiting_duration = 1.5;


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
        
        float result = pow(obs->first - sx, 2) + pow(obs->second - sy, 2) - 1.5;
        if (result < 0) // The state falls into either an inner region of an obstacle or an inflation area
            return false;
    }
    /*std::vector<std::pair<float, float> >::iterator uav_obs = other_uav_list[quad_name].begin();
    for(; uav_obs != other_uav_list[quad_name].end(); uav_obs++)
    {
        // other uavs must be handled differently than unit sphere obstacles. Hence their radius is different than
        // unit spheres'. That is 0.5 m each. Theoritecally, 0.5m is a drone's radius as an obstacle and other 0.5m is inflation radius.
        float result = pow(uav_obs->first - sx,2) + pow(obs->second - sy, 2) - 1.0;
        if(result < 0)
            return false;
    }*/
    

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
            timeBeforePlanning = clock();

            p_total_model_count = 2;
      }
      ~WrapperPlanner() {}
      void UAV_MainGoal(const std::string& robot_frame, const hector_uav_msgs::Vector::ConstPtr& goal)
      {
            goal_pos.first = goal->x;
            goal_pos.second = goal->y;

            manualPlanning();
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
            {
                //ROS_INFO("Continuing ...");
                return false;
            }
      }


      void manualPlanning()
      {
            ob::StateSpacePtr space(new ob::SE2StateSpace());
            ob::RealVectorBounds bounds(2);
            bounds.setLow(-10);
            bounds.setHigh(10);
            space->as<ob::SE2StateSpace>()->setBounds(bounds);
            ROS_INFO("Manual planning for %s",quad_name.c_str());
            ob::SpaceInformationPtr si(new ob::SpaceInformation(space));
            si->setStateValidityChecker(boost::bind(&isStateValid,quad_name,_1));

            si->setMotionValidator(std::make_shared<ob::DiscreteMotionValidator>(si));
            si->setStateValidityCheckingResolution(0.05);

            ob::ScopedState<ob::SE2StateSpace> robot(space);
            robot->setX(robot_pos.first);
            robot->setY(robot_pos.second);
            robot->setYaw(0.0);

            ob::ScopedState<ob::SE2StateSpace> goal(space);
            goal->setX(goal_pos.first);
            goal->setY(goal_pos.second);
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
                
                /* --------- Code from http://www.cplusplus.com/reference/ios/ios/rdbuf/ -------------
                ob::WrapperPlannerData pd(si);
                planner->getWrapperPlannerData(pd);
                
                std::streambuf *psbuf, *backup;
                std::ofstream filestr;
                std::string env_p = std::getenv("HOME");
                std::string file_path = "Desktop/visualize.graphml";
                env_p.append(file_path);

                filestr.open(env_p);

                backup = std::cout.rdbuf();     // back up cout's streambuf

                psbuf = filestr.rdbuf();        // get file's streambuf
                std::cout.rdbuf(psbuf);         // assign streambuf to cout

                pd.printGraphML(std::cout);
                std::cout.rdbuf(backup);        // restore cout's original streambuf

                filestr.close();
                
                /* -----------------------------------------------------------------------------------*/

                og::PathGeometric* path = (*p).as<og::PathGeometric> ();
                std::vector<ob::State*> path_states = path->getStates();
               
                for(unsigned int i=0;i<path_states.size();i++){
                        const ob::State* state = path_states[i];
                        const ob::SE2StateSpace::StateType *se2state = state->as<ob::SE2StateSpace::StateType>();
                        float x = se2state->getX();
                        float y = se2state->getY();
                        hector_uav_msgs::Vector p;
                        p.x = x;
                        p.y = y;
                        samp_pub.publish(p);
                        paths.emplace_back(std::make_pair(x,y));

                }
               
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
                ROS_INFO("Publishing step goal of %s",quad_name.c_str());
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
            if(found!=std::string::npos){
                for(unsigned int j= 1 ; j<model_count;j++){
                    if(j==i)//Exclude itself.
                        continue;
                    size_t inner_found = msg->name[j].find("uav");
                    if(inner_found!=std::string::npos)
                        other_uav_list[msg->name[i]].push_back(std::make_pair(msg->pose[j].position.x, msg->pose[j].position.y));
                }
            }
            else
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
    WrapperPlanner* planner2 = new WrapperPlanner(node,"uav2");
	ros::spin();
	return 0;
}
