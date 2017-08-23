#include <ompl/base/spaces/SE3StateSpace.h>
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
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>

#include <gazebo_msgs/ModelStates.h>

#include <fstream>
#include <iostream>
#include <ctime>
#include <cstdlib>
#include <boost/bind.hpp>


#include <vector>
#include <utility>
#include <cmath>
#include <tuple>
#include <map>

#include <mutex>
#include <thread>
#include <condition_variable>

#include "../include/Semaphore.h"

#define SEQUENTIAL_PLANNER_TIME 2.5 /*This is the time variable for planning. Planning can be useful down to 0.5 seconds.
                                      It should be designated to meet the purpose of application. */


namespace ob = ompl::base;
namespace og = ompl::geometric;

typedef std::tuple<float, float, float> Coord3D;

std::vector<Coord3D > obstacle_list;
std::map<std::string, std::vector< Coord3D > > all_paths;
std::map<std::string, Coord3D > uav_list;

std::vector<std::thread*> thread_refs;
std::mutex mtx;

std::condition_variable planning_cnd; 
Semaphore sem(0);
size_t total_model_count = 2;
float waiting_duration = 1.5;
int uav_count = 0;

void printPaths(){
    std::map<std::string, std::vector<Coord3D > >::iterator map_itr = all_paths.begin();
    for(;map_itr!=all_paths.end();map_itr++){
        std::cout<<map_itr->first<<" PATH"<<std::endl;
        for(unsigned int i=0;i<(map_itr->second).size();i++)
            std::cout<<"("<<std::get<0>(map_itr->second[i])<<","<<std::get<1>(map_itr->second[i])<<","<<std::get<2>(map_itr->second[i])<<")"<<std::endl;
        std::cout<<"---------------------"<<std::endl;
    }
}

bool notObstacle(float sx,float sy,float sz){
    std::vector<Coord3D >::iterator obs = obstacle_list.begin();
    for(; obs != obstacle_list.end(); obs++)
    {
        // obstacles are unit spheres (unit circles when projected on 2D) with 0.5 m as their radius
        // inflation radius (the radius that wraps the quadrotor properly) is at least 0.5 m. But for more safe-sought
        // implementations, it can be increased.
        
        float result = pow(std::get<0>(*obs) - sx, 2) + pow(std::get<1>(*obs) - sy, 2) + pow(std::get<2>(*obs) - sz, 2) - 2.0;
        if (result < 0) // The state falls into either an inner region of an obstacle or an inflation area
            return false;
    }

    std::map<std::string, Coord3D >::iterator it = uav_list.begin();
    for(;it!=uav_list.end();it++){
        
        float result = pow(std::get<0>(it->second) - sx,2) + pow(std::get<1>(it->second) - sy,2) + pow(std::get<2>(it->second) - sz,2) - 1.2;
        if(result<0)
            return false;
    }
    
    /*if(!all_paths.empty()){
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
    }*/
    return true;
}

bool isStateValid(const std::string& quad_name,const ob::State* state)
{
    const ob::SE3StateSpace::StateType *se3state = state->as<ob::SE3StateSpace::StateType>();
    //const ob::RealVectorStateSpace::StateType *pos = se2state->as<ob::RealVectorStateSpace::StateType>(0);
    //const ob::SO3StateSpace::StateType *rot = se3state->as<ob::SO3StateSpace::StateType>(1);

    // Check state if its in any of the occupied obstacle areas or the ones inflated with robot's radius
    float sx = se3state->getX();
    float sy = se3state->getY();
    float sz = se3state->getZ();
    return notObstacle(sx,sy,sz);
}

class WrapperPlanner{
    public:
      ros::NodeHandle node;
      ros::Publisher samp_pub, step_cmd,rem_pub,arrived_pub,waypoint_pub;
      ros::Subscriber done_sub, mainGoal_sub, rloc_sub, update_goal_sub;
      std::vector< Coord3D > paths;
      size_t p_total_model_count;
      
      Coord3D robot_pos, goal_pos,step_pos;

      std::string quad_name;
      clock_t timeBeforePlanning;
      bool erase_flag,step_acquired;
      float total_path_length,step_path_distance;

      WrapperPlanner(ros::NodeHandle _nh, std::string uav_name){
            node = _nh;
            quad_name = uav_name;
            done_sub = node.subscribe<hector_uav_msgs::Done>("/"+quad_name+"/Done", 10, boost::bind(&WrapperPlanner::StepDone,this,quad_name,_1));
            mainGoal_sub = node.subscribe<hector_uav_msgs::Vector>("/"+quad_name+"/actual_uav_goal", 1, boost::bind(&WrapperPlanner::MainGoal,this,quad_name,_1));
            /* ---------------- It might be useful when further plannings are considered as meaningfully costly. ---------------*/
            update_goal_sub = node.subscribe<std_msgs::Bool>("/"+quad_name+"/update_goal",1, boost::bind(&WrapperPlanner::UpdateGoal,this,quad_name,_1));
            /* -----------------------------------------------------------------------------------------------------------------*/
            

            step_cmd = node.advertise<geometry_msgs::PoseStamped>("/"+quad_name+"/move_base_simple/goal", 1);
            //samp_pub = node.advertise<hector_uav_msgs::Vector>("/"+quad_name+"/sampled_point", 1000);
            rem_pub = node.advertise<std_msgs::Float64>("/"+quad_name+"/remaining_step",1);
            arrived_pub = node.advertise<std_msgs::String>("/"+quad_name+"/arrival",1);
            waypoint_pub = node.advertise<std_msgs::Int32>("/"+quad_name+"/path_number",1);

            
            all_paths[quad_name] = std::vector<Coord3D>(2);
            rloc_sub = node.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 10,boost::bind(&WrapperPlanner::rloc_callback,this,quad_name,_1));
            
            p_total_model_count = 2;
            erase_flag = false;
            total_path_length = 0;
            step_path_distance = 0;
            step_acquired = false;
      }
      ~WrapperPlanner() {}
      void MainGoal(const std::string& robot_frame, const hector_uav_msgs::Vector::ConstPtr& goal)
      {
            std::get<0>(goal_pos) = goal->x;
            std::get<1>(goal_pos) = goal->y;
            std::get<2>(goal_pos) = goal->z;
            
            mtx.lock();
            timeBeforePlanning = clock();
            autoPlanning();
            //printPaths();
            mtx.unlock();
            
           
            sem.wait();
            publishStep();
            sem.notify();
      }
      void StepDone(const std::string& robot_frame,const hector_uav_msgs::Done::ConstPtr& msg)
      {
            mtx.lock();
            if (!paths.empty()){
                all_paths[quad_name][1] = paths[0];
                total_path_length -= step_path_distance;
                
                ROS_INFO("%s total_path_length = %.2f",robot_frame.c_str(),total_path_length);
                publishStep();
                //planning_cnd.notify_one();
            }
            else{
                    all_paths.erase(quad_name);
                    erase_flag = true;
                    ROS_INFO("%s arrived!",robot_frame.c_str());
                    std_msgs::String msg;
                    msg.data = "SUCCESS";
                    arrived_pub.publish(msg);

                    //Update current position of robot as an obstacle.
                    uav_list[robot_frame] = robot_pos;
                    ROS_INFO("%s position (%.2f,%.2f,%.2f) is added as an obstacle",robot_frame.c_str(),std::get<0>(robot_pos),std::get<1>(robot_pos),std::get<2>(robot_pos));
            }
            mtx.unlock();
      }

      void UpdateGoal(const std::string& robot_frame,const std_msgs::Bool::ConstPtr& msg){
            //A stimuli that dictates to change our path has arrived. Check that if goal is in a safe area.
            
            ros::Duration(1.0).sleep();
            //This sleep is necessary for UAV to stop its movement.
            mtx.lock();
            //This if block is necessary when the UAVs send to same or close goal points.
            if(notObstacle(std::get<0>(goal_pos),std::get<1>(goal_pos),std::get<2>(goal_pos)) == false){
                std::srand(std::time(0));
                while(1){
                        
                        float x_addee,y_addee,z_addee;
                        float temp_goalX,temp_goalY,temp_goalZ;
                        x_addee = (float)((std::rand()%200) - 100) / 100;
                        y_addee = (float)((std::rand()%200) - 100) / 100;
                        z_addee = (float)((std::rand()%200) - 100) / 100;
                        temp_goalX = std::get<0>(goal_pos) + x_addee;
                        temp_goalY = std::get<1>(goal_pos) + y_addee;
                        temp_goalZ = std::get<2>(goal_pos) + z_addee;

                        ROS_INFO("Checking %.2f,%.2f,%.2f as possible goal",temp_goalX,temp_goalY,temp_goalZ);
                        if(notObstacle(temp_goalX,temp_goalY,temp_goalZ))
                        {
                           
                            std::get<0>(goal_pos) = temp_goalX;
                            std::get<1>(goal_pos) = temp_goalY;
                            std::get<2>(goal_pos) = temp_goalZ;
                            ROS_INFO("New goal_point is (%.2f,%.2f,%.2f)",std::get<0>(goal_pos),std::get<1>(goal_pos),std::get<2>(goal_pos));
                            autoPlanning(); //Plan again.
                            break;
                        }
                }
            }
            else //No need to update goal. Check left path considering finished UAVs
                autoPlanning();
            mtx.unlock();
      }
      
      void rloc_callback(const std::string& robot_frame, const gazebo_msgs::ModelStates::ConstPtr& msg){
            /*if (!planning_done)
                return;
            */
            size_t model_count = msg->name.size();
            for (unsigned int i = 1; i < model_count; i++){
                size_t found = msg->name[i].find(quad_name);
                if(found!=std::string::npos){
                    robot_pos = std::make_tuple(msg->pose[i].position.x, msg->pose[i].position.y, msg->pose[i].position.z);
                    if(!step_acquired)
                        step_pos = robot_pos;
                    float current_distance_to_step = euclidean_distance(std::get<0>(step_pos),std::get<0>(robot_pos),
                        std::get<1>(step_pos),std::get<1>(robot_pos),std::get<2>(step_pos),std::get<2>(robot_pos)); 
                    total_path_length -= step_path_distance - current_distance_to_step;
                    step_path_distance = current_distance_to_step;
                    break;
                }
            }
            if(erase_flag)
                return;
            else
                all_paths[quad_name][0] = robot_pos;
      }

      float euclidean_distance(float x1, float y1, float x2, float y2, float z1, float z2)
      {
            return sqrt(pow(x1 - y1, 2) + pow(x2 - y2, 2) + pow(z1 - z2, 2));
      }
      bool terminalCondition()
      {
            clock_t checkTime = clock() - timeBeforePlanning;
            float seconds_passed = ((float)checkTime) / CLOCKS_PER_SEC;
            //ROS_INFO("%s seconds_passed: %f ",quad_name.c_str(),seconds_passed);
            if (seconds_passed >= waiting_duration)
            {
                ROS_INFO("Termination achieved.");
                waiting_duration += SEQUENTIAL_PLANNER_TIME;
                ROS_INFO("waiting_duration for %s is %.2f",quad_name.c_str(),waiting_duration);
                return true;

            }
            else
                return false;
      }

      ob::ProblemDefinitionPtr planner_setup(ob::StateSpacePtr& space,ob::SpaceInformationPtr& si){

            ob::RealVectorBounds bounds(3);
            // Bounds of X
            bounds.setLow(0,-10);
            bounds.setHigh(0,10);
            // Bounds of Y
            bounds.setLow(1,-10);
            bounds.setHigh(1,10);
            // Bounds of Z
            bounds.setLow(2,0);
            bounds.setHigh(2,4);

            space->as<ob::SE3StateSpace>()->setBounds(bounds);
            
            si->setStateValidityChecker(boost::bind(&isStateValid,quad_name,_1));

            si->setMotionValidator(std::make_shared<ob::DiscreteMotionValidator>(si));
            si->setStateValidityCheckingResolution(0.05);

            ob::ScopedState<ob::SE3StateSpace> robot(space);
            robot->setX(std::get<0>(robot_pos));
            robot->setY(std::get<1>(robot_pos));
            robot->setZ(std::get<2>(robot_pos));
            robot->as<ob::SO3StateSpace::StateType>(1)->setIdentity();

            ob::ScopedState<ob::SE3StateSpace> goal(space);
            goal->setX(std::get<0>(goal_pos));
            goal->setY(std::get<1>(goal_pos));
            goal->setZ(std::get<2>(goal_pos));
            goal->as<ob::SO3StateSpace::StateType>(1)->setIdentity();

            ROS_INFO("Manual planning for %s : start point(%.2f,%.2f,%.2f) | goal point(%.2f,%.2f,%.2f)",quad_name.c_str(),
                std::get<0>(robot_pos),std::get<1>(robot_pos),std::get<2>(robot_pos),std::get<0>(goal_pos),
                std::get<1>(goal_pos),std::get<2>(goal_pos));
            
            ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));
            pdef->setStartAndGoalStates(robot, goal);
            return pdef;
      }

      void autoPlanning()
      {
            int solverTimeout = 0;
            ob::StateSpacePtr space(new ob::SE3StateSpace());
            ob::SpaceInformationPtr si(new ob::SpaceInformation(space));
            
            ob::ProblemDefinitionPtr pdef = planner_setup(space,si);
            auto planner(std::make_shared<og::RRTstar>(si));
            planner->setProblemDefinition(pdef);
            planner->setup(); 

            
            std::function<bool()> f = std::bind(&WrapperPlanner::terminalCondition,this);
            ob::PlannerTerminationCondition ptc(f);
            ob::PlannerStatus solved;// = planner->solve(ptc);

            do
            {
                ROS_INFO("Trying to solve for %s",quad_name.c_str());
                solved = planner->solve(ptc);
                solverTimeout++;
                if(!solved){
                    //mtx.unlock();
                    //planning_cnd.wait(lock);
                    planner->clear();

                    ob::ProblemDefinitionPtr pdef = planner_setup(space,si);
                    auto planner(std::make_shared<og::RRTstar>(si));
                    planner->setProblemDefinition(pdef);
                    planner->setup();                    
                    
                    std::function<bool()> f = std::bind(&WrapperPlanner::terminalCondition,this);
                    ob::PlannerTerminationCondition ptc(f);
                }
                if(solverTimeout == 5)
                    break; //Proceed with the former plan.
            }while(!solved);
            if(solved){   
                ROS_INFO("Found a solution for %s!",quad_name.c_str());
                step_acquired = true;
                ob::PathPtr p = pdef->getSolutionPath();

                if(!paths.empty()){//If this is not the first planning of UAV, then clear old path.
                    uav_count--; // This line is added, because this UAV's motion was previously planned.
                    paths.clear();
                }
                og::PathGeometric* path = (*p).as<og::PathGeometric> ();
                
                
                std::vector<ob::State*> path_states = path->getStates();
                float previousX = std::get<0>(robot_pos);
                float previousY = std::get<1>(robot_pos);
                float previousZ = std::get<2>(robot_pos);

                total_path_length = 0;
                for(unsigned int i=0,j=0;i<path_states.size();i++,j++){
                    const ob::State* state = path_states[i];
                    const ob::SE3StateSpace::StateType *se3state = state->as<ob::SE3StateSpace::StateType>();
                    
                    float x = se3state->getX();
                    float y = se3state->getY();
                    float z = se3state->getZ();

                    paths.emplace_back(std::make_tuple(x, y, z));
                    total_path_length += euclidean_distance(x, previousX, y, previousY, z, previousZ);

                    previousX = x;
                    previousY = y;
                    previousZ = z;
                }
                /* This code block, sends the number of waypoints in the path, to the quad_manipulator. */
                std_msgs::Int32 msg;
                msg.data = path_states.size();
                waypoint_pub.publish(msg);
                all_paths[quad_name][1] = paths[0];
                /* It, then adjusts the velocity of quadrotor. This method is embraced, because more waypoints tend to have 
                   short distances between them. Short distances between waypoints result in low natural speeds (computed by PID unit) 
                   compared to longer distances. Therefore, we multiply this natural speed by the waypoint number.
                   --------------------------------------------------------------------------- 
                   To illustrate, assume the natural velocity factor is n. If the path consists of just 2 waypoints, then
                   velocity factor will become 2n; and if its computed velocity is vf, the final velocity would be 2n * vf.
                   On the contrary, if there are 8 waypoints in the path, velocity factor will become 8n; and if its computed velocity
                   is vs, the final velocity would be 8n * vs. These two terms (2n*vf, 8n*vs) would produce roughly the same velocity magnitudes.
                   Term "n" can be adjusted experimentally.
                */
                
            }
            uav_count++;
            if(uav_count == 3)
                sem.notify();
      }

      void publishStep()
      {
            if (!paths.empty())
            {
                std::vector< Coord3D >::iterator step_itr = paths.begin();
                        
                geometry_msgs::PoseStamped step;
                step.header.stamp = ros::Time(0);
                step.header.frame_id = "world";

                step.pose.position.x = std::get<0>(*step_itr);
                step.pose.position.y = std::get<1>(*step_itr);
                step.pose.position.z = std::get<2>(*step_itr);
                

                step.pose.orientation.x = 0;
                step.pose.orientation.y = 0;
                step.pose.orientation.z = 0;
                step.pose.orientation.w = 1;
                step_pos = std::make_tuple(std::get<0>(*step_itr),std::get<1>(*step_itr),std::get<2>(*step_itr));
                paths.erase(step_itr);
                if(!paths.empty())
                    all_paths[quad_name][1] = paths[0]; //Next element is loaded to all_paths.
                
                std_msgs::Float64 distance_msg;
                distance_msg.data = total_path_length;
               
               
                rem_pub.publish(distance_msg);

                step_cmd.publish(step);

                step_path_distance = euclidean_distance(std::get<0>(step_pos),std::get<0>(robot_pos),
                        std::get<1>(step_pos),std::get<1>(robot_pos),std::get<2>(step_pos),std::get<2>(robot_pos));
             } 
            else
                ROS_INFO("Either %s arrived or no path found!",quad_name.c_str());
      }


};


void initiate_UAV(ros::NodeHandle& nh, const std::string& uav_name){
    WrapperPlanner* planner = new WrapperPlanner(nh,uav_name);
}

void loadModels(ros::NodeHandle& nh, const gazebo_msgs::ModelStates::ConstPtr& msg)
{
    size_t model_count = msg->name.size();
    if (model_count > total_model_count)
    {
        for (unsigned int i = 1; i < model_count; i++){
            size_t found = msg->name[i].find("uav");
            if(found!=std::string::npos){
                std::thread *uav_thread = new std::thread(&initiate_UAV, std::ref(nh),msg->name[i]);
                uav_thread->detach();
                thread_refs.push_back(uav_thread);
            }
            else
                obstacle_list.push_back(std::make_tuple(msg->pose[i].position.x, msg->pose[i].position.y, msg->pose[i].position.z));
        }
        total_model_count = model_count;
    }
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "motion_planner_ompl");
    ros::NodeHandle node;

    ros::Subscriber model_sub = node.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 10,
                                boost::bind(&loadModels,std::ref(node),_1));
    
    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();
    return 0;
}
