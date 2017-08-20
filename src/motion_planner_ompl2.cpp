#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/DiscreteMotionValidator.h>
#include <ompl/base/samplers/ObstacleBasedValidStateSampler.h>

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <hector_uav_msgs/myDone.h>
#include <hector_uav_msgs/Vector.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>

#include <gazebo_msgs/ModelStates.h>

#include <fstream>
#include <iostream>
#include <boost/bind.hpp>

#include <ctime>
#include <vector>
#include <utility>
#include <tuple>
#include <cmath>
#include <map>

#include <mutex>
#include <thread>
#include <condition_variable>

namespace ob = ompl::base;
namespace og = ompl::geometric;

typedef std::tuple<float, float, float> Coord3D;

std::vector< Coord3D > obstacle_list;
std::map<std::string, std::vector< Coord3D > > all_paths;
std::map<std::string, Coord3D > uav_list;

std::vector<std::thread*> thread_refs;
std::mutex mtx;

// std::condition_variable planning_cnd;

size_t total_model_count = 2;
float waiting_duration = 0.5;
int uav_count = 0;

class Semaphore {
public:
    Semaphore (int count_ = 0)
        : count(count_) {}

    inline void notify()
    {
        std::unique_lock<std::mutex> lock(mtx);
        count++;
        cv.notify_one();
    }

    inline void wait()
    {
        std::unique_lock<std::mutex> lock(mtx);

        while(count <= 0){
            cv.wait(lock);
        }
        count--;
    }

private:
    std::mutex mtx;
    std::condition_variable cv;
    int count;
};

Semaphore sem(0);

bool isStateValid(const std::string& quad_name,const ob::State* state)
{
    const ob::SE3StateSpace::StateType *se3state = state->as<ob::SE3StateSpace::StateType>();

    // Check state if its in any of the occupied obstacle areas or the ones inflated with robot's radius
    float sx = se3state->getX();
    float sy = se3state->getY();
    float sz = se3state->getZ();
    std::vector< Coord3D >::iterator obs = obstacle_list.begin();
    for(; obs != obstacle_list.end(); obs++)
    {
        // obstacles are unit spheres (unit circles when projected on 2D) with 0.5 m as their radius
        // inflation radius (the radius that wraps the quadrotor properly) is at least 0.5 m
        
        float result = pow(std::get<0>(*obs) - sx, 2) + pow(std::get<1>(*obs) - sy, 2) + pow(std::get<2>(*obs) - sz, 2) - 2.0;
        if (result < 0){ // The state falls into either an inner region of an obstacle or an inflation area
            return false;
        }
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

class WrapperPlanner{
    public:
		ros::NodeHandle node;
		ros::Publisher step_cmd,rem_pub;
		ros::Subscriber done_sub, mainGoal_sub, rloc_sub;

		std::vector< Coord3D > paths;
		Coord3D robot_pos, goal_pos;

		std::string quad_name;
		size_t p_total_model_count;
		clock_t timeBeforePlanning;
		bool erase_flag, planning_done;
		float total_path_length,step_path_length;

		WrapperPlanner(ros::NodeHandle _nh, std::string uav_name)
		{
		    node = _nh;
		    quad_name = uav_name;
		    p_total_model_count = 2;
		    erase_flag = false;
		    planning_done = false;
		    total_path_length = 0;
		    step_path_length = 0;
		    all_paths[quad_name] = std::vector< Coord3D >(2);

		    done_sub = node.subscribe<hector_uav_msgs::myDone>("/" + quad_name + "/myDone", 10, 
		    	boost::bind(&WrapperPlanner::UAV_StepDone, this, quad_name, _1));
		    mainGoal_sub = node.subscribe<hector_uav_msgs::Vector>("/" + quad_name + "/actual_uav_goal", 1, 
		    	boost::bind(&WrapperPlanner::UAV_MainGoal, this, quad_name, _1));
		    rloc_sub = node.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 10, 
		    	boost::bind(&WrapperPlanner::rloc_callback, this, quad_name, _1));

		    step_cmd = node.advertise<geometry_msgs::PoseStamped>("/" + quad_name + "/move_base_simple/goal", 1);
		    rem_pub = node.advertise<std_msgs::Float64>("/" + quad_name + "/remaining_step", 1);
		}

		~WrapperPlanner() {}

		void UAV_MainGoal(const std::string& robot_frame, const hector_uav_msgs::Vector::ConstPtr& goal)
		{
		    std::get<0>(goal_pos) = goal->x;
		    std::get<1>(goal_pos) = goal->y;
		    std::get<2>(goal_pos) = goal->z;
		    
		    mtx.lock();
		    timeBeforePlanning = clock();
		    manualPlanning();
		    mtx.unlock();
		    
		    //mtx.lock();
		    //ROS_INFO("%s : Waiting semaphore",robot_frame.c_str());
		    sem.wait();
		    //ROS_INFO("%s : Passed semaphore",robot_frame.c_str());
		    publishStep();
		    sem.notify();
		    /*planning_cnd.notify_one();
		    mtx.unlock();*/
		}

		void UAV_StepDone(const std::string& robot_frame, const hector_uav_msgs::myDone::ConstPtr& msg)
		{
		    mtx.lock();
		    if (!paths.empty())
		    {
		        all_paths[quad_name][1] = paths[0];
		        total_path_length -= step_path_length;
		        publishStep();
		    }
		    else
		    {
		        all_paths.erase(quad_name);
		        erase_flag = true;
		        ROS_INFO("Either %s arrived or no path found!",robot_frame.c_str());
		    }
		    mtx.unlock();
		}

		void rloc_callback(const std::string& robot_frame, const gazebo_msgs::ModelStates::ConstPtr& msg)
		{
			if (!planning_done)
				return;

		    size_t model_count = msg->name.size();
		    for (unsigned int i = 1; i < model_count; i++)
		    {
		        size_t found = msg->name[i].find(quad_name);
		        if(found!=std::string::npos)
		        {
		            robot_pos = std::make_tuple(msg->pose[i].position.x, msg->pose[i].position.y, msg->pose[i].position.z);
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

		bool is_intersection(const std::vector< Coord3D >& uav_path)
		{
		    //Checks the robot_pos and step_goal_pos,if they are intersect with the uav_path.
		    float a_square = 1.2; //Elipse's short radius
		    float distance_between_step_goals = (pow(std::get<0>(uav_path[1]) - std::get<0>(uav_path[0]), 2) + 
		    	pow(std::get<1>(uav_path[1]) - std::get<1>(uav_path[0]), 2) + pow(std::get<2>(uav_path[1]) - std::get<2>(uav_path[0]), 2));

		    float b_square =pow(sqrt(distance_between_step_goals) + 0.5, 2); //Ellipse's long radius

		    float ellipse_center_x = (std::get<0>(uav_path[1]) + std::get<0>(uav_path[0])) / 2;
		    float ellipse_center_y = (std::get<1>(uav_path[1]) + std::get<1>(uav_path[0])) / 2;
		    // float ellipse_center_z =  (pow(std::get<2>(uav_path[1]) + pow(std::get<2>(uav_path[0])) / 2;

		    float robot_result = ((b_square * pow(std::get<0>(robot_pos) - ellipse_center_x, 2)) + 
		    	(a_square * pow(std::get<1>(robot_pos) - ellipse_center_y, 2)) - (a_square * b_square));

		    float step_result = ((b_square * pow(std::get<0>(all_paths[quad_name][1]) - ellipse_center_x, 2)) + 
		    	(a_square * pow(std::get<1>(all_paths[quad_name][1]) - ellipse_center_y, 2)) - (a_square * b_square));

		    //ROS_INFO("Robots distance to ellipse is %.2f",robot_result);
		    if(robot_result < 0 || step_result < 0) 
		        return true;

		    auto onSegment = [](Coord3D p, Coord3D q, Coord3D r){
		        if (std::get<0>(q) <= std::max(std::get<0>(p), std::get<0>(r)) && std::get<0>(q) >= std::min(std::get<0>(p), std::get<0>(r)) &&
		            std::get<1>(q) <= std::max(std::get<1>(p), std::get<1>(r)) && std::get<1>(q) >= std::min(std::get<1>(p), std::get<1>(r)))
		        return true;

		        return false;
		    };
		    
		    auto orientation = [](Coord3D p, Coord3D q, Coord3D r) {
		        float val = ((std::get<1>(q) - std::get<1>(p)) * (std::get<0>(r) - std::get<0>(q)) -
		                (std::get<0>(q) - std::get<0>(p)) * (std::get<1>(r) - std::get<1>(q)));

		        if (val == 0) return 0; // colinear

		        return (val > 0)? 1: 2; // clock or counterclock wise 
		    };

		    // Find the four orientations needed for general and
		    // special cases
		    Coord3D p1 = robot_pos;
		    Coord3D p2 = all_paths[quad_name][1];
		    Coord3D q1 = uav_path[0];
		    Coord3D q2 = uav_path[1];

		    int o1 = orientation(p1, q1, p2);
		    int o2 = orientation(p1, q1, q2);
		    int o3 = orientation(p2, q2, p1);
		    int o4 = orientation(p2, q2, q1);

		    // General case
		    if (o1 != o2 && o3 != o4)
		        return true;

		    // Special Cases
		    // p1, q1 and p2 are colinear and p2 lies on segment p1q1
		    if (o1 == 0 && onSegment(p1, p2, q1)) return true;

		    // p1, q1 and p2 are colinear and q2 lies on segment p1q1
		    if (o2 == 0 && onSegment(p1, q2, q1)) return true;

		    // p2, q2 and p1 are colinear and p1 lies on segment p2q2
		    if (o3 == 0 && onSegment(p2, p1, q2)) return true;

		    // p2, q2 and q1 are colinear and q1 lies on segment p2q2
		    if (o4 == 0 && onSegment(p2, q1, q2)) return true;

		    return false; // Doesn't fall in any of the above cases
		    
		}

		bool terminalCondition()
		{
		    clock_t checkTime = clock() - timeBeforePlanning;
		    float seconds_passed = ((float)checkTime) / CLOCKS_PER_SEC;
		    //ROS_INFO("%s seconds_passed: %f ",quad_name.c_str(),seconds_passed);
		    if (seconds_passed >= waiting_duration)
		    {
		        //ROS_INFO("Termination achieved.");
		        waiting_duration += waiting_duration + 0.1;
		        return true;

		    }
		    else
		        return false;
		}

		ob::ProblemDefinitionPtr planner_setup(ob::StateSpacePtr& space, ob::SpaceInformationPtr& si)
		{
		    ob::RealVectorBounds bounds(3);
		    // Bounds of X
		    bounds.setLow(-10);
		    bounds.setHigh(10);
		    // Bounds of Y
		    bounds.setLow(-10);
		    bounds.setHigh(10);
		    // Bounds of Z
		    bounds.setLow(0);
		    bounds.setHigh(4);
		    space->as<ob::SE3StateSpace>()->setBounds(bounds);
		    
		    si->setStateValidityChecker(boost::bind(&isStateValid,quad_name, _1));
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

		    ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));
		    pdef->setStartAndGoalStates(robot, goal);

		    return pdef;
		}

		void manualPlanning()
		{
		    //std::unique_lock<std::mutex> lock(mtx); //It gets automatically unlocked when goes out of scope.
		    ob::StateSpacePtr space(new ob::SE3StateSpace());
		    ob::SpaceInformationPtr si(new ob::SpaceInformation(space));
		    
		    ob::ProblemDefinitionPtr pdef = planner_setup(space,si);
		    auto planner(std::make_shared<og::RRTstar>(si));
		    planner->setProblemDefinition(pdef);
		    planner->setup(); 

		    std::function<bool()> f = std::bind(&WrapperPlanner::terminalCondition,this);
		    ob::PlannerTerminationCondition ptc(f);
		    ob::PlannerStatus solved = planner->solve(ptc);

		    /*do
		    {
		        ROS_INFO("Trying to solve for %s",quad_name.c_str());
		        solved = planner->solve(ptc);
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

		    } while(!solved);*/

		    if(solved)
		    {
		        ROS_INFO("Found a solution for %s!",quad_name.c_str());
		        ob::PathPtr p = pdef->getSolutionPath();

		        p->print(std::cout);

		        og::PathGeometric* path = (*p).as<og::PathGeometric>();
		        std::vector<ob::State*> path_states = path->getStates();

		        float previousX = std::get<0>(robot_pos);
		        float previousY = std::get<1>(robot_pos);
		        float previousZ = std::get<2>(robot_pos);

		        for(unsigned int i=0;i<path_states.size();i++)
		        {
		            const ob::State* state = path_states[i];
		            const ob::SE3StateSpace::StateType *se3state = state->as<ob::SE3StateSpace::StateType>();

		            float x = se3state->getX();
		            float y = se3state->getY();
		            float z = se3state->getZ();

		            hector_uav_msgs::Vector p;
		            p.x = x;
		            p.y = y;
		            p.z = z;

		            paths.emplace_back(std::make_tuple(x, y, z));
		            total_path_length += euclidean_distance(x, previousX, y, previousY, z, previousZ);

		            previousX = x;
		            previousY = y;
		            previousZ = z;
		        }

		        all_paths[quad_name][1] = paths[0];
		        // planning_done = true;
		    }

		    uav_count++;
		    planning_done = true;
		    if(uav_count == 3)
		        sem.notify();
		}

		bool is_inferior(const std::vector< Coord3D >& other_uav_path)
		{
		    float own_distance = (pow(std::get<0>(robot_pos) - std::get<0>(all_paths[quad_name][1]), 2) + 
		    	pow(std::get<1>(robot_pos) - std::get<1>(all_paths[quad_name][1]), 2) + 
		    	pow(std::get<2>(robot_pos) - std::get<2>(all_paths[quad_name][1]), 2));

		    float other_distance = (pow(std::get<0>(other_uav_path[0]) - std::get<0>(other_uav_path[1]), 2) + 
		    	pow(std::get<1>(other_uav_path[0]) - std::get<1>(other_uav_path[1]), 2) + 
		    	pow(std::get<2>(other_uav_path[0]) - std::get<2>(other_uav_path[1]), 2));

		    if(own_distance > other_distance)
		        return true;
		    return false;
		}

		void publishStep()
		{
			ROS_INFO("%s - Step publishing!...", quad_name.c_str());
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

		        /*bool inferior_flag = false;
		        if(inferior_flag){
		            step.pose.position.x = robot_pos.first;
		            step.pose.position.y = robot_pos.second;
		        }
		        else{*/

		        paths.erase(step_itr);
		        ROS_INFO("%s - Erasing step ...", quad_name.c_str());
		        if(!paths.empty())
		            all_paths[quad_name][1] = paths[0]; //Next element is loaded to all_paths.
		        
		        std_msgs::Float64 distance_msg;
		        distance_msg.data = total_path_length;
		        //ROS_INFO("total_path_length = %.2f for %s",total_path_length,quad_name.c_str());
		       
		        rem_pub.publish(distance_msg);
		        ROS_INFO("%s - remaining step published ...", quad_name.c_str());
		        step_cmd.publish(step);
		        ROS_INFO("%s - move base simple goal step published ...", quad_name.c_str());

		        //Calculate step_path_length for dynamically decreasing total_path at each step_done.
		        step_path_length = euclidean_distance(step.pose.position.x, std::get<0>(robot_pos), 
		        	step.pose.position.y, std::get<1>(robot_pos), step.pose.position.z, std::get<2>(robot_pos));
		    } 
		    else
		        ROS_INFO("Either quadrotor arrived or no path found!");
		}


};


void initiate_UAV(ros::NodeHandle& nh, const std::string& uav_name)
{
    WrapperPlanner* planner = new WrapperPlanner(nh,uav_name);
    //ros::spin();
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
    	boost::bind(&loadModels, std::ref(node), _1));
    
    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();
    return 0;
}