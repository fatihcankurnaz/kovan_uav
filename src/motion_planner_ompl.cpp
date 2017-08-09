#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/DiscreteMotionValidator.h>
#include <ompl/base/samplers/ObstacleBasedValidStateSampler.h>

#include <omplapp/apps/SE2MultiRigidBodyPlanning.h>

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

#include <ctime>
#include <vector>
#include <utility>
#include <cmath>
#include <map>

namespace ob = ompl::base;
namespace og = ompl::geometric;

ros::Publisher step_cmd, arrived, samp_pub;
std::map<std::string, std::vector< std::pair<float,float> > > paths;
std::vector<std::pair<float, float> > obstacle_list;
std::pair<float, float> robot_pos, goal_pos;
clock_t timeBeforePlanning;
int total_model_count = 2;
bool first_end_published = false;

bool isStateValid(const ob::State* state)
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

    return true;
}




void publishStep(std::string uav)
{
	if (!paths[uav].empty())
	{
        
		std::vector< std::pair<float,float> >::iterator step_itr = paths[uav].begin();
		        
		geometry_msgs::PoseStamped step;
		step.header.stamp = ros::Time(0);
		step.header.frame_id = "world";

        step.pose.position.x = step_itr->first;
		step.pose.position.y = step_itr->second;
        paths[uav].erase(step_itr);

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


bool terminalCondition()
{
    clock_t checkTime = clock() - timeBeforePlanning;
    float seconds_passed = ((float)checkTime) / CLOCKS_PER_SEC;
    if (seconds_passed >= 1.5)
    {
        ROS_INFO("Termination achieved.");
        return true;
    }
    else
    {
        //ROS_INFO("Continuing ...");
        return false;
    }
}

void multipleDronePlanning(){

    app::SE2MultiRigidBodyPlanning setup(2);
    base::ScopedState<base::CompoundStateSpace> start(setup.getSpaceInformation());
        base::ScopedState<base::CompoundStateSpace> goal(setup.getSpaceInformation());
    
        // define starting state for robot 1
        auto* start1 = start->as<base::SE2StateSpace::StateType>(0);
        start1->setXY(0., 0.);
        start1->setYaw(0.);
        // define goal state for robot 1
        auto* goal1 = goal->as<base::SE2StateSpace::StateType>(0);
        goal1->setXY(7., 5.);
       goal1->setYaw(0.);
    
        // define starting state for robot 2
        auto* start2 = start->as<base::SE2StateSpace::StateType>(1);
        start2->setXY(2., 0.);
        start2->setYaw(0.);
        // define goal state for robot 2
       auto* goal2 = goal->as<base::SE2StateSpace::StateType>(1);
        goal2->setXY(10., 10.);
        goal2->setYaw(0.);
    
        // set the start & goal states
        setup.setStartAndGoalStates(start, goal);
    
        // use RRTConnect for planning
        setup.setPlanner(std::make_shared<geometric::RRTstar>(setup.getSpaceInformation()));
        setup.setup();
        setup.print(std::cout);
        // attempt to solve the problem, and print it to screen if a solution is found
        if (setup.solve(60))
        {
            setup.simplifySolution();
            setup.getSolutionPath().printAsMatrix(std::cout);
        }
}


void manualPlanning()
{
    ob::StateSpacePtr space(new ob::SE2StateSpace());
    ob::RealVectorBounds bounds(2);
    bounds.setLow(-10);
    bounds.setHigh(10);
    space->as<ob::SE2StateSpace>()->setBounds(bounds);

    ob::SpaceInformationPtr si(new ob::SpaceInformation(space));
    si->setStateValidityChecker(isStateValid);

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

    ob::PlannerTerminationConditionFn f = &terminalCondition;
    ob::PlannerTerminationCondition ptc(f);
    ob::PlannerStatus solved = planner->solve(ptc);

    if (solved)
    {
        ROS_INFO("Found a solution!");
        ob::PathPtr p = pdef->getSolutionPath();
        /*ob::PlannerData pd(si);
        planner->getPlannerData(pd);
        /* --------- Code from http://www.cplusplus.com/reference/ios/ios/rdbuf/ -------------
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
                paths["quadrotor"].emplace_back(std::make_pair(x,y));

        }
       
    }
    else
    {
    	ROS_INFO("No solution is found!");
    	paths["quadrotor"] = std::vector< std::pair<float,float> >();
    }
}

void UAV_StepDone(const hector_uav_msgs::Done::ConstPtr& msg)
{
	if (!paths["quadrotor"].empty())
		publishStep("quadrotor");
	else
		ROS_INFO("Either quadrotor arrived or no path found!");
}

void loadModels(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
	int model_count = msg->name.size();
	if (model_count > total_model_count)
	{
		robot_pos.first = msg->pose[1].position.x;
		robot_pos.second = msg->pose[1].position.y;

		for (int i = 2; i < model_count; i++)
			obstacle_list.push_back(std::make_pair(msg->pose[i].position.x, msg->pose[i].position.y));

		total_model_count = model_count;
	}
}

void UAV_MainGoal(const hector_uav_msgs::Vector::ConstPtr& goal)
{
	goal_pos.first = goal->x;
	goal_pos.second = goal->y;

	// std::vector<ob::State*> goal_path = simpleSetupPlanning();

	//manualPlanning();
	multipleDronePlanning();
    publishStep("quadrotor");
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "motion_planner_ompl");
	ros::NodeHandle node;

	ros::Subscriber model_sub = node.subscribe("/gazebo/model_states", 10, &loadModels);
	ros::Subscriber done_sub = node.subscribe("/Done", 1, &UAV_StepDone);
	ros::Subscriber mainGoal_sub = node.subscribe("actual_uav_goal", 1, &UAV_MainGoal);
	step_cmd = node.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
	arrived = node.advertise<std_msgs::String>("ultimate_arrival", 1);
    samp_pub = node.advertise<hector_uav_msgs::Vector>("sampled_point", 1000);

	ros::spin();
	return 0;
}

