#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/PoseStamped.h>
#include <hector_uav_msgs/Vector.h>
#include <hector_uav_msgs/Done.h>
#include <std_msgs/String.h>
#include <vector>
#include <cstdlib>
#include <ctime>
#include <cmath>
#include <utility>

typedef struct MapNode MapNode;
typedef struct MapEdge MapEdge;

std::map<std::string, std::vector<std::pair<float, float> > > trajectories;
std::map<std::string, std::pair<float, float> > obstacles;
int total_model_count = 0;
double goalX, goalY, robotX, robotY;
ros::Subscriber uav_goal, uav_done;
ros::Publisher  uav_step, uav_arrived;

struct MapNode {
	bool visited;
	MapNode* parent;
	double euclidean_dist_to_goal;
	std::pair<float, float> location;
	std::vector<MapEdge*> neighbours;

	MapNode(): parent(NULL), visited(false), euclidean_dist_to_goal(-1) {}
	MapNode(std::pair<float, float> _loc): parent(NULL), visited(false) 
	{
		double heuristic_value = sqrt(pow(_loc.first - goalX, 2) + pow(_loc.second - goalY, 2));
		euclidean_dist_to_goal = heuristic_value;
	}
	MapNode(MapNode* _p): parent(_p), visited(false), euclidean_dist_to_goal(-1) {}
};

struct MapEdge {
	MapNode* neighbour;
	double cost;

	MapEdge(MapNode* _n, double _c): neighbour(_n), cost(_c) {}
};

class MapTree {
	public:
		std::vector<MapNode*> V;
		MapNode* robot;
		MapNode* goal;

		MapTree() {}
		MapTree(std::pair<float, float> _loc) 
		{
			goal = new MapNode();
			goal->location = _loc;
			goal->euclidean_dist_to_goal = 0;
			V.push_back(goal);
		}
		~MapTree() { /*....*/ }

		void setRobot(std::pair<float, float> _rloc) 
		{ 
			robot = new MapNode(_rloc);
			V.push_back(robot);
		}

		void refreshNodes()
		{
			std::vector<MapNode*>::iterator itr = V.begin();
			for(; itr != V.end(); itr++)
				(*itr)->visited = false;
		}

		void addNode(MapNode* _node) { V.push_back(_node); }
		MapNode* addNode(std::pair<float, float> _loc, MapNode* _p)
		{
			MapNode* new_node = new MapNode(_loc);
			new_node->parent = _p;
			V.push_back(new_node);
			return new_node;
		}

		void addEdge(MapNode* end_1, MapNode* end_2)
		{
			double _cost = (abs(end_1->location.first - end_2->location.first) + 
				abs(end_1->location.second - end_2->location.second));
			end_1->neighbours.push_back(new MapEdge(end_2, _cost));
			end_2->neighbours.push_back(new MapEdge(end_1, _cost));
		}

		MapNode* findNearestTo(std::pair<float, float> point) 
		{
			int index = 0;
			double min_dist = sqrt(pow(V[0]->location.first - point.first, 2) + 
				pow(V[0]->location.second - point.second, 2));
			int N = V.size();
			for (int i=1; i<N; i++)
			{
				double curr_dist = sqrt(pow(V[i]->location.first - point.first, 2) + 
					pow(V[i]->location.second - point.second, 2));
				if (curr_dist < min_dist)
				{
					min_dist = curr_dist;
					index = i;
				}
			}
			return V[index];
		}

		std::map<MapNode*, double>::iterator find_min_f(std::map<MapNode*, double>& _list)
		{
			std::map<MapNode*, double>::iterator itr = _list.begin(), result = _list.end();
			double min_dist = 999999;
			for(; itr != _list.end(); itr++)
			{
				double f = itr->second;
				if (f < min_dist)
				{
					min_dist = f;
					result = itr;
				}
			}

			return result;
		}

		std::vector<std::pair<float, float> > A_star_search()
		{
			refreshNodes();
			std::map<MapNode*, double> open_list;
			open_list.insert(std::make_pair(robot, robot->euclidean_dist_to_goal));
			MapNode* the_goal_node;
			bool goal_found = false;
			double total_path_cost = 0;

			while(!open_list.empty() && !goal_found)
			{
				std::map<MapNode*, double>::iterator next_to_expand = find_min_f(open_list);
				total_path_cost = next_to_expand->second - next_to_expand->first->euclidean_dist_to_goal;
				if (next_to_expand == open_list.end())
					return std::vector<std::pair<float, float> >();

				MapNode* next_node = next_to_expand->first;
				next_node->visited = true;
				open_list.erase(next_to_expand);

				std::vector<MapEdge*>::iterator it = next_node->neighbours.begin();
				for(; it != next_node->neighbours.end(); it++)
				{
					if ((*it)->neighbour->visited)
						continue;

					if ((*it)->neighbour->euclidean_dist_to_goal == 0)
					{
						goal_found = true;
						open_list.insert(std::make_pair((*it)->neighbour, total_path_cost + (*it)->cost));
						the_goal_node = (*it)->neighbour;
						break;
					}

					double f = total_path_cost + (*it)->cost + (*it)->neighbour->euclidean_dist_to_goal;
					open_list.insert(std::make_pair((*it)->neighbour, f));
				}
			}

			if (goal_found)
			{
				std::vector<std::pair<float, float> > path;
				while(the_goal_node != NULL)
				{
					path.push_back(the_goal_node->location);
					the_goal_node = the_goal_node->parent;
				}

				return path;
			}

			return std::vector<std::pair<float, float> >();
		}
};

MapTree *c_space;

std::pair<float, float> random_location_generator(int min, int max, unsigned int seed_val) 
{
	std::pair<float, float> rand_loc;
	srand(seed_val);

	int rand_x = rand() % max + min - 100;
	int rand_y = rand() % max + min - 100;
	rand_loc.first  = (float)rand_x / 10;
	rand_loc.second = (float)rand_y / 10;

	return rand_loc;
}

bool check_collision(float curr_x, float curr_y, float goal_x, float goal_y)
{
	bool no_collision = true;
	float A = goal_x * goal_x + goal_y * goal_y;
	std::map<std::string, std::pair<float, float> >::iterator itr = obstacles.begin();
	for(; itr != obstacles.end(); itr++)
	{
		std::pair<float, float> obs = itr->second;
		float B = 2 * (goal_x*(curr_x - obs.first) + goal_y*(curr_y - obs.second));
		float C = pow(curr_x - obs.first, 2) + pow(curr_y - obs.second, 2);
		float discriminant = B*B - 4*A*C;

		if (discriminant > 0)
		{
			no_collision = false;
			break;
		}
	}

	return no_collision;
}

MapNode* findNearestToGoal(MapTree* c_space, float gx, float gy)
{
	std::vector<MapNode*> reachables;
	std::vector<MapNode*>::iterator it = c_space->V.begin(), last = c_space->V.end(), ntg;
	for (; it != last; it++)
	{
		// we could add another constraint to check if the heuristic value of the nodes
		// do not exceed average heuristic value of all nodes
		if (check_collision((*it)->location.first, (*it)->location.second, gx, gy))
			reachables.push_back(*it);
	}

	ntg = reachables.end();
	double min_euclidean = 99999;
	for (it = reachables.begin(); it != reachables.end(); it++)
	{
		double heuristic = (*it)->euclidean_dist_to_goal;
		if (heuristic != 0 && heuristic < min_euclidean)
		{
			min_euclidean = heuristic;
			ntg = it;
		}
	}

	if (ntg == reachables.end())
		ROS_INFO("UNREACHABLE GOAL NODE!!!");
	else
	{
		ROS_INFO("Map generation is completed.");
		int v_size = c_space->V.size();
		for (int i=0; i<v_size; i++)
		{
			ROS_INFO("A vertex is found at (x, y, h): (%f, %f, %f)", c_space->V[i]->location.first, 
				c_space->V[i]->location.second, c_space->V[i]->euclidean_dist_to_goal);
		}
	}

	return *ntg;
}

void RapidRandomTree()
{
	unsigned int _seed = 11;
	int convergence_limit = 200;
	
	for(int i = 0; i < convergence_limit; i++, _seed += 2)
	{
		std::pair<float, float> random_location = random_location_generator(0, 200, _seed);
		ROS_INFO("Next point: (%f, %f).", random_location.first, random_location.second);
		MapNode *sub_nearest = c_space->findNearestTo(random_location);

		if (check_collision(random_location.first, random_location.second, sub_nearest->location.first, sub_nearest->location.second))
			continue;

		ROS_INFO("The sampled point has passed the collision check and will be added to c_space.");
		MapNode* added_node = c_space->addNode(random_location, sub_nearest);
		c_space->addEdge(sub_nearest, added_node);
	}

	MapNode* nearest_to_goal = findNearestToGoal(c_space, goalX, goalY);
	c_space->addEdge(nearest_to_goal, c_space->goal);
}

void publish_step_location(std::string uav)
{
	if (!trajectories[uav].empty())
	{
		std::pair<float, float> next_simple_loc = trajectories[uav].front();
		trajectories[uav].erase(trajectories[uav].begin());

		geometry_msgs::PoseStamped new_simple_goal;
		new_simple_goal.header.stamp = ros::Time(0);
		new_simple_goal.header.frame_id = "world";
		new_simple_goal.pose.position.x = next_simple_loc.first;
		new_simple_goal.pose.position.y = next_simple_loc.second;
		new_simple_goal.pose.position.z = 0.5;
		new_simple_goal.pose.orientation.x = 0;
		new_simple_goal.pose.orientation.y = 0;
		new_simple_goal.pose.orientation.z = 0;
		new_simple_goal.pose.orientation.w = 1;

		ROS_INFO("Publishing first simple location: (%f, %f).", next_simple_loc.first, next_simple_loc.second);
		uav_step.publish(new_simple_goal);
	}
	else
		ROS_INFO("Empty trajectory list... Something might have gone wrong!");
}

void modelStateCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
	int current_model_count = msg->name.size();
	if(current_model_count > total_model_count) 
	{
		robotX = msg->pose[1].position.x;
		robotY = msg->pose[1].position.y;
		//trajectories.insert(std::make_pair("quadrotor", std::vector<std::pair<float, float> >() ));

		for(int i=2; i<current_model_count; i++)
			obstacles.insert(std::make_pair(msg->name[i], std::make_pair(msg->pose[i].position.x, msg->pose[i].position.y)));

		total_model_count = current_model_count;
	}
}

void UAVGoalStepCallback(const hector_uav_msgs::Vector::ConstPtr& msg)
{
	// Reconsider for the messages that is come in the middle of some path planning / moving
	ROS_INFO("Got the ultimate goal: (%f, %f).", msg->x, msg->y);

	goalX = msg->x;
	goalY = msg->y;

	c_space = new MapTree(std::make_pair(goalX, goalY));
	c_space->setRobot(std::make_pair(robotX, robotY));
	
	RapidRandomTree();
	std::vector<std::pair<float, float> > P = c_space->A_star_search();
	trajectories.insert(std::make_pair("quadrotor", P));
	
	// publish first location
	publish_step_location("quadrotor");
}

void UAVDoneCallback(const hector_uav_msgs::Done::ConstPtr& msg)
{
	if (!trajectories["quadrotor"].empty())
	{
		ROS_INFO("Another simple step is going to be sent to the QuadController instance.");
		publish_step_location("quadrotor");
	}
	else 
	{
		/*std_msgs::String arrival_msg;

		if (robotX == goalX && robotY == goalY)
			arrival_msg.data = "SUCCESS";
		else 
			arrival_msg.data = "FAILURE";

		uav_arrived.publish(arrival_msg);*/
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "quad_motion_planner3");
	ros::NodeHandle node;

	ros::Subscriber model_sub = node.subscribe("/gazebo/model_states", 10, &modelStateCallback);
	uav_goal = node.subscribe("actual_uav_goal", 10, &UAVGoalStepCallback);
	uav_done = node.subscribe("/Done", 10, &UAVDoneCallback);
	uav_step = node.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
	uav_arrived = node.advertise<std_msgs::String>("ultimate_arrival", 10);

	ros::spin();
	return 0;
}