#include <ros/ros.h>
#include <hector_uav_msgs/PoseActionGoal.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <hector_uav_msgs/Done.h>
#include <hector_uav_msgs/EnableMotors.h>
#include <stdlib.h>


#define KP 1
#define KD 15
#define KI 0.015
#define MIN_VEL -5
#define MAX_VEL 5
#define MAX_LINKED_SIZE 100
#define EQUAL_CONST 0.5
bool desired_updated;
bool desired_achived;


bool isEqual(double a,double b){
	return abs(a-b)>EQUAL_CONST ? false:true;
}

bool shouldWait(){
	return (!desired_updated) && (desired_achived);
}

double goalX,goalY,goalZ;
double currentPositionX,currentPositionY,currentPositionZ;
class Linked{
	private:
		struct Node{
			double value;
			Node* next;
			Node* prev;
		};
		Node* head;
		Node* tail;
		int size;
		int maxsize;
		double sum;
	public:
		Linked(){	

		}
		void SetLinked(){
			size = 0;
			maxsize = MAX_LINKED_SIZE;
			sum = 0;
			head = NULL;
			tail = NULL;
		}
		
		~Linked(){
			Node * temp;
			Node * helper;
			temp = head;
			head = NULL;
			tail = NULL;
			if(temp){
				while(!temp->next){
					helper = temp;
					temp = temp->next;
					temp->prev = NULL;
					helper->next = NULL;
					delete helper;
				}
				delete temp;
			}
			
			
			
		}
		// Adds one new element to end of the linked list
		void AddNodeEnd(double value){
			Node* temp = new Node;
			temp->value = value;
			temp->next = NULL;
			if(head == NULL){
				temp->prev = NULL;
				head = temp;
			}
			else if(tail->prev == NULL){
				temp->prev = head;
				head->next = temp;
			}
			else{
				temp->prev = tail;
				tail->next = temp;
			}
			tail = temp;
			sum+= value;
			size++;
		}
		//Removes one element from the start of the linked list
		void DelNodeStart(){
			Node* temp = head;
			sum-=temp->value;
			if(head->next == NULL){
				delete temp;
			}
			else if(head->next == tail){
				tail->prev =NULL;
				head =tail;
				temp->next =NULL;
				delete temp;
			}
			else{
				head = head->next;
				head->prev=NULL;
				temp->next = NULL;
				delete temp;
			}
			
			size--;
		}
		
		double getSum(){
			return sum;
		}
		
		int getSize(){
			return size;
		}
		// first fills defined max size then starts removing from start while adding to end.
		void controlAdd(double value){
			if(size == maxsize){
				this->DelNodeStart();
				this->AddNodeEnd(value);
			}
			else
				this->AddNodeEnd(value);
		}
		
};
class PidVar{
	public:
		double dt;
		double max;
		double min;
		double Kp;
		double Kd;
		double Ki;
		double pre_error;
		Linked integral_help;
		PidVar(){ min = MIN_VEL;
			  max = MAX_VEL;
			  Kp =  KP;
			  Kd =  KD;
			  Ki =  KI;
			  pre_error = 0;
			  integral_help.SetLinked();
		}
		void SetVar(double _min, double _max, double _Kp, double _Kd, double _Ki){
			min = _min;
			max = _max;
			Kp = _Kp;
			Kd = _Kd;
			Ki = _Ki;
			pre_error = 0;
			integral_help.SetLinked();
		}
		void updateIntegral(double value){
			integral_help.controlAdd(value);
		}
		double getIntegral(){
			return integral_help.getSum();
		}
};

PidVar pidx;
PidVar pidy;
PidVar pidz;

double pidXCalculate(){
	double error = goalX-currentPositionX;
	double propor = error*pidx.Kp;
	double deriv = (error-pidx.pre_error)*pidx.Kd; //1
	pidx.updateIntegral(error);
	
	//double integ = (pidx.getIntegral())*pidx.Ki;
	pidx.pre_error = error;
	return propor + deriv;
}
double pidYCalculate(){
	double error = goalY-currentPositionY;
	double propor = error*pidy.Kp;
	double deriv = (error-pidy.pre_error)*pidy.Kd;
	pidy.updateIntegral(error);
	//double integ = (pidy.getIntegral())*pidy.Ki;
	pidy.pre_error = error;
	return propor + deriv;
}
double pidZCalculate(){
	double error = goalZ-currentPositionZ;
	double propor = error*pidz.Kp;
	double deriv = (error-pidz.pre_error)*pidz.Kd;
	pidz.updateIntegral(error);
	//double integ = (pidz.getIntegral())*pidz.Ki;
	pidz.pre_error = error;
	return propor + deriv ;
}


void fixForMax(std::vector<double>& holder){
	double div;
	int which = 0;
	double max = abs(holder[0]);
	if(abs(holder[1])>max){
		max = abs(holder[1]);
		which = 1;
	}

	if(abs(holder[2])>max){
		max = abs(holder[2]);
		which = 2;
	}

	if(max > MAX_VEL){
		div =MAX_VEL/max;
	}
	else
		return;
	for(int i=0;i<3;i++){
		holder[i] = holder[i]*div;
	}
	
}



void goalCallback(const hector_uav_msgs::PoseActionGoalPtr & goal){
	goalX = goal->goal.target_pose.pose.position.x;
	goalY = goal->goal.target_pose.pose.position.y;
	goalZ = goal->goal.target_pose.pose.position.z;

	ROS_INFO("Goal is set : %f,%f,%f",goalX,goalY,goalZ);

}





void currentPose_callback(const geometry_msgs::PoseStamped &msg){
	
	currentPositionX = msg.pose.position.x;
	currentPositionY = msg.pose.position.y;
	currentPositionZ = msg.pose.position.z;

	geometry_msgs::Twist pub_msg;
	ros::NodeHandle nh;	
	ros::Publisher pub_done = nh.advertise<hector_uav_msgs::Done>("Done",10);
	ros::Publisher pub_vel = nh.advertise<geometry_msgs::Twist>("/cmd_vel",10);
	
	if(shouldWait() == false){
		double x = pidXCalculate();
		double y = pidYCalculate();
		double z = pidZCalculate();
		// if UAV is in the correct position it will return to CommandDone
		if( isEqual(goalX, currentPositionX) && isEqual(goalY, currentPositionY) && isEqual(goalZ, currentPositionZ) ){
			
			
			hector_uav_msgs::Done done_send;
			done_send.commandDone = true;
			done_send.position.x = goalX;
			done_send.position.y = goalY;
			done_send.position.z = goalZ;
			done_send.orientation.x = 0;
			done_send.orientation.y = 0;
			done_send.orientation.z = 0;
			pub_done.publish(done_send);
		}
		// corrects the position of the quadro by giving velocity
		std::vector<double> holder;
		holder.resize(3);
		holder[0] = x;
		holder[1] = y;
		holder[2] = z;
		fixForMax(holder);
		pub_msg.linear.x = holder[0];
		pub_msg.linear.y = holder[1];
		pub_msg.linear.z = holder[2];

		ROS_INFO("Linear vel-x : %f, vel-y: %f, vel-z: %f",holder[0],holder[1],holder[2]);
		pub_vel.publish(pub_msg);
		 
	}
	
		
}




int main(int argc,char** argv){
	ros::init(argc,argv,"basic_flyer");
	ros::NodeHandle nh;
	
	ros::ServiceClient enable_motors = nh.serviceClient<hector_uav_msgs::EnableMotors>("/enable_motors");
	hector_uav_msgs::EnableMotors srv;

	ros::Subscriber goal_sub = nh.subscribe("/action/pose/goal",100, &goalCallback);
	ros::Subscriber sub_current = nh.subscribe("/ground_truth_to_tf/pose",1000,&currentPose_callback);
	srv.request.enable = true;
	/*if(enable_motors.call(srv)){
		if(srv.response.success){
			ROS_INFO("Motors are enabled");		
		}
	}*/
	
	ros::spin();
	return 0;
}
