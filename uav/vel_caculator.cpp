#include <ros/ros.h>
#include <uav/UAVPose.h>
#include <vector>
#include <geometry_msgs/Twist.h>
#include <stdlib.h>
using namespace std;

class PoseType{
	public:
		double x;
		double y;
		double z;
		PoseType operator=(const PoseType& pose) {
			if(this != &pose){
				this->x = pose.x;
				this->y = pose.y;
				this->z = pose.z;
			}
			return *this;
		}
};

class VelHelp{
	
	private:
		PoseType linear;
		PoseType angular;
	public:
		VelHelp(){}
		void setVel(PoseType a,PoseType b){
			linear = a;
			angular = b;
		}
		
		double getLinearXValue(){
			return linear.x;
		}
		double getLinearYValue(){
			return linear.y;
		}
		double getLinearZValue(){
			return linear.z;
		}

};


VelHelp vel_handle;
void differenceHandle(const uav::UAVPose &msg){
	vel_handle.setVel(msg.linear, msg.angular);
	
}

int main(int argc,char** argv){
	ros::init(argc,argv,"vel_calculator");
	ros::NodeHandle nh;
	nh::Subscriber sub = nh.subscribe("Difference",1000,differenceHandle);
	return 0;
}
