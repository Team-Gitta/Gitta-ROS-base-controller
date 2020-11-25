#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"

const double RADIUS_WHEEL = 0.0485;
const double L_X = 0.109; //half of the distance between front wheel and rear wheel
const double L_Y = 0.121; //half of the distance between fron wheels


class Transformation{
private:
	double _inversedJacobian[4][3];
	double _radius_wheel;
	double _l_x;
	double _l_y;

	void setJacobian(){
		double l_sum = getL_X() + getL_Y();
		_inversedJacobian[0][0] = 1.0 / _radius_wheel; _inversedJacobian[0][1] = -1.0 / _radius_wheel; _inversedJacobian[0][2] = -l_sum / _radius_wheel;
	        _inversedJacobian[1][0] = 1.0 / _radius_wheel; _inversedJacobian[1][1] = 1.0 / _radius_wheel; _inversedJacobian[1][2] = l_sum / _radius_wheel;
	        _inversedJacobian[2][0] = 1.0 / _radius_wheel; _inversedJacobian[2][1] = 1.0 / _radius_wheel; _inversedJacobian[2][2] = -l_sum / _radius_wheel;
	        _inversedJacobian[3][0] = 1.0 / _radius_wheel; _inversedJacobian[3][1] = -1.0 / _radius_wheel; _inversedJacobian[3][2] = l_sum / _radius_wheel;
	}
public:
	Transformation(double radius=RADIUS_WHEEL, double l_x=L_X, double l_y=L_Y){
	       _radius_wheel = radius;
	       _l_x = l_x;
	       _l_y = l_y;
	       setJacobian();
	}
	void setL_X(double l_x) {_l_x = l_x; setJacobian();}
	void setL_Y(double l_y) {_l_y = l_y; setJacobian();}
	double getL_X() {return _l_x;}
	double getL_Y() {return _l_y;}
	void transformToWheels(double base[], double wheels[]){
		for(int i=0; i<4; i++){
			double product = 0.0;
			for(int j=0; j<3; j++){
				product += _inversedJacobian[i][j] * base[j]; 
			}
			wheels[i] = product;
		}
	}

};


void velCallback(const geometry_msgs::Twist& msg)
{
	ROS_INFO_STREAM("Subscribed velocity Vx:"<<msg.linear.x<<" Vy:"<<msg.linear.y<<" angular:"<<msg.angular.z);
	Transformation transform;
	double base[3] = {msg.linear.x, msg.linear.y, msg.angular.z};
	double wheels[4];
	transform.transformToWheels(base, wheels);
	ROS_INFO_STREAM("W1:"<<wheels[0]<<" W2:"<<wheels[1]<<" W3:"<<wheels[2]<<" W4:"<<wheels[3]);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "base_controller");
  ros::NodeHandle n;

  // Subscribe "cmd_vel"
  ros::Subscriber sub = n.subscribe("cmd_vel", 1000, velCallback);

  ros::spin();

  return 0;
}
