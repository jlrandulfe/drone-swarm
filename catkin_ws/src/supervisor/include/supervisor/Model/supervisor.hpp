#include "ros/ros.h"
#include "formation_control/frame.hpp"
#include "formation_control/Formation.h" //service


class Supervisor
{
public:
	Supervisor(ros::NodeHandle nh);
	~Supervisor();
	void getFormation(int amount_of_drones, float distance, float v_shape_angle, char shape);

private:
	ros::NodeHandle n;
};