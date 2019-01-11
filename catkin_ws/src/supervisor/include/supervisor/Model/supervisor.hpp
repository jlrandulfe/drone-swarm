#include "ros/ros.h"
#include "formation_control/Formation.h" //service
#include "formation_control/frame.hpp"


class Supervisor
{
public:
	Supervisor(ros::NodeHandle nh);
	~Supervisor();
	void getFormation(int amount_of_drones, float distance, float v_shape_angle, char shape);

private:
	ros::NodeHandle n;
};