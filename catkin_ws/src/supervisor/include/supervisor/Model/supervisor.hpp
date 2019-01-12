#include "ros/ros.h"
#include "formation_control/Formation.h" //service
#include "std_msgs/Float64MultiArray.h"


class Supervisor
{
public:
	Supervisor(ros::NodeHandle nh);
	~Supervisor();
	void setupSimulation(int amount_of_drones, float distance, float v_shape_angle, char shape, float range);

private:
	void getFormation(int amount_of_drones, float distance, float v_shape_angle, char shape, float range);
	bool servicePyCopterCallback(formation_control::Formation::Request  &req, formation_control::Formation::Response &res);
	bool serviceKalmanCallback(formation_control::Formation::Request  &req, formation_control::Formation::Response &res);
	// void servicePyCopter(int amount_of_drones);


	ros::NodeHandle n;
	std::vector<std::vector<float> > connection_matrix;
	std::vector<std::vector<float> > start_pose;
	ros::ServiceServer pycopter_service;
	ros::ServiceServer kalman_service;
};