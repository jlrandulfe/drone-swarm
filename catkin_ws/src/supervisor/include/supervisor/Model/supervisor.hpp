#include "ros/ros.h"
#include "formation_control/Formation.h" //service
#include "pycopter/DroneSwarmMultiArray.h" //service
#include "pycopter/PycopterStartStop.h" //service



class Supervisor
{
public:
	Supervisor(ros::NodeHandle nh);
	~Supervisor();
	void setupSimulation(int amount_of_drones, float distance, float v_shape_angle, char shape, float range);
	void startSimulation();
	void stopSimulation();


private:
	void getFormation(int amount_of_drones, float distance, float v_shape_angle, char shape, float range);
	bool servicePyCopterCallback(pycopter::DroneSwarmMultiArray::Request  &req, pycopter::DroneSwarmMultiArray::Response &res);
	bool serviceKalmanCallback(pycopter::DroneSwarmMultiArray::Request  &req, pycopter::DroneSwarmMultiArray::Response &res);
	// void servicePyCopter(int amount_of_drones);
	ros::ServiceClient pycopter_client;

	bool is_simulation_running;

	ros::NodeHandle n;
	std::vector<std::vector<double> > connection_matrix;
	std::vector<std::vector<double> > start_pose;
	ros::ServiceServer pycopter_service;
	ros::ServiceServer kalman_service;
};