#include "supervisor/Model/supervisor.hpp"

Supervisor::Supervisor(ros::NodeHandle nh): n(nh)
{
	printf("Created Supervisor\n");
	is_simulation_running = false;
}

Supervisor::~Supervisor()
{

}


/*
	SYSTEM SETUP
		- call the formation design
		- send connection matrix to error_estimator & controller node (might be merged) 
		- send connection matrix to kalman_filter node
		- send number of drones and starting locations to pycopter
	
	SYSTEM CYCLE
		- get positions & velocities from pycopter (topic "pycopter/positions" and "pycopter/velocities")
		- pass this to kalman filter
		- kalman filter sends estimated distances to error estimator
		- error estimator sends distance error to controller
		- controller sends control action (how to move) as a velocity
		- velocity gets sent to pycopter

*/


	
void Supervisor::setupSimulation(int amount_of_drones, float distance, float v_shape_angle, char shape, float range, float resolution, float simtime, int movementPattern_, float x_vel_, float y_vel_, float sinusoid_freq, float noise_constant_)
{
	simTime = simtime;
	simRes = resolution;
	movementPattern = movementPattern_;
	x_vel = x_vel_;
	y_vel = y_vel_;
	noise_constant = noise_constant_;
	
	this->getFormation(amount_of_drones, distance, v_shape_angle, shape, range);
	
	//publish connection matrix to error_estimator/controller node and kalman_filter node
	if(!ros::service::exists("supervisor/kalman", false))
	{
		printf("Starting KalmanService\n");
		kalman_service = n.advertiseService("supervisor/kalman", &Supervisor::serviceKalmanCallback, this);
	}
	else
		printf("KalmanService already exists\n");
	
	//publish starting locations and number of drones to pycopter
	if(!ros::service::exists("supervisor/pycopter", false))
	{
		printf("Starting PyCopterService\n");
		pycopter_service = n.advertiseService("supervisor/pycopter", &Supervisor::servicePyCopterCallback, this);
	}
	else
		printf("PyCopterService already exists\n");
	
	printf("Waiting for Pycopter to set up Start Stop Simulation service\n");
	ros::service::waitForService("pycopter/start_stop");
	pycopter_client = n.serviceClient<pycopter::PycopterStartStop>("pycopter/start_stop");
	
}

void Supervisor::startSimulation()
{
	printf("Starting Simulation\n");
	pycopter::PycopterStartStop srv;
	srv.request.start = true;
	srv.request.stop = false;
	if(pycopter_client.call(srv))
  	{
  		is_simulation_running = srv.response.ack;
  	}
  	else
  		printf("Service call to pycopter/start_stop failed\n");
}

void Supervisor::stopSimulation()
{
	printf("Stopping Simulation\n");
	pycopter::PycopterStartStop srv;
	srv.request.start = false;
	srv.request.stop = true;
	if(pycopter_client.call(srv))
  	{
  		is_simulation_running = srv.response.ack;
  	}
  	else
  		printf("Service call to pycopter/start_stop failed\n");

	if(ros::service::exists("supervisor/kalman", false))
	{
		printf("Stopping KalmanService\n");
		kalman_service.shutdown();
	}
	else
		printf("KalmanService doesnt exists\n");
	
	if(ros::service::exists("supervisor/pycopter", false))
	{
		printf("Stopping PyCopterService\n");
		pycopter_service.shutdown();
	}
	else
		printf("PyCopterService doesnt exists\n");	
}


bool Supervisor::servicePyCopterCallback(pycopter::DroneSwarmMultiArray::Request  &req, pycopter::DroneSwarmMultiArray::Response &res)
{
	printf("PyCopter Service called\n");
	std::vector<double> start_pose_data(2*start_pose.size());	
	int iterator = 0;
	for(int i = 0; i < start_pose.size(); i++)
	{
		start_pose_data[iterator] = start_pose[i][0];//x
		iterator++;
		start_pose_data[iterator] = start_pose[i][1];//y
		iterator++;
	}
	res.data = start_pose_data;
	res.n_rows = start_pose.size();
	res.n_cols = 2;
	res.param1 = simTime;
	res.param2 = simRes;

	return true;
}

bool Supervisor::serviceKalmanCallback(pycopter::DroneSwarmMultiArray::Request  &req, pycopter::DroneSwarmMultiArray::Response &res)
{
	printf("Kalman Service called\n");
	std::vector<double> connection_data(connection_matrix.size()*connection_matrix.size());	
	int iterator = 0;
	for(int i = 0; i < connection_matrix.size(); i++)
		for (int j = 0; j < connection_matrix.size(); ++j)
		{
			connection_data[iterator] = connection_matrix[i][j];
			iterator++;
		}
	res.data = connection_data;
	res.n_rows = res.n_cols = connection_matrix.size();
	res.param1 = x_vel;
	res.param2 = y_vel;
	res.param3 = sinusoid_freq;
	res.param4 = movementPattern;
	res.param5 = noise_constant;
	return true;
}


void Supervisor::getFormation(int amount_of_drones, float distance, float v_shape_angle, char shape, float range)
{
  	ros::ServiceClient client = n.serviceClient<formation_control::Formation>("formation_control");
  	formation_control::Formation srv;
  	srv.request.amount_of_drones = amount_of_drones;
	srv.request.shape_type = shape;
	srv.request.distance = distance;
	srv.request.angle = v_shape_angle;
	srv.request.random_range = range;

	ros::service::waitForService("formation_control", 10);
	if (client.call(srv))
  	{
		std::cout << "Matrix Size: " << srv.response.matrix_size << std::endl;
		int iterator = 0;
		std::vector<double> temp_storage;
		for (int i = 0; i < srv.response.matrix_size; ++i)
		{
			for (int j = 0; j < srv.response.matrix_size; ++j)
			{
				temp_storage.push_back(srv.response.connection_matrix[iterator]);
				iterator++;
			}
			connection_matrix.push_back(temp_storage);
			temp_storage.clear();
		}
		iterator = 0;
		for(int i = 0; i < amount_of_drones; i++)
		{
			temp_storage.push_back(srv.response.start_pose[iterator]);
			iterator++;
			temp_storage.push_back(srv.response.start_pose[iterator]);
			iterator++;
			start_pose.push_back(temp_storage);
			temp_storage.clear();
		}
		iterator = 0;
		std::cout << "Printing response\nConnection Matrix" << std::endl;
		for (int i = 0; i < srv.response.matrix_size; ++i)
		{
			for (int j = 0; j < srv.response.matrix_size; ++j)
			{
				std::cout << connection_matrix[i][j] << ", ";
				iterator++;
			}
			std::cout << std::endl;
		}
		std::cout << "Start Positions" << std::endl;
		for (int i = 0; i < amount_of_drones; ++i)
		{
				std::cout << "Drone " << i << "  x: " << start_pose[i][0] << ", y: " << start_pose[i][1] << std::endl;
				iterator++;
		}

  	}
  	else
  	{
		ROS_ERROR("Failed to call service formation_control");
  	}
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "supervisor");
  	ros::NodeHandle n;

	Supervisor sup(n);


	// sup.setupSimulation(5, 5, 20, 'g', 0.9, 100, 5);
	ros::spin();
	return 0;
}


