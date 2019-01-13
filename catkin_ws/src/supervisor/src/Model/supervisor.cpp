#include "supervisor/Model/supervisor.hpp"

Supervisor::Supervisor(ros::NodeHandle nh): n(nh)
{
	printf("Created Supervisor\n");
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


	
void Supervisor::setupSimulation(int amount_of_drones, float distance, float v_shape_angle, char shape, float range)
{
	this->getFormation(amount_of_drones, distance, v_shape_angle, shape, range);
	//publish connection matrix to error_estimator/controller node and kalman_filter node
	printf("Starting KalmanService\n");
	kalman_service = n.advertiseService("supervisor/kalman", &Supervisor::serviceKalmanCallback, this);
	//publish starting locations and number of drones to pycopter
	printf("Starting PyCopterService\n");
	pycopter_service = n.advertiseService("supervisor/pycopter", &Supervisor::servicePyCopterCallback, this);
}


bool Supervisor::servicePyCopterCallback(pycopter::PycopterStartPositions::Request  &req, pycopter::PycopterStartPositions::Response &res)
{
	printf("PyCopterService called\n");
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
	res.matrix_size = start_pose.size();
	return true;
}

bool Supervisor::serviceKalmanCallback(formation_control::Formation::Request  &req, formation_control::Formation::Response &res)
{
	printf("PyCopterService called\n");

	/*

		int iterator = 0;
		for(int i = 0; i < amount_of_drones; i++)
		{
			start_pose_data[iterator] = random_positions[i].x;//x
			iterator++;
			start_pose_data[iterator] = random_positions[i].y;//x
			iterator++;
		}

	res.data = 

	*/
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
	// float result[amount_of_drones][amount_of_drones];

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
				// result[i][j] = srv.response.connection_matrix[iterator];
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

bool add(formation_control::Formation::Request  &req, formation_control::Formation::Response &res)
{
	std::cout << "Received Service Call" << std::endl;
	
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "supervisor");
  	ros::NodeHandle n;

	Supervisor sup(n);


	sup.setupSimulation(5, 5, 20, 'g', 0.9);
	ros::spin();
	return 0;
}


