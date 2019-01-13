#include "ros/ros.h"
#include "formation_control/Formation.h" //service


/*

IMPORTANT!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

the distance value differs in function per shape type:
	(p) polygon -> the distance specified is the radius of the polygon
	(g) grid -> the distance specified is the full length of the side of the grid (aka side of a square)
	(v) v-shape -> the distance specified is the distance between drones along the v lines, this call also uses the angle value
*/
int main(int argc, char **argv)
{
	ros::init(argc, argv, "srv_call");
  	ros::NodeHandle n;
	
	int amount_of_drones = 10;
	float distance = 10;
	float v_shape_angle = 45;
	char shape = 'g';
	float random_range = 0.9;
  	
  	ros::ServiceClient client = n.serviceClient<formation_control::Formation>("formation_control");
  	formation_control::Formation srv;
  	srv.request.amount_of_drones = amount_of_drones;
	srv.request.shape_type = shape;
	srv.request.distance = distance;
	srv.request.angle = v_shape_angle;
	srv.request.random_range = random_range;
	float connection_matrix_data[amount_of_drones][amount_of_drones];
  	float start_pose_data[amount_of_drones][2];
  	if (client.call(srv))
  	{
		std::cout << "Matrix Size: " << srv.response.matrix_size << std::endl;
		int iterator = 0;
		for (int i = 0; i < srv.response.matrix_size; ++i)
			for (int j = 0; j < srv.response.matrix_size; ++j)
			{
				connection_matrix_data[i][j] = srv.response.connection_matrix[iterator];
				iterator++;
			}
		iterator = 0;
		for(int i = 0; i < amount_of_drones; i++)
		{
			start_pose_data[i][0] = srv.response.start_pose[iterator];
			iterator++;
			start_pose_data[i][1] = srv.response.start_pose[iterator];
			iterator++;
		}
		iterator = 0;
		std::cout << "Printing response\nConnection Matrix" << std::endl;
		for (int i = 0; i < srv.response.matrix_size; ++i)
		{
			for (int j = 0; j < srv.response.matrix_size; ++j)
			{
				std::cout << connection_matrix_data[i][j] << ", ";
				iterator++;
			}
			std::cout << std::endl;
		}
		std::cout << "Start Positions" << std::endl;
		for (int i = 0; i < amount_of_drones; ++i)
		{
				std::cout << "Drone " << i << "  x: " << start_pose_data[i][0] << ", y: " << start_pose_data[i][1] << std::endl;
				iterator++;
		}
	
  	}
  	else
  	{
		ROS_ERROR("Failed to call service formation_control");
		return 1;
  	}

  	return 0;
}