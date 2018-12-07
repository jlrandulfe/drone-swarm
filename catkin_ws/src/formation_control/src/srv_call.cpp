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
  	ros::init(argc, argv, "add_two_ints_client");
  	ros::NodeHandle n;
	
	int amount_of_drones = 10;
	float distance = 10;
	float v_shape_angle = 45;
	char shape = 'g';
  	
  	ros::ServiceClient client = n.serviceClient<formation_control::Formation>("formation_control");
  	formation_control::Formation srv;
  	srv.request.amount_of_drones = amount_of_drones;
	srv.request.shape_type = shape;
	srv.request.distance = distance;
	srv.request.angle = v_shape_angle;
	float result[amount_of_drones][amount_of_drones];
  	
  	if (client.call(srv))
  	{
		std::cout << "Matrix Size: " << srv.response.matrix_size << std::endl;
		int iterator = 0;
		for (int i = 0; i < srv.response.matrix_size; ++i)
			for (int j = 0; j < srv.response.matrix_size; ++j)
			{
				result[i][j] = srv.response.data[iterator];
				iterator++;
			}

		std::cout << "Printing response" << std::endl;
		for (int i = 0; i < srv.response.matrix_size; ++i)
		{
			for (int j = 0; j < srv.response.matrix_size; ++j)
			{
				std::cout << result[i][j] << ", ";
				iterator++;
			}
			std::cout << std::endl;
		}

  	}
  	else
  	{
		ROS_ERROR("Failed to call service add_two_ints");
		return 1;
  	}

  	return 0;
}