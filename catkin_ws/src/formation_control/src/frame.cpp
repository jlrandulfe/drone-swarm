#define DEBUG 0
#define _USE_MATH_DEFINES

#include "ros/ros.h"
#include <math.h>
#include "formation_control/frame.hpp"
#include "formation_control/Formation.h"


Frame::Frame(int N_drones_, float radius_, char shape, float v_shape_angle_)
{
	N_drones = N_drones_;
	radius = radius_;
	std::vector<std::vector<int> > connected_nodes_(N_drones);
	connected_nodes = connected_nodes_;
	v_shape_angle = (v_shape_angle_*M_PI)/180;
	if (shape == 'p')
	{
		p_vector = this->createPolygonCoordinates(N_drones, radius);
		std::vector< std::vector <float> > blah = this->PolygonIncidenceMatrix();
		this->createDistanceMatrixForOuput();
	}
	else if(shape == 'g')
	{
		p_vector = this->createGridCoordinates(N_drones, radius);
		std::vector< std::vector <float> > blah = this->GridIncidenceMatrix();
		this->createDistanceMatrixForOuput();
	}
	else if(shape == 'v')
	{
		p_vector = this->createVCoordinates(N_drones, radius, v_shape_angle);
		std::vector< std::vector <float> > blah = this->VIncidenceMatrix();
		this->createDistanceMatrixForOuput();
	}
	else
	{
		std::cout << "Wrong input to shape" << std::endl;
		exit(0);
	}

	if(DEBUG) this->printConnections();
}

Frame::~Frame(){}


std::vector<Frame::point> Frame::createPolygonCoordinates(int vertices, float radius)
{
	std::vector<Frame::point> result;
	if(DEBUG) std::cout << "Creating polygon with " << vertices << " sides and a radius of " << radius << std::endl;

	float angle = 2*M_PI/vertices;
	if(DEBUG) std::cout << "angle determined is " << angle << std::endl;
	Frame::point calc_point;
	for(int i = 0; i < vertices; i++)
	{
		calc_point.x = radius*sin(i*angle);
		if(calc_point.x < 0.000001 && calc_point.x > -0.000001)
			calc_point.x = 0;
		calc_point.y = radius*cos(i*angle);
		if(calc_point.y < 0.000001 && calc_point.y > -0.000001)
			calc_point.y = 0;
		result.push_back(calc_point);
	}
	if(DEBUG) std::cout << result.size() << " vertices coordinates were created:" << std::endl;
	
	if(DEBUG) for (int i = 0; i < result.size(); ++i)
		std::cout << " " << i << ": " << result[i].x << ", " << result[i].y << std::endl;
	
	return result;
}

std::vector<Frame::point> Frame::createGridCoordinates(int drones_amount, float shape_side_length)
{
	std::vector<Frame::point> result;
	double sqrt_value = sqrt(drones_amount);
	int drones_per_side;
	int drones_left = 0;
	float side_length_x, side_length_y;

	if(sqrt_value - (int)sqrt_value > 0.0)
	{
		if(DEBUG) std::cout << "Value is not a full grid" << std::endl;
		drones_per_side = (int) sqrt_value;
		drones_left = drones_amount - drones_per_side*drones_per_side;
		side_length_x = shape_side_length / (drones_per_side);
		if(drones_left - drones_per_side <= 0)
			side_length_y = shape_side_length / (drones_per_side-1);
		else
			side_length_y = shape_side_length / (drones_per_side);
	}
	else
	{
		if(DEBUG) std::cout << "Value is a full grid" << std::endl;
		drones_per_side = sqrt_value;
		side_length_x =  shape_side_length / (drones_per_side-1);
		side_length_y = shape_side_length / (drones_per_side-1);
	}
	if(DEBUG) std::cout << "Length x: " << side_length_x << ", length y: " << side_length_y << std::endl;

	if(DEBUG) std::cout << "drones_amount: " << drones_amount << ", drones_per_side: " << drones_per_side << ", drones_left: " << drones_left << std::endl;
	int iterator_x = 0;
	int iterator_y = 0;
	Frame::point coordinate;

	//create full grid
	for(int i = 0; i < drones_per_side*drones_per_side; i++)
	{
		if(iterator_x == drones_per_side)
			iterator_x = 0;
		if(i % drones_per_side == 0 && i != 0)
			iterator_y++;

		coordinate.x = iterator_x*side_length_x;
		iterator_x++;
		coordinate.y = iterator_y*side_length_y;
		result.push_back(coordinate);
	}
	
	//add any extra drones to grid
	for(int i = 0; i < drones_left; i++)
	{
		if(i < drones_per_side)
		{
			coordinate.x = drones_per_side*side_length_x;
			coordinate.y = i*side_length_y;
			result.push_back(coordinate);
		}
		else
		{
			coordinate.x = (i-drones_per_side)*side_length_x;
			coordinate.y = drones_per_side*side_length_y;
			result.push_back(coordinate);
		}
	}
	if(DEBUG) std::cout << result.size() << " vertices coordinates were created:" << std::endl;
	
	if(DEBUG) for (int i = 0; i < result.size(); ++i)
		std::cout << " " << i << ": " << result[i].x << ", " << result[i].y << std::endl;
	
	return result;
}

std::vector<Frame::point> Frame::createVCoordinates(int vertices, float seperation, float angle)
{
	std::vector<Frame::point> result;
	Frame::point coordinate;
	coordinate.x = coordinate.y = 0;
	result.push_back(coordinate);
	int seperation_iterator = 1;
	for(int i = 1; i < vertices; i++)
	{
		if(i % 2 != 0)
		{
			coordinate.x = seperation_iterator*seperation*sin(M_PI + angle);
			coordinate.y = seperation_iterator*seperation*cos(M_PI + angle);
			result.push_back(coordinate);
		}
		else
		{
			coordinate.x = seperation_iterator*seperation*sin(M_PI - angle);
			coordinate.y = seperation_iterator*seperation*cos(M_PI - angle);
			result.push_back(coordinate);
			seperation_iterator++;
		}

	}

	if(DEBUG) std::cout << result.size() << " vertices coordinates were created:" << std::endl;
	if(DEBUG) for (int i = 0; i < result.size(); ++i)
		std::cout << " " << i << ": " << result[i].x << ", " << result[i].y << std::endl;
	
	return result;
}


std::vector< std::vector <float> > Frame::GridIncidenceMatrix()
{
	std::vector< std::vector <float> > incidence_matrix;
	std::vector <float> columns_matrix;
	std::vector <float> node_connection_check;
	int z = 0;
	for(int columns = 0; columns < 2*N_drones - 3; columns++)
		columns_matrix.push_back(0);
	for(int rows = 0; rows < N_drones; rows++)
		incidence_matrix.push_back(columns_matrix);
	for(int i = 0; i < N_drones; i++)
		node_connection_check.push_back(0);

	this->calcDistances();

	double sqrt_value = sqrt(N_drones);
	double fract_part;
	modf(sqrt_value, &fract_part);
	int drones_per_side;
	int drones_left = 0;
	float side_length_x, side_length_y;
	int iterator_x = 0;
	int iterator_y = 0;

	if(fract_part != 0)
	{
		drones_per_side = (int) sqrt_value;
		drones_left = N_drones - drones_per_side*drones_per_side;
	}
	else
		drones_per_side = sqrt_value;

	for(int i = 0; i < drones_per_side*drones_per_side-1; i++)
	{
		iterator_x++;

		if(iterator_y == drones_per_side - 1)
		{
			if(DEBUG) std::cout << "Reached End of Column :: connecting " << i << " to " << i + 1 << std::endl;
			this->addZtoIncidenceMatrixNoTracking(incidence_matrix, i, i + 1, z);
		}
		else if(iterator_x == drones_per_side)
		{
			if(DEBUG) std::cout << "Reached End of Row:: connecting " << i << " to " << i + drones_per_side << std::endl;
			this->addZtoIncidenceMatrixNoTracking(incidence_matrix, i, i + drones_per_side, z);
			iterator_x = 0;
			iterator_y++;
		}
		else
		{
			if(DEBUG) std::cout << "Within Grid :: connecting " << i << " to " << i + 1 << ", " << i + drones_per_side << ", " << i + drones_per_side + 1 <<std::endl;

			//right node
			this->addZtoIncidenceMatrixNoTracking(incidence_matrix, i, i + 1, z);
			//bottom node
			this->addZtoIncidenceMatrixNoTracking(incidence_matrix, i, i + drones_per_side, z);
			//diagonal node
			this->addZtoIncidenceMatrixNoTracking(incidence_matrix, i, i + drones_per_side + 1, z);
		}
	}
	if(drones_left > 0)
	{
		int start_iter = drones_per_side*drones_per_side;
		for (int i = 0; i < drones_left; ++i)
		{
			if(i == drones_per_side)
			{
				std::cout << "Filled one side, breaking" << std::endl;
				break;
			}	

			if(i == 0)
			{
				if(DEBUG) std::cout << "First Extra drone :: connecting " << start_iter << " to " << drones_per_side-1 << ", " << 2*drones_per_side-1 << std::endl;
				this->addZtoIncidenceMatrixNoTracking(incidence_matrix, start_iter, drones_per_side-1, z);
				this->addZtoIncidenceMatrixNoTracking(incidence_matrix, start_iter, 2*drones_per_side-1, z);
			}
			else if(i == 1)
			{
				if(DEBUG) std::cout << "Second Extra drone :: connecting " << start_iter + i << " to " << start_iter << ", " << 2*drones_per_side-1 << std::endl;
				this->addZtoIncidenceMatrixNoTracking(incidence_matrix, i + start_iter, 2*drones_per_side-1, z);
				this->addZtoIncidenceMatrixNoTracking(incidence_matrix, i + start_iter, start_iter, z);
			}
			else
			{
				if(DEBUG) std::cout << i << " Extra drone :: connecting " << start_iter + i << " to " << i + start_iter - 1 << ", " << (i+1)*drones_per_side-1 << " with interconnection between " << i + start_iter - 1 << " and " << (i+1)*drones_per_side-1 << std::endl;
				this->addZtoIncidenceMatrixNoTracking(incidence_matrix, i + start_iter, i + start_iter - 1, z);
				this->addZtoIncidenceMatrixNoTracking(incidence_matrix, i + start_iter, (i+1)*drones_per_side-1, z);
				this->addZtoIncidenceMatrixNoTracking(incidence_matrix, i + start_iter - 1, (i+1)*drones_per_side-1, z);
			}
		}
		int bottom_drones_left = drones_left - drones_per_side;
		start_iter = drones_per_side*(drones_per_side + 1);
		if(bottom_drones_left > 0)
		{
			std::cout << "Adding extra drones" << std::endl;
			for (int i = 0; i < bottom_drones_left; ++i)
			{
				if(i == 0)
				{
					if(DEBUG) std::cout << "First Extra drone :: connecting " << start_iter << " to " << (drones_per_side-1)*drones_per_side << ", " << ((drones_per_side-1)*drones_per_side) + 1 << std::endl;
					this->addZtoIncidenceMatrixNoTracking(incidence_matrix, start_iter, (drones_per_side-1)*drones_per_side, z);
					this->addZtoIncidenceMatrixNoTracking(incidence_matrix, start_iter, ((drones_per_side-1)*drones_per_side) + 1, z);
				}
				else if(i == 1)
				{
					if(DEBUG) std::cout << "Second Extra drone :: connecting " << start_iter + i << " to " << ((drones_per_side-1)*drones_per_side) + 1 << ", " << start_iter << std::endl;
					this->addZtoIncidenceMatrixNoTracking(incidence_matrix, i + start_iter, ((drones_per_side-1)*drones_per_side) + 1, z);
					this->addZtoIncidenceMatrixNoTracking(incidence_matrix, i + start_iter, start_iter, z);
				}
				else
				{
					if(DEBUG) std::cout << i+1 << "th Extra drone :: connecting " << start_iter + i << " to " << i + start_iter - 1 << ", " << ((drones_per_side-1)*drones_per_side) + i << " with interconnection between " << i + start_iter - 1 << " and " << ((drones_per_side-1)*drones_per_side) + i << std::endl;
					this->addZtoIncidenceMatrixNoTracking(incidence_matrix, i + start_iter, i + start_iter - 1, z);
					this->addZtoIncidenceMatrixNoTracking(incidence_matrix, i + start_iter, ((drones_per_side-1)*drones_per_side) + i, z);
					this->addZtoIncidenceMatrixNoTracking(incidence_matrix, i + start_iter - 1, ((drones_per_side-1)*drones_per_side) + i, z);
				}
			}
		}

		
	}
	return incidence_matrix;
}

std::vector< std::vector <float> > Frame::VIncidenceMatrix()
{
	std::vector< std::vector <float> > incidence_matrix;
	std::vector <float> columns_matrix;
	std::vector <float> node_connection_check;
	int z = 0;

	for(int columns = 0; columns < 2*N_drones - 3; columns++)
		columns_matrix.push_back(0);
	for(int rows = 0; rows < N_drones; rows++)
		incidence_matrix.push_back(columns_matrix);
	for(int i = 0; i < N_drones; i++)
		node_connection_check.push_back(0);

	this-> calcDistances();

	this->addZtoIncidenceMatrixNoTracking(incidence_matrix, 0, 1, z);
	this->addZtoIncidenceMatrixNoTracking(incidence_matrix, 0, 2, z);
	this->addZtoIncidenceMatrixNoTracking(incidence_matrix, 2, 1, z);\

	for(int i = 3; i < N_drones; i++)
	{
		this->addZtoIncidenceMatrixNoTracking(incidence_matrix, i, i-1, z);
		this->addZtoIncidenceMatrixNoTracking(incidence_matrix, i, i-2, z);
	}

	return incidence_matrix;
}


std::vector< std::vector <float> > Frame::PolygonIncidenceMatrix()
{
	std::vector< std::vector <float> > incidence_matrix;
	std::vector <float> columns_matrix;
	std::vector <float> node_connection_check;

	//Create empty matrices
	for(int columns = 0; columns < 2*N_drones - 3; columns++)
		columns_matrix.push_back(0);
	for(int rows = 0; rows < N_drones; rows++)
		incidence_matrix.push_back(columns_matrix);
	for(int i = 0; i < N_drones; i++)
		node_connection_check.push_back(0);

	if(DEBUG) std::cout << "Empty Incidence Matrix" << std::endl;
	if(DEBUG) this->printMatrix(incidence_matrix);
	//Calculate Distances between all nodes

	this->calcDistances();

	//Sort Distances for each node
	std::vector<std::vector<sort_struct> > full_distance_vector_sorted;
	std::vector<sort_struct> temp_vector;
	for(int i = 0; i < full_distance_vector.size(); i++)
	{
		temp_vector = this->sortDistances(full_distance_vector[i]);
		full_distance_vector_sorted.push_back(temp_vector);
	}
	
	if(DEBUG) std::cout << "Distance Matrix Sorted" << std::endl;
	if(DEBUG) this->printMatrixSort(full_distance_vector_sorted, true);


	int amount_of_edges = 2*N_drones - 3;
	int node = 0;
	std::vector<std::vector<int> > existing_connections;
	std::vector<int> nodes_completed;

	//make seperate triangles within the shape by creating a triangle between two closest points, and then going to the closest points to one of the vertices of
	//	the new triangle. remember to remove the first node from consideration.	
	int current_edge = 0;
	bool connection_exists	;
	bool incomplete_node = false;
	std::vector<int> connection(2);
	int choose_parent_node = 3;
	this->addZtoIncidenceMatrix(incidence_matrix, node, full_distance_vector_sorted[node][1].index, current_edge, existing_connections);
	this->addZtoIncidenceMatrix(incidence_matrix, node, full_distance_vector_sorted[node][2].index, current_edge, existing_connections);
	this->addZtoIncidenceMatrix(incidence_matrix, full_distance_vector_sorted[node][2].index, full_distance_vector_sorted[node][1].index, current_edge, existing_connections);
	nodes_completed.push_back(node);

	for(current_edge = 4; current_edge < amount_of_edges+1; current_edge++)
	{
		current_edge--;
		if(current_edge < amount_of_edges)
		{

			incomplete_node = false;
			while(!incomplete_node)
			{
				node = full_distance_vector_sorted[node][choose_parent_node].index;
				if(DEBUG) std::cout << "   Checking node " << node;
				if(std::find(nodes_completed.begin(), nodes_completed.end(), node) != nodes_completed.end())
				{
					choose_parent_node++;
					if(DEBUG) std::cout << "   Tried to choose complete node!!!!!" << std::endl;
				}
				else
				{
					incomplete_node = true;
					if(DEBUG) std::cout << "   Chose good node" << std::endl;
				}
			}
		}


		if(DEBUG) std::cout << "Node selected as parent: " << node << std::endl;
		int closest_neighbour_iterator = 1;
		int nearest_point_one, nearest_point_two;
		//Check if attempting to connect to complete node
		incomplete_node = false;
		while(!incomplete_node)
		{
			nearest_point_one = full_distance_vector_sorted[node][closest_neighbour_iterator].index;
			if(DEBUG) std::cout << "   checking nearest node 1 :" << nearest_point_one << " is complete: ";
			if(this->checkVectorForElement(nodes_completed, nearest_point_one))
			{
				closest_neighbour_iterator++;
				if(DEBUG) std::cout << "true";
			}
			else
			{
				incomplete_node = true;
				if(DEBUG) std::cout << "false";
			}
			if(DEBUG) std::cout << std::endl;
		}
		closest_neighbour_iterator++;
		incomplete_node = false;
		while(!incomplete_node )
		{
			nearest_point_two = full_distance_vector_sorted[node][closest_neighbour_iterator].index;
			if(DEBUG) std::cout << "   checking nearest node 2: " << nearest_point_two << " is complete: ";

			if(this->checkVectorForElement(nodes_completed, nearest_point_two))
			{
				closest_neighbour_iterator++;
				if(DEBUG) std::cout << "true";
			}
			else
			{
				incomplete_node = true;
				if(DEBUG) std::cout << "false";
			}
			if(DEBUG) std::cout << std::endl;
		}


		if(DEBUG) std::cout << "Nearest points are: " << nearest_point_one << ", " << nearest_point_two << std::endl;

		connection_exists = this->checkExistingConnection(node, nearest_point_one, existing_connections);
		if(!connection_exists)
			this->addZtoIncidenceMatrix(incidence_matrix, node, nearest_point_one, current_edge, existing_connections);			
		
		connection_exists = this->checkExistingConnection(node, nearest_point_two, existing_connections);
		if(!connection_exists)
			this->addZtoIncidenceMatrix(incidence_matrix, node, nearest_point_two, current_edge, existing_connections);			
		
		connection_exists = this->checkExistingConnection(nearest_point_two, nearest_point_one, existing_connections);
		if(!connection_exists)
			this->addZtoIncidenceMatrix(incidence_matrix, nearest_point_two, nearest_point_one, current_edge, existing_connections);

		nodes_completed.push_back(node);


	}

	if(DEBUG) std::cout << "Incidence Matrix" << std::endl;
	if(DEBUG) this->printMatrix(incidence_matrix);


	return incidence_matrix;
}


void Frame::createDistanceMatrixForOuput()
{

	output = new float*[N_drones];
	for(int i = 0; i < N_drones; ++i)
    	output[i] = new float[N_drones];
    
    for(int i = 0; i < N_drones; i++)
    	for(int j = 0; j < N_drones; j++)
    		output[i][j] = -1;

    for(int i = 0; i < connected_nodes.size(); i++)
    	for(int j = 0; j < connected_nodes[i].size(); j++)
 	    	output[i][connected_nodes[i][j]] = full_distance_vector[i][connected_nodes[i][j]].distance;

 	    if(DEBUG) std::cout << "FINAL INFO" << std::endl;
 	    if(DEBUG) 	
 	    {
 	    	for (int i = 0; i < N_drones; i++)
			{
		    	for (int j = 0; j < N_drones; j++)
			        std::cout << "\t" << output[i][j];
			    std::cout << std::endl;
			}
 	    }

}


//Return methods
std::vector<std::vector<int> > Frame::getConnectionList()
{
	return connected_nodes;
}


std::vector<std::vector<Frame::sort_struct> > Frame::getDistanceMatrix()
{
	return full_distance_vector;
}

std::vector<Frame::point> Frame::getCoordinateList()
{
	return p_vector;
}





//MATH CLASSES
void Frame::addZtoIncidenceMatrix(std::vector< std::vector <float> > &matrix, int node_1, int node_2, int &current_z, std::vector< std::vector <int> > &existing_connections)
{
	connected_nodes[node_1].push_back(node_2);
	connected_nodes[node_2].push_back(node_1);
	if(DEBUG) std::cout << "Connecting nodes " << node_1 << " and " << node_2 << std::endl;
	std::vector<int> connection(2);
	matrix[node_1][current_z] = 1;
	matrix[node_2][current_z] = -1;
	current_z++;
	connection[0] = node_1;
	connection[1] = node_2;
	existing_connections.push_back(connection);
}

void Frame::addZtoIncidenceMatrixNoTracking(std::vector< std::vector <float> > &matrix, int node_1, int node_2, int &current_z)
{
	connected_nodes[node_1].push_back(node_2);
	connected_nodes[node_2].push_back(node_1);
	if(DEBUG) std::cout << "Connecting nodes " << node_1 << " and " << node_2 << std::endl;
	matrix[node_1][current_z] = 1;
	matrix[node_2][current_z] = -1;

}

void Frame::calcDistances()
{
	std::vector<sort_struct> temp_vector_distance;
	for(int j = 0; j < N_drones; j++)
	{
		for(int i = 0; i < p_vector.size(); i++)
		{
			sort_struct temp_sort_struct;
			temp_sort_struct.index = i;
			temp_sort_struct.distance = this->distanceBetweenTwoPoints(p_vector[j], p_vector[i]);
			temp_vector_distance.push_back(temp_sort_struct);
		}
		full_distance_vector.push_back(temp_vector_distance);
		temp_vector_distance.clear();
	}
	if(DEBUG) std::cout << "Distance Matrix" << std::endl;
	if(DEBUG) this->printMatrixSort(full_distance_vector);
	
}
float Frame::distanceBetweenTwoPoints(point a, point b)
{
	float distance_x = a.x - b.x;
	float distance_y = a.y - b.y;
	float distance = sqrt(distance_x*distance_x + distance_y*distance_y);
	return distance;
}

std::vector<Frame::sort_struct> Frame::sortDistances(std::vector<sort_struct> input_vector)
{

	std::vector<sort_struct> temp_vec = input_vector;
	sort_struct temp;
	int current;

	for (int i = 0; i < temp_vec.size(); i++)
	{
		current = i;
		for (int j = i + 1; j < temp_vec.size(); j++)
		{
			if (temp_vec[j].distance < temp_vec[current].distance) //Change was here!
			{
				current = j;
			}
		}

		temp = temp_vec[current];
		temp_vec[current] = temp_vec[i];
		temp_vec[i] = temp;
	}

	return temp_vec;
}

bool Frame::checkExistingConnection(int node_1, int node_2, std::vector < std::vector<int > > connections)
{
	bool result = false;
	for(int check_connection = 0; check_connection < connections.size(); check_connection++)
	{
		if(connections[check_connection][0] == node_1 && connections[check_connection][1] == node_2)
			result = true;
		else if(connections[check_connection][0] == node_2 && connections[check_connection][1] == node_1)
			result = true;
	}
	return result;
}

bool Frame::checkVectorForElement(std::vector<int> vec, int check)
{
	bool result = false;
	for (int i = 0; i < vec.size(); ++i)
	{
		if(vec[i] == check)
			result = true;
	}
	return result;
}






//PRINTING CLASSES


void Frame::printMatrix(std::vector< std::vector <float> > matrix)
{
	for (int i = 0; i < matrix.size(); i++)
	{
    	for (int j = 0; j < matrix[i].size(); j++)
	        std::cout << "\t" << matrix[i][j];
	    std::cout << std::endl;
	}
}

void Frame::printVector(std::vector<float> vector)
{
    for (int j = 0; j < vector.size(); j++)
	    std::cout << vector[j] << " ";
	std::cout << std::endl;
}

void Frame::printMatrixSort(std::vector< std::vector <sort_struct> > matrix, bool print_index)
{
	if(print_index)
	{
		for (int i = 0; i < matrix.size(); i++)
		{
	    	for (int j = 0; j < matrix[i].size(); j++)
		        std::cout << "\t" << matrix[i][j].index  /*<< matrix[i][j].distance << ". "*/;
		    std::cout << std::endl;
		}
	}
	else
	{
		for (int i = 0; i < matrix.size(); i++)
			{
		    	for (int j = 0; j < matrix[i].size(); j++)
			        std::cout << "\t" << matrix[i][j].distance ;
			    std::cout << std::endl;
			}	
	}
}

void Frame::printVectorSort(std::vector<sort_struct> vector)
{
    for (int j = 0; j < vector.size(); j++)
	    std::cout << vector[j].index << ", " << vector[j].distance << ". ";
	std::cout << std::endl;
}

void Frame::printConnections()
{
	for(int i = 0; i < connected_nodes.size(); i++)
	{
		std::cout << "Node " << i << " -> ";
		for (int j = 0; j < connected_nodes[i].size()-1; ++j)
		{
			std::cout << connected_nodes[i][j] << ", ";
		}
		std::cout << connected_nodes[i][connected_nodes[i].size()-1] << std::endl;
	}
}


bool add(formation_control::Formation::Request  &req, formation_control::Formation::Response &res)
{
	std::cout << "Received Service Call" << std::endl;
	//'p' -> polygon, 'g' -> grid, 'v' -> v-shape
	int amount_of_drones = req.amount_of_drones;
	float distance = req.distance;
	float v_shape_angle = req.angle;
	char shape = req.shape_type;
	std::cout << "amount_of_drones: " << amount_of_drones << ", distance: " << distance << ", shape: " << shape << " , v shape angle: " << v_shape_angle << std::endl;
	Frame frame(amount_of_drones, distance, shape, v_shape_angle);
	std::vector<double> result(amount_of_drones*amount_of_drones);
	int iterator = 0;
	for(int i = 0; i < amount_of_drones; i++)
		for (int j = 0; j < amount_of_drones; ++j)
		{
			result[iterator] = frame.output[i][j];
			iterator++;
		}

	res.data = result;
	res.matrix_size = amount_of_drones;
	if(DEBUG)
	{
		std::cout << "Sending info: " << std::endl << "Matrix size: " << res.matrix_size << std::endl << "Data: ";
		for (int i = 0; i < amount_of_drones*amount_of_drones; ++i)
		{
			std::cout << res.data[i] << ", ";
		}
		std::cout << std::endl;
  	}
  	std::cout << std::endl;
  	return true;
}




int main( int argc, char** argv )
{
	ros::init(argc, argv, "Formation_Control");
	//'p' -> polygon, 'g' -> grid, 'v' -> v-shape
	int amount_of_drones = 9;
	float distance = 10;
	float v_shape_angle = 45;
	char shape = 'g';
	Frame frame(amount_of_drones, distance, shape, v_shape_angle);
	ros::NodeHandle n;
	ros::ServiceServer service = n.advertiseService("formation_control", add);
	std::cout << "Ready to receive service calls" << std::endl << std::endl;
	ros::spin();


	return 0;
}