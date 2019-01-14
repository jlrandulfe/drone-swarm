#ifndef FRAME_HPP_
#define FRAME_HPP_

#include <iostream>
#include <cstdlib> 
#include <ros/ros.h>
#include <stdio.h>
#include <math.h>
#include "formation_control/Formation.h"
class Frame
{


	public:
		struct point
		{
			float x;
			float y;
		};
		struct sort_struct
		{
			float distance;
			int index;
		};

		Frame(int N_drones_, float radius_, char shape, float v_shape_angle_);	
		~Frame();
		std::vector<point> p_vector;
		// std::vector<std::vector<sort_struct> > getDistanceMatrix();
		// std::vector<std::vector<int> > getConnectionList();
		// std::vector<point> getCoordinateList();
		float** output;
		void createDistanceMatrixForOuput();
		std::vector<Frame::point> randomizeStartPositions(float range);


	private:
		
		int N_drones;
		float  radius;
		float v_shape_angle;
		std::vector<std::vector<sort_struct> > full_distance_vector;

		std::vector<std::vector<int> > connected_nodes;
		std::vector< std::vector <float> > incidence_matrix;

		void calcDistances();

		void printConnections();
		float distanceBetweenTwoPoints(point a, point b);
		std::vector<sort_struct> sortDistances(std::vector<sort_struct> input_vector);
		void printMatrix(std::vector< std::vector <float> > matrix);
		void printMatrixSort(std::vector< std::vector <sort_struct> > matrix, bool print_index=false);
		void printVector(std::vector<float> vector);
		void printVectorSort(std::vector<sort_struct> vector);
		bool checkExistingConnection(int node_1, int node_2, std::vector < std::vector<int > > connections);
		void addZtoIncidenceMatrix(std::vector< std::vector <float> > &matrix, int node_1, int node_2, int &current_z, std::vector< std::vector <int> > &existing_connections);
		void addZtoIncidenceMatrixNoTracking(std::vector< std::vector <float> > &matrix, int node_1, int node_2, int &z);

		bool checkVectorForElement(std::vector<int> vec, int check);
		std::vector<point> createPolygonCoordinates(int vertices, float radius);
		std::vector<Frame::point> createGridCoordinates(int vertices, float radius);
		std::vector<Frame::point> createVCoordinates(int vertices, float radius, float angle);

		std::vector< std::vector <float> > PolygonIncidenceMatrix();
		std::vector< std::vector <float> > GridIncidenceMatrix();
		std::vector< std::vector <float> > VIncidenceMatrix();


};

#endif /* FRAME_HPP_ */
